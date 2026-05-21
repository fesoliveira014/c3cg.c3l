# M9v2 — Delaunay 2D (delaunator algorithm)

## Overview

Port of the [delaunator](https://github.com/mapbox/delaunator) algorithm for planar Delaunay triangulation. Given `Vec3f[]` positions (z ignored), returns a triangular `HalfEdgeMesh` with planar boundary. Uses a private builder that allocates all arrays upfront, then populates them incrementally via edge flipping and hull tracking.

## Module

`module cg::delaunay;` — rewritten `src/delaunay/delaunay_2d.c3`.

Imports: `cg`, `cg::geometry` (`in_circle_2d`, `orient_2d`), `std::sort`, `std::math`. No `cg::half_edge` import needed — mesh built manually at end.

Umbrella unchanged.

## Public API — unchanged

```c3
fn HalfEdgeMesh? delaunay_2d(Allocator alloc, Vec3f[] positions, DelaunayOptions options = {});
```

`DelaunayOptions.has_bounding_box` is validated (points must be inside the given AABB, x/y only) but no super-triangle is constructed — the bounding box only constrains valid input.

## Private builder

Instead of modifying `HalfEdgeMesh` during insertion, use a private builder with flat arrays:

```c3
struct DelaunayBuilder {
    VertexIndex[] triangles;  // flat: 3 per face, vertex indices
    HeIndex[] halfedges;      // flat: 3 per face, twin links (INVALID_HE = boundary)
    HeIndex[] vertex_halfedges; // canonical outgoing half-edge per vertex
    HeIndex[] hull_tri;       // hull vertex → adjacent half-edge
    VertexIndex[] hull_prev, hull_next; // linked list
    VertexIndex[] hull_hash;  // spatial hash
    usz face_count;           // current number of triangles
}
```

**Preallocation**: all arrays sized once before insertion. `triangles` and `halfedges` at `3 × max_triangles` where `max_triangles = 2n - 5`. Hull arrays at `n`. Hash at `⌈√n⌉` minimum 1.

**Vertex allocation**: all deduped unique vertices are placed in `positions[]` (the builder's copy, not the input). `HalfEdgeMesh.vertices[]` is allocated at `unique_count` with `half_edge = INVALID_HE` initially. These are set during triangle insertion.

**Mesh finalization**: after all insertions, allocate `HalfEdgeMesh` arrays, copy builder data, assign vertex handles. Call `mesh.validate()!`. This is a single allocation + copy pass — no `from_triangles` overhead.

## Algorithm

### Phase 0 — Dedup, validate

Remove exact `(x,y)` duplicates (z ignored). If < 3 non-collinear distinct points, fault `DEGENERATE_INPUT`. The collinearity check must find at least one non-collinear triple in the full point set — not just the first 3 points (find first 2 distinct, then scan for non-collinear third).

Build working positions from deduped points (unique_count). Allocate builder arrays at max size. Allocate `HalfEdgeMesh.vertices[]` at unique_count, initialize all `half_edge = INVALID_HE`.

### Phase 1 — Seed triangle

Compute bbox center: `center = ((min+max)/2)`. Find closest point to center → `i0`. Find closest point to `i0` (excluding `i0`) → `i1`. Scan all remaining points: for each candidate, compute `circumradius_sq(i0,i1,candidate)` (private helper: compute squared circumradius from three points; reject collinear where `orient_2d == ZERO`). Select candidate with smallest positive radius.

Private helper `circumradius_sq(a,b,c)`: compute circumcenter via `circumcenter_planar(a,b,c)`. If fault → return MAX (collinear, skip). Otherwise return squared distance from circumcenter to any vertex.

Orient CCW: if `orient_2d(i0,i1,i2) <= 0`, swap i1,i2. Then `orient_2d(i0,i1,i2)` must be POSITIVE (non-degenerate).

Add seed triangle to builder: `triangles = [i0,i1,i2]`, `halfedges = [INVALID_HE,INVALID_HE,INVALID_HE]`, `face_count = 1`.

Compute seed circumcenter via `circumcenter_planar`. Build distance sort: for each point, compute squared distance to seed center. Sort indices by this distance (ascending). Use `sort::quicksort` with context comparator. Skip seed triangle points (i0,i1,i2) during insertion.

### Phase 2 — Hull initialization

Initialize hull linked list (CCW order around the seed triangle):

```c3
hull_next[i0] = i1;  hull_next[i1] = i2;  hull_next[i2] = i0;
hull_prev[i1] = i0;  hull_prev[i2] = i1;  hull_prev[i0] = i2;
hull_tri[i0]  = 0;   hull_tri[i1]  = 1;   hull_tri[i2]  = 2;
```

Hash: for each hull vertex, compute `pseudo_angle(vector from seed center to vertex)`. Map to hash bucket: `hull_hash[key] = vertex_index`. Linear probe on collision.

Hash table size = `⌈√unique_count⌉`, minimum 1. Empty sentinel = `INVALID_VERTEX`.

### Phase 3 — Point insertion

For each point `p` in distance-sorted order (skip i0,i1,i2, skip near-duplicates where distance to previous point < EPSILON):

**3a. Find visible hull edge**:

Hash `p` to find a starting hull vertex `start`. Then advance CCW along the hull until finding edge `e` where `orient_2d(positions[e], positions[next[e]], positions[p]) < 0`. This means `p` is to the right of directed edge `e → next[e]` → `p` is outside the hull at this edge.

If no such edge after a full traversal (back to start), `p` is inside or on the hull — skip it (near-duplicate, numerical issue).

**3b. Walk hull forward**:

```
t = add_triangle(e, p, hull_next[e], INVALID_HE, INVALID_HE, hull_tri[e]);
hull_tri[p] = legalize(t + 2);
hull_tri[e] = t;

curr = hull_next[e];
while true:
    q = hull_next[curr];
    if orient_2d(positions[curr], positions[q], positions[p]) >= 0: break;
    t = add_triangle(curr, p, q, hull_tri[p], INVALID_HE, hull_tri[curr]);
    hull_tri[p] = legalize(t + 2);
    hull_tri[curr] = t;
    curr = q;
```

**3c. Walk hull backward:**

If the initial visible edge search returned `backwards = true` (the hash hit was behind the insertion point), also walk backward from the original start edge:

```
prev = hull_prev[e];
while true:
    q = hull_prev[prev];
    if orient_2d(positions[q], positions[prev], positions[p]) >= 0: break;
    t = add_triangle(q, p, prev, INVALID_HE, hull_tri[prev], hull_tri[q]);
    legalize(t + 2, positions, builder);
    hull_tri[q] = t;
    prev = q;
```

**3d. Update hull links**:

Insert `p` between the walk endpoints:

```
prev[p] = e;         next[e] = p;
prev[walk_end] = p;  next[p] = walk_end;
```

Hash the new hull vertex `p` and the updated edges.

### Phase 4 — `add_triangle` (private to builder)

```c3
fn HeIndex add_triangle(VertexIndex i0, VertexIndex i1, VertexIndex i2,
                         HeIndex twin0, HeIndex twin1, HeIndex twin2,
                         DelaunayBuilder* builder)
```

Appends one triangle to builder arrays. Returns index of first new half-edge.

```
t = face_count;
face_count++;

triangles[3*t]   = i0; triangles[3*t+1] = i1; triangles[3*t+2] = i2;
halfedges[3*t]   = twin0;
halfedges[3*t+1] = twin1;
halfedges[3*t+2] = twin2;

// Link twins.
if twin0 != INVALID_HE: halfedges[twin0] = 3*t;
if twin1 != INVALID_HE: halfedges[twin1] = 3*t+1;
if twin2 != INVALID_HE: halfedges[twin2] = 3*t+2;

// Set vertex handles if unset.
if vertex_halfedges[i0] == INVALID_HE: vertex_halfedges[i0] = 3*t;
if vertex_halfedges[i1] == INVALID_HE: vertex_halfedges[i1] = 3*t+1;
if vertex_halfedges[i2] == INVALID_HE: vertex_halfedges[i2] = 3*t+2;

return 3*t;
```

### Phase 5 — `legalize()` (stack-based, returns hull-facing half-edge)

```c3
fn HeIndex legalize(HeIndex he, Vec3f[] positions, DelaunayBuilder* builder)
```

Uses an explicit stack (preallocated at builder init). Returns the half-edge adjacent to a hull boundary after legalization (used by callers to update `hull_tri[]`).

```
stack[0] = he;
stack_size = 1;

while stack_size > 0:
    stack_size--;
    he = stack[stack_size];
    
    a = halfedges[he];  // twin half-edge
    if a == INVALID_HE: continue;  // boundary edge
    
    ar = prev_halfedge(he);   // edge before he in its triangle
    al = next_halfedge(he);   // edge after he
    bl = prev_halfedge(a);    // edge before twin
    
    p0 = triangles[ar];  // origin of ar (vertex on he's side)
    pr = triangles[he];  // origin of he (tip vertex)
    pl = triangles[al];  // origin of al
    p1 = triangles[bl];  // origin of bl (opposite vertex)
    
    // Convert Vec3f → Vec2f for predicate.
    Vec2f v0 = { positions[p0].x, positions[p0].y };
    Vec2f vr = { positions[pr].x, positions[pr].y };
    Vec2f vl = { positions[pl].x, positions[pl].y };
    Vec2f v1 = { positions[p1].x, positions[p1].y };
    
    // Lawson test: is p1 inside circumcircle of (p0, pr, pl)?
    if geometry::in_circle_2d(v0, vr, vl, v1) != PREDICATE_POSITIVE: continue;
    
    // Flip: swap opposite vertices.
    triangles[he] = p1;
    triangles[a]  = p0;
    
    // Save pre-flip hull state.
    hbl = halfedges[bl];
    har = halfedges[ar];
    
    // Relink half-edges.
    link(he, hbl);
    link(a, har);
    link(ar, bl);
    
    br = next_halfedge(a);
    
    // Push newly exposed edges for re-check.
    stack[stack_size] = he;    // continue checking this edge
    stack[stack_size + 1] = br;
    stack_size += 2;
    
    // Hull handle update: if edge ar was on hull boundary, update hull_tri.
    if har == INVALID_HE:
        hull_tri[triangles[ar]] = ar;

// Return the hull-adjacent half-edge.
return he;
```

**Helpers**:
- `next_halfedge(i)`: `i % 3 == 2 ? i - 2 : i + 1`
- `prev_halfedge(i)`: `i % 3 == 0 ? i + 2 : i - 1`
- `link(a, b)`: sets `halfedges[a] = b`. If `b != INVALID_HE`, also sets `halfedges[b] = a`. Precondition: `a != INVALID_HE`.

### Phase 6 — Mesh finalization

After all insertions, populate `HalfEdgeMesh` from builder data:

```c3
mesh.half_edges = alloc::new_array(alloc, HalfEdge, 3 * face_count);
mesh.faces      = alloc::new_array(alloc, HalfEdgeFace, face_count);
mesh.vertices   = alloc::new_array(alloc, HalfEdgeVertex, unique_count);
mesh.positions  = alloc::new_array(alloc, Vec3f, unique_count);

// Copy positions.
for i in 0..unique_count: mesh.positions[i] = working_positions[i];

// Populate half-edges.
for i in 0..3*face_count:
    mesh.half_edges[i].origin = triangles[i];
    mesh.half_edges[i].twin   = halfedges[i];
    mesh.half_edges[i].face   = (FaceIndex)(int)(i / 3);
    // next = CCW successor: (i+1) within the same triangle, wrapping.
    if i % 3 == 2: mesh.half_edges[i].next = (HeIndex)(int)(i - 2);
    else:          mesh.half_edges[i].next = (HeIndex)(int)(i + 1);

// Populate faces.
for i in 0..face_count:
    mesh.faces[i].half_edge = (HeIndex)(int)(3 * i);

// Populate vertices.
for i in 0..unique_count:
    mesh.vertices[i].half_edge = vertex_halfedges[i];

mesh.normals = {};
mesh.uvs     = {};

mesh.validate()!;
return mesh;
```

## Hull data structures (all in builder)

| Array | Type | Size | Purpose |
|-------|------|------|---------|
| `hull_prev` | `VertexIndex[]` | n | Previous vertex in CCW hull order |
| `hull_next` | `VertexIndex[]` | n | Next vertex in CCW hull order |
| `hull_tri` | `HeIndex[]` | n | Hull edge → adjacent half-edge |
| `hull_hash` | `VertexIndex[]` | √n | Hash table, `INVALID_VERTEX` = empty |

`pseudo_angle(dx, dy)`:
```
p = dx / (|dx| + |dy|)
if dy > 0: (3.0 - p) / 4.0
else:      (1.0 + p) / 4.0
// Result in [0, 1)
hash_idx = (int)(pseudo_angle * (float)hash_len)
```

Linear probe: `(hash_idx + j) % hash_len` for j=0,1,2,...

## Predicates — unchanged

| Predicate | Use |
|-----------|-----|
| `in_circle_2d(a,b,c,p)` | `== PREDICATE_POSITIVE` → edge is illegal |
| `orient_2d(a,b,c)` | `> 0` → CCW; `< 0` → visible hull edge (point outside) |

## Faults — unchanged

| Fault | When |
|-------|------|
| `EMPTY_INPUT` | `positions.len == 0` |
| `DEGENERATE_INPUT` | < 3 non-collinear distinct points, or bounding box excludes points |

## Memory

All arrays preallocated once before insertion:

| Array | Size | When freed |
|-------|------|------------|
| `triangles` | `3 × (2n - 5)` | Builder, freed after mesh finalization |
| `halfedges` | `3 × (2n - 5)` | Builder, freed after mesh finalization |
| `hull_prev/next/tri` | `n` | Builder, freed after mesh finalization |
| `hull_hash` | `max(1, √n)` | Builder, freed after mesh finalization |
| `int[] indices` (sort) | `n` | Freed after sort |
| `int[] stack` (legalize) | `2n` (prealloc) | Freed after insertion loop |
| `HalfEdgeMesh` | `unique_count` vertices + faces | Caller via `mesh.destroy()` |

No allocations in the insertion loop. All arrays preallocated at Phase 0.

## Edge Cases

- **Duplicate (x,y)**: deduped in Phase 0, first occurrence kept. z ignored.
- **Collinear**: full scan in Phase 0, not just first 3 points.
- **Near-duplicates during insertion**: skip if distance² to previous sorted point < EPSILON.
- **Points inside hull**: `orient_2d >= 0` for all hull edges → skip (should not happen for unique, non-duplicate points).
- **Cocircular**: `in_circle_2d == ZERO` → no flip, both diagonals valid.
- **Bounding box**: validated in Phase 0, fault if points outside. No super-triangle.

## Tests — unchanged (12 cases)

File: `test/test_delaunay_2d.c3`. Existing tests remain valid — API contract unchanged.

## Non-goals

- No `add_triangle` on `HalfEdgeMesh` — private builder only
- No super-triangle
- No `from_triangles` — manual mesh population
- No dynamic reallocation in insertion loop
