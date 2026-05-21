# M9v2 — Delaunay 2D (delaunator algorithm)

## Overview

Port of the [delaunator](https://github.com/mapbox/delaunator) algorithm for planar Delaunay triangulation. Given `Vec3f[]` positions (z ignored), returns a triangular `HalfEdgeMesh` with planar boundary. Uses a private builder that allocates all arrays upfront, then populates them incrementally via edge flipping and hull tracking.

## Module

`module cg::delaunay;` — rewritten `src/delaunay/delaunay_2d.c3`.

Imports: `cg`, `cg::geometry` (`in_circle_2d`, `orient_2d`, `circumradius`), `cg::half_edge` (`from_triangles` — for final mesh build only), `std::sort`, `std::math`.

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
    int[] triangles;     // flat: 3 per face, vertex indices (into positions)
    HeIndex[] halfedges; // flat: 3 per face, twin links
    HeIndex[] hull_tri;  // hull vertex → adjacent half-edge
    VertexIndex[] hull_prev, hull_next; // linked list
    VertexIndex[] hull_hash;            // spatial hash
    usz face_count;      // current number of triangles
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

Compute bbox center: `center = ((min+max)/2)`. Find closest point to center → `i0`. Find closest point to `i0` (excluding `i0`) → `i1`. Scan all remaining points: for each candidate, compute `circumradius(i0,i1,candidate)`. Select the one with smallest positive radius (skip collinear).

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
t = add_triangle(e, p, next[e], INVALID_HE, INVALID_HE, hull_tri[e]);
hull_tri[p] = legalize(t + 2);   // legalize the new edge opposite p
hull_tri[e] = t;                  // new edge on hull

next = next[e];
while true:
    q = next[next];
    if orient_2d(positions[next], positions[q], positions[p]) >= 0: break;
    t = add_triangle(next, p, q, hull_tri[p], INVALID_HE, hull_tri[next]);
    hull_tri[p] = legalize(t + 2);
    hull_tri[next] = t;
    next = q;
```

**3c. Walk hull backward**:

If the initial visible edge search moved backward (counter-clockwise from hash hit), also walk backward:

```
prev = prev[e];
while true:
    q = prev[prev];
    if orient_2d(positions[q], positions[prev], positions[p]) >= 0: break;
    t = add_triangle(q, p, prev, INVALID_HE, hull_tri[prev], hull_tri[q]);
    legalize(t + 2);
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
fn usz add_triangle(VertexIndex i0, VertexIndex i1, VertexIndex i2,
                     HeIndex twin0, HeIndex twin1, HeIndex twin2)
```

Appends one triangle to builder's `triangles[]` and `halfedges[]`. Returns the index of the first new half-edge.

```
t = face_count;
face_count++;

triangles[3*t]   = i0; triangles[3*t+1] = i1; triangles[3*t+2] = i2;
halfedges[3*t]   = twin0;
halfedges[3*t+1] = twin1;
halfedges[3*t+2] = twin2;

// Link twins.
for (twin0, twin1, twin2): if twin != INVALID_HE: halfedges[twin] = 3*t + i

// Set vertex handles if unset.
for (i0,i1,i2): if vertices[i].half_edge == INVALID_HE: vertices[i].half_edge = 3*t + i

return 3*t;
```

### Phase 5 — `legalize()` (stack-based)

```c3
fn HeIndex legalize(HeIndex he)
```

Repeatedly checks and flips the edge identified by `he`. Uses an explicit stack (preallocated, max depth = face_count):

```
stack[0] = he;
stack_size = 1;

while stack_size > 0:
    stack_size--;
    he = stack[stack_size];
    
    a = halfedges[he];
    if a == INVALID_HE: continue;           // boundary edge
    
    // in_circle test: is opposite vertex inside circumcircle?
    ar = prev_halfedge(he);
    al = next_halfedge(he);
    bl = prev_halfedge(a);
    
    p0 = triangles[ar];  // origin of he (vertex on one side)
    pr = triangles[he];  // origin of next (vertex at tip)
    pl = triangles[al];  // origin of next's next
    p1 = triangles[bl];  // opposite vertex across the edge
    
    if in_circle_2d(positions[p1], positions[p0], positions[pr], positions[pl]) != PREDICATE_POSITIVE:
        continue;                            // edge is Delaunay
    
    // Flip: swap the two triangles' third vertices.
    triangles[he] = p1;
    triangles[a]  = p0;
    
    // Reconnect half-edges.
    hbl = halfedges[bl];
    link(he, hbl);
    link(a, halfedges[ar]);
    link(ar, bl);
    
    br = next_halfedge(a);
    stack[stack_size] = br;   // push newly exposed edges
    stack[stack_size+1] = bl;
    stack_size += 2;
```

Helper: `link(a, b)` sets `halfedges[a] = b` and `halfedges[b] = a` (if b ≠ INVALID_HE).

Helper: `next_halfedge(i) = i % 3 == 2 ? i - 2 : i + 1`
Helper: `prev_halfedge(i) = i % 3 == 0 ? i + 2 : i - 1`

**Hull handle updates**: During legalization, if a flipped edge `he` or `a` is adjacent to the hull (its twin was INVALID_HE before the flip), the hull's `hull_tri[]` entry for the affected vertex must be updated. This is done by checking vertex origins: if `halfedges[next(he)].twin == INVALID_HE`, the vertex `triangles[next(he)]` is on the hull and its hull_tri entry should point to `next(he)`. Same for the other side.

### Phase 6 — Mesh finalization

After all insertions:

1. Collect surviving faces (skip any that reference super-triangle — none in delaunator, but check for completeness).
2. Build `uint[]` index array from builder's `triangles[]`.
3. Allocate `HalfEdgeMesh` arrays, populate `positions[]`, `vertices[]`, `faces[]`, `half_edges[]` from builder data.
4. `mesh.validate()!` then return.

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
