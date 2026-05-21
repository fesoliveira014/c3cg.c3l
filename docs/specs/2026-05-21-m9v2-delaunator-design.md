# M9v2 — Delaunay 2D (delaunator algorithm)

## Overview

Port of the [delaunator](https://github.com/mapbox/delaunator) algorithm for planar Delaunay triangulation. Given `Vec3f[]` positions (z ignored), returns a triangular `HalfEdgeMesh` with planar boundary. Uses a private builder with flat arrays preallocated upfront — no `HalfEdgeMesh` mutation during insertion, no `from_triangles`.

## Module

`module cg::delaunay;` — rewritten `src/delaunay/delaunay_2d.c3`.

Imports: `cg`, `cg::geometry` (`in_circle_2d`, `orient_2d`, `circumcenter_planar`), `std::sort`, `std::math`.

Umbrella unchanged.

## Public API — unchanged

```c3
fn HalfEdgeMesh? delaunay_2d(Allocator alloc, Vec3f[] positions, DelaunayOptions options = {});
```

`DelaunayOptions.has_bounding_box`: if true, all points must be inside the given AABB (x/y only, z ignored, inclusive bounds). Invalid AABB (`min > max`) → `DEGENERATE_INPUT`. No super-triangle is constructed.

## Private builder

```c3
struct DelaunayBuilder {
    VertexIndex[] triangles;   // flat: 3 per face
    HeIndex[] halfedges;       // flat: 3 per face, twin links (INVALID_HE = boundary)
    HeIndex[] hull_tri;        // hull vertex → adjacent half-edge
    VertexIndex[] hull_prev;   // linked list prev (INVALID_VERTEX = not on hull)
    VertexIndex[] hull_next;   // linked list next
    VertexIndex[] hull_hash;   // spatial hash (INVALID_VERTEX = empty slot)
    HeIndex[] legalize_stack;  // preallocated stack for legalize()
    usz face_count;            // current number of triangles
}
```

All arrays preallocated once:

| Array | Size |
|-------|------|
| `triangles`, `halfedges` | `3 × (2n - 5)` |
| `hull_prev`, `hull_next`, `hull_tri` | `n` |
| `hull_hash` | `max(1, √n)` |
| `legalize_stack` | `2n - 5` |

Vertex half-edges are NOT maintained during insertion. During finalization (Phase 6), `mesh.vertices[v].half_edge` is set by scanning `triangles[]` — for each half-edge `i` with `triangles[i] = v`, `mesh.vertices[v].half_edge` is set to `i` (first encountered).

No mutation of `HalfEdgeMesh` before Phase 6. No allocations in the insertion loop.

## Algorithm

### Phase 0 — Dedup, validate

Remove exact `(x,y)` duplicates (z ignored, first occurrence kept). If < 3 non-collinear distinct points → `DEGENERATE_INPUT`.

Collinearity check: find first 2 distinct `(x,y)` points, then scan for a third with `orient_2d != ZERO`. If none found → `DEGENERATE_INPUT`.

Build `working_positions` from deduped unique points. Allocate all builder arrays.

### Phase 1 — Seed triangle

Bbox center: `center = (min+max) / 2`. Find: closest to center → `i0`; closest to `i0` (excluding i0) → `i1`; point with smallest positive `circumradius_sq(i0,i1,candidate)` → `i2`.

`circumradius_sq(a,b,c)`: try `circumcenter_planar(a,b,c)`. If fault (collinear) → return ∞. Else return squared distance from center to vertex `a`.

Orient CCW: if `orient_2d(i0,i1,i2) <= 0`, swap i1,i2. Must be POSITIVE after swap.

Add seed triangle: `triangles = [i0,i1,i2]`, `halfedges = [INVALID_HE,INVALID_HE,INVALID_HE]`, `face_count = 1`.

Compute seed circumcenter. Sort points by squared distance to this center (ascending). Skip i0,i1,i2 during insertion.

### Phase 2 — Hull initialization

```c3
hull_next[i0] = i1;  hull_next[i1] = i2;  hull_next[i2] = i0;
hull_prev[i1] = i0;  hull_prev[i2] = i1;  hull_prev[i0] = i2;
hull_tri[i0]  = 0;   hull_tri[i1]  = 1;   hull_tri[i2]  = 2;
```

All other `hull_prev`, `hull_next` entries: `INVALID_VERTEX` (means "not on hull").

Hash: `pseudo_angle(vector from seed center to vertex)`. Map to `hull_hash[key] = vertex`. Linear probe.

Hash size = `max(1, ⌈√unique_count⌉)`. Hash sentinel = `INVALID_VERTEX`.

### Phase 3 — Point insertion

For each `p` in sorted order (skip i0,i1,i2; skip if `dist²(p, prev_p) < EPSILON`):

**3a. Find visible hull edge:**

Hash `p` to starting vertex `start`. Advance CCW until finding edge `e` where:
```
orient_2d(working_positions[e], working_positions[hull_next[e]], working_positions[p]) < 0
```
This means `p` is outside the hull across edge `e → hull_next[e]`. Track `backwards = (advanced past start)` (if hash hit is behind p, we moved backward first). If full traversal back to `start` without finding one, skip `p` (inside hull — should not happen).

**3b. Walk forward:**

```
t = add_triangle(e, p, hull_next[e], INVALID_HE, INVALID_HE, hull_tri[e]);
hull_tri[p] = legalize(t + 2);
curr = hull_next[e];
while true:
    q = hull_next[curr];
    if orient_2d(positions[curr], positions[q], positions[p]) >= 0: break;
    t = add_triangle(curr, p, q, hull_tri[p], INVALID_HE, hull_tri[curr]);
    hull_tri[p] = legalize(t + 2);
    curr = q;
```

Vertices `e` and all intermediate `curr` are removed from the hull (their edges become interior). The walk end is `curr` (still on hull).

**3c. Walk backward** (if `backwards`):

Start from `e`, move backward:
```
prev = hull_prev[e];
while true:
    q = hull_prev[prev];
    if orient_2d(positions[q], positions[prev], positions[p]) >= 0: break;
    t = add_triangle(q, p, prev, INVALID_HE, hull_tri[prev], hull_tri[q]);
    legalize(t + 2);
    prev = q;
```

Walk end becomes `prev` (still on hull).

**3d. Update hull links:**

Insert `p` between `e` and `walk_end` (forward) or between `prev` and `e` (backward).

Removed hull vertices are implicitly marked: they no longer appear in the linked list (their `hull_next`/`hull_prev` entries are bypassed). Hash lookups that land on removed vertices skip them (check `hull_next[v] != INVALID_VERTEX`).

### Phase 4 — `add_triangle`

```c3
fn HeIndex add_triangle(VertexIndex i0, VertexIndex i1, VertexIndex i2,
                         HeIndex twin0, HeIndex twin1, HeIndex twin2)
```

```
t = face_count++;
triangles[3*t]=i0; triangles[3*t+1]=i1; triangles[3*t+2]=i2;
halfedges[3*t]=twin0; halfedges[3*t+1]=twin1; halfedges[3*t+2]=twin2;
if twin0 != INVALID_HE: halfedges[twin0] = 3*t;
if twin1 != INVALID_HE: halfedges[twin1] = 3*t+1;
if twin2 != INVALID_HE: halfedges[twin2] = 3*t+2;
return 3*t;
```

### Phase 5 — `legalize()`

```c3
fn void legalize(HeIndex he, Vec3f[] positions, DelaunayBuilder* builder)
```

Stack-based. Pushes candidate edges, pops and checks each. Returns nothing — callers use `hull_tri[p]` which was set before calling legalize and remains valid because legalize flips but doesn't change which half-edge is hull-adjacent for vertex `p`.

```
stack[0] = he; stack_size = 1;
while stack_size > 0:
    he = stack[--stack_size];
    a = halfedges[he];
    if a == INVALID_HE: continue;

    ar = prev_halfedge(he); al = next_halfedge(he); bl = prev_halfedge(a);
    p0 = triangles[ar]; pr = triangles[he]; pl = triangles[al]; p1 = triangles[bl];

    v0 = {pos[p0].x,pos[p0].y}; vr = {pos[pr].x,pos[pr].y};
    vl = {pos[pl].x,pos[pl].y}; v1 = {pos[p1].x,pos[p1].y};
    if in_circle_2d(v0, vr, vl, v1) != PREDICATE_POSITIVE: continue;

    triangles[he] = p1; triangles[a] = p0;
    hbl = halfedges[bl]; har = halfedges[ar];
    link(he, hbl); link(a, har); link(ar, bl);
    br = next_halfedge(a);
    stack[stack_size++] = he;
    stack[stack_size++] = br;

    // Hull update: if bl was on hull boundary before link, update hull_tri.
    if hbl == INVALID_HE: hull_tri[triangles[bl]] = he;
```

Helpers: `next_halfedge(i) = i%3==2 ? i-2 : i+1`, `prev_halfedge(i) = i%3==0 ? i+2 : i-1`, `link(a,b) = { halfedges[a]=b; if b!=INVALID_HE: halfedges[b]=a; }`.

### Phase 6 — Mesh finalization

```c3
mesh.half_edges = mem::alloc::new_array(alloc, HalfEdge, (sz)(3 * face_count));
defer catch free(mesh.half_edges);
mesh.faces = mem::alloc::new_array(alloc, HalfEdgeFace, (sz) face_count);
defer catch free(mesh.faces);
mesh.vertices = mem::alloc::new_array(alloc, HalfEdgeVertex, (sz) unique_count);
defer catch free(mesh.vertices);
mesh.positions = mem::alloc::new_array(alloc, Vec3f, (sz) unique_count);
defer catch free(mesh.positions);

for (usz i = 0; i < unique_count; i++) mesh.positions[i] = working_positions[i];

// Vertex half-edges: scan triangles, set first half-edge for each vertex.
for (usz i = 0; i < unique_count; i++) mesh.vertices[i].half_edge = INVALID_HE;
for (usz i = 0; i < 3 * face_count; i++) {
    VertexIndex v = triangles[i];
    if mesh.vertices[v].half_edge == INVALID_HE: mesh.vertices[v].half_edge = (HeIndex)(int)i;
}

for (usz i = 0; i < 3 * face_count; i++) {
    mesh.half_edges[i].origin = triangles[i];
    mesh.half_edges[i].twin   = halfedges[i];
    mesh.half_edges[i].face   = (FaceIndex)(int)(i / 3);
    if i % 3 == 2: mesh.half_edges[i].next = (HeIndex)(int)(i - 2);
    else:          mesh.half_edges[i].next = (HeIndex)(int)(i + 1);
}
for (usz i = 0; i < face_count; i++) mesh.faces[i].half_edge = (HeIndex)(int)(3 * i);

mesh.normals = {}; mesh.uvs = {};
mesh.validate()!;
// Free builder arrays, return mesh.
```

## Hull details

`pseudo_angle(dx, dy)`:
```
p = dx / (|dx| + |dy|)
if dy > 0: (3.0 - p) / 4.0
else:      (1.0 + p) / 4.0
// Result in [0, 1)
hash_idx = (int)(pseudo_angle * (float)hash_len)
```

Linear probe on collision. Removed hull vertices (not in linked list) are skipped during probe: `if hull_next[v] == INVALID_VERTEX: continue probe`.

## Predicates

| Predicate | Use |
|-----------|-----|
| `in_circle_2d(a,b,c,p)` | `== PREDICATE_POSITIVE` → edge illegal (Lawson test) |
| `orient_2d(a,b,c)` | `< 0` → point outside hull edge; `== 0` → collinear |

All predicates take `Vec2f` — project from `Vec3f` via `{ pos[i].x, pos[i].y }`.

## Faults

| Fault | When |
|-------|------|
| `EMPTY_INPUT` | `positions.len == 0` |
| `DEGENERATE_INPUT` | < 3 non-collinear distinct points; bounding box excludes points; invalid AABB |

## Memory

| Array | When freed |
|-------|------------|
| Builder arrays (triangles, halfedges, hull_*, stack) | After mesh finalization, explicit free |
| `working_positions` | After mesh finalization, explicit free |
| `int[] indices` (sort) | After sort, explicit free |
| `HalfEdgeMesh` | Caller via `mesh.destroy()`. `defer catch` on partial allocation |

No allocations in insertion loop. `validate()!` before explicit frees (defer catch handles fault cleanup).

## Tests — unchanged (12 cases)

File: `test/test_delaunay_2d.c3`. Existing tests remain valid — API contract identical.

## Non-goals

- No `add_triangle` on `HalfEdgeMesh`
- No super-triangle
- No `from_triangles`
- No dynamic reallocation during insertion
