# M9v2 — Delaunay 2D (delaunator algorithm)

## Overview

Port of the [delaunator](https://github.com/mapbox/delaunator) algorithm for planar Delaunay triangulation. Replaces the previous Bowyer-Watson implementation. Given `Vec3f[]` positions (z ignored), returns a triangular `HalfEdgeMesh` with planar boundary. The algorithm builds the mesh incrementally using a new `add_triangle` primitive and edge flipping.

## Module

`module cg::delaunay;` — rewritten `src/delaunay/delaunay_2d.c3`.

Imports: `cg`, `cg::geometry` (`in_circle_2d`, `orient_2d`), `cg::half_edge` (new `add_triangle`, `from_triangles` no longer needed), `std::sort`, `std::math`.

Umbrella unchanged (`src/c3cg.c3i`):

```c3
module cg::delaunay;
import cg;

struct DelaunayOptions {
    bool has_bounding_box;
    Aabb bounding_box;
}

fn HalfEdgeMesh? delaunay_2d(Allocator alloc, Vec3f[] positions, DelaunayOptions options = {});
```

## Public API — unchanged

```c3
fn HalfEdgeMesh? delaunay_2d(Allocator alloc, Vec3f[] positions, DelaunayOptions options = {});
```

Returns a caller-owned `HalfEdgeMesh` (triangular, with planar boundary). Caller must `defer mesh.destroy()`.

## New primitive — `HalfEdgeMesh.add_triangle`

Added to `cg::half_edge` (`src/half_edge/builder.c3`):

```c3
fn HeIndex? HalfEdgeMesh.add_triangle(
    &self,
    Allocator alloc,
    VertexIndex i0, VertexIndex i1, VertexIndex i2,
    HeIndex twin0, HeIndex twin1, HeIndex twin2,
);
```

**Contract**: Appends one face + 3 half-edges to the mesh. Vertices `i0,i1,i2` must already exist in `positions[]`. `twinN = INVALID_HE` for boundary edges. For each non-INVALID twin, sets `half_edges[twinN].twin = new_he`. Returns the first new half-edge index.

**Behavior**:
1. Grow `faces[]` by 1, set `face.half_edge = n` (first new HE)
2. Grow `half_edges[]` by 3:
   - `he[n+0]`: origin=i0, next=n+1, twin=twin0, face=new_face
   - `he[n+1]`: origin=i1, next=n+2, twin=twin1, face=new_face  
   - `he[n+2]`: origin=i2, next=n+0, twin=twin2, face=new_face
3. For each twin ≠ INVALID_HE: `half_edges[twin].twin = n + i`
4. For each vertex i₀,i₁,i₂: if `vertices[i].half_edge == INVALID_HE`, set to `n+i`

**Umbrella addition**: none (methods defined in `.c3` files stay there per convention).

## Algorithm

### Phase 0 — Dedup, validate

Remove exact (x,y) duplicates (z ignored). If < 3 non-collinear distinct points, fault `DEGENERATE_INPUT`.

Compute initial mesh: for each unique vertex, append to `positions[]` via `add_vertex` (grow positions array). Build remap from original index → mesh VertexIndex.

### Phase 1 — Seed triangle

Compute bbox center. Find closest point to center (i0). Find closest point to i0 (i1). Scan all points for the one forming the smallest circumcircle with (i0,i1) → i2.

Orient CCW: if `counter_clockwise(i0,i1,i2)` is false, swap i1,i2.

Call `mesh.add_triangle(i0,i1,i2, INVALID_HE, INVALID_HE, INVALID_HE)` — all 3 edges are boundary (initial hull).

Compute seed circumcenter. Sort remaining points by distance from this center (ascending).

### Phase 2 — Hull initialization

Prealloc hull arrays (size = unique_count):

```c3
int[] hull_prev;  // linked list: prev vertex in CCW order
int[] hull_next;  // linked list: next vertex
int[] hull_tri;   // hull edge → adjacent triangle half-edge index
int[] hull_hash;  // spatial hash: hash key → vertex index
```

Initialize: `hull_next = { i1, i2, i0 }`, `hull_prev = { i2, i0, i1 }`, `hull_tri = { 0, 1, 2 }`. Build hash from hull center using `pseudo_angle`.

### Phase 3 — Point insertion

For each point `p` in distance-sorted order (skip seed triangle points, skip near-duplicates):

**3a. Find visible hull edge**:
Hash `p` to get a starting hull vertex. Advance along the hull (CCW) until finding edge `e` where `counter_clockwise(p, e, next[e])` is true. If no such edge (point inside hull), skip it.

**3b. Walk hull forward**:
```
t = mesh.add_triangle(e, p, next[e], INVALID_HE, INVALID_HE, hull_tri[e]);
hull_tri[p] = legalize(t + 2);   // legalize the new edge
hull_tri[e] = t;

next = next[e];
while true:
    q = next[next];
    if not counter_clockwise(p, next, q): break;
    t = mesh.add_triangle(next, p, q, hull_tri[p], INVALID_HE, hull_tri[next]);
    hull_tri[p] = legalize(t + 2);
    next = q;
```

**3c. Walk hull backward** (if first walk was backward from start):
Similar to forward but walking `prev` links.

**3d. Update hull links**:
Insert `p` into the hull between `e` and `walk_end`:
```
hull_prev[p] = e;  hull_next[e] = p;
hull_prev[walk_end] = p;  hull_next[p] = walk_end;
```
Hash the new edges.

### Phase 4 — `legalize()`

```c3
fn HeIndex legalize(HeIndex he):
    // Repeatedly check and flip the edge.
    while true:
        a = mesh.twin(he);
        if a == INVALID_HE: break;
        if not in_circle test fails (edge is Delaunay): break;
        
        mesh.flip(he)!!;
        // Continue with the newly created edges
        he = a;
```

Uses `mesh.flip()` — already in codebase. The loop handles recursive flips. Terminates when edge is on hull boundary or when `in_circle` test passes (edge is Delaunay-valid).

### Phase 5 — Return

`mesh.validate()!` then return. No `from_triangles` — the mesh was built incrementally.

## Hull data structures

| Array | Size | Purpose |
|-------|------|---------|
| `hull_prev[n]` | n | Previous vertex in CCW hull order |
| `hull_next[n]` | n | Next vertex in CCW hull order |
| `hull_tri[n]` | n | Hull edge adjacent triangle half-edge |
| `hull_hash[√n]` | ≈√n | Spatial hash: key → hull vertex index |

Hash function: `pseudo_angle(vector_from_center_to_point) * hash_len`. Linear probe on collision.

`pseudo_angle` maps a 2D vector to [0,1) monotonically with true angle:
```
p = dx / (|dx| + |dy|)
if dy > 0: (3 - p) / 4
else:      (1 + p) / 4
```

## Predicates

All existing in `cg::geometry`:

| Predicate | Use |
|-----------|-----|
| `in_circle_2d(a,b,c,p)` | `== PREDICATE_POSITIVE` → edge is illegal (flip needed) |
| `orient_2d(a,b,c)` | `== PREDICATE_POSITIVE` → CCW orientation |

## Faults — unchanged

| Fault | When |
|-------|------|
| `EMPTY_INPUT` | `positions.len == 0` |
| `DEGENERATE_INPUT` | < 3 non-collinear distinct points |

## Memory

| Allocation | When | Freed |
|-----------|------|-------|
| `hull_prev/next/tri/hash` | Phase 2 (prealloc, size n) | Explicit free after validate |
| `int[] dists` (for sort) | Phase 1 (size n) | Explicit free after sort |
| `HalfEdgeMesh` | Built incrementally | Caller via `mesh.destroy()` |

Mesh arrays grow dynamically: `add_triangle` extends `faces[]`, `half_edges[]`, and potentially `positions[]` (for new vertices). Use `mem::alloc::realloc_array` or manual reallocation.

## Edge Cases

- **Duplicate (x,y)**: deduped in Phase 0. Different z → first occurrence kept.
- **Collinear input**: orient_2d scan in Phase 0 → DEGENERATE_INPUT.
- **Points inside convex hull**: skipped (no visible edge found).
- **Cocircular points**: legalize handles them — in_circle returns ZERO (not POSITIVE), so no flip. Both diagonals are Delaunay-valid.
- **Bounding box option**: seed triangle constrained? Unclear — bounding box was for super-triangle sizing. With delaunator, no super-triangle. Deprecate or repurpose `has_bounding_box`.

## Tests — unchanged (12 cases)

File: `test/test_delaunay_2d.c3`.

Same tests as before — the API contract is identical. Internal algorithm differs but output properties (Delaunay condition, no isolated vertices, correct face count) are the same.

## Non-goals

- No super-triangle
- No `from_triangles` rebuild
- No face-list intermediate representation
- Bounding box option may be deprecated (no super-triangle to size)
