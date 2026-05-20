# M8 — Convex Hull 3D Design

## Overview

Incremental 3D convex hull algorithm. Given `Vec3f[]` positions, returns a closed triangular `HalfEdgeMesh` — every face is a triangle, every half-edge has a twin, `is_boundary` returns `false` everywhere.

The algorithm uses a face-list representation during construction, then calls `from_triangles` at the end to build the half-edge mesh. This avoids half-edge surgery and reuses the battle-tested builder.

## Module

`module cg::hull;` — new file `src/hull/hull_3d.c3`. Same module as `hull_2d`.

Imports: `cg`, `cg::geometry` (for `orient_3d`, `GEOMETRY_EPSILON`), `cg::half_edge` (for `from_triangles`), `std::collections::map` (for `HashMap` edge map).

Umbrella addition (`src/c3cg.c3i`, `module cg::hull;` section):

```c3
fn HalfEdgeMesh? hull_3d(Allocator alloc, Vec3f[] positions);
```

## Public API

```c3
fn HalfEdgeMesh? hull_3d(Allocator alloc, Vec3f[] positions);
```

Returns a caller-owned `HalfEdgeMesh` (closed, triangular, no boundary). Caller must `defer mesh.destroy()`.

## algorithm

### Face-list representation

During construction, the hull is represented as a dynamic list of triangle faces:

```c3
struct Face {
    VertexIndex v0, v1, v2;  // vertex indices into positions[]
    bool alive;               // false = removed (visible face)
}
```

A **directed edge map** `HashMap{int[<2>], FaceIndex}` maps each directed edge `(v0,v1)` to the face on its left. Each face registers 3 directed edges. Horizon detection: for a visible face's edge `(u,v)`, look up `edge_map[(v,u)]` (the reverse edge). If the reverse edge's face is NOT visible, `(u,v)` is a horizon edge.

Horizon edges are collected in cyclic order by following the reverse-edge map: after horizon edge `(u,v)`, the next horizon edge starts at `v`. This guarantees the stitch step produces a manifold cap.

### Phase 0 — Find initial tetrahedron

Scan input for 4 non-coplanar points using `orient_3d(a,b,c,d)` (signed tetrahedron volume). If no 4 non-coplanar points exist, fault `DEGENERATE_INPUT`.

Build 4 outward-facing triangles. For each face, orient vertices so that the fourth tetrahedron point lies inside (negative `orient_3d` relative to the face). Use epsilon-aware comparison: `orient_3d(a,b,c,d,GEOMETRY_EPSILON) != PREDICATE_POSITIVE` means the point is inside or on the face.

Edge map: register all 6 directed edges (3 per face, with reverse pair for each shared edge).

### Phase 1 — Process remaining points

For each remaining point `p`:

1. **Visibility** — for each alive face `(v0,v1,v2)`, compute `orient_3d(v0,v1,v2,p,GEOMETRY_EPSILON)`. If `PREDICATE_POSITIVE` → face is visible from `p` (mark `alive = false`). If no faces are visible, the point is inside the hull — skip it.

2. **Horizon** — for each visible face, for each of its 3 directed edges `(u,v)`:
   - Look up the adjacent face via reverse edge `edge_map[(v,u)]`
   - If the adjacent face is NOT visible, `(u,v)` is a horizon edge. Append to horizon list.

   Horizon edges are naturally ordered: after `(u,v)`, the next starts at `v`.

3. **Stitch** — for each horizon edge `(u,v)`, add a new face `(v,u,p)`. The orientation `(v,u,p)` replaces the visible face's outward-facing orientation. Register the new face's 3 directed edges in the edge map.

4. **Remove** — swap-remove all visible faces from the face list and update face indices in the edge map. Must be done before processing the next point.

### Phase 2 — Build HalfEdgeMesh

Collect all alive face vertices into `uint[]` index arrays (3 per face). Call `from_triangles(alloc, positions, indices)` — note: `from_triangles` takes `uint[]` for indices. Validate with `mesh.validate()`.

### Phase 3 — Return

Return the validated closed triangular mesh.

## Predicates

Add `orient_3d` to `cg::geometry` (`src/geometry/predicates.c3`), matching the existing public predicate convention (`PredicateSign` return, epsilon parameter):

```c3
fn PredicateSign orient_3d(Vec3f a, Vec3f b, Vec3f c, Vec3f d, float epsilon = DEFAULT_PREDICATE_EPSILON);
```

Internally, `orient_3d` computes the signed tetrahedron volume via `det3(b-a, c-a, d-a)` and classifies with `classify_predicate`. The raw determinant is a private helper.

Umbrella addition (`src/c3cg.c3i`, `module cg::geometry;` section):

```c3
fn PredicateSign orient_3d(Vec3f a, Vec3f b, Vec3f c, Vec3f d, float epsilon = DEFAULT_PREDICATE_EPSILON);
```

## Faults

No new `faultdef` entries — reuses existing faults:

| Fault              | When                                                     |
| ------------------ | -------------------------------------------------------- |
| `EMPTY_INPUT`      | `positions.len == 0`                                     |
| `DEGENERATE_INPUT` | < 4 non-coplanar distinct points, or all points coplanar |

## Memory

| Allocation     | Owner    | Lifetime                     |
| -------------- | -------- | ---------------------------- |
| `Face[]` list  | Internal | `alloc`, freed before return |
| Edge map       | Internal | `alloc`, freed before return |
| `HalfEdgeMesh` | Caller   | Must call `mesh.destroy()`   |

Intermediate arrays are allocated via `mem::alloc::new_array(alloc, T, (sz) count)` with `defer catch free(...)` on the following line, consistent with `src/half_edge/builder.c3`. On the success path, scratch arrays (dedup buffer, edge map) are freed before return. On fault paths, `defer catch` handles cleanup.

## Edge Cases

- **Duplicate points**: deduplicate exact (x,y,z) duplicates before building. Build a compacted positions array so the output mesh has no unused isolated vertices from discarded duplicates.
- **Collapsed tetrahedron**: if initial 4 points form a degenerate tetrahedron (orient_3d ≈ 0 with epsilon), continue scanning. Fault if none found.
- **Inside/coplanar points**: points inside or coplanar with existing faces produce no visible faces — skip them. This matches standard incremental hull behavior.

## Tests

File: `test/test_hull_3d.c3`, module `test`.

| Test                 | Input                        | Expected                                                        |
| -------------------- | ---------------------------- | --------------------------------------------------------------- |
| Empty input          | `{}`                         | `EMPTY_INPUT`                                                   |
| 1 point              | 1 point                      | `DEGENERATE_INPUT`                                              |
| 2 points             | 2 points                     | `DEGENERATE_INPUT`                                              |
| 3 points             | 3 points                     | `DEGENERATE_INPUT`                                              |
| Coplanar points      | 4+ coplanar points           | `DEGENERATE_INPUT`                                              |
| Tetrahedron          | 4 non-coplanar points        | 4 faces, closed mesh, `validate()` OK                           |
| Cube corners         | 8 corners of unit cube       | 12 faces (6 × 2), closed mesh                                   |
| Cube + interior      | 8 corners + center point     | 12 faces, interior point not in mesh                            |
| Cube + face coplanar | 8 corners + point on face    | 12 faces, coplanar point skipped                                |
| Duplicate points     | 8 corners + exact duplicates | 12 faces, no isolated vertices                                  |
| Convexity            | Cube hull                    | all points satisfy `orient_3d(face, point) <= 0` for every face |

Each test verifies: `mesh.validate()` passes, `is_boundary` false everywhere, and for convexity all input points are inside or on every hull face.

## Non-goals

- No `sort::quicksort` — points processed in input order
- No reuse of `hull_2d` code — 3D uses different algorithm family
- No 4D+ hull
- No parallelization
- No approach B (in-place half-edge mutation) — deferred as potential optimization
