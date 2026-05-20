# M9 — Delaunay 2D Design

## Overview

Bowyer-Watson algorithm for planar Delaunay triangulation. Given `Vec3f[]` positions (z ignored), returns a triangular `HalfEdgeMesh` with planar boundary. The algorithm wraps input in a super-triangle, inserts points one at a time in shuffled order, removes triangles whose circumcircle contains the new point, retriangulates the cavity, and strips the super-triangle.

## Module

`module cg::delaunay;` — new file `src/delaunay/delaunay_2d.c3`.

Imports: `cg`, `cg::geometry` (`in_circle_2d`, `orient_2d`), `cg::half_edge` (`from_triangles`), `std::collections::map` (HashMap for edge adjacency).

Umbrella addition (`src/c3cg.c3i`):

```c3
module cg::delaunay;
import cg;

struct DelaunayOptions {
    Aabb? bounding_box;
}

fn HalfEdgeMesh? delaunay_2d(Allocator alloc, Vec3f[] positions, DelaunayOptions options = {});
```

## Public API

```c3
struct DelaunayOptions {
    Aabb? bounding_box;  // optional — if set, super-triangle encloses this; else auto-computed from input
}

fn HalfEdgeMesh? delaunay_2d(Allocator alloc, Vec3f[] positions, DelaunayOptions options = {});
```

Returns a caller-owned `HalfEdgeMesh` (triangular, with planar boundary). Caller must `defer mesh.destroy()`.

## Algorithm

### Preallocation

All arrays allocated once before the insertion loop. No allocations during point insertion.

| Array | Size | Purpose |
|-------|------|---------|
| `DelaunayFace[]` | `2n + 10` | Active triangles (max 2n for planar triangulation + margin) |
| Edge map | `HashMap{int[<2>], int}` | Directed edge → face index. `init()` once; per-iteration `clear()` + rebuild (reuses table, no allocation) |
| `int[] indices` | `n` | Shuffled insertion order |
| `int[] boundary` | scratch, preallocated | Cavity boundary edges per insertion |

### Phase 0 — Dedup, validate, shuffle

Remove exact `(x,y)` duplicates (z ignored). If < 3 non-collinear distinct points, fault `DEGENERATE_INPUT`. Check collinearity via `orient_2d` on first 3 points.

Shuffle remaining indices (Fisher-Yates or equivalent). Prealloc face array and edge map.

### Phase 1 — Super-triangle

Compute or use provided AABB. Margin = 10 × diagonal. Super-triangle = 3 vertices appended to positions (indices n, n+1, n+2). Build initial face list: 1 triangle `(n, n+1, n+2)` covering all input points. Register its 3 directed edges in the edge map.

```c3
struct DelaunayFace {
    VertexIndex v0, v1, v2;  // CCW order
    bool alive;               // false = removed (bad triangle)
}
```

### Phase 2 — Point insertion

For each shuffled point `p`:

1. **Bad triangles** — for each alive face, check `in_circle_2d(v0, v1, v2, p) > 0` (epsilon-aware via `PREDICATE_POSITIVE`). If true, mark `alive = false`.

2. **Cavity boundary** — for each bad face, for each directed edge `(u,v)`:
   - Look up reverse edge `(v,u)` in edge map
   - If adjacent face is alive (not bad), `(u,v)` is a cavity boundary edge
   - Append `(u,v)` to the boundary list

3. **Retriangulate** — for each boundary edge `(u,v)`, add new face `(u,v,p)` in CCW order. Register its 3 directed edges in the edge map. New face inherits the orientation of the cavity boundary.

4. **Compact** — swap-remove bad faces from the face array. `edge_map.clear()` + rebuild from alive faces.

### Phase 3 — Strip super-triangle

Remove any face referencing super-triangle vertices (indices n, n+1, n+2). Compact remaining faces. The resulting mesh has a planar boundary where the super-triangle was removed.

### Phase 4 — Build HalfEdgeMesh

Collect alive face vertices into `uint[]` index arrays. Call `from_triangles(alloc, positions, indices)`. Validate with `mesh.validate()`. Return.

## Predicates

All predicates already exist in `cg::geometry`:

| Predicate | Use |
|-----------|-----|
| `in_circle_2d(a,b,c,p)` | Positive → p is inside circumcircle of triangle (a,b,c) — triangle is "bad" |
| `orient_2d(a,b,c)` | Degenerate detection (collinearity). Super-triangle orientation check. |

## Faults

No new `faultdef` entries — reuses existing:

| Fault | When |
|-------|------|
| `EMPTY_INPUT` | `positions.len == 0` |
| `DEGENERATE_INPUT` | < 3 non-collinear distinct points |

## Memory

| Allocation | When | Freed |
|-----------|------|-------|
| `DelaunayFace[]` | Phase 0 (2n + 10) | Explicit `free` after `from_triangles` |
| Edge map `HashMap` | Phase 0 (`init` once) | `free()` + `defer catch` |
| `int[] indices` | Phase 0 (n) | Explicit `free` |
| `int[] boundary` | Phase 0 (scratch, 3n) | Explicit `free` |
| `HalfEdgeMesh` | Phase 4 | Caller via `mesh.destroy()` |

Cleanup order: `validate()` before explicit frees, consistent with hull_3d pattern.

## Edge Cases

- **Duplicate points**: deduplicate exact `(x,y)` before processing. z is ignored.
- **Collinear input**: detected by `orient_2d` scan — fault early.
- **Cocircular points**: Bowyer-Watson naturally handles these. The in-circle test marks all triangles in the cocircular set as bad when the last point is inserted. Either retriangulation is valid Delaunay.
- **Explicit bounding box**: when `options.bounding_box` is set, use it for super-triangle sizing instead of auto-computed AABB. Useful when the desired domain extends beyond input points.
- **Empty cavity**: if no triangles are bad, the point is inside an existing triangle's circumcircle-free zone — skip it (duplicate or already triangulated).
- **HashMap per-iteration allocations**: calling `edge_map.clear()` then re-inserting keys reuses the existing hash table. The only heap allocation is the initial `init()`.

## Tests

File: `test/test_delaunay_2d.c3`, module `test`.

| Test | Input | Expected |
|------|-------|----------|
| Empty input | `{}` | `EMPTY_INPUT` |
| 1 point | 1 point | `DEGENERATE_INPUT` |
| 2 points | 2 points | `DEGENERATE_INPUT` |
| Collinear | 3+ collinear points | `DEGENERATE_INPUT` |
| Triangle | 3 non-collinear points | 1 face, bounded |
| Square | 4 unit-square corners | 2 faces, Delaunay property holds |
| Square + center | 4 corners + center point | All faces empty-circle check passes |
| Regular grid | 3×3 grid of 9 points | All edges locally Delaunay |
| Cocircular | 4 points on a circle | Either diagonal valid |
| Bounding box | 4 points + explicit AABB option | Uses provided box, correct Delaunay |

Each Delaunay-property test verifies: for every face, `in_circle_2d` of its 3 vertices + every other point is not positive (no point inside any circumcircle).

## Non-goals

- No constrained Delaunay (edge constraints)
- No robust adaptive predicates (uses existing `in_circle_2d`)
- No super-triangle bounding polygon (only AABB) — future `DelaunayOptions` extension
- No 3D Delaunay — that's M12 via spherical hull
