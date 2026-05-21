# M9 — Delaunay 2D Design

## Overview

Bowyer-Watson algorithm for planar Delaunay triangulation. Given `Vec3f[]` positions (z ignored), returns a triangular `HalfEdgeMesh` with planar boundary. The algorithm wraps input in a super-triangle, inserts points one at a time in shuffled order, removes triangles whose circumcircle contains the new point, retriangulates the cavity, compacts, and strips the super-triangle.

## Module

`module cg::delaunay;` — new file `src/delaunay/delaunay_2d.c3`.

Imports: `cg`, `cg::geometry` (`in_circle_2d`, `orient_2d`), `cg::half_edge` (`from_triangles`), `std::collections::map` (HashMap for edge adjacency).

Umbrella addition (`src/c3cg.c3i`):

```c3
module cg::delaunay;
import cg;

struct DelaunayOptions {
    bool has_bounding_box;
    Aabb bounding_box;
}

fn HalfEdgeMesh? delaunay_2d(Allocator alloc, Vec3f[] positions, DelaunayOptions options = {});
```

## Public API

```c3
struct DelaunayOptions {
    Aabb? bounding_box;  // optional — if set, must contain all input points (else DEGENERATE_INPUT)
}

fn HalfEdgeMesh? delaunay_2d(Allocator alloc, Vec3f[] positions, DelaunayOptions options = {});
```

Returns a caller-owned `HalfEdgeMesh` (triangular, with planar boundary). Caller must `defer mesh.destroy()`.

## Algorithm

### Data structures

```c3
struct DelaunayFace {
    VertexIndex v0, v1, v2;  // CCW order
    bool alive;               // false = bad triangle, pending removal
}
```

All arrays allocated once before the insertion loop.

| Array                            | Size          | Purpose                                                                    |
| -------------------------------- | ------------- | -------------------------------------------------------------------------- |
| `DelaunayFace[]`                 | `3n + 10`     | Face list (≥ 2n final faces + transient dead faces during retriangulation) |
| Edge map `HashMap{int[<2>],int}` | `init()` once | Directed edge → face index. Per-iteration `clear()` + rebuild              |
| `int[]` indices                  | `n`           | Shuffled insertion order                                                   |
| `int[]` boundary_from            | `3n`          | Cavity boundary from-vertices                                              |
| `int[]` boundary_to              | `3n`          | Cavity boundary to-vertices                                                |
| `Vec3f[]` working_positions      | `n + 3`       | Deduped positions + super-triangle vertices                                |

### Phase 0 — Dedup, collinearity check, shuffle

1. Deduplicate exact `(x,y)` duplicates (z ignored). Keep first occurrence.
2. Collinearity scan: find first two distinct `(x,y)` points, then scan remaining for a third point with `orient_2d != PREDICATE_ZERO`. If none found → `DEGENERATE_INPUT`. If < 3 distinct points → `DEGENERATE_INPUT`.
3. Build working positions: compacted deduped points (unique_count ≤ n). Shuffle insertion order with a deterministic seed for reproducibility.

### Phase 1 — Super-triangle

If `options.has_bounding_box`, validate all deduped points are inside it (x/y only, z ignored); if not, fault `DEGENERATE_INPUT`. Otherwise auto-compute AABB from points.

Margin = 10 × diagonal. Super-triangle = 3 vertices appended to `working_positions` at indices `unique_count`, `unique_count+1`, `unique_count+2`. Formula: construct triangle strictly enclosing the AABB with margin.

Build initial face `(unique_count, unique_count+1, unique_count+2)` as the only alive face (CCW, ensured by `orient_2d` swap). Register its 3 directed edges in the edge map.

### Phase 2 — Point insertion

For each shuffled point `p` (index into `working_positions`):

1. **Bad triangles** — for each alive face, project vertices to `Vec2f { positions[v].x, positions[v].y }`. Check `in_circle_2d(a, b, c, p_xy) == PREDICATE_POSITIVE`. If true, mark `alive = false`. If no bad triangles, skip point (duplicate/coincident after dedup — should not happen, defensive skip).

2. **Cavity boundary** — for each bad face, for each directed edge `(u,v)`:
   - Look up reverse edge `(v,u)` in edge map
   - If reverse absent (super-triangle edge) OR reverse exists AND adjacent face is alive → `(u,v)` is cavity boundary
   - Append `u` to `boundary_from`, `v` to `boundary_to`

3. **Retriangulate** — compact bad faces first: swap-remove them, reclaiming dead slots. Then for each boundary edge `(u,v)`, add new face `(u,v,p)` in CCW order. Register its 3 directed edges in the edge map.

4. **Rebuild edge map** — `edge_map.clear()` then re-insert all alive faces.

### Phase 3 — Strip super-triangle

Remove any face referencing super-triangle vertices (indices `unique_count` through `unique_count+2`). Compact.

### Phase 4 — Build output mesh

Build final compacted positions from `working_positions`, including only vertices referenced by surviving faces. Build remap array from working indices to final indices.

Collect face vertices into `uint[]` index arrays using final remap. Call `from_triangles(alloc, final_positions, indices)`. `defer catch mesh.destroy()`.

`mesh.validate()!` then free all scratch arrays. Return mesh.

## Predicates

All predicates already exist in `cg::geometry`. Since they take `Vec2f`, project `Vec3f` positions:

```c3
Vec2f a = { positions[v0].x, positions[v0].y };
Vec2f b = { positions[v1].x, positions[v1].y };
Vec2f c = { positions[v2].x, positions[v2].y };
PredicateSign result = geometry::in_circle_2d(a, b, c, p_xy);
```

| Predicate               | Use                                         | Compare to              |
| ----------------------- | ------------------------------------------- | ----------------------- |
| `in_circle_2d(a,b,c,p)` | Triangle is bad if p is inside circumcircle | `== PREDICATE_POSITIVE` |
| `orient_2d(a,b,c)`      | Collinearity check, face orientation        | `!= PREDICATE_ZERO`     |

Cocircular points (`PREDICATE_ZERO`) are NOT treated as bad — strict-positive threshold. This produces a valid Delaunay triangulation, though not necessarily the only one.

## Faults

No new `faultdef` entries — reuses existing:

| Fault              | When                                                                                   |
| ------------------ | -------------------------------------------------------------------------------------- |
| `EMPTY_INPUT`      | `positions.len == 0`                                                                   |
| `DEGENERATE_INPUT` | < 3 non-collinear distinct points, or explicit bounding box doesn't contain all points |

## Memory

| Allocation                  | When                  | Freed                                                              |
| --------------------------- | --------------------- | ------------------------------------------------------------------ |
| `DelaunayFace[]`            | Phase 0 (`3n + 10`)   | Explicit `free` after `validate()`                                 |
| Edge map `HashMap`          | Phase 0 (`init` once) | `free()` after `validate()` + `defer catch`                        |
| `int[]` indices             | Phase 0 (`n`)         | Explicit `free` after `validate()`                                 |
| `int[]` boundary_from/to    | Phase 0 (`3n` each)   | Explicit `free` after `validate()`                                 |
| `Vec3f[]` working_positions | Phase 0 (`n + 3`)     | Explicit `free` after `validate()`                                 |
| `Vec3f[]` final_positions   | Phase 4               | Explicit `free` after `validate()`                                 |
| `int[]` final_remap         | Phase 4               | Explicit `free` after `validate()`                                 |
| `uint[]` tri_indices        | Phase 4               | Explicit `free` after `validate()` + `defer catch`                 |
| `HalfEdgeMesh`              | Phase 4               | Caller via `mesh.destroy()`. `defer catch mesh.destroy()` on fault |

Cleanup order: `validate()` before explicit frees (no double-free from `defer catch`).

## Edge Cases

- **Duplicate (x,y) points**: deduped early, first occurrence kept. Different z values are ignored.
- **Collinear after dedup**: collinearity scan finds first 2 distinct points, then scans all remaining for non-collinear third. If none found → `DEGENERATE_INPUT`.
- **Cocircular points**: `PREDICATE_ZERO` means not bad. Result is a valid (non-unique) Delaunay triangulation.
- **Empty cavity**: defensive skip (should not occur after dedup + valid super-triangle).
- **Explicit bounding box**: validated to contain all deduped points before use. If not → `DEGENERATE_INPUT`.
- **Super-triangle sizing**: margin of 10× AABB diagonal. Super-triangle vertices are far enough that no input point's circumcircle can reach them.
- **Deterministic shuffle**: seeded with a fixed constant for test reproducibility.

## Tests

File: `test/test_delaunay_2d.c3`, module `test`.

| #   | Test                | Input                                      | Expected                                    |
| --- | ------------------- | ------------------------------------------ | ------------------------------------------- |
| 1   | Empty               | `{}`                                       | `EMPTY_INPUT`                               |
| 2   | 1 point             | 1 point                                    | `DEGENERATE_INPUT`                          |
| 3   | 2 points            | 2 points                                   | `DEGENERATE_INPUT`                          |
| 4   | Collinear           | 3+ collinear points                        | `DEGENERATE_INPUT`                          |
| 5   | Collinear-first     | first 3 collinear, 4th non-collinear       | Valid triangulation                         |
| 6   | Duplicate (x,y)     | 4 points, two share (x,y) with different z | 3 unique vertices, valid, no isolated       |
| 7   | Triangle            | 3 non-collinear                            | 1 face, bounded, no super-triangle vertices |
| 8   | Square              | 4 unit-square corners                      | 2 faces, Delaunay property                  |
| 9   | Square + center     | 4 corners + center                         | All faces have empty circumcircles          |
| 10  | Regular grid        | 3×3 grid of 9 points                       | All edges locally Delaunay                  |
| 11  | Cocircular          | 4 points on a circle                       | Either diagonal is valid Delaunay           |
| 12  | Bounding box option | 4 points + explicit AABB                   | Uses provided box, valid                    |

Each Delaunay test verifies:

- No super-triangle vertices in output
- All faces are CCW (`orient_2d > 0`)
- For every face, `in_circle_2d(face, every_other_point) != PREDICATE_POSITIVE`

## Non-goals

- No constrained Delaunay
- No robust adaptive predicates
- No bounding polygon (only AABB) — future `DelaunayOptions` extension
- No 3D Delaunay
