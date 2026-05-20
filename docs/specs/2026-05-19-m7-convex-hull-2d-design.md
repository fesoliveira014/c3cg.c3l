# M7 — Convex Hull 2D Design

## Overview

Andrew's monotone chain algorithm for computing the 2D convex hull of a set of points. Given `Vec3f[]` positions (only x,y used; z ignored), returns `int[]` of indices into the input array in CCW order.

The algorithm runs in O(n log n) time (dominated by the sort phase) with O(n) scratch space.

## Module

`module cg::hull;` — single file `src/hull/hull_2d.c3` (`.c3` extension for implementation files; C3 0.8.0 prohibits function bodies in `.c3i`).

Imports: `cg`, `cg::geometry` (for `orient_2d`), `std::sort` (for `sort::quicksort`).

## Public API

```c3
fn int[]? hull_2d(Allocator alloc, Vec3f[] positions);
```

Returns an owned `int[]` of hull vertex indices in CCW order. The caller must `defer free(result)`.

## Algorithm

Andrew's monotone chain in four phases:

### Phase 1 — Index sort and deduplication

Create `int[] indices = [0, 1, 2, ..., n-1]`. Sort lexicographically by `(positions[i].x, positions[i].y)`. After sorting, compact the index array to remove points with duplicate `(x, y)` coordinates (only the first occurrence of each projected position is kept).

### Phase 2 — Lower hull

Sweep left-to-right over deduplicated indices. Maintain a stack `hull` (dynamic `int[]`). For each candidate index `p`:

- While `hull.len >= 2` and `orient_2d(hull[last-1].xy, hull[last].xy, p.xy) == PREDICATE_NEGATIVE`, pop the stack (pop only on strict CW turns; collinear and CCW turns are kept, preserving collinear boundary points).
- Push `p`.

### Phase 3 — Upper hull

Same as lower, but sweep right-to-left (reverse iteration over deduplicated indices), skipping the rightmost point (already on the lower hull).

### Phase 4 — Combine & validate

Trim the last element of the hull stack (duplicate of the first element). If the resulting hull has < 3 points, fault `DEGENERATE_INPUT`. Also fault if the deduplication reduced the point count below 3.

The output is CCW by construction of Andrew's monotone chain.

## Faults

No new `faultdef` entries needed — reuses existing faults from `src/faults.c3i`:

| Fault              | When                                       |
| ------------------ | ------------------------------------------ |
| `EMPTY_INPUT`      | `positions.len == 0`                       |
| `DEGENERATE_INPUT` | Fewer than 3 non-collinear distinct points |

## Memory

Three allocations via the caller-supplied allocator:

1. Index array: `mem::alloc::new_array(alloc, int, (sz) positions.len)`
2. Hull stack: `mem::alloc::new_array(alloc, int, (sz)(2 * positions.len))` (worst case all points are collinear)
3. Output hull: `mem::alloc::new_array(alloc, int, (sz) hull_count)`

Deferred free on failure: `defer catch free(...)` on the line after each allocation, consistent with `src/half_edge/builder.c3`.

## Tests

File: `test/test_hull_2d.c3`, module `test`.

### Test cases

1. **Empty input** — `hull_2d(mem, {})` faults `EMPTY_INPUT`.
2. **Single point** — faults `DEGENERATE_INPUT`.
3. **Two points** — faults `DEGENERATE_INPUT`.
4. **Square** — 4 corners + 1 interior point → hull is the 4 corners in CCW order.
5. **Collinear points** — all points on a line → faults `DEGENERATE_INPUT`.
6. **Square + collinear edge points** — corners (0,0), (1,0), (1,1), (0,1) with extra points (0.5, 0), (0, 0.5) on edges → all boundary points (corners + edge-collinear) are included in the hull.
7. **Duplicate projected points** — points with identical (x,y) but different z → duplicates are deduplicated, hull is correct.
8. **Exact duplicate points** — identical points → deduplicated, hull is correct.
9. **Random cloud** — hull of random 2D points has all input points inside or on the hull (cross-product check).

## Non-goals

- No `Vec2f` convenience overload. Callers with `Vec2f[]` can wrap or inline the conversion.
- No 3D hull. That's M8.
- No robust predicates upgrade. Uses the existing `orient_2d` from `cg::geometry`.
