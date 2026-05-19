# M7 — Convex Hull 2D Design

## Overview

Andrew's monotone chain algorithm for computing the 2D convex hull of a set of points. Given `Vec3f[]` positions (only x,y used; z ignored), returns `int[]` of indices into the input array in CCW order.

The algorithm runs in O(n log n) time (dominated by the sort phase) with O(n) scratch space.

## Module

`module cg::hull;` — single file `src/hull/hull_2d.c3i` (`.c3i` extension for library code, consistent with C3 library conventions).

Imports: `cg`, `cg::geometry` (for `orient_2d`), `std::sort`.

## Public API

```c3
fn int[]? hull_2d(Allocator alloc, Vec3f[] positions);
```

Returns an owned `int[]` of hull vertex indices in CCW order. The caller must `defer free(result)`.

## Algorithm

Andrew's monotone chain in four phases:

### Phase 1 — Index sort

Create `int[] indices = [0, 1, 2, ..., n-1]`. Sort lexicographically by `(positions[i].x, positions[i].y)`.

### Phase 2 — Lower hull

Sweep left-to-right over sorted indices. Maintain a stack `hull` (dynamic `int[]`). For each candidate index `p`:

- While `hull.len >= 2` and `orient_2d(hull[last-1].xy, hull[last].xy, p.xy) != PREDICATE_POSITIVE`, pop the stack (epsilon = 0.0 to preserve collinear boundary points).
- Push `p`.

### Phase 3 — Upper hull

Same as lower, but sweep right-to-left (reverse iteration). The upper hull sweep starts with the stack retained from the lower sweep (minus the last element to avoid the turnaround duplicate).

### Phase 4 — Combine & validate

Trim the duplicate seam vertices. If the resulting hull has < 3 points, fault `DEGENERATE_INPUT`. Otherwise copy the hull indices into a fresh `int[]` and return.

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
2. Hull stack: `mem::alloc::new_array(alloc, int, (sz) positions.len)` (max hull size ≤ input size)
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
6. **Square + collinear edge points** — corners (0,0), (1,0), (1,1), (0,1) with extra points (0.5, 0), (0.5, 1) on edges → hull is the 4 corners. Collinear edge points are kept on the hull boundary.
7. **Random cloud** — hull of random 2D points has all input points inside or on the hull (cross-product check).

## Non-goals

- No `Vec2f` convenience overload. Callers with `Vec2f[]` can wrap or inline the conversion.
- No 3D hull. That's M8.
- No robust predicates upgrade. Uses the existing `orient_2d` from `cg::geometry`.
