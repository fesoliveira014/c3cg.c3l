# M14 — Polygon Triangulation, Loop Subdivision, Primitives Design

**Date:** 2026-05-22
**Status:** draft

## Overview

M14 adds three independent capabilities:
1. Monotone polygon triangulation (O(n log n))
2. One-step Loop subdivision (closed meshes only)
3. Platonic solids + icosphere primitives

New module declarations needed: `module cg::triangulate;`, `module cg::subdivide;`, `module cg::primitives;` in the umbrella `src/c3cg.c3i`.

New faults to add to the `faultdef` block in `src/c3cg.c3i`:
- `INSUFFICIENT_VERTICES` — polygon has fewer than 3 vertices
- `NON_SIMPLE_POLYGON` — polygon is degenerate (collinear chains, duplicate vertices, self-intersecting)

## 1. Polygon Triangulation (`cg::triangulate`)

### API

```c3
fn int[]? triangulate_polygon(Allocator alloc, Vec3f[] polygon);
```

Input: simple polygon vertices in CCW order (x,y only; z ignored).
Output: flat `int[]` of triangle indices (3 per triangle), indices into input polygon.
Faults:
- `INSUFFICIENT_VERTICES` if `polygon.len < 3`
- `NON_SIMPLE_POLYGON` if polygon is degenerate (collinear edge chains, duplicate vertices, or self-intersecting)

### Self-intersection scope

True self-intersection detection (e.g., bow-tie crossing edges) requires a separate O(n log n) sweep-line intersection pass (Bentley–Ottmann). For M14, the algorithm catches the degenerate cases it naturally encounters: collinear consecutive edges and duplicate vertices. Tricky self-intersections (bent bow-ties where edges cross at non-vertex points) may not be detected and the triangulation output for such input is undefined. Callers are responsible for ensuring simple input.

### Algorithm

Two-phase monotone decomposition approach:

**Phase 1 — Monotone decomposition (O(n log n)):**
1. Sort vertices by y (decreasing). If equal y, use x as tie-breaker.
2. Classify each vertex: start, end, split, merge, regular (left chain), regular (right chain) using `orient_2d` on neighboring edges.
3. Plane sweep maintaining active edges ordered by x at current y-scanline. Use sorted array + binary search.
4. At split vertices: find the edge directly above, add a diagonal to its helper.
5. At merge vertices: find the edge directly above; if its helper is a merge vertex, add a diagonal.
6. Add diagonals for regular vertices when the helper of adjacent edge needs connecting.
7. Output: list of y-monotone sub-polygons (each stored as vertex index runs into original polygon array).

**Phase 2 — Triangulate monotone polygons (O(n) total):**
1. For each monotone sub-polygon:
   - Merge left and right chains (already y-sorted) into descending y order.
   - Maintain a reflex chain stack.
   - For each vertex in order: pop from stack while `orient_2d(stack[-2], stack[-1], vertex)` signals valid triangle, emit (stack[-2], stack[-1], vertex), push vertex.
2. Collect all triangle indices into flat output array.

### Dependencies

- `std::sort` for vertex sorting
- `orient_2d` from `cg::geometry` for vertex classification and triangle validity

## 2. Loop Subdivision (`cg::subdivide`)

### API

```c3
fn HalfEdgeMesh? loop_subdivide(Allocator alloc, HalfEdgeMesh* mesh);
```

Input: triangular closed mesh (no boundary edges).
Output: fresh mesh with each triangle split into 4 sub-triangles.
Faults:
- `NON_TRIANGLE_FACE` if any face is not degree 3
- `BOUNDARY_HALF_EDGE` if mesh has boundary edges (closed meshes only)

### Algorithm

1. Validate all faces are triangular and mesh is closed.
2. For each edge, compute the new vertex position:
   - Internal edge: `3/8 * (A + B) + 1/8 * (C + D)` where A,B are edge endpoints and C,D are opposite vertices (the vertices opposite the edge in each of the two adjacent triangles).
   - Since mesh is closed, all edges are internal.
3. For each original vertex, compute the updated position:
   - `(1 - n*β) * V + β * sum(neighbors)` where n = degree, β = `1/n * (5/8 - (3/8 + 1/4*cos(2π/n))²)`.
4. Build new mesh: for each original triangle (A,B,C), the edges produce midpoints (ab, bc, ca). Create 4 sub-triangles: (A, ab, ca), (B, bc, ab), (C, ca, bc), (ab, bc, ca).

## 3. Primitives (`cg::primitives`)

### API

```c3
// platonic.c3
fn HalfEdgeMesh tetrahedron(Allocator alloc);
fn HalfEdgeMesh octahedron(Allocator alloc);
fn HalfEdgeMesh icosahedron(Allocator alloc);
fn HalfEdgeMesh triangulated_cube(Allocator alloc);

// icosphere.c3
fn HalfEdgeMesh icosphere(Allocator alloc, int subdivisions, float radius);
```

All platonic solids produce unit-radius inscribed shapes (vertices on unit sphere).
Vertices of `triangulated_cube` are at `±1` (corners of the axis-aligned cube of side length 2, not normalized to unit sphere). The cube faces are triangulated with a single diagonal each (12 triangles).

`icosphere` builds an icosahedron, applies `loop_subdivide` N times, then reprojects all vertex positions to the sphere surface by normalizing and multiplying by `radius`.

Faults:
- `DEGENERATE_INPUT` if `subdivisions < 0` or `radius <= 0`
- Propagates faults from `loop_subdivide`

## Files

| File | Content |
|------|---------|
| `src/triangulate/monotone.c3` | `triangulate_polygon` (monotone decomposition + triangulation) |
| `src/subdivide/loop.c3` | `loop_subdivide` |
| `src/primitives/platonic.c3` | `tetrahedron`, `octahedron`, `icosahedron`, `triangulated_cube` |
| `src/primitives/icosphere.c3` | `icosphere` |
| `test/test_triangulate.c3` | Polygon triangulation tests |
| `test/test_subdivide.c3` | Loop subdivision tests |
| `test/test_primitives.c3` | Primitives tests |

## Test Plan

### test_triangulate.c3

1. **Triangle** — 3-vertex CCW triangle → 1 triangle, indices match input
2. **Square** — 4-vertex CCW square → 2 triangles, covers entire area
3. **Pentagon** — 5-vertex CCW → 3 triangles
4. **L-shaped polygon** — 8-vertex non-convex → valid triangulation (no self-intersections among output triangles)
5. **Monotone polygon** — trapezoid → correct triangulation
6. **Star-shaped polygon** — 10-vertex star → valid triangulation (exercises split/merge vertices)
7. **Insufficient vertices** → `INSUFFICIENT_VERTICES` fault
8. **Duplicate vertices** → `NON_SIMPLE_POLYGON` fault

### test_subdivide.c3

9. **Tetrahedron** — `loop_subdivide` → 16 faces, 10 vertices
10. **Icosahedron** — `loop_subdivide` → 80 faces, 42 vertices
11. **Original mesh unchanged** — positions, faces, half_edges unmodified after subdivision
12. **Closed mesh stays closed** — no boundary edges after subdivision
13. **Non-triangle face** → `NON_TRIANGLE_FACE` fault
14. **Boundary mesh** → `BOUNDARY_HALF_EDGE` fault

### test_primitives.c3

15. **Tetrahedron** — 4 faces, 4 vertices, closed, vertices on unit sphere
16. **Octahedron** — 8 faces, 6 vertices, closed
17. **Icosahedron** — 20 faces, 12 vertices, closed
18. **Triangulated cube** — 12 faces, 8 vertices, closed, vertices at `±1`
19. **Icosphere** — `icosphere(alloc, 0, 1.0f)` → same face/vertex count as icosahedron; `icosphere(alloc, 2, 5.0f)` → 320 faces, all vertices on sphere radius 5.0 within tolerance
20. **Icosphere degenerate** — `icosphere(alloc, -1, 1.0f)` → `DEGENERATE_INPUT`; `icosphere(alloc, 0, 0.0f)` → `DEGENERATE_INPUT`
