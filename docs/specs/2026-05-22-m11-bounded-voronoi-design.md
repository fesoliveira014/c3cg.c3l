# M11 — Bounded Voronoi (in_polygon, in_box)

## Overview

Clip unbounded Voronoi cells to a convex polygon bounding region using Sutherland-Hodgman polygon clipping. Produces a polygonal `HalfEdgeMesh` where each face is a bounded Voronoi cell. AABB version is a thin wrapper.

## Module

New file `src/voronoi/bounded.c3` — part of `module cg::voronoi;`.

## Public API

```c3
fn VoronoiDiagram? in_polygon(Allocator alloc, Vec3f[] sites, Vec3f[] polygon);
fn VoronoiDiagram? in_box(Allocator alloc, Vec3f[] sites, Aabb bbox);
```

Both return `VoronoiDiagram { mesh, sites }` — same shape as `from_delaunay`.
`sites` contains only the sites whose cells survived clipping (cells entirely outside
are dropped). The mesh is polygonal.

### `in_polygon`

1. Validate `sites.len >= 1` — else `EMPTY_INPUT`.
2. Validate `polygon` is convex and CCW: for each consecutive triple, `orient_2d(prev, curr, next) >= DEFAULT_PREDICATE_EPSILON`. Failure → `NON_CONVEX_BOUNDING_POLYGON`.
3. Compute Delaunay triangulation via `delaunay::delaunay_2d(alloc, sites)!`.
4. Compute unbounded Voronoi via `voronoi::from_delaunay(alloc, &delaunay_mesh)!`.
5. For each face (cell) in the Voronoi mesh:
   a. Walk the face via `mesh.face_vertices(f, out[])` to collect vertex indices.
   b. Gather positions from `mesh.positions[out[i]]` — this is the cell polygon.
   c. Clip against each edge of the bounding polygon using Sutherland-Hodgman (see below).
   d. If clipped polygon has < 3 vertices, discard the cell. Otherwise accumulate.
6. Build a new `HalfEdgeMesh` from surviving polygons via `half_edge::from_polygons(alloc, positions, face_offsets, face_indices)`.
7. Copy sites for surviving cells only.
8. Return `VoronoiDiagram { mesh, sites }`.

### `in_box`

Thin wrapper: convert `Aabb` to a 4-vertex CCW polygon `{min, {max.x, min.y, 0}, max, {min.x, max.y, 0}}`, call `in_polygon`. No fault possible for the polygon itself (AABB is always convex).

### Sutherland-Hodgman

Clips a polygon against a single convex clipping polygon edge by edge.

```
For each clip_edge (a, b) in bounding_polygon:
    For each input_edge (p, q) in current_polygon:
        p_inside = orient_2d(a, b, p) >= 0   // left-of-or-on (CCW polygon)
        q_inside = orient_2d(a, b, q) >= 0
        if p_inside && q_inside:      output q
        if p_inside && !q_inside:     output intersection(line(a,b), line(p,q))
        if !p_inside && q_inside:     output intersection, q
        if !p_inside && !q_inside:    output nothing
```

Intersection of two line segments computed via `line_intersection(a, b, p, q)`.
Only the case where the edges actually cross is needed (one endpoint inside, one outside).
Works with positions (`Vec3f`), using only `.x` and `.y`.

## Faults

| Fault | When |
|-------|------|
| `EMPTY_INPUT` | `sites.len == 0` (existing in `src/c3cg.c3i`) |
| `NON_CONVEX_BOUNDING_POLYGON` | `polygon` is not convex or not CCW (new) |

New faultdef in `src/c3cg.c3i`:

```c3
faultdef NON_CONVEX_BOUNDING_POLYGON;
```

## Mesh construction

`half_edge::from_polygons(alloc, positions, face_offsets, face_indices)`
takes all clipped vertex positions (deduplicated internally) and polygon
indices. Already used by `dual_from_vertices`. Surviving cells become
polygonal faces.

## Memory

`in_polygon` builds two intermediate meshes (Delaunay + unbounded Voronoi),
both destroyed before return. The returned `VoronoiDiagram` owns only the
final clipped mesh and sites array.

## Tests

File: `test/test_voronoi_bounded.c3`

| Test | Input | Expected |
|------|-------|----------|
| 4 sites in unit square | `sites = [(0.25,0.25), (0.75,0.25), (0.75,0.75), (0.25,0.75)]`, `polygon = unit square` | 4 cells, all bounded. Output face vertices on or inside square. All sites retained. |
| Site near corner | Single site at `(0.01, 0.01)` in unit square | Cell clipped to square. Boundary vertices on polygon edges. |
| Site outside polygon | Site at `(-0.5, 0.5)` in unit square | Cell outside → dropped. `sites.len == 0`, `mesh.faces.len == 0`. |
| Diamond polygon | 5 sites inside diamond `[(1,0), (0,1), (-1,0), (0,-1)]` | All 5 cells clipped. Corner cells triangular, center cell quadrilateral. |
| Triangle polygon | 3 sites inside triangle `[(0,0), (2,0), (1,2)]` | All cells bounded. Boundary cells have one edge clipped to triangle side. |
| Many sites, large box | 100 random sites in `[0,10]²` | All cells clipped, no crashes. |
| Non-convex polygon faults | L-shaped polygon (6 vertices) | `NON_CONVEX_BOUNDING_POLYGON` |
| Empty sites | `sites = []` | `EMPTY_INPUT` |
| `in_box` basic | 2 sites in `Aabb{0,0,2,2}` | Bounded Voronoi, cells clipped to box boundary. |

## Edge cases (v1 no special handling)

- Sites exactly on the bounding polygon edge: cell may be zero-area or degenerate.
  v1 does not fault on this; output may contain degenerate faces.
- Sites collinear or coincident: Delaunay construction handles naturally.
- Empty output: if all sites are outside, return mesh with 0 faces and empty sites array.
- `polygon.len < 3`: caught by convexity check or explicit guard.
