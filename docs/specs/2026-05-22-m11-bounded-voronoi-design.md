# M11 — Bounded Voronoi (in_polygon, in_box)

## Overview

Clip Voronoi cells to a convex polygon using voronator-rs's approach: add distant helper points, compute Delaunay + unbounded Voronoi, then Sutherland-Hodgman clip each cell against the bounding polygon. Produces a polygonal `HalfEdgeMesh` where each face is a bounded Voronoi cell. `in_box` is a thin AABB→polygon wrapper.

## Module

New file `src/voronoi/bounded.c3` — part of `module cg::voronoi;`.

## Prerequisite changes

- Remove boundary check from `dual_from_vertices` (`DUAL_REQUIRES_CLOSED_MESH` fault). The dual of a boundary vertex produces a partial/open cell — this is valid unbounded Voronoi. With helper points, original sites are always interior so cells are complete.
- Remove boundary check from `from_delaunay` (`OPEN_CELL_ON_BOUNDARY` fault). Planar Delaunay naturally has boundary edges; unbounded Voronoi is valid output. Callers who need closed meshes check themselves.
- Update M10 test: boundary-mesh test changes from "faults" to "succeeds, has boundary vertices."

## Public API

```c3
fn VoronoiDiagram? in_polygon(Allocator alloc, Vec3f[] sites, Vec3f[] polygon);
fn VoronoiDiagram? in_box(Allocator alloc, Vec3f[] sites, Aabb bbox);
```

Umbrella additions in `src/c3cg.c3i`:

```c3
fn VoronoiDiagram? in_polygon(Allocator alloc, Vec3f[] sites, Vec3f[] polygon);
fn VoronoiDiagram? in_box(Allocator alloc, Vec3f[] sites, Aabb bbox);
```

## Pipeline

### `in_polygon`

1. Validate `sites.len >= 1` — else `EMPTY_INPUT`.
2. Validate `polygon` convex + CCW: `orient_2d({p.x, p.y}, {q.x, p.y}, {r.x, p.y}) >= 0` for each consecutive triple. Failure → `NON_CONVEX_BOUNDING_POLYGON`.
3. Compute polygon bbox, add 4 helper points at ~2× extents:
   ```
   helpers = [
     {min.x - w, min.y + h/2},
     {max.x + w, min.y + h/2},
     {min.x + w/2, min.y - h},
     {min.x + w/2, max.y + h}
   ]
   ```
4. `all_points = sites + helpers`.
5. `delaunay_mesh = delaunay::delaunay_2d(alloc, all_points)!`.
6. `voronoi_diagram = voronoi::from_delaunay(alloc, &delaunay_mesh)!` (no boundary fault — helpers make interior sites complete).
7. For each face `f` where `f < sites.len` (original sites only):
   a. Walk face via `voronoi_diagram.mesh.face_vertices(f, out[])` — collect positions.
   b. S-H clip the polygon against the bounding polygon.
   c. If clipped has < 3 vertices → drop (cell outside).
   d. Otherwise accumulate positions + indices.
8. Deduplicate positions (spatial hash or pairwise below threshold).
9. `clipped_mesh = half_edge::from_polygons(alloc, positions, offsets, indices)!`.
10. Copy sites for surviving cells.
11. Return `VoronoiDiagram { clipped_mesh, surviving_sites }`.

### `in_box`

Convert `Aabb` to 4-vertex CCW polygon `{min, {max.x, min.y, 0}, max, {min.x, max.y, 0}}`, call `in_polygon`.

### N=1 and N=2 special cases

- **N=1**: Cell = bounding polygon. Return mesh with one polygonal face.
- **N=2**: Compute perpendicular bisector of the two sites. Clip the bounding polygon by the bisector half-plane containing each site. Two cells. Uses the same half-plane S-H utility.

### Sutherland-Hodgman

Clips a polygon against a convex clipping polygon edge by edge. Both input and clip polygons assumed CCW. Uses `orient_2d` (with `Vec2f` conversion from `Vec3f.x, Vec3f.y`).

```
For each clip_edge (a, b) in bounding_polygon:
    For each input_edge (p, q) in current_polygon:
        p_inside = orient_2d(a, b, p) >= 0   // left-of-or-on
        q_inside = orient_2d(a, b, q) >= 0
        if p_inside && q_inside:      output q
        if p_inside && !q_inside:     output intersection(a,b, p,q)
        if !p_inside && q_inside:     output intersection(a,b, p,q), q
        if !p_inside && !q_inside:    output nothing
```

Half-plane variant (for n=2 bisector): clip by a single line (the bisector) rather than a polygon. Same logic with `a = bisector_point1`, `b = bisector_point2`, `inside = orient_2d(site, a, b) >= 0`.

Line intersection: solve two parametric segments. Only needed when one endpoint is inside, one outside (segments are guaranteed to cross).

## Faults

| Fault | When |
|-------|------|
| `EMPTY_INPUT` | `sites.len == 0` (existing) |
| `NON_CONVEX_BOUNDING_POLYGON` | `polygon` not convex or not CCW (new) |
| `DEGENERATE_INPUT` | from `delaunay_2d` for n < 3 (caught by special cases) |
| `POLYGON_HAS_BOUNDARY` | polygon mesh has boundary edges (new, from `from_polygons`) |

`NON_CONVEX_BOUNDING_POLYGON` added to `src/c3cg.c3i`:

```c3
faultdef NON_CONVEX_BOUNDING_POLYGON;
```

## Memory

Intermediate meshes (Delaunay + unbounded Voronoi) destroyed before return. Helper points freed. Returned `VoronoiDiagram` owns only the final clipped mesh and sites array.

## Tests

File: `test/test_voronoi_bounded.c3`

| Test | Input | Expected |
|------|-------|----------|
| 4 sites in unit square | `sites = [(0.25,0.25), (0.75,0.25), (0.75,0.75), (0.25,0.75)]`, `polygon = unit square` | 4 cells, all bounded. Output face vertices on or inside square. All sites retained. |
| Site near corner | Single site at `(0.01, 0.01)` in unit square | Cell = unit square (n=1 special case). |
| Two sites | `[(0.25,0.5), (0.75,0.5)]` in unit square | 2 cells split by vertical bisector. One cell left, one right. |
| 3 sites in triangle | `[(0.1,0.1), (0.5,0.9), (0.9,0.1)]` in unit square | 3 cells, all bounded, adjacent cells share edges. |
| Diamond polygon | 5 sites inside diamond `[(1,0), (0,1), (-1,0), (0,-1)]` | All 5 cells clipped. Corner cells triangular. |
| Triangle polygon | 3 sites inside triangle `[(0,0), (2,0), (1,2)]` | All cells bounded. Boundary cells clipped to triangle sides. |
| Sites outside polygon | Site at `(-0.5, 0.5)` in unit square | Cell outside → dropped. Output has 0 faces. |
| Many sites, large box | 100 random sites in `[0,10]²` | All cells clipped, no crashes. |
| Non-convex polygon faults | L-shaped 6-vertex polygon | `NON_CONVEX_BOUNDING_POLYGON` |
| Empty sites | `sites = []` | `EMPTY_INPUT` |
| `in_box` basic | 2 sites in `Aabb{0,0,2,2}` | Bounded Voronoi, cells clipped to box boundary. |
| Empty output mesh | 1 site at `(-10, -10)` in unit square | Empty result (site too far). 0 faces, 0 sites. |

## Edge cases (v1 no special handling)

- Sites exactly on polygon edge: cell may be degenerate/zero-area. No fault.
- Collinear/coincident sites: Delaunay handles, cells may be degenerate.
- `polygon.len < 3`: caught by convexity check or explicit guard.
