# M11 — Bounded Voronoi (in_polygon, in_box)

## Overview

Clip Voronoi cells to a convex polygon using voronator-rs's approach: add distant helper points, compute Delaunay + unbounded Voronoi, then Sutherland-Hodgman clip each original-site cell against the bounding polygon. Produces a polygonal `HalfEdgeMesh` where each face is a bounded Voronoi cell. `in_box` is a thin AABB→polygon wrapper.

## Module

New file `src/voronoi/bounded.c3` — part of `module cg::voronoi;`.

## Prerequisite changes (from M10)

- Remove boundary check from `dual_from_vertices` (line 17: `twin == INVALID_HE` → `DUAL_REQUIRES_CLOSED_MESH`). The dual of a boundary vertex may produce a face ring with < 3 vertices — callers must handle or discard those faces. With helper points in M11, original sites are strictly interior (≥ 3 incident Delaunay triangles) so their face rings are always complete.
- Remove boundary check from `from_delaunay` (line 18: `twin == INVALID_HE` → `OPEN_CELL_ON_BOUNDARY`). Planar Delaunay naturally has boundary edges; unbounded Voronoi is valid output. Boundary-vertex dual faces may be degenerate — callers should validate if needed.
- Update M10 test `test_voronoi_boundary_faults`: change from "faults OPEN_CELL_ON_BOUNDARY" to "succeeds, interior vertices produce valid faces."

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

1. Validate `polygon.len >= 3` — else `NON_CONVEX_BOUNDING_POLYGON`.
2. Validate `polygon` is convex + CCW + non-degenerate: for each consecutive triple `(p, q, r)`, `orient_2d({p.x, p.y}, {q.x, q.y}, {r.x, r.y}) >= 0`. Additionally, require at least one triple with `> 0` (strictly positive turn) to reject collinear/degenerate polygons. Also require `polygon` bbox extents > 0 (nonzero area). Failure → `NON_CONVEX_BOUNDING_POLYGON`.
3. Validate `sites.len >= 1` — else `EMPTY_INPUT`.

4. Compute polygon bbox, add 4 helper points ~2× polygon extent away:
   ```
   w = max.x - min.x;  h = max.y - min.y
   helpers = [
     {min.x - w,     min.y + h/2},
     {max.x + w,     min.y + h/2},
     {min.x + w/2,   min.y - h  },
     {min.x + w/2,   max.y + h  }
   ]
   ```
   Helper points are far enough outside that no original site's Voronoi cell includes a helper circumcenter. They serve only to extend the Delaunay beyond the polygon.

5. `all_points = alloc.concat(sites, helpers)`.
6. `delaunay_mesh = delaunay::delaunay_2d(alloc, all_points)!`; `defer free(all_points)`.
7. `defer delaunay_mesh.destroy()` (destroyed on both success and fault paths).
8. `voronoi_diagram = voronoi::from_delaunay(alloc, &delaunay_mesh)!` (no boundary fault).
9. `defer voronoi_diagram.destroy()`.

10. **Match Voronoi faces to original sites**: build a position→VertexIndex lookup by comparing each original `sites[i]` against `delaunay_mesh.positions` (linear scan is fine — N and V are bounded for 2D diagrams). The matched vertex index IS the Voronoi face index (because `from_delaunay` copies Delaunay positions to Voronoi sites, and each site becomes exactly one Voronoi face).

11. For each matched face `f`:
    a. Walk face via `voronoi_diagram.mesh.face_vertices(f, out[])`.
    b. Collect positions from `voronoi_diagram.mesh.positions[out[..]]`.
    c. S-H clip against the bounding polygon.
    d. If clipped has < 3 vertices → face is outside the polygon, drop it (cell falls entirely outside).
    e. Otherwise accumulate positions for `from_polygons` and record the site.

Sites outside the polygon produce cells that cover most of the plane. After S-H clipping, these cells reduce to nothing — they are naturally dropped. No explicit inside/outside prefiltering is needed; the clip handles it for all N ≥ 3.

12. If no surviving cells → construct empty result:
    ```c3
    VoronoiDiagram result;
    result.mesh = {};        // zero-length arrays, 0 faces
    result.sites = {};
    return result;
    ```

13. Otherwise:
    ```c3
    clipped_mesh = half_edge::from_polygons(alloc, accumulated_positions, face_offsets, face_indices)!;
    VoronoiDiagram result;
    result.mesh = clipped_mesh;
    result.sites = surviving_sites;   // copy of original sites for surviving cells
    return result;
    ```

14. `voronoi_diagram.destroy()` / `delaunay_mesh.destroy()` via `defer catch`.

### N=1 and N=2 (before Delaunay path)

**N=1**: test if site is inside polygon (point-in-convex-polygon via orient_2d against each edge). If inside → cell = polygon. Build via `from_polygons` with one face. If outside → empty result.

**N=2**: test each site's position:
- Both inside: compute perpendicular bisector of the two sites. Clip polygon against bisector half-plane for each site. Two cells.
- One inside, one outside: the inside site's cell = the polygon (no other site to clip against). Output 1 cell.
- Both outside: empty result.

Half-plane clip: for a candidate point `p`, compute `sign_p = orient_2d({a.x, a.y}, {b.x, b.y}, {p.x, p.y})` and `sign_site = same for the site`. `p` is inside the half-plane when `sign_p` matches `sign_site`. Use same S-H edge loop with the bisector line as the single clipping edge.

### Sutherland-Hodgman

Clips a polygon against a convex clipping polygon edge by edge. Both input and clip polygons are CCW. Uses `orient_2d` (all vectors as `Vec2f` from `Vec3f.{x, y}`). Clipping polygon stored as `Vec3f[]`; positions extracted with `{p.x, p.y}`.

```
For each clip_edge (a, b) in bounding_polygon:
    For each input_edge (p, q) in current_polygon:
        p_inside = orient_2d({a.x,a.y}, {b.x,b.y}, {p.x,p.y}) >= geometry::PREDICATE_ZERO
        q_inside = orient_2d({a.x,a.y}, {b.x,b.y}, {q.x,q.y}) >= geometry::PREDICATE_ZERO
        if p_inside && q_inside:      output q
        if p_inside && !q_inside:     output line_intersection(a, b, p, q)
        if !p_inside && q_inside:     output line_intersection(a, b, p, q), q
        if !p_inside && !q_inside:    output nothing
```

`line_intersection(a, b, p, q)` computes the 2D intersection of the infinite lines through `(a,b)` and `(p,q)`. Called only when the segments are confirmed to cross (one endpoint on each side). Uses Cramer's rule on the 2D linear system.

Half-plane variant (for N=2 bisector): same algorithm but with a single clipping edge `(a, b)` = two points on the bisector line.

### Vertex deduplication

Before `from_polygons`, deduplicate accumulated positions (within `1e-12`). Build a lookup from old position index to new deduplicated index, remap face indices. `from_polygons` copies positions as-is and pairs twins when faces share exact vertex indices — dedup prevents near-duplicate vertices from preventing twin pairing.

## Faults

| Fault | When |
|-------|------|
| `EMPTY_INPUT` | `sites.len == 0` (existing) |
| `NON_CONVEX_BOUNDING_POLYGON` | `polygon` not convex or not CCW (new) |

New faultdef in `src/c3cg.c3i`, in the root `module cg;` fault block (alongside `EMPTY_INPUT`, `INDEX_OUT_OF_RANGE`, etc.):

```c3
faultdef NON_CONVEX_BOUNDING_POLYGON;
```

### Internal faults (propagated from callees)

- `DEGENERATE_INPUT` from `delaunay_2d` (n<3 — caught by N=1,N=2 special cases)
- Faults from `half_edge::from_polygons` (OOM, index out of range)

## Memory

- `all_points` (sites + helpers): `defer free` after Delaunay call.
- `delaunay_mesh`: `defer catch delaunay_mesh.destroy()`.
- `voronoi_diagram`: `defer catch voronoi_diagram.destroy()`.
- Accumulated positions and indices for `from_polygons`: freed after mesh construction.
- On fault during final `from_polygons`: accumulated arrays freed by `defer catch`.
- Returned `VoronoiDiagram` owns only the final clipped mesh and sites array.

## Tests

File: `test/test_voronoi_bounded.c3`

| # | Test | Input | Expected |
|---|------|-------|----------|
| 1 | 4 sites in unit square | `[(0.25,0.25), (0.75,0.25), (0.75,0.75), (0.25,0.75)]`, unit square | 4 cells, all bounded, vertices on or inside square |
| 2 | N=1 inside | Single site at `(0.5, 0.5)`, unit square | 1 cell = unit square polygon |
| 3 | N=1 outside | Single site at `(-1, 0.5)`, unit square | Empty result (0 faces, 0 sites) |
| 4 | N=2 both inside | `[(0.25,0.5), (0.75,0.5)]`, unit square | 2 cells split by vertical bisector |
| 5 | N=2 one outside | `[(0.5,0.5), (-1,0.5)]`, unit square | 1 cell (the inside site gets the polygon) |
| 6 | N=2 both outside | `[(-1,0.5), (-2,0.5)]`, unit square | Empty result |
| 7 | Site near corner | Single site at `(0.01, 0.01)`, unit square | Cell clipped, boundary vertices on polygon edges |
| 8 | 3 sites in triangle | `[(0.1,0.1), (0.5,0.9), (0.9,0.1)]`, unit square | 3 cells, all bounded, adjacent cells share edges |
| 9 | Diamond polygon | 5 sites in diamond `[(1,0), (0,1), (-1,0), (0,-1)]` | All 5 cells clipped, corner cells triangular |
| 10 | Triangle polygon | 3 sites in triangle `[(0,0), (2,0), (1,2)]` | All cells bounded, boundary cells clipped |
| 11 | All sites outside | `[(-1,-1), (-1,2), (2,-1), (2,2)]` in unit square | Empty result |
| 12 | Many sites, large box | 100 random sites in `[0,10]²` | All cells clipped, no crashes |
| 13 | Non-convex polygon faults | L-shaped 6-vertex polygon | `NON_CONVEX_BOUNDING_POLYGON` |
| 14 | Clockwise polygon faults | Unit square CW | `NON_CONVEX_BOUNDING_POLYGON` |
| 15 | Polygon with <3 verts | 2-vertex polygon | `NON_CONVEX_BOUNDING_POLYGON` |
| 16 | Empty sites | `sites = []` | `EMPTY_INPUT` |
| 17 | `in_box` basic | 2 sites in `Aabb{0,0,2,2}` | Bounded Voronoi, cells clipped to box boundary |

### Prerequisite tests (updated in `test/test_voronoi.c3`)

| # | Test | Old expected | New expected |
|---|------|-------------|-------------|
| 18 | `from_delaunay` with boundary mesh | `OPEN_CELL_ON_BOUNDARY` | Succeeds, mesh has boundary half-edges |
| 19 | `dual_from_vertices` with boundary mesh | (untested) | Succeeds for interior vertices |
| 20 | `from_delaunay` round-trip (closed mesh) | Still passes | Unchanged — closed meshes still work |

## Edge cases (v1 no special handling)

- Sites exactly on polygon edge: cell may be degenerate/zero-area. No fault.
- Collinear/coincident sites: Delaunay handles, cells may be degenerate.
- Output mesh has boundary half-edges along the clipping polygon — this is expected for bounded Voronoi. No `POLYGON_HAS_BOUNDARY` fault.
