# M11 — Bounded Voronoi (in_polygon, in_box)

## Overview

Clip Voronoi cells to a convex polygon using voronator-rs's approach: add distant helper points, compute Delaunay + unbounded Voronoi, then Sutherland-Hodgman clip each cell against the bounding polygon. Produces a polygonal `HalfEdgeMesh` where each face is a bounded Voronoi cell. `in_box` is a thin AABB→polygon wrapper.

## Module

New file `src/voronoi/bounded.c3` — part of `module cg::voronoi;`.

## Prerequisite changes (from M10)

- Remove boundary check from `dual_from_vertices` (line 17: `twin == INVALID_HE` → `DUAL_REQUIRES_CLOSED_MESH`). Also skip boundary vertices with incomplete face rings (< 3 faces) instead of faulting — those vertices produce no face in the dual output (the face_offsets advance by 0 for that vertex index). This is required because helper points may be hull vertices with incomplete rings.
- Remove boundary check from `from_delaunay` (line 18: `twin == INVALID_HE` → `OPEN_CELL_ON_BOUNDARY`). Planar Delaunay naturally has boundary edges; unbounded Voronoi is valid output. M11 uses `from_delaunay` directly — helper-point vertices with incomplete rings are skipped by the dual, original-site vertices are interior and complete.
- Update M10 test `test_voronoi_boundary_faults`: change from "faults OPEN_CELL_ON_BOUNDARY" to "succeeds, interior vertices produce valid faces."

## Public API

```c3
fn VoronoiDiagram? in_polygon(Allocator alloc, Vec3f[] sites, Vec3f[] polygon);
fn VoronoiDiagram? in_box(Allocator alloc, Vec3f[] sites, Aabb bbox);
```

Umbrella additions in `src/c3cg.c3i`, under `module cg::voronoi`:

```c3
fn VoronoiDiagram? in_polygon(Allocator alloc, Vec3f[] sites, Vec3f[] polygon);
fn VoronoiDiagram? in_box(Allocator alloc, Vec3f[] sites, Aabb bbox);
```

## Pipeline

### `in_polygon`

1. Validate `polygon.len >= 3` — else `NON_CONVEX_BOUNDING_POLYGON`.
2. Validate `polygon` convex + CCW + non-degenerate: for each consecutive triple (p, q, r), `orient_2d({p.x, p.y}, {q.x, q.y}, {r.x, r.y}) >= 0`. Require at least one triple > 0 and bbox extents > 0. Failure → `NON_CONVEX_BOUNDING_POLYGON`.
3. Validate `sites.len >= 1` — else `EMPTY_INPUT`.

4. **Prefilter sites**: test each site against polygon (point-in-convex-polygon: `orient_2d` against each edge — site left-of-or-on all edges). Keep only inside/on sites in `inside_sites` (newly allocated). If 0 survive → return empty `VoronoiDiagram`.

5. **Branch on surviving count N** (`inside_sites.len`):
   - **N = 0**: return empty (mesh = {}, sites = {}).
   - **N = 1**: cell = polygon. Build via `from_polygons` with one face. Allocate 1-element `sites` array, copy the site. Return `VoronoiDiagram { mesh, sites }`.
   - **N = 2**: perpendicular bisector of the two sites (see N=2 section). Clip polygon via half-plane S-H for each site → two cells. Allocate 2-element `sites` array. Return.
   - **N ≥ 3**: helper-point Delaunay path below. `inside_sites` is freed after use (replaced by the returned diagram's sites).

#### N ≥ 3 helper-point path

6. Compute polygon bbox, add 4 helper points ~2× extent away:
   ```
   w = max.x - min.x;  h = max.y - min.y
   helpers = [
     {min.x - w,     min.y + h/2},
     {max.x + w,     min.y + h/2},
     {min.x + w/2,   min.y - h  },
     {min.x + w/2,   max.y + h  }
   ]
   ```
   Helper points extend the Delaunay beyond the polygon so boundary sites produce complete Voronoi cells (≥ 3 circumcenter vertices).

7. Consolidated point array:
   ```c3
   usz total = sites.len + (usz)4;
   Vec3f[] all_points = mem::alloc::new_array(alloc, Vec3f, (sz)total);
   defer free(all_points);
   // copy sites, then helpers
   ```

8. `delaunay_mesh = delaunay::delaunay_2d(alloc, all_points)!`.
9. `defer delaunay_mesh.destroy()` (unconditional — temporary).
10. `voronoi_diagram = voronoi::from_delaunay(alloc, &delaunay_mesh)!`.
11. `defer voronoi_diagram.destroy()` (unconditional — temporary).

12. **Match faces to inside_sites**: compare each `inside_sites[i]` against `voronoi_diagram.sites[]` (position equality within 1e-12). Each site maps to one Voronoi face index.

13. Per matched face:
    a. `face_vertices(f, out[])`. < 3 vertices → skip (won't happen for interior sites).
    b. Collect positions from `voronoi_diagram.mesh.positions[out[..]]`.
    c. S-H clip against bounding polygon.
    d. < 3 after clip → drop.
    e. Accumulate positions + indices for `from_polygons`.

14. If no surviving cells → empty result.
15. Otherwise: `from_polygons(alloc, positions, offsets, indices)` → allocate sites array for surviving cells only, copy from `inside_sites`. Return `VoronoiDiagram { mesh, sites }`.

### N=2 half-plane clip

Sites `a`, `b`. Midpoint `m = (a + b) / 2`. Direction `d = b - a`.

A point `p` is on site `a`'s side of the perpendicular bisector when `dot(p - m, d) <= 0`.

Clipping edge: the bisector line through `m` with direction `perp = {-d.y, d.x, 0}`. For S-H, use two far-apart points on that line as the single clip edge. `p` is "inside" when the dot-product above matches the expected sign for the target site.

Output two cells, each clipped to one half of the bounding polygon. Allocate 2-element sites array with copies of both sites.

### Sutherland-Hodgman

Clips polygon against convex CCW clip polygon edge by edge. Uses `Vec2f` from `Vec3f.{x, y}`.

```
For each clip_edge (a, b):
    For each input_edge (p, q):
        p_in = orient_2d({a.x,a.y}, {b.x,b.y}, {p.x,p.y}) >= PREDICATE_ZERO
        q_in = orient_2d({a.x,a.y}, {b.x,b.y}, {q.x,q.y}) >= PREDICATE_ZERO
        if p_in && q_in:       output q
        if p_in && !q_in:      output intersection(a,b, p,q)
        if !p_in && q_in:      output intersection(a,b, p,q), q
        if !p_in && !q_in:     nothing
```

`intersection` via Cramer's rule on the 2D lines — only called when endpoints are on opposite sides.

### Vertex deduplication

Before `from_polygons`, deduplicate positions (within 1e-12), remap face indices. `from_polygons` copies positions as-is; near-duplicates prevent twin pairing.

## Faults

| Fault | When |
|-------|------|
| `EMPTY_INPUT` | `sites.len == 0` (existing in root `module cg;`) |
| `NON_CONVEX_BOUNDING_POLYGON` | polygon not convex/CCW/degenerate (new) |

Add to root fault block in `src/c3cg.c3i`:

```c3
faultdef NON_CONVEX_BOUNDING_POLYGON;
```

## Memory

- `all_points`: `defer free` after Delaunay call.
- `delaunay_mesh`, `voronoi_diagram`: `defer destroy()` (unconditional — temporaries).
- Accumulated positions/indices: freed after `from_polygons` or on fault via `defer catch`.
- Returned `VoronoiDiagram` owns only final clipped mesh + sites.

## Tests

File: `test/test_voronoi_bounded.c3`

| # | Test | Input | Expected |
|---|------|-------|----------|
| 1 | 4 sites in unit square | `[(0.25,0.25), (0.75,0.25), (0.75,0.75), (0.25,0.75)]`, unit square | 4 cells, all bounded, vertices inside/on square |
| 2 | N=1 inside | `[(0.5, 0.5)]`, unit square | 1 cell = unit square |
| 3 | N=1 outside | `[(-1, 0.5)]`, unit square | Empty (prefiltered) |
| 4 | N=2 both inside | `[(0.25,0.5), (0.75,0.5)]`, unit square | 2 cells split by vertical bisector |
| 5 | N=2 one outside | `[(0.5,0.5), (-1,0.5)]`, unit square | 1 cell (inside site gets polygon) |
| 6 | N=2 both outside | `[(-1,0.5), (-2,0.5)]`, unit square | Empty |
| 7 | Site near corner | `[(0.01, 0.01)]`, unit square | 1 cell clipped correctly |
| 8 | 3 sites in square | `[(0.1,0.1), (0.5,0.9), (0.9,0.1)]`, unit square | 3 cells, all bounded |
| 9 | Diamond polygon | 5 sites in diamond `[(1,0), (0,1), (-1,0), (0,-1)]` | 5 cells clipped |
| 10 | Triangle polygon | 3 sites in triangle `[(0,0), (2,0), (1,2)]` | 3 cells bounded |
| 11 | All sites outside | `[(-1,-1), (-1,2), (2,-1), (2,2)]`, unit square | Empty (prefiltered) |
| 12 | 100 random sites | `[0,10]²` box | All clipped, no crash |
| 13 | Non-convex polygon | L-shaped 6-vertex | `NON_CONVEX_BOUNDING_POLYGON` |
| 14 | Clockwise polygon | Unit square CW | `NON_CONVEX_BOUNDING_POLYGON` |
| 15 | <3 verts | 2-vertex polygon | `NON_CONVEX_BOUNDING_POLYGON` |
| 16 | Collinear polygon | 4 collinear vertices | `NON_CONVEX_BOUNDING_POLYGON` |
| 17 | Empty sites | `sites = []` | `EMPTY_INPUT` |
| 18 | `in_box` basic | 2 sites in `Aabb{0,0,2,2}` | Bounded cells |

### Prerequisite tests (update `test/test_voronoi.c3`)

| # | Test | Old | New |
|---|------|-----|-----|
| 19 | `from_delaunay` boundary | Faults | Succeeds, interior vertices produce valid faces |

## Edge cases (v1, no special handling)

- Sites on polygon edge: degenerate/zero-area cells possible. No fault.
- Collinear/coincident sites: Delaunay handles; N=2 path catches.
- Output mesh has boundary half-edges along clipping polygon — expected for bounded Voronoi.
