# M11v2 — Bounded Voronoi (voronator-rs direct port)

## Overview

Bounded Voronoi via voronator-rs algorithm: Delaunay triangulation with helper points, per-vertex circumcenter cell construction (no dual), Sutherland-Hodgman clipping against convex bounding polygon. `in_box` is an AABB→polygon wrapper.

## Architecture (voronator-rs style)

```
in_polygon(sites, polygon)
    ├─ validate polygon (convex, CCW, non-degenerate)
    ├─ prefilter sites → inside polysites
    ├─ N=0: empty
    ├─ N=1: cell = polygon
    ├─ N=2: bisect polygon via dot-product half-planes
    └─ N≥3:
        ├─ 4 helper points (2× bbox extents outside polygon)
        ├─ Delaunay(all_points)           // sites + helpers, no dual
        ├─ circumcenters_planar(mesh)     // per-face circumcenters
        ├─ per inside vertex:
        │   ├─ vertex_one_ring_faces(v) → incident face indices
        │   ├─ faces → circumcenters → Voronoi cell polygon
        │   └─ S-H clip against bounding polygon
        ├─ deduplicate positions
        └─ from_polygons(...) → HalfEdgeMesh
```

No dual, no `from_delaunay`, no boundary checks to remove. Direct voronator-rs port.

## Module

`src/voronoi/bounded.c3` — `module cg::voronoi;`

```c3
fn VoronoiDiagram? in_polygon(Allocator alloc, Vec3f[] sites, Vec3f[] polygon);
fn VoronoiDiagram? in_box(Allocator alloc, Vec3f[] sites, Aabb bbox);
```

## Public API (umbrella `src/c3cg.c3i`)

```c3
module cg::voronoi;
// ... existing ...
fn VoronoiDiagram? in_polygon(Allocator alloc, Vec3f[] sites, Vec3f[] polygon);
fn VoronoiDiagram? in_box(Allocator alloc, Vec3f[] sites, Aabb bbox);
```

New fault in root `module cg;` faultdef block:

```c3
faultdef NON_CONVEX_BOUNDING_POLYGON;
```

## Private helpers (in bounded.c3, not umbrella)

```c3
fn Vec3f[]? sutherland_hodgman(Allocator alloc, Vec3f[] polygon, Vec3f[] clip_polygon);
fn Vec3f[]? half_plane_clip(Allocator alloc, Vec3f[] polygon, Vec3f a, Vec3f b, bool keep_a_side);
fn Vec3f? line_intersection(Vec2f a, Vec2f b, Vec2f p, Vec2f q);
fn bool point_in_convex_polygon(Vec3f p, Vec3f[] polygon);
fn bool is_convex_polygon(Vec3f[] polygon);
```

## Pipeline details

### Sutherland-Hodgman

Clips polygon against convex CCW clip polygon edge by edge. Uses `orient_2d({x,y})`.

```
For each clip_edge (a, b):
    For each input_edge (p, q):
        p_in = orient_2d(a2,b2,p2) >= 0   // Vec3f→Vec2f conversion
        q_in = orient_2d(a2,b2,q2) >= 0
        if p_in && q_in:       output q
        if p_in && !q_in:      output intersect(a,b,p,q)
        if !p_in && q_in:      output intersect(a,b,p,q), q
        if !p_in && !q_in:     skip
    if output_len < 3: return empty
```

`intersect(a,b,p,q)` via Cramer's rule. Only called when endpoints are on opposite sides of the clip edge (segments cross).

### N=2 half-plane clip

Perpendicular bisector of a, b. Midpoint m = (a+b)/2. Direction d = b-a.

Point p is on a's side when `dot(p-m, d) * dot(a-m, d) >= 0`.

Clip polygon against the bisector half-plane. Two cells produced (one per side). Each preserved only if ≥3 vertices after clip.

### N≥3 helper-point path

1. Build `all_points = inside_sites + 4 helpers`
2. `delaunay_mesh = delaunay_2d(alloc, all_points)!`
3. `centers = circumcenters_planar(alloc, &delaunay_mesh)!`
4. Match `inside_sites[i]` to Delaunay vertex index by position equality
5. For each matched vertex: `vertex_one_ring_faces(v)` → circumcenters → polygon → S-H clip
6. Accumulate surviving cell polygons → `from_polygons`

### Vertex deduplication

Before `from_polygons`, deduplicate positions within 1e-12. Near-duplicates from independent S-H runs on adjacent cells prevent twin pairing.

### Memory

- `all_points`, `centers`: `defer free` after use
- `delaunay_mesh`: `defer destroy()` (temporary)
- Accumulation arrays: freed after `from_polygons`
- Returned `VoronoiDiagram`: owns mesh + sites

## Faults

| Fault | When |
|-------|------|
| `EMPTY_INPUT` | `sites.len == 0` (existing) |
| `NON_CONVEX_BOUNDING_POLYGON` | polygon not convex/CCW/non-degenerate (new) |

## Tests

File: `test/test_voronoi_bounded.c3`

| # | Test | Expected |
|---|------|----------|
| 1 | 4 sites in unit square, unit square polygon | 4 bounded cells |
| 2 | N=1 inside | 1 cell = polygon |
| 3 | N=1 outside | Empty |
| 4 | N=2 both inside | 2 cells split by bisector |
| 5 | N=2 one outside | 1 cell |
| 6 | N=2 both outside | Empty |
| 7 | 3 sites in triangle polygon | 3 bounded cells |
| 8 | All sites outside polygon | Empty |
| 9 | Non-convex L-shape polygon | `NON_CONVEX_BOUNDING_POLYGON` |
| 10 | Clockwise polygon | `NON_CONVEX_BOUNDING_POLYGON` |
| 11 | 2-vertex polygon | `NON_CONVEX_BOUNDING_POLYGON` |
| 12 | Collinear polygon | `NON_CONVEX_BOUNDING_POLYGON` |
| 13 | Empty sites | `EMPTY_INPUT` |
| 14 | `in_box` with 2 sites | 2 bounded cells |

### Empty-return semantics

When no cells survive (N=0 after prefilter, or all cells clipped outside), return a valid `VoronoiDiagram` with zero-length arrays:

```c3
VoronoiDiagram result;
result.mesh = {};   // empty arrays, 0 faces
result.sites = {};
return result;
```

This avoids `from_polygons`'s `EMPTY_INPUT` fault and keeps the return type uniform. Callers check `diagram.mesh.faces.len == 0`.
