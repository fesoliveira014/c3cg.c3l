# M12 — Spherical Delaunay and Voronoi Design

**Date:** 2026-05-22
**Status:** draft

## Overview

M12 adds spherical Delaunay triangulation and spherical Voronoi diagrams.
Points lie on a sphere of given radius.
Spherical Delaunay = convex hull + vertex reprojection (no circumcircle tests).
Spherical Voronoi = dual with spherical generators (circumcenters or centroids).

## API

All declarations in `src/c3cg.c3i` under `module cg::delaunay;`, `module cg::voronoi;`, `module cg::geometry;`.

### Spherical Delaunay

```c3
fn HalfEdgeMesh? delaunay_on_sphere(Allocator alloc, Vec3f[] points, float radius);
```

`points` must all lie on the sphere of given radius (up to floating-point tolerance).
Faults:
- `DEGENERATE_INPUT` if `points.len < 4` or `radius <= 0`
- `EMPTY_INPUT` if `points.len == 0`

Implementation:
1. `hull_3d(alloc, points)` → closed triangular mesh
2. Reproject each vertex position to sphere: `normalize(pos) * radius`
3. Return mesh

### Spherical generators (cg::geometry)

```c3
fn Vec3f[]? centroids_on_sphere(Allocator alloc, HalfEdgeMesh* tri_mesh, float radius);
```

- `centroids_on_sphere`: planar face centroid → normalize → multiply by radius
- The existing `circumcenters_on_sphere` (already in `src/c3cg.c3i` and `src/geometry/circumcenter.c3`) provides circumcenter-based generators.

Faults propagated from `face_centroids` / `circumcenter_on_sphere`.

### Spherical Voronoi

```c3
fn VoronoiDiagram? voronoi_on_sphere(Allocator alloc, Vec3f[] points, float radius);
fn VoronoiDiagram? voronoi_dual_sphere(Allocator alloc, HalfEdgeMesh* tri_mesh, Vec3f[] spatial_points);
```

- `voronoi_on_sphere`: composes `delaunay_on_sphere` + `circumcenters_on_sphere` + `dual_from_vertices` + copies input sites
- `voronoi_dual_sphere`: low-level; caller supplies the triangulated mesh and spatial points (pre-computed circumcenters or centroids). No radius needed — spatial points are already on the sphere. Sites are the spatial points themselves (passed through to `diagram.sites`).

Faults:
- `DEGENERATE_INPUT` if `points.len < 4` or `radius <= 0`
- `EMPTY_INPUT` if `points.len == 0`
- Propagates faults from composed functions

## Files

| File | Content |
|------|---------|
| `src/delaunay/delaunay_sphere.c3` | `delaunay_on_sphere` |
| `src/voronoi/voronoi_sphere.c3` | `voronoi_on_sphere`, `voronoi_dual_sphere` |
| `src/geometry/spherical.c3` | `centroids_on_sphere` |
| `test/test_delaunay_sphere.c3` | Spherical Delaunay tests |
| `test/test_voronoi_sphere.c3` | Spherical Voronoi tests |

## Test Plan

### test_delaunay_sphere.c3

1. **Icosahedron vertices produce icosahedral triangulation**
   - 12 icosahedron vertices on unit sphere → `delaunay_on_sphere`
   - 20 faces (icosahedron)
   - Closed mesh: `is_boundary` false for every half-edge
   - Every vertex on sphere: `|len(pos) - 1.0| < 1e-4`

2. **Radius preservation**
   - Same test on sphere radius 5 → all vertices `|len(pos) - 5.0| < 1e-4`

3. **Empty input** → `EMPTY_INPUT` fault (`points.len == 0`)

4. **Degenerate input** → `DEGENERATE_INPUT` fault (`0 < points.len < 4` or `radius <= 0`)

### test_voronoi_sphere.c3

5. **Icosahedron → dodecahedral Voronoi**
   - 12 input points → `voronoi_on_sphere`
   - 12 faces (dodecahedron)
   - 20 vertices
   - Closed mesh, no boundary edges
   - Each face is pentagonal (5 vertices)

6. **Round-trip preserves mesh**
   - `voronoi::to_delaunay(&voronoi_on_sphere(points, r)!!)` has same topology as `delaunay_on_sphere(points, r)`

7. **Centroid variant**
   - `delaunay_on_sphere` + `centroids_on_sphere` + `voronoi_dual_sphere`
   - Produces valid closed mesh with 12 faces

8. **Radius preservation**
   - Same as Delaunay: all sites on sphere within tolerance

9. **Empty / degenerate input** → appropriate fault

## Open Questions

- [x] Both circumcenters and centroids exposed → resolved: yes
- [x] Single milestone → resolved: yes
- [x] Naming: `on_sphere` vs `spherical` prefix → resolved: `delaunay_on_sphere`, `voronoi_on_sphere`, `centroids_on_sphere`. Uses existing `circumcenters_on_sphere`.
