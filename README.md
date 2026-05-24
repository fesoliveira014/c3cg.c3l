# c3cg — Computational Geometry Library

A [C3](https://c3-lang.org/) library providing Delaunay triangulation, Voronoi diagrams, convex hulls, polygon triangulation, mesh subdivision, and edge operations — built on a single flat-array half-edge mesh data structure.

## Build & Test

```bash
c3c build debug         # debug static library (O0)
c3c build release       # release static library (O3)
c3c test                # run all tests
c3c clean               # wipe build/
```

## Visualization Scripts

```bash
c3c build random-delaunay   && ./out/random-delaunay    # → random_delaunay.ppm
c3c build random-voronoi    && ./out/random-voronoi      # → random_voronoi.ppm, random_overlay.ppm
c3c build triangulate-polygon && ./out/triangulate-polygon  # → triangulate_polygon.ppm
c3c build loop-subdivide    && ./out/loop-subdivide       # → loop_subdivide.ppm
```

All output P6 binary PPM images — viewable with any image viewer.

## Capabilities

### Core Data Structure

**`HalfEdgeMesh`** — a single flat-array mesh type for both triangular and polygonal geometry. Integer indices, no internal pointers, trivially serialisable.

### Builders (points → mesh)

| Function | Module | Description |
|----------|--------|-------------|
| `delaunay_2d` | `cg::delaunay` | 2D Delaunay triangulation (delaunator algorithm, O(n log n)) |
| `delaunay_on_sphere` | `cg::delaunay` | Spherical Delaunay via convex hull reprojection |
| `hull_2d` | `cg::hull` | 2D convex hull (Andrew's monotone chain, O(n log n)) |
| `hull_3d` | `cg::hull` | 3D convex hull (incremental, O(n²) worst case) |
| `triangulate_polygon` | `cg::triangulate` | Simple polygon triangulation (monotone decomposition, O(n log n)) |
| `from_triangles` | `cg::half_edge` | Build mesh from triangle index buffer |
| `from_polygons` | `cg::half_edge` | Build mesh from polygon face offsets + indices |

### Voronoi Diagrams

| Function | Module | Description |
|----------|--------|-------------|
| `from_delaunay` | `cg::voronoi` | Unbounded Voronoi from Delaunay triangulation |
| `to_delaunay` | `cg::voronoi` | Reverse dual — recover Delaunay from Voronoi |
| `in_polygon` | `cg::voronoi` | Bounded Voronoi clipped to a convex polygon |
| `in_box` | `cg::voronoi` | Bounded Voronoi clipped to an axis-aligned box |
| `voronoi_on_sphere` | `cg::voronoi` | Spherical Voronoi from points on a sphere |
| `voronoi_dual_sphere` | `cg::voronoi` | Low-level spherical dual with caller-supplied generators |

### Graph Views (fast read-only access)

| Type | Module | Description |
|------|--------|-------------|
| `VoronoiGraph` | `cg::graph` | CSR per-cell neighbor lookup, flat iteration |
| `DelaunayGraph` | `cg::graph` | Per-triangle vertex/neighbor/circumcenter arrays |
| `from_voronoi` | `cg::graph` | Build Voronoi graph from a Voronoi diagram |
| `from_delaunay` | `cg::graph` | Build Delaunay graph from mesh + circumcenters |
| `from_planar_delaunay` | `cg::graph` | Build Delaunay graph with auto-computed planar circumcenters |
| `from_spherical_delaunay` | `cg::graph` | Build Delaunay graph with auto-computed spherical circumcenters |

### Operators (mesh → mesh)

| Function | Module | Description |
|----------|--------|-------------|
| `loop_subdivide` | `cg::subdivide` | One-step Loop subdivision (closed triangular meshes) |
| `dual_from_vertices` | `cg::dual` | Dual mesh from pre-computed vertex positions |
| `flip` | `cg::half_edge` | Edge flip with six-rewire topology update |

### Geometry

| Function | Module | Description |
|----------|--------|-------------|
| `circumcenter` | `cg::geometry` | Planar triangle circumcenter |
| `circumcenters_planar` | `cg::geometry` | All planar face circumcenters |
| `circumcenters_on_sphere` | `cg::geometry` | All spherical face circumcenters |
| `centroids_planar` | `cg::geometry` | All planar face centroids |
| `centroids_on_sphere` | `cg::geometry` | All spherical face centroids |
| `orient_2d` | `cg::geometry` | 2D orientation predicate |

### Primitives

| Function | Module | Description |
|----------|--------|-------------|
| `tetrahedron` | `cg::primitives` | Unit tetrahedron |
| `octahedron` | `cg::primitives` | Unit octahedron |
| `icosahedron` | `cg::primitives` | Unit icosahedron |
| `triangulated_cube` | `cg::primitives` | Triangulated cube (corners at ±1) |
| `icosphere` | `cg::primitives` | Icosahedron with N Loop subdivisions + sphere projection |

### Visualization (PPM)

| Function | Module | Description |
|----------|--------|-------------|
| `render_delaunay` | `cg::utils` | Render Delaunay triangulation with edges + sites |
| `render_voronoi` | `cg::utils` | Render Voronoi diagram with colored cell fill |
| `render_overlay` | `cg::utils` | Render Delaunay + Voronoi on one image |
| `new_ppm` / `fill` / `write_ppm` | `cg::utils::ppm` | P6 binary PPM image creation |
| `draw_line` / `draw_circle` | `cg::utils::ppm` | Bresenham line and circle drawing |

## Future

- **Constrained Delaunay triangulation** — Delaunay-quality triangulations respecting constraint edges
- **Robust adaptive predicates** — Shewchuk-style exact arithmetic for degenerate inputs
- **Catmull–Clark subdivision** — quad-aware subdivision surface algorithm
- **Mesh simplification** — half-edge collapse and split operations
- **3D volumetric** — 3D Delaunay tetrahedralisation and volumetric Voronoi
