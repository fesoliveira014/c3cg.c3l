# c3cg — Computational Geometry Library (C3)

C3 library (`c3cg.c3l`) providing Delaunay triangulation, Voronoi diagrams, convex hull, polygon triangulation, mesh subdivision, and edge operations — all built on a single flat-array `HalfEdgeMesh` data structure.

Targets C3 0.7.x. Source lives under `cg/` (not yet scaffolded — see M0 plan below).

## Build & Test

```bash
c3c build linux         # debug build (O0)
c3c run linux           # build + execute
c3c test linux          # run tests under test/**
c3c clean               # wipe build/
```

Verification before every commit:
```bash
c3c build linux && c3c test linux
```

## Required Skills

Before writing/modifying any C3 code, reviewing C3 code, editing `project.json` or manifest files, or diagnosing C3 compiler errors, invoke `c3-expert`. C3 is pre-1.0 — do not rely on memorised syntax.

For any non-trivial code change, use `test-driven-development` (write failing test first).

## Code Style

Source of truth: `docs/style.md`. This guide wins over any other style guidance.

| Kind | Convention | Examples |
|------|-----------|----------|
| Variables, fields, params, functions, methods | `snake_case` | `voxel_size`, `from_triangles` |
| Structs, enums, typedefs, aliases | `PascalCase` | `HalfEdgeMesh`, `HeIndex` |
| Constants, enum values | `SCREAMING_SNAKE_CASE` | `BRICK_DIM`, `INVALID_HE` |
| Module names | lowercase, dotted | `cg::half_edge` |
| File names | `snake_case.c3` | `half_edge_mesh.c3` |

Definition order within a file: typedefs → aliases → constants → enums → structs → struct methods → free functions.

### Key Anti-Patterns (never do these)

- `null` as error signal — use optional + fault
- Runtime `assert()` in production code — use returned faults
- Sentinel return values (`-1`, `0xFFFFFFFF`) outside documented `INVALID_*` constants
- Bare numeric literals where a named constant would explain the meaning
- Raw `malloc`/`free` — use `mem::new*`
- `sizeof(T)` — use `T.sizeof` or `$sizeof(expr)`
- Arrow `->` for pointer access — C3 uses `.` for both
- Goto-cleanup chains — use `defer`
- `if (catch foo)` without binding — write `if (catch err = foo)`
- CamelCase identifiers in any role

### Memory

- Allocations via `mem::new(T)` / `mem::new_array(T, n)`, paired with `defer free(p)` on the next line
- `@pool() { ... }` for scope-bound temp allocations
- `defer catch destroy_X(handle)` for resources freed only on failure
- Never write goto-cleanup — use `defer`

### Error Handling

- Optionals (`T?`) and granular faults (`faultdef`) — no nulls or sentinels
- Lift fault to optional: `return rdi::SLOT_TABLE_FULL~;`
- Propagate: `TextureHandle h = device.create_texture(&desc)!;`
- Catch and inspect: `if (catch err = thing) { ... }`
- Force-unwrap (panics): `!!` — use sparingly
- Use specific faults, not generic catch-alls
- Runtime `assert` belongs in tests only (not production). Compile-time `$assert` is fine everywhere.

## Architecture

The library is organised around a single `HalfEdgeMesh` type that represents both triangular and polygonal meshes. A Delaunay triangulation is a triangular `HalfEdgeMesh`; a Voronoi diagram is a polygonal `HalfEdgeMesh` + a parallel `sites` array.

### Module Layout

```
cg/
├── src/
│   ├── cg.c3                  module cg;          umbrella
│   ├── types.c3               Vec aliases, HeIndex, FaceIndex, VertexIndex, Aabb
│   ├── half_edge_mesh.c3      HalfEdge, HalfEdgeFace, HalfEdgeVertex, HalfEdgeMesh structs
│   ├── faults.c3              Cross-cutting faults
│   │
│   ├── render/                module cg::render    RenderingData + to_rendering_data
│   ├── half_edge/             module cg::half_edge   builder, topology, walks, flip, validate
│   ├── geometry/              module cg::geometry     circumcenter, centroid, predicates
│   ├── dual/                  module cg::dual         dual(source, positions) → mesh
│   ├── hull/                  module cg::hull         2D monotone chain, 3D incremental
│   ├── delaunay/              module cg::delaunay     Bowyer-Watson 2D, spherical via hull
│   ├── voronoi/               module cg::voronoi      from_delaunay, in_polygon, in_box, on_sphere
│   ├── graph/                 module cg::graph        VoronoiGraph (CSR), DelaunayGraph
│   ├── triangulate/           module cg::triangulate  Ear clipping
│   ├── subdivide/             module cg::subdivide    Loop subdivision
│   └── primitives/            module cg::primitives   Icosphere, platonic solids
└── test/
```

### Core Types

**Identifier typedefs** (in `types.c3`):
```c3
typedef HeIndex     = inline int;
typedef FaceIndex   = inline int;
typedef VertexIndex = inline int;
const HeIndex     INVALID_HE     = -1;
const FaceIndex   INVALID_FACE   = -1;
const VertexIndex INVALID_VERTEX = -1;
```

**HalfEdgeMesh** — flat arrays, integer indices, no internal pointers:
```c3
struct HalfEdgeMesh {
    HalfEdge[]       half_edges;       // topology
    HalfEdgeFace[]   faces;
    HalfEdgeVertex[] vertices;
    Vec3f[] positions;                 // geometry (parallel to vertices)
    Vec3f[] normals;
    Vec2f[] uvs;
}
struct HalfEdge       { VertexIndex origin; HeIndex next, twin; FaceIndex face; }
struct HalfEdgeFace   { HeIndex half_edge; }
struct HalfEdgeVertex { HeIndex half_edge; }
```

**RenderingData** — caller-owned, GPU-uploadable:
```c3
struct RenderingData { Vec3f[] vertices; uint[] indices; Vec3f[] normals; Vec2f[] uvs; }
```

**VoronoiDiagram** — polygonal mesh + parallel sites array:
```c3
struct VoronoiDiagram { HalfEdgeMesh mesh; Vec3f[] sites; }
```

**VoronoiGraph** — flat per-cell access via CSR storage (no half-edge walking):
```c3
struct VoronoiGraph {
    Vec3f[] vertices, sites, centroids;
    int[] cell_offsets;
    VertexIndex[] ring_indices;
    FaceIndex[] neighbor_indices;
}
```

**DelaunayGraph** — flat per-triangle view:
```c3
struct DelaunayGraph {
    Vec3f[] vertices, circumcenters;
    DelaunayTriangle[] triangles;
}
```

### Design Principles

1. **Flat arrays, integer indices, no internal pointers** — every reference is an int index. Cache-friendly, trivially serialisable.
2. **One mesh type, two roles** — `HalfEdgeMesh` handles both triangles and polygons. Algorithms enforce constraints by faulting.
3. **Fault discipline** — no nulls, no sentinels outside `INVALID_*`. Fallible ops return optionals with granular faults.
4. **Method syntax** — operations on `HalfEdgeMesh` are methods (`mesh.flip(he)`), declared in operation submodules.
5. **Topology/geometry split** — `half_edges`/`faces`/`vertices` carry topology only; `positions`/`normals`/`uvs` are parallel geometry arrays.
6. **Topology ↔ geometry mapping is `VertexIndex` → array index** — consistent across all types.

## Commits

Conventional Commits format: `<scope>: <imperative summary>` (e.g. `half_edge: add flip operation (M4)`).
One logical change per commit. Run `c3c build linux && c3c test linux` before committing.

## Documentation

Full architecture, API patterns, and milestone plan: `docs/architecture.md` (r3).
C3 bindings conventions: `docs/bindings_guidelines.md`.

## Development Plan (14 Milestones)

| Milestone | What | Status |
|-----------|------|--------|
| M0 | Project scaffolding — types, faults, struct stubs | Not started |
| M1 | Half-edge construction (`from_triangles`) + topology queries | Not started |
| M2 | Walks + validation | Not started |
| M3 | Rendering data extraction | Not started |
| M4 | Edge flipping | Not started |
| M5 | Geometry helpers (circumcenters, centroids, predicates) | Not started |
| M6 | Dual operation | Not started |
| M7 | Convex hull 2D (Andrew's monotone chain) | Not started |
| M8 | Convex hull 3D (incremental) | Not started |
| M9 | Delaunay 2D (Bowyer-Watson) | Not started |
| M10 | Voronoi from Delaunay (unbounded) | Not started |
| M11 | Bounded Voronoi (in_polygon, in_box) | Not started |
| M12 | Spherical Delaunay + Voronoi | Not started |
| M13 | Voronoi/Delaunay graph views | Not started |
| M14 | Polygon triangulation, Loop subdivision, primitives | Not started |

Each milestone = commit(s) with tests. `c3c build && c3c test` green at every boundary.

## Open Questions (Resolve in M0)

These need `c3-expert` verification before coding:
- Vector type spelling: `float[<3>]` vs `std::math::Vec3f`
- Methods on imported types: can `fn Ret HalfEdgeMesh.method()` be declared from `cg::half_edge` for a type in `cg`?
- HashMap instantiation syntax in target C3 release
- Allocator-passing convention (explicit param vs context allocator)
- `*self = {}` zero-clear syntax
- Test annotations (`@test`) and assertion semantics
- `project.json` shape for library vs application
- Submodule visibility (can `cg::voronoi` see `cg::half_edge` operations through import?)
