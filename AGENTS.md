# c3cg — Computational Geometry Library (C3)

C3 library (`c3cg.c3l`) providing Delaunay triangulation, Voronoi diagrams, convex hull, polygon triangulation, mesh subdivision, and edge operations — all built on a single flat-array `HalfEdgeMesh` data structure.

Targets C3 0.8.0. Source lives under `src/`.

## Build & Test

```bash
c3c build debug         # debug static library build (O0)
c3c build release       # release static library build (O3)
c3c test                # run tests under test/**
c3c clean               # wipe build/
```

Verification before every commit:

```bash
c3c build debug && c3c test
```

## Required Skills

Before writing/modifying any C3 code, reviewing C3 code, editing `project.json` or manifest files, or diagnosing C3 compiler errors, invoke `c3-expert`. C3 is pre-1.0 — do not rely on memorised syntax.

For any non-trivial code change, use `test-driven-development` (write failing test first).

## Code Style

Source of truth: `docs/style.md`. This guide wins over any other style guidance.

| Kind                                          | Convention             | Examples                       |
| --------------------------------------------- | ---------------------- | ------------------------------ |
| Variables, fields, params, functions, methods | `snake_case`           | `voxel_size`, `from_triangles` |
| Structs, enums, typedefs, aliases             | `PascalCase`           | `HalfEdgeMesh`, `HeIndex`      |
| Constants, enum values                        | `SCREAMING_SNAKE_CASE` | `BRICK_DIM`, `INVALID_HE`      |
| Module names                                  | lowercase, dotted      | `cg::half_edge`                |
| File names                                    | `snake_case.c3`        | `half_edge_mesh.c3`            |

Definition order within a file: typedefs → aliases → constants → enums → structs → struct methods → free functions.

### Key Anti-Patterns (never do these)

- `null` as error signal — use optional + fault
- Runtime `assert()` in production code — use returned faults
- Sentinel return values (`-1`, `0xFFFFFFFF`) outside documented `INVALID_*` constants
- Bare numeric literals where a named constant would explain the meaning
- Raw `malloc`/`free` — use `mem::new*`
- `sizeof(T)` — use `T::size` or `@sizeof(expr)`
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

All type, enum, struct, constant, and public free-function declarations live in a single umbrella interface file `src/c3cg.c3i`. Implementation files (`src/<module>/*.c3`) contain only `module`, `import`, and function bodies. This keeps the library interface in one place for external callers.

```
src/
├── c3cg.c3i               module cg; module cg::geometry; module cg::half_edge; ...  (umbrella interface)
│
├── render/                module cg::render    RenderingData + to_rendering_data
├── half_edge/             module cg::half_edge   builder, topology, walks, flip, validate
├── geometry/              module cg::geometry     circumcenter, centroid, predicates
├── dual/                  module cg::dual         dual(source, positions) → mesh
├── hull/                  module cg::hull         2D monotone chain, 3D incremental
├── delaunay/              module cg::delaunay     Bowyer-Watson 2D, spherical via hull
├── voronoi/               module cg::voronoi      from_delaunay, in_polygon, in_box, on_sphere
├── graph/                 module cg::graph        VoronoiGraph (CSR), DelaunayGraph
├── triangulate/           module cg::triangulate  Ear clipping
├── subdivide/             module cg::subdivide    Loop subdivision
└── primitives/            module cg::primitives   Icosphere, platonic solids
test/
```

**Interface convention:** `src/c3cg.c3i` is the single source of truth for all public API declarations.

When adding a new submodule (e.g., M8 `cg::hull_3d`), follow this checklist:

1. **Add free-function signatures + type/enum/const declarations** to `src/c3cg.c3i` under the appropriate `module cg::xxx;` section.
2. **Add implementations** to the submodule's `.c3` file(s) — `module`, `import`, function bodies only (no type/const redeclarations).
3. Method declarations on structs stay in their `.c3` files — C3 0.8.0 treats method declarations as definitions and they will conflict with the umbrella.

Both the umbrella declarations and `.c3` definitions are visible to consumers at compile time — the umbrella is additive, not restrictive.

### Core Types

**Identifier typedefs** (in `src/c3cg.c3i`):

```c3
typedef HeIndex     = inline int;
typedef FaceIndex   = inline int;
typedef VertexIndex = inline int;
const HeIndex     INVALID_HE     = (HeIndex)-1;
const FaceIndex   INVALID_FACE   = (FaceIndex)-1;
const VertexIndex INVALID_VERTEX = (VertexIndex)-1;
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
One logical change per commit. Run `c3c build debug && c3c test` before committing.

## Documentation

Full architecture, API patterns, and milestone plan: `docs/architecture.md` (r3).
C3 bindings conventions: `docs/bindings_guidelines.md`.

## Development Plan (14 Milestones)

| Milestone | What                                                         | Status      |
| --------- | ------------------------------------------------------------ | ----------- |
| M0        | Project scaffolding — types, faults, struct stubs            | ✅ Complete |
| M1        | Half-edge construction (`from_triangles`) + topology queries | ✅ Complete |
| M2        | Walks + validation                                           | ✅ Complete |
| M3        | Rendering data extraction                                    | ✅ Complete |
| M4        | Edge flipping                                                | ✅ Complete |
| M5        | Geometry helpers (circumcenters, centroids, predicates)      | ✅ Complete |
| M6        | Dual operation                                               | ✅ Complete |
| M7        | Convex hull 2D (Andrew's monotone chain)                     | ✅ Complete |
| M8        | Convex hull 3D (incremental)                                 | ✅ Complete |
| M9        | Delaunay 2D (delaunator)                                     | ✅ Complete |
| M10       | Voronoi from Delaunay (unbounded)                            | ✅ Complete |
| M11       | Bounded Voronoi (in_polygon, in_box)                         | ✅ Complete |
| M12       | Spherical Delaunay + Voronoi                                 | ✅ Complete |
| M13       | Voronoi/Delaunay graph views                                 | ✅ Complete |
| M14       | Polygon triangulation, Loop subdivision, primitives          | Not started |

Each milestone = commit(s) with tests. `c3c build && c3c test` green at every boundary.

## Open Questions (Resolved in M0)

These were verified with `c3-expert` during M0 on c3c 0.7.11 and rechecked as needed during the C3 0.8.0 migration:

- [x] Vector type spelling: `float[<3>]` — used with local aliases (`Vec2f`, etc.)
- [x] Methods on imported types: confirmed working with `smoke/smoke.c3` + test
- [x] `project.json` shape for library — `static-lib` + `debug` + `release` targets
- [x] Test annotations (`@test`) — after params in test `.c3` files, `assert()` per style §11

Deferred to later milestones:

- HashMap instantiation syntax — needed in M1 (twin-pairing)
- Allocator-passing convention — needed in M1 (construction)
- `*self = {}` zero-clear syntax — needed in M1 (`destroy`)
- Submodule visibility — needed in M1+
