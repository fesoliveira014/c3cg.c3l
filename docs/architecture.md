# `cg` — Computational Geometry Library

**Design and development plan (revision 2)**

A computational-geometry library for C3, organised around a flat-array half-edge mesh that doubles as both triangulation and Voronoi representation, with operations layered as submodules. This document is both the architectural reference and the executable plan: design decisions here should not be re-litigated during implementation, and §7's milestone checklist is the order of work.

The library targets C3 0.8.0 (pinned in `.claude/c3-skill.json`). Style follows the c3vq style guide attached to this project — `snake_case` for functions and variables, `PascalCase` for types, `SCREAMING_SNAKE_CASE` for constants, granular faults instead of `null`/sentinels, no runtime `assert` in production code.

> **Revision note.** This is r3 of the design. r1 placed `HalfEdgeMesh` under `cg::half_edge` and used a bespoke CSR `VoronoiDiagram`; r2 lifted the type into `cg`, made the half-edge mesh polygonal-capable, defined a generic dual operation, and represented Voronoi as a polygonal `HalfEdgeMesh` + parallel `sites` array. r3 adds flat graph views — `VoronoiGraph` and `DelaunayGraph` (in `cg::graph`) — over the existing mesh-based representations, for callers who want denormalised per-cell or per-triangle access without walking half-edges.

---

## 1. Goals and non-goals

### Goals

Provide idiomatic C3 APIs for the standard computational-geometry workhorses — Delaunay triangulation, Voronoi diagrams, convex hull, polygon triangulation, mesh subdivision, edge operations — all built on top of one shared half-edge mesh data structure that handles both triangular and polygonal cases. Output is renderable: every mesh-like type produces a `RenderingData` (vertices, indices, normals, uvs) suitable for direct upload to a GPU.

The half-edge mesh ports the _structure_ of `half_edge_mesh.gd` (Godot, GDScript) into idiomatic C3, replacing object references with integer indices into flat arrays.

The library handles three geometric domains uniformly:

- **Planar 2D** — points in the plane (interpreted as `Vec3f` with `z = 0`).
- **Bounded 2D** — points clipped to an arbitrary convex polygon or AABB.
- **Spherical 3D** — points on a sphere of given radius.

### Non-goals

- **Not a CGAL replacement.** Robustness is "good enough for typical floating-point inputs", not exact predicates. Robust-predicate option may come later.
- **Not a renderer.** No GPU resources, no graphics-API knowledge.
- **Not constrained / weighted / anisotropic in v1.** Later extensions on top of the core.
- **No serialisation format in v1.** Persistence is the caller's problem; flat arrays make it trivial when needed.

---

## 2. Design principles

### 2.1 Flat arrays, integer indices, no internal pointers

Non-negotiable. Every reference between mesh elements is an integer index into one of the mesh's flat arrays. The Godot port encodes this directly: every `_twin`, `_next`, `_face`, and `half_edge` field is an `int`, never an object reference. We carry that into C3 without compromise. The only pointers in the public API are `&self` on methods.

Why: cache locality, trivial serialisation, no GC pressure, no aliasing surprises, easy parallelisation later. The shape stays compatible with a possible future GPU-side workflow.

### 2.2 One mesh type, two roles

`HalfEdgeMesh` represents both triangular and polygonal meshes. A Delaunay triangulation is a `HalfEdgeMesh` whose faces are all triangles. A Voronoi diagram is a `HalfEdgeMesh` whose faces are arbitrary convex polygons, paired with a `sites` array (one site per face). The data layout doesn't distinguish — algorithms enforce face-cycle constraints they need by faulting with `NON_TRIANGLE_FACE` if they encounter the wrong shape.

This unifies Delaunay↔Voronoi duality: the dual of a triangular mesh is a polygonal mesh, and vice versa, all using the same struct.

### 2.3 Fault discipline (style §10)

No `null` returns, no sentinel `-1` outside the documented `INVALID_INDEX` constant for "this slot is unoccupied", no `bool ok` out-params. Fallible operations return optionals carrying _granular_ faults — `NON_TRIANGLE_FACE`, `BOUNDARY_HALF_EDGE`, `FLIP_NOT_ALLOWED`, `OPEN_CELL_ON_BOUNDARY` — never an umbrella `INVALID_INPUT`.

### 2.4 Method syntax for type APIs

All operations on a mesh-like type are methods on it: `mesh.flip(he)`, `mesh.to_rendering_data()`, `mesh.is_boundary(he)`. Methods are declared in operation submodules (`cg::half_edge`, etc.) on types from `cg`. Free functions are reserved for constructors and pure cross-type utilities.

### 2.5 Topology / geometry split

`half_edges`, `faces`, `vertices` carry topology only. `positions`, `normals`, `uvs` carry geometry. They're parallel arrays indexed by `VertexIndex`. Algorithms that only touch topology (flip, manifold validation, dual construction) never read the geometry arrays — they stay cold in cache.

### 2.6 Identifier typedefs (style §5)

Distinct typedefs for index kinds. Cost zero at runtime; signature-mismatch bugs caught at compile time:

```c3
typedef HeIndex     = inline int;
typedef FaceIndex   = inline int;
typedef VertexIndex = inline int;
```

### 2.7 Incremental commits (style §17)

Each milestone in §7 is a commit (or short series), with its own tests. `c3c build && c3c test` stays green at every milestone boundary.

---

## 3. Module layout

```
cg/
├── project.json
├── src/
│   ├── cg.c3                       module cg;          umbrella
│   ├── types.c3                    module cg;          Vec aliases, identifier typedefs, Aabb
│   ├── half_edge_mesh.c3           module cg;          HalfEdge, HalfEdgeFace, HalfEdgeVertex, HalfEdgeMesh
│   ├── faults.c3                   module cg;          cross-cutting faults
│   │
│   ├── render/
│   │   └── rendering_data.c3       module cg::render;  RenderingData + to_rendering_data
│   │
│   ├── half_edge/                  module cg::half_edge; — operations on cg::HalfEdgeMesh
│   │   ├── builder.c3                                  from_triangles, from_polygons
│   │   ├── topology.c3                                 twin/next/prev/from_vertex/to_vertex
│   │   ├── walks.c3                                    one-ring, face-cycle iterators
│   │   ├── flip.c3                                     flip + is_flip_ok
│   │   └── validate.c3                                 manifold checks
│   │
│   ├── geometry/
│   │   ├── circumcenter.c3         module cg::geometry; planar + spherical circumcenters
│   │   ├── centroid.c3                                  face centroids
│   │   └── predicates.c3                                in-circle, orient-2d, on-sphere
│   │
│   ├── dual/
│   │   └── dual.c3                 module cg::dual;     dual(mesh, dual_positions) → mesh
│   │
│   ├── hull/
│   │   ├── hull_2d.c3              module cg::hull;     Andrew's monotone chain
│   │   └── hull_3d.c3                                   incremental hull → HalfEdgeMesh
│   │
│   ├── delaunay/
│   │   ├── delaunay_2d.c3          module cg::delaunay; Bowyer–Watson
│   │   └── delaunay_sphere.c3                           via 3D convex hull
│   │
│   ├── voronoi/
│   │   ├── voronoi.c3              module cg::voronoi;  VoronoiDiagram, from_delaunay, to_delaunay
│   │   ├── bounded.c3                                   in_polygon, in_box, clip helpers
│   │   └── voronoi_sphere.c3                            on_sphere (no clipping needed)
│   │
│   ├── graph/
│   │   ├── voronoi_graph.c3        module cg::graph;    VoronoiGraph + from_voronoi
│   │   └── delaunay_graph.c3                            DelaunayGraph + from_delaunay variants
│   │
│   ├── triangulate/
│   │   └── ear_clipping.c3         module cg::triangulate;
│   │
│   ├── subdivide/
│   │   └── loop.c3                 module cg::subdivide;
│   │
│   └── primitives/
│       ├── icosphere.c3            module cg::primitives;
│       └── platonic.c3
│
└── test/
    └── ... (one file per topic; module cg::test;)
```

`module cg;` (across `cg.c3`, `types.c3`, `half_edge_mesh.c3`, `faults.c3`) defines the data layer — types, identifiers, faults — and nothing else. Every operations module imports `cg` and adds methods or constructors.

`module cg::half_edge;` is the central operations module: construction, topology queries, walks, flip, validate. Most of the user's manual mesh manipulation goes through this module.

`module cg::dual;` is the bridge between triangulation-style and Voronoi-style meshes and the basis for several higher-level builders.

> **Verification flag for c3-expert.** Confirm that C3 allows `fn Ret HalfEdgeMesh.method(...)` declarations from `module cg::half_edge` when `HalfEdgeMesh` is declared in `module cg`. The bindings guidelines show this pattern works for externally-imported types (Jolt), so it should work intra-project — but verify in M0 with a one-line smoke test before committing to the architecture.

---

## 4. Core types

All types in this section are declared in `module cg;` unless otherwise noted.

### 4.1 Math primitives (`src/types.c3`)

```c3
module cg;

alias Vec2f = float[<2>];
alias Vec3f = float[<3>];
alias Vec4f = float[<4>];

alias Vec2i = int[<2>];
alias Vec3i = int[<3>];
```

> **Verification flag for c3-expert.** The exact spelling of stdlib vector types in the configured release. C3 has built-in vector types (`float[<N>]`); the stdlib may also expose `std::math::Vec3f` aliases. Pick one source of truth in M0.

### 4.2 Identifier typedefs and Aabb (`src/types.c3`)

```c3
typedef HeIndex     = inline int;
typedef FaceIndex   = inline int;
typedef VertexIndex = inline int;

const HeIndex     INVALID_HE     = -1;
const FaceIndex   INVALID_FACE   = -1;
const VertexIndex INVALID_VERTEX = -1;

struct Aabb {
    Vec3f min;
    Vec3f max;
}
```

`INVALID_*` is the **only** sentinel pattern in the library — used in slot fields like `HalfEdge.twin` where "no value" is a structural property of the data. Function returns never use sentinels; they return optionals.

`Aabb` lives in the top-level module because it's used across `voronoi`, `hull`, and primitive generators.

### 4.3 Faults (`src/faults.c3`)

Cross-cutting faults visible to every submodule. Per-module faults stay in their owning files.

```c3
module cg;

faultdef
    INDEX_OUT_OF_RANGE,
    INVALID_TRIANGLE_INDEX_COUNT,
    NON_MANIFOLD_INPUT,
    DUPLICATE_HALF_EDGE,
    NON_TRIANGLE_FACE,
    BOUNDARY_HALF_EDGE,
    DEGENERATE_INPUT,
    EMPTY_INPUT,
    OPEN_CELL_ON_BOUNDARY;
```

Submodule-specific faults sit in their own files. For example `src/half_edge/flip.c3`:

```c3
faultdef FLIP_NOT_ALLOWED, FLIP_PRODUCES_DUPLICATE_EDGE, FLIP_DEGENERATE;
```

And `src/dual/dual.c3`:

```c3
faultdef DUAL_REQUIRES_CLOSED_MESH, DUAL_VERTEX_COUNT_MISMATCH;
```

### 4.4 `HalfEdge`, `HalfEdgeFace`, `HalfEdgeVertex` (`src/half_edge_mesh.c3`)

```c3
module cg;

struct HalfEdge {
    VertexIndex origin;     // tail vertex
    HeIndex     next;       // next half-edge around the face (any cycle length)
    HeIndex     twin;       // opposite half-edge across the edge, INVALID_HE on boundary
    FaceIndex   face;       // owning face, INVALID_FACE for boundary half-edges
}

struct HalfEdgeFace {
    HeIndex half_edge;      // any half-edge bounding this face
}

struct HalfEdgeVertex {
    HeIndex half_edge;      // any outgoing half-edge from this vertex
}
```

**Departures from the Godot version, deliberately:**

- No stored destination (`_vertex_1` in Godot). Recoverable via `half_edges[he.next].origin`. Storing one source-of-truth eliminates the "did the flip rewrite both endpoints consistently?" class of bugs the Godot routine has to avoid by hand.
- No stored `_index`. The half-edge's index is its position in the array.

`HalfEdge` size: `4 * 4 = 16` bytes. The Godot GDScript object is at least an order of magnitude larger and heap-allocated.

### 4.5 `HalfEdgeMesh` (`src/half_edge_mesh.c3`)

```c3
module cg;

struct HalfEdgeMesh {
    // Topology (parallel arrays)
    HalfEdge[]       half_edges;
    HalfEdgeFace[]   faces;
    HalfEdgeVertex[] vertices;

    // Geometry, indexed by VertexIndex.
    // normals and uvs may be empty when not provided / not relevant.
    Vec3f[] positions;
    Vec3f[] normals;
    Vec2f[] uvs;
}
```

**Crucially: faces have arbitrary cycle length.** A triangulation has 3-cycles everywhere; a Voronoi cell can be a 7-gon. The struct doesn't care. Algorithms that need triangles check and fault.

**Invariants** (enforced on construction, validated on demand by `cg::half_edge::validate`):

1. For each interior half-edge `he`, `half_edges[he.twin].twin == he`.
2. For each half-edge `he` with face `f != INVALID_FACE`, walking `next` from `he` returns to `he` after some number of steps and visits only half-edges with `face == f`.
3. `vertices[v].half_edge` points to one canonical outgoing half-edge from `v`.
4. Boundary edges are represented by interior half-edges whose `twin == INVALID_HE`; those half-edges retain their owning `face`. We do not allocate ghost boundary half-edges or ghost faces.
5. Triangle-assuming algorithms (flip, planar Delaunay step, Loop subdivision) check the face cycle is exactly 3 and return `NON_TRIANGLE_FACE` if not.

**Lifecycle.** The mesh owns its arrays. Construction allocates; `destroy(&self)` (in `cg::half_edge`) frees. Pattern:

```c3
HalfEdgeMesh mesh = cg::half_edge::from_triangles(mem, positions, indices)!;
defer mesh.destroy();
```

> **Verification flag for c3-expert.** Allocator-passing convention: explicit `Allocator` parameter on each constructor (à la C3 stdlib idiom) vs context allocator. Pick one in M0 and apply uniformly.

### 4.6 `RenderingData` (`src/render/rendering_data.c3`)

```c3
module cg::render;
import cg;

struct RenderingData {
    Vec3f[] vertices;
    uint[]  indices;
    Vec3f[] normals;
    Vec2f[] uvs;
}

fn void RenderingData.destroy(&self) {
    free(self.vertices);
    free(self.indices);
    free(self.normals);
    free(self.uvs);
    *self = {};
}
```

The caller owns the returned data. `*self = {}` zeroes the struct so a double-free is a noop, not a use-after-free.

### 4.7 `VoronoiDiagram` (`src/voronoi/voronoi.c3`)

```c3
module cg::voronoi;
import cg;

struct VoronoiDiagram {
    HalfEdgeMesh mesh;     // each face = a Voronoi cell, polygonal
    Vec3f[]      sites;    // sites[face_index] = the original site for that cell
}

fn void VoronoiDiagram.destroy(&self) {
    self.mesh.destroy();
    free(self.sites);
    *self = {};
}
```

A `VoronoiDiagram` is a `HalfEdgeMesh` plus a parallel `sites` array indexed by `FaceIndex`. The mesh provides topology + Voronoi-vertex positions (= circumcentres of the underlying Delaunay triangles); the sites array provides the source points each cell is built around. Their relationship is exactly the dual relationship: `sites.len == mesh.faces.len`, and `mesh.positions.len` equals the Delaunay triangle count.

The duality is symmetric. `cg::voronoi::to_delaunay(diagram)` reconstructs the Delaunay mesh by applying `cg::dual::dual(diagram.mesh, diagram.sites)`. Likewise `from_delaunay(tri)` is `dual(tri, circumcentres(tri))` paired with the original positions.

### 4.8 `VoronoiGraph` and `DelaunayGraph` (`src/graph/`)

Flat per-cell / per-triangle views over the canonical mesh-based representations. Both are _snapshots_ — built by an explicit constructor from a `VoronoiDiagram` or `HalfEdgeMesh`, owned independently, not auto-invalidated by changes to the source. Once built, the graph carries everything needed for read-only iteration and the source can be discarded.

These graphs are the answer to "I want a `Cell[]` I can iterate without walking half-edges". The Godot `CellMesh`/`Cell` pair is the prototype. Internal layout is still flat-array + indices (no per-cell allocations); the view methods provide the per-cell ergonomic access.

#### VoronoiGraph

```c3
module cg::graph;
import cg;
import cg::voronoi;

struct VoronoiGraph {
    Vec3f[] vertices;       // Voronoi vertices = circumcentres of underlying Delaunay triangles
    Vec3f[] sites;          // one site per cell, indexed by cell index (= FaceIndex)
    Vec3f[] centroids;      // one centroid per cell

    // CSR storage. Cell c's ring is ring_indices[cell_offsets[c] .. cell_offsets[c+1]];
    // its neighbors are neighbor_indices over the same offsets. The neighbor across
    // ring edge (ring[i] -> ring[(i+1) % k]) is neighbor_indices[cell_offsets[c] + i].
    int[]         cell_offsets;     // length = cell_count + 1
    VertexIndex[] ring_indices;     // total length = sum of ring sizes
    FaceIndex[]   neighbor_indices; // INVALID_FACE for boundary edges; same length as ring_indices
}

struct VoronoiCellView {
    FaceIndex     index;
    Vec3f         site;
    Vec3f         centroid;
    VertexIndex[] ring;       // slice into graph.ring_indices
    FaceIndex[]   neighbors;  // slice into graph.neighbor_indices, parallel to ring edges
}

fn void            VoronoiGraph.destroy(&self);
fn int             VoronoiGraph.cell_count(&self);
fn VoronoiCellView VoronoiGraph.cell(&self, FaceIndex c);
```

The single `cell_offsets` array is shared between `ring_indices` and `neighbor_indices` because each ring edge has exactly one neighbor; their per-cell counts are identical. This collapses what would otherwise be two parallel offset arrays and removes a class of "did the offsets drift out of sync?" bug.

The `cell(c)` method constructs the view inline — `ring` and `neighbors` are slices into the graph's storage, no copying. Iteration is a tight loop:

```c3
for (FaceIndex c = 0; c < graph.cell_count(); c++) {
    VoronoiCellView cell = graph.cell(c);
    // cell.ring, cell.neighbors immediately accessible
}
```

#### DelaunayGraph

Triangles have fixed degree 3, so a `DelaunayTriangle` struct array suffices — no CSR offsets needed:

```c3
struct DelaunayTriangle {
    VertexIndex[3] vertices;
    FaceIndex[3]   neighbors;     // INVALID_FACE on boundary edges
}

struct DelaunayGraph {
    Vec3f[]            vertices;       // input points (Delaunay vertices)
    Vec3f[]            circumcenters;  // one per triangle (= dual Voronoi vertices)
    DelaunayTriangle[] triangles;
}

struct DelaunayTriangleView {
    FaceIndex      index;
    Vec3f          circumcenter;
    VertexIndex[3] vertices;
    FaceIndex[3]   neighbors;
}

fn void                 DelaunayGraph.destroy(&self);
fn int                  DelaunayGraph.triangle_count(&self);
fn DelaunayTriangleView DelaunayGraph.triangle(&self, FaceIndex t);
```

The `[3]` arrays are by-value fixed-size (style §5; `triangle.vertices[0]` etc.), so the view returns them directly without heap copying.

#### Relationship to the underlying mesh

A graph is a denormalised snapshot. The half-edge mesh remains the canonical store for topology mutation; if you need to flip an edge, subdivide, or otherwise modify, you operate on the `HalfEdgeMesh` and rebuild the graph after. The graph constructors are cheap (single pass; allocations proportional to total ring/triangle size) so this is a fine workflow even for iterative algorithms — but they're not free, so don't put one inside a hot loop that mutates.

---

## 5. Key API patterns

### 5.1 Topology queries (`src/half_edge/topology.c3`)

Direct port of the Godot queries, as methods declared from `cg::half_edge` on `cg::HalfEdgeMesh`:

```c3
module cg::half_edge;
import cg;

fn HeIndex     HalfEdgeMesh.twin(&self, HeIndex he);
fn HeIndex     HalfEdgeMesh.next(&self, HeIndex he);
fn HeIndex     HalfEdgeMesh.prev(&self, HeIndex he);    // walks `next` until it returns to `he`
fn VertexIndex HalfEdgeMesh.from_vertex(&self, HeIndex he);
fn VertexIndex HalfEdgeMesh.to_vertex(&self, HeIndex he);
fn FaceIndex   HalfEdgeMesh.face_of(&self, HeIndex he);
fn bool        HalfEdgeMesh.is_boundary(&self, HeIndex he);
fn int         HalfEdgeMesh.face_degree(&self, FaceIndex f);  // cycle length; useful for triangle checks
```

Pure index lookups, no allocation, no faults. Bounds-checking is via C3's built-in array bounds checks in debug mode; in release the user is trusted.

### 5.2 Mesh construction (`src/half_edge/builder.c3`)

```c3
fn HalfEdgeMesh? from_triangles(Allocator alloc, Vec3f[] positions, uint[] indices);
fn HalfEdgeMesh? from_triangles_with_attrs(
    Allocator alloc,
    Vec3f[] positions,
    Vec3f[] normals,
    Vec2f[] uvs,
    uint[]  indices,
);
fn HalfEdgeMesh? from_polygons(Allocator alloc, Vec3f[] positions, uint[] face_offsets, uint[] face_indices);
```

`from_triangles` is the primary entry. `from_polygons` accepts arbitrary polygonal faces in CSR form (`face_offsets` is `face_count + 1` long; face `f`'s vertex indices are `face_indices[face_offsets[f] .. face_offsets[f+1]]`). Same hash-map twin-pairing pass for both.

Faults on either:

- `INVALID_TRIANGLE_INDEX_COUNT` if `indices.len % 3 != 0` (triangles only).
- `INDEX_OUT_OF_RANGE` if any index ≥ `positions.len`.
- `DUPLICATE_HALF_EDGE` if two faces produce the same directed half-edge (non-manifold or wrongly-oriented input).
- `EMPTY_INPUT` for empty positions or indices.

The implementation mirrors the Godot `from_triangle_mesh`: build half-edges per face, register `(origin, dest) → he_index` in a hash map, second pass pairs twins by reversed-edge lookup. Differences from the Godot version: catch duplicates as faults rather than logging and continuing; vertex `half_edge` field set to the _first_ outgoing edge encountered and never overwritten, so vertex one-ring walks are deterministic.

### 5.3 Edge flipping (`src/half_edge/flip.c3`)

Direct port of the Godot routine:

```c3
fn bool  HalfEdgeMesh.is_flip_ok(&self, HeIndex he);
fn void? HalfEdgeMesh.flip(&self, HeIndex he);    // FLIP_NOT_ALLOWED on bad input
```

`flip` returns `void?` so the caller learns _why_ a flip didn't happen; the Godot version logs and returns `false`. We do not rewrite an external index buffer in `flip` — the index buffer is regenerated lazily by `to_rendering_data` from the current topology, which keeps repeated flips (Delaunay refinement) cheap.

`is_flip_ok` checks: not on boundary; would-be new endpoints distinct; would-be new edge does not already exist in the mesh (`vertex_has_edge_to`, ported from `_edge_exists`); the source face _and_ the across-twin face are both triangles (otherwise `NON_TRIANGLE_FACE`).

### 5.4 Mesh walks (`src/half_edge/walks.c3`)

Helpers used by Voronoi construction, dual operation, validation. Style §10 — pure topology, no allocation when possible.

```c3
fn int? HalfEdgeMesh.vertex_one_ring_outgoing(&self, VertexIndex v, HeIndex[] out);
fn int? HalfEdgeMesh.vertex_one_ring_faces(&self, VertexIndex v, FaceIndex[] out);
fn int? HalfEdgeMesh.face_half_edges(&self, FaceIndex f, HeIndex[] out);
fn int? HalfEdgeMesh.face_vertices(&self, FaceIndex f, VertexIndex[] out);
```

Caller supplies the output slice; helper returns the count written or `OUTPUT_BUFFER_TOO_SMALL`. No per-call heap traffic. The dual operation in §5.6 is the heaviest user.

### 5.5 Rendering data extraction (`src/render/rendering_data.c3`)

```c3
module cg::render;
fn RenderingData? HalfEdgeMesh.to_rendering_data(&self);
```

Two paths, chosen per face:

- **Triangle face** — emit three indices in face-cycle order. Cheap.
- **Polygonal face** — fan-triangulate around the first vertex. Correct for convex polygons (Voronoi cells are convex by construction). Faults `NON_PLANAR_FACE` if a coplanarity check fails on a polygonal face for which fan triangulation would be invalid.

Positions, normals, and uvs are copied from the source mesh into freshly allocated slices. If `mesh.normals` is empty, generate face-averaged vertex normals on the fly. If `mesh.uvs` is empty, emit zeros.

`VoronoiDiagram.to_rendering_data` is a thin wrapper that calls `mesh.to_rendering_data()` — no special path is needed because the polygonal fan triangulation handles cells uniformly.

### 5.6 Dual operation (`src/dual/dual.c3`)

The central abstraction connecting triangulations and Voronoi-style meshes.

```c3
module cg::dual;
import cg;

fn HalfEdgeMesh? dual(HalfEdgeMesh source, Vec3f[] dual_vertex_positions);
```

Semantics. For source mesh `M` with V vertices, F faces, E edges:

- `dual(M)` has V faces, F vertices, E edges.
- `dual_vertex_positions` must have length `M.faces.len` — one position per source face. The caller supplies the mapping (circumcentres for Delaunay→Voronoi, original sites for Voronoi→Delaunay, centroids for generic case).
- Each source vertex `v` becomes a dual face whose ring is the source's one-ring of incident faces around `v`, walked in CCW order using `vertex_one_ring_faces`.
- Each source face `f` becomes a dual vertex at `dual_vertex_positions[f]`.
- Each interior source half-edge becomes a dual half-edge.

Faults:

- `DUAL_VERTEX_COUNT_MISMATCH` if `dual_vertex_positions.len != source.faces.len`.
- `DUAL_REQUIRES_CLOSED_MESH` if the source has any boundary edge. (For meshes with boundary, the caller must clip first — see §5.7 — or use the spherical case where there is no boundary.)

Round-trip property: `dual(dual(M, positions_a), positions_b) ≅ M` topologically, with `M`'s positions replaced by `positions_b`. This is exactly Voronoi↔Delaunay duality.

Convenience wrappers for the common cases:

```c3
fn HalfEdgeMesh? dual_with_circumcenters(HalfEdgeMesh tri_mesh);
fn HalfEdgeMesh? dual_with_centroids(HalfEdgeMesh source);
fn HalfEdgeMesh? dual_with_spherical_circumcenters(HalfEdgeMesh tri_mesh, float radius);
```

These are all 2–3 lines: compute the position array via `cg::geometry::*`, then call `dual`.

### 5.7 Voronoi construction (`src/voronoi/`)

Three top-level constructors, all returning a `VoronoiDiagram`:

```c3
module cg::voronoi;

// Unbounded — only valid when the Delaunay has no boundary (i.e., on a sphere or already wrapped).
// Faults OPEN_CELL_ON_BOUNDARY for planar / open meshes.
fn VoronoiDiagram? from_delaunay(HalfEdgeMesh delaunay);

// Bounded — clips cells to a convex polygon (CCW ring of vertices).
fn VoronoiDiagram? in_polygon(Vec3f[] sites, Vec3f[] polygon);

// Bounded — clips cells to an axis-aligned box. Thin wrapper over in_polygon.
fn VoronoiDiagram? in_box(Vec3f[] sites, Aabb bbox);

// Spherical — closed surface, no clipping needed.
fn VoronoiDiagram? on_sphere(Vec3f[] points, float radius);

// Inverse of from_delaunay: recover the Delaunay mesh from a Voronoi diagram via dual().
// Lossless when the original triangulation is recoverable from cell-site positions alone.
fn HalfEdgeMesh? to_delaunay(VoronoiDiagram diagram);
```

Implementation sketches (`src/voronoi/bounded.c3`):

- `in_polygon(sites, polygon)` — compute the unbounded Voronoi as a planar Delaunay first, then clip each cell against the polygon. Implementation choice between Sutherland–Hodgman per-cell vs. ghost-site reflection is M11's call; the API contract doesn't depend on which.
- `in_box(sites, bbox)` — build the four corners + edges as a CCW polygon and call `in_polygon`. No specialised AABB code path in v1; profile first.

### 5.8 Spherical Delaunay and Voronoi

Spherical Delaunay equals the 3D convex hull of the input points (a known equivalence for points on a sphere):

```c3
module cg::delaunay;
fn HalfEdgeMesh? on_sphere(Vec3f[] points, float radius);
```

Implementation: call `cg::hull::hull_3d(points)`, optionally re-project vertices to the sphere of given radius (input may not be exactly on the sphere). The result is a closed triangular `HalfEdgeMesh` — every face is a triangle, every half-edge has a twin, `is_boundary` returns `false` everywhere.

Spherical Voronoi is then `dual` with spherical circumcentres:

```c3
module cg::voronoi;
fn VoronoiDiagram? on_sphere(Vec3f[] points, float radius);
// Roughly: tri = delaunay::on_sphere(points, radius);
//          centres = geometry::circumcenters_on_sphere(tri, radius);
//          mesh = dual::dual(tri, centres);
//          return { mesh, sites: points.copy() };
```

The closed-surface property is the gift here: no boundary, no clipping, no `OPEN_CELL_ON_BOUNDARY` to worry about. The whole pipeline is three function calls and one struct construction.

### 5.9 Geometry helpers (`src/geometry/`)

```c3
module cg::geometry;

fn Vec3f circumcenter_planar(Vec3f a, Vec3f b, Vec3f c);
fn Vec3f circumcenter_on_sphere(Vec3f a, Vec3f b, Vec3f c, float radius);
fn Vec3f face_centroid(HalfEdgeMesh mesh, FaceIndex f);

fn Vec3f[]? circumcenters_planar(HalfEdgeMesh tri_mesh);            // owned, faults NON_TRIANGLE_FACE
fn Vec3f[]? circumcenters_on_sphere(HalfEdgeMesh tri_mesh, float r);
fn Vec3f[]  face_centroids(HalfEdgeMesh mesh);                       // owned

// Classical predicates, kept simple in v1 (non-robust). Robust adaptive versions later.
fn float orient_2d(Vec2f a, Vec2f b, Vec2f c);
fn float in_circle_2d(Vec2f a, Vec2f b, Vec2f c, Vec2f p);
fn float on_sphere(Vec3f a, Vec3f b, Vec3f c, Vec3f d, Vec3f p);
```

These are the building blocks the higher-level algorithms compose. Keeping them isolated in `cg::geometry` means the robust-predicate upgrade later is a single-module change.

### 5.10 Graph extraction (`src/graph/`)

```c3
module cg::graph;

// Voronoi graph from any VoronoiDiagram (planar bounded, planar in-polygon, or spherical).
fn VoronoiGraph? from_voronoi(VoronoiDiagram diagram);

// Delaunay graph builders, parameterised by which circumcentre to use.
// The wrapper functions are 2–3 lines of glue and exist for ergonomics.
fn DelaunayGraph? from_delaunay(HalfEdgeMesh delaunay, Vec3f[] circumcenters);
fn DelaunayGraph? from_planar_delaunay(HalfEdgeMesh delaunay);
fn DelaunayGraph? from_spherical_delaunay(HalfEdgeMesh delaunay, float radius);
```

Implementation outline for `from_voronoi`:

1. Copy `diagram.mesh.positions` → `vertices`, `diagram.sites` → `sites`.
2. Compute `centroids` using `cg::geometry::face_centroids(diagram.mesh)`.
3. Walk each face's half-edge cycle to build `cell_offsets` (cumulative ring sizes), `ring_indices` (origin vertex of each half-edge in cycle order), and `neighbor_indices` (face on the other side of each half-edge's twin, or `INVALID_FACE` if the twin is on the boundary).
4. One pass, allocations only for the four output arrays.

For `from_delaunay`, the analogous walk over each (triangular) face produces `triangle_vertices` (the three origins of the triangle's three half-edges) and `triangle_neighbors` (the three across-twin face indices). `circumcenters` is the caller-supplied array, copied into the graph.

Faults:

- `NON_TRIANGLE_FACE` from `from_delaunay*` when a non-triangular face is encountered.
- `DUAL_VERTEX_COUNT_MISMATCH` (reused) when `circumcenters.len != delaunay.faces.len`.

Once built, the graph is independently destroyable; the caller can `destroy()` the source `VoronoiDiagram` or `HalfEdgeMesh` if they only need the graph view.

---

## 6. Algorithm scope

| Module            | Algorithm             | Input → Output                                   | Notes                                                   |
| ----------------- | --------------------- | ------------------------------------------------ | ------------------------------------------------------- |
| `cg::hull`        | Convex hull 2D        | `Vec2f[]` → `int[]` (CCW ring)                   | Andrew's monotone chain.                                |
| `cg::hull`        | Convex hull 3D        | `Vec3f[]` → `HalfEdgeMesh`                       | Incremental hull. Used by spherical Delaunay.           |
| `cg::delaunay`    | Delaunay 2D           | `Vec3f[]` (z=0) → `HalfEdgeMesh`                 | Bowyer–Watson with super-triangle.                      |
| `cg::delaunay`    | Delaunay on sphere    | `Vec3f[]`, radius → `HalfEdgeMesh`               | = 3D convex hull, optionally re-projected. Closed mesh. |
| `cg::voronoi`     | Voronoi from Delaunay | `HalfEdgeMesh` → `VoronoiDiagram`                | Via `dual`. Unbounded cells fault.                      |
| `cg::voronoi`     | Voronoi in polygon    | `Vec3f[]` sites + polygon → `VoronoiDiagram`     | Cells clipped to convex polygon.                        |
| `cg::voronoi`     | Voronoi in box        | `Vec3f[]` sites + Aabb → `VoronoiDiagram`        | Wrapper over `in_polygon`.                              |
| `cg::voronoi`     | Voronoi on sphere     | `Vec3f[]` points, radius → `VoronoiDiagram`      | Via spherical Delaunay + spherical dual.                |
| `cg::voronoi`     | Voronoi → Delaunay    | `VoronoiDiagram` → `HalfEdgeMesh`                | Via `dual` with sites as positions.                     |
| `cg::dual`        | Generic dual          | `HalfEdgeMesh`, `Vec3f[]` → `HalfEdgeMesh`       | The bridge.                                             |
| `cg::graph`       | Voronoi graph view    | `VoronoiDiagram` → `VoronoiGraph`                | Flat per-cell access (CSR ring + neighbor storage).     |
| `cg::graph`       | Delaunay graph view   | `HalfEdgeMesh` + circumcentres → `DelaunayGraph` | Flat per-triangle access.                               |
| `cg::triangulate` | Polygon triangulation | `Vec3f[]` simple polygon → `int[]`               | Ear clipping.                                           |
| `cg::subdivide`   | Loop subdivision      | `HalfEdgeMesh` (tri) → `HalfEdgeMesh` (tri)      | One-step.                                               |
| `cg::primitives`  | Icosphere             | subdivisions, radius → `HalfEdgeMesh`            | Loop on icosahedron.                                    |
| `cg::primitives`  | Platonic solids       | → `HalfEdgeMesh`                                 | Tet, octa, icosa, cube (triangulated).                  |

Each algorithm sits behind a free-function entry point. Output is always a fully-owned mesh / diagram that the caller `defer`s `.destroy()` on.

---

## 7. Development plan

Each milestone is a commit (or short series). `c3c build && c3c test` is green at every milestone boundary. TDD per style §11 — failing test in first.

The milestones are ordered so that each builds only on what came before. The dependency chain is: foundation → topology → geometry helpers → dual → builders → bounded/spherical extensions.

### M0 — Project scaffolding

- [x] `c3-expert`: confirm version pin in `.claude/c3-skill.json`.
- [ ] `c3-expert`: confirm method-on-imported-type works (declare `fn int Foo.bar(&self)` from a different module than `Foo`'s defining module). One-line smoke test before committing to the architecture.
- [ ] `project.json` with module name `cg`, debug + release targets, test target.
- [ ] `src/cg.c3` umbrella, `src/types.c3` with vector aliases, identifier typedefs, `INVALID_*`, `Aabb`.
- [ ] `src/faults.c3` with cross-cutting faults from §4.3.
- [ ] `src/half_edge_mesh.c3` with `HalfEdge`, `HalfEdgeFace`, `HalfEdgeVertex`, `HalfEdgeMesh` struct declarations only (no methods yet).
- [ ] `test/test_smoke.c3` — empty `@test fn void test_compiles()`.
- [ ] `c3c build && c3c test` green.

### M1 — Half-edge construction and queries

- [x] `src/half_edge/builder.c3` — `from_triangles`, `from_triangles_with_attrs`, hash-map twin-pairing pass, fault paths.
- [ ] `src/half_edge/topology.c3` — `twin`, `next`, `prev`, `from_vertex`, `to_vertex`, `face_of`, `is_boundary`, `face_degree`.
- [ ] `HalfEdgeMesh.destroy(&self)` (probably in `builder.c3` since it pairs with construction).
- [ ] `test/test_builder.c3` — single tetrahedron round-trip; degenerate-input fault paths exhaustively (empty, count mismatch, out-of-range index, duplicate edge).
- [ ] `test/test_topology.c3` — twin/next/from/to spot-checks on tetrahedron and a hand-built diamond.
- [ ] `c3c build && c3c test` green.

### M2 — Walks and validation

- [x] `src/half_edge/walks.c3` — `vertex_one_ring_outgoing`, `vertex_one_ring_faces`, `face_half_edges`, `face_vertices`. Caller-supplied scratch buffers.
- [ ] `src/half_edge/validate.c3` — `validate(&self)` checks invariants 1–4 from §4.5; returns first violation as a fault.
- [ ] `test/test_walks.c3` — one-ring on tetrahedron (every vertex degree 3) and on a hand-built degree-5 vertex; validate returns OK on a known-good mesh and the right fault on a doctored bad one.
- [ ] `c3c build && c3c test` green.

### M3 — Rendering data

- [x] `src/render/rendering_data.c3` — `RenderingData` struct + `destroy` + `to_rendering_data` triangular path + polygonal fan path.
- [ ] Generated face-averaged normals when `mesh.normals.len == 0`; zero-fill UVs when absent.
- [ ] `test/test_render.c3` — round-trip from triangle soup; index count = `3 * faces.len` for triangular; generated normals unit-length and outward on tetrahedron; polygonal-face triangulation correct for a hand-built quad.
- [ ] `c3c build && c3c test` green.

### M4 — Edge flipping

- [x] `src/half_edge/flip.c3` — `is_flip_ok`, `flip`. Six-rewire ported from Godot. No external index buffer rewrite. Triangle check faults `NON_TRIANGLE_FACE`.
- [ ] `test/test_flip.c3` — diamond fixture: flip works; flipping back returns to original; boundary faults; would-create-duplicate faults; non-triangle face faults; Euler characteristic preserved.
- [ ] `c3c build && c3c test` green.

### M5 — Geometry helpers

- [x] `src/geometry/circumcenter.c3` — `circumcenter_planar`, `circumcenter_on_sphere`, `circumcenters_planar(mesh)`, `circumcenters_on_sphere(mesh, r)`.
- [ ] `src/geometry/centroid.c3` — `face_centroid`, `face_centroids(mesh)`.
- [ ] `src/geometry/predicates.c3` — `orient_2d`, `in_circle_2d`, `on_sphere` (non-robust v1).
- [ ] `test/test_geometry.c3` — known circumcentres of equilateral triangle (planar and spherical); centroid of a known polygon; predicate signs on hand-built cases.
- [ ] `c3c build && c3c test` green.

### M6 — Dual operation

- [x] `src/dual/dual.c3` — `dual(mesh, mode)` with `DualMode` enum and `dual_from_vertices(mesh, positions)` raw path.
- [x] Faults: `DUAL_VERTEX_COUNT_MISMATCH`, `DUAL_REQUIRES_CLOSED_MESH`.
- [x] Tests on a closed source mesh: dual of an icosahedron is a dodecahedron (each vertex of the icosa becomes a 5-gon face in the dual, by inspection).
- [x] Round-trip property test: `dual(dual(M, A), B)` topologically equals `M` (vertex/face/half-edge counts; correspondence by index).
- [x] `test/test_dual.c3` — dual of icosahedron, double-dual round-trip, mismatched vertex array faults, open-mesh faults.
- [x] `c3c build && c3c test` green.

### M7 — Convex hull 2D

- [ ] `src/hull/hull_2d.c3` — Andrew's monotone chain. Output `int[]` of indices into the input, CCW order.
- [ ] Faults: `EMPTY_INPUT`, `DEGENERATE_INPUT` (all collinear).
- [ ] `test/test_hull_2d.c3` — square; square + interior points; collinear fault.
- [ ] `c3c build && c3c test` green.

### M8 — Convex hull 3D

- [ ] `src/hull/hull_3d.c3` — incremental hull. Build initial tetrahedron from four non-coplanar points, add points one at a time, remove visible faces, stitch new faces around the horizon. Output: `HalfEdgeMesh` (closed, triangular, no boundary).
- [ ] Faults: `EMPTY_INPUT`, `DEGENERATE_INPUT` (fewer than 4 non-coplanar points).
- [ ] `test/test_hull_3d.c3` — cube corners (12-tri hull); tetrahedron (4-tri); coplanar fault.
- [ ] `c3c build && c3c test` green.

### M9 — Delaunay 2D

- [ ] `src/delaunay/delaunay_2d.c3` — Bowyer–Watson. Wrap input in super-triangle, insert points one at a time, remove triangles whose circumcircle contains the point (using `in_circle_2d`), retriangulate cavity, strip super-triangle at end. Output: `HalfEdgeMesh` (triangular, with planar boundary).
- [ ] Faults: `EMPTY_INPUT`, `DEGENERATE_INPUT` (all collinear).
- [ ] `test/test_delaunay_2d.c3` — square accepts either valid triangulation; regular grid satisfies Delaunay condition globally; cocircular tricky case.
- [ ] `c3c build && c3c test` green.

### M10 — Voronoi from Delaunay (unbounded path)

- [ ] `src/voronoi/voronoi.c3` — `VoronoiDiagram` struct + `destroy`; `from_delaunay` (faults `OPEN_CELL_ON_BOUNDARY` if the input has boundary); `to_delaunay` via `dual` with `sites` as positions.
- [ ] Implementation: compose `circumcenters_planar` + `dual::dual`. Mostly glue.
- [ ] `test/test_voronoi_unbounded.c3` — closed input mesh (e.g., dual of icosahedron) round-trips; planar input faults `OPEN_CELL_ON_BOUNDARY`.
- [ ] `c3c build && c3c test` green.

### M11 — Bounded Voronoi (in_polygon, in_box)

- [ ] `src/voronoi/bounded.c3` — `in_polygon(sites, polygon)`, `in_box(sites, bbox)`. Strategy decision: per-cell Sutherland–Hodgman vs. ghost-site reflection; pick one and document. AABB version is a thin wrapper that converts the box to a 4-vertex polygon ring.
- [ ] Faults: `EMPTY_INPUT`, `NON_CONVEX_BOUNDING_POLYGON` (v1 limit; document as a constraint).
- [ ] `test/test_voronoi_bounded.c3` — 4 sites in a unit square produce 4 cells with correct boundary edges on the square; sites near corner produce expected clipping; non-convex polygon faults.
- [ ] `c3c build && c3c test` green.

### M12 — Spherical Delaunay and Voronoi

- [ ] `src/delaunay/delaunay_sphere.c3` — `on_sphere(points, radius)` calling `hull_3d` and re-projecting vertices to radius.
- [ ] `src/voronoi/voronoi_sphere.c3` — `on_sphere(points, radius)` composing spherical Delaunay + spherical circumcentres + `dual`.
- [ ] `test/test_delaunay_sphere.c3` — 12 icosahedron vertices on sphere produce icosahedral triangulation; closed mesh (`is_boundary` false everywhere); all triangles have all three vertices on the sphere within tolerance.
- [ ] `test/test_voronoi_sphere.c3` — same 12 input points produce dodecahedral Voronoi; `from_delaunay` round-trip works because there's no boundary; cells cover the sphere (sum of cell areas ≈ `4πr²`).
- [ ] `c3c build && c3c test` green.

### M13 — Voronoi and Delaunay graph views

- [ ] `src/graph/voronoi_graph.c3` — `VoronoiGraph` struct, `VoronoiCellView`, `destroy`, `cell_count`, `cell`, `from_voronoi(diagram)`.
- [ ] `src/graph/delaunay_graph.c3` — `DelaunayGraph` struct, `DelaunayTriangle`, `DelaunayTriangleView`, `destroy`, `triangle_count`, `triangle`, `from_delaunay(mesh, circumcenters)`, `from_planar_delaunay(mesh)`, `from_spherical_delaunay(mesh, radius)`.
- [ ] `test/test_voronoi_graph.c3` — graph from a 4-site square Voronoi (M11 fixture): assert `cell_count`, ring lengths, neighbor symmetry (`graph.cell(a).neighbors` contains `b` iff `graph.cell(b).neighbors` contains `a`), centroid sanity (lies inside cell), `INVALID_FACE` on boundary edges where expected.
- [ ] `test/test_delaunay_graph.c3` — graph from a known triangulation: assert `triangle_count`, neighbor symmetry, circumcenter values match `cg::geometry::circumcenters_planar` output element-wise.
- [ ] Spherical variant test: graph from spherical Delaunay (M12 fixture) — every triangle's `neighbors` has no `INVALID_FACE` (closed surface).
- [ ] `c3c build && c3c test` green.

### M14 — Polygon triangulation, Loop subdivision, primitives

- [ ] `src/triangulate/ear_clipping.c3` — input simple polygon as `Vec3f[]` (CCW); output `int[]` triangle indices; O(n²); `NON_SIMPLE_POLYGON`, `INSUFFICIENT_VERTICES` faults.
- [ ] `src/subdivide/loop.c3` — one-step Loop subdivision; allocates fresh mesh; original untouched.
- [ ] `src/primitives/platonic.c3` — tetrahedron, octahedron, icosahedron, cube (triangulated).
- [ ] `src/primitives/icosphere.c3` — icosahedron + N Loop subdivisions + sphere projection.
- [ ] `test/test_*` for each of the above.
- [ ] `c3c build && c3c test` green.

### M15 — Constrained Delaunay

Add edge constraints to the Bowyer–Watson pipeline so callers can specify required edges that must appear in the triangulation.

- [ ] `src/delaunay/constrained.c3` — `constrained_delaunay(sites, constraints)` where `constraints` is a `pair<VertexIndex>[2]` array of required edges. Edge recovery by edge-flipping or cavity retriangulation around the constraint.
- [ ] Faults: `CROSSING_CONSTRAINTS`, `DUPLICATE_CONSTRAINT`, `EMPTY_INPUT`.
- [ ] `test/test_delaunay_constrained.c3` — simple polygon as Delaunay with its edges constrained; crossing constraints fault; duplicate fault; verify constrained edges present in output mesh.
- [ ] `c3c build && c3c test` green.

### M16 — Robust adaptive predicates

Replace the v1 non-robust geometry predicates with adaptive Shewchuk-style exact arithmetic.

- [ ] `src/geometry/robust.c3` — `orient_2d_robust`, `in_circle_2d_robust`, `on_sphere_robust` using adaptive precision with expansion arithmetic. Fall through from fast float → exact if result is ambiguous.
- [ ] `test/test_geometry_robust.c3` — near-degenerate inputs that trip up the non-robust versions produce correct signs with the robust versions; standard test suite from Shewchuk's predicates.c.
- [ ] Existing algorithm modules (`delaunay_2d.c3`, `hull*.c3`) optionally swap to robust predicates behind a compile-time flag (`$if ROBUST_PREDICATES`).
- [ ] `c3c build && c3c test` green (with and without the flag).

### M17 — Non-convex bounding polygons in voronoi::in_polygon

Lift the convex-only restriction on bounding polygons so `in_polygon` accepts simple (possibly non-convex) polygons.

- [ ] `src/voronoi/bounded.c3` — extend `in_polygon` clipping to handle non-convex regions. Strategy: decompose the non-convex polygon into convex sub-polygons (ear clipping, already available from M14), clip each Voronoi cell against each sub-polygon, and union the per-cell results into a single face.
- [ ] Remove or relax `NON_CONVEX_BOUNDING_POLYGON` fault.
- [ ] `test/test_voronoi_bounded.c3` — add non-convex bounding polygon cases: L-shaped polygon, star-shaped polygon; verify cells are correctly clipped to the bounding shape.
- [ ] `c3c build && c3c test` green.

### M18 — Catmull–Clark subdivision

Add Catmull–Clark subdivision as a counterpart to Loop subdivision, targeting quad-dominant meshes.

- [ ] `src/subdivide/catmull_clark.c3` — `catmull_clark(mesh)` producing a new `HalfEdgeMesh`. Compute face points (centroids of polygonal faces), edge points (average of incident face points and edge endpoints), and new vertex positions (weighted blend of original vertex, incident face points, and incident edge points). Output all-quad mesh from an arbitrary polygonal input.
- [ ] Faults: `EMPTY_INPUT`, `NON_MANIFOLD_INPUT`.
- [ ] `test/test_subdivide_catmull_clark.c3` — subdivide a cube: verify face count quadruples, all output faces are quads, limit-surface tangency for a known case; round-trip with a single-quad input.
- [ ] `c3c build && c3c test` green.

### M19 — Half-edge collapse / split (mesh simplification)

Add local mesh-simplification primitives: edge collapse and vertex split.

- [ ] `src/half_edge/collapse.c3` — `collapse(he)` removing an edge and merging its two vertices, updating all affected topology. Returns an optional fault. Preserves manifold property when possible.
- [ ] `src/half_edge/split.c3` — `split(he)` inserting a new vertex at the midpoint of a half-edge, splitting the incident face(s).
- [ ] Faults: `COLLAPSE_NOT_ALLOWED` (would produce non-manifold or degenerate geometry), `BOUNDARY_HALF_EDGE`.
- [ ] `test/test_collapse.c3` — collapse an interior edge on a tetrahedron, verify vertex/face counts, Euler characteristic; collapse that would produce a non-manifold faults.
- [ ] `test/test_split.c3` — split an edge on a tetrahedron, verify new vertex and face counts; split a boundary edge.
- [ ] `c3c build && c3c test` green.

### M20 — 3D Delaunay (tetrahedralisation) and 3D volumetric Voronoi

Extend Delaunay and Voronoi to 3D, producing tetrahedral meshes and volumetric Voronoi cells.

- [ ] `src/delaunay/delaunay_3d.c3` — Bowyer–Watson in 3D: 3D super-tetrahedron, point insertion, cavity identified by circumsphere test (`in_sphere_3d` predicate), cavity retriangulated into tetrahedra. Output: tetrahedral `HalfEdgeMesh` (all faces degree-3, closed, no boundary). May reuse `HalfEdgeMesh` with the understanding that faces are now volume-bounding triangles; interior tetrahedra require a parallel tet array or a second-mesh approach.
- [ ] `src/geometry/predicates_3d.c3` — `in_sphere_3d(a, b, c, d, p)` for the 3D Delaunay predicate.
- [ ] `src/voronoi/voronoi_3d.c3` — `VoronoiDiagram3D` struct with a 3D dual; each Voronoi cell is a convex polyhedron. CSR or mesh-based storage. `from_delaunay_3d`, `to_delaunay_3d` via 3D dual.
- [ ] `test/test_delaunay_3d.c3` — a cube's 8 corners as input produce a valid tetrahedralisation, all input vertices present, no degenerate tets, Delaunay condition holds for random interior points.
- [ ] `test/test_voronoi_3d.c3` — Voronoi from a known tetrahedralisation round-trips; cell volume sum equals bounding volume.
- [ ] `c3c build && c3c test` green.

---

## 8. Open questions for `c3-expert` verification

Items to resolve at M0 before code goes in. None of these change the architecture; they're surface-syntax confirmations.

- [ ] Vector type spelling: `float[<3>]` vs `std::math::Vec3f` vs both with one as alias.
- [ ] Methods on imported types: `fn Ret HalfEdgeMesh.method(...)` declared in `module cg::half_edge` for a `HalfEdgeMesh` defined in `module cg`. Verify with a one-file smoke test in M0; the entire architecture rests on this.
- [ ] HashMap instantiation syntax in the configured release; key type for edge pairing (probably `Vec2i` or a packed `long`).
- [ ] Allocator-passing convention: explicit `Allocator` parameter vs context allocator. Pick one in M0 and apply uniformly.
- [ ] `*self = {};` zero-clear syntax.
- [ ] Test annotations (`@test`) and assertion macros against the c3vq test idiom in style §11.
- [ ] `project.json` shape for a library project (vs. application).
- [ ] Visibility annotations: confirm submodule symbols in `cg::voronoi` can see `cg::half_edge` operations through `import cg::half_edge`, and that operations on `cg::HalfEdgeMesh` declared from `cg::half_edge` are usable from `cg::voronoi` without re-import.

---

## 9. What "done" looks like at v1

After M13, a downstream user can do all of the following with the listed lines of code:

```c3
import cg;
import cg::half_edge;
import cg::delaunay;
import cg::voronoi;
import cg::graph;
import cg::render;

fn void? planar_voronoi_in_box(Vec3f[] sites, Aabb bbox) {
    VoronoiDiagram vor = cg::voronoi::in_box(sites, bbox)!;
    defer vor.destroy();

    RenderingData rd = vor.mesh.to_rendering_data()!;
    defer rd.destroy();

    upload_to_gpu(rd);
}

fn void? spherical_voronoi(Vec3f[] points_on_sphere, float radius) {
    VoronoiDiagram vor = cg::voronoi::on_sphere(points_on_sphere, radius)!;
    defer vor.destroy();

    RenderingData rd = vor.mesh.to_rendering_data()!;
    defer rd.destroy();

    upload_to_gpu(rd);
}

fn void? delaunay_voronoi_roundtrip(Vec3f[] sites_on_sphere, float radius) {
    HalfEdgeMesh tri = cg::delaunay::on_sphere(sites_on_sphere, radius)!;
    defer tri.destroy();

    VoronoiDiagram vor = cg::voronoi::from_delaunay(tri)!;
    defer vor.destroy();

    HalfEdgeMesh tri2 = cg::voronoi::to_delaunay(vor)!;   // back to triangulation
    defer tri2.destroy();

    // tri and tri2 are topologically equivalent.
}

fn void? iterate_cells(Vec3f[] sites, Aabb bbox) {
    VoronoiDiagram vor = cg::voronoi::in_box(sites, bbox)!;
    defer vor.destroy();

    VoronoiGraph graph = cg::graph::from_voronoi(vor)!;
    defer graph.destroy();

    for (FaceIndex c = 0; c < graph.cell_count(); c++) {
        VoronoiCellView cell = graph.cell(c);
        // cell.site, cell.centroid, cell.ring, cell.neighbors all flat-array access
        foreach (n : cell.neighbors) {
            if (n != INVALID_FACE) {
                // visit neighboring cell — no half-edge walks required
            }
        }
    }
}
```

That's the minimum useful surface. Polygon triangulation, subdivision, and primitives extend it but aren't on the critical path to "Voronoi of points in 2D, in a polygon, in a box, or on a sphere is one call away — and iterating its cells with neighbor information is one more".
