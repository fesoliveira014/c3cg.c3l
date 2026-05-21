# M10 — Voronoi from Delaunay (unbounded)

## Overview

Voronoi diagram construction as the dual of a Delaunay triangulation. Composes existing `circumcenters_planar` and `dual::dual` to produce a polygonal `HalfEdgeMesh` where each face is a Voronoi cell. A parallel `sites` array maps each cell to its generating input point.

## Module

`module cg::voronoi;` — new file `src/voronoi/voronoi.c3`.

Imports: `cg`, `cg::geometry` (`circumcenters_planar`), `cg::dual` (`dual`).

Umbrella addition (`src/c3cg.c3i`):

```c3
module cg::voronoi;
import cg;

struct VoronoiDiagram {
    HalfEdgeMesh mesh;
    Vec3f[] sites;
}

fn void VoronoiDiagram.destroy(&self);
fn VoronoiDiagram? from_delaunay(Allocator alloc, HalfEdgeMesh* delaunay);
fn HalfEdgeMesh? to_delaunay(Allocator alloc, VoronoiDiagram* diagram);
```

## Public API

```c3
struct VoronoiDiagram {
    HalfEdgeMesh mesh;   // each face = one Voronoi cell (polygonal)
    Vec3f[]      sites;  // sites[face_index] = original point for that cell
}

fn void VoronoiDiagram.destroy(&self) {
    self.mesh.destroy();
    free(self.sites);
    *self = {};
}

fn VoronoiDiagram? from_delaunay(Allocator alloc, HalfEdgeMesh* delaunay);
fn HalfEdgeMesh? to_delaunay(Allocator alloc, VoronoiDiagram* diagram);
```

### `from_delaunay`

Converts a Delaunay triangulation to a Voronoi diagram.

1. Validates `delaunay` (must be closed — no boundary edges). If any half-edge has `twin == INVALID_HE`, fault `OPEN_CELL_ON_BOUNDARY`.
2. Computes circumcenters via `geometry::circumcenters_planar(alloc, delaunay)`.
3. Computes dual mesh via `dual::dual(alloc, delaunay, centers)` using `DualMode.CIRCUMCENTER` (or the raw `dual_from_vertices` to avoid the mode enum). The dual mesh has one face per Delaunay vertex, with face vertices at the circumcenters of the Delaunay triangles incident to that vertex.
4. Copies `delaunay.positions` → `sites` array (one site per Voronoi cell).
5. Returns `VoronoiDiagram { mesh, sites }`.

### `to_delaunay`

Recovers the Delaunay triangulation from a Voronoi diagram. Applies `dual::dual` with `diagram.sites` as dual vertex positions. Topologically equivalent to the original Delaunay mesh (modulo position values).

## Faults

| Fault | When |
|-------|------|
| `OPEN_CELL_ON_BOUNDARY` | Delaunay mesh has boundary edges (unbounded Voronoi cells) |

No new `faultdef` needed — `OPEN_CELL_ON_BOUNDARY` already exists in `src/c3cg.c3i`.

## Memory

`VoronoiDiagram` owns both `mesh` and `sites`. Caller must call `diagram.destroy()`.

## Tests

File: `test/test_voronoi.c3` (or `test/test_voronoi_unbounded.c3`).

| Test | Input | Expected |
|------|-------|----------|
| Closed mesh (icosahedron dual) | Icosahedron mesh | Produces valid Voronoi (dodecahedron), cell count = vertex count |
| Boundary mesh faults | Single triangle (has boundary) | `OPEN_CELL_ON_BOUNDARY` |
| Round-trip | `to_delaunay(from_delaunay(mesh))` | Topologically equivalent to original |

## Comparison to voronator-rs

| | c3cg | voronator-rs |
|---|---|---|
| Algorithm | `dual::dual` (structured) | Manual half-edge walk per vertex |
| Output | `HalfEdgeMesh` (general) | `Vec<Vec<Point>>` per cell |
| Clipping | None in M10 (M11 adds) | Built-in via Sutherland-Hodgman |
| Sites | Parallel array indexed by FaceIndex | Inline with cell data |
