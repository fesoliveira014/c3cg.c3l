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

1. Pre-scans `delaunay.half_edges` for boundary edges (`twin == INVALID_HE`). If any found, fault `OPEN_CELL_ON_BOUNDARY`.
2. Calls `mesh.validate()!` on the Delaunay mesh.
3. Computes circumcenters via `geometry::circumcenters_planar(alloc, delaunay)!`. The centers array has length `delaunay.faces.len`.
4. Computes dual mesh via `dual::dual_from_vertices(alloc, delaunay, centers)!`. Frees `centers` after the call (dual copies positions internally).
5. Allocates `sites` by copying `delaunay.positions`. If allocation fails, `mesh.destroy()` fires via `defer catch`.
6. Returns `VoronoiDiagram { mesh, sites }`.

### `to_delaunay`

Recovers the Delaunay triangulation from a Voronoi diagram. Applies `dual::dual_from_vertices(alloc, &diagram.mesh, diagram.sites)!`. Topologically equivalent to the original Delaunay mesh.

## Faults

| Fault                   | When                                                       |
| ----------------------- | ---------------------------------------------------------- |
| `OPEN_CELL_ON_BOUNDARY` | Delaunay mesh has boundary edges (unbounded Voronoi cells) |

No new `faultdef` needed — `OPEN_CELL_ON_BOUNDARY` already exists in `src/c3cg.c3i`.

## Memory

`VoronoiDiagram` owns both `mesh` and `sites`. Caller must call `diagram.destroy()`.

Internal: `centers` is freed after `dual_from_vertices` returns (dual copies positions). `sites` is a fresh copy of `delaunay.positions`. On fault after `mesh` is built, `defer catch mesh.destroy()` cleans up.

## Tests

File: `test/test_voronoi.c3` (or `test/test_voronoi_unbounded.c3`).

| Test                           | Input                              | Expected                                                                                                                                  |
| ------------------------------ | ---------------------------------- | ----------------------------------------------------------------------------------------------------------------------------------------- |
| Closed mesh (icosahedron dual) | Icosahedron mesh                   | Cell count = vertex count; `sites.len == mesh.faces.len`; `mesh.positions.len == delaunay.faces.len`; `sites[i] == delaunay.positions[i]` |
| Boundary mesh faults           | Single triangle (has boundary)     | `OPEN_CELL_ON_BOUNDARY` (not `DUAL_REQUIRES_CLOSED_MESH`)                                                                                 |
| Round-trip                     | `to_delaunay(from_delaunay(mesh))` | Topologically equivalent (same vertex/face/half-edge counts)                                                                              |

## Comparison to voronator-rs

|           | c3cg                                | voronator-rs                     |
| --------- | ----------------------------------- | -------------------------------- |
| Algorithm | `dual::dual` (structured)           | Manual half-edge walk per vertex |
| Output    | `HalfEdgeMesh` (general)            | `Vec<Vec<Point>>` per cell       |
| Clipping  | None in M10 (M11 adds)              | Built-in via Sutherland-Hodgman  |
| Sites     | Parallel array indexed by FaceIndex | Inline with cell data            |
