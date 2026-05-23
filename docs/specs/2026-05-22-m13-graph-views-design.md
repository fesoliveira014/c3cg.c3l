# M13 — Voronoi and Delaunay Graph Views Design

**Date:** 2026-05-22
**Status:** draft

## Overview

M13 adds flat read-only graph views over the existing mesh-based representations.
Both are snapshots — built from a `VoronoiDiagram` or `HalfEdgeMesh`, owned independently.
Once built, the graph carries everything needed for read-only iteration and the source can be discarded.

## API

All declarations in `src/c3cg.c3i` under `module cg::graph;`.

### VoronoiGraph

```c3
struct VoronoiGraph {
    Vec3f[] vertices;           // Voronoi vertices (circumcenters of Delaunay triangles)
    Vec3f[] sites;              // one site per cell (= FaceIndex)
    Vec3f[] centroids;          // one centroid per cell
    int[]    cell_offsets;      // length = cell_count + 1
    VertexIndex[] ring_indices;      // vertex ring for each cell
    FaceIndex[]   neighbor_indices;  // same offsets as ring_indices, INVALID_FACE on boundary
}

struct VoronoiCellView {
    FaceIndex     index;
    Vec3f         site;
    Vec3f         centroid;
    VertexIndex[] ring;       // slice into graph.ring_indices
    FaceIndex[]   neighbors;  // slice into graph.neighbor_indices
}

fn VoronoiGraph?   from_voronoi(Allocator alloc, VoronoiDiagram* diagram);
fn void            VoronoiGraph.destroy(&self);
fn int             VoronoiGraph.cell_count(&self);
fn VoronoiCellView VoronoiGraph.cell(&self, FaceIndex c);
```

`from_voronoi` borrows the diagram (pointer), caller retains ownership and destruction responsibility.

`ring_indices` and `neighbor_indices` share a single `cell_offsets` array (CSR layout).
Cell `c`'s ring is `ring_indices[cell_offsets[c] .. cell_offsets[c+1]]`.
The neighbor across edge `(ring[i], ring[(i+1)%k])` is `neighbor_indices[cell_offsets[c] + i]`.
Boundary edges have `INVALID_FACE` as neighbor.

`cell(c)` constructs the view inline — slices into graph storage, no copying.

### DelaunayGraph

```c3
struct DelaunayTriangle {
    VertexIndex[3] vertices;    // three vertices of the triangle
    FaceIndex[3]   neighbors;   // INVALID_FACE on boundary edges
}

struct DelaunayGraph {
    Vec3f[]            vertices;       // input points (Delaunay vertices)
    Vec3f[]            circumcenters;  // one per triangle
    DelaunayTriangle[] triangles;
}

struct DelaunayTriangleView {
    FaceIndex      index;
    Vec3f          circumcenter;
    VertexIndex[3] vertices;
    FaceIndex[3]   neighbors;
}

fn DelaunayGraph?         from_delaunay(Allocator alloc, HalfEdgeMesh* mesh, Vec3f[] circumcenters);
fn DelaunayGraph?         from_planar_delaunay(Allocator alloc, HalfEdgeMesh* mesh);
fn DelaunayGraph?         from_spherical_delaunay(Allocator alloc, HalfEdgeMesh* mesh, float radius);
fn void                   DelaunayGraph.destroy(&self);
fn int                    DelaunayGraph.triangle_count(&self);
fn DelaunayTriangleView   DelaunayGraph.triangle(&self, FaceIndex t);
```

All constructors borrow the mesh (pointer), caller retains ownership.

Faults:
- `EMPTY_INPUT` if mesh has no faces
- `NON_TRIANGLE_FACE` if a non-triangular face is encountered
- `DUAL_VERTEX_COUNT_MISMATCH` if `circumcenters.len != mesh.faces.len` (for `from_delaunay`)
- Convenience constructors propagate faults from `circumcenters_planar` / `circumcenters_on_sphere`

## Files

| File | Content |
|------|---------|
| `src/graph/voronoi_graph.c3` | `VoronoiGraph`, `VoronoiCellView`, `destroy`, `cell_count`, `cell`, `from_voronoi` |
| `src/graph/delaunay_graph.c3` | `DelaunayGraph`, `DelaunayTriangle`, `DelaunayTriangleView`, `destroy`, `triangle_count`, `triangle`, `from_delaunay`, `from_planar_delaunay`, `from_spherical_delaunay` |
| `test/test_voronoi_graph.c3` | Voronoi graph tests |
| `test/test_delaunay_graph.c3` | Delaunay graph tests |

## Test Plan

### test_voronoi_graph.c3

1. **4-site square bounded Voronoi**
   - `from_voronoi` on bounded 4-site Voronoi (M11 fixture)
   - `cell_count` == 4
   - Each cell ring has vertices
   - Neighbor symmetry: `graph.cell(a).neighbors` contains `b` iff `graph.cell(b).neighbors` contains `a`
   - Boundary edges have `INVALID_FACE` where expected
   - Centroids lie inside corresponding cells (within tolerance)

2. **Icosahedron Voronoi (spherical, no boundary)**
   - `from_voronoi` on spherical Voronoi (M12 fixture)
   - 12 cells, no boundary edges, no `INVALID_FACE` in any neighbor slot
   - Neighbor symmetry holds for all cells

3. **Empty input** → `EMPTY_INPUT` fault

4. **Cell view slices match ring_indices lengths**

### test_delaunay_graph.c3

5. **Known planar triangulation**
   - 4-site square Delaunay → `from_delaunay` with `circumcenters_planar`
   - `triangle_count` matches face count
   - `circumcenters` match `circumcenters_planar` output element-wise
   - Neighbor symmetry: `graph.triangle(a).neighbors[i] == b` implies `graph.triangle(b).neighbors[j] == a`

6. **Convenience constructors**
   - `from_planar_delaunay` produces same result as manual `from_delaunay` + `circumcenters_planar`
   - `from_spherical_delaunay` produces same result as manual `from_delaunay` + `circumcenters_on_sphere`

7. **Icosahedron Delaunay (spherical, no boundary)**
   - 20 triangles, no boundary, no `INVALID_FACE`

8. **Empty input** → `EMPTY_INPUT` fault

9. **Non-triangle face** → `NON_TRIANGLE_FACE` fault

10. **Vertex count mismatch** → `DUAL_VERTEX_COUNT_MISMATCH` fault
