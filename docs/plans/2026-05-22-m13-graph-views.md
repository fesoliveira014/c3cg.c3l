# M13 — Voronoi and Delaunay Graph Views Implementation Plan

> **For Hermes:** Use subagent-driven-development skill to implement this plan task-by-task.

**Goal:** Add `VoronoiGraph` and `DelaunayGraph` flat read-only views over existing mesh representations.

**Architecture:** CSR storage for Voronoi (variable-degree cells), fixed-width per-triangle arrays for Delaunay. Both are snapshots — built from source mesh, owned independently. View structs provide by-value windowing without copying.

**Spec:** `docs/specs/2026-05-22-m13-graph-views-design.md`

**Key existing APIs used:**
- `geometry::face_centroids(alloc, mesh)` → `Vec3f[]` (one per face — Voronoi cell centroids)
- `geometry::circumcenters_planar(alloc, mesh)` → `Vec3f[]` (one per face — Delaunay circumcenters)
- `geometry::circumcenters_on_sphere(alloc, mesh, radius)` → `Vec3f[]` (spherical circumcenters)
- `HalfEdgeMesh.face_degree(f)` → `int` (ring size)
- `HalfEdgeMesh.face_half_edges(f, out[]`) → walk face half-edges
- `HalfEdgeMesh.face_vertices(f, out[])` → walk face vertices
- `HalfEdgeMesh.twin(he)` → `HeIndex`
- `HalfEdgeMesh.face_of(he)` → `FaceIndex`
- `HalfEdgeMesh.from_vertex(he)` → `VertexIndex`
- `HalfEdgeMesh.is_boundary(he)` → `bool`
- `HalfEdgeMesh.validate()` → validates mesh topology
- `RenderingData.destroy(&self)` — pattern for flat-array struct `destroy`

---

### Task 1: Stubs + umbrella declarations (green build)

**Objective:** Create placeholder modules and add all API declarations to the umbrella so the build compiles cleanly. No real logic yet.

**Files:**
- Create: `src/graph/voronoi_graph.c3`
- Create: `src/graph/delaunay_graph.c3`
- Create: `test/test_voronoi_graph.c3`
- Create: `test/test_delaunay_graph.c3`
- Modify: `src/c3cg.c3i`

---

**Step 1: Create `src/graph/voronoi_graph.c3` stub**

```c3
module cg::graph;
import cg;
import cg::geometry;

struct VoronoiGraph {
    Vec3f[]       vertices;
    Vec3f[]       sites;
    Vec3f[]       centroids;
    int[]         cell_offsets;
    VertexIndex[] ring_indices;
    FaceIndex[]   neighbor_indices;
}

struct VoronoiCellView {
    FaceIndex     index;
    Vec3f         site;
    Vec3f         centroid;
    VertexIndex[] ring;
    FaceIndex[]   neighbors;
}

fn void VoronoiGraph.destroy(&self)
{
    free(self.vertices);
    free(self.sites);
    free(self.centroids);
    free(self.cell_offsets);
    free(self.ring_indices);
    free(self.neighbor_indices);
    *self = {};
}

fn int VoronoiGraph.cell_count(&self)
{
    return (int)self.sites.len;
}

fn VoronoiCellView VoronoiGraph.cell(&self, FaceIndex c)
{
    int start = self.cell_offsets[(int)c];
    int end   = self.cell_offsets[(int)c + 1];
    return {
        .index     = c,
        .site      = self.sites[(int)c],
        .centroid  = self.centroids[(int)c],
        .ring      = self.ring_indices[start..end],
        .neighbors = self.neighbor_indices[start..end],
    };
}

fn VoronoiGraph? from_voronoi(Allocator alloc, VoronoiDiagram* diagram)
{
    if (diagram.mesh.faces.len == 0) return cg::EMPTY_INPUT~;
    VoronoiGraph g;
    g.sites = copy_sites(alloc, diagram)!;
    g.centroids = geometry::face_centroids(alloc, &diagram.mesh)!;
    g.vertices = copy_positions(alloc, &diagram.mesh)!;
    g.cell_offsets = alloc_offsets(alloc, (usz)g.sites.len)!;
    g.ring_indices = {};
    g.neighbor_indices = {};
    return g;
}

// ── internal helpers (not in umbrella) ──

fn Vec3f[]? copy_sites(Allocator alloc, VoronoiDiagram* diagram)
{
    Vec3f[] out = mem::alloc::new_array(alloc, Vec3f, (sz)diagram.sites.len);
    for (usz i = 0; i < diagram.sites.len; i++) out[i] = diagram.sites[i];
    return out;
}

fn Vec3f[]? copy_positions(Allocator alloc, HalfEdgeMesh* mesh)
{
    Vec3f[] out = mem::alloc::new_array(alloc, Vec3f, (sz)mesh.positions.len);
    for (usz i = 0; i < mesh.positions.len; i++) out[i] = mesh.positions[i];
    return out;
}

fn int[]? alloc_offsets(Allocator alloc, usz cell_count)
{
    int[] out = mem::alloc::new_array(alloc, int, (sz)(cell_count + 1));
    for (usz i = 0; i <= cell_count; i++) out[i] = 0;
    return out;
}
```

---

**Step 2: Create `src/graph/delaunay_graph.c3` stub**

```c3
module cg::graph;
import cg;
import cg::geometry;

struct DelaunayTriangle {
    VertexIndex[3] vertices;
    FaceIndex[3]   neighbors;
}

struct DelaunayGraph {
    Vec3f[]            vertices;
    Vec3f[]            circumcenters;
    DelaunayTriangle[] triangles;
}

struct DelaunayTriangleView {
    FaceIndex      index;
    Vec3f          circumcenter;
    VertexIndex[3] vertices;
    FaceIndex[3]   neighbors;
}

fn void DelaunayGraph.destroy(&self)
{
    free(self.vertices);
    free(self.circumcenters);
    free(self.triangles);
    *self = {};
}

fn int DelaunayGraph.triangle_count(&self)
{
    return (int)self.triangles.len;
}

fn DelaunayTriangleView DelaunayGraph.triangle(&self, FaceIndex t)
{
    DelaunayTriangle tri = self.triangles[(int)t];
    return {
        .index         = t,
        .circumcenter  = self.circumcenters[(int)t],
        .vertices      = tri.vertices,
        .neighbors     = tri.neighbors,
    };
}

fn DelaunayGraph? from_delaunay(Allocator alloc, HalfEdgeMesh* mesh, Vec3f[] circumcenters)
{
    if (mesh.faces.len == 0) return cg::EMPTY_INPUT~;
    DelaunayGraph g;
    g.vertices = copy_vertex_positions(alloc, mesh)!;
    g.circumcenters = copy_circumcenters(alloc, circumcenters)!;
    g.triangles = {};
    return g;
}

fn DelaunayGraph? from_planar_delaunay(Allocator alloc, HalfEdgeMesh* mesh)
{
    return from_delaunay(alloc, mesh, {});
}

fn DelaunayGraph? from_spherical_delaunay(Allocator alloc, HalfEdgeMesh* mesh, float radius)
{
    _ = radius;
    return from_delaunay(alloc, mesh, {});
}

// ── internal helpers (not in umbrella) ──

fn Vec3f[]? copy_vertex_positions(Allocator alloc, HalfEdgeMesh* mesh)
{
    Vec3f[] out = mem::alloc::new_array(alloc, Vec3f, (sz)mesh.positions.len);
    for (usz i = 0; i < mesh.positions.len; i++) out[i] = mesh.positions[i];
    return out;
}

fn Vec3f[]? copy_circumcenters(Allocator alloc, Vec3f[] src)
{
    Vec3f[] out = mem::alloc::new_array(alloc, Vec3f, (sz)src.len);
    for (usz i = 0; i < src.len; i++) out[i] = src[i];
    return out;
}
```

---

**Step 3: Add umbrella declarations to `src/c3cg.c3i`**

Add after the `module cg::utils::ppm;` block (at the end of the file):

```c3
module cg::graph;
import cg;

struct VoronoiGraph {
    Vec3f[]       vertices;
    Vec3f[]       sites;
    Vec3f[]       centroids;
    int[]         cell_offsets;
    VertexIndex[] ring_indices;
    FaceIndex[]   neighbor_indices;
}

struct VoronoiCellView {
    FaceIndex     index;
    Vec3f         site;
    Vec3f         centroid;
    VertexIndex[] ring;
    FaceIndex[]   neighbors;
}

fn VoronoiGraph?   from_voronoi(Allocator alloc, VoronoiDiagram* diagram);
fn void            VoronoiGraph.destroy(&self);
fn int             VoronoiGraph.cell_count(&self);
fn VoronoiCellView VoronoiGraph.cell(&self, FaceIndex c);

struct DelaunayTriangle {
    VertexIndex[3] vertices;
    FaceIndex[3]   neighbors;
}

struct DelaunayGraph {
    Vec3f[]            vertices;
    Vec3f[]            circumcenters;
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

Note: The VoronoiGraph-related declarations need `VoronoiDiagram` in scope, so add `import cg::voronoi;` at the top of the `module cg::graph;` block in the umbrella. However, the umbrella already declares `cg::voronoi` before `cg::graph`, and all modules in the umbrella share `import cg;`. Since `VoronoiDiagram` is declared in the `cg::voronoi` umbrella block and umbrella-declared types are visible across the umbrella, an explicit import should not be needed. If `c3c` requires it, add `import cg::voronoi;` inside the `module cg::graph;` block.

---

**Step 4: Create `test/test_voronoi_graph.c3` stub**

```c3
module test;

// Placeholder — real tests in Task 4.
fn void test_voronoi_graph_compiles() @test
{
}
```

---

**Step 5: Create `test/test_delaunay_graph.c3` stub**

```c3
module test;

// Placeholder — real tests in Task 5.
fn void test_delaunay_graph_compiles() @test
{
}
```

---

**Step 6: Build and commit**

```bash
c3c build debug && c3c test
```

Expected: build passes, all existing tests pass, 2 new stub tests pass.

```bash
git add src/graph/voronoi_graph.c3 src/graph/delaunay_graph.c3 src/c3cg.c3i test/test_voronoi_graph.c3 test/test_delaunay_graph.c3
git commit -m "cg::graph: stub modules and umbrella declarations (M13)"
```

---

### Task 2: Implement `from_voronoi`

**Objective:** Replace the `from_voronoi` stub body in `src/graph/voronoi_graph.c3` with the full CSR construction algorithm. Helper functions (`copy_sites`, `copy_positions`, `alloc_offsets`) remain; only `from_voronoi` changes.

**File:** `src/graph/voronoi_graph.c3`

**Replace the `from_voronoi` function body:**

```c3
fn VoronoiGraph? from_voronoi(Allocator alloc, VoronoiDiagram* diagram)
{
    if (diagram.mesh.faces.len == 0) return cg::EMPTY_INPUT~;
    if (diagram.sites.len == 0) return cg::EMPTY_INPUT~;
    diagram.mesh.validate()!;

    // 1. Copy sites
    Vec3f[] sites = copy_sites(alloc, diagram)!;
    defer catch {
        free(sites);
    };

    // 2. Compute centroids (one per Voronoi cell = one per face)
    Vec3f[] centroids = geometry::face_centroids(alloc, &diagram.mesh)!;
    defer catch {
        free(sites);
        free(centroids);
    };

    // 3. Copy vertex positions (Voronoi vertices = mesh.positions)
    Vec3f[] vertices = copy_positions(alloc, &diagram.mesh)!;
    defer catch {
        free(sites);
        free(centroids);
        free(vertices);
    };

    usz cell_count = diagram.mesh.faces.len;

    // 4. Count total ring entries (one per half-edge in all faces)
    usz total_ring_entries = 0;
    for (usz f = 0; f < cell_count; f++) {
        int deg = diagram.mesh.face_degree((FaceIndex)(int)f);
        total_ring_entries += (usz)deg;
    }

    // 5. Allocate CSR arrays
    int[] cell_offsets = mem::alloc::new_array(alloc, int, (sz)(cell_count + 1));
    defer catch {
        free(sites);
        free(centroids);
        free(vertices);
        free(cell_offsets);
    };

    VertexIndex[] ring_indices = mem::alloc::new_array(alloc, VertexIndex, (sz)total_ring_entries);
    defer catch {
        free(sites);
        free(centroids);
        free(vertices);
        free(cell_offsets);
        free(ring_indices);
    };

    FaceIndex[] neighbor_indices = mem::alloc::new_array(alloc, FaceIndex, (sz)total_ring_entries);
    defer catch {
        free(sites);
        free(centroids);
        free(vertices);
        free(cell_offsets);
        free(ring_indices);
        free(neighbor_indices);
    };

    // 6. Build ring-to-face map: for each half-edge, which face does it belong to?
    //    (HalfEdge.face already stores this, so we can look it up directly.)

    // 7. Build CSR: walk each face's ring, collect vertices and neighbors
    cell_offsets[0] = 0;
    usz max_deg = 0;
    for (usz f = 0; f < cell_count; f++) {
        int deg = diagram.mesh.face_degree((FaceIndex)(int)f);
        if ((usz)deg > max_deg) max_deg = (usz)deg;
    }

    HeIndex[] he_scratch = mem::alloc::new_array(alloc, HeIndex, (sz)max_deg);
    defer free(he_scratch);

    for (usz f = 0; f < cell_count; f++) {
        FaceIndex face = (FaceIndex)(int)f;
        int deg = diagram.mesh.face_degree(face);
        _ = diagram.mesh.face_half_edges(face, he_scratch[..(usz)deg])!;
        usz offset = (usz)cell_offsets[f];

        for (int i = 0; i < deg; i++) {
            HeIndex he = he_scratch[i];
            ring_indices[offset + (usz)i] = diagram.mesh.from_vertex(he);

            HeIndex twin = diagram.mesh.twin(he);
            if (diagram.mesh.is_boundary(he)) {
                neighbor_indices[offset + (usz)i] = cg::INVALID_FACE;
            } else {
                neighbor_indices[offset + (usz)i] = diagram.mesh.face_of(twin);
            }
        }

        cell_offsets[f + 1] = cell_offsets[f] + deg;
    }

    VoronoiGraph g;
    g.vertices         = vertices;
    g.sites            = sites;
    g.centroids        = centroids;
    g.cell_offsets     = cell_offsets;
    g.ring_indices     = ring_indices;
    g.neighbor_indices = neighbor_indices;
    return g;
}
```

**Note:** The `defer catch` chains must be removed/commented once `from_voronoi` is fully structured, because the `RenderingData` pattern in the codebase uses a single `defer catch` per allocation with the manual free-if-fail approach. For the stub-to-real transition here, the `defer catch` pattern is correct for C3 0.8.0. If compilation fails due to re-declaration of `defer catch` scoping, fall back to the manual approach:

```c3
fn VoronoiGraph? from_voronoi(Allocator alloc, VoronoiDiagram* diagram)
{
    if (diagram.mesh.faces.len == 0) return cg::EMPTY_INPUT~;
    if (diagram.sites.len == 0) return cg::EMPTY_INPUT~;
    diagram.mesh.validate()!;

    Vec3f[] sites = copy_sites(alloc, diagram)!;
    Vec3f[] centroids = geometry::face_centroids(alloc, &diagram.mesh)!;
    Vec3f[] vertices = copy_positions(alloc, &diagram.mesh)!;

    usz cell_count = diagram.mesh.faces.len;

    usz total_ring_entries = 0;
    for (usz f = 0; f < cell_count; f++) {
        int deg = diagram.mesh.face_degree((FaceIndex)(int)f);
        total_ring_entries += (usz)deg;
    }

    int[] cell_offsets = mem::alloc::new_array(alloc, int, (sz)(cell_count + 1));
    VertexIndex[] ring_indices = mem::alloc::new_array(alloc, VertexIndex, (sz)total_ring_entries);
    FaceIndex[] neighbor_indices = mem::alloc::new_array(alloc, FaceIndex, (sz)total_ring_entries);

    usz max_deg = 0;
    for (usz f = 0; f < cell_count; f++) {
        int deg = diagram.mesh.face_degree((FaceIndex)(int)f);
        if ((usz)deg > max_deg) max_deg = (usz)deg;
    }

    HeIndex[] he_scratch = mem::alloc::new_array(alloc, HeIndex, (sz)max_deg);
    defer free(he_scratch);

    cell_offsets[0] = 0;
    for (usz f = 0; f < cell_count; f++) {
        FaceIndex face = (FaceIndex)(int)f;
        int deg = diagram.mesh.face_degree(face);
        _ = diagram.mesh.face_half_edges(face, he_scratch[..(usz)deg])!;
        usz offset = (usz)cell_offsets[f];

        for (int i = 0; i < deg; i++) {
            HeIndex he = he_scratch[i];
            ring_indices[offset + (usz)i] = diagram.mesh.from_vertex(he);

            if (diagram.mesh.is_boundary(he)) {
                neighbor_indices[offset + (usz)i] = cg::INVALID_FACE;
            } else {
                HeIndex twin = diagram.mesh.twin(he);
                neighbor_indices[offset + (usz)i] = diagram.mesh.face_of(twin);
            }
        }

        cell_offsets[f + 1] = cell_offsets[f] + deg;
    }

    VoronoiGraph g;
    g.vertices         = vertices;
    g.sites            = sites;
    g.centroids        = centroids;
    g.cell_offsets     = cell_offsets;
    g.ring_indices     = ring_indices;
    g.neighbor_indices = neighbor_indices;
    return g;
}
```

(The manual approach matches `from_delaunay` in `src/voronoi/voronoi.c3` more closely — no `defer catch` chains, just direct construction.)

**Also remove the helper functions from the stub that are no longer needed** — `copy_sites`, `copy_positions`, `alloc_offsets` can remain as internal helpers since they're still used by the real `from_voronoi`.

```bash
c3c build debug && c3c test
```

Expected: builds, all existing tests pass.

```bash
git add src/graph/voronoi_graph.c3
git commit -m "cg::graph: implement from_voronoi with CSR layout (M13)"
```

---

### Task 3: Implement `from_delaunay` + convenience constructors

**Objective:** Replace the stub bodies in `src/graph/delaunay_graph.c3` with real implementations.

**File:** `src/graph/delaunay_graph.c3`

**Replace the `from_delaunay` function body:**

```c3
fn DelaunayGraph? from_delaunay(Allocator alloc, HalfEdgeMesh* mesh, Vec3f[] circumcenters)
{
    if (mesh.faces.len == 0) return cg::EMPTY_INPUT~;
    if (circumcenters.len != mesh.faces.len) return cg::DUAL_VERTEX_COUNT_MISMATCH~;
    mesh.validate()!;

    // Validate all faces are triangular
    for (usz f = 0; f < mesh.faces.len; f++) {
        int deg = mesh.face_degree((FaceIndex)(int)f);
        if (deg != 3) return cg::NON_TRIANGLE_FACE~;
    }

    // Copy vertices
    Vec3f[] vertices = copy_vertex_positions(alloc, mesh)!;

    // Copy circumcenters
    Vec3f[] ccs = copy_circumcenters(alloc, circumcenters)!;

    // Allocate triangles array
    DelaunayTriangle[] triangles = mem::alloc::new_array(alloc, DelaunayTriangle, (sz)mesh.faces.len);

    // Scratch buffer for half-edge walk (degree=3, so 3 is always enough)
    HeIndex[3] he_scratch;

    for (usz f = 0; f < mesh.faces.len; f++) {
        FaceIndex face = (FaceIndex)(int)f;
        int deg = mesh.face_half_edges(face, he_scratch[..])!;
        _ = deg; // already validated deg == 3 above

        // Collect 3 origin vertices
        VertexIndex[3] verts;
        FaceIndex[3]   neigh;
        for (int i = 0; i < 3; i++) {
            HeIndex he = he_scratch[i];
            verts[i] = mesh.from_vertex(he);

            if (mesh.is_boundary(he)) {
                neigh[i] = cg::INVALID_FACE;
            } else {
                HeIndex twin = mesh.twin(he);
                neigh[i] = mesh.face_of(twin);
            }
        }

        triangles[f] = { .vertices = verts, .neighbors = neigh };
    }

    DelaunayGraph g;
    g.vertices      = vertices;
    g.circumcenters = ccs;
    g.triangles     = triangles;
    return g;
}
```

**Replace the `from_planar_delaunay` stub body:**

```c3
fn DelaunayGraph? from_planar_delaunay(Allocator alloc, HalfEdgeMesh* mesh)
{
    if (mesh.faces.len == 0) return cg::EMPTY_INPUT~;
    mesh.validate()!;

    Vec3f[] ccs = geometry::circumcenters_planar(alloc, mesh)!;
    defer free(ccs);

    return from_delaunay(alloc, mesh, ccs);
}
```

**Replace the `from_spherical_delaunay` stub body:**

```c3
fn DelaunayGraph? from_spherical_delaunay(Allocator alloc, HalfEdgeMesh* mesh, float radius)
{
    if (mesh.faces.len == 0) return cg::EMPTY_INPUT~;
    mesh.validate()!;

    Vec3f[] ccs = geometry::circumcenters_on_sphere(alloc, mesh, radius)!;
    defer free(ccs);

    return from_delaunay(alloc, mesh, ccs);
}
```

**Keep the `copy_vertex_positions` and `copy_circumcenters` helpers** (they are still used by `from_delaunay`).

```bash
c3c build debug && c3c test
```

Expected: builds, all existing tests pass.

```bash
git add src/graph/delaunay_graph.c3
git commit -m "cg::graph: implement from_delaunay and convenience constructors (M13)"
```

---

### Task 4: Write `test_voronoi_graph.c3` test suite

**Objective:** Replace the stub with full test suite covering the spec's test plan for VoronoiGraph.

**File:** `test/test_voronoi_graph.c3`

Replace entire file:

```c3
module test;
import cg;
import cg::voronoi;
import cg::graph;

fn void test_voronoi_graph_cell_count() @test
{
    VoronoiDiagram d = bounded_4_site_square();
    defer d.destroy();

    VoronoiGraph g = graph::from_voronoi(mem, &d)!!;
    defer g.destroy();

    assert(g.cell_count() == 4);
}

fn void test_voronoi_graph_cells_have_vertices() @test
{
    VoronoiDiagram d = bounded_4_site_square();
    defer d.destroy();

    VoronoiGraph g = graph::from_voronoi(mem, &d)!!;
    defer g.destroy();

    for (int c = 0; c < g.cell_count(); c++) {
        VoronoiCellView cell = g.cell((FaceIndex)c);
        assert(cell.ring.len > 0);
    }
}

fn void test_voronoi_graph_neighbor_symmetry() @test
{
    VoronoiDiagram d = bounded_4_site_square();
    defer d.destroy();

    VoronoiGraph g = graph::from_voronoi(mem, &d)!!;
    defer g.destroy();

    for (int a = 0; a < g.cell_count(); a++) {
        VoronoiCellView ca = g.cell((FaceIndex)a);
        for (usz i = 0; i < ca.neighbors.len; i++) {
            if (ca.neighbors[i] == cg::INVALID_FACE) continue;
            int b = (int)ca.neighbors[i];
            VoronoiCellView cb = g.cell((FaceIndex)b);
            bool found = false;
            for (usz j = 0; j < cb.neighbors.len; j++) {
                if ((int)cb.neighbors[j] == a) {
                    found = true;
                    break;
                }
            }
            assert(found);
        }
    }
}

fn void test_voronoi_graph_boundary_edges_have_invalid_face() @test
{
    VoronoiDiagram d = bounded_4_site_square();
    defer d.destroy();

    VoronoiGraph g = graph::from_voronoi(mem, &d)!!;
    defer g.destroy();

    bool saw_boundary = false;
    for (int c = 0; c < g.cell_count(); c++) {
        VoronoiCellView cell = g.cell((FaceIndex)c);
        for (usz i = 0; i < cell.neighbors.len; i++) {
            if (cell.neighbors[i] == cg::INVALID_FACE) {
                saw_boundary = true;
            }
        }
    }
    assert(saw_boundary);
}

fn void test_voronoi_graph_centroids() @test
{
    VoronoiDiagram d = bounded_4_site_square();
    defer d.destroy();

    VoronoiGraph g = graph::from_voronoi(mem, &d)!!;
    defer g.destroy();

    // Centroids should be finite (non-zero area cells produce valid centroids)
    for (int c = 0; c < g.cell_count(); c++) {
        VoronoiCellView cell = g.cell((FaceIndex)c);
        // centroid should be inside [0,1] x [0,1] with tolerance
        assert(cell.centroid.x >= -0.1f && cell.centroid.x <= 1.1f);
        assert(cell.centroid.y >= -0.1f && cell.centroid.y <= 1.1f);
    }
}

fn void test_voronoi_graph_cell_view_slices() @test
{
    VoronoiDiagram d = bounded_4_site_square();
    defer d.destroy();

    VoronoiGraph g = graph::from_voronoi(mem, &d)!!;
    defer g.destroy();

    for (int c = 0; c < g.cell_count(); c++) {
        VoronoiCellView cell = g.cell((FaceIndex)c);
        assert(cell.ring.len == cell.neighbors.len);
        assert(cell.index == (FaceIndex)c);
    }
}

fn void test_voronoi_graph_sites_preserved() @test
{
    VoronoiDiagram d = bounded_4_site_square();
    defer d.destroy();

    VoronoiGraph g = graph::from_voronoi(mem, &d)!!;
    defer g.destroy();

    for (int c = 0; c < g.cell_count(); c++) {
        VoronoiCellView cell = g.cell((FaceIndex)c);
        assert(cell.site.x == d.sites[c].x);
        assert(cell.site.y == d.sites[c].y);
        assert(cell.site.z == d.sites[c].z);
    }
}

fn void test_voronoi_graph_icosahedron_spherical() @test
{
    VoronoiDiagram d = voronoi::voronoi_on_sphere(mem, ICO_SPHERE[..], 1.0f)!!;
    defer d.destroy();

    VoronoiGraph g = graph::from_voronoi(mem, &d)!!;
    defer g.destroy();

    assert(g.cell_count() == 12);

    for (int c = 0; c < g.cell_count(); c++) {
        VoronoiCellView cell = g.cell((FaceIndex)c);
        for (usz i = 0; i < cell.neighbors.len; i++) {
            assert(cell.neighbors[i] != cg::INVALID_FACE);
        }
    }
}

fn void test_voronoi_graph_empty_input() @test
{
    VoronoiDiagram d;  // zero-initialized, empty
    if (catch err = graph::from_voronoi(mem, &d)) {
        assert(err == cg::EMPTY_INPUT);
        return;
    }
    unreachable();
}

// ── bounded 4-site square fixture ──

fn VoronoiDiagram bounded_4_site_square()
{
    Vec3f[4] s = { {0.25,0.25,0}, {0.75,0.25,0}, {0.75,0.75,0}, {0.25,0.75,0} };
    Vec3f[4] p = { {0,0,0}, {1,0,0}, {1,1,0}, {0,1,0} };
    return voronoi::in_polygon(mem, s[..], p[..])!!;
}
```

```bash
c3c build debug && c3c test
```

Expected: 10 new test functions pass.

```bash
git add test/test_voronoi_graph.c3
git commit -m "test: add VoronoiGraph test suite (M13)"
```

---

### Task 5: Write `test_delaunay_graph.c3` test suite

**Objective:** Replace the stub with full test suite covering the spec's test plan for DelaunayGraph.

**File:** `test/test_delaunay_graph.c3`

Replace entire file:

```c3
module test;
import cg;
import cg::delaunay;
import cg::geometry;
import cg::graph;
import cg::half_edge;
import std::math;

fn void test_delaunay_graph_triangle_count() @test
{
    HalfEdgeMesh mesh = delaunay_4_site_square();
    defer mesh.destroy();

    DelaunayGraph g = graph::from_planar_delaunay(mem, &mesh)!!;
    defer g.destroy();

    assert(g.triangle_count() == 2);
}

fn void test_delaunay_graph_circumcenters_match() @test
{
    HalfEdgeMesh mesh = delaunay_4_site_square();
    defer mesh.destroy();

    Vec3f[] expected = geometry::circumcenters_planar(mem, &mesh)!!;
    defer free(expected);

    DelaunayGraph g = graph::from_planar_delaunay(mem, &mesh)!!;
    defer g.destroy();

    for (int t = 0; t < g.triangle_count(); t++) {
        DelaunayTriangleView tv = g.triangle((FaceIndex)t);
        assert(approx_abs(tv.circumcenter.x - expected[t].x) < 0.001f);
        assert(approx_abs(tv.circumcenter.y - expected[t].y) < 0.001f);
        assert(approx_abs(tv.circumcenter.z - expected[t].z) < 0.001f);
    }
}

fn void test_delaunay_graph_neighbor_symmetry() @test
{
    HalfEdgeMesh mesh = delaunay_4_site_square();
    defer mesh.destroy();

    DelaunayGraph g = graph::from_planar_delaunay(mem, &mesh)!!;
    defer g.destroy();

    for (int a = 0; a < g.triangle_count(); a++) {
        DelaunayTriangleView ta = g.triangle((FaceIndex)a);
        for (int i = 0; i < 3; i++) {
            if (ta.neighbors[i] == cg::INVALID_FACE) continue;
            int b = (int)ta.neighbors[i];
            DelaunayTriangleView tb = g.triangle((FaceIndex)b);
            bool found = false;
            for (int j = 0; j < 3; j++) {
                if ((int)tb.neighbors[j] == a) {
                    found = true;
                    break;
                }
            }
            assert(found);
        }
    }
}

fn void test_delaunay_graph_vertices_preserved() @test
{
    HalfEdgeMesh mesh = delaunay_4_site_square();
    defer mesh.destroy();

    DelaunayGraph g = graph::from_planar_delaunay(mem, &mesh)!!;
    defer g.destroy();

    assert(g.vertices.len == mesh.positions.len);
    for (usz i = 0; i < g.vertices.len; i++) {
        assert(g.vertices[i].x == mesh.positions[i].x);
        assert(g.vertices[i].y == mesh.positions[i].y);
        assert(g.vertices[i].z == mesh.positions[i].z);
    }
}

fn void test_delaunay_graph_convenience_constructors_equal() @test
{
    HalfEdgeMesh mesh = delaunay_4_site_square();
    defer mesh.destroy();

    Vec3f[] ccs = geometry::circumcenters_planar(mem, &mesh)!!;
    defer free(ccs);

    DelaunayGraph g1 = graph::from_delaunay(mem, &mesh, ccs)!!;
    defer g1.destroy();

    DelaunayGraph g2 = graph::from_planar_delaunay(mem, &mesh)!!;
    defer g2.destroy();

    assert(g1.triangle_count() == g2.triangle_count());
    for (int t = 0; t < g1.triangle_count(); t++) {
        DelaunayTriangleView v1 = g1.triangle((FaceIndex)t);
        DelaunayTriangleView v2 = g2.triangle((FaceIndex)t);
        for (int i = 0; i < 3; i++) {
            assert(v1.vertices[i] == v2.vertices[i]);
            assert(v1.neighbors[i] == v2.neighbors[i]);
        }
    }
}

fn void test_delaunay_graph_icosahedron_spherical() @test
{
    HalfEdgeMesh mesh = delaunay::delaunay_on_sphere(mem, ICO_SPHERE[..], 1.0f)!!;
    defer mesh.destroy();

    DelaunayGraph g = graph::from_spherical_delaunay(mem, &mesh, 1.0f)!!;
    defer g.destroy();

    assert(g.triangle_count() == 20);

    for (int t = 0; t < g.triangle_count(); t++) {
        DelaunayTriangleView tv = g.triangle((FaceIndex)t);
        for (int i = 0; i < 3; i++) {
            assert(tv.neighbors[i] != cg::INVALID_FACE);
        }
    }
}

fn void test_delaunay_graph_spherical_convenience_equal() @test
{
    HalfEdgeMesh mesh = delaunay::delaunay_on_sphere(mem, ICO_SPHERE[..], 1.0f)!!;
    defer mesh.destroy();

    Vec3f[] ccs = geometry::circumcenters_on_sphere(mem, &mesh, 1.0f)!!;
    defer free(ccs);

    DelaunayGraph g1 = graph::from_delaunay(mem, &mesh, ccs)!!;
    defer g1.destroy();

    DelaunayGraph g2 = graph::from_spherical_delaunay(mem, &mesh, 1.0f)!!;
    defer g2.destroy();

    assert(g1.triangle_count() == g2.triangle_count());
    for (int t = 0; t < g1.triangle_count(); t++) {
        DelaunayTriangleView v1 = g1.triangle((FaceIndex)t);
        DelaunayTriangleView v2 = g2.triangle((FaceIndex)t);
        for (int i = 0; i < 3; i++) {
            assert(v1.vertices[i] == v2.vertices[i]);
            assert(v1.neighbors[i] == v2.neighbors[i]);
        }
    }
}

fn void test_delaunay_graph_icosahedron_vertices_on_sphere() @test
{
    HalfEdgeMesh mesh = delaunay::delaunay_on_sphere(mem, ICO_SPHERE[..], 1.0f)!!;
    defer mesh.destroy();

    DelaunayGraph g = graph::from_spherical_delaunay(mem, &mesh, 1.0f)!!;
    defer g.destroy();

    for (int t = 0; t < g.triangle_count(); t++) {
        DelaunayTriangleView tv = g.triangle((FaceIndex)t);
        for (int i = 0; i < 3; i++) {
            Vec3f v = g.vertices[(int)tv.vertices[i]];
            float r = (float)math::sqrt((double)(v.x*v.x + v.y*v.y + v.z*v.z));
            assert(approx_abs(r - 1.0f) < 0.001f);
        }
    }
}

fn void test_delaunay_graph_empty_input() @test
{
    HalfEdgeMesh mesh;
    if (catch err = graph::from_planar_delaunay(mem, &mesh)) {
        assert(err == cg::EMPTY_INPUT);
        return;
    }
    unreachable();
}

fn void test_delaunay_graph_non_triangle_face() @test
{
    Vec3f[4] pts = { {0,0,0}, {1,0,0}, {1,1,0}, {0,1,0} };
    uint[2] offsets = {0, 4};
    uint[4] indices = {0, 1, 2, 3};
    HalfEdgeMesh mesh = half_edge::from_polygons(mem, pts[..], offsets[..], indices[..])!!;
    defer mesh.destroy();

    Vec3f[1] ccs = { {0,0,0} };
    if (catch err = graph::from_delaunay(mem, &mesh, ccs[..])) {
        assert(err == cg::NON_TRIANGLE_FACE);
        return;
    }
    unreachable();
}

fn void test_delaunay_graph_vertex_count_mismatch() @test
{
    HalfEdgeMesh mesh = delaunay_4_site_square();
    defer mesh.destroy();

    Vec3f[1] wrong = { {0,0,0} };
    if (catch err = graph::from_delaunay(mem, &mesh, wrong[..])) {
        assert(err == cg::DUAL_VERTEX_COUNT_MISMATCH);
        return;
    }
    unreachable();
}

fn void test_delaunay_graph_triangle_view() @test
{
    HalfEdgeMesh mesh = delaunay_4_site_square();
    defer mesh.destroy();

    DelaunayGraph g = graph::from_planar_delaunay(mem, &mesh)!!;
    defer g.destroy();

    for (int t = 0; t < g.triangle_count(); t++) {
        DelaunayTriangleView tv = g.triangle((FaceIndex)t);
        assert(tv.index == (FaceIndex)t);
        // All 3 vertices should be valid indices
        for (int i = 0; i < 3; i++) {
            assert((int)tv.vertices[i] >= 0);
            assert((usz)tv.vertices[i] < g.vertices.len);
        }
    }
}

// ── fixture ──

fn HalfEdgeMesh delaunay_4_site_square()
{
    Vec3f[4] pts = { {0,0,0}, {1,0,0}, {1,1,0}, {0,1,0} };
    return delaunay::delaunay_2d(mem, pts[..])!!;
}
```

```bash
c3c build debug && c3c test
```

Expected: 12 new test functions pass.

```bash
git add test/test_delaunay_graph.c3
git commit -m "test: add DelaunayGraph test suite (M13)"
```

---

### Task 6: Final verification

```bash
c3c clean && c3c build release && c3c build debug && c3c test
```

Expected: all tests pass (including existing ~180+ tests and 17 new graph tests), no warnings.

```bash
git commit --allow-empty -m "cg::graph: final verification (M13)"
```

Update AGENTS.md milestone status.

---

## Potential issues

1. **`defer catch` in `from_voronoi`**: The RenderingData pattern uses single `defer catch free(...)` per allocation with no chain. Our `from_voronoi` uses multiple `defer catch` blocks in sequence. C3 0.8.0 supports this, but if the compiler complains, switch to the manual approach shown as the alternative in Task 2 (no defer catch — just allocate, build, and assign directly like `from_delaunay` in `src/voronoi/voronoi.c3`).

2. **`mem` allocator in test files**: The `mem` variable is provided by the C3 test runner as a built-in global allocator. No explicit declaration needed — all existing test files use it directly (pattern: `mem::alloc::new_array(mem, T, n)`).

3. **`approx_abs` helper**: Defined in `test_delaunay_sphere.c3` and accessible from `test_delaunay_graph.c3` since both share `module test;`. If the spherical test file is not yet merged, define `approx_abs` locally:

```c3
fn float approx_abs(float f) { if (f < 0) return -f; return f; }
```

4. **`import cg::voronoi` in umbrella's `cg::graph` block**: The `from_voronoi` declaration references `VoronoiDiagram` which is declared in the `cg::voronoi` umbrella block. Since umbrella blocks are forward-declared in order, and `cg::voronoi` appears before `cg::graph`, the type should be visible. If `c3c` rejects it, add `import cg::voronoi;` inside the `module cg::graph;` block in the umbrella.

5. **Order of umbrella declarations**: Place `module cg::graph;` at the end of `src/c3cg.c3i` (after `cg::utils::ppm`). This ensures all types it references (`VoronoiDiagram`, `HalfEdgeMesh`, `Vec3f`, `FaceIndex`, `VertexIndex`) are already declared.

6. **C3 0.8.0 type properties**: `T::size` (not `T.sizeof`). Array allocation uses `mem::alloc::new_array(alloc, T, (sz)n)`. Use `sz` not `isz`.

7. **Helper visibility**: `copy_sites`, `copy_positions`, `alloc_offsets` in `voronoi_graph.c3` and `copy_vertex_positions`, `copy_circumcenters` in `delaunay_graph.c3` are module-private helpers — they are NOT declared in the umbrella and do NOT conflict between the two files because C3 resolves `fn` visibility per-file.

8. **`face_half_edges` scratch buffer sizing**: Uses `he_scratch[..(usz)deg]` to pass a slice of exactly the right length. The scratch buffer `HeIndex[3]` for Delaunay (always degree 3) is stack-allocated. For Voronoi, the scratch buffer is heap-allocated to the maximum face degree.

9. **Boundary detection in `from_voronoi`**: `HalfEdgeMesh.is_boundary(he)` returns `true` when `half_edges[he].twin == INVALID_HE`. For Voronoi graphs built from bounded Voronoi diagrams, boundary edges exist on the domain boundary. These are correctly mapped to `INVALID_FACE` neighbors.

10. **`face_of(twin)` for neighbor lookup**: When the twin exists (not boundary), the twin half-edge's `face` field gives the neighboring face. This is the Delaunay face that the edge in the Voronoi cell corresponds to — i.e., the adjacent Voronoi cell.
