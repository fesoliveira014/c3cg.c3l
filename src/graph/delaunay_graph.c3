module cg::graph;
import cg;
import cg::geometry;

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
    if (circumcenters.len != mesh.faces.len) return cg::DUAL_VERTEX_COUNT_MISMATCH~;
    mesh.validate()!;

    for (usz f = 0; f < mesh.faces.len; f++) {
        int deg = mesh.face_degree((FaceIndex)(int)f);
        if (deg != 3) return cg::NON_TRIANGLE_FACE~;
    }

    Vec3f[] vertices = copy_vertex_positions(alloc, mesh)!;
    Vec3f[] ccs = copy_circumcenters(alloc, circumcenters)!;
    DelaunayTriangle[] triangles = mem::alloc::new_array(alloc, DelaunayTriangle, (sz)mesh.faces.len);

    HeIndex[3] he_scratch;

    for (usz f = 0; f < mesh.faces.len; f++) {
        FaceIndex face = (FaceIndex)(int)f;
        mesh.face_half_edges(face, he_scratch[..])!;

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

fn DelaunayGraph? from_planar_delaunay(Allocator alloc, HalfEdgeMesh* mesh)
{
    if (mesh.faces.len == 0) return cg::EMPTY_INPUT~;
    mesh.validate()!;

    Vec3f[] ccs = geometry::circumcenters_planar(alloc, mesh)!;
    defer free(ccs);

    return from_delaunay(alloc, mesh, ccs);
}

fn DelaunayGraph? from_spherical_delaunay(Allocator alloc, HalfEdgeMesh* mesh, float radius)
{
    if (mesh.faces.len == 0) return cg::EMPTY_INPUT~;
    mesh.validate()!;

    Vec3f[] ccs = geometry::circumcenters_on_sphere(alloc, mesh, radius)!;
    defer free(ccs);

    return from_delaunay(alloc, mesh, ccs);
}

// ── internal helpers ──

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
