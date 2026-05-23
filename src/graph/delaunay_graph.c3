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
    return from_delaunay(alloc, mesh, {});
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
