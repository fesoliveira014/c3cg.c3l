module cg::voronoi;
import cg;

fn VoronoiDiagram? voronoi_on_sphere(Allocator alloc, Vec3f[] points, float radius)
{
    if (points.len == 0) return cg::EMPTY_INPUT~;
    if (points.len < 4 || radius <= 0) return cg::DEGENERATE_INPUT~;
    return cg::EMPTY_INPUT~;
}

fn VoronoiDiagram? voronoi_dual_sphere(Allocator alloc, HalfEdgeMesh* tri_mesh, Vec3f[] spatial_points)
{
    if (spatial_points.len == 0) return cg::EMPTY_INPUT~;
    return cg::EMPTY_INPUT~;
}
