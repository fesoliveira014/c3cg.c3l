module cg::delaunay;
import cg;
import cg::hull;

fn HalfEdgeMesh? delaunay_on_sphere(Allocator alloc, Vec3f[] points, float radius)
{
    return hull::hull_3d(alloc, points);
}
