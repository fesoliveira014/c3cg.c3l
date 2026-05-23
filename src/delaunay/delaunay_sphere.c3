module cg::delaunay;
import cg;
import cg::hull;
import std::math;

fn HalfEdgeMesh? delaunay_on_sphere(Allocator alloc, Vec3f[] points, float radius)
{
    if (points.len == 0) return cg::EMPTY_INPUT~;
    if (points.len < 4) return cg::DEGENERATE_INPUT~;
    if (radius <= 0) return cg::DEGENERATE_INPUT~;

    HalfEdgeMesh mesh = hull::hull_3d(alloc, points)!;

    for (usz i = 0; i < mesh.positions.len; i++) {
        float x = mesh.positions[i].x;
        float y = mesh.positions[i].y;
        float z = mesh.positions[i].z;
        float len2 = x * x + y * y + z * z;
        if (len2 <= 0.0f) {
            mesh.destroy();
            return cg::DEGENERATE_INPUT~;
        }
        float inv_len = 1.0f / (float)math::sqrt((double)len2);
        mesh.positions[i] = {
            x * inv_len * radius,
            y * inv_len * radius,
            z * inv_len * radius,
        };
    }

    return mesh;
}
