module cg::hull;
import cg;

fn HalfEdgeMesh? hull_3d(Allocator alloc, Vec3f[] positions)
{
    if (positions.len == 0) return cg::EMPTY_INPUT~;
    return cg::DEGENERATE_INPUT~;
}
