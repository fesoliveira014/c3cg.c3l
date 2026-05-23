module cg::primitives;
import cg;
import cg::subdivide;
import std::math;

fn HalfEdgeMesh? icosphere(Allocator alloc, int subdivisions, float radius)
{
    if (subdivisions < 0) return cg::DEGENERATE_INPUT~;
    if (radius <= 0.0f) return cg::DEGENERATE_INPUT~;

    HalfEdgeMesh mesh = icosahedron(alloc)!!;
    if (subdivisions == 0) {
        for (usz vi = 0; vi < mesh.positions.len; vi++) {
            Vec3f p = mesh.positions[vi];
            float len = (float)math::sqrt((double)(p.x * p.x + p.y * p.y + p.z * p.z));
            float scale = radius / len;
            mesh.positions[vi] = { p.x * scale, p.y * scale, p.z * scale };
        }
        return mesh;
    }

    HalfEdgeMesh current = mesh;
    for (int i = 0; i < subdivisions; i++) {
        HalfEdgeMesh next = subdivide::loop_subdivide(alloc, &current)!!;
        current.destroy();
        current = next;
    }

    for (usz vi = 0; vi < current.positions.len; vi++) {
        Vec3f p = current.positions[vi];
        float len = (float)math::sqrt((double)(p.x * p.x + p.y * p.y + p.z * p.z));
        float scale = radius / len;
        current.positions[vi] = { p.x * scale, p.y * scale, p.z * scale };
    }
    return current;
}
