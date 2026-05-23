module cg::subdivide;
import cg;
import cg::half_edge;

fn HalfEdgeMesh? loop_subdivide(Allocator alloc, HalfEdgeMesh* mesh)
{
    for (usz i = 0; i < mesh.faces.len; i++) {
        if (mesh.face_degree((FaceIndex)(int)i) != 3) return cg::NON_TRIANGLE_FACE~;
    }
    for (usz i = 0; i < mesh.half_edges.len; i++) {
        if (mesh.is_boundary((HeIndex)(int)i)) return cg::BOUNDARY_HALF_EDGE~;
    }

    return cg::EMPTY_INPUT~;
}
