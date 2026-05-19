module cg::dual;
import cg;
import cg::half_edge;

fn HalfEdgeMesh? dual_from_vertices(Allocator alloc, HalfEdgeMesh* mesh, Vec3f[] dual_vertex_positions)
{
    mesh.validate()!;

    if (dual_vertex_positions.len != mesh.faces.len) return cg::DUAL_VERTEX_COUNT_MISMATCH~;

    for (usz i = 0; i < mesh.half_edges.len; i++) {
        if (mesh.half_edges[i].twin == cg::INVALID_HE) return cg::DUAL_REQUIRES_CLOSED_MESH~;
    }

    // Stub: return so the boundary test can pass (not the count test yet).
    return cg::DUAL_VERTEX_COUNT_MISMATCH~;
}
