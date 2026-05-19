module test;
import cg;
import cg::half_edge;
import cg::dual;

fn void test_dual_closed_mesh_required() @test
{
    // Single triangle has boundary edges (no twins across its 3 edges).
    Vec3f[3] tri_positions = {
        { 0, 0, 0 },
        { 1, 0, 0 },
        { 0, 1, 0 },
    };
    uint[3] tri_indices = { 0, 1, 2 };
    HalfEdgeMesh mesh = half_edge::from_triangles(mem, tri_positions[..], tri_indices[..])!!;
    defer mesh.destroy();

    // Any position array of correct length — doesn't matter, should fail before reading it.
    Vec3f[1] positions = { { 0, 0, 0 } };
    if (catch err = dual::dual_from_vertices(mem, &mesh, positions[..])) {
        assert(err == cg::DUAL_REQUIRES_CLOSED_MESH);
        return;
    }
    unreachable();
}
