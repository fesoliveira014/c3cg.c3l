module test;
import cg;
import cg::half_edge;

fn void test_flip_rejects_invalid_half_edge() @test
{
    HalfEdgeMesh mesh = half_edge::from_triangles(mem, DIAMOND_POSITIONS[..], DIAMOND_INDICES[..])!!;
    defer mesh.destroy();

    assert(!mesh.is_flip_ok(cg::INVALID_HE));
    if (catch err = mesh.flip(cg::INVALID_HE)) {
        assert(err == cg::INVALID_HALF_EDGE_REFERENCE);
    } else {
        assert(false);
    }

    HeIndex out_of_range = (HeIndex)(int)mesh.half_edges.len;
    assert(!mesh.is_flip_ok(out_of_range));
    if (catch err = mesh.flip(out_of_range)) {
        assert(err == cg::INVALID_HALF_EDGE_REFERENCE);
    } else {
        assert(false);
    }
}

fn void test_flip_boundary_faults() @test
{
    HalfEdgeMesh mesh = half_edge::from_triangles(mem, TRI_POSITIONS[..], TRI_INDICES[..])!!;
    defer mesh.destroy();

    HeIndex boundary_he = 0;
    HeIndex old_next = mesh.half_edges[boundary_he].next;
    VertexIndex old_origin = mesh.half_edges[boundary_he].origin;
    FaceIndex old_face = mesh.half_edges[boundary_he].face;

    assert(!mesh.is_flip_ok(boundary_he));
    if (catch err = mesh.flip(boundary_he)) {
        assert(err == cg::BOUNDARY_HALF_EDGE);
    } else {
        assert(false);
    }

    assert(mesh.half_edges[boundary_he].next == old_next);
    assert(mesh.half_edges[boundary_he].origin == old_origin);
    assert(mesh.half_edges[boundary_he].face == old_face);
    mesh.validate()!!;
}
