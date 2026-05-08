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

fn bool m4_edge_matches(HalfEdgeMesh* mesh, HeIndex he, VertexIndex from, VertexIndex to)
{
    return mesh.from_vertex(he) == from && mesh.to_vertex(he) == to;
}

fn bool m4_undirected_edge_matches(HalfEdgeMesh* mesh, HeIndex he, VertexIndex a, VertexIndex b)
{
    VertexIndex from = mesh.from_vertex(he);
    VertexIndex to = mesh.to_vertex(he);
    return (from == a && to == b) || (from == b && to == a);
}

fn void test_flip_ok_on_interior_diamond() @test
{
    HalfEdgeMesh mesh = half_edge::from_triangles(mem, DIAMOND_POSITIONS[..], DIAMOND_INDICES[..])!!;
    defer mesh.destroy();

    assert(mesh.twin(1) == 3);
    assert(mesh.is_flip_ok(1));
}

fn void test_flip_rewrites_diamond_diagonal() @test
{
    HalfEdgeMesh mesh = half_edge::from_triangles(mem, DIAMOND_POSITIONS[..], DIAMOND_INDICES[..])!!;
    defer mesh.destroy();

    mesh.flip(1)!!;

    mesh.validate()!!;
    assert(mesh.face_degree(0) == 3);
    assert(mesh.face_degree(1) == 3);
    assert(m4_undirected_edge_matches(&mesh, 1, 0, 3));
    assert(m4_undirected_edge_matches(&mesh, mesh.twin(1), 0, 3));
    assert(!m4_undirected_edge_matches(&mesh, 1, 1, 2));
}

fn void test_flip_refreshes_local_vertex_handles() @test
{
    HalfEdgeMesh mesh = half_edge::from_triangles(mem, DIAMOND_POSITIONS[..], DIAMOND_INDICES[..])!!;
    defer mesh.destroy();

    mesh.flip(1)!!;

    VertexIndex[4] vertices = { 0, 1, 2, 3 };
    foreach (v : vertices) {
        HeIndex handle = mesh.vertices[v].half_edge;
        assert(handle != cg::INVALID_HE);
        assert(mesh.half_edges[handle].origin == v);
    }
    mesh.validate()!!;
}
