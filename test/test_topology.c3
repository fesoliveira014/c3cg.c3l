module test;
import cg;
import cg::half_edge;

const Vec3f[3] TRI_POSITIONS = {
    { 0, 0, 0 },
    { 1, 0, 0 },
    { 0, 1, 0 },
};

const uint[3] TRI_INDICES = { 0, 1, 2 };

const Vec3f[4] DIAMOND_POSITIONS = {
    { 0, 0, 0 },
    { 1, 0, 0 },
    { 1, 1, 0 },
    { 0, 1, 0 },
};

const uint[6] DIAMOND_INDICES = { 0, 1, 2, 2, 1, 3 };

fn void test_twin_roundtrip() @test
{
    HalfEdgeMesh mesh = half_edge::from_triangles(mem, TET_POSITIONS[..], TET_INDICES[..])!!;
    defer mesh.destroy();

    for (usz i = 0; i < mesh.half_edges.len; i++) {
        HeIndex he = (HeIndex)(int)i;
        HeIndex twin = mesh.twin(he);
        assert(twin != cg::INVALID_HE);
        assert(mesh.twin(twin) == he);
    }
}

fn void test_next_face_cycle() @test
{
    HalfEdgeMesh mesh = half_edge::from_triangles(mem, TRI_POSITIONS[..], TRI_INDICES[..])!!;
    defer mesh.destroy();

    HeIndex start = mesh.faces[0].half_edge;
    HeIndex he = start;
    int count = 0;
    while (true) {
        he = mesh.next(he);
        count++;
        if (he == start) break;
        if (count > 10) {
            assert(false);
        }
    }
    assert(count == 3);
}

fn void test_prev_roundtrip() @test
{
    HalfEdgeMesh mesh = half_edge::from_triangles(mem, TRI_POSITIONS[..], TRI_INDICES[..])!!;
    defer mesh.destroy();

    for (usz i = 0; i < mesh.half_edges.len; i++) {
        HeIndex he = (HeIndex)(int)i;
        HeIndex prev_he = mesh.prev(he);
        assert(mesh.next(prev_he) == he);
    }
}

fn void test_from_to_face_boundary_degree() @test
{
    HalfEdgeMesh mesh = half_edge::from_triangles(mem, TRI_POSITIONS[..], TRI_INDICES[..])!!;
    defer mesh.destroy();

    HeIndex he = mesh.faces[0].half_edge;
    assert(mesh.from_vertex(he) == 0);
    assert(mesh.to_vertex(he) == 1);
    assert(mesh.face_of(he) == 0);
    assert(mesh.is_boundary(he));
    assert(mesh.face_degree(0) == 3);
}

fn void test_single_triangle_boundary_edges() @test
{
    HalfEdgeMesh mesh = half_edge::from_triangles(mem, TRI_POSITIONS[..], TRI_INDICES[..])!!;
    defer mesh.destroy();

    for (usz i = 0; i < mesh.half_edges.len; i++) {
        assert(mesh.is_boundary((HeIndex)(int)i));
    }
}

fn void test_diamond_topology() @test
{
    HalfEdgeMesh mesh = half_edge::from_triangles(mem, DIAMOND_POSITIONS[..], DIAMOND_INDICES[..])!!;
    defer mesh.destroy();

    assert(mesh.faces.len == 2);
    assert(mesh.half_edges.len == 6);

    assert(mesh.twin(1) == 3);
    assert(mesh.twin(3) == 1);

    assert(mesh.is_boundary(0));
    assert(mesh.is_boundary(2));
    assert(mesh.is_boundary(4));
    assert(mesh.is_boundary(5));

    assert(!mesh.is_boundary(1));
    assert(!mesh.is_boundary(3));

    assert(mesh.face_of(0) == 0);
    assert(mesh.face_of(1) == 0);
    assert(mesh.face_of(2) == 0);
    assert(mesh.face_of(3) == 1);
    assert(mesh.face_of(4) == 1);
    assert(mesh.face_of(5) == 1);

    assert(mesh.from_vertex(0) == 0);
    assert(mesh.to_vertex(0) == 1);
    assert(mesh.from_vertex(1) == 1);
    assert(mesh.to_vertex(1) == 2);
    assert(mesh.from_vertex(2) == 2);
    assert(mesh.to_vertex(2) == 0);

    assert(mesh.next(0) == 1);
    assert(mesh.next(1) == 2);
    assert(mesh.next(2) == 0);

    assert(mesh.prev(0) == 2);
    assert(mesh.prev(1) == 0);
    assert(mesh.prev(2) == 1);

    assert(mesh.face_degree(0) == 3);
    assert(mesh.face_degree(1) == 3);
}

fn void test_tetrahedron_no_boundary_edges() @test
{
    HalfEdgeMesh mesh = half_edge::from_triangles(mem, TET_POSITIONS[..], TET_INDICES[..])!!;
    defer mesh.destroy();

    for (usz i = 0; i < mesh.half_edges.len; i++) {
        assert(!mesh.is_boundary((HeIndex)(int)i));
    }
}
