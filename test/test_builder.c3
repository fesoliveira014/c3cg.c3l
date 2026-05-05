module test;
import cg;
import cg::half_edge;

const Vec3f[4] TET_POSITIONS = {
    { 0, 0, 0 },
    { 1, 0, 0 },
    { 0, 1, 0 },
    { 0, 0, 1 },
};

const uint[12] TET_INDICES = {
    0, 1, 2,
    0, 3, 1,
    0, 2, 3,
    1, 3, 2,
};

fn void test_empty_positions_fault() @test
{
    Vec3f[] positions = {};
    uint[3] indices = { 0, 1, 2 };
    if (catch err = half_edge::from_triangles(mem, positions, indices[..])) {
        assert(err == cg::EMPTY_INPUT);
        return;
    }
    unreachable();
}

fn void test_empty_indices_fault() @test
{
    uint[] indices = {};
    if (catch err = half_edge::from_triangles(mem, TET_POSITIONS[..], indices)) {
        assert(err == cg::EMPTY_INPUT);
        return;
    }
    unreachable();
}

fn void test_invalid_triangle_index_count_fault() @test
{
    uint[4] indices = { 0, 1, 2, 3 };
    if (catch err = half_edge::from_triangles(mem, TET_POSITIONS[..], indices[..])) {
        assert(err == cg::INVALID_TRIANGLE_INDEX_COUNT);
        return;
    }
    unreachable();
}

fn void test_index_out_of_range_fault() @test
{
    uint[3] indices = { 0, 1, 99 };
    if (catch err = half_edge::from_triangles(mem, TET_POSITIONS[..], indices[..])) {
        assert(err == cg::INDEX_OUT_OF_RANGE);
        return;
    }
    unreachable();
}

fn void test_tetrahedron_counts() @test
{
    HalfEdgeMesh mesh = half_edge::from_triangles(mem, TET_POSITIONS[..], TET_INDICES[..])!!;
    defer mesh.destroy();

    assert(mesh.positions.len == 4);
    assert(mesh.vertices.len == 4);
    assert(mesh.faces.len == 4);
    assert(mesh.half_edges.len == 12);
    assert(mesh.normals.len == 0);
    assert(mesh.uvs.len == 0);

    assert(mesh.positions[0] == TET_POSITIONS[0]);
    assert(mesh.positions[1] == TET_POSITIONS[1]);
    assert(mesh.positions[2] == TET_POSITIONS[2]);
    assert(mesh.positions[3] == TET_POSITIONS[3]);
}

fn void test_duplicate_directed_edge_fault() @test
{
    uint[6] indices = {
        0, 1, 2,
        0, 1, 3,
    };
    if (catch err = half_edge::from_triangles(mem, TET_POSITIONS[..], indices[..])) {
        assert(err == cg::DUPLICATE_HALF_EDGE);
        return;
    }
    unreachable();
}

fn void test_tetrahedron_twins_are_paired() @test
{
    HalfEdgeMesh mesh = half_edge::from_triangles(mem, TET_POSITIONS[..], TET_INDICES[..])!!;
    defer mesh.destroy();

    for (usz i = 0; i < mesh.half_edges.len; i++) {
        HeIndex twin = mesh.half_edges[i].twin;
        assert(twin != cg::INVALID_HE);
        assert(mesh.half_edges[twin].twin == (HeIndex)(int)i);
    }
}
