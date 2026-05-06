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

const Vec3f[4] TET_NORMALS = {
    { 0, 0, 1 },
    { 0, 1, 0 },
    { 1, 0, 0 },
    { 1, 1, 1 },
};

const Vec2f[4] TET_UVS = {
    { 0, 0 },
    { 1, 0 },
    { 0, 1 },
    { 1, 1 },
};

fn void test_tetrahedron_with_attrs_copies_normals_and_uvs() @test
{
    HalfEdgeMesh mesh = half_edge::from_triangles_with_attrs(
        mem, TET_POSITIONS[..], TET_NORMALS[..], TET_UVS[..], TET_INDICES[..])!!;
    defer mesh.destroy();

    assert(mesh.positions.len == 4);
    assert(mesh.normals.len == 4);
    assert(mesh.uvs.len == 4);
    assert(mesh.half_edges.len == 12);

    assert(mesh.normals[0] == TET_NORMALS[0]);
    assert(mesh.normals[1] == TET_NORMALS[1]);
    assert(mesh.normals[2] == TET_NORMALS[2]);
    assert(mesh.normals[3] == TET_NORMALS[3]);
    assert(mesh.uvs[0] == TET_UVS[0]);
    assert(mesh.uvs[1] == TET_UVS[1]);
    assert(mesh.uvs[2] == TET_UVS[2]);
    assert(mesh.uvs[3] == TET_UVS[3]);
}

fn void test_empty_attrs_stay_empty() @test
{
    Vec3f[] no_normals = {};
    Vec2f[] no_uvs = {};
    HalfEdgeMesh mesh = half_edge::from_triangles_with_attrs(
        mem, TET_POSITIONS[..], no_normals, no_uvs, TET_INDICES[..])!!;
    defer mesh.destroy();

    assert(mesh.normals.len == 0);
    assert(mesh.uvs.len == 0);
    assert(mesh.half_edges.len == 12);
}

fn void test_normals_count_mismatch_fault() @test
{
    Vec3f[2] bad_normals = { { 0, 0, 1 }, { 0, 1, 0 } };
    if (catch err = half_edge::from_triangles_with_attrs(
        mem, TET_POSITIONS[..], bad_normals[..], {}, TET_INDICES[..])) {
        assert(err == cg::ATTRIBUTE_COUNT_MISMATCH);
        return;
    }
    unreachable();
}

fn void test_uvs_count_mismatch_fault() @test
{
    Vec2f[2] bad_uvs = { { 0, 0 }, { 1, 1 } };
    if (catch err = half_edge::from_triangles_with_attrs(
        mem, TET_POSITIONS[..], {}, bad_uvs[..], TET_INDICES[..])) {
        assert(err == cg::ATTRIBUTE_COUNT_MISMATCH);
        return;
    }
    unreachable();
}

fn void test_destroy_zeroes_mesh() @test
{
    HalfEdgeMesh mesh = half_edge::from_triangles(mem, TET_POSITIONS[..], TET_INDICES[..])!!;
    mesh.destroy();
    assert(mesh.positions.len == 0);
    assert(mesh.half_edges.len == 0);
    assert(mesh.faces.len == 0);
    assert(mesh.vertices.len == 0);
    assert(mesh.normals.len == 0);
    assert(mesh.uvs.len == 0);
}

fn void test_polygon_quad() @test
{
    Vec3f[4] positions = {
        { 0, 0, 0 },
        { 1, 0, 0 },
        { 1, 1, 0 },
        { 0, 1, 0 },
    };
    uint[2] offsets = { 0, 4 };
    uint[4] indices = { 0, 1, 2, 3 };
    HalfEdgeMesh mesh = half_edge::from_polygons(mem, positions[..], offsets[..], indices[..])!!;
    defer mesh.destroy();
    assert(mesh.faces.len == 1);
    assert(mesh.half_edges.len == 4);
    assert(mesh.positions.len == 4);
    assert(mesh.vertices.len == 4);
    assert(mesh.normals.len == 0);
    assert(mesh.uvs.len == 0);
    assert(mesh.half_edges[0].next == 1);
    assert(mesh.half_edges[1].next == 2);
    assert(mesh.half_edges[2].next == 3);
    assert(mesh.half_edges[3].next == 0);
    assert(mesh.half_edges[0].origin == 0);
    assert(mesh.half_edges[1].origin == 1);
    assert(mesh.half_edges[2].origin == 2);
    assert(mesh.half_edges[3].origin == 3);
    assert(mesh.faces[0].half_edge == 0);
}

fn void test_polygon_empty_offsets_fault() @test
{
    Vec3f[4] pos = { { 0, 0, 0 }, { 1, 0, 0 }, { 1, 1, 0 }, { 0, 1, 0 } };
    uint[] offsets = {};
    uint[4] indices = { 0, 1, 2, 3 };
    if (catch err = half_edge::from_polygons(mem, pos[..], offsets, indices[..])) {
        assert(err == cg::EMPTY_INPUT);
        return;
    }
    unreachable();
}

fn void test_polygon_bad_first_offset_fault() @test
{
    Vec3f[4] pos = { { 0, 0, 0 }, { 1, 0, 0 }, { 1, 1, 0 }, { 0, 1, 0 } };
    uint[2] offsets = { 1, 4 };
    uint[4] indices = { 0, 1, 2, 3 };
    if (catch err = half_edge::from_polygons(mem, pos[..], offsets[..], indices[..])) {
        assert(err == cg::INDEX_OUT_OF_RANGE);
        return;
    }
    unreachable();
}

fn void test_polygon_degenerate_face_fault() @test
{
    Vec3f[4] pos = { { 0, 0, 0 }, { 1, 0, 0 }, { 1, 1, 0 }, { 0, 1, 0 } };
    uint[2] offsets = { 0, 2 };
    uint[2] indices = { 0, 1 };
    if (catch err = half_edge::from_polygons(mem, pos[..], offsets[..], indices[..])) {
        assert(err == cg::DEGENERATE_INPUT);
        return;
    }
    unreachable();
}

fn void test_polygon_index_out_of_range_fault() @test
{
    Vec3f[3] pos = { { 0, 0, 0 }, { 1, 0, 0 }, { 0, 1, 0 } };
    uint[2] offsets = { 0, 3 };
    uint[3] indices = { 0, 1, 99 };
    if (catch err = half_edge::from_polygons(mem, pos[..], offsets[..], indices[..])) {
        assert(err == cg::INDEX_OUT_OF_RANGE);
        return;
    }
    unreachable();
}

fn void test_polygon_offset_mismatch_fault() @test
{
    Vec3f[4] pos = { { 0, 0, 0 }, { 1, 0, 0 }, { 1, 1, 0 }, { 0, 1, 0 } };
    uint[2] offsets = { 0, 5 };
    uint[4] indices = { 0, 1, 2, 3 };
    if (catch err = half_edge::from_polygons(mem, pos[..], offsets[..], indices[..])) {
        assert(err == cg::INDEX_OUT_OF_RANGE);
        return;
    }
    unreachable();
}
