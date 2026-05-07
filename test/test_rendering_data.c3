module test;
import cg;
import cg::half_edge;
import cg::render;

fn void test_render_destroy_zeroes_data() @test
{
    RenderingData data;
    data.vertices = mem::alloc::new_array(mem, Vec3f, 1);
    data.indices = mem::alloc::new_array(mem, uint, 1);
    data.normals = mem::alloc::new_array(mem, Vec3f, 1);
    data.uvs = mem::alloc::new_array(mem, Vec2f, 1);

    data.destroy();

    assert(data.vertices.len == 0);
    assert(data.indices.len == 0);
    assert(data.normals.len == 0);
    assert(data.uvs.len == 0);
}

fn void test_render_triangle_preserves_vertices_and_indices() @test
{
    Vec3f[3] positions = {
        { 0, 0, 0 },
        { 1, 0, 0 },
        { 0, 1, 0 },
    };
    uint[3] indices = { 0, 1, 2 };
    HalfEdgeMesh mesh = half_edge::from_triangles(mem, positions[..], indices[..])!!;
    defer mesh.destroy();

    RenderingData data = mesh.to_rendering_data(mem)!!;
    defer data.destroy();

    assert(data.vertices.len == 3);
    assert(data.indices.len == 3);
    assert(data.normals.len == 3);
    assert(data.uvs.len == 3);

    assert(data.vertices[0] == positions[0]);
    assert(data.vertices[1] == positions[1]);
    assert(data.vertices[2] == positions[2]);

    assert(data.indices[0] == 0);
    assert(data.indices[1] == 1);
    assert(data.indices[2] == 2);
}

fn void test_render_tetrahedron_index_count() @test
{
    HalfEdgeMesh mesh = half_edge::from_triangles(mem, TET_POSITIONS[..], TET_INDICES[..])!!;
    defer mesh.destroy();

    RenderingData data = mesh.to_rendering_data(mem)!!;
    defer data.destroy();

    assert(data.vertices.len == 4);
    assert(data.indices.len == 12);
}

fn void test_render_quad_fan_triangulates() @test
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

    RenderingData data = mesh.to_rendering_data(mem)!!;
    defer data.destroy();

    assert(data.vertices.len == 4);
    assert(data.indices.len == 6);
    assert(data.indices[0] == 0);
    assert(data.indices[1] == 1);
    assert(data.indices[2] == 2);
    assert(data.indices[3] == 0);
    assert(data.indices[4] == 2);
    assert(data.indices[5] == 3);
}

fn void test_render_copies_normals_and_uvs() @test
{
    HalfEdgeMesh mesh = half_edge::from_triangles_with_attrs(
        mem, TET_POSITIONS[..], TET_NORMALS[..], TET_UVS[..], TET_INDICES[..])!!;
    defer mesh.destroy();

    RenderingData data = mesh.to_rendering_data(mem)!!;
    defer data.destroy();

    assert(data.normals.len == 4);
    assert(data.uvs.len == 4);
    assert(data.normals[0] == TET_NORMALS[0]);
    assert(data.normals[1] == TET_NORMALS[1]);
    assert(data.normals[2] == TET_NORMALS[2]);
    assert(data.normals[3] == TET_NORMALS[3]);
    assert(data.uvs[0] == TET_UVS[0]);
    assert(data.uvs[1] == TET_UVS[1]);
    assert(data.uvs[2] == TET_UVS[2]);
    assert(data.uvs[3] == TET_UVS[3]);
}

fn void test_render_missing_attrs_zero_filled() @test
{
    HalfEdgeMesh mesh = half_edge::from_triangles(mem, TET_POSITIONS[..], TET_INDICES[..])!!;
    defer mesh.destroy();

    RenderingData data = mesh.to_rendering_data(mem)!!;
    defer data.destroy();

    assert(data.normals.len == 4);
    assert(data.uvs.len == 4);
    for (usz i = 0; i < data.vertices.len; i++) {
        assert(data.normals[i][0] == 0);
        assert(data.normals[i][1] == 0);
        assert(data.normals[i][2] == 0);
        assert(data.uvs[i][0] == 0);
        assert(data.uvs[i][1] == 0);
    }
}

fn void test_render_invalid_mesh_returns_validation_fault() @test
{
    HalfEdgeMesh mesh = half_edge::from_triangles(mem, TET_POSITIONS[..], TET_INDICES[..])!!;
    defer mesh.destroy();

    mesh.half_edges[0].next = -2;

    if (catch err = mesh.to_rendering_data(mem)) {
        assert(err == cg::INVALID_HALF_EDGE_REFERENCE);
        return;
    }
    unreachable();
}

fn void test_render_output_owned_independently() @test
{
    Vec3f[3] positions = {
        { 0, 0, 0 },
        { 1, 0, 0 },
        { 0, 1, 0 },
    };
    uint[3] indices = { 0, 1, 2 };
    HalfEdgeMesh mesh = half_edge::from_triangles(mem, positions[..], indices[..])!!;
    defer mesh.destroy();

    RenderingData data = mesh.to_rendering_data(mem)!!;
    defer data.destroy();

    mesh.positions[0] = { 9, 9, 9 };

    assert(data.vertices[0] == positions[0]);
    assert(data.vertices[0] != mesh.positions[0]);
}
