module test;
import cg;
import cg::half_edge;
import cg::render;
import std::math;

const float RENDER_TEST_EPSILON = 0.001f;

fn bool float_approx(float a, float b)
{
    float diff = a - b;
    if (diff < 0.0f) diff = -diff;
    return diff <= RENDER_TEST_EPSILON;
}

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
    assert(data.normals.len == 0);
    assert(data.uvs.len == 0);

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
    assert(data.normals.len == 0);
    assert(data.uvs.len == 0);
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
    assert(data.normals.len == 0);
    assert(data.uvs.len == 0);
    assert(data.indices[0] == 0);
    assert(data.indices[1] == 1);
    assert(data.indices[2] == 2);
    assert(data.indices[3] == 0);
    assert(data.indices[4] == 2);
    assert(data.indices[5] == 3);
}

fn void test_render_source_normals_copy_when_present() @test
{
    HalfEdgeMesh mesh = half_edge::from_triangles_with_attrs(
        mem, TET_POSITIONS[..], TET_NORMALS[..], TET_UVS[..], TET_INDICES[..])!!;
    defer mesh.destroy();

    render::RenderOptions options = render::DEFAULT_RENDER_OPTIONS;
    options.normal_mode = (uint)render::RenderNormalMode.RENDER_NORMAL_SOURCE;

    RenderingData data = mesh.to_rendering_data(mem, options)!!;
    defer data.destroy();

    assert(data.normals.len == mesh.positions.len);
    assert(data.uvs.len == 0);
    assert(data.normals[0] == TET_NORMALS[0]);
    assert(data.normals[1] == TET_NORMALS[1]);
    assert(data.normals[2] == TET_NORMALS[2]);
    assert(data.normals[3] == TET_NORMALS[3]);
}

fn void test_render_source_normals_missing_faults() @test
{
    HalfEdgeMesh mesh = half_edge::from_triangles(mem, TET_POSITIONS[..], TET_INDICES[..])!!;
    defer mesh.destroy();

    render::RenderOptions options = render::DEFAULT_RENDER_OPTIONS;
    options.normal_mode = (uint)render::RenderNormalMode.RENDER_NORMAL_SOURCE;

    if (catch err = mesh.to_rendering_data(mem, options)) {
        assert(err == cg::MISSING_NORMALS);
        return;
    }
    unreachable();
}

fn void test_render_source_uvs_copy_when_present() @test
{
    HalfEdgeMesh mesh = half_edge::from_triangles_with_attrs(
        mem, TET_POSITIONS[..], TET_NORMALS[..], TET_UVS[..], TET_INDICES[..])!!;
    defer mesh.destroy();

    render::RenderOptions options = render::DEFAULT_RENDER_OPTIONS;
    options.uv_mode = (uint)render::RenderUvMode.RENDER_UV_SOURCE;

    RenderingData data = mesh.to_rendering_data(mem, options)!!;
    defer data.destroy();

    assert(data.uvs.len == mesh.positions.len);
    assert(data.normals.len == 0);
    assert(data.uvs[0] == TET_UVS[0]);
    assert(data.uvs[1] == TET_UVS[1]);
    assert(data.uvs[2] == TET_UVS[2]);
    assert(data.uvs[3] == TET_UVS[3]);
}

fn void test_render_source_uvs_missing_faults() @test
{
    HalfEdgeMesh mesh = half_edge::from_triangles(mem, TET_POSITIONS[..], TET_INDICES[..])!!;
    defer mesh.destroy();

    render::RenderOptions options = render::DEFAULT_RENDER_OPTIONS;
    options.uv_mode = (uint)render::RenderUvMode.RENDER_UV_SOURCE;

    if (catch err = mesh.to_rendering_data(mem, options)) {
        assert(err == cg::MISSING_UVS);
        return;
    }
    unreachable();
}

fn void test_render_source_or_zero_uvs_zero_fill_when_missing() @test
{
    HalfEdgeMesh mesh = half_edge::from_triangles(mem, TET_POSITIONS[..], TET_INDICES[..])!!;
    defer mesh.destroy();

    render::RenderOptions options = render::DEFAULT_RENDER_OPTIONS;
    options.uv_mode = (uint)render::RenderUvMode.RENDER_UV_SOURCE_OR_ZERO;

    RenderingData data = mesh.to_rendering_data(mem, options)!!;
    defer data.destroy();

    assert(data.uvs.len == mesh.positions.len);
    assert(data.normals.len == 0);
    for (usz i = 0; i < data.uvs.len; i++) {
        assert(data.uvs[i].x == 0);
        assert(data.uvs[i].y == 0);
    }
}

fn void test_render_invalid_options_reserved_bits_fault() @test
{
    HalfEdgeMesh mesh = half_edge::from_triangles(mem, TET_POSITIONS[..], TET_INDICES[..])!!;
    defer mesh.destroy();

    render::RenderOptions options = (render::RenderOptions)(1u << 5);

    if (catch err = mesh.to_rendering_data(mem, options)) {
        assert(err == cg::INVALID_RENDER_OPTIONS);
        return;
    }
    unreachable();
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

fn void test_render_non_planar_polygon_faults() @test
{
    Vec3f[4] positions = {
        { 0, 0, 0 },
        { 1, 0, 0 },
        { 1, 1, 0.1 },
        { 0, 1, 0 },
    };
    uint[2] offsets = { 0, 4 };
    uint[4] indices = { 0, 1, 2, 3 };
    HalfEdgeMesh mesh = half_edge::from_polygons(mem, positions[..], offsets[..], indices[..])!!;
    defer mesh.destroy();

    if (catch err = mesh.to_rendering_data(mem)) {
        assert(err == cg::NON_PLANAR_FACE);
        return;
    }
    unreachable();
}

fn void test_render_skip_planarity_allows_non_planar_polygon_fan() @test
{
    Vec3f[4] positions = {
        { 0, 0, 0 },
        { 1, 0, 0 },
        { 1, 1, 0.1 },
        { 0, 1, 0 },
    };
    uint[2] offsets = { 0, 4 };
    uint[4] indices = { 0, 1, 2, 3 };
    HalfEdgeMesh mesh = half_edge::from_polygons(mem, positions[..], offsets[..], indices[..])!!;
    defer mesh.destroy();

    render::RenderOptions options = render::DEFAULT_RENDER_OPTIONS;
    options.skip_planarity_check = true;

    RenderingData data = mesh.to_rendering_data(mem, options)!!;
    defer data.destroy();

    assert(data.vertices.len == 4);
    assert(data.indices.len == 6);
    assert(data.normals.len == 0);
    assert(data.uvs.len == 0);
}

fn void test_render_degenerate_polygon_plane_faults() @test
{
    Vec3f[4] positions = {
        { 0, 0, 0 },
        { 1, 0, 0 },
        { 2, 0, 0 },
        { 3, 0, 0 },
    };
    uint[2] offsets = { 0, 4 };
    uint[4] indices = { 0, 1, 2, 3 };
    HalfEdgeMesh mesh = half_edge::from_polygons(mem, positions[..], offsets[..], indices[..])!!;
    defer mesh.destroy();

    if (catch err = mesh.to_rendering_data(mem)) {
        assert(err == cg::DEGENERATE_INPUT);
        return;
    }
    unreachable();
}

fn void test_render_smooth_normals_are_unit_length_on_tetrahedron() @test
{
    HalfEdgeMesh mesh = half_edge::from_triangles(mem, TET_POSITIONS[..], TET_INDICES[..])!!;
    defer mesh.destroy();

    render::RenderOptions options = render::DEFAULT_RENDER_OPTIONS;
    options.normal_mode = (uint)render::RenderNormalMode.RENDER_NORMAL_SMOOTH;

    RenderingData data = mesh.to_rendering_data(mem, options)!!;
    defer data.destroy();

    assert(data.vertices.len == 4);
    assert(data.indices.len == 12);
    assert(data.normals.len == 4);
    assert(data.uvs.len == 0);

    for (usz i = 0; i < data.normals.len; i++) {
        float len = render::length3(data.normals[i]);
        assert(float_approx(len, 1.0f));
    }
}

fn void test_render_smooth_normals_are_angle_weighted() @test
{
    Vec3f[4] positions = {
        { 0, 0, 0 },
        { 2, 0, 0 },
        { 0, 1, 0 },
        { 0, 0, 1 },
    };
    uint[6] indices = { 0, 1, 2, 0, 3, 1 };
    HalfEdgeMesh mesh = half_edge::from_triangles(mem, positions[..], indices[..])!!;
    defer mesh.destroy();

    render::RenderOptions options = render::DEFAULT_RENDER_OPTIONS;
    options.normal_mode = (uint)render::RenderNormalMode.RENDER_NORMAL_SMOOTH;

    RenderingData data = mesh.to_rendering_data(mem, options)!!;
    defer data.destroy();

    assert(data.normals.len == 4);

    Vec3f n0 = data.normals[0];
    assert(float_approx(n0.x, 0.0f));
    assert(float_approx(n0.y, 0.7071067f));
    assert(float_approx(n0.z, 0.7071067f));
}
