module test;
import cg;
import cg::half_edge;
import cg::dual;

const float ICO_PHI = 1.618033988749895f;
const float SPHERE_TOLERANCE = 0.001f;

const Vec3f[12] ICO_POSITIONS = {
    { -1,  ICO_PHI,  0 },
    {  1,  ICO_PHI,  0 },
    { -1, -ICO_PHI,  0 },
    {  1, -ICO_PHI,  0 },
    {  0, -1,  ICO_PHI },
    {  0,  1,  ICO_PHI },
    {  0, -1, -ICO_PHI },
    {  0,  1, -ICO_PHI },
    {  ICO_PHI,  0, -1 },
    {  ICO_PHI,  0,  1 },
    { -ICO_PHI,  0, -1 },
    { -ICO_PHI,  0,  1 },
};

const uint[60] ICO_INDICES = {
    0, 11, 5,
    0, 5, 1,
    0, 1, 7,
    0, 7, 10,
    0, 10, 11,
    1, 5, 9,
    5, 11, 4,
    11, 10, 2,
    10, 7, 6,
    7, 1, 8,
    3, 9, 4,
    3, 4, 2,
    3, 2, 6,
    3, 6, 8,
    3, 8, 9,
    4, 9, 5,
    2, 4, 11,
    6, 2, 10,
    8, 6, 7,
    9, 8, 1,
};

fn void test_dual_closed_mesh_required() @test
{
    Vec3f[3] tri_positions = {
        { 0, 0, 0 },
        { 1, 0, 0 },
        { 0, 1, 0 },
    };
    uint[3] tri_indices = { 0, 1, 2 };
    HalfEdgeMesh mesh = half_edge::from_triangles(mem, tri_positions[..], tri_indices[..])!!;
    defer mesh.destroy();

    Vec3f[1] positions = { { 0, 0, 0 } };
    if (catch err = dual::dual_from_vertices(mem, &mesh, positions[..])) {
        assert(err == cg::DUAL_REQUIRES_CLOSED_MESH);
        return;
    }
    unreachable();
}

fn void test_dual_vertex_count_mismatch() @test
{
    Vec3f[4] tet_positions = {
        { 0, 0, 0 },
        { 1, 0, 0 },
        { 0, 1, 0 },
        { 0, 0, 1 },
    };
    uint[12] tet_indices = {
        0, 1, 2,
        0, 3, 1,
        0, 2, 3,
        1, 3, 2,
    };
    HalfEdgeMesh mesh = half_edge::from_triangles(mem, tet_positions[..], tet_indices[..])!!;
    defer mesh.destroy();

    Vec3f[3] wrong_positions = {
        { 0, 0, 0 },
        { 1, 0, 0 },
        { 0, 1, 0 },
    };
    if (catch err = dual::dual_from_vertices(mem, &mesh, wrong_positions[..])) {
        assert(err == cg::DUAL_VERTEX_COUNT_MISMATCH);
        return;
    }
    unreachable();
}

fn void test_dual_tetrahedron_counts() @test
{
    Vec3f[4] tet_positions = {
        { 0, 0, 0 },
        { 1, 0, 0 },
        { 0, 1, 0 },
        { 0, 0, 1 },
    };
    uint[12] tet_indices = {
        0, 1, 2,
        0, 3, 1,
        0, 2, 3,
        1, 3, 2,
    };
    HalfEdgeMesh mesh = half_edge::from_triangles(mem, tet_positions[..], tet_indices[..])!!;
    defer mesh.destroy();

    Vec3f[4] positions = {
        { 10, 10, 10 },
        { 20, 20, 20 },
        { 30, 30, 30 },
        { 40, 40, 40 },
    };
    HalfEdgeMesh dual_mesh = dual::dual_from_vertices(mem, &mesh, positions[..])!!;
    defer dual_mesh.destroy();

    assert(dual_mesh.vertices.len == 4);
    assert(dual_mesh.faces.len == 4);
    assert(dual_mesh.half_edges.len == 12);
}

fn void test_dual_tetrahedron_closed() @test
{
    Vec3f[4] tet_positions = {
        { 0, 0, 0 },
        { 1, 0, 0 },
        { 0, 1, 0 },
        { 0, 0, 1 },
    };
    uint[12] tet_indices = {
        0, 1, 2,
        0, 3, 1,
        0, 2, 3,
        1, 3, 2,
    };
    HalfEdgeMesh mesh = half_edge::from_triangles(mem, tet_positions[..], tet_indices[..])!!;
    defer mesh.destroy();

    Vec3f[4] positions = {
        { 0, 0, 0 },
        { 0, 0, 0 },
        { 0, 0, 0 },
        { 0, 0, 0 },
    };
    HalfEdgeMesh dual_mesh = dual::dual_from_vertices(mem, &mesh, positions[..])!!;
    defer dual_mesh.destroy();

    for (usz i = 0; i < dual_mesh.half_edges.len; i++) {
        assert(dual_mesh.half_edges[i].twin != cg::INVALID_HE);
    }
}

fn void test_dual_tetrahedron_validates() @test
{
    Vec3f[4] tet_positions = {
        { 0, 0, 0 },
        { 1, 0, 0 },
        { 0, 1, 0 },
        { 0, 0, 1 },
    };
    uint[12] tet_indices = {
        0, 1, 2,
        0, 3, 1,
        0, 2, 3,
        1, 3, 2,
    };
    HalfEdgeMesh mesh = half_edge::from_triangles(mem, tet_positions[..], tet_indices[..])!!;
    defer mesh.destroy();

    Vec3f[4] positions = {
        { 0, 0, 0 },
        { 0, 0, 0 },
        { 0, 0, 0 },
        { 0, 0, 0 },
    };
    HalfEdgeMesh dual_mesh = dual::dual_from_vertices(mem, &mesh, positions[..])!!;
    defer dual_mesh.destroy();

    dual_mesh.validate()!!;
}

fn void test_dual_double_dual_roundtrip() @test
{
    Vec3f[4] tet_positions = {
        { 0, 0, 0 },
        { 1, 0, 0 },
        { 0, 1, 0 },
        { 0, 0, 1 },
    };
    uint[12] tet_indices = {
        0, 1, 2,
        0, 3, 1,
        0, 2, 3,
        1, 3, 2,
    };
    HalfEdgeMesh mesh = half_edge::from_triangles(mem, tet_positions[..], tet_indices[..])!!;
    defer mesh.destroy();

    Vec3f[4] positions_a = {
        { 10, 10, 10 },
        { 20, 20, 20 },
        { 30, 30, 30 },
        { 40, 40, 40 },
    };
    HalfEdgeMesh dual1 = dual::dual_from_vertices(mem, &mesh, positions_a[..])!!;
    defer dual1.destroy();

    Vec3f[4] positions_b = {
        { 1, 0, 0 },
        { 0, 1, 0 },
        { 0, 0, 1 },
        { 0, 0, 0 },
    };
    HalfEdgeMesh dual2 = dual::dual_from_vertices(mem, &dual1, positions_b[..])!!;
    defer dual2.destroy();

    assert(dual2.vertices.len == mesh.vertices.len);
    assert(dual2.faces.len == mesh.faces.len);
    assert(dual2.half_edges.len == mesh.half_edges.len);
    dual2.validate()!!;
}

fn void test_dual_icosahedron_to_dodecahedron() @test
{
    HalfEdgeMesh ico = half_edge::from_triangles(mem, ICO_POSITIONS[..], ICO_INDICES[..])!!;
    defer ico.destroy();

    Vec3f[20] positions;
    for (int i = 0; i < 20; i++) {
        positions[i] = { (float)i, (float)i, (float)i };
    }

    HalfEdgeMesh dual_mesh = dual::dual_from_vertices(mem, &ico, positions[..])!!;
    defer dual_mesh.destroy();

    assert(dual_mesh.vertices.len == 20);
    assert(dual_mesh.faces.len == 12);
    assert(dual_mesh.half_edges.len == 60);

    for (usz f = 0; f < dual_mesh.faces.len; f++) {
        int deg = dual_mesh.face_degree((FaceIndex)(int)f);
        assert(deg == 5);
    }

    dual_mesh.validate()!!;
}

fn void test_dual_icosahedron_double_dual() @test
{
    HalfEdgeMesh ico = half_edge::from_triangles(mem, ICO_POSITIONS[..], ICO_INDICES[..])!!;
    defer ico.destroy();

    Vec3f[20] positions_a;
    for (int i = 0; i < 20; i++) {
        positions_a[i] = { (float)i, (float)i, (float)i };
    }
    HalfEdgeMesh dual1 = dual::dual_from_vertices(mem, &ico, positions_a[..])!!;
    defer dual1.destroy();

    Vec3f[12] positions_b;
    for (int i = 0; i < 12; i++) {
        positions_b[i] = { 0, 0, 0 };
    }
    HalfEdgeMesh dual2 = dual::dual_from_vertices(mem, &dual1, positions_b[..])!!;
    defer dual2.destroy();

    assert(dual2.vertices.len == 12);
    assert(dual2.faces.len == 20);
    assert(dual2.half_edges.len == 60);
    dual2.validate()!!;
}

fn void test_dual_from_vertices_raw() @test
{
    Vec3f[4] tet_positions = {
        { 0, 0, 0 },
        { 1, 0, 0 },
        { 0, 1, 0 },
        { 0, 0, 1 },
    };
    uint[12] tet_indices = {
        0, 1, 2,
        0, 3, 1,
        0, 2, 3,
        1, 3, 2,
    };
    HalfEdgeMesh mesh = half_edge::from_triangles(mem, tet_positions[..], tet_indices[..])!!;
    defer mesh.destroy();

    Vec3f[4] custom = {
        { 100, 200, 300 },
        { 400, 500, 600 },
        { 700, 800, 900 },
        { 111, 222, 333 },
    };
    HalfEdgeMesh dual_mesh = dual::dual_from_vertices(mem, &mesh, custom[..])!!;
    defer dual_mesh.destroy();

    assert(dual_mesh.positions.len == 4);
    assert(dual_mesh.positions[0].x == 100);
    assert(dual_mesh.positions[3].z == 333);
    dual_mesh.validate()!!;
}

fn void test_dual_mode_circumcenter() @test
{
    Vec3f[4] tet_positions = {
        { 0, 0, 0 },
        { 1, 0, 0 },
        { 0, 1, 0 },
        { 0, 0, 1 },
    };
    uint[12] tet_indices = {
        0, 1, 2,
        0, 3, 1,
        0, 2, 3,
        1, 3, 2,
    };
    HalfEdgeMesh mesh = half_edge::from_triangles(mem, tet_positions[..], tet_indices[..])!!;
    defer mesh.destroy();

    HalfEdgeMesh dual_mesh = dual::dual(mem, &mesh, dual::DualMode.CIRCUMCENTER)!!;
    defer dual_mesh.destroy();

    assert(dual_mesh.vertices.len == 4);
    assert(dual_mesh.faces.len == 4);
    dual_mesh.validate()!!;
}

fn void test_dual_mode_centroid() @test
{
    Vec3f[4] tet_positions = {
        { 0, 0, 0 },
        { 1, 0, 0 },
        { 0, 1, 0 },
        { 0, 0, 1 },
    };
    uint[12] tet_indices = {
        0, 1, 2,
        0, 3, 1,
        0, 2, 3,
        1, 3, 2,
    };
    HalfEdgeMesh mesh = half_edge::from_triangles(mem, tet_positions[..], tet_indices[..])!!;
    defer mesh.destroy();

    HalfEdgeMesh dual_mesh = dual::dual(mem, &mesh, dual::DualMode.CENTROID)!!;
    defer dual_mesh.destroy();

    assert(dual_mesh.vertices.len == 4);
    dual_mesh.validate()!!;
}

fn void test_dual_mode_spherical() @test
{
    HalfEdgeMesh ico = half_edge::from_triangles(mem, ICO_POSITIONS[..], ICO_INDICES[..])!!;
    defer ico.destroy();

    HalfEdgeMesh dual_mesh = dual::dual(mem, &ico, dual::DualMode.SPHERICAL_CIRCUMCENTER, 1.0f)!!;
    defer dual_mesh.destroy();

    assert(dual_mesh.vertices.len == 20);
    assert(dual_mesh.faces.len == 12);
    dual_mesh.validate()!!;

    for (usz i = 0; i < dual_mesh.positions.len; i++) {
        float x = dual_mesh.positions[i].x;
        float y = dual_mesh.positions[i].y;
        float z = dual_mesh.positions[i].z;
        float len2 = x * x + y * y + z * z;
        float diff = len2 - 1.0f;
        if (diff < 0) diff = -diff;
        assert(diff < SPHERE_TOLERANCE);
    }
}

fn void test_dual_spherical_radius_zero_faults() @test
{
    Vec3f[4] tet_positions = {
        { 0, 0, 0 },
        { 1, 0, 0 },
        { 0, 1, 0 },
        { 0, 0, 1 },
    };
    uint[12] tet_indices = {
        0, 1, 2,
        0, 3, 1,
        0, 2, 3,
        1, 3, 2,
    };
    HalfEdgeMesh mesh = half_edge::from_triangles(mem, tet_positions[..], tet_indices[..])!!;
    defer mesh.destroy();

    if (catch err = dual::dual(mem, &mesh, dual::DualMode.SPHERICAL_CIRCUMCENTER, 0.0f)) {
        assert(err == cg::DEGENERATE_INPUT);
        return;
    }
    unreachable();
}
