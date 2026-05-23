module test;
import cg;
import cg::primitives;

fn float approx_abs_f(float f)
{
    if (f < 0) return -f;
    return f;
}

fn void test_tetrahedron() @test
{
    HalfEdgeMesh mesh = primitives::tetrahedron(mem)!!;
    defer mesh.destroy();

    mesh.validate()!!;

    assert(mesh.faces.len == 4);
    assert(mesh.positions.len == 4);

    for (usz i = 0; i < mesh.positions.len; i++) {
        Vec3f p = mesh.positions[i];
        float len2 = p.x * p.x + p.y * p.y + p.z * p.z;
        assert(approx_abs_f(len2 - 1.0f) < 0.0001f);
    }
}

fn void test_octahedron() @test
{
    HalfEdgeMesh mesh = primitives::octahedron(mem)!!;
    defer mesh.destroy();

    mesh.validate()!!;

    assert(mesh.faces.len == 8);
    assert(mesh.positions.len == 6);

    for (usz i = 0; i < mesh.positions.len; i++) {
        Vec3f p = mesh.positions[i];
        float len2 = p.x * p.x + p.y * p.y + p.z * p.z;
        assert(approx_abs_f(len2 - 1.0f) < 0.0001f);
    }
}

fn void test_icosahedron() @test
{
    HalfEdgeMesh mesh = primitives::icosahedron(mem)!!;
    defer mesh.destroy();

    mesh.validate()!!;

    assert(mesh.faces.len == 20);
    assert(mesh.positions.len == 12);

    for (usz i = 0; i < mesh.positions.len; i++) {
        Vec3f p = mesh.positions[i];
        float len2 = p.x * p.x + p.y * p.y + p.z * p.z;
        assert(approx_abs_f(len2 - 1.0f) < 0.0001f);
    }
}

fn void test_triangulated_cube() @test
{
    HalfEdgeMesh mesh = primitives::triangulated_cube(mem)!!;
    defer mesh.destroy();

    mesh.validate()!!;

    assert(mesh.faces.len == 12);
    assert(mesh.positions.len == 8);

    for (usz i = 0; i < mesh.positions.len; i++) {
        Vec3f p = mesh.positions[i];
        assert(approx_abs_f(p.x) <= 1.0001f);
        assert(approx_abs_f(p.y) <= 1.0001f);
        assert(approx_abs_f(p.z) <= 1.0001f);
        float len2 = p.x * p.x + p.y * p.y + p.z * p.z;
        assert(approx_abs_f(len2 - 3.0f) < 0.001f);
    }
}

fn void test_icosphere_subdiv_0() @test
{
    HalfEdgeMesh mesh = primitives::icosphere(mem, 0, 1.0f)!!;
    defer mesh.destroy();

    mesh.validate()!!;

    assert(mesh.faces.len == 20);
    assert(mesh.positions.len == 12);

    for (usz i = 0; i < mesh.positions.len; i++) {
        Vec3f p = mesh.positions[i];
        float len2 = p.x * p.x + p.y * p.y + p.z * p.z;
        assert(approx_abs_f(len2 - 1.0f) < 0.0001f);
    }
}

fn void test_icosphere_subdiv_2() @test
{
    HalfEdgeMesh mesh = primitives::icosphere(mem, 2, 5.0f)!!;
    defer mesh.destroy();

    mesh.validate()!!;

    assert(mesh.faces.len == 320);
    assert(mesh.positions.len == 162);

    for (usz i = 0; i < mesh.positions.len; i++) {
        Vec3f p = mesh.positions[i];
        float len2 = p.x * p.x + p.y * p.y + p.z * p.z;
        assert(approx_abs_f(len2 - 25.0f) < 0.01f);
    }
}

fn void test_icosphere_degenerate_subdiv() @test
{
    if (catch err = primitives::icosphere(mem, -1, 1.0f)) {
        assert(err == cg::DEGENERATE_INPUT);
        return;
    }
    unreachable();
}

fn void test_icosphere_degenerate_radius() @test
{
    if (catch err = primitives::icosphere(mem, 0, 0.0f)) {
        assert(err == cg::DEGENERATE_INPUT);
        return;
    }
    unreachable();
}
