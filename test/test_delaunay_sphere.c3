module test;
import cg;
import cg::delaunay;
import std::math;

const float SPHERE_EPSILON = 0.0001f;

const float ICO_R  = 1.902113032590307f;

const Vec3f[12] ICO_SPHERE = {
    { -1.0f / ICO_R,  ICO_PHI / ICO_R,  0.0f },
    {  1.0f / ICO_R,  ICO_PHI / ICO_R,  0.0f },
    { -1.0f / ICO_R, -ICO_PHI / ICO_R,  0.0f },
    {  1.0f / ICO_R, -ICO_PHI / ICO_R,  0.0f },
    {  0.0f, -1.0f / ICO_R,  ICO_PHI / ICO_R },
    {  0.0f,  1.0f / ICO_R,  ICO_PHI / ICO_R },
    {  0.0f, -1.0f / ICO_R, -ICO_PHI / ICO_R },
    {  0.0f,  1.0f / ICO_R, -ICO_PHI / ICO_R },
    {  ICO_PHI / ICO_R,  0.0f, -1.0f / ICO_R },
    {  ICO_PHI / ICO_R,  0.0f,  1.0f / ICO_R },
    { -ICO_PHI / ICO_R,  0.0f, -1.0f / ICO_R },
    { -ICO_PHI / ICO_R,  0.0f,  1.0f / ICO_R },
};

fn float approx_abs(float f)
{
    if (f < 0) return -f;
    return f;
}

fn float vec3_len(Vec3f v)
{
    return (float)math::sqrt((double)(v.x * v.x + v.y * v.y + v.z * v.z));
}

fn void test_delaunay_sphere_icosahedron() @test
{
    HalfEdgeMesh mesh = delaunay::delaunay_on_sphere(mem, ICO_SPHERE[..], 1.0f)!!;
    defer mesh.destroy();

    mesh.validate()!!;

    assert(mesh.faces.len == 20);

    for (usz i = 0; i < mesh.half_edges.len; i++) {
        assert(!mesh.is_boundary((HeIndex)(int)i));
    }

    for (usz i = 0; i < mesh.positions.len; i++) {
        float len = vec3_len(mesh.positions[i]);
        assert(approx_abs(len - 1.0f) < SPHERE_EPSILON);
    }
}

fn void test_delaunay_sphere_radius_5() @test
{
    HalfEdgeMesh mesh = delaunay::delaunay_on_sphere(mem, ICO_SPHERE[..], 5.0f)!!;
    defer mesh.destroy();

    mesh.validate()!!;

    for (usz i = 0; i < mesh.positions.len; i++) {
        float len = vec3_len(mesh.positions[i]);
        assert(approx_abs(len - 5.0f) < SPHERE_EPSILON);
    }
}

fn void test_delaunay_sphere_empty_input() @test
{
    Vec3f[] pts = {};
    if (catch err = delaunay::delaunay_on_sphere(mem, pts, 1.0f)) {
        assert(err == cg::EMPTY_INPUT);
        return;
    }
    unreachable();
}

fn void test_delaunay_sphere_too_few_points() @test
{
    Vec3f[3] pts = { {1,0,0}, {0,1,0}, {0,0,1} };
    if (catch err = delaunay::delaunay_on_sphere(mem, pts[..], 1.0f)) {
        assert(err == cg::DEGENERATE_INPUT);
        return;
    }
    unreachable();
}

fn void test_delaunay_sphere_radius_zero() @test
{
    if (catch err = delaunay::delaunay_on_sphere(mem, ICO_SPHERE[..], 0.0f)) {
        assert(err == cg::DEGENERATE_INPUT);
        return;
    }
    unreachable();
}

fn void test_delaunay_sphere_radius_negative() @test
{
    if (catch err = delaunay::delaunay_on_sphere(mem, ICO_SPHERE[..], -1.0f)) {
        assert(err == cg::DEGENERATE_INPUT);
        return;
    }
    unreachable();
}
