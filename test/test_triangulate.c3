module test;
import cg;
import cg::triangulate;

fn void test_triangulate_insufficient_vertices() @test
{
    Vec3f[2] two_pts = { {0,0,0}, {1,0,0} };
    if (catch err = triangulate::triangulate_polygon(mem, two_pts[..])) {
        assert(err == cg::INSUFFICIENT_VERTICES);
        return;
    }
    unreachable();
}

fn void test_triangulate_duplicate_vertices() @test
{
    Vec3f[3] dup = { {0,0,0}, {1,0,0}, {0,0,0} };
    if (catch err = triangulate::triangulate_polygon(mem, dup[..])) {
        assert(err == cg::NON_SIMPLE_POLYGON);
        return;
    }
    unreachable();
}

fn void test_triangulate_triangle() @test
{
    Vec3f[3] tri = { {0,0,0}, {1,0,0}, {0,1,0} };
    HalfEdgeMesh mesh = triangulate::triangulate_polygon(mem, tri[..])!!;
    defer mesh.destroy();
    assert(mesh.faces.len == 1);
    mesh.validate()!!;
}

fn void test_triangulate_square() @test
{
    Vec3f[4] square = { {0,0,0}, {1,0,0}, {1,1,0}, {0,1,0} };
    HalfEdgeMesh mesh = triangulate::triangulate_polygon(mem, square[..])!!;
    defer mesh.destroy();
    assert(mesh.faces.len == 2);
    mesh.validate()!!;
}

fn void test_triangulate_pentagon() @test
{
    Vec3f[5] pent = { {0,1,0}, {1,0.3f,0}, {0.6f,-1,0}, {-0.6f,-1,0}, {-1,0.3f,0} };
    HalfEdgeMesh mesh = triangulate::triangulate_polygon(mem, pent[..])!!;
    defer mesh.destroy();
    assert(mesh.faces.len == 3);
    mesh.validate()!!;
}

fn void test_triangulate_trapezoid() @test
{
    Vec3f[4] trapezoid = {
        { -2, 0, 0 },
        { 2, 0, 0 },
        { 1, 2, 0 },
        { -1, 2, 0 },
    };
    HalfEdgeMesh mesh = triangulate::triangulate_polygon(mem, trapezoid[..])!!;
    defer mesh.destroy();
    assert(mesh.faces.len == 2);
    mesh.validate()!!;
}

fn void test_triangulate_l_shaped() @test
{
    Vec3f[6] lshape = { {0,0,0}, {2,0,0}, {2,1,0}, {1,1,0}, {1,2,0}, {0,2,0} };
    HalfEdgeMesh mesh = triangulate::triangulate_polygon(mem, lshape[..])!!;
    defer mesh.destroy();
    assert(mesh.faces.len == 4);
    mesh.validate()!!;
}

fn void test_triangulate_star_shaped() @test
{
    Vec3f[6] concave = {
        { 0, 0, 0 },
        { 4, 0, 0 },
        { 4, 2, 0 },
        { 2, 1, 0 },
        { 4, 4, 0 },
        { 0, 4, 0 },
    };
    HalfEdgeMesh mesh = triangulate::triangulate_polygon(mem, concave[..])!!;
    defer mesh.destroy();
    assert(mesh.faces.len == 4);
    mesh.validate()!!;
}
