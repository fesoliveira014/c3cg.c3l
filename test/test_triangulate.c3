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
    int[] indices = triangulate::triangulate_polygon(mem, tri[..])!!;
    defer free(indices);
    assert(indices.len == 3);
    assert(indices[0] == 0);
    assert(indices[1] == 1);
    assert(indices[2] == 2);
}

fn void test_triangulate_square() @test
{
    Vec3f[4] square = { {0,0,0}, {1,0,0}, {1,1,0}, {0,1,0} };
    int[] indices = triangulate::triangulate_polygon(mem, square[..])!!;
    defer free(indices);
    assert(indices.len == 6);
    for (usz i = 0; i < indices.len; i++) {
        assert(indices[i] >= 0 && (usz)indices[i] < 4);
    }
}

fn void test_triangulate_pentagon() @test
{
    Vec3f[5] pent = { {0,1,0}, {1,0.3f,0}, {0.6f,-1,0}, {-0.6f,-1,0}, {-1,0.3f,0} };
    int[] indices = triangulate::triangulate_polygon(mem, pent[..])!!;
    defer free(indices);
    assert(indices.len == 9);
    for (usz i = 0; i < indices.len; i++) {
        assert(indices[i] >= 0 && (usz)indices[i] < 5);
    }
}

fn void test_triangulate_trapezoid() @test
{
    Vec3f[4] trapezoid = {
        { -2, 0, 0 },
        { 2, 0, 0 },
        { 1, 2, 0 },
        { -1, 2, 0 },
    };
    int[] indices = triangulate::triangulate_polygon(mem, trapezoid[..])!!;
    defer free(indices);
    assert(indices.len == 6);
    for (usz i = 0; i < indices.len; i++) {
        assert(indices[i] >= 0 && (usz)indices[i] < 4);
    }
}

fn void test_triangulate_l_shaped() @test
{
    Vec3f[6] lshape = { {0,0,0}, {2,0,0}, {2,1,0}, {1,1,0}, {1,2,0}, {0,2,0} };
    int[] indices = triangulate::triangulate_polygon(mem, lshape[..])!!;
    defer free(indices);
    assert(indices.len == 12);
    for (usz i = 0; i < indices.len; i++) {
        assert(indices[i] >= 0 && (usz)indices[i] < 6);
    }
}

fn void test_triangulate_star_shaped() @test
{
    Vec3f[10] star = {
        { 0, 3, 0 },
        { 1, 1, 0 },
        { 3, 0, 0 },
        { 1, -1, 0 },
        { 0, -3, 0 },
        { -1, -1, 0 },
        { -3, 0, 0 },
        { -1, 1, 0 },
        { -0.5f, 0, 0 },
        { 0.5f, 0, 0 },
    };
    int[] indices = triangulate::triangulate_polygon(mem, star[..])!!;
    defer free(indices);
    assert(indices.len == 24);
    for (usz i = 0; i < indices.len; i++) {
        assert(indices[i] >= 0 && (usz)indices[i] < 10);
    }
}
