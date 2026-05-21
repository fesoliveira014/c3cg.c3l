module test;
import cg;
import cg::delaunay;

fn void test_delaunay_2d_empty() @test
{
    Vec3f[] pts = {};
    if (catch err = delaunay::delaunay_2d(mem, pts)) {
        assert(err == cg::EMPTY_INPUT);
        return;
    }
    unreachable();
}

fn void test_delaunay_2d_one_point() @test
{
    Vec3f[1] pts = { { 0, 0, 0 } };
    if (catch err = delaunay::delaunay_2d(mem, pts[..])) {
        assert(err == cg::DEGENERATE_INPUT);
        return;
    }
    unreachable();
}

fn void test_delaunay_2d_two_points() @test
{
    Vec3f[2] pts = { { 0, 0, 0 }, { 1, 0, 0 } };
    if (catch err = delaunay::delaunay_2d(mem, pts[..])) {
        assert(err == cg::DEGENERATE_INPUT);
        return;
    }
    unreachable();
}

fn void test_delaunay_2d_collinear() @test
{
    Vec3f[4] pts = { { 0, 0, 0 }, { 1, 0, 0 }, { 2, 0, 0 }, { 3, 0, 0 } };
    if (catch err = delaunay::delaunay_2d(mem, pts[..])) {
        assert(err == cg::DEGENERATE_INPUT);
        return;
    }
    unreachable();
}
