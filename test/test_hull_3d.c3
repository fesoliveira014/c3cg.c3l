module test;
import cg;
import cg::hull;

fn void test_hull_3d_empty_faults() @test
{
    Vec3f[] positions = {};
    if (catch err = hull::hull_3d(mem, positions)) {
        assert(err == cg::EMPTY_INPUT);
        return;
    }
    unreachable();
}

fn void test_hull_3d_one_point_faults() @test
{
    Vec3f[1] pts = { { 0, 0, 0 } };
    if (catch err = hull::hull_3d(mem, pts[..])) {
        assert(err == cg::DEGENERATE_INPUT);
        return;
    }
    unreachable();
}

fn void test_hull_3d_two_points_faults() @test
{
    Vec3f[2] pts = { { 0, 0, 0 }, { 1, 0, 0 } };
    if (catch err = hull::hull_3d(mem, pts[..])) {
        assert(err == cg::DEGENERATE_INPUT);
        return;
    }
    unreachable();
}

fn void test_hull_3d_three_points_faults() @test
{
    Vec3f[3] pts = { { 0, 0, 0 }, { 1, 0, 0 }, { 0, 1, 0 } };
    if (catch err = hull::hull_3d(mem, pts[..])) {
        assert(err == cg::DEGENERATE_INPUT);
        return;
    }
    unreachable();
}

fn void test_hull_3d_coplanar_faults() @test
{
    Vec3f[5] pts = {
        { 0, 0, 0 }, { 1, 0, 0 }, { 0, 1, 0 }, { 1, 1, 0 }, { 0.5, 0.5, 0 },
    };
    if (catch err = hull::hull_3d(mem, pts[..])) {
        assert(err == cg::DEGENERATE_INPUT);
        return;
    }
    unreachable();
}
