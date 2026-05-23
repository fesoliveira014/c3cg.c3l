module test;
import cg;

fn void test_delaunay_sphere_empty_input() @test
{
    Vec3f[] pts = {};
    if (catch err = delaunay::delaunay_on_sphere(mem, pts, 1.0f)) {
        assert(err == cg::EMPTY_INPUT);
        return;
    }
    unreachable();
}
