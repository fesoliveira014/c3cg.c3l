module test;
import cg;

fn void test_voronoi_sphere_empty_input() @test
{
    Vec3f[] pts = {};
    if (catch err = voronoi::voronoi_on_sphere(mem, pts, 1.0f)) {
        assert(err == cg::EMPTY_INPUT);
        return;
    }
    unreachable();
}
