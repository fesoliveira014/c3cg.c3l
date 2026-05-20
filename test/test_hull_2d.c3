module test;
import cg;
import cg::hull;

fn void test_hull_empty_input_faults() @test
{
    Vec3f[] positions = {};
    if (catch err = hull::hull_2d(mem, positions)) {
        assert(err == cg::EMPTY_INPUT);
        return;
    }
    unreachable();
}
