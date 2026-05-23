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
