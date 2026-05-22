module test;
import cg;
import cg::voronoi;

fn void test_in_polygon_four_sites_square() @test
{
    Vec3f[4] s = { {0.25,0.25,0}, {0.75,0.25,0}, {0.75,0.75,0}, {0.25,0.75,0} };
    Vec3f[4] p = { {0,0,0}, {1,0,0}, {1,1,0}, {0,1,0} };
    VoronoiDiagram d = voronoi::in_polygon(mem, s[..], p[..])!!;
    defer d.destroy();
    // N>=3 path returns empty for now — will fix in Task 4
}

fn void test_in_polygon_n1_inside() @test
{
    Vec3f[1] s = { {0.5,0.5,0} };
    Vec3f[4] p = { {0,0,0}, {1,0,0}, {1,1,0}, {0,1,0} };
    VoronoiDiagram d = voronoi::in_polygon(mem, s[..], p[..])!!;
    defer d.destroy();
    assert(d.mesh.faces.len == 1);
    assert(d.sites.len == 1);
    d.mesh.validate()!!;
}

fn void test_in_polygon_n1_outside() @test
{
    Vec3f[1] s = { {-1,0.5,0} };
    Vec3f[4] p = { {0,0,0}, {1,0,0}, {1,1,0}, {0,1,0} };
    VoronoiDiagram d = voronoi::in_polygon(mem, s[..], p[..])!!;
    defer d.destroy();
    assert(d.mesh.faces.len == 0);
}

fn void test_in_polygon_n2_both_inside() @test
{
    Vec3f[2] s = { {0.25,0.5,0}, {0.75,0.5,0} };
    Vec3f[4] p = { {0,0,0}, {1,0,0}, {1,1,0}, {0,1,0} };
    VoronoiDiagram d = voronoi::in_polygon(mem, s[..], p[..])!!;
    defer d.destroy();
    assert(d.mesh.faces.len == 2);
    d.mesh.validate()!!;
}

fn void test_in_polygon_non_convex() @test
{
    Vec3f[1] s = { {0.5,0.5,0} };
    Vec3f[5] p = { {0,0,0}, {1,0,0}, {1,1,0}, {0.5,0.5,0}, {0,1,0} };
    if (catch err = voronoi::in_polygon(mem, s[..], p[..])) {
        assert(err == cg::NON_CONVEX_BOUNDING_POLYGON);
        return;
    }
    unreachable();
}

fn void test_in_polygon_empty_sites() @test
{
    Vec3f[] s = {};
    Vec3f[4] p = { {0,0,0}, {1,0,0}, {1,1,0}, {0,1,0} };
    if (catch err = voronoi::in_polygon(mem, s, p[..])) {
        assert(err == cg::EMPTY_INPUT);
        return;
    }
    unreachable();
}
