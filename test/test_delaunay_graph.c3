module test;
import cg;
import cg::delaunay;
import cg::geometry;
import cg::graph;
import cg::half_edge;
import std::math;

fn void test_delaunay_graph_triangle_count() @test
{
    HalfEdgeMesh mesh = delaunay_4_site_square();
    defer mesh.destroy();

    DelaunayGraph g = graph::from_planar_delaunay(mem, &mesh)!!;
    defer g.destroy();

    assert(g.triangle_count() == 2);
}

fn void test_delaunay_graph_circumcenters_match() @test
{
    HalfEdgeMesh mesh = delaunay_4_site_square();
    defer mesh.destroy();

    Vec3f[] expected = geometry::circumcenters_planar(mem, &mesh)!!;
    defer free(expected);

    DelaunayGraph g = graph::from_planar_delaunay(mem, &mesh)!!;
    defer g.destroy();

    for (int t = 0; t < g.triangle_count(); t++) {
        DelaunayTriangleView tv = g.triangle((FaceIndex)t);
        assert(approx_abs(tv.circumcenter.x - expected[t].x) < 0.001f);
        assert(approx_abs(tv.circumcenter.y - expected[t].y) < 0.001f);
        assert(approx_abs(tv.circumcenter.z - expected[t].z) < 0.001f);
    }
}

fn void test_delaunay_graph_neighbor_symmetry() @test
{
    HalfEdgeMesh mesh = delaunay_4_site_square();
    defer mesh.destroy();

    DelaunayGraph g = graph::from_planar_delaunay(mem, &mesh)!!;
    defer g.destroy();

    for (int a = 0; a < g.triangle_count(); a++) {
        DelaunayTriangleView ta = g.triangle((FaceIndex)a);
        for (int i = 0; i < 3; i++) {
            if (ta.neighbors[i] == cg::INVALID_FACE) continue;
            int b = (int)ta.neighbors[i];
            DelaunayTriangleView tb = g.triangle((FaceIndex)b);
            bool found = false;
            for (int j = 0; j < 3; j++) {
                if ((int)tb.neighbors[j] == a) {
                    found = true;
                    break;
                }
            }
            assert(found);
        }
    }
}

fn void test_delaunay_graph_vertices_preserved() @test
{
    HalfEdgeMesh mesh = delaunay_4_site_square();
    defer mesh.destroy();

    DelaunayGraph g = graph::from_planar_delaunay(mem, &mesh)!!;
    defer g.destroy();

    assert(g.vertices.len == mesh.positions.len);
    for (usz i = 0; i < g.vertices.len; i++) {
        assert(g.vertices[i].x == mesh.positions[i].x);
        assert(g.vertices[i].y == mesh.positions[i].y);
        assert(g.vertices[i].z == mesh.positions[i].z);
    }
}

fn void test_delaunay_graph_convenience_constructors_equal() @test
{
    HalfEdgeMesh mesh = delaunay_4_site_square();
    defer mesh.destroy();

    Vec3f[] ccs = geometry::circumcenters_planar(mem, &mesh)!!;
    defer free(ccs);

    DelaunayGraph g1 = graph::from_delaunay(mem, &mesh, ccs)!!;
    defer g1.destroy();

    DelaunayGraph g2 = graph::from_planar_delaunay(mem, &mesh)!!;
    defer g2.destroy();

    assert(g1.triangle_count() == g2.triangle_count());
    for (int t = 0; t < g1.triangle_count(); t++) {
        DelaunayTriangleView v1 = g1.triangle((FaceIndex)t);
        DelaunayTriangleView v2 = g2.triangle((FaceIndex)t);
        for (int i = 0; i < 3; i++) {
            assert(v1.vertices[i] == v2.vertices[i]);
            assert(v1.neighbors[i] == v2.neighbors[i]);
        }
    }
}

fn void test_delaunay_graph_icosahedron_spherical() @test
{
    HalfEdgeMesh mesh = delaunay::delaunay_on_sphere(mem, ICO_SPHERE[..], 1.0f)!!;
    defer mesh.destroy();

    DelaunayGraph g = graph::from_spherical_delaunay(mem, &mesh, 1.0f)!!;
    defer g.destroy();

    assert(g.triangle_count() == 20);

    for (int t = 0; t < g.triangle_count(); t++) {
        DelaunayTriangleView tv = g.triangle((FaceIndex)t);
        for (int i = 0; i < 3; i++) {
            assert(tv.neighbors[i] != cg::INVALID_FACE);
        }
    }
}

fn void test_delaunay_graph_spherical_convenience_equal() @test
{
    HalfEdgeMesh mesh = delaunay::delaunay_on_sphere(mem, ICO_SPHERE[..], 1.0f)!!;
    defer mesh.destroy();

    Vec3f[] ccs = geometry::circumcenters_on_sphere(mem, &mesh, 1.0f)!!;
    defer free(ccs);

    DelaunayGraph g1 = graph::from_delaunay(mem, &mesh, ccs)!!;
    defer g1.destroy();

    DelaunayGraph g2 = graph::from_spherical_delaunay(mem, &mesh, 1.0f)!!;
    defer g2.destroy();

    assert(g1.triangle_count() == g2.triangle_count());
    for (int t = 0; t < g1.triangle_count(); t++) {
        DelaunayTriangleView v1 = g1.triangle((FaceIndex)t);
        DelaunayTriangleView v2 = g2.triangle((FaceIndex)t);
        for (int i = 0; i < 3; i++) {
            assert(v1.vertices[i] == v2.vertices[i]);
            assert(v1.neighbors[i] == v2.neighbors[i]);
        }
    }
}

fn void test_delaunay_graph_icosahedron_vertices_on_sphere() @test
{
    HalfEdgeMesh mesh = delaunay::delaunay_on_sphere(mem, ICO_SPHERE[..], 1.0f)!!;
    defer mesh.destroy();

    DelaunayGraph g = graph::from_spherical_delaunay(mem, &mesh, 1.0f)!!;
    defer g.destroy();

    for (int t = 0; t < g.triangle_count(); t++) {
        DelaunayTriangleView tv = g.triangle((FaceIndex)t);
        for (int i = 0; i < 3; i++) {
            Vec3f v = g.vertices[(int)tv.vertices[i]];
            float r = (float)math::sqrt((double)(v.x*v.x + v.y*v.y + v.z*v.z));
            assert(approx_abs(r - 1.0f) < 0.001f);
        }
    }
}

fn void test_delaunay_graph_empty_input() @test
{
    HalfEdgeMesh mesh;
    if (catch err = graph::from_planar_delaunay(mem, &mesh)) {
        assert(err == cg::EMPTY_INPUT);
        return;
    }
    unreachable();
}

fn void test_delaunay_graph_non_triangle_face() @test
{
    Vec3f[4] pts = { {0,0,0}, {1,0,0}, {1,1,0}, {0,1,0} };
    uint[2] offsets = {0, 4};
    uint[4] indices = {0, 1, 2, 3};
    HalfEdgeMesh mesh = half_edge::from_polygons(mem, pts[..], offsets[..], indices[..])!!;
    defer mesh.destroy();

    Vec3f[1] ccs = { {0,0,0} };
    if (catch err = graph::from_delaunay(mem, &mesh, ccs[..])) {
        assert(err == cg::NON_TRIANGLE_FACE);
        return;
    }
    unreachable();
}

fn void test_delaunay_graph_vertex_count_mismatch() @test
{
    HalfEdgeMesh mesh = delaunay_4_site_square();
    defer mesh.destroy();

    Vec3f[1] wrong = { {0,0,0} };
    if (catch err = graph::from_delaunay(mem, &mesh, wrong[..])) {
        assert(err == cg::DUAL_VERTEX_COUNT_MISMATCH);
        return;
    }
    unreachable();
}

fn void test_delaunay_graph_triangle_view() @test
{
    HalfEdgeMesh mesh = delaunay_4_site_square();
    defer mesh.destroy();

    DelaunayGraph g = graph::from_planar_delaunay(mem, &mesh)!!;
    defer g.destroy();

    for (int t = 0; t < g.triangle_count(); t++) {
        DelaunayTriangleView tv = g.triangle((FaceIndex)t);
        assert(tv.index == (FaceIndex)t);
        for (int i = 0; i < 3; i++) {
            assert((int)tv.vertices[i] >= 0);
            assert((usz)tv.vertices[i] < g.vertices.len);
        }
    }
}

fn HalfEdgeMesh delaunay_4_site_square()
{
    Vec3f[4] pts = { {0,0,0}, {1,0,0}, {1,1,0}, {0,1,0} };
    return delaunay::delaunay_2d(mem, pts[..])!!;
}
