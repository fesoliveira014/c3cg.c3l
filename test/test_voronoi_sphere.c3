module test;
import cg;
import cg::delaunay;
import cg::voronoi;
import cg::geometry;
import std::math;

fn void test_voronoi_on_sphere_icosahedron_to_dodecahedron() @test
{
    VoronoiDiagram diagram = voronoi::voronoi_on_sphere(mem, ICO_SPHERE[..], 1.0f)!!;
    defer diagram.destroy();

    diagram.mesh.validate()!!;

    assert(diagram.mesh.faces.len == 12);
    assert(diagram.mesh.positions.len == 20);
    assert(diagram.sites.len == 12);

    for (usz i = 0; i < diagram.mesh.half_edges.len; i++) {
        assert(!diagram.mesh.is_boundary((HeIndex)(int)i));
    }

    for (FaceIndex f = (FaceIndex)0; (int)f < (int)diagram.mesh.faces.len; f++) {
        int deg = diagram.mesh.face_degree(f);
        assert(deg == 5);
    }
}

fn void test_voronoi_on_sphere_roundtrip() @test
{
    VoronoiDiagram diagram = voronoi::voronoi_on_sphere(mem, ICO_SPHERE[..], 1.0f)!!;
    defer diagram.destroy();

    HalfEdgeMesh delaunay2 = voronoi::to_delaunay(mem, &diagram)!!;
    defer delaunay2.destroy();

    delaunay2.validate()!!;

    assert(delaunay2.vertices.len == 12);
    assert(delaunay2.faces.len == 20);
    assert(delaunay2.half_edges.len == 60);

    for (usz i = 0; i < delaunay2.half_edges.len; i++) {
        assert(!delaunay2.is_boundary((HeIndex)(int)i));
    }
}

fn void test_voronoi_dual_sphere_centroid() @test
{
    HalfEdgeMesh tri_mesh = delaunay::delaunay_on_sphere(mem, ICO_SPHERE[..], 1.0f)!!;
    defer tri_mesh.destroy();

    Vec3f[] centroids = geometry::centroids_on_sphere(mem, &tri_mesh, 1.0f)!!;
    defer free(centroids);

    VoronoiDiagram diagram = voronoi::voronoi_dual_sphere(mem, &tri_mesh, centroids)!!;
    defer diagram.destroy();

    diagram.mesh.validate()!!;

    assert(diagram.mesh.faces.len == 12);

    for (usz i = 0; i < diagram.mesh.half_edges.len; i++) {
        assert(!diagram.mesh.is_boundary((HeIndex)(int)i));
    }
}

fn void test_voronoi_on_sphere_radius_preservation() @test
{
    VoronoiDiagram diagram = voronoi::voronoi_on_sphere(mem, ICO_SPHERE[..], 5.0f)!!;
    defer diagram.destroy();

    for (usz i = 0; i < diagram.sites.len; i++) {
        float x = diagram.sites[i].x;
        float y = diagram.sites[i].y;
        float z = diagram.sites[i].z;
        float r2 = x * x + y * y + z * z;
        float r = (float)math::sqrt((double)r2);
        assert(approx_abs(r - 5.0f) < SPHERE_EPSILON);
    }
}

fn void test_voronoi_on_sphere_empty_input() @test
{
    Vec3f[] pts = {};
    if (catch err = voronoi::voronoi_on_sphere(mem, pts, 1.0f)) {
        assert(err == cg::EMPTY_INPUT);
        return;
    }
    unreachable();
}

fn void test_voronoi_on_sphere_too_few_points() @test
{
    Vec3f[3] pts = { {1,0,0}, {0,1,0}, {0,0,1} };
    if (catch err = voronoi::voronoi_on_sphere(mem, pts[..], 1.0f)) {
        assert(err == cg::DEGENERATE_INPUT);
        return;
    }
    unreachable();
}

fn void test_voronoi_on_sphere_radius_zero() @test
{
    if (catch err = voronoi::voronoi_on_sphere(mem, ICO_SPHERE[..], 0.0f)) {
        assert(err == cg::DEGENERATE_INPUT);
        return;
    }
    unreachable();
}
