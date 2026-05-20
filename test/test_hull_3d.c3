module test;
import cg;
import cg::hull;
import cg::geometry;
import cg::half_edge;

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

const float M8_EPSILON = 0.001f;

fn void test_hull_3d_tetrahedron() @test
{
    Vec3f[4] pts = {
        { 0, 0, 0 }, { 1, 0, 0 }, { 0, 1, 0 }, { 0, 0, 1 },
    };
    HalfEdgeMesh mesh = hull::hull_3d(mem, pts[..])!!;
    defer mesh.destroy();
    mesh.validate()!!;
    assert(mesh.faces.len == 4);
    for (usz i = 0; i < mesh.half_edges.len; i++) {
        assert(!mesh.is_boundary((HeIndex)(int)i));
    }
}

fn void test_hull_3d_cube() @test
{
    Vec3f[8] pts = {
        { 0, 0, 0 }, { 1, 0, 0 }, { 1, 1, 0 }, { 0, 1, 0 },
        { 0, 0, 1 }, { 1, 0, 1 }, { 1, 1, 1 }, { 0, 1, 1 },
    };
    HalfEdgeMesh mesh = hull::hull_3d(mem, pts[..])!!;
    defer mesh.destroy();
    mesh.validate()!!;
    assert(mesh.faces.len == 12);
    for (usz i = 0; i < mesh.half_edges.len; i++) {
        assert(!mesh.is_boundary((HeIndex)(int)i));
    }
}

fn void test_hull_3d_cube_interior() @test
{
    Vec3f[9] pts = {
        { 0, 0, 0 }, { 1, 0, 0 }, { 1, 1, 0 }, { 0, 1, 0 },
        { 0, 0, 1 }, { 1, 0, 1 }, { 1, 1, 1 }, { 0, 1, 1 },
        { 0.5, 0.5, 0.5 },
    };
    HalfEdgeMesh mesh = hull::hull_3d(mem, pts[..])!!;
    defer mesh.destroy();
    assert(mesh.faces.len == 12);
    mesh.validate()!!;
}

fn void test_hull_3d_cube_coplanar_face() @test
{
    Vec3f[9] pts = {
        { 0, 0, 0 }, { 1, 0, 0 }, { 1, 1, 0 }, { 0, 1, 0 },
        { 0, 0, 1 }, { 1, 0, 1 }, { 1, 1, 1 }, { 0, 1, 1 },
        { 0.5, 0.5, 1.0 },
    };
    HalfEdgeMesh mesh = hull::hull_3d(mem, pts[..])!!;
    defer mesh.destroy();
    assert(mesh.faces.len == 12);
    mesh.validate()!!;
}

fn void test_hull_3d_cube_duplicates() @test
{
    Vec3f[10] pts = {
        { 0, 0, 0 }, { 1, 0, 0 }, { 1, 1, 0 }, { 0, 1, 0 },
        { 0, 0, 1 }, { 1, 0, 1 }, { 1, 1, 1 }, { 0, 1, 1 },
        { 0, 0, 0 }, { 1, 0, 0 },
    };
    HalfEdgeMesh mesh = hull::hull_3d(mem, pts[..])!!;
    defer mesh.destroy();
    assert(mesh.faces.len == 12);
    mesh.validate()!!;
}

fn void test_hull_3d_cube_convexity() @test
{
    Vec3f[8] pts = {
        { 0, 0, 0 }, { 1, 0, 0 }, { 1, 1, 0 }, { 0, 1, 0 },
        { 0, 0, 1 }, { 1, 0, 1 }, { 1, 1, 1 }, { 0, 1, 1 },
    };
    HalfEdgeMesh mesh = hull::hull_3d(mem, pts[..])!!;
    defer mesh.destroy();

    VertexIndex[3] verts;
    for (FaceIndex f = (FaceIndex)0; (int)f < (int)mesh.faces.len; f++) {
        mesh.face_vertices(f, verts[..])!!;
        for (usz j = 0; j < pts.len; j++) {
            assert(geometry::orient_3d(
                mesh.positions[(int)verts[0]],
                mesh.positions[(int)verts[1]],
                mesh.positions[(int)verts[2]],
                pts[j], M8_EPSILON) != geometry::PREDICATE_POSITIVE);
        }
    }
}
