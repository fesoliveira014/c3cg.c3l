module test;
import cg;
import cg::hull;
import cg::geometry;

const float HULL_TEST_EPSILON = 0.001f;

fn int m7_find_in_slice(int[] slice, int value)
{
    for (usz i = 0; i < slice.len; i++) {
        if (slice[i] == value) return (int)i;
    }
    return -1;
}

fn void test_hull_empty_input_faults() @test
{
    Vec3f[] positions = {};
    if (catch err = hull::hull_2d(mem, positions)) {
        assert(err == cg::EMPTY_INPUT);
        return;
    }
    unreachable();
}

fn void test_hull_single_point_faults() @test
{
    Vec3f[1] pts = { { 0, 0, 0 } };
    if (catch err = hull::hull_2d(mem, pts[..])) {
        assert(err == cg::DEGENERATE_INPUT);
        return;
    }
    unreachable();
}

fn void test_hull_two_points_faults() @test
{
    Vec3f[2] pts = { { 0, 0, 0 }, { 1, 0, 0 } };
    if (catch err = hull::hull_2d(mem, pts[..])) {
        assert(err == cg::DEGENERATE_INPUT);
        return;
    }
    unreachable();
}

fn void test_hull_all_collinear_faults() @test
{
    Vec3f[4] pts = {
        { 0, 0, 0 },
        { 1, 0, 0 },
        { 2, 0, 0 },
        { 3, 0, 0 },
    };
    if (catch err = hull::hull_2d(mem, pts[..])) {
        assert(err == cg::DEGENERATE_INPUT);
        return;
    }
    unreachable();
}

fn void test_hull_square_four_corners() @test
{
    Vec3f[4] pts = {
        { 0, 0, 0 },
        { 1, 0, 0 },
        { 1, 1, 0 },
        { 0, 1, 0 },
    };
    int[] hull = hull::hull_2d(mem, pts[..])!!;
    defer free(hull);

    assert(hull.len == 4);

    bool[4] seen = { false, false, false, false };
    for (usz i = 0; i < hull.len; i++) {
        assert(hull[i] >= 0 && hull[i] < 4);
        seen[hull[i]] = true;
    }
    for (usz i = 0; i < 4; i++) assert(seen[i]);
}

fn void test_hull_square_with_interior_point() @test
{
    Vec3f[5] pts = {
        { 0, 0, 0 },
        { 1, 0, 0 },
        { 1, 1, 0 },
        { 0, 1, 0 },
        { 0.5, 0.5, 0 },
    };
    int[] hull = hull::hull_2d(mem, pts[..])!!;
    defer free(hull);

    assert(hull.len == 4);
    assert(m7_find_in_slice(hull, 4) == -1);
}

fn void test_hull_square_collinear_on_edges() @test
{
    Vec3f[6] pts = {
        { 0, 0, 0 },
        { 1, 0, 0 },
        { 1, 1, 0 },
        { 0, 1, 0 },
        { 0.5, 0, 0 },
        { 0, 0.5, 0 },
    };
    int[] hull = hull::hull_2d(mem, pts[..])!!;
    defer free(hull);

    assert(m7_find_in_slice(hull, 4) >= 0);
    assert(m7_find_in_slice(hull, 5) >= 0);
    for (int i = 0; i < 4; i++) {
        assert(m7_find_in_slice(hull, i) >= 0);
    }
}

fn void test_hull_triangle_smoke() @test
{
    Vec3f[3] pts = {
        { 0, 0, 0 },
        { 2, 0, 0 },
        { 0, 2, 0 },
    };
    int[] hull = hull::hull_2d(mem, pts[..])!!;
    defer free(hull);

    assert(hull.len == 3);
    for (int i = 0; i < 3; i++) {
        assert(m7_find_in_slice(hull, i) >= 0);
    }
}

fn void test_hull_square_ccw_order() @test
{
    Vec3f[4] pts = {
        { 0, 0, 0 },
        { 1, 0, 0 },
        { 1, 1, 0 },
        { 0, 1, 0 },
    };
    int[] hull = hull::hull_2d(mem, pts[..])!!;
    defer free(hull);

    assert(hull.len == 4);
    for (usz i = 0; i < hull.len; i++) {
        usz j = (i + (usz)1) % (usz)hull.len;
        usz k = (i + (usz)2) % (usz)hull.len;
        int a_idx = hull[i];
        int b_idx = hull[j];
        int c_idx = hull[k];
        Vec2f a = { pts[a_idx].x, pts[a_idx].y };
        Vec2f b = { pts[b_idx].x, pts[b_idx].y };
        Vec2f c = { pts[c_idx].x, pts[c_idx].y };
        assert(geometry::orient_2d(a, b, c) != geometry::PREDICATE_NEGATIVE);
    }
}

fn void test_hull_random_cloud_points_inside() @test
{
    Vec3f[8] pts = {
        { 0, 1, 0 },
        { 0.8660254, 0.5, 0 },
        { 0.8660254, -0.5, 0 },
        { 0, -1, 0 },
        { -0.8660254, -0.5, 0 },
        { -0.8660254, 0.5, 0 },
        { 0.2, 0.2, 0 },
        { -0.2, -0.1, 0 },
    };
    int[] hull = hull::hull_2d(mem, pts[..])!!;
    defer free(hull);

    for (int i = 0; i < 6; i++) {
        assert(m7_find_in_slice(hull, i) >= 0);
    }
    assert(m7_find_in_slice(hull, 6) == -1);
    assert(m7_find_in_slice(hull, 7) == -1);

    for (usz ei = 0; ei < hull.len; ei++) {
        usz ej = (ei + (usz)1) % (usz)hull.len;
        int a_idx = hull[ei];
        int b_idx = hull[ej];
        Vec2f a = { pts[a_idx].x, pts[a_idx].y };
        Vec2f b = { pts[b_idx].x, pts[b_idx].y };
        for (usz j = 0; j < pts.len; j++) {
            Vec2f p = { pts[j].x, pts[j].y };
            assert(geometry::orient_2d(a, b, p) != geometry::PREDICATE_NEGATIVE);
        }
    }
}
