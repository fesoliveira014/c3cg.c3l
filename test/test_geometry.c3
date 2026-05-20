module test;
import cg;
import cg::half_edge;
import cg::geometry;

const float M5_TEST_EPSILON = 0.001f;

fn float m5_abs(float value)
{
    if (value < 0.0f) return -value;
    return value;
}

fn bool m5_float_approx(float a, float b)
{
    return m5_abs(a - b) <= M5_TEST_EPSILON;
}

fn bool m5_vec3_approx(Vec3f a, Vec3f b)
{
    return m5_float_approx(a.x, b.x)
        && m5_float_approx(a.y, b.y)
        && m5_float_approx(a.z, b.z);
}

fn float m5_dot3(Vec3f a, Vec3f b)
{
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

fn Vec3f m5_cross3(Vec3f a, Vec3f b)
{
    return {
        a.y * b.z - a.z * b.y,
        a.z * b.x - a.x * b.z,
        a.x * b.y - a.y * b.x,
    };
}

fn Vec3f m5_sub3(Vec3f a, Vec3f b)
{
    return { a.x - b.x, a.y - b.y, a.z - b.z };
}

fn void test_predicate_sign_constants() @test
{
    assert((int)geometry::PREDICATE_NEGATIVE == -1);
    assert((int)geometry::PREDICATE_ZERO == 0);
    assert((int)geometry::PREDICATE_POSITIVE == 1);
}

fn void test_orient_2d_classifies_ccw_cw_collinear() @test
{
    Vec2f a = { 0, 0 };
    Vec2f b = { 1, 0 };
    Vec2f c = { 0, 1 };

    assert(geometry::orient_2d(a, b, c) == geometry::PREDICATE_POSITIVE);
    assert(geometry::orient_2d(a, c, b) == geometry::PREDICATE_NEGATIVE);
    assert(geometry::orient_2d({ 0, 0 }, { 1, 1 }, { 2, 2 }) == geometry::PREDICATE_ZERO);
}

fn void test_orient_2d_uses_absolute_negative_epsilon() @test
{
    Vec2f a = { 0, 0 };
    Vec2f b = { 1, 0 };
    Vec2f c = { 0, 0.000001 };

    assert(geometry::orient_2d(a, b, c, -0.01f) == geometry::PREDICATE_ZERO);
    assert(geometry::orient_2d(a, b, c, 0.0f) == geometry::PREDICATE_POSITIVE);
}

fn void test_in_circle_2d_classifies_for_ccw_triangle() @test
{
    Vec2f a = { 0, 0 };
    Vec2f b = { 2, 0 };
    Vec2f c = { 0, 2 };

    assert(geometry::in_circle_2d(a, b, c, { 0.5, 0.5 }) == geometry::PREDICATE_POSITIVE);
    assert(geometry::in_circle_2d(a, b, c, { 3, 3 }) == geometry::PREDICATE_NEGATIVE);
    assert(geometry::in_circle_2d(a, b, c, { 2, 2 }) == geometry::PREDICATE_ZERO);
}

fn void test_in_circle_2d_orientation_dependency() @test
{
    Vec2f a = { 0, 0 };
    Vec2f b = { 2, 0 };
    Vec2f c = { 0, 2 };
    Vec2f p = { 0.5, 0.5 };

    assert(geometry::in_circle_2d(a, b, c, p) == geometry::PREDICATE_POSITIVE);
    assert(geometry::in_circle_2d(a, c, b, p) == geometry::PREDICATE_NEGATIVE);
}

fn void test_on_sphere_classifies_known_tetrahedron() @test
{
    Vec3f a = { 1, 0, 0 };
    Vec3f b = { 0, 0, 1 };
    Vec3f c = { 0, 1, 0 };
    Vec3f d = { 0, 0, 0 };

    assert(geometry::on_sphere(a, b, c, d, { 0.5, 0.5, 0.5 }) == geometry::PREDICATE_POSITIVE);
    assert(geometry::on_sphere(a, b, c, d, { 3, 3, 3 }) == geometry::PREDICATE_NEGATIVE);
    assert(geometry::on_sphere(a, b, c, d, { 1, 1, 1 }) == geometry::PREDICATE_ZERO);
}

fn void test_on_sphere_negative_orientation_flips_sign() @test
{
    Vec3f a = { 1, 0, 0 };
    Vec3f b = { 0, 0, 1 };
    Vec3f c = { 0, 1, 0 };
    Vec3f d = { 0, 0, 0 };
    Vec3f p = { 0.5, 0.5, 0.5 };

    assert(geometry::on_sphere(a, b, c, d, p) == geometry::PREDICATE_POSITIVE);
    assert(geometry::on_sphere(a, c, b, d, p) == geometry::PREDICATE_NEGATIVE);
}

fn void test_on_sphere_degenerate_coplanar_classifies_zero() @test
{
    assert(geometry::on_sphere(
        { 0, 0, 0 }, { 1, 0, 0 }, { 0, 1, 0 }, { 1, 1, 0 }, { 0.25, 0.25, 0 }) == geometry::PREDICATE_ZERO);
}

fn void test_predicates_negative_epsilon_does_not_invert_classification() @test
{
    assert(geometry::in_circle_2d({ 0, 0 }, { 2, 0 }, { 0, 2 }, { 0.5, 0.5 }, -10.0f)
        == geometry::PREDICATE_ZERO);
    assert(geometry::on_sphere({ 1, 0, 0 }, { 0, 0, 1 }, { 0, 1, 0 }, { 0, 0, 0 }, { 0.5, 0.5, 0.5 }, -10.0f)
        == geometry::PREDICATE_ZERO);
}

fn void test_circumcenter_planar_right_triangle() @test
{
    Vec3f center = geometry::circumcenter_planar({ 0, 0, 0 }, { 2, 0, 0 }, { 0, 2, 0 })!!;
    assert(m5_vec3_approx(center, { 1, 1, 0 }));
}

fn void test_circumcenter_planar_tilted_triangle_equal_distances() @test
{
    Vec3f a = { 0, 0, 1 };
    Vec3f b = { 2, 0, 1 };
    Vec3f c = { 0, 2, 3 };
    Vec3f center = geometry::circumcenter_planar(a, b, c)!!;

    Vec3f da = m5_sub3(center, a);
    Vec3f db = m5_sub3(center, b);
    Vec3f dc = m5_sub3(center, c);
    float ra = m5_dot3(da, da);
    float rb = m5_dot3(db, db);
    float rc = m5_dot3(dc, dc);

    Vec3f normal = m5_cross3(m5_sub3(b, a), m5_sub3(c, a));
    float plane_distance_scaled = m5_dot3(m5_sub3(center, a), normal);

    assert(m5_float_approx(ra, rb));
    assert(m5_float_approx(ra, rc));
    assert(m5_float_approx(plane_distance_scaled, 0.0f));
}

fn void test_circumcenter_planar_degenerate_faults() @test
{
    if (catch err = geometry::circumcenter_planar({ 0, 0, 0 }, { 1, 1, 1 }, { 2, 2, 2 })) {
        assert(err == cg::DEGENERATE_INPUT);
    } else {
        unreachable();
    }

    if (catch err = geometry::circumcenter_planar({ 0, 0, 0 }, { 0, 0, 0 }, { 1, 0, 0 })) {
        assert(err == cg::DEGENERATE_INPUT);
        return;
    }
    unreachable();
}

fn void test_circumcenter_on_sphere_unit_axes() @test
{
    Vec3f center = geometry::circumcenter_on_sphere({ 1, 0, 0 }, { 0, 1, 0 }, { 0, 0, 1 }, 1.0f)!!;
    assert(m5_float_approx(m5_dot3(center, center), 1.0f));
    assert(center.x > 0.0f);
    assert(center.y > 0.0f);
    assert(center.z > 0.0f);
}

fn void test_circumcenter_on_sphere_invalid_radius_faults() @test
{
    if (catch err = geometry::circumcenter_on_sphere({ 1, 0, 0 }, { 0, 1, 0 }, { 0, 0, 1 }, 0.0f)) {
        assert(err == cg::DEGENERATE_INPUT);
        return;
    }
    unreachable();
}

fn void test_circumcenters_planar_triangle_mesh() @test
{
    Vec3f[4] positions = {
        { 0, 0, 0 },
        { 2, 0, 0 },
        { 0, 2, 0 },
        { 2, 2, 0 },
    };
    uint[6] indices = { 0, 1, 2, 1, 3, 2 };
    HalfEdgeMesh mesh = half_edge::from_triangles(mem, positions[..], indices[..])!!;
    defer mesh.destroy();

    Vec3f[] centers = geometry::circumcenters_planar(mem, &mesh)!!;
    defer free(centers);

    assert(centers.len == mesh.faces.len);
    assert(m5_vec3_approx(centers[0], { 1, 1, 0 }));
    assert(m5_vec3_approx(centers[1], { 1, 1, 0 }));
}

fn void test_circumcenters_planar_rejects_non_triangle_face() @test
{
    Vec3f[4] positions = {
        { 0, 0, 0 },
        { 1, 0, 0 },
        { 1, 1, 0 },
        { 0, 1, 0 },
    };
    uint[2] offsets = { 0, 4 };
    uint[4] indices = { 0, 1, 2, 3 };
    HalfEdgeMesh mesh = half_edge::from_polygons(mem, positions[..], offsets[..], indices[..])!!;
    defer mesh.destroy();

    if (catch err = geometry::circumcenters_planar(mem, &mesh)) {
        assert(err == cg::NON_TRIANGLE_FACE);
        return;
    }
    unreachable();
}

fn void test_circumcenters_planar_rejects_degenerate_triangle() @test
{
    Vec3f[3] positions = {
        { 0, 0, 0 },
        { 1, 1, 1 },
        { 2, 2, 2 },
    };
    uint[3] indices = { 0, 1, 2 };
    HalfEdgeMesh mesh = half_edge::from_triangles(mem, positions[..], indices[..])!!;
    defer mesh.destroy();

    if (catch err = geometry::circumcenters_planar(mem, &mesh)) {
        assert(err == cg::DEGENERATE_INPUT);
        return;
    }
    unreachable();
}

fn void test_circumcenters_on_sphere_triangle_mesh() @test
{
    Vec3f[3] positions = {
        { 1, 0, 0 },
        { 0, 1, 0 },
        { 0, 0, 1 },
    };
    uint[3] indices = { 0, 1, 2 };
    HalfEdgeMesh mesh = half_edge::from_triangles(mem, positions[..], indices[..])!!;
    defer mesh.destroy();

    Vec3f[] centers = geometry::circumcenters_on_sphere(mem, &mesh, 1.0f)!!;
    defer free(centers);

    assert(centers.len == 1);
    assert(m5_float_approx(m5_dot3(centers[0], centers[0]), 1.0f));
}

fn void test_face_centroid_triangle_average() @test
{
    Vec3f[3] positions = {
        { 0, 0, 0 },
        { 3, 0, 0 },
        { 0, 3, 0 },
    };
    uint[3] indices = { 0, 1, 2 };
    HalfEdgeMesh mesh = half_edge::from_triangles(mem, positions[..], indices[..])!!;
    defer mesh.destroy();

    Vec3f centroid = geometry::face_centroid(&mesh, (FaceIndex)0)!!;
    assert(m5_vec3_approx(centroid, { 1, 1, 0 }));
}

fn void test_face_centroid_quad_vertex_average() @test
{
    Vec3f[4] positions = {
        { 0, 0, 0 },
        { 2, 0, 0 },
        { 2, 2, 0 },
        { 0, 2, 0 },
    };
    uint[2] offsets = { 0, 4 };
    uint[4] indices = { 0, 1, 2, 3 };
    HalfEdgeMesh mesh = half_edge::from_polygons(mem, positions[..], offsets[..], indices[..])!!;
    defer mesh.destroy();

    Vec3f centroid = geometry::face_centroid(&mesh, (FaceIndex)0)!!;
    assert(m5_vec3_approx(centroid, { 1, 1, 0 }));
}

fn void test_face_centroid_invalid_face_faults() @test
{
    Vec3f[3] positions = {
        { 0, 0, 0 },
        { 1, 0, 0 },
        { 0, 1, 0 },
    };
    uint[3] indices = { 0, 1, 2 };
    HalfEdgeMesh mesh = half_edge::from_triangles(mem, positions[..], indices[..])!!;
    defer mesh.destroy();

    if (catch err = geometry::face_centroid(&mesh, (FaceIndex)99)) {
        assert(err == cg::INVALID_FACE_REFERENCE);
        return;
    }
    unreachable();
}

fn void test_face_centroids_batch_preserves_face_index_mapping() @test
{
    Vec3f[4] positions = {
        { 0, 0, 0 },
        { 3, 0, 0 },
        { 0, 3, 0 },
        { 3, 3, 0 },
    };
    uint[6] indices = { 0, 1, 2, 1, 3, 2 };
    HalfEdgeMesh mesh = half_edge::from_triangles(mem, positions[..], indices[..])!!;
    defer mesh.destroy();

    Vec3f[] centroids = geometry::face_centroids(mem, &mesh)!!;
    defer free(centroids);

    assert(centroids.len == mesh.faces.len);
    assert(m5_vec3_approx(centroids[0], { 1, 1, 0 }));
    assert(m5_vec3_approx(centroids[1], { 2, 2, 0 }));
}

fn void test_face_centroid_scalar_matches_batch() @test
{
    Vec3f[4] positions = {
        { 0, 0, 0 },
        { 2, 0, 0 },
        { 2, 2, 0 },
        { 0, 2, 0 },
    };
    uint[2] offsets = { 0, 4 };
    uint[4] indices = { 0, 1, 2, 3 };
    HalfEdgeMesh mesh = half_edge::from_polygons(mem, positions[..], offsets[..], indices[..])!!;
    defer mesh.destroy();

    Vec3f[] batch = geometry::face_centroids(mem, &mesh)!!;
    defer free(batch);

    Vec3f scalar = geometry::face_centroid(&mesh, (FaceIndex)0)!!;
    assert(m5_vec3_approx(scalar, batch[0]));
}

fn void test_orient_3d_positive() @test
{
    assert(geometry::orient_3d({0,1,0}, {1,0,0}, {0,0,1}, {0,0,0}) == geometry::PREDICATE_POSITIVE);
}

fn void test_orient_3d_negative() @test
{
    assert(geometry::orient_3d({1,0,0}, {0,1,0}, {0,0,1}, {0,0,0}) == geometry::PREDICATE_NEGATIVE);
}

fn void test_orient_3d_zero_coplanar() @test
{
    assert(geometry::orient_3d({0,0,0}, {1,0,0}, {0,1,0}, {1,1,0}) == geometry::PREDICATE_ZERO);
}

fn void test_orient_3d_epsilon() @test
{
    assert(geometry::orient_3d({0,0,0}, {1,0,0}, {0,1,0}, {1,1,0.000001f}, 0.0f) == geometry::PREDICATE_POSITIVE);
    assert(geometry::orient_3d({0,0,0}, {1,0,0}, {0,1,0}, {1,1,0.000001f}, 0.01f) == geometry::PREDICATE_ZERO);
}
