module test;
import cg;
import cg::half_edge;
import cg::geometry;

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
