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
