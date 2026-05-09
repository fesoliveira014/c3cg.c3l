module cg::geometry;
import cg;

typedef PredicateSign = int;

const float GEOMETRY_EPSILON = 0.00001f;
const float DEFAULT_PREDICATE_EPSILON = 0.00001f;
const PredicateSign PREDICATE_NEGATIVE = -1;
const PredicateSign PREDICATE_ZERO = 0;
const PredicateSign PREDICATE_POSITIVE = 1;

fn float abs_float(float value)
{
    if (value < 0.0f) return -value;
    return value;
}

fn PredicateSign classify_predicate(float value, float epsilon)
{
    float e = abs_float(epsilon);
    if (value > e) return PREDICATE_POSITIVE;
    if (value < -e) return PREDICATE_NEGATIVE;
    return PREDICATE_ZERO;
}

fn PredicateSign orient_2d(Vec2f a, Vec2f b, Vec2f c, float epsilon = DEFAULT_PREDICATE_EPSILON)
{
    float value = (b.x - a.x) * (c.y - a.y)
                - (b.y - a.y) * (c.x - a.x);
    return classify_predicate(value, epsilon);
}
