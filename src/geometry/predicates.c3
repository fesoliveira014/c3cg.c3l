module cg::geometry;
import cg;

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

fn PredicateSign in_circle_2d(Vec2f a, Vec2f b, Vec2f c, Vec2f p, float epsilon = DEFAULT_PREDICATE_EPSILON)
{
    float ax = a.x - p.x;
    float ay = a.y - p.y;
    float bx = b.x - p.x;
    float by = b.y - p.y;
    float cx = c.x - p.x;
    float cy = c.y - p.y;

    float aa = ax * ax + ay * ay;
    float bb = bx * bx + by * by;
    float cc = cx * cx + cy * cy;

    float value = ax * (by * cc - bb * cy)
                - ay * (bx * cc - bb * cx)
                + aa * (bx * cy - by * cx);
    return classify_predicate(value, epsilon);
}

fn float det3(
    float a00, float a01, float a02,
    float a10, float a11, float a12,
    float a20, float a21, float a22)
{
    return a00 * (a11 * a22 - a12 * a21)
         - a01 * (a10 * a22 - a12 * a20)
         + a02 * (a10 * a21 - a11 * a20);
}

fn float det4(
    float r00, float r01, float r02, float r03,
    float r10, float r11, float r12, float r13,
    float r20, float r21, float r22, float r23,
    float r30, float r31, float r32, float r33)
{
    return r00 * det3(r11, r12, r13, r21, r22, r23, r31, r32, r33)
         - r01 * det3(r10, r12, r13, r20, r22, r23, r30, r32, r33)
         + r02 * det3(r10, r11, r13, r20, r21, r23, r30, r31, r33)
         - r03 * det3(r10, r11, r12, r20, r21, r22, r30, r31, r32);
}

fn float vec3_dot_for_predicates(Vec3f a, Vec3f b)
{
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

fn Vec3f vec3_sub_for_predicates(Vec3f a, Vec3f b)
{
    return { a.x - b.x, a.y - b.y, a.z - b.z };
}

fn PredicateSign on_sphere(Vec3f a, Vec3f b, Vec3f c, Vec3f d, Vec3f p, float epsilon = DEFAULT_PREDICATE_EPSILON)
{
    Vec3f ar = vec3_sub_for_predicates(a, p);
    Vec3f br = vec3_sub_for_predicates(b, p);
    Vec3f cr = vec3_sub_for_predicates(c, p);
    Vec3f dr = vec3_sub_for_predicates(d, p);

    float a_len = vec3_dot_for_predicates(ar, ar);
    float b_len = vec3_dot_for_predicates(br, br);
    float c_len = vec3_dot_for_predicates(cr, cr);
    float d_len = vec3_dot_for_predicates(dr, dr);

    float value = -det4(
        ar.x, ar.y, ar.z, a_len,
        br.x, br.y, br.z, b_len,
        cr.x, cr.y, cr.z, c_len,
        dr.x, dr.y, dr.z, d_len);

    return classify_predicate(value, epsilon);
}
