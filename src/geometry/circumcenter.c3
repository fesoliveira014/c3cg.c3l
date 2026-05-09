module cg::geometry;
import cg;
import cg::half_edge;
import std::math;

fn Vec3f vec3_add(Vec3f a, Vec3f b)
{
    return { a.x + b.x, a.y + b.y, a.z + b.z };
}

fn Vec3f vec3_sub(Vec3f a, Vec3f b)
{
    return { a.x - b.x, a.y - b.y, a.z - b.z };
}

fn Vec3f vec3_scale(Vec3f v, float scale)
{
    return { v.x * scale, v.y * scale, v.z * scale };
}

fn float vec3_dot(Vec3f a, Vec3f b)
{
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

fn Vec3f vec3_cross(Vec3f a, Vec3f b)
{
    return {
        a.y * b.z - a.z * b.y,
        a.z * b.x - a.x * b.z,
        a.x * b.y - a.y * b.x,
    };
}

fn float vec3_length(Vec3f v)
{
    return (float)math::sqrt((double)vec3_dot(v, v));
}

fn Vec3f? circumcenter_planar(Vec3f a, Vec3f b, Vec3f c)
{
    Vec3f u = vec3_sub(b, a);
    Vec3f v = vec3_sub(c, a);
    Vec3f n = vec3_cross(u, v);
    float n_len_sq = vec3_dot(n, n);
    if (n_len_sq <= GEOMETRY_EPSILON * GEOMETRY_EPSILON) return cg::DEGENERATE_INPUT~;

    float uu = vec3_dot(u, u);
    float vv = vec3_dot(v, v);
    Vec3f term_u = vec3_scale(vec3_cross(n, u), vv);
    Vec3f term_v = vec3_scale(vec3_cross(v, n), uu);
    return vec3_add(a, vec3_scale(vec3_add(term_u, term_v), 1.0f / (2.0f * n_len_sq)));
}

fn Vec3f? circumcenter_on_sphere(Vec3f a, Vec3f b, Vec3f c, float radius)
{
    if (radius <= GEOMETRY_EPSILON) return cg::DEGENERATE_INPUT~;

    Vec3f planar = circumcenter_planar(a, b, c)!;
    float len = vec3_length(planar);
    if (len <= GEOMETRY_EPSILON) return cg::DEGENERATE_INPUT~;

    return vec3_scale(planar, radius / len);
}
