module cg::voronoi;
import cg;
import cg::geometry;
import cg::half_edge;
import cg::delaunay;
import std::math;

// ── Private helpers ──

fn Vec3f? line_intersection(Vec2f a, Vec2f b, Vec2f p, Vec2f q)
{
    float d1x = b.x - a.x;
    float d1y = b.y - a.y;
    float d2x = q.x - p.x;
    float d2y = q.y - p.y;
    float det = d1x * d2y - d1y * d2x;
    if (math::abs(det) < geometry::GEOMETRY_EPSILON) return {};

    float t = ((p.x - a.x) * d2y - (p.y - a.y) * d2x) / det;
    return { a.x + t * d1x, a.y + t * d1y, 0 };
}

fn Vec3f[]? sutherland_hodgman(Allocator alloc, Vec3f[] polygon, Vec3f[] clip_polygon)
{
    usz cap = polygon.len * 4;
    Vec3f[] input = mem::alloc::new_array(alloc, Vec3f, (sz)cap);
    defer free(input);
    Vec3f[] output = mem::alloc::new_array(alloc, Vec3f, (sz)cap);
    defer free(output);
    for (usz i = 0; i < polygon.len; i++) input[i] = polygon[i];
    usz input_len = polygon.len;

    for (usz e = 0; e < clip_polygon.len; e++) {
        Vec3f a = clip_polygon[e];
        Vec3f b = clip_polygon[(e + 1) % (usz)clip_polygon.len];
        Vec2f a2 = { a.x, a.y };
        Vec2f b2 = { b.x, b.y };

        usz out_len = 0;
        for (usz i = 0; i < input_len; i++) {
            Vec3f p = input[i];
            Vec3f q = input[(i + 1) % (usz)input_len];
            Vec2f p2 = { p.x, p.y };
            Vec2f q2 = { q.x, q.y };

            PredicateSign p_sign = geometry::orient_2d(a2, b2, p2);
            PredicateSign q_sign = geometry::orient_2d(a2, b2, q2);
            bool p_in = p_sign >= geometry::PREDICATE_ZERO;
            bool q_in = q_sign >= geometry::PREDICATE_ZERO;

            if (p_in && q_in) {
                output[out_len] = q; out_len++;
            } else if (p_in && !q_in) {
                Vec3f? isect = line_intersection(a2, b2, p2, q2);
                if (try iv = isect) { output[out_len] = iv; out_len++; }
            } else if (!p_in && q_in) {
                Vec3f? isect = line_intersection(a2, b2, p2, q2);
                if (try iv = isect) { output[out_len] = iv; out_len++; }
                output[out_len] = q; out_len++;
            }
        }

        if (out_len < 3) {
            Vec3f[] empty = {};
            return empty;
        }

        for (usz i = 0; i < out_len; i++) input[i] = output[i];
        input_len = out_len;
    }

    Vec3f[] result = mem::alloc::new_array(alloc, Vec3f, (sz)input_len);
    defer catch free(result);
    for (usz i = 0; i < input_len; i++) result[i] = input[i];
    return result;
}

fn Vec3f[]? half_plane_clip(Allocator alloc, Vec3f[] polygon, Vec3f a, Vec3f b, bool keep_a_side)
{
    Vec3f m = { (a.x + b.x) / 2, (a.y + b.y) / 2, 0 };
    Vec3f d = { b.x - a.x, b.y - a.y, 0 };
    float a_dot = (a.x - m.x) * d.x + (a.y - m.y) * d.y;

    usz cap = polygon.len * 2;
    Vec3f[] input = mem::alloc::new_array(alloc, Vec3f, (sz)cap);
    defer free(input);
    Vec3f[] output = mem::alloc::new_array(alloc, Vec3f, (sz)cap);
    defer free(output);
    for (usz i = 0; i < polygon.len; i++) input[i] = polygon[i];
    usz input_len = polygon.len;

    usz out_len = 0;
    for (usz i = 0; i < input_len; i++) {
        Vec3f p = input[i];
        Vec3f q = input[(i + 1) % (usz)input_len];
        float p_dot = (p.x - m.x) * d.x + (p.y - m.y) * d.y;
        float q_dot = (q.x - m.x) * d.x + (q.y - m.y) * d.y;
        bool p_in = keep_a_side ? (p_dot * a_dot >= 0) : (p_dot * a_dot <= 0);
        bool q_in = keep_a_side ? (q_dot * a_dot >= 0) : (q_dot * a_dot <= 0);

        if (p_in && q_in) {
            output[out_len] = q; out_len++;
        } else if (p_in && !q_in) {
            float t = p_dot / (p_dot - q_dot);
            output[out_len] = { p.x + t * (q.x - p.x), p.y + t * (q.y - p.y), 0 };
            out_len++;
        } else if (!p_in && q_in) {
            float t = p_dot / (p_dot - q_dot);
            output[out_len] = { p.x + t * (q.x - p.x), p.y + t * (q.y - p.y), 0 };
            out_len++;
            output[out_len] = q; out_len++;
        }
    }

    if (out_len < 3) {
        Vec3f[] empty = {};
        return empty;
    }

    Vec3f[] result = mem::alloc::new_array(alloc, Vec3f, (sz)out_len);
    defer catch free(result);
    for (usz i = 0; i < out_len; i++) result[i] = output[i];
    return result;
}

fn bool point_in_convex_polygon(Vec3f p, Vec3f[] polygon)
{
    for (usz e = 0; e < polygon.len; e++) {
        Vec3f a = polygon[e];
        Vec3f b = polygon[(e + 1) % (usz)polygon.len];
        if (geometry::orient_2d({a.x,a.y}, {b.x,b.y}, {p.x,p.y}) < geometry::PREDICATE_ZERO) return false;
    }
    return true;
}

fn bool is_convex_polygon(Vec3f[] polygon)
{
    if (polygon.len < 3) return false;
    bool has_strict = false;
    for (usz i = 0; i < polygon.len; i++) {
        Vec3f p = polygon[i];
        Vec3f q = polygon[(i + 1) % (usz)polygon.len];
        Vec3f r = polygon[(i + 2) % (usz)polygon.len];
        PredicateSign sign = geometry::orient_2d({p.x,p.y}, {q.x,q.y}, {r.x,r.y});
        if (sign < geometry::PREDICATE_ZERO) return false;
        if (sign > geometry::PREDICATE_ZERO) has_strict = true;
    }
    return has_strict;
}
