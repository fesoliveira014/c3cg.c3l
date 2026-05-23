module cg::triangulate;
import cg;
import cg::geometry;

const float TRIANGULATE_EPSILON = 0.00001f;

fn bool same_xy(Vec3f a, Vec3f b)
{
    float dx = a.x - b.x;
    float dy = a.y - b.y;
    return -TRIANGULATE_EPSILON < dx && dx < TRIANGULATE_EPSILON && -TRIANGULATE_EPSILON < dy && dy < TRIANGULATE_EPSILON;
}

fn int[]? triangulate_polygon(Allocator alloc, Vec3f[] polygon)
{
    if (polygon.len < 3) return cg::INSUFFICIENT_VERTICES~;

    for (usz i = 0; i < polygon.len; i++) {
        for (usz j = i + (usz)1; j < polygon.len; j++) {
            if (same_xy(polygon[i], polygon[j])) return cg::NON_SIMPLE_POLYGON~;
        }
    }

    for (usz i = 0; i < polygon.len; i++) {
        usz j = i + (usz)1;
        if (j >= polygon.len) j -= polygon.len;
        usz k = i + (usz)2;
        if (k >= polygon.len) k -= polygon.len;
        float dx1 = polygon[j].x - polygon[i].x;
        float dy1 = polygon[j].y - polygon[i].y;
        float dx2 = polygon[k].x - polygon[j].x;
        float dy2 = polygon[k].y - polygon[j].y;
        float cross = dx1 * dy2 - dy1 * dx2;
        if (-TRIANGULATE_EPSILON < cross && cross < TRIANGULATE_EPSILON) {
            return cg::NON_SIMPLE_POLYGON~;
        }
    }

    return cg::EMPTY_INPUT~;
}
