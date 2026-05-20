module cg::hull;
import cg;
import cg::geometry;
import std::sort;

struct SortCtx {
    Vec3f[] positions;
}

fn int compare_by_position(int a, int b, SortCtx ctx)
{
    Vec3f va = ctx.positions[a];
    Vec3f vb = ctx.positions[b];
    if (va.x < vb.x) return -1;
    if (va.x > vb.x) return 1;
    if (va.y < vb.y) return -1;
    if (va.y > vb.y) return 1;
    return 0;
}

fn int[]? hull_2d(Allocator alloc, Vec3f[] positions)
{
    if (positions.len == 0) return cg::EMPTY_INPUT~;

    // Phase 1 — index array + lexicographic sort by (x, y).
    int[] indices = mem::alloc::new_array(alloc, int, (sz) positions.len);
    defer catch free(indices);
    for (usz i = 0; i < positions.len; i++) indices[i] = (int)i;

    SortCtx sort_ctx = { positions };
    sort::quicksort(indices, &compare_by_position, sort_ctx);

    // Phase 2 — lower hull (left-to-right sweep).
    // Pop only on strict CW (PREDICATE_NEGATIVE), keeping collinear boundary points.
    int[] hull = mem::alloc::new_array(alloc, int, (sz)(2 * positions.len));
    defer catch free(hull);
    usz hull_len = 0;

    for (usz i = 0; i < indices.len; i++) {
        int p = indices[i];
        Vec2f p2 = { positions[p].x, positions[p].y };
        while (hull_len >= 2) {
            int a = hull[hull_len - 2];
            int b = hull[hull_len - 1];
            Vec2f a2 = { positions[a].x, positions[a].y };
            Vec2f b2 = { positions[b].x, positions[b].y };
            if (geometry::orient_2d(a2, b2, p2, 0.0f) != geometry::PREDICATE_NEGATIVE) break;
            hull_len--;
        }
        hull[hull_len] = p;
        hull_len++;
    }

    // Phase 3 — upper hull (right-to-left sweep, skip rightmost point).
    for (int i = (int)indices.len - 1; i > 0; i--) {
        int p = indices[i - 1];
        Vec2f p2 = { positions[p].x, positions[p].y };
        while (hull_len >= 2) {
            int a = hull[hull_len - 2];
            int b = hull[hull_len - 1];
            Vec2f a2 = { positions[a].x, positions[a].y };
            Vec2f b2 = { positions[b].x, positions[b].y };
            if (geometry::orient_2d(a2, b2, p2, 0.0f) != geometry::PREDICATE_NEGATIVE) break;
            hull_len--;
        }
        hull[hull_len] = p;
        hull_len++;
    }

    // Phase 4 — trim last element (duplicate of hull[0]) and validate.
    if (hull_len <= 1) return cg::DEGENERATE_INPUT~;
    usz result_len = hull_len - 1;
    if (result_len < 3) return cg::DEGENERATE_INPUT~;
    if (result_len > (usz)positions.len) return cg::DEGENERATE_INPUT~;

    int[] result = mem::alloc::new_array(alloc, int, (sz) result_len);
    defer catch free(result);
    for (usz i = 0; i < result_len; i++) result[i] = hull[i];

    free(indices);
    free(hull);
    return result;
}
