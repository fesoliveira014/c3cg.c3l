module cg::triangulate;
import cg;
import cg::geometry;
import cg::half_edge;
import std::sort;

const float TRIANGULATE_EPSILON = 0.00001f;
const int INVALID_EDGE_NODE = -1;

enum VertexType {
    START,
    END,
    SPLIT,
    MERGE,
    REGULAR,
}

fn bool same_xy(Vec3f a, Vec3f b)
{
    float dx = a.x - b.x;
    float dy = a.y - b.y;
    return -TRIANGULATE_EPSILON < dx && dx < TRIANGULATE_EPSILON
        && -TRIANGULATE_EPSILON < dy && dy < TRIANGULATE_EPSILON;
}

fn Vec2f xy(Vec3f p) { return { p.x, p.y }; }

fn bool above(Vec3f a, Vec3f b)
{
    if (a.y > b.y + TRIANGULATE_EPSILON) return true;
    if (a.y < b.y - TRIANGULATE_EPSILON) return false;
    return a.x < b.x;
}

fn bool below(Vec3f a, Vec3f b) { return above(b, a); }

struct SortCtx { Vec3f[] polygon; }

fn int compare_vertex_y_desc(int a, int b, SortCtx ctx)
{
    Vec3f pa = ctx.polygon[(usz)a];
    Vec3f pb = ctx.polygon[(usz)b];
    if (above(pa, pb)) return -1;
    if (above(pb, pa)) return 1;
    return 0;
}

fn float edge_x_at_y(Vec3f a, Vec3f b, float y)
{
    float dy = b.y - a.y;
    if (-TRIANGULATE_EPSILON < dy && dy < TRIANGULATE_EPSILON) return a.x < b.x ? a.x : b.x;
    float t = (y - a.y) / dy;
    return a.x + t * (b.x - a.x);
}

fn VertexType classify_vertex(Vec3f prev, Vec3f curr, Vec3f next)
{
    PredicateSign turn = geometry::orient_2d(xy(prev), xy(curr), xy(next));
    bool prev_below = below(prev, curr);
    bool next_below = below(next, curr);
    bool prev_above = above(prev, curr);
    bool next_above = above(next, curr);

    if (prev_below && next_below) {
        if (turn == geometry::PREDICATE_POSITIVE) return VertexType.START;
        return VertexType.SPLIT;
    }
    if (prev_above && next_above) {
        if (turn == geometry::PREDICATE_POSITIVE) return VertexType.END;
        return VertexType.MERGE;
    }
    return VertexType.REGULAR;
}

struct DiagonalList { int[] pairs; usz count; }

fn void add_diagonal(DiagonalList* diagonals, int v1, int v2)
{
    if (v1 == v2) return;
    diagonals.pairs[diagonals.count * (usz)2] = v1;
    diagonals.pairs[diagonals.count * (usz)2 + (usz)1] = v2;
    diagonals.count++;
}

struct EdgeNode {
    int vertex_a;
    int vertex_b;
    int helper;
    int next;
}

fn bool edge_matches(EdgeNode e, int a, int b)
{
    return (e.vertex_a == a && e.vertex_b == b) || (e.vertex_a == b && e.vertex_b == a);
}

fn int find_active_edge(EdgeNode[] edges, int active_head, int a, int b)
{
    int node = active_head;
    while (node != INVALID_EDGE_NODE) {
        if (edge_matches(edges[(usz)node], a, b)) return node;
        node = edges[(usz)node].next;
    }
    return INVALID_EDGE_NODE;
}

fn int find_edge_above(Vec3f[] polygon, EdgeNode[] edges, int active_head, float vertex_y, float vertex_x)
{
    int best = INVALID_EDGE_NODE;
    float best_x = -1000000000.0f;
    int node = active_head;
    while (node != INVALID_EDGE_NODE) {
        EdgeNode e = edges[(usz)node];
        float x = edge_x_at_y(polygon[(usz)e.vertex_a], polygon[(usz)e.vertex_b], vertex_y);
        if (x < vertex_x && x > best_x) {
            best = node;
            best_x = x;
        }
        node = e.next;
    }
    return best;
}

fn int insert_active_edge(Vec3f[] polygon, EdgeNode[] edges, int* edge_count, int active_head, int a, int b, int helper, float sweep_y)
{
    int id = *edge_count;
    *edge_count = id + 1;
    edges[(usz)id] = { a, b, helper, INVALID_EDGE_NODE };

    float x = edge_x_at_y(polygon[(usz)a], polygon[(usz)b], sweep_y);
    if (active_head == INVALID_EDGE_NODE) return id;

    float head_x = edge_x_at_y(polygon[(usz)edges[(usz)active_head].vertex_a], polygon[(usz)edges[(usz)active_head].vertex_b], sweep_y);
    if (x < head_x) {
        edges[(usz)id].next = active_head;
        return id;
    }

    int prev = active_head;
    int curr = edges[(usz)prev].next;
    while (curr != INVALID_EDGE_NODE) {
        float curr_x = edge_x_at_y(polygon[(usz)edges[(usz)curr].vertex_a], polygon[(usz)edges[(usz)curr].vertex_b], sweep_y);
        if (x < curr_x) break;
        prev = curr;
        curr = edges[(usz)curr].next;
    }
    edges[(usz)id].next = curr;
    edges[(usz)prev].next = id;
    return active_head;
}

fn int remove_active_edge(EdgeNode[] edges, int active_head, int node)
{
    if (node == INVALID_EDGE_NODE) return active_head;
    if (node == active_head) return edges[(usz)node].next;
    int prev = active_head;
    while (prev != INVALID_EDGE_NODE && edges[(usz)prev].next != node) {
        prev = edges[(usz)prev].next;
    }
    if (prev != INVALID_EDGE_NODE) edges[(usz)prev].next = edges[(usz)node].next;
    return active_head;
}

fn void add_merge_diagonal_if_needed(Vec3f[] polygon, EdgeNode[] edges, int node, int vertex, DiagonalList* diagonals)
{
    if (node == INVALID_EDGE_NODE) return;
    int helper = edges[(usz)node].helper;
    int n = (int)polygon.len;
    int hprev = helper - 1;
    if (hprev < 0) hprev += n;
    int hnext = helper + 1;
    if (hnext >= n) hnext -= n;
    if (classify_vertex(polygon[(usz)hprev], polygon[(usz)helper], polygon[(usz)hnext]) == VertexType.MERGE) {
        add_diagonal(diagonals, vertex, helper);
    }
}

fn void decompose_to_monotone_diagonals(Allocator alloc, Vec3f[] polygon, DiagonalList* diagonals)
{
    int n = (int)polygon.len;
    int[] order = mem::alloc::new_array(alloc, int, (sz)n);
    defer free(order);
    for (usz i = 0; i < polygon.len; i++) order[i] = (int)i;
    SortCtx sort_ctx = { polygon };
    sort::quicksort(order, &compare_vertex_y_desc, sort_ctx);

    EdgeNode[] edges = mem::alloc::new_array(alloc, EdgeNode, (sz)(polygon.len * (usz)2));
    defer free(edges);
    int edge_count = 0;
    int active_head = INVALID_EDGE_NODE;

    for (usz oi = 0; oi < order.len; oi++) {
        int v = order[oi];
        int prev = v - 1;
        if (prev < 0) prev += n;
        int next_idx = v + 1;
        if (next_idx >= n) next_idx -= n;
        VertexType kind = classify_vertex(polygon[(usz)prev], polygon[(usz)v], polygon[(usz)next_idx]);

        switch (kind) {
            case VertexType.START:
                active_head = insert_active_edge(polygon, edges, &edge_count, active_head, v, next_idx, v, polygon[(usz)v].y);
            case VertexType.END:
                int end_edge = find_active_edge(edges, active_head, prev, v);
                add_merge_diagonal_if_needed(polygon, edges, end_edge, v, diagonals);
                active_head = remove_active_edge(edges, active_head, end_edge);
            case VertexType.SPLIT:
                int split_edge = find_edge_above(polygon, edges, active_head, polygon[(usz)v].y, polygon[(usz)v].x);
                if (split_edge != INVALID_EDGE_NODE) {
                    add_diagonal(diagonals, v, edges[(usz)split_edge].helper);
                    edges[(usz)split_edge].helper = v;
                }
                active_head = insert_active_edge(polygon, edges, &edge_count, active_head, v, next_idx, v, polygon[(usz)v].y);
            case VertexType.MERGE:
                int merge_edge = find_active_edge(edges, active_head, prev, v);
                add_merge_diagonal_if_needed(polygon, edges, merge_edge, v, diagonals);
                active_head = remove_active_edge(edges, active_head, merge_edge);
                int above_edge = find_edge_above(polygon, edges, active_head, polygon[(usz)v].y, polygon[(usz)v].x);
                add_merge_diagonal_if_needed(polygon, edges, above_edge, v, diagonals);
                if (above_edge != INVALID_EDGE_NODE) edges[(usz)above_edge].helper = v;
            case VertexType.REGULAR:
                if (below(polygon[(usz)next_idx], polygon[(usz)v])) {
                    int regular_prev_edge = find_active_edge(edges, active_head, prev, v);
                    add_merge_diagonal_if_needed(polygon, edges, regular_prev_edge, v, diagonals);
                    active_head = remove_active_edge(edges, active_head, regular_prev_edge);
                    active_head = insert_active_edge(polygon, edges, &edge_count, active_head, v, next_idx, v, polygon[(usz)v].y);
                } else {
                    int regular_above_edge = find_edge_above(polygon, edges, active_head, polygon[(usz)v].y, polygon[(usz)v].x);
                    add_merge_diagonal_if_needed(polygon, edges, regular_above_edge, v, diagonals);
                    if (regular_above_edge != INVALID_EDGE_NODE) edges[(usz)regular_above_edge].helper = v;
                }
        }
    }
}

struct PieceList { int[] values; usz[] offsets; usz[] lengths; usz count; usz cursor; }

fn bool piece_contains(PieceList* pieces, usz piece, int vertex, usz* pos)
{
    usz offset = pieces.offsets[piece];
    usz len = pieces.lengths[piece];
    for (usz i = 0; i < len; i++) {
        if (pieces.values[offset + i] == vertex) {
            *pos = i;
            return true;
        }
    }
    return false;
}

fn void copy_piece_chain(PieceList* pieces, usz dst_offset, int[] src, usz len, usz from, usz to)
{
    usz out = 0;
    usz i = from;
    while (true) {
        pieces.values[dst_offset + out] = src[i];
        out++;
        if (i == to) break;
        i++;
        if (i >= len) i -= len;
    }
}

fn void split_piece(Allocator alloc, PieceList* pieces, usz piece, usz pos_a, usz pos_b)
{
    usz offset = pieces.offsets[piece];
    usz len = pieces.lengths[piece];
    int[] old_values = mem::alloc::new_array(alloc, int, (sz)len);
    defer free(old_values);
    for (usz i = 0; i < len; i++) old_values[i] = pieces.values[offset + i];

    usz len_a;
    if (pos_b >= pos_a) {
        len_a = pos_b - pos_a + (usz)1;
    } else {
        len_a = len - pos_a + pos_b + (usz)1;
    }
    usz len_b;
    if (pos_a >= pos_b) {
        len_b = pos_a - pos_b + (usz)1;
    } else {
        len_b = len - pos_b + pos_a + (usz)1;
    }

    usz new_offset = pieces.cursor;
    copy_piece_chain(pieces, offset, old_values, len, pos_a, pos_b);
    copy_piece_chain(pieces, new_offset, old_values, len, pos_b, pos_a);
    pieces.lengths[piece] = len_a;
    pieces.offsets[pieces.count] = new_offset;
    pieces.lengths[pieces.count] = len_b;
    pieces.count++;
    pieces.cursor += len_b;
}

fn PieceList extract_monotone_sub_polygons(Allocator alloc, Vec3f[] polygon, DiagonalList* diagonals)
{
    usz max_pieces = diagonals.count + (usz)1;
    PieceList pieces;
    pieces.values = mem::alloc::new_array(alloc, int, (sz)(polygon.len * max_pieces));
    pieces.offsets = mem::alloc::new_array(alloc, usz, (sz)max_pieces);
    pieces.lengths = mem::alloc::new_array(alloc, usz, (sz)max_pieces);
    pieces.count = 1;
    pieces.cursor = polygon.len;
    pieces.offsets[0] = 0;
    pieces.lengths[0] = polygon.len;
    for (usz i = 0; i < polygon.len; i++) pieces.values[i] = (int)i;

    for (usz d = 0; d < diagonals.count; d++) {
        int a = diagonals.pairs[d * (usz)2];
        int b = diagonals.pairs[d * (usz)2 + (usz)1];
        for (usz p = 0; p < pieces.count; p++) {
            usz pos_a = 0;
            usz pos_b = 0;
            if (piece_contains(&pieces, p, a, &pos_a) && piece_contains(&pieces, p, b, &pos_b)) {
                split_piece(alloc, &pieces, p, pos_a, pos_b);
                break;
            }
        }
    }
    return pieces;
}

fn usz append_triangle(Vec3f[] polygon, uint[] out_indices, usz out_count, int a, int b, int c)
{
    if (geometry::orient_2d(xy(polygon[(usz)a]), xy(polygon[(usz)b]), xy(polygon[(usz)c])) == geometry::PREDICATE_NEGATIVE) {
        int tmp = b;
        b = c;
        c = tmp;
    }
    out_indices[out_count] = (uint)a;
    out_indices[out_count + (usz)1] = (uint)b;
    out_indices[out_count + (usz)2] = (uint)c;
    return out_count + (usz)3;
}

fn bool same_chain(bool[] chain, int a, int b) { return chain[(usz)a] == chain[(usz)b]; }

fn usz triangulate_monotone_piece(Allocator alloc, Vec3f[] polygon, int[] piece, uint[] out_indices, usz out_count)
{
    if (piece.len == (usz)3) return append_triangle(polygon, out_indices, out_count, piece[0], piece[1], piece[2]);

    int[] sorted = mem::alloc::new_array(alloc, int, (sz)piece.len);
    defer free(sorted);
    bool[] left_chain = mem::alloc::new_array(alloc, bool, (sz)polygon.len);
    defer free(left_chain);

    int top_pos = 0;
    int bottom_pos = 0;
    for (usz i = 1; i < piece.len; i++) {
        if (above(polygon[(usz)piece[i]], polygon[(usz)piece[(usz)top_pos]])) top_pos = (int)i;
        if (below(polygon[(usz)piece[i]], polygon[(usz)piece[(usz)bottom_pos]])) bottom_pos = (int)i;
    }

    usz p = (usz)top_pos;
    while (true) {
        left_chain[(usz)piece[p]] = true;
        if (p == (usz)bottom_pos) break;
        p++;
        if (p >= piece.len) p -= piece.len;
    }

    for (usz i = 0; i < piece.len; i++) sorted[i] = piece[i];
    SortCtx sort_ctx = { polygon };
    sort::quicksort(sorted, &compare_vertex_y_desc, sort_ctx);

    int[] stack = mem::alloc::new_array(alloc, int, (sz)piece.len);
    defer free(stack);
    usz stack_len = 0;
    stack[stack_len++] = sorted[0];
    stack[stack_len++] = sorted[1];

    for (usz i = 2; i + (usz)1 < sorted.len; i++) {
        int curr = sorted[i];
        if (!same_chain(left_chain, curr, stack[stack_len - (usz)1])) {
            while (stack_len > 1) {
                out_count = append_triangle(polygon, out_indices, out_count, curr, stack[(int)(stack_len - (usz)1)], stack[(int)(stack_len - (usz)2)]);
                stack_len--;
            }
            stack[0] = sorted[i - (usz)1];
            stack[1] = curr;
            stack_len = 2;
        } else {
            int last = stack[(int)(stack_len - (usz)1)];
            stack_len--;
            while (stack_len > 0) {
                int prev = stack[(int)(stack_len - (usz)1)];
                PredicateSign turn = geometry::orient_2d(xy(polygon[(usz)curr]), xy(polygon[(usz)last]), xy(polygon[(usz)prev]));
                bool can_emit = left_chain[(usz)curr] ? turn == geometry::PREDICATE_NEGATIVE : turn == geometry::PREDICATE_POSITIVE;
                if (!can_emit) break;
                out_count = append_triangle(polygon, out_indices, out_count, curr, last, prev);
                last = prev;
                stack_len--;
            }
            stack[stack_len++] = last;
            stack[stack_len++] = curr;
        }
    }

    int bottom = sorted[sorted.len - (usz)1];
    for (usz i = 1; i < stack_len; i++) {
        out_count = append_triangle(polygon, out_indices, out_count, bottom, stack[i - (usz)1], stack[i]);
    }
    return out_count;
}

fn HalfEdgeMesh? triangulate_polygon(Allocator alloc, Vec3f[] polygon)
{
    if (polygon.len < (usz)3) return cg::INSUFFICIENT_VERTICES~;

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
        if (geometry::orient_2d(xy(polygon[i]), xy(polygon[j]), xy(polygon[k])) == geometry::PREDICATE_ZERO) {
            return cg::NON_SIMPLE_POLYGON~;
        }
    }

    int n = (int)polygon.len;
    uint[] out_indices = mem::alloc::new_array(alloc, uint, (sz)((n - 2) * 3));
    defer free(out_indices);

    DiagonalList diagonals;
    diagonals.pairs = mem::alloc::new_array(alloc, int, (sz)(n * 4));
    diagonals.count = 0;
    defer free(diagonals.pairs);

    decompose_to_monotone_diagonals(alloc, polygon, &diagonals);
    PieceList pieces = extract_monotone_sub_polygons(alloc, polygon, &diagonals);
    defer free(pieces.values);
    defer free(pieces.offsets);
    defer free(pieces.lengths);

    usz out_count = 0;
    for (usz p = 0; p < pieces.count; p++) {
        usz offset = pieces.offsets[p];
        usz len = pieces.lengths[p];
        out_count = triangulate_monotone_piece(alloc, polygon, pieces.values[offset : len], out_indices, out_count);
    }
    return half_edge::from_triangles(alloc, polygon, out_indices[:out_count]);
}
