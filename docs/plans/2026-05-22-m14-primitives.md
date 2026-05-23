# M14 — Polygon Triangulation, Loop Subdivision, Primitives Implementation Plan

> **For Hermes:** Use subagent-driven-development skill to implement this plan task-by-task.

**Goal:** Add `cg::triangulate` (monotone decomposition triangulation), `cg::subdivide` (Loop subdivision for closed meshes), and `cg::primitives` (platonic solids + icosphere).

**Architecture:** Three independent modules with separate source trees under `src/triangulate/`, `src/subdivide/`, and `src/primitives/`. `cg::primitives::icosphere` depends on `cg::subdivide::loop_subdivide`.

**Spec:** `docs/specs/2026-05-22-m14-primitives-design.md`

**Tech Stack:** C3 0.8.0, c3c, existing `cg`, `cg::half_edge`, `cg::geometry` modules.

**Key existing APIs used:**
- `half_edge::from_triangles(alloc, positions, indices)` → `HalfEdgeMesh?`
- `geometry::orient_2d(a, b, c)` → `PredicateSign`
- `HalfEdgeMesh.face_degree(f)` → `int`
- `HalfEdgeMesh.face_vertices(f, out[])` → `int?`
- `HalfEdgeMesh.face_half_edges(f, out[])` → `int?`
- `HalfEdgeMesh.is_boundary(he)` → `bool`
- `HalfEdgeMesh.to_vertex(he)` → `VertexIndex`
- `HalfEdgeMesh.from_vertex(he)` → `VertexIndex`
- `HalfEdgeMesh.prev(he)` → `HeIndex`
- `HalfEdgeMesh.next(he)` → `HeIndex`
- `HalfEdgeMesh.twin(he)` → `HeIndex`
- `HalfEdgeMesh.face_of(he)` → `FaceIndex`
- `HalfEdgeMesh.vertex_one_ring_outgoing(v, out[])` → `int?`
- `HalfEdgeMesh.validate()` → `void?`
- `HalfEdgeMesh.destroy(&self)`

**Stub-then-replace:** Tasks 1–2 create infrastructure stubs to keep `c3c build debug && c3c test` green at every commit. Tasks 3+ replace stubs with real code, one capability at a time, following TDD (RED → GREEN → commit).

---

## Task 1: Add new faults to umbrella

**Objective:** Register `INSUFFICIENT_VERTICES` and `NON_SIMPLE_POLYGON` before any code fires them.

**Files:**
- Modify: `src/c3cg.c3i`

Change the last two lines of the `faultdef` block from:
```c3
    DUAL_VERTEX_COUNT_MISMATCH,
    NON_CONVEX_BOUNDING_POLYGON;
```
to:
```c3
    DUAL_VERTEX_COUNT_MISMATCH,
    NON_CONVEX_BOUNDING_POLYGON,
    INSUFFICIENT_VERTICES,
    NON_SIMPLE_POLYGON;
```

```bash
c3c build debug && c3c test
git add src/c3cg.c3i
git commit -m "faults: add INSUFFICIENT_VERTICES and NON_SIMPLE_POLYGON"
```

---

## Task 2: Stubs + umbrella declarations

**Objective:** Create placeholder files and umbrella declarations for all 3 modules.

**Files:**
- Create: `src/triangulate/monotone.c3`, `src/subdivide/loop.c3`, `src/primitives/platonic.c3`, `src/primitives/icosphere.c3`
- Modify: `src/c3cg.c3i`

**Step 1: Create `src/triangulate/monotone.c3` stub**
```c3
module cg::triangulate;
import cg;

fn int[]? triangulate_polygon(Allocator alloc, Vec3f[] polygon)
{
    return cg::EMPTY_INPUT~;
}
```

**Step 2: Create `src/subdivide/loop.c3` stub**
```c3
module cg::subdivide;
import cg;

fn HalfEdgeMesh? loop_subdivide(Allocator alloc, HalfEdgeMesh* mesh)
{
    return cg::EMPTY_INPUT~;
}
```

**Step 3: Create `src/primitives/platonic.c3` stub**
```c3
module cg::primitives;
import cg;

fn HalfEdgeMesh? tetrahedron(Allocator alloc) { return cg::EMPTY_INPUT~; }
fn HalfEdgeMesh? octahedron(Allocator alloc) { return cg::EMPTY_INPUT~; }
fn HalfEdgeMesh? icosahedron(Allocator alloc) { return cg::EMPTY_INPUT~; }
fn HalfEdgeMesh? triangulated_cube(Allocator alloc) { return cg::EMPTY_INPUT~; }
```

**Step 4: Create `src/primitives/icosphere.c3` stub**
```c3
module cg::primitives;
import cg;

fn HalfEdgeMesh? icosphere(Allocator alloc, int subdivisions, float radius)
{
    return cg::EMPTY_INPUT~;
}
```

**Step 5: Add to umbrella `src/c3cg.c3i`** — after `module cg::graph;` block:
```c3
module cg::triangulate;
import cg;
fn int[]? triangulate_polygon(Allocator alloc, Vec3f[] polygon);

module cg::subdivide;
import cg;
fn HalfEdgeMesh? loop_subdivide(Allocator alloc, HalfEdgeMesh* mesh);

module cg::primitives;
import cg;
fn HalfEdgeMesh? tetrahedron(Allocator alloc);
fn HalfEdgeMesh? octahedron(Allocator alloc);
fn HalfEdgeMesh? icosahedron(Allocator alloc);
fn HalfEdgeMesh? triangulated_cube(Allocator alloc);
fn HalfEdgeMesh? icosphere(Allocator alloc, int subdivisions, float radius);
```

```bash
c3c build debug && c3c test
git add src/c3cg.c3i src/triangulate/monotone.c3 src/subdivide/loop.c3 src/primitives/platonic.c3 src/primitives/icosphere.c3
git commit -m "triangulate,subdivide,primitives: add module stubs and umbrella declarations"
```

---

## Task 3: Triangulate fault tests (TDD: RED)

**File:** `test/test_triangulate.c3`

```c3
module test;
import cg;
import cg::triangulate;

fn void test_triangulate_insufficient_vertices() @test
{
    Vec3f[2] two_pts = { {0,0,0}, {1,0,0} };
    if (catch err = triangulate::triangulate_polygon(mem, two_pts[..])) {
        assert(err == cg::INSUFFICIENT_VERTICES);
        return;
    }
    unreachable();
}

fn void test_triangulate_duplicate_vertices() @test
{
    Vec3f[3] dup = { {0,0,0}, {1,0,0}, {0,0,0} };
    if (catch err = triangulate::triangulate_polygon(mem, dup[..])) {
        assert(err == cg::NON_SIMPLE_POLYGON);
        return;
    }
    unreachable();
}
```

Expected: FAIL (stub returns EMPTY_INPUT).

```bash
git add test/test_triangulate.c3
git commit -m "test: add triangulate fault tests (RED)"
```

---

## Task 4: triangulate_polygon validation (TDD: GREEN for faults)

Replace `src/triangulate/monotone.c3` with input validation only:

```c3
module cg::triangulate;
import cg;

fn bool same_xy(Vec3f a, Vec3f b)
{
    float dx = a.x - b.x;
    float dy = a.y - b.y;
    return -0.00001f < dx && dx < 0.00001f && -0.00001f < dy && dy < 0.00001f;
}

fn int[]? triangulate_polygon(Allocator alloc, Vec3f[] polygon)
{
    if (polygon.len < 3) return cg::INSUFFICIENT_VERTICES~;

    // Check for any duplicate vertex, adjacent or non-adjacent.
    for (usz i = 0; i < polygon.len; i++) {
        for (usz j = i + 1; j < polygon.len; j++) {
            if (same_xy(polygon[i], polygon[j])) return cg::NON_SIMPLE_POLYGON~;
        }
    }

    // Check for collinear consecutive edges.
    for (usz i = 0; i < polygon.len; i++) {
        usz j = (i + 1) % polygon.len;
        usz k = (i + 2) % polygon.len;
        float dx1 = polygon[j].x - polygon[i].x;
        float dy1 = polygon[j].y - polygon[i].y;
        float dx2 = polygon[k].x - polygon[j].x;
        float dy2 = polygon[k].y - polygon[j].y;
        float cross = dx1 * dy2 - dy1 * dx2;
        if (-0.00001f < cross && cross < 0.00001f) {
            return cg::NON_SIMPLE_POLYGON~;
        }
    }

    return cg::EMPTY_INPUT~;
}
```

Expected: fault tests PASS.

```bash
git add src/triangulate/monotone.c3
git commit -m "triangulate: implement input validation (GREEN for fault tests)"
```

---

## Task 5: Triangle test + full monotone algorithm (TDD: RED→GREEN)

**Step 1: Add triangle test to `test/test_triangulate.c3`** (expect FAIL):

```c3
fn void test_triangulate_triangle() @test
{
    Vec3f[3] tri = { {0,0,0}, {1,0,0}, {0,1,0} };
    int[] indices = triangulate::triangulate_polygon(mem, tri[..])!!;
    defer free(indices);
    assert(indices.len == 3);
    assert(indices[0] == 0);
    assert(indices[1] == 1);
    assert(indices[2] == 2);
}
```

Commit RED:
```bash
git add test/test_triangulate.c3
git commit -m "test: add triangle triangulation test (RED)"
```

**Step 2: Full implementation** — replace `src/triangulate/monotone.c3` with a two-phase triangulation:

1. Sweep-line monotone decomposition:
   - Sort vertices by descending y.
   - Classify each vertex as start, end, split, merge, or regular via `orient_2d`.
   - Sweep with an active edge list (`EdgeNode[]`, sorted linked list with O(n) insertion/search; O(n²) worst case is acceptable).
   - Maintain a helper vertex for each active edge.
   - Add diagonals at split/merge vertices.
2. Triangulate each extracted y-monotone sub-polygon with the stack algorithm.

```c3
module cg::triangulate;
import cg;
import cg::geometry;
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

struct EdgeNode {
    int vertex_a;
    int vertex_b;
    int helper;
    int next;
}

struct SortCtx { Vec3f[] polygon; }
struct DiagonalList { int[] pairs; usz count; }
struct PieceList { int[] values; usz[] offsets; usz[] lengths; usz count; usz cursor; }

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

fn void add_diagonal(DiagonalList* diagonals, int v1, int v2)
{
    if (v1 == v2) return;
    diagonals.pairs[diagonals.count * 2] = v1;
    diagonals.pairs[diagonals.count * 2 + 1] = v2;
    diagonals.count++;
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
    int hprev = (helper + (int)polygon.len - 1) % (int)polygon.len;
    int hnext = (helper + 1) % (int)polygon.len;
    if (classify_vertex(polygon[(usz)hprev], polygon[(usz)helper], polygon[(usz)hnext]) == VertexType.MERGE) {
        add_diagonal(diagonals, vertex, helper);
    }
}

fn void decompose_to_monotone_diagonals(Allocator alloc, Vec3f[] polygon, DiagonalList* diagonals)
{
    int[] order = mem::alloc::new_array(alloc, int, (sz)polygon.len);
    defer free(order);
    for (usz i = 0; i < polygon.len; i++) order[i] = (int)i;
    SortCtx sort_ctx = { polygon };
    sort::quicksort(order, &compare_vertex_y_desc, sort_ctx);

    EdgeNode[] edges = mem::alloc::new_array(alloc, EdgeNode, (sz)(polygon.len * 2));
    defer free(edges);
    int edge_count = 0;
    int active_head = INVALID_EDGE_NODE;

    for (usz oi = 0; oi < order.len; oi++) {
        int v = order[oi];
        int prev = (v + (int)polygon.len - 1) % (int)polygon.len;
        int next = (v + 1) % (int)polygon.len;
        VertexType kind = classify_vertex(polygon[(usz)prev], polygon[(usz)v], polygon[(usz)next]);

        switch (kind) {
            case VertexType.START:
                active_head = insert_active_edge(polygon, edges, &edge_count, active_head, v, next, v, polygon[(usz)v].y);
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
                active_head = insert_active_edge(polygon, edges, &edge_count, active_head, v, next, v, polygon[(usz)v].y);
            case VertexType.MERGE:
                int merge_edge = find_active_edge(edges, active_head, prev, v);
                add_merge_diagonal_if_needed(polygon, edges, merge_edge, v, diagonals);
                active_head = remove_active_edge(edges, active_head, merge_edge);
                int above_edge = find_edge_above(polygon, edges, active_head, polygon[(usz)v].y, polygon[(usz)v].x);
                add_merge_diagonal_if_needed(polygon, edges, above_edge, v, diagonals);
                if (above_edge != INVALID_EDGE_NODE) edges[(usz)above_edge].helper = v;
            case VertexType.REGULAR:
                if (below(polygon[(usz)next], polygon[(usz)v])) {
                    int regular_prev_edge = find_active_edge(edges, active_head, prev, v);
                    add_merge_diagonal_if_needed(polygon, edges, regular_prev_edge, v, diagonals);
                    active_head = remove_active_edge(edges, active_head, regular_prev_edge);
                    active_head = insert_active_edge(polygon, edges, &edge_count, active_head, v, next, v, polygon[(usz)v].y);
                } else {
                    int regular_above_edge = find_edge_above(polygon, edges, active_head, polygon[(usz)v].y, polygon[(usz)v].x);
                    add_merge_diagonal_if_needed(polygon, edges, regular_above_edge, v, diagonals);
                    if (regular_above_edge != INVALID_EDGE_NODE) edges[(usz)regular_above_edge].helper = v;
                }
        }
    }
}

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
        i = (i + 1) % len;
    }
}

fn void split_piece(Allocator alloc, PieceList* pieces, usz piece, usz pos_a, usz pos_b)
{
    usz offset = pieces.offsets[piece];
    usz len = pieces.lengths[piece];
    int[] old_values = mem::alloc::new_array(alloc, int, (sz)len);
    defer free(old_values);
    for (usz i = 0; i < len; i++) old_values[i] = pieces.values[offset + i];

    usz len_a = pos_b >= pos_a ? pos_b - pos_a + 1 : len - pos_a + pos_b + 1;
    usz len_b = pos_a >= pos_b ? pos_a - pos_b + 1 : len - pos_b + pos_a + 1;

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
    usz max_pieces = diagonals.count + 1;
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
        int a = diagonals.pairs[d * 2];
        int b = diagonals.pairs[d * 2 + 1];
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

fn usz append_triangle(Vec3f[] polygon, int[] out_indices, usz out_count, int a, int b, int c)
{
    if (geometry::orient_2d(xy(polygon[(usz)a]), xy(polygon[(usz)b]), xy(polygon[(usz)c])) == geometry::PREDICATE_NEGATIVE) {
        int tmp = b;
        b = c;
        c = tmp;
    }
    out_indices[out_count] = a;
    out_indices[out_count + 1] = b;
    out_indices[out_count + 2] = c;
    return out_count + 3;
}

fn bool same_chain(bool[] chain, int a, int b) { return chain[(usz)a] == chain[(usz)b]; }

fn usz triangulate_monotone_piece(Allocator alloc, Vec3f[] polygon, int[] piece, int[] out_indices, usz out_count)
{
    if (piece.len == 3) return append_triangle(polygon, out_indices, out_count, piece[0], piece[1], piece[2]);

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
        p = (p + 1) % piece.len;
    }

    for (usz i = 0; i < piece.len; i++) sorted[i] = piece[i];
    SortCtx sort_ctx = { polygon };
    sort::quicksort(sorted, &compare_vertex_y_desc, sort_ctx);

    int[] stack = mem::alloc::new_array(alloc, int, (sz)piece.len);
    defer free(stack);
    usz stack_len = 0;
    stack[stack_len++] = sorted[0];
    stack[stack_len++] = sorted[1];

    for (usz i = 2; i + 1 < sorted.len; i++) {
        int curr = sorted[i];
        if (!same_chain(left_chain, curr, stack[stack_len - 1])) {
            while (stack_len > 1) {
                out_count = append_triangle(polygon, out_indices, out_count, curr, stack[stack_len - 1], stack[stack_len - 2]);
                stack_len--;
            }
            stack[0] = sorted[i - 1];
            stack[1] = curr;
            stack_len = 2;
        } else {
            int last = stack[--stack_len];
            while (stack_len > 0) {
                int prev = stack[stack_len - 1];
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

    int bottom = sorted[sorted.len - 1];
    for (usz i = 1; i < stack_len; i++) {
        out_count = append_triangle(polygon, out_indices, out_count, bottom, stack[i - 1], stack[i]);
    }
    return out_count;
}

fn int[]? triangulate_polygon(Allocator alloc, Vec3f[] polygon)
{
    if (polygon.len < 3) return cg::INSUFFICIENT_VERTICES~;
    for (usz i = 0; i < polygon.len; i++) {
        for (usz j = i + 1; j < polygon.len; j++) {
            if (same_xy(polygon[i], polygon[j])) return cg::NON_SIMPLE_POLYGON~;
        }
    }
    for (usz i = 0; i < polygon.len; i++) {
        usz j = (i + 1) % polygon.len;
        usz k = (i + 2) % polygon.len;
        if (geometry::orient_2d(xy(polygon[i]), xy(polygon[j]), xy(polygon[k])) == geometry::PREDICATE_ZERO) {
            return cg::NON_SIMPLE_POLYGON~;
        }
    }

    int[] out_indices = mem::alloc::new_array(alloc, int, (sz)((polygon.len - 2) * 3));
    DiagonalList diagonals;
    diagonals.pairs = mem::alloc::new_array(alloc, int, (sz)(polygon.len * 4));
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
        out_count = triangulate_monotone_piece(alloc, polygon, pieces.values[offset..offset + len], out_indices, out_count);
    }
    return out_indices;
}
```

Expected: triangle, square, pentagon, trapezoid, L-shaped, and star-shaped tests PASS.

```bash
git add src/triangulate/monotone.c3 test/test_triangulate.c3
git commit -m "triangulate: implement full monotone decomposition algorithm (GREEN)"
```

---

## Task 6: More triangulate tests (GREEN)

Add square, pentagon, trapezoid, L-shaped, and star-shaped tests. Expected: all PASS.

```c3
fn void test_triangulate_trapezoid() @test
{
    Vec3f[4] trapezoid = {
        { -2, 0, 0 },
        { 2, 0, 0 },
        { 1, 2, 0 },
        { -1, 2, 0 },
    };
    int[] indices = triangulate::triangulate_polygon(mem, trapezoid[..])!!;
    defer free(indices);
    assert(indices.len == 6);
    for (usz i = 0; i < indices.len; i++) {
        assert(indices[i] >= 0 && (usz)indices[i] < 4);
    }
}

fn void test_triangulate_star_shaped() @test
{
    // 10-vertex star-shaped polygon (CCW)
    Vec3f[10] star = {
        { 0, 3, 0 },
        { 1, 1, 0 },
        { 3, 0, 0 },
        { 1, -1, 0 },
        { 0, -3, 0 },
        { -1, -1, 0 },
        { -3, 0, 0 },
        { -1, 1, 0 },
        { -0.5, 0, 0 },
        { 0.5, 0, 0 },
    };
    int[] indices = triangulate::triangulate_polygon(mem, star[..])!!;
    defer free(indices);
    assert(indices.len == 24);
    for (usz i = 0; i < indices.len; i++) {
        assert(indices[i] >= 0 && (usz)indices[i] < 10);
    }
}
```

```bash
git add test/test_triangulate.c3
git commit -m "test: add square, pentagon, trapezoid, L-shaped, and star-shaped triangulation tests"
```

---

## Task 7: Subdivide fault tests (TDD: RED)

**File:** `test/test_subdivide.c3` — test `BOUNDARY_HALF_EDGE` (single triangle = boundary) and `NON_TRIANGLE_FACE` (quad polygon).

Expected: FAIL. Commit RED.

---

## Task 8: loop_subdivide validation (TDD: GREEN for faults)

Replace `src/subdivide/loop.c3` with validation only: check all faces degree 3, check no boundary edges.

Expected: fault tests PASS.

```bash
git add src/subdivide/loop.c3
git commit -m "subdivide: implement mesh validation (GREEN for fault tests)"
```

---

## Task 9: Tetrahedron test + full Loop subdivision (TDD: RED→GREEN)

**Step 1: Add tetrahedron test** to `test/test_subdivide.c3` — 4 faces → 16, 4 verts → 10.

Expected: FAIL. Commit RED.

**Step 2: Full Loop implementation** — replace `src/subdivide/loop.c3`:

```c3
module cg::subdivide;
import cg;
import cg::half_edge;
import std::math;
```

- Validate triangular + closed
- Edge points: `3/8*(A+B) + 1/8*(C+D)`
- Vertex update: `(1-n*β)*V + β*sum(neighbors)`, β formula
- Build 4 sub-triangles per original face
- Return via `from_triangles`

Expected: tetrahedron test PASSES.

```bash
git add src/subdivide/loop.c3 test/test_subdivide.c3
git commit -m "subdivide: implement Loop subdivision algorithm (GREEN)"
```

---

## Task 10: Remaining subdivide tests (GREEN)

Add icosahedron subdivide (20→80 faces), original mesh unchanged, closed stays closed tests.

```bash
git add test/test_subdivide.c3
git commit -m "test: add icosahedron subdivide, mesh unchanged, and closed tests"
```

---

## Task 11: Primitives implementation + tests (TDD: RED→GREEN)

**Step 1: Write all tests** in `test/test_primitives.c3`:
- tetrahedron (4 faces, unit sphere)
- octahedron (8 faces)
- icosahedron (20 faces)
- triangulated_cube (12 faces, ±1)
- icosphere subdiv 0 (same as icosahedron)
- icosphere subdiv 2 (320 faces, radius 5.0)
- degenerate subdivision / radius → DEGENERATE_INPUT

Expected: all FAIL. Commit RED.

**Step 2: Implement platonic.c3** — hardcoded vertices + indices for all 4 shapes via `from_triangles`; every primitive returns `HalfEdgeMesh?` and propagates `from_triangles` faults.

```c3
module cg::primitives;
import cg;
import cg::half_edge;
import std::math;
```

**Step 3: Implement icosphere.c3** — `icosahedron` → `loop_subdivide` N times → reproject; `icosphere` returns `HalfEdgeMesh?`.

```c3
module cg::primitives;
import cg;
import cg::subdivide;
import std::math;
```

Expected: all primitives tests PASS.

```bash
git add src/primitives/platonic.c3 src/primitives/icosphere.c3 test/test_primitives.c3
git commit -m "primitives: implement all platonic solids and icosphere"
```

---

## Task 12: Final verification

```bash
c3c clean && c3c build release && c3c build debug && c3c test
```

Expected: 22 new tests PASS, all existing tests still pass. Green across all targets.

---

## Potential issues

1. **sort::quicksort comparator signature**: Must be `fn int cmp(int a, int b, SortCtx ctx)` with context LAST.
2. **usz arithmetic**: All array index arithmetic needs explicit `(usz)` and `(int)` casts.
3. **orient_2d signature**: Returns `PredicateSign` enum — check for exact enum value `PREDICATE_POSITIVE`/`PREDICATE_NEGATIVE`.
4. **Loop β formula NaN**: `cos(2π/n)` for n=0 requires guard; the `n <= 2` branch avoids it.
5. **vertex_one_ring_outgoing**: Returns number of outgoing half-edges from a vertex (equals degree for closed meshes).
6. **Icosphere reprojection**: `radius / len` not `radius * len`.
7. **Cube winding**: Must be outward-facing for `from_triangles` to not fault.
8. **Monotone decomposition invariants**: Verify every inserted split/merge diagonal is non-crossing, belongs to one current piece, and the extracted pieces are y-monotone before stack triangulation.
