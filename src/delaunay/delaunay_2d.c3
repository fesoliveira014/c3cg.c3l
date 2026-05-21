module cg::delaunay;
import cg;
import cg::geometry;
import cg::half_edge;
import std::sort;
import std::math;

const float DELAUNAY_NEAR_DUP_EPSILON = 1e-12f;

struct DelaunayBuilder {
    VertexIndex[] triangles;
    HeIndex[] halfedges;
    HeIndex[] hull_tri;
    VertexIndex[] hull_prev;
    VertexIndex[] hull_next;
    VertexIndex[] hull_hash;
    HeIndex[] legalize_stack;
    Vec3f hull_center;
    VertexIndex hull_start;
    usz face_count;
}

struct DistSortCtx { Vec3f[] pts; Vec3f center; }

fn HeIndex next_halfedge(HeIndex i) { return (i % 3 == 2) ? i - 2 : i + 1; }
fn HeIndex prev_halfedge(HeIndex i) { return (i % 3 == 0) ? i + 2 : i - 1; }
fn usz fast_mod(usz i, usz c) { return i >= c ? i % c : i; }

fn void link_halfedges(HeIndex[] halfedges, HeIndex a, HeIndex b)
{
    halfedges[a] = b;
    if (b != cg::INVALID_HE) {
        halfedges[b] = a;
    }
}

fn float pseudo_angle(float dx, float dy)
{
    float denom = (dx < 0 ? -dx : dx) + (dy < 0 ? -dy : dy);
    float p = dx / denom;
    if (dy > 0.0f) { return (3.0f - p) / 4.0f; }
    return (1.0f + p) / 4.0f;
}

fn int compare_by_distance(int a, int b, DistSortCtx ctx)
{
    float da = (ctx.pts[a].x - ctx.center.x) * (ctx.pts[a].x - ctx.center.x)
             + (ctx.pts[a].y - ctx.center.y) * (ctx.pts[a].y - ctx.center.y);
    float db = (ctx.pts[b].x - ctx.center.x) * (ctx.pts[b].x - ctx.center.x)
             + (ctx.pts[b].y - ctx.center.y) * (ctx.pts[b].y - ctx.center.y);
    if (da < db) { return -1; }
    if (da > db) { return 1; }
    return 0;
}

fn float circumradius_sq(Vec3f a, Vec3f b, Vec3f c)
{
    if (try cc = geometry::circumcenter_planar(a, b, c)) {
        float dx = cc.x - a.x;
        float dy = cc.y - a.y;
        return dx * dx + dy * dy;
    }
    return 1e30f;
}

fn int find_closest(Vec3f[] pts, usz count, Vec3f ref, int exclude)
{
    int best = -1;
    float best_dist = 1e30f;
    for (usz i = 0; i < count; i++) {
        if ((int)i == exclude) { continue; }
        float dx = pts[i].x - ref.x;
        float dy = pts[i].y - ref.y;
        float d2 = dx * dx + dy * dy;
        if (d2 < best_dist) { best = (int)i; best_dist = d2; }
    }
    return best;
}

fn HeIndex add_triangle(DelaunayBuilder* b, VertexIndex i0, VertexIndex i1, VertexIndex i2,
                         HeIndex twin0, HeIndex twin1, HeIndex twin2)
{
    usz t = b.face_count;
    b.face_count = t + 1;
    b.triangles[3*t]   = i0;
    b.triangles[3*t+1] = i1;
    b.triangles[3*t+2] = i2;
    b.halfedges[3*t]   = twin0;
    b.halfedges[3*t+1] = twin1;
    b.halfedges[3*t+2] = twin2;
    if (twin0 != cg::INVALID_HE) { b.halfedges[twin0] = (HeIndex)(int)(3*t); }
    if (twin1 != cg::INVALID_HE) { b.halfedges[twin1] = (HeIndex)(int)(3*t+1); }
    if (twin2 != cg::INVALID_HE) { b.halfedges[twin2] = (HeIndex)(int)(3*t+2); }
    return (HeIndex)(int)(3*t);
}

fn HeIndex legalize(HeIndex start_he, Vec3f[] positions, DelaunayBuilder* b)
{
    HeIndex ar = cg::INVALID_HE;
    usz stack_size = 0;
    HeIndex he = start_he;

    while (true) {
        HeIndex a = b.halfedges[he];
        if (a != cg::INVALID_HE) {
            ar = prev_halfedge(he);
            HeIndex al = next_halfedge(he);
            HeIndex bl = prev_halfedge(a);

            VertexIndex p0 = b.triangles[ar];
            VertexIndex pr = b.triangles[he];
            VertexIndex pl = b.triangles[al];
            VertexIndex p1 = b.triangles[bl];

            Vec2f v0 = { positions[(int)p0].x, positions[(int)p0].y };
            Vec2f vr = { positions[(int)pr].x, positions[(int)pr].y };
            Vec2f vl = { positions[(int)pl].x, positions[(int)pl].y };
            Vec2f v1 = { positions[(int)p1].x, positions[(int)p1].y };

            if (geometry::in_circle_2d(v0, vr, vl, v1) == geometry::PREDICATE_POSITIVE) {
                b.triangles[he] = p1;
                b.triangles[a]  = p0;

                HeIndex hbl = b.halfedges[bl];
                HeIndex har = b.halfedges[ar];

                link_halfedges(b.halfedges, he, hbl);
                link_halfedges(b.halfedges, a, har);
                link_halfedges(b.halfedges, ar, bl);

                HeIndex br = next_halfedge(a);
                b.legalize_stack[stack_size] = he;
                stack_size++;
                b.legalize_stack[stack_size] = br;
                stack_size++;

                if (hbl == cg::INVALID_HE) {
                    usz max_he = 3 * b.face_count;
                    for (usz vi = 0; vi < max_he; vi++) {
                        if (b.hull_tri[(int)b.triangles[vi]] == bl) {
                            b.hull_tri[(int)b.triangles[vi]] = he;
                            break;
                        }
                    }
                }
            }
        }

        if (stack_size > 0) {
            stack_size--;
            he = b.legalize_stack[stack_size];
        } else {
            break;
        }
    }
    if (ar == cg::INVALID_HE) { return prev_halfedge(start_he); }
    return ar;
}

fn HalfEdgeMesh? delaunay_2d(Allocator alloc, Vec3f[] positions, DelaunayOptions options = {})
{
    if (positions.len == 0) { return cg::EMPTY_INPUT~; }

    // ── Phase 0: Dedup + validate ──
    int[] indices = mem::alloc::new_array(alloc, int, (sz) positions.len);
    defer free(indices);
    for (usz i = 0; i < positions.len; i++) { indices[i] = (int)i; }

    usz unique_count = 0;
    for (usz i = 0; i < positions.len; i++) {
        bool dup = false;
        for (usz j = 0; j < unique_count; j++) {
            int p = indices[j];
            int c = indices[i];
            if (positions[p].x == positions[c].x && positions[p].y == positions[c].y) {
                dup = true;
                break;
            }
        }
        if (!dup) {
            indices[unique_count] = indices[i];
            unique_count++;
        }
    }
    if (unique_count < 3) { return cg::DEGENERATE_INPUT~; }

    int t0 = indices[0];
    int t1 = -1;
    for (usz i = 1; i < unique_count; i++) {
        int p = indices[i];
        if (positions[p].x != positions[t0].x || positions[p].y != positions[t0].y) {
            t1 = p;
            break;
        }
    }
    if (t1 < 0) { return cg::DEGENERATE_INPUT~; }

    int t2 = -1;
    Vec2f d0 = { positions[t0].x, positions[t0].y };
    Vec2f d1 = { positions[t1].x, positions[t1].y };
    for (usz i = 0; i < unique_count; i++) {
        int p = indices[i];
        if (p == t0 || p == t1) { continue; }
        Vec2f dp = { positions[p].x, positions[p].y };
        if (geometry::orient_2d(d0, d1, dp) != geometry::PREDICATE_ZERO) {
            t2 = p;
            break;
        }
    }
    if (t2 < 0) { return cg::DEGENERATE_INPUT~; }

    if (options.has_bounding_box) {
        if (options.bounding_box.min.x > options.bounding_box.max.x
            || options.bounding_box.min.y > options.bounding_box.max.y) {
            return cg::DEGENERATE_INPUT~;
        }
        for (usz i = 0; i < unique_count; i++) {
            int p = indices[i];
            if (positions[p].x < options.bounding_box.min.x
                || positions[p].x > options.bounding_box.max.x
                || positions[p].y < options.bounding_box.min.y
                || positions[p].y > options.bounding_box.max.y) {
                return cg::DEGENERATE_INPUT~;
            }
        }
    }

    Vec3f[] work_pos = mem::alloc::new_array(alloc, Vec3f, (sz) unique_count);
    defer free(work_pos);
    for (usz i = 0; i < unique_count; i++) { work_pos[i] = positions[indices[i]]; }

    // ── Phase 1: Seed triangle + distance sort ──
    float min_x = work_pos[0].x;
    float max_x = work_pos[0].x;
    float min_y = work_pos[0].y;
    float max_y = work_pos[0].y;
    for (usz i = 1; i < unique_count; i++) {
        if (work_pos[i].x < min_x) { min_x = work_pos[i].x; }
        if (work_pos[i].x > max_x) { max_x = work_pos[i].x; }
        if (work_pos[i].y < min_y) { min_y = work_pos[i].y; }
        if (work_pos[i].y > max_y) { max_y = work_pos[i].y; }
    }
    Vec3f center = { (min_x + max_x) * 0.5f, (min_y + max_y) * 0.5f, 0 };

    int i0 = find_closest(work_pos, unique_count, center, -1);
    int i1 = find_closest(work_pos, unique_count, work_pos[i0], i0);

    int i2 = -1;
    float min_radius = 1e30f;
    for (usz i = 0; i < unique_count; i++) {
        if ((int)i == i0 || (int)i == i1) { continue; }
        float r = circumradius_sq(work_pos[i0], work_pos[i1], work_pos[i]);
        if (r < min_radius) { i2 = (int)i; min_radius = r; }
    }
    if (i2 < 0) { return cg::DEGENERATE_INPUT~; }

    Vec2f s0 = { work_pos[i0].x, work_pos[i0].y };
    Vec2f s1 = { work_pos[i1].x, work_pos[i1].y };
    Vec2f s2 = { work_pos[i2].x, work_pos[i2].y };
    if (geometry::orient_2d(s0, s1, s2) <= geometry::PREDICATE_ZERO) {
        int tmp = i1; i1 = i2; i2 = tmp;
    }

    Vec3f seed_center = geometry::circumcenter_planar(work_pos[i0], work_pos[i1], work_pos[i2])!!;

    for (usz i = 0; i < unique_count; i++) { indices[i] = (int)i; }
    DistSortCtx dctx = { work_pos, seed_center };
    sort::quicksort(indices[0..(usz)(unique_count - 1)], &compare_by_distance, dctx);

    // ── Phase 2: Allocate builder + init hull ──
    usz max_triangles = 2 * unique_count;
    usz hash_len = (usz)math::sqrt((double)unique_count);
    if (hash_len < 1) { hash_len = 1; }

    DelaunayBuilder builder;
    builder.triangles      = mem::alloc::new_array(alloc, VertexIndex, (sz)(3 * max_triangles));
    builder.halfedges      = mem::alloc::new_array(alloc, HeIndex, (sz)(3 * max_triangles));
    builder.hull_tri       = mem::alloc::new_array(alloc, HeIndex, (sz) unique_count);
    builder.hull_prev      = mem::alloc::new_array(alloc, VertexIndex, (sz) unique_count);
    builder.hull_next      = mem::alloc::new_array(alloc, VertexIndex, (sz) unique_count);
    builder.hull_hash      = mem::alloc::new_array(alloc, VertexIndex, (sz) hash_len);
    builder.legalize_stack = mem::alloc::new_array(alloc, HeIndex, (sz) max_triangles);
    builder.face_count = 0;
    defer free(builder.triangles); defer free(builder.halfedges);
    defer free(builder.hull_tri); defer free(builder.hull_prev);
    defer free(builder.hull_next); defer free(builder.hull_hash);
    defer free(builder.legalize_stack);

    builder.triangles[0] = (VertexIndex)i0;
    builder.triangles[1] = (VertexIndex)i1;
    builder.triangles[2] = (VertexIndex)i2;
    builder.halfedges[0] = cg::INVALID_HE;
    builder.halfedges[1] = cg::INVALID_HE;
    builder.halfedges[2] = cg::INVALID_HE;
    builder.face_count = 1;

    for (usz i = 0; i < unique_count; i++) {
        builder.hull_next[i] = cg::INVALID_VERTEX;
        builder.hull_prev[i] = cg::INVALID_VERTEX;
    }
    builder.hull_next[(int)i0] = (VertexIndex)i1;
    builder.hull_next[(int)i1] = (VertexIndex)i2;
    builder.hull_next[(int)i2] = (VertexIndex)i0;
    builder.hull_prev[(int)i1] = (VertexIndex)i0;
    builder.hull_prev[(int)i2] = (VertexIndex)i1;
    builder.hull_prev[(int)i0] = (VertexIndex)i2;
    builder.hull_tri[(int)i0]  = (HeIndex)0;
    builder.hull_tri[(int)i1]  = (HeIndex)1;
    builder.hull_tri[(int)i2]  = (HeIndex)2;
    builder.hull_center   = seed_center;
    builder.hull_start    = (VertexIndex)i0;

    for (usz i = 0; i < hash_len; i++) { builder.hull_hash[i] = cg::INVALID_VERTEX; }
    {
        int[3] hull_verts = { i0, i1, i2 };
        for (int hi = 0; hi < 3; hi++) {
            int v = hull_verts[hi];
            float dx = work_pos[(int)v].x - seed_center.x;
            float dy = work_pos[(int)v].y - seed_center.y;
            float angle = pseudo_angle(dx, dy);
            usz key = (usz)(angle * (float)hash_len);
            builder.hull_hash[key] = (VertexIndex)v;
        }
    }

    // ── Phase 3: Point insertion ──
    for (usz si = 0; si < unique_count; si++) {
        int p = indices[si];
        if (p == i0 || p == i1 || p == i2) { continue; }

        if (si > 0) {
            int prev_p = indices[si - 1];
            float dx = work_pos[(int)p].x - work_pos[(int)prev_p].x;
            float dy = work_pos[(int)p].y - work_pos[(int)prev_p].y;
            if (dx * dx + dy * dy < DELAUNAY_NEAR_DUP_EPSILON) { continue; }
        }

        // find_visible_edge(p)
        float pdx = work_pos[(int)p].x - builder.hull_center.x;
        float pdy = work_pos[(int)p].y - builder.hull_center.y;
        float pa = pseudo_angle(pdx, pdy);
        usz key = fast_mod((usz)(pa * (float)hash_len), hash_len);
        VertexIndex start = cg::INVALID_VERTEX;
        for (usz j = 0; j < hash_len; j++) {
            start = builder.hull_hash[fast_mod(key + j, hash_len)];
            if (start != cg::INVALID_VERTEX && builder.hull_next[(int)start] != cg::INVALID_VERTEX) { break; }
        }
        if (start == cg::INVALID_VERTEX || builder.hull_next[(int)start] == cg::INVALID_VERTEX) { continue; }

        VertexIndex e = start;
        VertexIndex q = builder.hull_next[(int)e];
        while (true) {
            Vec2f ve = { work_pos[(int)e].x, work_pos[(int)e].y };
            Vec2f vq = { work_pos[(int)q].x, work_pos[(int)q].y };
            Vec2f vp = { work_pos[(int)p].x, work_pos[(int)p].y };
            if (geometry::orient_2d(ve, vq, vp) < geometry::PREDICATE_ZERO) { break; }
            e = q;
            q = builder.hull_next[(int)e];
            if (e == start) { e = cg::INVALID_VERTEX; break; }
        }
        if (e == cg::INVALID_VERTEX) { continue; }

        // Walk hull forward from e.
        HeIndex t = add_triangle(&builder, e, (VertexIndex)p, builder.hull_next[(int)e],
                                  cg::INVALID_HE, cg::INVALID_HE, builder.hull_tri[(int)e]);
        builder.hull_tri[(int)p] = legalize(t + 2, work_pos, &builder);
        builder.hull_tri[(int)e] = t;

        VertexIndex curr = builder.hull_next[(int)e];
        while (true) {
            VertexIndex qq = builder.hull_next[(int)curr];
            Vec2f vcurr = { work_pos[(int)curr].x, work_pos[(int)curr].y };
            Vec2f vqq   = { work_pos[(int)qq].x, work_pos[(int)qq].y };
            Vec2f vpp   = { work_pos[(int)p].x, work_pos[(int)p].y };
            if (geometry::orient_2d(vcurr, vqq, vpp) >= geometry::PREDICATE_ZERO) { break; }

            t = add_triangle(&builder, curr, (VertexIndex)p, qq,
                              builder.hull_tri[(int)p], cg::INVALID_HE, builder.hull_tri[(int)curr]);
            builder.hull_tri[(int)p] = legalize(t + 2, work_pos, &builder);
            builder.hull_tri[(int)curr] = t;

            builder.hull_next[(int)curr] = cg::INVALID_VERTEX;
            curr = qq;
        }

        // Update hull links.
        builder.hull_prev[(int)p] = e;
        builder.hull_next[(int)e] = (VertexIndex)p;
        builder.hull_prev[(int)curr] = (VertexIndex)p;
        builder.hull_next[(int)p] = curr;

        // Hash new hull vertex.
        float pdx2 = work_pos[(int)p].x - builder.hull_center.x;
        float pdy2 = work_pos[(int)p].y - builder.hull_center.y;
        float angle2 = pseudo_angle(pdx2, pdy2);
        key = (usz)(angle2 * (float)hash_len);
        builder.hull_hash[key] = (VertexIndex)p;
    }

    // ── Phase 6: Mesh finalization ──
    HalfEdgeMesh mesh;
    mesh.half_edges = mem::alloc::new_array(alloc, HalfEdge, (sz)(3 * builder.face_count));
    defer catch free(mesh.half_edges);
    mesh.faces = mem::alloc::new_array(alloc, HalfEdgeFace, (sz) builder.face_count);
    defer catch free(mesh.faces);
    mesh.vertices = mem::alloc::new_array(alloc, HalfEdgeVertex, (sz) unique_count);
    defer catch free(mesh.vertices);
    mesh.positions = mem::alloc::new_array(alloc, Vec3f, (sz) unique_count);
    defer catch free(mesh.positions);

    for (usz i = 0; i < unique_count; i++) { mesh.positions[i] = work_pos[i]; }

    for (usz i = 0; i < unique_count; i++) { mesh.vertices[i].half_edge = cg::INVALID_HE; }
    for (usz i = 0; i < 3 * builder.face_count; i++) {
        VertexIndex v = builder.triangles[i];
        if (mesh.vertices[(int)v].half_edge == cg::INVALID_HE) {
            mesh.vertices[(int)v].half_edge = (HeIndex)(int)i;
        }
    }

    for (usz i = 0; i < 3 * builder.face_count; i++) {
        mesh.half_edges[i].origin = builder.triangles[i];
        mesh.half_edges[i].twin   = builder.halfedges[i];
        mesh.half_edges[i].face   = (FaceIndex)(int)(i / 3);
        if (i % 3 == 2) {
            mesh.half_edges[i].next = (HeIndex)(int)(i - 2);
        } else {
            mesh.half_edges[i].next = (HeIndex)(int)(i + 1);
        }
    }

    for (usz i = 0; i < builder.face_count; i++) {
        mesh.faces[i].half_edge = (HeIndex)(int)(3 * i);
    }

    mesh.normals = {};
    mesh.uvs = {};

    mesh.validate()!;
    return mesh;
}
