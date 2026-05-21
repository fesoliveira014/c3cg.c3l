module cg::delaunay;
import cg;
import cg::geometry;
import cg::half_edge;
import std::collections::map;
import std::math;

const float DELAUNAY_SUPER_MARGIN = 10.0f;
const usz DELAUNAY_FACE_FACTOR = 3;
const usz DELAUNAY_FACE_MARGIN = 10;
const usz DELAUNAY_BOUNDARY_FACTOR = 3;
const usz DELAUNAY_TRI_VERT_COUNT = 3;
const usz DELAUNAY_SHUFFLE_MULTIPLIER = 2654435761;

struct DelaunayFace {
    VertexIndex v0, v1, v2;
    bool alive;
}

fn HalfEdgeMesh? delaunay_2d(Allocator alloc, Vec3f[] positions, DelaunayOptions options = {})
{
    if (positions.len == 0) return cg::EMPTY_INPUT~;

    // Deduplicate exact (x,y).
    int[] indices = mem::alloc::new_array(alloc, int, (sz) positions.len);
    defer catch free(indices);
    for (usz i = 0; i < positions.len; i++) indices[i] = (int)i;

    usz unique_count = 0;
    for (usz i = 0; i < positions.len; i++) {
        bool dup = false;
        for (usz j = 0; j < unique_count; j++) {
            int p = indices[j];
            int c = indices[i];
            if (positions[p].x == positions[c].x && positions[p].y == positions[c].y) {
                dup = true; break;
            }
        }
        if (!dup) { indices[unique_count] = indices[i]; unique_count++; }
    }
    if (unique_count < 3) return cg::DEGENERATE_INPUT~;

    // Collinearity scan: find first 2 distinct, then non-collinear 3rd.
    int t0 = indices[0];
    int t1 = -1;
    for (usz i = 1; i < unique_count; i++) {
        int p = indices[i];
        if (positions[p].x != positions[t0].x || positions[p].y != positions[t0].y) {
            t1 = p; break;
        }
    }
    if (t1 < 0) return cg::DEGENERATE_INPUT~;

    int t2 = -1;
    Vec2f d0 = { positions[t0].x, positions[t0].y };
    Vec2f d1 = { positions[t1].x, positions[t1].y };
    for (usz i = 0; i < unique_count; i++) {
        int p = indices[i];
        if (p == t0 || p == t1) continue;
        Vec2f dp = { positions[p].x, positions[p].y };
        if (geometry::orient_2d(d0, d1, dp) != geometry::PREDICATE_ZERO) {
            t2 = p; break;
        }
    }
    if (t2 < 0) return cg::DEGENERATE_INPUT~;

    // Build working positions from deduped original indices.
    Vec3f[] work_pos = mem::alloc::new_array(alloc, Vec3f, (sz)(unique_count + 3));
    defer catch free(work_pos);
    for (usz i = 0; i < unique_count; i++) work_pos[i] = positions[indices[i]];

    // Rebuild indices as compact 0..unique_count-1 for insertion order, then shuffle.
    for (usz i = 0; i < unique_count; i++) indices[i] = (int)i;

    // Shuffle compact indices (deterministic LCG).
    for (usz i = unique_count; i > 1; i--) {
        usz j = ((i - 1) * DELAUNAY_SHUFFLE_MULTIPLIER) % i;
        int tmp = indices[i - 1];
        indices[i - 1] = indices[j];
        indices[j] = tmp;
    }

    // Super-triangle: AABB + margin.
    float min_x = work_pos[0].x;
    float max_x = work_pos[0].x;
    float min_y = work_pos[0].y;
    float max_y = work_pos[0].y;
    if (options.has_bounding_box) {
        // Validate all points inside bbox (x/y only).
        for (usz i = 0; i < unique_count; i++) {
            if (work_pos[i].x < options.bounding_box.min.x
                || work_pos[i].x > options.bounding_box.max.x
                || work_pos[i].y < options.bounding_box.min.y
                || work_pos[i].y > options.bounding_box.max.y) {
                return cg::DEGENERATE_INPUT~;
            }
        }
        min_x = options.bounding_box.min.x;
        max_x = options.bounding_box.max.x;
        min_y = options.bounding_box.min.y;
        max_y = options.bounding_box.max.y;
    } else {
        for (usz i = 1; i < unique_count; i++) {
            if (work_pos[i].x < min_x) min_x = work_pos[i].x;
            if (work_pos[i].x > max_x) max_x = work_pos[i].x;
            if (work_pos[i].y < min_y) min_y = work_pos[i].y;
            if (work_pos[i].y > max_y) max_y = work_pos[i].y;
        }
    }

    float dx = max_x - min_x;
    float dy = max_y - min_y;
    float diag = dx * dx + dy * dy;
    if (diag < geometry::GEOMETRY_EPSILON * geometry::GEOMETRY_EPSILON) {
        diag = 1.0f;
    } else {
        diag = (float)math::sqrt((double)diag);
    }
    float margin = DELAUNAY_SUPER_MARGIN * diag;
    float mid_x = (min_x + max_x) * 0.5f;

    usz st0 = unique_count;
    usz st1 = unique_count + 1;
    usz st2 = unique_count + 2;
    work_pos[st0] = { min_x - margin, min_y - margin, 0 };
    work_pos[st1] = { max_x + margin, min_y - margin, 0 };
    work_pos[st2] = { mid_x,           max_y + margin, 0 };

    // Orient super-triangle CCW.
    Vec2f s0 = { work_pos[st0].x, work_pos[st0].y };
    Vec2f s1 = { work_pos[st1].x, work_pos[st1].y };
    Vec2f s2 = { work_pos[st2].x, work_pos[st2].y };
    if (geometry::orient_2d(s0, s1, s2) != geometry::PREDICATE_POSITIVE) {
        usz tmp = st1; st1 = st2; st2 = tmp;
        Vec3f tp = work_pos[st1]; work_pos[st1] = work_pos[st2]; work_pos[st2] = tp;
    }

    // Face list + edge map.
    usz face_cap = DELAUNAY_FACE_FACTOR * unique_count + DELAUNAY_FACE_MARGIN;
    DelaunayFace[] faces = mem::alloc::new_array(alloc, DelaunayFace, (sz) face_cap);
    defer catch free(faces);

    HashMap{int[<2>], int} edge_map;
    edge_map.init(alloc);
    defer catch edge_map.free();

    usz face_count = 1;
    faces[0] = { (VertexIndex)(int)st0, (VertexIndex)(int)st1, (VertexIndex)(int)st2, true };
    int[<2>] e0 = { (int)st0, (int)st1 };
    int[<2>] e1 = { (int)st1, (int)st2 };
    int[<2>] e2 = { (int)st2, (int)st0 };
    edge_map[e0] = 0;
    edge_map[e1] = 0;
    edge_map[e2] = 0;

    // Boundary scratch arrays.
    int[] bnd_from = mem::alloc::new_array(alloc, int, (sz)(DELAUNAY_BOUNDARY_FACTOR * unique_count));
    defer catch free(bnd_from);
    int[] bnd_to = mem::alloc::new_array(alloc, int, (sz)(DELAUNAY_BOUNDARY_FACTOR * unique_count));
    defer catch free(bnd_to);

    // Phase 2 — point insertion.
    for (usz si = 0; si < unique_count; si++) {
        int p = indices[si];
        Vec2f p2 = { work_pos[p].x, work_pos[p].y };

        // Mark bad triangles.
        usz bad_count = 0;
        for (usz fi = 0; fi < face_count; fi++) {
            if (!faces[fi].alive) continue;
            Vec2f a = { work_pos[(int)faces[fi].v0].x, work_pos[(int)faces[fi].v0].y };
            Vec2f b = { work_pos[(int)faces[fi].v1].x, work_pos[(int)faces[fi].v1].y };
            Vec2f c = { work_pos[(int)faces[fi].v2].x, work_pos[(int)faces[fi].v2].y };
            if (geometry::in_circle_2d(a, b, c, p2) == geometry::PREDICATE_POSITIVE) {
                faces[fi].alive = false;
                bad_count++;
            }
        }
        if (bad_count == 0) continue;

        // Cavity boundary.
        usz bnd_count = 0;
        for (usz fi = 0; fi < face_count; fi++) {
            if (faces[fi].alive) continue;
            int v0 = (int)faces[fi].v0;
            int v1 = (int)faces[fi].v1;
            int v2 = (int)faces[fi].v2;
            int[2][3] edges = { { v0, v1 }, { v1, v2 }, { v2, v0 } };
            for (int ei = 0; ei < 3; ei++) {
                int u = edges[ei][0];
                int v = edges[ei][1];
                int[<2>] rev = { v, u };
                if (try adj = edge_map[rev]) {
                    if ((usz)adj < face_count && faces[adj].alive) {
                        bnd_from[bnd_count] = u;
                        bnd_to[bnd_count] = v;
                        bnd_count++;
                    }
                } else {
                    bnd_from[bnd_count] = u;
                    bnd_to[bnd_count] = v;
                    bnd_count++;
                }
            }
        }

        // Remove dead faces with incremental edge-map patching.
        usz i = 0;
        while (i < face_count) {
            if (faces[i].alive) { i++; continue; }

            // Remove this dead face's edges.
            int dv0 = (int)faces[i].v0;
            int dv1 = (int)faces[i].v1;
            int dv2 = (int)faces[i].v2;
            edge_map.remove({ dv0, dv1 });
            edge_map.remove({ dv1, dv2 });
            edge_map.remove({ dv2, dv0 });

            face_count--;

            // Remove any other dead faces at the end.
            while (face_count > i && !faces[face_count].alive) {
                int lv0 = (int)faces[face_count].v0;
                int lv1 = (int)faces[face_count].v1;
                int lv2 = (int)faces[face_count].v2;
                edge_map.remove({ lv0, lv1 });
                edge_map.remove({ lv1, lv2 });
                edge_map.remove({ lv2, lv0 });
                face_count--;
            }

            if (i < face_count) {
                // Move last alive face into the dead slot, updating its edge indices.
                faces[i] = faces[face_count];
                int mv0 = (int)faces[i].v0;
                int mv1 = (int)faces[i].v1;
                int mv2 = (int)faces[i].v2;
                edge_map.remove({ mv0, mv1 });
                edge_map.remove({ mv1, mv2 });
                edge_map.remove({ mv2, mv0 });
                edge_map[{ mv0, mv1 }] = (int)i;
                edge_map[{ mv1, mv2 }] = (int)i;
                edge_map[{ mv2, mv0 }] = (int)i;
                i++;
            }
        }

        // Retriangulate: add new faces (u,v,p) for each boundary edge.
        for (usz bi = 0; bi < bnd_count; bi++) {
            int u = bnd_from[bi];
            int v = bnd_to[bi];
            faces[face_count] = { (VertexIndex)u, (VertexIndex)v, (VertexIndex)p, true };
            int[<2>] ne0 = { u, v };
            int[<2>] ne1 = { v, p };
            int[<2>] ne2 = { p, u };
            edge_map[ne0] = (int)face_count;
            edge_map[ne1] = (int)face_count;
            edge_map[ne2] = (int)face_count;
            face_count++;
        }
    }

    // Phase 3 — strip super-triangle faces.
    usz strip = 0;
    for (usz fi = 0; fi < face_count; fi++) {
        int v0 = (int)faces[fi].v0;
        int v1 = (int)faces[fi].v1;
        int v2 = (int)faces[fi].v2;
        if ((usz)v0 == st0 || (usz)v1 == st0 || (usz)v2 == st0
            || (usz)v0 == st1 || (usz)v1 == st1 || (usz)v2 == st1
            || (usz)v0 == st2 || (usz)v1 == st2 || (usz)v2 == st2) continue;
        if (strip != fi) faces[strip] = faces[fi];
        strip++;
    }
    face_count = strip;

    // Phase 4 — compact output: only vertices referenced by surviving faces.
    bool[] used = mem::alloc::new_array(alloc, bool, (sz)(unique_count + 3));
    defer catch free(used);
    for (usz i = 0; i < unique_count + 3; i++) used[i] = false;
    for (usz fi = 0; fi < face_count; fi++) {
        used[(int)faces[fi].v0] = true;
        used[(int)faces[fi].v1] = true;
        used[(int)faces[fi].v2] = true;
    }

    usz used_vert_count = 0;
    for (usz i = 0; i < unique_count; i++) { if (used[i]) used_vert_count++; }

    Vec3f[] final_positions = mem::alloc::new_array(alloc, Vec3f, (sz) used_vert_count);
    defer catch free(final_positions);
    int[] final_remap = mem::alloc::new_array(alloc, int, (sz)(unique_count + 3));
    defer catch free(final_remap);
    for (usz i = 0; i < unique_count + 3; i++) final_remap[i] = -1;

    usz w = 0;
    for (usz i = 0; i < unique_count; i++) {
        if (used[i]) { final_positions[w] = work_pos[i]; final_remap[i] = (int)w; w++; }
    }

    uint[] tri_indices = mem::alloc::new_array(alloc, uint, (sz)(3 * face_count));
    defer catch free(tri_indices);
    for (usz fi = 0; fi < face_count; fi++) {
        tri_indices[3 * fi]     = (uint)final_remap[(int)faces[fi].v0];
        tri_indices[3 * fi + 1] = (uint)final_remap[(int)faces[fi].v1];
        tri_indices[3 * fi + 2] = (uint)final_remap[(int)faces[fi].v2];
    }

    HalfEdgeMesh mesh = half_edge::from_triangles(alloc, final_positions, tri_indices)!;
    defer catch mesh.destroy();

    mesh.validate()!;

    free(tri_indices);
    free(final_remap);
    free(final_positions);
    free(used);
    free(bnd_from);
    free(bnd_to);
    edge_map.free();
    free(faces);
    free(work_pos);
    free(indices);

    return mesh;
}
