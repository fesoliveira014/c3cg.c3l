module cg::voronoi;
import cg;
import cg::geometry;
import cg::half_edge;
import cg::delaunay;
import std::math;
import std::sort;

// ── Angle-sort helper for circumcenter ordering ──

struct AngleCtx { float[] angles; }

fn int cmp_angle(int a, int b, AngleCtx ctx)
{
    if (ctx.angles[a] < ctx.angles[b]) return -1;
    if (ctx.angles[a] > ctx.angles[b]) return 1;
    return 0;
}

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

// ── Public API ──

fn VoronoiDiagram? in_polygon(Allocator alloc, Vec3f[] sites, Vec3f[] polygon)
{
    if (!is_convex_polygon(polygon)) return cg::NON_CONVEX_BOUNDING_POLYGON~;
    if (sites.len == 0) return cg::EMPTY_INPUT~;

    Vec3f[] inside_sites = mem::alloc::new_array(alloc, Vec3f, (sz)sites.len);
    defer free(inside_sites);
    usz inside_count = 0;
    for (usz i = 0; i < sites.len; i++) {
        if (point_in_convex_polygon(sites[i], polygon)) {
            inside_sites[inside_count] = sites[i];
            inside_count++;
        }
    }

    if (inside_count == 0) {
        return {
            .mesh = {},
            .sites = {},
        };
    }

    if (inside_count == 1) {
        Vec3f[] positions = mem::alloc::new_array(alloc, Vec3f, (sz)polygon.len);
        defer free(positions);
        for (usz i = 0; i < polygon.len; i++) positions[i] = polygon[i];

        uint[] face_indices = mem::alloc::new_array(alloc, uint, (sz)polygon.len);
        defer free(face_indices);
        for (usz i = 0; i < polygon.len; i++) face_indices[i] = (uint)i;

        uint[] face_offsets = mem::alloc::new_array(alloc, uint, (sz)2);
        defer free(face_offsets);
        face_offsets[0] = 0;
        face_offsets[1] = (uint)polygon.len;

        HalfEdgeMesh mesh = half_edge::from_polygons(alloc, positions, face_offsets, face_indices)!;

        Vec3f[] result_sites = mem::alloc::new_array(alloc, Vec3f, (sz)1);
        defer catch { mesh.destroy(); free(result_sites); };
        result_sites[0] = inside_sites[0];

        return {
            .mesh = mesh,
            .sites = result_sites,
        };
    }

    if (inside_count == 2) {
        Vec3f[] poly0 = half_plane_clip(alloc, polygon, inside_sites[0], inside_sites[1], true)!;
        defer free(poly0);
        Vec3f[] poly1 = half_plane_clip(alloc, polygon, inside_sites[0], inside_sites[1], false)!;
        defer free(poly1);

        usz n_cells = 0;
        if (poly0.len >= 3) n_cells++;
        if (poly1.len >= 3) n_cells++;

        if (n_cells == 0) {
            return {
                .mesh = {},
                .sites = {},
            };
        }

        usz total_verts = 0;
        if (poly0.len >= 3) total_verts += poly0.len;
        if (poly1.len >= 3) total_verts += poly1.len;

        Vec3f[] positions = mem::alloc::new_array(alloc, Vec3f, (sz)total_verts);
        defer free(positions);
        uint[] face_indices = mem::alloc::new_array(alloc, uint, (sz)total_verts);
        defer free(face_indices);
        uint[] face_offsets = mem::alloc::new_array(alloc, uint, (sz)(n_cells + 1));
        defer free(face_offsets);

        usz pos_idx = 0;
        usz cell = 0;
        face_offsets[0] = 0;
        if (poly0.len >= 3) {
            for (usz i = 0; i < poly0.len; i++) {
                positions[pos_idx + i] = poly0[i];
                face_indices[pos_idx + i] = (uint)(pos_idx + i);
            }
            pos_idx += poly0.len;
            cell++;
            face_offsets[cell] = (uint)pos_idx;
        }
        if (poly1.len >= 3) {
            for (usz i = 0; i < poly1.len; i++) {
                positions[pos_idx + i] = poly1[i];
                face_indices[pos_idx + i] = (uint)(pos_idx + i);
            }
            pos_idx += poly1.len;
            cell++;
            face_offsets[cell] = (uint)pos_idx;
        }

        HalfEdgeMesh mesh = half_edge::from_polygons(alloc, positions, face_offsets, face_indices)!;

        Vec3f[] result_sites = mem::alloc::new_array(alloc, Vec3f, (sz)n_cells);
        defer catch { mesh.destroy(); free(result_sites); };
        usz site_idx = 0;
        if (poly0.len >= 3) { result_sites[site_idx] = inside_sites[0]; site_idx++; }
        if (poly1.len >= 3) { result_sites[site_idx] = inside_sites[1]; }

        return {
            .mesh = mesh,
            .sites = result_sites,
        };
    }


    // ── N ≥ 3: helper-point Delaunay path ──

    float min_x = polygon[0].x;
    float max_x = min_x;
    float min_y = polygon[0].y;
    float max_y = min_y;
    for (usz i = 1; i < polygon.len; i++) {
        if (polygon[i].x < min_x) min_x = polygon[i].x;
        if (polygon[i].x > max_x) max_x = polygon[i].x;
        if (polygon[i].y < min_y) min_y = polygon[i].y;
        if (polygon[i].y > max_y) max_y = polygon[i].y;
    }
    float w = max_x - min_x;
    float h = max_y - min_y;
    float margin_x = w * 2;
    float margin_y = h * 2;
    Vec3f[4] helpers = {
        { min_x - margin_x, min_y - margin_y, 0 },
        { max_x + margin_x, min_y - margin_y, 0 },
        { max_x + margin_x, max_y + margin_y, 0 },
        { min_x - margin_x, max_y + margin_y, 0 },
    };

    usz total = inside_count + 4;
    Vec3f[] all_points = mem::alloc::new_array(alloc, Vec3f, (sz)total);
    defer free(all_points);
    for (usz i = 0; i < inside_count; i++) all_points[i] = inside_sites[i];
    for (usz i = 0; i < 4; i++) all_points[inside_count + i] = helpers[i];

    HalfEdgeMesh delaunay_mesh = delaunay::delaunay_2d(alloc, all_points)!;
    defer delaunay_mesh.destroy();

    Vec3f[] centers = geometry::circumcenters_planar(alloc, &delaunay_mesh)!;
    defer free(centers);

    // Match inside_sites to Delaunay vertex indices.
    // delaunay_2d compacts vertex indices, so we match by position.
    VertexIndex[] vertex_map = mem::alloc::new_array(alloc, VertexIndex, (sz)inside_count);
    defer free(vertex_map);
    for (usz i = 0; i < inside_count; i++) {
        vertex_map[i] = cg::INVALID_VERTEX;
        for (usz s = 0; s < delaunay_mesh.positions.len; s++) {
            float dx = delaunay_mesh.positions[s].x - inside_sites[i].x;
            float dy = delaunay_mesh.positions[s].y - inside_sites[i].y;
            if (dx * dx + dy * dy < 1e-24f) {
                vertex_map[i] = (VertexIndex)(int)s;
                break;
            }
        }
    }

    // Collect incident triangle faces for each vertex by scanning all faces.
    // This works even for boundary vertices where vertex_one_ring_faces stops early.
    // We store faces per vertex in contiguous runs: all_face_lists[cell_start[i] .. cell_start[i+1]].
    int[] cell_start = mem::alloc::new_array(alloc, int, (sz)(inside_count + 1));
    defer free(cell_start);

    // First pass: count faces per vertex
    usz total_face_refs = 0;
    for (usz i = 0; i < inside_count; i++) {
        cell_start[i] = (int)total_face_refs;
        if (vertex_map[i] == cg::INVALID_VERTEX) continue;
        VertexIndex v = vertex_map[i];
        for (usz f = 0; f < delaunay_mesh.faces.len; f++) {
            HeIndex he = delaunay_mesh.faces[f].half_edge;
            HeIndex cursor = he;
            while (true) {
                if (delaunay_mesh.half_edges[cursor].origin == v) {
                    total_face_refs++;
                    break;
                }
                cursor = delaunay_mesh.half_edges[cursor].next;
                if (cursor == he) break;
            }
        }
    }
    cell_start[inside_count] = (int)total_face_refs;

    FaceIndex[] all_face_lists = mem::alloc::new_array(alloc, FaceIndex, (sz)total_face_refs);
    defer free(all_face_lists);

    // Second pass: fill face lists
    for (usz i = 0; i < inside_count; i++) {
        if (vertex_map[i] == cg::INVALID_VERTEX) continue;
        VertexIndex v = vertex_map[i];
        usz cursor = (usz)cell_start[i];
        for (usz f = 0; f < delaunay_mesh.faces.len; f++) {
            HeIndex he = delaunay_mesh.faces[f].half_edge;
            HeIndex c = he;
            while (true) {
                if (delaunay_mesh.half_edges[c].origin == v) {
                    all_face_lists[cursor] = (FaceIndex)(int)f;
                    cursor++;
                    break;
                }
                c = delaunay_mesh.half_edges[c].next;
                if (c == he) break;
            }
        }
    }

    // Scratch for angle-sorted indices
    int[] sorted_indices = mem::alloc::new_array(alloc, int, (sz)3);
    defer free(sorted_indices);

    // Pass: build Voronoi cell for each vertex, sort circumcenters by angle, clip
    usz n_surviving = 0;
    usz total_verts = 0;
    for (usz i = 0; i < inside_count; i++) {
        if (vertex_map[i] == cg::INVALID_VERTEX) continue;
        int start = cell_start[i];
        int end = cell_start[i + 1];
        int n_faces = end - start;
        if (n_faces < 3) continue;

        // Ensure sorted_indices is large enough
        if ((usz)n_faces > sorted_indices.len) {
            free(sorted_indices);
            sorted_indices = mem::alloc::new_array(alloc, int, (sz)n_faces);
        }

        // Compute angles of circumcenters relative to site position
        Vec3f vpos = inside_sites[i];
        float[] angles = mem::alloc::new_array(alloc, float, (sz)n_faces);
        defer free(angles);
        for (int j = 0; j < n_faces; j++) {
            sorted_indices[j] = j;
            FaceIndex f = all_face_lists[start + j];
            Vec3f cc = centers[(int)f];
            float dx = cc.x - vpos.x;
            float dy = cc.y - vpos.y;
            angles[j] = (float)math::atan2((double)dy, (double)dx);
        }

        // Sort face positions (0..n_faces-1) by angle
        AngleCtx actx = { angles };
        sort::quicksort(sorted_indices[0..(usz)(n_faces - 1)], &cmp_angle, actx);

        // Build polygon from sorted circumcenters
        Vec3f[] face_poly = mem::alloc::new_array(alloc, Vec3f, (sz)n_faces);
        for (int j = 0; j < n_faces; j++) {
            int idx = sorted_indices[j];
            FaceIndex f = all_face_lists[start + idx];
            face_poly[j] = centers[(int)f];
        }
        Vec3f[] clipped = sutherland_hodgman(alloc, face_poly, polygon)!;
        free(face_poly);
        if (clipped.len >= 3) { n_surviving++; total_verts += clipped.len; }
        free(clipped);
    }

    if (n_surviving == 0) {
        return {
            .mesh = {},
            .sites = {},
        };
    }

    // Build output mesh from surviving Voronoi cells
    Vec3f[] positions = mem::alloc::new_array(alloc, Vec3f, (sz)total_verts);
    defer free(positions);
    uint[] face_indices = mem::alloc::new_array(alloc, uint, (sz)total_verts);
    defer free(face_indices);
    uint[] face_offsets = mem::alloc::new_array(alloc, uint, (sz)(n_surviving + 1));
    defer free(face_offsets);
    Vec3f[] result_sites = mem::alloc::new_array(alloc, Vec3f, (sz)n_surviving);
    defer catch free(result_sites);

    usz pos_idx = 0;
    usz cell = 0;
    face_offsets[0] = 0;
    for (usz i = 0; i < inside_count; i++) {
        if (vertex_map[i] == cg::INVALID_VERTEX) continue;
        int start = cell_start[i];
        int end = cell_start[i + 1];
        int n_faces = end - start;
        if (n_faces < 3) continue;

        if ((usz)n_faces > sorted_indices.len) {
            free(sorted_indices);
            sorted_indices = mem::alloc::new_array(alloc, int, (sz)n_faces);
        }

        Vec3f vpos = inside_sites[i];
        float[] angles = mem::alloc::new_array(alloc, float, (sz)n_faces);
        defer free(angles);
        for (int j = 0; j < n_faces; j++) {
            sorted_indices[j] = j;
            FaceIndex f = all_face_lists[start + j];
            float dx = centers[(int)f].x - vpos.x;
            float dy = centers[(int)f].y - vpos.y;
            angles[j] = (float)math::atan2((double)dy, (double)dx);
        }
        AngleCtx actx2 = { angles };
        sort::quicksort(sorted_indices[0..(usz)(n_faces - 1)], &cmp_angle, actx2);

        Vec3f[] face_poly = mem::alloc::new_array(alloc, Vec3f, (sz)n_faces);
        for (int j = 0; j < n_faces; j++) {
            int idx = sorted_indices[j];
            FaceIndex f = all_face_lists[start + idx];
            face_poly[j] = centers[(int)f];
        }
        Vec3f[] clipped = sutherland_hodgman(alloc, face_poly, polygon)!;
        free(face_poly);
        if (clipped.len >= 3) {
            for (usz j = 0; j < clipped.len; j++) positions[pos_idx + j] = clipped[j];
            for (usz j = 0; j < clipped.len; j++) face_indices[pos_idx + j] = (uint)(pos_idx + j);
            pos_idx += clipped.len;
            result_sites[cell] = inside_sites[i];
            cell++;
            face_offsets[cell] = (uint)pos_idx;
        }
        free(clipped);
    }

    HalfEdgeMesh mesh = half_edge::from_polygons(alloc, positions, face_offsets, face_indices)!;

    Vec3f[] final_sites = mem::alloc::new_array(alloc, Vec3f, (sz)n_surviving);
    defer catch { mesh.destroy(); free(final_sites); };
    for (usz i = 0; i < n_surviving; i++) final_sites[i] = result_sites[i];
    free(result_sites);

    return {
        .mesh = mesh,
        .sites = final_sites,
    };
}

fn VoronoiDiagram? in_box(Allocator alloc, Vec3f[] sites, Aabb bbox)
{
    Vec3f[4] polygon = {
        { bbox.min.x, bbox.min.y, 0 },
        { bbox.max.x, bbox.min.y, 0 },
        { bbox.max.x, bbox.max.y, 0 },
        { bbox.min.x, bbox.max.y, 0 },
    };
    return in_polygon(alloc, sites, polygon[..]);
}
