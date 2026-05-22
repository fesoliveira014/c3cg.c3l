module cg::voronoi;
import cg;
import cg::geometry;
import cg::half_edge;
import cg::delaunay;
import cg::dual;
import std::math;

// ── Private helpers ──

fn Vec3f? line_intersection(Vec2f a, Vec2f b, Vec2f p, Vec2f q)
{
    float d1x = b.x - a.x;
    float d1y = b.y - a.y;
    float d2x = q.x - p.x;
    float d2y = q.y - p.y;
    float det = d1x * d2y - d1y * d2x;
    if (math::abs(det) < 1e-12) return {};
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

            float p_sign = (float)geometry::orient_2d(a2, b2, p2);
            float q_sign = (float)geometry::orient_2d(a2, b2, q2);
            bool p_in = p_sign >= 0;
            bool q_in = q_sign >= 0;

            if (p_in && q_in) {
                output[out_len] = q; out_len++;
            } else if (p_in && !q_in) {
                Vec3f? isect = line_intersection(a2, b2, p2, q2);
                if (try v = isect) { output[out_len] = v; out_len++; }
            } else if (!p_in && q_in) {
                Vec3f? isect = line_intersection(a2, b2, p2, q2);
                if (try v = isect) { output[out_len] = v; out_len++; }
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
        if (geometry::orient_2d({a.x,a.y}, {b.x,b.y}, {p.x,p.y}) < (PredicateSign)0) return false;
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
        float sign = (float)geometry::orient_2d({p.x,p.y}, {q.x,q.y}, {r.x,r.y});
        if (sign < 0) return false;
        if (sign > 0) has_strict = true;
    }
    return has_strict;
}

// ── Public API ──

fn VoronoiDiagram? in_polygon(Allocator alloc, Vec3f[] sites, Vec3f[] polygon)
{
    if (!is_convex_polygon(polygon)) return cg::NON_CONVEX_BOUNDING_POLYGON~;
    if (sites.len == 0) return cg::EMPTY_INPUT~;

    // Prefilter: collect sites inside/on polygon
    Vec3f[] inside_sites = mem::alloc::new_array(alloc, Vec3f, (sz)sites.len);
    defer free(inside_sites);
    usz inside_count = 0;
    for (usz i = 0; i < sites.len; i++) {
        if (point_in_convex_polygon(sites[i], polygon)) {
            inside_sites[inside_count] = sites[i];
            inside_count++;
        }
    }

    // N = 0: empty
    if (inside_count == 0) {
        VoronoiDiagram result;
        result.mesh = {};
        result.sites = {};
        return result;
    }

    // N = 1: cell = polygon
    if (inside_count == 1) {
        Vec3f[] positions = mem::alloc::new_array(alloc, Vec3f, (sz)polygon.len);
        defer free(positions);
        for (usz i = 0; i < polygon.len; i++) positions[i] = polygon[i];

        uint[] face_indices = mem::alloc::new_array(alloc, uint, (sz)polygon.len);
        defer free(face_indices);
        for (usz i = 0; i < polygon.len; i++) face_indices[i] = (uint)i;

        uint[] face_offsets = mem::alloc::new_array(alloc, uint, 2);
        defer free(face_offsets);
        face_offsets[0] = 0;
        face_offsets[1] = (uint)polygon.len;

        HalfEdgeMesh mesh = half_edge::from_polygons(alloc, positions, face_offsets, face_indices)!;

        Vec3f[] result_sites = mem::alloc::new_array(alloc, Vec3f, 1);
        defer catch { mesh.destroy(); free(result_sites); };
        result_sites[0] = inside_sites[0];

        VoronoiDiagram result;
        result.mesh = mesh;
        result.sites = result_sites;
        return result;
    }

    // N = 2: bisect polygon
    if (inside_count == 2) {
        Vec3f[] poly0 = half_plane_clip(alloc, polygon, inside_sites[0], inside_sites[1], true)!;
        defer free(poly0);
        Vec3f[] poly1 = half_plane_clip(alloc, polygon, inside_sites[0], inside_sites[1], false)!;
        defer free(poly1);

        usz n_cells = 0;
        if (poly0.len >= 3) n_cells++;
        if (poly1.len >= 3) n_cells++;

        if (n_cells == 0) {
            VoronoiDiagram result;
            result.mesh = {};
            result.sites = {};
            return result;
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

        VoronoiDiagram result;
        result.mesh = mesh;
        result.sites = result_sites;
        return result;
    }

    // ── N ≥ 3: helper-point Delaunay path ──

    // Compute polygon bbox
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
    Vec3f[4] helpers = {
        { min_x - w,     min_y + h/2, 0 },
        { max_x + w,     min_y + h/2, 0 },
        { min_x + w/2,   min_y - h,   0 },
        { min_x + w/2,   max_y + h,   0 },
    };

    usz total = inside_count + 4;
    Vec3f[] all_points = mem::alloc::new_array(alloc, Vec3f, (sz)total);
    defer free(all_points);
    for (usz i = 0; i < inside_count; i++) all_points[i] = inside_sites[i];
    for (usz i = 0; i < 4; i++) all_points[inside_count + i] = helpers[i];

    HalfEdgeMesh delaunay_mesh = delaunay::delaunay_2d(alloc, all_points)!;
    defer delaunay_mesh.destroy();
    VoronoiDiagram unbounded = voronoi::from_delaunay(alloc, &delaunay_mesh)!;
    defer unbounded.destroy();

    // Match inside_sites to Voronoi faces (linear scan, position equality within 1e-12)
    FaceIndex[] face_map = mem::alloc::new_array(alloc, FaceIndex, (sz)inside_count);
    defer free(face_map);
    for (usz i = 0; i < inside_count; i++) {
        face_map[i] = cg::INVALID_FACE;
        for (usz s = 0; s < unbounded.sites.len; s++) {
            float dx = unbounded.sites[s].x - inside_sites[i].x;
            float dy = unbounded.sites[s].y - inside_sites[i].y;
            if (dx * dx + dy * dy < (float)1e-24) {
                face_map[i] = (FaceIndex)(int)s;
                break;
            }
        }
    }

    // Allocate scratch for face_vertices
    VertexIndex[] scratch = mem::alloc::new_array(alloc, VertexIndex, (sz)unbounded.mesh.vertices.len);
    defer free(scratch);

    // Pass 1: count surviving cells and total vertices
    usz n_surviving = 0;
    usz total_verts = 0;
    for (usz i = 0; i < inside_count; i++) {
        if (face_map[i] == cg::INVALID_FACE) continue;
        int n = unbounded.mesh.face_vertices(face_map[i], scratch)!;
        if (n < 3) continue;

        Vec3f[] face_poly = mem::alloc::new_array(alloc, Vec3f, (sz)n);
        for (int j = 0; j < n; j++) face_poly[j] = unbounded.mesh.positions[scratch[j]];
        Vec3f[] clipped = sutherland_hodgman(alloc, face_poly, polygon)!;
        free(face_poly);
        if (clipped.len >= 3) {
            n_surviving++;
            total_verts += clipped.len;
        }
        free(clipped);
    }

    if (n_surviving == 0) {
        VoronoiDiagram result;
        result.mesh = {};
        result.sites = {};
        return result;
    }

    // Pass 2: build mesh from survivors
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
        if (face_map[i] == cg::INVALID_FACE) continue;
        int n = unbounded.mesh.face_vertices(face_map[i], scratch)!;
        if (n < 3) continue;

        Vec3f[] face_poly = mem::alloc::new_array(alloc, Vec3f, (sz)n);
        for (int j = 0; j < n; j++) face_poly[j] = unbounded.mesh.positions[scratch[j]];
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

    VoronoiDiagram result;
    result.mesh = mesh;
    result.sites = final_sites;
    return result;
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
