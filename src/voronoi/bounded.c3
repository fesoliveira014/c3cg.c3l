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
        VoronoiDiagram result;
        result.mesh = {};
        result.sites = {};
        return result;
    }

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

    // N >= 3 — placeholder
    VoronoiDiagram result;
    result.mesh = {};
    result.sites = {};
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
