module cg::render;
import cg;
import cg::half_edge;
import std::math;

fn void? validate_required_render_attrs(HalfEdgeMesh* mesh, RenderOptions options)
{
    if (options.normal_mode == (uint)RenderNormalMode.RENDER_NORMAL_SOURCE && mesh.normals.len == 0) {
        return cg::MISSING_NORMALS~;
    }
    if (options.uv_mode == (uint)RenderUvMode.RENDER_UV_SOURCE && mesh.uvs.len == 0) {
        return cg::MISSING_UVS~;
    }
}

fn void copy_source_normals(HalfEdgeMesh* mesh, Vec3f[] out_normals)
{
    for (usz i = 0; i < mesh.positions.len; i++) {
        out_normals[i] = mesh.normals[i];
    }
}

fn float angle_at(Vec3f a, Vec3f b, Vec3f c)
{
    Vec3f ab = { b.x - a.x, b.y - a.y, b.z - a.z };
    Vec3f ac = { c.x - a.x, c.y - a.y, c.z - a.z };

    float ab_len_sq = length_squared3(ab);
    float ac_len_sq = length_squared3(ac);

    if (ab_len_sq <= PLANAR_EPSILON * PLANAR_EPSILON) return 0.0f;
    if (ac_len_sq <= PLANAR_EPSILON * PLANAR_EPSILON) return 0.0f;

    float len_ab = length3(ab);
    float len_ac = length3(ac);

    Vec3f norm_ab = { ab.x / len_ab, ab.y / len_ab, ab.z / len_ab };
    Vec3f norm_ac = { ac.x / len_ac, ac.y / len_ac, ac.z / len_ac };

    float dot = dot3(norm_ab, norm_ac);
    dot = clamp_float(dot, -1.0f, 1.0f);

    return (float)math::acos((double)dot);
}

fn void? generate_smooth_normals(HalfEdgeMesh* mesh, Allocator alloc, Vec3f[] out_normals)
{
    usz vertex_count = mesh.positions.len;
    usz max_deg = max_face_degree(mesh);

    Vec3f[] accum = mem::alloc::new_array(alloc, Vec3f, vertex_count);
    defer free(accum);

    for (usz i = 0; i < vertex_count; i++) {
        accum[i] = { 0, 0, 0 };
    }

    VertexIndex[] face_vertices = mem::alloc::new_array(alloc, VertexIndex, max_deg);
    defer free(face_vertices);

    for (usz f = 0; f < mesh.faces.len; f++) {
        int degree = collect_face_vertices(mesh, (FaceIndex)(int)f, face_vertices)!;
        Vec3f p0 = mesh.positions[face_vertices[0]];

        for (int i = 1; i < degree - 1; i++) {
            VertexIndex v0 = face_vertices[0];
            VertexIndex v1 = face_vertices[i];
            VertexIndex v2 = face_vertices[i + 1];

            Vec3f p1 = mesh.positions[v1];
            Vec3f p2 = mesh.positions[v2];

            Vec3f raw = cross3({ p1.x - p0.x, p1.y - p0.y, p1.z - p0.z },
                               { p2.x - p0.x, p2.y - p0.y, p2.z - p0.z });

            float raw_len_sq = length_squared3(raw);
            if (raw_len_sq <= PLANAR_EPSILON * PLANAR_EPSILON) continue;

            Vec3f face_n = normalize_or_zero3(raw);

            float angle0 = angle_at(p0, p1, p2);
            float angle1 = angle_at(p1, p2, p0);
            float angle2 = angle_at(p2, p0, p1);

            accum[v0].x += face_n.x * angle0;
            accum[v0].y += face_n.y * angle0;
            accum[v0].z += face_n.z * angle0;

            accum[v1].x += face_n.x * angle1;
            accum[v1].y += face_n.y * angle1;
            accum[v1].z += face_n.z * angle1;

            accum[v2].x += face_n.x * angle2;
            accum[v2].y += face_n.y * angle2;
            accum[v2].z += face_n.z * angle2;
        }
    }

    for (usz i = 0; i < vertex_count; i++) {
        out_normals[i] = normalize_or_zero3(accum[i]);
    }
}
