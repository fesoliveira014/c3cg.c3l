module cg::render;
import cg;
import cg::half_edge;
import std::math;

fn float dot3(Vec3f a, Vec3f b)
{
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

fn Vec3f cross3(Vec3f a, Vec3f b)
{
    return {
        a.y * b.z - a.z * b.y,
        a.z * b.x - a.x * b.z,
        a.x * b.y - a.y * b.x,
    };
}

fn float length_squared3(Vec3f v)
{
    return dot3(v, v);
}

fn float length3(Vec3f v)
{
    return (float)math::sqrt((double)length_squared3(v));
}

fn Vec3f normalize_or_zero3(Vec3f v)
{
    float len_sq = length_squared3(v);
    if (len_sq <= PLANAR_EPSILON * PLANAR_EPSILON) {
        return { 0, 0, 0 };
    }
    float inv_len = 1.0f / length3(v);
    return { v.x * inv_len, v.y * inv_len, v.z * inv_len };
}

fn float clamp_float(float value, float min_value, float max_value)
{
    if (value < min_value) return min_value;
    if (value > max_value) return max_value;
    return value;
}

fn float abs_float(float value)
{
    if (value < 0.0f) return -value;
    return value;
}

fn usz max_face_degree(HalfEdgeMesh* mesh)
{
    usz max_deg = 0;
    for (usz f = 0; f < mesh.faces.len; f++) {
        int degree = mesh.face_degree((FaceIndex)(int)f);
        usz face_degree = (usz)degree;
        if (face_degree > max_deg) max_deg = face_degree;
    }
    return max_deg;
}

fn usz fan_triangle_count(HalfEdgeMesh* mesh)
{
    usz total = 0;
    for (usz f = 0; f < mesh.faces.len; f++) {
        int degree = mesh.face_degree((FaceIndex)(int)f);
        total += (usz)(degree - 2);
    }
    return total;
}

fn int? collect_face_vertices(HalfEdgeMesh* mesh, FaceIndex face, VertexIndex[] out)
{
    return mesh.face_vertices(face, out)!;
}

fn Vec3f? face_normal_from_vertices(HalfEdgeMesh* mesh, VertexIndex[] face_vertices, int degree)
{
    VertexIndex v0 = face_vertices[0];
    Vec3f p0 = mesh.positions[v0];

    for (int i = 1; i < degree - 1; i++) {
        Vec3f pi = mesh.positions[face_vertices[i]];
        Vec3f pn = mesh.positions[face_vertices[i + 1]];
        Vec3f raw = cross3({ pi.x - p0.x, pi.y - p0.y, pi.z - p0.z },
                           { pn.x - p0.x, pn.y - p0.y, pn.z - p0.z });
        if (length_squared3(raw) > PLANAR_EPSILON * PLANAR_EPSILON) {
            return normalize_or_zero3(raw);
        }
    }

    return cg::DEGENERATE_INPUT~;
}

fn Vec3f? face_normal(HalfEdgeMesh* mesh, FaceIndex face, VertexIndex[] scratch)
{
    int degree = collect_face_vertices(mesh, face, scratch)!;
    return face_normal_from_vertices(mesh, scratch, degree);
}

fn void? validate_polygon_planarity(HalfEdgeMesh* mesh, Allocator alloc, float epsilon)
{
    usz max_deg = max_face_degree(mesh);
    VertexIndex[] scratch = mem::alloc::new_array(alloc, VertexIndex, max_deg);
    defer free(scratch);

    for (usz f = 0; f < mesh.faces.len; f++) {
        int degree = collect_face_vertices(mesh, (FaceIndex)(int)f, scratch)!;
        if (degree == 3) continue;

        Vec3f? normal_result = face_normal_from_vertices(mesh, scratch, degree);
        if (catch err = normal_result) return err~;

        Vec3f plane_normal = normal_result;
        Vec3f plane_origin = mesh.positions[scratch[0]];

        for (int i = 0; i < degree; i++) {
            VertexIndex v = scratch[i];
            Vec3f delta = {
                mesh.positions[v].x - plane_origin.x,
                mesh.positions[v].y - plane_origin.y,
                mesh.positions[v].z - plane_origin.z,
            };
            float distance = abs_float(dot3(delta, plane_normal));
            if (distance > epsilon) return cg::NON_PLANAR_FACE~;
        }
    }
}

fn void? validate_all_faces_triangular_and_non_degenerate_if_flat(
    HalfEdgeMesh* mesh,
    Allocator alloc,
    RenderOptions options)
{
    if (options.normal_mode != (uint)RenderNormalMode.RENDER_NORMAL_FLAT) return;

    usz max_deg = max_face_degree(mesh);
    VertexIndex[] scratch = mem::alloc::new_array(alloc, VertexIndex, max_deg);
    defer free(scratch);

    for (usz f = 0; f < mesh.faces.len; f++) {
        int degree = collect_face_vertices(mesh, (FaceIndex)(int)f, scratch)!;
        if (degree != 3) return cg::NON_TRIANGLE_FACE~;

        Vec3f? normal_result = face_normal_from_vertices(mesh, scratch, degree);
        if (catch err = normal_result) return err~;
    }
}
