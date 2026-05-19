module cg::render;
import cg;
import cg::half_edge;

const int PROJECT_DROP_X = 0;
const int PROJECT_DROP_Y = 1;
const int PROJECT_DROP_Z = 2;

fn void fill_zero_uvs(Vec2f[] out_uvs)
{
    for (usz i = 0; i < out_uvs.len; i++) {
        out_uvs[i] = { 0, 0 };
    }
}

fn void copy_source_uvs(HalfEdgeMesh* mesh, Vec2f[] out_uvs)
{
    for (usz i = 0; i < mesh.positions.len; i++) {
        out_uvs[i] = mesh.uvs[i];
    }
}

fn void copy_or_zero_source_uvs(HalfEdgeMesh* mesh, Vec2f[] out_uvs)
{
    if (mesh.uvs.len > 0) {
        copy_source_uvs(mesh, out_uvs);
    } else {
        fill_zero_uvs(out_uvs);
    }
}

fn int? choose_projection_axis(HalfEdgeMesh* mesh, Allocator alloc)
{
    usz max_deg = max_face_degree(mesh);
    VertexIndex[] face_vertices = mem::alloc::new_array(alloc, VertexIndex, (sz) max_deg);
    defer free(face_vertices);

    Vec3f normal_sum = { 0, 0, 0 };
    bool contributed = false;

    for (usz f = 0; f < mesh.faces.len; f++) {
        Vec3f? result = face_normal(mesh, (FaceIndex)(int)f, face_vertices);
        if (catch err = result) {
            if (err == cg::DEGENERATE_INPUT) continue;
            return err~;
        }
        Vec3f normal = result;
        normal_sum.x += normal.x;
        normal_sum.y += normal.y;
        normal_sum.z += normal.z;
        contributed = true;
    }

    if (!contributed || length_squared3(normal_sum) <= PLANAR_EPSILON * PLANAR_EPSILON) {
        return PROJECT_DROP_Z;
    }

    float abs_x = abs_float(normal_sum.x);
    float abs_y = abs_float(normal_sum.y);
    float abs_z = abs_float(normal_sum.z);

    if (abs_z >= abs_x && abs_z >= abs_y) return PROJECT_DROP_Z;
    if (abs_y >= abs_x && abs_y > abs_z) return PROJECT_DROP_Y;
    return PROJECT_DROP_X;
}

fn Vec2f project_position_to_axis(Vec3f position, int axis)
{
    if (axis == PROJECT_DROP_Z) return { position.x, position.y };
    if (axis == PROJECT_DROP_Y) return { position.x, position.z };
    return { position.y, position.z };
}

fn void? project_planar_uvs(HalfEdgeMesh* mesh, Allocator alloc, Vec2f[] out_uvs)
{
    int axis = choose_projection_axis(mesh, alloc)!;

    if (mesh.positions.len == 0) return;

    float min_u;
    float max_u;
    float min_v;
    float max_v;

    Vec2f first = project_position_to_axis(mesh.positions[0], axis);
    out_uvs[0] = first;
    min_u = first.x;
    max_u = first.x;
    min_v = first.y;
    max_v = first.y;

    for (usz i = 1; i < mesh.positions.len; i++) {
        Vec2f projected = project_position_to_axis(mesh.positions[i], axis);
        out_uvs[i] = projected;
        if (projected.x < min_u) min_u = projected.x;
        if (projected.x > max_u) max_u = projected.x;
        if (projected.y < min_v) min_v = projected.y;
        if (projected.y > max_v) max_v = projected.y;
    }

    float width = max_u - min_u;
    float height = max_v - min_v;

    for (usz i = 0; i < mesh.positions.len; i++) {
        float u;
        float v;
        if (abs_float(width) <= PLANAR_EPSILON) {
            u = 0;
        } else {
            u = (out_uvs[i].x - min_u) / width;
        }
        if (abs_float(height) <= PLANAR_EPSILON) {
            v = 0;
        } else {
            v = (out_uvs[i].y - min_v) / height;
        }
        out_uvs[i] = { u, v };
    }
}
