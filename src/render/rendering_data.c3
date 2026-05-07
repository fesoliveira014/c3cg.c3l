module cg::render;
import cg;
import cg::half_edge;

const float PLANAR_EPSILON = 0.00001;

enum RenderNormalMode : uint
{
    RENDER_NORMAL_NONE,
    RENDER_NORMAL_SOURCE,
    RENDER_NORMAL_SMOOTH,
    RENDER_NORMAL_FLAT,
}

enum RenderUvMode : uint
{
    RENDER_UV_NONE,
    RENDER_UV_SOURCE,
    RENDER_UV_SOURCE_OR_ZERO,
    RENDER_UV_PLANAR_PROJECT,
}

bitstruct RenderOptions : uint
{
    uint normal_mode : 0..1;
    uint uv_mode : 2..3;
    bool skip_planarity_check : 4;
    uint reserved : 5..31;
}

const RenderOptions DEFAULT_RENDER_OPTIONS = {
    .normal_mode = (uint)RenderNormalMode.RENDER_NORMAL_NONE,
    .uv_mode = (uint)RenderUvMode.RENDER_UV_NONE,
    .skip_planarity_check = false,
    .reserved = 0,
};

struct RenderingData
{
    Vec3f[] vertices;
    uint[] indices;
    Vec3f[] normals;
    Vec2f[] uvs;
}

fn void RenderingData.destroy(&self)
{
    free(self.vertices);
    free(self.indices);
    free(self.normals);
    free(self.uvs);
    *self = {};
}

fn void? validate_render_options(RenderOptions options)
{
    if (options.reserved != 0) return cg::INVALID_RENDER_OPTIONS~;
    if (options.normal_mode > (uint)RenderNormalMode.RENDER_NORMAL_FLAT) return cg::INVALID_RENDER_OPTIONS~;
    if (options.uv_mode > (uint)RenderUvMode.RENDER_UV_PLANAR_PROJECT) return cg::INVALID_RENDER_OPTIONS~;
}

fn RenderingData? build_shared_rendering_data(HalfEdgeMesh* mesh, Allocator alloc, RenderOptions options)
{
    usz triangle_count = fan_triangle_count(mesh);
    usz max_deg = max_face_degree(mesh);

    RenderingData data;

    data.vertices = mem::alloc::new_array(alloc, Vec3f, mesh.positions.len);
    defer catch free(data.vertices);
    data.indices = mem::alloc::new_array(alloc, uint, triangle_count * 3);
    defer catch free(data.indices);

    if (options.normal_mode == (uint)RenderNormalMode.RENDER_NORMAL_SOURCE) {
        data.normals = mem::alloc::new_array(alloc, Vec3f, mesh.positions.len);
    } else if (options.normal_mode == (uint)RenderNormalMode.RENDER_NORMAL_SMOOTH) {
        data.normals = mem::alloc::new_array(alloc, Vec3f, mesh.positions.len);
    } else {
        data.normals = mem::alloc::new_array(alloc, Vec3f, 0);
    }
    defer catch free(data.normals);

    if (options.uv_mode == (uint)RenderUvMode.RENDER_UV_SOURCE ||
        options.uv_mode == (uint)RenderUvMode.RENDER_UV_SOURCE_OR_ZERO ||
        options.uv_mode == (uint)RenderUvMode.RENDER_UV_PLANAR_PROJECT) {
        data.uvs = mem::alloc::new_array(alloc, Vec2f, mesh.positions.len);
    } else {
        data.uvs = mem::alloc::new_array(alloc, Vec2f, 0);
    }
    defer catch free(data.uvs);

    VertexIndex[] face_vertices = mem::alloc::new_array(alloc, VertexIndex, max_deg);
    defer free(face_vertices);

    for (usz i = 0; i < mesh.positions.len; i++) {
        data.vertices[i] = mesh.positions[i];
    }

    if (options.normal_mode == (uint)RenderNormalMode.RENDER_NORMAL_SOURCE) {
        copy_source_normals(mesh, data.normals);
    } else if (options.normal_mode == (uint)RenderNormalMode.RENDER_NORMAL_SMOOTH) {
        generate_smooth_normals(mesh, alloc, data.normals)!;
    }

    if (options.uv_mode == (uint)RenderUvMode.RENDER_UV_SOURCE) {
        copy_source_uvs(mesh, data.uvs);
    } else if (options.uv_mode == (uint)RenderUvMode.RENDER_UV_SOURCE_OR_ZERO) {
        copy_or_zero_source_uvs(mesh, data.uvs);
    } else if (options.uv_mode == (uint)RenderUvMode.RENDER_UV_PLANAR_PROJECT) {
        project_planar_uvs(mesh, alloc, data.uvs)!;
    }

    usz out_index = 0;
    for (usz f = 0; f < mesh.faces.len; f++) {
        int degree = mesh.face_vertices((FaceIndex)(int)f, face_vertices)!;
        VertexIndex v0 = face_vertices[0];
        for (int i = 1; i < degree - 1; i++) {
            data.indices[out_index] = (uint)v0;
            out_index++;
            data.indices[out_index] = (uint)face_vertices[i];
            out_index++;
            data.indices[out_index] = (uint)face_vertices[i + 1];
            out_index++;
        }
    }

    return data;
}

fn RenderingData? build_flat_rendering_data(HalfEdgeMesh* mesh, Allocator alloc, RenderOptions options)
{
    usz triangle_count = mesh.faces.len;
    usz expanded_count = triangle_count * 3;

    RenderingData data;

    data.vertices = mem::alloc::new_array(alloc, Vec3f, expanded_count);
    defer catch free(data.vertices);
    data.indices = mem::alloc::new_array(alloc, uint, expanded_count);
    defer catch free(data.indices);
    data.normals = mem::alloc::new_array(alloc, Vec3f, expanded_count);
    defer catch free(data.normals);

    if (options.uv_mode == (uint)RenderUvMode.RENDER_UV_SOURCE ||
        options.uv_mode == (uint)RenderUvMode.RENDER_UV_SOURCE_OR_ZERO ||
        options.uv_mode == (uint)RenderUvMode.RENDER_UV_PLANAR_PROJECT) {
        data.uvs = mem::alloc::new_array(alloc, Vec2f, expanded_count);
    } else {
        data.uvs = mem::alloc::new_array(alloc, Vec2f, 0);
    }
    defer catch free(data.uvs);

    usz max_deg = max_face_degree(mesh);
    VertexIndex[] face_vertices = mem::alloc::new_array(alloc, VertexIndex, max_deg);
    defer free(face_vertices);

    Vec2f[] shared_uvs = {};
    if (options.uv_mode == (uint)RenderUvMode.RENDER_UV_PLANAR_PROJECT) {
        shared_uvs = mem::alloc::new_array(alloc, Vec2f, mesh.positions.len);
        project_planar_uvs(mesh, alloc, shared_uvs)!;
    }
    defer free(shared_uvs);

    usz out = 0;
    for (usz f = 0; f < mesh.faces.len; f++) {
        int degree = collect_face_vertices(mesh, (FaceIndex)(int)f, face_vertices)!;
        VertexIndex v0 = face_vertices[0];
        VertexIndex v1 = face_vertices[1];
        VertexIndex v2 = face_vertices[2];

        data.vertices[out] = mesh.positions[v0];
        data.vertices[out + 1] = mesh.positions[v1];
        data.vertices[out + 2] = mesh.positions[v2];

        data.indices[out] = (uint)out;
        data.indices[out + 1] = (uint)(out + 1);
        data.indices[out + 2] = (uint)(out + 2);

        write_flat_triangle_normal(mesh, v0, v1, v2, data.normals, out)!;

        if (options.uv_mode == (uint)RenderUvMode.RENDER_UV_SOURCE) {
            data.uvs[out] = mesh.uvs[v0];
            data.uvs[out + 1] = mesh.uvs[v1];
            data.uvs[out + 2] = mesh.uvs[v2];
        } else if (options.uv_mode == (uint)RenderUvMode.RENDER_UV_SOURCE_OR_ZERO) {
            if (mesh.uvs.len > 0) {
                data.uvs[out] = mesh.uvs[v0];
                data.uvs[out + 1] = mesh.uvs[v1];
                data.uvs[out + 2] = mesh.uvs[v2];
            } else {
                data.uvs[out] = { 0, 0 };
                data.uvs[out + 1] = { 0, 0 };
                data.uvs[out + 2] = { 0, 0 };
            }
        } else if (options.uv_mode == (uint)RenderUvMode.RENDER_UV_PLANAR_PROJECT) {
            data.uvs[out] = shared_uvs[v0];
            data.uvs[out + 1] = shared_uvs[v1];
            data.uvs[out + 2] = shared_uvs[v2];
        }

        out += 3;
    }

    return data;
}

fn RenderingData? HalfEdgeMesh.to_rendering_data(
    &self,
    Allocator alloc,
    RenderOptions options = DEFAULT_RENDER_OPTIONS)
{
    validate_render_options(options)!;
    self.validate()!;
    validate_required_render_attrs(self, options)!;
    validate_all_faces_triangular_and_non_degenerate_if_flat(self, alloc, options)!;
    if (!options.skip_planarity_check) {
        validate_polygon_planarity(self, alloc, PLANAR_EPSILON)!;
    }

    if (options.normal_mode == (uint)RenderNormalMode.RENDER_NORMAL_FLAT) {
        return build_flat_rendering_data(self, alloc, options)!;
    }

    return build_shared_rendering_data(self, alloc, options)!;
}
