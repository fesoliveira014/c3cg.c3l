module cg::render;
import cg;
import cg::half_edge;

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
