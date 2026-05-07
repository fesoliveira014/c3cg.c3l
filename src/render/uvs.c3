module cg::render;
import cg;
import cg::half_edge;

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
