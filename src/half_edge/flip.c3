module cg::half_edge;
import cg;

fn bool is_valid_flip_he_index(HalfEdgeMesh* mesh, HeIndex he)
{
    if (he < 0) return false;
    return (usz)he < mesh.half_edges.len;
}

fn void? validate_flip(HalfEdgeMesh* mesh, HeIndex he)
{
    if (!is_valid_flip_he_index(mesh, he)) return cg::INVALID_HALF_EDGE_REFERENCE~;

    HeIndex twin = mesh.half_edges[he].twin;
    if (twin == cg::INVALID_HE) return cg::BOUNDARY_HALF_EDGE~;

    return cg::NON_TRIANGLE_FACE~;
}

fn bool HalfEdgeMesh.is_flip_ok(&self, HeIndex he)
{
    if (catch err = validate_flip(self, he)) {
        return false;
    }
    return true;
}

fn void? HalfEdgeMesh.flip(&self, HeIndex he)
{
    validate_flip(self, he)!;
    return;
}
