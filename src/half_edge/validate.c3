module cg::half_edge;
import cg;

fn bool is_valid_he_index(HalfEdgeMesh* mesh, HeIndex he)
{
    if (he < 0) return false;
    return (usz)he < mesh.half_edges.len;
}

fn bool is_valid_vertex_index(HalfEdgeMesh* mesh, VertexIndex v)
{
    if (v < 0) return false;
    return (usz)v < mesh.vertices.len;
}

fn bool is_valid_face_index(HalfEdgeMesh* mesh, FaceIndex f)
{
    if (f < 0) return false;
    return (usz)f < mesh.faces.len;
}

fn void? HalfEdgeMesh.validate(&self)
{
    if (self.half_edges.len == 0 || self.faces.len == 0 || self.vertices.len == 0) {
        return cg::INVALID_TOPOLOGY~;
    }
    if (self.positions.len != self.vertices.len) return cg::INVALID_VERTEX_REFERENCE~;
    if (self.normals.len > 0 && self.normals.len != self.positions.len) return cg::ATTRIBUTE_COUNT_MISMATCH~;
    if (self.uvs.len > 0 && self.uvs.len != self.positions.len) return cg::ATTRIBUTE_COUNT_MISMATCH~;

    for (usz i = 0; i < self.half_edges.len; i++) {
        HalfEdge he = self.half_edges[i];
        if (!is_valid_vertex_index(self, he.origin)) return cg::INVALID_VERTEX_REFERENCE~;
        if (!is_valid_he_index(self, he.next)) return cg::INVALID_HALF_EDGE_REFERENCE~;
        if (he.twin != cg::INVALID_HE && !is_valid_he_index(self, he.twin)) return cg::INVALID_HALF_EDGE_REFERENCE~;
        if (!is_valid_face_index(self, he.face)) return cg::INVALID_FACE_REFERENCE~;
    }

    for (usz i = 0; i < self.faces.len; i++) {
        if (!is_valid_he_index(self, self.faces[i].half_edge)) return cg::INVALID_HALF_EDGE_REFERENCE~;
    }

    for (usz i = 0; i < self.vertices.len; i++) {
        HeIndex he = self.vertices[i].half_edge;
        if (he == cg::INVALID_HE) continue;
        if (!is_valid_he_index(self, he)) return cg::INVALID_HALF_EDGE_REFERENCE~;
        VertexIndex origin = self.half_edges[he].origin;
        if ((usz)origin != i) return cg::INVALID_VERTEX_REFERENCE~;
    }

    return;
}
