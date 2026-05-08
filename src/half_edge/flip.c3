module cg::half_edge;
import cg;

fn bool is_valid_flip_he_index(HalfEdgeMesh* mesh, HeIndex he)
{
    if (he < 0) return false;
    return (usz)he < mesh.half_edges.len;
}

fn bool edge_exists_excluding(
    HalfEdgeMesh* mesh,
    VertexIndex from,
    VertexIndex to,
    HeIndex excluded_a,
    HeIndex excluded_b)
{
    for (usz i = 0; i < mesh.half_edges.len; i++) {
        HeIndex candidate = (HeIndex)(int)i;
        if (candidate == excluded_a || candidate == excluded_b) continue;
        if (mesh.half_edges[i].origin != from) continue;
        if (mesh.to_vertex(candidate) == to) return true;
    }
    return false;
}

fn void? validate_flip(HalfEdgeMesh* mesh, HeIndex he)
{
    if (!is_valid_flip_he_index(mesh, he)) return cg::INVALID_HALF_EDGE_REFERENCE~;

    HeIndex h0 = he;
    HeIndex t0 = mesh.half_edges[h0].twin;
    if (t0 == cg::INVALID_HE) return cg::BOUNDARY_HALF_EDGE~;

    FaceIndex f0 = mesh.half_edges[h0].face;
    FaceIndex f1 = mesh.half_edges[t0].face;
    if (mesh.face_degree(f0) != 3) return cg::NON_TRIANGLE_FACE~;
    if (mesh.face_degree(f1) != 3) return cg::NON_TRIANGLE_FACE~;

    HeIndex h1 = mesh.half_edges[h0].next;
    HeIndex h2 = mesh.half_edges[h1].next;
    HeIndex t1 = mesh.half_edges[t0].next;
    HeIndex t2 = mesh.half_edges[t1].next;

    VertexIndex c = mesh.half_edges[h2].origin;
    VertexIndex d = mesh.half_edges[t2].origin;
    if (c == d) return cg::DEGENERATE_INPUT~;

    if (edge_exists_excluding(mesh, c, d, h0, t0)) return cg::EDGE_ALREADY_EXISTS~;
    if (edge_exists_excluding(mesh, d, c, h0, t0)) return cg::EDGE_ALREADY_EXISTS~;

    return;
}

fn bool HalfEdgeMesh.is_flip_ok(&self, HeIndex he)
{
    if (catch err = validate_flip(self, he)) {
        return false;
    }
    return true;
}

fn bool local_half_edge_origin_matches(HalfEdgeMesh* mesh, HeIndex he, VertexIndex v)
{
    if (!is_valid_flip_he_index(mesh, he)) return false;
    return mesh.half_edges[he].origin == v;
}

fn void refresh_vertex_handle_from_local(
    HalfEdgeMesh* mesh,
    VertexIndex v,
    HeIndex h0,
    HeIndex h1,
    HeIndex h2,
    HeIndex t0,
    HeIndex t1,
    HeIndex t2)
{
    if (local_half_edge_origin_matches(mesh, h0, v)) {
        mesh.vertices[v].half_edge = h0;
        return;
    }
    if (local_half_edge_origin_matches(mesh, h1, v)) {
        mesh.vertices[v].half_edge = h1;
        return;
    }
    if (local_half_edge_origin_matches(mesh, h2, v)) {
        mesh.vertices[v].half_edge = h2;
        return;
    }
    if (local_half_edge_origin_matches(mesh, t0, v)) {
        mesh.vertices[v].half_edge = t0;
        return;
    }
    if (local_half_edge_origin_matches(mesh, t1, v)) {
        mesh.vertices[v].half_edge = t1;
        return;
    }
    if (local_half_edge_origin_matches(mesh, t2, v)) {
        mesh.vertices[v].half_edge = t2;
        return;
    }

    HeIndex current = mesh.vertices[v].half_edge;
    if (is_valid_flip_he_index(mesh, current) && mesh.half_edges[current].origin == v) return;

    for (usz i = 0; i < mesh.half_edges.len; i++) {
        if (mesh.half_edges[i].origin == v) {
            mesh.vertices[v].half_edge = (HeIndex)(int)i;
            return;
        }
    }

    mesh.vertices[v].half_edge = cg::INVALID_HE;
}

fn void? HalfEdgeMesh.flip(&self, HeIndex he)
{
    validate_flip(self, he)!;

    HeIndex h0 = he;
    HeIndex h1 = self.half_edges[h0].next;
    HeIndex h2 = self.half_edges[h1].next;

    HeIndex t0 = self.half_edges[h0].twin;
    HeIndex t1 = self.half_edges[t0].next;
    HeIndex t2 = self.half_edges[t1].next;

    FaceIndex f0 = self.half_edges[h0].face;
    FaceIndex f1 = self.half_edges[t0].face;

    VertexIndex a = self.half_edges[h0].origin;
    VertexIndex b = self.half_edges[t0].origin;
    VertexIndex c = self.half_edges[h2].origin;
    VertexIndex d = self.half_edges[t2].origin;

    self.half_edges[h0].origin = c;
    self.half_edges[t0].origin = d;

    self.half_edges[h0].next = t2;
    self.half_edges[t2].next = h1;
    self.half_edges[h1].next = h0;

    self.half_edges[t0].next = h2;
    self.half_edges[h2].next = t1;
    self.half_edges[t1].next = t0;

    self.half_edges[h0].face = f0;
    self.half_edges[t2].face = f0;
    self.half_edges[h1].face = f0;

    self.half_edges[t0].face = f1;
    self.half_edges[h2].face = f1;
    self.half_edges[t1].face = f1;

    self.faces[f0].half_edge = h0;
    self.faces[f1].half_edge = t0;

    refresh_vertex_handle_from_local(self, a, h0, h1, h2, t0, t1, t2);
    refresh_vertex_handle_from_local(self, b, h0, h1, h2, t0, t1, t2);
    refresh_vertex_handle_from_local(self, c, h0, h1, h2, t0, t1, t2);
    refresh_vertex_handle_from_local(self, d, h0, h1, h2, t0, t1, t2);

    return;
}
