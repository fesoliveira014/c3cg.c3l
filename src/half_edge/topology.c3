module cg::half_edge;
import cg;

fn HeIndex HalfEdgeMesh.twin(&self, HeIndex he)
{
    return self.half_edges[he].twin;
}

fn HeIndex HalfEdgeMesh.next(&self, HeIndex he)
{
    return self.half_edges[he].next;
}

fn HeIndex HalfEdgeMesh.prev(&self, HeIndex he)
{
    HeIndex cursor = self.faces[self.half_edges[he].face].half_edge;
    while (true) {
        if (self.half_edges[cursor].next == he) return cursor;
        cursor = self.half_edges[cursor].next;
    }
}

fn VertexIndex HalfEdgeMesh.from_vertex(&self, HeIndex he)
{
    return self.half_edges[he].origin;
}

fn VertexIndex HalfEdgeMesh.to_vertex(&self, HeIndex he)
{
    HeIndex next_he = self.half_edges[he].next;
    return self.half_edges[next_he].origin;
}

fn FaceIndex HalfEdgeMesh.face_of(&self, HeIndex he)
{
    return self.half_edges[he].face;
}

fn bool HalfEdgeMesh.is_boundary(&self, HeIndex he)
{
    return self.half_edges[he].twin == cg::INVALID_HE;
}

fn int HalfEdgeMesh.face_degree(&self, FaceIndex f)
{
    HeIndex start = self.faces[f].half_edge;
    HeIndex cursor = start;
    int count = 0;
    while (true) {
        count++;
        cursor = self.half_edges[cursor].next;
        if (cursor == start) break;
    }
    return count;
}
