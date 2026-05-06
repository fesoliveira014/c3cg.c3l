module cg::half_edge;
import cg;

fn int? HalfEdgeMesh.face_half_edges(&self, FaceIndex f, HeIndex[] out)
{
    HeIndex start = self.faces[f].half_edge;
    HeIndex cursor = start;
    usz count = 0;

    while (true) {
        if (count >= out.len) return cg::OUTPUT_BUFFER_TOO_SMALL~;
        out[count] = cursor;
        count++;

        cursor = self.half_edges[cursor].next;
        if (cursor == start) break;
    }

    return (int)count;
}

fn int? HalfEdgeMesh.face_vertices(&self, FaceIndex f, VertexIndex[] out)
{
    HeIndex start = self.faces[f].half_edge;
    HeIndex cursor = start;
    usz count = 0;

    while (true) {
        if (count >= out.len) return cg::OUTPUT_BUFFER_TOO_SMALL~;
        out[count] = self.half_edges[cursor].origin;
        count++;

        cursor = self.half_edges[cursor].next;
        if (cursor == start) break;
    }

    return (int)count;
}

fn int? HalfEdgeMesh.vertex_one_ring_outgoing(&self, VertexIndex v, HeIndex[] out)
{
    return cg::OUTPUT_BUFFER_TOO_SMALL~;
}

fn int? HalfEdgeMesh.vertex_one_ring_faces(&self, VertexIndex v, FaceIndex[] out)
{
    return cg::OUTPUT_BUFFER_TOO_SMALL~;
}
