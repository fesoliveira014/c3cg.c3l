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
    HeIndex start = self.vertices[v].half_edge;
    if (start == cg::INVALID_HE) return 0;

    HeIndex cursor = start;
    usz count = 0;

    while (true) {
        if (count >= out.len) return cg::OUTPUT_BUFFER_TOO_SMALL~;
        out[count] = cursor;
        count++;

        HeIndex prev_he = self.prev(cursor);
        HeIndex next_he = self.half_edges[prev_he].twin;
        if (next_he == cg::INVALID_HE) break;
        cursor = next_he;
        if (cursor == start) break;
    }

    return (int)count;
}

fn int? HalfEdgeMesh.vertex_one_ring_faces(&self, VertexIndex v, FaceIndex[] out)
{
    HeIndex start = self.vertices[v].half_edge;
    if (start == cg::INVALID_HE) return 0;

    HeIndex cursor = start;
    usz count = 0;

    while (true) {
        FaceIndex face = self.half_edges[cursor].face;
        if (face != cg::INVALID_FACE) {
            if (count >= out.len) return cg::OUTPUT_BUFFER_TOO_SMALL~;
            out[count] = face;
            count++;
        } else {
            break;
        }

        HeIndex prev_he = self.prev(cursor);
        HeIndex next_he = self.half_edges[prev_he].twin;
        if (next_he == cg::INVALID_HE) break;
        cursor = next_he;
        if (cursor == start) break;
    }

    return (int)count;
}
