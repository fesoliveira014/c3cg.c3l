module cg::half_edge;
import cg;

fn void HalfEdgeMesh.destroy(&self)
{
    free(self.half_edges);
    free(self.faces);
    free(self.vertices);
    free(self.positions);
    free(self.normals);
    free(self.uvs);
    *self = {};
}

fn HalfEdgeMesh? from_triangles(Allocator alloc, Vec3f[] positions, uint[] indices)
{
    if (positions.len == 0 || indices.len == 0) return cg::EMPTY_INPUT~;
    if (indices.len % 3 != 0) return cg::INVALID_TRIANGLE_INDEX_COUNT~;
    foreach (index : indices) {
        if (index >= positions.len) return cg::INDEX_OUT_OF_RANGE~;
    }

    usz face_count = indices.len / 3;
    usz half_edge_count = indices.len;

    HalfEdgeMesh mesh;
    mesh.half_edges = mem::alloc::new_array(alloc, HalfEdge, half_edge_count);
    defer catch free(mesh.half_edges);
    mesh.faces = mem::alloc::new_array(alloc, HalfEdgeFace, face_count);
    defer catch free(mesh.faces);
    mesh.vertices = mem::alloc::new_array(alloc, HalfEdgeVertex, positions.len);
    defer catch free(mesh.vertices);
    mesh.positions = mem::alloc::new_array(alloc, Vec3f, positions.len);
    defer catch free(mesh.positions);

    for (usz i = 0; i < mesh.vertices.len; i++) {
        mesh.vertices[i].half_edge = cg::INVALID_HE;
        mesh.positions[i] = positions[i];
    }

    for (usz face = 0; face < face_count; face++) {
        usz base = face * 3;
        mesh.faces[face].half_edge = (HeIndex)(int)base;

        for (usz local = 0; local < 3; local++) {
            usz he_index = base + local;
            usz next_index = base + ((local + 1) % 3);
            uint origin_index = indices[he_index];

            mesh.half_edges[he_index].origin = (VertexIndex)(int)origin_index;
            mesh.half_edges[he_index].next = (HeIndex)(int)next_index;
            mesh.half_edges[he_index].twin = cg::INVALID_HE;
            mesh.half_edges[he_index].face = (FaceIndex)(int)face;

            if (mesh.vertices[origin_index].half_edge == cg::INVALID_HE) {
                mesh.vertices[origin_index].half_edge = (HeIndex)(int)he_index;
            }
        }
    }

    return mesh;
}

fn HalfEdgeMesh? from_triangles_with_attrs(
    Allocator alloc,
    Vec3f[] positions,
    Vec3f[] normals,
    Vec2f[] uvs,
    uint[] indices)
{
    return cg::EMPTY_INPUT~;
}

fn HalfEdgeMesh? from_polygons(Allocator alloc, Vec3f[] positions, uint[] face_offsets, uint[] face_indices)
{
    return cg::EMPTY_INPUT~;
}
