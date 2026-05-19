module cg::half_edge;
import cg;
import std::collections::map;

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
    Vec3f[] no_normals = {};
    Vec2f[] no_uvs = {};
    return from_triangles_with_attrs(alloc, positions, no_normals, no_uvs, indices);
}

fn HalfEdgeMesh? from_triangles_with_attrs(
    Allocator alloc,
    Vec3f[] positions,
    Vec3f[] normals,
    Vec2f[] uvs,
    uint[] indices)
{
    if (positions.len == 0 || indices.len == 0) return cg::EMPTY_INPUT~;
    if (indices.len % 3 != 0) return cg::INVALID_TRIANGLE_INDEX_COUNT~;
    if (normals.len > 0 && normals.len != positions.len) return cg::ATTRIBUTE_COUNT_MISMATCH~;
    if (uvs.len > 0 && uvs.len != positions.len) return cg::ATTRIBUTE_COUNT_MISMATCH~;
    foreach (index : indices) {
        if (index >= positions.len) return cg::INDEX_OUT_OF_RANGE~;
    }

    usz face_count = indices.len / 3;
    usz half_edge_count = indices.len;

    HalfEdgeMesh mesh;
    mesh.half_edges = mem::alloc::new_array(alloc, HalfEdge, (sz) half_edge_count);
    defer catch free(mesh.half_edges);
    mesh.faces = mem::alloc::new_array(alloc, HalfEdgeFace, (sz) face_count);
    defer catch free(mesh.faces);
    mesh.vertices = mem::alloc::new_array(alloc, HalfEdgeVertex, (sz) positions.len);
    defer catch free(mesh.vertices);
    mesh.positions = mem::alloc::new_array(alloc, Vec3f, (sz) positions.len);
    defer catch free(mesh.positions);

    if (normals.len > 0) {
        mesh.normals = mem::alloc::new_array(alloc, Vec3f, (sz) normals.len);
        defer catch free(mesh.normals);
    }
    if (uvs.len > 0) {
        mesh.uvs = mem::alloc::new_array(alloc, Vec2f, (sz) uvs.len);
        defer catch free(mesh.uvs);
    }

    for (usz i = 0; i < mesh.vertices.len; i++) {
        mesh.vertices[i].half_edge = cg::INVALID_HE;
        mesh.positions[i] = positions[i];
    }
    for (usz i = 0; i < normals.len; i++) {
        mesh.normals[i] = normals[i];
    }
    for (usz i = 0; i < uvs.len; i++) {
        mesh.uvs[i] = uvs[i];
    }

    HashMap{int[<2>], HeIndex} edge_map;
    edge_map.init(alloc);
    defer catch edge_map.free();

    for (usz face = 0; face < face_count; face++) {
        usz base = face * 3;
        mesh.faces[face].half_edge = (HeIndex)(int)base;

        for (usz local = 0; local < 3; local++) {
            usz he_index = base + local;
            usz next_index = base + ((local + 1) % 3);
            uint origin_index = indices[he_index];
            uint dest_index = indices[next_index];

            mesh.half_edges[he_index].origin = (VertexIndex)(int)origin_index;
            mesh.half_edges[he_index].next = (HeIndex)(int)next_index;
            mesh.half_edges[he_index].twin = cg::INVALID_HE;
            mesh.half_edges[he_index].face = (FaceIndex)(int)face;

            int[<2>] key = { (int)origin_index, (int)dest_index };
            if (edge_map.has_key(key)) return cg::DUPLICATE_HALF_EDGE~;
            edge_map[key] = (HeIndex)(int)he_index;

            if (mesh.vertices[origin_index].half_edge == cg::INVALID_HE) {
                mesh.vertices[origin_index].half_edge = (HeIndex)(int)he_index;
            }
        }
    }

    for (usz he_index = 0; he_index < half_edge_count; he_index++) {
        VertexIndex from_v = mesh.half_edges[he_index].origin;
        VertexIndex to_v = mesh.half_edges[mesh.half_edges[he_index].next].origin;
        int[<2>] reverse_key = { (int)to_v, (int)from_v };
        if (try twin_he = edge_map[reverse_key]) {
            mesh.half_edges[he_index].twin = twin_he;
        }
    }

    edge_map.free();
    return mesh;
}

fn HalfEdgeMesh? from_polygons(Allocator alloc, Vec3f[] positions, uint[] face_offsets, uint[] face_indices)
{
    if (positions.len == 0 || face_offsets.len < 2 || face_indices.len == 0) return cg::EMPTY_INPUT~;
    if (face_offsets[0] != 0) return cg::INDEX_OUT_OF_RANGE~;
    if (face_offsets[face_offsets.len - 1] != face_indices.len) return cg::INDEX_OUT_OF_RANGE~;

    usz face_count = face_offsets.len - 1;
    usz half_edge_count = face_indices.len;

    for (usz f = 0; f < face_count; f++) {
        usz start = face_offsets[f];
        usz end = face_offsets[f + 1];
        if (end < start) return cg::INDEX_OUT_OF_RANGE~;
        if (end - start < 3) return cg::DEGENERATE_INPUT~;
    }

    foreach (index : face_indices) {
        if (index >= positions.len) return cg::INDEX_OUT_OF_RANGE~;
    }

    HalfEdgeMesh mesh;
    mesh.half_edges = mem::alloc::new_array(alloc, HalfEdge, (sz) half_edge_count);
    defer catch free(mesh.half_edges);
    mesh.faces = mem::alloc::new_array(alloc, HalfEdgeFace, (sz) face_count);
    defer catch free(mesh.faces);
    mesh.vertices = mem::alloc::new_array(alloc, HalfEdgeVertex, (sz) positions.len);
    defer catch free(mesh.vertices);
    mesh.positions = mem::alloc::new_array(alloc, Vec3f, (sz) positions.len);
    defer catch free(mesh.positions);

    for (usz i = 0; i < mesh.vertices.len; i++) {
        mesh.vertices[i].half_edge = cg::INVALID_HE;
        mesh.positions[i] = positions[i];
    }

    HashMap{int[<2>], HeIndex} edge_map;
    edge_map.init(alloc);
    defer catch edge_map.free();

    for (usz face = 0; face < face_count; face++) {
        usz start = face_offsets[face];
        usz end = face_offsets[face + 1];
        usz degree = end - start;
        mesh.faces[face].half_edge = (HeIndex)(int)start;

        for (usz local = 0; local < degree; local++) {
            usz he_index = start + local;
            usz next_index = start + ((local + 1) % degree);
            uint origin_index = face_indices[he_index];
            uint dest_index = face_indices[next_index];

            mesh.half_edges[he_index].origin = (VertexIndex)(int)origin_index;
            mesh.half_edges[he_index].next = (HeIndex)(int)next_index;
            mesh.half_edges[he_index].twin = cg::INVALID_HE;
            mesh.half_edges[he_index].face = (FaceIndex)(int)face;

            int[<2>] key = { (int)origin_index, (int)dest_index };
            if (edge_map.has_key(key)) return cg::DUPLICATE_HALF_EDGE~;
            edge_map[key] = (HeIndex)(int)he_index;

            if (mesh.vertices[origin_index].half_edge == cg::INVALID_HE) {
                mesh.vertices[origin_index].half_edge = (HeIndex)(int)he_index;
            }
        }
    }

    for (usz he_index = 0; he_index < half_edge_count; he_index++) {
        VertexIndex from_v = mesh.half_edges[he_index].origin;
        VertexIndex to_v = mesh.half_edges[mesh.half_edges[he_index].next].origin;
        int[<2>] reverse_key = { (int)to_v, (int)from_v };
        if (try twin_he = edge_map[reverse_key]) {
            mesh.half_edges[he_index].twin = twin_he;
        }
    }

    edge_map.free();
    return mesh;
}
