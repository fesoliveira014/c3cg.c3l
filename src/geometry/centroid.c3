module cg::geometry;
import cg;
import cg::half_edge;

fn Vec3f centroid_vec3_add(Vec3f a, Vec3f b)
{
    return { a.x + b.x, a.y + b.y, a.z + b.z };
}

fn Vec3f centroid_vec3_scale(Vec3f v, float scale)
{
    return { v.x * scale, v.y * scale, v.z * scale };
}

fn Vec3f? centroid_for_valid_face(HalfEdgeMesh* mesh, FaceIndex face)
{
    HeIndex start = mesh.faces[face].half_edge;
    HeIndex current = start;
    Vec3f sum = { 0, 0, 0 };
    int count = 0;

    while (true) {
        if (current == cg::INVALID_HE) return cg::INVALID_FACE_CYCLE~;
        if ((usz)count >= mesh.half_edges.len) return cg::INVALID_FACE_CYCLE~;

        HalfEdge edge = mesh.half_edges[current];
        if (edge.origin < 0 || (usz)edge.origin >= mesh.positions.len) return cg::INVALID_TOPOLOGY~;

        sum = centroid_vec3_add(sum, mesh.positions[edge.origin]);
        count++;
        current = edge.next;
        if (current == start) break;
    }

    if (count == 0) return cg::INVALID_FACE_CYCLE~;
    return centroid_vec3_scale(sum, 1.0f / (float)count);
}

fn Vec3f? face_centroid(HalfEdgeMesh* mesh, FaceIndex face)
{
    if (face < 0 || (usz)face >= mesh.faces.len) return cg::INVALID_FACE_REFERENCE~;
    mesh.validate()!;
    return centroid_for_valid_face(mesh, face);
}

fn Vec3f[]? face_centroids(Allocator alloc, HalfEdgeMesh* mesh)
{
    mesh.validate()!;

    Vec3f[] out = mem::alloc::new_array(alloc, Vec3f, (sz) mesh.faces.len);
    defer catch free(out);

    for (usz i = 0; i < mesh.faces.len; i++) {
        out[i] = centroid_for_valid_face(mesh, (FaceIndex)(int)i)!;
    }

    return out;
}
