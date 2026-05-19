module cg::dual;
import cg;
import cg::half_edge;
import cg::geometry;

enum DualMode : uint
{
    CIRCUMCENTER,
    CENTROID,
    SPHERICAL_CIRCUMCENTER,
}

const usz DUAL_SCRATCH_INITIAL = 3;
const usz DUAL_SCRATCH_GROWTH = 2;

fn HalfEdgeMesh? dual_from_vertices(Allocator alloc, HalfEdgeMesh* mesh, Vec3f[] dual_vertex_positions)
{
    mesh.validate()!;

    if (dual_vertex_positions.len != mesh.faces.len) return cg::DUAL_VERTEX_COUNT_MISMATCH~;

    for (usz i = 0; i < mesh.half_edges.len; i++) {
        if (mesh.half_edges[i].twin == cg::INVALID_HE) return cg::DUAL_REQUIRES_CLOSED_MESH~;
    }

    uint[] face_offsets = mem::alloc::new_array(alloc, uint, (sz)(mesh.vertices.len + 1));
    defer catch free(face_offsets);

    FaceIndex[] scratch = mem::alloc::new_array(alloc, FaceIndex, (sz) DUAL_SCRATCH_INITIAL);
    defer free(scratch);

    face_offsets[0] = 0;
    for (usz v = 0; v < mesh.vertices.len; v++) {
        int n = 0;
        while (true) {
            int? result = mesh.vertex_one_ring_faces((VertexIndex)(int)v, scratch);
            if (catch err = result) {
                if (err == cg::OUTPUT_BUFFER_TOO_SMALL) {
                    usz new_cap = scratch.len * DUAL_SCRATCH_GROWTH;
                    free(scratch);
                    scratch = mem::alloc::new_array(alloc, FaceIndex, (sz) new_cap);
                    continue;
                }
                return err~;
            }
            n = result;
            break;
        }
        face_offsets[v + 1] = face_offsets[v] + (uint)n;
    }

    uint[] face_indices = mem::alloc::new_array(alloc, uint, (sz) face_offsets[mesh.vertices.len]);
    defer catch free(face_indices);

    for (usz v = 0; v < mesh.vertices.len; v++) {
        int n = mesh.vertex_one_ring_faces((VertexIndex)(int)v, scratch)!;
        for (int i = 0; i < n; i++) {
            face_indices[face_offsets[v] + (uint)i] = (uint)scratch[i];
        }
    }

    HalfEdgeMesh result = half_edge::from_polygons(alloc, dual_vertex_positions, face_offsets, face_indices)!;

    free(face_offsets);
    free(face_indices);
    return result;
}

fn HalfEdgeMesh? dual(Allocator alloc, HalfEdgeMesh* mesh, DualMode mode = DualMode.CIRCUMCENTER, float radius = 1.0f)
{
    Vec3f[] positions;
    switch (mode) {
        case DualMode.CIRCUMCENTER:
            positions = geometry::circumcenters_planar(alloc, mesh)!;
        case DualMode.CENTROID:
            positions = geometry::face_centroids(alloc, mesh)!;
        case DualMode.SPHERICAL_CIRCUMCENTER:
            positions = geometry::circumcenters_on_sphere(alloc, mesh, radius)!;
    }
    defer free(positions);
    return dual_from_vertices(alloc, mesh, positions)!;
}
