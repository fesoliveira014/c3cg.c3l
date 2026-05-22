module cg::dual;
import cg;
import cg::half_edge;
import cg::geometry;

const usz DUAL_SCRATCH_INITIAL = 3;
const usz DUAL_SCRATCH_GROWTH = 2;
const float DUAL_DEFAULT_RADIUS = 1.0f;

fn HalfEdgeMesh? dual_from_vertices(Allocator alloc, HalfEdgeMesh* mesh, Vec3f[] dual_vertex_positions)
{
    mesh.validate()!;

    if (dual_vertex_positions.len != mesh.faces.len) return cg::DUAL_VERTEX_COUNT_MISMATCH~;

    // Boundary edges are allowed. Vertices with incomplete face rings
    // (n < 3) are skipped and excluded from the output.

    FaceIndex[] scratch = mem::alloc::new_array(alloc, FaceIndex, (sz) DUAL_SCRATCH_INITIAL);
    defer free(scratch);

    // Pass 1: compute ring sizes for every vertex, mark survivors
    int[] ring_sizes = mem::alloc::new_array(alloc, int, (sz) mesh.vertices.len);
    defer free(ring_sizes);
    usz surviving_count = 0;
    for (usz v = 0; v < mesh.vertices.len; v++) {
        int n = 0;
        while (true) {
            int? result = mesh.vertex_one_ring_faces((VertexIndex)(int)v, scratch);
            if (catch err = result) {
                if (err == cg::OUTPUT_BUFFER_TOO_SMALL) {
                    usz new_cap = scratch.len * DUAL_SCRATCH_GROWTH;
                    FaceIndex[] new_scratch = mem::alloc::new_array(alloc, FaceIndex, (sz) new_cap);
                    free(scratch);
                    scratch = new_scratch;
                    continue;
                }
                return err~;
            }
            n = result;
            break;
        }
        ring_sizes[v] = n;
        if (n >= 3) surviving_count++;
    }

    if (surviving_count == 0) {
        HalfEdgeMesh result;
        result.half_edges = {};
        result.faces = {};
        result.vertices = {};
        result.positions = {};
        result.normals = {};
        result.uvs = {};
        return result;
    }

    // Pass 2: build compacted face_offsets + face_indices
    uint[] face_offsets = mem::alloc::new_array(alloc, uint, (sz)(surviving_count + 1));
    defer free(face_offsets);

    usz total_indices = 0;
    usz out_f = 0;
    for (usz v = 0; v < mesh.vertices.len; v++) {
        if (ring_sizes[v] < 3) continue;
        total_indices += (usz)ring_sizes[v];
    }

    uint[] face_indices = mem::alloc::new_array(alloc, uint, (sz) total_indices);
    defer free(face_indices);

    face_offsets[0] = 0;
    out_f = 0;
    usz idx_offset = 0;
    for (usz v = 0; v < mesh.vertices.len; v++) {
        if (ring_sizes[v] < 3) continue;
        int n = mesh.vertex_one_ring_faces((VertexIndex)(int)v, scratch)!;
        for (int i = 0; i < n; i++) {
            face_indices[idx_offset + (usz)i] = (uint)scratch[i];
        }
        idx_offset += (usz)n;
        out_f++;
        face_offsets[out_f] = (uint)idx_offset;
    }

    HalfEdgeMesh result = half_edge::from_polygons(
        alloc:              alloc,
        positions:          dual_vertex_positions,
        face_offsets:       face_offsets,
        face_indices:       face_indices,
    )!;
    return result;
}

fn HalfEdgeMesh? dual(Allocator alloc, HalfEdgeMesh* mesh, DualMode mode = DualMode.CIRCUMCENTER, float radius = DUAL_DEFAULT_RADIUS)
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
