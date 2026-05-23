module cg::subdivide;
import cg;
import cg::half_edge;
import std::math;

fn HalfEdgeMesh? loop_subdivide(Allocator alloc, HalfEdgeMesh* mesh)
{
    for (usz i = 0; i < mesh.faces.len; i++) {
        if (mesh.face_degree((FaceIndex)(int)i) != 3) return cg::NON_TRIANGLE_FACE~;
    }
    for (usz i = 0; i < mesh.half_edges.len; i++) {
        if (mesh.is_boundary((HeIndex)(int)i)) return cg::BOUNDARY_HALF_EDGE~;
    }

    usz v_count = mesh.vertices.len;
    usz he_count = mesh.half_edges.len;
    usz face_count = mesh.faces.len;
    usz edge_count = he_count / 2;
    usz new_v_count = v_count + edge_count;
    usz new_face_count = face_count * 4;
    usz new_index_count = new_face_count * 3;

    int[] edge_map = mem::alloc::new_array(alloc, int, (sz)he_count);
    defer free(edge_map);
    for (usz i = 0; i < he_count; i++) edge_map[i] = -1;

    int next_edge = 0;
    for (usz i = 0; i < he_count; i++) {
        HeIndex he = (HeIndex)(int)i;
        HeIndex twin = mesh.twin(he);
        if (he < twin) {
            edge_map[(usz)he] = (int)v_count + next_edge;
            edge_map[(usz)twin] = (int)v_count + next_edge;
            next_edge++;
        }
    }

    Vec3f[] new_positions = mem::alloc::new_array(alloc, Vec3f, (sz)new_v_count);
    defer free(new_positions);

    // Compute updated vertex positions (Loop formula)
    for (usz vi = 0; vi < v_count; vi++) {
        VertexIndex v = (VertexIndex)(int)vi;
        HeIndex[64] ring;
        int n = mesh.vertex_one_ring_outgoing(v, ring[..])!!;

        if (n <= 2) {
            new_positions[vi] = mesh.positions[vi];
            continue;
        }

        Vec3f sum_neighbors = { 0, 0, 0 };
        for (int r = 0; r < n; r++) {
            VertexIndex nb = mesh.to_vertex(ring[(usz)r]);
            Vec3f np = mesh.positions[(usz)nb];
            sum_neighbors.x += np.x;
            sum_neighbors.y += np.y;
            sum_neighbors.z += np.z;
        }

        double n_d = (double)n;
        double angle = 2.0 * math::PI / n_d;
        double cos_a = math::cos(angle);
        double term = 0.375 + 0.25 * cos_a;   // 3/8 + 1/4*cos(2π/n)
        double beta = (0.625 - term * term) / n_d;  // (5/8 - term²) / n

        float bn = (float)((double)n * beta);
        float b = (float)beta;
        Vec3f old = mesh.positions[vi];
        new_positions[vi] = {
            (1.0f - bn) * old.x + b * sum_neighbors.x,
            (1.0f - bn) * old.y + b * sum_neighbors.y,
            (1.0f - bn) * old.z + b * sum_neighbors.z,
        };
    }

    // Compute edge point positions
    for (usz ei = 0; ei < edge_count; ei++) {
        // Find the first half-edge mapped to this edge point
        HeIndex he = cg::INVALID_HE;
        for (usz i = 0; i < he_count; i++) {
            if (edge_map[i] == (int)(v_count + ei)) {
                he = (HeIndex)(int)i;
                break;
            }
        }

        VertexIndex a = mesh.from_vertex(he);
        VertexIndex b = mesh.to_vertex(he);
        VertexIndex c = mesh.to_vertex(mesh.next(he));
        VertexIndex d = mesh.to_vertex(mesh.next(mesh.twin(he)));

        Vec3f pa = mesh.positions[(usz)a];
        Vec3f pb = mesh.positions[(usz)b];
        Vec3f pc = mesh.positions[(usz)c];
        Vec3f pd = mesh.positions[(usz)d];

        new_positions[v_count + ei] = {
            0.375f * (pa.x + pb.x) + 0.125f * (pc.x + pd.x),
            0.375f * (pa.y + pb.y) + 0.125f * (pc.y + pd.y),
            0.375f * (pa.z + pb.z) + 0.125f * (pc.z + pd.z),
        };
    }

    // Build new index buffer
    uint[] new_indices = mem::alloc::new_array(alloc, uint, (sz)new_index_count);
    defer free(new_indices);
    usz idx_out = 0;

    for (usz fi = 0; fi < face_count; fi++) {
        FaceIndex f = (FaceIndex)(int)fi;
        HeIndex he0 = mesh.faces[fi].half_edge;
        HeIndex he1 = mesh.next(he0);
        HeIndex he2 = mesh.next(he1);

        VertexIndex v0 = mesh.from_vertex(he0);
        VertexIndex v1 = mesh.from_vertex(he1);
        VertexIndex v2 = mesh.from_vertex(he2);

        int e01 = edge_map[(usz)he0];
        int e12 = edge_map[(usz)he1];
        int e20 = edge_map[(usz)he2];

        // Triangle 1: v0, e01, e20
        new_indices[idx_out++] = (uint)(int)v0;
        new_indices[idx_out++] = (uint)e01;
        new_indices[idx_out++] = (uint)e20;

        // Triangle 2: v1, e12, e01
        new_indices[idx_out++] = (uint)(int)v1;
        new_indices[idx_out++] = (uint)e12;
        new_indices[idx_out++] = (uint)e01;

        // Triangle 3: v2, e20, e12
        new_indices[idx_out++] = (uint)(int)v2;
        new_indices[idx_out++] = (uint)e20;
        new_indices[idx_out++] = (uint)e12;

        // Triangle 4: e01, e12, e20
        new_indices[idx_out++] = (uint)e01;
        new_indices[idx_out++] = (uint)e12;
        new_indices[idx_out++] = (uint)e20;
    }

    return half_edge::from_triangles(alloc, new_positions[..], new_indices[..]);
}
