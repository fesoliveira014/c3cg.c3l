module cg::graph;
import cg;
import cg::geometry;

fn void VoronoiGraph.destroy(&self)
{
    free(self.vertices);
    free(self.sites);
    free(self.centroids);
    free(self.cell_offsets);
    free(self.ring_indices);
    free(self.neighbor_indices);
    *self = {};
}

fn int VoronoiGraph.cell_count(&self)
{
    return (int)self.sites.len;
}

fn VoronoiCellView VoronoiGraph.cell(&self, FaceIndex c)
{
    int start = self.cell_offsets[(int)c];
    int end   = self.cell_offsets[(int)c + 1];
    int count = end - start;
    return {
        .index     = c,
        .site      = self.sites[(int)c],
        .centroid  = self.centroids[(int)c],
        .ring      = self.ring_indices[start : count],
        .neighbors = self.neighbor_indices[start : count],
    };
}

fn VoronoiGraph? from_voronoi(Allocator alloc, VoronoiDiagram* diagram)
{
    if (diagram.mesh.faces.len == 0) return cg::EMPTY_INPUT~;
    if (diagram.sites.len == 0) return cg::EMPTY_INPUT~;
    if (diagram.sites.len != diagram.mesh.faces.len) return cg::DUAL_VERTEX_COUNT_MISMATCH~;
    diagram.mesh.validate()!;

    Vec3f[] sites = copy_sites(alloc, diagram)!;
    Vec3f[] centroids = geometry::face_centroids(alloc, &diagram.mesh)!;
    Vec3f[] vertices = copy_positions(alloc, &diagram.mesh)!;

    usz cell_count = diagram.mesh.faces.len;

    usz total_ring_entries = 0;
    for (usz f = 0; f < cell_count; f++) {
        int deg = diagram.mesh.face_degree((FaceIndex)(int)f);
        total_ring_entries += (usz)deg;
    }

    int[] cell_offsets = mem::alloc::new_array(alloc, int, (sz)(cell_count + 1));
    VertexIndex[] ring_indices = mem::alloc::new_array(alloc, VertexIndex, (sz)total_ring_entries);
    FaceIndex[] neighbor_indices = mem::alloc::new_array(alloc, FaceIndex, (sz)total_ring_entries);

    usz max_deg = 0;
    for (usz f = 0; f < cell_count; f++) {
        int deg = diagram.mesh.face_degree((FaceIndex)(int)f);
        if ((usz)deg > max_deg) max_deg = (usz)deg;
    }

    HeIndex[] he_scratch = mem::alloc::new_array(alloc, HeIndex, (sz)max_deg);
    defer free(he_scratch);

    cell_offsets[0] = 0;
    for (usz f = 0; f < cell_count; f++) {
        FaceIndex face = (FaceIndex)(int)f;
        int deg = diagram.mesh.face_degree(face);
        diagram.mesh.face_half_edges(face, he_scratch)!;
        usz offset = (usz)cell_offsets[f];

        for (int i = 0; i < deg; i++) {
            HeIndex he = he_scratch[i];
            ring_indices[offset + (usz)i] = diagram.mesh.from_vertex(he);

            if (diagram.mesh.is_boundary(he)) {
                neighbor_indices[offset + (usz)i] = cg::INVALID_FACE;
            } else {
                HeIndex twin = diagram.mesh.twin(he);
                neighbor_indices[offset + (usz)i] = diagram.mesh.face_of(twin);
            }
        }

        cell_offsets[f + 1] = cell_offsets[f] + deg;
    }

    return {
        .vertices = vertices,
        .sites = sites,
        .centroids = centroids,
        .cell_offsets = cell_offsets,
        .ring_indices = ring_indices,
        .neighbor_indices = neighbor_indices,
    };
}

// ── internal helpers ──

fn Vec3f[]? copy_sites(Allocator alloc, VoronoiDiagram* diagram)
{
    Vec3f[] out = mem::alloc::new_array(alloc, Vec3f, (sz)diagram.sites.len);
    for (usz i = 0; i < diagram.sites.len; i++) out[i] = diagram.sites[i];
    return out;
}

fn Vec3f[]? copy_positions(Allocator alloc, HalfEdgeMesh* mesh)
{
    Vec3f[] out = mem::alloc::new_array(alloc, Vec3f, (sz)mesh.positions.len);
    for (usz i = 0; i < mesh.positions.len; i++) out[i] = mesh.positions[i];
    return out;
}
