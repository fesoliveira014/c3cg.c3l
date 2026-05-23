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
    return {
        .index     = c,
        .site      = self.sites[(int)c],
        .centroid  = self.centroids[(int)c],
        .ring      = self.ring_indices[start..end],
        .neighbors = self.neighbor_indices[start..end],
    };
}

fn VoronoiGraph? from_voronoi(Allocator alloc, VoronoiDiagram* diagram)
{
    if (diagram.mesh.faces.len == 0) return cg::EMPTY_INPUT~;
    VoronoiGraph g;
    g.sites = copy_sites(alloc, diagram)!;
    g.centroids = geometry::face_centroids(alloc, &diagram.mesh)!;
    g.vertices = copy_positions(alloc, &diagram.mesh)!;
    g.cell_offsets = alloc_offsets(alloc, (usz)g.sites.len)!;
    g.ring_indices = {};
    g.neighbor_indices = {};
    return g;
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

fn int[]? alloc_offsets(Allocator alloc, usz cell_count)
{
    int[] out = mem::alloc::new_array(alloc, int, (sz)(cell_count + 1));
    for (usz i = 0; i <= cell_count; i++) out[i] = 0;
    return out;
}
