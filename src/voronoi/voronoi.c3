module cg::voronoi;
import cg;
import cg::geometry;
import cg::dual;

fn void VoronoiDiagram.destroy(&self)
{
    self.mesh.destroy();
    free(self.sites);
    *self = {};
}

fn VoronoiDiagram? from_delaunay(Allocator alloc, HalfEdgeMesh* delaunay)
{
    delaunay.validate()!;

    for (usz i = 0; i < delaunay.half_edges.len; i++) {
        if (delaunay.half_edges[i].twin == cg::INVALID_HE) {
            return cg::OPEN_CELL_ON_BOUNDARY~;
        }
    }

    Vec3f[] centers = geometry::circumcenters_planar(alloc, delaunay)!;
    defer free(centers);

    HalfEdgeMesh mesh = dual::dual_from_vertices(alloc, delaunay, centers)!;

    Vec3f[] sites = mem::alloc::new_array(alloc, Vec3f, (sz) delaunay.positions.len);
    defer catch {
        mesh.destroy();
        free(sites);
    };
    for (usz i = 0; i < delaunay.positions.len; i++) {
        sites[i] = delaunay.positions[i];
    }

    return {
        .mesh = mesh,
        .sites = sites,
    };
}

fn HalfEdgeMesh? to_delaunay(Allocator alloc, VoronoiDiagram* diagram)
{
    return dual::dual_from_vertices(alloc, &diagram.mesh, diagram.sites)!;
}
