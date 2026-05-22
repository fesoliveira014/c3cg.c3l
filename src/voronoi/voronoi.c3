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

    VoronoiDiagram diagram;
    diagram.mesh = mesh;
    diagram.sites = sites;
    return diagram;
}

fn HalfEdgeMesh? to_delaunay(Allocator alloc, VoronoiDiagram* diagram)
{
    return dual::dual_from_vertices(alloc, &diagram.mesh, diagram.sites)!;
}
