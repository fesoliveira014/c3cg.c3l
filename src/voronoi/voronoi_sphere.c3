module cg::voronoi;
import cg;
import cg::delaunay;
import cg::geometry;
import cg::dual;

fn VoronoiDiagram? voronoi_on_sphere(Allocator alloc, Vec3f[] points, float radius)
{
    if (points.len == 0) return cg::EMPTY_INPUT~;
    if (points.len < 4) return cg::DEGENERATE_INPUT~;
    if (radius <= 0) return cg::DEGENERATE_INPUT~;

    HalfEdgeMesh tri_mesh = delaunay::delaunay_on_sphere(alloc, points, radius)!;
    defer tri_mesh.destroy();

    Vec3f[] centers = geometry::circumcenters_on_sphere(alloc, &tri_mesh, radius)!;
    defer free(centers);

    HalfEdgeMesh voronoi_mesh = dual::dual_from_vertices(alloc, &tri_mesh, centers)!;

    Vec3f[] sites = mem::alloc::new_array(alloc, Vec3f, (sz)tri_mesh.positions.len);
    defer catch {
        voronoi_mesh.destroy();
        free(sites);
    };
    for (usz i = 0; i < tri_mesh.positions.len; i++) sites[i] = tri_mesh.positions[i];

    return {
        .mesh = voronoi_mesh,
        .sites = sites,
    };
}

fn VoronoiDiagram? voronoi_dual_sphere(Allocator alloc, HalfEdgeMesh* tri_mesh, Vec3f[] spatial_points)
{
    HalfEdgeMesh voronoi_mesh = dual::dual_from_vertices(alloc, tri_mesh, spatial_points)!;

    Vec3f[] sites = mem::alloc::new_array(alloc, Vec3f, (sz)spatial_points.len);
    defer catch {
        voronoi_mesh.destroy();
        free(sites);
    };
    for (usz i = 0; i < spatial_points.len; i++) sites[i] = spatial_points[i];

    return {
        .mesh = voronoi_mesh,
        .sites = sites,
    };
}
