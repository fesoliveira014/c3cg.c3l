module bench::all;
import cg;
import cg::delaunay;
import cg::voronoi;
import cg::hull;
import cg::triangulate;
import cg::subdivide;
import cg::dual;
import cg::geometry;
import cg::graph;
import cg::primitives;
import cg::half_edge;
import std::math;
import std::time::clock;
import std::io;

const int[] SIZES        = { 100, 500, 1000, 2000, 5000, 10000 };
const int[] SMALL_SIZES  = { 100, 200, 500 };
const int[] HULL_3D_SIZES = { 100, 500, 1000, 2000 };
const int[] POLYGON_SIZES = { 10, 50, 100, 200 };
const int[] SUBDIV_SIZES  = { 0, 1, 2, 3 };
const int[] SPHERE_SIZES  = { 100, 500, 1000, 2000 };

fn Vec3f[] generate_2d_points(int n)
{
    Vec3f[] pts = mem::alloc::new_array(mem, Vec3f, (sz)n);
    for (int i = 0; i < n; i++) {
        pts[i] = {
            (float)((int)((uint)i * 2654435761u % 1000000u)) * 0.001f,
            (float)((int)((uint)i * 1737350767u % 1000000u)) * 0.001f,
            0,
        };
    }
    return pts;
}

fn Vec3f[] generate_3d_points(int n)
{
    Vec3f[] pts = mem::alloc::new_array(mem, Vec3f, (sz)n);
    for (int i = 0; i < n; i++) {
        pts[i] = {
            (float)((int)((uint)i * 2654435761u % 1000000u)) * 0.001f,
            (float)((int)((uint)i * 1737350767u % 1000000u)) * 0.001f,
            (float)((int)((uint)i * 936911057u % 1000000u)) * 0.001f,
        };
    }
    return pts;
}

fn Vec3f[] generate_sphere_points(int n)
{
    Vec3f[] pts = mem::alloc::new_array(mem, Vec3f, (sz)n);
    for (int i = 0; i < n; i++) {
        float theta = 2.0f * 3.14159265f * (float)i / (float)n;
        float phi   = (float)math::acos(2.0 * (float)((uint)i * 1737350767u % 1000000000u) * 1e-9 - 1.0);
        pts[i] = {
            (float)math::sin((double)phi) * (float)math::cos((double)theta),
            (float)math::sin((double)phi) * (float)math::sin((double)theta),
            (float)math::cos((double)phi),
        };
    }
    return pts;
}

fn Vec3f[] generate_ccw_polygon(int n)
{
    Vec3f[] pts = mem::alloc::new_array(mem, Vec3f, (sz)n);
    for (int i = 0; i < n; i++) {
        float angle = 2.0f * 3.14159265f * (float)i / (float)n;
        float r = 1.0f + (float)(i % 3) * 0.2f;
        pts[i] = { r * (float)math::cos((double)angle), r * (float)math::sin((double)angle), 0 };
    }
    return pts;
}

fn void bench_delaunay_2d()
{
    io::printn("=== delaunay_2d ===");
    for (int si = 0; si < SIZES.len; si++) {
        int n = SIZES[si];
        Vec3f[] pts = generate_2d_points(n);
        defer free(pts);

        Clock t0 = clock::now();
        HalfEdgeMesh? mesh = delaunay::delaunay_2d(mem, pts[..]);
        if (catch err = mesh) {
            io::printfn("  n=%d ERROR: delaunay_2d failed", n);
            return;
        }
        NanoDuration elapsed = t0.to_now();
        defer mesh.destroy();

        io::printfn("  n=%d time=%d ns faces=%d halvedges=%d",
            n, (int)(long)elapsed, (int)mesh.faces.len, (int)mesh.half_edges.len);
    }
}

fn void bench_hull_2d()
{
    io::printn("=== hull_2d ===");
    for (int si = 0; si < SIZES.len; si++) {
        int n = SIZES[si];
        Vec3f[] pts = generate_2d_points(n);
        defer free(pts);

        Clock t0 = clock::now();
        int[]? hull_indices = hull::hull_2d(mem, pts[..]);
        if (catch err = hull_indices) {
            io::printfn("  n=%d ERROR: hull_2d failed", n);
            return;
        }
        NanoDuration elapsed = t0.to_now();
        defer free(hull_indices);

        io::printfn("  n=%d time=%d ns indices=%d",
            n, (int)(long)elapsed, (int)hull_indices.len);
    }
}

fn void bench_hull_3d()
{
    io::printn("=== hull_3d ===");
    for (int si = 0; si < HULL_3D_SIZES.len; si++) {
        int n = HULL_3D_SIZES[si];
        Vec3f[] pts = generate_3d_points(n);
        defer free(pts);

        Clock t0 = clock::now();
        HalfEdgeMesh? mesh = hull::hull_3d(mem, pts[..]);
        if (catch err = mesh) {
            io::printfn("  n=%d ERROR: hull_3d failed", n);
            return;
        }
        NanoDuration elapsed = t0.to_now();
        defer mesh.destroy();

        io::printfn("  n=%d time=%d ns faces=%d",
            n, (int)(long)elapsed, (int)mesh.faces.len);
    }
}

fn void bench_triangulate_polygon()
{
    io::printn("=== triangulate_polygon ===");
    for (int si = 0; si < POLYGON_SIZES.len; si++) {
        int n = POLYGON_SIZES[si];
        Vec3f[] poly = generate_ccw_polygon(n);
        defer free(poly);

        Clock t0 = clock::now();
        HalfEdgeMesh? mesh = triangulate::triangulate_polygon(mem, poly[..]);
        if (catch err = mesh) {
            io::printfn("  n=%d ERROR: triangulate_polygon failed", n);
            return;
        }
        NanoDuration elapsed = t0.to_now();
        defer mesh.destroy();

        io::printfn("  n=%d time=%d ns faces=%d",
            n, (int)(long)elapsed, (int)mesh.faces.len);
    }
}

fn void bench_voronoi_from_delaunay()
{
    io::printn("=== voronoi from_delaunay + bounded ===");
    for (int si = 0; si < SMALL_SIZES.len; si++) {
        int n = SMALL_SIZES[si];
        Vec3f[] pts = generate_2d_points(n);
        defer free(pts);

        float min_x = pts[0].x;
        float max_x = pts[0].x;
        float min_y = pts[0].y;
        float max_y = pts[0].y;
        for (usz i = 1; i < pts.len; i++) {
            if (pts[i].x < min_x) min_x = pts[i].x;
            if (pts[i].x > max_x) max_x = pts[i].x;
            if (pts[i].y < min_y) min_y = pts[i].y;
            if (pts[i].y > max_y) max_y = pts[i].y;
        }
        float pad = 0.1f * ((max_x - min_x) > (max_y - min_y) ? (max_x - min_x) : (max_y - min_y));
        Vec3f[4] bbox = {
            { min_x - pad, min_y - pad, 0 },
            { max_x + pad, min_y - pad, 0 },
            { max_x + pad, max_y + pad, 0 },
            { min_x - pad, max_y + pad, 0 },
        };

        Clock t0 = clock::now();
        VoronoiDiagram? diagram = voronoi::in_polygon(mem, pts[..], bbox[..]);
        if (catch err = diagram) {
            io::printfn("  n=%d ERROR: voronoi in_polygon failed", n);
            return;
        }
        NanoDuration elapsed = t0.to_now();
        defer diagram.destroy();

        io::printfn("  n=%d time=%d ns cells=%d",
            n, (int)(long)elapsed, (int)diagram.mesh.faces.len);
    }
}

fn void bench_spherical()
{
    io::printn("=== spherical delaunay + voronoi ===");
    for (int si = 0; si < SPHERE_SIZES.len; si++) {
        int n = SPHERE_SIZES[si];
        Vec3f[] pts = generate_sphere_points(n);
        defer free(pts);

        Clock t0 = clock::now();
        VoronoiDiagram? diagram = voronoi::voronoi_on_sphere(mem, pts[..], 1.0f);
        if (catch err = diagram) {
            io::printfn("  n=%d ERROR: voronoi_on_sphere failed", n);
            return;
        }
        NanoDuration elapsed = t0.to_now();
        defer diagram.destroy();

        io::printfn("  n=%d time=%d ns faces=%d",
            n, (int)(long)elapsed, (int)diagram.mesh.faces.len);
    }
}

fn void bench_dual()
{
    io::printn("=== dual_from_vertices ===");
    for (int si = 0; si < SMALL_SIZES.len; si++) {
        int n = SMALL_SIZES[si];
        Vec3f[] pts = generate_2d_points(n);
        defer free(pts);

        HalfEdgeMesh? delaunay_mesh = delaunay::delaunay_2d(mem, pts[..]);
        if (catch err = delaunay_mesh) {
            io::printfn("  n=%d ERROR: delaunay_2d failed for dual bench", n);
            return;
        }
        defer delaunay_mesh.destroy();
        Vec3f[]? centers = geometry::circumcenters_planar(mem, &delaunay_mesh);
        if (catch err = centers) {
            io::printfn("  n=%d ERROR: circumcenters_planar failed for dual bench", n);
            return;
        }
        defer free(centers);

        Clock t0 = clock::now();
        HalfEdgeMesh? dual_mesh = dual::dual_from_vertices(mem, &delaunay_mesh, centers);
        if (catch err = dual_mesh) {
            io::printfn("  n=%d ERROR: dual_from_vertices failed", n);
            return;
        }
        NanoDuration elapsed = t0.to_now();
        defer dual_mesh.destroy();

        io::printfn("  n=%d time=%d ns faces=%d",
            n, (int)(long)elapsed, (int)dual_mesh.faces.len);
    }
}

fn void bench_loop_subdivide()
{
    io::printn("=== loop_subdivide ===");
    for (int si = 0; si < SUBDIV_SIZES.len; si++) {
        int subdivs = SUBDIV_SIZES[si];
        HalfEdgeMesh? mesh = primitives::icosahedron(mem);
        if (catch err = mesh) {
            io::printfn("  subdivs=%d ERROR: icosahedron failed", subdivs);
            return;
        }
        defer mesh.destroy();

        Clock t0 = clock::now();
        HalfEdgeMesh? result = subdivide::loop_subdivide(mem, &mesh);
        if (catch err = result) {
            io::printfn("  subdivs=%d ERROR: loop_subdivide failed", subdivs);
            return;
        }
        NanoDuration elapsed = t0.to_now();
        defer result.destroy();

        io::printfn("  subdivs=%d time=%d ns faces=%d",
            subdivs, (int)(long)elapsed, (int)result.faces.len > 0 ? (int)result.faces.len : 0);
    }
}

fn void bench_graph_views()
{
    io::printn("=== graph views ===");
    for (int si = 0; si < SMALL_SIZES.len; si++) {
        int n = SMALL_SIZES[si];
        Vec3f[] pts = generate_2d_points(n);
        defer free(pts);

        HalfEdgeMesh? delaunay_mesh = delaunay::delaunay_2d(mem, pts[..]);
        if (catch err = delaunay_mesh) {
            io::printfn("  n=%d ERROR: delaunay_2d failed for graph views", n);
            return;
        }
        defer delaunay_mesh.destroy();

        Clock t0 = clock::now();
        DelaunayGraph? dg = graph::from_planar_delaunay(mem, &delaunay_mesh);
        if (catch err = dg) {
            io::printfn("  n=%d ERROR: from_planar_delaunay failed", n);
            return;
        }
        NanoDuration elapsed = t0.to_now();
        defer dg.destroy();

        io::printfn("  delaunay_graph n=%d time=%d ns triangles=%d",
            n, (int)(long)elapsed, dg.triangle_count());
    }
}

fn void bench_primitives()
{
    io::printn("=== primitives ===");
    Clock t0 = clock::now();
    HalfEdgeMesh? tet = primitives::tetrahedron(mem);
    if (catch err = tet) {
        io::printfn("  tetrahedron  ERROR: failed");
        return;
    }
    NanoDuration e1 = t0.to_now();
    defer tet.destroy();
    io::printfn("  tetrahedron  time=%d ns", (int)(long)e1);

    t0 = clock::now();
    HalfEdgeMesh? ico = primitives::icosahedron(mem);
    if (catch err = ico) {
        io::printfn("  icosahedron  ERROR: failed");
        return;
    }
    NanoDuration e2 = t0.to_now();
    defer ico.destroy();
    io::printfn("  icosahedron  time=%d ns", (int)(long)e2);

    t0 = clock::now();
    HalfEdgeMesh? cube = primitives::triangulated_cube(mem);
    if (catch err = cube) {
        io::printfn("  cube         ERROR: failed");
        return;
    }
    NanoDuration e3 = t0.to_now();
    defer cube.destroy();
    io::printfn("  cube         time=%d ns", (int)(long)e3);

    t0 = clock::now();
    HalfEdgeMesh? sphere = primitives::icosphere(mem, 3, 1.0f);
    if (catch err = sphere) {
        io::printfn("  icosphere(3) ERROR: failed");
        return;
    }
    NanoDuration e4 = t0.to_now();
    defer sphere.destroy();
    io::printfn("  icosphere(3) time=%d ns faces=%d", (int)(long)e4, (int)sphere.faces.len);
}

fn void main()
{
    bench_delaunay_2d();
    bench_hull_2d();
    bench_hull_3d();
    bench_triangulate_polygon();
    bench_voronoi_from_delaunay();
    bench_spherical();
    bench_dual();
    bench_loop_subdivide();
    bench_graph_views();
    bench_primitives();
}
