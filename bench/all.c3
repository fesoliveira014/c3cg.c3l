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

const float BENCH_PI           = 3.14159265358979323846f;
const float BENCH_TWO_PI       = 6.28318530717958647692f;
const float BENCH_GOLDEN_ANGLE = 2.39996322972865332223f;
const float BENCH_PLANAR_RANGE = 1.0f;
const float BENCH_BOUND        = 10.0f;

const int[] PLANAR_SIZES    = { 100, 500, 1000, 2000, 5000 };
const int[] GRAPH_SIZES     = { 100, 500, 1000 };
const int[] BOUNDED_SIZES   = { 25, 50, 100, 200 };
const int[] HULL_3D_SIZES   = { 100, 500, 1000, 2000 };
const int[] POLYGON_SIZES   = { 10, 50, 100, 200 };
const int[] SUBDIV_LEVELS   = { 0, 1, 2, 3, 4, 5, 6 };
const int[] SPHERE_SIZES    = { 100, 500, 1000 };

fn uint bench_hash(uint x)
{
    x ^= x >> 16;
    x *= 0x7feb352du;
    x ^= x >> 15;
    x *= 0x846ca68bu;
    x ^= x >> 16;
    return x;
}

fn float hash_unit_float(uint seed)
{
    uint bits = bench_hash(seed) & 0x00ffffffu;
    return (float)bits * (1.0f / 16777215.0f);
}

fn Vec3f[] generate_sphere_points(int n, float radius)
{
    Vec3f[] pts = mem::alloc::new_array(mem, Vec3f, (sz)n);
    for (int i = 0; i < n; i++) {
        float z = 1.0f - (2.0f * ((float)i + 0.5f) / (float)n);
        float ring_radius = (float)math::sqrt((double)(1.0f - z * z));
        float theta = BENCH_GOLDEN_ANGLE * (float)i;
        float x = ring_radius * (float)math::cos((double)theta);
        float y = ring_radius * (float)math::sin((double)theta);
        float len = (float)math::sqrt((double)(x * x + y * y + z * z));
        float scale = radius / len;
        pts[i] = { x * scale, y * scale, z * scale };
    }
    return pts;
}

fn Vec3f[] generate_2d_points_uniform(int n, float range)
{
    Vec3f[] pts = mem::alloc::new_array(mem, Vec3f, (sz)n);
    for (int i = 0; i < n; i++) {
        float x = hash_unit_float((uint)i + 0x9e3779b9u);
        float y = hash_unit_float((uint)i + 0x243f6a88u);
        pts[i] = {
            (2.0f * x - 1.0f) * range,
            (2.0f * y - 1.0f) * range,
            0,
        };
    }
    return pts;
}

fn Vec3f[] generate_convex_polygon(int n, float radius)
{
    Vec3f[] pts = mem::alloc::new_array(mem, Vec3f, (sz)n);
    for (int i = 0; i < n; i++) {
        float angle = BENCH_TWO_PI * (float)i / (float)n;
        pts[i] = {
            radius * (float)math::cos((double)angle),
            radius * (float)math::sin((double)angle),
            0,
        };
    }
    return pts;
}

fn void bench_delaunay_2d()
{
    io::printn("=== delaunay_2d ===");
    for (int si = 0; si < PLANAR_SIZES.len; si++) {
        int n = PLANAR_SIZES[si];
        Vec3f[] pts = generate_2d_points_uniform(n, BENCH_PLANAR_RANGE);
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
    for (int si = 0; si < PLANAR_SIZES.len; si++) {
        int n = PLANAR_SIZES[si];
        Vec3f[] pts = generate_2d_points_uniform(n, BENCH_PLANAR_RANGE);
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
        Vec3f[] pts = generate_sphere_points(n, 1.0f);
        defer free(pts);

        Clock t0 = clock::now();
        HalfEdgeMesh? mesh = hull::hull_3d(mem, pts[..]);
        if (catch err = mesh) {
            io::printfn("  n=%d ERROR: hull_3d failed", n);
            return;
        }
        NanoDuration elapsed = t0.to_now();
        defer mesh.destroy();

        io::printfn("  n=%d time=%d ns faces=%d vertices=%d",
            n, (int)(long)elapsed, (int)mesh.faces.len, (int)mesh.vertices.len);
    }
}

fn void bench_triangulate_polygon()
{
    io::printn("=== triangulate_polygon ===");
    for (int si = 0; si < POLYGON_SIZES.len; si++) {
        int n = POLYGON_SIZES[si];
        Vec3f[] poly = generate_convex_polygon(n, 1.0f);
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

fn void bench_voronoi_bounded()
{
    io::printn("=== voronoi bounded ===");
    Vec3f[4] bounds = {
        { -BENCH_BOUND, -BENCH_BOUND, 0 },
        {  BENCH_BOUND, -BENCH_BOUND, 0 },
        {  BENCH_BOUND,  BENCH_BOUND, 0 },
        { -BENCH_BOUND,  BENCH_BOUND, 0 },
    };

    for (int si = 0; si < BOUNDED_SIZES.len; si++) {
        int n = BOUNDED_SIZES[si];
        Vec3f[] pts = generate_2d_points_uniform(n, BENCH_PLANAR_RANGE);
        defer free(pts);

        Clock t0 = clock::now();
        VoronoiDiagram? diagram = voronoi::in_polygon(mem, pts[..], bounds[..]);
        if (catch err = diagram) {
            io::printfn("  n=%d ERROR: voronoi in_polygon failed", n);
            return;
        }
        NanoDuration elapsed = t0.to_now();
        defer diagram.destroy();

        io::printfn("  n=%d time=%d ns cells=%d vertices=%d",
            n, (int)(long)elapsed, (int)diagram.mesh.faces.len, (int)diagram.mesh.vertices.len);
    }
}

fn void bench_spherical()
{
    io::printn("=== spherical delaunay + voronoi ===");
    for (int si = 0; si < SPHERE_SIZES.len; si++) {
        int n = SPHERE_SIZES[si];
        Vec3f[] pts = generate_sphere_points(n, 1.0f);
        defer free(pts);

        Clock t0 = clock::now();
        HalfEdgeMesh? tri_mesh = delaunay::delaunay_on_sphere(mem, pts[..], 1.0f);
        if (catch err = tri_mesh) {
            io::printfn("  n=%d ERROR: delaunay_on_sphere failed", n);
            return;
        }
        NanoDuration d_elapsed = t0.to_now();
        defer tri_mesh.destroy();

        t0 = clock::now();
        VoronoiDiagram? diagram = voronoi::voronoi_on_sphere(mem, pts[..], 1.0f);
        if (catch err = diagram) {
            io::printfn("  n=%d ERROR: voronoi_on_sphere failed", n);
            return;
        }
        NanoDuration v_elapsed = t0.to_now();
        defer diagram.destroy();

        io::printfn("  n=%d delaunay_time=%d ns voronoi_time=%d ns triangles=%d cells=%d",
            n, (int)(long)d_elapsed, (int)(long)v_elapsed, (int)tri_mesh.faces.len, (int)diagram.mesh.faces.len);
    }
}

fn void bench_dual()
{
    io::printn("=== dual_from_vertices ===");
    for (int si = 0; si < SUBDIV_LEVELS.len; si++) {
        int level = SUBDIV_LEVELS[si];
        HalfEdgeMesh? mesh = primitives::icosphere(mem, level, 1.0f);
        if (catch err = mesh) {
            io::printfn("  level=%d ERROR: icosphere failed for dual bench", level);
            return;
        }
        defer mesh.destroy();

        Vec3f[]? centroids = geometry::face_centroids(mem, &mesh);
        if (catch err = centroids) {
            io::printfn("  level=%d ERROR: face_centroids failed for dual bench", level);
            return;
        }
        defer free(centroids);

        Clock t0 = clock::now();
        HalfEdgeMesh? dual_mesh = dual::dual_from_vertices(mem, &mesh, centroids);
        if (catch err = dual_mesh) {
            io::printfn("  level=%d ERROR: dual_from_vertices failed", level);
            return;
        }
        NanoDuration elapsed = t0.to_now();
        defer dual_mesh.destroy();

        io::printfn("  level=%d time=%d ns input_faces=%d dual_faces=%d",
            level, (int)(long)elapsed, (int)mesh.faces.len, (int)dual_mesh.faces.len);
    }
}

fn void bench_loop_subdivide()
{
    io::printn("=== loop_subdivide ===");
    for (int si = 0; si < SUBDIV_LEVELS.len; si++) {
        int level = SUBDIV_LEVELS[si];
        HalfEdgeMesh? mesh = primitives::icosphere(mem, level, 1.0f);
        if (catch err = mesh) {
            io::printfn("  input_level=%d ERROR: icosphere failed", level);
            return;
        }
        defer mesh.destroy();

        Clock t0 = clock::now();
        HalfEdgeMesh? result = subdivide::loop_subdivide(mem, &mesh);
        if (catch err = result) {
            io::printfn("  input_level=%d ERROR: loop_subdivide failed", level);
            return;
        }
        NanoDuration elapsed = t0.to_now();
        defer result.destroy();

        io::printfn("  input_level=%d time=%d ns input_faces=%d output_faces=%d",
            level, (int)(long)elapsed, (int)mesh.faces.len, (int)result.faces.len);
    }
}

fn void bench_graph_views()
{
    io::printn("=== graph views ===");
    for (int si = 0; si < GRAPH_SIZES.len; si++) {
        int n = GRAPH_SIZES[si];
        Vec3f[] pts = generate_2d_points_uniform(n, BENCH_PLANAR_RANGE);
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
        NanoDuration d_elapsed = t0.to_now();
        defer dg.destroy();

        t0 = clock::now();
        VoronoiDiagram? diagram = voronoi::in_polygon(mem, pts[..], {
            { -BENCH_BOUND, -BENCH_BOUND, 0 },
            {  BENCH_BOUND, -BENCH_BOUND, 0 },
            {  BENCH_BOUND,  BENCH_BOUND, 0 },
            { -BENCH_BOUND,  BENCH_BOUND, 0 },
        });
        if (catch err = diagram) {
            io::printfn("  n=%d ERROR: voronoi in_polygon failed for graph views", n);
            return;
        }
        VoronoiGraph? vg = graph::from_voronoi(mem, &diagram);
        if (catch err = vg) {
            io::printfn("  n=%d ERROR: from_voronoi failed", n);
            return;
        }
        NanoDuration v_elapsed = t0.to_now();
        defer vg.destroy();
        defer diagram.destroy();

        io::printfn("  n=%d delaunay_graph_time=%d ns triangles=%d voronoi_graph_time=%d ns cells=%d",
            n, (int)(long)d_elapsed, dg.triangle_count(), (int)(long)v_elapsed, vg.cell_count());
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
    io::printfn("  tetrahedron  time=%d ns faces=%d", (int)(long)e1, (int)tet.faces.len);

    t0 = clock::now();
    HalfEdgeMesh? oct = primitives::octahedron(mem);
    if (catch err = oct) {
        io::printfn("  octahedron   ERROR: failed");
        return;
    }
    NanoDuration e2 = t0.to_now();
    defer oct.destroy();
    io::printfn("  octahedron   time=%d ns faces=%d", (int)(long)e2, (int)oct.faces.len);

    t0 = clock::now();
    HalfEdgeMesh? ico = primitives::icosahedron(mem);
    if (catch err = ico) {
        io::printfn("  icosahedron  ERROR: failed");
        return;
    }
    NanoDuration e3 = t0.to_now();
    defer ico.destroy();
    io::printfn("  icosahedron  time=%d ns faces=%d", (int)(long)e3, (int)ico.faces.len);

    t0 = clock::now();
    HalfEdgeMesh? cube = primitives::triangulated_cube(mem);
    if (catch err = cube) {
        io::printfn("  cube         ERROR: failed");
        return;
    }
    NanoDuration e4 = t0.to_now();
    defer cube.destroy();
    io::printfn("  cube         time=%d ns faces=%d", (int)(long)e4, (int)cube.faces.len);

    t0 = clock::now();
    HalfEdgeMesh? sphere = primitives::icosphere(mem, 3, 1.0f);
    if (catch err = sphere) {
        io::printfn("  icosphere(3) ERROR: failed");
        return;
    }
    NanoDuration e5 = t0.to_now();
    defer sphere.destroy();
    io::printfn("  icosphere(3) time=%d ns faces=%d", (int)(long)e5, (int)sphere.faces.len);
}

fn void main()
{
    bench_delaunay_2d();
    bench_hull_2d();
    bench_hull_3d();
    bench_triangulate_polygon();
    bench_voronoi_bounded();
    bench_spherical();
    bench_dual();
    bench_loop_subdivide();
    bench_graph_views();
    bench_primitives();
}
