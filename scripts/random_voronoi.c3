module tools;
import cg;
import cg::delaunay;
import cg::voronoi;
import cg::utils;
import cg::utils::ppm;

fn void main()
{
    usz n = 20;

    // Generate pseudo-random points in [0,1]²
    Vec3f[] points = mem::alloc::new_array(mem, Vec3f, (sz)n);
    defer free(points);
    {
        uint state = 42;
        for (usz i = 0; i < n; i++) {
            state = state * 1103515245 + 12345;
            float x = (float)(state % 10000) / 10000.0f;
            state = state * 1103515245 + 12345;
            float y = (float)(state % 10000) / 10000.0f;
            points[i] = { x, y, 0 };
        }
    }

    // Bounding polygon: unit square
    Vec3f[4] polygon = { {0,0,0}, {1,0,0}, {1,1,0}, {0,1,0} };

    // Compute bounded Voronoi
    VoronoiDiagram diagram = voronoi::in_polygon(mem, points[..], polygon[..])!!;
    defer diagram.destroy();

    // Render filled cells
    PpmOptions opts;
    opts.width = 800;
    opts.height = 0;
    opts.edge_color = 0xFF000000;
    opts.site_color = 0xFFFF0000;
    opts.bg_color = 0xFFFFFFFF;
    opts.line_width = 1;
    opts.dot_radius = 3;
    opts.margin = 0.05f;

    PpmImage img = utils::render_voronoi(mem, &diagram, opts)!!;
    defer img.destroy();

    ppm::write_ppm(&img, "random_voronoi.ppm");

    // Also render overlay
    HalfEdgeMesh delaunay_mesh = delaunay::delaunay_2d(mem, points[..])!!;
    defer delaunay_mesh.destroy();

    OverlayOptions ov_opts;
    ov_opts.common = opts;
    ov_opts.delaunay_edge_color = 0xFF000000;
    ov_opts.voronoi_edge_color = 0xFF0000FF;
    ov_opts.site_color = 0xFFFF0000;

    PpmImage ov_img = utils::render_overlay(mem, &delaunay_mesh, &diagram, ov_opts)!!;
    defer ov_img.destroy();

    ppm::write_ppm(&ov_img, "random_overlay.ppm");
}
