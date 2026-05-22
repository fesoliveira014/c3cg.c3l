module tools;
import cg;
import cg::delaunay;
import cg::utils;
import cg::utils::ppm;

fn void main()
{
    usz n = 100;

    // Generate pseudo-random points
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

    // Compute Delaunay
    HalfEdgeMesh mesh = delaunay::delaunay_2d(mem, points[..])!!;
    defer mesh.destroy();

    // Render
    PpmOptions opts;
    opts.width = 800;
    opts.height = 0;
    opts.edge_color = 0xFF000000;
    opts.site_color = 0xFFFF0000;
    opts.bg_color = 0xFFFFFFFF;
    opts.line_width = 1;
    opts.dot_radius = 2;
    opts.margin = 0.05f;

    PpmImage img = utils::render_delaunay(mem, &mesh, opts)!!;
    defer img.destroy();

    ppm::write_ppm(&img, "random_delaunay.ppm");
}
