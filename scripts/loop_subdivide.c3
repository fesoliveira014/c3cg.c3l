module tools;
import cg;
import cg::primitives;
import cg::subdivide;
import cg::utils;
import cg::utils::ppm;

fn void main()
{
    // Build icosahedron
    HalfEdgeMesh mesh = primitives::icosahedron(mem)!!;
    defer mesh.destroy();

    // Apply one Loop subdivision
    HalfEdgeMesh refined = subdivide::loop_subdivide(mem, &mesh)!!;
    // Free original, keep refined
    mesh.destroy();

    // Render
    PpmOptions opts;
    opts.width = 800;
    opts.height = 0;
    opts.edge_color = 0xFF000000;
    opts.site_color = 0xFFFF0000;
    opts.bg_color = 0xFFFFFFFF;
    opts.line_width = 1;
    opts.dot_radius = 2;
    opts.margin = 0.1f;

    PpmImage img = utils::render_delaunay(mem, &refined, opts)!!;
    defer img.destroy();
    refined.destroy();

    ppm::write_ppm(&img, "loop_subdivide.ppm");
}
