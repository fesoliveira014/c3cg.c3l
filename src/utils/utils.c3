module cg::utils;
import cg;
import cg::utils::ppm;

fn PpmImage? render_delaunay(Allocator alloc, HalfEdgeMesh* mesh, PpmOptions options)
{
    if (mesh.positions.len == 0) return cg::EMPTY_INPUT~;

    // Compute bounding box from positions (x,y only)
    Vec3f bbox_min = mesh.positions[0];
    Vec3f bbox_max = mesh.positions[0];
    for (usz i = 1; i < mesh.positions.len; i++) {
        Vec3f p = mesh.positions[i];
        if (p.x < bbox_min.x) bbox_min.x = p.x;
        if (p.y < bbox_min.y) bbox_min.y = p.y;
        if (p.x > bbox_max.x) bbox_max.x = p.x;
        if (p.y > bbox_max.y) bbox_max.y = p.y;
    }

    // Apply defaults: line_width >= 1, dot_radius >= 1
    uint line_width = options.line_width;
    if (line_width < 1) line_width = 1;
    uint dot_radius = options.dot_radius;
    if (dot_radius < 1) dot_radius = 1;
    float margin = options.margin;
    if (margin < 0.0f) margin = 0.05f;

    // Compute bbox extent; guard against degenerate input
    float bbox_w = bbox_max.x - bbox_min.x;
    float bbox_h = bbox_max.y - bbox_min.y;
    if (bbox_w < 0.0001f) bbox_w = 1.0f;
    if (bbox_h < 0.0001f) bbox_h = 1.0f;

    // Auto-compute height preserving aspect ratio when height == 0
    uint width = options.width;
    uint height = options.height;
    if (height == 0) {
        height = (uint)((float)width * bbox_h / bbox_w);
        if (height < 1) height = 1;
    }

    // Create image filled with background color
    PpmImage img = ppm::new_ppm(alloc, width, height)!!;
    defer catch img.destroy();
    ppm::fill(&img, options.bg_color);

    // World-to-pixel coordinate mapping with margin + Y-flip
    float world_w = bbox_w * (1.0f + 2.0f * margin);
    float world_h = bbox_h * (1.0f + 2.0f * margin);
    float world_min_x = bbox_min.x - bbox_w * margin;
    float world_max_y = bbox_max.y + bbox_h * margin;

    // Draw each half-edge as a line
    for (usz i = 0; i < mesh.half_edges.len; i++) {
        HalfEdge he = mesh.half_edges[i];
        if (he.twin < 0) continue;
        HalfEdge twin = mesh.half_edges[he.twin];

        Vec3f p0 = mesh.positions[he.origin];
        Vec3f p1 = mesh.positions[twin.origin];

        int x0 = (int)((p0.x - world_min_x) / world_w * (float)width);
        int y0 = (int)((world_max_y - p0.y) / world_h * (float)height);
        int x1 = (int)((p1.x - world_min_x) / world_w * (float)width);
        int y1 = (int)((world_max_y - p1.y) / world_h * (float)height);

        ppm::draw_line(&img, x0, y0, x1, y1, options.edge_color, line_width);
    }

    // Draw each vertex as a filled circle
    for (usz i = 0; i < mesh.positions.len; i++) {
        Vec3f p = mesh.positions[i];
        int cx = (int)((p.x - world_min_x) / world_w * (float)width);
        int cy = (int)((world_max_y - p.y) / world_h * (float)height);
        ppm::draw_circle(&img, cx, cy, dot_radius, options.site_color, true, line_width);
    }

    return img;
}
