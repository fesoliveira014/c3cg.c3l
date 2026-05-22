module cg::utils;
import cg;
import cg::utils::ppm;

// ── HSV → RGB helpers ──

fn uint hsv_to_rgb(float hue, float s, float v)
{
    float c = v * s;
    float hp = hue / 60.0f;
    int hi = (int)hp;
    if (hi > 5) { hi = 5; }
    else if (hi < 0) { hi = 0; }
    float fr = hp - (float)hi;
    if ((hi & 1) != 0) fr = 1.0f - fr;
    float m = v - c;
    float rf, gf, bf;
    switch (hi)
    {
        case 0: rf = c;      gf = fr * c; bf = 0.0f;
        case 1: rf = fr * c; gf = c;      bf = 0.0f;
        case 2: rf = 0.0f;   gf = c;      bf = fr * c;
        case 3: rf = 0.0f;   gf = fr * c; bf = c;
        case 4: rf = fr * c; gf = 0.0f;   bf = c;
        default: rf = c;    gf = 0.0f;   bf = fr * c;
    }
    uint r = (uint)((rf + m) * 255.0f);
    uint g = (uint)((gf + m) * 255.0f);
    uint b = (uint)((bf + m) * 255.0f);
    return (r << 16) | (g << 8) | b;
}

fn void fill_polygon_scanline(PpmImage* img, int[] px, int[] py, usz n, uint color)
{
    if (n < 3) return;

    int min_y = py[0];
    int max_y = py[0];
    for (usz i = 1; i < n; i++) {
        if (py[i] < min_y) min_y = py[i];
        if (py[i] > max_y) max_y = py[i];
    }

    if (min_y < 0) min_y = 0;
    if (max_y >= (int)img.height) max_y = (int)img.height - 1;
    if (min_y > max_y) return;

    int[64] x_intersections;
    for (int y = min_y; y <= max_y; y++)
    {
        usz num_x = 0;
        for (usz e = 0; e < n; e++)
        {
            usz i0 = e;
            usz i1 = (e + 1) % n;
            int y0 = py[i0];
            int y1 = py[i1];
            if (y0 == y1) continue;

            int y_lo = y0 < y1 ? y0 : y1;
            int y_hi = y0 > y1 ? y0 : y1;
            if (y < y_lo || y >= y_hi) continue;

            int x0 = px[i0];
            int x1 = px[i1];
            int xi = x0 + (int)((float)(y - y0) * (float)(x1 - x0) / (float)(y1 - y0));
            x_intersections[num_x] = xi;
            num_x++;
        }

        for (usz i = 0; i < num_x; i++)
        {
            for (usz j = i + 1; j < num_x; j++)
            {
                if (x_intersections[i] > x_intersections[j])
                {
                    int tmp = x_intersections[i];
                    x_intersections[i] = x_intersections[j];
                    x_intersections[j] = tmp;
                }
            }
        }

        for (usz i = 0; i + 1 < num_x; i += 2)
        {
            int x_left = x_intersections[i];
            int x_right = x_intersections[i + 1];
            if (x_left < 0) x_left = 0;
            if (x_right >= (int)img.width) x_right = (int)img.width - 1;
            for (int x = x_left; x <= x_right; x++)
            {
                img.pixels[(usz)y * (usz)img.width + (usz)x] = color;
            }
        }
    }
}

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

    // Draw each half-edge as a line (skip reverse to avoid double-draw)
    for (usz i = 0; i < mesh.half_edges.len; i++) {
        HalfEdge he = mesh.half_edges[i];
        // Draw boundary edges always. For interior edges, draw only one direction.
        if (he.twin != cg::INVALID_HE && i > (usz)he.twin) continue;

        Vec3f p0 = mesh.positions[he.origin];
        HeIndex next_he = he.next;
        Vec3f p1 = mesh.positions[mesh.half_edges[next_he].origin];

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

fn PpmImage? render_voronoi(Allocator alloc, VoronoiDiagram* diagram, PpmOptions options)
{
    HalfEdgeMesh* mesh = &diagram.mesh;
    if (mesh.positions.len == 0) return cg::EMPTY_INPUT~;

    Vec3f bbox_min = mesh.positions[0];
    Vec3f bbox_max = mesh.positions[0];
    for (usz i = 1; i < mesh.positions.len; i++) {
        Vec3f p = mesh.positions[i];
        if (p.x < bbox_min.x) bbox_min.x = p.x;
        if (p.y < bbox_min.y) bbox_min.y = p.y;
        if (p.x > bbox_max.x) bbox_max.x = p.x;
        if (p.y > bbox_max.y) bbox_max.y = p.y;
    }

    uint line_width = options.line_width;
    if (line_width < 1) line_width = 1;
    uint dot_radius = options.dot_radius;
    if (dot_radius < 1) dot_radius = 1;
    float margin = options.margin;
    if (margin < 0.0f) margin = 0.05f;

    float bbox_w = bbox_max.x - bbox_min.x;
    float bbox_h = bbox_max.y - bbox_min.y;
    if (bbox_w < 0.0001f) bbox_w = 1.0f;
    if (bbox_h < 0.0001f) bbox_h = 1.0f;

    uint width = options.width;
    uint height = options.height;
    if (height == 0) {
        height = (uint)((float)width * bbox_h / bbox_w);
        if (height < 1) height = 1;
    }

    PpmImage img = ppm::new_ppm(alloc, width, height)!!;
    defer catch img.destroy();
    ppm::fill(&img, options.bg_color);

    float world_w = bbox_w * (1.0f + 2.0f * margin);
    float world_h = bbox_h * (1.0f + 2.0f * margin);
    float world_min_x = bbox_min.x - bbox_w * margin;
    float world_max_y = bbox_max.y + bbox_h * margin;

    VertexIndex[] face_verts = mem::alloc::new_array(alloc, VertexIndex, (sz) mesh.vertices.len);
    defer free(face_verts);
    int[] px = mem::alloc::new_array(alloc, int, (sz) mesh.vertices.len);
    defer free(px);
    int[] py = mem::alloc::new_array(alloc, int, (sz) mesh.vertices.len);
    defer free(py);

    float total_faces = (float)mesh.faces.len;
    for (usz fi = 0; fi < mesh.faces.len; fi++) {
        FaceIndex f = (FaceIndex)(int)fi;
        int degree = mesh.face_vertices(f, face_verts)!;

        for (int i = 0; i < degree; i++) {
            Vec3f p = mesh.positions[face_verts[i]];
            px[i] = (int)((p.x - world_min_x) / world_w * (float)width);
            py[i] = (int)((world_max_y - p.y) / world_h * (float)height);
        }

        float hue = (float)fi / total_faces * 360.0f;
        uint fill_color = hsv_to_rgb(hue, 0.7f, 0.9f);
        fill_polygon_scanline(&img, px[..degree], py[..degree], (usz)degree, fill_color);
    }

    for (usz i = 0; i < mesh.half_edges.len; i++) {
        HalfEdge he = mesh.half_edges[i];
        if (he.twin != cg::INVALID_HE && i > (usz)he.twin) continue;

        Vec3f p0 = mesh.positions[he.origin];
        Vec3f p1 = mesh.positions[mesh.half_edges[he.next].origin];

        int x0 = (int)((p0.x - world_min_x) / world_w * (float)width);
        int y0 = (int)((world_max_y - p0.y) / world_h * (float)height);
        int x1 = (int)((p1.x - world_min_x) / world_w * (float)width);
        int y1 = (int)((world_max_y - p1.y) / world_h * (float)height);

        ppm::draw_line(&img, x0, y0, x1, y1, options.edge_color, line_width);
    }

    for (usz i = 0; i < diagram.sites.len; i++) {
        Vec3f p = diagram.sites[i];
        int cx = (int)((p.x - world_min_x) / world_w * (float)width);
        int cy = (int)((world_max_y - p.y) / world_h * (float)height);
        ppm::draw_circle(&img, cx, cy, dot_radius, options.site_color, true, line_width);
    }

    return img;
}
