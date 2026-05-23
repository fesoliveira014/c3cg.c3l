module tools;
import cg;
import cg::triangulate;
import cg::utils;
import cg::utils::ppm;

fn int map_x(float x, float wx, float ww) { return (int)((x - wx) / ww); }
fn int map_y(float y, float wy, float wh) { return (int)((wy - y) / wh); }

fn void main()
{
    // Star-shaped polygon (CCW)
    Vec3f[10] polygon = {
        { 0, 3, 0 },
        { 0.951f, 1.309f, 0 },
        { 3, 0, 0 },
        { 0.951f, -1.309f, 0 },
        { 0, -3, 0 },
        { -0.951f, -1.309f, 0 },
        { -3, 0, 0 },
        { -0.951f, 1.309f, 0 },
        { 0.25f, 0, 0 },
        { -0.25f, 0, 0 },
    };

    // Triangulate
    int[] tri_indices = triangulate::triangulate_polygon(mem, polygon[..])!!;
    defer free(tri_indices);

    // Compute bounding box
    float min_x = polygon[0].x;
    float max_x = polygon[0].x;
    float min_y = polygon[0].y;
    float max_y = polygon[0].y;
    for (usz i = 1; i < polygon.len; i++) {
        Vec3f p = polygon[i];
        if (p.x < min_x) min_x = p.x;
        if (p.x > max_x) max_x = p.x;
        if (p.y < min_y) min_y = p.y;
        if (p.y > max_y) max_y = p.y;
    }

    float margin = 0.1f;
    float world_w = max_x - min_x;
    float world_h = max_y - min_y;
    if (world_w < 0.0001f) world_w = 1.0f;
    if (world_h < 0.0001f) world_h = 1.0f;
    float ww = world_w * (1.0f + 2.0f * margin);
    float wh = world_h * (1.0f + 2.0f * margin);
    float wx = min_x - world_w * margin;
    float wy = max_y + world_h * margin;

    uint width = 800;
    uint height = (uint)((float)width * world_h / world_w);
    if (height < 1) height = 1;

    PpmImage img = ppm::new_ppm(mem, width, height)!!;
    defer img.destroy();
    ppm::fill(&img, 0xFFFFFFFF);

    // Draw polygon edges (black, thick)
    for (usz i = 0; i < polygon.len; i++) {
        usz j = (i + 1) % polygon.len;
        ppm::draw_line(&img,
            map_x(polygon[i].x, wx, ww), map_y(polygon[i].y, wy, wh),
            map_x(polygon[j].x, wx, ww), map_y(polygon[j].y, wy, wh),
            0xFF000000, 2);
    }

    // Draw triangulation edges (red, thin)
    for (usz t = 0; t < tri_indices.len; t += 3) {
        int a = tri_indices[t];
        int b = tri_indices[(usz)(t + 1)];
        int c = tri_indices[(usz)(t + 2)];
        int ax = map_x(polygon[a].x, wx, ww);
        int ay = map_y(polygon[a].y, wy, wh);
        int bx = map_x(polygon[b].x, wx, ww);
        int by = map_y(polygon[b].y, wy, wh);
        int cx = map_x(polygon[c].x, wx, ww);
        int cy = map_y(polygon[c].y, wy, wh);
        ppm::draw_line(&img, ax, ay, bx, by, 0xFFFF0000, 1);
        ppm::draw_line(&img, bx, by, cx, cy, 0xFFFF0000, 1);
        ppm::draw_line(&img, cx, cy, ax, ay, 0xFFFF0000, 1);
    }

    // Draw vertices (red dots)
    for (usz i = 0; i < polygon.len; i++) {
        ppm::draw_circle(&img,
            map_x(polygon[i].x, wx, ww), map_y(polygon[i].y, wy, wh),
            4, 0xFFFF0000, true, 1);
    }

    ppm::write_ppm(&img, "triangulate_polygon.ppm");
}
