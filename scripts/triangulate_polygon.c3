module tools;
import cg;
import cg::triangulate;
import cg::utils;
import cg::utils::ppm;

fn int map_x(float x, float wx, float ww, uint w) { return (int)((x - wx) / ww * (float)w); }
fn int map_y(float y, float wy, float wh, uint h) { return (int)((wy - y) / wh * (float)h); }

fn void main()
{
    // L-shaped polygon (CCW) — simple, non-convex, monotone
    Vec3f[8] polygon = {
        { 0, 0, 0 },
        { 3, 0, 0 },
        { 3, 1, 0 },
        { 2, 1, 0 },
        { 2, 3, 0 },
        { 1, 3, 0 },
        { 1, 1, 0 },
        { 0, 1, 0 },
    };

    // Triangulate
    HalfEdgeMesh mesh = triangulate::triangulate_polygon(mem, polygon[..])!!;
    defer mesh.destroy();

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

    // Draw triangulation edges FIRST (thin red) — boundary will overlay later
    for (FaceIndex f = (FaceIndex)0; (int)f < (int)mesh.faces.len; f++) {
        HeIndex he = mesh.faces[(usz)f].half_edge;
        int a = (int)mesh.from_vertex(he);
        int b = (int)mesh.to_vertex(he);
        int c = (int)mesh.from_vertex(mesh.prev(he));
        int ax = map_x(polygon[a].x, wx, ww, width);
        int ay = map_y(polygon[a].y, wy, wh, height);
        int bx = map_x(polygon[b].x, wx, ww, width);
        int by = map_y(polygon[b].y, wy, wh, height);
        int cx = map_x(polygon[c].x, wx, ww, width);
        int cy = map_y(polygon[c].y, wy, wh, height);
        ppm::draw_line(&img, ax, ay, bx, by, 0xFFFF0000, 1);
        ppm::draw_line(&img, bx, by, cx, cy, 0xFFFF0000, 1);
        ppm::draw_line(&img, cx, cy, ax, ay, 0xFFFF0000, 1);
    }

    // Draw polygon boundary LAST (thick black) — overlays matching triangle edges
    for (usz i = 0; i < polygon.len; i++) {
        usz j = (i + 1) % polygon.len;
        ppm::draw_line(&img,
            map_x(polygon[i].x, wx, ww, width), map_y(polygon[i].y, wy, wh, height),
            map_x(polygon[j].x, wx, ww, width), map_y(polygon[j].y, wy, wh, height),
            0xFF000000, 2);
    }

    // Draw vertices (blue dots)
    for (usz i = 0; i < polygon.len; i++) {
        ppm::draw_circle(&img,
            map_x(polygon[i].x, wx, ww, width), map_y(polygon[i].y, wy, wh, height),
            5, 0xFF0000FF, true, 1);
    }

    ppm::write_ppm(&img, "triangulate_polygon.ppm");
}
