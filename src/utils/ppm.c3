module cg::utils::ppm;
import cg;
import std::math;
import libc;

const uint PPM_DEFAULT_BG = 0xFF000000;

fn void PpmImage.destroy(&self)
{
    free(self.pixels);
    *self = {};
}

fn PpmImage? new_ppm(Allocator alloc, uint width, uint height)
{
    PpmImage img;
    img.width = width;
    img.height = height;
    img.pixels = mem::alloc::new_array(alloc, uint, (sz)((usz)width * (usz)height));
    defer catch free(img.pixels);
    for (usz i = 0; i < img.pixels.len; i++) img.pixels[i] = PPM_DEFAULT_BG;
    return img;
}

fn void fill(PpmImage* img, uint color)
{
    for (usz i = 0; i < img.pixels.len; i++) img.pixels[i] = color;
}

// ── Internal helpers ──

fn void set_pixel(PpmImage* img, int x, int y, uint color)
{
    if (x < 0 || x >= (int)img.width || y < 0 || y >= (int)img.height) return;
    img.pixels[(usz)y * (usz)img.width + (usz)x] = color;
}

fn void draw_line(PpmImage* img, int x0, int y0, int x1, int y1, uint color, uint line_width)
{
    if (line_width < 1) line_width = 1;

    int dx = math::abs(x1 - x0);
    int dy = -math::abs(y1 - y0);
    int sx = x0 < x1 ? 1 : (x0 > x1 ? -1 : 0);
    int sy = y0 < y1 ? 1 : (y0 > y1 ? -1 : 0);

    // Precompute perpendicular offsets for line width
    int perp_x = -(y1 - y0);
    int perp_y = x1 - x0;
    float len = math::sqrt((float)(perp_x * perp_x + perp_y * perp_y));
    int step_x = 0;
    int step_y = 0;
    if (len > 0.5f) {
        step_x = (int)math::round((float)perp_x / len);
        step_y = (int)math::round((float)perp_y / len);
    }

    int err = dx + dy;
    int cx = x0;
    int cy = y0;
    while (true) {
        // Draw perpendicular offsets for line width
        int hw = (int)line_width;
        int offset_start = -(hw - 1) / 2;
        for (int w = 0; w < hw; w++) {
            set_pixel(img, cx + (offset_start + w) * step_x, cy + (offset_start + w) * step_y, color);
        }
        if (cx == x1 && cy == y1) break;
        int e2 = 2 * err;
        if (e2 >= dy) { err += dy; cx += sx; }
        if (e2 <= dx) { err += dx; cy += sy; }
    }
}

fn void draw_circle(PpmImage* img, int cx, int cy, uint radius, uint color, bool fill, uint line_width)
{
    int r = (int)radius;
    if (r < 0) return;

    if (fill) {
        for (int y = cy - r; y <= cy + r; y++) {
            if (y < 0 || y >= (int)img.height) continue;
            int dx = (int)math::sqrt((float)(r * r - (y - cy) * (y - cy)));
            int left = cx - dx;
            int right = cx + dx;
            if (left < 0) left = 0;
            if (right >= (int)img.width) right = (int)img.width - 1;
            for (int x = left; x <= right; x++) {
                img.pixels[(usz)y * (usz)img.width + (usz)x] = color;
            }
        }
    } else {
        if (line_width < 1) line_width = 1;
        int lw = (int)line_width;
        int outer_r = r + lw / 2;
        int inner_r = r - (lw + 1) / 2;
        if (inner_r < 0) inner_r = 0;

        for (int y = cy - outer_r; y <= cy + outer_r; y++) {
            if (y < 0 || y >= (int)img.height) continue;
            int dy2 = (y - cy) * (y - cy);
            int o_dx = (int)math::sqrt((float)(outer_r * outer_r - dy2));
            int i_dx = inner_r > 0 ? (int)math::sqrt((float)(inner_r * inner_r - dy2)) : -1;

            int left = cx - o_dx;
            int right = cx + o_dx;
            if (left < 0) left = 0;
            if (right >= (int)img.width) right = (int)img.width - 1;

            if (inner_r <= 0) {
                for (int x = left; x <= right; x++) {
                    img.pixels[(usz)y * (usz)img.width + (usz)x] = color;
                }
            } else {
                int i_left = cx - i_dx;
                int i_right = cx + i_dx;
                // Left outer segment
                for (int x = left; x <= right; x++) {
                    bool in_inner = (x >= i_left && x <= i_right);
                    if (!in_inner) {
                        img.pixels[(usz)y * (usz)img.width + (usz)x] = color;
                    }
                }
            }
        }
    }
}

// write_ppm uses libc for file I/O
fn void write_ppm(PpmImage* img, char[] path)
{
    String header = string::tformat("P6\n%d %d\n255\n", img.width, img.height);
    ZString path_z = (ZString)path;
    CFile f = libc::fopen(path_z, "wb");
    if (!f) return;
    defer libc::fclose(f);

    libc::fwrite((char[])header, 1, (usz)header.len, f);
    for (usz i = 0; i < img.pixels.len; i++) {
        uint p = img.pixels[i];
        char[3] rgb = { (char)((p >> 16) & 0xFF), (char)((p >> 8) & 0xFF), (char)(p & 0xFF) };
        libc::fwrite(&rgb, 1, 3, f);
    }
}
