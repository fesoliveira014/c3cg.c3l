# PPM Image Writer + Delaunay Renderer Implementation Plan

> **For Hermes:** Use subagent-driven-development skill to implement this plan task-by-task.

**Goal:** Implement `cg::utils::ppm` (PPM writer) + `cg::utils::render_delaunay`.

**Architecture:** Two new submodules: `cg::utils::ppm` for image buffer + drawing primitives, `cg::utils` for Delaunay rendering. P6 binary PPM output.

**Spec:** `docs/specs/2026-05-22-ppm-writer-design.md`

**Key files:**
- `src/utils/ppm.c3` (new — PPM writer)
- `src/utils/utils.c3` (new — render_delaunay)
- `src/c3cg.c3i` (modify — new modules + API)
- `test/test_ppm.c3` (new)
- `project.json` (probably auto-detects `src/**` — verify)

---

### Task 1: Stubs + umbrella (green build)

**Files:**
- Create: `src/utils/ppm.c3`, `src/utils/utils.c3`, `test/test_ppm.c3`
- Modify: `src/c3cg.c3i`

**Step 1: ppm.c3 stub**
```c3
module cg::utils::ppm;
import cg;
```

**Step 2: utils.c3 stub**
```c3
module cg::utils;
import cg;
import cg::utils::ppm;
```

**Step 3: test stub**
```c3
module test;
```

**Step 4: umbrella**

In `src/c3cg.c3i`, after existing module sections add:

```c3
module cg::utils;
import cg;
import cg::utils::ppm;

fn PpmImage? render_delaunay(Allocator alloc, HalfEdgeMesh* mesh, PpmOptions options);

module cg::utils::ppm;
import cg;

struct PpmOptions {
    uint width;
    uint height;
    uint edge_color;
    uint site_color;
    uint bg_color;
    uint line_width;
    uint dot_radius;
    float margin;
}

struct PpmImage {
    uint   width;
    uint   height;
    uint[] pixels;
}

fn void      PpmImage.destroy(&self);
fn PpmImage? new_ppm(Allocator alloc, uint width, uint height);
fn void      fill(PpmImage* img, uint color);
fn void      draw_line(PpmImage* img, int x0, int y0, int x1, int y1, uint color, uint line_width = 1);
fn void      draw_circle(PpmImage* img, int cx, int cy, uint radius, uint color, bool fill = true, uint line_width = 1);
fn void      write_ppm(PpmImage* img, char[] path) @io;
```

**Step 5: Build & commit**
```bash
c3c build debug && git add -A && git commit -m "utils: stub ppm and utils modules"
```

---

### Task 2: PPM core (new_ppm, fill, destroy, write_ppm)

**File:** `src/utils/ppm.c3`

**Step 1: Implement destroy, new_ppm, fill**

```c3
fn void PpmImage.destroy(&self) {
    free(self.pixels);
    *self = {};
}

fn PpmImage? new_ppm(Allocator alloc, uint width, uint height) {
    PpmImage img;
    img.width = width;
    img.height = height;
    img.pixels = mem::alloc::new_array(alloc, uint, (sz)((usz)width * (usz)height));
    defer catch free(img.pixels);
    // Initialize to black
    for (usz i = 0; i < img.pixels.len; i++) img.pixels[i] = 0xFF000000;
    return img;
}

fn void fill(PpmImage* img, uint color) {
    for (usz i = 0; i < img.pixels.len; i++) img.pixels[i] = color;
}
```

**Step 2: Implement write_ppm (P6 binary)**

```c3
fn void write_ppm(PpmImage* img, char[] path) @io {
    // Open file, write header, write pixel bytes
    // P6\n<w> <h>\n255\n + RGB bytes
    // Uses std::io::open / std::io::write
}
```

**Step 3: Write basic tests**
- `test_new_ppm`: check dimensions + pixel count
- `test_fill`: fill with color, verify all pixels

**Step 4: Build & test → commit**

---

### Task 3: Bresenham line (draw_line)

**File:** `src/utils/ppm.c3`

```c3
fn void draw_line(PpmImage* img, int x0, int y0, int x1, int y1, uint color, uint line_width = 1) {
    // Standard integer Bresenham
    int dx = abs(x1 - x0), dy = -abs(y1 - y0);
    int sx = x0 < x1 ? 1 : -1, sy = y0 < y1 ? 1 : -1;
    int err = dx + dy;
    while (true) {
        set_pixel(img, x0, y0, color);  // inline clamp
        if (line_width > 1) {
            // Draw perpendicular offsets
            int perp_x = -(y1 - y0), perp_y = x1 - x0;
            float len = sqrt(perp_x*perp_x + perp_y*perp_y);
            int step_x = (int)(perp_x / len), step_y = (int)(perp_y / len);
            for (uint w = 1; w <= line_width/2; w++) { ... }
        }
        if (x0 == x1 && y0 == y1) break;
        int e2 = 2 * err;
        if (e2 >= dy) { err += dy; x0 += sx; }
        if (e2 <= dx) { err += dx; y0 += sy; }
    }
}
```

For `line_width > 1`: precompute perpendicular vector, draw `line_width` copies at ± offsets. Center for odd widths.

Helper: `set_pixel(img, x, y, color)` — clamps coordinates, writes to `pixels[y * width + x]`.

Tests: horizontal, vertical, diagonal, single-pixel, width=3, line outside bounds (clamped).

---

### Task 4: Circle drawing (draw_circle)

**File:** `src/utils/ppm.c3`

**Filled circle:**
```
for y from cy - r to cy + r:
    if y < 0 or y >= height: continue
    dx = (int)sqrt(r² - (y-cy)²)
    for x from max(0, cx-dx) to min(width-1, cx+dx):
        pixels[y*width + x] = color
```

**Outline circle (fill=false):**
Draw filled circle at `radius + line_width/2` then fill the interior region `radius - line_width/2` with bg_color. Or draw a ring directly by checking per-pixel distance.

Tests: filled (center + pixels in radius), outline (ring, interior clear), radius 0 (single pixel).

---

### Task 5: render_delaunay

**File:** `src/utils/utils.c3`

```c3
fn PpmImage? render_delaunay(Allocator alloc, HalfEdgeMesh* mesh, PpmOptions options) {
    // Defaults
    if (options.line_width == 0) options.line_width = 1;
    if (options.dot_radius == 0) options.dot_radius = 3;
    if (options.margin == 0.0f) options.margin = 0.05f;

    // Compute bbox
    float min_x = ..., max_x = ..., min_y = ..., max_y = ...;
    // If degenerate bbox, add epsilon
    float bw = max_x - min_x, bh = max_y - min_y;
    if (bw < 1e-12f) { min_x -= 1; max_x += 1; bw = 2; }
    if (bh < 1e-12f) { min_y -= 1; max_y += 1; bh = 2; }

    // Compute image dimensions
    uint img_w = options.width;
    uint img_h = options.height;
    if (img_h == 0) img_h = (uint)(img_w * bh / bw);
    if (img_h == 0) img_h = img_w;

    PpmImage img = new_ppm(alloc, img_w, img_h)!;
    fill(&img, options.bg_color);

    // Draw edges (half-edges, skip reverse by twin > e)
    for (usz e = 0; e < mesh.half_edges.len; e++) {
        HeIndex twin = mesh.half_edges[e].twin;
        if (twin != INVALID_HE && (int)twin < (int)e) continue;
        Vec3f p0 = mesh.positions[mesh.half_edges[e].origin];
        Vec3f p1 = mesh.positions[mesh.half_edges[mesh.half_edges[e].next].origin];
        int px0, py0, px1, py1;
        world_to_pixel(p0.x, p0.y, min_x, max_x, min_y, max_y, options.margin, img_w, img_h, &px0, &py0);
        world_to_pixel(p1.x, p1.y, min_x, max_x, min_y, max_y, options.margin, img_w, img_h, &px1, &py1);
        draw_line(&img, px0, py0, px1, py1, options.edge_color, options.line_width);
    }

    // Draw sites
    for (usz v = 0; v < mesh.vertices.len; v++) {
        Vec3f p = mesh.positions[v];
        int px, py;
        world_to_pixel(p.x, p.y, min_x, max_x, min_y, max_y, options.margin, img_w, img_h, &px, &py);
        draw_circle(&img, px, py, options.dot_radius, options.site_color, true, 1);
    }

    return img;
}
```

Tests: icosahedron render → non-empty image with edge + site pixels.

---

### Task 6: Full test suite + final verification

**File:** `test/test_ppm.c3`

All tests from spec table + icosahedron render test.

```bash
c3c clean && c3c build release && c3c test
```

Expected: all pass.

Note: write_ppm file test can be skipped (requires temp file I/O in test runner).
