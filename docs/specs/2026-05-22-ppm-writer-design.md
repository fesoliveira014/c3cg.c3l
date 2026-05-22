# PPM Image Writer + Delaunay Renderer

## Overview

Simple PPM (P6 binary) image writer for visualizing Delaunay triangulations. New `cg::utils::ppm` module for the image format, `cg::utils` for Delaunay rendering. No external dependencies.

## Modules

```
src/utils/
├── utils.c3    module cg::utils;           — render_delaunay
└── ppm.c3      module cg::utils::ppm;       — PPM writer
```

## Public API

### `cg::utils::ppm` — `src/utils/ppm.c3`

```c3
module cg::utils::ppm;
import cg;

struct PpmOptions {
    uint width;          // image width in pixels
    uint height;         // 0 = auto-compute from aspect ratio
    uint edge_color;     // 0x00RRGGBB
    uint site_color;     // 0x00RRGGBB
    uint bg_color;       // 0x00RRGGBB
    uint line_width;     // ≥ 1 (default 1)
    uint dot_radius;     // ≥ 1 (default 3)
    float margin;        // fraction of bbox padding, e.g. 0.05
}

struct PpmImage {
    uint   width;
    uint   height;
    uint[] pixels;       // row-major, packed 0x00RRGGBB
}

fn void      PpmImage.destroy(&self);
fn PpmImage? new_ppm(Allocator alloc, uint width, uint height);
fn void      fill(PpmImage* img, uint color);

// Draw a Bresenham line segment. line_width > 1 → parallel offset copies.
fn void draw_line(PpmImage* img, int x0, int y0, int x1, int y1, uint color, uint line_width = 1);

// Draw a circle. fill=true (default) → filled disc. fill=false → outline ring
// of thickness line_width centered at radius.
fn void draw_circle(PpmImage* img, int cx, int cy, uint radius, uint color, bool fill = true, uint line_width = 1);

// Write PPM image to file (P6 binary format).
fn void write_ppm(PpmImage* img, char[] path) @io;
```

### `cg::utils` — `src/utils/utils.c3`

```c3
module cg::utils;
import cg;
import cg::utils::ppm;

fn PpmImage? render_delaunay(Allocator alloc, HalfEdgeMesh* mesh, PpmOptions options);
```

## Pipeline: `render_delaunay`

1. Compute bbox from `mesh.positions` (min/max x,y; ignore z).
2. `options.height == 0` → compute from `options.width × (bbox_height / bbox_width)`.
3. `new_ppm(alloc, width, height)`.
4. `fill(img, options.bg_color)`.
5. For each unique half-edge (only forward, skip reverse by twin comparison to avoid double-draw):
   - `p0 = mesh.positions[mesh.half_edges[e].origin]`
   - `p1 = mesh.positions[mesh.half_edges[mesh.half_edges[e].next].origin]`
   - Map `(p0.x, p0.y)` → pixel via `world_to_pixel`, then `draw_line(img, x0, y0, x1, y1, options.edge_color, options.line_width)`.
6. For each vertex `v`:
   - Map vertex position to pixel.
   - `draw_circle(img, px, py, options.dot_radius, options.site_color, true, 1)`.
7. Return `PpmImage` (caller owns, must destroy). Caller optionally calls `write_ppm` to save.

### Coordinate mapping

```
world_to_pixel(p, bbox, img_w, img_h, margin):
    // margin must be < 0.5
    x_norm = (p.x - bbox.min.x) / (bbox.max.x - bbox.min.x)
    y_norm = (p.y - bbox.min.y) / (bbox.max.y - bbox.min.y)
    x_norm = margin + x_norm * (1 - 2 * margin)
    y_norm = margin + y_norm * (1 - 2 * margin)
    px = (int)(x_norm * img_w)
    py = (int)((1.0 - y_norm) * img_h)
    clamp to [0, img_w-1] × [0, img_h-1]
```

## PPM Format (P6 binary)

```
P6\n
<width> <height>\n
255\n
<RGB bytes, width×height×3>
```

Header is ASCII. Pixel data is binary, row-major, top-left origin.

## Bresenham line

Standard integer Bresenham. For `line_width > 1`: compute perpendicular direction, draw `line_width` offset copies at `±0, ±1, ..., ±(line_width/2)` pixel steps. Odd widths center, even widths lean.

## Circle drawing

**fill=true**: for `y` from `cy - r` to `cy + r`, compute `dx = sqrt(r² - (y - cy)²)`, then horizontal fill from `cx - dx` to `cx + dx`.

**fill=false**: two passes. Inner passes compute pixels where `r - line_width/2 ≤ distance ≤ r + line_width/2`. Or draw filled circle at radius `r + line_width/2` then "subtract" filled circle at radius `r - line_width/2` via drawing in bg_color. Simpler and correct for all widths.

## Memory

`PpmImage` owns `pixels` array. Caller calls `destroy()`. All internal operations write directly to the pixel buffer — no intermediate allocations.

## Tests

File: `test/test_ppm.c3`

| Test | Expected |
|------|----------|
| `new_ppm` creates correct dimensions | `img.width == w`, `img.height == h`, `img.pixels.len == w*h` |
| `fill` sets all pixels | Random color, verify all pixels equal |
| `draw_line` horizontal | Correct pixels set, no OOB |
| `draw_line` vertical | Same |
| `draw_line` diagonal (45°) | Pixel count correct, endpoints included |
| `draw_line` single-pixel (point) | Exactly one pixel set |
| `draw_circle` filled | Pixels within radius are set, outside are not, center included |
| `draw_circle` outline | Ring pixels set, interior not |
| `draw_circle` radius 0 | Single pixel |
| `draw_line` width=3 | Wider than width=1, no gaps between offsets |
| `render_delaunay` with icosahedron | Non-empty image, edge-colored pixels exist, site-colored pixels exist |
| `write_ppm` round-trip | File created, non-empty |

## Edge cases

- Empty mesh (0 vertices): empty image filled with bg_color, no crash
- All vertices at same position (degenerate bbox): add small epsilon to bbox size
- Line coordinates outside image: clamp to bounds before writing pixels
- Circle radius 0: single pixel drawn
- `line_width` odd vs even: both centered correctly
- `options.height == 0` and bbox has zero height: fallback to `width`
