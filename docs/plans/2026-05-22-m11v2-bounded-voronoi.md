# M11v2 — Bounded Voronoi Implementation Plan

> **For Hermes:** Use subagent-driven-development skill to implement this plan task-by-task.

**Goal:** Implement `in_polygon` and `in_box` — bounded Voronoi via voronator-rs direct approach (Delaunay → circumcenters → vertex_one_ring_faces → S-H clip).

**Architecture:** No dual, no boundary-check removals. Prefilter sites, add 4 helper points, Delaunay, circumcenters, walk vertex one-rings for cell polygons, S-H clip, `from_polygons`.

**Spec:** `docs/specs/2026-05-22-m11v2-bounded-voronoi-design.md`

**Key files:**
- `src/voronoi/bounded.c3` (new — single file, all logic)
- `src/c3cg.c3i` (modify — new faultdef + API declarations)
- `test/test_voronoi_bounded.c3` (new — 14 tests)

---

### Task 1: Stubs + API declarations (green build)

**Files:**
- Create: `src/voronoi/bounded.c3`, `test/test_voronoi_bounded.c3`
- Modify: `src/c3cg.c3i`

**Step 1: bounded.c3 stub**

```c3
module cg::voronoi;
import cg;
import cg::geometry;
import cg::half_edge;
import cg::delaunay;
import std::math;
```

**Step 2: test stub**

```c3
module test;
import cg;
import cg::voronoi;
```

**Step 3: c3cg.c3i additions**

In root fault block add `NON_CONVEX_BOUNDING_POLYGON,` (before final `;`).

In `module cg::voronoi;` after `to_delaunay`:
```c3
fn VoronoiDiagram? in_polygon(Allocator alloc, Vec3f[] sites, Vec3f[] polygon);
fn VoronoiDiagram? in_box(Allocator alloc, Vec3f[] sites, Aabb bbox);
```

**Step 4: Build & commit**

```bash
c3c build debug && git add -A && git commit -m "voronoi: stub bounded.c3, faultdef, API declarations"
```

---

### Task 2: Geometry helpers (S-H, line intersection, polygon validation)

**File:** `src/voronoi/bounded.c3`

Implement 5 private helpers.

**line_intersection**:
```c3
fn Vec3f? line_intersection(Vec2f a, Vec2f b, Vec2f p, Vec2f q) {
    float d1x = b.x - a.x; float d1y = b.y - a.y;
    float d2x = q.x - p.x; float d2y = q.y - p.y;
    float det = d1x * d2y - d1y * d2x;
    if (math::abs(det) < geometry::GEOMETRY_EPSILON) return {};
    float t = ((p.x - a.x) * d2y - (p.y - a.y) * d2x) / det;
    return { a.x + t * d1x, a.y + t * d1y, 0 };
}
```

**sutherland_hodgman**: uses `cap = polygon.len * 4`, Vec3f→Vec2f conversion for `orient_2d`. Returns `Vec3f[]?`. On <3 output → empty slice. Uses `(float)orient_2d(...)` cast. Optional check: `if (try iv = isect)` → `output[out_len] = iv; out_len++;`

**half_plane_clip**: perpendicular bisector via dot product. `dot(p-m, d) * a_dot >= 0` for keep_a_side.

**point_in_convex_polygon**: all `orient_2d({a.x,a.y}, {b.x,b.y}, {p.x,p.y}) >= (PredicateSign)0`.

**is_convex_polygon**: all triples `>= 0`, at least one `> 0`.

Commit: `git commit -m "voronoi: implement S-H, half-plane clip, polygon helpers"`

---

### Task 3: in_polygon (prefilter, N=0/1/2 branches)

**File:** `src/voronoi/bounded.c3`

Implement `in_polygon` with prefilter and N<3 branches. N≥3 body is a stub (`return {};`).

**Prefilter**: `inside_sites` allocated to sites.len capacity (not all will be used). Then for each site, `point_in_convex_polygon(site, polygon)` → count survivors, compact in-place.

**N=0**: `VoronoiDiagram result; result.mesh = {}; result.sites = {}; return result;`

**N=1**: positions = polygon, indices = 0..n-1, offsets = {0, n}. `from_polygons`, allocate 1-site array.

**N=2**: call `half_plane_clip` for each side, accumulate surviving cells, `from_polygons`, allocate sites.

Commit: `git commit -m "voronoi: implement in_polygon prefilter and N=0/1/2 branches"`

---

### Task 4: in_polygon N≥3 (helper-point path)

**File:** `src/voronoi/bounded.c3`

Full N≥3 body:

```
1. Bbox computation, 4 helpers at exact positions from spec
2. all_points = inside_sites + helpers
3. delaunay_mesh = delaunay_2d(alloc, all_points)!
4. centers = circumcenters_planar(alloc, &delaunay_mesh)!
5. vertex_map[i] = position match inside_sites[i] to delaunay_mesh.positions
6. First pass: for each matched vertex, vertex_one_ring_faces → circumcenters → S-H clip, count survivors + total verts
7. Second pass: same, accumulate positions + face_indices, build face_offsets
8. Deduplicate vertex positions (nearest-neighbor within 1e-12, remap indices)
9. from_polygons → allocate sites → return VoronoiDiagram
```

Commit: `git commit -m "voronoi: implement in_polygon N≥3 helper-point Delaunay path"`

---

### Task 5: in_box and full test suite

**File:** `src/voronoi/bounded.c3`, `test/test_voronoi_bounded.c3`

**in_box**: convert Aabb to 4-vertex polygon, call `in_polygon`.

**Tests** (14 per spec table):

```c3
fn void test_in_polygon_four_sites_square() @test {
    Vec3f[4] s = { {0.25,0.25,0}, {0.75,0.25,0}, {0.75,0.75,0}, {0.25,0.75,0} };
    Vec3f[4] p = { {0,0,0}, {1,0,0}, {1,1,0}, {0,1,0} };
    VoronoiDiagram d = voronoi::in_polygon(mem, s[..], p[..])!!; defer d.destroy();
    assert(d.mesh.faces.len == 4); assert(d.sites.len == 4);
    d.mesh.validate()!!;
}

// N=1 inside, N=1 outside, N=2 both, N=2 one out, N=2 both out
// Triangle polygon, all outside, non-convex fault, CW fault
// <3 verts fault, collinear fault, empty sites fault, in_box basic
```

Commit: `git commit -m "voronoi: implement in_box and bounded voronoi test suite"`

---

### Task 6: Final verification

```bash
c3c clean && c3c build release && c3c test
```

Mark M11 complete in AGENTS.md. Commit.
