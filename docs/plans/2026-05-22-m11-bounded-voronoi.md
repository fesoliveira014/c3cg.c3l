# M11 — Bounded Voronoi Implementation Plan

> **For Hermes:** Use subagent-driven-development skill to implement this plan task-by-task.

**Goal:** Implement `in_polygon` and `in_box` — bounded Voronoi via helper-point Delaunay + per-cell Sutherland-Hodgman clipping.

**Architecture:** Prefilter sites to inside/on polygon. For N ≥ 3, add 4 helper points, compute Delaunay + unbounded Voronoi, clip each original-site cell against the bounding polygon with S-H, rebuild mesh via `from_polygons`. N=1 returns the polygon, N=2 bisects it.

**Tech Stack:** C3 0.8.0, c3c compiler, project's `half_edge::from_polygons`, `dual_from_vertices`, `delaunay_2d`, `orient_2d`.

**Spec:** `docs/specs/2026-05-22-m11-bounded-voronoi-design.md`

**Key files:**
- `src/voronoi/bounded.c3` (new)
- `src/voronoi/voronoi.c3` (modify — remove boundary check)
- `src/dual/dual.c3` (modify — remove boundary check, skip incomplete rings)
- `src/c3cg.c3i` (modify — new faultdef, new API declarations)
- `test/test_voronoi.c3` (modify — update boundary test)
- `test/test_voronoi_bounded.c3` (new)

---

### Task 0: Stub all files (green build)

**Objective:** Create stubs so the build passes before real content is written.

**Files:**
- Create: `src/voronoi/bounded.c3`
- Create: `test/test_voronoi_bounded.c3`

**Step 1: Create bounded.c3 stub**

```c3
module cg::voronoi;
import cg;
```

**Step 2: Create test_voronoi_bounded.c3 stub**

```c3
module test;
```

**Step 3: Build**

```bash
c3c build debug
```
Expected: `Static library 'out/debug.a' created.`

**Step 4: Commit**

```bash
git add src/voronoi/bounded.c3 test/test_voronoi_bounded.c3
git commit -m "voronoi: stub bounded.c3 and test_voronoi_bounded.c3"
```

---

### Task 1: Remove boundary checks from dual and from_delaunay

**Objective:** Allow planar Delaunay meshes (with boundary edges) to produce unbounded Voronoi.

**Files:**
- Modify: `src/dual/dual.c3:17`
- Modify: `src/voronoi/voronoi.c3:18`

**Step 1: Remove boundary check from dual_from_vertices**

In `src/dual/dual.c3`, replace:

```c3
    for (usz i = 0; i < mesh.half_edges.len; i++) {
        if (mesh.half_edges[i].twin == cg::INVALID_HE) return cg::DUAL_REQUIRES_CLOSED_MESH~;
    }
```

With: (remove the block entirely — comment explanation)

```c3
    // Boundary edges are allowed. Vertices with incomplete face rings
    // (walk hits INVALID_HE) are skipped: face_offsets advances by 0.
```

Then modify `vertex_one_ring_faces` loop to skip vertices where the walk is incomplete. In the `for (usz v = 0; v < mesh.vertices.len; v++)` loop, after collecting the one-ring, check if the ring was truncated (the last face's twin hits boundary before returning to start). If incomplete, `face_offsets[v+1] = face_offsets[v]` (skip this vertex).

**Implementation detail:** The `vertex_one_ring_faces` walk naturally stops when it hits `INVALID_HE`. After the walk completes, check if the last edge's twin is `INVALID_HE` — if so, the ring is open/incomplete. For incomplete rings, don't increment the offset.

**Step 2: Remove boundary check from from_delaunay**

In `src/voronoi/voronoi.c3`, remove:

```c3
    for (usz i = 0; i < delaunay.half_edges.len; i++) {
        if (delaunay.half_edges[i].twin == cg::INVALID_HE) {
            return cg::OPEN_CELL_ON_BOUNDARY~;
        }
    }
```

**Step 3: Build & test**

```bash
c3c build debug && c3c test
```
Expected: all existing tests still pass. The boundary test in `test_voronoi.c3` should now succeed (not fault).

**Step 4: Commit**

```bash
git add src/dual/dual.c3 src/voronoi/voronoi.c3
git commit -m "dual/voronoi: remove boundary checks, skip incomplete rings"
```

---

### Task 2: Update M10 boundary test

**Objective:** Change boundary test from expecting fault to expecting success.

**Files:**
- Modify: `test/test_voronoi.c3`

**Step 1: Rewrite test_voronoi_boundary_faults**

Replace:

```c3
fn void test_voronoi_boundary_faults() @test
{
    Vec3f[3] pts = { {0,0,0}, {1,0,0}, {0,1,0} };
    uint[3] idx = {0,1,2};
    HalfEdgeMesh mesh = half_edge::from_triangles(mem, pts[..], idx[..])!!;
    defer mesh.destroy();

    if (catch err = voronoi::from_delaunay(mem, &mesh)) {
        assert(err == cg::OPEN_CELL_ON_BOUNDARY);
        return;
    }
    unreachable();
}
```

With:

```c3
fn void test_voronoi_planar_boundary() @test
{
    Vec3f[3] pts = { {0,0,0}, {1,0,0}, {0,1,0} };
    uint[3] idx = {0,1,2};
    HalfEdgeMesh mesh = half_edge::from_triangles(mem, pts[..], idx[..])!!;
    defer mesh.destroy();

    VoronoiDiagram diagram = voronoi::from_delaunay(mem, &mesh)!!;
    defer diagram.destroy();

    // Planar Delaunay produces unbounded Voronoi — success, not fault.
    assert(diagram.mesh.faces.len >= 1);
    assert(diagram.sites.len == diagram.mesh.faces.len);
}
```

**Step 2: Build & test**

```bash
c3c build debug && c3c test
```
Expected: all tests pass (~176 tests, 0 failures).

**Step 3: Commit**

```bash
git add test/test_voronoi.c3
git commit -m "test: update boundary test — from_delaunay succeeds on planar meshes"
```

---

### Task 3: Add NON_CONVEX_BOUNDING_POLYGON faultdef and API declarations

**Objective:** Add the new fault and public API signatures to the umbrella.

**Files:**
- Modify: `src/c3cg.c3i`

**Step 1: Add faultdef**

In the root `module cg;` fault block (near `EMPTY_INPUT`, `INDEX_OUT_OF_RANGE`), add:

```c3
faultdef NON_CONVEX_BOUNDING_POLYGON;
```

**Step 2: Add voronoi API**

In the `module cg::voronoi;` section, after `to_delaunay`, add:

```c3
fn VoronoiDiagram? in_polygon(Allocator alloc, Vec3f[] sites, Vec3f[] polygon);
fn VoronoiDiagram? in_box(Allocator alloc, Vec3f[] sites, Aabb bbox);
```

**Step 3: Replace bounded.c3 stub**

```c3
module cg::voronoi;
import cg;
import cg::geometry;  // orient_2d, line_intersection
import cg::half_edge;  // from_polygons, face_vertices
import cg::delaunay;   // delaunay_2d
import cg::dual;       // dual_from_vertices

// Sutherland-Hodgman polygon clipping — private helper
fn Vec3f[]? sutherland_hodgman(Allocator alloc, Vec3f[] polygon, Vec3f[] clip_polygon);

// Half-plane clip for N=2 bisector — private helper
fn Vec3f[]? half_plane_clip(Allocator alloc, Vec3f[] polygon, Vec3f a, Vec3f b, bool keep_a_side);

// Line intersection — private helper
fn Vec3f? line_intersection(Vec2f a, Vec2f b, Vec2f p, Vec2f q);
```

**Step 4: Build**

```bash
c3c build debug
```
Expected: compiles (functions not yet defined but signatures match umbrella).

**Step 5: Commit**

```bash
git add src/c3cg.c3i src/voronoi/bounded.c3
git commit -m "voronoi: add NON_CONVEX_BOUNDING_POLYGON fault and bounded API declarations"
```

---

### Task 4: Implement Sutherland-Hodgman and helpers

**Objective:** Core polygon clipping algorithm.

**Files:**
- Modify: `src/voronoi/bounded.c3`

**Step 1: Write failing test**

In `test/test_voronoi_bounded.c3`, add:

```c3
module test;
import cg;
import cg::voronoi;

fn void test_in_polygon_four_sites_square() @test
{
    Vec3f[4] sites = { {0.25,0.25,0}, {0.75,0.25,0}, {0.75,0.75,0}, {0.25,0.75,0} };
    Vec3f[4] poly = { {0,0,0}, {1,0,0}, {1,1,0}, {0,1,0} };

    VoronoiDiagram diagram = voronoi::in_polygon(mem, sites[..], poly[..])!!;
    defer diagram.destroy();

    assert(diagram.mesh.faces.len == 4);
    diagram.mesh.validate()!!;
}
```

**Step 2: Run — expected FAIL**

```bash
c3c build debug && c3c test
```
Expected: test fails — `in_polygon` not fully implemented.

**Step 3: Implement line_intersection**

```c3
fn Vec3f? line_intersection(Vec2f a, Vec2f b, Vec2f p, Vec2f q)
{
    // Cramer's rule for two lines: a→b and p→q
    float d1x = b.x - a.x;
    float d1y = b.y - a.y;
    float d2x = q.x - p.x;
    float d2y = q.y - p.y;
    float det = d1x * d2y - d1y * d2x;
    if (math::abs(det) < 1e-12) return {};

    float t = ((p.x - a.x) * d2y - (p.y - a.y) * d2x) / det;
    return { a.x + t * d1x, a.y + t * d1y, 0 };
}
```

**Step 4: Implement sutherland_hodgman**

```c3
fn Vec3f[]? sutherland_hodgman(Allocator alloc, Vec3f[] polygon, Vec3f[] clip_polygon)
{
    usz cap = polygon.len * 2;  // output can grow
    Vec3f[] input = mem::alloc::new_array(alloc, Vec3f, (sz)cap);
    defer free(input);
    Vec3f[] output = mem::alloc::new_array(alloc, Vec3f, (sz)cap);
    defer free(output);
    for (usz i = 0; i < polygon.len; i++) input[i] = polygon[i];
    usz input_len = polygon.len;

    for (usz e = 0; e < clip_polygon.len; e++) {
        Vec3f a = clip_polygon[e];
        Vec3f b = clip_polygon[(e + 1) % clip_polygon.len];
        Vec2f a2 = { a.x, a.y };
        Vec2f b2 = { b.x, b.y };

        usz out_len = 0;
        for (usz i = 0; i < input_len; i++) {
            Vec3f p = input[i];
            Vec3f q = input[(i + 1) % input_len];
            Vec2f p2 = { p.x, p.y };
            Vec2f q2 = { q.x, q.y };

            float p_sign = geometry::orient_2d(a2, b2, p2);
            float q_sign = geometry::orient_2d(a2, b2, q2);
            bool p_in = p_sign >= 0;
            bool q_in = q_sign >= 0;

            if (p_in && q_in) {
                output[out_len] = q; out_len++;
            } else if (p_in && !q_in) {
                Vec3f? isect = line_intersection(a2, b2, p2, q2);
                if (isect) { output[out_len] = isect; out_len++; }
            } else if (!p_in && q_in) {
                Vec3f? isect = line_intersection(a2, b2, p2, q2);
                if (isect) { output[out_len] = isect; out_len++; }
                output[out_len] = q; out_len++;
            }
        }

        if (out_len < 3) {
            Vec3f[] empty = {};
            return empty;
        }

        // Swap input/output for next edge
        for (usz i = 0; i < out_len; i++) input[i] = output[i];
        input_len = out_len;
    }

    // Trim to exact size
    Vec3f[] result = mem::alloc::new_array(alloc, Vec3f, (sz)input_len);
    defer catch free(result);
    for (usz i = 0; i < input_len; i++) result[i] = input[i];
    return result;
}
```

**Step 5: Implement half_plane_clip**

```c3
fn Vec3f[]? half_plane_clip(Allocator alloc, Vec3f[] polygon, Vec3f a, Vec3f b, bool keep_a_side)
{
    Vec3f m = { (a.x + b.x) / 2, (a.y + b.y) / 2, 0 };
    Vec3f d = { b.x - a.x, b.y - a.y, 0 };
    // Perpendicular direction for clip line
    Vec3f perp = { -d.y, d.x, 0 };
    // Two far-apart points on the bisector line
    float L = 1e6f;
    Vec3f p1 = { m.x - L * perp.x, m.y - L * perp.y, 0 };
    Vec3f p2 = { m.x + L * perp.x, m.y + L * perp.y, 0 };

    // dot(p - m, d) determines side: < 0 = a's side, > 0 = b's side
    // For keep_a_side: inside when dot <= 0
    // Map half-plane to S-H: clip edge is the bisector line (p1→p2)
    // "Inside" = keep_a_side means (dot(p - m, d) <= 0) XOR (!keep_a_side)

    // For simplicity, build a 3-vertex clip triangle large enough to enclose
    // the polygon, then clip. Actually S-H with a single infinite edge:
    // Use the bisector line as clip edge.

    Vec3f[2] clip = { p1, p2 };
    // orient_2d on the bisector line: test which side of p1→p2 the point is on.
    // keep_a_side determines the sign requirement.

    float a_dot = (a.x - m.x) * d.x + (a.y - m.y) * d.y;
    // short-circuit: instead of converting to S-H edge orientation,
    // use the dot product directly to test inside/outside.

    usz cap = polygon.len * 2;
    Vec3f[] input = mem::alloc::new_array(alloc, Vec3f, (sz)cap);
    defer free(input);
    Vec3f[] output = mem::alloc::new_array(alloc, Vec3f, (sz)cap);
    defer free(output);
    for (usz i = 0; i < polygon.len; i++) input[i] = polygon[i];
    usz input_len = polygon.len;

    usz out_len = 0;
    for (usz i = 0; i < input_len; i++) {
        Vec3f p = input[i];
        Vec3f q = input[(i + 1) % input_len];
        float p_dot = (p.x - m.x) * d.x + (p.y - m.y) * d.y;
        float q_dot = (q.x - m.x) * d.x + (q.y - m.y) * d.y;
        bool p_in = keep_a_side ? (p_dot * a_dot >= 0) : (p_dot * a_dot <= 0);
        bool q_in = keep_a_side ? (q_dot * a_dot >= 0) : (q_dot * a_dot <= 0);

        if (p_in && q_in) {
            output[out_len] = q; out_len++;
        } else if (p_in && !q_in) {
            // intersection of line p→q with the bisector plane
            float t = p_dot / (p_dot - q_dot);
            output[out_len] = { p.x + t * (q.x - p.x), p.y + t * (q.y - p.y), 0 };
            out_len++;
        } else if (!p_in && q_in) {
            float t = p_dot / (p_dot - q_dot);
            output[out_len] = { p.x + t * (q.x - p.x), p.y + t * (q.y - p.y), 0 };
            out_len++;
            output[out_len] = q; out_len++;
        }
    }

    if (out_len < 3) {
        Vec3f[] empty = {};
        return empty;
    }

    Vec3f[] result = mem::alloc::new_array(alloc, Vec3f, (sz)out_len);
    defer catch free(result);
    for (usz i = 0; i < out_len; i++) result[i] = output[i];
    return result;
}
```

**Step 6: Implement is_polygon_inside helper**

```c3
// Test if a point is inside/on a convex CCW polygon.
fn bool point_in_convex_polygon(Vec3f p, Vec3f[] polygon)
{
    for (usz e = 0; e < polygon.len; e++) {
        Vec3f a = polygon[e];
        Vec3f b = polygon[(e + 1) % polygon.len];
        if (geometry::orient_2d({a.x,a.y}, {b.x,b.y}, {p.x,p.y}) < 0) return false;
    }
    return true;
}

// Validate polygon is convex, CCW, non-degenerate.
fn bool is_convex_polygon(Vec3f[] polygon)
{
    if (polygon.len < 3) return false;
    bool has_strict = false;
    for (usz i = 0; i < polygon.len; i++) {
        Vec3f p = polygon[i];
        Vec3f q = polygon[(i + 1) % polygon.len];
        Vec3f r = polygon[(i + 2) % polygon.len];
        float sign = geometry::orient_2d({p.x,p.y}, {q.x,q.y}, {r.x,r.y});
        if (sign < 0) return false;
        if (sign > 0) has_strict = true;
    }
    return has_strict;
}
```

**Step 7: Build & verify compiles**

```bash
c3c build debug
```
Expected: compiles. Test should fail at runtime.

**Step 8: Commit**

```bash
git add src/voronoi/bounded.c3 test/test_voronoi_bounded.c3
git commit -m "voronoi: implement S-H, half-plane clip, and polygon helpers"
```

---

### Task 5: Implement in_polygon (N=0, N=1, N=2)

**Objective:** Handle degenerate and small-site-count cases.

**Files:**
- Modify: `src/voronoi/bounded.c3`

**Step 1: Implement N=0, N=1, N=2 branches in in_polygon**

```c3
fn VoronoiDiagram? in_polygon(Allocator alloc, Vec3f[] sites, Vec3f[] polygon)
{
    // Validate polygon
    if (!is_convex_polygon(polygon)) return cg::NON_CONVEX_BOUNDING_POLYGON~;
    if (sites.len == 0) return cg::EMPTY_INPUT~;

    // Prefilter: keep only sites inside/on polygon
    usz inside_count = 0;
    for (usz i = 0; i < sites.len; i++) {
        if (point_in_convex_polygon(sites[i], polygon)) inside_count++;
    }

    if (inside_count == 0) {
        // Empty result
        VoronoiDiagram result;
        result.mesh = {};
        result.sites = {};
        return result;
    }

    // N = 1: cell = polygon
    if (inside_count == 1) {
        // Find the inside site
        Vec3f single_site;
        for (usz i = 0; i < sites.len; i++) {
            if (point_in_convex_polygon(sites[i], polygon)) {
                single_site = sites[i];
                break;
            }
        }

        Vec3f[] positions = mem::alloc::new_array(alloc, Vec3f, (sz)polygon.len);
        defer catch free(positions);
        for (usz i = 0; i < polygon.len; i++) positions[i] = polygon[i];

        uint[] face_indices = mem::alloc::new_array(alloc, uint, (sz)polygon.len);
        defer catch free(face_indices);
        for (usz i = 0; i < polygon.len; i++) face_indices[i] = (uint)i;

        uint[2] face_offsets_arr = { 0, (uint)polygon.len };
        // Need heap-alloc face_offsets for from_polygons
        uint[] face_offsets = mem::alloc::new_array(alloc, uint, 2);
        defer catch free(face_offsets);
        face_offsets[0] = 0;
        face_offsets[1] = (uint)polygon.len;

        HalfEdgeMesh mesh = half_edge::from_polygons(alloc, positions, face_offsets, face_indices)!;

        Vec3f[] result_sites = mem::alloc::new_array(alloc, Vec3f, 1);
        defer catch { mesh.destroy(); free(result_sites); };
        result_sites[0] = single_site;

        VoronoiDiagram result;
        result.mesh = mesh;
        result.sites = result_sites;
        return result;
    }

    // N = 2: bisect polygon
    if (inside_count == 2) {
        // Collect the two inside sites
        Vec3f[2] two_sites;
        usz found = 0;
        for (usz i = 0; i < sites.len; i++) {
            if (point_in_convex_polygon(sites[i], polygon)) {
                two_sites[found] = sites[i];
                found++;
                if (found == 2) break;
            }
        }

        Vec3f[] poly0 = half_plane_clip(alloc, polygon, two_sites[0], two_sites[1], true)!;
        defer free(poly0);
        Vec3f[] poly1 = half_plane_clip(alloc, polygon, two_sites[0], two_sites[1], false)!;
        defer free(one1);

        // Count surviving cells
        usz n_cells = 0;
        if (poly0.len >= 3) n_cells++;
        if (poly1.len >= 3) n_cells++;

        // Build + allocate sites for surviving cells
        // (Construct from_polygons with accumulated positions and face data)
        // ... (detailed implementation — accumulate surviving polygons, call from_polygons)
        // Return VoronoiDiagram
    }

    // N >= 3 — helper-point Delaunay path (Task 6)
}
```

**Step 2: Build & test N=1, N=2 test cases**

```bash
c3c build debug && c3c test
```

**Step 3: Commit**

```bash
git add src/voronoi/bounded.c3
git commit -m "voronoi: implement in_polygon N=0/N=1/N=2 branches"
```

---

### Task 6: Implement in_polygon N≥3 (helper-point Delaunay)

**Objective:** The main bounded Voronoi pipeline for 3+ sites.

**Files:**
- Modify: `src/voronoi/bounded.c3`

**Step 1: Implement N≥3 path**

```c3
    // N >= 3 path
    // Step 1: collect inside_sites
    Vec3f[] inside_sites = mem::alloc::new_array(alloc, Vec3f, (sz)inside_count);
    defer free(inside_sites);
    {
        usz j = 0;
        for (usz i = 0; i < sites.len; i++) {
            if (point_in_convex_polygon(sites[i], polygon)) {
                inside_sites[j] = sites[i];
                j++;
            }
        }
    }

    // Step 2: compute helper points from polygon bbox
    float min_x = polygon[0].x, max_x = min_x;
    float min_y = polygon[0].y, max_y = min_y;
    for (usz i = 1; i < polygon.len; i++) {
        if (polygon[i].x < min_x) min_x = polygon[i].x;
        if (polygon[i].x > max_x) max_x = polygon[i].x;
        if (polygon[i].y < min_y) min_y = polygon[i].y;
        if (polygon[i].y > max_y) max_y = polygon[i].y;
    }
    float w = max_x - min_x;
    float h = max_y - min_y;
    Vec3f[4] helpers = {
        { min_x - w,     min_y + h/2, 0 },
        { max_x + w,     min_y + h/2, 0 },
        { min_x + w/2,   min_y - h,   0 },
        { min_x + w/2,   max_y + h,   0 },
    };

    // Step 3: build all_points = inside_sites + helpers
    usz total = inside_count + 4;
    Vec3f[] all_points = mem::alloc::new_array(alloc, Vec3f, (sz)total);
    defer free(all_points);
    for (usz i = 0; i < inside_count; i++) all_points[i] = inside_sites[i];
    for (usz i = 0; i < 4; i++) all_points[inside_count + i] = helpers[i];

    // Step 4: Delaunay → Voronoi
    HalfEdgeMesh delaunay_mesh = delaunay::delaunay_2d(alloc, all_points)!;
    defer delaunay_mesh.destroy();
    VoronoiDiagram unbounded = voronoi::from_delaunay(alloc, &delaunay_mesh)!;
    defer unbounded.destroy();

    // Step 5: Match faces to inside_sites
    // For each inside_site, find its index in unbounded.sites (position equality)
    // That index is the face index in unbounded.mesh
    // Clip each matched face polygon, accumulate survivors

    // (Detailed implementation: linear scan matching, face walk, S-H clip,
    //  accumulate positions/indices, dedup, from_polygons, allocate sites)

    // ... implementation continues
}
```

**Step 2: Build & test — 3+ site cases should pass**

```bash
c3c build debug && c3c test
```

**Step 3: Commit**

```bash
git add src/voronoi/bounded.c3
git commit -m "voronoi: implement in_polygon N≥3 helper-point Delaunay path"
```

---

### Task 7: Implement in_box and write all tests

**Objective:** in_box wrapper and comprehensive test suite.

**Files:**
- Modify: `src/voronoi/bounded.c3`
- Modify: `test/test_voronoi_bounded.c3`

**Step 1: Implement in_box**

```c3
fn VoronoiDiagram? in_box(Allocator alloc, Vec3f[] sites, Aabb bbox)
{
    Vec3f[4] polygon = {
        { bbox.min.x, bbox.min.y, 0 },
        { bbox.max.x, bbox.min.y, 0 },
        { bbox.max.x, bbox.max.y, 0 },
        { bbox.min.x, bbox.max.y, 0 },
    };
    return in_polygon(alloc, sites, polygon[..]);
}
```

**Step 2: Write all tests per spec** (tests 1-18 from spec table)
- 4 sites in square, N=1 inside/outside, N=2 cases, near corner, diamond, triangle, outside sites, large box, non-convex/cw/<3/collinear fault tests, empty sites, in_box basic

**Step 3: Build & test**

```bash
c3c build debug && c3c test
```
Expected: ~195 tests, 0 failures.

**Step 4: Commit**

```bash
git add src/voronoi/bounded.c3 test/test_voronoi_bounded.c3
git commit -m "voronoi: implement in_box and bounded voronoi test suite"
```

---

### Task 8: Final verification and milestone completion

**Objective:** Clean build, all tests pass, mark M11 complete.

**Step 1: Clean build + release**

```bash
c3c clean && c3c build release
```

**Step 2: Full test run**

```bash
c3c test
```
Expected: all pass.

**Step 3: Mark M11 complete in AGENTS.md**

Change `| M11  | Bounded Voronoi ... | Not started |` to `✅ Complete`.

**Step 4: Commit**

```bash
git add AGENTS.md
git commit -m "docs: mark M11 bounded Voronoi as complete"
```
