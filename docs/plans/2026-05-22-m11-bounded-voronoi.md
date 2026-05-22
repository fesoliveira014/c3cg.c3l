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

In `src/dual/dual.c3`, remove the boundary check block:

```c3
    for (usz i = 0; i < mesh.half_edges.len; i++) {
        if (mesh.half_edges[i].twin == cg::INVALID_HE) return cg::DUAL_REQUIRES_CLOSED_MESH~;
    }
```

Replace with:

```c3
    // Boundary edges are allowed. Vertices with incomplete face rings
    // (walk hits INVALID_HE before returning to start) produce no face
    // in the dual output: face_offsets advances by 0 for that vertex.
    // vertex_one_ring_faces already handles this — it returns early
    // when INVALID_HE is hit. No additional check needed.
```

The `vertex_one_ring_faces` call in the loop already handles boundary vertices correctly (returns early when `INVALID_HE` is hit). For vertices with incomplete rings, the returned count will be less than the number of incident faces. The face_offsets will still advance by that count, producing a valid but possibly degenerate ring. `from_polygons` will reject rings with < 3 vertices, so the implementer should add a check: if `n < 3`, skip the vertex entirely (set `face_offsets[v+1] = face_offsets[v]`).

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
Expected: all existing tests still pass. The boundary test in `test_voronoi.c3` should now succeed.

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

Replace with:

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
    // Single triangle: all 3 vertices are boundary, dual skips them → 0 faces.
    assert(diagram.mesh.faces.len == 0);
    assert(diagram.sites.len == 0);
}
```

**Step 2: Build & test**

```bash
c3c build debug && c3c test
```
Expected: all tests pass.

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

In the root `module cg;` fault block, add:

```c3
faultdef NON_CONVEX_BOUNDING_POLYGON;
```

**Step 2: Add voronoi API**

In the `module cg::voronoi;` section, after `to_delaunay`, add:

```c3
fn VoronoiDiagram? in_polygon(Allocator alloc, Vec3f[] sites, Vec3f[] polygon);
fn VoronoiDiagram? in_box(Allocator alloc, Vec3f[] sites, Aabb bbox);
```

**Step 3: Replace bounded.c3 stub with full declarations**

```c3
module cg::voronoi;
import cg;
import cg::geometry;
import cg::half_edge;
import cg::delaunay;
import cg::dual;

// Private helpers — not in umbrella
fn Vec3f[]? sutherland_hodgman(Allocator alloc, Vec3f[] polygon, Vec3f[] clip_polygon);
fn Vec3f[]? half_plane_clip(Allocator alloc, Vec3f[] polygon, Vec3f a, Vec3f b, bool keep_a_side);
fn Vec3f? line_intersection(Vec2f a, Vec2f b, Vec2f p, Vec2f q);
fn bool point_in_convex_polygon(Vec3f p, Vec3f[] polygon);
fn bool is_convex_polygon(Vec3f[] polygon);
```

**Step 4: Build**

```bash
c3c build debug
```
Expected: compiles (functions not yet defined but signatures match).

**Step 5: Commit**

```bash
git add src/c3cg.c3i src/voronoi/bounded.c3
git commit -m "voronoi: add NON_CONVEX_BOUNDING_POLYGON fault and bounded API declarations"
```

---

### Task 4: Implement Sutherland-Hodgman and helpers

**Objective:** Core polygon clipping and geometry utilities.

**Files:**
- Modify: `src/voronoi/bounded.c3`

**Step 1: Write failing test (TDD)**

In `test/test_voronoi_bounded.c3`:

```c3
module test;
import cg;
import cg::voronoi;

fn void test_in_polygon_four_sites_square() @test
{
    Vec3f[4] sites = { {0.25,0.25,0}, {0.75,0.25,0}, {0.75,0.75,0}, {0.25,0.75,0} };
    Vec3f[4] poly  = { {0,0,0}, {1,0,0}, {1,1,0}, {0,1,0} };

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

**Step 3: Implement line_intersection**

```c3
fn Vec3f? line_intersection(Vec2f a, Vec2f b, Vec2f p, Vec2f q)
{
    float d1x = b.x - a.x;
    float d1y = b.y - a.y;
    float d2x = q.x - p.x;
    float d2y = q.y - p.y;
    float det = d1x * d2y - d1y * d2x;
    if (math::abs(det) < geometry::GEOMETRY_EPSILON) return {};

    float t = ((p.x - a.x) * d2y - (p.y - a.y) * d2x) / det;
    return { a.x + t * d1x, a.y + t * d1y, 0 };
}
```

Note: `import std::math;` may be needed for `math::abs`. If not available, use `det > -GEOMETRY_EPSILON && det < GEOMETRY_EPSILON`.

**Step 4: Implement sutherland_hodgman**

Uses a growth strategy: starts with `input_len * 2` capacity, reallocates if output exceeds capacity across multiple clip edges. For convex clip polygons of typical size, initial capacity suffices.

```c3
fn Vec3f[]? sutherland_hodgman(Allocator alloc, Vec3f[] polygon, Vec3f[] clip_polygon)
{
    usz cap = polygon.len * 4;  // generous — multiple clip edges can grow output
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
            bool p_in = p_sign >= geometry::PREDICATE_ZERO;
            bool q_in = q_sign >= geometry::PREDICATE_ZERO;

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

        for (usz i = 0; i < out_len; i++) input[i] = output[i];
        input_len = out_len;
    }

    Vec3f[] result = mem::alloc::new_array(alloc, Vec3f, (sz)input_len);
    defer catch free(result);
    for (usz i = 0; i < input_len; i++) result[i] = input[i];
    return result;
}
```

**Step 5: Implement half_plane_clip**

Uses dot product on perpendicular bisector, not `orient_2d`.

```c3
fn Vec3f[]? half_plane_clip(Allocator alloc, Vec3f[] polygon, Vec3f a, Vec3f b, bool keep_a_side)
{
    Vec3f m = { (a.x + b.x) / 2, (a.y + b.y) / 2, 0 };
    Vec3f d = { b.x - a.x, b.y - a.y, 0 };
    float a_dot = (a.x - m.x) * d.x + (a.y - m.y) * d.y;

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

**Step 6: Implement validation helpers**

```c3
fn bool point_in_convex_polygon(Vec3f p, Vec3f[] polygon)
{
    for (usz e = 0; e < polygon.len; e++) {
        Vec3f a = polygon[e];
        Vec3f b = polygon[(e + 1) % polygon.len];
        if (geometry::orient_2d({a.x,a.y}, {b.x,b.y}, {p.x,p.y}) < 0) return false;
    }
    return true;
}

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
    // Reject degenerate/collinear: need at least one positive turn
    return has_strict;
}
```

**Step 7: Build — test still fails (in_polygon not done)**

```bash
c3c build debug
```
Expected: compiles.

**Step 8: Commit**

```bash
git add src/voronoi/bounded.c3 test/test_voronoi_bounded.c3
git commit -m "voronoi: implement S-H, half-plane clip, and polygon helpers"
```

---

### Task 5: Implement in_polygon full pipeline

**Objective:** Complete in_polygon — prefilter, N=0/1/2 branches, N≥3 helper-point path.

**Files:**
- Modify: `src/voronoi/bounded.c3`

**Step 1: Implement in_polygon (complete)**

```c3
fn VoronoiDiagram? in_polygon(Allocator alloc, Vec3f[] sites, Vec3f[] polygon)
{
    // Validate
    if (!is_convex_polygon(polygon)) return cg::NON_CONVEX_BOUNDING_POLYGON~;
    if (sites.len == 0) return cg::EMPTY_INPUT~;

    // Prefilter: collect sites inside/on polygon
    Vec3f[] inside_sites = mem::alloc::new_array(alloc, Vec3f, (sz)sites.len);
    defer free(inside_sites);
    usz inside_count = 0;
    for (usz i = 0; i < sites.len; i++) {
        if (point_in_convex_polygon(sites[i], polygon)) {
            inside_sites[inside_count] = sites[i];
            inside_count++;
        }
    }

    // N = 0: empty
    if (inside_count == 0) {
        VoronoiDiagram result;
        result.mesh = {};
        result.sites = {};
        return result;
    }

    // N = 1: cell = polygon
    if (inside_count == 1) {
        Vec3f[] positions = mem::alloc::new_array(alloc, Vec3f, (sz)polygon.len);
        defer free(positions);
        for (usz i = 0; i < polygon.len; i++) positions[i] = polygon[i];

        uint[] face_indices = mem::alloc::new_array(alloc, uint, (sz)polygon.len);
        defer free(face_indices);
        for (usz i = 0; i < polygon.len; i++) face_indices[i] = (uint)i;

        uint[] face_offsets = mem::alloc::new_array(alloc, uint, 2);
        defer free(face_offsets);
        face_offsets[0] = 0;
        face_offsets[1] = (uint)polygon.len;

        HalfEdgeMesh mesh = half_edge::from_polygons(alloc, positions, face_offsets, face_indices)!;

        Vec3f[] result_sites = mem::alloc::new_array(alloc, Vec3f, 1);
        defer catch { mesh.destroy(); free(result_sites); };
        result_sites[0] = inside_sites[0];

        VoronoiDiagram result;
        result.mesh = mesh;
        result.sites = result_sites;
        return result;
    }

    // N = 2: bisect polygon
    if (inside_count == 2) {
        Vec3f[] poly0 = half_plane_clip(alloc, polygon, inside_sites[0], inside_sites[1], true)!;
        defer free(poly0);
        Vec3f[] poly1 = half_plane_clip(alloc, polygon, inside_sites[0], inside_sites[1], false)!;
        defer free(poly1);

        usz n_cells = 0;
        if (poly0.len >= 3) n_cells++;
        if (poly1.len >= 3) n_cells++;

        if (n_cells == 0) {
            VoronoiDiagram result;
            result.mesh = {};
            result.sites = {};
            return result;
        }

        // Accumulate surviving cells into from_polygons format
        usz total_verts = 0;
        if (poly0.len >= 3) total_verts += poly0.len;
        if (poly1.len >= 3) total_verts += poly1.len;

        Vec3f[] positions = mem::alloc::new_array(alloc, Vec3f, (sz)total_verts);
        defer free(positions);
        uint[] face_indices = mem::alloc::new_array(alloc, uint, (sz)total_verts);
        defer free(face_indices);
        uint[] face_offsets = mem::alloc::new_array(alloc, uint, (sz)(n_cells + 1));
        defer free(face_offsets);

        usz pos_idx = 0;
        usz cell = 0;
        face_offsets[0] = 0;
        if (poly0.len >= 3) {
            for (usz i = 0; i < poly0.len; i++) {
                positions[pos_idx + i] = poly0[i];
                face_indices[pos_idx + i] = (uint)(pos_idx + i);
            }
            pos_idx += poly0.len;
            cell++;
            face_offsets[cell] = (uint)pos_idx;
        }
        if (poly1.len >= 3) {
            for (usz i = 0; i < poly1.len; i++) {
                positions[pos_idx + i] = poly1[i];
                face_indices[pos_idx + i] = (uint)(pos_idx + i);
            }
            pos_idx += poly1.len;
            cell++;
            face_offsets[cell] = (uint)pos_idx;
        }

        HalfEdgeMesh mesh = half_edge::from_polygons(alloc, positions, face_offsets, face_indices)!;

        Vec3f[] result_sites = mem::alloc::new_array(alloc, Vec3f, (sz)n_cells);
        defer catch { mesh.destroy(); free(result_sites); };
        usz site_idx = 0;
        if (poly0.len >= 3) { result_sites[site_idx] = inside_sites[0]; site_idx++; }
        if (poly1.len >= 3) { result_sites[site_idx] = inside_sites[1]; }

        VoronoiDiagram result;
        result.mesh = mesh;
        result.sites = result_sites;
        return result;
    }

    // ── N ≥ 3: helper-point Delaunay path ──

    // Compute polygon bbox
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

    // Build all_points = inside_sites + helpers
    usz total = inside_count + 4;
    Vec3f[] all_points = mem::alloc::new_array(alloc, Vec3f, (sz)total);
    defer free(all_points);
    for (usz i = 0; i < inside_count; i++) all_points[i] = inside_sites[i];
    for (usz i = 0; i < 4; i++) all_points[inside_count + i] = helpers[i];

    // Delaunay → unbounded Voronoi
    HalfEdgeMesh delaunay_mesh = delaunay::delaunay_2d(alloc, all_points)!;
    defer delaunay_mesh.destroy();
    VoronoiDiagram unbounded = voronoi::from_delaunay(alloc, &delaunay_mesh)!;
    defer unbounded.destroy();

    // Match inside_sites to Voronoi faces
    // For each inside_site, linear-scan unbounded.sites for position match
    // (positions are unique — no duplicates between original and helpers)
    FaceIndex[] face_map = mem::alloc::new_array(alloc, FaceIndex, (sz)inside_count);
    defer free(face_map);
    for (usz i = 0; i < inside_count; i++) {
        face_map[i] = cg::INVALID_FACE;
        for (usz s = 0; s < unbounded.sites.len; s++) {
            float dx = unbounded.sites[s].x - inside_sites[i].x;
            float dy = unbounded.sites[s].y - inside_sites[i].y;
            if (dx * dx + dy * dy < 1e-24) {
                face_map[i] = (FaceIndex)(int)s;
                break;
            }
        }
    }

    // First pass: count surviving cells and total vertices
    // Allocate scratch for face_vertices (worst-case: all vertices)
    VertexIndex[] scratch = mem::alloc::new_array(alloc, VertexIndex, (sz)unbounded.mesh.vertices.len);
    defer free(scratch);

    usz n_surviving = 0;
    usz total_verts = 0;
    for (usz i = 0; i < inside_count; i++) {
        if (face_map[i] == cg::INVALID_FACE) continue;
        int n = unbounded.mesh.face_vertices(face_map[i], scratch)!;
        if (n < 3) continue;

        // Collect face polygon
        // Clip against bounding polygon
        Vec3f[] face_poly = mem::alloc::new_array(alloc, Vec3f, (sz)n);
        for (int j = 0; j < n; j++) {
            face_poly[j] = unbounded.mesh.positions[scratch[j]];
        }
        Vec3f[] clipped = sutherland_hodgman(alloc, face_poly, polygon)!;
        free(face_poly);
        if (clipped.len >= 3) {
            n_surviving++;
            total_verts += clipped.len;
        }
        free(clipped);
        // Note: per-iteration alloc/free is ok — cells are few, vertices bounded
    }

    if (n_surviving == 0) {
        VoronoiDiagram result;
        result.mesh = {};
        result.sites = {};
        return result;
    }

    // Second pass: build mesh from survivors
    Vec3f[] positions = mem::alloc::new_array(alloc, Vec3f, (sz)total_verts);
    defer free(positions);
    uint[] face_indices = mem::alloc::new_array(alloc, uint, (sz)total_verts);
    defer free(face_indices);
    uint[] face_offsets = mem::alloc::new_array(alloc, uint, (sz)(n_surviving + 1));
    defer free(face_offsets);
    Vec3f[] result_sites = mem::alloc::new_array(alloc, Vec3f, (sz)n_surviving);
    defer catch free(result_sites);

    usz pos_idx = 0;
    usz cell = 0;
    face_offsets[0] = 0;
    for (usz i = 0; i < inside_count; i++) {
        if (face_map[i] == cg::INVALID_FACE) continue;
        int n = unbounded.mesh.face_vertices(face_map[i], scratch)!;
        if (n < 3) continue;

        Vec3f[] face_poly = mem::alloc::new_array(alloc, Vec3f, (sz)n);
        for (int j = 0; j < n; j++) {
            face_poly[j] = unbounded.mesh.positions[scratch[j]];
        }
        Vec3f[] clipped = sutherland_hodgman(alloc, face_poly, polygon)!;
        free(face_poly);
        if (clipped.len >= 3) {
            for (usz j = 0; j < clipped.len; j++) positions[pos_idx + j] = clipped[j];
            for (usz j = 0; j < clipped.len; j++) face_indices[pos_idx + j] = (uint)(pos_idx + j);
            pos_idx += clipped.len;
            result_sites[cell] = inside_sites[i];
            cell++;
            face_offsets[cell] = (uint)pos_idx;
        }
        free(clipped);
    }

    HalfEdgeMesh mesh = half_edge::from_polygons(alloc, positions, face_offsets, face_indices)!;

    // Transfer result_sites ownership (allocate fresh — positions/face data already freed)
    Vec3f[] final_sites = mem::alloc::new_array(alloc, Vec3f, (sz)n_surviving);
    defer catch { mesh.destroy(); free(final_sites); };
    for (usz i = 0; i < n_surviving; i++) final_sites[i] = result_sites[i];
    free(result_sites);

    VoronoiDiagram result;
    result.mesh = mesh;
    result.sites = final_sites;
    return result;
}
```

**Step 2: Build & test**

```bash
c3c build debug && c3c test
```
Expected: 4-site test passes (~177 tests, 0 failures).

**Step 3: Commit**

```bash
git add src/voronoi/bounded.c3 test/test_voronoi_bounded.c3
git commit -m "voronoi: implement in_polygon full pipeline with all N branches"
```

---

### Task 6: Implement in_box and write all tests

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

**Step 2: Write remaining tests per spec**

Keep existing `test_in_polygon_four_sites_square`. Add:

```c3
// --- N=1 tests ---
fn void test_in_polygon_n1_inside() @test
{
    Vec3f[1] sites = { {0.5, 0.5, 0} };
    Vec3f[4] poly  = { {0,0,0}, {1,0,0}, {1,1,0}, {0,1,0} };
    VoronoiDiagram d = voronoi::in_polygon(mem, sites[..], poly[..])!!;
    defer d.destroy();
    assert(d.mesh.faces.len == 1);
    assert(d.sites.len == 1);
    d.mesh.validate()!!;
}

fn void test_in_polygon_n1_outside() @test
{
    Vec3f[1] sites = { {-1, 0.5, 0} };
    Vec3f[4] poly  = { {0,0,0}, {1,0,0}, {1,1,0}, {0,1,0} };
    VoronoiDiagram d = voronoi::in_polygon(mem, sites[..], poly[..])!!;
    defer d.destroy();
    assert(d.mesh.faces.len == 0);
    assert(d.sites.len == 0);
}

// --- N=2 tests ---
fn void test_in_polygon_n2_both_inside() @test
{
    Vec3f[2] sites = { {0.25,0.5,0}, {0.75,0.5,0} };
    Vec3f[4] poly  = { {0,0,0}, {1,0,0}, {1,1,0}, {0,1,0} };
    VoronoiDiagram d = voronoi::in_polygon(mem, sites[..], poly[..])!!;
    defer d.destroy();
    assert(d.mesh.faces.len == 2);
    assert(d.sites.len == 2);
    d.mesh.validate()!!;
}

fn void test_in_polygon_n2_one_outside() @test
{
    Vec3f[2] sites = { {0.5,0.5,0}, {-1,0.5,0} };
    Vec3f[4] poly  = { {0,0,0}, {1,0,0}, {1,1,0}, {0,1,0} };
    VoronoiDiagram d = voronoi::in_polygon(mem, sites[..], poly[..])!!;
    defer d.destroy();
    assert(d.mesh.faces.len == 1);
    assert(d.sites.len == 1);
}

fn void test_in_polygon_n2_both_outside() @test
{
    Vec3f[2] sites = { {-1,0.5,0}, {-2,0.5,0} };
    Vec3f[4] poly  = { {0,0,0}, {1,0,0}, {1,1,0}, {0,1,0} };
    VoronoiDiagram d = voronoi::in_polygon(mem, sites[..], poly[..])!!;
    defer d.destroy();
    assert(d.mesh.faces.len == 0);
}

// --- Fault tests ---
fn void test_in_polygon_non_convex() @test
{
    Vec3f[4] sites = { {0.25,0.25,0}, {0.75,0.25,0}, {0.75,0.75,0}, {0.25,0.75,0} };
    Vec3f[5] poly  = { {0,0,0}, {1,0,0}, {1,1,0}, {0.5,0.5,0}, {0,1,0} };
    if (catch err = voronoi::in_polygon(mem, sites[..], poly[..])) {
        assert(err == cg::NON_CONVEX_BOUNDING_POLYGON);
        return;
    }
    unreachable();
}

fn void test_in_polygon_clockwise_faults() @test
{
    Vec3f[1] sites = { {0.5,0.5,0} };
    Vec3f[4] poly  = { {0,0,0}, {0,1,0}, {1,1,0}, {1,0,0} };  // CW
    if (catch err = voronoi::in_polygon(mem, sites[..], poly[..])) {
        assert(err == cg::NON_CONVEX_BOUNDING_POLYGON);
        return;
    }
    unreachable();
}

fn void test_in_polygon_few_verts_faults() @test
{
    Vec3f[1] sites = { {0.5,0.5,0} };
    Vec3f[2] poly  = { {0,0,0}, {1,1,0} };
    if (catch err = voronoi::in_polygon(mem, sites[..], poly[..])) {
        assert(err == cg::NON_CONVEX_BOUNDING_POLYGON);
        return;
    }
    unreachable();
}

fn void test_in_polygon_collinear_faults() @test
{
    Vec3f[1] sites = { {0.5,0.5,0} };
    Vec3f[4] poly  = { {0,0,0}, {1,0,0}, {2,0,0}, {0,0,0} };  // collinear
    if (catch err = voronoi::in_polygon(mem, sites[..], poly[..])) {
        assert(err == cg::NON_CONVEX_BOUNDING_POLYGON);
        return;
    }
    unreachable();
}

fn void test_in_polygon_empty_sites() @test
{
    Vec3f[] sites = {};
    Vec3f[4] poly  = { {0,0,0}, {1,0,0}, {1,1,0}, {0,1,0} };
    if (catch err = voronoi::in_polygon(mem, sites, poly[..])) {
        assert(err == cg::EMPTY_INPUT);
        return;
    }
    unreachable();
}

// --- Triangle polygon ---
fn void test_in_polygon_triangle() @test
{
    Vec3f[3] sites = { {0.5,0.25,0}, {0.25,0.75,0}, {0.75,0.75,0} };
    Vec3f[3] poly  = { {0,0,0}, {2,0,0}, {1,2,0} };
    VoronoiDiagram d = voronoi::in_polygon(mem, sites[..], poly[..])!!;
    defer d.destroy();
    assert(d.mesh.faces.len == 3);
    d.mesh.validate()!!;
}

// --- Outside sites ---
fn void test_in_polygon_all_outside() @test
{
    Vec3f[4] sites = { {-1,-1,0}, {-1,2,0}, {2,-1,0}, {2,2,0} };
    Vec3f[4] poly  = { {0,0,0}, {1,0,0}, {1,1,0}, {0,1,0} };
    VoronoiDiagram d = voronoi::in_polygon(mem, sites[..], poly[..])!!;
    defer d.destroy();
    assert(d.mesh.faces.len == 0);
}

// --- in_box ---
fn void test_in_box_basic() @test
{
    Vec3f[2] sites = { {0.5,0.5,0}, {1.5,1.5,0} };
    Aabb bbox = { {0,0,0}, {2,2,0} };
    VoronoiDiagram d = voronoi::in_box(mem, sites[..], bbox)!!;
    defer d.destroy();
    assert(d.mesh.faces.len == 2);
    d.mesh.validate()!!;
}
```

**Step 3: Build & test**

```bash
c3c build debug && c3c test
```
Expected: all tests pass (~190 tests, 0 failures).

**Step 4: Commit**

```bash
git add src/voronoi/bounded.c3 test/test_voronoi_bounded.c3
git commit -m "voronoi: implement in_box and complete bounded voronoi test suite"
```

---

### Task 7: Final verification and milestone completion

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
