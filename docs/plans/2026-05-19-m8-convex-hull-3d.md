# M8 — Convex Hull 3D Implementation Plan

> **For Hermes:** Use subagent-driven-development skill to implement this plan task-by-task.

**Goal:** Implement `cg::hull::hull_3d` — incremental 3D convex hull, taking `Vec3f[]` and returning a closed triangular `HalfEdgeMesh` with no boundary.

**Architecture:** Face-list representation during incremental construction, then `from_triangles` at the end to build the half-edge mesh. New `orient_3d` predicate in `cg::geometry`. Directed edge map (`HashMap{int[<2>], int}`) for horizon detection.

**Spec:** `docs/specs/2026-05-19-m8-convex-hull-3d-design.md`

**Tech Stack:** C3 0.8.0, c3c, existing `cg`, `cg::geometry`, `cg::half_edge`, `std::collections::map` modules.

**TDD & green commits:** Every task follows RED→GREEN→commit. No RED commits — stub-replace pattern keeps `c3c build debug && c3c test` green at every boundary.

---

### Task 1: Add orient_3d tests (RED)

**Objective:** Write failing tests for `orient_3d` before the predicate exists.

**Files:**
- Modify: `test/test_geometry.c3`

**Step 1: Append tests**

```c3
fn void test_orient_3d_positive_for_ccw_orientation() @test
{
    assert(geometry::orient_3d({1,0,0}, {0,1,0}, {0,0,1}, {0,0,0}) == geometry::PREDICATE_POSITIVE);
}

fn void test_orient_3d_negative_for_reversed_order() @test
{
    assert(geometry::orient_3d({0,1,0}, {1,0,0}, {0,0,1}, {0,0,0}) == geometry::PREDICATE_NEGATIVE);
}

fn void test_orient_3d_zero_for_coplanar() @test
{
    assert(geometry::orient_3d({0,0,0}, {1,0,0}, {0,1,0}, {1,1,0}) == geometry::PREDICATE_ZERO);
}

fn void test_orient_3d_respects_epsilon() @test
{
    assert(geometry::orient_3d({0,0,0}, {1,0,0}, {0,1,0}, {1,1,0.000001f}) == geometry::PREDICATE_POSITIVE);
    assert(geometry::orient_3d({0,0,0}, {1,0,0}, {0,1,0}, {1,1,0.000001f}, 0.01f) == geometry::PREDICATE_ZERO);
}
```

**Step 2: Run — expect FAIL**

```bash
c3c test
```

Expected: compilation FAIL — `geometry::orient_3d` not defined.

**Step 3: Commit**

```bash
git add test/test_geometry.c3
git commit -m "test: add orient_3d predicate tests (RED)"
```

---

### Task 2: Add orient_3d predicate + umbrella (GREEN)

**Objective:** Implement `orient_3d` in `cg::geometry` so Task 1 tests pass.

**Files:**
- Modify: `src/geometry/predicates.c3`
- Modify: `src/c3cg.c3i` (umbrella — `module cg::geometry;` section)

**Step 1: Add to predicates.c3**

After `det4`, append:

```c3
fn float orient_3d_det(Vec3f a, Vec3f b, Vec3f c, Vec3f d)
{
    Vec3f ab = vec3_sub_for_predicates(b, a);
    Vec3f ac = vec3_sub_for_predicates(c, a);
    Vec3f ad = vec3_sub_for_predicates(d, a);
    return det3(ab.x, ab.y, ab.z, ac.x, ac.y, ac.z, ad.x, ad.y, ad.z);
}

fn PredicateSign orient_3d(Vec3f a, Vec3f b, Vec3f c, Vec3f d, float epsilon = DEFAULT_PREDICATE_EPSILON)
{
    return classify_predicate(orient_3d_det(a, b, c, d), epsilon);
}
```

**Step 2: Add to umbrella**

In `src/c3cg.c3i`, `module cg::geometry;`, after `on_sphere`:

```c3
fn PredicateSign orient_3d(Vec3f a, Vec3f b, Vec3f c, Vec3f d, float epsilon = DEFAULT_PREDICATE_EPSILON);
```

**Step 3: Run — expect PASS**

```bash
c3c test
```

Expected: all 4 orient_3d tests PASS, all existing tests still PASS.

**Step 4: Commit**

```bash
git add src/geometry/predicates.c3 src/c3cg.c3i
git commit -m "geometry: add orient_3d 3D predicate"
```

---

### Task 3: Write hull_3d fault-path tests + add umbrella stub (RED → GREEN in one commit)

**Objective:** Write fault-path tests that will FAIL, then add the umbrella declaration + a stub so they immediately PASS (stub-then-replace pattern).

**Files:**
- Create: `test/test_hull_3d.c3`
- Create: `src/hull/hull_3d.c3` (stub)
- Modify: `src/c3cg.c3i` (umbrella — add `hull_3d`)

**Step 1: Add hull_3d to umbrella**

In `src/c3cg.c3i`, `module cg::hull;`:

```c3
fn HalfEdgeMesh? hull_3d(Allocator alloc, Vec3f[] positions);
```

**Step 2: Create stub**

```c3
module cg::hull;
import cg;

fn HalfEdgeMesh? hull_3d(Allocator alloc, Vec3f[] positions)
{
    if (positions.len == 0) return cg::EMPTY_INPUT~;
    return cg::DEGENERATE_INPUT~;
}
```

**Step 3: Create test file**

```c3
module test;
import cg;
import cg::hull;

fn void test_hull_3d_empty_faults() @test
{
    Vec3f[] positions = {};
    if (catch err = hull::hull_3d(mem, positions)) {
        assert(err == cg::EMPTY_INPUT);
        return;
    }
    unreachable();
}

fn void test_hull_3d_one_point_faults() @test
{
    Vec3f[1] pts = { { 0, 0, 0 } };
    if (catch err = hull::hull_3d(mem, pts[..])) {
        assert(err == cg::DEGENERATE_INPUT);
        return;
    }
    unreachable();
}

fn void test_hull_3d_two_points_faults() @test
{
    Vec3f[2] pts = { { 0, 0, 0 }, { 1, 0, 0 } };
    if (catch err = hull::hull_3d(mem, pts[..])) {
        assert(err == cg::DEGENERATE_INPUT);
        return;
    }
    unreachable();
}

fn void test_hull_3d_three_points_faults() @test
{
    Vec3f[3] pts = { { 0, 0, 0 }, { 1, 0, 0 }, { 0, 1, 0 } };
    if (catch err = hull::hull_3d(mem, pts[..])) {
        assert(err == cg::DEGENERATE_INPUT);
        return;
    }
    unreachable();
}

fn void test_hull_3d_coplanar_faults() @test
{
    Vec3f[5] pts = {
        { 0, 0, 0 },
        { 1, 0, 0 },
        { 0, 1, 0 },
        { 1, 1, 0 },
        { 0.5, 0.5, 0 },
    };
    if (catch err = hull::hull_3d(mem, pts[..])) {
        assert(err == cg::DEGENERATE_INPUT);
        return;
    }
    unreachable();
}
```

**Step 4: Run — expect PASS**

```bash
c3c test
```

Expected: all 5 fault-path tests PASS (stub returns correct faults).

**Step 5: Commit**

```bash
git add test/test_hull_3d.c3 src/hull/hull_3d.c3 src/c3cg.c3i
git commit -m "test: add hull_3d fault-path tests with stub"
```

---

### Task 4: Add happy-path tests (RED)

**Objective:** Add tests for tetrahedron, cube, interior, coplanar, duplicates, and convexity. These FAIL because the stub faults `DEGENERATE_INPUT`.

**Files:**
- Modify: `test/test_hull_3d.c3`

**Step 1: Append happy-path tests**

After the coplanar test, append:

```c3
const float M8_EPSILON = 0.001f;

fn void test_hull_3d_tetrahedron() @test
{
    Vec3f[4] pts = {
        { 0, 0, 0 },
        { 1, 0, 0 },
        { 0, 1, 0 },
        { 0, 0, 1 },
    };
    HalfEdgeMesh mesh = hull::hull_3d(mem, pts[..])!!;
    defer mesh.destroy();

    mesh.validate()!!;
    assert(mesh.faces.len == 4);
    for (usz i = 0; i < mesh.half_edges.len; i++) {
        assert(!mesh.is_boundary((HeIndex)(int)i));
    }
}

fn void test_hull_3d_cube() @test
{
    Vec3f[8] pts = {
        { 0, 0, 0 }, { 1, 0, 0 }, { 1, 1, 0 }, { 0, 1, 0 },
        { 0, 0, 1 }, { 1, 0, 1 }, { 1, 1, 1 }, { 0, 1, 1 },
    };
    HalfEdgeMesh mesh = hull::hull_3d(mem, pts[..])!!;
    defer mesh.destroy();

    mesh.validate()!!;
    assert(mesh.faces.len == 12);
    for (usz i = 0; i < mesh.half_edges.len; i++) {
        assert(!mesh.is_boundary((HeIndex)(int)i));
    }
}

fn void test_hull_3d_cube_with_interior_point() @test
{
    Vec3f[9] pts = {
        { 0, 0, 0 }, { 1, 0, 0 }, { 1, 1, 0 }, { 0, 1, 0 },
        { 0, 0, 1 }, { 1, 0, 1 }, { 1, 1, 1 }, { 0, 1, 1 },
        { 0.5, 0.5, 0.5 },
    };
    HalfEdgeMesh mesh = hull::hull_3d(mem, pts[..])!!;
    defer mesh.destroy();

    assert(mesh.faces.len == 12);
    mesh.validate()!!;
}

fn void test_hull_3d_cube_with_face_coplanar_point() @test
{
    Vec3f[9] pts = {
        { 0, 0, 0 }, { 1, 0, 0 }, { 1, 1, 0 }, { 0, 1, 0 },
        { 0, 0, 1 }, { 1, 0, 1 }, { 1, 1, 1 }, { 0, 1, 1 },
        { 0.5, 0.5, 1.0 },
    };
    HalfEdgeMesh mesh = hull::hull_3d(mem, pts[..])!!;
    defer mesh.destroy();

    assert(mesh.faces.len == 12);
    mesh.validate()!!;
}

fn void test_hull_3d_cube_with_duplicates() @test
{
    Vec3f[10] pts = {
        { 0, 0, 0 }, { 1, 0, 0 }, { 1, 1, 0 }, { 0, 1, 0 },
        { 0, 0, 1 }, { 1, 0, 1 }, { 1, 1, 1 }, { 0, 1, 1 },
        { 0, 0, 0 }, { 1, 0, 0 },
    };
    HalfEdgeMesh mesh = hull::hull_3d(mem, pts[..])!!;
    defer mesh.destroy();

    assert(mesh.faces.len == 12);
    mesh.validate()!!;
}

fn void test_hull_3d_cube_convexity() @test
{
    Vec3f[8] pts = {
        { 0, 0, 0 }, { 1, 0, 0 }, { 1, 1, 0 }, { 0, 1, 0 },
        { 0, 0, 1 }, { 1, 0, 1 }, { 1, 1, 1 }, { 0, 1, 1 },
    };
    HalfEdgeMesh mesh = hull::hull_3d(mem, pts[..])!!;
    defer mesh.destroy();

    VertexIndex[3] verts;
    for (FaceIndex f = 0; (int)f < (int)mesh.faces.len; f++) {
        mesh.face_vertices(f, verts[..])!;
        for (usz j = 0; j < pts.len; j++) {
            PredicateSign s = geometry::orient_3d(
                mesh.positions[(int)verts[0]],
                mesh.positions[(int)verts[1]],
                mesh.positions[(int)verts[2]],
                pts[j],
                M8_EPSILON,
            );
            assert(s != geometry::PREDICATE_POSITIVE);
        }
    }
}
```

**Step 2: Run — expect FAIL**

```bash
c3c test
```

Expected: 7 happy-path tests FAIL (stub returns `DEGENERATE_INPUT`).

**Step 3: Commit**

```bash
git add test/test_hull_3d.c3
git commit -m "test: add hull_3d happy-path tests (RED)"
```

---

### Task 5: Implement hull_3d algorithm (GREEN)

**Objective:** Replace the stub with the full incremental hull algorithm.

**Files:**
- Modify: `src/hull/hull_3d.c3`

**Step 1: Write implementation**

```c3
module cg::hull;
import cg;
import cg::geometry;
import cg::half_edge;
import std::collections::map;

const usz H3_FACE_CAP_FACTOR = 4;

struct H3Face {
    VertexIndex v0, v1, v2;
    bool alive;
}

// Remove exact (x,y,z) duplicate indices from the sorted-indices array.
// Returns the number of unique indices written back into the array.
fn usz h3_deduplicate_indices(int[] indices, Vec3f[] positions)
{
    usz dedup_len = 0;
    for (usz i = 0; i < indices.len; i++) {
        int curr = indices[i];
        bool duplicate = false;
        for (usz j = 0; j < dedup_len; j++) {
            int prev = indices[j];
            if (positions[prev].x == positions[curr].x
                && positions[prev].y == positions[curr].y
                && positions[prev].z == positions[curr].z) {
                duplicate = true;
                break;
            }
        }
        if (!duplicate) {
            indices[dedup_len] = curr;
            dedup_len++;
        }
    }
    return dedup_len;
}

fn HalfEdgeMesh? hull_3d(Allocator alloc, Vec3f[] positions)
{
    if (positions.len == 0) return cg::EMPTY_INPUT~;

    // Deduplicate exact (x,y,z) duplicates.
    int[] indices = mem::alloc::new_array(alloc, int, (sz) positions.len);
    defer catch free(indices);
    for (usz i = 0; i < positions.len; i++) indices[i] = (int)i;

    usz unique_count = h3_deduplicate_indices(indices, positions);
    if (unique_count < 4) return cg::DEGENERATE_INPUT~;

    // Phase 0 — find non-coplanar tetrahedron.
    // Scan for non-collinear base triangle first, then a non-coplanar 4th point.
    int[4] tet = { indices[0], -1, -1, -1 };

    // Find second distinct point (non-coincident).
    for (usz i = 1; i < unique_count; i++) {
        int p = indices[i];
        if (positions[p].x != positions[tet[0]].x
            || positions[p].y != positions[tet[0]].y
            || positions[p].z != positions[tet[0]].z) {
            tet[1] = p;
            break;
        }
    }
    if (tet[1] < 0) return cg::DEGENERATE_INPUT~;

    // Find third non-collinear point.
    for (usz i = 2; i < unique_count; i++) {
        int p = indices[i];
        // Use orient_2d on projected x-y to detect collinearity quickly,
        // but 3D collinearity requires checking that cross product ≠ 0.
        // Simplified: try orient_3d with a dummy 4th point — if the volume
        // is non-zero, points are non-coplanar (hence non-collinear).
        // Actually, just check that the three points aren't collinear by
        // seeing if they form a non-zero volume with any 4th point later.
        // For now, pick any distinct point.
        if (p != tet[0] && p != tet[1]) {
            tet[2] = p;
            break;
        }
    }
    if (tet[2] < 0) return cg::DEGENERATE_INPUT~;

    // Find fourth non-coplanar point.
    bool found = false;
    for (usz i = 0; i < unique_count; i++) {
        int p = indices[i];
        if (p == tet[0] || p == tet[1] || p == tet[2]) continue;
        if (geometry::orient_3d(positions[tet[0]], positions[tet[1]],
                positions[tet[2]], positions[p],
                geometry::GEOMETRY_EPSILON) != geometry::PREDICATE_ZERO) {
            tet[3] = p;
            found = true;
            break;
        }
    }
    if (!found) return cg::DEGENERATE_INPUT~;

    // Reject collinear base: if orient_3d of first 3 + 4th is zero with the
    // same epsilon, the first 3 are collinear. We already found a non-coplanar
    // 4th, so this is covered by the check above.

    // Build compacted positions array for from_triangles.
    Vec3f[] compacted = mem::alloc::new_array(alloc, Vec3f, (sz) unique_count);
    defer catch free(compacted);
    for (usz i = 0; i < unique_count; i++) {
        compacted[i] = positions[indices[i]];
    }

    // Build index remap: original index → compacted index.
    // Since we may have up to positions.len indices, use indices array as scratch.
    int[] remap = mem::alloc::new_array(alloc, int, (sz) positions.len);
    defer catch free(remap);
    for (usz i = 0; i < positions.len; i++) remap[i] = -1;
    for (usz i = 0; i < unique_count; i++) remap[indices[i]] = (int)i;

    // Build initial tetrahedron in compacted space.
    int t0 = remap[tet[0]];
    int t1 = remap[tet[1]];
    int t2 = remap[tet[2]];
    int t3 = remap[tet[3]];

    // Compute centroid of the 4 tetrahedron vertices for orientation.
    Vec3f centroid = {
        (compacted[t0].x + compacted[t1].x + compacted[t2].x + compacted[t3].x) / 4.0f,
        (compacted[t0].y + compacted[t1].y + compacted[t2].y + compacted[t3].y) / 4.0f,
        (compacted[t0].z + compacted[t1].z + compacted[t2].z + compacted[t3].z) / 4.0f,
    };

    usz face_cap = (usz)(H3_FACE_CAP_FACTOR * unique_count);
    H3Face[] faces = mem::alloc::new_array(alloc, H3Face, (sz) face_cap);
    defer catch free(faces);

    HashMap{int[<2>], int} edge_map;
    edge_map.init(alloc);
    defer catch edge_map.free();

    usz face_count = 0;

    // 4 faces of the tetrahedron.
    int[3][4] tet_faces = {
        { { t0, t1, t2 } },
        { { t0, t3, t1 } },
        { { t0, t2, t3 } },
        { { t1, t3, t2 } },
    };

    for (int fi = 0; fi < 4; fi++) {
        int v0 = tet_faces[fi][0];
        int v1 = tet_faces[fi][1];
        int v2 = tet_faces[fi][2];
        // Orient outward: centroid behind each face.
        if (geometry::orient_3d(compacted[v0], compacted[v1], compacted[v2], centroid,
                geometry::GEOMETRY_EPSILON) == geometry::PREDICATE_POSITIVE) {
            int tmp = v1; v1 = v2; v2 = tmp;
        }
        faces[face_count] = { (VertexIndex)v0, (VertexIndex)v1, (VertexIndex)v2, true };
        int[<2>] e0 = { v0, v1 };
        int[<2>] e1 = { v1, v2 };
        int[<2>] e2 = { v2, v0 };
        edge_map[e0] = (int)face_count;
        edge_map[e1] = (int)face_count;
        edge_map[e2] = (int)face_count;
        face_count++;
    }

    // Mark tetrahedron vertices as processed.
    bool[4] tet_processed = { true, true, true, true };

    // Phase 1 — process remaining points.
    for (usz si = 0; si < unique_count; si++) {
        // Skip tetrahedron vertices.
        if (si < 4 && tet_processed[si]) continue;

        int p = (int)si;

        // Visibility.
        usz visible_mask = 0;
        usz visible_count = 0;
        for (usz fi = 0; fi < face_count; fi++) {
            if (!faces[fi].alive) continue;
            if (geometry::orient_3d(
                    compacted[(int)faces[fi].v0],
                    compacted[(int)faces[fi].v1],
                    compacted[(int)faces[fi].v2],
                    compacted[p],
                    geometry::GEOMETRY_EPSILON) == geometry::PREDICATE_POSITIVE) {
                faces[fi].alive = false;
                visible_count++;
            }
        }
        if (visible_count == 0) continue;

        // Horizon edges. Collect in order by walking the horizon cycle.
        // Strategy: find a visible face, then walk its boundary looking for edges
        // whose reverse maps to an alive face. Follow the cycle.
        int[] horizon = mem::alloc::new_array(alloc, int, (sz)(6 * visible_count));
        defer catch free(horizon);
        usz horizon_count = 0;

        // Build the horizon by scanning all visible faces and their edges.
        for (usz fi = 0; fi < face_count; fi++) {
            if (faces[fi].alive) continue; // skip alive faces (visible=dead)

            int[2][3] fe = {
                { { (int)faces[fi].v0, (int)faces[fi].v1 } },
                { { (int)faces[fi].v1, (int)faces[fi].v2 } },
                { { (int)faces[fi].v2, (int)faces[fi].v0 } },
            };
            for (int ei = 0; ei < 3; ei++) {
                int u = fe[ei][0];
                int v = fe[ei][1];
                int[<2>] rev = { v, u };
                if (try adj = edge_map[rev]) {
                    if ((usz)adj < face_count && faces[adj].alive) {
                        // (u,v) is a horizon edge — adjacent face is alive.
                        horizon[horizon_count] = u;
                        horizon[horizon_count + 1] = v;
                        horizon_count += 2;
                    }
                }
            }
        }

        // Stitch new faces from horizon edges.
        usz added = 0;
        for (usz hi = 0; hi < horizon_count; hi += 2) {
            int u = horizon[hi];
            int v = horizon[hi + 1];
            faces[face_count + added] = { (VertexIndex)v, (VertexIndex)u, (VertexIndex)p, true };
            int[<2>] e0 = { v, u };
            int[<2>] e1 = { u, p };
            int[<2>] e2 = { p, v };
            edge_map[e0] = (int)(face_count + added);
            edge_map[e1] = (int)(face_count + added);
            edge_map[e2] = (int)(face_count + added);
            added++;
        }
        face_count += added;

        free(horizon);

        // Compact: remove dead faces and rebuild edge map with updated indices.
        usz write = 0;
        for (usz fi = 0; fi < face_count; fi++) {
            if (faces[fi].alive) {
                if (write != fi) faces[write] = faces[fi];
                write++;
            }
        }
        face_count = write;

        // Rebuild edge map from scratch for alive faces.
        edge_map.free();
        edge_map.init(alloc);
        for (usz fi = 0; fi < face_count; fi++) {
            int v0 = (int)faces[fi].v0;
            int v1 = (int)faces[fi].v1;
            int v2 = (int)faces[fi].v2;
            int[<2>] e0 = { v0, v1 };
            int[<2>] e1 = { v1, v2 };
            int[<2>] e2 = { v2, v0 };
            edge_map[e0] = (int)fi;
            edge_map[e1] = (int)fi;
            edge_map[e2] = (int)fi;
        }
    }

    edge_map.free();
    free(remap);

    // Phase 2 — build HalfEdgeMesh from compacted positions.
    uint[] tri_indices = mem::alloc::new_array(alloc, uint, (sz)(3 * face_count));
    defer catch free(tri_indices);
    for (usz fi = 0; fi < face_count; fi++) {
        tri_indices[3 * fi]     = (uint)(int)faces[fi].v0;
        tri_indices[3 * fi + 1] = (uint)(int)faces[fi].v1;
        tri_indices[3 * fi + 2] = (uint)(int)faces[fi].v2;
    }

    HalfEdgeMesh mesh = half_edge::from_triangles(alloc, compacted, tri_indices)!;
    defer catch mesh.destroy();

    mesh.validate()!;

    free(tri_indices);
    free(faces);
    free(compacted);
    free(indices);

    return mesh;
}
```

**Implementation notes for the agent:**
- `int[3][4] tet_faces` is C3 syntax for a fixed array of 4 elements, each an `int[3]`
- `int[2][3] fe` is a fixed array of 3 elements, each an `int[2]`
- After explicit `free()`s, `validate()` runs — if it faults, only `mesh` has `defer catch destroy()`, the scratch arrays are already freed
- Edge map is rebuilt from scratch after each point insertion to avoid stale indices

**Step 2: Run tests**

```bash
c3c test
```

Expected: all hull_3d tests PASS. All existing tests still PASS.

**Step 3: Debug if needed, then commit**

```bash
git add src/hull/hull_3d.c3
git commit -m "hull: implement incremental 3D convex hull"
```

---

### Task 6: Final verification

**Objective:** Clean build + full test suite + style check.

**Step 1: Clean build (release)**

```bash
c3c clean && c3c build release
```

Expected: No warnings, no errors.

**Step 2: Full test suite**

```bash
c3c test
```

Expected: All tests pass.

**Step 3: Verify style**

- [ ] `src/hull/hull_3d.c3`: `snake_case`, `PascalCase`, `SCREAMING_SNAKE_CASE`
- [ ] `const usz H3_FACE_CAP_FACTOR` before `struct H3Face` (constants before structs)
- [ ] Allocations: `mem::alloc::new_array(alloc, T, (sz) count)` + `defer catch free`
- [ ] No runtime `assert()` in production code
- [ ] `module cg::hull;`
- [ ] No `return null` — all fault paths use `~`
- [ ] `HashMap` accessed via `if (try adj = edge_map[rev])`

**Step 4: Commit if fixes made**

```bash
git add src/hull/hull_3d.c3
git commit -m "hull: final verification and style pass"
```

---

## Implementation Notes

### C3 0.8.0 fixed-array syntax

C3 uses type-suffix array declarations: `int[4] arr` not `int arr[4]`. Multi-dimensional: `int[3][4] matrix` is 4 elements of `int[3]`.

### HashMap patterns (from builder.c3)

```c3
HashMap{int[<2>], int} edge_map;
edge_map.init(alloc);
defer catch edge_map.free();

edge_map[key] = value;                    // insert
if (try val = edge_map[key]) { ... }     // lookup (optional)
if (edge_map.has_key(key)) { ... }        // contains check
edge_map.free();                          // cleanup
```

### Edge map rebuild strategy

After removing visible faces and compacting the face array, face indices shift. To avoid stale entries, the edge map is completely rebuilt from scratch after each point insertion. This is O(faces) per point, but total work is O(n·h) where h is the hull face count — acceptable for the incremental algorithm.

### Compacted positions

The output must use only vertices that actually appear in the output faces (no isolated vertices from duplicates or skipped points). The implementation builds a `compacted` positions array indexed by the deduplicated unique indices, and a `remap` array mapping original indices → compacted indices.

### Memory ownership

| Array | Cleanup |
|-------|---------|
| `indices` (dedup scratch) | Explicit `free` + `defer catch` |
| `remap` | Explicit `free` + `defer catch` |
| `compacted` | Explicit `free` + `defer catch` |
| `faces` | Explicit `free` + `defer catch` |
| `edge_map` | `edge_map.free()` (rebuilt each iteration) + `defer catch` |
| `horizon` | Explicit `free` per iteration + `defer catch` |
| `tri_indices` | Explicit `free` + `defer catch` |
| `HalfEdgeMesh` | Caller via `mesh.destroy()`. On fault after `from_triangles`, `defer catch mesh.destroy()`. |

Explicit frees happen BEFORE `mesh.validate()!` so a validation fault doesn't double-free scratch arrays.
