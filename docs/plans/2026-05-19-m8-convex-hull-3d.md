# M8 — Convex Hull 3D Implementation Plan

> **For Hermes:** Use subagent-driven-development skill to implement this plan task-by-task.

**Goal:** Implement `cg::hull::hull_3d` — incremental 3D convex hull, taking `Vec3f[]` and returning a closed triangular `HalfEdgeMesh` with no boundary.

**Architecture:** Face-list representation during incremental construction, then `from_triangles` at the end to build the half-edge mesh. New `orient_3d` predicate in `cg::geometry`. Directed edge map for horizon detection.

**Spec:** `docs/specs/2026-05-19-m8-convex-hull-3d-design.md`

**Tech Stack:** C3 0.8.0, c3c, existing `cg`, `cg::geometry`, `cg::half_edge`, `std::collections::map` modules.

**Stub-then-replace:** Tasks 1–3 create infrastructure (predicate, tests, stub). Tasks 4+ replace stubs with real code. All files auto-discovered by `project.json` (`"sources": ["src/**"]`).

---

### Task 1: Add orient_3d predicate to cg::geometry (GREEN)

**Objective:** Add the `orient_3d` predicate to `cg::geometry` before any hull code references it.

**Files:**
- Modify: `src/geometry/predicates.c3`
- Modify: `src/c3cg.c3i` (umbrella — `module cg::geometry;` section)

**Step 1: Add private helper and public predicate to predicates.c3**

After the existing `det4` function, append:

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

In `src/c3cg.c3i`, under `module cg::geometry;`, after the `on_sphere` line, add:

```c3
fn PredicateSign orient_3d(Vec3f a, Vec3f b, Vec3f c, Vec3f d, float epsilon = DEFAULT_PREDICATE_EPSILON);
```

**Step 3: Build**

```bash
c3c build debug
```

Expected: static library created successfully.

**Step 4: Run tests**

```bash
c3c test
```

Expected: all existing tests still pass.

**Step 5: Commit**

```bash
git add src/geometry/predicates.c3 src/c3cg.c3i
git commit -m "geometry: add orient_3d 3D predicate"
```

---

### Task 2: Write orient_3d tests (GREEN)

**Objective:** Add tests for the new predicate to `test/test_geometry.c3`.

**Files:**
- Modify: `test/test_geometry.c3`

**Step 1: Append tests**

```c3
fn void test_orient_3d_positive_for_ccw_orientation() @test
{
    // right-handed coordinate system: tetrahedron a=(1,0,0), b=(0,1,0), c=(0,0,1), d=(0,0,0)
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
    // Nearly coplanar — should be classified as zero with large epsilon.
    assert(geometry::orient_3d({0,0,0}, {1,0,0}, {0,1,0}, {1,1,0.000001f}) == geometry::PREDICATE_POSITIVE);
    assert(geometry::orient_3d({0,0,0}, {1,0,0}, {0,1,0}, {1,1,0.000001f}, 0.01f) == geometry::PREDICATE_ZERO);
}
```

**Step 2: Run tests**

```bash
c3c test
```

Expected: all 4 new predicate tests PASS, all existing tests still PASS.

**Step 3: Commit**

```bash
git add test/test_geometry.c3
git commit -m "test: add orient_3d predicate tests"
```

---

### Task 3: Write hull_3d fault-path tests (RED)

**Objective:** Write tests for empty, 1-3 points, and coplanar inputs. Add stub to umbrella so tests compile.

**Files:**
- Create: `test/test_hull_3d.c3`
- Modify: `src/c3cg.c3i` (umbrella — add `hull_3d` declaration)

**Step 1: Add hull_3d to umbrella**

In `src/c3cg.c3i`, `module cg::hull;` section, after `hull_2d`:

```c3
fn HalfEdgeMesh? hull_3d(Allocator alloc, Vec3f[] positions);
```

**Step 2: Create test file with fault-path tests**

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

**Step 3: Run tests — expect FAIL**

```bash
c3c test
```

Expected: 5 hull_3d fault-path tests FAIL — no `hull_3d` function defined (only declared in umbrella).

**Step 4: Commit**

```bash
git add test/test_hull_3d.c3 src/c3cg.c3i
git commit -m "test: add hull_3d fault-path tests (RED)"
```

---

### Task 4: Create hull_3d stub (GREEN for fault tests)

**Objective:** Create `src/hull/hull_3d.c3` with a stub that handles only fault paths.

**Files:**
- Create: `src/hull/hull_3d.c3`

**Step 1: Create stub**

```c3
module cg::hull;
import cg;

fn HalfEdgeMesh? hull_3d(Allocator alloc, Vec3f[] positions)
{
    if (positions.len == 0) return cg::EMPTY_INPUT~;
    return cg::DEGENERATE_INPUT~;
}
```

**Step 2: Run tests**

```bash
c3c test
```

Expected: all 5 hull_3d fault-path tests PASS (stub returns `DEGENERATE_INPUT` for non-empty).

**Step 3: Commit**

```bash
git add src/hull/hull_3d.c3
git commit -m "hull: add hull_3d module stub with fault paths"
```

---

### Task 5: Add happy-path tests (RED)

**Objective:** Add tetrahedron, cube, cube+interior, cube+face-coplanar, cube+duplicates, and convexity tests.

**Files:**
- Modify: `test/test_hull_3d.c3`

**Step 1: Append happy-path tests**

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

    // Check closed: no boundary half-edges.
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
    // Point on the top face — should be skipped.
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

    // Verify convexity: every face has all other points inside or on it.
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

**Step 2: Run tests — expect FAIL**

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

### Task 6: Implement hull_3d algorithm (GREEN)

**Objective:** Replace the stub with the full incremental hull algorithm — find tetrahedron, process points, build mesh.

**Files:**
- Modify: `src/hull/hull_3d.c3`

**Step 1: Implement the full algorithm**

```c3
module cg::hull;
import cg;
import cg::geometry;
import cg::half_edge;
import std::collections::map;

struct H3Face {
    VertexIndex v0, v1, v2;
    bool alive;
}

const usz H3_SCRATCH_MULTIPLIER = 4;

fn int[]? h3_deduplicate(Allocator alloc, Vec3f[] positions)
{
    if (positions.len == 0) return null;
    int[] indices = mem::alloc::new_array(alloc, int, (sz) positions.len);
    defer catch free(indices);
    for (usz i = 0; i < positions.len; i++) indices[i] = (int)i;

    usz dedup_len = 0;
    for (usz i = 0; i < positions.len; i++) {
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

    int[] result = mem::alloc::new_array(alloc, int, (sz) dedup_len);
    defer catch free(result);
    for (usz i = 0; i < dedup_len; i++) result[i] = indices[i];
    free(indices);
    return result;
}

fn HalfEdgeMesh? hull_3d(Allocator alloc, Vec3f[] positions)
{
    if (positions.len == 0) return cg::EMPTY_INPUT~;

    // Deduplicate exact (x,y,z) duplicates.
    int[] unique_indices = h3_deduplicate(alloc, positions)!;
    defer catch free(unique_indices);
    if (unique_indices.len < 4) return cg::DEGENERATE_INPUT~;

    // Phase 0 — find 4 non-coplanar points.
    int tet[4] = { unique_indices[0], unique_indices[1], unique_indices[2], -1 };
    bool found = false;
    for (int i = 3; i < (int)unique_indices.len; i++) {
        int d = unique_indices[i];
        if (geometry::orient_3d(
                positions[tet[0]], positions[tet[1]], positions[tet[2]], positions[d],
                geometry::GEOMETRY_EPSILON) != geometry::PREDICATE_ZERO) {
            tet[3] = d;
            found = true;
            break;
        }
    }
    if (!found) return cg::DEGENERATE_INPUT~;

    // Build initial 4 faces, oriented outward from centroid.
    Vec3f centroid = {
        (positions[tet[0]].x + positions[tet[1]].x + positions[tet[2]].x + positions[tet[3]].x) / 4.0f,
        (positions[tet[0]].y + positions[tet[1]].y + positions[tet[2]].y + positions[tet[3]].y) / 4.0f,
        (positions[tet[0]].z + positions[tet[1]].z + positions[tet[2]].z + positions[tet[3]].z) / 4.0f,
    };

    usz face_cap = (usz)(H3_SCRATCH_MULTIPLIER * unique_indices.len);
    H3Face[] faces = mem::alloc::new_array(alloc, H3Face, (sz) face_cap);
    defer catch free(faces);

    HashMap{int[<2>], int} edge_map;
    edge_map.init(alloc);
    defer catch edge_map.free();

    usz face_count = 0;
    int tet_faces[4][3] = {
        { tet[0], tet[1], tet[2] },
        { tet[0], tet[3], tet[1] },
        { tet[0], tet[2], tet[3] },
        { tet[1], tet[3], tet[2] },
    };

    for (int fi = 0; fi < 4; fi++) {
        int v0 = tet_faces[fi][0];
        int v1 = tet_faces[fi][1];
        int v2 = tet_faces[fi][2];
        // Orient outward: centroid should be behind each face (negative orient_3d).
        if (geometry::orient_3d(positions[v0], positions[v1], positions[v2], centroid,
                geometry::GEOMETRY_EPSILON) == geometry::PREDICATE_POSITIVE) {
            int tmp = v1; v1 = v2; v2 = tmp;
        }
        faces[face_count] = { (VertexIndex)v0, (VertexIndex)v1, (VertexIndex)v2, true };
        int[<2>] e0 = { v0, v1 }; int[<2>] e1 = { v1, v2 }; int[<2>] e2 = { v2, v0 };
        edge_map[e0] = (int)face_count;
        edge_map[e1] = (int)face_count;
        edge_map[e2] = (int)face_count;
        face_count++;
    }

    // Phase 1 — process remaining points.
    for (int si = 0; si < (int)unique_indices.len; si++) {
        int p = unique_indices[si];
        // Skip points already used in the initial tetrahedron.
        bool used = false;
        for (int ti = 0; ti < 4; ti++) { if (p == tet[ti]) { used = true; break; } }
        if (used) continue;

        // Visibility check.
        usz visible_count = 0;
        for (usz fi = 0; fi < face_count; fi++) {
            if (!faces[fi].alive) continue;
            if (geometry::orient_3d(
                    positions[(int)faces[fi].v0],
                    positions[(int)faces[fi].v1],
                    positions[(int)faces[fi].v2],
                    positions[p],
                    geometry::GEOMETRY_EPSILON) == geometry::PREDICATE_POSITIVE) {
                faces[fi].alive = false;
                visible_count++;
            }
        }
        if (visible_count == 0) continue;

        // Horizon edges.
        // Collect in order: for each horizon edge (u,v), the next starts at v.
        int[] horizon_u = mem::alloc::new_array(alloc, int, (sz)(3 * visible_count));
        defer catch free(horizon_u);
        int[] horizon_v = mem::alloc::new_array(alloc, int, (sz)(3 * visible_count));
        defer catch free(horizon_v);
        usz horizon_count = 0;

        for (usz fi = 0; fi < face_count; fi++) {
            if (faces[fi].alive) continue;
            int fe[3][2] = {
                { (int)faces[fi].v1, (int)faces[fi].v0 },
                { (int)faces[fi].v2, (int)faces[fi].v1 },
                { (int)faces[fi].v0, (int)faces[fi].v2 },
            };
            for (int ei = 0; ei < 3; ei++) {
                int u = fe[ei][0];
                int v = fe[ei][1];
                int[<2>] rev = { v, u };
                if (!edge_map.has_key(rev)) continue;
                int adj = edge_map[rev];
                if ((usz)adj < face_count && faces[adj].alive) {
                    horizon_u[horizon_count] = u;
                    horizon_v[horizon_count] = v;
                    horizon_count++;
                }
            }
        }

        // Stitch new faces.
        for (usz hi = 0; hi < horizon_count; hi++) {
            int u = horizon_u[hi];
            int v = horizon_v[hi];
            faces[face_count] = { (VertexIndex)v, (VertexIndex)u, (VertexIndex)p, true };
            int[<2>] e0 = { v, u }; int[<2>] e1 = { u, p }; int[<2>] e2 = { p, v };
            edge_map[e0] = (int)face_count;
            edge_map[e1] = (int)face_count;
            edge_map[e2] = (int)face_count;
            face_count++;
        }

        // Remove visible faces by swap-remove.
        usz write = 0;
        for (usz fi = 0; fi < face_count; fi++) {
            if (faces[fi].alive) {
                if (write != fi) faces[write] = faces[fi];
                write++;
            }
        }
        face_count = write;

        free(horizon_u);
        free(horizon_v);
    }

    // Phase 2 — build HalfEdgeMesh.
    uint[] indices = mem::alloc::new_array(alloc, uint, (sz)(3 * face_count));
    defer catch free(indices);
    for (usz fi = 0; fi < face_count; fi++) {
        indices[3 * fi]     = (uint)(int)faces[fi].v0;
        indices[3 * fi + 1] = (uint)(int)faces[fi].v1;
        indices[3 * fi + 2] = (uint)(int)faces[fi].v2;
    }

    HalfEdgeMesh mesh = half_edge::from_triangles(alloc, positions, indices)!;
    defer catch mesh.destroy();

    free(indices);
    edge_map.free();
    free(faces);
    free(unique_indices);

    mesh.validate()!;
    return mesh;
}
```

**Step 2: Run tests**

```bash
c3c test
```

Expected: all hull_3d tests PASS.

**Step 3: If any test fails, debug and fix. Then commit.**

```bash
git add src/hull/hull_3d.c3
git commit -m "hull: implement incremental 3D convex hull"
```

---

### Task 7: Final verification

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

- [ ] `src/hull/hull_3d.c3` uses `snake_case`, `PascalCase` for structs
- [ ] Allocations use `mem::alloc::new_array(alloc, T, (sz) count)` + `defer catch free`
- [ ] No runtime `assert()` in production code
- [ ] Module: `module cg::hull;`
- [ ] Definition order: constants → structs → free functions

**Step 4: Commit if fixes made**

```bash
git add src/hull/hull_3d.c3
git commit -m "hull: final style and verification pass"
```

---

## Implementation Notes

### Directed edge map

The edge map uses `HashMap{int[<2>], int}` mapping directed edge `(from, to)` to face index. Horizon detection: for each visible face, check reverse edge `(to, from)`. If the reverse maps to an alive face, `(from, to)` is NOT a horizon edge. Otherwise it is.

### Face orientation

Initial tetrahedron faces must be consistently oriented outward. The centroid check ensures this: for each face `(v0,v1,v2)`, if `orient_3d(v0,v1,v2,centroid) > 0`, swap v1↔v2. This guarantees the centroid is on the negative (inside) side of every face.

### Deduplication

Unlike hull_2d which sorts and removes adjacent duplicates, 3D duplicates aren't adjacent after any simple sort. Use O(n²) scan or build a temporary set. The plan uses an O(n²) scan for simplicity — hull_3d is O(n²) overall anyway.

### Memory ownership

| Array | Freed by |
|-------|----------|
| `unique_indices` | Explicit `free` + `defer catch` |
| `faces` | Explicit `free` + `defer catch` |
| `edge_map` | `edge_map.free()` on success + `defer catch` |
| `horizon_u/v` | Explicit `free` + `defer catch` per iteration |
| `indices` (for from_triangles) | Explicit `free` + `defer catch` |
| `HalfEdgeMesh` | Caller via `mesh.destroy()` |
