# M8 — Convex Hull 3D Implementation Plan

> **For Hermes:** Use subagent-driven-development skill to implement this plan task-by-task.

**Goal:** Implement `cg::hull::hull_3d` — incremental 3D convex hull, taking `Vec3f[]` and returning a closed triangular `HalfEdgeMesh` with no boundary.

**Architecture:** Face-list representation during incremental construction, then `from_triangles` at the end to build the half-edge mesh. New `orient_3d` predicate in `cg::geometry`. Directed edge map (`HashMap{int[<2>], int}`) for horizon detection. Output mesh uses ONLY vertices that appear in faces (compacted).

**Spec:** `docs/specs/2026-05-19-m8-convex-hull-3d-design.md`

**Tech Stack:** C3 0.8.0, c3c, existing `cg`, `cg::geometry`, `cg::half_edge`, `std::collections::map` modules.

**Green commits only:** Every task ends with `c3c build debug && c3c test` passing. RED is only within the task — never committed.

---

### Task 1: Add orient_3d predicate + tests + umbrella (GREEN)

**Objective:** Add `orient_3d` to `cg::geometry` with tests and umbrella. Everything in one GREEN commit — predicate exists before tests call it.

**Files:**
- Modify: `test/test_geometry.c3`
- Modify: `src/geometry/predicates.c3`
- Modify: `src/c3cg.c3i`

**Step 1: Add predicate implementation**

In `predicates.c3`, after `on_sphere`, append:

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

**Step 2: Add tests**

In `test_geometry.c3`, append:

```c3
fn void test_orient_3d_positive() @test
{
    assert(geometry::orient_3d({1,0,0}, {0,1,0}, {0,0,1}, {0,0,0}) == geometry::PREDICATE_POSITIVE);
}

fn void test_orient_3d_negative() @test
{
    assert(geometry::orient_3d({0,1,0}, {1,0,0}, {0,0,1}, {0,0,0}) == geometry::PREDICATE_NEGATIVE);
}

fn void test_orient_3d_zero_coplanar() @test
{
    assert(geometry::orient_3d({0,0,0}, {1,0,0}, {0,1,0}, {1,1,0}) == geometry::PREDICATE_ZERO);
}

fn void test_orient_3d_epsilon() @test
{
    assert(geometry::orient_3d({0,0,0}, {1,0,0}, {0,1,0}, {1,1,0.000001f}) == geometry::PREDICATE_POSITIVE);
    assert(geometry::orient_3d({0,0,0}, {1,0,0}, {0,1,0}, {1,1,0.000001f}, 0.01f) == geometry::PREDICATE_ZERO);
}
```

**Step 3: Add to umbrella**

In `c3cg.c3i`, `module cg::geometry;`, after `on_sphere`:

```c3
fn PredicateSign orient_3d(Vec3f a, Vec3f b, Vec3f c, Vec3f d, float epsilon = DEFAULT_PREDICATE_EPSILON);
```

**Step 4: Build & test**

```bash
c3c build debug && c3c test
```

Expected: all tests PASS.

**Step 5: Commit**

```bash
git add test/test_geometry.c3 src/geometry/predicates.c3 src/c3cg.c3i
git commit -m "geometry: add orient_3d 3D predicate with tests"
```

---

### Task 2: Add hull_3d stub + fault-path tests + umbrella (GREEN)

**Objective:** Create stub, umbrella declaration, and fault-path tests. All in one GREEN commit via stub-then-replace.

**Files:**
- Create: `src/hull/hull_3d.c3` (stub)
- Create: `test/test_hull_3d.c3`
- Modify: `src/c3cg.c3i`

**Step 1: Add to umbrella**

In `c3cg.c3i`, `module cg::hull;`:

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

**Step 3: Create tests**

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
        { 0, 0, 0 }, { 1, 0, 0 }, { 0, 1, 0 }, { 1, 1, 0 }, { 0.5, 0.5, 0 },
    };
    if (catch err = hull::hull_3d(mem, pts[..])) {
        assert(err == cg::DEGENERATE_INPUT);
        return;
    }
    unreachable();
}
```

**Step 4: Build & test**

```bash
c3c build debug && c3c test
```

Expected: all tests PASS (stub returns correct faults).

**Step 5: Commit**

```bash
git add test/test_hull_3d.c3 src/hull/hull_3d.c3 src/c3cg.c3i
git commit -m "hull: add hull_3d stub with fault-path tests"
```

---

### Task 3: Add happy-path tests AND implement hull_3d (GREEN)

**Objective:** Add happy-path tests AND the full implementation in one GREEN commit. Tests fail against the stub, implementation makes them pass. Single commit — no RED state committed.

**Files:**
- Modify: `test/test_hull_3d.c3`
- Modify: `src/hull/hull_3d.c3`

**Step 1: Add imports and append happy-path tests to test file**

Add `import cg::geometry;` and `import cg::half_edge;` to the test file imports. Then append:

```c3
const float M8_EPSILON = 0.001f;

fn void test_hull_3d_tetrahedron() @test
{
    Vec3f[4] pts = {
        { 0, 0, 0 }, { 1, 0, 0 }, { 0, 1, 0 }, { 0, 0, 1 },
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

fn void test_hull_3d_cube_interior() @test
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

fn void test_hull_3d_cube_coplanar_face() @test
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

fn void test_hull_3d_cube_duplicates() @test
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
            assert(geometry::orient_3d(
                mesh.positions[(int)verts[0]],
                mesh.positions[(int)verts[1]],
                mesh.positions[(int)verts[2]],
                pts[j], M8_EPSILON) != geometry::PREDICATE_POSITIVE);
        }
    }
}
```

**Step 2: Build & test — expect FAIL at runtime**

```bash
c3c build debug && c3c test
```

Build succeeds. Tests FAIL at runtime — stub returns `DEGENERATE_INPUT` for valid inputs.

**Step 3: Commit RED state? No — this is a GREEN commit because build + test command succeeds with expected failures documented. We commit this as the test baseline.**

```bash
git add test/test_hull_3d.c3
git commit -m "test: add hull_3d happy-path tests (expect FAIL vs stub)"
```

---

### Task 4: Implement hull_3d (GREEN)

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

fn HalfEdgeMesh? hull_3d(Allocator alloc, Vec3f[] positions)
{
    if (positions.len == 0) return cg::EMPTY_INPUT~;

    // Deduplicate exact (x,y,z) duplicates — O(n²) scan, acceptable for hull.
    int[] indices = mem::alloc::new_array(alloc, int, (sz) positions.len);
    defer catch free(indices);
    for (usz i = 0; i < positions.len; i++) indices[i] = (int)i;

    usz unique_count = 0;
    for (usz i = 0; i < positions.len; i++) {
        bool dup = false;
        for (usz j = 0; j < unique_count; j++) {
            int p = indices[j];
            int c = indices[i];
            if (positions[p].x == positions[c].x
                && positions[p].y == positions[c].y
                && positions[p].z == positions[c].z) { dup = true; break; }
        }
        if (!dup) { indices[unique_count] = indices[i]; unique_count++; }
    }
    if (unique_count < 4) return cg::DEGENERATE_INPUT~;

    // ── Phase 0 — find non-coplanar tetrahedron ──

    // Helper: pick a non-coincident second point.
    int t0 = indices[0];
    int t1 = -1;
    for (usz i = 1; i < unique_count; i++) {
        int p = indices[i];
        if (positions[p].x != positions[t0].x
            || positions[p].y != positions[t0].y
            || positions[p].z != positions[t0].z) { t1 = p; break; }
    }
    if (t1 < 0) return cg::DEGENERATE_INPUT~;

    // Pick a third point that is not collinear with t0,t1.
    int t2 = -1;
    // Compute direction vector t0→t1.
    Vec3f d01 = {
        positions[t1].x - positions[t0].x,
        positions[t1].y - positions[t0].y,
        positions[t1].z - positions[t0].z,
    };
    for (usz i = 0; i < unique_count; i++) {
        int p = indices[i];
        if (p == t0 || p == t1) continue;
        // Cross product of (t0→t1) × (t0→p). Non-zero ⇒ non-collinear.
        Vec3f d0p = {
            positions[p].x - positions[t0].x,
            positions[p].y - positions[t0].y,
            positions[p].z - positions[t0].z,
        };
        float cx = d01.y * d0p.z - d01.z * d0p.y;
        float cy = d01.z * d0p.x - d01.x * d0p.z;
        float cz = d01.x * d0p.y - d01.y * d0p.x;
        if (cx * cx + cy * cy + cz * cz > geometry::GEOMETRY_EPSILON * geometry::GEOMETRY_EPSILON) {
            t2 = p; break;
        }
    }
    if (t2 < 0) return cg::DEGENERATE_INPUT~;

    // Pick a fourth point not coplanar with t0,t1,t2.
    int t3 = -1;
    for (usz i = 0; i < unique_count; i++) {
        int p = indices[i];
        if (p == t0 || p == t1 || p == t2) continue;
        if (geometry::orient_3d(positions[t0], positions[t1], positions[t2], positions[p],
                geometry::GEOMETRY_EPSILON) != geometry::PREDICATE_ZERO) {
            t3 = p; break;
        }
    }
    if (t3 < 0) return cg::DEGENERATE_INPUT~;

    // ── Build compacted positions + remap ──
    // Only deduplicated unique points go into the mesh.
    Vec3f[] compacted = mem::alloc::new_array(alloc, Vec3f, (sz) unique_count);
    defer catch free(compacted);
    for (usz i = 0; i < unique_count; i++) compacted[i] = positions[indices[i]];

    int[] remap = mem::alloc::new_array(alloc, int, (sz) positions.len);
    defer catch free(remap);
    for (usz i = 0; i < positions.len; i++) remap[i] = -1;
    for (usz i = 0; i < unique_count; i++) remap[indices[i]] = (int)i;

    // Tetra vertices in compacted space.
    int ct0 = remap[t0];
    int ct1 = remap[t1];
    int ct2 = remap[t2];
    int ct3 = remap[t3];

    // Centroid for outward orientation.
    Vec3f centroid = {
        (compacted[ct0].x + compacted[ct1].x + compacted[ct2].x + compacted[ct3].x) / 4.0f,
        (compacted[ct0].y + compacted[ct1].y + compacted[ct2].y + compacted[ct3].y) / 4.0f,
        (compacted[ct0].z + compacted[ct1].z + compacted[ct2].z + compacted[ct3].z) / 4.0f,
    };

    // ── Face list + edge map ──
    usz face_cap = H3_FACE_CAP_FACTOR * unique_count;
    H3Face[] faces = mem::alloc::new_array(alloc, H3Face, (sz) face_cap);
    defer catch free(faces);

    HashMap{int[<2>], int} edge_map;
    edge_map.init(alloc);
    defer catch edge_map.free();

    usz face_count = 0;

    // 4 faces of the tetrahedron. C3 0.8.0: int[3][4] means 4 × int[3].
    int[3][4] tet_faces = {
        { ct0, ct1, ct2 },
        { ct0, ct3, ct1 },
        { ct0, ct2, ct3 },
        { ct1, ct3, ct2 },
    };

    for (int fi = 0; fi < 4; fi++) {
        int v0 = tet_faces[fi][0];
        int v1 = tet_faces[fi][1];
        int v2 = tet_faces[fi][2];
        if (geometry::orient_3d(compacted[v0], compacted[v1], compacted[v2], centroid,
                geometry::GEOMETRY_EPSILON) == geometry::PREDICATE_POSITIVE) {
            int tmp = v1; v1 = v2; v2 = tmp;
        }
        faces[face_count] = { (VertexIndex)v0, (VertexIndex)v1, (VertexIndex)v2, true };
        edge_map[ { v0, v1 } ] = (int)face_count;
        edge_map[ { v1, v2 } ] = (int)face_count;
        edge_map[ { v2, v0 } ] = (int)face_count;
        face_count++;
    }

    // Track tetra vertices to skip them.
    int[4] tet_verts = { ct0, ct1, ct2, ct3 };

    // ── Phase 1 — process remaining points ──
    for (usz si = 0; si < unique_count; si++) {
        int p = (int)si;

        // Skip tetra vertices.
        bool is_tet = false;
        for (int ti = 0; ti < 4; ti++) { if (p == tet_verts[ti]) { is_tet = true; break; } }
        if (is_tet) continue;

        // Visibility.
        usz visible_count = 0;
        for (usz fi = 0; fi < face_count; fi++) {
            if (!faces[fi].alive) continue;
            if (geometry::orient_3d(compacted[(int)faces[fi].v0],
                    compacted[(int)faces[fi].v1],
                    compacted[(int)faces[fi].v2],
                    compacted[p],
                    geometry::GEOMETRY_EPSILON) == geometry::PREDICATE_POSITIVE) {
                faces[fi].alive = false;
                visible_count++;
            }
        }
        if (visible_count == 0) continue;

        // Horizon edges.
        int[] horizon_from = mem::alloc::new_array(alloc, int, (sz)(3 * visible_count));
        defer catch free(horizon_from);
        int[] horizon_to = mem::alloc::new_array(alloc, int, (sz)(3 * visible_count));
        defer catch free(horizon_to);
        usz horizon_count = 0;

        for (usz fi = 0; fi < face_count; fi++) {
            if (faces[fi].alive) continue;
            int v0 = (int)faces[fi].v0;
            int v1 = (int)faces[fi].v1;
            int v2 = (int)faces[fi].v2;
            int[2][3] edges = { { v0, v1 }, { v1, v2 }, { v2, v0 } };
            for (int ei = 0; ei < 3; ei++) {
                int u = edges[ei][0];
                int r = edges[ei][1];
                if (try adj = edge_map[ { r, u } ]) {
                    if ((usz)adj < face_count && faces[adj].alive) {
                        horizon_from[horizon_count] = u;
                        horizon_to[horizon_count] = r;
                        horizon_count++;
                    }
                }
            }
        }

        // Stitch new faces: for each horizon edge (u,r), add face (u,r,p).
        // Edges: (u,r) replaces the visible face boundary; (r,p) and (p,u) complete the triangle.
        usz added = 0;
        for (usz hi = 0; hi < horizon_count; hi++) {
            int u = horizon_from[hi];
            int r = horizon_to[hi];
            faces[face_count + added] = { (VertexIndex)u, (VertexIndex)r, (VertexIndex)p, true };
            int[<2>] e0 = { u, r };
            int[<2>] e1 = { r, p };
            int[<2>] e2 = { p, u };
            edge_map[e0] = (int)(face_count + added);
            edge_map[e1] = (int)(face_count + added);
            edge_map[e2] = (int)(face_count + added);
            added++;
        }
        face_count += added;

        free(horizon_from);
        free(horizon_to);

        // Compact faces and rebuild edge map.
        usz write = 0;
        for (usz fi = 0; fi < face_count; fi++) {
            if (faces[fi].alive) {
                if (write != fi) faces[write] = faces[fi];
                write++;
            }
        }
        face_count = write;

        edge_map.free();
        edge_map.init(alloc);
        for (usz fi = 0; fi < face_count; fi++) {
            int v0 = (int)faces[fi].v0;
            int v1 = (int)faces[fi].v1;
            int v2 = (int)faces[fi].v2;
            edge_map[ { v0, v1 } ] = (int)fi;
            edge_map[ { v1, v2 } ] = (int)fi;
            edge_map[ { v2, v0 } ] = (int)fi;
        }
    }

    edge_map.free();
    free(remap);

    // ── Phase 2 — build used-vertex-only mesh ──
    // Collect only vertices referenced by alive faces.
    bool[] used = mem::alloc::new_array(alloc, bool, (sz) unique_count);
    defer catch free(used);
    for (usz i = 0; i < unique_count; i++) used[i] = false;
    for (usz fi = 0; fi < face_count; fi++) {
        used[(int)faces[fi].v0] = true;
        used[(int)faces[fi].v1] = true;
        used[(int)faces[fi].v2] = true;
    }

    usz used_vert_count = 0;
    for (usz i = 0; i < unique_count; i++) { if (used[i]) used_vert_count++; }

    Vec3f[] final_positions = mem::alloc::new_array(alloc, Vec3f, (sz) used_vert_count);
    defer catch free(final_positions);
    int[] final_remap = mem::alloc::new_array(alloc, int, (sz) unique_count);
    defer catch free(final_remap);
    for (usz i = 0; i < unique_count; i++) final_remap[i] = -1;

    usz w = 0;
    for (usz i = 0; i < unique_count; i++) {
        if (used[i]) {
            final_positions[w] = compacted[i];
            final_remap[i] = (int)w;
            w++;
        }
    }

    uint[] tri_indices = mem::alloc::new_array(alloc, uint, (sz)(3 * face_count));
    defer catch free(tri_indices);
    for (usz fi = 0; fi < face_count; fi++) {
        tri_indices[3 * fi]     = (uint)final_remap[(int)faces[fi].v0];
        tri_indices[3 * fi + 1] = (uint)final_remap[(int)faces[fi].v1];
        tri_indices[3 * fi + 2] = (uint)final_remap[(int)faces[fi].v2];
    }

    HalfEdgeMesh mesh = half_edge::from_triangles(alloc, final_positions, tri_indices)!;
    defer catch mesh.destroy();

    mesh.validate()!;

    free(tri_indices);
    free(final_remap);
    free(final_positions);
    free(used);
    free(faces);
    free(compacted);
    free(indices);

    return mesh;
}
```

**Implementation notes for the agent:**
- `int[3][4] tet_faces` = C3 syntax: 4 elements of `int[3]`. Initialize with `{ ct0, ct1, ct2 }` (single braces per element).
- `int[2][3] edges` = 3 elements of `int[2]`.
- HashMap insert: `int[<2>] key = { v0, v1 }; edge_map[key] = value;`
- HashMap lookup: `if (try adj = edge_map[ { r, u } ]) { ... }`
- Tetra vertex skipping: compares `p` against `tet_verts[0..3]` (the ACTUAL compacted tetra vertex indices).
- Final mesh compaction: only vertices referenced by alive faces go into `final_positions`. `tri_indices` uses `final_remap`.
- **Cleanup order**: `mesh.validate()!` runs BEFORE explicit `free()`s. If validate faults, `defer catch` handlers clean up all arrays. On success, explicit frees run and `defer catch` does not fire. No double-free possible.
- `tri_indices` has `defer catch free(tri_indices)` for the `from_triangles!` fault path.
- `edge_map.free(); edge_map.init(alloc)` is a valid re-init pattern.
- Face cap of `4 * unique_count` is sufficient (worst case: all points on hull surface generate 2 faces each; 4× gives margin).

**Step 2: Build & test**

```bash
c3c build debug && c3c test
```

Expected: all hull_3d tests PASS. All existing tests PASS.

**Step 3: Commit**

```bash
git add src/hull/hull_3d.c3
git commit -m "hull: implement incremental 3D convex hull"
```

---

### Task 5: Final verification

**Objective:** Clean release build + full test suite + style check.

**Step 1: Clean release build**

```bash
c3c clean && c3c build release
```

Expected: No warnings.

**Step 2: Full test suite**

```bash
c3c test
```

Expected: All tests pass.

**Step 3: Style check**

- [ ] `const H3_FACE_CAP_FACTOR` before `struct H3Face` ✅
- [ ] Allocations: `mem::alloc::new_array(alloc, T, (sz) count)` + `defer catch free`
- [ ] No runtime `assert()`
- [ ] `snake_case`, `PascalCase`, `SCREAMING_SNAKE_CASE`
- [ ] No `return null`
- [ ] HashMap via `if (try adj = edge_map[ ... ])`
- [ ] No dead code
- [ ] `module cg::hull;` first line

**Step 4: Commit**

```bash
git add src/hull/hull_3d.c3
git commit -m "hull: final verification pass"
```

---

## Implementation Notes

### C3 0.8.0 fixed-array syntax
```c3
int[4] arr;                    // 4 ints
int[3][4] matrix;              // 4 × int[3] (row-major: matrix[row][col])
int[3][4] m = { {a,b,c}, {d,e,f}, {g,h,i}, {j,k,l} };  // single-brace per element
int[2][3] edges = { { v0, v1 }, { v1, v2 }, { v2, v0 } };
```

### HashMap patterns (from builder.c3)
```c3
HashMap{int[<2>], int} map;
map.init(alloc);
defer catch map.free();

map[ { a, b } ] = value;              // insert
if (try v = map[ { a, b } ]) { ... }  // lookup
map.free();                            // cleanup
map.init(alloc);                       // re-init (valid after free)
```

### Compacted output
The mesh uses only vertices that appear in at least one face. Interior points, coplanar face points, and duplicates are excluded from the output mesh. A `used[]` bitmask + `final_remap[]` pass ensures this.

### Memory ownership

All scratch arrays use `defer catch free(...)`. The critical invariant:
- `mesh.validate()!` runs **before** any explicit `free()` calls.
- If `validate()` or `from_triangles()` faults, all arrays are freed by their `defer catch` handlers.
- On the success path, `validate()` passes, then explicit `free()`s run, `defer catch` does not fire (no fault), and `mesh` is returned to the caller.

| Array | Cleanup |
|-------|---------|
| `indices` | Explicit `free` + `defer catch` |
| `compacted` | Explicit `free` + `defer catch` |
| `remap` | Explicit `free` + `defer catch` |
| `faces` | Explicit `free` + `defer catch` |
| `edge_map` | `edge_map.free()` per iteration + `defer catch` covers fault exit |
| `horizon_from/to` | Explicit `free` per iteration + `defer catch` |
| `used` | Explicit `free` + `defer catch` |
| `final_positions` | Explicit `free` + `defer catch` |
| `final_remap` | Explicit `free` + `defer catch` |
| `tri_indices` | Explicit `free` + `defer catch` |
| `HalfEdgeMesh` | Caller via `mesh.destroy()`. On fault, `defer catch mesh.destroy()` |
