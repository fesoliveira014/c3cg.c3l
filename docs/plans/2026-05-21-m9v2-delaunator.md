# M9v2 — Delaunay 2D (delaunator) Implementation Plan

> **For Hermes:** Use subagent-driven-development skill to implement this plan task-by-task.

**Goal:** Rewrite `cg::delaunay::delaunay_2d` with the delaunator algorithm. Same public API, same 12 tests must pass.

**Strategy:** Build the new implementation in a temporary file `src/delaunay/delaunay_2d_v2.c3` while the old implementation stays live. Compile-check each task against a simple smoke test. Replace the production file in the final task when all tests pass.

**Spec:** `docs/specs/2026-05-21-m9v2-delaunator-design.md`

**Tech Stack:** C3 0.8.0, existing `cg`, `cg::geometry`, `std::sort`, `std::math`, `cg::half_edge`.

**Green commits only.** The old `delaunay_2d.c3` stays untouched until the final swap — all 168 tests pass throughout.

---

### Task 1: Create v2 file with skeleton + smoke test (GREEN)

**Objective:** Create `delaunay_2d_v2.c3` with builder struct + helpers. Write a smoke test that compiles against it.

**Files:**
- Create: `src/delaunay/delaunay_2d_v2.c3`
- Create: `test/test_delaunay_2d_v2.c3` (smoke only)

**Step 1: Create v2 skeleton**

```c3
module cg::delaunay::v2;
import cg;
import cg::geometry;
import cg::half_edge;
import std::sort;
import std::math;

const float DELAUNAY_NEAR_DUP_EPSILON = 1e-12f;

struct DelaunayBuilder {
    VertexIndex[] triangles;
    HeIndex[] halfedges;
    HeIndex[] hull_tri;
    VertexIndex[] hull_prev;
    VertexIndex[] hull_next;
    VertexIndex[] hull_hash;
    HeIndex[] legalize_stack;
    Vec3f hull_center;
    VertexIndex hull_start;
    usz face_count;
}

struct DistSortCtx { Vec3f[] pts; Vec3f center; }

// ── Helpers (all private to this module) ──

fn HeIndex next_halfedge(HeIndex i) { return (i % 3 == 2) ? i - 2 : i + 1; }
fn HeIndex prev_halfedge(HeIndex i) { return (i % 3 == 0) ? i + 2 : i - 1; }
fn usz fast_mod(usz i, usz c) { return i >= c ? i % c : i; }

fn void link_halfedges(HeIndex[] halfedges, HeIndex a, HeIndex b)
{
    halfedges[a] = b;
    if (b != INVALID_HE) halfedges[b] = a;
}

fn float pseudo_angle(float dx, float dy)
{
    float denom = (dx < 0 ? -dx : dx) + (dy < 0 ? -dy : dy);
    float p = dx / denom;
    if (dy > 0.0f) return (3.0f - p) / 4.0f;
    return (1.0f + p) / 4.0f;
}

fn int compare_by_distance(int a, int b, DistSortCtx ctx)
{
    float da = (ctx.pts[a].x - ctx.center.x) * (ctx.pts[a].x - ctx.center.x)
             + (ctx.pts[a].y - ctx.center.y) * (ctx.pts[a].y - ctx.center.y);
    float db = (ctx.pts[b].x - ctx.center.x) * (ctx.pts[b].x - ctx.center.x)
             + (ctx.pts[b].y - ctx.center.y) * (ctx.pts[b].y - ctx.center.y);
    if (da < db) return -1;
    if (da > db) return 1;
    return 0;
}

fn float circumradius_sq(Vec3f a, Vec3f b, Vec3f c)
{
    if (catch err = geometry::circumcenter_planar(a, b, c)) return 1e30f;
    Vec3f cc = geometry::circumcenter_planar(a, b, c)!!;
    float dx = cc.x - a.x;
    float dy = cc.y - a.y;
    return dx * dx + dy * dy;
}

fn int find_closest(Vec3f[] pts, usz count, Vec3f ref, int exclude)
{
    int best = -1;
    float best_dist = 1e30f;
    for (usz i = 0; i < count; i++) {
        if ((int)i == exclude) continue;
        float dx = pts[i].x - ref.x;
        float dy = pts[i].y - ref.y;
        float d2 = dx * dx + dy * dy;
        if (d2 < best_dist) { best = (int)i; best_dist = d2; }
    }
    return best;
}

fn HalfEdgeMesh? delaunay_2d_v2(Allocator alloc, Vec3f[] positions, DelaunayOptions options = {})
{
    if (positions.len == 0) return cg::EMPTY_INPUT~;

    // No defer catch here — explicit free before fault return.
    // Use unconditional defer free for scratch arrays that survive past fault paths.

    // Allocate scratch: unconditional defer free (always runs).
    int[] indices = mem::alloc::new_array(alloc, int, (sz) positions.len);
    defer free(indices);
    for (usz i = 0; i < positions.len; i++) indices[i] = (int)i;

    usz unique_count = 0;
    for (usz i = 0; i < positions.len; i++) {
        bool dup = false;
        for (usz j = 0; j < unique_count; j++) {
            int p = indices[j];
            int c = indices[i];
            if (positions[p].x == positions[c].x && positions[p].y == positions[c].y) {
                dup = true; break;
            }
        }
        if (!dup) { indices[unique_count] = indices[i]; unique_count++; }
    }
    if (unique_count < 3) return cg::DEGENERATE_INPUT~;

    // Collinearity scan.
    int t0 = indices[0];
    int t1 = -1;
    for (usz i = 1; i < unique_count; i++) {
        int p = indices[i];
        if (positions[p].x != positions[t0].x || positions[p].y != positions[t0].y) {
            t1 = p; break;
        }
    }
    if (t1 < 0) return cg::DEGENERATE_INPUT~;

    int t2 = -1;
    Vec2f d0 = { positions[t0].x, positions[t0].y };
    Vec2f d1 = { positions[t1].x, positions[t1].y };
    for (usz i = 0; i < unique_count; i++) {
        int p = indices[i];
        if (p == t0 || p == t1) continue;
        Vec2f dp = { positions[p].x, positions[p].y };
        if (geometry::orient_2d(d0, d1, dp) != geometry::PREDICATE_ZERO) {
            t2 = p; break;
        }
    }
    if (t2 < 0) return cg::DEGENERATE_INPUT~;

    // Bounding box validation.
    if (options.has_bounding_box) {
        if (options.bounding_box.min.x > options.bounding_box.max.x
            || options.bounding_box.min.y > options.bounding_box.max.y) {
            return cg::DEGENERATE_INPUT~;
        }
        for (usz i = 0; i < unique_count; i++) {
            int p = indices[i];
            if (positions[p].x < options.bounding_box.min.x
                || positions[p].x > options.bounding_box.max.x
                || positions[p].y < options.bounding_box.min.y
                || positions[p].y > options.bounding_box.max.y) {
                return cg::DEGENERATE_INPUT~;
            }
        }
    }

    // Build working positions.
    Vec3f[] work_pos = mem::alloc::new_array(alloc, Vec3f, (sz) unique_count);
    defer free(work_pos);
    for (usz i = 0; i < unique_count; i++) work_pos[i] = positions[indices[i]];

    // ── Phase 1: seed triangle ──
    float min_x = work_pos[0].x, max_x = work_pos[0].x;
    float min_y = work_pos[0].y, max_y = work_pos[0].y;
    for (usz i = 1; i < unique_count; i++) {
        if (work_pos[i].x < min_x) min_x = work_pos[i].x;
        if (work_pos[i].x > max_x) max_x = work_pos[i].x;
        if (work_pos[i].y < min_y) min_y = work_pos[i].y;
        if (work_pos[i].y > max_y) max_y = work_pos[i].y;
    }
    Vec3f center = { (min_x + max_x) * 0.5f, (min_y + max_y) * 0.5f, 0 };

    int i0 = find_closest(work_pos, unique_count, center, -1);
    int i1 = find_closest(work_pos, unique_count, work_pos[i0], i0);

    int i2 = -1;
    float min_radius = 1e30f;
    for (usz i = 0; i < unique_count; i++) {
        if ((int)i == i0 || (int)i == i1) continue;
        float r = circumradius_sq(work_pos[i0], work_pos[i1], work_pos[i]);
        if (r < min_radius) { i2 = (int)i; min_radius = r; }
    }
    if (i2 < 0) return cg::DEGENERATE_INPUT~;

    Vec2f s0 = { work_pos[i0].x, work_pos[i0].y };
    Vec2f s1 = { work_pos[i1].x, work_pos[i1].y };
    Vec2f s2 = { work_pos[i2].x, work_pos[i2].y };
    if (geometry::orient_2d(s0, s1, s2) <= geometry::PREDICATE_ZERO) {
        int tmp = i1; i1 = i2; i2 = tmp;
    }

    Vec3f seed_center = geometry::circumcenter_planar(work_pos[i0], work_pos[i1], work_pos[i2])!!;

    // Distance sort.
    for (usz i = 0; i < unique_count; i++) indices[i] = (int)i;
    DistSortCtx dctx = { work_pos, seed_center };
    sort::quicksort(indices, &compare_by_distance, dctx);

    // ── Phase 2-5 will be added in subsequent tasks ──
    free(work_pos);
    free(indices);
    return cg::DEGENERATE_INPUT~;
}
```

Note: `defer catch free(work_pos)` and `defer catch free(indices)` are active. Explicit `free()` is on the success path (which is actually a fault return here). On fault, `defer catch` fires and explicit `free` is NOT reached. On explicit return, `defer catch` does NOT fire. This is correct — the explicit free runs before `return`, then the function returns normally. The `defer catch` doesn't fire because it's not a fault return (it's a normal return of `DEGENERATE_INPUT~`).

Wait — `return cg::DEGENERATE_INPUT~` IS a fault return. So `defer catch` fires AND explicit free runs. Double-free!

Fix: remove `defer catch` from arrays that get explicitly freed before fault returns. The pattern is: only use `defer catch` on arrays that survive past all fault returns.

For now, since this is a skeleton that always faults, just don't use `defer catch` on arrays freed before the fault:

```c3
    // No defer catch on work_pos and indices — they're always freed before return.
    free(work_pos);
    free(indices);
    return cg::DEGENERATE_INPUT~;
```

**Step 2: Create smoke test**

```c3
module test;
import cg;
import cg::delaunay::v2;

fn void test_delaunay_2d_v2_smoke_empty() @test
{
    Vec3f[] pts = {};
    if (catch err = v2::delaunay_2d_v2(mem, pts)) {
        assert(err == cg::EMPTY_INPUT);
        return;
    }
    unreachable();
}
```

**Step 3: Build & test**

```bash
c3c build debug && c3c test
```

Expected: 169 passed (168 existing + 1 new smoke test). The new file is not in `src/**` glob? Check — `project.json` has `"sources": ["src/**"]` so `src/delaunay/delaunay_2d_v2.c3` should be auto-picked up.

**Step 4: Commit**

```bash
git add src/delaunay/delaunay_2d_v2.c3 test/test_delaunay_2d_v2.c3
git commit -m "delaunay: add v2 skeleton with Phase 0-1 — dedup, seed triangle, sort"
```

---

### Task 2: Phase 2 — Builder allocation + hull init (GREEN)

**Objective:** Add builder allocation and hull initialization. Function still returns `DEGENERATE_INPUT` after hull init — smoke test still passes.

**Files:**
- Modify: `src/delaunay/delaunay_2d_v2.c3`

**Step 1: After distance sort, before the final return, insert:**

```c3
    // ── Phase 2: allocate builder + init hull ──
    usz max_triangles = 2 * unique_count;
    usz hash_len = (usz)math::sqrt((double)unique_count);
    if (hash_len < 1) hash_len = 1;

    DelaunayBuilder builder;
    builder.triangles      = mem::alloc::new_array(alloc, VertexIndex, (sz)(3 * max_triangles));
    builder.halfedges      = mem::alloc::new_array(alloc, HeIndex, (sz)(3 * max_triangles));
    builder.hull_tri       = mem::alloc::new_array(alloc, HeIndex, (sz) unique_count);
    builder.hull_prev      = mem::alloc::new_array(alloc, VertexIndex, (sz) unique_count);
    builder.hull_next      = mem::alloc::new_array(alloc, VertexIndex, (sz) unique_count);
    builder.hull_hash      = mem::alloc::new_array(alloc, VertexIndex, (sz) hash_len);
    builder.legalize_stack = mem::alloc::new_array(alloc, HeIndex, (sz) max_triangles);
    builder.face_count     = 0;

    // Seed triangle.
    builder.triangles[0] = (VertexIndex)i0;
    builder.triangles[1] = (VertexIndex)i1;
    builder.triangles[2] = (VertexIndex)i2;
    builder.halfedges[0] = INVALID_HE;
    builder.halfedges[1] = INVALID_HE;
    builder.halfedges[2] = INVALID_HE;
    builder.face_count = 1;

    // Hull linked list.
    for (usz i = 0; i < unique_count; i++) {
        builder.hull_next[i] = INVALID_VERTEX;
        builder.hull_prev[i] = INVALID_VERTEX;
    }
    builder.hull_next[i0] = (VertexIndex)i1;
    builder.hull_next[i1] = (VertexIndex)i2;
    builder.hull_next[i2] = (VertexIndex)i0;
    builder.hull_prev[i1] = (VertexIndex)i0;
    builder.hull_prev[i2] = (VertexIndex)i1;
    builder.hull_prev[i0] = (VertexIndex)i2;
    builder.hull_tri[i0]  = 0;
    builder.hull_tri[i1]  = 1;
    builder.hull_tri[i2]  = 2;
    builder.hull_center   = seed_center;
    builder.hull_start    = (VertexIndex)i0;

    // Hash hull vertices.
    for (usz i = 0; i < hash_len; i++) builder.hull_hash[i] = INVALID_VERTEX;
    int[3] hull_verts = { i0, i1, i2 };
    for (int hi = 0; hi < 3; hi++) {
        int v = hull_verts[hi];
        float dx = work_pos[v].x - seed_center.x;
        float dy = work_pos[v].y - seed_center.y;
        float angle = pseudo_angle(dx, dy);
        usz key = (usz)(angle * (float)hash_len);
        while (builder.hull_hash[key] != INVALID_VERTEX) key = fast_mod(key + 1, hash_len);
        builder.hull_hash[key] = (VertexIndex)v;
    }

    // Free builder + scratch, return stub.
    free(builder.triangles); free(builder.halfedges); free(builder.hull_tri);
    free(builder.hull_prev); free(builder.hull_next); free(builder.hull_hash);
    free(builder.legalize_stack);
    free(work_pos);
    free(indices);
    return cg::DEGENERATE_INPUT~;
```

Include `find_visible_edge`, forward walk, backward walk (if backwards), hull link update, hash update, marking removed vertices. See spec §Phase 3 for complete pseudocode.

**Step 5: Build & test**

```bash
c3c build debug && c3c test
```

Expected: 169 passed.

**Step 3: Commit**

```bash
git add src/delaunay/delaunay_2d_v2.c3
git commit -m "delaunay: add v2 Phase 2 — builder allocation, hull initialization"
```

---

### Task 3: Phases 3+5 — insertion loop + legalize (GREEN)

**Objective:** Implement `find_visible_edge`, `add_triangle`, `legalize`, forward/backward walk, hull link update — the full insertion loop. Removes the stub return. Now produces a valid Delaunay mesh in `DelaunayBuilder`, but still returns `DEGENERATE_INPUT~` (finalization is Task 4).

**Files:**
- Modify: `src/delaunay/delaunay_2d_v2.c3`

**Step 1: Add `add_triangle` and `find_visible_edge`**

```c3
fn HeIndex add_triangle(DelaunayBuilder* b, VertexIndex i0, VertexIndex i1, VertexIndex i2,
                         HeIndex twin0, HeIndex twin1, HeIndex twin2)
{
    usz t = b.face_count++;
    b.triangles[3*t]   = i0;
    b.triangles[3*t+1] = i1;
    b.triangles[3*t+2] = i2;
    b.halfedges[3*t]   = twin0;
    b.halfedges[3*t+1] = twin1;
    b.halfedges[3*t+2] = twin2;
    if (twin0 != INVALID_HE) b.halfedges[twin0] = (HeIndex)(int)(3*t);
    if (twin1 != INVALID_HE) b.halfedges[twin1] = (HeIndex)(int)(3*t+1);
    if (twin2 != INVALID_HE) b.halfedges[twin2] = (HeIndex)(int)(3*t+2);
    return (HeIndex)(int)(3*t);
}
```

**Step 3: Add `legalize()` (matches spec Phase 5)**

```c3
fn HeIndex legalize(HeIndex start_he, Vec3f[] positions, DelaunayBuilder* b)
{
    HeIndex ar = INVALID_HE;
    usz stack_size = 0;
    HeIndex he = start_he;

    while (true) {
        HeIndex a = b.halfedges[he];
        if (a != INVALID_HE) {
            ar = prev_halfedge(he);
            HeIndex al = next_halfedge(he);
            HeIndex bl = prev_halfedge(a);

            VertexIndex p0 = b.triangles[ar];
            VertexIndex pr = b.triangles[he];
            VertexIndex pl = b.triangles[al];
            VertexIndex p1 = b.triangles[bl];

            Vec2f v0 = { positions[(int)p0].x, positions[(int)p0].y };
            Vec2f vr = { positions[(int)pr].x, positions[(int)pr].y };
            Vec2f vl = { positions[(int)pl].x, positions[(int)pl].y };
            Vec2f v1 = { positions[(int)p1].x, positions[(int)p1].y };

            if (geometry::in_circle_2d(v0, vr, vl, v1) == geometry::PREDICATE_NEGATIVE) {
                b.triangles[he] = p1;
                b.triangles[a]  = p0;

                HeIndex hbl = b.halfedges[bl];
                HeIndex har = b.halfedges[ar];

                link_halfedges(b.halfedges, he, hbl);
                link_halfedges(b.halfedges, a, har);
                link_halfedges(b.halfedges, ar, bl);

                HeIndex br = next_halfedge(a);
                b.legalize_stack[stack_size++] = br;

                if (hbl == INVALID_HE) {
                    for (usz hi = 0; hi < 3 * b.face_count; hi++) {
                        if (b.hull_tri[(int)b.triangles[hi]] == bl) {
                            b.hull_tri[(int)b.triangles[hi]] = he;
                            break;
                        }
                    }
                }
            }
        }

        if (stack_size > 0) {
            he = b.legalize_stack[--stack_size];
        } else {
            break;
        }
    }
    return ar;
}
}
```

**Step 4: Add the full insertion loop**

After hull init, before free+return, insert the insertion loop:

```c3
    // ── Phase 3: point insertion (forward walk only for now) ──
    for (usz si = 0; si < unique_count; si++) {
        int p = indices[si];
        if (p == i0 || p == i1 || p == i2) continue;

        // find_visible_edge(p) — see spec for full implementation.
        // Hash p to find starting hull vertex, advance CCW.
        // Find edge e where orient_2d(work_pos[e], work_pos[next[e]], work_pos[p]) < 0.
        // If none found (inside hull), skip.

        // Forward walk + add_triangle — see spec.
        // t = add_triangle(&builder, e, p, hull_next[e], INVALID_HE, INVALID_HE, hull_tri[e]);
        // hull_tri[p] = legalize(t + 2, work_pos, &builder);
        // Advance curr = hull_next[e], add triangles while orient_2d < 0.
    }
```

The full Phase 3 implementation follows the spec's pseudocode for `find_visible_edge`, forward walk, backward walk (if backwards), hull link update, hash update, and marking removed vertices. See `docs/specs/2026-05-21-m9v2-delaunator-design.md` §Phase 3 for the complete code.

**Step 3: Build & test**

```bash
c3c build debug && c3c test
```

Expected: 169 passed.

**Step 6: Commit**

```bash
git add src/delaunay/delaunay_2d_v2.c3
git commit -m "delaunay: add v2 insertion loop + legalize"
```

---

### Task 4: Phase 6 — Mesh finalization + swap (GREEN)

**Objective:** Add mesh finalization. Run full test suite against v2. Swap into production file when all pass.

**Files:**
- Modify: `src/delaunay/delaunay_2d_v2.c3` (add finalization, remove stub return)
- Modify: `test/test_delaunay_2d.c3` (temporarily use v2)
- Modify: `src/delaunay/delaunay_2d.c3` (final swap)

**Step 1: Add Phase 6 finalization**

Replace the `return cg::DEGENERATE_INPUT~` stub with:

```c3
    HalfEdgeMesh mesh;

    mesh.half_edges = mem::alloc::new_array(alloc, HalfEdge, (sz)(3 * builder.face_count));
    defer catch free(mesh.half_edges);
    mesh.faces = mem::alloc::new_array(alloc, HalfEdgeFace, (sz) builder.face_count);
    defer catch free(mesh.faces);
    mesh.vertices = mem::alloc::new_array(alloc, HalfEdgeVertex, (sz) unique_count);
    defer catch free(mesh.vertices);
    mesh.positions = mem::alloc::new_array(alloc, Vec3f, (sz) unique_count);
    defer catch free(mesh.positions);

    // Populate mesh from builder (see spec Phase 6) ...

    mesh.validate()!;

    // On success: defer catch does NOT fire. Builder/scratch freed by their defer free.
    // On validate fault: defer catch frees mesh arrays. defer free frees builder/scratch.
    return mesh;
```

**Memory pattern**:
- Scratch + builder arrays: `defer free()` (unconditional — always cleaned up)
- Mesh arrays: `defer catch free()` (only on fault — caller owns them on success)
- No explicit `free()` calls — `defer` handles everything.

```c3
    // ── Phase 6: mesh finalization ──
    HalfEdgeMesh mesh;

    mesh.half_edges = mem::alloc::new_array(alloc, HalfEdge, (sz)(3 * builder.face_count));
    defer catch free(mesh.half_edges);
    mesh.faces = mem::alloc::new_array(alloc, HalfEdgeFace, (sz) builder.face_count);
    defer catch free(mesh.faces);
    mesh.vertices = mem::alloc::new_array(alloc, HalfEdgeVertex, (sz) unique_count);
    defer catch free(mesh.vertices);
    mesh.positions = mem::alloc::new_array(alloc, Vec3f, (sz) unique_count);
    defer catch free(mesh.positions);

    for (usz i = 0; i < unique_count; i++) mesh.positions[i] = work_pos[i];

    for (usz i = 0; i < unique_count; i++) mesh.vertices[i].half_edge = INVALID_HE;
    for (usz i = 0; i < 3 * builder.face_count; i++) {
        VertexIndex v = builder.triangles[i];
        if (mesh.vertices[(int)v].half_edge == INVALID_HE) {
            mesh.vertices[(int)v].half_edge = (HeIndex)(int)i;
        }
    }

    for (usz i = 0; i < 3 * builder.face_count; i++) {
        mesh.half_edges[i].origin = builder.triangles[i];
        mesh.half_edges[i].twin   = builder.halfedges[i];
        mesh.half_edges[i].face   = (FaceIndex)(int)(i / 3);
        if (i % 3 == 2) mesh.half_edges[i].next = (HeIndex)(int)(i - 2);
        else            mesh.half_edges[i].next = (HeIndex)(int)(i + 1);
    }

    for (usz i = 0; i < builder.face_count; i++) {
        mesh.faces[i].half_edge = (HeIndex)(int)(3 * i);
    }

    mesh.normals = {};
    mesh.uvs = {};

    mesh.validate()!;

    free(builder.triangles); free(builder.halfedges); free(builder.hull_tri);
    free(builder.hull_prev); free(builder.hull_next); free(builder.hull_hash);
    free(builder.legalize_stack);
    free(work_pos);
    free(indices);

    return mesh;
```

**Step 2: Run full v2 test suite**

In `test/test_delaunay_2d_v2.c3`, add copies of the existing Delaunay tests with `_v2` suffix on function names and calling `v2::delaunay_2d_v2`. This avoids name collisions with the original `module test` functions.

```bash
c3c build debug && c3c test
```

Expected: all v2 tests pass.

**Step 3: Swap into production**

1. Delete `src/delaunay/delaunay_2d.c3` (the old implementation).
2. Rename `src/delaunay/delaunay_2d_v2.c3` → `src/delaunay/delaunay_2d.c3`.
3. In the renamed file: change `module cg::delaunay::v2;` → `module cg::delaunay;`, change `fn HalfEdgeMesh? delaunay_2d_v2` → `fn HalfEdgeMesh? delaunay_2d`.
4. Update the test file: change `v2::delaunay_2d_v2` → `delaunay::delaunay_2d`.
5. Delete `test/test_delaunay_2d_v2.c3`, restore the original `test/test_delaunay_2d.c3` (which already calls `delaunay::delaunay_2d`).

Run full test suite:

```bash
c3c build debug && c3c test
```

Expected: 168 passed.

**Step 4: Commit**

```bash
git add src/delaunay/delaunay_2d.c3
git rm src/delaunay/delaunay_2d_v2.c3 test/test_delaunay_2d_v2.c3
git commit -m "delaunay: swap v2 delaunator into production"
```

---

### Task 6: Final verification

**Step 1: Clean release build**

```bash
c3c clean && c3c build release
```

**Step 2: Full test suite**

```bash
c3c test
```

Expected: 168 passed.

**Step 3: Commit if fixes made**

```bash
git add src/delaunay/delaunay_2d.c3
git commit -m "delaunay: final verification pass"
```
