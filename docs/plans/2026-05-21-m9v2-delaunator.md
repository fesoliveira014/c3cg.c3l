# M9v2 — Delaunay 2D (delaunator) Implementation Plan

> **For Hermes:** Use subagent-driven-development skill to implement this plan task-by-task.

**Goal:** Rewrite `cg::delaunay::delaunay_2d` with the delaunator algorithm. Same public API, same 12 tests must pass.

**Architecture:** `DelaunayBuilder` struct with preallocated flat arrays. Seed triangle + hull tracking + distance-sorted insertion + stack-based `legalize()`. Mesh built manually in finalization phase — no `from_triangles`, no `HalfEdgeMesh` mutation during insertion.

**Spec:** `docs/specs/2026-05-21-m9v2-delaunator-design.md`

**Tech Stack:** C3 0.8.0, existing `cg`, `cg::geometry`, `std::sort`, `std::math`, `std::collections::map` (no longer needed — removed from imports).

**Green commits only.** Each task ends with `c3c build debug && c3c test` passing.

---

### Task 1: Add import + replace stub with skeleton (GREEN)

**Objective:** Update imports, add needed `import cg::half_edge;` for `mesh.validate()`, replace the function body with a skeleton that still returns `DEGENERATE_INPUT`. All existing tests still pass.

**Files:**
- Modify: `src/delaunay/delaunay_2d.c3`

**Step 1: Replace file**

```c3
module cg::delaunay;
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

fn HalfEdgeMesh? delaunay_2d(Allocator alloc, Vec3f[] positions, DelaunayOptions options = {})
{
    if (positions.len == 0) return cg::EMPTY_INPUT~;
    return cg::DEGENERATE_INPUT~;
}
```

**Step 2: Build & test**

```bash
c3c build debug && c3c test
```

Expected: 168 passed (no behavior change yet).

**Step 3: Commit**

```bash
git add src/delaunay/delaunay_2d.c3
git commit -m "delaunay: add builder struct skeleton, import half_edge"
```

---

### Task 2: Phase 0 — Dedup + collinearity + working positions (GREEN)

**Objective:** Implement Phase 0: dedup `(x,y)`, collinearity scan, build `working_positions`. Function still returns `DEGENERATE_INPUT` after Phase 0 — existing tests still pass (they fault on <3 distinct or all-collinear).

**Files:**
- Modify: `src/delaunay/delaunay_2d.c3`

**Step 1: Replace the function body**

```c3
fn HalfEdgeMesh? delaunay_2d(Allocator alloc, Vec3f[] positions, DelaunayOptions options = {})
{
    if (positions.len == 0) return cg::EMPTY_INPUT~;

    // Deduplicate exact (x,y).
    int[] indices = mem::alloc::new_array(alloc, int, (sz) positions.len);
    defer catch free(indices);
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
    defer catch free(work_pos);
    for (usz i = 0; i < unique_count; i++) work_pos[i] = positions[indices[i]];

    free(work_pos);
    free(indices);
    return cg::DEGENERATE_INPUT~;
}
```

**Step 2: Build & test**

```bash
c3c build debug && c3c test
```

Expected: 168 passed (still DEGENERATE_INPUT for all non-empty inputs).

**Step 3: Commit**

```bash
git add src/delaunay/delaunay_2d.c3
git commit -m "delaunay: implement Phase 0 — dedup, collinearity, working positions"
```

---

### Task 3: Phase 1 — Seed triangle + distance sort (GREEN)

**Objective:** Implement seed triangle selection, distance sort. Helper: `circumradius_sq`. Still returns `DEGENERATE_INPUT` — existing tests pass.

**Files:**
- Modify: `src/delaunay/delaunay_2d.c3`

**Step 1: Add helpers and insert Phase 1 code after working_positions build (before the `free` + return)**

```c3
// Helper: squared circumradius, returns MAX on collinear.
fn float circumradius_sq(Vec3f a, Vec3f b, Vec3f c)
{
    if (catch err = geometry::circumcenter_planar(a, b, c)) return 1e30f;
    Vec3f cc = geometry::circumcenter_planar(a, b, c)!!;
    float dx = cc.x - a.x;
    float dy = cc.y - a.y;
    return dx * dx + dy * dy;
}

// Helper: find closest point to a reference position.
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
```

Insert after `work_pos` build in `delaunay_2d`:

```c3
    // Bbox center for seed triangle.
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

    // Orient CCW.
    Vec2f s0 = { work_pos[i0].x, work_pos[i0].y };
    Vec2f s1 = { work_pos[i1].x, work_pos[i1].y };
    Vec2f s2 = { work_pos[i2].x, work_pos[i2].y };
    if (geometry::orient_2d(s0, s1, s2) <= geometry::PREDICATE_ZERO) {
        int tmp = i1; i1 = i2; i2 = tmp;
    }

    // Seed circumcenter for distance sort.
    Vec3f seed_center = geometry::circumcenter_planar(work_pos[i0], work_pos[i1], work_pos[i2])!!;

    // Build distance-sort indices.
    for (usz i = 0; i < unique_count; i++) indices[i] = (int)i;

    struct DistSortCtx { Vec3f[] pts; Vec3f center; }
    DistSortCtx dctx = { work_pos, seed_center };
    sort::quicksort(indices, &compare_by_distance, dctx);
```

Add comparator before the function:

```c3
struct DistSortCtx { Vec3f[] pts; Vec3f center; }

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
```

**Step 2: Build & test**

```bash
c3c build debug && c3c test
```

Expected: 168 passed.

**Step 3: Commit**

```bash
git add src/delaunay/delaunay_2d.c3
git commit -m "delaunay: implement Phase 1 — seed triangle, distance sort"
```

---

### Task 4: Phase 2-5 — Builder + hull + insertion + legalize + finalization (GREEN)

**Objective:** Implement the full algorithm. This is the core task — all remaining phases. Because the algorithm is tightly coupled (hull init needs builder, insertion needs hull, legalize needs halfedges, finalization needs everything), these are implemented together.

**Files:**
- Modify: `src/delaunay/delaunay_2d.c3`

**Step 1: Add helpers**

```c3
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
    float p = dx / (dx < 0 ? -dx : dx + (dy < 0 ? -dy : dy));
    if (dy > 0.0f) return (3.0f - p) / 4.0f;
    return (1.0f + p) / 4.0f;
}
```

**Step 2: Replace the Phase 1 `free(work_pos); free(indices); return DEGENERATE_INPUT~;` with the full algorithm**

Follow the spec's Phase 2 (hull init), Phase 3 (point insertion loop with `find_visible_edge`, forward/backward walks, `add_triangle`, hull link update), Phase 5 (`legalize`), and Phase 6 (mesh finalization).

Key structure:
```
// Phase 2 — allocate builder arrays.
// Init hull linked list + hash.

// Phase 3 — point insertion loop.
for each p in sorted order (skip i0,i1,i2):
    (e, backwards) = find_visible_edge(p)
    if e == INVALID_VERTEX: continue
    
    t = add_triangle(e, p, hull_next[e], ...)
    hull_tri[p] = legalize(t + 2)
    // forward walk...
    // backward walk (if backwards)...
    // update hull links...

// Phase 6 — mesh finalization.
// Allocate mesh arrays, populate from builder.
// mesh.validate()!
// Free builder + scratch arrays.
// return mesh;
```

**Step 3: Build & test — iterate until all pass**

```bash
c3c build debug && c3c test
```

Expected: all tests pass. Debug iteration expected — this is a complex algorithm.

**Step 4: Commit**

```bash
git add src/delaunay/delaunay_2d.c3
git commit -m "delaunay: implement delaunator algorithm — hull tracking, edge flipping, finalization"
```

---

### Task 5: Final verification

**Step 1: Clean release build**

```bash
c3c clean && c3c build release
```

**Step 2: Full test suite**

```bash
c3c test
```

**Step 3: Style check**

- [ ] `snake_case`, `PascalCase`, `SCREAMING_SNAKE_CASE`
- [ ] `const` before `struct` before helpers before main function
- [ ] `mem::alloc::new_array(alloc, T, (sz) count)` + `defer catch free`
- [ ] No runtime `assert()`, no `return null`
- [ ] `module cg::delaunay;` first line
- [ ] No milestone references in names
- [ ] `validate()!` before explicit frees

**Step 4: Commit if fixes made**

```bash
git add src/delaunay/delaunay_2d.c3
git commit -m "delaunay: final verification pass"
```

---

## Implementation Notes

### Key C3 0.8.0 patterns

- Comparator for `sort::quicksort`: `fn int cmp(Type a, Type b, Context ctx)` — returns int, context last.
- `circumcenter_planar(a,b,c)` returns `Vec3f?` — use `!!` after collinearity guard.
- `HeIndex`, `VertexIndex` casts: `(HeIndex)(int)i`, `(VertexIndex)i`.
- `INVALID_HE`, `INVALID_VERTEX` sentinels from `cg` module.
- `mem::alloc::new_array(alloc, T, (sz) count)` with `defer catch free(array)`.

### Memory ownership

| Array | Cleanup |
|-------|---------|
| `indices` | Explicit free + defer catch |
| `work_pos` | Explicit free + defer catch |
| Builder arrays (6) | Explicit free + defer catch |
| `HalfEdgeMesh` | Caller via `mesh.destroy()`. defer catch on partial |

`validate()!` before explicit frees — defer catch handles fault cleanup.
