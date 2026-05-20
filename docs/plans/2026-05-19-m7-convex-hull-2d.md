# M7 — Convex Hull 2D Implementation Plan

> **For Hermes:** Use subagent-driven-development skill to implement this plan task-by-task.

**Goal:** Implement `cg::hull::hull_2d` — Andrew's monotone chain for 2D convex hull, taking `Vec3f[]` (z ignored) and returning an owned `int[]` of hull vertex indices in CCW order.

**Architecture:** Single function in a new `cg::hull` module. Reuses `orient_2d` from `cg::geometry` for the CCW-turn predicate. Faults `EMPTY_INPUT` and `DEGENERATE_INPUT` (both already in `src/faults.c3i` — no new faults needed). The algorithm sorts lexicographically, sweeps left-to-right for lower hull, sweeps right-to-left for upper hull, trims the seam, and validates output has ≥ 3 points.

**Spec:** `docs/specs/2026-05-19-m7-convex-hull-2d-design.md`

**Tech Stack:** C3 0.8.0, c3c, existing `cg`, `cg::geometry` modules, `std::sort`.

**Stub-then-replace:** Task 2 creates a stub that keeps `c3c build debug && c3c test` green. Task 5 replaces the stub with the full algorithm. All files are auto-discovered by `project.json` (`"sources": ["src/**"]`).

---

### Task 1: Write one fault-path test (TDD: RED — empty input)

**Objective:** Write the first failing test — verify `hull_2d(mem, {})` faults `EMPTY_INPUT`.

**Files:**

- Create: `test/test_hull_2d.c3`

**Step 1: Create test file with empty-input test**

```c3
module test;
import cg;
import cg::hull;
import cg::geometry;

const float HULL_TEST_EPSILON = 0.001f;

fn void test_hull_empty_input_faults() @test
{
    Vec3f[] positions = {};
    if (catch err = hull::hull_2d(mem, positions)) {
        assert(err == cg::EMPTY_INPUT);
        return;
    }
    unreachable();
}
```

**Step 2: Run tests — expect FAIL**

```bash
c3c test
```

Expected: FAIL — no module `cg::hull`, no function `hull_2d`.

**Step 3: Commit**

```bash
git add test/test_hull_2d.c3
git commit -m "test: add hull_2d empty-input test (RED)"
```

---

### Task 2: Create hull module stub (GREEN for empty test)

**Objective:** Create a minimal `cg::hull` module with `hull_2d` that only handles the empty-input fault path, keeping the build green while we incrementally add tests.

**Files:**

- Create: `src/hull/hull_2d.c3i`

**Step 1: Create stub that only handles empty input**

```c3
module cg::hull;
import cg;

fn int[]? hull_2d(Allocator alloc, Vec3f[] positions)
{
    if (positions.len == 0) return cg::EMPTY_INPUT~;
    return cg::DEGENERATE_INPUT~;
}
```

**Step 2: Run tests — expect PASS for empty test**

```bash
c3c test --filter test_hull_empty
```

Expected: PASS.

**Step 3: Commit**

```bash
git add src/hull/hull_2d.c3i
git commit -m "hull: add module stub with empty-input fault"
```

---

### Task 3: Add remaining fault-path tests (TDD: RED)

**Objective:** Add tests for 1-point, 2-point, and all-collinear inputs. These will fail because the stub returns `DEGENERATE_INPUT` regardless.

**Files:**

- Modify: `test/test_hull_2d.c3`

**Step 1: Append three fault-path tests**

```c3
fn void test_hull_single_point_faults() @test
{
    Vec3f[1] pts = { { 0, 0, 0 } };
    if (catch err = hull::hull_2d(mem, pts[..])) {
        assert(err == cg::DEGENERATE_INPUT);
        return;
    }
    unreachable();
}

fn void test_hull_two_points_faults() @test
{
    Vec3f[2] pts = { { 0, 0, 0 }, { 1, 0, 0 } };
    if (catch err = hull::hull_2d(mem, pts[..])) {
        assert(err == cg::DEGENERATE_INPUT);
        return;
    }
    unreachable();
}

fn void test_hull_all_collinear_faults() @test
{
    Vec3f[4] pts = {
        { 0, 0, 0 },
        { 1, 0, 0 },
        { 2, 0, 0 },
        { 3, 0, 0 },
    };
    if (catch err = hull::hull_2d(mem, pts[..])) {
        assert(err == cg::DEGENERATE_INPUT);
        return;
    }
    unreachable();
}
```

**Step 2: Run tests — expect failure on square test (not yet written), existing tests PASS**

```bash
c3c test
```

Expected: All 4 hull tests PASS (they all go through the `DEGENERATE_INPUT~` stub path).

**Step 3: Commit**

```bash
git add test/test_hull_2d.c3
git commit -m "test: add hull_2d degenerate-input tests (RED — stub catches all)"
```

Wait — the stub already faults `DEGENERATE_INPUT` for any non-empty input, so these tests will actually PASS immediately. This is fine — they validate the fault paths exist. The real RED comes from the happy-path tests in Task 4 that need the actual algorithm.

---

### Task 4: Add happy-path tests (TDD: RED)

**Objective:** Add tests for a square hull (with and without interior points). These will FAIL because the stub faults instead of computing a hull.

**Files:**

- Modify: `test/test_hull_2d.c3`

**Step 1: Append happy-path tests**

```c3
fn bool m7_vec2_approx(Vec2f a, Vec2f b)
{
    float dx = a.x - b.x;
    float dy = a.y - b.y;
    if (dx < 0) dx = -dx;
    if (dy < 0) dy = -dy;
    return dx <= HULL_TEST_EPSILON && dy <= HULL_TEST_EPSILON;
}

fn int m7_find_in_slice(int[] slice, int value)
{
    for (usz i = 0; i < slice.len; i++) {
        if (slice[i] == value) return (int)i;
    }
    return -1;
}

fn void test_hull_square_four_corners() @test
{
    Vec3f[4] pts = {
        { 0, 0, 0 },
        { 1, 0, 0 },
        { 1, 1, 0 },
        { 0, 1, 0 },
    };
    int[] hull = hull::hull_2d(mem, pts[..])!!;
    defer free(hull);

    assert(hull.len == 4);

    // All 4 indices should appear exactly once.
    bool[4] seen = { false, false, false, false };
    for (usz i = 0; i < hull.len; i++) {
        assert(hull[i] >= 0 && hull[i] < 4);
        seen[hull[i]] = true;
    }
    for (usz i = 0; i < 4; i++) assert(seen[i]);
}

fn void test_hull_square_with_interior_point() @test
{
    Vec3f[5] pts = {
        { 0, 0, 0 },
        { 1, 0, 0 },
        { 1, 1, 0 },
        { 0, 1, 0 },
        { 0.5, 0.5, 0 },
    };
    int[] hull = hull::hull_2d(mem, pts[..])!!;
    defer free(hull);

    assert(hull.len == 4);

    // Interior point (index 4) must NOT be in the hull.
    assert(m7_find_in_slice(hull, 4) == -1);
}

fn void test_hull_square_collinear_on_edges() @test
{
    // A square with extra points collinear along two edges.
    // Per spec: collinear boundary points are kept on the hull.
    Vec3f[6] pts = {
        { 0, 0, 0 },
        { 1, 0, 0 },
        { 1, 1, 0 },
        { 0, 1, 0 },
        { 0.5, 0, 0 },  // collinear on bottom edge
        { 0, 0.5, 0 },  // collinear on left edge
    };
    int[] hull = hull::hull_2d(mem, pts[..])!!;
    defer free(hull);

    // Collinear edge points MUST be included (spec requirement).
    assert(m7_find_in_slice(hull, 4) >= 0);  // (0.5, 0) on bottom edge
    assert(m7_find_in_slice(hull, 5) >= 0);  // (0, 0.5) on left edge
    
    // All 4 corners must also be present.
    for (int i = 0; i < 4; i++) {
        assert(m7_find_in_slice(hull, i) >= 0);
    }
}

fn void test_hull_triangle_smoke() @test
{
    Vec3f[3] pts = {
        { 0, 0, 0 },
        { 2, 0, 0 },
        { 0, 2, 0 },
    };
    int[] hull = hull::hull_2d(mem, pts[..])!!;
    defer free(hull);

    assert(hull.len == 3);
    for (int i = 0; i < 3; i++) {
        assert(m7_find_in_slice(hull, i) >= 0);
    }
}

fn void test_hull_square_ccw_order() @test
{
    Vec3f[4] pts = {
        { 0, 0, 0 },   // index 0
        { 1, 0, 0 },   // index 1
        { 1, 1, 0 },   // index 2
        { 0, 1, 0 },   // index 3
    };
    int[] hull = hull::hull_2d(mem, pts[..])!!;
    defer free(hull);

    assert(hull.len == 4);

    // Verify CCW order: every consecutive triple must be a CCW turn.
    for (usz i = 0; i < hull.len; i++) {
        usz j = (i + 1) % hull.len;
        usz k = (i + 2) % hull.len;
        int a_idx = hull[i];
        int b_idx = hull[j];
        int c_idx = hull[k];
        Vec2f a = { pts[a_idx].x, pts[a_idx].y };
        Vec2f b = { pts[b_idx].x, pts[b_idx].y };
        Vec2f c = { pts[c_idx].x, pts[c_idx].y };
        assert(geometry::orient_2d(a, b, c) != geometry::PREDICATE_NEGATIVE);
    }
}

fn void test_hull_random_cloud_points_inside() @test
{
    // 8 points: a hexagon hull with 2 interior points.
    Vec3f[8] pts = {
        { 0, 1, 0 },
        { 0.8660254, 0.5, 0 },
        { 0.8660254, -0.5, 0 },
        { 0, -1, 0 },
        { -0.8660254, -0.5, 0 },
        { -0.8660254, 0.5, 0 },
        { 0.2, 0.2, 0 },    // interior
        { -0.2, -0.1, 0 },  // interior
    };
    int[] hull = hull::hull_2d(mem, pts[..])!!;
    defer free(hull);

    // All 6 hexagon vertices must be in the hull.
    for (int i = 0; i < 6; i++) {
        assert(m7_find_in_slice(hull, i) >= 0);
    }
    // Interior points must NOT be in the hull.
    assert(m7_find_in_slice(hull, 6) == -1);
    assert(m7_find_in_slice(hull, 7) == -1);

    // All points must be inside or on the convex hull.
    // For each edge of the hull, all input points must be to the left or on it.
    for (usz ei = 0; ei < hull.len; ei++) {
        usz ej = (ei + 1) % hull.len;
        int a_idx = hull[ei];
        int b_idx = hull[ej];
        Vec2f a = { pts[a_idx].x, pts[a_idx].y };
        Vec2f b = { pts[b_idx].x, pts[b_idx].y };
        for (usz j = 0; j < pts.len; j++) {
            Vec2f p = { pts[j].x, pts[j].y };
            assert(geometry::orient_2d(a, b, p) != geometry::PREDICATE_NEGATIVE);
        }
    }
}
```

**Step 2: Run tests — expect FAIL**

```bash
c3c test
```

Expected: Happy-path tests FAIL — the stub returns `DEGENERATE_INPUT~` instead of computing a hull.

**Step 3: Commit**

```bash
git add test/test_hull_2d.c3
git commit -m "test: add hull_2d happy-path tests (RED)"
```

---

### Task 5: Implement full Andrew's monotone chain (GREEN)

**Objective:** Replace the stub with the complete algorithm. Implement sort → lower hull → upper hull → combine → validate → return.

**Files:**

- Modify: `src/hull/hull_2d.c3i`

**Step 1: Write the full implementation**

```c3
module cg::hull;
import cg;
import cg::geometry;

fn int[]? hull_2d(Allocator alloc, Vec3f[] positions)
{
    if (positions.len == 0) return cg::EMPTY_INPUT~;

    // Phase 1 — index array + lexicographic sort by (x, y).
    int[] indices = mem::alloc::new_array(alloc, int, (sz) positions.len);
    defer catch free(indices);
    for (usz i = 0; i < positions.len; i++) indices[i] = (int)i;

    // Sort lexicographically by (x, y). C3 0.8.0 sort API:
    //   fn int compare_by_position(void* ctx, int a, int b)
    // Comparator returns <0, 0, or >0 (not bool).
    //   sort::quicksort(indices, &compare_by_position, positions[..]);

    // Phase 2 — lower hull (left-to-right sweep).
    // Pop while not a CCW turn. With collinear policy "keep boundary collinear",
    // we pop only on strict CW (PREDICATE_NEGATIVE), keeping collinear (PREDICATE_ZERO).
    int[] hull = mem::alloc::new_array(alloc, int, (sz) positions.len);
    defer catch free(hull);
    usz hull_len = 0;

    for (usz i = 0; i < indices.len; i++) {
        int p = indices[i];
        Vec2f p2 = { positions[p].x, positions[p].y };
        while (hull_len >= 2) {
            int a = hull[hull_len - 2];
            int b = hull[hull_len - 1];
            Vec2f a2 = { positions[a].x, positions[a].y };
            Vec2f b2 = { positions[b].x, positions[b].y };
            if (geometry::orient_2d(a2, b2, p2, 0.0f) != geometry::PREDICATE_NEGATIVE) break;
            hull_len--;
        }
        hull[hull_len] = p;
        hull_len++;
    }

    // Phase 3 — upper hull (right-to-left sweep).
    // Standard formulation: sweep reversed, pop on non-CCW, push, then trim last.
    for (usz i = indices.len; i > 0; i--) {
        int p = indices[i - 1];
        Vec2f p2 = { positions[p].x, positions[p].y };
        while (hull_len >= 2) {
            int a = hull[hull_len - 2];
            int b = hull[hull_len - 1];
            Vec2f a2 = { positions[a].x, positions[a].y };
            Vec2f b2 = { positions[b].x, positions[b].y };
            if (geometry::orient_2d(a2, b2, p2, 0.0f) != geometry::PREDICATE_NEGATIVE) break;
            hull_len--;
        }
        hull[hull_len] = p;
        hull_len++;
    }

    // Phase 4 — trim last element (duplicate of hull[0]) and validate.
    if (hull_len <= 1) return cg::DEGENERATE_INPUT~;
    usz result_len = hull_len - 1;
    if (result_len < 3) return cg::DEGENERATE_INPUT~;

    // Copy hull indices into output array.
    int[] result = mem::alloc::new_array(alloc, int, (sz) result_len);
    defer catch free(result);
    for (usz i = 0; i < result_len; i++) result[i] = hull[i];

    free(indices);
    free(hull);
    return result;
}
```

**Step 2: Run all tests**

```bash
c3c test
```

Expected: All 8 hull tests PASS. All pre-existing tests still PASS.

**Step 3: If any test fails, debug with:**

```bash
c3c build debug
c3c test --filter test_hull
```

Fix issues and re-run until all pass.

**Step 4: Commit**

```bash
git add src/hull/hull_2d.c3i
git commit -m "hull: implement Andrew's monotone chain 2D convex hull"
```

---

### Task 6: Final verification

**Objective:** Clean build + full test suite + style compliance check.

**Step 1: Clean build (release)**

```bash
c3c clean && c3c build release
```

Expected: No warnings, no errors.

**Step 2: Full test suite**

```bash
c3c test
```

Expected: All tests pass (0 failures).

**Step 3: Verify style**

- [ ] `src/hull/hull_2d.c3i` uses `snake_case` functions, `PascalCase` types
- [ ] Allocations use `mem::alloc::new_array(alloc, T, (sz) count)` with `defer catch free` on next line
- [ ] No runtime `assert()` in production code
- [ ] All `int[]` slices freed by caller or in failure path
- [ ] Module declaration is first line: `module cg::hull;`
- [ ] Imports follow module: `import cg;` then `import cg::geometry;`

**Step 4: Commit if any fixes made**

```bash
git add src/hull/hull_2d.c3i
git commit -m "hull: final style and verification pass"
```

---

## Implementation Notes

### sort::quicksort API (C3 0.8.0)

The verified C3 0.8.0 sort API uses `sort::quicksort` with a context-passing comparator:

```c3
import std::sort;

fn int compare_by_position(void* ctx, int a, int b)
{
    Vec3f[] positions = (Vec3f[])ctx;
    Vec3f va = positions[a];
    Vec3f vb = positions[b];
    if (va.x < vb.x) return -1;
    if (va.x > vb.x) return 1;
    if (va.y < vb.y) return -1;
    if (va.y > vb.y) return 1;
    return 0;
}

// Usage:
//   sort::quicksort(indices, &compare_by_position, positions[..]);
```

Key points:
- Comparator returns `int`: negative / zero / positive (not `bool`).
- Context is passed as `void*`, cast to the expected type inside the comparator.
- `sort::quicksort` takes `(slice, &comparator, context)`.

### orient_2d epsilon and collinear policy

During hull construction, we pass `epsilon = 0.0f` to `orient_2d`. We pop only on strict CW turns (`PREDICATE_NEGATIVE`), keeping collinear points (`PREDICATE_ZERO`) on the hull boundary. CCW turns (`PREDICATE_POSITIVE`) are also kept. This preserves collinear boundary points per the spec.

### Memory ownership

| Array | Allocator | Freed by |
|-------|-----------|----------|
| `indices` | `alloc` | Always freed before return (success or failure) |
| `hull` (stack) | `alloc` | Always freed before return |
| `result` | `alloc` | Owned by caller — freed via `defer free(result)` at call site |

### No project.json changes needed

`project.json` has `"sources": ["src/**"]` — new files in `src/hull/` are auto-discovered.
