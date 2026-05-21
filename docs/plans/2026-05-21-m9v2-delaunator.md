# M9v2 — Delaunay 2D (delaunator) Implementation Plan

> **For Hermes:** Use subagent-driven-development skill to implement this plan task-by-task.

**Goal:** Rewrite `src/delaunay/delaunay_2d.c3` with the delaunator algorithm. Same public API, same tests must pass.

**Spec:** `docs/specs/2026-05-21-m9v2-delaunator-design.md` — this spec contains complete pseudocode for every phase. The implementer follows it directly. **For memory ownership, this plan supersedes the spec**: scratch arrays use unconditional `defer free()`, mesh arrays use `defer catch free()`. No explicit `free()` calls.

**Tech Stack:** C3 0.8.0, `cg`, `cg::geometry`, `cg::half_edge`, `std::sort`, `std::math`.

---

### Task 1: Rewrite delaunay_2d.c3

**Objective:** Replace the entire `src/delaunay/delaunay_2d.c3` with the delaunator implementation from the spec. Iterate until all tests pass.

**Files:**
- Modify: `src/delaunay/delaunay_2d.c3`

**Step 1 (RED): Write a new test that will fail with a Bowyer-Watson implementation but pass with delaunator.**

Add to `test/test_delaunay_2d.c3`:

```c3
fn void test_delaunay_2d_timing_scale() @test
{
    // 500-point timing check — delaunator should handle this in < 5ms (not 1.3ms O(n²) Bowyer-Watson).
    // This test verifies the algorithm change actually took effect.
    Vec3f[500] pts;
    for (int i = 0; i < 500; i++) {
        pts[i] = {
            (float)((i * 2654435761u) % 1000000) * 0.001f,
            (float)((i * 1737350767u) % 1000000) * 0.001f,
            0,
        };
    }
    HalfEdgeMesh mesh = delaunay::delaunay_2d(mem, pts[..])!!;
    defer mesh.destroy();
    mesh.validate()!!;
    // Delaunay property: no point inside any triangle's circumcircle.
    VertexIndex[3] verts;
    for (FaceIndex f = (FaceIndex)0; (int)f < (int)mesh.faces.len; f++) {
        mesh.face_vertices(f, verts[..])!!;
        for (usz j = 0; j < pts.len; j++) {
            Vec3f a = mesh.positions[(int)verts[0]];
            Vec3f b = mesh.positions[(int)verts[1]];
            Vec3f c = mesh.positions[(int)verts[2]];
            Vec3f p = pts[j];
            if ((a.x == p.x && a.y == p.y) || (b.x == p.x && b.y == p.y) || (c.x == p.x && c.y == p.y)) continue;
            Vec2f a2 = { a.x, a.y }; Vec2f b2 = { b.x, b.y };
            Vec2f c2 = { c.x, c.y }; Vec2f p2 = { p.x, p.y };
            assert(geometry::in_circle_2d(a2, b2, c2, p2) != geometry::PREDICATE_POSITIVE);
        }
    }
}
```

Run: `c3c build debug && c3c test` — this test PASSES with current implementation (Delaunay property holds regardless of algorithm). The RED comes when we switch to delaunator — if the implementation has bugs, this test catches them.

**Step 2: Rewrite the file**

The spec at `docs/specs/2026-05-21-m9v2-delaunator-design.md` contains complete pseudocode. Follow these sections in order:

1. **Module + imports**: `cg`, `cg::geometry`, `cg::half_edge`, `std::sort`, `std::math`.
2. **Constants**: `DELAUNAY_NEAR_DUP_EPSILON = 1e-12f`.
3. **`DelaunayBuilder` struct** — preallocated flat arrays.
4. **Helpers**: `next_halfedge`, `prev_halfedge`, `fast_mod`, `link_halfedges`, `pseudo_angle`, `circumradius_sq`, `find_closest`, `compare_by_distance`.
5. **Phase 0**: Dedup, collinearity scan, bounding box validation, working positions. Arrays use `defer free()` (unconditional).
6. **Phase 1**: Seed triangle (bbox center → closest → closest → smallest circumradius). Distance sort.
7. **Phase 2**: Allocate builder arrays. Init hull linked list + hash. Hash init: overwrite on collision (no probing with `hash_len >= 1` and small vertex count).
8. **Phase 3**: Point insertion loop with `find_visible_edge`, forward/backward walks, `add_triangle`, hull link update, hash updates.
9. **Phase 5**: `legalize()` following spec §Phase 5 exactly.
10. **Phase 6**: Mesh finalization. Mesh arrays use `defer catch free()` (only on fault). Scratch arrays use `defer free()` (unconditional). No explicit free calls.

**Build & iterate:**

```bash
c3c build debug && c3c test
```

Debug until all tests pass. Then run full verification:

```bash
c3c clean && c3c build release
c3c test
```

**Style check:** snake_case, PascalCase, SCREAMING_SNAKE_CASE. Constants before structs before functions. `mem::alloc::new_array(alloc, T, (sz) count)` + `defer free` (scratch) / `defer catch free` (mesh). No runtime `assert()`, no `return null`. `module cg::delaunay;` first line. No milestone references.

**Commit (single commit — all verification passes):**

```bash
git add src/delaunay/delaunay_2d.c3 test/test_delaunay_2d.c3
git commit -m "delaunay: rewrite with delaunator algorithm"
```

## Memory pattern

```
Scratch arrays (builder, indices, work_pos):
  defer free(array);    // unconditional — freed on success AND fault

Mesh arrays (half_edges, faces, vertices, positions):
  defer catch free(array);  // only on fault — caller owns on success
```

No explicit `free()` calls. Defer handles all cleanup.
