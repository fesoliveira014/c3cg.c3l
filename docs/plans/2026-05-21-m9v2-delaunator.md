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

**Implementation guide:**

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

Debug until all 168 tests pass. No intermediate commits — single commit when all pass.

**Commit:**

```bash
git add src/delaunay/delaunay_2d.c3
git commit -m "delaunay: rewrite with delaunator algorithm"
```

---

### Task 2: Final verification

**Step 1:** `c3c clean && c3c build release`

**Step 2:** `c3c test` — all 168 pass.

**Step 3:** Style check: snake_case, PascalCase, SCREAMING_SNAKE_CASE. Constants before structs before functions. `mem::alloc::new_array(alloc, T, (sz) count)` + `defer free` (scratch) / `defer catch free` (mesh). No runtime `assert()`, no `return null`. `module cg::delaunay;` first line. No milestone references.

**Step 4:** Commit if fixes made.

---

## Memory pattern

```
Scratch arrays (builder, indices, work_pos):
  defer free(array);    // unconditional — freed on success AND fault

Mesh arrays (half_edges, faces, vertices, positions):
  defer catch free(array);  // only on fault — caller owns on success
```

No explicit `free()` calls. Defer handles all cleanup.
