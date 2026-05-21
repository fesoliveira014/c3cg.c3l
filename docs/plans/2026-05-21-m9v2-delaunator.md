# M9v2 — Delaunay 2D (delaunator) Implementation Plan

> **For Hermes:** Use subagent-driven-development skill to implement this plan task-by-task.

**Goal:** Rewrite `cg::delaunay::delaunay_2d` with the delaunator algorithm (private builder, hull tracking, edge flipping). Same public API, same 12 tests must pass.

**Architecture:** `DelaunayBuilder` struct with preallocated flat arrays. Seed triangle + hull init, distance-sorted insertion, stack-based `legalize()`. Mesh built manually during finalization — no `from_triangles`, no `HalfEdgeMesh` mutation during insertion.

**Spec:** `docs/specs/2026-05-21-m9v2-delaunator-design.md`

**Tech Stack:** C3 0.8.0, existing `cg`, `cg::geometry`, `std::sort`, `std::math`.

**Green commits only.** Tests may fail during implementation but must pass at commit.

---

### Task 1: Rewrite delaunay_2d.c3 (the full implementation)

**Objective:** Replace the entire `src/delaunay/delaunay_2d.c3` with the delaunator implementation. This is a single-task rewrite — the spec provides complete pseudocode.

**Files:**
- Modify: `src/delaunay/delaunay_2d.c3`

**Implementation follows the spec sections:**

**Phase 0** — Dedup, collinearity check, working_positions, builder allocation.

**Phase 1** — Seed triangle: find i0 (closest to bbox center), i1 (closest to i0), i2 (smallest circumradius). `circumradius_sq` helper using `circumcenter_planar`. Distance sort with `sort::quicksort`.

**Phase 2** — Hull init: `hull_next`/`hull_prev` linked list, `hull_tri`, `hull_hash` via `pseudo_angle`. `hull_start` and `hull_center`.

**Phase 3** — Point insertion loop:
- `find_visible_edge(p)` → returns `(e, backwards)`
- Forward hull walk: `add_triangle`, `legalize`, advance
- Backward hull walk (if backwards): same
- Hull link update: insert `p`, update hash, mark removed vertices `INVALID_VERTEX`

**`add_triangle`** — append to `triangles[]`/`halfedges[]`, link twins.

**`legalize`** — stack-based, Lawson test (`in_circle_2d`), `link()` helper, hull update when `hbl == INVALID_HE`. Returns `ar`.

**Phase 6** — Mesh finalization:
- Allocate `mesh.half_edges`, `mesh.faces`, `mesh.vertices`, `mesh.positions`
- Copy `working_positions`
- Scan `triangles[]` for vertex half-edges
- Populate half-edge `origin`, `twin`, `next`, `face`
- Populate face `half_edge`
- `mesh.normals = {}`, `mesh.uvs = {}`
- `mesh.validate()!`
- Explicit free builder arrays

**Helpers (private):**
```
pseudo_angle(dx, dy) → float
circumradius_sq(a, b, c) → float (MAX on collinear)
find_closest_point(points, center) → VertexIndex
fast_mod(i, c) → usz
next_halfedge(i) → HeIndex
prev_halfedge(i) → HeIndex
link(halfedges[], a, b) → void
```

**Build & test:**

```bash
c3c build debug && c3c test
```

Expected: all 168 tests PASS (128 existing + 12 delaunay + 28 hull/geometry/dual/render).

**Commit:**

```bash
git add src/delaunay/delaunay_2d.c3
git commit -m "delaunay: rewrite with delaunator algorithm (hull tracking, edge flipping)"
```

---

### Task 2: Final verification

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
- [ ] `const` before `struct` before functions
- [ ] `mem::alloc::new_array(alloc, T, (sz) count)` + `defer catch free`
- [ ] No runtime `assert()`, no `return null`
- [ ] `module cg::delaunay;` first line
- [ ] No milestone references in names

**Step 4: Commit if fixes made**

```bash
git add src/delaunay/delaunay_2d.c3
git commit -m "delaunay: final verification pass"
```

---

## Implementation Notes

### Key data structures (all in builder)

| Array | Type | Size | Purpose |
|-------|------|------|---------|
| `triangles` | `VertexIndex[]` | `3 × (2n-5)` | Triangle vertex triples |
| `halfedges` | `HeIndex[]` | `3 × (2n-5)` | Twin links |
| `hull_tri` | `HeIndex[]` | `n` | Hull-adjacent half-edge per vertex |
| `hull_prev/next` | `VertexIndex[]` | `n` | Linked list, `INVALID_VERTEX` = removed/not-on-hull |
| `hull_hash` | `VertexIndex[]` | `max(1, √n)` | Spatial hash |
| `legalize_stack` | `HeIndex[]` | `2n-5` | Stack for recursive edge checking |

### C3 0.8.0 patterns

```c3
// HashMap not needed — direct indexed arrays for adjacency.
// sort::quicksort with context comparator (int return, context last param).
// Vec2f projection for predicates: { positions[i].x, positions[i].y }
// HeIndex, VertexIndex casts needed for array indexing.
// INVALID_VERTEX, INVALID_HE sentinels.
```

### Memory

No allocations in insertion loop. All builder arrays preallocated once. `validate()!` before explicit frees (defer catch handles fault cleanup).
