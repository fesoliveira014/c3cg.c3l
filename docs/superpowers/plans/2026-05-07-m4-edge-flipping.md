# M4 Edge Flipping Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add legal local edge flipping to `HalfEdgeMesh` with `is_flip_ok(he)` and `flip(he)`.

**Architecture:** Implement a topology-only six-half-edge rewrite in `src/half_edge/flip.c3`. Validate all public failure cases before mutation, keep `is_flip_ok` as a pure bool wrapper, and refresh only the four local vertex handles after a successful flip.

**Tech Stack:** C3 0.7.11, `c3c`, flat-array `HalfEdgeMesh`, `cg::half_edge` cross-module methods, C3 faults/optionals, existing test module conventions.

---

## Source Documents

Read these before implementing:

- Spec: `docs/specs/2026-05-07-m4-edge-flipping-design.md`
- Repo instructions: `AGENTS.md`
- Style guide: `docs/style.md`

Parent agent should load the external `c3-expert` skill before dispatching implementation subagents and include any needed C3 quickref/library-structure notes in subagent context. Those skill reference files are not repository files.

Project root:

```sh
/home/fesol/source/repos/c3cg.c3l
```

Recommended implementation worktree:

```sh
git worktree add .worktrees/m4-edge-flipping -b feat/m4-edge-flipping
cd .worktrees/m4-edge-flipping
```

Do not touch unrelated pre-existing untracked files in the main worktree:

- `.claude/`
- `.superpowers/`
- `LICENSE`
- `README.md`
- `docs/architecture.md`
- `docs/bindings_guidelines.md`
- `docs/style.md`
- `linked-libs/`
- `scripts/`

## Scope Check

This plan implements one focused subsystem: M4 edge flipping.

In scope:

- `HalfEdgeMesh.is_flip_ok(&self, HeIndex he) -> bool`
- `HalfEdgeMesh.flip(&self, HeIndex he) -> void?`
- local six-half-edge topology rewrite
- validation/faults for invalid, boundary, non-triangle, degenerate, duplicate-new-edge cases
- deterministic local vertex-handle refresh
- tests covering success, failure, flip-back, attributes, Euler counts

Out of scope:

- Delaunay geometric legality
- convexity/angle/length checks
- polygonal face flipping
- boundary flipping
- geometry/attribute mutation
- external rendering/index-buffer mutation
- global remeshing or repair of corrupted meshes

## Files and Responsibilities

### Create

`src/half_edge/flip.c3`

- Module: `cg::half_edge`
- Imports: `cg`
- Public methods:
  - `fn bool HalfEdgeMesh.is_flip_ok(&self, HeIndex he)`
  - `fn void? HalfEdgeMesh.flip(&self, HeIndex he)`
- Module-scope helpers:
  - index validation
  - validation function returning `void?`
  - edge-exists excluding two half-edge slots
  - local vertex-handle refresh
  - local vertex-handle setter/search helper

`test/test_flip.c3`

- Module: `test`
- Imports: `cg`, `cg::half_edge`
- Tests all M4 success/fault/preservation behavior
- Reuses existing same-module fixtures:
  - `TET_POSITIONS`, `TET_INDICES`
  - `TET_NORMALS`, `TET_UVS`
  - `DIAMOND_POSITIONS`, `DIAMOND_INDICES`
  - `TRI_POSITIONS`, `TRI_INDICES`
- Adds new uniquely named local fixtures/helpers only when needed

### Modify

`src/faults.c3i`

- Add `EDGE_ALREADY_EXISTS` near `DUPLICATE_HALF_EDGE`
- Keep the final fault in the block ending with `;`

`project.json`

- Add `src/half_edge/flip.c3` to `sources`
- Place it after `src/half_edge/validate.c3` and before render sources

`manifest.json`

- Add `src/half_edge/flip.c3` to `sources`
- Place it after `src/half_edge/validate.c3` and before render sources

## C3 Conventions to Preserve

- Do not add function bodies to `.c3i` files.
- Do not use `@private` on module-level helper functions.
- Do not use production `assert()` or `unreachable()` in `src/half_edge/flip.c3`.
- Use returned faults in production code.
- Use `.` for pointer and value member access; never use `->`.
- Do not import `std::mem`; existing code uses `mem`, `free`, and `mem::alloc::new_array` without that import.
- Tests may use `assert()`.
- In test functions returning `void`, do not use `!`; use `!!` for expected-success optionals.
- Fault comparisons outside module `cg` must use `cg::FAULT_NAME`.
- Same-module test files share namespace. Avoid defining constants/helpers with names already used in other `module test;` files.

## Implementation Design Details

### Public API

```c3
fn bool HalfEdgeMesh.is_flip_ok(&self, HeIndex he)
{
    if (catch err = validate_flip(self, he)) {
        return false;
    }
    return true;
}

fn void? HalfEdgeMesh.flip(&self, HeIndex he)
{
    validate_flip(self, he)!;
    // collect locals again or collect inside a local struct-free helper pattern
    // apply rewire table
    // refresh local vertex handles
    return;
}
```

Implementation note:

- C3 has no need for an allocation or object to share validation output.
- It is acceptable for `flip` to recompute local variables after `validate_flip`; repeated reads are cheap and keep validation side-effect-free.
- If implementation uses a local `struct FlipLocal`, declare it in `src/half_edge/flip.c3` after any constants and before functions, following project definition order. Do not over-engineer if local variables are clear.

### Local Names

For `h0 = he`:

```text
h1 = h0.next
h2 = h1.next

t0 = h0.twin
t1 = t0.next
t2 = t1.next

f0 = h0.face
f1 = t0.face

a = h0.origin
b = t0.origin
c = h2.origin
d = t2.origin
```

Before:

```text
f0: h0(a -> b) -> h1(b -> c) -> h2(c -> a)
f1: t0(b -> a) -> t1(a -> d) -> t2(d -> b)
```

After:

```text
f0: h0(c -> d) -> t2(d -> b) -> h1(b -> c)
f1: t0(d -> c) -> h2(c -> a) -> t1(a -> d)
```

Field writes:

```text
h0.origin = c
t0.origin = d

h0.next = t2
t2.next = h1
h1.next = h0

t0.next = h2
h2.next = t1
t1.next = t0

h0.face = f0
t2.face = f0
h1.face = f0

t0.face = f1
h2.face = f1
t1.face = f1

faces[f0].half_edge = h0
faces[f1].half_edge = t0
```

Twin links are not changed.

### Validation Helper Shape

Recommended helper:

```c3
fn void? validate_flip(HalfEdgeMesh* mesh, HeIndex he)
```

Validation order:

1. Invalid half-edge index -> `cg::INVALID_HALF_EDGE_REFERENCE~`
2. Boundary `twin == INVALID_HE` -> `cg::BOUNDARY_HALF_EDGE~`
3. Either adjacent face degree not 3 -> `cg::NON_TRIANGLE_FACE~`
4. Opposite vertices `c == d` -> `cg::DEGENERATE_INPUT~`
5. New edge already exists in either direction -> `cg::EDGE_ALREADY_EXISTS~`
6. Return normally

Helper snippets:

```c3
fn bool is_valid_flip_he_index(HalfEdgeMesh* mesh, HeIndex he)
{
    if (he < 0) return false;
    return (usz)he < mesh.half_edges.len;
}
```

Use a distinct helper name from `is_valid_he_index` in `validate.c3` because all files in `module cg::half_edge` share one namespace. Do not reuse `is_valid_he_index` name.

```c3
fn bool edge_exists_excluding(
    HalfEdgeMesh* mesh,
    VertexIndex from,
    VertexIndex to,
    HeIndex excluded_a,
    HeIndex excluded_b)
{
    for (usz i = 0; i < mesh.half_edges.len; i++) {
        HeIndex candidate = (HeIndex)(int)i;
        if (candidate == excluded_a || candidate == excluded_b) continue;
        if (mesh.half_edges[i].origin != from) continue;
        if (mesh.to_vertex(candidate) == to) return true;
    }
    return false;
}
```

When checking duplicate new edge, test both directions:

```c3
if (edge_exists_excluding(mesh, c, d, h0, t0)) return cg::EDGE_ALREADY_EXISTS~;
if (edge_exists_excluding(mesh, d, c, h0, t0)) return cg::EDGE_ALREADY_EXISTS~;
```

### Vertex Handle Refresh Helper Shape

Recommended helpers:

```c3
fn void refresh_vertex_handle_from_local(
    HalfEdgeMesh* mesh,
    VertexIndex v,
    HeIndex h0,
    HeIndex h1,
    HeIndex h2,
    HeIndex t0,
    HeIndex t1,
    HeIndex t2)
```

Behavior:

1. Check local half-edges in this exact order:
   - `h0`, `h1`, `h2`, `t0`, `t1`, `t2`
2. If a local half-edge has `origin == v`, set `vertices[v].half_edge` to it and return.
3. If current handle is valid and still originates at `v`, keep it and return.
4. Scan all half-edges for first `origin == v`, set it and return.
5. If none exists, set `vertices[v].half_edge = cg::INVALID_HE`.

For valid meshes and valid flips, step 2 should always find a local half-edge for each of `a`, `b`, `c`, and `d`.

Call after the rewire using the snapshotted vertices:

```c3
refresh_vertex_handle_from_local(self, a, h0, h1, h2, t0, t1, t2);
refresh_vertex_handle_from_local(self, b, h0, h1, h2, t0, t1, t2);
refresh_vertex_handle_from_local(self, c, h0, h1, h2, t0, t1, t2);
refresh_vertex_handle_from_local(self, d, h0, h1, h2, t0, t1, t2);
```

## Commit Strategy

Use one commit per logical task:

1. `half_edge: add flip validation API (M4)`
2. `half_edge: flip interior triangle edges (M4)`
3. `half_edge: cover flip preservation behavior (M4)`
4. `half_edge: reject invalid flip topologies (M4)`
5. `half_edge: verify edge flipping milestone (M4)` only if a cleanup-only final commit is needed; otherwise skip this commit and just run verification.

Do not commit unrelated untracked files.

---

## Task 0: Create Implementation Worktree and Verify Baseline

**Files:**
- No tracked source files changed

- [ ] **Step 1: Create the worktree**

Run from main worktree:

```sh
cd /home/fesol/source/repos/c3cg.c3l
git status --short --branch
git worktree add .worktrees/m4-edge-flipping -b feat/m4-edge-flipping
cd .worktrees/m4-edge-flipping
```

Expected:

- new worktree at `.worktrees/m4-edge-flipping`
- branch `feat/m4-edge-flipping`
- no tracked changes in new worktree

- [ ] **Step 2: Verify baseline**

Run:

```sh
git status --short --branch
c3c build static-lib
c3c test
```

Expected:

- branch is `feat/m4-edge-flipping`
- build passes
- current suite passes with 82 tests

Do not proceed if baseline fails.

---

## Task 1: Wire Flip Source, Fault, and Basic Validation API

**Files:**
- Create: `src/half_edge/flip.c3`
- Create: `test/test_flip.c3`
- Modify: `src/faults.c3i`
- Modify: `project.json`
- Modify: `manifest.json`

### Goal

Make the M4 API visible to the build and implement the first validation paths:

- invalid half-edge
- boundary half-edge
- `is_flip_ok` returns false for those failures

### Tests to Add First

- `test_flip_rejects_invalid_half_edge`
- `test_flip_boundary_faults`

### Implementation Steps

- [ ] **Step 1: Add failing tests**

Create `test/test_flip.c3`:

```c3
module test;
import cg;
import cg::half_edge;

fn void test_flip_rejects_invalid_half_edge() @test
{
    HalfEdgeMesh mesh = half_edge::from_triangles(mem, DIAMOND_POSITIONS[..], DIAMOND_INDICES[..])!!;
    defer mesh.destroy();

    assert(!mesh.is_flip_ok(cg::INVALID_HE));
    if (catch err = mesh.flip(cg::INVALID_HE)) {
        assert(err == cg::INVALID_HALF_EDGE_REFERENCE);
    } else {
        assert(false);
    }

    HeIndex out_of_range = (HeIndex)(int)mesh.half_edges.len;
    assert(!mesh.is_flip_ok(out_of_range));
    if (catch err = mesh.flip(out_of_range)) {
        assert(err == cg::INVALID_HALF_EDGE_REFERENCE);
    } else {
        assert(false);
    }
}

fn void test_flip_boundary_faults() @test
{
    HalfEdgeMesh mesh = half_edge::from_triangles(mem, TRI_POSITIONS[..], TRI_INDICES[..])!!;
    defer mesh.destroy();

    HeIndex boundary_he = 0;
    HeIndex old_next = mesh.half_edges[boundary_he].next;
    VertexIndex old_origin = mesh.half_edges[boundary_he].origin;
    FaceIndex old_face = mesh.half_edges[boundary_he].face;

    assert(!mesh.is_flip_ok(boundary_he));
    if (catch err = mesh.flip(boundary_he)) {
        assert(err == cg::BOUNDARY_HALF_EDGE);
    } else {
        assert(false);
    }

    assert(mesh.half_edges[boundary_he].next == old_next);
    assert(mesh.half_edges[boundary_he].origin == old_origin);
    assert(mesh.half_edges[boundary_he].face == old_face);
    mesh.validate()!!;
}
```

Important:

- `DIAMOND_POSITIONS`, `DIAMOND_INDICES`, `TRI_POSITIONS`, and `TRI_INDICES` already exist in `module test` files.
- Do not redefine those constants.

- [ ] **Step 2: Run tests and confirm failure**

Run:

```sh
c3c test
```

Expected:

- compile failure because `is_flip_ok` and `flip` are not defined yet, or `EDGE_ALREADY_EXISTS` is not wired yet.

- [ ] **Step 3: Add fault**

Modify `src/faults.c3i`.

Current tail resembles:

```c3
    NON_MANIFOLD_INPUT,
    DUPLICATE_HALF_EDGE,
    NON_TRIANGLE_FACE,
```

Insert:

```c3
    EDGE_ALREADY_EXISTS,
```

near `DUPLICATE_HALF_EDGE`. Keep the final entry in the `faultdef` block ending with `;`.

- [ ] **Step 4: Add source file to configs**

Modify `project.json` sources:

```json
"src/half_edge/validate.c3",
"src/half_edge/flip.c3",
"src/render/rendering_data.c3",
```

Modify `manifest.json` sources the same way:

```json
"src/half_edge/validate.c3",
"src/half_edge/flip.c3",
"src/render/rendering_data.c3",
```

- [ ] **Step 5: Create minimal `src/half_edge/flip.c3` implementation**

Create:

```c3
module cg::half_edge;
import cg;

fn bool is_valid_flip_he_index(HalfEdgeMesh* mesh, HeIndex he)
{
    if (he < 0) return false;
    return (usz)he < mesh.half_edges.len;
}

fn void? validate_flip(HalfEdgeMesh* mesh, HeIndex he)
{
    if (!is_valid_flip_he_index(mesh, he)) return cg::INVALID_HALF_EDGE_REFERENCE~;

    HeIndex twin = mesh.half_edges[he].twin;
    if (twin == cg::INVALID_HE) return cg::BOUNDARY_HALF_EDGE~;

    return cg::NON_TRIANGLE_FACE~;
}

fn bool HalfEdgeMesh.is_flip_ok(&self, HeIndex he)
{
    if (catch err = validate_flip(self, he)) {
        return false;
    }
    return true;
}

fn void? HalfEdgeMesh.flip(&self, HeIndex he)
{
    validate_flip(self, he)!;
    return;
}
```

This intentionally rejects all non-boundary candidates after the first checks. Task 2 will add triangle validation and successful rewrite. The `NON_TRIANGLE_FACE` return is a short-lived placeholder so Task 1 stays small; Task 2 must replace it with the full validation chain.

- [ ] **Step 6: Run tests and verify Task 1 passes**

Run:

```sh
c3c build static-lib
c3c test
```

Expected:

- build passes
- existing tests pass
- the two new tests pass

- [ ] **Step 7: Commit Task 1**

Run:

```sh
git diff --check
git status --short
git add src/faults.c3i project.json manifest.json src/half_edge/flip.c3 test/test_flip.c3
git commit -m "half_edge: add flip validation API (M4)"
```

---

## Task 2: Implement Successful Interior Diamond Flip

**Files:**
- Modify: `src/half_edge/flip.c3`
- Modify: `test/test_flip.c3`

### Goal

Implement a successful local six-half-edge rewrite for a legal interior edge and refresh local vertex handles.

### Tests to Add First

- `test_flip_ok_on_interior_diamond`
- `test_flip_rewrites_diamond_diagonal`
- `test_flip_refreshes_local_vertex_handles`

### Implementation Steps

- [ ] **Step 1: Add helper test functions**

In `test/test_flip.c3`, add uniquely named helpers:

```c3
fn bool m4_edge_matches(HalfEdgeMesh* mesh, HeIndex he, VertexIndex from, VertexIndex to)
{
    return mesh.from_vertex(he) == from && mesh.to_vertex(he) == to;
}

fn bool m4_undirected_edge_matches(HalfEdgeMesh* mesh, HeIndex he, VertexIndex a, VertexIndex b)
{
    VertexIndex from = mesh.from_vertex(he);
    VertexIndex to = mesh.to_vertex(he);
    return (from == a && to == b) || (from == b && to == a);
}
```

Use `m4_` prefix to avoid same-module helper name collisions.

- [ ] **Step 2: Add failing success tests**

Add:

```c3
fn void test_flip_ok_on_interior_diamond() @test
{
    HalfEdgeMesh mesh = half_edge::from_triangles(mem, DIAMOND_POSITIONS[..], DIAMOND_INDICES[..])!!;
    defer mesh.destroy();

    assert(mesh.twin(1) == 3);
    assert(mesh.is_flip_ok(1));
}

fn void test_flip_rewrites_diamond_diagonal() @test
{
    HalfEdgeMesh mesh = half_edge::from_triangles(mem, DIAMOND_POSITIONS[..], DIAMOND_INDICES[..])!!;
    defer mesh.destroy();

    mesh.flip(1)!!;

    mesh.validate()!!;
    assert(mesh.face_degree(0) == 3);
    assert(mesh.face_degree(1) == 3);
    assert(m4_undirected_edge_matches(&mesh, 1, 0, 3));
    assert(m4_undirected_edge_matches(&mesh, mesh.twin(1), 0, 3));
    assert(!m4_undirected_edge_matches(&mesh, 1, 1, 2));
}

fn void test_flip_refreshes_local_vertex_handles() @test
{
    HalfEdgeMesh mesh = half_edge::from_triangles(mem, DIAMOND_POSITIONS[..], DIAMOND_INDICES[..])!!;
    defer mesh.destroy();

    mesh.flip(1)!!;

    VertexIndex[4] vertices = { 0, 1, 2, 3 };
    foreach (v : vertices) {
        HeIndex handle = mesh.vertices[v].half_edge;
        assert(handle != cg::INVALID_HE);
        assert(mesh.half_edges[handle].origin == v);
    }
    mesh.validate()!!;
}
```

Notes for diamond fixture:

- `DIAMOND_INDICES = { 0, 1, 2, 2, 1, 3 }`
- shared edge slots are `1` and `3`
- old diagonal is vertices `1 <-> 2`
- new diagonal after flipping slot `1` is vertices `0 <-> 3`

- [ ] **Step 3: Run tests and confirm failure**

Run:

```sh
c3c test
```

Expected:

- `test_flip_ok_on_interior_diamond` fails because Task 1 validation still returns `NON_TRIANGLE_FACE` for legal candidates.

- [ ] **Step 4: Implement full validation through duplicate-edge check**

Replace `validate_flip` with the full validation except duplicate-edge helper may initially return false until Task 4 if needed. Prefer implementing edge scan now because it is simple.

Add helper:

```c3
fn bool edge_exists_excluding(
    HalfEdgeMesh* mesh,
    VertexIndex from,
    VertexIndex to,
    HeIndex excluded_a,
    HeIndex excluded_b)
{
    for (usz i = 0; i < mesh.half_edges.len; i++) {
        HeIndex candidate = (HeIndex)(int)i;
        if (candidate == excluded_a || candidate == excluded_b) continue;
        if (mesh.half_edges[i].origin != from) continue;
        if (mesh.to_vertex(candidate) == to) return true;
    }
    return false;
}
```

Update validation:

```c3
fn void? validate_flip(HalfEdgeMesh* mesh, HeIndex he)
{
    if (!is_valid_flip_he_index(mesh, he)) return cg::INVALID_HALF_EDGE_REFERENCE~;

    HeIndex h0 = he;
    HeIndex t0 = mesh.half_edges[h0].twin;
    if (t0 == cg::INVALID_HE) return cg::BOUNDARY_HALF_EDGE~;

    FaceIndex f0 = mesh.half_edges[h0].face;
    FaceIndex f1 = mesh.half_edges[t0].face;
    if (mesh.face_degree(f0) != 3) return cg::NON_TRIANGLE_FACE~;
    if (mesh.face_degree(f1) != 3) return cg::NON_TRIANGLE_FACE~;

    HeIndex h1 = mesh.half_edges[h0].next;
    HeIndex h2 = mesh.half_edges[h1].next;
    HeIndex t1 = mesh.half_edges[t0].next;
    HeIndex t2 = mesh.half_edges[t1].next;

    VertexIndex c = mesh.half_edges[h2].origin;
    VertexIndex d = mesh.half_edges[t2].origin;
    if (c == d) return cg::DEGENERATE_INPUT~;

    if (edge_exists_excluding(mesh, c, d, h0, t0)) return cg::EDGE_ALREADY_EXISTS~;
    if (edge_exists_excluding(mesh, d, c, h0, t0)) return cg::EDGE_ALREADY_EXISTS~;

    return;
}
```

- [ ] **Step 5: Implement vertex refresh helpers**

Add:

```c3
fn bool local_half_edge_origin_matches(HalfEdgeMesh* mesh, HeIndex he, VertexIndex v)
{
    if (!is_valid_flip_he_index(mesh, he)) return false;
    return mesh.half_edges[he].origin == v;
}

fn void refresh_vertex_handle_from_local(
    HalfEdgeMesh* mesh,
    VertexIndex v,
    HeIndex h0,
    HeIndex h1,
    HeIndex h2,
    HeIndex t0,
    HeIndex t1,
    HeIndex t2)
{
    if (local_half_edge_origin_matches(mesh, h0, v)) { mesh.vertices[v].half_edge = h0; return; }
    if (local_half_edge_origin_matches(mesh, h1, v)) { mesh.vertices[v].half_edge = h1; return; }
    if (local_half_edge_origin_matches(mesh, h2, v)) { mesh.vertices[v].half_edge = h2; return; }
    if (local_half_edge_origin_matches(mesh, t0, v)) { mesh.vertices[v].half_edge = t0; return; }
    if (local_half_edge_origin_matches(mesh, t1, v)) { mesh.vertices[v].half_edge = t1; return; }
    if (local_half_edge_origin_matches(mesh, t2, v)) { mesh.vertices[v].half_edge = t2; return; }

    HeIndex current = mesh.vertices[v].half_edge;
    if (is_valid_flip_he_index(mesh, current) && mesh.half_edges[current].origin == v) return;

    for (usz i = 0; i < mesh.half_edges.len; i++) {
        if (mesh.half_edges[i].origin == v) {
            mesh.vertices[v].half_edge = (HeIndex)(int)i;
            return;
        }
    }

    mesh.vertices[v].half_edge = cg::INVALID_HE;
}
```

If the one-line `if (...) { ...; return; }` style conflicts with `docs/style.md`, expand each branch to multiple lines. Use style guide over this plan.

- [ ] **Step 6: Implement rewire in `flip`**

Replace `flip` body:

```c3
fn void? HalfEdgeMesh.flip(&self, HeIndex he)
{
    validate_flip(self, he)!;

    HeIndex h0 = he;
    HeIndex h1 = self.half_edges[h0].next;
    HeIndex h2 = self.half_edges[h1].next;

    HeIndex t0 = self.half_edges[h0].twin;
    HeIndex t1 = self.half_edges[t0].next;
    HeIndex t2 = self.half_edges[t1].next;

    FaceIndex f0 = self.half_edges[h0].face;
    FaceIndex f1 = self.half_edges[t0].face;

    VertexIndex a = self.half_edges[h0].origin;
    VertexIndex b = self.half_edges[t0].origin;
    VertexIndex c = self.half_edges[h2].origin;
    VertexIndex d = self.half_edges[t2].origin;

    self.half_edges[h0].origin = c;
    self.half_edges[t0].origin = d;

    self.half_edges[h0].next = t2;
    self.half_edges[t2].next = h1;
    self.half_edges[h1].next = h0;

    self.half_edges[t0].next = h2;
    self.half_edges[h2].next = t1;
    self.half_edges[t1].next = t0;

    self.half_edges[h0].face = f0;
    self.half_edges[t2].face = f0;
    self.half_edges[h1].face = f0;

    self.half_edges[t0].face = f1;
    self.half_edges[h2].face = f1;
    self.half_edges[t1].face = f1;

    self.faces[f0].half_edge = h0;
    self.faces[f1].half_edge = t0;

    refresh_vertex_handle_from_local(self, a, h0, h1, h2, t0, t1, t2);
    refresh_vertex_handle_from_local(self, b, h0, h1, h2, t0, t1, t2);
    refresh_vertex_handle_from_local(self, c, h0, h1, h2, t0, t1, t2);
    refresh_vertex_handle_from_local(self, d, h0, h1, h2, t0, t1, t2);

    return;
}
```

C3 indexing note:

- Existing code indexes slices with inline typedefs (`mesh.half_edges[he]`) successfully. If compiler rejects a typed index in a new context, cast to `(usz)he` consistently. Do not preemptively overcast unless needed.

- [ ] **Step 7: Run tests and fix compile issues**

Run:

```sh
c3c build static-lib
c3c test
```

Expected:

- new success tests pass
- previous tests pass

Common likely fixes:

- If helper name collides in `module cg::half_edge`, rename with `flip_` prefix.
- If test helper name collides in `module test`, rename with `m4_` prefix.
- If C3 rejects same-line branches as style/syntax, expand them.

- [ ] **Step 8: Commit Task 2**

Run:

```sh
git diff --check
git status --short
git add src/half_edge/flip.c3 test/test_flip.c3
git commit -m "half_edge: flip interior triangle edges (M4)"
```

---

## Task 3: Cover Flip-Back, Counts, and Attribute Preservation

**Files:**
- Modify: `test/test_flip.c3`
- Modify: `src/half_edge/flip.c3` only if tests expose a bug

### Goal

Verify successful flips preserve mesh identity and attributes at the topology level.

### Tests to Add First

- `test_flip_back_restores_original_topology_equivalence`
- `test_flip_preserves_counts_and_attributes`
- `test_flip_preserves_euler_counts`

### Implementation Steps

- [ ] **Step 1: Add order-insensitive face-set helpers**

Add to `test/test_flip.c3`:

```c3
fn bool m4_face_has_vertices(HalfEdgeMesh* mesh, FaceIndex face, VertexIndex a, VertexIndex b, VertexIndex c)
{
    VertexIndex[3] verts;
    int count = mesh.face_vertices(face, verts[..])!!;
    if (count != 3) return false;

    bool has_a = false;
    bool has_b = false;
    bool has_c = false;
    for (usz i = 0; i < 3; i++) {
        if (verts[i] == a) has_a = true;
        if (verts[i] == b) has_b = true;
        if (verts[i] == c) has_c = true;
    }
    return has_a && has_b && has_c;
}

fn bool m4_mesh_has_face_vertices(HalfEdgeMesh* mesh, VertexIndex a, VertexIndex b, VertexIndex c)
{
    for (usz i = 0; i < mesh.faces.len; i++) {
        if (m4_face_has_vertices(mesh, (FaceIndex)(int)i, a, b, c)) return true;
    }
    return false;
}

fn bool m4_mesh_has_undirected_edge(HalfEdgeMesh* mesh, VertexIndex a, VertexIndex b)
{
    for (usz i = 0; i < mesh.half_edges.len; i++) {
        if (m4_undirected_edge_matches(mesh, (HeIndex)(int)i, a, b)) return true;
    }
    return false;
}
```

- [ ] **Step 2: Add flip-back test**

```c3
fn void test_flip_back_restores_original_topology_equivalence() @test
{
    HalfEdgeMesh mesh = half_edge::from_triangles(mem, DIAMOND_POSITIONS[..], DIAMOND_INDICES[..])!!;
    defer mesh.destroy();

    assert(m4_mesh_has_face_vertices(&mesh, 0, 1, 2));
    assert(m4_mesh_has_face_vertices(&mesh, 1, 2, 3));
    assert(m4_mesh_has_undirected_edge(&mesh, 1, 2));

    mesh.flip(1)!!;
    mesh.flip(1)!!;

    mesh.validate()!!;
    assert(m4_mesh_has_face_vertices(&mesh, 0, 1, 2));
    assert(m4_mesh_has_face_vertices(&mesh, 1, 2, 3));
    assert(m4_mesh_has_undirected_edge(&mesh, 1, 2));
}
```

- [ ] **Step 3: Add count and attribute tests**

```c3
fn void test_flip_preserves_euler_counts() @test
{
    HalfEdgeMesh mesh = half_edge::from_triangles(mem, DIAMOND_POSITIONS[..], DIAMOND_INDICES[..])!!;
    defer mesh.destroy();

    usz vertex_count = mesh.vertices.len;
    usz half_edge_count = mesh.half_edges.len;
    usz face_count = mesh.faces.len;

    mesh.flip(1)!!;

    assert(mesh.vertices.len == vertex_count);
    assert(mesh.half_edges.len == half_edge_count);
    assert(mesh.faces.len == face_count);
    mesh.validate()!!;
}

fn void test_flip_preserves_counts_and_attributes() @test
{
    HalfEdgeMesh mesh = half_edge::from_triangles_with_attrs(
        mem, DIAMOND_POSITIONS[..], TET_NORMALS[..], TET_UVS[..], DIAMOND_INDICES[..])!!;
    defer mesh.destroy();

    usz positions_len = mesh.positions.len;
    usz normals_len = mesh.normals.len;
    usz uvs_len = mesh.uvs.len;
    Vec3f position_0 = mesh.positions[0];
    Vec3f normal_0 = mesh.normals[0];
    Vec2f uv_0 = mesh.uvs[0];

    mesh.flip(1)!!;

    assert(mesh.positions.len == positions_len);
    assert(mesh.normals.len == normals_len);
    assert(mesh.uvs.len == uvs_len);
    assert(mesh.positions[0] == position_0);
    assert(mesh.normals[0] == normal_0);
    assert(mesh.uvs[0] == uv_0);
    mesh.validate()!!;
}
```

Note:

- `DIAMOND_POSITIONS.len == 4`, `TET_NORMALS.len == 4`, and `TET_UVS.len == 4`, so using the tetrahedron attributes as generic four-element attribute arrays is valid.

- [ ] **Step 4: Run tests and confirm failure or pass**

Run:

```sh
c3c test
```

Expected:

- Tests may already pass after Task 2. If so, still commit the added tests as coverage.
- If any fail, inspect whether the implementation violates the spec or the test assumes slot-exact behavior forbidden by spec.

- [ ] **Step 5: Fix only if needed**

Potential fix areas:

- vertex refresh order
- face assignment in rewire table
- duplicate-edge helper accidentally sees the reused current diagonal after first flip

Do not add new behavior beyond the spec.

- [ ] **Step 6: Commit Task 3**

Run:

```sh
git diff --check
git add src/half_edge/flip.c3 test/test_flip.c3
git commit -m "half_edge: cover flip preservation behavior (M4)"
```

If `src/half_edge/flip.c3` did not change:

```sh
git add test/test_flip.c3
git commit -m "half_edge: cover flip preservation behavior (M4)"
```

---

## Task 4: Add Rejection Coverage for Non-Triangle, Degenerate, and Duplicate New Edge

**Files:**
- Modify: `test/test_flip.c3`
- Modify: `src/half_edge/flip.c3` only if tests expose a bug

### Goal

Complete all M4 failure-path coverage and prove `flip` is failure-atomic on otherwise valid rejected meshes where possible.

### Tests to Add First

- `test_flip_rejects_non_triangle_face`
- `test_flip_rejects_degenerate_output_endpoints`
- `test_flip_rejects_duplicate_new_edge`

### Helper for Fault Atomicity Snapshots

Add a focused snapshot helper for small tests:

```c3
struct M4HalfEdgeSnapshot {
    VertexIndex origin;
    HeIndex next;
    HeIndex twin;
    FaceIndex face;
}

fn M4HalfEdgeSnapshot m4_snapshot_half_edge(HalfEdgeMesh* mesh, HeIndex he)
{
    M4HalfEdgeSnapshot snapshot;
    snapshot.origin = mesh.half_edges[he].origin;
    snapshot.next = mesh.half_edges[he].next;
    snapshot.twin = mesh.half_edges[he].twin;
    snapshot.face = mesh.half_edges[he].face;
    return snapshot;
}

fn void m4_assert_half_edge_snapshot(HalfEdgeMesh* mesh, HeIndex he, M4HalfEdgeSnapshot snapshot)
{
    assert(mesh.half_edges[he].origin == snapshot.origin);
    assert(mesh.half_edges[he].next == snapshot.next);
    assert(mesh.half_edges[he].twin == snapshot.twin);
    assert(mesh.half_edges[he].face == snapshot.face);
}
```

Place `struct M4HalfEdgeSnapshot` near the top of `test/test_flip.c3`, after constants if any local constants are added and before functions. Same-module namespace applies; use `M4` prefix.

### Non-Triangle Fixture Strategy

Use `from_polygons` to build one triangle adjacent to one quad across an edge.

Candidate fixture:

```c3
Vec3f[5] positions = {
    { 0, 0, 0 },
    { 1, 0, 0 },
    { 0, 1, 0 },
    { 1, 1, 0 },
    { 2, 0, 0 },
};
uint[3] offsets = { 0, 3, 7 };
uint[7] indices = {
    0, 1, 2,
    1, 0, 3, 4,
};
```

Topology:

- face 0 triangle has edge `0 -> 1` at half-edge `0`
- face 1 quad has reverse edge `1 -> 0` at half-edge `3`
- `0` and `3` are twins
- flipping `0` should return `NON_TRIANGLE_FACE`

Test:

```c3
fn void test_flip_rejects_non_triangle_face() @test
{
    Vec3f[5] positions = {
        { 0, 0, 0 },
        { 1, 0, 0 },
        { 0, 1, 0 },
        { 1, 1, 0 },
        { 2, 0, 0 },
    };
    uint[3] offsets = { 0, 3, 7 };
    uint[7] indices = { 0, 1, 2, 1, 0, 3, 4 };
    HalfEdgeMesh mesh = half_edge::from_polygons(mem, positions[..], offsets[..], indices[..])!!;
    defer mesh.destroy();
    mesh.validate()!!;

    M4HalfEdgeSnapshot before = m4_snapshot_half_edge(&mesh, 0);
    assert(!mesh.is_flip_ok(0));
    if (catch err = mesh.flip(0)) {
        assert(err == cg::NON_TRIANGLE_FACE);
    } else {
        assert(false);
    }
    m4_assert_half_edge_snapshot(&mesh, 0, before);
    mesh.validate()!!;
}
```

### Degenerate Fixture Strategy

Try this builder fixture first:

```c3
Vec3f[3] positions = {
    { 0, 0, 0 },
    { 1, 0, 0 },
    { 0, 1, 0 },
};
uint[6] indices = {
    0, 1, 2,
    1, 0, 2,
};
```

Topology:

- `h0 = 0`, `t0 = 3`
- both opposite vertices are `2`, so `c == d`
- if builder accepts it and `mesh.validate()` passes, use this fixture.

Test:

```c3
fn void test_flip_rejects_degenerate_output_endpoints() @test
{
    Vec3f[3] positions = {
        { 0, 0, 0 },
        { 1, 0, 0 },
        { 0, 1, 0 },
    };
    uint[6] indices = { 0, 1, 2, 1, 0, 2 };
    HalfEdgeMesh mesh = half_edge::from_triangles(mem, positions[..], indices[..])!!;
    defer mesh.destroy();
    mesh.validate()!!;

    M4HalfEdgeSnapshot before = m4_snapshot_half_edge(&mesh, 0);
    assert(!mesh.is_flip_ok(0));
    if (catch err = mesh.flip(0)) {
        assert(err == cg::DEGENERATE_INPUT);
    } else {
        assert(false);
    }
    m4_assert_half_edge_snapshot(&mesh, 0, before);
    mesh.validate()!!;
}
```

If builder rejects this fixture with `DUPLICATE_HALF_EDGE` or validation fails, manually construct the minimal arrays using `mem::alloc::new_array(mem, ...)` in the test and set fields directly. Keep the fixture just valid enough to reach the `c == d` guard.

### Duplicate New-Edge Fixture Strategy

Use the existing tetrahedron fixture. In `TET_INDICES`:

```c3
const uint[12] TET_INDICES = {
    0, 1, 2,
    0, 3, 1,
    0, 2, 3,
    1, 3, 2,
};
```

Half-edge `0` is `0 -> 1`; its twin is half-edge `5`, `1 -> 0`. The opposite vertices are `2` and `3`. The tetrahedron already contains edge `2 <-> 3` outside half-edge slots `0` and `5`, so flipping half-edge `0` must return `cg::EDGE_ALREADY_EXISTS`.

Test:

```c3
fn void test_flip_rejects_duplicate_new_edge() @test
{
    HalfEdgeMesh mesh = half_edge::from_triangles(mem, TET_POSITIONS[..], TET_INDICES[..])!!;
    defer mesh.destroy();
    mesh.validate()!!;

    HeIndex he = 0;
    assert(mesh.twin(he) == 5);
    assert(m4_mesh_has_undirected_edge(&mesh, 2, 3));

    M4HalfEdgeSnapshot before = m4_snapshot_half_edge(&mesh, he);

    assert(!mesh.is_flip_ok(he));
    if (catch err = mesh.flip(he)) {
        assert(err == cg::EDGE_ALREADY_EXISTS);
    } else {
        assert(false);
    }

    m4_assert_half_edge_snapshot(&mesh, he, before);
    mesh.validate()!!;
}
```

### Implementation Steps

- [ ] **Step 1: Add snapshot helper and non-triangle test**

Add helpers and `test_flip_rejects_non_triangle_face`.

Run:

```sh
c3c test
```

Expected:

- test passes if Task 2 validation already checks face degree.
- if it fails, fix validation order to return `NON_TRIANGLE_FACE` before mutation.

- [ ] **Step 2: Add degenerate endpoint test**

Add `test_flip_rejects_degenerate_output_endpoints`.

Run:

```sh
c3c test
```

Expected:

- test passes if Task 2 validation checks `c == d`.
- if builder fixture fails, replace with manual fixture and rerun.

- [ ] **Step 3: Add duplicate new-edge test**

Add the tetrahedron-based duplicate new-edge test from the fixture strategy above.

Run:

```sh
c3c test
```

Expected:

- if Task 2 implemented `edge_exists_excluding`, test passes.
- if not, implement/fix duplicate-edge validation.

- [ ] **Step 4: Run full focused verification**

Run:

```sh
git diff --check
c3c build static-lib
c3c test
```

Expected:

- build passes
- all tests pass
- no whitespace errors

- [ ] **Step 5: Commit Task 4**

Run:

```sh
git add src/half_edge/flip.c3 test/test_flip.c3
git commit -m "half_edge: reject invalid flip topologies (M4)"
```

If only tests changed:

```sh
git add test/test_flip.c3
git commit -m "half_edge: reject invalid flip topologies (M4)"
```

---

## Task 5: Final Audit, Reviews, and Milestone Verification

**Files:**
- Modify only if review finds issues

### Goal

Verify M4 is complete, clean, reviewed, and ready to merge.

### Implementation Steps

- [ ] **Step 1: Inspect final diff**

Run:

```sh
git status --short --branch
git log --oneline -n 8
git diff main...HEAD --stat
git diff main...HEAD -- src/half_edge/flip.c3 test/test_flip.c3 src/faults.c3i project.json manifest.json
```

Expected:

- only M4 files changed
- commits are focused
- no unrelated untracked files staged

- [ ] **Step 2: Search for production anti-patterns**

Run:

```sh
python3 - <<'PY'
from pathlib import Path
for path in [Path('src/half_edge/flip.c3')]:
    text = path.read_text()
    for bad in ['assert(', 'unreachable(', '->', '@private', 'goto', 'sizeof(']:
        if bad in text:
            print(f'{path}: found {bad}')
PY
```

Expected:

- no output

If output appears, fix before continuing.

- [ ] **Step 3: Run full verification**

Run:

```sh
git diff --check
c3c build static-lib
c3c test
c3c build debug
c3c build release
```

Expected:

- all commands pass
- test suite count increased from 82 by the number of M4 tests added

- [ ] **Step 4: Dispatch final code review**

Use a code-review subagent with this context:

```text
Review M4 edge flipping implementation in /home/fesol/source/repos/c3cg.c3l/.worktrees/m4-edge-flipping.
Diff scope: main...HEAD.
Spec: docs/specs/2026-05-07-m4-edge-flipping-design.md.
Focus: half-edge rewire correctness, fault order, failure atomicity, vertex-handle refresh, duplicate-edge check, C3/project style, tests.
Do not implement. Return blocking issues vs advisory notes.
```

Expected:

- Approved or only advisory issues

- [ ] **Step 5: Dispatch C3 expert review**

Use a C3/HalfEdgeMesh expert subagent with this context:

```text
Review M4 edge flipping implementation for C3 0.7.11 feasibility and idiom.
Repo: /home/fesol/source/repos/c3cg.c3l/.worktrees/m4-edge-flipping.
Diff scope: main...HEAD.
Spec: docs/specs/2026-05-07-m4-edge-flipping-design.md.
Check: C3 syntax/optionals/faults, .c3/.c3i separation, no @private helper functions, pointer dot access, module namespace collisions, test fixture validity, and half-edge topology correctness.
Do not implement. Return blocking issues vs advisory notes.
```

Expected:

- Approved or only advisory issues

- [ ] **Step 6: Fix any blocking review issues**

If review finds blocking issues:

1. Write or adjust failing test first.
2. Run `c3c test` and confirm failure.
3. Fix implementation.
4. Run full focused verification.
5. Amend or add a follow-up commit depending on scope.
6. Re-run the relevant review.

- [ ] **Step 7: Final status report**

Run:

```sh
git status --short --branch
git log --oneline -n 8
```

Expected:

- clean tracked worktree
- HEAD contains all M4 commits

Report to user:

- branch name
- commit list
- verification results
- review status
- merge options

---

## Expected Final File Changes

`src/faults.c3i`

```c3
faultdef
    INDEX_OUT_OF_RANGE,
    INVALID_TRIANGLE_INDEX_COUNT,
    NON_MANIFOLD_INPUT,
    DUPLICATE_HALF_EDGE,
    EDGE_ALREADY_EXISTS,
    NON_TRIANGLE_FACE,
    ...
```

Exact insertion may differ, but `EDGE_ALREADY_EXISTS` must be present and the final fault must retain the semicolon.

`project.json`

```json
"src/half_edge/validate.c3",
"src/half_edge/flip.c3",
"src/render/rendering_data.c3",
```

`manifest.json`

```json
"src/half_edge/validate.c3",
"src/half_edge/flip.c3",
"src/render/rendering_data.c3",
```

`src/half_edge/flip.c3`

Must contain:

- `module cg::half_edge;`
- `import cg;`
- no production `assert`
- no `@private`
- no `->`
- `is_flip_ok`
- `flip`
- validation helper
- duplicate edge helper
- vertex refresh helper

`test/test_flip.c3`

Must contain tests for:

- interior diamond flip ok
- diamond diagonal rewrite
- flip-back topology equivalence
- boundary fault
- invalid half-edge fault
- degenerate output endpoints fault
- non-triangle face fault
- duplicate new-edge fault
- counts/attributes preservation
- local vertex handle refresh
- Euler count preservation

## Success Criteria Checklist

- [ ] `src/half_edge/flip.c3` added to both config files.
- [ ] `EDGE_ALREADY_EXISTS` added to `src/faults.c3i`.
- [ ] `HalfEdgeMesh.is_flip_ok` returns true for legal interior triangle flips and false for all specified rejections.
- [ ] `HalfEdgeMesh.flip` returns specific faults for illegal flips.
- [ ] Successful flip rewires local topology according to spec.
- [ ] Successful flip preserves all array lengths.
- [ ] Successful flip preserves `positions`, `normals`, and `uvs` values.
- [ ] Successful flip refreshes the four local vertex handles.
- [ ] `mesh.validate()` passes after successful flips.
- [ ] Failure-path tests prove no mutation for otherwise valid rejected meshes.
- [ ] Full verification passes:
  - [ ] `git diff --check`
  - [ ] `c3c build static-lib`
  - [ ] `c3c test`
  - [ ] `c3c build debug`
  - [ ] `c3c build release`
- [ ] Code review approved.
- [ ] C3 expert review approved.

## Handoff Notes for Subagents

When dispatching Task 1-4 implementation subagents:

- Give the subagent only this plan, the spec path, and exact task number.
- Require TDD: failing tests first, verify failure, implement, verify pass.
- Require a commit at the end of each task.
- Require the subagent to report:
  - commit hash
  - tests run
  - files changed
  - any deviations from plan
- Parent agent must verify each subagent commit before dispatching the next task.

Suggested subagent task order:

1. Task 1 subagent: API/fault/config/basic validation.
2. Task 2 subagent: successful rewire and vertex handles.
3. Task 3 subagent: flip-back/preservation tests and fixes.
4. Task 4 subagent: non-triangle/degenerate/duplicate rejection tests and fixes.
5. Parent or reviewer subagents: final verification/reviews.
