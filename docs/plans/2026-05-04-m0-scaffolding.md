# M0 — Project Scaffolding Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.
> **REQUIRED SKILL:** `c3-expert` — invoke before writing any C3 code, editing project.json/manifest.json, or diagnosing c3c errors.

**Goal:** Stand up the C3 library project skeleton with build system, type definitions, fault catalog, data struct stubs, and two passing tests.

**Architecture:** Flat `.c3i` files at project root (`.c3l` library convention), all sharing `module cg;` namespace. Submodule `cg::smoke` validates cross-module method declarations. Independent `module test;` in `test/` imports the library.

**Tech Stack:** C3 0.7.11, c3c compiler

**Spec:** `docs/specs/2026-05-04-m0-scaffolding-design.md`

---

### Task 1: Create `project.json` and source stubs

**Files:**
- Create: `project.json`
- Create: `types.c3i` (stub)
- Create: `faults.c3i` (stub)
- Create: `half_edge_mesh.c3i` (stub)
- Create: `smoke/smoke.c3` (stub)

- [ ] **Step 1.1: Write project.json**

```json
{
  "langrev": "1",
  "warnings": ["no-unused"],
  "sources": [
    "c3cg.c3i",
    "types.c3i",
    "faults.c3i",
    "half_edge_mesh.c3i",
    "smoke/smoke.c3"
  ],
  "test-sources": ["test/**"],
  "targets": {
    "static-lib": {
      "type": "static-lib"
    },
    "debug": {
      "type": "static-lib",
      "opt": "O0"
    },
    "release": {
      "type": "static-lib",
      "opt": "O3"
    }
  }
}
```

- [ ] **Step 1.2: Create source stubs**

```c3
module cg;
```
Create this same content in `types.c3i`, `faults.c3i`, and `half_edge_mesh.c3i`.

And for smoke:
```c3
module cg::smoke;
```
Create this in `smoke/smoke.c3`.

These minimal stubs let the build pass while Tasks 2-6 replace them with real content.

- [ ] **Step 1.3: Verify build with stubs**

```bash
c3c build static-lib
```

Expected: `Static library 'out/static-lib.a' created.`

- [ ] **Step 1.4: Commit**

```bash
git add project.json smoke/smoke.c3
git commit -m "build: add project.json with static-lib, debug, and release targets"
```

---

### Task 2: Write `types.c3i`

**Files:**
- Modify: `types.c3i` (replace stub with real content)

- [ ] **Step 2.1: Write types.c3i**

```c3
module cg;

typedef HeIndex     = inline int;
typedef FaceIndex   = inline int;
typedef VertexIndex = inline int;

alias Vec2f = float[<2>];
alias Vec3f = float[<3>];
alias Vec4f = float[<4>];
alias Vec2i = int[<2>];
alias Vec3i = int[<3>];

const HeIndex     INVALID_HE     = -1;
const FaceIndex   INVALID_FACE   = -1;
const VertexIndex INVALID_VERTEX = -1;

struct Aabb {
    Vec3f min;
    Vec3f max;
}
```

- [ ] **Step 2.2: Verify compiles**

```bash
c3c build static-lib
```

Expected: `Static library 'out/static-lib.a' created.`

- [ ] **Step 2.3: Commit**

```bash
git add types.c3i
git commit -m "feat: add vector aliases, identifier typedefs, INVALID_* constants, and Aabb struct"
```

---

### Task 3: Write `faults.c3i`

**Files:**
- Modify: `faults.c3i` (replace stub with real content)

- [ ] **Step 3.1: Write faults.c3i**

```c3
module cg;

faultdef
    INDEX_OUT_OF_RANGE,
    INVALID_TRIANGLE_INDEX_COUNT,
    NON_MANIFOLD_INPUT,
    DUPLICATE_HALF_EDGE,
    NON_TRIANGLE_FACE,
    BOUNDARY_HALF_EDGE,
    DEGENERATE_INPUT,
    EMPTY_INPUT,
    OPEN_CELL_ON_BOUNDARY;
```

- [ ] **Step 3.2: Verify compiles**

```bash
c3c build static-lib
```

Expected: `Static library 'out/static-lib.a' created.`

- [ ] **Step 3.3: Commit**

```bash
git add faults.c3i
git commit -m "feat: add 9 cross-cutting faults for computational geometry library"
```

---

### Task 4: Write `half_edge_mesh.c3i`

**Files:**
- Modify: `half_edge_mesh.c3i` (replace stub with real content)

- [ ] **Step 4.1: Write half_edge_mesh.c3i**

```c3
module cg;

struct HalfEdge {
    VertexIndex origin;
    HeIndex     next;
    HeIndex     twin;
    FaceIndex   face;
}

struct HalfEdgeFace {
    HeIndex half_edge;
}

struct HalfEdgeVertex {
    HeIndex half_edge;
}

struct HalfEdgeMesh {
    HalfEdge[]       half_edges;
    HalfEdgeFace[]   faces;
    HalfEdgeVertex[] vertices;
    Vec3f[]          positions;
    Vec3f[]          normals;
    Vec2f[]          uvs;
}
```

- [ ] **Step 4.2: Verify compiles**

```bash
c3c build static-lib
```

Expected: `Static library 'out/static-lib.a' created.`

- [ ] **Step 4.3: Commit**

```bash
git add half_edge_mesh.c3i
git commit -m "feat: add HalfEdgeMesh core data structures (structs only, no methods)"
```

---

### Task 5: Update `c3cg.c3i` and `manifest.json`

**Files:**
- Modify: `c3cg.c3i`
- Modify: `manifest.json`

- [ ] **Step 5.1: Replace c3cg.c3i content**

Existing content (`module c3cg;`) → replace with:
```c3
module cg;
```

- [ ] **Step 5.2: Update manifest.json sources**

Replace the commented `// "sources"` line with:
```json
"sources": [
    "c3cg.c3i",
    "types.c3i",
    "faults.c3i",
    "half_edge_mesh.c3i"
],
```

Keep `"provides": "c3cg"`, `"linklib-dir"`, and the 16 platform entries under `"targets"` unchanged. The smoke module is verification scaffolding, not part of the library's public API — it does NOT go in manifest.json.

- [ ] **Step 5.3: Verify compiles**

```bash
c3c build static-lib
```

Expected: `Static library 'out/static-lib.a' created.`

- [ ] **Step 5.4: Commit**

```bash
git add c3cg.c3i manifest.json
git commit -m "refactor: change module to cg, update manifest sources"
```

---

### Task 6: Write `smoke/smoke.c3` and `test/test_cross_module.c3`

**Files:**
- Modify: `smoke/smoke.c3` (replace stub with real method)
- Create: `test/test_cross_module.c3`

> **Purpose:** Verify the architecture-critical assumption that methods can be declared on a type from a different module than the one that defines it. `HalfEdgeMesh` is in `module cg;` — `module cg::smoke;` adds a method to it.

- [ ] **Step 6.1: Create smoke/smoke.c3**

```c3
module cg::smoke;
import cg;

fn int HalfEdgeMesh.dummy(&self) {
    return 42;
}
```

- [ ] **Step 6.2: Create test/test_cross_module.c3**

```c3
module test;
import cg;
import cg::smoke;

fn void test_cross_module_method() @test
{
    HalfEdgeMesh mesh = {};
    assert(mesh.dummy() == 42);
}
```

- [ ] **Step 6.3: Verify cross-module test passes**

```bash
c3c test
```

Expected: `Testing test::test_cross_module_method .. [PASS]`  (1 passed; test_smoke.c3 created in Task 7)

- [ ] **Step 6.4: Commit**

```bash
git add smoke/smoke.c3 test/test_cross_module.c3
git commit -m "test: verify cross-module method declarations work on c3c 0.7.11"
```

---

### Task 7: Create `test/test_smoke.c3`

**Files:**
- Create: `test/test_smoke.c3`

- [ ] **Step 7.1: Write test/test_smoke.c3**

```c3
module test;
import cg;

fn void test_compiles() @test
{
}
```

- [ ] **Step 7.2: Verify both tests pass**

```bash
c3c test
```

Expected:
```
Testing test::test_compiles ............. [PASS]
Testing test::test_cross_module_method .. [PASS]

2 tests run.
Test Result: PASSED: 2 passed, 0 failed, 0 skipped.
```

- [ ] **Step 7.3: Commit**

```bash
git add test/test_smoke.c3
git commit -m "test: add harness verification smoke test"
```

---

### Task 8: Full verification and commit

- [ ] **Step 8.1: Run full build + test**

```bash
c3c build static-lib && c3c test
```

Expected: `Static library 'out/static-lib.a' created.` + `PASSED: 2 passed, 0 failed, 0 skipped.`

- [ ] **Step 8.2: Verify debug and release targets build**

```bash
c3c build debug && c3c build release
```

Expected: Both produce static libraries successfully.

- [ ] **Step 8.3: Final commit (if any cleanup needed)**

```bash
git status
# If clean, done. If not, commit remaining changes.
```

---

### Task 9: Update `.claude/c3-skill.json` (already done)

**Files:**
- Modify: `.claude/c3-skill.json`

Already pinned to `"0.7.11"` during spec phase. Verify:

```bash
cat .claude/c3-skill.json
```

Expected: `{"version": "0.7.11"}`

No separate commit needed — this was done before the plan.
