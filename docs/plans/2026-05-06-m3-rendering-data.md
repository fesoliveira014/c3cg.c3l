# M3 Rendering Data Extraction Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.
> **REQUIRED SKILLS:** `c3-expert` before any C3/build-file edit, `test-driven-development` for implementation tasks, `verification-before-completion` before claiming done.

**Goal:** Add lean, owned rendering-data extraction for `HalfEdgeMesh`.

**Architecture:** Add a new one-way `cg::render` module that depends on `cg` and `cg::half_edge`. `RenderingData` is an owned snapshot: vertices are copied from mesh positions, indices are emitted from validated face cycles, and normals/uvs are full-length arrays copied or zero-filled. The extraction preserves source vertex sharing and fan-triangulates polygon faces; generated normals and planarity checks stay deferred to M3b.

**Tech Stack:** C3 0.7.11, c3c, `HalfEdgeMesh`, `cg::half_edge` validation/walk helpers, explicit `Allocator`, owned slices, optionals/faults.

**Spec:** `docs/specs/2026-05-06-m3-rendering-data-design.md`

---

## File Structure

- Modify: `project.json`
  - Add `src/render/rendering_data.c3` to `sources` after the M2 half-edge source files.
- Modify: `manifest.json`
  - Add `src/render/rendering_data.c3` to consumer library sources.
- Create: `src/render/rendering_data.c3`
  - Declares `module cg::render;`.
  - Defines `RenderingData`.
  - Defines `RenderingData.destroy(&self)`.
  - Defines `HalfEdgeMesh.to_rendering_data(&self, Allocator alloc)`.
  - May contain plain module-scope helpers. Do not use `@private` on helper free functions in this project/c3c version.
- Create: `test/test_rendering_data.c3`
  - Declares `module test;`.
  - Imports `cg`, `cg::half_edge`, and `cg::render`.
  - Owns all M3 rendering extraction tests.

Important implementation notes:

- Current source layout is `src/`; do not recreate top-level library source files.
- `project.json` includes `smoke/smoke.c3`; `manifest.json` should not.
- `mem::alloc::new_array(alloc, T, n)` returns a slice directly in this repo's verified c3c 0.7.11 style. Do not append `!` to `new_array` calls.
- Register `defer catch free(slice)` immediately after every output allocation in `to_rendering_data`.
- Use plain `defer free(face_vertices)` for the temporary scratch buffer.
- Production code must not use runtime `assert`, `unreachable`, raw `malloc`, `sizeof`, `->`, `goto`, null-as-error, or bare `catch`.
- Tests may use `assert`, `unreachable`, and `!!`.
- Commit only green states. For each implementation task: write failing test, run and observe failure, implement, run and observe pass, then commit.

---

### Task 1: Wire the render module and owned data type

**Files:**
- Modify: `project.json`
- Modify: `manifest.json`
- Create: `src/render/rendering_data.c3`
- Create: `test/test_rendering_data.c3`

- [ ] **Step 1.1: Add the first failing test for `RenderingData.destroy`**

Create `test/test_rendering_data.c3` with only the destroy test first:

```c3
module test;
import cg;
import cg::half_edge;
import cg::render;
import std::mem;

fn void test_render_destroy_zeroes_data() @test
{
    RenderingData data;
    data.vertices = mem::alloc::new_array(mem, Vec3f, 1);
    data.indices = mem::alloc::new_array(mem, uint, 1);
    data.normals = mem::alloc::new_array(mem, Vec3f, 1);
    data.uvs = mem::alloc::new_array(mem, Vec2f, 1);

    data.destroy();

    assert(data.vertices.len == 0);
    assert(data.indices.len == 0);
    assert(data.normals.len == 0);
    assert(data.uvs.len == 0);
}
```

- [ ] **Step 1.2: Run test to verify RED**

```bash
c3c test
```

Expected: compile failure because `cg::render`, `RenderingData`, or `RenderingData.destroy` does not exist yet.

- [ ] **Step 1.3: Add render source to build metadata**

In `project.json`, add the new source after `src/half_edge/validate.c3`:

```json
"src/half_edge/validate.c3",
"src/render/rendering_data.c3"
```

In `manifest.json`, add the same new source after `src/half_edge/validate.c3`:

```json
"src/half_edge/validate.c3",
"src/render/rendering_data.c3"
```

Keep `smoke/smoke.c3` only in `project.json`.

- [ ] **Step 1.4: Implement `RenderingData` and `destroy`**

Create `src/render/rendering_data.c3`:

```c3
module cg::render;
import cg;
import cg::half_edge;
import std::mem;

struct RenderingData {
    Vec3f[] vertices;
    uint[] indices;
    Vec3f[] normals;
    Vec2f[] uvs;
}

fn void RenderingData.destroy(&self)
{
    free(self.vertices);
    free(self.indices);
    free(self.normals);
    free(self.uvs);
    *self = {};
}
```

- [ ] **Step 1.5: Run test to verify GREEN**

```bash
c3c test
```

Expected: all existing tests pass plus `test_render_destroy_zeroes_data` passes. Test count should be 58.

- [ ] **Step 1.6: Commit Task 1**

```bash
git add project.json manifest.json src/render/rendering_data.c3 test/test_rendering_data.c3
git commit -m "render: add RenderingData owner type"
```

---

### Task 2: Implement valid-mesh extraction, copies, zero attrs, and fan indices

**Files:**
- Modify: `src/render/rendering_data.c3`
- Modify: `test/test_rendering_data.c3`

Task 2 implements extraction for valid meshes. It may omit the initial `self.validate()!` guard until Task 3 so Task 3 has a focused validation RED/GREEN step.

- [ ] **Step 2.1: Add valid-output tests**

Append these tests to `test/test_rendering_data.c3`:

```c3
fn void test_render_triangle_preserves_vertices_and_indices() @test
{
    Vec3f[3] positions = {
        { 0, 0, 0 },
        { 1, 0, 0 },
        { 0, 1, 0 },
    };
    uint[3] indices = { 0, 1, 2 };
    HalfEdgeMesh mesh = half_edge::from_triangles(mem, positions[..], indices[..])!!;
    defer mesh.destroy();

    RenderingData data = mesh.to_rendering_data(mem)!!;
    defer data.destroy();

    assert(data.vertices.len == 3);
    assert(data.indices.len == 3);
    assert(data.normals.len == 3);
    assert(data.uvs.len == 3);

    assert(data.vertices[0] == positions[0]);
    assert(data.vertices[1] == positions[1]);
    assert(data.vertices[2] == positions[2]);

    assert(data.indices[0] == 0);
    assert(data.indices[1] == 1);
    assert(data.indices[2] == 2);
}

fn void test_render_tetrahedron_index_count() @test
{
    HalfEdgeMesh mesh = half_edge::from_triangles(mem, TET_POSITIONS[..], TET_INDICES[..])!!;
    defer mesh.destroy();

    RenderingData data = mesh.to_rendering_data(mem)!!;
    defer data.destroy();

    assert(data.vertices.len == 4);
    assert(data.indices.len == 12);
}

fn void test_render_quad_fan_triangulates() @test
{
    Vec3f[4] positions = {
        { 0, 0, 0 },
        { 1, 0, 0 },
        { 1, 1, 0 },
        { 0, 1, 0 },
    };
    uint[2] offsets = { 0, 4 };
    uint[4] indices = { 0, 1, 2, 3 };
    HalfEdgeMesh mesh = half_edge::from_polygons(mem, positions[..], offsets[..], indices[..])!!;
    defer mesh.destroy();

    RenderingData data = mesh.to_rendering_data(mem)!!;
    defer data.destroy();

    assert(data.vertices.len == 4);
    assert(data.indices.len == 6);
    assert(data.indices[0] == 0);
    assert(data.indices[1] == 1);
    assert(data.indices[2] == 2);
    assert(data.indices[3] == 0);
    assert(data.indices[4] == 2);
    assert(data.indices[5] == 3);
}

fn void test_render_copies_normals_and_uvs() @test
{
    HalfEdgeMesh mesh = half_edge::from_triangles_with_attrs(
        mem, TET_POSITIONS[..], TET_NORMALS[..], TET_UVS[..], TET_INDICES[..])!!;
    defer mesh.destroy();

    RenderingData data = mesh.to_rendering_data(mem)!!;
    defer data.destroy();

    assert(data.normals.len == 4);
    assert(data.uvs.len == 4);
    assert(data.normals[0] == TET_NORMALS[0]);
    assert(data.normals[1] == TET_NORMALS[1]);
    assert(data.normals[2] == TET_NORMALS[2]);
    assert(data.normals[3] == TET_NORMALS[3]);
    assert(data.uvs[0] == TET_UVS[0]);
    assert(data.uvs[1] == TET_UVS[1]);
    assert(data.uvs[2] == TET_UVS[2]);
    assert(data.uvs[3] == TET_UVS[3]);
}

fn void test_render_missing_attrs_zero_filled() @test
{
    HalfEdgeMesh mesh = half_edge::from_triangles(mem, TET_POSITIONS[..], TET_INDICES[..])!!;
    defer mesh.destroy();

    RenderingData data = mesh.to_rendering_data(mem)!!;
    defer data.destroy();

    assert(data.normals.len == 4);
    assert(data.uvs.len == 4);
    for (usz i = 0; i < data.vertices.len; i++) {
        assert(data.normals[i] == (Vec3f){ 0, 0, 0 });
        assert(data.uvs[i] == (Vec2f){ 0, 0 });
    }
}
```

If c3c rejects compound vector casts such as `(Vec3f){ 0, 0, 0 }`, replace those asserts with component checks:

```c3
assert(data.normals[i][0] == 0);
assert(data.normals[i][1] == 0);
assert(data.normals[i][2] == 0);
assert(data.uvs[i][0] == 0);
assert(data.uvs[i][1] == 0);
```

- [ ] **Step 2.2: Run tests to verify RED**

```bash
c3c test
```

Expected: compile failure because `HalfEdgeMesh.to_rendering_data` does not exist yet.

- [ ] **Step 2.3: Add the extraction method**

Append this method to `src/render/rendering_data.c3`. In Task 2, this version supports valid meshes and all valid-output tests. Task 3 will add the validation guard.

```c3
fn RenderingData? HalfEdgeMesh.to_rendering_data(&self, Allocator alloc)
{
    usz triangle_count = 0;
    usz max_face_degree = 0;

    for (usz f = 0; f < self.faces.len; f++) {
        int degree = self.face_degree((FaceIndex)(int)f);
        usz face_degree = (usz)degree;
        triangle_count += face_degree - 2;
        if (face_degree > max_face_degree) max_face_degree = face_degree;
    }

    RenderingData data;
    data.vertices = mem::alloc::new_array(alloc, Vec3f, self.positions.len);
    defer catch free(data.vertices);
    data.indices = mem::alloc::new_array(alloc, uint, triangle_count * 3);
    defer catch free(data.indices);
    data.normals = mem::alloc::new_array(alloc, Vec3f, self.positions.len);
    defer catch free(data.normals);
    data.uvs = mem::alloc::new_array(alloc, Vec2f, self.positions.len);
    defer catch free(data.uvs);

    VertexIndex[] face_vertices = mem::alloc::new_array(alloc, VertexIndex, max_face_degree);
    defer free(face_vertices);

    for (usz i = 0; i < self.positions.len; i++) {
        data.vertices[i] = self.positions[i];
        if (self.normals.len > 0) {
            data.normals[i] = self.normals[i];
        } else {
            data.normals[i] = { 0, 0, 0 };
        }
        if (self.uvs.len > 0) {
            data.uvs[i] = self.uvs[i];
        } else {
            data.uvs[i] = { 0, 0 };
        }
    }

    usz out_index = 0;
    for (usz f = 0; f < self.faces.len; f++) {
        int degree = self.face_vertices((FaceIndex)(int)f, face_vertices)!;
        VertexIndex v0 = face_vertices[0];
        for (int i = 1; i < degree - 1; i++) {
            data.indices[out_index] = (uint)v0;
            out_index++;
            data.indices[out_index] = (uint)face_vertices[i];
            out_index++;
            data.indices[out_index] = (uint)face_vertices[i + 1];
            out_index++;
        }
    }

    return data;
}
```

- [ ] **Step 2.4: Run tests to verify GREEN**

```bash
c3c test
```

Expected: all tests pass. Test count should be 63: 57 M2 baseline + destroy test + 5 valid-output tests.

- [ ] **Step 2.5: Commit Task 2**

```bash
git add src/render/rendering_data.c3 test/test_rendering_data.c3
git commit -m "render: extract owned rendering data"
```

---

### Task 3: Add validation guard and ownership regression

**Files:**
- Modify: `src/render/rendering_data.c3`
- Modify: `test/test_rendering_data.c3`

- [ ] **Step 3.1: Add invalid-mesh validation test**

Append this test:

```c3
fn void test_render_invalid_mesh_returns_validation_fault() @test
{
    HalfEdgeMesh mesh = half_edge::from_triangles(mem, TET_POSITIONS[..], TET_INDICES[..])!!;
    defer mesh.destroy();

    mesh.half_edges[0].next = -2;

    if (catch err = mesh.to_rendering_data(mem)) {
        assert(err == cg::INVALID_HALF_EDGE_REFERENCE);
        return;
    }
    unreachable();
}
```

- [ ] **Step 3.2: Run the targeted test to verify RED**

```bash
c3c test test_render_invalid_mesh_returns_validation_fault
```

Expected: failure, crash, or wrong behavior because Task 2 extraction does not call `self.validate()!` yet.

If this exact targeted command is not accepted by c3c, run:

```bash
c3c test
```

and confirm the invalid-mesh test fails.

- [ ] **Step 3.3: Add validation guard**

At the start of `HalfEdgeMesh.to_rendering_data`, before counting triangles, add:

```c3
self.validate()!;
```

The method should now begin:

```c3
fn RenderingData? HalfEdgeMesh.to_rendering_data(&self, Allocator alloc)
{
    self.validate()!;

    usz triangle_count = 0;
    usz max_face_degree = 0;
    ...
}
```

- [ ] **Step 3.4: Run test to verify GREEN**

```bash
c3c test
```

Expected: all tests pass. Test count should be 64.

- [ ] **Step 3.5: Add owned-snapshot regression test**

Append this test:

```c3
fn void test_render_output_owned_independently() @test
{
    Vec3f[3] positions = {
        { 0, 0, 0 },
        { 1, 0, 0 },
        { 0, 1, 0 },
    };
    uint[3] indices = { 0, 1, 2 };
    HalfEdgeMesh mesh = half_edge::from_triangles(mem, positions[..], indices[..])!!;
    defer mesh.destroy();

    RenderingData data = mesh.to_rendering_data(mem)!!;
    defer data.destroy();

    mesh.positions[0] = { 9, 9, 9 };

    assert(data.vertices[0] == positions[0]);
    assert(data.vertices[0] != mesh.positions[0]);
}
```

- [ ] **Step 3.6: Run tests to verify GREEN**

```bash
c3c test
```

Expected: all tests pass. Test count should be 65.

If vector inequality `!=` is rejected, replace the final assert with component checks:

```c3
assert(data.vertices[0][0] != mesh.positions[0][0]);
assert(data.vertices[0][1] != mesh.positions[0][1]);
assert(data.vertices[0][2] != mesh.positions[0][2]);
```

- [ ] **Step 3.7: Commit Task 3**

```bash
git add src/render/rendering_data.c3 test/test_rendering_data.c3
git commit -m "render: validate mesh before extraction"
```

---

### Task 4: Final polish and verification

**Files:**
- Inspect: `src/render/rendering_data.c3`
- Inspect: `test/test_rendering_data.c3`
- Inspect: `project.json`
- Inspect: `manifest.json`

- [ ] **Step 4.1: Inspect final render source for anti-patterns**

Check `src/render/rendering_data.c3` manually or with search for forbidden production patterns:

```bash
python3 - <<'PY'
from pathlib import Path
p = Path('src/render/rendering_data.c3')
text = p.read_text()
for bad in ['assert(', 'unreachable', 'malloc', 'sizeof', '->', 'goto', 'null']:
    if bad in text:
        print(f'{p}: found {bad}')
PY
```

Expected: no output.

- [ ] **Step 4.2: Verify build metadata**

Confirm both metadata files include the render source once:

```bash
python3 - <<'PY'
from pathlib import Path
for name in ['project.json', 'manifest.json']:
    text = Path(name).read_text()
    count = text.count('src/render/rendering_data.c3')
    print(name, count)
    assert count == 1
PY
```

Expected:

```text
project.json 1
manifest.json 1
```

- [ ] **Step 4.3: Run full verification**

```bash
git diff --check
c3c build static-lib
c3c test
c3c build debug
c3c build release
```

Expected:

- static-lib, debug, and release builds pass.
- `c3c test` reports 65 tests passed, 0 failed, 0 skipped.

- [ ] **Step 4.4: Review final diff**

```bash
git diff --stat HEAD~3..HEAD
git diff HEAD~3..HEAD -- src/render/rendering_data.c3 test/test_rendering_data.c3 project.json manifest.json
```

Check:

- `RenderingData.destroy` frees all four slices and zeroes `self`.
- `to_rendering_data` starts with `self.validate()!`.
- Output allocations use `mem::alloc::new_array(alloc, T, len)` with no `!`.
- Output arrays use `defer catch`; scratch uses plain `defer`.
- Positions, provided normals, and provided uvs are copied.
- Missing normals and uvs are zero-filled to full length.
- Polygon fan emission matches `(v0, vi, vi+1)`.
- No M3b scope slipped in: no generated normals, no planarity checks, no new faults.

- [ ] **Step 4.5: Commit any final polish if needed**

If Step 4 finds fixes, apply them and commit:

```bash
git add src/render/rendering_data.c3 test/test_rendering_data.c3 project.json manifest.json
git commit -m "render: polish rendering data extraction"
```

If no fixes are needed, do not create an empty commit.

- [ ] **Step 4.6: Final handoff**

Report:

- branch name
- final commit SHA
- verification commands and results
- test count
- any intentionally untouched untracked files

Do not merge automatically. Use the finishing branch workflow to ask whether to merge locally, push/create PR, keep branch, or discard.
