# M3b Render Options Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Implement M3b rendering extraction options for `HalfEdgeMesh.to_rendering_data`, including generated smooth/flat normals, planar UV generation, and polygon planarity validation.

**Architecture:** Keep one public `HalfEdgeMesh.to_rendering_data(alloc, options = DEFAULT_RENDER_OPTIONS)` API in `module cg::render`. Split implementation by responsibility inside `src/render/`: public owner/API and dispatch in `rendering_data.c3`, topology/geometry validation helpers in `geometry.c3`, normal generation in `normals.c3`, and UV generation in `uvs.c3`. Shared-output modes preserve one output vertex per mesh vertex; flat-normal mode uses an expanded-output builder that duplicates every triangle corner.

**Tech Stack:** C3 0.7.11, `.c3l` library package, `c3c build static-lib`, `c3c test`, explicit `Allocator`, `mem::alloc::new_array(alloc, T, len)`, C3 faults/optionals, C3 `bitstruct` options with `uint` mode fields.

---

## Source Documents

- Design spec: `docs/specs/2026-05-07-m3b-render-options-design.md`
- Current implementation: `src/render/rendering_data.c3`
- Current tests: `test/test_rendering_data.c3`
- C3 package configs: `project.json`, `manifest.json`
- Existing topology helpers: `src/half_edge/walks.c3`, `src/half_edge/topology.c3`, `src/half_edge/validate.c3`
- Repo guidance: `AGENTS.md`
- C3 skill notes to follow:
  - `.c3i` files contain declarations only.
  - Do not add `import std::mem;` for `mem`, `free`, or `mem::alloc::new_array`.
  - Use `mem::alloc::new_array(alloc, T, len)` with explicit allocator parameters.
  - `mem::alloc::new_array` returns the slice directly in this project; do not append `!`.
  - Pair allocations that must survive success with `defer catch free(slice)`.
  - Keep helper free functions plain module-scope; do not use `@private` on helper free functions.
  - In tests, use `!!` instead of `!` inside `fn void ... @test` functions.
  - Test files are all `module test;`, so helper names and constants share one test-module namespace across all test files.

## Files and Responsibilities

### Modify `src/faults.c3i`

Add four faults to the existing `faultdef` block, before `OPEN_CELL_ON_BOUNDARY` to keep punctuation simple:

```c3
    INVALID_RENDER_OPTIONS,
    MISSING_NORMALS,
    MISSING_UVS,
    NON_PLANAR_FACE,
```

Do not create a fault named `INVALID_TRIANGLE`; that collides with existing project naming patterns and prior decisions.

### Modify `src/render/rendering_data.c3`

Keep:

- `module cg::render;`
- `import cg;`
- `import cg::half_edge;`
- `struct RenderingData`
- `fn void RenderingData.destroy(&self)`

Add in this file, in definition order after imports and before `RenderingData`. Keep `PLANAR_EPSILON` before enums because it is an independent constant. Keep `DEFAULT_RENDER_OPTIONS` after `RenderOptions` because it depends on the bitstruct type.

```c3
const float PLANAR_EPSILON = 0.00001;

enum RenderNormalMode : uint
{
    RENDER_NORMAL_NONE,
    RENDER_NORMAL_SOURCE,
    RENDER_NORMAL_SMOOTH,
    RENDER_NORMAL_FLAT,
}

enum RenderUvMode : uint
{
    RENDER_UV_NONE,
    RENDER_UV_SOURCE,
    RENDER_UV_SOURCE_OR_ZERO,
    RENDER_UV_PLANAR_PROJECT,
}

bitstruct RenderOptions : uint
{
    uint normal_mode : 0..1;
    uint uv_mode : 2..3;
    bool skip_planarity_check : 4;
    uint reserved : 5..31;
}

const RenderOptions DEFAULT_RENDER_OPTIONS = {
    .normal_mode = (uint)RENDER_NORMAL_NONE,
    .uv_mode = (uint)RENDER_UV_NONE,
    .skip_planarity_check = false,
    .reserved = 0,
};
```

If c3c 0.7.11 rejects the exact designated initializer syntax, change only the initializer spelling to the verified C3 syntax. Keep the public constant name and exact field values.

The plan uses `uint` fields for `normal_mode` and `uv_mode` intentionally because the C3 docs say bitstruct fields contain integer types and booleans. This is the spec-approved fallback representation for c3c 0.7.11. Keep the public enum types for named mode values, and cast enum values to `uint` when assigning bitstruct fields. If the syntax spike proves enum-typed bitstruct fields compile and make the code simpler, the implementer may use them only if all tests and option validation semantics remain identical.

Replace the current method signature:

```c3
fn RenderingData? HalfEdgeMesh.to_rendering_data(&self, Allocator alloc)
```

with:

```c3
fn RenderingData? HalfEdgeMesh.to_rendering_data(
    &self,
    Allocator alloc,
    RenderOptions options = DEFAULT_RENDER_OPTIONS)
```

Do not add separate public methods for smooth, flat, or UV-specific extraction.

Keep `RenderingData.destroy` unchanged except if formatting is necessary.

Implement public dispatch in this order:

1. `validate_render_options(options)!;`
2. `self.validate()!;`
3. `validate_required_render_attrs(&self, options)!;`
4. `validate_all_faces_triangular_and_non_degenerate_if_flat(&self, options)!;`
5. If `!options.skip_planarity_check`, run `validate_polygon_planarity(&self, alloc, PLANAR_EPSILON)!;`
6. If `options.normal_mode == (uint)RENDER_NORMAL_FLAT`, call `build_flat_rendering_data(&self, alloc, options)!;`
7. Otherwise call `build_shared_rendering_data(&self, alloc, options)!;`

`validate_polygon_planarity` must skip triangle faces internally. Do not skip it globally just because the mesh is likely triangular; the helper decides by face degree.

### Create `src/render/geometry.c3`

Responsibility: reusable render-geometry math, face collection, fan triangle counting, flat-mode prevalidation, planarity validation.

This file must declare:

```c3
module cg::render;
import cg;
import cg::half_edge;
import std::math;
```

If `std::math` import spelling fails, verify the current C3 docs/compiler and use the spelling that enables vector `.dot(...)`, `.length()`, `.normalize()`, scalar `math::sqrt`, and scalar `math::acos`. Do not guess silently; compile a tiny snippet first in the worktree.

Define helpers as plain module-scope functions, not `@private`.

Required helpers:

```c3
fn float dot3(Vec3f a, Vec3f b)
fn Vec3f cross3(Vec3f a, Vec3f b)
fn float length_squared3(Vec3f v)
fn float length3(Vec3f v)
fn Vec3f normalize_or_zero3(Vec3f v)
fn float clamp_float(float value, float min_value, float max_value)
fn float abs_float(float value)
fn usz max_face_degree(HalfEdgeMesh* mesh)
fn usz fan_triangle_count(HalfEdgeMesh* mesh)
fn int? collect_face_vertices(HalfEdgeMesh* mesh, FaceIndex face, VertexIndex[] out)
fn Vec3f? face_normal_from_vertices(HalfEdgeMesh* mesh, VertexIndex[] face_vertices, int degree)
fn Vec3f? face_normal(HalfEdgeMesh* mesh, FaceIndex face, VertexIndex[] scratch)
fn void? validate_polygon_planarity(HalfEdgeMesh* mesh, Allocator alloc, float epsilon)
fn void? validate_all_faces_triangular_and_non_degenerate_if_flat(HalfEdgeMesh* mesh, RenderOptions options)
```

Exact signatures may change only if the compiler requires pointer/reference spelling changes. Keep names and behavior.

Algorithm requirements:

- `cross3(a, b)` returns:

```c3
{
    a.y * b.z - a.z * b.y,
    a.z * b.x - a.x * b.z,
    a.x * b.y - a.y * b.x,
}
```

- `dot3(a, b)` returns `a.x * b.x + a.y * b.y + a.z * b.z`.
- `length_squared3(v)` returns `dot3(v, v)`.
- `length3(v)` returns square root of `length_squared3(v)` using verified math syntax.
- `normalize_or_zero3(v)` returns `{ 0, 0, 0 }` when `length_squared3(v) <= PLANAR_EPSILON * PLANAR_EPSILON`, otherwise returns `v / length3(v)`.
- `max_face_degree(mesh)` walks every face with `mesh.face_degree((FaceIndex)(int)f)` and returns the largest degree as `usz`. It may assume `self.validate()` already passed before callers use it.
- `fan_triangle_count(mesh)` sums `(usz)(degree - 2)` for every face and returns the total triangle count as `usz`. Cast after subtracting, not before, so the expression cannot underflow if a future caller accidentally asks before validation.
- `collect_face_vertices(mesh, face, out)` calls `mesh.face_vertices(face, out)!` and returns the degree as `int?`. The optional return is required because `mesh.face_vertices` can return `OUTPUT_BUFFER_TOO_SMALL` and `!` propagation is illegal from a plain `int` function.
- `face_normal_from_vertices` scans fan triangles rooted at `face_vertices[0]`:
  - For `i = 1` through `degree - 2`, compute `raw = cross3(p_i - p_0, p_next - p_0)`.
  - Return the first normalized raw normal whose squared length is greater than `PLANAR_EPSILON * PLANAR_EPSILON`.
  - If no usable raw normal exists, return `cg::DEGENERATE_INPUT~`.
- `face_normal(mesh, face, scratch)` collects vertices into `scratch` and calls `face_normal_from_vertices`.
- `validate_polygon_planarity`:
  - Allocate one `VertexIndex[] scratch` with length `max_face_degree(mesh)` using the provided allocator.
  - Use `defer free(scratch);` because it is temporary validation storage.
  - For every face, collect vertices and skip if degree is exactly 3.
  - For degree greater than 3, derive plane normal with `face_normal_from_vertices`.
  - Use `plane_origin = mesh.positions[face_vertices[0]]`.
  - For each face vertex, compute `distance = abs_float(dot3(mesh.positions[v] - plane_origin, plane_normal))`.
  - If `distance > epsilon`, return `cg::NON_PLANAR_FACE~`.
  - If plane derivation faults with `DEGENERATE_INPUT`, propagate it.
- `validate_all_faces_triangular_and_non_degenerate_if_flat`:
  - If `options.normal_mode != (uint)RENDER_NORMAL_FLAT`, return normally without work.
  - Allocate one `VertexIndex[] scratch` with length `max_face_degree(mesh)`.
  - For every face, collect vertices.
  - If degree is not 3, return `cg::NON_TRIANGLE_FACE~`.
  - For degree 3, call `face_normal_from_vertices`; propagate `cg::DEGENERATE_INPUT` if it faults.
  - This validation must run before planarity validation in `to_rendering_data` so flat mode deterministically returns `NON_TRIANGLE_FACE` for polygon meshes.

### Create `src/render/normals.c3`

Responsibility: source-normal copying, smooth-normal generation, flat-normal writes.

This file must declare:

```c3
module cg::render;
import cg;
import cg::half_edge;
import std::math;
```

Required helpers:

```c3
fn void? validate_required_render_attrs(HalfEdgeMesh* mesh, RenderOptions options)
fn void copy_source_normals(HalfEdgeMesh* mesh, Vec3f[] out_normals)
fn void? generate_smooth_normals(HalfEdgeMesh* mesh, Allocator alloc, Vec3f[] out_normals)
fn float angle_at(Vec3f a, Vec3f b, Vec3f c)
fn void write_flat_triangle_normal(HalfEdgeMesh* mesh, VertexIndex v0, VertexIndex v1, VertexIndex v2, Vec3f[] out_normals, usz out_vertex)
```

Behavior:

- `validate_required_render_attrs` checks both normals and UVs because it is the single source-attribute gate:
  - If `options.normal_mode == (uint)RENDER_NORMAL_SOURCE` and `mesh.normals.len == 0`, return `cg::MISSING_NORMALS~`.
  - If `options.uv_mode == (uint)RENDER_UV_SOURCE` and `mesh.uvs.len == 0`, return `cg::MISSING_UVS~`.
  - Do not require normals for `RENDER_NORMAL_SMOOTH` or `RENDER_NORMAL_FLAT`.
  - Do not require UVs for `RENDER_UV_SOURCE_OR_ZERO` or `RENDER_UV_PLANAR_PROJECT`.
- `copy_source_normals` copies exactly `mesh.positions.len` slots from `mesh.normals` to `out_normals`. It may assume validation already ensured `mesh.normals.len > 0` and current builders allocate output length equal to `mesh.positions.len` or expanded length as appropriate.
- `angle_at(a, b, c)` computes the angle at point `a`:
  - `ab = b - a`, `ac = c - a`.
  - If either vector length squared is `<= PLANAR_EPSILON * PLANAR_EPSILON`, return `0`.
  - Normalize both vectors.
  - Clamp their dot product into `[-1, 1]`.
  - Return `acos(clamped)` using verified C3 math syntax.
- `generate_smooth_normals`:
  - Allocate `Vec3f[] accum` length `mesh.positions.len`; `defer free(accum);`.
  - Zero every accumulator element.
  - Allocate `VertexIndex[] face_vertices` length `max_face_degree(mesh)`; `defer free(face_vertices);`.
  - For every face, collect vertices.
  - For every fan triangle `(v0, vi, vnext)`, compute `raw_normal = cross3(p_i - p_0, p_next - p_0)`.
  - Skip this triangle if `length_squared3(raw_normal) <= PLANAR_EPSILON * PLANAR_EPSILON`.
  - Normalize the face normal.
  - Accumulate `face_normal * angle_at(p0, pi, pnext)` into `accum[v0]`.
  - Accumulate `face_normal * angle_at(pi, pnext, p0)` into `accum[vi]`.
  - Accumulate `face_normal * angle_at(pnext, p0, pi)` into `accum[vnext]`.
  - After all faces, for every vertex index:
    - If `length_squared3(accum[i]) > PLANAR_EPSILON * PLANAR_EPSILON`, write `normalize_or_zero3(accum[i])` to `out_normals[i]`.
    - Otherwise write `{ 0, 0, 0 }`.
- `write_flat_triangle_normal`:
  - Compute `raw_normal = cross3(p1 - p0, p2 - p0)`.
  - Normalize with `normalize_or_zero3`.
  - Write the same normalized normal to `out_normals[out_vertex]`, `[out_vertex + 1]`, and `[out_vertex + 2]`.
  - It may assume prevalidation already rejected degenerate triangles.

### Create `src/render/uvs.c3`

Responsibility: source UV copying, source-or-zero behavior, planar projection, shared-to-flat expansion support.

This file must declare:

```c3
module cg::render;
import cg;
import cg::half_edge;
```

Required helpers:

```c3
fn void fill_zero_uvs(Vec2f[] out_uvs)
fn void copy_source_uvs(HalfEdgeMesh* mesh, Vec2f[] out_uvs)
fn void copy_or_zero_source_uvs(HalfEdgeMesh* mesh, Vec2f[] out_uvs)
fn void? project_planar_uvs(HalfEdgeMesh* mesh, Allocator alloc, Vec2f[] out_uvs)
fn Vec2f project_position_to_axis(Vec3f position, int axis)
fn int choose_projection_axis(HalfEdgeMesh* mesh, Allocator alloc)
```

Axis constants must be named module-scope constants:

```c3
const int PROJECT_DROP_X = 0;
const int PROJECT_DROP_Y = 1;
const int PROJECT_DROP_Z = 2;
```

Behavior:

- `fill_zero_uvs` writes `{ 0, 0 }` to every output slot.
- `copy_source_uvs` copies `mesh.uvs[i]` into `out_uvs[i]` for every mesh vertex. It assumes validation already ensured source UVs exist and output length matches either shared vertices or a temporary shared UV buffer.
- `copy_or_zero_source_uvs` copies source UVs if `mesh.uvs.len > 0`, otherwise zero-fills.
- `choose_projection_axis`:
  - Allocate `VertexIndex[] face_vertices` length `max_face_degree(mesh)`; `defer free(face_vertices);`.
  - Initialize `normal_sum = { 0, 0, 0 }`.
  - Initialize `bool contributed = false`.
  - For every face, call `face_normal(mesh, face, face_vertices)`.
  - If the face normal is present, add it to `normal_sum` and set `contributed = true`.
  - If the face normal faults with `cg::DEGENERATE_INPUT`, skip that face for projection-axis selection. Do not return a UV-generation fault for an individual degenerate face; planarity validation handles degenerate polygon faults when planarity is enabled, and axis selection has a mesh-level fallback.
  - If `!contributed` or `length_squared3(normal_sum) <= PLANAR_EPSILON * PLANAR_EPSILON`, return `PROJECT_DROP_Z`.
  - Compute absolute components with `abs_float`.
  - If `abs_z >= abs_x && abs_z >= abs_y`, return `PROJECT_DROP_Z`.
  - Else if `abs_y >= abs_x && abs_y > abs_z`, return `PROJECT_DROP_Y`.
  - Else return `PROJECT_DROP_X`.
- `project_position_to_axis`:
  - `PROJECT_DROP_Z`: return `{ position.x, position.y }`.
  - `PROJECT_DROP_Y`: return `{ position.x, position.z }`.
  - `PROJECT_DROP_X`: return `{ position.y, position.z }`.
- `project_planar_uvs`:
  - Call `choose_projection_axis`.
  - Compute raw projected `Vec2f` for every mesh vertex.
  - Compute `min_u`, `max_u`, `min_v`, `max_v` across all raw projected coordinates.
  - For every vertex:
    - `u = 0` if `abs_float(width) <= PLANAR_EPSILON`, else `(raw_u - min_u) / width`.
    - `v = 0` if `abs_float(height) <= PLANAR_EPSILON`, else `(raw_v - min_v) / height`.
    - Write `{ u, v }` to `out_uvs[i]`.
- For flat output with planar UVs, build shared projected UVs in a temporary `Vec2f[] shared_uvs` length `mesh.positions.len`, then expand by original vertex index.

### Modify `project.json`

Add the new source files immediately after `src/render/rendering_data.c3` or replace the single render source entry with this ordered group:

```json
    "src/render/rendering_data.c3",
    "src/render/geometry.c3",
    "src/render/normals.c3",
    "src/render/uvs.c3"
```

Keep valid JSON commas.

### Modify `manifest.json`

Add the same new render source files in the consumer source list:

```json
    "src/render/rendering_data.c3",
    "src/render/geometry.c3",
    "src/render/normals.c3",
    "src/render/uvs.c3"
```

Keep the existing JSON-with-comments style. Do not change unrelated target entries.

### Modify `test/test_rendering_data.c3`

Update existing tests and add new tests. Do not create a second rendering test file unless same-module symbol collisions become hard to manage; one file keeps all render fixtures local.

Add helper functions near the top after imports:

```c3
fn bool approx(float a, float b)
fn bool approx_eps(float a, float b, float epsilon)
fn bool vec2_approx(Vec2f a, Vec2f b)
fn bool vec3_approx(Vec3f a, Vec3f b)
fn float test_dot3(Vec3f a, Vec3f b)
fn float test_len_sq3(Vec3f v)
```

Use a local epsilon constant name that does not collide with other test files, for example:

```c3
const float RENDER_TEST_EPSILON = 0.0001;
```

Do not reuse names that might exist in other `module test;` files without checking with `search_files` first.

Required test changes:

1. `test_render_triangle_preserves_vertices_and_indices`
   - Keep vertex and index assertions.
   - Change expected default attribute lengths to `data.normals.len == 0` and `data.uvs.len == 0`.
2. `test_render_tetrahedron_index_count`
   - Add `assert(data.normals.len == 0);` and `assert(data.uvs.len == 0);` for default behavior.
3. `test_render_quad_fan_triangulates`
   - Keep index assertions.
   - Add `assert(data.normals.len == 0);` and `assert(data.uvs.len == 0);`.
4. Replace `test_render_copies_normals_and_uvs` with two explicit tests:
   - `test_render_source_normals_copy_when_present`
   - `test_render_source_uvs_copy_when_present`
   - Both must pass explicit options.
5. Replace `test_render_missing_attrs_zero_filled` with:
   - `test_render_source_normals_missing_faults`
   - `test_render_source_uvs_missing_faults`
   - `test_render_source_or_zero_uvs_zero_fill_when_missing`

Required new tests:

- `test_render_default_options_emit_no_attrs`
- `test_render_source_normals_copy_when_present`
- `test_render_source_normals_missing_faults`
- `test_render_smooth_normals_are_unit_length_on_tetrahedron`
- `test_render_smooth_normals_are_angle_weighted`
- `test_render_flat_normals_expand_triangle_mesh`
- `test_render_flat_normals_fault_on_polygon_mesh`
- `test_render_flat_normals_preserve_hard_edges_by_expansion`
- `test_render_source_uvs_copy_when_present`
- `test_render_source_uvs_missing_faults`
- `test_render_source_or_zero_uvs_zero_fill_when_missing`
- `test_render_planar_project_uvs_normalize_tilted_quad`
- `test_render_flat_source_uvs_expand_by_original_vertex`
- `test_render_flat_planar_project_uvs_expand_from_original_vertices`
- `test_render_non_planar_polygon_faults`
- `test_render_skip_planarity_allows_non_planar_polygon_fan`
- `test_render_degenerate_polygon_plane_faults`
- `test_render_flat_degenerate_triangle_faults_before_allocation_path`
- `test_render_invalid_options_reserved_bits_fault`

Use explicit options in tests. Example option construction:

```c3
RenderOptions options = DEFAULT_RENDER_OPTIONS;
options.normal_mode = (uint)RENDER_NORMAL_SOURCE;
```

If c3c requires enum constants to be qualified or cast differently, adjust every test consistently.

For fault tests, use this pattern:

```c3
if (catch err = mesh.to_rendering_data(mem, options)) {
    assert(err == cg::MISSING_NORMALS);
    return;
}
unreachable();
```

For invalid reserved bits, use direct cast into bitstruct if supported:

```c3
RenderOptions options = (RenderOptions)(1u << 5);
```

If direct cast syntax fails, use a verified alternative to set `reserved` non-zero, such as:

```c3
RenderOptions options = DEFAULT_RENDER_OPTIONS;
options.reserved = 1;
```

The test must prove non-zero reserved bits return `cg::INVALID_RENDER_OPTIONS`.

## C3 Syntax Spike Requirement

Before implementing Task 1 production code, the implementer must verify these syntax points in the worktree using a tiny temporary file or direct project tests:

1. Bitstruct with `uint` backing type, `uint` bit ranges, `bool` bit, and `reserved : 5..31` compiles.
2. `RenderOptions options = DEFAULT_RENDER_OPTIONS; options.normal_mode = (uint)RENDER_NORMAL_SOURCE;` compiles.
3. Casting from `uint` to `RenderOptions` compiles, or a fallback way to set `reserved` is identified.
4. `import std::math;` enables scalar square root and arccosine calls, or manual helper functions use the correct verified math calls. The spike must explicitly test the exact spelling used later for both square root and `acos`.
5. Both bitstruct designated initializer forms are tested: `.field = value` and `field = value`. Use the form c3c 0.7.11 accepts.
6. Vector swizzles `.x`, `.y`, `.z` compile for `Vec3f` and `Vec2f`.

Delete temporary spike files before committing. If the spike discovers syntax different from this plan, implement the verified syntax and mention it in the task handoff summary.

## Implementation Tasks

### Task 1: Render options API, faults, and default/source attribute behavior

**Files:**
- Modify: `src/faults.c3i`
- Modify: `src/render/rendering_data.c3`
- Modify: `src/render/normals.c3` (create)
- Modify: `src/render/uvs.c3` (create)
- Modify: `project.json`
- Modify: `manifest.json`
- Test: `test/test_rendering_data.c3`

**Goal:** Add public options/faults and make shared-output extraction support default no-attrs, source normals/UVs, source-or-zero UVs, and option validation. Do not implement smooth normals, flat normals, planar UVs, or planarity yet beyond stubs that fault or are unreachable from tests.

- [ ] **Step 1: Run baseline tests in current worktree**

Run:

```bash
c3c build static-lib && c3c test
```

Expected: existing suite passes before edits.

- [ ] **Step 2: Perform C3 syntax spike for bitstruct and math import**

Create a temporary spike file outside tracked source or use a short edit that will be deleted before commit. Verify the five syntax points listed in `C3 Syntax Spike Requirement`.

Run:

```bash
c3c build static-lib
```

Expected: spike compiles or produces concrete syntax corrections. Delete spike before proceeding.

- [ ] **Step 3: Write failing tests for default/source/invalid option behavior**

Update `test/test_rendering_data.c3`:

- Update existing default extraction tests to expect `normals.len == 0` and `uvs.len == 0`.
- Add `test_render_source_normals_copy_when_present`.
- Add `test_render_source_normals_missing_faults`.
- Add `test_render_source_uvs_copy_when_present`.
- Add `test_render_source_uvs_missing_faults`.
- Add `test_render_source_or_zero_uvs_zero_fill_when_missing`.
- Add `test_render_invalid_options_reserved_bits_fault`.

Example source-normal test shape:

```c3
fn void test_render_source_normals_copy_when_present() @test
{
    HalfEdgeMesh mesh = half_edge::from_triangles_with_attrs(
        mem, TET_POSITIONS[..], TET_NORMALS[..], TET_UVS[..], TET_INDICES[..])!!;
    defer mesh.destroy();

    RenderOptions options = DEFAULT_RENDER_OPTIONS;
    options.normal_mode = (uint)RENDER_NORMAL_SOURCE;

    RenderingData data = mesh.to_rendering_data(mem, options)!!;
    defer data.destroy();

    assert(data.normals.len == mesh.positions.len);
    assert(data.uvs.len == 0);
    assert(data.normals[0] == TET_NORMALS[0]);
    assert(data.normals[1] == TET_NORMALS[1]);
    assert(data.normals[2] == TET_NORMALS[2]);
    assert(data.normals[3] == TET_NORMALS[3]);
}
```

- [ ] **Step 4: Run tests and verify RED**

Run:

```bash
c3c test
```

Expected: fails because `RenderOptions`, new enum values, new faults, or the new method signature do not exist yet. If tests pass, the tests are wrong; strengthen them.

- [ ] **Step 5: Implement faults and public option types**

Modify `src/faults.c3i` with the four new faults.

Modify `src/render/rendering_data.c3` with `RenderNormalMode`, `RenderUvMode`, `RenderOptions`, `PLANAR_EPSILON`, and `DEFAULT_RENDER_OPTIONS` exactly as described in the file responsibilities section, using verified syntax.

- [ ] **Step 6: Create render helper files and update configs**

Create `src/render/normals.c3` with `validate_required_render_attrs` and `copy_source_normals`.

Create `src/render/uvs.c3` with `fill_zero_uvs`, `copy_source_uvs`, and `copy_or_zero_source_uvs`.

Update `project.json` and `manifest.json` to include the new source files.

- [ ] **Step 7: Implement option validation and shared builder for supported modes**

In `src/render/rendering_data.c3`:

- Add `fn void? validate_render_options(RenderOptions options)`:
  - If `options.reserved != 0`, return `cg::INVALID_RENDER_OPTIONS~`.
  - If `options.normal_mode > (uint)RENDER_NORMAL_FLAT`, return `cg::INVALID_RENDER_OPTIONS~`.
  - If `options.uv_mode > (uint)RENDER_UV_PLANAR_PROJECT`, return `cg::INVALID_RENDER_OPTIONS~`.
- Add `fn RenderingData? build_shared_rendering_data(HalfEdgeMesh* mesh, Allocator alloc, RenderOptions options)`.
- Shared builder must allocate:
  - `data.vertices` length `mesh.positions.len` always.
  - `data.indices` length `fan_triangle_count(mesh) * 3`; for Task 1 this can keep current local triangle counting until `geometry.c3` lands, but the final code must use `fan_triangle_count` from Task 2.
  - `data.normals` length `0` for NONE, length `mesh.positions.len` for SOURCE only in this task.
  - `data.uvs` length `0` for NONE, length `mesh.positions.len` for SOURCE and SOURCE_OR_ZERO only in this task.
- Copy positions and fan indices as M3 did.
- For `RENDER_NORMAL_SOURCE`, call `copy_source_normals`.
- For `RENDER_UV_SOURCE`, call `copy_source_uvs`.
- For `RENDER_UV_SOURCE_OR_ZERO`, call `copy_or_zero_source_uvs`.
- If `RENDER_NORMAL_SMOOTH`, `RENDER_NORMAL_FLAT`, or `RENDER_UV_PLANAR_PROJECT` is requested before later tasks implement them, return `cg::INVALID_RENDER_OPTIONS~` only in this task. Later tasks must remove these temporary faults and implement the modes.

- [ ] **Step 8: Run GREEN verification for Task 1**

Run:

```bash
c3c build static-lib && c3c test
```

Expected: Task 1 tests pass. Tests for future modes should not exist yet or should remain pending for their tasks.

- [ ] **Step 9: Self-review Task 1**

Check:

```bash
git diff --check
git diff -- src/faults.c3i src/render/rendering_data.c3 src/render/normals.c3 src/render/uvs.c3 project.json manifest.json test/test_rendering_data.c3
```

Confirm:

- No `@private` helper functions.
- No `import std::mem;`.
- No production `assert()`.
- No null/sentinel error returns.
- New allocations use `defer catch` if returned in `RenderingData`, and `defer free` for temporary arrays.

- [ ] **Step 10: Commit Task 1**

```bash
git add src/faults.c3i src/render/rendering_data.c3 src/render/normals.c3 src/render/uvs.c3 project.json manifest.json test/test_rendering_data.c3
git commit -m "render: add render options API (M3b)"
```

### Task 2: Render geometry helpers and polygon planarity validation

**Files:**
- Create: `src/render/geometry.c3`
- Modify: `src/render/rendering_data.c3`
- Modify: `project.json`
- Modify: `manifest.json`
- Test: `test/test_rendering_data.c3`

**Goal:** Add reusable geometry helpers and enable default polygon planarity validation with opt-out.

- [ ] **Step 1: Write failing planarity tests**

Add:

```c3
fn void test_render_non_planar_polygon_faults() @test
```

Fixture:

- positions: `{ {0,0,0}, {1,0,0}, {1,1,0.1}, {0,1,0} }`
- offsets: `{ 0, 4 }`
- indices: `{ 0, 1, 2, 3 }`
- default options.
- Expected fault: `cg::NON_PLANAR_FACE`.

Add:

```c3
fn void test_render_skip_planarity_allows_non_planar_polygon_fan() @test
```

Same fixture, but:

```c3
RenderOptions options = DEFAULT_RENDER_OPTIONS;
options.skip_planarity_check = true;
```

Expected success:

- `data.vertices.len == 4`
- `data.indices.len == 6`
- `data.normals.len == 0`
- `data.uvs.len == 0`

Add:

```c3
fn void test_render_degenerate_polygon_plane_faults() @test
```

Fixture:

- Four collinear positions on X axis.
- One polygon face `{0,1,2,3}`.
- Default options.
- Expected fault: `cg::DEGENERATE_INPUT`.

- [ ] **Step 2: Run tests and verify RED**

```bash
c3c test
```

Expected: tests fail because planarity validation is not implemented and `src/render/geometry.c3` does not exist.

- [ ] **Step 3: Create `src/render/geometry.c3`**

Implement all helpers listed in the file responsibilities section.

If `std::math` import or scalar math syntax differs, use the syntax verified in Task 1 spike. Keep helper names and behavior unchanged.

- [ ] **Step 4: Update configs for `geometry.c3`**

Add `src/render/geometry.c3` to both `project.json` and `manifest.json` if Task 1 did not already add it.

- [ ] **Step 5: Wire planarity validation into `to_rendering_data`**

In `src/render/rendering_data.c3` dispatch, after source-attr validation and before builder dispatch:

```c3
validate_all_faces_triangular_and_non_degenerate_if_flat(&self, options)!;
if (!options.skip_planarity_check) {
    validate_polygon_planarity(&self, alloc, PLANAR_EPSILON)!;
}
```

For Task 2, flat validation helper can return normally for all non-flat options; Task 4 will add its full behavior if not implemented here.

Replace local max-face-degree and fan-triangle-count calculations in shared builder with `max_face_degree(mesh)` and `fan_triangle_count(mesh)`.

- [ ] **Step 6: Run GREEN verification for Task 2**

```bash
c3c build static-lib && c3c test
```

Expected: all tests through Task 2 pass.

- [ ] **Step 7: Self-review Task 2**

Check:

```bash
git diff --check
git diff -- src/render/geometry.c3 src/render/rendering_data.c3 project.json manifest.json test/test_rendering_data.c3
```

Confirm:

- `validate_polygon_planarity` skips degree-3 faces.
- `DEGENERATE_INPUT` comes from no usable face normal.
- `NON_PLANAR_FACE` comes only from distance greater than epsilon.
- Temporary scratch arrays are freed on all paths.

- [ ] **Step 8: Commit Task 2**

```bash
git add src/render/geometry.c3 src/render/rendering_data.c3 project.json manifest.json test/test_rendering_data.c3
git commit -m "render: validate polygon planarity (M3b)"
```

### Task 3: Smooth angle-weighted normals

**Files:**
- Modify: `src/render/normals.c3`
- Modify: `src/render/rendering_data.c3`
- Test: `test/test_rendering_data.c3`

**Goal:** Implement `RENDER_NORMAL_SMOOTH` in shared-output mode.

- [ ] **Step 1: Write failing smooth-normal tests**

Add `test_render_smooth_normals_are_unit_length_on_tetrahedron`:

- Build tetrahedron from `TET_POSITIONS` and `TET_INDICES`.
- Options: `normal_mode = (uint)RENDER_NORMAL_SMOOTH`.
- Expected:
  - `data.vertices.len == 4`
  - `data.indices.len == 12`
  - `data.normals.len == 4`
  - `data.uvs.len == 0`
  - every normal has length approximately `1` using `RENDER_TEST_EPSILON`.

Add `test_render_smooth_normals_are_angle_weighted` with this fixture:

```c3
Vec3f[4] positions = {
    { 0, 0, 0 },
    { 2, 0, 0 },
    { 0, 1, 0 },
    { 0, 0, 1 },
};
uint[6] indices = { 0, 1, 2, 0, 3, 1 };
```

Options: smooth normals.

Expected for vertex `0`:

- First triangle contributes normal `{0, 0, 1}` with angle `atan` equivalent of 90 degrees.
- Second triangle contributes normal `{0, 1, 0}` with angle 90 degrees if winding is `{0,3,1}`; verify expected sign from actual cross product.
- Expected normalized result is approximately `{0, 0.7071067, 0.7071067}` if both normals are positive. If actual winding makes one component negative, set the test expectation to the mathematically correct winding result and document it in a short test-local variable name, not a prose comment.

Do not assert exact floating-point equality for generated normals.

- [ ] **Step 2: Run tests and verify RED**

```bash
c3c test
```

Expected: smooth tests fail because smooth mode still faults or outputs no generated normals.

- [ ] **Step 3: Implement `angle_at`**

In `src/render/normals.c3`, implement the exact algorithm:

- zero-length edge -> `0`
- normalize edge vectors
- clamp dot product
- `acos`

- [ ] **Step 4: Implement `generate_smooth_normals`**

Follow the algorithm in the file responsibilities section. Reuse `collect_face_vertices`, `max_face_degree`, `cross3`, `length_squared3`, `normalize_or_zero3`, and `angle_at`.

- [ ] **Step 5: Wire smooth mode into shared builder**

In `build_shared_rendering_data`:

- Allocate normals length `mesh.positions.len` when `options.normal_mode == (uint)RENDER_NORMAL_SMOOTH`.
- Call `generate_smooth_normals(mesh, alloc, data.normals)!;`.
- Remove any temporary `INVALID_RENDER_OPTIONS` fallback for smooth mode from Task 1.

- [ ] **Step 6: Run GREEN verification for Task 3**

```bash
c3c build static-lib && c3c test
```

Expected: smooth tests and all prior tests pass.

- [ ] **Step 7: Self-review Task 3**

Check:

```bash
git diff --check
git diff -- src/render/normals.c3 src/render/rendering_data.c3 test/test_rendering_data.c3
```

Confirm:

- Smooth normals use fan triangles exactly like indices.
- Degenerate fan triangles are skipped, not faulted.
- Zero accumulators become `{0, 0, 0}`.
- No area weighting was accidentally used.

- [ ] **Step 8: Commit Task 3**

```bash
git add src/render/normals.c3 src/render/rendering_data.c3 test/test_rendering_data.c3
git commit -m "render: generate smooth normals (M3b)"
```

### Task 4: Planar-projected UVs in shared output

**Files:**
- Modify: `src/render/uvs.c3`
- Modify: `src/render/rendering_data.c3`
- Test: `test/test_rendering_data.c3`

**Goal:** Implement `RENDER_UV_PLANAR_PROJECT` for shared-output rendering data.

- [ ] **Step 1: Write failing planar UV test**

Add `test_render_planar_project_uvs_normalize_tilted_quad`:

Fixture:

```c3
Vec3f[4] positions = {
    { 0, 0, 0 },
    { 2, 0, 0 },
    { 2, 2, 1 },
    { 0, 2, 1 },
};
uint[2] offsets = { 0, 4 };
uint[4] indices = { 0, 1, 2, 3 };
```

This tilted quad has a dominant normal with a large Z component, so projection should drop Z and use XY.

Options:

```c3
RenderOptions options = DEFAULT_RENDER_OPTIONS;
options.uv_mode = (uint)RENDER_UV_PLANAR_PROJECT;
```

Expected:

- `data.uvs.len == 4`
- `data.uvs[0] ~= {0, 0}`
- `data.uvs[1] ~= {1, 0}`
- `data.uvs[2] ~= {1, 1}`
- `data.uvs[3] ~= {0, 1}`
- `data.normals.len == 0`

- [ ] **Step 2: Run tests and verify RED**

```bash
c3c test
```

Expected: planar UV test fails because planar projection is not implemented.

- [ ] **Step 3: Implement UV projection helpers**

In `src/render/uvs.c3`, implement:

- `PROJECT_DROP_X`, `PROJECT_DROP_Y`, `PROJECT_DROP_Z`
- `choose_projection_axis`
- `project_position_to_axis`
- `project_planar_uvs`

Use the exact axis selection and normalization behavior from the file responsibilities section.

- [ ] **Step 4: Wire planar UV mode into shared builder**

In `build_shared_rendering_data`:

- Allocate `data.uvs` length `mesh.positions.len` for planar UV mode.
- Call `project_planar_uvs(mesh, alloc, data.uvs)!;`.
- Remove any temporary `INVALID_RENDER_OPTIONS` fallback for planar UV mode from Task 1.

- [ ] **Step 5: Run GREEN verification for Task 4**

```bash
c3c build static-lib && c3c test
```

Expected: planar UV test and all prior tests pass.

- [ ] **Step 6: Self-review Task 4**

Check:

```bash
git diff --check
git diff -- src/render/uvs.c3 src/render/rendering_data.c3 test/test_rendering_data.c3
```

Confirm:

- UV projection is mesh-level, not per-face.
- Flat-output expansion is not implemented in this task.
- Degenerate face normals in projection are skipped for axis selection; if no face contributes or the summed normal is near zero, projection falls back to drop-Z / XY.
- Width or height near zero yields coordinate `0`, not a fault.

- [ ] **Step 7: Commit Task 4**

```bash
git add src/render/uvs.c3 src/render/rendering_data.c3 test/test_rendering_data.c3
git commit -m "render: generate planar uvs (M3b)"
```

### Task 5: Flat-normal expanded-output path

**Files:**
- Modify: `src/render/rendering_data.c3`
- Modify: `src/render/geometry.c3`
- Modify: `src/render/normals.c3`
- Modify: `src/render/uvs.c3`
- Test: `test/test_rendering_data.c3`

**Goal:** Implement `RENDER_NORMAL_FLAT` with expanded positions, sequential indices, flat normals, and expanded UVs for all UV modes.

- [ ] **Step 1: Write failing flat-output tests**

Add `test_render_flat_normals_expand_triangle_mesh`:

- Mesh: tetrahedron from `TET_POSITIONS` and `TET_INDICES`.
- Options: `normal_mode = (uint)RENDER_NORMAL_FLAT`.
- Expected:
  - `data.vertices.len == 12`
  - `data.indices.len == 12`
  - `data.normals.len == 12`
  - `data.uvs.len == 0`
  - for every `i`, `data.indices[i] == (uint)i`
  - every normal length is approximately `1`

Add `test_render_flat_normals_fault_on_polygon_mesh`:

- Mesh: planar quad polygon.
- Options: flat normals.
- Expected fault: `cg::NON_TRIANGLE_FACE`.

Add `test_render_flat_normals_preserve_hard_edges_by_expansion`:

- Mesh: two triangles sharing an edge with different face normals.
- Options: flat normals.
- Expected:
  - Shared original vertex appears in multiple output slots.
  - Output normals for that original vertex's slots differ between the two face triangles.
  - This proves hard edges are represented by expansion, not smoothing.

Add `test_render_flat_source_uvs_expand_by_original_vertex`:

- Build a triangle mesh with source UVs using `from_triangles_with_attrs`.
- Options: flat normals + `RENDER_UV_SOURCE`.
- Expected:
  - `data.vertices.len == 12` for the tetrahedron fixture, or `3` for a one-triangle fixture.
  - `data.uvs.len == data.vertices.len`.
  - For each emitted triangle slot, the UV equals `mesh.uvs[original_vertex_index]` for that slot's source vertex.

Add `test_render_flat_planar_project_uvs_expand_from_original_vertices`:

- Mesh: one triangle or tetrahedron with flat normals and planar UV mode.
- Expected:
  - `data.vertices.len == data.uvs.len`
  - For each output slot corresponding to an original vertex, UV equals shared projected UV of that original vertex.

Extend or add a flat `SOURCE_OR_ZERO` UV test:

- Options: flat normals + `RENDER_UV_SOURCE_OR_ZERO` on a mesh without source UVs.
- Expected: `data.uvs.len == data.vertices.len` and every UV is `{0, 0}`.

Add `test_render_flat_degenerate_triangle_faults_before_allocation_path`:

- Mesh from one degenerate triangle with collinear positions if builder permits it.
- Options: flat normals.
- Expected fault: `cg::DEGENERATE_INPUT`.
- If `from_triangles` rejects the mesh earlier with `DEGENERATE_INPUT`, assert that same fault and note in the test name if needed.

- [ ] **Step 2: Run tests and verify RED**

```bash
c3c test
```

Expected: flat tests fail because flat mode is not implemented.

- [ ] **Step 3: Complete flat prevalidation**

In `validate_all_faces_triangular_and_non_degenerate_if_flat`:

- Return immediately for non-flat normal modes.
- For flat mode, collect every face.
- Return `cg::NON_TRIANGLE_FACE~` if any degree is not 3.
- Return `cg::DEGENERATE_INPUT~` if any triangular face cannot produce a non-zero normal.

- [ ] **Step 4: Implement `build_flat_rendering_data`**

In `src/render/rendering_data.c3`:

- `triangle_count = self.faces.len` is valid after flat prevalidation because every face degree is 3.
- Allocate:
  - `data.vertices` length `triangle_count * 3`
  - `data.indices` length `triangle_count * 3`
  - `data.normals` length `triangle_count * 3`
  - `data.uvs` length `0` for `RENDER_UV_NONE`, otherwise `triangle_count * 3`
- Use `defer catch free(...)` for every returned slice allocation, immediately after the allocation line. Example:

```c3
data.vertices = mem::alloc::new_array(alloc, Vec3f, triangle_count * 3);
defer catch free(data.vertices);
```
- Allocate one `VertexIndex[] face_vertices` scratch length `3` or `max_face_degree(mesh)`; `defer free(face_vertices);`.
- If `options.uv_mode == (uint)RENDER_UV_PLANAR_PROJECT`, allocate temporary `Vec2f[] shared_uvs` length `mesh.positions.len`, `defer free(shared_uvs);`, and call `project_planar_uvs(mesh, alloc, shared_uvs)!;` before writing slots.
- For every face:
  - collect its three original vertex indices as `v0`, `v1`, `v2`.
  - write `mesh.positions[v0]`, `mesh.positions[v1]`, `mesh.positions[v2]` into consecutive output vertices.
  - write sequential indices `(uint)out`, `(uint)(out + 1)`, `(uint)(out + 2)`.
  - call `write_flat_triangle_normal(mesh, v0, v1, v2, data.normals, out)`.
  - If UV mode is SOURCE, copy `mesh.uvs[v0]`, `mesh.uvs[v1]`, `mesh.uvs[v2]`.
  - If UV mode is SOURCE_OR_ZERO and source UVs exist, copy them; if missing, write `{0,0}` for all three slots.
  - If UV mode is PLANAR_PROJECT, copy `shared_uvs[v0]`, `shared_uvs[v1]`, `shared_uvs[v2]`.
  - Increase `out` by 3.

- [ ] **Step 5: Wire flat mode dispatch**

In `to_rendering_data`, ensure:

```c3
if (options.normal_mode == (uint)RENDER_NORMAL_FLAT) {
    return build_flat_rendering_data(&self, alloc, options)!;
}
return build_shared_rendering_data(&self, alloc, options)!;
```

Remove any temporary flat-mode `INVALID_RENDER_OPTIONS` fallback from Task 1.

- [ ] **Step 6: Run GREEN verification for Task 5**

```bash
c3c build static-lib && c3c test
```

Expected: flat tests and all prior tests pass.

- [ ] **Step 7: Self-review Task 5**

Check:

```bash
git diff --check
git diff -- src/render/rendering_data.c3 src/render/geometry.c3 src/render/normals.c3 src/render/uvs.c3 test/test_rendering_data.c3
```

Confirm:

- Flat polygon faults with `NON_TRIANGLE_FACE` before planarity.
- Flat path never emits shared indices.
- `data.indices[i] == i` for all flat output.
- Flat UVs expand to `data.vertices.len` whenever UV mode is not NONE.
- Temporary `shared_uvs` is freed on success and fault paths.

- [ ] **Step 8: Commit Task 5**

```bash
git add src/render/rendering_data.c3 src/render/geometry.c3 src/render/normals.c3 src/render/uvs.c3 test/test_rendering_data.c3
git commit -m "render: add flat rendering output (M3b)"
```

### Task 6: Final integration, cleanup, and full M3b verification

**Files:**
- Modify as needed: `src/render/rendering_data.c3`
- Modify as needed: `src/render/geometry.c3`
- Modify as needed: `src/render/normals.c3`
- Modify as needed: `src/render/uvs.c3`
- Modify as needed: `test/test_rendering_data.c3`
- Modify as needed: `project.json`
- Modify as needed: `manifest.json`

**Goal:** Remove temporary scaffolding, ensure all specified M3b behavior is tested, and make final verification green.

- [ ] **Step 1: Audit required behavior against the spec**

Create a local checklist in the task handoff summary, not in the repo, covering:

- Default options: positions + indices only; no normals/UVs.
- Source normals copy when present.
- Source normals missing -> `MISSING_NORMALS`.
- Smooth normals generated and unit-length for non-degenerate connected vertices.
- Flat normals expand output and preserve hard edges.
- Flat normals on polygon -> `NON_TRIANGLE_FACE`.
- Source UVs copy when present.
- Source UVs missing -> `MISSING_UVS`.
- SOURCE_OR_ZERO UVs zero-fill when missing in shared and flat paths.
- Planar UVs normalize to `[0, 1]` and expand in flat path.
- Non-planar polygon -> `NON_PLANAR_FACE` when planarity enabled.
- `skip_planarity_check` allows non-planar polygon fan extraction.
- Degenerate polygon plane -> `DEGENERATE_INPUT`.
- Invalid reserved bits -> `INVALID_RENDER_OPTIONS`.
- Existing invalid mesh validation faults still propagate.

- [ ] **Step 2: Remove temporary mode rejection code**

Search for any temporary `INVALID_RENDER_OPTIONS` returns that reject valid mode combinations:

```bash
rg "INVALID_RENDER_OPTIONS|RENDER_NORMAL_SMOOTH|RENDER_NORMAL_FLAT|RENDER_UV_PLANAR_PROJECT" src/render test/test_rendering_data.c3
```

Use `search_files`, not shell `rg`, if running under the main agent. Subagents may use shell if their environment allows it.

Valid `INVALID_RENDER_OPTIONS` conditions after final implementation:

- `options.reserved != 0`
- fallback integer mode out-of-range if such values can be constructed

No valid normal/UV mode combination should return `INVALID_RENDER_OPTIONS`.

- [ ] **Step 3: Run targeted render tests**

```bash
c3c test
```

Expected: all render and non-render tests pass.

- [ ] **Step 4: Run full verification**

```bash
git diff --check
c3c build static-lib
c3c test
c3c build debug
c3c build release
```

Expected: every command exits 0.

- [ ] **Step 5: Final self-review**

Inspect diffs:

```bash
git diff -- src/faults.c3i src/render/rendering_data.c3 src/render/geometry.c3 src/render/normals.c3 src/render/uvs.c3 test/test_rendering_data.c3 project.json manifest.json
```

Confirm:

- File definition order follows repo style: types/enums/constants/structs/methods/free functions.
- No production `assert()` or `unreachable()`.
- No raw `malloc`/`free` allocations beyond project-standard `free(slice)` cleanup for `mem::alloc::new_array` slices.
- No `@private` on helper free functions.
- No `null` error signals.
- No `sizeof(T)`.
- No `->` pointer access.
- No goto cleanup.
- `project.json` and `manifest.json` both list all new render source files.
- All returned `RenderingData` allocations are cleaned up with `defer catch` on fault.
- All temporary arrays are cleaned up with `defer free`.

- [ ] **Step 6: Commit Task 6**

If Task 6 made code/test/config changes:

```bash
git add src/faults.c3i src/render/rendering_data.c3 src/render/geometry.c3 src/render/normals.c3 src/render/uvs.c3 test/test_rendering_data.c3 project.json manifest.json
git commit -m "render: finish M3b integration"
```

If Task 6 only verified and made no file changes, do not create an empty commit. Record verification output in the implementer summary.

## Review Requirements During Execution

After each implementation task commit:

1. Run a spec-compliance reviewer subagent with:
   - this plan path,
   - design spec path,
   - task number and task text,
   - base commit before the task,
   - head commit after the task.
2. If the spec reviewer finds issues, dispatch an implementer/fix subagent for the same task, then rerun spec review.
3. Only after spec review passes, run a code-quality reviewer subagent.
4. If the code-quality reviewer finds Critical or Important issues, dispatch a fix subagent and rerun code-quality review.
5. Do not start the next task until both review stages approve.

After all tasks:

1. Run final full verification:

```bash
git diff --check
c3c build static-lib
c3c test
c3c build debug
c3c build release
```

2. Run final whole-branch code review.
3. Run final C3-expert review focusing on C3 0.7.11 syntax, C3 idioms, `.c3l` package config, allocation/fault discipline, and known project pitfalls.
4. Fix any blocking review issues and rerun the relevant verification.

## Branch and Worktree Requirements

Implementation must not happen on `main`.

Use an isolated worktree:

```bash
git worktree add .worktrees/m3b-render-options -b feat/m3b-render-options
cd .worktrees/m3b-render-options
c3c build static-lib && c3c test
```

Expected baseline: build passes and current tests pass before Task 1 edits.

If `.worktrees/` is not ignored, add it to `.gitignore` and commit that cleanup separately before creating the worktree. In this repo `.worktrees/` already exists, but the implementer must still verify ignore status.

## Final Completion Criteria

M3b is complete only when all are true:

- `RenderOptions`, `RenderNormalMode`, `RenderUvMode`, `PLANAR_EPSILON`, and `DEFAULT_RENDER_OPTIONS` exist in `module cg::render`.
- `HalfEdgeMesh.to_rendering_data(&self, Allocator alloc, RenderOptions options = DEFAULT_RENDER_OPTIONS)` is the single public extraction API.
- Default extraction emits positions + indices only.
- Shared-output and flat-output paths match the spec exactly.
- All new faults exist and are returned in the specified cases.
- Planarity validation is enabled by default and skipped only by `skip_planarity_check`.
- Generated smooth normals are angle-weighted over fan triangles.
- Generated flat normals expand vertices/indices/normals and expand UVs when enabled.
- Planar UVs use mesh-level dominant-axis projection and normalize to `[0, 1]`.
- `project.json` and `manifest.json` include every new render source file.
- `git diff --check` passes.
- `c3c build static-lib` passes.
- `c3c test` passes.
- `c3c build debug` passes.
- `c3c build release` passes.
- Per-task spec and code-quality reviews passed.
- Final whole-branch code review passed.
- Final C3-expert review passed.
