# M1 Half-Edge Construction Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.
> **REQUIRED SKILLS:** `c3-expert` before any C3/build-file edit, `test-driven-development` for every implementation task, `verification-before-completion` before claiming done.

**Goal:** Build `cg::half_edge` construction and topology-query APIs for `HalfEdgeMesh`, with explicit allocator ownership and full fault-path tests.

**Architecture:** Keep core data in flat arrays on `HalfEdgeMesh`; `cg::half_edge` adds free constructors and methods on `cg::HalfEdgeMesh`. Construction is a two-pass algorithm: first create face-local half-edges and register directed edges in `HashMap{int[<2>], HeIndex}`, then pair twins by reverse-edge lookup. Boundary edges keep their owning face and use `twin == INVALID_HE`; no ghost half-edges/faces.

**Tech Stack:** C3 0.7.11, c3c, std::collections::map HashMap, explicit `Allocator alloc`, `mem::alloc::new_array(alloc, T, n)`.

**Spec:** `docs/specs/2026-05-04-m1-half-edge-construction-design.md`

---

## File Structure

- Modify: `faults.c3i`
  - Add `ATTRIBUTE_COUNT_MISMATCH` to the `faultdef` block.
- Modify: `project.json`
  - Add `half_edge/builder.c3` and `half_edge/topology.c3` to `sources`.
- Modify: `manifest.json`
  - Add `half_edge/builder.c3` and `half_edge/topology.c3` to library `sources`.
- Create: `half_edge/builder.c3`
  - Owns `HalfEdgeMesh.destroy`, `from_triangles`, `from_triangles_with_attrs`, `from_polygons`, and private builder helpers.
- Create: `half_edge/topology.c3`
  - Owns pure topology query methods: `twin`, `next`, `prev`, `from_vertex`, `to_vertex`, `face_of`, `is_boundary`, `face_degree`.
- Create: `test/test_builder.c3`
  - Constructor, ownership, attribute, polygon, and fault-path tests.
- Create: `test/test_topology.c3`
  - Topology method tests on tetrahedron, single-triangle boundary, and two-triangle diamond fixtures.

Important implementation notes:
- `HashMap{int[<2>], HeIndex}` and `*self = {}` are smoke-verified on c3c 0.7.11.
- `HashMap.get` via `edge_map[key]` returns `HeIndex?`; use `if (try twin_he = edge_map[reverse_key])` or `!!` only in tests after `has_key`.
- `HashMap.set` via `edge_map[key] = value` returns a bool whose meaning is not needed here. Use it as a statement, not inside `assert`.
- Indices arrive as `uint`; mesh index typedefs are inline `int`. Cast deliberately where the compiler requires it.
- Definition order in `builder.c3`: imports/constants/helpers are okay, but `HalfEdgeMesh.destroy` must appear before free constructors.

---

### Task 1: Wire files and first failing builder tests

**Files:**
- Modify: `faults.c3i`
- Modify: `project.json`
- Modify: `manifest.json`
- Create: `test/test_builder.c3`
- Create: `half_edge/builder.c3` (minimal stubs only)

- [ ] **Step 1.1: Add the new fault**

In `faults.c3i`, add `ATTRIBUTE_COUNT_MISMATCH` before `OPEN_CELL_ON_BOUNDARY` or at the end before `;`:

```c3
faultdef
    INDEX_OUT_OF_RANGE,
    INVALID_TRIANGLE_INDEX_COUNT,
    NON_MANIFOLD_INPUT,
    DUPLICATE_HALF_EDGE,
    NON_TRIANGLE_FACE,
    BOUNDARY_HALF_EDGE,
    DEGENERATE_INPUT,
    EMPTY_INPUT,
    ATTRIBUTE_COUNT_MISMATCH,
    OPEN_CELL_ON_BOUNDARY;
```

- [ ] **Step 1.2: Add builder/topology sources to build metadata**

In `project.json`, append to `sources`:

```json
"half_edge/builder.c3",
"half_edge/topology.c3"
```

In `manifest.json`, append to `sources`:

```json
"half_edge/builder.c3",
"half_edge/topology.c3"
```

Keep `smoke/smoke.c3` in `project.json`; do not add it to `manifest.json`.

- [ ] **Step 1.3: Create minimal `half_edge/builder.c3` stubs**

```c3
module cg::half_edge;
import cg;

fn void HalfEdgeMesh.destroy(&self)
{
    free(self.half_edges);
    free(self.faces);
    free(self.vertices);
    free(self.positions);
    free(self.normals);
    free(self.uvs);
    *self = {};
}

fn HalfEdgeMesh? from_triangles(Allocator alloc, Vec3f[] positions, uint[] indices)
{
    return EMPTY_INPUT~;
}

fn HalfEdgeMesh? from_triangles_with_attrs(
    Allocator alloc,
    Vec3f[] positions,
    Vec3f[] normals,
    Vec2f[] uvs,
    uint[] indices)
{
    return EMPTY_INPUT~;
}

fn HalfEdgeMesh? from_polygons(Allocator alloc, Vec3f[] positions, uint[] face_offsets, uint[] face_indices)
{
    return EMPTY_INPUT~;
}
```

- [ ] **Step 1.4: Create failing `test/test_builder.c3` with base fixtures and validation tests**

```c3
module test;
import cg;
import cg::half_edge;

const Vec3f[4] TET_POSITIONS = {
    { 0, 0, 0 },
    { 1, 0, 0 },
    { 0, 1, 0 },
    { 0, 0, 1 },
};

const uint[12] TET_INDICES = {
    0, 1, 2,
    0, 3, 1,
    0, 2, 3,
    1, 3, 2,
};

fn void expect_mesh_fault(HalfEdgeMesh? result, anyfault expected)
{
    if (catch err = result) {
        assert(err == expected);
        return;
    }
    unreachable();
}

fn void test_empty_positions_fault() @test
{
    Vec3f[] positions = {};
    uint[3] indices = { 0, 1, 2 };
    expect_mesh_fault(from_triangles(mem, positions, indices[..]), EMPTY_INPUT);
}

fn void test_empty_indices_fault() @test
{
    uint[] indices = {};
    expect_mesh_fault(from_triangles(mem, TET_POSITIONS[..], indices), EMPTY_INPUT);
}

fn void test_invalid_triangle_index_count_fault() @test
{
    uint[4] indices = { 0, 1, 2, 3 };
    expect_mesh_fault(from_triangles(mem, TET_POSITIONS[..], indices[..]), INVALID_TRIANGLE_INDEX_COUNT);
}

fn void test_index_out_of_range_fault() @test
{
    uint[3] indices = { 0, 1, 99 };
    expect_mesh_fault(from_triangles(mem, TET_POSITIONS[..], indices[..]), INDEX_OUT_OF_RANGE);
}

fn void test_tetrahedron_counts() @test
{
    HalfEdgeMesh mesh = from_triangles(mem, TET_POSITIONS[..], TET_INDICES[..])!;
    defer mesh.destroy();

    assert(mesh.positions.len == 4);
    assert(mesh.vertices.len == 4);
    assert(mesh.faces.len == 4);
    assert(mesh.half_edges.len == 12);
    assert(mesh.normals.len == 0);
    assert(mesh.uvs.len == 0);
}
```

If `anyfault` is not accepted by c3c 0.7.11, inline the `if (catch err = ...)` checks per test instead of using `expect_mesh_fault`.

- [ ] **Step 1.5: Run tests and verify RED**

```bash
c3c test
```

Expected: `test_empty_positions_fault` and `test_empty_indices_fault` pass because the stub returns `EMPTY_INPUT`. `test_invalid_triangle_index_count_fault`, `test_index_out_of_range_fault`, and `test_tetrahedron_counts` fail because the stub returns the wrong fault or no mesh.

- [ ] **Step 1.6: Commit metadata + failing tests + stubs**

```bash
git add faults.c3i project.json manifest.json half_edge/builder.c3 test/test_builder.c3
git commit -m "test: add M1 builder tests and constructor stubs"
```

---

### Task 2: Implement minimal `from_triangles` allocation path

**Files:**
- Modify: `half_edge/builder.c3`
- Modify: `test/test_builder.c3`

- [ ] **Step 2.1: Add constructor validation and array allocation helpers**

Implement validation before allocation:

```c3
if (positions.len == 0 || indices.len == 0) return EMPTY_INPUT~;
if (indices.len % 3 != 0) return INVALID_TRIANGLE_INDEX_COUNT~;
foreach (index : indices) {
    if (index >= positions.len) return INDEX_OUT_OF_RANGE~;
}
```

Allocate and initialize:

```c3
usz face_count = indices.len / 3;
usz half_edge_count = indices.len;

HalfEdgeMesh mesh;
mesh.half_edges = mem::alloc::new_array(alloc, HalfEdge, half_edge_count);
defer catch free(mesh.half_edges);
mesh.faces = mem::alloc::new_array(alloc, HalfEdgeFace, face_count);
defer catch free(mesh.faces);
mesh.vertices = mem::alloc::new_array(alloc, HalfEdgeVertex, positions.len);
defer catch free(mesh.vertices);
mesh.positions = mem::alloc::new_array(alloc, Vec3f, positions.len);
defer catch free(mesh.positions);

for (usz i = 0; i < mesh.vertices.len; i++) {
    mesh.vertices[i].half_edge = INVALID_HE;
    mesh.positions[i] = positions[i];
}
```

Then build face-local half-edges without twins yet:

```c3
for (usz face = 0; face < face_count; face++) {
    usz base = face * 3;
    mesh.faces[face].half_edge = (HeIndex)(int)base;

    for (usz local = 0; local < 3; local++) {
        usz he_index = base + local;
        usz next_index = base + ((local + 1) % 3);
        uint origin_index = indices[he_index];

        mesh.half_edges[he_index].origin = (VertexIndex)(int)origin_index;
        mesh.half_edges[he_index].next = (HeIndex)(int)next_index;
        mesh.half_edges[he_index].twin = INVALID_HE;
        mesh.half_edges[he_index].face = (FaceIndex)(int)face;

        if (mesh.vertices[origin_index].half_edge == INVALID_HE) {
            mesh.vertices[origin_index].half_edge = (HeIndex)(int)he_index;
        }
    }
}
```

- [ ] **Step 2.2: Run focused test**

```bash
c3c test test/test_builder.c3
```

Expected: `test_tetrahedron_counts` passes; duplicate-edge and twin tests not added yet.

- [ ] **Step 2.3: Add position-copy assertions**

Add to `test_tetrahedron_counts`:

```c3
assert(mesh.positions[0] == TET_POSITIONS[0]);
assert(mesh.positions[1] == TET_POSITIONS[1]);
assert(mesh.positions[2] == TET_POSITIONS[2]);
assert(mesh.positions[3] == TET_POSITIONS[3]);
```

- [ ] **Step 2.4: Re-run focused test**

```bash
c3c test test/test_builder.c3
```

Expected: all current builder tests pass.

- [ ] **Step 2.5: Commit**

```bash
git add half_edge/builder.c3 test/test_builder.c3
git commit -m "half_edge: allocate triangle mesh topology arrays (M1)"
```

---

### Task 3: Add HashMap duplicate detection and twin pairing

**Files:**
- Modify: `half_edge/builder.c3`
- Modify: `test/test_builder.c3`

- [ ] **Step 3.1: Add failing duplicate-edge and twin tests**

```c3
fn void test_duplicate_directed_edge_fault() @test
{
    uint[6] indices = {
        0, 1, 2,
        0, 1, 3,
    };
    expect_mesh_fault(from_triangles(mem, TET_POSITIONS[..], indices[..]), DUPLICATE_HALF_EDGE);
}

fn void test_tetrahedron_twins_are_paired() @test
{
    HalfEdgeMesh mesh = from_triangles(mem, TET_POSITIONS[..], TET_INDICES[..])!;
    defer mesh.destroy();

    for (usz i = 0; i < mesh.half_edges.len; i++) {
        HeIndex twin = mesh.half_edges[i].twin;
        assert(twin != INVALID_HE);
        assert(mesh.half_edges[twin].twin == (HeIndex)(int)i);
    }
}
```

- [ ] **Step 3.2: Run focused test and verify RED**

```bash
c3c test test/test_builder.c3
```

Expected: duplicate test fails because duplicate detection is missing; twin test fails because twins remain `INVALID_HE`.

- [ ] **Step 3.3: Implement edge map registration during first pass**

In `from_triangles`, import map at top:

```c3
import std::collections::map;
```

Initialize after arrays:

```c3
HashMap{int[<2>], HeIndex} edge_map;
edge_map.init(alloc);
defer edge_map.free();
```

For each directed edge:

```c3
uint from_index = indices[he_index];
uint to_index = indices[next_index];
int[<2>] key = { (int)from_index, (int)to_index };
if (edge_map.has_key(key)) return DUPLICATE_HALF_EDGE~;
edge_map[key] = (HeIndex)(int)he_index;
```

- [ ] **Step 3.4: Implement second-pass twin pairing**

After first pass:

```c3
for (usz he_index = 0; he_index < mesh.half_edges.len; he_index++) {
    VertexIndex from_vertex = mesh.half_edges[he_index].origin;
    HeIndex next_he = mesh.half_edges[he_index].next;
    VertexIndex to_vertex = mesh.half_edges[next_he].origin;
    int[<2>] reverse_key = { (int)to_vertex, (int)from_vertex };

    if (try twin_he = edge_map[reverse_key]) {
        mesh.half_edges[he_index].twin = twin_he;
    }
}
```

Do not fault on missing reverse key; that is a boundary edge.

- [ ] **Step 3.5: Run focused test**

```bash
c3c test test/test_builder.c3
```

Expected: all builder tests added so far pass.

- [ ] **Step 3.6: Commit**

```bash
git add half_edge/builder.c3 test/test_builder.c3
git commit -m "half_edge: pair triangle half-edge twins with hash map (M1)"
```

---

### Task 4: Implement attributes and destroy behavior

**Files:**
- Modify: `half_edge/builder.c3`
- Modify: `test/test_builder.c3`

- [ ] **Step 4.1: Add failing attribute tests**

```c3
const Vec3f[4] TET_NORMALS = {
    { 0, 0, 1 },
    { 0, 1, 0 },
    { 1, 0, 0 },
    { 1, 1, 1 },
};

const Vec2f[4] TET_UVS = {
    { 0, 0 },
    { 1, 0 },
    { 0, 1 },
    { 1, 1 },
};

fn void test_tetrahedron_with_attrs_copies_normals_and_uvs() @test
{
    HalfEdgeMesh mesh = from_triangles_with_attrs(
        mem,
        TET_POSITIONS[..],
        TET_NORMALS[..],
        TET_UVS[..],
        TET_INDICES[..])!;
    defer mesh.destroy();

    assert(mesh.normals.len == 4);
    assert(mesh.uvs.len == 4);
    assert(mesh.normals[2] == TET_NORMALS[2]);
    assert(mesh.uvs[3] == TET_UVS[3]);
}

fn void test_empty_attrs_stay_empty() @test
{
    Vec3f[] normals = {};
    Vec2f[] uvs = {};
    HalfEdgeMesh mesh = from_triangles_with_attrs(mem, TET_POSITIONS[..], normals, uvs, TET_INDICES[..])!;
    defer mesh.destroy();

    assert(mesh.normals.len == 0);
    assert(mesh.uvs.len == 0);
}

fn void test_attr_count_mismatch_fault() @test
{
    Vec3f[1] normals = { { 0, 0, 1 } };
    Vec2f[] uvs = {};
    expect_mesh_fault(from_triangles_with_attrs(mem, TET_POSITIONS[..], normals[..], uvs, TET_INDICES[..]), ATTRIBUTE_COUNT_MISMATCH);
}

fn void test_uv_count_mismatch_fault() @test
{
    Vec3f[] normals = {};
    Vec2f[1] uvs = { { 0, 0 } };
    expect_mesh_fault(from_triangles_with_attrs(mem, TET_POSITIONS[..], normals, uvs[..], TET_INDICES[..]), ATTRIBUTE_COUNT_MISMATCH);
}

fn void test_destroy_zeroes_mesh() @test
{
    HalfEdgeMesh mesh = from_triangles(mem, TET_POSITIONS[..], TET_INDICES[..])!;
    mesh.destroy();

    assert(mesh.half_edges.len == 0);
    assert(mesh.faces.len == 0);
    assert(mesh.vertices.len == 0);
    assert(mesh.positions.len == 0);
}
```

- [ ] **Step 4.2: Run focused test and verify RED**

```bash
c3c test test/test_builder.c3
```

Expected: attr tests fail until `from_triangles_with_attrs` copies attributes.

- [ ] **Step 4.3: Refactor shared triangle builder**

Make `from_triangles` call `from_triangles_with_attrs`:

```c3
fn HalfEdgeMesh? from_triangles(Allocator alloc, Vec3f[] positions, uint[] indices)
{
    Vec3f[] normals = {};
    Vec2f[] uvs = {};
    return from_triangles_with_attrs(alloc, positions, normals, uvs, indices)!;
}
```

Move the full triangle build into `from_triangles_with_attrs`, with pre-validation:

```c3
if (normals.len > 0 && normals.len != positions.len) return ATTRIBUTE_COUNT_MISMATCH~;
if (uvs.len > 0 && uvs.len != positions.len) return ATTRIBUTE_COUNT_MISMATCH~;
```

Copy optional attrs only when provided:

```c3
if (normals.len > 0) {
    mesh.normals = mem::alloc::new_array(alloc, Vec3f, normals.len);
    defer catch free(mesh.normals);
    for (usz i = 0; i < normals.len; i++) mesh.normals[i] = normals[i];
}

if (uvs.len > 0) {
    mesh.uvs = mem::alloc::new_array(alloc, Vec2f, uvs.len);
    defer catch free(mesh.uvs);
    for (usz i = 0; i < uvs.len; i++) mesh.uvs[i] = uvs[i];
}
```

- [ ] **Step 4.4: Run focused test**

```bash
c3c test test/test_builder.c3
```

Expected: all builder tests added so far pass.

- [ ] **Step 4.5: Commit**

```bash
git add half_edge/builder.c3 test/test_builder.c3
git commit -m "half_edge: add attribute-copying triangle constructor (M1)"
```

---

### Task 5: Implement `from_polygons`

**Files:**
- Modify: `half_edge/builder.c3`
- Modify: `test/test_builder.c3`

- [ ] **Step 5.1: Add failing polygon tests**

```c3
fn void test_polygon_quad() @test
{
    Vec3f[4] positions = {
        { 0, 0, 0 },
        { 1, 0, 0 },
        { 1, 1, 0 },
        { 0, 1, 0 },
    };
    uint[2] offsets = { 0, 4 };
    uint[4] indices = { 0, 1, 2, 3 };

    HalfEdgeMesh mesh = from_polygons(mem, positions[..], offsets[..], indices[..])!;
    defer mesh.destroy();

    assert(mesh.faces.len == 1);
    assert(mesh.half_edges.len == 4);
    assert(mesh.half_edges[0].next == 1);
    assert(mesh.half_edges[1].next == 2);
    assert(mesh.half_edges[2].next == 3);
    assert(mesh.half_edges[3].next == 0);
}

fn void test_polygon_empty_offsets_fault() @test
{
    uint[] offsets = {};
    uint[] indices = {};
    expect_mesh_fault(from_polygons(mem, TET_POSITIONS[..], offsets, indices), EMPTY_INPUT);
}

fn void test_polygon_bad_first_offset_fault() @test
{
    uint[2] offsets = { 1, 4 };
    uint[4] indices = { 0, 1, 2, 3 };
    expect_mesh_fault(from_polygons(mem, TET_POSITIONS[..], offsets[..], indices[..]), INDEX_OUT_OF_RANGE);
}

fn void test_polygon_degenerate_face_fault() @test
{
    uint[2] offsets = { 0, 2 };
    uint[2] indices = { 0, 1 };
    expect_mesh_fault(from_polygons(mem, TET_POSITIONS[..], offsets[..], indices[..]), DEGENERATE_INPUT);
}
```

- [ ] **Step 5.2: Run focused test and verify RED**

```bash
c3c test test/test_builder.c3
```

Expected: polygon quad fails because `from_polygons` is still stubbed.

- [ ] **Step 5.3: Implement polygon validation**

Validation rules:

```c3
if (positions.len == 0 || face_offsets.len < 2 || face_indices.len == 0) return EMPTY_INPUT~;
if (face_offsets[0] != 0) return INDEX_OUT_OF_RANGE~;
if (face_offsets[face_offsets.len - 1] != face_indices.len) return INDEX_OUT_OF_RANGE~;

for (usz f = 0; f + 1 < face_offsets.len; f++) {
    uint start = face_offsets[f];
    uint end = face_offsets[f + 1];
    if (end < start) return INDEX_OUT_OF_RANGE~;
    if (end - start < 3) return DEGENERATE_INPUT~;
}

foreach (index : face_indices) {
    if (index >= positions.len) return INDEX_OUT_OF_RANGE~;
}
```

- [ ] **Step 5.4: Implement generalized face builder helper**

Extract shared helper logic if useful:
- allocate mesh arrays with `face_count` and `half_edge_count`
- copy positions
- init vertices to `INVALID_HE`
- init `HashMap{int[<2>], HeIndex}`
- for each face, iterate local ring length and set `next` wrapping inside that ring
- detect duplicate directed edges with `edge_map.has_key(key)`
- pair twins in the same second pass used by triangles

For polygons:

```c3
usz face_count = face_offsets.len - 1;
usz half_edge_count = face_indices.len;

for (usz face = 0; face < face_count; face++) {
    usz start = face_offsets[face];
    usz end = face_offsets[face + 1];
    usz degree = end - start;
    mesh.faces[face].half_edge = (HeIndex)(int)start;

    for (usz local = 0; local < degree; local++) {
        usz he_index = start + local;
        usz next_index = start + ((local + 1) % degree);
        uint from_index = face_indices[he_index];
        uint to_index = face_indices[next_index];
        // fill mesh.half_edges[he_index] and register edge_map[{from,to}]
    }
}
```

- [ ] **Step 5.5: Run focused test**

```bash
c3c test test/test_builder.c3
```

Expected: all builder tests pass.

- [ ] **Step 5.6: Commit**

```bash
git add half_edge/builder.c3 test/test_builder.c3
git commit -m "half_edge: construct polygonal half-edge meshes (M1)"
```

---

### Task 6: Add topology query tests and stubs

**Files:**
- Create: `half_edge/topology.c3`
- Create: `test/test_topology.c3`

- [ ] **Step 6.1: Create topology stubs**

```c3
module cg::half_edge;
import cg;

fn HeIndex HalfEdgeMesh.twin(&self, HeIndex he) { return INVALID_HE; }
fn HeIndex HalfEdgeMesh.next(&self, HeIndex he) { return INVALID_HE; }
fn HeIndex HalfEdgeMesh.prev(&self, HeIndex he) { return INVALID_HE; }
fn VertexIndex HalfEdgeMesh.from_vertex(&self, HeIndex he) { return INVALID_VERTEX; }
fn VertexIndex HalfEdgeMesh.to_vertex(&self, HeIndex he) { return INVALID_VERTEX; }
fn FaceIndex HalfEdgeMesh.face_of(&self, HeIndex he) { return INVALID_FACE; }
fn bool HalfEdgeMesh.is_boundary(&self, HeIndex he) { return true; }
fn int HalfEdgeMesh.face_degree(&self, FaceIndex f) { return 0; }
```

- [ ] **Step 6.2: Create failing topology tests**

```c3
module test;
import cg;
import cg::half_edge;

const Vec3f[4] TOPO_TET_POSITIONS = {
    { 0, 0, 0 },
    { 1, 0, 0 },
    { 0, 1, 0 },
    { 0, 0, 1 },
};

const uint[12] TOPO_TET_INDICES = {
    0, 1, 2,
    0, 3, 1,
    0, 2, 3,
    1, 3, 2,
};

fn void test_twin_roundtrip() @test
{
    HalfEdgeMesh mesh = from_triangles(mem, TOPO_TET_POSITIONS[..], TOPO_TET_INDICES[..])!;
    defer mesh.destroy();

    for (usz i = 0; i < mesh.half_edges.len; i++) {
        HeIndex twin = mesh.twin((HeIndex)(int)i);
        assert(twin != INVALID_HE);
        assert(mesh.twin(twin) == (HeIndex)(int)i);
    }
}

fn void test_next_face_cycle() @test
{
    HalfEdgeMesh mesh = from_triangles(mem, TOPO_TET_POSITIONS[..], TOPO_TET_INDICES[..])!;
    defer mesh.destroy();

    assert(mesh.next(0) == 1);
    assert(mesh.next(1) == 2);
    assert(mesh.next(2) == 0);
}

fn void test_prev_roundtrip() @test
{
    HalfEdgeMesh mesh = from_triangles(mem, TOPO_TET_POSITIONS[..], TOPO_TET_INDICES[..])!;
    defer mesh.destroy();

    for (usz i = 0; i < mesh.half_edges.len; i++) {
        HeIndex he = (HeIndex)(int)i;
        assert(mesh.prev(mesh.next(he)) == he);
    }
}

fn void test_from_to_face_boundary_degree() @test
{
    HalfEdgeMesh mesh = from_triangles(mem, TOPO_TET_POSITIONS[..], TOPO_TET_INDICES[..])!;
    defer mesh.destroy();

    assert(mesh.from_vertex(0) == 0);
    assert(mesh.to_vertex(0) == 1);
    assert(mesh.face_of(0) == 0);
    assert(!mesh.is_boundary(0));
    assert(mesh.face_degree(0) == 3);
}

fn void test_single_triangle_boundary_edges() @test
{
    uint[3] indices = { 0, 1, 2 };
    HalfEdgeMesh mesh = from_triangles(mem, TOPO_TET_POSITIONS[..], indices[..])!;
    defer mesh.destroy();

    assert(mesh.is_boundary(0));
    assert(mesh.is_boundary(1));
    assert(mesh.is_boundary(2));
    assert(mesh.prev(0) == 2);
    assert(mesh.face_degree(0) == 3);
}
```

- [ ] **Step 6.3: Run focused test and verify RED**

```bash
c3c test test/test_topology.c3
```

Expected: topology tests fail because methods are stubs.

- [ ] **Step 6.4: Commit stubs + failing topology tests**

```bash
git add half_edge/topology.c3 test/test_topology.c3
git commit -m "test: add M1 topology query tests and stubs"
```

---

### Task 7: Implement topology query methods

**Files:**
- Modify: `half_edge/topology.c3`
- Modify: `test/test_topology.c3`

- [ ] **Step 7.1: Replace stubs with direct lookups**

```c3
fn HeIndex HalfEdgeMesh.twin(&self, HeIndex he)
{
    return self.half_edges[he].twin;
}

fn HeIndex HalfEdgeMesh.next(&self, HeIndex he)
{
    return self.half_edges[he].next;
}

fn VertexIndex HalfEdgeMesh.from_vertex(&self, HeIndex he)
{
    return self.half_edges[he].origin;
}

fn VertexIndex HalfEdgeMesh.to_vertex(&self, HeIndex he)
{
    return self.half_edges[self.half_edges[he].next].origin;
}

fn FaceIndex HalfEdgeMesh.face_of(&self, HeIndex he)
{
    return self.half_edges[he].face;
}

fn bool HalfEdgeMesh.is_boundary(&self, HeIndex he)
{
    return self.half_edges[he].twin == INVALID_HE;
}
```

- [ ] **Step 7.2: Implement `prev` and `face_degree`**

```c3
fn HeIndex HalfEdgeMesh.prev(&self, HeIndex he)
{
    FaceIndex face = self.half_edges[he].face;
    HeIndex cursor = self.faces[face].half_edge;

    while (true) {
        HeIndex next_he = self.half_edges[cursor].next;
        if (next_he == he) return cursor;
        cursor = next_he;
    }

    return INVALID_HE;
}

fn int HalfEdgeMesh.face_degree(&self, FaceIndex f)
{
    HeIndex start = self.faces[f].half_edge;
    HeIndex cursor = start;
    int count = 0;

    do {
        count += 1;
        cursor = self.half_edges[cursor].next;
    } while (cursor != start);

    return count;
}
```

If C3 rejects `do`/`while` syntax, rewrite as a `while (true)` loop with a break after advancing.

- [ ] **Step 7.3: Add diamond topology test**

```c3
fn void test_diamond_topology() @test
{
    Vec3f[4] positions = {
        { 0, 0, 0 },
        { 1, 0, 0 },
        { 0, 1, 0 },
        { 1, 1, 0 },
    };
    uint[6] indices = {
        0, 1, 2,
        2, 1, 3,
    };

    HalfEdgeMesh mesh = from_triangles(mem, positions[..], indices[..])!;
    defer mesh.destroy();

    assert(mesh.faces.len == 2);
    assert(mesh.half_edges.len == 6);
    assert(mesh.twin(1) == 3);
    assert(mesh.twin(3) == 1);
    assert(mesh.is_boundary(0));
    assert(mesh.is_boundary(2));
    assert(mesh.is_boundary(4));
    assert(mesh.is_boundary(5));
}
```

- [ ] **Step 7.4: Run topology tests**

```bash
c3c test test/test_topology.c3
```

Expected: all topology tests pass.

- [ ] **Step 7.5: Run builder tests**

```bash
c3c test test/test_builder.c3
```

Expected: all builder tests still pass.

- [ ] **Step 7.6: Commit**

```bash
git add half_edge/topology.c3 test/test_topology.c3
git commit -m "half_edge: add topology query methods (M1)"
```

---

### Task 8: Full verification and cleanup

**Files:**
- Review: `half_edge/builder.c3`
- Review: `half_edge/topology.c3`
- Review: `test/test_builder.c3`
- Review: `test/test_topology.c3`
- Review: `project.json`, `manifest.json`, `faults.c3i`

- [ ] **Step 8.1: Check formatting and style manually**

Verify:
- no `assert()` in production files
- no `alloc::new_array`; only `mem::alloc::new_array(alloc, T, n)`
- no `->`, `sizeof`, raw `malloc`, `goto`, or null-as-error
- no comments explaining obvious C3 syntax
- all functions/methods are `snake_case`
- `HalfEdgeMesh.destroy` appears before free constructors in `builder.c3`

- [ ] **Step 8.2: Run full verification**

```bash
c3c build static-lib && c3c test
```

Expected:
- `Static library 'out/static-lib.a' created.`
- all existing M0 tests pass
- all M1 builder/topology tests pass

- [ ] **Step 8.3: Build debug and release targets**

```bash
c3c build debug && c3c build release
```

Expected: both targets produce static libraries successfully.

- [ ] **Step 8.4: Inspect git diff**

```bash
git diff --check
git status --short
```

Expected: no whitespace errors; only intentional M1 files changed.

- [ ] **Step 8.5: Final commit if cleanup changes were made**

```bash
git add faults.c3i project.json manifest.json half_edge/builder.c3 half_edge/topology.c3 test/test_builder.c3 test/test_topology.c3
git commit -m "half_edge: finish M1 construction and topology queries"
```

Skip this commit if the working tree is clean because earlier task commits already captured all changes.

---

## Execution Handoff Notes

Recommended execution mode: subagent-driven development.

Reason: tasks are independent enough for fresh context per phase, but each phase has concrete tests and verification commands. Use one fresh worker per task, then review the produced diff before moving on.
