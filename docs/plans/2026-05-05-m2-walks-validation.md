# M2 Walks and Validation Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.
> **REQUIRED SKILLS:** `c3-expert` before any C3/build-file edit, `test-driven-development` for every implementation task, `verification-before-completion` before claiming done.

**Goal:** Add non-allocating half-edge walk helpers and topology validation for `HalfEdgeMesh`.

**Architecture:** Keep `HalfEdgeMesh` as the only topology storage. Add focused `cg::half_edge` method files: `walks.c3` for face/vertex enumeration and `validate.c3` for invariant checks. Walk helpers take caller-owned output buffers and return counts; validation returns the first granular fault without allocation or mutation.

**Tech Stack:** C3 0.7.11, c3c, flat-array half-edge mesh, optionals/faults, caller-provided slices.

**Spec:** `docs/specs/2026-05-05-m2-walks-validation-design.md`

---

## File Structure

- Modify: `src/faults.c3i`
  - Add `OUTPUT_BUFFER_TOO_SMALL`, `INVALID_HALF_EDGE_REFERENCE`, `INVALID_VERTEX_REFERENCE`, `INVALID_FACE_REFERENCE`, `INVALID_TWIN`, `INVALID_FACE_CYCLE`, `INVALID_TOPOLOGY`.
- Modify: `project.json`
  - Add `src/half_edge/walks.c3` and `src/half_edge/validate.c3` to `sources` after `src/half_edge/topology.c3`.
- Modify: `manifest.json`
  - Add the same two library source files.
- Create: `src/half_edge/walks.c3`
  - Owns `face_half_edges`, `face_vertices`, `vertex_one_ring_outgoing`, `vertex_one_ring_faces`.
- Create: `src/half_edge/validate.c3`
  - Owns `HalfEdgeMesh.validate(&self)` plus private validation helpers.
- Create: `test/test_walks.c3`
  - Owns M2 walk and validation tests. It may reuse `TET_POSITIONS`, `TET_INDICES`, `TET_NORMALS`, `TET_UVS`, `TRI_POSITIONS`, `TRI_INDICES`, `DIAMOND_POSITIONS`, and `DIAMOND_INDICES` from existing `module test` files.

Important implementation notes:
- Current code lives under `src/`; do not recreate top-level library source directories.
- All new production files declare `module cg::half_edge;` and `import cg;`.
- Functions from `cg::half_edge` must be called with `half_edge::` prefix from tests.
- Walk helpers assume valid topology except for buffer size and isolated vertices.
- Validation must not call walk helpers until basic bounds are proven, because doctored meshes may contain invalid indices.
- Production code must not use runtime `assert` or `unreachable`.
- Tests may use `assert`, `unreachable`, and `!!`.
- Commit only green states. For each task: write failing test, run and observe failure, implement, run and observe pass, then commit.

---

### Task 1: Wire M2 files, faults, and face-walk tests

**Files:**
- Modify: `src/faults.c3i`
- Modify: `project.json`
- Modify: `manifest.json`
- Create: `src/half_edge/walks.c3`
- Create: `src/half_edge/validate.c3`
- Create: `test/test_walks.c3`

- [ ] **Step 1.1: Add M2 faults**

In `src/faults.c3i`, add the M2 faults before `OPEN_CELL_ON_BOUNDARY`:

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
    OUTPUT_BUFFER_TOO_SMALL,
    INVALID_HALF_EDGE_REFERENCE,
    INVALID_VERTEX_REFERENCE,
    INVALID_FACE_REFERENCE,
    INVALID_TWIN,
    INVALID_FACE_CYCLE,
    INVALID_TOPOLOGY,
    OPEN_CELL_ON_BOUNDARY;
```

- [ ] **Step 1.2: Add M2 files to build metadata**

In `project.json`, append these after `src/half_edge/topology.c3`:

```json
"src/half_edge/walks.c3",
"src/half_edge/validate.c3"
```

In `manifest.json`, append the same two paths to `sources`.

Keep `smoke/smoke.c3` in `project.json`; do not add smoke to `manifest.json`.

- [ ] **Step 1.3: Create minimal stubs**

Create `src/half_edge/walks.c3`:

```c3
module cg::half_edge;
import cg;

fn int? HalfEdgeMesh.face_half_edges(&self, FaceIndex f, HeIndex[] out)
{
    return cg::OUTPUT_BUFFER_TOO_SMALL~;
}

fn int? HalfEdgeMesh.face_vertices(&self, FaceIndex f, VertexIndex[] out)
{
    return cg::OUTPUT_BUFFER_TOO_SMALL~;
}

fn int? HalfEdgeMesh.vertex_one_ring_outgoing(&self, VertexIndex v, HeIndex[] out)
{
    return cg::OUTPUT_BUFFER_TOO_SMALL~;
}

fn int? HalfEdgeMesh.vertex_one_ring_faces(&self, VertexIndex v, FaceIndex[] out)
{
    return cg::OUTPUT_BUFFER_TOO_SMALL~;
}
```

Create `src/half_edge/validate.c3`:

```c3
module cg::half_edge;
import cg;

fn void? HalfEdgeMesh.validate(&self)
{
    return cg::INVALID_TOPOLOGY~;
}
```

- [ ] **Step 1.4: Add face-walk tests**

Create `test/test_walks.c3`:

```c3
module test;
import cg;
import cg::half_edge;

fn void expect_int_fault(int? result, anyfault expected)
{
    if (catch err = result) {
        assert(err == expected);
        return;
    }
    unreachable();
}

fn void expect_void_fault(void? result, anyfault expected)
{
    if (catch err = result) {
        assert(err == expected);
        return;
    }
    unreachable();
}

fn void test_face_half_edges_triangle_cycle() @test
{
    HalfEdgeMesh mesh = half_edge::from_triangles(mem, TET_POSITIONS[..], TET_INDICES[..])!!;
    defer mesh.destroy();

    HeIndex[3] out;
    int count = mesh.face_half_edges(0, out[..])!!;

    assert(count == 3);
    assert(out[0] == mesh.faces[0].half_edge);
    assert(out[1] == mesh.next(out[0]));
    assert(out[2] == mesh.next(out[1]));
    assert(mesh.next(out[2]) == out[0]);
}

fn void test_face_vertices_triangle_cycle() @test
{
    HalfEdgeMesh mesh = half_edge::from_triangles(mem, TET_POSITIONS[..], TET_INDICES[..])!!;
    defer mesh.destroy();

    VertexIndex[3] out;
    int count = mesh.face_vertices(0, out[..])!!;

    assert(count == 3);
    assert(out[0] == mesh.from_vertex(mesh.faces[0].half_edge));
    assert(out[1] == mesh.to_vertex(mesh.faces[0].half_edge));
    assert(out[2] == mesh.to_vertex(mesh.next(mesh.faces[0].half_edge)));
}

fn void test_face_vertices_quad_cycle() @test
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

    VertexIndex[4] out;
    int count = mesh.face_vertices(0, out[..])!!;

    assert(count == 4);
    assert(out[0] == 0);
    assert(out[1] == 1);
    assert(out[2] == 2);
    assert(out[3] == 3);
}

fn void test_face_walk_output_buffer_too_small() @test
{
    HalfEdgeMesh mesh = half_edge::from_triangles(mem, TET_POSITIONS[..], TET_INDICES[..])!!;
    defer mesh.destroy();

    HeIndex[2] half_edges;
    VertexIndex[2] vertices;

    expect_int_fault(mesh.face_half_edges(0, half_edges[..]), cg::OUTPUT_BUFFER_TOO_SMALL);
    expect_int_fault(mesh.face_vertices(0, vertices[..]), cg::OUTPUT_BUFFER_TOO_SMALL);
}
```

If `anyfault` is not accepted by c3c 0.7.11 in helper signatures, inline the `if (catch err = ...)` checks in each test.

- [ ] **Step 1.5: Run tests and verify RED**

```bash
c3c test
```

Expected: the new face-walk tests fail because stubs return `OUTPUT_BUFFER_TOO_SMALL`.

- [ ] **Step 1.6: Implement face-walk helpers**

Replace `face_half_edges` and `face_vertices` in `src/half_edge/walks.c3`:

```c3
fn int? HalfEdgeMesh.face_half_edges(&self, FaceIndex f, HeIndex[] out)
{
    HeIndex start = self.faces[f].half_edge;
    HeIndex cursor = start;
    usz count = 0;

    while (true) {
        if (count >= out.len) return cg::OUTPUT_BUFFER_TOO_SMALL~;
        out[count] = cursor;
        count++;

        cursor = self.half_edges[cursor].next;
        if (cursor == start) break;
    }

    return (int)count;
}

fn int? HalfEdgeMesh.face_vertices(&self, FaceIndex f, VertexIndex[] out)
{
    HeIndex start = self.faces[f].half_edge;
    HeIndex cursor = start;
    usz count = 0;

    while (true) {
        if (count >= out.len) return cg::OUTPUT_BUFFER_TOO_SMALL~;
        out[count] = self.half_edges[cursor].origin;
        count++;

        cursor = self.half_edges[cursor].next;
        if (cursor == start) break;
    }

    return (int)count;
}
```

- [ ] **Step 1.7: Run tests and verify GREEN**

```bash
c3c build static-lib && c3c test
```

Expected: all existing tests and new face-walk tests pass.

- [ ] **Step 1.8: Commit face walks**

```bash
git add src/faults.c3i project.json manifest.json src/half_edge/walks.c3 src/half_edge/validate.c3 test/test_walks.c3
git commit -m "half_edge: add face walk helpers (M2)"
```

---

### Task 2: Implement vertex one-ring walks

**Files:**
- Modify: `src/half_edge/walks.c3`
- Modify: `test/test_walks.c3`

- [ ] **Step 2.1: Add closed one-ring tests**

Append to `test/test_walks.c3`:

```c3
fn void test_tetrahedron_vertex_one_ring_outgoing_degree_three() @test
{
    HalfEdgeMesh mesh = half_edge::from_triangles(mem, TET_POSITIONS[..], TET_INDICES[..])!!;
    defer mesh.destroy();

    for (usz v = 0; v < mesh.vertices.len; v++) {
        HeIndex[3] out;
        int count = mesh.vertex_one_ring_outgoing((VertexIndex)(int)v, out[..])!!;
        assert(count == 3);
        for (int i = 0; i < count; i++) {
            assert(out[i] != cg::INVALID_HE);
            assert(mesh.from_vertex(out[i]) == (VertexIndex)(int)v);
        }
    }
}

fn void test_tetrahedron_vertex_one_ring_faces_degree_three() @test
{
    HalfEdgeMesh mesh = half_edge::from_triangles(mem, TET_POSITIONS[..], TET_INDICES[..])!!;
    defer mesh.destroy();

    for (usz v = 0; v < mesh.vertices.len; v++) {
        FaceIndex[3] out;
        int count = mesh.vertex_one_ring_faces((VertexIndex)(int)v, out[..])!!;
        assert(count == 3);
        for (int i = 0; i < count; i++) {
            assert(out[i] != cg::INVALID_FACE);
        }
    }
}
```

- [ ] **Step 2.2: Add degree-5 fixture test**

Use a closed bipyramid-like mesh with five triangles around vertex `0` and five triangles around opposite vertex `6`:

```c3
const Vec3f[7] DEGREE_FIVE_POSITIONS = {
    { 0, 0, 1 },
    { 1, 0, 0 },
    { 0.309016, 0.951057, 0 },
    { -0.809017, 0.587785, 0 },
    { -0.809017, -0.587785, 0 },
    { 0.309016, -0.951057, 0 },
    { 0, 0, -1 },
};

const uint[30] DEGREE_FIVE_INDICES = {
    0, 1, 2,
    0, 2, 3,
    0, 3, 4,
    0, 4, 5,
    0, 5, 1,
    6, 2, 1,
    6, 3, 2,
    6, 4, 3,
    6, 5, 4,
    6, 1, 5,
};

fn void test_degree_five_vertex_one_ring() @test
{
    HalfEdgeMesh mesh = half_edge::from_triangles(mem, DEGREE_FIVE_POSITIONS[..], DEGREE_FIVE_INDICES[..])!!;
    defer mesh.destroy();

    HeIndex[5] outgoing;
    FaceIndex[5] faces;

    int outgoing_count = mesh.vertex_one_ring_outgoing(0, outgoing[..])!!;
    int face_count = mesh.vertex_one_ring_faces(0, faces[..])!!;

    assert(outgoing_count == 5);
    assert(face_count == 5);
    for (int i = 0; i < outgoing_count; i++) {
        assert(mesh.from_vertex(outgoing[i]) == 0);
    }
}
```

- [ ] **Step 2.3: Add boundary, isolated, deterministic, and buffer tests**

Append:

```c3
fn void test_boundary_vertex_walk_stops_cleanly() @test
{
    HalfEdgeMesh mesh = half_edge::from_triangles(mem, TRI_POSITIONS[..], TRI_INDICES[..])!!;
    defer mesh.destroy();

    HeIndex[3] out;
    int count = mesh.vertex_one_ring_outgoing(0, out[..])!!;

    assert(count >= 1);
    assert(count <= 3);
    assert(out[0] == mesh.vertices[0].half_edge);
    for (int i = 0; i < count; i++) {
        assert(mesh.from_vertex(out[i]) == 0);
    }
}

fn void test_isolated_vertex_one_ring_zero() @test
{
    HalfEdgeMesh mesh = half_edge::from_triangles(mem, TRI_POSITIONS[..], TRI_INDICES[..])!!;
    defer mesh.destroy();

    mesh.vertices[0].half_edge = cg::INVALID_HE;

    HeIndex[1] outgoing;
    FaceIndex[1] faces;
    assert(mesh.vertex_one_ring_outgoing(0, outgoing[..])!! == 0);
    assert(mesh.vertex_one_ring_faces(0, faces[..])!! == 0);
}

fn void test_vertex_one_ring_deterministic() @test
{
    HalfEdgeMesh mesh = half_edge::from_triangles(mem, TET_POSITIONS[..], TET_INDICES[..])!!;
    defer mesh.destroy();

    HeIndex[3] first;
    HeIndex[3] second;
    int first_count = mesh.vertex_one_ring_outgoing(0, first[..])!!;
    int second_count = mesh.vertex_one_ring_outgoing(0, second[..])!!;

    assert(first_count == second_count);
    for (int i = 0; i < first_count; i++) {
        assert(first[i] == second[i]);
    }
}

fn void test_vertex_walk_output_buffer_too_small() @test
{
    HalfEdgeMesh mesh = half_edge::from_triangles(mem, TET_POSITIONS[..], TET_INDICES[..])!!;
    defer mesh.destroy();

    HeIndex[2] outgoing;
    FaceIndex[2] faces;

    expect_int_fault(mesh.vertex_one_ring_outgoing(0, outgoing[..]), cg::OUTPUT_BUFFER_TOO_SMALL);
    expect_int_fault(mesh.vertex_one_ring_faces(0, faces[..]), cg::OUTPUT_BUFFER_TOO_SMALL);
}
```

- [ ] **Step 2.4: Run tests and verify RED**

```bash
c3c test
```

Expected: vertex one-ring tests fail because stubs still return `OUTPUT_BUFFER_TOO_SMALL`.

- [ ] **Step 2.5: Implement vertex one-ring helpers**

Replace the two vertex stubs in `src/half_edge/walks.c3`:

```c3
fn int? HalfEdgeMesh.vertex_one_ring_outgoing(&self, VertexIndex v, HeIndex[] out)
{
    HeIndex start = self.vertices[v].half_edge;
    if (start == cg::INVALID_HE) return 0;

    HeIndex cursor = start;
    usz count = 0;

    while (true) {
        if (count >= out.len) return cg::OUTPUT_BUFFER_TOO_SMALL~;
        out[count] = cursor;
        count++;

        HeIndex prev_he = self.prev(cursor);
        HeIndex next_he = self.half_edges[prev_he].twin;
        if (next_he == cg::INVALID_HE) break;
        cursor = next_he;
        if (cursor == start) break;
    }

    return (int)count;
}

fn int? HalfEdgeMesh.vertex_one_ring_faces(&self, VertexIndex v, FaceIndex[] out)
{
    HeIndex start = self.vertices[v].half_edge;
    if (start == cg::INVALID_HE) return 0;

    HeIndex cursor = start;
    usz count = 0;

    while (true) {
        FaceIndex face = self.half_edges[cursor].face;
        if (face != cg::INVALID_FACE) {
            if (count >= out.len) return cg::OUTPUT_BUFFER_TOO_SMALL~;
            out[count] = face;
            count++;
        }

        HeIndex prev_he = self.prev(cursor);
        HeIndex next_he = self.half_edges[prev_he].twin;
        if (next_he == cg::INVALID_HE) break;
        cursor = next_he;
        if (cursor == start) break;
    }

    return (int)count;
}
```

- [ ] **Step 2.6: Run tests and verify GREEN**

```bash
c3c build static-lib && c3c test
```

Expected: all tests pass.

- [ ] **Step 2.7: Commit vertex walks**

```bash
git add src/half_edge/walks.c3 test/test_walks.c3
git commit -m "half_edge: add vertex one-ring walks (M2)"
```

---

### Task 3: Implement validation for required arrays and reference bounds

**Files:**
- Modify: `src/half_edge/validate.c3`
- Modify: `test/test_walks.c3`

- [ ] **Step 3.1: Add validation good-case and required-array tests**

Append:

```c3
fn void test_validate_tetrahedron() @test
{
    HalfEdgeMesh mesh = half_edge::from_triangles(mem, TET_POSITIONS[..], TET_INDICES[..])!!;
    defer mesh.destroy();
    mesh.validate()!!;
}

fn void test_validate_single_triangle_boundary() @test
{
    HalfEdgeMesh mesh = half_edge::from_triangles(mem, TRI_POSITIONS[..], TRI_INDICES[..])!!;
    defer mesh.destroy();
    mesh.validate()!!;
}

fn void test_validate_polygon_quad() @test
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
    mesh.validate()!!;
}

fn void test_validate_empty_topology_fault() @test
{
    HalfEdgeMesh mesh;
    expect_void_fault(mesh.validate(), cg::INVALID_TOPOLOGY);
}
```

- [ ] **Step 3.2: Add reference-bound tests**

Append:

```c3
fn void test_validate_bad_origin_fault() @test
{
    HalfEdgeMesh mesh = half_edge::from_triangles(mem, TRI_POSITIONS[..], TRI_INDICES[..])!!;
    defer mesh.destroy();
    mesh.half_edges[0].origin = 99;
    expect_void_fault(mesh.validate(), cg::INVALID_VERTEX_REFERENCE);
}

fn void test_validate_bad_next_fault() @test
{
    HalfEdgeMesh mesh = half_edge::from_triangles(mem, TRI_POSITIONS[..], TRI_INDICES[..])!!;
    defer mesh.destroy();
    mesh.half_edges[0].next = 99;
    expect_void_fault(mesh.validate(), cg::INVALID_HALF_EDGE_REFERENCE);
}

fn void test_validate_bad_face_reference_fault() @test
{
    HalfEdgeMesh mesh = half_edge::from_triangles(mem, TRI_POSITIONS[..], TRI_INDICES[..])!!;
    defer mesh.destroy();
    mesh.half_edges[0].face = 99;
    expect_void_fault(mesh.validate(), cg::INVALID_FACE_REFERENCE);
}

fn void test_validate_bad_face_half_edge_fault() @test
{
    HalfEdgeMesh mesh = half_edge::from_triangles(mem, TRI_POSITIONS[..], TRI_INDICES[..])!!;
    defer mesh.destroy();
    mesh.faces[0].half_edge = 99;
    expect_void_fault(mesh.validate(), cg::INVALID_HALF_EDGE_REFERENCE);
}

fn void test_validate_bad_vertex_half_edge_fault() @test
{
    HalfEdgeMesh mesh = half_edge::from_triangles(mem, TRI_POSITIONS[..], TRI_INDICES[..])!!;
    defer mesh.destroy();
    mesh.vertices[0].half_edge = 99;
    expect_void_fault(mesh.validate(), cg::INVALID_HALF_EDGE_REFERENCE);
}
```

- [ ] **Step 3.3: Run tests and verify RED**

```bash
c3c test
```

Expected: validation tests fail because `validate` still returns `INVALID_TOPOLOGY` for every mesh.

- [ ] **Step 3.4: Implement required-array and reference-bound validation**

Replace `src/half_edge/validate.c3` with:

```c3
module cg::half_edge;
import cg;

@private fn bool is_valid_he_index(HalfEdgeMesh* mesh, HeIndex he)
{
    if (he < 0) return false;
    return (usz)(int)he < mesh.half_edges.len;
}

@private fn bool is_valid_vertex_index(HalfEdgeMesh* mesh, VertexIndex v)
{
    if (v < 0) return false;
    return (usz)(int)v < mesh.vertices.len;
}

@private fn bool is_valid_face_index(HalfEdgeMesh* mesh, FaceIndex f)
{
    if (f < 0) return false;
    return (usz)(int)f < mesh.faces.len;
}

fn void? HalfEdgeMesh.validate(&self)
{
    if (self.half_edges.len == 0 || self.faces.len == 0 || self.vertices.len == 0) {
        return cg::INVALID_TOPOLOGY~;
    }
    if (self.positions.len != self.vertices.len) return cg::INVALID_VERTEX_REFERENCE~;
    if (self.normals.len > 0 && self.normals.len != self.positions.len) return cg::ATTRIBUTE_COUNT_MISMATCH~;
    if (self.uvs.len > 0 && self.uvs.len != self.positions.len) return cg::ATTRIBUTE_COUNT_MISMATCH~;

    for (usz i = 0; i < self.half_edges.len; i++) {
        HalfEdge he = self.half_edges[i];
        if (!is_valid_vertex_index(self, he.origin)) return cg::INVALID_VERTEX_REFERENCE~;
        if (!is_valid_he_index(self, he.next)) return cg::INVALID_HALF_EDGE_REFERENCE~;
        if (he.twin != cg::INVALID_HE && !is_valid_he_index(self, he.twin)) return cg::INVALID_HALF_EDGE_REFERENCE~;
        if (!is_valid_face_index(self, he.face)) return cg::INVALID_FACE_REFERENCE~;
    }

    for (usz i = 0; i < self.faces.len; i++) {
        if (!is_valid_he_index(self, self.faces[i].half_edge)) return cg::INVALID_HALF_EDGE_REFERENCE~;
    }

    for (usz i = 0; i < self.vertices.len; i++) {
        HeIndex he = self.vertices[i].half_edge;
        if (he == cg::INVALID_HE) continue;
        if (!is_valid_he_index(self, he)) return cg::INVALID_HALF_EDGE_REFERENCE~;
        if (self.half_edges[he].origin != (VertexIndex)(int)i) return cg::INVALID_VERTEX_REFERENCE~;
    }

    return;
}
```

The snippet above uses `@private` free helper functions instead of public `HalfEdgeMesh` methods so C3 does not expose them as part of the mesh API.

- [ ] **Step 3.5: Run tests and verify GREEN**

```bash
c3c build static-lib && c3c test
```

Expected: all tests pass.

- [ ] **Step 3.6: Commit reference validation**

```bash
git add src/half_edge/validate.c3 test/test_walks.c3
git commit -m "half_edge: validate mesh reference bounds (M2)"
```

---

### Task 4: Implement twin and face-cycle validation

**Files:**
- Modify: `src/half_edge/validate.c3`
- Modify: `test/test_walks.c3`

- [ ] **Step 4.1: Add twin validation tests**

Append:

```c3
fn void test_validate_bad_twin_roundtrip_fault() @test
{
    HalfEdgeMesh mesh = half_edge::from_triangles(mem, TET_POSITIONS[..], TET_INDICES[..])!!;
    defer mesh.destroy();

    HeIndex twin = mesh.half_edges[0].twin;
    mesh.half_edges[twin].twin = 1;

    expect_void_fault(mesh.validate(), cg::INVALID_TWIN);
}

fn void test_validate_bad_twin_endpoints_fault() @test
{
    HalfEdgeMesh mesh = half_edge::from_triangles(mem, TET_POSITIONS[..], TET_INDICES[..])!!;
    defer mesh.destroy();

    HeIndex twin = mesh.half_edges[0].twin;
    mesh.half_edges[twin].next = mesh.half_edges[0].next;

    expect_void_fault(mesh.validate(), cg::INVALID_TWIN);
}
```

- [ ] **Step 4.2: Add face-cycle validation tests**

Append:

```c3
fn void test_validate_bad_face_cycle_fault() @test
{
    HalfEdgeMesh mesh = half_edge::from_triangles(mem, TET_POSITIONS[..], TET_INDICES[..])!!;
    defer mesh.destroy();

    mesh.half_edges[1].face = 1;

    expect_void_fault(mesh.validate(), cg::INVALID_FACE_CYCLE);
}

fn void test_validate_non_closing_face_cycle_fault() @test
{
    HalfEdgeMesh mesh = half_edge::from_triangles(mem, TRI_POSITIONS[..], TRI_INDICES[..])!!;
    defer mesh.destroy();

    mesh.half_edges[2].next = 1;

    expect_void_fault(mesh.validate(), cg::INVALID_FACE_CYCLE);
}

fn void test_validate_degree_too_small_fault() @test
{
    HalfEdgeMesh mesh = half_edge::from_triangles(mem, TRI_POSITIONS[..], TRI_INDICES[..])!!;
    defer mesh.destroy();

    mesh.half_edges[0].next = 0;

    expect_void_fault(mesh.validate(), cg::INVALID_FACE_CYCLE);
}
```

Important: avoid doctored changes that trigger earlier reference-bound or twin faults unless that is the test's purpose. Each corruption should isolate the expected first fault.

- [ ] **Step 4.3: Run tests and verify RED**

```bash
c3c test
```

Expected: new twin and face-cycle tests fail because `validate` does not yet check these invariants.

- [ ] **Step 4.4: Add twin invariant checks**

In `validate`, after reference-bound loops and before face-cycle checks, add:

```c3
    for (usz i = 0; i < self.half_edges.len; i++) {
        HeIndex he_index = (HeIndex)(int)i;
        HalfEdge he = self.half_edges[i];
        if (he.twin == cg::INVALID_HE) continue;

        HalfEdge twin = self.half_edges[he.twin];
        if (twin.twin != he_index) return cg::INVALID_TWIN~;

        VertexIndex he_to = self.half_edges[he.next].origin;
        VertexIndex twin_to = self.half_edges[twin.next].origin;
        if (he.origin != twin_to) return cg::INVALID_TWIN~;
        if (he_to != twin.origin) return cg::INVALID_TWIN~;
    }
```

- [ ] **Step 4.5: Add face-cycle checks**

After twin checks, add:

```c3
    for (usz face_index = 0; face_index < self.faces.len; face_index++) {
        FaceIndex face = (FaceIndex)(int)face_index;
        HeIndex start = self.faces[face_index].half_edge;
        HeIndex cursor = start;
        usz degree = 0;

        while (true) {
            if (degree >= self.half_edges.len) return cg::INVALID_FACE_CYCLE~;
            if (self.half_edges[cursor].face != face) return cg::INVALID_FACE_CYCLE~;
            degree++;

            cursor = self.half_edges[cursor].next;
            if (cursor == start) break;
        }

        if (degree < 3) return cg::INVALID_FACE_CYCLE~;
    }
```

- [ ] **Step 4.6: Run tests and verify GREEN**

```bash
c3c build static-lib && c3c test
```

Expected: all tests pass.

- [ ] **Step 4.7: Commit twin and cycle validation**

```bash
git add src/half_edge/validate.c3 test/test_walks.c3
git commit -m "half_edge: validate twin and face cycles (M2)"
```

---

### Task 5: Add remaining validation regressions and polish

**Files:**
- Modify: `src/half_edge/validate.c3`
- Modify: `test/test_walks.c3`

- [ ] **Step 5.1: Add canonical vertex and attribute mismatch tests**

Append:

```c3
fn void test_validate_bad_vertex_canonical_fault() @test
{
    HalfEdgeMesh mesh = half_edge::from_triangles(mem, TRI_POSITIONS[..], TRI_INDICES[..])!!;
    defer mesh.destroy();

    mesh.vertices[0].half_edge = 1;

    expect_void_fault(mesh.validate(), cg::INVALID_VERTEX_REFERENCE);
}

fn void test_validate_attribute_mismatch_fault() @test
{
    HalfEdgeMesh mesh = half_edge::from_triangles_with_attrs(
        mem, TET_POSITIONS[..], TET_NORMALS[..], TET_UVS[..], TET_INDICES[..])!!;
    defer mesh.destroy();

    Vec3f[] original_normals = mesh.normals;
    mesh.normals = original_normals[0..2];

    expect_void_fault(mesh.validate(), cg::ATTRIBUTE_COUNT_MISMATCH);

    mesh.normals = original_normals;
}
```

The test restores the original heap-owned normals slice before `defer mesh.destroy()` runs, so destroy still frees the correct allocation.

- [ ] **Step 5.2: Run tests and verify behavior**

```bash
c3c build static-lib && c3c test
```

Expected: all tests pass. If the attribute mismatch test faults with `INVALID_VERTEX_REFERENCE` first, move it earlier in validation only if doing so does not contradict the spec's order; otherwise adjust the test to isolate the attribute mismatch on an otherwise valid mesh.

- [ ] **Step 5.3: Run anti-pattern scan on new production files**

```bash
python3 - <<'PY'
from pathlib import Path
files = [Path('src/half_edge/walks.c3'), Path('src/half_edge/validate.c3')]
needles = ['assert(', 'malloc', 'sizeof(', '->', 'goto', 'alloc::new_array']
for path in files:
    text = path.read_text()
    for needle in needles:
        if needle in text:
            print(f'{path}: contains {needle}')
PY
```

Expected: no output.

- [ ] **Step 5.4: Run full milestone verification**

```bash
git diff --check
c3c build static-lib && c3c test && c3c build debug && c3c build release
```

Expected:
- `git diff --check` has no output.
- static-lib build succeeds.
- all tests pass.
- debug and release static libraries build.

- [ ] **Step 5.5: Commit polish**

```bash
git add src/half_edge/validate.c3 test/test_walks.c3
git commit -m "half_edge: complete validation regressions (M2)"
```

If Task 5 produces no code changes after tests already exist and pass, skip this commit.

---

### Task 6: Final M2 review and branch completion

**Files:**
- Review all changed M2 files.

- [ ] **Step 6.1: Inspect final diff**

```bash
git --no-pager diff main...HEAD -- src/faults.c3i project.json manifest.json src/half_edge/walks.c3 src/half_edge/validate.c3 test/test_walks.c3
```

If working directly on `main`, use:

```bash
git --no-pager log --oneline -8
git --no-pager show --stat HEAD
```

- [ ] **Step 6.2: Run final verification**

```bash
git diff --check
c3c build static-lib && c3c test && c3c build debug && c3c build release
```

Expected: no diff-check output; builds and tests pass.

- [ ] **Step 6.3: Review against spec checklist**

Confirm every spec item is covered:

- [ ] `src/half_edge/walks.c3` exists and implements all four walk helpers.
- [ ] `src/half_edge/validate.c3` exists and implements `validate`.
- [ ] Walk helpers do not allocate.
- [ ] Validation does not allocate or mutate the mesh.
- [ ] `OUTPUT_BUFFER_TOO_SMALL` is returned for undersized walk buffers.
- [ ] Boundary meshes validate.
- [ ] Boundary vertex walks stop without fault.
- [ ] Closed tetrahedron one-ring degree is 3 for every vertex.
- [ ] Degree-5 vertex fixture is tested.
- [ ] Doctored invalid meshes return granular faults.
- [ ] `project.json` and `manifest.json` include new source files.
- [ ] `c3c build static-lib && c3c test` is green.

- [ ] **Step 6.4: Request final code review**

Dispatch reviewers with this context:

```text
Review M2 half-edge walks and validation implementation in c3cg.c3l.
Spec: docs/specs/2026-05-05-m2-walks-validation-design.md
Plan: docs/plans/2026-05-05-m2-walks-validation.md
Focus: spec coverage, C3 idioms, no allocation in walks/validate, correct boundary behavior, no invalid dereferences before bounds checks, deterministic tests.
Return blocking issues only plus advisory notes.
```

- [ ] **Step 6.5: Fix any blocking review issues**

Use TDD where fixes affect behavior:

```bash
c3c build static-lib && c3c test
```

Commit fixes with a focused message, e.g.:

```bash
git commit -m "fix: correct M2 validation edge case"
```

- [ ] **Step 6.6: Finish development branch**

Use `finishing-a-development-branch` after final verification. Present the four completion options and wait for user choice.
