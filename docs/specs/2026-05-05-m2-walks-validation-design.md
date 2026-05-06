# M2 — Half-Edge Walks and Validation

**Date:** 2026-05-05
**Module:** `cg::half_edge`
**C3 version:** 0.7.11
**Depends on:** M1 and the `src/` source-layout reorg

## Goal

Add non-allocating topology walk helpers and mesh validation for `HalfEdgeMesh`.

M2 gives later milestones a stable way to enumerate face cycles, face vertices, vertex one-rings, and to reject corrupted topology before algorithms such as dual construction, rendering extraction, and edge flipping assume invariants. The API stays flat-array friendly: callers provide output buffers; walk helpers return a count or a fault.

`c3c build static-lib && c3c test` must stay green.

## Scope

Create:

| File | Module | Purpose |
|------|--------|---------|
| `src/half_edge/walks.c3` | `cg::half_edge` | Face-cycle and vertex one-ring walk helpers |
| `src/half_edge/validate.c3` | `cg::half_edge` | `HalfEdgeMesh.validate(&self)` topology checks |
| `test/test_walks.c3` | `test` | Walk and validation coverage |

Update:

| File | Change |
|------|--------|
| `src/faults.c3i` | Add M2 faults |
| `project.json` | Add new M2 source files |
| `manifest.json` | Add new M2 source files |

Out of scope:

- Edge flipping.
- Rendering extraction.
- Allocating iterator objects.
- Rich diagnostic structs with failing index/field. M2 returns only the first fault.
- Repairing corrupted meshes.

## API

```c3
module cg::half_edge;
import cg;

fn int?  HalfEdgeMesh.face_half_edges(&self, FaceIndex f, HeIndex[] out);
fn int?  HalfEdgeMesh.face_vertices(&self, FaceIndex f, VertexIndex[] out);
fn int?  HalfEdgeMesh.vertex_one_ring_outgoing(&self, VertexIndex v, HeIndex[] out);
fn int?  HalfEdgeMesh.vertex_one_ring_faces(&self, VertexIndex v, FaceIndex[] out);
fn void? HalfEdgeMesh.validate(&self);
```

All walk helpers return the number of elements written. They do not allocate. If the output slice cannot hold the result, they return `OUTPUT_BUFFER_TOO_SMALL~` and leave any already-written prefix unspecified.

`validate` returns normally for valid topology and returns the first detected fault otherwise.

## Faults

Add to `src/faults.c3i`:

```c3
OUTPUT_BUFFER_TOO_SMALL,
INVALID_HALF_EDGE_REFERENCE,
INVALID_VERTEX_REFERENCE,
INVALID_FACE_REFERENCE,
INVALID_TWIN,
INVALID_FACE_CYCLE,
INVALID_TOPOLOGY,
```

Usage:

| Fault | Returned when |
|-------|---------------|
| `OUTPUT_BUFFER_TOO_SMALL` | Caller-provided walk output buffer is too small |
| `INVALID_HALF_EDGE_REFERENCE` | A `next`, `twin`, `face.half_edge`, or `vertex.half_edge` reference is outside valid range, excluding documented `INVALID_HE` cases |
| `INVALID_VERTEX_REFERENCE` | A half-edge origin is outside `vertices`, a canonical vertex half-edge does not originate at its vertex, or `vertices.len != positions.len` |
| `INVALID_FACE_REFERENCE` | A half-edge face is outside `faces` |
| `INVALID_TWIN` | Twin round-trip or reversed-endpoint invariant fails |
| `INVALID_FACE_CYCLE` | A face cycle does not close, has a wrong face id, or has degree less than 3 |
| `INVALID_TOPOLOGY` | Required topology arrays are empty |

Existing `ATTRIBUTE_COUNT_MISMATCH` should be reused if `normals.len > 0 && normals.len != positions.len` or `uvs.len > 0 && uvs.len != positions.len` during validation.

Validation checks execute in the order listed below. Tests should assert the fault produced by a single isolated corruption so "first fault" behavior stays deterministic.

## Face Walks

### `face_half_edges(f, out)`

Algorithm:

1. Start at `self.faces[f].half_edge`.
2. Walk `next` until the cursor returns to start.
3. Write each visited `HeIndex` to `out` in face-cycle order.
4. Return count written.

Precondition for M2 walk helpers: the mesh is valid. Call `validate` when walking untrusted or doctored meshes. Tests cover valid inputs for walks and invalid inputs for `validate`.

Boundary policy: boundary half-edges still belong to their owning face, so face walks do not need ghost edges or special boundary handling.

### `face_vertices(f, out)`

Same traversal as `face_half_edges`, but writes `self.half_edges[he].origin` for each visited half-edge.

## Vertex One-Ring Walks

### Closed vertices

For closed manifold vertices, `vertex_one_ring_outgoing(v, out)` starts from `self.vertices[v].half_edge` and rotates around the vertex with:

```text
next outgoing = twin(prev(current))
```

It stops when it returns to the starting half-edge. The returned order is deterministic because M1 sets `vertices[v].half_edge` to the first outgoing edge encountered by construction.

`vertex_one_ring_faces(v, out)` uses the same walk and writes each outgoing half-edge's owning face.

### Boundary vertices

Boundary vertices are valid. Because M1 represents boundaries as ordinary face half-edges with `twin == INVALID_HE` and no ghost boundary edges, a boundary one-ring may be an open fan rather than a closed cycle.

M2 keeps the API simple:

- Start from the canonical `vertices[v].half_edge`.
- Walk with `twin(prev(current))` while twins exist.
- Stop when the next step would require `INVALID_HE`.
- Return the ordered reachable fan from the canonical half-edge.

This may be a partial fan for boundary vertices if the canonical half-edge is not at one end of the fan. Later algorithms that require complete closed one-rings, such as dual construction, must call `validate` and separately reject boundary meshes with an appropriate fault.

If a vertex has `INVALID_HE`, the one-ring count is zero.

## Validation

`HalfEdgeMesh.validate(&self)` checks constructed mesh invariants. It does not allocate and does not modify the mesh.

### 1. Required arrays and parallel geometry

- `half_edges.len > 0`
- `faces.len > 0`
- `vertices.len > 0`
- `positions.len == vertices.len`
- `normals.len == 0 || normals.len == positions.len`
- `uvs.len == 0 || uvs.len == positions.len`

Faults:

- Empty topology arrays: `INVALID_TOPOLOGY~`
- Vertex/position mismatch: `INVALID_VERTEX_REFERENCE~`
- Attribute mismatch: `ATTRIBUTE_COUNT_MISMATCH~`

### 2. Reference bounds

For each half-edge `he`:

- `origin` must be in `0 .. vertices.len`.
- `next` must be in `0 .. half_edges.len`.
- `twin` must be either `INVALID_HE` or in `0 .. half_edges.len`.
- `face` must be in `0 .. faces.len`.

For each face `f`:

- `faces[f].half_edge` must be in `0 .. half_edges.len`.

For each vertex `v`:

- `vertices[v].half_edge` must be either `INVALID_HE` or in `0 .. half_edges.len`.
- If not `INVALID_HE`, the referenced half-edge must have `origin == v`.

Faults:

- Bad half-edge index reference: `INVALID_HALF_EDGE_REFERENCE~`
- Bad vertex index/reference: `INVALID_VERTEX_REFERENCE~`
- Bad face index/reference: `INVALID_FACE_REFERENCE~`

### 3. Twin invariant

For each half-edge with `twin != INVALID_HE`:

- `self.half_edges[twin].twin == he`.
- Endpoints are reversed:
  - `self.half_edges[he].origin == self.half_edges[self.half_edges[twin].next].origin`
  - `self.half_edges[self.half_edges[he].next].origin == self.half_edges[twin].origin`

Fault: `INVALID_TWIN~`.

Boundary half-edges with `twin == INVALID_HE` are valid.

### 4. Face-cycle invariant

For each face `f`:

- Start at `faces[f].half_edge`.
- Walk `next`.
- Every visited half-edge must have `face == f`.
- The cycle must return to start within at most `half_edges.len` steps.
- Degree must be at least 3.

Fault: `INVALID_FACE_CYCLE~`.

## Tests

Create `test/test_walks.c3` under `module test;` and reuse shared test fixtures from existing `module test` files when possible.

Walk tests:

- `test_face_half_edges_triangle_cycle` — tetrahedron face returns 3 half-edges in cycle order.
- `test_face_vertices_triangle_cycle` — tetrahedron face returns the expected 3 vertices in cycle order.
- `test_face_vertices_quad_cycle` — polygon quad returns 4 vertices in order.
- `test_tetrahedron_vertex_one_ring_outgoing_degree_three` — every tetrahedron vertex has degree 3 and no `INVALID_HE` entries.
- `test_tetrahedron_vertex_one_ring_faces_degree_three` — every tetrahedron vertex reports 3 incident faces.
- `test_degree_five_vertex_one_ring` — hand-built closed fan or closed bipyramid-like fixture exercises a degree-5 vertex.
- `test_walk_output_buffer_too_small` — each walk family returns `OUTPUT_BUFFER_TOO_SMALL` when given an undersized buffer.
- `test_boundary_vertex_walk_stops_cleanly` — single triangle boundary vertex walk returns at least the canonical reachable edge and does not fault.

Validation tests:

- `test_validate_tetrahedron` — known-good closed mesh validates.
- `test_validate_single_triangle_boundary` — boundary mesh validates.
- `test_validate_polygon_quad` — polygonal face validates.
- `test_validate_empty_topology_fault` — empty topology arrays return `INVALID_TOPOLOGY`.
- `test_validate_bad_origin_fault` — doctored `origin` outside `vertices` returns `INVALID_VERTEX_REFERENCE`.
- `test_validate_bad_next_fault` — doctored `next` outside `half_edges` returns `INVALID_HALF_EDGE_REFERENCE`.
- `test_validate_bad_face_reference_fault` — doctored half-edge `.face` outside `faces` returns `INVALID_FACE_REFERENCE`.
- `test_validate_bad_face_half_edge_fault` — doctored `faces[f].half_edge` outside `half_edges` returns `INVALID_HALF_EDGE_REFERENCE`.
- `test_validate_bad_vertex_half_edge_fault` — doctored `vertices[v].half_edge` outside `half_edges` returns `INVALID_HALF_EDGE_REFERENCE`.
- `test_validate_bad_twin_roundtrip_fault` — doctored twin mismatch returns `INVALID_TWIN`.
- `test_validate_bad_twin_endpoints_fault` — mutual twins with non-reversed endpoints return `INVALID_TWIN`.
- `test_validate_bad_face_cycle_fault` — doctored wrong-face cycle or non-closing cycle returns `INVALID_FACE_CYCLE`.
- `test_validate_degree_too_small_fault` — doctored 1-gon or 2-gon face cycle returns `INVALID_FACE_CYCLE`.
- `test_validate_bad_vertex_canonical_fault` — vertex canonical half-edge not originating at that vertex returns `INVALID_VERTEX_REFERENCE`.
- `test_validate_attribute_mismatch_fault` — doctored attr length mismatch returns `ATTRIBUTE_COUNT_MISMATCH`.

Optional low-cost regression tests:

- `test_isolated_vertex_one_ring_zero` — vertex with `INVALID_HE` returns count 0 for both one-ring walk functions.
- `test_vertex_one_ring_deterministic` — repeated one-ring calls on the same vertex return identical output.

Tests may use `assert` and `unreachable`. Production code must return faults, not assert.

## Verification

Minimum gate:

```bash
c3c build static-lib && c3c test
```

Milestone boundary gate:

```bash
c3c build static-lib && c3c test && c3c build debug && c3c build release
```

Also run `git diff --check` before commit.

## Implementation Notes

- Validate first in functions that operate on untrusted structure. The walk helpers themselves may assume valid topology to stay simple and fast.
- Use `usz` for slice lengths and cast to `int` only for returned counts where needed.
- Do not allocate in `walks.c3` or `validate.c3`.
- Do not add runtime `assert` to production files.
- Keep module names unchanged: files live under `src/half_edge/`, but still declare `module cg::half_edge;`.
- Add new source files to both `project.json` and `manifest.json`.
