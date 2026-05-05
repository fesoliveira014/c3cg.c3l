# M1 — Half-Edge Construction and Topology Queries

**Date:** 2026-05-04
**Module:** `cg::half_edge`
**C3 version:** 0.7.11 (pinned in `.claude/c3-skill.json`)
**Depends on:** M0

## Goal

Implement mesh construction (`from_triangles`, `from_triangles_with_attrs`, `from_polygons`) and topology query methods on `HalfEdgeMesh`. The constructor builds the half-edge data structure from indexed triangle/polygon data using a hash-map twin-pairing pass. Topology methods provide read-only index access to the mesh structure. All methods are declared in `module cg::half_edge` on the `cg::HalfEdgeMesh` type (cross-module method pattern verified in M0).

`c3c build static-lib && c3c test` green.

## Files Created / Modified

| File | Action | Module | What |
|------|--------|--------|------|
| `project.json` | Update | — | Add `half_edge/builder.c3`, `half_edge/topology.c3` to sources |
| `manifest.json` | Update | — | Add `half_edge/builder.c3`, `half_edge/topology.c3` to sources |
| `faults.c3i` | Update | `cg` | Add `ATTRIBUTE_COUNT_MISMATCH` fault |
| `half_edge/builder.c3` | Create | `cg::half_edge` | `from_triangles`, `from_triangles_with_attrs`, `from_polygons`, `HalfEdgeMesh.destroy` |
| `half_edge/topology.c3` | Create | `cg::half_edge` | `twin`, `next`, `prev`, `from_vertex`, `to_vertex`, `face_of`, `is_boundary`, `face_degree` |
| `test/test_builder.c3` | Create | `test` | Tetrahedron round-trip + degenerate input fault path tests |
| `test/test_topology.c3` | Create | `test` | Twin/next/from/to spot-checks on tetrahedron and hand-built diamond |

Files with function bodies use `.c3` extension. The `half_edge/` directory contains `module cg::half_edge;` files.

## `project.json` Changes

Append to `"sources"`:
```json
"half_edge/builder.c3",
"half_edge/topology.c3"
```

Full sources list:
```json
"sources": [
    "c3cg.c3i",
    "types.c3i",
    "faults.c3i",
    "half_edge_mesh.c3i",
    "smoke/smoke.c3",
    "half_edge/builder.c3",
    "half_edge/topology.c3"
]
```

## `manifest.json` Changes

Append to `"sources"`:
```json
"half_edge/builder.c3",
"half_edge/topology.c3"
```

## `half_edge/builder.c3` — Mesh Construction

```c3
module cg::half_edge;
import cg;
import std::collections::map;

fn HalfEdgeMesh? from_triangles(Allocator alloc, Vec3f[] positions, uint[] indices);
fn HalfEdgeMesh? from_triangles_with_attrs(
    Allocator alloc,
    Vec3f[] positions,
    Vec3f[] normals,
    Vec2f[] uvs,
    uint[]  indices,
);
fn HalfEdgeMesh? from_polygons(Allocator alloc, Vec3f[] positions, uint[] face_offsets, uint[] face_indices);
fn void HalfEdgeMesh.destroy(&self);
```

### Algorithm — `from_triangles`

**Phase 1 — Validate input:**
- `indices.len % 3 != 0` → `INVALID_TRIANGLE_INDEX_COUNT~`
- `positions.len == 0 || indices.len == 0` → `EMPTY_INPUT~`
- Any index ≥ `positions.len` → `INDEX_OUT_OF_RANGE~`

**Phase 2 — Allocate topology arrays:**
- `face_count = indices.len / 3`
- `half_edge_count = indices.len` (3 per face)
- Allocate via `mem::alloc::new_array(alloc, ...)`:
  - `HalfEdge[half_edge_count]` → `mesh.half_edges`
  - `HalfEdgeFace[face_count]` → `mesh.faces`
  - `HalfEdgeVertex[positions.len]` → `mesh.vertices`
- Initialize vertices: `for (uint i = 0; i < positions.len; i++) vertices[i].half_edge = INVALID_HE;`
- Copy positions: `mem::alloc::new_array(alloc, Vec3f, positions.len)` + element copy
- Zero-fill `normals` and `uvs` as empty slices
- Initialize `HashMap{int[<2>], HeIndex} edge_map` with `edge_map.init(alloc)`

**Phase 3 — Build half-edges (first pass):**
For each triangle `t` (0..face_count-1):
  ```c3
  i0 = indices[t*3]; i1 = indices[t*3+1]; i2 = indices[t*3+2];
  ```
  For each directed edge (i0→i1, i1→i2, i2→i0):
  - Set `he = t*3 + edge_idx`
  - `half_edges[he].origin = i*`
  - `half_edges[he].next = t*3 + (edge_idx+1)%3`
  - `half_edges[he].twin = INVALID_HE`
  - `half_edges[he].face = t`
  - If `edge_map` already has key `[i_from, i_to]` → `DUPLICATE_HALF_EDGE~`
  - `edge_map[{i_from, i_to}] = he`
  - If `vertices[i_from].half_edge == INVALID_HE`, set to `he`
  - `faces[t].half_edge = t*3`

**Phase 4 — Pair twins (second pass):**
For each half-edge `he` (0..half_edge_count-1):
  - If `half_edges[he].twin == INVALID_HE`:
    - `from_v = half_edges[he].origin`
    - `to_v = half_edges[half_edges[he].next].origin`
    - Lookup reversed: `edge_map[{to_v, from_v}]`
    - If found: set both twins
    - If not found: boundary edge — leave `INVALID_HE`

**Phase 5 — Free edge_map, return mesh:**
```c3
edge_map.free();
return mesh;
```

### Memory Management on Fault Paths

Construction allocates arrays and a HashMap. If a fault occurs mid-construction (e.g., `DUPLICATE_HALF_EDGE` in Phase 3), all prior allocations must be freed. Strategy:

```c3
// Immediately after each allocation:
half_edges = mem::alloc::new_array(alloc, HalfEdge, half_edge_count);
defer free(half_edges);  // freed on any exit (fault or success)
```

For the HashMap: `edge_map.init(alloc); defer edge_map.free();`

On success, the mesh struct owns the arrays — `defer free()` runs but `*self` still holds the pointers. Solution: on the success path, zero-out the local variables so `defer` is a noop (arrays are now owned by the returned mesh), or use a separate success/failure cleanup. Simplest approach: use `defer` on the error path only with `defer catch`.

```c3
fn HalfEdgeMesh? from_triangles(Allocator alloc, Vec3f[] positions, uint[] indices) {
    // Validate first (no allocations yet)

    HalfEdgeMesh mesh;
    mesh.half_edges = mem::alloc::new_array(alloc, HalfEdge, half_edge_count);
    defer catch free(mesh.half_edges);
    mesh.faces = mem::alloc::new_array(alloc, HalfEdgeFace, face_count);
    defer catch free(mesh.faces);
    // ... etc

    HashMap{int[<2>], HeIndex} edge_map;
    edge_map.init(alloc);
    defer catch edge_map.free();
    defer catch free(mesh.positions);
    defer catch free(mesh.vertices);

    // Build half-edges (may fault with DUPLICATE_HALF_EDGE~)
    // ... faulting here triggers all defer catch blocks

    // Success — mesh is fully constructed, don't free
    edge_map.free();  // explicit — don't need the map anymore
    return mesh;
}
```

`defer catch` runs only on fault exit. On success, the mesh owns its arrays and the caller must call `mesh.destroy()`.

### New Fault: `ATTRIBUTE_COUNT_MISMATCH`

`from_triangles_with_attrs` validates that `normals` and `uvs` lengths match. Add to `faults.c3i`:

```c3
ATTRIBUTE_COUNT_MISMATCH,
```

Returned when `normals.len > 0 && normals.len != positions.len` or `uvs.len > 0 && uvs.len != positions.len`.

### `from_triangles_with_attrs`

Same algorithm as `from_triangles` but:
- Instead of zero-filling `normals` and `uvs` (Phase 2), copies them from the parameters: `mem::alloc::new_array(alloc, Vec3f, positions.len)` for normals, `mem::alloc::new_array(alloc, Vec2f, positions.len)` for uvs, then element-wise copy. If either input array is empty, leave the corresponding mesh field as an empty slice.
- Validation first: check `ATTRIBUTE_COUNT_MISMATCH` for normals and uvs length vs positions.

### `from_polygons`

Same twin-pairing algorithm but iterates over CSR-formatted faces:
- `face_offsets` length = `face_count + 1`
- Face `f`'s vertices are `face_indices[face_offsets[f] .. face_offsets[f+1]]`
- Build half-edges with arbitrary cycle length (not fixed 3)
- `next` links wrap around within the face's vertex ring
- `INVALID_TRIANGLE_INDEX_COUNT` does not apply

**Validation rules:**
- `face_offsets.len < 2` → `EMPTY_INPUT~`
- `face_offsets[0] != 0` → `INDEX_OUT_OF_RANGE~`
- `face_offsets` must be non-decreasing; last element must equal `face_indices.len` — else `INDEX_OUT_OF_RANGE~`
- Any face with < 3 vertices → `DEGENERATE_INPUT~`
- All indices in `face_indices` < `positions.len` → `INDEX_OUT_OF_RANGE~`
- Duplicate directed edge → `DUPLICATE_HALF_EDGE~`

### `HalfEdgeMesh.destroy`

```c3
fn void HalfEdgeMesh.destroy(&self) {
    free(self.half_edges);
    free(self.faces);
    free(self.vertices);
    free(self.positions);
    free(self.normals);
    free(self.uvs);
    *self = {};
}
```

`*self = {}` zeroes the struct so double-free is a noop. Matches the `RenderingData.destroy` pattern in the architecture doc §4.6.

## `half_edge/topology.c3` — Topology Queries

```c3
module cg::half_edge;
import cg;

fn HeIndex     HalfEdgeMesh.twin(&self, HeIndex he);
fn HeIndex     HalfEdgeMesh.next(&self, HeIndex he);
fn HeIndex     HalfEdgeMesh.prev(&self, HeIndex he);
fn VertexIndex HalfEdgeMesh.from_vertex(&self, HeIndex he);
fn VertexIndex HalfEdgeMesh.to_vertex(&self, HeIndex he);
fn FaceIndex   HalfEdgeMesh.face_of(&self, HeIndex he);
fn bool        HalfEdgeMesh.is_boundary(&self, HeIndex he);
fn int         HalfEdgeMesh.face_degree(&self, FaceIndex f);
```

All pure index lookups. No allocation, no faults (per architecture §5.1).

### Implementations

| Method | Returns |
|--------|---------|
| `twin(he)` | `self.half_edges[he].twin` |
| `next(he)` | `self.half_edges[he].next` |
| `prev(he)` | Walk `next` from `he` around its face cycle until `.next == he`, then return the index of the half-edge whose `.next == he`. Works for both interior and boundary faces. |
| `from_vertex(he)` | `self.half_edges[he].origin` |
| `to_vertex(he)` | `self.half_edges[self.half_edges[he].next].origin` |
| `face_of(he)` | `self.half_edges[he].face` |
| `is_boundary(he)` | `self.half_edges[he].twin == INVALID_HE` |
| `face_degree(f)` | Count steps walking `next` from `faces[f].half_edge` until return |

`prev` is O(face_degree). `face_degree` is O(face_degree). All others are O(1).

## `test/test_builder.c3` — Construction Tests

```c3
module test;
import cg;
import cg::half_edge;

// Tetrahedron vertices: 4 points, 4 triangles
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
```

Tests:

| Test | What it verifies |
|------|-----------------|
| `test_tetrahedron_roundtrip` | Construct mesh, verify 4 faces, 12 half-edges, 4 vertices. Check each face has exactly 3 half-edges. Verify select twin pairs. |
| `test_tetrahedron_destroy` | Construct + destroy, verify no crash. |
| `test_tetrahedron_with_attrs` | Construct with normals and UVs. Verify mesh.normals and mesh.uvs match input element-wise. |
| `test_attrs_empty` | `from_triangles_with_attrs` with empty normals/uvs arrays produces empty mesh normals/uvs. |
| `test_attrs_mismatched` | `normals.len != positions.len` → `ATTRIBUTE_COUNT_MISMATCH` fault. Same for UVs. |
| `test_empty_input` | `positions = {}` → `EMPTY_INPUT` fault. |
| `test_empty_indices` | `indices = {}` → `EMPTY_INPUT` fault. |
| `test_count_mismatch` | `indices.len = 7` (not divisible by 3) → `INVALID_TRIANGLE_INDEX_COUNT` fault. |
| `test_index_out_of_range` | Index ≥ `positions.len` → `INDEX_OUT_OF_RANGE` fault. |
| `test_duplicate_edge` | Two faces with same directed edge → `DUPLICATE_HALF_EDGE` fault. |
| `test_polygon_quad` | `from_polygons` with a single quad → 1 face, 4 half-edges, degree 4. |
| `test_polygon_empty` | `from_polygons` with empty offsets → `EMPTY_INPUT` fault. |
| `test_polygon_degenerate` | Face with < 3 vertices → `DEGENERATE_INPUT` fault. |

## `test/test_topology.c3` — Topology Query Tests

Tests on a tetrahedron mesh:

| Test | What it verifies |
|------|-----------------|
| `test_twin_roundtrip` | For every interior HE, `twin(twin(he)) == he`. |
| `test_next_face_cycle` | Walking `next` from a face's `half_edge` returns to start after 3 steps. |
| `test_prev` | `prev(next(he)) == he` for every HE. |
| `test_from_vertex` | `from_vertex(he) == he_origin_index` on known edges. |
| `test_to_vertex` | `to_vertex(he)` equals the next HE's origin. |
| `test_face_of` | Every HE's `face_of` matches its triangle index. |
| `test_is_boundary` | Tetrahedron is closed → no boundary edges exist. |
| `test_boundary_edge` | Hand-built single triangle → 3 boundary edges, `is_boundary` true for all. |
| `test_boundary_prev` | `prev` on a boundary half-edge walks correctly (no crash). |
| `test_face_degree` | Every face of tetrahedron has degree 3. |
| `test_diamond_topology` | Hand-built 2-triangle diamond: verify twin links, face counts, vertex degrees. |

## Allocator Convention

All constructors take `Allocator alloc` as first parameter. Internal allocations use `mem::alloc::new_array(alloc, T, n)` and `map.init(alloc)`. Callers pass `mem` (standard heap allocator), `tmem` (temp allocator), or a custom allocator.

Destroy uses bare `free()` on each owned slice. This matches C3 0.7.11 allocation metadata: arrays allocated with `mem::alloc::new_array(alloc, T, n)` can be released with `free(slice)`. Callers still own the lifecycle and should call `mesh.destroy()` exactly once for heap/custom-allocated meshes. Temp-allocated meshes inside `@pool()` do not need explicit destroy unless the caller wants early release.

## Verification

```bash
c3c build static-lib && c3c test
```

Expected: static library created + all tests pass (2 existing + ~15 new).

## Notes

- Cross-module method pattern: `cg::half_edge` declares methods on `cg::HalfEdgeMesh`. Verified working in M0.
- `HashMap{int[<2>], HeIndex}` uses vector keys for edge pairing. Verified on c3c 0.7.11 — `HashMap{int[<2>], T}` compiles and works with vector-type keys.
- `mem::alloc::new_array(alloc, T, n)` for explicit-allocator array allocation.
- `*self = {}` zero-clears the struct in `destroy`, making double-free safe.
- `vertex.half_edge` set to first outgoing edge encountered, never overwritten — deterministic one-ring walks (per architecture §5.2).
- `from_polygons` shares the same twin-pairing pass as `from_triangles` — only the face-traversal loop differs (CSR vs fixed 3).
- Only one new fault needed: `ATTRIBUTE_COUNT_MISMATCH`. All other faults (`EMPTY_INPUT`, `INDEX_OUT_OF_RANGE`, `INVALID_TRIANGLE_INDEX_COUNT`, `DUPLICATE_HALF_EDGE`, `DEGENERATE_INPUT`) already exist in M0's `faults.c3i`.
