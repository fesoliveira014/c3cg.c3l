# M3 — Rendering Data Extraction

**Date:** 2026-05-06
**Module:** `cg::render`
**C3 version:** 0.7.11
**Depends on:** M1 half-edge construction, M2 walks and validation, `src/` source layout

## Goal

Add lean rendering data extraction for `HalfEdgeMesh`.

M3 converts a validated half-edge mesh into caller-owned, GPU-uploadable flat arrays: copied vertices, triangulated indices, and full-length normals/uvs arrays. It does not create GPU resources and does not depend on any rendering backend.

M3 intentionally splits richer geometry behavior into a later M3b. Generated normals and polygon planarity checks are out of scope for this milestone.

`c3c build static-lib && c3c test` must stay green.

## Scope

Create:

| File | Module | Purpose |
|------|--------|---------|
| `src/render/rendering_data.c3` | `cg::render` | `RenderingData`, `destroy`, `HalfEdgeMesh.to_rendering_data` |
| `test/test_rendering_data.c3` | `test` | Rendering extraction coverage |

Update:

| File | Change |
|------|--------|
| `project.json` | Add `src/render/rendering_data.c3` |
| `manifest.json` | Add `src/render/rendering_data.c3` |

Out of scope:

- GPU handles, buffers, graphics APIs, or render-backend integration.
- Generated normals when `mesh.normals` is empty.
- Polygon planarity checks and `NON_PLANAR_FACE`.
- Flat-shaded expanded output.
- Automatic invalidation or live views into a source mesh.
- New faults. M3 reuses validation and allocation faults.

## API

```c3
module cg::render;
import cg;
import cg::half_edge;

struct RenderingData {
    Vec3f[] vertices;
    uint[]  indices;
    Vec3f[] normals;
    Vec2f[] uvs;
}

fn void RenderingData.destroy(&self);
fn RenderingData? HalfEdgeMesh.to_rendering_data(&self, Allocator alloc);
```

`to_rendering_data` uses the explicit allocator convention chosen for the library. The returned `RenderingData` owns all four slices and the caller must call `data.destroy()` when done.

`RenderingData` is a snapshot. It does not reference the source mesh after extraction.

## Ownership and cleanup

`to_rendering_data` allocates all returned slices with `mem::alloc::new_array(alloc, T, len)`:

- `vertices`
- `indices`
- `normals`
- `uvs`

Each successful allocation is immediately registered with `defer catch free(slice)` so any later fault cleans up already-owned memory.

`RenderingData.destroy(&self)` frees all four slices and then zeroes the struct:

```c3
fn void RenderingData.destroy(&self)
{
    free(self.vertices);
    free(self.indices);
    free(self.normals);
    free(self.uvs);
    *self = {};
}
```

Production code must not use `assert`, `unreachable`, raw `malloc`, `sizeof`, `->`, `goto`, null-as-error, or bare `catch`.

## Extraction algorithm

`HalfEdgeMesh.to_rendering_data(&self, Allocator alloc)` executes in this order:

1. Call `self.validate()!`.
2. Count output triangles from face degrees.
3. Allocate output slices.
4. Copy positions and attrs.
5. Emit triangulated indices.
6. Return the owned `RenderingData`.

### 1. Validate first

`to_rendering_data` calls `self.validate()!` before any extraction work. Invalid topology returns the existing M2 validation fault directly. This keeps rendering extraction safe against corrupted references and lets the implementation use M2 traversal helpers under their valid-mesh precondition.

### 2. Count output triangles

For each face `f`:

- `degree = self.face_degree(f)`
- A degree-3 face emits 1 triangle.
- A degree-N polygon emits `degree - 2` fan triangles.

Because `validate()` already rejects face cycles with degree less than 3, no special degree fault is needed here.

Total index count:

```text
sum_over_faces((face_degree(f) - 2) * 3)
```

### 3. Allocate output slices

The output preserves source vertex sharing:

- `data.vertices.len == self.positions.len`
- `data.normals.len == self.positions.len`
- `data.uvs.len == self.positions.len`
- `data.indices.len == total_triangle_count * 3`

M3 does not expand vertices per emitted triangle.

### 4. Copy and fill attributes

Positions:

- Copy `self.positions[i]` to `data.vertices[i]`.

Normals:

- If `self.normals.len > 0`, copy `self.normals[i]` to `data.normals[i]`.
- If `self.normals.len == 0`, fill every output normal with `{0, 0, 0}`.

UVs:

- If `self.uvs.len > 0`, copy `self.uvs[i]` to `data.uvs[i]`.
- If `self.uvs.len == 0`, fill every output uv with `{0, 0}`.

`validate()` already guarantees non-empty attrs match `positions.len`, so no separate M3 attr mismatch check is needed.

### 5. Emit indices

For each face:

1. Get the face vertices in cycle order.
2. Emit fan triangles around the first vertex.

For a triangle face with vertices `[v0, v1, v2]`, emit:

```text
v0, v1, v2
```

For a polygon face with vertices `[v0, v1, v2, v3, ..., vN-1]`, emit:

```text
v0, v1, v2
v0, v2, v3
...
v0, vN-2, vN-1
```

Indices are stored as `uint`, matching the builder input and `RenderingData.indices` type. `VertexIndex` is an inline `int`, so emission casts each vertex index to `uint` after validation has already guaranteed it came from valid builder-style mesh indices.

M3 assumes polygon faces are fan-safe. Non-convex polygon handling and planarity checks are deferred to M3b.

## Scratch storage for face vertices

The implementation needs a temporary `VertexIndex[]` buffer to collect one face's vertices before fan emission.

Recommended approach:

1. First pass over faces computes:
   - total output triangle count
   - maximum face degree
2. Allocate one scratch buffer with `mem::alloc::new_array(alloc, VertexIndex, max_face_degree)`.
3. Reuse that scratch buffer for every face during index emission.
4. Free the scratch buffer before returning; it is not part of `RenderingData`.

This keeps extraction O(V + F + output triangles), avoids per-face heap churn, and supports polygon faces of any degree.

If scratch allocation fails, return the allocation fault and let `defer catch` clean up any previously allocated output slices.

## Extraction pseudocode

This is C3-shaped pseudocode, not final source. It fixes the intended implementation order and cleanup pattern for planning. It assumes the imports listed in the module section, including `import std::mem;`.

```c3
fn RenderingData? HalfEdgeMesh.to_rendering_data(&self, Allocator alloc)
{
    self.validate()!;

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

Implementation notes:

- The first pass over faces computes both `triangle_count` and `max_face_degree`.
- `validate()` guarantees each face degree is at least 3, so `face_degree - 2` cannot underflow.
- The output arrays use `defer catch`; they are returned on success and freed only on fault.
- The scratch `face_vertices` buffer uses plain `defer` because it is always temporary and never returned.
- The final implementation may factor copy/fill loops into small helpers, but helpers should stay plain module-scope functions on c3c 0.7.11.
- `out_index` should equal `data.indices.len` after emission; tests cover this through exact index counts and index contents.

## Fault behavior

M3 adds no new faults.

`to_rendering_data` can return:

- Any fault from `self.validate()`:
  - `INVALID_TOPOLOGY`
  - `INVALID_VERTEX_REFERENCE`
  - `ATTRIBUTE_COUNT_MISMATCH`
  - `INVALID_HALF_EDGE_REFERENCE`
  - `INVALID_FACE_REFERENCE`
  - `INVALID_TWIN`
  - `INVALID_FACE_CYCLE`
- Allocation faults from `mem::alloc::new_array`.

M3 does not return `NON_TRIANGLE_FACE`; polygon faces are supported by fan triangulation.

M3 does not return `NON_PLANAR_FACE`; that belongs to M3b.

## Module and dependency notes

`src/render/rendering_data.c3` declares `module cg::render;` and imports:

```c3
import cg;
import cg::half_edge;
import std::mem;
```

The module depends on M2 methods:

- `HalfEdgeMesh.validate`
- `HalfEdgeMesh.face_degree`
- `HalfEdgeMesh.face_vertices`

No render module symbols are needed by `cg::half_edge`, so this dependency is one-way and avoids cycles.

## Tests

Create `test/test_rendering_data.c3` under `module test;` and import `cg`, `cg::half_edge`, and `cg::render`.

Test cases:

1. `test_render_triangle_preserves_vertices_and_indices`
   - Build a single triangle mesh.
   - Convert to rendering data.
   - Assert `vertices.len == 3`, `indices.len == 3`, `normals.len == 3`, `uvs.len == 3`.
   - Assert vertices match source positions.
   - Assert indices are `0, 1, 2` in face-cycle order.

2. `test_render_tetrahedron_index_count`
   - Build tetrahedron fixture.
   - Convert to rendering data.
   - Assert `vertices.len == 4` and `indices.len == 12`.

3. `test_render_quad_fan_triangulates`
   - Build one quad with `from_polygons` using positions `{0,0,0}`, `{1,0,0}`, `{1,1,0}`, `{0,1,0}`, `face_offsets = {0, 4}`, and `face_indices = {0, 1, 2, 3}`.
   - Convert to rendering data.
   - Assert indices are `0, 1, 2, 0, 2, 3`.

4. `test_render_copies_normals_and_uvs`
   - Build a triangle or tetrahedron with attrs using `from_triangles_with_attrs`.
   - Convert to rendering data.
   - Assert output normals/uvs equal source attrs.

5. `test_render_missing_attrs_zero_filled`
   - Build a mesh without normals/uvs.
   - Convert to rendering data.
   - Assert normals and uvs are full length and zero-filled.

6. `test_render_destroy_zeroes_data`
   - Convert a valid mesh.
   - Call `data.destroy()`.
   - Assert all slices have length 0.

7. `test_render_invalid_mesh_returns_validation_fault`
   - Build a valid mesh.
   - Doctor a single topology reference: `mesh.half_edges[0].next = -2`.
   - Call `to_rendering_data`.
   - Assert it returns `cg::INVALID_HALF_EDGE_REFERENCE`, proving validation runs before extraction.

8. `test_render_output_owned_independently`
   - Convert a valid mesh.
   - Mutate `mesh.positions[0]` after extraction.
   - Assert `data.vertices[0]` keeps the original value.

All tests may use `assert`, `unreachable`, and `!!`.

## Verification

Before M3 is complete:

```bash
git diff --check
c3c build static-lib
c3c test
c3c build debug
c3c build release
```

Expected test count increases from the M2 baseline of 57 to 65 after the 8 M3 rendering tests are added.

## Deferred to M3b

M3b should decide and implement:

- Generated normals when `mesh.normals` is empty.
- Face/polygon planarity checks.
- A `NON_PLANAR_FACE` fault if planarity is required before fan triangulation.
- Whether generated normals are smooth vertex normals, face-averaged vertex normals, or flat normals requiring expanded vertices.

Keeping these decisions out of M3 prevents normal-generation policy from blocking the basic owned rendering-data snapshot API.
