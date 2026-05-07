# M3b — Render Options, Generated Normals, Generated UVs, and Planarity

Date: 2026-05-07
Module: `cg::render`
C3 version: 0.7.11
Depends on: M3 rendering data extraction, M2 validation/walk helpers

## Goal

Extend `HalfEdgeMesh.to_rendering_data` with configurable rendering-output options.

M3b replaces the fixed M3 behavior with a single public extraction API that accepts a C3 bitstruct options value. Options control whether normals and UVs are omitted, copied, or generated. Extraction also validates polygon planarity before fan triangulation by default.

M3b keeps `RenderingData` as the one owned, GPU-uploadable output type. It does not add GPU resources, renderer integration, serialization, or a separate flat-rendering type.

## Public API

```c3
module cg::render;
import cg;
import cg::half_edge;

bitstruct RenderOptions : uint
{
    RenderNormalMode normal_mode : 0..1;
    RenderUvMode     uv_mode : 2..3;
    bool             skip_planarity_check : 4;
    uint             reserved : 5..31;
}

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

const float PLANAR_EPSILON = 0.00001;
const RenderOptions DEFAULT_RENDER_OPTIONS = {
    normal_mode = RENDER_NORMAL_NONE,
    uv_mode = RENDER_UV_NONE,
    skip_planarity_check = false,
};

fn RenderingData? HalfEdgeMesh.to_rendering_data(
    &self,
    Allocator alloc,
    RenderOptions options = DEFAULT_RENDER_OPTIONS,
);
```

Exact C3 syntax for `bitstruct` defaults and enum field widths must be verified against c3c 0.7.11 during implementation. If an enum field cannot be stored directly in a bitstruct, use a small integer field and helper accessors while keeping the public options type as a bitstruct.

`reserved` bits must be zero. If c3c rejects this exact reserved-field syntax, use the nearest verified bitstruct representation that still leaves bits 5..31 reserved and validates them.

## Option semantics

### Normal modes

- `RENDER_NORMAL_NONE`
  - Output `data.normals.len == 0`.
- `RENDER_NORMAL_SOURCE`
  - Copy `mesh.normals` to `data.normals`.
  - If `mesh.normals.len == 0`, return `MISSING_NORMALS`.
- `RENDER_NORMAL_SMOOTH`
  - Generate angle-weighted vertex normals.
  - Shared-output path: `data.normals.len == data.vertices.len == mesh.positions.len`.
- `RENDER_NORMAL_FLAT`
  - Generate one flat normal per triangle and duplicate it for each triangle vertex.
  - Requires every face to be triangular. If any face has degree other than 3, return `NON_TRIANGLE_FACE`.
  - Uses expanded-output path: `data.vertices.len == data.indices.len` and `data.indices[i] == i`.

### UV modes

- `RENDER_UV_NONE`
  - Output `data.uvs.len == 0`.
- `RENDER_UV_SOURCE`
  - Copy `mesh.uvs` to output UVs.
  - If `mesh.uvs.len == 0`, return `MISSING_UVS`.
- `RENDER_UV_SOURCE_OR_ZERO`
  - Copy `mesh.uvs` when present.
  - If missing, zero-fill output UVs.
- `RENDER_UV_PLANAR_PROJECT`
  - Generate UVs by automatic dominant-axis projection.
  - Normalize projected coordinates into `[0, 1]` using the projected mesh AABB.

### Planarity

By default, extraction checks every polygon face for planarity before fan triangulation. `skip_planarity_check = true` disables this check.

Planarity checks apply to polygon faces regardless of normal or UV mode. This protects the existing fan triangulation path from silently emitting invalid triangles for non-planar faces.

## Faults

Add these faults to `src/faults.c3i`:

- `INVALID_RENDER_OPTIONS`
- `MISSING_NORMALS`
- `MISSING_UVS`
- `NON_PLANAR_FACE`

Reuse existing faults:

- `NON_TRIANGLE_FACE` for flat normals on polygon faces.
- `DEGENERATE_INPUT` when a face cannot define a normal or plane.
- M2 validation faults from `self.validate()`.
- allocation faults from `mem::alloc::new_array`.

Validation order:

1. Validate option values. Return `INVALID_RENDER_OPTIONS` if any reserved bit is non-zero or, in a fallback integer-field implementation, if `normal_mode` or `uv_mode` is outside its defined enum range.
2. Call `self.validate()!`.
3. Validate required source attributes:
   - `RENDER_NORMAL_SOURCE` + missing normals -> `MISSING_NORMALS`.
   - `RENDER_UV_SOURCE` + missing UVs -> `MISSING_UVS`.
4. If `RENDER_NORMAL_FLAT`, validate all faces are triangular and non-degenerate before output allocation. This check runs before planarity so flat mode reports `NON_TRIANGLE_FACE` deterministically for polygon meshes.
5. If planarity is enabled, validate polygon faces before output allocation. Triangle faces are skipped, so flat triangle-only output does not do extra planarity work.

## Data flow

`to_rendering_data` dispatches after validation:

```text
validate_render_options(options)
self.validate()
validate_required_attrs(options)
validate_all_faces_triangular_and_non_degenerate_if_flat(options)
validate_planarity_if_enabled(options)

if options.normal_mode == RENDER_NORMAL_FLAT:
    build_flat_rendering_data(...)
else:
    build_shared_rendering_data(...)
```

The public API stays small while internals stay split by output shape.

## Shared-output path

Used for normal modes `NONE`, `SOURCE`, and `SMOOTH`.

Output shape:

- `vertices.len = mesh.positions.len`
- `indices.len = fan-triangulated index count`
- `normals.len`:
  - `0` for `RENDER_NORMAL_NONE`
  - `mesh.positions.len` for `SOURCE` and `SMOOTH`
- `uvs.len`:
  - `0` for `RENDER_UV_NONE`
  - `mesh.positions.len` for `SOURCE`, `SOURCE_OR_ZERO`, and `PLANAR_PROJECT`

Behavior:

- Copy positions into `data.vertices`.
- Emit fan-triangulated indices as M3 does today.
- For source normals and source UVs, copy attribute arrays.
- For smooth normals, generate angle-weighted vertex normals.
- For `SOURCE_OR_ZERO`, zero-fill UVs when missing.
- For planar-projected UVs, generate UVs by mesh-level dominant-axis projection.

## Flat-output path

Used only for `RENDER_NORMAL_FLAT`.

Precondition:

- Every face must have degree exactly 3.
- If any face is polygonal, return `NON_TRIANGLE_FACE`.

Output shape:

- `vertices.len = triangle_count * 3`
- `indices.len = vertices.len`
- `indices[i] = i`
- `normals.len = vertices.len`
- `uvs.len`:
  - `0` for `RENDER_UV_NONE`
  - `vertices.len` for `SOURCE`, `SOURCE_OR_ZERO`, and `PLANAR_PROJECT`

Behavior:

- Duplicate triangle vertices into `data.vertices`.
- Generate a normalized face normal and duplicate it for the three emitted vertices.
- Expand UVs to match expanded vertices when UVs are enabled.
- For source UVs, copy source UVs by original vertex index into expanded output slots.
- For `SOURCE_OR_ZERO` with missing source UVs, zero-fill the expanded UV slots.
- For planar-projected UVs, generate shared UVs first, then expand them by original vertex index into output slots.

## Geometry details

### Face normal

A face normal is computed from the first non-degenerate triangle in that face's fan. The implementation should scan for non-collinear edges instead of assuming the first three vertices define a usable plane.

Candidate triangles use the face fan root and two later vertices from the same face walk:

```text
face_vertices = collect_face_vertices(mesh, face)
root = face_vertices[0]
for i in 1 .. face_vertices.len - 2:
    a = mesh.positions[root]
    b = mesh.positions[face_vertices[i]]
    c = mesh.positions[face_vertices[i + 1]]
    n = cross(b - a, c - a)
    if length_squared(n) > PLANAR_EPSILON * PLANAR_EPSILON:
        return normalize(n)
return DEGENERATE_INPUT
```

The normal orientation follows the face half-edge winding. Do not flip normals to face a camera or to match any world-up convention.

If no non-degenerate triangle can be found, return `DEGENERATE_INPUT`.

### Planarity check

For each polygon face:

1. Collect face vertices into scratch storage.
2. Derive a plane from the first non-degenerate triangle in the face fan.
3. For every vertex in the face, compute absolute point-to-plane distance.
4. If distance exceeds `PLANAR_EPSILON`, return `NON_PLANAR_FACE`.
5. If no plane can be derived, return `DEGENERATE_INPUT`.

Scratch storage should be temporary and caller-owned by the validation helper, not stored on `HalfEdgeMesh` or `RenderingData`. A simple implementation can allocate one temporary `VertexIndex[]` sized to the maximum face degree in the extraction scope and reuse it per face.

Use the normalized face normal from the plane derivation. Distance is:

```text
distance = abs(dot(position - plane_origin, plane_normal))
```

Triangle faces do not need a planarity check.

### Smooth normals

Smooth normals are angle-weighted vertex normals over the same fan triangulation used for rendering indices.

Algorithm:

1. Allocate a `Vec3f[]` accumulator with length `mesh.positions.len`.
2. Initialize accumulators to zero.
3. Walk every face in winding order and emit the same triangles as the index builder:
   - triangle 0 for a face uses vertices `(v0, v1, v2)`,
   - triangle 1 uses `(v0, v2, v3)`,
   - triangle `i` uses `(v0, v(i + 1), v(i + 2))`.
4. For each fan triangle:
   - read `p0`, `p1`, `p2` from `mesh.positions`,
   - compute `raw_normal = cross(p1 - p0, p2 - p0)`,
   - skip the triangle if `length_squared(raw_normal) <= PLANAR_EPSILON * PLANAR_EPSILON`,
   - compute `face_normal = normalize(raw_normal)`.
5. For each triangle corner, compute the angle at that corner and accumulate `face_normal * angle` into that corner's original vertex index.
6. Normalize every non-zero accumulator into `data.normals`.
7. Leave zero accumulators as `{0, 0, 0}` rather than faulting.

Corner-angle computation:

```text
angle_at(a, b, c):
    u = normalize(b - a)
    v = normalize(c - a)
    cosine = clamp(dot(u, v), -1, 1)
    return acos(cosine)
```

If either corner edge has near-zero length, use angle `0` for that corner. This avoids a secondary fault after mesh validation and keeps isolated degenerate triangles from poisoning the whole normal field.

For polygons, smooth normals intentionally use fan-triangulated corners rather than exact polygon interior angles. That keeps normals consistent with the triangles actually emitted to the renderer. For triangles, this is exact. For planar convex polygons, it is a stable rendering approximation. For concave polygons, behavior follows the existing fan triangulation contract.

Accumulator normalization:

```text
if length_squared(accum[v]) > PLANAR_EPSILON * PLANAR_EPSILON:
    normal[v] = normalize(accum[v])
else:
    normal[v] = {0, 0, 0}
```

This produces higher-quality smooth shading than unweighted normals while remaining deterministic and local to the mesh data. It also avoids area weighting, which can bias vertex normals toward large triangles in triangulated polygon fans.

### Flat normals

Flat normals are generated only on the expanded-output path.

Algorithm:

1. Validate every face has degree exactly 3 before allocation.
2. Validate every triangular face can produce a non-zero face normal before allocation.
3. For each triangular face, read its three vertex indices in half-edge winding order.
4. Compute `raw_normal = cross(p1 - p0, p2 - p0)`.
5. Normalize the face normal.
6. Emit three duplicated positions, three sequential indices, and the same normal in the three matching normal slots.
7. If UVs are enabled, emit three matching UV slots using the selected UV mode.

Flat normals must not average across adjacent faces. Shared mesh vertices become separate output vertices, so hard edges are preserved by output shape rather than by a smoothing-group system.

### Planar-projected UVs

Planar-projected UV generation produces one shared UV per mesh vertex first. The flat-output path expands those shared UVs into duplicated output slots by original vertex index.

Projection-axis selection:

1. Initialize `normal_sum = {0, 0, 0}`.
2. For each face:
   - compute the face normal using the face-normal algorithm above,
   - add the normal to `normal_sum`.
3. If `length_squared(normal_sum) <= PLANAR_EPSILON * PLANAR_EPSILON`, fall back to `{0, 0, 1}` so projection drops Z and uses XY.
4. Let `abs_normal = abs(normal_sum)`.
5. Drop the largest absolute component:
   - `abs_normal.z >= abs_normal.x && abs_normal.z >= abs_normal.y`: drop Z -> project to XY,
   - `abs_normal.y >= abs_normal.x && abs_normal.y > abs_normal.z`: drop Y -> project to XZ,
   - otherwise: drop X -> project to YZ.

The tie-break intentionally prefers XY when the mesh has no clear dominant direction. This keeps common 2D/ground-plane meshes predictable.

Projection and normalization:

1. For every position, map 3D to 2D using the chosen axis pair:
   - XY: `(u_raw, v_raw) = (x, y)`,
   - XZ: `(u_raw, v_raw) = (x, z)`,
   - YZ: `(u_raw, v_raw) = (y, z)`.
2. Compute `min_u`, `max_u`, `min_v`, and `max_v` over all mesh positions.
3. Compute `width = max_u - min_u` and `height = max_v - min_v`.
4. For each vertex:
   - if `abs(width) > PLANAR_EPSILON`, `u = (u_raw - min_u) / width`, else `u = 0`,
   - if `abs(height) > PLANAR_EPSILON`, `v = (v_raw - min_v) / height`, else `v = 0`.

Projected UVs are mesh-level, not per-face. They do not create seams, rotate islands, preserve material scale, or minimize distortion. They are a simple deterministic fallback for planar preview/rendering data.

For flat output, UVs must expand to match expanded vertices when UV mode is not `NONE`.

For flat output with `RENDER_UV_PLANAR_PROJECT`, generate shared projected UVs first, then expand them by original vertex index into output slots. This keeps the flat path consistent with the shared path and avoids duplicate projection logic.

## Internal organization

Keep `src/render/rendering_data.c3` focused. It may contain plain module-scope helper functions. Do not use `@private` for helper free functions on c3c 0.7.11.

Suggested helpers:

- `validate_render_options(options)`
- `validate_required_render_attrs(mesh, options)`
- `validate_polygon_planarity(mesh, scratch, epsilon)`
- `validate_all_faces_triangular_and_non_degenerate_if_flat(mesh, options)`
- `build_shared_rendering_data(mesh, alloc, options)`
- `build_flat_rendering_data(mesh, alloc, options)`
- `copy_or_generate_normals(...)`
- `copy_or_generate_uvs(...)`
- `face_normal(...)`
- `angle_at(...)`
- `project_planar_uvs(...)`

If the file becomes too large during implementation, split helpers into additional files under `src/render/`, such as `normals.c3` and `uvs.c3`, and update both `project.json` and `manifest.json` together.

## Tests

Update existing M3 tests for the new default behavior and add M3b coverage.

Required tests:

1. Default options produce positions and indices only:
   - normals.len == 0
   - uvs.len == 0
2. Source normals copy when present.
3. Source normals missing returns `MISSING_NORMALS`.
4. Smooth normals are unit length on a tetrahedron.
5. Smooth normals are angle-weighted on a non-uniform fixture.
6. Flat normals expand triangle mesh output:
   - vertices.len == indices.len
   - normals.len == vertices.len
   - indices are sequential.
7. Flat normals return `NON_TRIANGLE_FACE` on polygon mesh.
8. Source UVs copy when present.
9. Source UVs missing returns `MISSING_UVS`.
10. Source-or-zero UVs zero-fill when missing.
11. Planar-projected UVs produce normalized `[0, 1]` UVs on a tilted planar quad.
12. Non-planar polygon returns `NON_PLANAR_FACE`.
13. Degenerate polygon plane returns `DEGENERATE_INPUT`.
14. Invalid options return `INVALID_RENDER_OPTIONS`:
   - non-zero reserved bits, or
   - out-of-range mode values if implementation uses integer fields instead of enum fields.
15. Existing triangle and polygon index tests continue to pass with explicit options where attributes are expected.

## Verification

Before M3b is complete:

```bash
git diff --check
c3c build static-lib
c3c test
c3c build debug
c3c build release
```

## Out of scope

- GPU buffer/resource creation.
- Texture atlasing or seam-aware UV unwrapping.
- Spherical UV projection.
- Tangents/bitangents.
- Normal smoothing groups or crease angles.
- Exact-predicate planarity.
- Public mode-specific rendering methods.
