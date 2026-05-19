# M6 — Dual Operation Design

## Overview

The dual operation is the central bridge between triangulation and Voronoi representations. Given a closed `HalfEdgeMesh` and per-face vertex positions, it produces a new mesh where:

- Each source **vertex** → dual **face** (one face per source vertex)
- Each source **face** → dual **vertex** (at the corresponding `dual_vertex_positions[f]`)
- Each interior source **half-edge** → dual **half-edge** (same count)

The operation is its own inverse (modulo positions): `dual(dual(M, A), B)` is topologically equivalent to `M`.

## Module

`module cg::dual;` — single file `src/dual/dual.c3`.

Imports: `cg`, `cg::half_edge`, `cg::geometry`.

## Public API

### `dual_from_vertices` — raw path

```c3
fn HalfEdgeMesh? dual_from_vertices(
    Allocator alloc,
    HalfEdgeMesh* mesh,
    Vec3f[] dual_vertex_positions,
);
```

The caller supplies one position per source face (`dual_vertex_positions.len == mesh.faces.len`). The source mesh must be closed (no boundary half-edges).

### `dual` — convenience wrapper

```c3
fn HalfEdgeMesh? dual(
    Allocator alloc,
    HalfEdgeMesh* mesh,
    DualMode mode = DualMode.CIRCUMCENTER,
    float radius = 1.0f,
);
```

Selects a position-generation strategy via `DualMode`, computes positions using `cg::geometry::*`, then delegates to `dual_from_vertices`.

### `DualMode` enum

```c3
enum DualMode : uint
{
    CIRCUMCENTER,             // cg::geometry::circumcenters_planar(alloc, mesh)
    CENTROID,                 // cg::geometry::face_centroids(alloc, mesh)
    SPHERICAL_CIRCUMCENTER,   // cg::geometry::circumcenters_on_sphere(alloc, mesh, radius)
}
```

`radius` is only consumed by `SPHERICAL_CIRCUMCENTER`. Default: `1.0f` (unit sphere).

## Faults

Appended to the `faultdef` block in `src/faults.c3i` (before the closing `;`):

```c3
    DUAL_REQUIRES_CLOSED_MESH,   // source mesh has ≥1 boundary half-edge
    DUAL_VERTEX_COUNT_MISMATCH;  // dual_vertex_positions.len != source.faces.len
```

### Fault propagation

`dual_from_vertices` fires:

- `INVALID_TOPOLOGY`, `INVALID_TWIN`, `INVALID_FACE_CYCLE`, `ATTRIBUTE_COUNT_MISMATCH` — from `source.validate()`
- `DUAL_REQUIRES_CLOSED_MESH` — boundary scan
- `DUAL_VERTEX_COUNT_MISMATCH` — position array length mismatch
- `OUTPUT_BUFFER_TOO_SMALL` — caught internally (scratch buffer grown), never surfaced
- `EMPTY_INPUT`, `INDEX_OUT_OF_RANGE`, `DEGENERATE_INPUT`, `DUPLICATE_HALF_EDGE` — from `from_polygons`

`dual` fires all of the above plus:

- `NON_TRIANGLE_FACE` — from `circumcenters_planar` / `circumcenters_on_sphere` (if mesh has non-triangle faces)
- `INVALID_FACE_REFERENCE`, `INVALID_FACE_CYCLE` — from `face_centroids`
- `DEGENERATE_INPUT` — from geometry helpers (near-zero cross product, radius ≤ epsilon)

## Algorithm (`dual_from_vertices`)

Two-pass approach, building CSR data for `from_polygons`.

### Step 1 — Validate

```
source.validate()!
```

### Step 2 — Boundary check

```
for each half-edge:
    if twin == INVALID_HE → DUAL_REQUIRES_CLOSED_MESH
```

### Step 3 — Position count check

```
if dual_vertex_positions.len != source.faces.len → DUAL_VERTEX_COUNT_MISMATCH
```

### Step 4 — Pass 1: collect ring sizes

```c3
face_offsets = mem::alloc::new_array(alloc, uint, (sz) (source.vertices.len + 1));
defer catch free(face_offsets);

scratch = mem::alloc::new_array(alloc, FaceIndex, (sz) 3);  // start at capacity 3; grows if needed
defer free(scratch);
```

```
face_offsets[0] = 0
for each source vertex v:
    n = source.vertex_one_ring_faces(v, scratch)!
    if n > scratch capacity → realloc scratch to size n, retry the walk
    face_offsets[v + 1] = face_offsets[v] + n
```

The re-walk after growing scratch is deterministic: `vertex_one_ring_faces` always starts from the same canonical half-edge, so it returns the same result on retry.

### Step 5 — Allocate face indices

```c3
face_indices = mem::alloc::new_array(alloc, uint, (sz) face_offsets[last]);
defer catch free(face_indices);
```

### Step 6 — Pass 2: fill face indices

```
for each source vertex v:
    n = source.vertex_one_ring_faces(v, scratch)!
    for i in 0..n:
        face_indices[face_offsets[v] + i] = (uint)scratch[i]
```

`FaceIndex` values are explicitly cast to `uint` for `from_polygons`.

### Step 7 — Construct mesh

```
result = from_polygons(alloc, dual_vertex_positions, face_offsets, face_indices)!
```

### Step 8 — Cleanup and return

```c3
free(face_offsets);
free(face_indices);
return result;
```

`scratch` is freed by its `defer free()` at end of scope. The `defer catch free()` on `face_offsets` and `face_indices` cleans up if `from_polygons` fails before the explicit `free()` calls.

`face_offsets` and `face_indices` are NOT part of the returned mesh — `from_polygons` reads them as CSR input and allocates its own internal arrays. They are intermediate temporaries that must be freed after construction succeeds.

### Memory ownership

| Allocation                | Lifecycle                                                                             |
| ------------------------- | ------------------------------------------------------------------------------------- |
| `face_offsets`            | Allocated for CSR construction; `defer catch free()`, explicitly `free()` after `from_polygons` returns |
| `face_indices`            | Allocated for CSR construction; `defer catch free()`, explicitly `free()` after `from_polygons` returns |
| `scratch`                 | Temp, `defer free()` at end of `dual_from_vertices` scope                             |
| `positions` (in `dual()`) | Temp from geometry helper, `defer free()` after `dual_from_vertices` returns          |

## `dual()` wrapper

```c3
fn HalfEdgeMesh? dual(Allocator alloc, HalfEdgeMesh* mesh, DualMode mode, float radius)
{
    Vec3f[] positions;
    switch (mode) {
        case DualMode.CIRCUMCENTER:
            positions = cg::geometry::circumcenters_planar(alloc, mesh)!;
        case DualMode.CENTROID:
            positions = cg::geometry::face_centroids(alloc, mesh)!;
        case DualMode.SPHERICAL_CIRCUMCENTER:
            positions = cg::geometry::circumcenters_on_sphere(alloc, mesh, radius)!;
    }
    defer free(positions);
    return dual_from_vertices(alloc, mesh, positions)!;
}
```

## Testing

File: `test/test_dual.c3` — `module cg::test;`

### Fixture A: Tetrahedron

4 vertices, 4 faces, 6 half-edges. Self-dual (dual of tetrahedron is topologically a tetrahedron). Built via `from_triangles`.

### Fixture B: Hand-built icosahedron

12 vertices on unit sphere: `(±1, ±φ, 0)` and cyclic permutations, where φ = (1+√5)/2 (~1.618).
20 triangular faces. Dual is a dodecahedron: 20 vertices, 12 faces (degree 5 each), 60 half-edges.

### Test cases

| #   | Test                                     | Fixture         | Assertion                                                                                   |
| --- | ---------------------------------------- | --------------- | ------------------------------------------------------------------------------------------- |
| 1   | `test_dual_tetrahedron_counts`           | A               | `dual(tetra).vertices.len == 4`, `faces.len == 4`, `half_edges.len == 6`                    |
| 2   | `test_dual_tetrahedron_closed`           | A               | No boundary edges in dual output                                                            |
| 3   | `test_dual_tetrahedron_validates`        | A               | `dual(tetra).validate()` succeeds                                                           |
| 4   | `test_dual_double_dual_roundtrip`        | A               | `dual(dual(tetra, A), B)` has same topology counts as `tetra`                               |
| 5   | `test_dual_icosahedron_to_dodecahedron`  | B               | 20 vertices, 12 faces, 60 half-edges; every face degree == 5                                |
| 6   | `test_dual_icosahedron_double_dual`      | B               | Round-trip: `dual(dual(ico, A), B)` has 12 vertices, 20 faces, 60 half-edges                |
| 7   | `test_dual_closed_mesh_required`         | Single triangle | `DUAL_REQUIRES_CLOSED_MESH`                                                                 |
| 8   | `test_dual_vertex_count_mismatch`        | A               | Wrong-length `dual_vertex_positions` → `DUAL_VERTEX_COUNT_MISMATCH`                         |
| 9   | `test_dual_mode_circumcenter`            | A               | `dual(tetra, CIRCUMCENTER)` valid, vertex positions ≠ centroid positions                    |
| 10  | `test_dual_mode_centroid`                | A               | `dual(tetra, CENTROID)` valid                                                               |
| 11  | `test_dual_mode_spherical`               | B               | `dual(ico, SPHERICAL_CIRCUMCENTER, 1.0)` → dodecahedron vertices on sphere within tolerance |
| 12  | `test_dual_spherical_radius_zero_faults` | A               | `dual(tetra, SPHERICAL_CIRCUMCENTER, 0.0)` → `DEGENERATE_INPUT`                             |
| 13  | `test_dual_from_vertices_raw`            | A               | `dual_from_vertices(tetra, custom_positions)` with hand-built positions works               |

### Dependencies for tests

- `from_triangles` — already tested in M1
- `face_degree` — already tested in M1
- `vertex_one_ring_faces` — already tested in M2
- `validate` — already tested in M2
- `circumcenters_planar`, `face_centroids`, `circumcenters_on_sphere` — already tested in M5

## Project.json changes

Add to `sources`:

```json
"src/dual/dual.c3"
```
