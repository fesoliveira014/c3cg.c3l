# M5 — Geometry Helpers Design

**Date:** 2026-05-08
**Module:** `cg::geometry`
**C3 version:** 0.7.11
**Depends on:** M1 half-edge construction, M2 walks/validation, M4 edge flipping only indirectly through shared topology conventions

## Goal

Add the `cg::geometry` module promised by the architecture: circumcenters, face centroids, and predicate classifiers. M5 provides the numerical building blocks needed by later dual, hull, Delaunay, Voronoi, graph, and spherical milestones.

This milestone is deliberately non-robust v1 geometry. It uses straightforward floating-point formulas plus epsilon classification. Exact/adaptive robust predicates are out of scope.

## Non-goals

- Do not implement Delaunay, Voronoi, hull, graph extraction, or dual construction.
- Do not expose raw determinant predicate values as public API.
- Do not introduce exact/adaptive robust predicates.
- Do not refactor render internals unless implementation discovers a compile/blocking need. `cg::geometry` must not depend on `cg::render`.
- Do not add broad vector-math modules beyond tiny helpers needed by M5.

## Files

Add:

- `src/geometry/circumcenter.c3`
- `src/geometry/centroid.c3`
- `src/geometry/predicates.c3`
- `test/test_geometry.c3`

Update:

- `project.json`
- `manifest.json`

The three geometry implementation files all use:

```c3
module cg::geometry;
import cg;
import cg::half_edge;
```

Add `import std::math;` only in files that need square root or absolute-value support from the standard library. Do not import `std::mem` just to use `mem`, `free`, or `mem::alloc::new_array`.

## Public API

All public geometry functions live in `module cg::geometry`.

Definition order within files follows project style: typedefs, aliases, constants, structs if needed, then functions.

```c3
typedef PredicateSign = int;

const float GEOMETRY_EPSILON = 0.00001f;
const float DEFAULT_PREDICATE_EPSILON = 0.00001f;
const PredicateSign PREDICATE_NEGATIVE = -1;
const PredicateSign PREDICATE_ZERO = 0;
const PredicateSign PREDICATE_POSITIVE = 1;

fn Vec3f? circumcenter_planar(Vec3f a, Vec3f b, Vec3f c);
fn Vec3f? circumcenter_on_sphere(Vec3f a, Vec3f b, Vec3f c, float radius);

fn Vec3f[]? circumcenters_planar(Allocator alloc, HalfEdgeMesh* tri_mesh);
fn Vec3f[]? circumcenters_on_sphere(Allocator alloc, HalfEdgeMesh* tri_mesh, float radius);

fn Vec3f? face_centroid(HalfEdgeMesh* mesh, FaceIndex face);
fn Vec3f[]? face_centroids(Allocator alloc, HalfEdgeMesh* mesh);

fn PredicateSign orient_2d(Vec2f a, Vec2f b, Vec2f c, float epsilon = DEFAULT_PREDICATE_EPSILON);
fn PredicateSign in_circle_2d(Vec2f a, Vec2f b, Vec2f c, Vec2f p, float epsilon = DEFAULT_PREDICATE_EPSILON);
fn PredicateSign on_sphere(Vec3f a, Vec3f b, Vec3f c, Vec3f d, Vec3f p, float epsilon = DEFAULT_PREDICATE_EPSILON);
```

### Predicate sign representation

C3 0.7.11 does not support C-style enum value assignment with negative values in the desired shape. Use a typed integer alias plus typed constants for `PredicateSign`. This preserves a named return type and named comparison constants without exposing raw determinant values.

## Ownership and allocation

Owned array functions take explicit `Allocator alloc`, matching project convention:

- `circumcenters_planar(alloc, tri_mesh)`
- `circumcenters_on_sphere(alloc, tri_mesh, radius)`
- `face_centroids(alloc, mesh)`

Returned arrays are caller-owned and must be freed by the caller with `free(array)`.

Implementation pattern:

```c3
Vec3f[] out = mem::alloc::new_array(alloc, Vec3f, mesh.faces.len);
defer catch free(out);
// fill out
return out;
```

Do not append `!` to `mem::alloc::new_array`; in this project on c3c 0.7.11 it returns the slice directly.

## Fault policy

Reuse existing faults:

- `INVALID_FACE_REFERENCE`
- `NON_TRIANGLE_FACE`
- `DEGENERATE_INPUT`
- `INVALID_FACE_CYCLE`
- `INVALID_TOPOLOGY`
- validation faults propagated from `mesh.validate()`
- `OUTPUT_BUFFER_TOO_SMALL` only if an internal walk helper unexpectedly produces it

No new M5 fault is expected.

Scalar circumcenter helpers are fallible. Degenerate geometry should fault instead of returning sentinel values or arbitrary zeros.

Predicate functions are not fallible. They classify determinant values using epsilon and return `PredicateSign`.

## Circumcenters

### `circumcenter_planar`

`circumcenter_planar(a, b, c)` computes the circumcenter of a triangle embedded in 3D space. It must not assume the triangle lies in the XY plane.

Recommended formula:

```text
u = b - a
v = c - a
n = cross(u, v)
denom = 2 * dot(n, n)
center = a + (cross(n, u) * dot(v, v) + cross(v, n) * dot(u, u)) / denom
```

Degeneracy:

- fault `DEGENERATE_INPUT` if `dot(n, n) <= GEOMETRY_EPSILON * GEOMETRY_EPSILON`
- covers duplicate points, collinear points, and near-zero area triangles
- this threshold intentionally catches true or near-true degeneracy; it is not a general thin-triangle quality filter
- implementation may define a module-scope squared-epsilon helper constant to avoid repeating `GEOMETRY_EPSILON * GEOMETRY_EPSILON`

Tests should verify:

- known right-triangle center `(1, 1, 0)` for `(0,0,0), (2,0,0), (0,2,0)`
- tilted-plane triangle center has equal distance to all three vertices and lies in the triangle plane
- collinear and duplicate input faults `DEGENERATE_INPUT`

### `circumcenter_on_sphere`

`circumcenter_on_sphere(a, b, c, radius)` computes the planar circumcenter, then projects that direction to the sphere centered at the origin:

```text
planar = circumcenter_planar(a, b, c)!
len = length(planar)
center = planar * (radius / len)
```

Degeneracy:

- fault `DEGENERATE_INPUT` if `radius <= GEOMETRY_EPSILON`
- fault `DEGENERATE_INPUT` if planar circumcenter calculation faults
- fault `DEGENERATE_INPUT` if `length(planar) <= GEOMETRY_EPSILON`

M5 does not validate that each input point is exactly on the sphere. Later spherical algorithms may add stricter caller-side validation if needed.

### Batch circumcenters

`circumcenters_planar(alloc, tri_mesh)`:

1. Validate `tri_mesh.validate()` before allocation.
2. Allocate `Vec3f[]` with length `tri_mesh.faces.len`.
3. For each face index `f`:
   - require `face_degree(f) == 3`, else `NON_TRIANGLE_FACE`
   - read the three vertex indices from the face cycle
   - call `circumcenter_planar`
   - write output at `out[f]`
4. Use `defer catch free(out)` for fault cleanup after allocation.

`circumcenters_on_sphere(alloc, tri_mesh, radius)`:

1. Validate `radius > GEOMETRY_EPSILON` before allocation.
2. Validate `tri_mesh.validate()` before allocation.
3. Follow the same face loop and ownership policy, calling `circumcenter_on_sphere`.

All batch functions preserve face-index mapping: output index equals source face index.

All batch functions propagate `mesh.validate()` faults before allocation for empty or otherwise invalid meshes; M5 does not define zero-face batch success.

## Centroids

### `face_centroid`

`face_centroid(mesh, face)` returns one centroid for a face.

Validation:

- fault `INVALID_FACE_REFERENCE` if `face < 0` or `face >= mesh.faces.len`
- public `face_centroid` should validate topology with `mesh.validate()` before walking
- the batch function validates once, then uses an internal helper to avoid repeated full-mesh validation

Centroid definition:

- triangle faces: exact triangle centroid `(a + b + c) / 3`
- polygon faces: vertex-average centroid, i.e. average all face vertex positions in face-cycle order

M5 intentionally does not use area-weighted polygon centroids. Vertex average is deterministic, simple, and enough for the graph-view centroids planned later. Area-weighted polygon centroid can be added later if a consumer requires it.

Implementation detail:

- Do not allocate scratch just to compute a centroid.
- Walk the face cycle directly from `mesh.faces[face].half_edge`.
- Guard malformed cycles with a step limit based on `mesh.half_edges.len` even though public entry points validate first; if a malformed cycle is detected, return an existing topology fault such as `INVALID_FACE_CYCLE` or `INVALID_TOPOLOGY`.

Use this internal helper shape:

```c3
fn Vec3f? centroid_for_valid_face(HalfEdgeMesh* mesh, FaceIndex face);
```

`centroid_for_valid_face` assumes the caller already checked the face index and validated mesh topology. Public `face_centroid` checks the face reference, calls `mesh.validate()`, then calls the helper. Batch `face_centroids` calls `mesh.validate()` once, then calls the helper for every face.

### `face_centroids`

`face_centroids(alloc, mesh)`:

1. Validate `mesh.validate()` before allocation.
2. Allocate `Vec3f[]` with length `mesh.faces.len`.
3. For each face index `f`, compute centroid and store at `out[f]`.
4. Use `defer catch free(out)` for fault cleanup after allocation.

Output index equals source face index.

## Predicates

Public predicates are classifier-first. They return `PredicateSign` and do not expose raw determinant magnitudes.

Shared classification rule:

```text
value > epsilon   => PREDICATE_POSITIVE
value < -epsilon  => PREDICATE_NEGATIVE
otherwise         => PREDICATE_ZERO
```

If a negative epsilon is passed, implementation should use its absolute value or clamp to `DEFAULT_PREDICATE_EPSILON`; choose one in the implementation plan and test it. Do not let negative epsilon invert classifier behavior.

### `orient_2d`

Use the standard signed 2D orientation determinant:

```text
(b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x)
```

Convention:

- positive: `a, b, c` are counter-clockwise
- negative: clockwise
- zero: collinear within epsilon

### `in_circle_2d`

Use the standard incircle determinant for points `a`, `b`, `c`, and query point `p`, preferably after translating coordinates by `p` to reduce arithmetic size:

```text
ax = a.x - p.x; ay = a.y - p.y; aa = ax*ax + ay*ay
bx = b.x - p.x; by = b.y - p.y; bb = bx*bx + by*by
cx = c.x - p.x; cy = c.y - p.y; cc = cx*cx + cy*cy
value = ax * (by * cc - bb * cy)
      - ay * (bx * cc - bb * cx)
      + aa * (bx * cy - by * cx)
```

Convention:

- for counter-clockwise `a,b,c`, positive means `p` is inside the circumcircle
- for clockwise `a,b,c`, sign is inverted
- zero means on the circle within epsilon

This orientation dependency must be documented in tests. Later Delaunay code should orient triangles consistently before interpreting incircle signs.

### `on_sphere`

Use the standard 3D insphere determinant for tetrahedron points `a`, `b`, `c`, `d` and query point `p`, preferably after translating by `p`. The public name remains `on_sphere` to match the architecture and common predicate naming, even though it classifies inside/outside/on the circumsphere.

Convention:

- returns the sign of the determinant after epsilon classification
- tetrahedron `a,b,c,d` is positively oriented when `dot(cross(b - a, c - a), d - a) > 0`
- for a positively oriented tetrahedron, positive means `p` is inside the circumsphere; negative means outside; zero means on the sphere within epsilon
- for a negatively oriented tetrahedron, inside/outside signs are inverted

Implementation may compute the 4x4 lifted determinant using helper `det3`/`det4` module-scope functions. These helpers are not public API and should not be documented as user-facing functions. If the chosen determinant arrangement naturally produces the opposite sign from the convention above, negate the value before classification.

For degenerate input, such as collinear `in_circle_2d` points or coplanar `on_sphere` points, the determinant should classify as `PREDICATE_ZERO`. Callers that need inside/outside semantics must avoid degenerate base geometry.

## Helper visibility

The user chose no public raw predicate helper API. Because C3 free-function visibility can be version-sensitive and `@private` is not reliable for module-level helper functions in this project, implementation should avoid adding documented public `*_value` functions.

Raw determinant calculations may be:

- inline inside public predicate functions, or
- placed in plain module-scope helpers used only inside `src/geometry/predicates.c3`

If helper functions are used, keep them undocumented and use names that make internal use clear. Do not add them to any `.c3i` interface file.

## Relationship to existing render math

`src/render/geometry.c3` already contains small vector helpers such as dot, cross, length, normalization, and planarity checks. M5 must not import `cg::render`.

For M5:

- duplicate tiny vector helpers in `cg::geometry` where needed, or
- define focused module-scope helpers in each geometry file

Do not move render helpers as part of M5 unless implementation uncovers a direct blocking issue. A shared math cleanup can be a later standalone refactor.

## Tests

Add `test/test_geometry.c3` with `module test; import cg; import cg::half_edge; import cg::geometry;`.

Use local helper assertions for approximate vector/float equality to avoid colliding with existing `module test` helpers. Prefix helpers with `m5_`.

### Circumcenter tests

- `test_circumcenter_planar_right_triangle`
  - `(0,0,0), (2,0,0), (0,2,0)` => `(1,1,0)`
- `test_circumcenter_planar_tilted_triangle_equal_distances`
  - assert center is equidistant from all three vertices
  - assert center lies in the triangle plane
- `test_circumcenter_planar_degenerate_faults`
  - collinear or duplicate points fault `DEGENERATE_INPUT`
- `test_circumcenter_on_sphere_unit_axes`
  - axis-aligned unit-sphere points produce a unit-length center in the expected direction
- `test_circumcenter_on_sphere_invalid_radius_faults`
  - zero/negative/small radius faults `DEGENERATE_INPUT`

### Batch circumcenter tests

- `test_circumcenters_planar_triangle_mesh`
  - use a small triangle mesh; output length equals face count; face-index mapping correct
- `test_circumcenters_planar_rejects_non_triangle_face`
  - polygon mesh faults `NON_TRIANGLE_FACE`
- `test_circumcenters_planar_rejects_degenerate_triangle`
  - degenerate triangle face faults `DEGENERATE_INPUT`
- `test_circumcenters_on_sphere_triangle_mesh`
  - output length equals face count; every output has radius length within tolerance

### Centroid tests

- `test_face_centroid_triangle_average`
  - triangle face centroid equals average of vertices
- `test_face_centroid_quad_vertex_average`
  - quad face centroid equals vertex average
- `test_face_centroid_invalid_face_faults`
  - invalid face index faults `INVALID_FACE_REFERENCE`
- `test_face_centroids_batch_preserves_face_index_mapping`
  - output length equals face count; values correspond to face indices
- `test_face_centroid_scalar_matches_batch`
  - for a small polygon mesh, `face_centroid(mesh, f)` matches `face_centroids(alloc, mesh)[f]` for every face

### Predicate tests

- `test_orient_2d_classifies_ccw_cw_collinear`
  - CCW => `PREDICATE_POSITIVE`
  - CW => `PREDICATE_NEGATIVE`
  - collinear/near-collinear => `PREDICATE_ZERO`
- `test_in_circle_2d_classifies_for_ccw_triangle`
  - inside => `PREDICATE_POSITIVE`
  - outside => `PREDICATE_NEGATIVE`
  - on circle => `PREDICATE_ZERO`
- `test_in_circle_2d_orientation_dependency`
  - same circle with reversed triangle orientation flips sign for inside/outside
- `test_on_sphere_classifies_known_tetrahedron`
  - inside/outside/on circumsphere cases for a known tetrahedron; document the orientation convention in assertions
- `test_on_sphere_degenerate_coplanar_classifies_zero`
  - coplanar tetrahedron base input classifies as `PREDICATE_ZERO`
- `test_predicates_negative_epsilon_does_not_invert_classification`
  - passing a negative epsilon still produces the same sign as the corresponding positive epsilon for representative orientation/incircle/sphere cases

## Verification

Before committing M5 implementation:

```sh
git diff --check
c3c build static-lib
c3c test
c3c build debug
c3c build release
```

Expected:

- all commands pass
- new geometry tests pass
- existing M0-M4 tests remain green

## Implementation notes for the future plan

- Start with `PredicateSign` typedef/constants and a focused compile spike to verify the typed constants and default-argument signatures on c3c 0.7.11.
- Keep each geometry file focused:
  - circumcenter formulas and batch circumcenters in `circumcenter.c3`
  - centroid walking and batch centroids in `centroid.c3`
  - classifier predicates in `predicates.c3`
- Use TDD: add each test/fault case before implementation.
- Use `defer catch free(out)` for returned arrays that may fault during fill.
- Avoid sentinel values and production `assert()`.
- Do not use `@private` for free-function helpers.
