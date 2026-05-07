# M4 — Edge Flipping Design

**Date:** 2026-05-07
**Module:** `cg::half_edge`
**C3 version:** 0.7.11
**Depends on:** M1 half-edge construction, M2 walks/validation, M3/M3b rendering extraction

## Goal

Implement local edge flipping for triangular half-edge meshes.

M4 adds a cheap topology mutation used by later Delaunay and remeshing work. It flips one interior edge shared by two triangle faces. It rewires only the six half-edges in the two adjacent triangles, refreshes the four local vertex handles, preserves all array lengths, and leaves geometry/attribute arrays unchanged.

The public API is:

```c3
fn bool  HalfEdgeMesh.is_flip_ok(&self, HeIndex he);
fn void? HalfEdgeMesh.flip(&self, HeIndex he);
```

`is_flip_ok` is a bool convenience. `flip` repeats the same validation and returns a specific fault on failure. No mutation happens until all validation succeeds.

## Scope

Create:

| File | Module | Purpose |
|------|--------|---------|
| `src/half_edge/flip.c3` | `cg::half_edge` | Edge flip API and local helpers |
| `test/test_flip.c3` | `test` | M4 coverage |

Update:

| File | Change |
|------|--------|
| `project.json` | Add `src/half_edge/flip.c3` to `sources` |
| `manifest.json` | Add `src/half_edge/flip.c3` to `sources` |
| `src/faults.c3i` | Add `EDGE_ALREADY_EXISTS` |

Out of scope:

- Geometric legality tests such as Delaunay in-circle checks.
- Angle, length, or convexity-based flip criteria.
- Any mutation of `positions`, `normals`, or `uvs`.
- Any external index-buffer rewrite.
- Polygonal-face flipping.
- Boundary edge flipping.
- Rich diagnostic structs carrying failing indices.

## Chosen Approach

Use a direct local six-half-edge rewrite with local vertex-handle refresh.

Alternative approaches considered:

1. Local six-half-edge rewrite plus local vertex refresh — chosen.
   - Pros: fast, focused, matches architecture, suited for future repeated Delaunay flips.
   - Cons: requires a precise rewire table.
2. Rebuild the two local triangles from a mini triangle description.
   - Pros: higher-level mental model.
   - Cons: still must preserve external twin links and can obscure which slots are reused.
3. Flip locally, then globally scan/repair all vertex handles.
   - Pros: robust against local-handle mistakes.
   - Cons: unnecessary `O(V + E)` cost per flip.

## Public API

`src/half_edge/flip.c3` declares methods on `cg::HalfEdgeMesh` from `module cg::half_edge`:

```c3
module cg::half_edge;
import cg;

fn bool HalfEdgeMesh.is_flip_ok(&self, HeIndex he);
fn void? HalfEdgeMesh.flip(&self, HeIndex he);
```

`is_flip_ok`:

- returns `true` only when `flip(he)` would be allowed by M4 topology rules.
- returns `false` for invalid half-edge indices, boundary edges, non-triangle adjacent faces, degenerate output endpoints, or duplicate output edge conflicts.
- never mutates the mesh.
- does not expose the failure fault.

`flip`:

- returns normally after a successful local topology rewrite.
- returns a fault on invalid input or disallowed topology.
- must not mutate the mesh on failure.
- must run validation before any field assignment.

## Faults

Add to `src/faults.c3i`:

```c3
EDGE_ALREADY_EXISTS,
```

Insert it near `DUPLICATE_HALF_EDGE` because both faults describe edge identity conflicts. Keep the final fault in the block terminated with `;`.

Fault usage:

| Fault | Returned by `flip` when |
|-------|--------------------------|
| `INVALID_HALF_EDGE_REFERENCE` | `he` is negative or outside `self.half_edges` |
| `BOUNDARY_HALF_EDGE` | `self.half_edges[he].twin == INVALID_HE` |
| `NON_TRIANGLE_FACE` | either adjacent face has degree other than 3 |
| `DEGENERATE_INPUT` | the two opposite vertices are the same vertex |
| `EDGE_ALREADY_EXISTS` | the proposed new diagonal already exists outside the two half-edge slots being reused |

`is_flip_ok` converts any of these failures to `false`.

Do not reuse `DUPLICATE_HALF_EDGE` for mutation-time new-edge conflicts. `DUPLICATE_HALF_EDGE` remains construction-input terminology. `EDGE_ALREADY_EXISTS` describes the attempted topology mutation more plainly.

## Validation Order

`flip(he)` validates in this exact order for deterministic tests:

1. Validate `he` is a valid half-edge index.
   - If not, return `cg::INVALID_HALF_EDGE_REFERENCE~`.
2. Read `t0 = self.half_edges[he].twin`.
   - If `t0 == cg::INVALID_HE`, return `cg::BOUNDARY_HALF_EDGE~`.
3. Validate the source face and twin face are triangles.
   - Let `f0 = self.half_edges[he].face`.
   - Let `f1 = self.half_edges[t0].face`.
   - If `self.face_degree(f0) != 3`, return `cg::NON_TRIANGLE_FACE~`.
   - If `self.face_degree(f1) != 3`, return `cg::NON_TRIANGLE_FACE~`.
4. Collect the six local half-edges and four local vertices.
   - M4 assumes callers pass a mesh that satisfies `mesh.validate()` before the call.
   - If the mesh is already corrupted in a way that violates the assumed triangle cycles, `flip` may return `cg::INVALID_TOPOLOGY~`, `cg::INVALID_FACE_CYCLE~`, or an existing reference fault as appropriate.
   - Tests for M4 failure atomicity should use otherwise-valid meshes. M4 is not a general repair operation.
5. Validate the opposite vertices are distinct.
   - If `c == d`, return `cg::DEGENERATE_INPUT~`.
6. Validate the new edge does not already exist.
   - Search for an existing directed edge `c -> d` or `d -> c`, excluding `h0` and `t0` because those slots will be reused for the new diagonal.
   - If found, return `cg::EDGE_ALREADY_EXISTS~`.
7. Mutate the six local half-edges and two face handles.
8. Refresh `vertices[v].half_edge` for the four local vertices.

No field assignment may occur before step 7.

## Local Topology Names

For a requested flip of `h0 = he`:

```text
h0 = he
h1 = h0.next
h2 = h1.next

t0 = h0.twin
t1 = t0.next
t2 = t1.next
```

Before the flip:

```text
face f0: h0(a -> b) -> h1(b -> c) -> h2(c -> a)
face f1: t0(b -> a) -> t1(a -> d) -> t2(d -> b)
```

Vertex derivation:

```text
a = h0.origin
b = t0.origin
c = h2.origin
d = t2.origin
```

The old diagonal is `a <-> b`. The new diagonal is `c <-> d`.

## Rewire Table

After a successful flip:

```text
h0: c -> d
t0: d -> c

face f0: h0(c -> d) -> t2(d -> b) -> h1(b -> c)
face f1: t0(d -> c) -> h2(c -> a) -> t1(a -> d)
```

Concrete field updates:

```text
h0.origin = c
t0.origin = d

h0.next = t2
t2.next = h1
h1.next = h0

t0.next = h2
h2.next = t1
t1.next = t0

h0.face = f0
t2.face = f0
h1.face = f0

t0.face = f1
h2.face = f1
t1.face = f1

faces[f0].half_edge = h0
faces[f1].half_edge = t0
```

Twin links:

- `h0.twin` remains `t0`.
- `t0.twin` remains `h0`.
- `h1.twin`, `h2.twin`, `t1.twin`, and `t2.twin` remain unchanged.
- External adjacent faces are not touched.

Array lengths unchanged:

- `self.half_edges.len`
- `self.faces.len`
- `self.vertices.len`
- `self.positions.len`
- `self.normals.len`
- `self.uvs.len`

## Vertex Handle Refresh

After rewiring, refresh only the four local vertices:

```text
a, b, c, d
```

Policy:

- Set each `vertices[v].half_edge` to the first half-edge in the ordered local list `[h0, h1, h2, t0, t1, t2]` whose current `origin == v`.
- If no local half-edge originates at `v`, keep the existing handle only if it still originates at `v`.
- If the existing handle is invalid or points to another origin, search the mesh for the first half-edge with `origin == v` and assign it.
- For a valid mesh produced by M1 construction and a valid M4 flip, the six local half-edges should cover the four vertices, so the whole-mesh fallback should not normally run. It is allowed as a defensive repair for local handles only.

Postcondition:

```c3
self.vertices[v].half_edge == cg::INVALID_HE
    || self.half_edges[self.vertices[v].half_edge].origin == v
```

for each affected local vertex.

## Duplicate New-Edge Check

Add a module-scope helper conceptually shaped as:

```c3
fn bool edge_exists_excluding(
    HalfEdgeMesh* mesh,
    VertexIndex from,
    VertexIndex to,
    HeIndex excluded_a,
    HeIndex excluded_b,
);
```

Behavior:

- Returns `true` if any half-edge except `excluded_a` or `excluded_b` has `origin == from` and `to_vertex(he) == to`.
- The implementation may scan all half-edges for simplicity and reliability in M4.
- A future optimization can replace the scan with a one-ring walk if needed.

`flip` should reject if either directed version exists:

```text
edge_exists_excluding(c, d, h0, t0)
edge_exists_excluding(d, c, h0, t0)
```

Checking both directions catches either orientation of the undirected edge.

## Atomicity

`flip` must be failure-atomic for all faults it detects before mutation.

Required behavior:

- invalid half-edge: no mutation.
- boundary half-edge: no mutation.
- non-triangle adjacent face: no mutation.
- degenerate opposite vertices: no mutation.
- existing new edge: no mutation.

Testing should snapshot enough topology before fault calls to verify failure does not mutate the mesh:

- selected half-edge origins,
- selected `next` values,
- selected `face` values,
- face handles,
- array lengths.

M4 does not promise to recover from an already-corrupted mesh passed into `flip`. Tests for failure atomicity should use otherwise valid meshes.

## Flip-Back Semantics

A second flip on the current diagonal restores the original undirected diagonal and a valid pair of triangle faces.

Do not require byte-for-byte restoration of half-edge slot orientation after two flips. Depending on which directed half-edge is passed to the second flip, `h0` and `t0` may hold the restored diagonal in opposite directed slots from the original snapshot. Tests should assert topology-level equivalence:

- the original undirected diagonal exists again,
- face cycles contain the original triangle vertex sets,
- `mesh.validate()` succeeds,
- counts and attributes remain unchanged.

If a future algorithm needs stable directed slot identity after flip-back, that should be a separate design decision.

## Geometry and Rendering Interaction

M4 is topology-only.

`positions`, `normals`, and `uvs` remain parallel arrays indexed by `VertexIndex`. Since a flip only changes which vertices form adjacent faces, no attribute data is copied, swapped, regenerated, or destroyed.

Rendering data is regenerated from current topology by `to_rendering_data`. M4 does not update previously returned `RenderingData` snapshots.

## Test Plan

Add `test/test_flip.c3` with `module test;`, `import cg;`, and `import cg::half_edge;`.

Reuse same-module fixtures when available:

- `TET_POSITIONS`, `TET_INDICES` from `test_builder.c3`.
- `DIAMOND_POSITIONS`, `DIAMOND_INDICES` from `test_topology.c3`.
- Add new local fixtures only when names do not collide with existing `module test` symbols.

Tests:

### `test_flip_ok_on_interior_diamond`

- Build diamond from `DIAMOND_POSITIONS` and `DIAMOND_INDICES`.
- Existing shared edge in that fixture is `1` with twin `3`.
- Assert `mesh.is_flip_ok(1)` is true.

### `test_flip_rewrites_diamond_diagonal`

- Build diamond.
- Flip half-edge `1`.
- Assert `mesh.validate()` succeeds.
- Assert half-edge `1` and its twin now connect the opposite vertex pair of the diamond.
- Assert both face degrees are 3.
- Assert the old shared diagonal is not the active diagonal in slots `1` and `3`.

### `test_flip_back_restores_original_topology_equivalence`

- Build diamond.
- Snapshot original face vertex sets, not exact half-edge slot order.
- Flip shared edge.
- Flip the current diagonal again.
- Assert original undirected diagonal exists again.
- Assert the two face vertex sets match the original two triangle sets, order-insensitively.
- Assert `mesh.validate()` succeeds.

### `test_flip_boundary_faults`

- Build a single triangle or diamond.
- Choose a boundary half-edge.
- Assert `mesh.is_flip_ok(boundary_he)` is false.
- Call `flip(boundary_he)` and assert fault `cg::BOUNDARY_HALF_EDGE`.
- Assert key topology snapshot is unchanged.

### `test_flip_rejects_invalid_half_edge`

- Build diamond.
- Assert `mesh.is_flip_ok(cg::INVALID_HE)` is false.
- Call `flip(cg::INVALID_HE)`.
- Assert fault `cg::INVALID_HALF_EDGE_REFERENCE`.
- Assert `mesh.is_flip_ok((HeIndex)(int)mesh.half_edges.len)` is false.
- Call `flip((HeIndex)(int)mesh.half_edges.len)`.
- Assert fault `cg::INVALID_HALF_EDGE_REFERENCE`.

### `test_flip_rejects_degenerate_output_endpoints`

- Manually construct a reference-valid local two-triangle topology where the two opposite vertices of the requested flip are the same vertex.
- Prefer a fixture that passes `mesh.validate()`; if that proves impossible without violating existing builder assumptions, confirm the relevant arrays and references are valid enough for `flip` to reach the opposite-vertex check.
- Assert `is_flip_ok(he)` is false.
- Assert `flip(he)` faults with `cg::DEGENERATE_INPUT`.
- Assert no mutation.

Keep this fixture narrowly scoped to the `c == d` guard and snapshot fields before the call.

### `test_flip_rejects_non_triangle_face`

- Build or manually construct a valid mesh with a quad face adjacent to a triangle across one edge.
- Assert `is_flip_ok(shared_he)` is false.
- Assert `flip(shared_he)` faults with `cg::NON_TRIANGLE_FACE`.
- Assert no mutation.

If builder support makes the adjacent quad fixture awkward because of directed-edge duplicate rules, construct the mesh arrays directly in the test with valid half-edge/twin/face references and confirm `mesh.validate()` succeeds before calling `flip`.

### `test_flip_rejects_duplicate_new_edge`

- Construct a valid triangular mesh where the opposite vertices `c` and `d` already have an edge outside the two faces being flipped.
- Assert `is_flip_ok(he)` is false.
- Assert `flip(he)` faults with `cg::EDGE_ALREADY_EXISTS`.
- Assert `mesh.validate()` still succeeds afterward.

A suitable fixture can be a triangular bipyramid-like mesh or a manually constructed local topology. The test must avoid invalid duplicate directed half-edges at construction time.

### `test_flip_preserves_counts_and_attributes`

- Build a diamond or tetrahedron with source normals and uvs using `from_triangles_with_attrs` if needed.
- Save lengths of all owned arrays.
- Save representative `positions`, `normals`, and `uvs` values.
- Flip a valid interior edge.
- Assert all lengths and representative attribute values are unchanged.
- Assert `mesh.validate()` succeeds.

### `test_flip_refreshes_local_vertex_handles`

- Build diamond.
- Flip shared edge.
- For the four local vertices, assert `vertices[v].half_edge != INVALID_HE`.
- Assert `half_edges[vertices[v].half_edge].origin == v`.
- Assert `mesh.validate()` succeeds.

### `test_flip_preserves_euler_counts`

- Build diamond or tetrahedron.
- Save `V = vertices.len`, `E2 = half_edges.len`, `F = faces.len`.
- Flip a valid interior edge.
- Assert the three counts are unchanged.
- Assert `mesh.validate()` succeeds.

## Implementation Notes

- Do not put function bodies in `.c3i` files.
- Do not use `@private` on module-level helper functions; keep helpers as plain module-scope functions in `src/half_edge/flip.c3`.
- Do not use `assert`, `unreachable`, or panic-style checks in production code.
- Use returned faults in production and `assert` only in tests.
- Use C3 pointer member access with `.` only; no `->`.
- No allocations are required for the core flip path.
- If a test directly constructs a mesh with heap arrays, use `mem::alloc::new_array(mem, T, n)` and clean up with `mesh.destroy()` or explicit `free` as appropriate.

## Verification

Before committing M4 implementation, use the currently defined project targets:

```sh
git diff --check
c3c build static-lib
c3c test
c3c build debug
c3c build release
```

Expected result:

- all builds pass.
- all tests pass.
- `mesh.validate()` succeeds after every successful flip test.
- failure-path tests prove no mutation for valid meshes rejected by M4 rules.

## Success Criteria

M4 is complete when:

- `src/half_edge/flip.c3` is listed in both `project.json` and `manifest.json`.
- `HalfEdgeMesh.is_flip_ok` returns true only for legal interior triangle-edge flips.
- `HalfEdgeMesh.flip` returns specific faults for illegal flips.
- successful flips rewire the two adjacent triangles according to the table above.
- successful flips preserve all array lengths and geometry/attribute data.
- local vertex handles are valid after flip.
- test suite covers success, flip-back topology equivalence, boundary rejection, invalid index rejection, degenerate output rejection, non-triangle rejection, duplicate-edge rejection, attribute preservation, and Euler count preservation.
- full verification commands pass.
