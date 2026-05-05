# M0 — Project Scaffolding

**Date:** 2026-05-04
**Module:** `cg` (package: `c3cg`)
**C3 version:** 0.7.11 (verified)

## Goal

Stand up the project skeleton: build system, type definitions, fault catalog, and data struct stubs. One empty passing test to confirm the test harness works. `c3c build static-lib && c3c test` green.

## Files Created / Modified

| File | Action | Module | What |
|------|--------|--------|------|
| `project.json` | Create | — | Static library target, sources, test-sources |
| `manifest.json` | Update | — | Set `"sources"` to list source files |
| `c3cg.c3i` | Replace | `cg` | Umbrella entry (empty body; same-module files auto-share scope) |
| `types.c3i` | Create | `cg` | Vector aliases, identifier typedefs (`HeIndex`, `FaceIndex`, `VertexIndex`), `INVALID_*` constants, `Aabb` |
| `faults.c3i` | Create | `cg` | 9 cross-cutting faults (§4.3 of architecture) |
| `half_edge_mesh.c3i` | Create | `cg` | `HalfEdge`, `HalfEdgeFace`, `HalfEdgeVertex`, `HalfEdgeMesh` structs (no methods) |
| `test/test_smoke.c3` | Create | `test` | One empty `@test` function |

All source files use `.c3i` (interface files, declarations only). Tests use `.c3` (can contain function bodies).

## `project.json`

```json
{
  "langrev": "1",
  "warnings": ["no-unused"],
  "sources": [
    "c3cg.c3i",
    "types.c3i",
    "faults.c3i",
    "half_edge_mesh.c3i"
  ],
  "test-sources": ["test/**"],
  "targets": {
    "static-lib": {
      "type": "static-lib"
    }
  }
}
```

Future milestones append additional files to `sources` as new submodule directories are added.

## `manifest.json`

Update: set `"sources"` to match the file list. The existing `"provides": "c3cg"` and `"targets"` remain unchanged.

```json
{
  "provides": "c3cg",
  "sources": [
    "c3cg.c3i",
    "types.c3i",
    "faults.c3i",
    "half_edge_mesh.c3i"
  ],
  "linklib-dir": "linked-libs",
  "targets": { ... }
}
```

## `c3cg.c3i` — Umbrella

```c3
module cg;
```

All `.c3i` files in this directory share the `module cg;` namespace. No imports needed between same-module files — declarations from `types.c3i`, `faults.c3i`, and `half_edge_mesh.c3i` are automatically visible.

## `types.c3i` — Vector Aliases & Identifier Typedefs

```c3
module cg;

typedef HeIndex     = inline int;
typedef FaceIndex   = inline int;
typedef VertexIndex = inline int;

alias Vec2f = float[<2>];
alias Vec3f = float[<3>];
alias Vec4f = float[<4>];
alias Vec2i = int[<2>];
alias Vec3i = int[<3>];

const HeIndex     INVALID_HE     = -1;
const FaceIndex   INVALID_FACE   = -1;
const VertexIndex INVALID_VERTEX = -1;

struct Aabb {
    Vec3f min;
    Vec3f max;
}
```

## `faults.c3i` — Cross-Cutting Faults

```c3
module cg;

faultdef
    INDEX_OUT_OF_RANGE,
    INVALID_TRIANGLE_INDEX_COUNT,
    NON_MANIFOLD_INPUT,
    DUPLICATE_HALF_EDGE,
    NON_TRIANGLE_FACE,
    BOUNDARY_HALF_EDGE,
    DEGENERATE_INPUT,
    EMPTY_INPUT,
    OPEN_CELL_ON_BOUNDARY;
```

## `half_edge_mesh.c3i` — Core Data Structures (Structs Only)

```c3
module cg;

struct HalfEdge {
    VertexIndex origin;
    HeIndex     next;
    HeIndex     twin;
    FaceIndex   face;
}

struct HalfEdgeFace {
    HeIndex half_edge;
}

struct HalfEdgeVertex {
    HeIndex half_edge;
}

struct HalfEdgeMesh {
    HalfEdge[]       half_edges;
    HalfEdgeFace[]   faces;
    HalfEdgeVertex[] vertices;
    Vec3f[]          positions;
    Vec3f[]          normals;
    Vec2f[]          uvs;
}
```

No methods yet. Construction, `destroy`, and topology queries arrive in M1.

## `test/test_smoke.c3` — Test Harness Verification

```c3
module test;
import cg;

fn void test_compiles() @test
{
}
```

## Verification

```bash
c3c build static-lib && c3c test
```

Expected output: `Static library 'out/static-lib.a' created.` + `PASSED: 1 passed, 0 failed, 0 skipped.`

## Notes

- Module name is `cg` (not `c3cg`). The package provides `c3cg`, but code uses `import cg;`.
- `.c3i` extension chosen for all library source files — these are interface files with declarations only. This is semantically correct for the struct/typedef/const/faultdef content and avoids accidentally putting function bodies in the wrong files later. Tests use `.c3`.
- `project.json` lists sources individually because C3's source glob resolution doesn't support bare `*.c3i` patterns. When submodule directories are added (M1+), they'll be added as additional entries.
- The umbrella `c3cg.c3i` body is empty — C3's same-module sharing means all declarations from sibling `.c3i` files are visible without explicit imports.
- `typedef = inline int` follows the stdlib pattern (used for `String`, `ZString`, etc.).
- Test module is `module test;` (not `module cg::test;` as the architecture doc suggests). This follows the dessert.c3l convention where test modules are independent and `import` the library explicitly.
- No `@public` annotations needed — C3 symbols are visible to all modules by default.

## Open Questions (Deferred Past M0)

These are from architecture doc §8. Some are resolved by M0 verification above; the rest are deferred until they become material:

| Question | Status |
|----------|--------|
| Vector type spelling (`float[<3>]` vs stdlib `Vec3f`) | ✅ Resolved — `float[<2>]` vectors used, local aliases defined |
| HashMap instantiation syntax | Deferred — needed in M1 (twin-pairing) |
| Allocator-passing convention | Deferred — needed in M1 (construction) |
| `*self = {}` zero-clear syntax | Deferred — needed in M1 (`destroy`) |
| Methods on imported types (can `cg::half_edge` add methods to `cg::HalfEdgeMesh`?) | Deferred — needed in M1 |
| `project.json` shape for library (vs application) | ✅ Resolved — `static-lib` target |
| Submodule visibility | Deferred — needed in M1+ |
