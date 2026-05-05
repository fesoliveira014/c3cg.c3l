# M0 — Project Scaffolding

**Date:** 2026-05-04
**Module:** `cg` (package: `c3cg`)
**C3 version:** 0.7.11 (pinned in `.claude/c3-skill.json`)

## Goal

Stand up the project skeleton: build system, type definitions, fault catalog, and data struct stubs. One empty passing test to confirm the test harness works. `c3c build static-lib && c3c test` green.

## Files Created / Modified

| File | Action | Module | What |
|------|--------|--------|------|
| `project.json` | Create | — | Static library + debug + release targets, sources, test-sources |
| `manifest.json` | Update | — | Set `"sources"` to list source files |
| `c3cg.c3i` | Replace | `cg` | Umbrella entry (empty body; same-module files auto-share scope) |
| `types.c3i` | Create | `cg` | Vector aliases, identifier typedefs (`HeIndex`, `FaceIndex`, `VertexIndex`), `INVALID_*` constants, `Aabb` |
| `faults.c3i` | Create | `cg` | 9 cross-cutting faults (§4.3 of architecture) |
| `half_edge_mesh.c3i` | Create | `cg` | `HalfEdge`, `HalfEdgeFace`, `HalfEdgeVertex`, `HalfEdgeMesh` structs (no methods) |
| `smoke/smoke.c3` | Create | `cg::smoke` | One method on `HalfEdgeMesh` to verify cross-module method declarations work |
| `test/test_smoke.c3` | Create | `test` | One empty `@test` function |
| `test/test_cross_module.c3` | Create | `test` | Verify method-on-imported-type pattern compiles and runs |
| `.claude/c3-skill.json` | Update | — | Pin version to `"0.7.11"` |

All source files use `.c3i` (interface files, declarations only). Files with function bodies (`smoke/smoke.c3`, test files) use `.c3`. Later milestones with methods (M1+) will also use `.c3`.

## `project.json`

```json
{
  "langrev": "1",
  "warnings": ["no-unused"],
  "sources": [
    "c3cg.c3i",
    "types.c3i",
    "faults.c3i",
    "half_edge_mesh.c3i",
    "smoke/smoke.c3"
  ],
  "test-sources": ["test/**"],
  "targets": {
    "static-lib": {
      "type": "static-lib"
    },
    "debug": {
      "type": "static-lib",
      "opt": "O0"
    },
    "release": {
      "type": "static-lib",
      "opt": "O3"
    }
  }
}
```

Future milestones append additional files to `sources` as new submodule directories are added.

Three targets reflect the architecture doc's requirement for debug + release builds:
- `static-lib` — default (O0, no `opt` field = compiler default)
- `debug` — explicit O0 debug build
- `release` — O3 optimized build

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

## `smoke/smoke.c3` — Cross-Module Method Verification

```c3
module cg::smoke;
import cg;

fn int HalfEdgeMesh.dummy(&self) {
    return 42;
}
```

This file exists only to verify that methods can be declared on a type (`HalfEdgeMesh`) from a different module (`cg::smoke`) than the one that defines the type (`cg`). The entire architecture depends on this working — M1+ submodules (`cg::half_edge`, `cg::geometry`, etc.) declare methods on `cg::HalfEdgeMesh` this way. The test below confirms it compiles and runs.

The `smoke/` directory will be removed after verification if desired. The `smoke.c3i` file and its `cg::smoke` module are not part of the library's public API.

## `test/test_smoke.c3` — Test Harness Verification

```c3
module test;
import cg;

fn void test_compiles() @test
{
}
```

## `test/test_cross_module.c3` — Cross-Module Method Verification

```c3
module test;
import cg;
import cg::smoke;

fn void test_cross_module_method() @test
{
    HalfEdgeMesh mesh = {};
    assert(mesh.dummy() == 42);
}
```

## Verification

```bash
c3c build static-lib && c3c test
```

Expected output: `Static library 'out/static-lib.a' created.` + `PASSED: 2 passed, 0 failed, 0 skipped.`

## Notes

- Module name is `cg` (not `c3cg`). The package provides `c3cg`, but code uses `import cg;`.
- **File layout deviation from architecture doc:** Source files live at the project root (not in `src/`). This follows the `.c3l` library convention where the `.c3i` entry point sits at the root and sibling files share the same module namespace. Submodule directories (e.g., `smoke/`, later `half_edge/`, `geometry/`) nest under root. The architecture doc's `cg/src/` layout assumed a standalone project; for a `.c3l` library the root IS the source root.
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
| Test annotations (`@test`) and assertion semantics | ✅ Resolved — `@test` after params, `assert()` in test code per style §11 |
| HashMap instantiation syntax | Deferred — needed in M1 (twin-pairing) |
| Allocator-passing convention | Deferred — needed in M1 (construction) |
| `*self = {}` zero-clear syntax | Deferred — needed in M1 (`destroy`) |
| Methods on imported types (can `cg::half_edge` add methods to `cg::HalfEdgeMesh`?) | ✅ Resolved — verified with `smoke/smoke.c3i` + test |
| `project.json` shape for library (vs application) | ✅ Resolved — `static-lib` + `debug` + `release` targets |
| Submodule visibility | Deferred — needed in M1+ |
