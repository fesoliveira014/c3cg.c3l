# M5 Geometry Helpers Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add `cg::geometry` circumcenters, centroids, and classifier predicates with tests, explicit allocator-owned batch outputs, and no public raw determinant helpers.

**Architecture:** Implement M5 as three focused `cg::geometry` source files: predicates, circumcenters, and centroids. Keep tiny vector/determinant helpers module-local, avoid `cg::render` dependencies, validate meshes before batch allocation, and use fallible scalar circumcenter helpers for degeneracy.

**Tech Stack:** C3 0.7.11, c3c, c3cg.c3l flat-array `HalfEdgeMesh`, C3 optionals/faults, `mem::alloc::new_array`, `c3c test`.

---

## Required context

Spec:

- `docs/specs/2026-05-08-m5-geometry-helpers-design.md`

Repo/project constraints:

- Follow `AGENTS.md` and `docs/style.md`.
- Before writing C3, load `c3-expert`; before implementation, use `test-driven-development`.
- Definition order: typedefs -> aliases -> constants -> enums/structs -> functions.
- No production `assert()`, `unreachable()`, sentinel returns, `@private`, `->`, `sizeof(T)`, `goto`, or raw `malloc/free`.
- Tests may use `assert()` and `unreachable()`.
- Explicit allocator convention for owned arrays.
- Use `mem::alloc::new_array(alloc, T, len)` without `!`.
- Use `defer catch free(out)` for owned outputs that can fault while filling.
- `.c3i` files are declarations only. M5 does not need a new `.c3i` unless implementation discovers a hard compile need.

Current baseline:

- Main has M4 merged.
- Baseline verification at plan time: `c3c test` -> `93 passed, 0 failed, 0 skipped`.
- Pre-existing untracked files in main should remain untouched unless needed: `.claude/`, `.superpowers/`, `LICENSE`, `README.md`, `docs/architecture.md`, `docs/bindings_guidelines.md`, `docs/style.md`, `linked-libs/`, `scripts/`.

## File map

Create:

- `src/geometry/predicates.c3`
  - `module cg::geometry;`
  - `PredicateSign` typedef/constants.
  - `GEOMETRY_EPSILON` and `DEFAULT_PREDICATE_EPSILON`.
  - module-local helpers: `abs_float`, `classify_predicate`, `det3`, `det4`.
  - public predicates: `orient_2d`, `in_circle_2d`, `on_sphere`.

- `src/geometry/circumcenter.c3`
  - `module cg::geometry;`
  - imports `cg`, `cg::half_edge`, `std::math`.
  - module-local vector helpers: `vec3_add`, `vec3_sub`, `vec3_scale`, `vec3_dot`, `vec3_cross`, `vec3_length`.
  - module-local `collect_triangle_face_vertices`.
  - public scalar and batch circumcenter functions.

- `src/geometry/centroid.c3`
  - `module cg::geometry;`
  - imports `cg`, `cg::half_edge`.
  - module-local vector helpers: `vec3_add`, `vec3_scale`.
  - internal `centroid_for_valid_face`.
  - public scalar and batch centroid functions.

- `test/test_geometry.c3`
  - `module test;`
  - imports `cg`, `cg::half_edge`, `cg::geometry`.
  - local `m5_` assertion/helpers for approximate equality and expected faults.

Modify:

- `project.json`
  - Add new source files as they are introduced:
    - `src/geometry/predicates.c3`
    - `src/geometry/circumcenter.c3`
    - `src/geometry/centroid.c3`

- `manifest.json`
  - Add same new source files under `sources`.

Do not modify:

- `src/render/*` except if a compile blocker is found and explicitly reviewed.
- Existing M0-M4 behavior.

## C3 syntax/pattern notes to verify early

Known-good patterns from current repo:

```c3
fn float abs_float(float value)
{
    if (value < 0.0f) return -value;
    return value;
}

fn float length3(Vec3f v)
{
    return (float)math::sqrt((double)length_squared3(v));
}

Vec3f[] out = mem::alloc::new_array(alloc, Vec3f, mesh.faces.len);
defer catch free(out);
```

Known-good test fault pattern:

```c3
if (catch err = some_fallible_call()) {
    assert(err == cg::DEGENERATE_INPUT);
    return;
}
unreachable();
```

Use casts like current code:

```c3
FaceIndex face = (FaceIndex)(int)f;
```

Use `face_vertices` only after validation or with a prior `face_degree` check. Existing walks do not validate indices internally.

## Task 0: Execution setup and baseline

**Files:**
- No repo files changed.

- [ ] **Step 1: Create a dedicated branch/worktree**

Run from repo root:

```bash
git status --short --branch
git worktree add .worktrees/m5-geometry-helpers -b feat/m5-geometry-helpers main
```

Expected:

- Main has only known pre-existing untracked files.
- New worktree exists at `.worktrees/m5-geometry-helpers`.

- [ ] **Step 2: Verify baseline in worktree**

```bash
cd .worktrees/m5-geometry-helpers
git status --short --branch
c3c test
```

Expected:

- Branch is `feat/m5-geometry-helpers`.
- `93 passed, 0 failed, 0 skipped` before M5 changes.

- [ ] **Step 3: Commit policy**

No commit for setup. Every implementation task below ends with a focused commit.

## Task 1: Predicate constants and `orient_2d`

**Files:**
- Create: `src/geometry/predicates.c3`
- Create: `test/test_geometry.c3`
- Modify: `project.json`
- Modify: `manifest.json`

### Purpose

Introduce `cg::geometry` with the classifier-first public predicate surface and the smallest useful implementation: predicate sign constants plus `orient_2d`.

### Steps

- [ ] **Step 1: Write failing tests for PredicateSign constants and `orient_2d`**

Create `test/test_geometry.c3` with the module/import header and initial tests:

```c3
module test;
import cg;
import cg::half_edge;
import cg::geometry;

fn void test_predicate_sign_constants() @test
{
    assert((int)geometry::PREDICATE_NEGATIVE == -1);
    assert((int)geometry::PREDICATE_ZERO == 0);
    assert((int)geometry::PREDICATE_POSITIVE == 1);
}

fn void test_orient_2d_classifies_ccw_cw_collinear() @test
{
    Vec2f a = { 0, 0 };
    Vec2f b = { 1, 0 };
    Vec2f c = { 0, 1 };

    assert(geometry::orient_2d(a, b, c) == geometry::PREDICATE_POSITIVE);
    assert(geometry::orient_2d(a, c, b) == geometry::PREDICATE_NEGATIVE);
    assert(geometry::orient_2d({ 0, 0 }, { 1, 1 }, { 2, 2 }) == geometry::PREDICATE_ZERO);
}

fn void test_orient_2d_uses_absolute_negative_epsilon() @test
{
    Vec2f a = { 0, 0 };
    Vec2f b = { 1, 0 };
    Vec2f c = { 0, 0.000001 };

    assert(geometry::orient_2d(a, b, c, -0.01f) == geometry::PREDICATE_ZERO);
    assert(geometry::orient_2d(a, b, c, 0.0f) == geometry::PREDICATE_POSITIVE);
}
```

- [ ] **Step 2: Run tests and verify compile failure**

```bash
c3c test
```

Expected:

- Fails because `cg::geometry` or predicate names are not defined.

- [ ] **Step 3: Add `src/geometry/predicates.c3` and source lists**

Add `src/geometry/predicates.c3`:

```c3
module cg::geometry;
import cg;

typedef PredicateSign = int;

const float GEOMETRY_EPSILON = 0.00001f;
const float DEFAULT_PREDICATE_EPSILON = 0.00001f;
const PredicateSign PREDICATE_NEGATIVE = -1;
const PredicateSign PREDICATE_ZERO = 0;
const PredicateSign PREDICATE_POSITIVE = 1;

fn float abs_float(float value)
{
    if (value < 0.0f) return -value;
    return value;
}

fn PredicateSign classify_predicate(float value, float epsilon)
{
    float e = abs_float(epsilon);
    if (value > e) return PREDICATE_POSITIVE;
    if (value < -e) return PREDICATE_NEGATIVE;
    return PREDICATE_ZERO;
}

fn PredicateSign orient_2d(Vec2f a, Vec2f b, Vec2f c, float epsilon = DEFAULT_PREDICATE_EPSILON)
{
    float value = (b.x - a.x) * (c.y - a.y)
                - (b.y - a.y) * (c.x - a.x);
    return classify_predicate(value, epsilon);
}
```

Update `project.json` sources, after `src/half_edge/flip.c3` and before render files:

```json
"src/half_edge/flip.c3",
"src/geometry/predicates.c3",
"src/render/rendering_data.c3",
```

Update `manifest.json` the same way.

- [ ] **Step 4: Run tests**

```bash
c3c test
```

Expected:

- New predicate tests pass.
- Existing 93 tests still pass.

If C3 rejects `typedef PredicateSign = int;`, stop and report. The spec chose this form because negative enum values failed in C3 0.7.11, so do not silently change API shape.

- [ ] **Step 5: Commit Task 1**

```bash
git add project.json manifest.json src/geometry/predicates.c3 test/test_geometry.c3
git commit -m "geometry: add predicate signs and orientation (M5)"
```

## Task 2: Complete predicate classifiers

**Files:**
- Modify: `src/geometry/predicates.c3`
- Modify: `test/test_geometry.c3`

### Purpose

Implement `in_circle_2d` and `on_sphere` with classifier-first public APIs and no public raw determinant helpers.

### Steps

- [ ] **Step 1: Add failing incircle tests**

Append tests to `test/test_geometry.c3`:

```c3
fn void test_in_circle_2d_classifies_for_ccw_triangle() @test
{
    Vec2f a = { 0, 0 };
    Vec2f b = { 2, 0 };
    Vec2f c = { 0, 2 };

    assert(geometry::in_circle_2d(a, b, c, { 0.5, 0.5 }) == geometry::PREDICATE_POSITIVE);
    assert(geometry::in_circle_2d(a, b, c, { 3, 3 }) == geometry::PREDICATE_NEGATIVE);
    assert(geometry::in_circle_2d(a, b, c, { 2, 2 }) == geometry::PREDICATE_ZERO);
}

fn void test_in_circle_2d_orientation_dependency() @test
{
    Vec2f a = { 0, 0 };
    Vec2f b = { 2, 0 };
    Vec2f c = { 0, 2 };
    Vec2f p = { 0.5, 0.5 };

    assert(geometry::in_circle_2d(a, b, c, p) == geometry::PREDICATE_POSITIVE);
    assert(geometry::in_circle_2d(a, c, b, p) == geometry::PREDICATE_NEGATIVE);
}
```

- [ ] **Step 2: Run and verify failure**

```bash
c3c test
```

Expected:

- Fails because `in_circle_2d` is missing.

- [ ] **Step 3: Implement `in_circle_2d`**

Add to `src/geometry/predicates.c3`:

```c3
fn PredicateSign in_circle_2d(Vec2f a, Vec2f b, Vec2f c, Vec2f p, float epsilon = DEFAULT_PREDICATE_EPSILON)
{
    float ax = a.x - p.x;
    float ay = a.y - p.y;
    float bx = b.x - p.x;
    float by = b.y - p.y;
    float cx = c.x - p.x;
    float cy = c.y - p.y;

    float aa = ax * ax + ay * ay;
    float bb = bx * bx + by * by;
    float cc = cx * cx + cy * cy;

    float value = ax * (by * cc - bb * cy)
                - ay * (bx * cc - bb * cx)
                + aa * (bx * cy - by * cx);
    return classify_predicate(value, epsilon);
}
```

- [ ] **Step 4: Run tests**

```bash
c3c test
```

Expected: all current tests pass.

- [ ] **Step 5: Add failing `on_sphere` tests**

Use the positively oriented tetrahedron:

```text
a = (1, 0, 0)
b = (0, 0, 1)
c = (0, 1, 0)
d = (0, 0, 0)
```

Its circumsphere center is `(0.5, 0.5, 0.5)` and radius squared is `0.75`.

Append tests:

```c3
fn void test_on_sphere_classifies_known_tetrahedron() @test
{
    Vec3f a = { 1, 0, 0 };
    Vec3f b = { 0, 0, 1 };
    Vec3f c = { 0, 1, 0 };
    Vec3f d = { 0, 0, 0 };

    assert(geometry::on_sphere(a, b, c, d, { 0.5, 0.5, 0.5 }) == geometry::PREDICATE_POSITIVE);
    assert(geometry::on_sphere(a, b, c, d, { 3, 3, 3 }) == geometry::PREDICATE_NEGATIVE);
    assert(geometry::on_sphere(a, b, c, d, { 1, 1, 1 }) == geometry::PREDICATE_ZERO);
}

fn void test_on_sphere_negative_orientation_flips_sign() @test
{
    Vec3f a = { 1, 0, 0 };
    Vec3f b = { 0, 0, 1 };
    Vec3f c = { 0, 1, 0 };
    Vec3f d = { 0, 0, 0 };
    Vec3f p = { 0.5, 0.5, 0.5 };

    assert(geometry::on_sphere(a, b, c, d, p) == geometry::PREDICATE_POSITIVE);
    assert(geometry::on_sphere(a, c, b, d, p) == geometry::PREDICATE_NEGATIVE);
}

fn void test_on_sphere_degenerate_coplanar_classifies_zero() @test
{
    assert(geometry::on_sphere(
        { 0, 0, 0 }, { 1, 0, 0 }, { 0, 1, 0 }, { 1, 1, 0 }, { 0.25, 0.25, 0 }) == geometry::PREDICATE_ZERO);
}

fn void test_predicates_negative_epsilon_does_not_invert_classification() @test
{
    assert(geometry::in_circle_2d({ 0, 0 }, { 2, 0 }, { 0, 2 }, { 0.5, 0.5 }, -0.01f)
        == geometry::PREDICATE_ZERO);
    assert(geometry::on_sphere({ 1, 0, 0 }, { 0, 0, 1 }, { 0, 1, 0 }, { 0, 0, 0 }, { 0.5, 0.5, 0.5 }, -10.0f)
        == geometry::PREDICATE_ZERO);
}
```

- [ ] **Step 6: Run and verify failure**

```bash
c3c test
```

Expected: fails because `on_sphere` is missing.

- [ ] **Step 7: Implement `det3`, `det4`, `on_sphere`**

Add module-local helpers and public function. Keep helper functions undocumented and do not add `.c3i` declarations.

```c3
fn float det3(
    float a00, float a01, float a02,
    float a10, float a11, float a12,
    float a20, float a21, float a22)
{
    return a00 * (a11 * a22 - a12 * a21)
         - a01 * (a10 * a22 - a12 * a20)
         + a02 * (a10 * a21 - a11 * a20);
}

fn float det4(
    float r00, float r01, float r02, float r03,
    float r10, float r11, float r12, float r13,
    float r20, float r21, float r22, float r23,
    float r30, float r31, float r32, float r33)
{
    return r00 * det3(r11, r12, r13, r21, r22, r23, r31, r32, r33)
         - r01 * det3(r10, r12, r13, r20, r22, r23, r30, r32, r33)
         + r02 * det3(r10, r11, r13, r20, r21, r23, r30, r31, r33)
         - r03 * det3(r10, r11, r12, r20, r21, r22, r30, r31, r32);
}

fn float vec3_dot_for_predicates(Vec3f a, Vec3f b)
{
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

fn Vec3f vec3_sub_for_predicates(Vec3f a, Vec3f b)
{
    return { a.x - b.x, a.y - b.y, a.z - b.z };
}

fn PredicateSign on_sphere(Vec3f a, Vec3f b, Vec3f c, Vec3f d, Vec3f p, float epsilon = DEFAULT_PREDICATE_EPSILON)
{
    Vec3f ar = vec3_sub_for_predicates(a, p);
    Vec3f br = vec3_sub_for_predicates(b, p);
    Vec3f cr = vec3_sub_for_predicates(c, p);
    Vec3f dr = vec3_sub_for_predicates(d, p);

    float a_len = vec3_dot_for_predicates(ar, ar);
    float b_len = vec3_dot_for_predicates(br, br);
    float c_len = vec3_dot_for_predicates(cr, cr);
    float d_len = vec3_dot_for_predicates(dr, dr);

    float value = det4(
        ar.x, ar.y, ar.z, a_len,
        br.x, br.y, br.z, b_len,
        cr.x, cr.y, cr.z, c_len,
        dr.x, dr.y, dr.z, d_len);

    return classify_predicate(value, epsilon);
}
```

If the known tetrahedron test reports inside as negative, do not add orientation-based conditional normalization. Instead negate `value` unconditionally before classification or swap determinant row order so the positive-orientation convention holds.

- [ ] **Step 8: Run tests**

```bash
c3c test
```

Expected:

- Predicate tests pass.
- Existing tests pass.

- [ ] **Step 9: Commit Task 2**

```bash
git add src/geometry/predicates.c3 test/test_geometry.c3
git commit -m "geometry: add circle and sphere predicates (M5)"
```

## Task 3: Scalar circumcenters

**Files:**
- Create: `src/geometry/circumcenter.c3`
- Modify: `project.json`
- Modify: `manifest.json`
- Modify: `test/test_geometry.c3`

### Purpose

Add scalar fallible circumcenter helpers before batch mesh extraction.

### Steps

- [ ] **Step 1: Add failing scalar circumcenter tests and helpers**

Append local test helpers near the top of `test/test_geometry.c3`:

```c3
const float M5_TEST_EPSILON = 0.001f;

fn float m5_abs(float value)
{
    if (value < 0.0f) return -value;
    return value;
}

fn bool m5_float_approx(float a, float b)
{
    return m5_abs(a - b) <= M5_TEST_EPSILON;
}

fn bool m5_vec3_approx(Vec3f a, Vec3f b)
{
    return m5_float_approx(a.x, b.x)
        && m5_float_approx(a.y, b.y)
        && m5_float_approx(a.z, b.z);
}

fn float m5_dot3(Vec3f a, Vec3f b)
{
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

fn Vec3f m5_cross3(Vec3f a, Vec3f b)
{
    return {
        a.y * b.z - a.z * b.y,
        a.z * b.x - a.x * b.z,
        a.x * b.y - a.y * b.x,
    };
}

fn Vec3f m5_sub3(Vec3f a, Vec3f b)
{
    return { a.x - b.x, a.y - b.y, a.z - b.z };
}
```

Add tests:

```c3
fn void test_circumcenter_planar_right_triangle() @test
{
    Vec3f center = geometry::circumcenter_planar({ 0, 0, 0 }, { 2, 0, 0 }, { 0, 2, 0 })!!;
    assert(m5_vec3_approx(center, { 1, 1, 0 }));
}

fn void test_circumcenter_planar_tilted_triangle_equal_distances() @test
{
    Vec3f a = { 0, 0, 1 };
    Vec3f b = { 2, 0, 1 };
    Vec3f c = { 0, 2, 3 };
    Vec3f center = geometry::circumcenter_planar(a, b, c)!!;

    Vec3f da = m5_sub3(center, a);
    Vec3f db = m5_sub3(center, b);
    Vec3f dc = m5_sub3(center, c);
    float ra = m5_dot3(da, da);
    float rb = m5_dot3(db, db);
    float rc = m5_dot3(dc, dc);

    Vec3f normal = m5_cross3(m5_sub3(b, a), m5_sub3(c, a));
    float plane_distance_scaled = m5_dot3(m5_sub3(center, a), normal);

    assert(m5_float_approx(ra, rb));
    assert(m5_float_approx(ra, rc));
    assert(m5_float_approx(plane_distance_scaled, 0.0f));
}

fn void test_circumcenter_planar_degenerate_faults() @test
{
    if (catch err = geometry::circumcenter_planar({ 0, 0, 0 }, { 1, 1, 1 }, { 2, 2, 2 })) {
        assert(err == cg::DEGENERATE_INPUT);
    } else {
        unreachable();
    }

    if (catch err = geometry::circumcenter_planar({ 0, 0, 0 }, { 0, 0, 0 }, { 1, 0, 0 })) {
        assert(err == cg::DEGENERATE_INPUT);
        return;
    }
    unreachable();
}

fn void test_circumcenter_on_sphere_unit_axes() @test
{
    Vec3f center = geometry::circumcenter_on_sphere({ 1, 0, 0 }, { 0, 1, 0 }, { 0, 0, 1 }, 1.0f)!!;
    assert(m5_float_approx(m5_dot3(center, center), 1.0f));
    assert(center.x > 0.0f);
    assert(center.y > 0.0f);
    assert(center.z > 0.0f);
}

fn void test_circumcenter_on_sphere_invalid_radius_faults() @test
{
    if (catch err = geometry::circumcenter_on_sphere({ 1, 0, 0 }, { 0, 1, 0 }, { 0, 0, 1 }, 0.0f)) {
        assert(err == cg::DEGENERATE_INPUT);
        return;
    }
    unreachable();
}
```


- [ ] **Step 2: Run and verify failure**

```bash
c3c test
```

Expected: fails because circumcenter functions are missing and source is not listed.

- [ ] **Step 3: Create `src/geometry/circumcenter.c3` and list it**

Add source entry after `src/geometry/predicates.c3` in both `project.json` and `manifest.json`:

```json
"src/geometry/predicates.c3",
"src/geometry/circumcenter.c3",
"src/render/rendering_data.c3",
```

Create `src/geometry/circumcenter.c3`:

```c3
module cg::geometry;
import cg;
import cg::half_edge;
import std::math;

fn Vec3f vec3_add(Vec3f a, Vec3f b)
{
    return { a.x + b.x, a.y + b.y, a.z + b.z };
}

fn Vec3f vec3_sub(Vec3f a, Vec3f b)
{
    return { a.x - b.x, a.y - b.y, a.z - b.z };
}

fn Vec3f vec3_scale(Vec3f v, float scale)
{
    return { v.x * scale, v.y * scale, v.z * scale };
}

fn float vec3_dot(Vec3f a, Vec3f b)
{
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

fn Vec3f vec3_cross(Vec3f a, Vec3f b)
{
    return {
        a.y * b.z - a.z * b.y,
        a.z * b.x - a.x * b.z,
        a.x * b.y - a.y * b.x,
    };
}

fn float vec3_length(Vec3f v)
{
    return (float)math::sqrt((double)vec3_dot(v, v));
}

fn Vec3f? circumcenter_planar(Vec3f a, Vec3f b, Vec3f c)
{
    Vec3f u = vec3_sub(b, a);
    Vec3f v = vec3_sub(c, a);
    Vec3f n = vec3_cross(u, v);
    float n_len_sq = vec3_dot(n, n);
    if (n_len_sq <= GEOMETRY_EPSILON * GEOMETRY_EPSILON) return cg::DEGENERATE_INPUT~;

    float uu = vec3_dot(u, u);
    float vv = vec3_dot(v, v);
    Vec3f term_u = vec3_scale(vec3_cross(n, u), vv);
    Vec3f term_v = vec3_scale(vec3_cross(v, n), uu);
    return vec3_add(a, vec3_scale(vec3_add(term_u, term_v), 1.0f / (2.0f * n_len_sq)));
}

fn Vec3f? circumcenter_on_sphere(Vec3f a, Vec3f b, Vec3f c, float radius)
{
    if (radius <= GEOMETRY_EPSILON) return cg::DEGENERATE_INPUT~;

    Vec3f planar = circumcenter_planar(a, b, c)!;
    float len = vec3_length(planar);
    if (len <= GEOMETRY_EPSILON) return cg::DEGENERATE_INPUT~;

    return vec3_scale(planar, radius / len);
}
```

If duplicate helper names conflict across files in the same module, rename with file prefix, e.g. `circ_vec3_dot`, `centroid_vec3_add`, and `predicate_vec3_dot`. Do not use `@private`.

- [ ] **Step 4: Run tests and fix compile issues**

```bash
c3c test
```

Expected:

- Scalar circumcenter tests pass.
- Existing predicate tests pass.

Common fixes:

- If duplicate helper names conflict, rename helpers.
- If `std::math` function differs, mirror existing `render/geometry.c3` pattern exactly.
- If C3 complains about optional assignment, follow existing `Vec3f? result; if (catch err = result)` patterns from render.

- [ ] **Step 5: Commit Task 3**

```bash
git add project.json manifest.json src/geometry/circumcenter.c3 test/test_geometry.c3
git commit -m "geometry: add scalar circumcenters (M5)"
```

## Task 4: Batch circumcenters

**Files:**
- Modify: `src/geometry/circumcenter.c3`
- Modify: `test/test_geometry.c3`

### Purpose

Add allocator-owned face-index-mapped circumcenter arrays for triangle meshes.

### Steps

- [ ] **Step 1: Add failing batch circumcenter tests**

Add tests:

```c3
fn void test_circumcenters_planar_triangle_mesh() @test
{
    Vec3f[4] positions = {
        { 0, 0, 0 },
        { 2, 0, 0 },
        { 0, 2, 0 },
        { 2, 2, 0 },
    };
    uint[6] indices = { 0, 1, 2, 1, 3, 2 };
    HalfEdgeMesh mesh = half_edge::from_triangles(mem, positions[..], indices[..])!!;
    defer mesh.destroy();

    Vec3f[] centers = geometry::circumcenters_planar(mem, &mesh)!!;
    defer free(centers);

    assert(centers.len == mesh.faces.len);
    assert(m5_vec3_approx(centers[0], { 1, 1, 0 }));
    assert(m5_vec3_approx(centers[1], { 1, 1, 0 }));
}

fn void test_circumcenters_planar_rejects_non_triangle_face() @test
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

    if (catch err = geometry::circumcenters_planar(mem, &mesh)) {
        assert(err == cg::NON_TRIANGLE_FACE);
        return;
    }
    unreachable();
}

fn void test_circumcenters_planar_rejects_degenerate_triangle() @test
{
    Vec3f[3] positions = {
        { 0, 0, 0 },
        { 1, 1, 1 },
        { 2, 2, 2 },
    };
    uint[3] indices = { 0, 1, 2 };
    HalfEdgeMesh mesh = half_edge::from_triangles(mem, positions[..], indices[..])!!;
    defer mesh.destroy();

    if (catch err = geometry::circumcenters_planar(mem, &mesh)) {
        assert(err == cg::DEGENERATE_INPUT);
        return;
    }
    unreachable();
}

fn void test_circumcenters_on_sphere_triangle_mesh() @test
{
    Vec3f[3] positions = {
        { 1, 0, 0 },
        { 0, 1, 0 },
        { 0, 0, 1 },
    };
    uint[3] indices = { 0, 1, 2 };
    HalfEdgeMesh mesh = half_edge::from_triangles(mem, positions[..], indices[..])!!;
    defer mesh.destroy();

    Vec3f[] centers = geometry::circumcenters_on_sphere(mem, &mesh, 1.0f)!!;
    defer free(centers);

    assert(centers.len == 1);
    assert(m5_float_approx(m5_dot3(centers[0], centers[0]), 1.0f));
}
```

- [ ] **Step 2: Run and verify failure**

```bash
c3c test
```

Expected: missing batch functions.

- [ ] **Step 3: Implement `collect_triangle_face_vertices` and batch functions**

Add to `src/geometry/circumcenter.c3`:

```c3
fn void? collect_triangle_face_vertices(HalfEdgeMesh* mesh, FaceIndex face, VertexIndex[] out)
{
    if (mesh.face_degree(face) != 3) return cg::NON_TRIANGLE_FACE~;
    if (out.len < 3) return cg::OUTPUT_BUFFER_TOO_SMALL~;

    int count = mesh.face_vertices(face, out)!;
    if (count != 3) return cg::NON_TRIANGLE_FACE~;
    return;
}

fn Vec3f[]? circumcenters_planar(Allocator alloc, HalfEdgeMesh* tri_mesh)
{
    tri_mesh.validate()!;

    Vec3f[] out = mem::alloc::new_array(alloc, Vec3f, tri_mesh.faces.len);
    defer catch free(out);

    for (usz i = 0; i < tri_mesh.faces.len; i++) {
        FaceIndex face = (FaceIndex)(int)i;
        VertexIndex[3] verts;
        collect_triangle_face_vertices(tri_mesh, face, verts[..])!;

        Vec3f a = tri_mesh.positions[verts[0]];
        Vec3f b = tri_mesh.positions[verts[1]];
        Vec3f c = tri_mesh.positions[verts[2]];
        out[i] = circumcenter_planar(a, b, c)!;
    }

    return out;
}

fn Vec3f[]? circumcenters_on_sphere(Allocator alloc, HalfEdgeMesh* tri_mesh, float radius)
{
    if (radius <= GEOMETRY_EPSILON) return cg::DEGENERATE_INPUT~;
    tri_mesh.validate()!;

    Vec3f[] out = mem::alloc::new_array(alloc, Vec3f, tri_mesh.faces.len);
    defer catch free(out);

    for (usz i = 0; i < tri_mesh.faces.len; i++) {
        FaceIndex face = (FaceIndex)(int)i;
        VertexIndex[3] verts;
        collect_triangle_face_vertices(tri_mesh, face, verts[..])!;

        Vec3f a = tri_mesh.positions[verts[0]];
        Vec3f b = tri_mesh.positions[verts[1]];
        Vec3f c = tri_mesh.positions[verts[2]];
        out[i] = circumcenter_on_sphere(a, b, c, radius)!;
    }

    return out;
}
```

If C3 rejects indexing with inline typedefs, cast vertex indices to `int`/`usz` using patterns already accepted in the codebase.

- [ ] **Step 4: Run tests**

```bash
c3c test
```

Expected:

- Batch circumcenter tests pass.
- No regressions.

- [ ] **Step 5: Commit Task 4**

```bash
git add src/geometry/circumcenter.c3 test/test_geometry.c3
git commit -m "geometry: add batch circumcenters (M5)"
```

## Task 5: Face centroids

**Files:**
- Create: `src/geometry/centroid.c3`
- Modify: `project.json`
- Modify: `manifest.json`
- Modify: `test/test_geometry.c3`

### Purpose

Add scalar and batch face centroid helpers using triangle exact average and polygon vertex-average.

### Steps

- [ ] **Step 1: Add failing centroid tests**

Add tests:

```c3
fn void test_face_centroid_triangle_average() @test
{
    Vec3f[3] positions = {
        { 0, 0, 0 },
        { 3, 0, 0 },
        { 0, 3, 0 },
    };
    uint[3] indices = { 0, 1, 2 };
    HalfEdgeMesh mesh = half_edge::from_triangles(mem, positions[..], indices[..])!!;
    defer mesh.destroy();

    Vec3f centroid = geometry::face_centroid(&mesh, 0)!!;
    assert(m5_vec3_approx(centroid, { 1, 1, 0 }));
}

fn void test_face_centroid_quad_vertex_average() @test
{
    Vec3f[4] positions = {
        { 0, 0, 0 },
        { 2, 0, 0 },
        { 2, 2, 0 },
        { 0, 2, 0 },
    };
    uint[2] offsets = { 0, 4 };
    uint[4] indices = { 0, 1, 2, 3 };
    HalfEdgeMesh mesh = half_edge::from_polygons(mem, positions[..], offsets[..], indices[..])!!;
    defer mesh.destroy();

    Vec3f centroid = geometry::face_centroid(&mesh, 0)!!;
    assert(m5_vec3_approx(centroid, { 1, 1, 0 }));
}

fn void test_face_centroid_invalid_face_faults() @test
{
    Vec3f[3] positions = { { 0, 0, 0 }, { 1, 0, 0 }, { 0, 1, 0 } };
    uint[3] indices = { 0, 1, 2 };
    HalfEdgeMesh mesh = half_edge::from_triangles(mem, positions[..], indices[..])!!;
    defer mesh.destroy();

    if (catch err = geometry::face_centroid(&mesh, 99)) {
        assert(err == cg::INVALID_FACE_REFERENCE);
        return;
    }
    unreachable();
}

fn void test_face_centroids_batch_preserves_face_index_mapping() @test
{
    Vec3f[4] positions = {
        { 0, 0, 0 },
        { 3, 0, 0 },
        { 0, 3, 0 },
        { 3, 3, 0 },
    };
    uint[6] indices = { 0, 1, 2, 1, 3, 2 };
    HalfEdgeMesh mesh = half_edge::from_triangles(mem, positions[..], indices[..])!!;
    defer mesh.destroy();

    Vec3f[] centroids = geometry::face_centroids(mem, &mesh)!!;
    defer free(centroids);

    assert(centroids.len == mesh.faces.len);
    assert(m5_vec3_approx(centroids[0], { 1, 1, 0 }));
    assert(m5_vec3_approx(centroids[1], { 2, 2, 0 }));
}

fn void test_face_centroid_scalar_matches_batch() @test
{
    Vec3f[4] positions = {
        { 0, 0, 0 },
        { 2, 0, 0 },
        { 2, 2, 0 },
        { 0, 2, 0 },
    };
    uint[2] offsets = { 0, 4 };
    uint[4] indices = { 0, 1, 2, 3 };
    HalfEdgeMesh mesh = half_edge::from_polygons(mem, positions[..], offsets[..], indices[..])!!;
    defer mesh.destroy();

    Vec3f[] batch = geometry::face_centroids(mem, &mesh)!!;
    defer free(batch);

    Vec3f scalar = geometry::face_centroid(&mesh, 0)!!;
    assert(m5_vec3_approx(scalar, batch[0]));
}
```

- [ ] **Step 2: Run and verify failure**

```bash
c3c test
```

Expected: missing centroid functions.

- [ ] **Step 3: Add source file and source-list entries**

Add source entry after `src/geometry/circumcenter.c3` in both `project.json` and `manifest.json`:

```json
"src/geometry/predicates.c3",
"src/geometry/circumcenter.c3",
"src/geometry/centroid.c3",
"src/render/rendering_data.c3",
```

Create `src/geometry/centroid.c3`:

```c3
module cg::geometry;
import cg;
import cg::half_edge;

fn Vec3f centroid_vec3_add(Vec3f a, Vec3f b)
{
    return { a.x + b.x, a.y + b.y, a.z + b.z };
}

fn Vec3f centroid_vec3_scale(Vec3f v, float scale)
{
    return { v.x * scale, v.y * scale, v.z * scale };
}

fn Vec3f? centroid_for_valid_face(HalfEdgeMesh* mesh, FaceIndex face)
{
    HeIndex start = mesh.faces[face].half_edge;
    HeIndex current = start;
    Vec3f sum = { 0, 0, 0 };
    int count = 0;

    while (true) {
        if (current == cg::INVALID_HE) return cg::INVALID_FACE_CYCLE~;
        if ((usz)count >= mesh.half_edges.len) return cg::INVALID_FACE_CYCLE~;

        HalfEdge edge = mesh.half_edges[current];
        if (edge.origin < 0 || (usz)edge.origin >= mesh.positions.len) return cg::INVALID_TOPOLOGY~;

        sum = centroid_vec3_add(sum, mesh.positions[edge.origin]);
        count++;
        current = edge.next;
        if (current == start) break;
    }

    if (count == 0) return cg::INVALID_FACE_CYCLE~;
    return centroid_vec3_scale(sum, 1.0f / (float)count);
}

fn Vec3f? face_centroid(HalfEdgeMesh* mesh, FaceIndex face)
{
    if (face < 0 || (usz)face >= mesh.faces.len) return cg::INVALID_FACE_REFERENCE~;
    mesh.validate()!;
    return centroid_for_valid_face(mesh, face);
}

fn Vec3f[]? face_centroids(Allocator alloc, HalfEdgeMesh* mesh)
{
    mesh.validate()!;

    Vec3f[] out = mem::alloc::new_array(alloc, Vec3f, mesh.faces.len);
    defer catch free(out);

    for (usz i = 0; i < mesh.faces.len; i++) {
        out[i] = centroid_for_valid_face(mesh, (FaceIndex)(int)i)!;
    }

    return out;
}
```

If indexing with typed aliases requires casts, adjust consistently with compiler feedback.

- [ ] **Step 4: Run tests**

```bash
c3c test
```

Expected:

- Centroid tests pass.
- Existing tests pass.

- [ ] **Step 5: Commit Task 5**

```bash
git add project.json manifest.json src/geometry/centroid.c3 test/test_geometry.c3
git commit -m "geometry: add face centroids (M5)"
```

## Task 6: Integration cleanup and milestone verification

**Files:**
- Modify only if review/verification finds issues.

### Purpose

Ensure M5 is clean, self-contained, and ready to merge.

### Steps

- [ ] **Step 1: Inspect final diff**

```bash
git status --short --branch
git diff --stat main...HEAD
git diff --check
```

Expected:

- Only M5 files changed/added:
  - `project.json`
  - `manifest.json`
  - `src/geometry/predicates.c3`
  - `src/geometry/circumcenter.c3`
  - `src/geometry/centroid.c3`
  - `test/test_geometry.c3`
- `git diff --check` passes.

- [ ] **Step 2: Anti-pattern scan**

Search changed source files for forbidden patterns:

```bash
git grep -n "@private\|->\|sizeof\|goto\|malloc\|assert(\|unreachable" -- src/geometry || true
git grep -n "@private\|->\|sizeof\|goto\|malloc" -- test/test_geometry.c3 || true
```

Expected:

- No production hits in `src/geometry`.
- Test hits for `assert`/`unreachable` are acceptable.

- [ ] **Step 3: Full verification**

```bash
c3c build static-lib
c3c test
c3c build debug
c3c build release
```

Expected:

- All builds pass.
- Test count increases from 93 by the new geometry tests.
- Existing M0-M4 tests remain green.

- [ ] **Step 4: Final branch review**

Dispatch two review subagents before merging:

1. Whole-branch code review:
   - Diff scope: `main...HEAD`
   - Check spec compliance, test coverage, ownership/fault handling, no unrelated refactors.

2. C3/project expert review:
   - Focus C3 0.7.11 syntax/idioms, module structure, fault propagation, allocator ownership, determinant/circumcenter formulas, hidden-helper policy.

Expected:

- Both approve or return concrete fixes.
- If fixes are needed, apply them with TDD/verification and create focused follow-up commit(s).

- [ ] **Step 5: Final commit if cleanup was needed**

Only if Step 4 caused changes:

```bash
git add <changed-files>
git commit -m "geometry: polish M5 implementation"
```

- [ ] **Step 6: Report handoff status**

Report:

- branch name
- worktree path
- commits
- test/build commands run
- review approvals
- merge options

## Subagent handoff strategy

Recommended execution mode: subagent-driven.

- Parent creates the worktree and verifies baseline.
- Parent dispatches one subagent per task.
- Each subagent must:
  - load required skills inside its own context if available (`c3-expert`, `test-driven-development`)
  - implement only its assigned task
  - run the task-specific tests plus `c3c test`
  - commit its changes
  - return commit hash, files changed, tests run, and deviations
- Parent verifies each subagent commit before dispatching the next task.
- Parent dispatches two final review subagents after Task 6.

Task grouping for subagents:

1. Task 1: Predicate constants and `orient_2d`
2. Task 2: Complete predicates
3. Task 3: Scalar circumcenters
4. Task 4: Batch circumcenters
5. Task 5: Face centroids
6. Task 6: Final audit/review/verification

## Known risks and mitigations

- **C3 source visibility:** Helpers may be callable from the module even if undocumented. Do not add raw predicate helpers to `.c3i`; keep helper names internal-looking.
- **Duplicate helper names:** Files in the same `cg::geometry` module may collide. If this happens, prefix helpers by file purpose.
- **Inline typedef indexing:** If `VertexIndex`/`FaceIndex` direct indexing fails, cast using patterns already accepted in existing code.
- **On-sphere sign:** Do not orientation-normalize conditionally. Use tests to verify the determinant arrangement; if sign is opposite for a positively oriented tetrahedron, negate the determinant unconditionally or swap rows.
- **Batch cleanup:** Use `defer catch free(out)`, not plain `defer free(out)`, for arrays returned on success.
- **Geometry vs render helpers:** Do not import `cg::render`; duplicate tiny helpers locally.
- **Negative epsilon:** Use absolute value, not default clamping. Tests should cover this.

## Completion criteria

M5 is complete when:

- `src/geometry/predicates.c3`, `circumcenter.c3`, and `centroid.c3` exist and are listed in both source manifests.
- All public API from the M5 spec exists.
- No public raw determinant helper API is introduced.
- All new tests pass.
- `c3c build static-lib`, `c3c test`, `c3c build debug`, and `c3c build release` pass.
- Final review subagents approve.
- Implementation is committed in focused commits.
