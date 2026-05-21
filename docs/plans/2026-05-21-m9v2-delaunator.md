# M9v2 — Delaunay 2D (delaunator algorithm)

## Overview

Port of the [delaunator](https://github.com/mapbox/delaunator) algorithm for planar Delaunay triangulation. Given `Vec3f[]` positions (z ignored), returns a triangular `HalfEdgeMesh` with planar boundary. Uses a private builder with flat arrays preallocated upfront — no `HalfEdgeMesh` mutation during insertion, no `from_triangles`.

This spec follows the reference implementation's hull linked-list, hull hash, point insertion, and `legalize()` behavior. An implementer should not need to read delaunator source code.

## Module

`module cg::delaunay;` — rewritten `src/delaunay/delaunay_2d.c3`.

Imports: `cg`, `cg::geometry` (`in_circle_2d`, `orient_2d`, `circumcenter_planar`), `std::sort`, `std::math`.

Umbrella unchanged.

## Public API — unchanged

```c3
fn HalfEdgeMesh? delaunay_2d(Allocator alloc, Vec3f[] positions, DelaunayOptions options = {});
```

`DelaunayOptions.has_bounding_box`: if true, all points must be inside the given AABB (x/y only, z ignored, inclusive bounds). Invalid AABB means:

```c3
if (options.bounding_box.min.x > options.bounding_box.max.x
    || options.bounding_box.min.y > options.bounding_box.max.y)
{
    return DEGENERATE_INPUT~;
}
```

No super-triangle is constructed.

## Private builder

```c3
struct DelaunayBuilder {
    VertexIndex[] triangles;   // flat: 3 per face
    HeIndex[] halfedges;       // flat: 3 per face, twin links (INVALID_HE = boundary)
    HeIndex[] hull_tri;        // hull vertex -> adjacent boundary half-edge
    VertexIndex[] hull_prev;   // linked-list prev (INVALID_VERTEX = not on hull)
    VertexIndex[] hull_next;   // linked-list next (INVALID_VERTEX = not on hull / removed)
    VertexIndex[] hull_hash;   // spatial hash (INVALID_VERTEX = empty slot)
    HeIndex[] legalize_stack;  // preallocated stack for legalize()
    Vec3f hull_center;         // seed circumcenter used for hash keys
    VertexIndex hull_start;    // any current hull vertex
    usz face_count;            // current number of triangles
}
```

All arrays preallocated once:

| Array                    | Size           | Initial value                  |
| ------------------------ | -------------- | ------------------------------ |
| `triangles`, `halfedges` | `3 × (2n - 5)` | `INVALID_VERTEX`, `INVALID_HE` |
| `hull_prev`, `hull_next` | `n`            | `INVALID_VERTEX`               |
| `hull_tri`               | `n`            | `INVALID_HE`                   |
| `hull_hash`              | `max(1, √n)`   | `INVALID_VERTEX`               |
| `legalize_stack`         | `2n - 5`       | unspecified                    |

Vertex half-edges are NOT maintained during insertion. During finalization (Phase 6), `mesh.vertices[v].half_edge` is set by scanning `triangles[]` — for each half-edge `i` with `triangles[i] = v`, `mesh.vertices[v].half_edge` is set to `i` if it has not already been set.

No mutation of `HalfEdgeMesh` before Phase 6. No allocations in the insertion loop.

## Algorithm

### Phase 0 — Dedup, validate

Remove exact `(x,y)` duplicates (z ignored, first occurrence kept). If fewer than 3 non-collinear distinct points remain, return `DEGENERATE_INPUT~`.

Collinearity check: find first 2 distinct `(x,y)` points, then scan for a third with `orient_2d != PREDICATE_ZERO`. If none found, return `DEGENERATE_INPUT~`.

Build `working_positions` from deduped unique points. Allocate all builder arrays.

### Phase 1 — Seed triangle and sort order

Bbox center: `bbox_center = (min + max) / 2`. Find:

1. `i0`: closest point to `bbox_center`.
2. `i1`: closest point to `i0`, excluding `i0`.
3. `i2`: point with smallest finite `circumradius_sq(i0, i1, candidate)`, excluding `i0` and `i1`.

`circumradius_sq(a, b, c)`: try `circumcenter_planar(a, b, c)`. If fault (collinear), return infinity. Else return squared distance from the circumcenter to vertex `a`.

Orient seed CCW:

```c3
Vec2f p0 = { working_positions[(int)i0].x, working_positions[(int)i0].y };
Vec2f p1 = { working_positions[(int)i1].x, working_positions[(int)i1].y };
Vec2f p2 = { working_positions[(int)i2].x, working_positions[(int)i2].y };
if (geometry::orient_2d(p0, p1, p2) != geometry::PREDICATE_POSITIVE)
{
    VertexIndex tmp = i1;
    i1 = i2;
    i2 = tmp;
}
```

Must be `PREDICATE_POSITIVE` after the swap or the input is degenerate.

Add seed triangle: `triangles = [i0, i1, i2]`, `halfedges = [INVALID_HE, INVALID_HE, INVALID_HE]`, `face_count = 1`.

Compute the seed circumcenter. Sort point indices by squared distance to this center (ascending). Skip `i0`, `i1`, and `i2` during insertion.

Use a valid C3 0.8.0 sort comparator. If sorting indices with precomputed distances, context is the last comparator parameter:

```c3
struct DistanceSortContext {
    float[] distance_sq;
}

fn int compare_by_seed_distance(VertexIndex a, VertexIndex b, DistanceSortContext ctx)
{
    float da = ctx.distance_sq[(int)a];
    float db = ctx.distance_sq[(int)b];
    if (da < db) return -1;
    if (da > db) return 1;
    return (int)a - (int)b;
}

DistanceSortContext sort_ctx = { distance_sq };
sort::quicksort(indices, &compare_by_seed_distance, sort_ctx);
```

### Phase 2 — Hull initialization

```c3
builder.hull_start = i0;
builder.hull_center = seed_center;

builder.hull_next[(int)i0] = i1;
builder.hull_next[(int)i1] = i2;
builder.hull_next[(int)i2] = i0;

builder.hull_prev[(int)i0] = i2;
builder.hull_prev[(int)i1] = i0;
builder.hull_prev[(int)i2] = i1;

builder.hull_tri[(int)i0] = (HeIndex)0;
builder.hull_tri[(int)i1] = (HeIndex)1;
builder.hull_tri[(int)i2] = (HeIndex)2;

hash_edge(&builder, working_positions[(int)i0], i0);
hash_edge(&builder, working_positions[(int)i1], i1);
hash_edge(&builder, working_positions[(int)i2], i2);
```

All other `hull_prev` and `hull_next` entries remain `INVALID_VERTEX`. Removed hull vertices must also be explicitly marked by setting `hull_next[v] = INVALID_VERTEX` so hash probes skip stale entries.

Hash size = `max(1, ceil(sqrt(unique_count)))`. Hash sentinel = `INVALID_VERTEX`.

### Phase 3 — Point insertion

For each sorted `VertexIndex i`:

1. Skip seed vertices `i0`, `i1`, `i2`.
2. Skip near-duplicate sorted points. Use the same component-wise test as the reference: `abs(p.x - prev_p.x) <= EPSILON && abs(p.y - prev_p.y) <= EPSILON`.
3. Find a visible hull edge.
4. Add triangles fan-wise, legalizing after each addition.
5. Relink the hull and update the hash.

#### 3a. Find visible hull edge

The reference first hashes the point to a hull vertex, steps one vertex backward, then walks forward until it finds a visible edge. In this spec, a visible edge `e -> q` satisfies:

```c3
fn bool counter_clockwise_from_point(Vec3f p, Vec3f e, Vec3f q)
{
    Vec2f pp = { p.x, p.y };
    Vec2f ee = { e.x, e.y };
    Vec2f qq = { q.x, q.y };
    return geometry::orient_2d(pp, ee, qq) == geometry::PREDICATE_POSITIVE;
}
```

That is the C3 translation of `counter_clockwise(p, points[e], points[q])` in the reference.

```c3
struct VisibleEdgeResult {
    VertexIndex edge;
    bool backwards;
}

fn VisibleEdgeResult find_visible_edge(DelaunayBuilder* builder, Vec3f p, float span, Vec3f[] positions)
{
    VertexIndex start = INVALID_VERTEX;
    usz key = hash_key(builder, p);
    usz hash_len = (usz)builder.hull_hash.len;

    for (usz j = 0; j < hash_len; j++)
    {
        usz hash_index = fast_mod(key + j, hash_len);
        VertexIndex candidate = builder.hull_hash[hash_index];
        if (candidate == INVALID_VERTEX) continue;

        VertexIndex candidate_next = builder.hull_next[(int)candidate];
        if (candidate_next == INVALID_VERTEX) continue;
        if (candidate_next == candidate) continue;

        start = candidate;
        break;
    }

    if (start == INVALID_VERTEX) return { INVALID_VERTEX, false };
    if (builder.hull_prev[(int)start] == INVALID_VERTEX) return { INVALID_VERTEX, false };
    if (builder.hull_prev[(int)start] == start) return { INVALID_VERTEX, false };

    start = builder.hull_prev[(int)start];
    VertexIndex e = start;

    while (true)
    {
        VertexIndex q = builder.hull_next[(int)e];
        if (q == INVALID_VERTEX) return { INVALID_VERTEX, false };

        if (equals_with_span(p, positions[(int)e], span)
            || equals_with_span(p, positions[(int)q], span))
        {
            return { INVALID_VERTEX, false };
        }

        if (counter_clockwise_from_point(p, positions[(int)e], positions[(int)q]))
        {
            return { e, e == start };
        }

        e = q;
        if (e == start) return { INVALID_VERTEX, false };
    }
}
```

`backwards` is exactly `edge == start` from the reference. Only run the backward walk when this is true.

`equals_with_span(a, b, span)` uses the reference's scaled duplicate check:

```c3
fn bool equals_with_span(Vec3f a, Vec3f b, float span)
{
    float dx = b.x - a.x;
    float dy = b.y - a.y;
    return (dx * dx + dy * dy) / span < 1.0e-20f;
}
```

#### 3b. Add first triangle and walk forward

`e` stays on the hull. The forward walk removes only vertices between `e` and the final `walk_end`.

```c3
VisibleEdgeResult visible = find_visible_edge(&builder, working_positions[(int)i], span, working_positions);
VertexIndex e = visible.edge;
if (e == INVALID_VERTEX) continue;

VertexIndex next = builder.hull_next[(int)e];
HeIndex t = add_triangle(&builder,
    e, i, next,
    INVALID_HE, INVALID_HE, builder.hull_tri[(int)e]);

builder.hull_tri[(int)i] = legalize(&builder, (HeIndex)((int)t + 2), working_positions);
builder.hull_tri[(int)e] = t;

while (true)
{
    VertexIndex q = builder.hull_next[(int)next];
    if (!counter_clockwise_from_point(working_positions[(int)i], working_positions[(int)next], working_positions[(int)q]))
    {
        break;
    }

    t = add_triangle(&builder,
        next, i, q,
        builder.hull_tri[(int)i], INVALID_HE, builder.hull_tri[(int)next]);

    builder.hull_tri[(int)i] = legalize(&builder, (HeIndex)((int)t + 2), working_positions);

    VertexIndex removed = next;
    next = q;
    builder.hull_next[(int)removed] = INVALID_VERTEX;
    builder.hull_prev[(int)removed] = INVALID_VERTEX;
    builder.hull_tri[(int)removed] = INVALID_HE;
}
```

After this loop:

- `e` is still on the hull.
- `next` is the forward walk end and is still on the hull.
- Every removed vertex was between `e` and `next` in the old linked list and has `hull_next[v] = INVALID_VERTEX`.
- `builder.hull_tri[i]` is the `HeIndex` returned by the most recent `legalize(t + 2)` and must be kept for hull relinking.

#### 3c. Walk backward when required

The backward walk starts from `e`. When it removes an edge endpoint, the current `e` is removed and the previous vertex `q` becomes the new endpoint.

```c3
if (visible.backwards)
{
    while (true)
    {
        VertexIndex q = builder.hull_prev[(int)e];
        if (!counter_clockwise_from_point(working_positions[(int)i], working_positions[(int)q], working_positions[(int)e]))
        {
            break;
        }

        t = add_triangle(&builder,
            q, i, e,
            INVALID_HE, builder.hull_tri[(int)e], builder.hull_tri[(int)q]);

        (void)legalize(&builder, (HeIndex)((int)t + 2), working_positions);
        builder.hull_tri[(int)q] = t;

        VertexIndex removed = e;
        e = q;
        builder.hull_next[(int)removed] = INVALID_VERTEX;
        builder.hull_prev[(int)removed] = INVALID_VERTEX;
        builder.hull_tri[(int)removed] = INVALID_HE;
    }
}
```

After this loop:

- `e` is the final backward endpoint and is still on the hull.
- `next` is still the final forward endpoint and is still on the hull.
- Removed vertices are explicitly marked with `hull_next[v] = INVALID_VERTEX`.
- The `legalize()` return value is intentionally ignored in the backward loop, matching the reference; `hull_tri[q] = t` is the required update for the new endpoint.

#### 3d. Relink hull and update hash

The new point `i` is inserted between final endpoint `e` and final endpoint `next`.

```c3
builder.hull_prev[(int)i] = e;
builder.hull_next[(int)e] = i;

builder.hull_prev[(int)next] = i;
builder.hull_next[(int)i] = next;

builder.hull_start = e;

hash_edge(&builder, working_positions[(int)i], i);
hash_edge(&builder, working_positions[(int)e], e);
```

Do not try to delete stale hash entries. Hash probes skip stale entries by checking `hull_next[candidate] != INVALID_VERTEX`.

### Phase 4 — `add_triangle`

```c3
fn HeIndex add_triangle(DelaunayBuilder* builder,
    VertexIndex i0, VertexIndex i1, VertexIndex i2,
    HeIndex twin0, HeIndex twin1, HeIndex twin2)
{
    usz face_index = builder.face_count;
    builder.face_count++;

    usz base_index = 3 * face_index;
    HeIndex base = (HeIndex)(int)base_index;
    HeIndex base_next = (HeIndex)(int)(base_index + 1);
    HeIndex base_prev = (HeIndex)(int)(base_index + 2);

    builder.triangles[base_index] = i0;
    builder.triangles[base_index + 1] = i1;
    builder.triangles[base_index + 2] = i2;

    link(builder, base, twin0);
    link(builder, base_next, twin1);
    link(builder, base_prev, twin2);

    return base;
}

fn void link(DelaunayBuilder* builder, HeIndex a, HeIndex b)
{
    builder.halfedges[(int)a] = b;
    if (b != INVALID_HE)
    {
        builder.halfedges[(int)b] = a;
    }
}
```

### Phase 5 — `legalize()`

`legalize()` is stack-based and returns the final `ar` half-edge, exactly like the reference. Callers assign this return value to `hull_tri[i]` after the first and forward-walk triangles.

```c3
fn HeIndex next_halfedge(HeIndex i)
{
    if ((int)i % 3 == 2) return (HeIndex)((int)i - 2);
    return (HeIndex)((int)i + 1);
}

fn HeIndex prev_halfedge(HeIndex i)
{
    if ((int)i % 3 == 0) return (HeIndex)((int)i + 2);
    return (HeIndex)((int)i - 1);
}

fn HeIndex legalize(DelaunayBuilder* builder, HeIndex start_he, Vec3f[] positions)
{
    usz stack_size = 0;
    HeIndex a = start_he;
    HeIndex ar = INVALID_HE;

    while (true)
    {
        HeIndex b = builder.halfedges[(int)a];
        ar = prev_halfedge(a);

        if (b == INVALID_HE)
        {
            if (stack_size == 0) return ar;
            stack_size--;
            a = builder.legalize_stack[stack_size];
            continue;
        }

        HeIndex al = next_halfedge(a);
        HeIndex bl = prev_halfedge(b);

        VertexIndex p0 = builder.triangles[(int)ar];
        VertexIndex pr = builder.triangles[(int)a];
        VertexIndex pl = builder.triangles[(int)al];
        VertexIndex p1 = builder.triangles[(int)bl];

        Vec2f v0 = { positions[(int)p0].x, positions[(int)p0].y };
        Vec2f vr = { positions[(int)pr].x, positions[(int)pr].y };
        Vec2f vl = { positions[(int)pl].x, positions[(int)pl].y };
        Vec2f v1 = { positions[(int)p1].x, positions[(int)p1].y };

        bool illegal = geometry::in_circle_2d(v0, vr, vl, v1) == geometry::PREDICATE_NEGATIVE;
        if (illegal)
        {
            builder.triangles[(int)a] = p1;
            builder.triangles[(int)b] = p0;

            HeIndex hbl = builder.halfedges[(int)bl];

            if (hbl == INVALID_HE)
            {
                VertexIndex e = builder.hull_start;
                while (true)
                {
                    if (builder.hull_tri[(int)e] == bl)
                    {
                        builder.hull_tri[(int)e] = a;
                        break;
                    }

                    e = builder.hull_prev[(int)e];
                    if (e == builder.hull_start) break;
                }
            }

            link(builder, a, hbl);
            link(builder, b, builder.halfedges[(int)ar]);
            link(builder, ar, bl);

            HeIndex br = next_halfedge(b);
            builder.legalize_stack[stack_size] = br;
            stack_size++;
            continue;
        }

        if (stack_size == 0) return ar;
        stack_size--;
        a = builder.legalize_stack[stack_size];
    }
}
```

Important details:

- The input `start_he` is not pushed first. It is processed immediately.
- After an illegal flip, push only `br = next_halfedge(b)` and continue processing the current `a`.
- `legalize()` returns `ar`; it is not `void`.
- If `hbl == INVALID_HE`, the edge `bl` was on the hull. Scan current hull vertices for `hull_tri[e] == bl` and replace it with `a`.
- The in-circle call uses the same argument order as the reference: `(p0, pr, pl, p1)`. With this project's `in_circle_2d`, illegal is `PREDICATE_NEGATIVE` for that order.

### Phase 6 — Mesh finalization

```c3
mesh.half_edges = mem::alloc::new_array(alloc, HalfEdge, (sz)(3 * face_count));
defer catch free(mesh.half_edges);
mesh.faces = mem::alloc::new_array(alloc, HalfEdgeFace, (sz)face_count);
defer catch free(mesh.faces);
mesh.vertices = mem::alloc::new_array(alloc, HalfEdgeVertex, (sz)unique_count);
defer catch free(mesh.vertices);
mesh.positions = mem::alloc::new_array(alloc, Vec3f, (sz)unique_count);
defer catch free(mesh.positions);

for (usz i = 0; i < unique_count; i++)
{
    mesh.positions[i] = working_positions[i];
}

for (usz i = 0; i < unique_count; i++)
{
    mesh.vertices[i].half_edge = INVALID_HE;
}

for (usz i = 0; i < 3 * face_count; i++)
{
    VertexIndex v = triangles[i];
    if (mesh.vertices[(int)v].half_edge == INVALID_HE)
    {
        mesh.vertices[(int)v].half_edge = (HeIndex)(int)i;
    }
}

for (usz i = 0; i < 3 * face_count; i++)
{
    mesh.half_edges[i].origin = triangles[i];
    mesh.half_edges[i].twin = halfedges[i];
    mesh.half_edges[i].face = (FaceIndex)(int)(i / 3);
    if (i % 3 == 2)
    {
        mesh.half_edges[i].next = (HeIndex)(int)(i - 2);
    }
    else
    {
        mesh.half_edges[i].next = (HeIndex)(int)(i + 1);
    }
}

for (usz i = 0; i < face_count; i++)
{
    mesh.faces[i].half_edge = (HeIndex)(int)(3 * i);
}

mesh.normals = {};
mesh.uvs = {};
mesh.validate()!;
```

Free builder arrays and `working_positions` after validation. `defer catch` handles partial-allocation cleanup on faults; successful ownership transfers to the returned `HalfEdgeMesh`.

## Hull details

### Hash key

```c3
fn float pseudo_angle(float dx, float dy)
{
    float abs_dx = dx;
    if (abs_dx < 0.0f)
    {
        abs_dx = -abs_dx;
    }
    float abs_dy = dy;
    if (abs_dy < 0.0f)
    {
        abs_dy = -abs_dy;
    }

    float p = dx / (abs_dx + abs_dy);
    if (dy > 0.0f)
    {
        return (3.0f - p) / 4.0f;
    }
    return (1.0f + p) / 4.0f;
}

fn usz hash_key(DelaunayBuilder* builder, Vec3f p)
{
    float dx = p.x - builder.hull_center.x;
    float dy = p.y - builder.hull_center.y;
    usz hash_len = (usz)builder.hull_hash.len;
    return fast_mod((usz)(pseudo_angle(dx, dy) * (float)hash_len), hash_len);
}

fn void hash_edge(DelaunayBuilder* builder, Vec3f p, VertexIndex vertex)
{
    builder.hull_hash[hash_key(builder, p)] = vertex;
}
```

### Removed vertices

Removed hull vertices are not deleted from `hull_hash`. They must be marked in the linked-list arrays:

```c3
builder.hull_next[(int)removed] = INVALID_VERTEX;
builder.hull_prev[(int)removed] = INVALID_VERTEX;
builder.hull_tri[(int)removed] = INVALID_HE;
```

Hash probes must skip a candidate if `candidate == INVALID_VERTEX`, `hull_next[candidate] == INVALID_VERTEX`, or `hull_next[candidate] == candidate`.

## Predicates

All predicates take `Vec2f`; project from `Vec3f` via `{ pos[i].x, pos[i].y }`.

| Predicate call                 | Use                                                                 |
| ------------------------------ | ------------------------------------------------------------------- |
| `orient_2d(p, e, q)`           | `PREDICATE_POSITIVE` → `e -> q` is visible from insertion point `p` |
| `orient_2d(p, next, q)`        | `PREDICATE_POSITIVE` → keep walking forward                         |
| `orient_2d(p, q, e)`           | `PREDICATE_POSITIVE` → keep walking backward                        |
| `in_circle_2d(p0, pr, pl, p1)` | `PREDICATE_NEGATIVE` → edge illegal, flip                           |

`PREDICATE_ZERO` is treated as not counter-clockwise for hull walks and not illegal for `legalize()`.

## Faults

| Fault              | When                                                                                       |
| ------------------ | ------------------------------------------------------------------------------------------ |
| `EMPTY_INPUT`      | `positions.len == 0`                                                                       |
| `DEGENERATE_INPUT` | fewer than 3 non-collinear distinct points; bounding box excludes points; invalid x/y AABB |

## Memory

| Array                                                      | When freed                                                       |
| ---------------------------------------------------------- | ---------------------------------------------------------------- |
| Builder arrays (`triangles`, `halfedges`, `hull_*`, stack) | After mesh finalization, explicit free                           |
| `working_positions`                                        | After mesh finalization, explicit free                           |
| `VertexIndex[] indices`                                    | After insertion, explicit free                                   |
| `float[] distance_sq`                                      | After insertion, explicit free                                   |
| `HalfEdgeMesh`                                             | Caller via `mesh.destroy()`. `defer catch` on partial allocation |

No allocations in insertion loop. `validate()!` before explicit frees (`defer catch` handles fault cleanup).

## Tests — unchanged (12 cases)

File: `test/test_delaunay_2d.c3`. Existing tests remain valid — API contract identical.

## Non-goals

- No `add_triangle` on `HalfEdgeMesh`
- No super-triangle
- No `from_triangles`
- No dynamic reallocation during insertion
