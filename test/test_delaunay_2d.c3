module test;
import cg;
import cg::delaunay;
import cg::geometry;
import cg::half_edge;

const float DELAUNAY_TEST_EPSILON = 0.001f;

fn bool d2_is_ccw(Vec3f a, Vec3f b, Vec3f c)
{
    Vec2f a2 = { a.x, a.y };
    Vec2f b2 = { b.x, b.y };
    Vec2f c2 = { c.x, c.y };
    return geometry::orient_2d(a2, b2, c2) == geometry::PREDICATE_POSITIVE;
}

fn void assert_all_reverse_edges_are_twinned(HalfEdgeMesh* mesh)
{
    for (usz i = 0; i < mesh.half_edges.len; i++) {
        VertexIndex from_i = mesh.half_edges[i].origin;
        VertexIndex to_i = mesh.half_edges[mesh.half_edges[i].next].origin;
        for (usz j = 0; j < mesh.half_edges.len; j++) {
            if (i == j) { continue; }
            VertexIndex from_j = mesh.half_edges[j].origin;
            VertexIndex to_j = mesh.half_edges[mesh.half_edges[j].next].origin;
            if (from_j == to_i && to_j == from_i) {
                assert(mesh.half_edges[i].twin == (HeIndex)(int)j);
                break;
            }
        }
    }
}

fn bool d2_signs_strictly_opposite(geometry::PredicateSign a, geometry::PredicateSign b)
{
    return (a == geometry::PREDICATE_POSITIVE && b == geometry::PREDICATE_NEGATIVE)
        || (a == geometry::PREDICATE_NEGATIVE && b == geometry::PREDICATE_POSITIVE);
}

fn bool d2_segments_cross_strict(Vec3f a, Vec3f b, Vec3f c, Vec3f d)
{
    Vec2f a2 = { a.x, a.y };
    Vec2f b2 = { b.x, b.y };
    Vec2f c2 = { c.x, c.y };
    Vec2f d2 = { d.x, d.y };
    geometry::PredicateSign o1 = geometry::orient_2d(a2, b2, c2);
    geometry::PredicateSign o2 = geometry::orient_2d(a2, b2, d2);
    geometry::PredicateSign o3 = geometry::orient_2d(c2, d2, a2);
    geometry::PredicateSign o4 = geometry::orient_2d(c2, d2, b2);
    return d2_signs_strictly_opposite(o1, o2) && d2_signs_strictly_opposite(o3, o4);
}

fn bool d2_is_unique_mesh_edge(HalfEdgeMesh* mesh, usz he_index)
{
    HeIndex twin = mesh.half_edges[he_index].twin;
    return twin == cg::INVALID_HE || he_index < (usz)twin;
}

fn bool d2_edges_share_vertex(HalfEdgeMesh* mesh, usz a, usz b)
{
    VertexIndex a0 = mesh.half_edges[a].origin;
    VertexIndex a1 = mesh.half_edges[mesh.half_edges[a].next].origin;
    VertexIndex b0 = mesh.half_edges[b].origin;
    VertexIndex b1 = mesh.half_edges[mesh.half_edges[b].next].origin;
    return a0 == b0 || a0 == b1 || a1 == b0 || a1 == b1;
}

fn int count_crossing_mesh_edges(HalfEdgeMesh* mesh)
{
    int crossings = 0;
    for (usz i = 0; i < mesh.half_edges.len; i++) {
        if (!d2_is_unique_mesh_edge(mesh, i)) { continue; }
        VertexIndex i0 = mesh.half_edges[i].origin;
        VertexIndex i1 = mesh.half_edges[mesh.half_edges[i].next].origin;
        for (usz j = i + 1; j < mesh.half_edges.len; j++) {
            if (!d2_is_unique_mesh_edge(mesh, j)) { continue; }
            if (d2_edges_share_vertex(mesh, i, j)) { continue; }
            VertexIndex j0 = mesh.half_edges[j].origin;
            VertexIndex j1 = mesh.half_edges[mesh.half_edges[j].next].origin;
            if (d2_segments_cross_strict(
                    mesh.positions[(int)i0], mesh.positions[(int)i1],
                    mesh.positions[(int)j0], mesh.positions[(int)j1])) {
                crossings++;
            }
        }
    }
    return crossings;
}

fn void test_delaunay_2d_empty() @test
{
    Vec3f[] pts = {};
    if (catch err = delaunay::delaunay_2d(mem, pts)) {
        assert(err == cg::EMPTY_INPUT);
        return;
    }
    unreachable();
}

fn void test_delaunay_2d_one_point() @test
{
    Vec3f[1] pts = { { 0, 0, 0 } };
    if (catch err = delaunay::delaunay_2d(mem, pts[..])) {
        assert(err == cg::DEGENERATE_INPUT);
        return;
    }
    unreachable();
}

fn void test_delaunay_2d_two_points() @test
{
    Vec3f[2] pts = { { 0, 0, 0 }, { 1, 0, 0 } };
    if (catch err = delaunay::delaunay_2d(mem, pts[..])) {
        assert(err == cg::DEGENERATE_INPUT);
        return;
    }
    unreachable();
}

fn void test_delaunay_2d_collinear() @test
{
    Vec3f[4] pts = { { 0, 0, 0 }, { 1, 0, 0 }, { 2, 0, 0 }, { 3, 0, 0 } };
    if (catch err = delaunay::delaunay_2d(mem, pts[..])) {
        assert(err == cg::DEGENERATE_INPUT);
        return;
    }
    unreachable();
}

fn void test_delaunay_2d_collinear_first_then_valid() @test
{
    Vec3f[4] pts = { { 0, 0, 0 }, { 1, 0, 0 }, { 2, 0, 0 }, { 0, 2, 0 } };
    HalfEdgeMesh mesh = delaunay::delaunay_2d(mem, pts[..])!!;
    defer mesh.destroy();
    mesh.validate()!!;
    assert(mesh.faces.len == 2);
}

fn void test_delaunay_2d_duplicate_xy() @test
{
    Vec3f[4] pts = { { 0, 0, 0 }, { 1, 0, 0 }, { 0, 1, 0 }, { 0, 1, 99 } };
    HalfEdgeMesh mesh = delaunay::delaunay_2d(mem, pts[..])!!;
    defer mesh.destroy();
    mesh.validate()!!;
    assert(mesh.faces.len == 1);
}

fn void test_delaunay_2d_near_duplicate_is_not_output_vertex() @test
{
    Vec3f[4] pts = { { 0, 0, 0 }, { 2, 0, 0 }, { 0, 2, 0 }, { 0, 2.0000005f, 99 } };
    HalfEdgeMesh mesh = delaunay::delaunay_2d(mem, pts[..])!!;
    defer mesh.destroy();
    mesh.validate()!!;
    assert(mesh.faces.len == 1);
    assert(mesh.positions.len == 3);
}

fn void test_delaunay_2d_triangle() @test
{
    Vec3f[3] pts = { { 0, 0, 0 }, { 2, 0, 0 }, { 0, 2, 0 } };
    HalfEdgeMesh mesh = delaunay::delaunay_2d(mem, pts[..])!!;
    defer mesh.destroy();
    mesh.validate()!!;
    assert(mesh.faces.len == 1);
}

fn void test_delaunay_2d_square() @test
{
    Vec3f[4] pts = { { 0, 0, 0 }, { 1, 0, 0 }, { 1, 1, 0 }, { 0, 1, 0 } };
    HalfEdgeMesh mesh = delaunay::delaunay_2d(mem, pts[..])!!;
    defer mesh.destroy();
    mesh.validate()!!;
    assert(mesh.faces.len == 2);
}

fn void test_delaunay_2d_square_delaunay_property() @test
{
    Vec3f[4] pts = { { 0, 0, 0 }, { 1, 0, 0 }, { 1, 1, 0 }, { 0, 1, 0 } };
    HalfEdgeMesh mesh = delaunay::delaunay_2d(mem, pts[..])!!;
    defer mesh.destroy();

    VertexIndex[3] verts;
    for (FaceIndex f = (FaceIndex)0; (int)f < (int)mesh.faces.len; f++) {
        mesh.face_vertices(f, verts[..])!!;
        Vec3f a = mesh.positions[(int)verts[0]];
        Vec3f b = mesh.positions[(int)verts[1]];
        Vec3f c = mesh.positions[(int)verts[2]];
        assert(d2_is_ccw(a, b, c));
        for (usz j = 0; j < pts.len; j++) {
            // Skip if pts[j] is one of the triangle's vertices (match by x,y).
            Vec2f pj = { pts[j].x, pts[j].y };
            Vec2f va2 = { a.x, a.y };
            Vec2f vb2 = { b.x, b.y };
            Vec2f vc2 = { c.x, c.y };
            if ((pj.x == va2.x && pj.y == va2.y)
                || (pj.x == vb2.x && pj.y == vb2.y)
                || (pj.x == vc2.x && pj.y == vc2.y)) continue;
            assert(geometry::in_circle_2d(va2, vb2, vc2, pj) != geometry::PREDICATE_POSITIVE);
        }
    }
}

fn void test_delaunay_2d_square_center() @test
{
    Vec3f[5] pts = { { 0, 0, 0 }, { 1, 0, 0 }, { 1, 1, 0 }, { 0, 1, 0 }, { 0.5, 0.5, 0 } };
    HalfEdgeMesh mesh = delaunay::delaunay_2d(mem, pts[..])!!;
    defer mesh.destroy();
    mesh.validate()!!;
    assert(mesh.faces.len == 4);
}

fn void test_delaunay_2d_grid() @test
{
    Vec3f[9] pts;
    for (int i = 0; i < 9; i++) pts[i] = { (float)(i % 3), (float)(i / 3), 0 };
    HalfEdgeMesh mesh = delaunay::delaunay_2d(mem, pts[..])!!;
    defer mesh.destroy();
    mesh.validate()!!;
    assert(mesh.faces.len >= 8);
}

fn void test_delaunay_2d_grid_pairs_all_interior_twins() @test
{
    Vec3f[9] pts;
    for (int i = 0; i < 9; i++) pts[i] = { (float)(i % 3), (float)(i / 3), 0 };
    HalfEdgeMesh mesh = delaunay::delaunay_2d(mem, pts[..])!!;
    defer mesh.destroy();
    mesh.validate()!!;
    assert_all_reverse_edges_are_twinned(&mesh);
}

fn void test_delaunay_2d_random_regression_n8() @test
{
    Vec3f[8] pts;
    for (int i = 0; i < 8; i++) {
        pts[i] = {
            (float)((int)((uint)i * 2654435761u % 1000000u)) * 0.001f,
            (float)((int)((uint)i * 1737350767u % 1000000u)) * 0.001f,
            0,
        };
    }
    HalfEdgeMesh mesh = delaunay::delaunay_2d(mem, pts[..])!!;
    defer mesh.destroy();
    mesh.validate()!!;
}

fn void test_delaunay_2d_random_prefixes_validate() @test
{
    for (int n = 3; n <= 128; n++) {
        Vec3f[] pts = mem::alloc::new_array(mem, Vec3f, (sz)n);
        for (int i = 0; i < n; i++) {
            pts[i] = {
                (float)((int)((uint)i * 2654435761u % 1000000u)) * 0.001f,
                (float)((int)((uint)i * 1737350767u % 1000000u)) * 0.001f,
                0,
            };
        }
        HalfEdgeMesh mesh = delaunay::delaunay_2d(mem, pts[..])!!;
        mesh.validate()!!;
        mesh.destroy();
        free(pts);
    }
}

fn void test_delaunay_2d_random_hundred_has_no_crossing_edges() @test
{
    Vec3f[100] pts;
    uint state = 42;
    for (usz i = 0; i < pts.len; i++) {
        state = state * 1103515245 + 12345;
        float x = (float)(state % 10000) / 10000.0f;
        state = state * 1103515245 + 12345;
        float y = (float)(state % 10000) / 10000.0f;
        pts[i] = { x, y, 0 };
    }

    HalfEdgeMesh mesh = delaunay::delaunay_2d(mem, pts[..])!!;
    defer mesh.destroy();
    mesh.validate()!!;
    assert(count_crossing_mesh_edges(&mesh) == 0);
}

fn void test_delaunay_2d_cocircular() @test
{
    Vec3f[4] pts = { { 0, 0, 0 }, { 2, 0, 0 }, { 2, 2, 0 }, { 0, 2, 0 } };
    HalfEdgeMesh mesh = delaunay::delaunay_2d(mem, pts[..])!!;
    defer mesh.destroy();
    mesh.validate()!!;
    assert(mesh.faces.len == 2);
}

fn void test_delaunay_2d_bounding_box_option() @test
{
    Vec3f[4] pts = { { 1, 1, 0 }, { 2, 1, 0 }, { 2, 2, 0 }, { 1, 2, 0 } };
    Aabb bbox = { { 0, 0, 0 }, { 3, 3, 3 } };
    DelaunayOptions opts = { .has_bounding_box = true, .bounding_box = bbox };
    HalfEdgeMesh mesh = delaunay::delaunay_2d(mem, pts[..], opts)!!;
    defer mesh.destroy();
    mesh.validate()!!;
    assert(mesh.faces.len == 2);
}
