module test;
import cg;
import cg::half_edge;

struct M4HalfEdgeSnapshot {
    VertexIndex origin;
    HeIndex   next;
    HeIndex   twin;
    FaceIndex face;
}

fn M4HalfEdgeSnapshot m4_snapshot_half_edge(HalfEdgeMesh* mesh, HeIndex he)
{
    M4HalfEdgeSnapshot snapshot;
    snapshot.origin = mesh.half_edges[he].origin;
    snapshot.next   = mesh.half_edges[he].next;
    snapshot.twin   = mesh.half_edges[he].twin;
    snapshot.face   = mesh.half_edges[he].face;
    return snapshot;
}

fn void m4_assert_half_edge_snapshot(HalfEdgeMesh* mesh, HeIndex he, M4HalfEdgeSnapshot snapshot)
{
    assert(mesh.half_edges[he].origin == snapshot.origin);
    assert(mesh.half_edges[he].next   == snapshot.next);
    assert(mesh.half_edges[he].twin   == snapshot.twin);
    assert(mesh.half_edges[he].face   == snapshot.face);
}

fn void test_flip_rejects_invalid_half_edge() @test
{
    HalfEdgeMesh mesh = half_edge::from_triangles(mem, DIAMOND_POSITIONS[..], DIAMOND_INDICES[..])!!;
    defer mesh.destroy();

    assert(!mesh.is_flip_ok(cg::INVALID_HE));
    if (catch err = mesh.flip(cg::INVALID_HE)) {
        assert(err == cg::INVALID_HALF_EDGE_REFERENCE);
    } else {
        assert(false);
    }

    HeIndex out_of_range = (HeIndex)(int)mesh.half_edges.len;
    assert(!mesh.is_flip_ok(out_of_range));
    if (catch err = mesh.flip(out_of_range)) {
        assert(err == cg::INVALID_HALF_EDGE_REFERENCE);
    } else {
        assert(false);
    }
}

fn void test_flip_boundary_faults() @test
{
    HalfEdgeMesh mesh = half_edge::from_triangles(mem, TRI_POSITIONS[..], TRI_INDICES[..])!!;
    defer mesh.destroy();

    HeIndex boundary_he = (HeIndex)0;
    HeIndex old_next = mesh.half_edges[boundary_he].next;
    VertexIndex old_origin = mesh.half_edges[boundary_he].origin;
    FaceIndex old_face = mesh.half_edges[boundary_he].face;

    assert(!mesh.is_flip_ok(boundary_he));
    if (catch err = mesh.flip(boundary_he)) {
        assert(err == cg::BOUNDARY_HALF_EDGE);
    } else {
        assert(false);
    }

    assert(mesh.half_edges[boundary_he].next == old_next);
    assert(mesh.half_edges[boundary_he].origin == old_origin);
    assert(mesh.half_edges[boundary_he].face == old_face);
    mesh.validate()!!;
}

fn bool m4_edge_matches(HalfEdgeMesh* mesh, HeIndex he, VertexIndex from, VertexIndex to)
{
    return mesh.from_vertex(he) == from && mesh.to_vertex(he) == to;
}

fn bool m4_undirected_edge_matches(HalfEdgeMesh* mesh, HeIndex he, VertexIndex a, VertexIndex b)
{
    VertexIndex from = mesh.from_vertex(he);
    VertexIndex to = mesh.to_vertex(he);
    return (from == a && to == b) || (from == b && to == a);
}

fn bool m4_face_has_vertices(HalfEdgeMesh* mesh, FaceIndex face, VertexIndex a, VertexIndex b, VertexIndex c)
{
    VertexIndex[3] verts;
    int count = mesh.face_vertices(face, verts[..])!!;
    if (count != 3) return false;

    bool has_a = false;
    bool has_b = false;
    bool has_c = false;
    for (usz i = 0; i < 3; i++) {
        if (verts[i] == a) has_a = true;
        if (verts[i] == b) has_b = true;
        if (verts[i] == c) has_c = true;
    }
    return has_a && has_b && has_c;
}

fn bool m4_mesh_has_face_vertices(HalfEdgeMesh* mesh, VertexIndex a, VertexIndex b, VertexIndex c)
{
    for (usz i = 0; i < mesh.faces.len; i++) {
        if (m4_face_has_vertices(mesh, (FaceIndex)(int)i, a, b, c)) return true;
    }
    return false;
}

fn bool m4_mesh_has_undirected_edge(HalfEdgeMesh* mesh, VertexIndex a, VertexIndex b)
{
    for (usz i = 0; i < mesh.half_edges.len; i++) {
        if (m4_undirected_edge_matches(mesh, (HeIndex)(int)i, a, b)) return true;
    }
    return false;
}

fn void test_flip_ok_on_interior_diamond() @test
{
    HalfEdgeMesh mesh = half_edge::from_triangles(mem, DIAMOND_POSITIONS[..], DIAMOND_INDICES[..])!!;
    defer mesh.destroy();

    assert(mesh.twin((HeIndex)1) == 3);
    assert(mesh.is_flip_ok((HeIndex)1));
}

fn void test_flip_rewrites_diamond_diagonal() @test
{
    HalfEdgeMesh mesh = half_edge::from_triangles(mem, DIAMOND_POSITIONS[..], DIAMOND_INDICES[..])!!;
    defer mesh.destroy();

    mesh.flip((HeIndex)1)!!;

    mesh.validate()!!;
    assert(mesh.face_degree((FaceIndex)0) == 3);
    assert(mesh.face_degree((FaceIndex)1) == 3);
    assert(m4_undirected_edge_matches(&mesh, (HeIndex)1, (VertexIndex)0, (VertexIndex)3));
    assert(m4_undirected_edge_matches(&mesh, mesh.twin((HeIndex)1), (VertexIndex)0, (VertexIndex)3));
    assert(!m4_undirected_edge_matches(&mesh, (HeIndex)1, (VertexIndex)1, (VertexIndex)2));
}

fn void test_flip_refreshes_local_vertex_handles() @test
{
    HalfEdgeMesh mesh = half_edge::from_triangles(mem, DIAMOND_POSITIONS[..], DIAMOND_INDICES[..])!!;
    defer mesh.destroy();

    mesh.flip((HeIndex)1)!!;

    VertexIndex[4] vertices = { (VertexIndex)0, (VertexIndex)1, (VertexIndex)2, (VertexIndex)3 };
    foreach (v : vertices) {
        HeIndex handle = mesh.vertices[v].half_edge;
        assert(handle != cg::INVALID_HE);
        assert(mesh.half_edges[handle].origin == v);
    }
    mesh.validate()!!;
}

fn void test_flip_back_restores_original_topology_equivalence() @test
{
    HalfEdgeMesh mesh = half_edge::from_triangles(mem, DIAMOND_POSITIONS[..], DIAMOND_INDICES[..])!!;
    defer mesh.destroy();

    assert(m4_mesh_has_face_vertices(&mesh, (VertexIndex)0, (VertexIndex)1, (VertexIndex)2));
    assert(m4_mesh_has_face_vertices(&mesh, (VertexIndex)1, (VertexIndex)2, (VertexIndex)3));
    assert(m4_mesh_has_undirected_edge(&mesh, (VertexIndex)1, (VertexIndex)2));

    mesh.flip((HeIndex)1)!!;
    mesh.flip((HeIndex)1)!!;

    mesh.validate()!!;
    assert(m4_mesh_has_face_vertices(&mesh, (VertexIndex)0, (VertexIndex)1, (VertexIndex)2));
    assert(m4_mesh_has_face_vertices(&mesh, (VertexIndex)1, (VertexIndex)2, (VertexIndex)3));
    assert(m4_mesh_has_undirected_edge(&mesh, (VertexIndex)1, (VertexIndex)2));
}

fn void test_flip_preserves_euler_counts() @test
{
    HalfEdgeMesh mesh = half_edge::from_triangles(mem, DIAMOND_POSITIONS[..], DIAMOND_INDICES[..])!!;
    defer mesh.destroy();

    usz vertex_count = mesh.vertices.len;
    usz half_edge_count = mesh.half_edges.len;
    usz face_count = mesh.faces.len;

    mesh.flip((HeIndex)1)!!;

    assert(mesh.vertices.len == vertex_count);
    assert(mesh.half_edges.len == half_edge_count);
    assert(mesh.faces.len == face_count);
    mesh.validate()!!;
}

fn void test_flip_preserves_counts_and_attributes() @test
{
    HalfEdgeMesh mesh = half_edge::from_triangles_with_attrs(
        mem, DIAMOND_POSITIONS[..], TET_NORMALS[..], TET_UVS[..], DIAMOND_INDICES[..])!!;
    defer mesh.destroy();

    usz positions_len = mesh.positions.len;
    usz normals_len = mesh.normals.len;
    usz uvs_len = mesh.uvs.len;
    Vec3f position_0 = mesh.positions[0];
    Vec3f normal_0 = mesh.normals[0];
    Vec2f uv_0 = mesh.uvs[0];

    mesh.flip((HeIndex)1)!!;

    assert(mesh.positions.len == positions_len);
    assert(mesh.normals.len == normals_len);
    assert(mesh.uvs.len == uvs_len);
    assert(mesh.positions[0] == position_0);
    assert(mesh.normals[0] == normal_0);
    assert(mesh.uvs[0] == uv_0);
    mesh.validate()!!;
}

fn void test_flip_rejects_non_triangle_face() @test
{
    Vec3f[5] positions = {
        { 0, 0, 0 },
        { 1, 0, 0 },
        { 0, 1, 0 },
        { 1, 1, 0 },
        { 2, 0, 0 },
    };
    uint[3] offsets = { 0, 3, 7 };
    uint[7] indices = { 0, 1, 2, 1, 0, 3, 4 };
    HalfEdgeMesh mesh = half_edge::from_polygons(mem, positions[..], offsets[..], indices[..])!!;
    defer mesh.destroy();
    mesh.validate()!!;

    assert(mesh.twin((HeIndex)0) == 3);
    M4HalfEdgeSnapshot before = m4_snapshot_half_edge(&mesh, (HeIndex)0);
    assert(!mesh.is_flip_ok((HeIndex)0));
    if (catch err = mesh.flip((HeIndex)0)) {
        assert(err == cg::NON_TRIANGLE_FACE);
    } else {
        assert(false);
    }
    m4_assert_half_edge_snapshot(&mesh, (HeIndex)0, before);
    mesh.validate()!!;
}

fn void test_flip_rejects_degenerate_output_endpoints() @test
{
    Vec3f[3] positions = {
        { 0, 0, 0 },
        { 1, 0, 0 },
        { 0, 1, 0 },
    };
    uint[6] indices = { 0, 1, 2, 1, 0, 2 };
    HalfEdgeMesh mesh = half_edge::from_triangles(mem, positions[..], indices[..])!!;
    defer mesh.destroy();
    mesh.validate()!!;

    assert(mesh.twin((HeIndex)0) == 3);
    M4HalfEdgeSnapshot before = m4_snapshot_half_edge(&mesh, (HeIndex)0);
    assert(!mesh.is_flip_ok((HeIndex)0));
    if (catch err = mesh.flip((HeIndex)0)) {
        assert(err == cg::DEGENERATE_INPUT);
    } else {
        assert(false);
    }
    m4_assert_half_edge_snapshot(&mesh, (HeIndex)0, before);
    mesh.validate()!!;
}

fn void test_flip_rejects_duplicate_new_edge() @test
{
    HalfEdgeMesh mesh = half_edge::from_triangles(mem, TET_POSITIONS[..], TET_INDICES[..])!!;
    defer mesh.destroy();
    mesh.validate()!!;

    HeIndex he = (HeIndex)0;
    assert(mesh.twin(he) == 5);
    assert(m4_mesh_has_undirected_edge(&mesh, (VertexIndex)2, (VertexIndex)3));

    M4HalfEdgeSnapshot before = m4_snapshot_half_edge(&mesh, he);

    assert(!mesh.is_flip_ok(he));
    if (catch err = mesh.flip(he)) {
        assert(err == cg::EDGE_ALREADY_EXISTS);
    } else {
        assert(false);
    }

    m4_assert_half_edge_snapshot(&mesh, he, before);
    mesh.validate()!!;
}
