module test;
import cg;
import cg::half_edge;

fn void test_face_half_edges_triangle_cycle() @test
{
    HalfEdgeMesh mesh = half_edge::from_triangles(mem, TET_POSITIONS[..], TET_INDICES[..])!!;
    defer mesh.destroy();

    HeIndex[3] out;
    int count = mesh.face_half_edges((FaceIndex)0, out[..])!!;

    assert(count == 3);
    assert(out[0] == mesh.faces[0].half_edge);
    assert(out[1] == mesh.next(out[0]));
    assert(out[2] == mesh.next(out[1]));
    assert(mesh.next(out[2]) == out[0]);
}

fn void test_face_vertices_triangle_cycle() @test
{
    HalfEdgeMesh mesh = half_edge::from_triangles(mem, TET_POSITIONS[..], TET_INDICES[..])!!;
    defer mesh.destroy();

    VertexIndex[3] out;
    int count = mesh.face_vertices((FaceIndex)0, out[..])!!;

    assert(count == 3);
    assert(out[0] == mesh.from_vertex(mesh.faces[0].half_edge));
    assert(out[1] == mesh.to_vertex(mesh.faces[0].half_edge));
    assert(out[2] == mesh.to_vertex(mesh.next(mesh.faces[0].half_edge)));
}

fn void test_face_vertices_quad_cycle() @test
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

    VertexIndex[4] out;
    int count = mesh.face_vertices((FaceIndex)0, out[..])!!;

    assert(count == 4);
    assert(out[0] == 0);
    assert(out[1] == 1);
    assert(out[2] == 2);
    assert(out[3] == 3);
}

fn void test_face_walk_output_buffer_too_small() @test
{
    HalfEdgeMesh mesh = half_edge::from_triangles(mem, TET_POSITIONS[..], TET_INDICES[..])!!;
    defer mesh.destroy();

    HeIndex[2] half_edges;
    VertexIndex[2] vertices;

    if (catch err = mesh.face_half_edges((FaceIndex)0, half_edges[..])) {
        assert(err == cg::OUTPUT_BUFFER_TOO_SMALL);
    } else {
        unreachable();
    }

    if (catch err = mesh.face_vertices((FaceIndex)0, vertices[..])) {
        assert(err == cg::OUTPUT_BUFFER_TOO_SMALL);
    } else {
        unreachable();
    }
}

// ---------------------------------------------------------------------------
// Vertex one-ring tests
// ---------------------------------------------------------------------------

fn void test_tetrahedron_vertex_one_ring_outgoing_degree_three() @test
{
    HalfEdgeMesh mesh = half_edge::from_triangles(mem, TET_POSITIONS[..], TET_INDICES[..])!!;
    defer mesh.destroy();

    for (usz v = 0; v < mesh.vertices.len; v++) {
        HeIndex[3] out;
        int count = mesh.vertex_one_ring_outgoing((VertexIndex)(int)v, out[..])!!;
        assert(count == 3);
        for (int i = 0; i < count; i++) {
            assert(out[i] != cg::INVALID_HE);
            assert(mesh.from_vertex(out[i]) == (VertexIndex)(int)v);
        }
    }
}

fn void test_tetrahedron_vertex_one_ring_faces_degree_three() @test
{
    HalfEdgeMesh mesh = half_edge::from_triangles(mem, TET_POSITIONS[..], TET_INDICES[..])!!;
    defer mesh.destroy();

    for (usz v = 0; v < mesh.vertices.len; v++) {
        FaceIndex[3] out;
        int count = mesh.vertex_one_ring_faces((VertexIndex)(int)v, out[..])!!;
        assert(count == 3);
        for (int i = 0; i < count; i++) {
            assert(out[i] != cg::INVALID_FACE);
        }
    }
}

// Closed bipyramid-like mesh with vertex 0 having degree 5
const Vec3f[7] DEGREE_FIVE_POSITIONS = {
    { 0, 0, 1 },
    { 1, 0, 0 },
    { 0.309016, 0.951057, 0 },
    { -0.809017, 0.587785, 0 },
    { -0.809017, -0.587785, 0 },
    { 0.309016, -0.951057, 0 },
    { 0, 0, -1 },
};

const uint[30] DEGREE_FIVE_INDICES = {
    0, 1, 2,
    0, 2, 3,
    0, 3, 4,
    0, 4, 5,
    0, 5, 1,
    6, 2, 1,
    6, 3, 2,
    6, 4, 3,
    6, 5, 4,
    6, 1, 5,
};

fn void test_degree_five_vertex_one_ring() @test
{
    HalfEdgeMesh mesh = half_edge::from_triangles(mem, DEGREE_FIVE_POSITIONS[..], DEGREE_FIVE_INDICES[..])!!;
    defer mesh.destroy();

    HeIndex[5] outgoing;
    FaceIndex[5] faces;

    int outgoing_count = mesh.vertex_one_ring_outgoing((VertexIndex)0, outgoing[..])!!;
    int face_count = mesh.vertex_one_ring_faces((VertexIndex)0, faces[..])!!;

    assert(outgoing_count == 5);
    assert(face_count == 5);
    for (int i = 0; i < outgoing_count; i++) {
        assert(mesh.from_vertex(outgoing[i]) == 0);
    }
}

fn void test_boundary_vertex_walk_stops_cleanly() @test
{
    HalfEdgeMesh mesh = half_edge::from_triangles(mem, TRI_POSITIONS[..], TRI_INDICES[..])!!;
    defer mesh.destroy();

    HeIndex[3] out;
    int count = mesh.vertex_one_ring_outgoing((VertexIndex)0, out[..])!!;

    assert(count >= 1);
    assert(count <= 3);
    assert(out[0] == mesh.vertices[0].half_edge);
    for (int i = 0; i < count; i++) {
        assert(mesh.from_vertex(out[i]) == 0);
    }
}

fn void test_boundary_vertex_one_ring_faces() @test
{
    HalfEdgeMesh mesh = half_edge::from_triangles(mem, TRI_POSITIONS[..], TRI_INDICES[..])!!;
    defer mesh.destroy();

    FaceIndex[3] faces;
    int count = mesh.vertex_one_ring_faces((VertexIndex)0, faces[..])!!;

    assert(count >= 1);
    assert(count <= 3);
    for (int i = 0; i < count; i++) {
        assert(faces[i] != cg::INVALID_FACE);
    }
}

fn void test_isolated_vertex_one_ring_zero() @test
{
    HalfEdgeMesh mesh = half_edge::from_triangles(mem, TRI_POSITIONS[..], TRI_INDICES[..])!!;
    defer mesh.destroy();

    mesh.vertices[0].half_edge = cg::INVALID_HE;

    HeIndex[1] outgoing;
    FaceIndex[1] faces;
    assert(mesh.vertex_one_ring_outgoing((VertexIndex)0, outgoing[..])!! == 0);
    assert(mesh.vertex_one_ring_faces((VertexIndex)0, faces[..])!! == 0);
}

fn void test_vertex_one_ring_deterministic() @test
{
    HalfEdgeMesh mesh = half_edge::from_triangles(mem, TET_POSITIONS[..], TET_INDICES[..])!!;
    defer mesh.destroy();

    HeIndex[3] first;
    HeIndex[3] second;
    int first_count = mesh.vertex_one_ring_outgoing((VertexIndex)0, first[..])!!;
    int second_count = mesh.vertex_one_ring_outgoing((VertexIndex)0, second[..])!!;

    assert(first_count == second_count);
    for (int i = 0; i < first_count; i++) {
        assert(first[i] == second[i]);
    }
}

fn void test_vertex_walk_output_buffer_too_small() @test
{
    HalfEdgeMesh mesh = half_edge::from_triangles(mem, TET_POSITIONS[..], TET_INDICES[..])!!;
    defer mesh.destroy();

    HeIndex[2] outgoing;
    FaceIndex[2] faces;

    if (catch err = mesh.vertex_one_ring_outgoing((VertexIndex)0, outgoing[..])) {
        assert(err == cg::OUTPUT_BUFFER_TOO_SMALL);
    } else {
        unreachable();
    }

    if (catch err = mesh.vertex_one_ring_faces((VertexIndex)0, faces[..])) {
        assert(err == cg::OUTPUT_BUFFER_TOO_SMALL);
    } else {
        unreachable();
    }
}

// ---------------------------------------------------------------------------
// Validation tests
// ---------------------------------------------------------------------------

fn void test_validate_tetrahedron() @test
{
    HalfEdgeMesh mesh = half_edge::from_triangles(mem, TET_POSITIONS[..], TET_INDICES[..])!!;
    defer mesh.destroy();
    mesh.validate()!!;
}

fn void test_validate_single_triangle_boundary() @test
{
    HalfEdgeMesh mesh = half_edge::from_triangles(mem, TRI_POSITIONS[..], TRI_INDICES[..])!!;
    defer mesh.destroy();
    mesh.validate()!!;
}

fn void test_validate_polygon_quad() @test
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
    mesh.validate()!!;
}

fn void test_validate_empty_topology_fault() @test
{
    HalfEdgeMesh mesh;
    if (catch err = mesh.validate()) {
        assert(err == cg::INVALID_TOPOLOGY);
    } else {
        unreachable();
    }
}

fn void test_validate_bad_origin_fault() @test
{
    HalfEdgeMesh mesh = half_edge::from_triangles(mem, TRI_POSITIONS[..], TRI_INDICES[..])!!;
    defer mesh.destroy();
    mesh.half_edges[0].origin = (VertexIndex)99;
    if (catch err = mesh.validate()) {
        assert(err == cg::INVALID_VERTEX_REFERENCE);
    } else {
        unreachable();
    }
}

fn void test_validate_bad_next_fault() @test
{
    HalfEdgeMesh mesh = half_edge::from_triangles(mem, TRI_POSITIONS[..], TRI_INDICES[..])!!;
    defer mesh.destroy();
    mesh.half_edges[0].next = (HeIndex)99;
    if (catch err = mesh.validate()) {
        assert(err == cg::INVALID_HALF_EDGE_REFERENCE);
    } else {
        unreachable();
    }
}

fn void test_validate_bad_face_reference_fault() @test
{
    HalfEdgeMesh mesh = half_edge::from_triangles(mem, TRI_POSITIONS[..], TRI_INDICES[..])!!;
    defer mesh.destroy();
    mesh.half_edges[0].face = (FaceIndex)99;
    if (catch err = mesh.validate()) {
        assert(err == cg::INVALID_FACE_REFERENCE);
    } else {
        unreachable();
    }
}

fn void test_validate_bad_face_half_edge_fault() @test
{
    HalfEdgeMesh mesh = half_edge::from_triangles(mem, TRI_POSITIONS[..], TRI_INDICES[..])!!;
    defer mesh.destroy();
    mesh.faces[0].half_edge = (HeIndex)99;
    if (catch err = mesh.validate()) {
        assert(err == cg::INVALID_HALF_EDGE_REFERENCE);
    } else {
        unreachable();
    }
}

fn void test_validate_bad_vertex_half_edge_fault() @test
{
    HalfEdgeMesh mesh = half_edge::from_triangles(mem, TRI_POSITIONS[..], TRI_INDICES[..])!!;
    defer mesh.destroy();
    mesh.vertices[0].half_edge = (HeIndex)99;
    if (catch err = mesh.validate()) {
        assert(err == cg::INVALID_HALF_EDGE_REFERENCE);
    } else {
        unreachable();
    }
}

fn void test_validate_negative_next_fault() @test
{
    HalfEdgeMesh mesh = half_edge::from_triangles(mem, TRI_POSITIONS[..], TRI_INDICES[..])!!;
    defer mesh.destroy();
    mesh.half_edges[0].next = cg::INVALID_HE;
    if (catch err = mesh.validate()) {
        assert(err == cg::INVALID_HALF_EDGE_REFERENCE);
    } else {
        unreachable();
    }
}

fn void test_validate_negative_origin_fault() @test
{
    HalfEdgeMesh mesh = half_edge::from_triangles(mem, TRI_POSITIONS[..], TRI_INDICES[..])!!;
    defer mesh.destroy();
    mesh.half_edges[0].origin = cg::INVALID_VERTEX;
    if (catch err = mesh.validate()) {
        assert(err == cg::INVALID_VERTEX_REFERENCE);
    } else {
        unreachable();
    }
}

// ---------------------------------------------------------------------------
// Twin validation tests
// ---------------------------------------------------------------------------

fn void test_validate_bad_twin_roundtrip_fault() @test
{
    HalfEdgeMesh mesh = half_edge::from_triangles(mem, TET_POSITIONS[..], TET_INDICES[..])!!;
    defer mesh.destroy();

    HeIndex twin = mesh.half_edges[0].twin;
    mesh.half_edges[twin].twin = (HeIndex)1;

    if (catch err = mesh.validate()) {
        assert(err == cg::INVALID_TWIN);
    } else {
        unreachable();
    }
}

fn void test_validate_bad_twin_endpoints_fault() @test
{
    HalfEdgeMesh mesh = half_edge::from_triangles(mem, TET_POSITIONS[..], TET_INDICES[..])!!;
    defer mesh.destroy();

    HeIndex twin = mesh.half_edges[0].twin;
    mesh.half_edges[twin].next = mesh.half_edges[0].next;

    if (catch err = mesh.validate()) {
        assert(err == cg::INVALID_TWIN);
    } else {
        unreachable();
    }
}

// ---------------------------------------------------------------------------
// Face-cycle validation tests
// ---------------------------------------------------------------------------

fn void test_validate_bad_face_cycle_fault() @test
{
    HalfEdgeMesh mesh = half_edge::from_triangles(mem, TET_POSITIONS[..], TET_INDICES[..])!!;
    defer mesh.destroy();

    mesh.half_edges[1].face = (FaceIndex)1;

    if (catch err = mesh.validate()) {
        assert(err == cg::INVALID_FACE_CYCLE);
    } else {
        unreachable();
    }
}

fn void test_validate_non_closing_face_cycle_fault() @test
{
    HalfEdgeMesh mesh = half_edge::from_triangles(mem, TRI_POSITIONS[..], TRI_INDICES[..])!!;
    defer mesh.destroy();

    mesh.half_edges[2].next = (HeIndex)1;

    if (catch err = mesh.validate()) {
        assert(err == cg::INVALID_FACE_CYCLE);
    } else {
        unreachable();
    }
}

fn void test_validate_degree_too_small_fault() @test
{
    HalfEdgeMesh mesh = half_edge::from_triangles(mem, TRI_POSITIONS[..], TRI_INDICES[..])!!;
    defer mesh.destroy();

    mesh.half_edges[0].next = (HeIndex)0;

    if (catch err = mesh.validate()) {
        assert(err == cg::INVALID_FACE_CYCLE);
    } else {
        unreachable();
    }
}

fn void test_validate_bad_vertex_canonical_fault() @test
{
    HalfEdgeMesh mesh = half_edge::from_triangles(mem, TET_POSITIONS[..], TET_INDICES[..])!!;
    defer mesh.destroy();

    assert(mesh.half_edges[1].origin != 0);
    mesh.vertices[0].half_edge = (HeIndex)1;

    if (catch err = mesh.validate()) {
        assert(err == cg::INVALID_VERTEX_REFERENCE);
    } else {
        unreachable();
    }
}

fn void test_validate_attribute_mismatch_fault() @test
{
    HalfEdgeMesh mesh = half_edge::from_triangles_with_attrs(
        mem, TET_POSITIONS[..], TET_NORMALS[..], TET_UVS[..], TET_INDICES[..])!!;
    defer mesh.destroy();

    Vec3f[] original_normals = mesh.normals;
    mesh.normals = original_normals[0..2];
    defer mesh.normals = original_normals;

    if (catch err = mesh.validate()) {
        assert(err == cg::ATTRIBUTE_COUNT_MISMATCH);
    } else {
        unreachable();
    }
}
