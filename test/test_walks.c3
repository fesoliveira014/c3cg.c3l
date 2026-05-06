module test;
import cg;
import cg::half_edge;

fn void test_face_half_edges_triangle_cycle() @test
{
    HalfEdgeMesh mesh = half_edge::from_triangles(mem, TET_POSITIONS[..], TET_INDICES[..])!!;
    defer mesh.destroy();

    HeIndex[3] out;
    int count = mesh.face_half_edges(0, out[..])!!;

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
    int count = mesh.face_vertices(0, out[..])!!;

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
    int count = mesh.face_vertices(0, out[..])!!;

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

    if (catch err = mesh.face_half_edges(0, half_edges[..])) {
        assert(err == cg::OUTPUT_BUFFER_TOO_SMALL);
    } else {
        unreachable();
    }

    if (catch err = mesh.face_vertices(0, vertices[..])) {
        assert(err == cg::OUTPUT_BUFFER_TOO_SMALL);
    } else {
        unreachable();
    }
}
