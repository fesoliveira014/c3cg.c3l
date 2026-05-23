module test;
import cg;
import cg::half_edge;
import cg::subdivide;

fn void test_subdivide_single_triangle_boundary() @test
{
    Vec3f[3] tri = { {0,0,0}, {1,0,0}, {0,1,0} };
    uint[3] idx = { 0, 1, 2 };
    HalfEdgeMesh mesh = half_edge::from_triangles(mem, tri[..], idx[..])!!;
    defer mesh.destroy();

    if (catch err = subdivide::loop_subdivide(mem, &mesh)) {
        assert(err == cg::BOUNDARY_HALF_EDGE);
        return;
    }
    unreachable();
}

fn void test_subdivide_quad_mesh_non_triangle() @test
{
    Vec3f[4] quad = { {0,0,0}, {1,0,0}, {1,1,0}, {0,1,0} };
    uint[2] offsets = { 0, 4 };
    uint[4] face_idx = { 0, 1, 2, 3 };
    HalfEdgeMesh mesh = half_edge::from_polygons(mem, quad[..], offsets[..], face_idx[..])!!;
    defer mesh.destroy();

    if (catch err = subdivide::loop_subdivide(mem, &mesh)) {
        assert(err == cg::NON_TRIANGLE_FACE);
        return;
    }
    unreachable();
}

fn void test_subdivide_tetrahedron() @test
{
    HalfEdgeMesh tet = half_edge::from_triangles(mem, TET_POSITIONS[..], TET_INDICES[..])!!;
    defer tet.destroy();

    HalfEdgeMesh sub = subdivide::loop_subdivide(mem, &tet)!!;
    defer sub.destroy();

    assert(sub.faces.len == 16);
    assert(sub.vertices.len == 10);
    sub.validate()!!;
}
