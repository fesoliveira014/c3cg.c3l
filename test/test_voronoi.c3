module test;
import cg;
import cg::half_edge;
import cg::voronoi;

// ICO_POSITIONS and ICO_INDICES defined in test_dual.c3 (same module).

fn void test_voronoi_from_icosahedron() @test
{
    HalfEdgeMesh ico = half_edge::from_triangles(mem, ICO_POSITIONS[..], ICO_INDICES[..])!!;
    defer ico.destroy();

    VoronoiDiagram diagram = voronoi::from_delaunay(mem, &ico)!!;
    defer diagram.destroy();

    assert(diagram.mesh.faces.len == 12);
    assert(diagram.mesh.positions.len == 20);
    assert(diagram.sites.len == diagram.mesh.faces.len);

    for (usz i = 0; i < diagram.sites.len; i++) {
        assert(diagram.sites[i].x == ico.positions[i].x);
        assert(diagram.sites[i].y == ico.positions[i].y);
        assert(diagram.sites[i].z == ico.positions[i].z);
    }

    for (FaceIndex f = (FaceIndex)0; (int)f < (int)diagram.mesh.faces.len; f++) {
        int deg = diagram.mesh.face_degree(f);
        assert(deg == 5);
    }

    diagram.mesh.validate()!!;
}

fn void test_voronoi_planar_boundary() @test
{
    Vec3f[3] pts = { {0,0,0}, {1,0,0}, {0,1,0} };
    uint[3] idx = {0,1,2};
    HalfEdgeMesh mesh = half_edge::from_triangles(mem, pts[..], idx[..])!!;
    defer mesh.destroy();

    // Planar Delaunay produces unbounded Voronoi — success.
    // Single triangle: all vertices are boundary, dual skips them → 0 faces.
    VoronoiDiagram diagram = voronoi::from_delaunay(mem, &mesh)!!;
    defer diagram.destroy();
    assert(diagram.mesh.faces.len == 0);
}

fn void test_voronoi_roundtrip() @test
{
    HalfEdgeMesh ico = half_edge::from_triangles(mem, ICO_POSITIONS[..], ICO_INDICES[..])!!;
    defer ico.destroy();

    VoronoiDiagram diagram = voronoi::from_delaunay(mem, &ico)!!;
    defer diagram.destroy();

    HalfEdgeMesh delaunay2 = voronoi::to_delaunay(mem, &diagram)!!;
    defer delaunay2.destroy();

    assert(delaunay2.vertices.len == ico.vertices.len);
    assert(delaunay2.faces.len == ico.faces.len);
    assert(delaunay2.half_edges.len == ico.half_edges.len);

    delaunay2.validate()!!;
}
