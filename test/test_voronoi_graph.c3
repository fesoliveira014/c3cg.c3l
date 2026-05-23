module test;
import cg;
import cg::voronoi;
import cg::graph;

fn void test_voronoi_graph_cell_count() @test
{
    VoronoiDiagram d = bounded_4_site_square();
    defer d.destroy();

    VoronoiGraph g = graph::from_voronoi(mem, &d)!!;
    defer g.destroy();

    assert(g.cell_count() == 4);
}

fn void test_voronoi_graph_cells_have_vertices() @test
{
    VoronoiDiagram d = bounded_4_site_square();
    defer d.destroy();

    VoronoiGraph g = graph::from_voronoi(mem, &d)!!;
    defer g.destroy();

    for (int c = 0; c < g.cell_count(); c++) {
        VoronoiCellView cell = g.cell((FaceIndex)c);
        assert(cell.ring.len > 0);
    }
}

fn void test_voronoi_graph_neighbor_symmetry() @test
{
    VoronoiDiagram d = bounded_4_site_square();
    defer d.destroy();

    VoronoiGraph g = graph::from_voronoi(mem, &d)!!;
    defer g.destroy();

    for (int a = 0; a < g.cell_count(); a++) {
        VoronoiCellView ca = g.cell((FaceIndex)a);
        for (usz i = 0; i < ca.neighbors.len; i++) {
            if (ca.neighbors[i] == cg::INVALID_FACE) continue;
            int b = (int)ca.neighbors[i];
            VoronoiCellView cb = g.cell((FaceIndex)b);
            bool found = false;
            for (usz j = 0; j < cb.neighbors.len; j++) {
                if ((int)cb.neighbors[j] == a) {
                    found = true;
                    break;
                }
            }
            assert(found);
        }
    }
}

fn void test_voronoi_graph_boundary_edges_have_invalid_face() @test
{
    VoronoiDiagram d = bounded_4_site_square();
    defer d.destroy();

    VoronoiGraph g = graph::from_voronoi(mem, &d)!!;
    defer g.destroy();

    bool saw_boundary = false;
    for (int c = 0; c < g.cell_count(); c++) {
        VoronoiCellView cell = g.cell((FaceIndex)c);
        for (usz i = 0; i < cell.neighbors.len; i++) {
            if (cell.neighbors[i] == cg::INVALID_FACE) {
                saw_boundary = true;
            }
        }
    }
    assert(saw_boundary);
}

fn void test_voronoi_graph_centroids() @test
{
    VoronoiDiagram d = bounded_4_site_square();
    defer d.destroy();

    VoronoiGraph g = graph::from_voronoi(mem, &d)!!;
    defer g.destroy();

    for (int c = 0; c < g.cell_count(); c++) {
        VoronoiCellView cell = g.cell((FaceIndex)c);
        assert(cell.centroid.x >= -0.1f && cell.centroid.x <= 1.1f);
        assert(cell.centroid.y >= -0.1f && cell.centroid.y <= 1.1f);
    }
}

fn void test_voronoi_graph_cell_view_slices() @test
{
    VoronoiDiagram d = bounded_4_site_square();
    defer d.destroy();

    VoronoiGraph g = graph::from_voronoi(mem, &d)!!;
    defer g.destroy();

    for (int c = 0; c < g.cell_count(); c++) {
        VoronoiCellView cell = g.cell((FaceIndex)c);
        assert(cell.ring.len == cell.neighbors.len);
        assert(cell.index == (FaceIndex)c);
    }
}

fn void test_voronoi_graph_sites_preserved() @test
{
    VoronoiDiagram d = bounded_4_site_square();
    defer d.destroy();

    VoronoiGraph g = graph::from_voronoi(mem, &d)!!;
    defer g.destroy();

    for (int c = 0; c < g.cell_count(); c++) {
        VoronoiCellView cell = g.cell((FaceIndex)c);
        assert(cell.site.x == d.sites[c].x);
        assert(cell.site.y == d.sites[c].y);
        assert(cell.site.z == d.sites[c].z);
    }
}

fn void test_voronoi_graph_icosahedron_spherical() @test
{
    VoronoiDiagram d = voronoi::voronoi_on_sphere(mem, ICO_SPHERE[..], 1.0f)!!;
    defer d.destroy();

    VoronoiGraph g = graph::from_voronoi(mem, &d)!!;
    defer g.destroy();

    assert(g.cell_count() == 12);

    for (int c = 0; c < g.cell_count(); c++) {
        VoronoiCellView cell = g.cell((FaceIndex)c);
        for (usz i = 0; i < cell.neighbors.len; i++) {
            assert(cell.neighbors[i] != cg::INVALID_FACE);
        }
    }
}

fn void test_voronoi_graph_empty_input() @test
{
    VoronoiDiagram d;
    if (catch err = graph::from_voronoi(mem, &d)) {
        assert(err == cg::EMPTY_INPUT);
        return;
    }
    unreachable();
}

fn VoronoiDiagram bounded_4_site_square()
{
    Vec3f[4] s = { {0.25f,0.25f,0}, {0.75f,0.25f,0}, {0.75f,0.75f,0}, {0.25f,0.75f,0} };
    Vec3f[4] p = { {0,0,0}, {1,0,0}, {1,1,0}, {0,1,0} };
    return voronoi::in_polygon(mem, s[..], p[..])!!;
}
