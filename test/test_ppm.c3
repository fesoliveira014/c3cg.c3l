module test;

import cg;
import cg::utils;
import cg::utils::ppm;
import cg::half_edge;
import cg::delaunay;
import cg::voronoi;

fn void test_new_ppm() @test
{
    PpmImage img = ppm::new_ppm(mem, 10, 20)!!;
    defer img.destroy();
    assert(img.width == 10);
    assert(img.height == 20);
    assert(img.pixels.len == 200);
    // Default background is opaque black
    for (usz i = 0; i < img.pixels.len; i++) {
        assert(img.pixels[i] == 0xFF000000);
    }
}

fn void test_fill() @test
{
    PpmImage img = ppm::new_ppm(mem, 5, 5)!!;
    defer img.destroy();
    ppm::fill(&img, 0x00FF00FF);
    for (usz i = 0; i < img.pixels.len; i++) {
        assert(img.pixels[i] == 0x00FF00FF);
    }
}

fn void test_draw_line_horizontal() @test
{
    PpmImage img = ppm::new_ppm(mem, 10, 10)!!;
    defer img.destroy();
    ppm::draw_line(&img, 1, 5, 8, 5, 0xFFFFFFFF, 1);
    // Midpoint on the line should be colored
    assert(img.pixels[5 * 10 + 3] == 0xFFFFFFFF);
    assert(img.pixels[5 * 10 + 5] == 0xFFFFFFFF);
    // Pixels above and below should remain background
    assert(img.pixels[4 * 10 + 5] == 0xFF000000);
    assert(img.pixels[6 * 10 + 5] == 0xFF000000);
}

fn void test_draw_line_vertical() @test
{
    PpmImage img = ppm::new_ppm(mem, 10, 10)!!;
    defer img.destroy();
    ppm::draw_line(&img, 5, 1, 5, 8, 0xFFFFFFFF, 1);
    // Midpoint on the line should be colored
    assert(img.pixels[3 * 10 + 5] == 0xFFFFFFFF);
    assert(img.pixels[5 * 10 + 5] == 0xFFFFFFFF);
    // Pixels left and right should remain background
    assert(img.pixels[5 * 10 + 4] == 0xFF000000);
    assert(img.pixels[5 * 10 + 6] == 0xFF000000);
}

fn void test_draw_line_diagonal() @test
{
    PpmImage img = ppm::new_ppm(mem, 10, 10)!!;
    defer img.destroy();
    ppm::draw_line(&img, 0, 0, 9, 9, 0xFFFFFFFF, 1);
    // Points on the diagonal should be colored
    assert(img.pixels[2 * 10 + 2] == 0xFFFFFFFF);
    assert(img.pixels[5 * 10 + 5] == 0xFFFFFFFF);
    // Point off the diagonal should remain background
    assert(img.pixels[5 * 10 + 4] == 0xFF000000);
}

fn void test_draw_circle_filled() @test
{
    PpmImage img = ppm::new_ppm(mem, 20, 20)!!;
    defer img.destroy();
    ppm::draw_circle(&img, 10, 10, 3, 0xFFFFFFFF, true, 1);
    // Center should be filled
    assert(img.pixels[10 * 20 + 10] == 0xFFFFFFFF);
    // Point at radius ~2.5 horizontally should be filled
    assert(img.pixels[10 * 20 + 7] == 0xFFFFFFFF);
    // Point well outside the circle should be background
    assert(img.pixels[10 * 20 + 4] == 0xFF000000);
}

fn void test_draw_circle_outline() @test
{
    PpmImage img = ppm::new_ppm(mem, 20, 20)!!;
    defer img.destroy();
    ppm::draw_circle(&img, 10, 10, 5, 0xFFFFFFFF, false, 1);
    // Center should remain background
    assert(img.pixels[10 * 20 + 10] == 0xFF000000);
    // Horizontal endpoints of the ring at radius 5
    assert(img.pixels[10 * 20 + 5] == 0xFFFFFFFF);
    assert(img.pixels[10 * 20 + 15] == 0xFFFFFFFF);
}

fn void test_render_delaunay_icosahedron() @test
{
    // Create a Delaunay triangulation of the icosahedron vertices (using x,y)
    Vec3f[] points = mem::alloc::new_array(mem, Vec3f, 12);
    defer free(points);
    for (int i = 0; i < 12; i++) {
        points[i] = ICO_POSITIONS[i];
    }

    HalfEdgeMesh mesh = delaunay::delaunay_2d(mem, points[..])!!;
    defer mesh.destroy();

    PpmOptions opts = {
        .width      = 200,
        .height     = 200,
        .edge_color = 0xFFFFFFFF,
        .site_color = 0xFF0000FF,
        .bg_color   = 0xFF000000,
        .line_width = 1,
        .dot_radius = 3,
        .margin     = 0.1f,
    };

    PpmImage img = utils::render_delaunay(mem, &mesh, opts)!!;
    defer img.destroy();

    assert(img.width == 200);
    assert(img.height == 200);

    // Count non-background pixels
    uint edge_px = 0;
    uint site_px = 0;
    for (usz i = 0; i < img.pixels.len; i++) {
        if (img.pixels[i] == opts.edge_color) edge_px++;
        if (img.pixels[i] == opts.site_color) site_px++;
    }

    // Should have some edge pixels and some vertex pixels
    assert(edge_px > 0);
    assert(site_px > 0);
}

fn void test_render_overlay_icosahedron() @test
{
    HalfEdgeMesh ico = half_edge::from_triangles(mem, ICO_POSITIONS[..], ICO_INDICES[..])!!;
    defer ico.destroy();

    VoronoiDiagram diagram = voronoi::from_delaunay(mem, &ico)!!;
    defer diagram.destroy();

    OverlayOptions opts = {
        .common = {
            .width      = 200,
            .height     = 200,
            .edge_color = 0,
            .site_color = 0,
            .bg_color   = 0xFF000000,
            .line_width = 1,
            .dot_radius = 3,
            .margin     = 0.1f,
        },
        .delaunay_edge_color = 0xFF00FF00,
        .voronoi_edge_color  = 0xFF0000FF,
        .site_color          = 0xFFFF0000,
    };

    PpmImage img = utils::render_overlay(mem, &ico, &diagram, opts)!!;
    defer img.destroy();

    assert(img.width == 200);
    assert(img.height == 200);

    uint delaunay_px = 0;
    uint voronoi_px = 0;
    uint site_px = 0;
    for (usz i = 0; i < img.pixels.len; i++) {
        if (img.pixels[i] == opts.delaunay_edge_color) delaunay_px++;
        if (img.pixels[i] == opts.voronoi_edge_color) voronoi_px++;
        if (img.pixels[i] == opts.site_color) site_px++;
    }

    assert(delaunay_px > 0);
    assert(voronoi_px > 0);
    assert(site_px > 0);
}
