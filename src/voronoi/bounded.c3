module cg::voronoi;
import cg;
import cg::geometry;
import cg::half_edge;
import cg::delaunay;
import cg::dual;

fn Vec3f[]? sutherland_hodgman(Allocator alloc, Vec3f[] polygon, Vec3f[] clip_polygon) { return {}; }
fn Vec3f[]? half_plane_clip(Allocator alloc, Vec3f[] polygon, Vec3f a, Vec3f b, bool keep_a_side) { return {}; }
fn Vec3f? line_intersection(Vec2f a, Vec2f b, Vec2f p, Vec2f q) { return {}; }
fn bool point_in_convex_polygon(Vec3f p, Vec3f[] polygon) { return true; }
fn bool is_convex_polygon(Vec3f[] polygon) { return true; }
