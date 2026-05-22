module c3cg__debug;
import cg;
import cg::voronoi;
import std::io;

fn void main()
{
    Vec3f[4] sites = { {0.25,0.25,0}, {0.75,0.25,0}, {0.75,0.75,0}, {0.25,0.75,0} };
    Vec3f[4] poly = { {0,0,0}, {1,0,0}, {1,1,0}, {0,1,0} };

    VoronoiDiagram d = voronoi::in_polygon(mem, sites[..], poly[..])!!;
    defer d.destroy();

    d.mesh.validate()!!;

    io::printfn("faces=%d sites=%d", (int)d.mesh.faces.len, (int)d.sites.len);
}
