module bench::delaunay;
import cg;
import cg::delaunay;
import std::io;

fn void main()
{
    int n = 8;
    Vec3f[] points = mem::alloc::new_array(mem, Vec3f, (sz) n);
    defer free(points);
    for (int i = 0; i < n; i++) {
        points[i] = {
            (float)((int)((uint)i * 2654435761u % 1000000u)) * 0.001f,
            (float)((int)((uint)i * 1737350767u % 1000000u)) * 0.001f,
            0,
        };
        io::printfn("pt[%d] = (%.3f, %.3f)", i, points[i].x, points[i].y);
    }
    if (catch err = delaunay::delaunay_2d(mem, points[..])) {
        io::printfn("FAULT");
    } else {
        io::printfn("OK");
    }
}
