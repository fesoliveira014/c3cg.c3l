module bench::delaunay;
import cg;
import cg::delaunay;
import std::time::clock;
import std::io;

const int[] SIZES = { 100, 500, 1000, 2000, 5000, 10000 };

fn void main()
{
    for (int si = 0; si < SIZES.len; si++) {
        int n = SIZES[si];

        Vec3f[] points = mem::alloc::new_array(mem, Vec3f, (sz) n);
        defer free(points);
        for (int i = 0; i < n; i++) {
            points[i] = {
                (float)((int)((uint)i * 2654435761u % 1000000u)) * 0.001f,
                (float)((int)((uint)i * 1737350767u % 1000000u)) * 0.001f,
                0,
            };
        }

        Clock t0 = clock::now();
        HalfEdgeMesh mesh = delaunay::delaunay_2d(mem, points[..])!!;
        NanoDuration elapsed = t0.to_now();
        defer mesh.destroy();

        io::printfn("n=%d time=%d ns faces=%d",
            n,
            (int)(long)elapsed,
            (int)mesh.faces.len);
    }
}
