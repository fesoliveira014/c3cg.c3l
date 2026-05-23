module cg::geometry;
import cg;

fn Vec3f[]? centroids_on_sphere(Allocator alloc, HalfEdgeMesh* tri_mesh, float radius)
{
    return face_centroids(alloc, tri_mesh);
}
