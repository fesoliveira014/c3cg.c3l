module cg::geometry;
import cg;
import std::math;

fn Vec3f[]? centroids_on_sphere(Allocator alloc, HalfEdgeMesh* tri_mesh, float radius)
{
    if (radius <= GEOMETRY_EPSILON) return cg::DEGENERATE_INPUT~;

    Vec3f[] centroids = face_centroids(alloc, tri_mesh)!;

    for (usz i = 0; i < centroids.len; i++) {
        float x = centroids[i].x;
        float y = centroids[i].y;
        float z = centroids[i].z;
        float len2 = x * x + y * y + z * z;
        if (len2 <= GEOMETRY_EPSILON * GEOMETRY_EPSILON) {
            free(centroids);
            return cg::DEGENERATE_INPUT~;
        }
        float inv_len = 1.0f / (float)math::sqrt((double)len2);
        centroids[i] = {
            x * inv_len * radius,
            y * inv_len * radius,
            z * inv_len * radius,
        };
    }

    return centroids;
}
