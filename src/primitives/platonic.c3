module cg::primitives;
import cg;
import cg::half_edge;
import std::math;

const float ICO_PHI = 1.618033988749895f;
const float ICO_R   = 1.902113032590307f;

fn HalfEdgeMesh? tetrahedron(Allocator alloc)
{
    float inv_sqrt3 = 1.0f / (float)math::sqrt(3.0);
    Vec3f[4] positions = {
        {  1.0f * inv_sqrt3,  1.0f * inv_sqrt3,  1.0f * inv_sqrt3 },
        {  1.0f * inv_sqrt3, -1.0f * inv_sqrt3, -1.0f * inv_sqrt3 },
        { -1.0f * inv_sqrt3,  1.0f * inv_sqrt3, -1.0f * inv_sqrt3 },
        { -1.0f * inv_sqrt3, -1.0f * inv_sqrt3,  1.0f * inv_sqrt3 },
    };
    uint[12] indices = {
        0, 1, 2,
        0, 2, 3,
        0, 3, 1,
        3, 2, 1,
    };
    return half_edge::from_triangles(alloc, positions[..], indices[..]);
}

fn HalfEdgeMesh? octahedron(Allocator alloc)
{
    Vec3f[6] positions = {
        {  1,  0,  0 },
        { -1,  0,  0 },
        {  0,  1,  0 },
        {  0, -1,  0 },
        {  0,  0,  1 },
        {  0,  0, -1 },
    };
    uint[24] indices = {
        4, 0, 2,
        4, 2, 1,
        4, 1, 3,
        4, 3, 0,
        5, 2, 0,
        5, 1, 2,
        5, 3, 1,
        5, 0, 3,
    };
    return half_edge::from_triangles(alloc, positions[..], indices[..]);
}

fn HalfEdgeMesh? icosahedron(Allocator alloc)
{
    float phi = ICO_PHI;
    float r   = ICO_R;
    Vec3f[12] positions = {
        { -1.0f / r,  phi / r,  0.0f },
        {  1.0f / r,  phi / r,  0.0f },
        { -1.0f / r, -phi / r,  0.0f },
        {  1.0f / r, -phi / r,  0.0f },
        {  0.0f, -1.0f / r,  phi / r },
        {  0.0f,  1.0f / r,  phi / r },
        {  0.0f, -1.0f / r, -phi / r },
        {  0.0f,  1.0f / r, -phi / r },
        {  phi / r,  0.0f, -1.0f / r },
        {  phi / r,  0.0f,  1.0f / r },
        { -phi / r,  0.0f, -1.0f / r },
        { -phi / r,  0.0f,  1.0f / r },
    };
    uint[60] indices = {
        0, 11, 5,
        0, 5, 1,
        0, 1, 7,
        0, 7, 10,
        0, 10, 11,
        1, 5, 9,
        5, 11, 4,
        11, 10, 2,
        10, 7, 6,
        7, 1, 8,
        3, 9, 4,
        3, 4, 2,
        3, 2, 6,
        3, 6, 8,
        3, 8, 9,
        4, 9, 5,
        2, 4, 11,
        6, 2, 10,
        8, 6, 7,
        9, 8, 1,
    };
    return half_edge::from_triangles(alloc, positions[..], indices[..]);
}

fn HalfEdgeMesh? triangulated_cube(Allocator alloc)
{
    Vec3f[8] positions = {
        { -1, -1, -1 },
        {  1, -1, -1 },
        {  1,  1, -1 },
        { -1,  1, -1 },
        { -1, -1,  1 },
        {  1, -1,  1 },
        {  1,  1,  1 },
        { -1,  1,  1 },
    };
    uint[36] indices = {
        0, 3, 2,  0, 2, 1,
        4, 5, 6,  4, 6, 7,
        0, 4, 7,  0, 7, 3,
        0, 1, 5,  0, 5, 4,
        1, 6, 5,  1, 2, 6,
        2, 7, 6,  2, 3, 7,
    };
    return half_edge::from_triangles(alloc, positions[..], indices[..]);
}
