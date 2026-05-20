module cg::hull;
import cg;
import cg::geometry;
import cg::half_edge;
import std::collections::map;

const usz H3_FACE_CAP_FACTOR = 4;

struct H3Face {
    VertexIndex v0, v1, v2;
    bool alive;
}

fn HalfEdgeMesh? hull_3d(Allocator alloc, Vec3f[] positions)
{
    if (positions.len == 0) return cg::EMPTY_INPUT~;

    // Deduplicate exact (x,y,z) duplicates.
    int[] indices = mem::alloc::new_array(alloc, int, (sz) positions.len);
    defer catch free(indices);
    for (usz i = 0; i < positions.len; i++) indices[i] = (int)i;

    usz unique_count = 0;
    for (usz i = 0; i < positions.len; i++) {
        bool dup = false;
        for (usz j = 0; j < unique_count; j++) {
            int p = indices[j];
            int c = indices[i];
            if (positions[p].x == positions[c].x
                && positions[p].y == positions[c].y
                && positions[p].z == positions[c].z) { dup = true; break; }
        }
        if (!dup) { indices[unique_count] = indices[i]; unique_count++; }
    }
    if (unique_count < 4) return cg::DEGENERATE_INPUT~;

    // Phase 0 — find non-coplanar tetrahedron.
    int t0 = indices[0];
    int t1 = -1;
    for (usz i = 1; i < unique_count; i++) {
        int p = indices[i];
        if (positions[p].x != positions[t0].x
            || positions[p].y != positions[t0].y
            || positions[p].z != positions[t0].z) { t1 = p; break; }
    }
    if (t1 < 0) return cg::DEGENERATE_INPUT~;

    // Non-collinear third point via cross-product check.
    int t2 = -1;
    Vec3f d01 = {
        positions[t1].x - positions[t0].x,
        positions[t1].y - positions[t0].y,
        positions[t1].z - positions[t0].z,
    };
    for (usz i = 0; i < unique_count; i++) {
        int p = indices[i];
        if (p == t0 || p == t1) continue;
        Vec3f d0p = {
            positions[p].x - positions[t0].x,
            positions[p].y - positions[t0].y,
            positions[p].z - positions[t0].z,
        };
        float cx = d01.y * d0p.z - d01.z * d0p.y;
        float cy = d01.z * d0p.x - d01.x * d0p.z;
        float cz = d01.x * d0p.y - d01.y * d0p.x;
        if (cx * cx + cy * cy + cz * cz > geometry::GEOMETRY_EPSILON * geometry::GEOMETRY_EPSILON) {
            t2 = p; break;
        }
    }
    if (t2 < 0) return cg::DEGENERATE_INPUT~;

    // Non-coplanar fourth point.
    int t3 = -1;
    for (usz i = 0; i < unique_count; i++) {
        int p = indices[i];
        if (p == t0 || p == t1 || p == t2) continue;
        if (geometry::orient_3d(positions[t0], positions[t1], positions[t2], positions[p],
                geometry::GEOMETRY_EPSILON) != geometry::PREDICATE_ZERO) {
            t3 = p; break;
        }
    }
    if (t3 < 0) return cg::DEGENERATE_INPUT~;

    // Compact positions + remap.
    Vec3f[] compacted = mem::alloc::new_array(alloc, Vec3f, (sz) unique_count);
    defer catch free(compacted);
    for (usz i = 0; i < unique_count; i++) compacted[i] = positions[indices[i]];

    int[] remap = mem::alloc::new_array(alloc, int, (sz) positions.len);
    defer catch free(remap);
    for (usz i = 0; i < positions.len; i++) remap[i] = -1;
    for (usz i = 0; i < unique_count; i++) remap[indices[i]] = (int)i;

    int ct0 = remap[t0];
    int ct1 = remap[t1];
    int ct2 = remap[t2];
    int ct3 = remap[t3];

    Vec3f centroid = {
        (compacted[ct0].x + compacted[ct1].x + compacted[ct2].x + compacted[ct3].x) / 4.0f,
        (compacted[ct0].y + compacted[ct1].y + compacted[ct2].y + compacted[ct3].y) / 4.0f,
        (compacted[ct0].z + compacted[ct1].z + compacted[ct2].z + compacted[ct3].z) / 4.0f,
    };

    // Face list + edge map.
    usz face_cap = H3_FACE_CAP_FACTOR * unique_count;
    H3Face[] faces = mem::alloc::new_array(alloc, H3Face, (sz) face_cap);
    defer catch free(faces);

    HashMap{int[<2>], int} edge_map;
    edge_map.init(alloc);
    defer catch edge_map.free();

    usz face_count = 0;

    int[3][4] tet_faces = {
        { ct0, ct1, ct2 },
        { ct0, ct3, ct1 },
        { ct0, ct2, ct3 },
        { ct1, ct3, ct2 },
    };

    for (int fi = 0; fi < 4; fi++) {
        int v0 = tet_faces[fi][0];
        int v1 = tet_faces[fi][1];
        int v2 = tet_faces[fi][2];
        if (geometry::orient_3d(compacted[v0], compacted[v1], compacted[v2], centroid,
                geometry::GEOMETRY_EPSILON) == geometry::PREDICATE_POSITIVE) {
            int tmp = v1; v1 = v2; v2 = tmp;
        }
        faces[face_count] = { (VertexIndex)v0, (VertexIndex)v1, (VertexIndex)v2, true };
        int[<2>] e0 = { v0, v1 };
        int[<2>] e1 = { v1, v2 };
        int[<2>] e2 = { v2, v0 };
        edge_map[e0] = (int)face_count;
        edge_map[e1] = (int)face_count;
        edge_map[e2] = (int)face_count;
        face_count++;
    }

    int[4] tet_verts = { ct0, ct1, ct2, ct3 };

    // Phase 1 — process remaining points.
    for (usz si = 0; si < unique_count; si++) {
        int p = (int)si;

        bool is_tet = false;
        for (int ti = 0; ti < 4; ti++) { if (p == tet_verts[ti]) { is_tet = true; break; } }
        if (is_tet) continue;

        // Visibility.
        usz visible_count = 0;
        for (usz fi = 0; fi < face_count; fi++) {
            if (!faces[fi].alive) continue;
            if (geometry::orient_3d(compacted[(int)faces[fi].v0],
                    compacted[(int)faces[fi].v1],
                    compacted[(int)faces[fi].v2],
                    compacted[p],
                    geometry::GEOMETRY_EPSILON) == geometry::PREDICATE_POSITIVE) {
                faces[fi].alive = false;
                visible_count++;
            }
        }
        if (visible_count == 0) continue;

        // Horizon edges.
        int[] horizon_from = mem::alloc::new_array(alloc, int, (sz)(3 * visible_count));
        defer catch free(horizon_from);
        int[] horizon_to = mem::alloc::new_array(alloc, int, (sz)(3 * visible_count));
        defer catch free(horizon_to);
        usz horizon_count = 0;

        for (usz fi = 0; fi < face_count; fi++) {
            if (faces[fi].alive) continue;
            int v0 = (int)faces[fi].v0;
            int v1 = (int)faces[fi].v1;
            int v2 = (int)faces[fi].v2;
            int[2][3] edges = { { v0, v1 }, { v1, v2 }, { v2, v0 } };
            for (int ei = 0; ei < 3; ei++) {
                int u = edges[ei][0];
                int r = edges[ei][1];
                int[<2>] rev = { r, u };
                if (try adj = edge_map[rev]) {
                    if ((usz)adj < face_count && faces[adj].alive) {
                        horizon_from[horizon_count] = u;
                        horizon_to[horizon_count] = r;
                        horizon_count++;
                    }
                }
            }
        }

        // Stitch new faces: (u,r,p) for each horizon edge (u,r).
        usz added = 0;
        for (usz hi = 0; hi < horizon_count; hi++) {
            int u = horizon_from[hi];
            int r = horizon_to[hi];
            faces[face_count + added] = { (VertexIndex)u, (VertexIndex)r, (VertexIndex)p, true };
            int[<2>] e0 = { u, r };
            int[<2>] e1 = { r, p };
            int[<2>] e2 = { p, u };
            edge_map[e0] = (int)(face_count + added);
            edge_map[e1] = (int)(face_count + added);
            edge_map[e2] = (int)(face_count + added);
            added++;
        }
        face_count += added;

        free(horizon_from);
        free(horizon_to);

        // Compact faces and rebuild edge map.
        usz write = 0;
        for (usz fi = 0; fi < face_count; fi++) {
            if (faces[fi].alive) {
                if (write != fi) faces[write] = faces[fi];
                write++;
            }
        }
        face_count = write;

        edge_map.clear();
        for (usz fi = 0; fi < face_count; fi++) {
            int v0 = (int)faces[fi].v0;
            int v1 = (int)faces[fi].v1;
            int v2 = (int)faces[fi].v2;
            int[<2>] e0 = { v0, v1 };
            int[<2>] e1 = { v1, v2 };
            int[<2>] e2 = { v2, v0 };
            edge_map[e0] = (int)fi;
            edge_map[e1] = (int)fi;
            edge_map[e2] = (int)fi;
        }
    }

    edge_map.free();
    free(remap);

    // Phase 2 — build used-vertex-only mesh.
    bool[] used = mem::alloc::new_array(alloc, bool, (sz) unique_count);
    defer catch free(used);
    for (usz i = 0; i < unique_count; i++) used[i] = false;
    for (usz fi = 0; fi < face_count; fi++) {
        used[(int)faces[fi].v0] = true;
        used[(int)faces[fi].v1] = true;
        used[(int)faces[fi].v2] = true;
    }

    usz used_vert_count = 0;
    for (usz i = 0; i < unique_count; i++) { if (used[i]) used_vert_count++; }

    Vec3f[] final_positions = mem::alloc::new_array(alloc, Vec3f, (sz) used_vert_count);
    defer catch free(final_positions);
    int[] final_remap = mem::alloc::new_array(alloc, int, (sz) unique_count);
    defer catch free(final_remap);
    for (usz i = 0; i < unique_count; i++) final_remap[i] = -1;

    usz w = 0;
    for (usz i = 0; i < unique_count; i++) {
        if (used[i]) {
            final_positions[w] = compacted[i];
            final_remap[i] = (int)w;
            w++;
        }
    }

    uint[] tri_indices = mem::alloc::new_array(alloc, uint, (sz)(3 * face_count));
    defer catch free(tri_indices);
    for (usz fi = 0; fi < face_count; fi++) {
        tri_indices[3 * fi]     = (uint)final_remap[(int)faces[fi].v0];
        tri_indices[3 * fi + 1] = (uint)final_remap[(int)faces[fi].v1];
        tri_indices[3 * fi + 2] = (uint)final_remap[(int)faces[fi].v2];
    }

    HalfEdgeMesh mesh = half_edge::from_triangles(alloc, final_positions, tri_indices)!;
    defer catch mesh.destroy();

    mesh.validate()!;

    free(tri_indices);
    free(final_remap);
    free(final_positions);
    free(used);
    free(faces);
    free(compacted);
    free(indices);

    return mesh;
}
