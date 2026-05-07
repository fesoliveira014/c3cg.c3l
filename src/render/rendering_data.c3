module cg::render;
import cg;
import cg::half_edge;

struct RenderingData
{
    Vec3f[] vertices;
    uint[] indices;
    Vec3f[] normals;
    Vec2f[] uvs;
}

fn void RenderingData.destroy(&self)
{
    free(self.vertices);
    free(self.indices);
    free(self.normals);
    free(self.uvs);
    *self = {};
}

fn RenderingData? HalfEdgeMesh.to_rendering_data(&self, Allocator alloc)
{
    self.validate()!;

    usz triangle_count = 0;
    usz max_face_degree = 0;

    for (usz f = 0; f < self.faces.len; f++) {
        int degree = self.face_degree((FaceIndex)(int)f);
        usz face_degree = (usz)degree;
        triangle_count += face_degree - 2;
        if (face_degree > max_face_degree) max_face_degree = face_degree;
    }

    RenderingData data;
    data.vertices = mem::alloc::new_array(alloc, Vec3f, self.positions.len);
    defer catch free(data.vertices);
    data.indices = mem::alloc::new_array(alloc, uint, triangle_count * 3);
    defer catch free(data.indices);
    data.normals = mem::alloc::new_array(alloc, Vec3f, self.positions.len);
    defer catch free(data.normals);
    data.uvs = mem::alloc::new_array(alloc, Vec2f, self.positions.len);
    defer catch free(data.uvs);

    VertexIndex[] face_vertices = mem::alloc::new_array(alloc, VertexIndex, max_face_degree);
    defer free(face_vertices);

    for (usz i = 0; i < self.positions.len; i++) {
        data.vertices[i] = self.positions[i];
        if (self.normals.len > 0) {
            data.normals[i] = self.normals[i];
        } else {
            data.normals[i] = { 0, 0, 0 };
        }
        if (self.uvs.len > 0) {
            data.uvs[i] = self.uvs[i];
        } else {
            data.uvs[i] = { 0, 0 };
        }
    }

    usz out_index = 0;
    for (usz f = 0; f < self.faces.len; f++) {
        int degree = self.face_vertices((FaceIndex)(int)f, face_vertices)!;
        VertexIndex v0 = face_vertices[0];
        for (int i = 1; i < degree - 1; i++) {
            data.indices[out_index] = (uint)v0;
            out_index++;
            data.indices[out_index] = (uint)face_vertices[i];
            out_index++;
            data.indices[out_index] = (uint)face_vertices[i + 1];
            out_index++;
        }
    }

    return data;
}
