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
