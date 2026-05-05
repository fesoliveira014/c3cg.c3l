module test;
import cg;
import cg::smoke;

fn void test_cross_module_method() @test
{
    HalfEdgeMesh mesh = {};
    assert(mesh.dummy() == 42);
}
