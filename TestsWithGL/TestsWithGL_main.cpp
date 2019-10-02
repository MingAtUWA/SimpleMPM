#include "TestsWithGL_pcp.h"

#include <cstdlib>

#include "test_sim_core.h"
#include "test_post_processor.h"

int main(int argc, void **argv)
{
	//test_triangle_mesh_circle();
	//test_triangle_mesh_square();
	//test_rigid_body_square();
	
	test_mpm_rigidbody_square();

	//system("pause");
	return 0;
}