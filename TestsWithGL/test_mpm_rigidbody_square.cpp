#include "TestsWithGL_pcp.h"

#include <fstream>

#include "Model_S2D_ME_s_RigidBody.h"
#include "Step_S2D_ME_s_RigidBody.h"
#include "TimeHistory_S2D_ME_s_RigidBody.h"

#include "test_sim_core.h"

void test_mpm_rigidbody_square(void)
{
	Model_S2D_ME_s_RigidBody model;

	Step_S2D_ME_s_RigidBody step;
	step.set_model(&model);
	step.set_dt(0.01);

	std::fstream res_file;
	//res_file.open();
	
	model.output(res_file);

	TimeHistory_S2D_ME_s_RigidBody out1(res_file);
	out1.set_interval_num(10);

	step.add_output(&out1);
	//step.solve();
}
