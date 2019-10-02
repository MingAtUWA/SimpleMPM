#include "TestsWithGL_pcp.h"

#include <fstream>

#include "Model_S2D_ME_s_RigidBody.h"
#include "Step_S2D_ME_s_RigidBody.h"
#include "TimeHistory_S2D_ME_s_RigidBody.h"

#include "test_sim_core.h"

void test_mpm_rigidbody_square(void)
{
	Model_S2D_ME_s_RigidBody model;

	model.rigid_body.load_and_init_mesh("..\\..\\Asset\\square_mesh.mesh_data", 0.3);
	model.rigid_body.set_params(1.0, 0.0, 4.0, 0.0);

	model.init_mesh(1.0, 4, 3, -2.0, 0.0);
	model.init_pcl(4, 1.0, 1.0, 100.0, 0.0);
	size_t pcl_id = 0;
	for (size_t y_id = 0; y_id < 2; y_id++)
		for (size_t x_id = 0; x_id < 2; x_id++)
		{
			Model_S2D_ME_s_RigidBody::Particle &pcl = model.pcls[pcl_id];
			pcl.x = -0.5 + double(x_id) * 1.0;
			pcl.y =  0.5 + double(y_id) * 1.0;
			++pcl_id;
		}
	// vy bc
	model.vy_num = 3;
	model.vys = new VelocityBC[model.vy_num];
	model.vys[0].node_id = 1;
	model.vys[0].v = 0.0;
	model.vys[1].node_id = 2;
	model.vys[1].v = 0.0;
	model.vys[2].node_id = 3;
	model.vys[2].v = 0.0;
	// vx bc
	model.vx_num = 1;
	model.vxs = new VelocityBC[model.vx_num];
	model.vxs[0].node_id = 2;
	model.vxs[0].v = 0.0;

	std::fstream res_file;
	res_file.open("rigid_body_res.bin", std::ios::binary | std::ios::out);
	model.output(res_file);

	Step_S2D_ME_s_RigidBody step;
	step.set_name("init_step");
	step.set_model(&model);
	step.set_step_time(0.01);
	step.set_dt(0.01);
	
	TimeHistory_S2D_ME_s_RigidBody out1(res_file);
	out1.set_interval_num(10);
	step.add_output(&out1);

	step.solve();
}
