#include "Test_pcp.h"

#include "Model_S2D_ME_MPM_s.h"
#include "Step_S2D_ME_MPM_s_GIMP.h"

#include "TimeHistory_Particle_S2D_ME_AllPcl.h"
#include "TimeHistory_ConsoleProgressBar.h"
#include "ResultFile_Text.h"

#include "test_sim_core.h"

// One diemnsional compression
void test_me_mpm_gimp1(void)
{
	Model_S2D_ME_MPM_s model;
	
	size_t elem_x_num = 1;
	size_t elem_y_num = 1;
	double elem_len = 1.0 / double(elem_y_num);
	model.init_mesh(elem_len, elem_x_num, elem_y_num);

	double pcl_density = 10.0;
	model.init_pcl(elem_x_num * elem_y_num * 4,
				   elem_len * elem_len * 0.5 * 0.5 * pcl_density,
				   pcl_density, 100.0, 0.25);
	size_t k = 0;
	for (size_t j = 0; j < elem_y_num * 2; ++j)
		for (size_t i = 0; i < elem_x_num * 2; ++i)
		{
			Particle_S2D_ME &pcl = model.pcls[k];
			pcl.x = (0.25 + double(i) * 0.5) * elem_len;
			pcl.y = (0.25 + double(j) * 0.5) * elem_len;
			++k;
		}

	model.ty_num = elem_x_num * 2;
	model.tys = new TractionBC_MPM[model.ty_num];
	for (size_t i = 0; i < model.ty_num; ++i)
	{
		model.tys[i].pcl_id = (elem_y_num * 2 - 1) * elem_x_num * 2 + i;
		model.tys[i].t = -1.0 * 0.5 * elem_len;
	}

	model.vx_num = model.node_y_num * 2;
	model.vxs = new VelocityBC[model.vx_num];
	for (size_t i = 0; i < model.node_y_num; i++)
	{
		model.vxs[i].node_id = i * model.node_x_num;
		model.vxs[i].v = 0.0;
		model.vxs[i + model.node_y_num].node_id = (i + 1) * model.node_x_num - 1;
		model.vxs[i + model.node_y_num].v = 0.0;
	}

	model.vy_num = model.node_x_num;
	model.vys = new VelocityBC[model.vy_num];
	for (size_t i = 0; i < model.vy_num; i++)
	{
		model.vys[i].node_id = i;
		model.vys[i].v = 0.0;
	}

	ResultFile_Text res_file;
	res_file.set_filename("res_file");

	res_file.output_model_state(model);

	TimeHistory_Particle_S2D_ME_AllPcl th1;
	th1.set_name("test_out1");
	th1.set_interval_num(100);
	th1.set_if_output_initial_state(false);
	Particle_Field_2D_ME fld1[8] = {
		Particle_Field_2D_ME::x,
		Particle_Field_2D_ME::y,
		Particle_Field_2D_ME::vol,
		Particle_Field_2D_ME::vy,
		Particle_Field_2D_ME::vx,
		Particle_Field_2D_ME::e11,
		Particle_Field_2D_ME::e12,
		Particle_Field_2D_ME::e22
	};
	for (size_t i = 0; i < sizeof(fld1)/sizeof(fld1[0]); ++i)
		th1.add_field(fld1[i]);

	TimeHistory_ConsoleProgressBar th2;

	Step_S2D_ME_MPM_s_GIMP step1;
	step1.set_name("initial_step");
	step1.set_model(&model);
	step1.set_result_file(&res_file);
	step1.set_step_time(1.0e-1);
	step1.set_dt(1.0e-2);
	step1.add_output(&th1);
	step1.add_output(&th2);

	step1.solve();
}