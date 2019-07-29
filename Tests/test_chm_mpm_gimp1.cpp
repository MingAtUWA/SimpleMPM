#include "Test_pcp.h"

#include "Model_S2D_CHM_MPM_s.h"
#include "Step_S2D_CHM_MPM_s_GIMP.h"

#include "TimeHistory_Particle_S2D_CHM_AllPcl.h"
#include "TimeHistory_ConsoleProgressBar.h"
#include "ResultFile_Text.h"

#include "test_sim_core.h"

// 1 by 1 element
void test_chm_mpm_gimp1(void)
{
	Model_S2D_CHM_MPM_s model;

	size_t elem_x_num = 1;
	size_t elem_y_num = 1;
	double elem_len = 1.0 / double(elem_y_num);
	model.init_mesh(elem_len, elem_x_num, elem_y_num);

	double pcl_vol = elem_len * elem_len * 0.5 * 0.5;
	model.init_pcl(elem_x_num * elem_y_num * 4,
		0.3, pcl_vol * (1.0 - 0.3) * 2.0, 2.0, 1.0,
		100.0, 0.3, 1000.0, 1.0e-3, 1.0);
	size_t k = 0;
	for (size_t j = 0; j < elem_y_num * 2; ++j)
		for (size_t i = 0; i < elem_x_num * 2; ++i)
		{
			Particle_S2D_CHM &pcl = model.pcls[k];
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

	model.vsx_num = model.node_y_num * 2;
	model.vsxs = new VelocityBC[model.vsx_num];
	for (size_t i = 0; i < model.node_y_num; i++)
	{
		model.vsxs[i].node_id = i * model.node_x_num;
		model.vsxs[i].v = 0.0;
		model.vsxs[i + model.node_y_num].node_id = (i + 1) * model.node_x_num - 1;
		model.vsxs[i + model.node_y_num].v = 0.0;
	}

	model.vsy_num = model.node_x_num;
	model.vsys = new VelocityBC[model.vsy_num];
	for (size_t i = 0; i < model.vsy_num; i++)
	{
		model.vsys[i].node_id = i;
		model.vsys[i].v = 0.0;
	}

	model.vfx_num = model.node_y_num * 2;
	model.vfxs = new VelocityBC[model.vfx_num];
	for (size_t i = 0; i < model.node_y_num; i++)
	{
		model.vfxs[i].node_id = i * model.node_x_num;
		model.vfxs[i].v = 0.0;
		model.vfxs[i + model.node_y_num].node_id = (i + 1) * model.node_x_num - 1;
		model.vfxs[i + model.node_y_num].v = 0.0;
	}

	model.vfy_num = model.node_x_num * 2;
	model.vfys = new VelocityBC[model.vfy_num];
	for (size_t i = 0; i < model.node_x_num; i++)
	{
		model.vfys[i].node_id = i;
		model.vfys[i].v = 0.0;
		model.vfys[i + model.node_x_num].node_id = model.node_x_num * (model.node_y_num - 1) + i;
		model.vfys[i + model.node_x_num].v = 0.0;

	}

	ResultFile_Text res_file;
	res_file.set_filename("res_file");

	res_file.output_model_state(model);

	TimeHistory_Particle_S2D_CHM_AllPcl th1;
	th1.set_name("test_out1");
	th1.set_interval_num(100);
	th1.set_if_output_initial_state(false);
	Particle_Field_2D_CHM fld1[4] = {
		Particle_Field_2D_CHM::x,
		Particle_Field_2D_CHM::y,
		Particle_Field_2D_CHM::vol,
		Particle_Field_2D_CHM::p,
	};
	for (size_t i = 0; i < sizeof(fld1) / sizeof(fld1[0]); ++i)
		th1.add_field(fld1[i]);

	TimeHistory_ConsoleProgressBar th2;

	// step1
	Step_S2D_CHM_MPM_s_GIMP step1;
	step1.set_name("initial_step");
	step1.set_model(&model);
	step1.set_result_file(&res_file);
	step1.set_step_time(10.0);
	step1.set_dt(1.0e-4);
	step1.add_output(&th1);
	step1.add_output(&th2);

	step1.solve();

	//// step2
	//Step_S2D_CHM_MPM_s_GIMP step2;
	//step2.set_name("consolidation_step");
	//step2.set_prev_step(&step1);
	//step2.set_step_time(30.0); // total_time
	//step2.set_dt(1.0e-4);

	//// free drainage bcs
	//model.afy_num = model.node_x_num;
	//th1.set_interval_num(30);
	//step2.add_output(&th1);
	//step2.add_output(&th2);

	//step2.solve();
}