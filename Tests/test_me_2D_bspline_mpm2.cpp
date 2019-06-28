#include "Test_pcp.h"

#include "Model_R2D_ME_MPM_BSpline_s.h"

#include "Step_R2D_ME_MPM_BSpline_APIC_s.h"

#include "TimeHistory_ConsoleProgressBar.h"
#include "TimeHistory_Particle_2D_ME_MPM_BSpline_s_AllPcl.h"

#include "ResultFile_Text.h"

#include "test_sim_core.h"

// 2D bspline mechanics MPM
// 1D compression
void test_me_2D_bspline_mpm2(void)
{
	size_t elem_div_num = 20;
	size_t pcl_div_num = 4;
	double bar_len = 1.0;

	Model_R2D_ME_MPM_BSpline_s model;
	double elem_len = bar_len / double(elem_div_num);
	size_t elem_x_num = 2 + 2;
	size_t elem_y_num = elem_div_num + 2;
	double pcl_len = elem_len / double(pcl_div_num);
	size_t pcl_x_num = 2 * pcl_div_num;
	size_t pcl_y_num = elem_div_num * pcl_div_num;
	size_t pcl_num = pcl_x_num * pcl_y_num;

	Particle_R2D_ME_Grid pcl;
	pcl.init();
	pcl.density = 20.0;
	pcl.m = pcl_len * pcl_len * pcl.density;
	pcl.E = 100.0;
	pcl.niu = 0.0;
	model.set_pcl_num(pcl_num);
	for (size_t i = 0; i < pcl_y_num; ++i)
		for (size_t j = 0; j < pcl_x_num; ++j)
		{
			pcl.x = (j + 0.5) * pcl_len;
			pcl.y = (i + 0.5) * pcl_len;
			model.add_pcl(pcl);
		}
	
	model.set_mesh(elem_x_num, elem_y_num, elem_len, -elem_len, -elem_len);

	// velocity bc
	model.set_vy_num((elem_x_num + 1) * 2);
	VelocityBC vbc;
	vbc.v = 0.0;
	for (size_t i = 0; i < (elem_x_num + 1) * 2; i++)
	{
		vbc.node_id = i;
		model.add_vy(vbc);
	}
	model.set_vx_num((elem_x_num + 1) * (elem_y_num + 1));
	for (size_t i = 0; i < (elem_x_num + 1) * (elem_y_num + 1); i++)
	{
		vbc.node_id = i;
		model.add_vx(vbc);
	}

	model.set_ty_num(2);
	TractionBC_MPM tbc;
	tbc.t = -20.0 * pcl_len;
	for (size_t i = 0; i < pcl_x_num; i++)
	{
		tbc.pcl_id = pcl_num - i - 1;
		model.add_ty(tbc);
	}

	model.update();

	ResultFile_Text res_file;
	res_file.set_filename("res_file");
	
	res_file.output_model_state(model);

	TimeHistory_Particle_2D_ME_MPM_BSpline_s_AllPcl th1;
	th1.set_name("test_out1");
	th1.set_interval_num(500);
	//th1.set_if_output_initial_state(false);
	Particle_Field_2D_ME fld1[] = {
		Particle_Field_2D_ME::x,
		Particle_Field_2D_ME::y,
		Particle_Field_2D_ME::vol,
		Particle_Field_2D_ME::s22,
		Particle_Field_2D_ME::vy,
		Particle_Field_2D_ME::density,
		Particle_Field_2D_ME::vx,
		Particle_Field_2D_ME::s11
	};
	th1.set_field_num(sizeof(fld1) / sizeof(fld1[0]));
	for (size_t i = 0; i < sizeof(fld1) / sizeof(fld1[0]); i++)
		th1.add_field(fld1[i]);

	TimeHistory_ConsoleProgressBar th2;

	Step_R2D_ME_MPM_BSpline_APIC_s step1;
	step1.set_name("initial_step");
	step1.set_model(&model);
	step1.set_result_file(&res_file);
	step1.set_step_time(10.0); // total_time
	step1.set_dt(1.0e-3);
	step1.add_output(&th1);
	step1.add_output(&th2);

	step1.solve();
}