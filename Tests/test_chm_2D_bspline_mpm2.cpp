#include "Test_pcp.h"

#include "Model_R2D_CHM_MPM_BSpline_s.h"

#include "Step_R2D_CHM_MPM_BSpline_APIC_s.h"

#include "TimeHistory_ConsoleProgressBar.h"
#include "TimeHistory_Particle_2D_CHM_MPM_BSpline_s_AllPcl.h"

#include "ResultFile_Text.h"

#include "test_sim_core.h"

// 2D bspline mechanics MPM
// 1D compression
void test_chm_2D_bspline_mpm2(void)
{
	size_t elem_div_num = 10;
	size_t pcl_div_num = 4;
	double bar_len = 1.0;

	Model_R2D_CHM_MPM_BSpline_s model;
	double elem_len = bar_len / double(elem_div_num);
	size_t elem_x_num = 1 + 2;
	size_t elem_y_num = elem_div_num + 2;
	double pcl_len = elem_len / double(pcl_div_num);
	size_t pcl_x_num = (elem_x_num - 2) * pcl_div_num;
	size_t pcl_y_num = (elem_y_num - 2) * pcl_div_num;
	size_t pcl_num = pcl_x_num * pcl_y_num;

	Particle_R2D_CHM_Grid pcl;
	pcl.init();
	pcl.density_s = 2650.0;
	pcl.density_f = 1000.0;
	pcl.n = 0.3;
	pcl.m_s = pcl_len * pcl_len * (1.0 - pcl.n) * pcl.density_s;
	pcl.E = 1.0e3;
	pcl.niu = 0.25;
	pcl.Kf = 50.0e3;
	pcl.k = 1.0e-4;
	pcl.miu = 1.0;
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
	VelocityBC vbc;
	vbc.v = 0.0;
	// solid phase
	model.set_vsy_num((elem_x_num + 1) * 2);
	for (size_t i = 0; i < (elem_x_num + 1) * 2; i++)
	{
		vbc.node_id = i;
		model.add_vsy(vbc);
	}
	model.set_vsx_num((elem_y_num + 1) * 4);
	for (size_t i = 0; i < elem_y_num + 1; i++)
	{
		vbc.node_id = (elem_x_num+1) * i;
		model.add_vsx(vbc);
		vbc.node_id = (elem_x_num+1) * i + 1;
		model.add_vsx(vbc);
		vbc.node_id = (elem_x_num+1) * (i+1) - 2;
		model.add_vsx(vbc);
		vbc.node_id = (elem_x_num+1) * (i+1) - 1;
		model.add_vsx(vbc);
	}
	// fluid phase
	model.set_vfy_num((elem_x_num + 1) * 4);
	for (size_t i = 0; i < (elem_x_num + 1) * 2; i++)
	{
		vbc.node_id = i;
		model.add_vfy(vbc);
		vbc.node_id = (elem_x_num + 1) * (elem_y_num - 1) + i;
		model.add_vfy(vbc);
	}
	model.set_vfx_num((elem_y_num + 1) * 4);
	for (size_t i = 0; i < elem_y_num + 1; i++)
	{
		vbc.node_id = (elem_x_num+1) * i;
		model.add_vfx(vbc);
		vbc.node_id = (elem_x_num+1) * i + 1;
		model.add_vfx(vbc);
		vbc.node_id = (elem_x_num+1) * (i+1) - 2;
		model.add_vfx(vbc);
		vbc.node_id = (elem_x_num+1) * (i+1) - 1;
		model.add_vfx(vbc);
	}

	model.set_ty_num(2);
	TractionBC_MPM tbc;
	tbc.t = -400.0 * pcl_len;
	for (size_t i = 0; i < pcl_x_num; i++)
	{
		tbc.pcl_id = pcl_num - i - 1;
		model.add_ty(tbc);
	}

	model.update();

	ResultFile_Text res_file;
	res_file.set_filename("res_file");
	
	res_file.output_model_state(model);

	TimeHistory_Particle_2D_CHM_MPM_BSpline_s_AllPcl th1;
	th1.set_name("test_out1");
	//th1.set_if_output_initial_state(false);
	Particle_Field_2D_CHM fld1[] = {
		Particle_Field_2D_CHM::x,
		Particle_Field_2D_CHM::y,
		Particle_Field_2D_CHM::vol,
		Particle_Field_2D_CHM::p,
		Particle_Field_2D_CHM::n,
		Particle_Field_2D_CHM::density_f,
		Particle_Field_2D_CHM::density_s,
		Particle_Field_2D_CHM::vx_f,
		Particle_Field_2D_CHM::vy_f,
		Particle_Field_2D_CHM::vx_s,
		Particle_Field_2D_CHM::vy_s,
		Particle_Field_2D_CHM::s11
	};
	th1.set_field_num(sizeof(fld1) / sizeof(fld1[0]));
	for (size_t i = 0; i < sizeof(fld1) / sizeof(fld1[0]); i++)
		th1.add_field(fld1[i]);
	th1.set_if_output_initial_state(false);

	TimeHistory_ConsoleProgressBar th2;

	// ===================== first step =====================
	Step_R2D_CHM_MPM_BSpline_APIC_s step1;
	step1.set_name("initial_step");
	step1.set_model(&model);
	step1.set_result_file(&res_file);
	step1.set_step_time(10.0);
	step1.set_dt(2.0e-4); // smaller time step when load is larger
	th1.set_interval_num(20);
	step1.add_output(&th1);
	step1.add_output(&th2);

	step1.solve();

	// ===================== second step =====================
	// free draining boundary condition
	model.clear_vfy();
	vbc.v = 0.0;
	for (size_t i = 0; i < (elem_x_num + 1) * 2; i++)
	{
		vbc.node_id = i;
		model.add_vfy(vbc);
	}

	Step_R2D_CHM_MPM_BSpline_APIC_s step2;
	step2.set_name("consolidation_step");
	step2.set_prev_step(&step1);
	step2.set_step_time(30.0);
	step2.set_dt(2.0e-4); // smaller time step when load is larger
	th1.set_interval_num(60);
	step2.add_output(&th1);
	step2.add_output(&th2);

	step2.solve();
}