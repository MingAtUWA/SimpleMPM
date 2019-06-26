#include "Test_pcp.h"

#include "Model_R2D_CHM_MPM_BSpline_s.h"

#include "Step_R2D_CHM_MPM_BSpline_APIC_s.h"

#include "TimeHistory_ConsoleProgressBar.h"
#include "TimeHistory_Particle_2D_CHM_MPM_BSpline_s_AllPcl.h"

#include "ResultFile_Text.h"

#include "test_sim_core.h"

// 2D bspline coupled hydro-mechanics MPM
// 3 by 3 element (len = 2.0), 2 by 2 particles
void test_chm_2D_bspline_mpm1(void)
{
	Model_R2D_CHM_MPM_BSpline_s model;

	Particle_R2D_CHM_Grid pcl;
	pcl.init();
	pcl.density_s = 2.0;
	pcl.density_f = 1.5;
	pcl.n = 0.5;
	pcl.m_s = 1.0 * pcl.n * pcl.density_s;
	pcl.E = 100.0;
	pcl.niu = 0.3;
	pcl.Kf = 5000.0;
	pcl.k = 1.0e-3;
	pcl.miu = 1.0;
	model.set_pcl_num(4);
	pcl.x = 2.5;
	pcl.y = 2.5;
	//pcl.vx_s = 1.0;
	//pcl.vy_s = 2.0;
	//pcl.vx_f = 2.0;
	//pcl.vy_f = 3.0;
	model.add_pcl(pcl);
	pcl.x = 3.5;
	pcl.y = 2.5;
	//pcl.vx_s = 1.0;
	//pcl.vy_s = 2.0;
	//pcl.vx_f = 2.0;
	//pcl.vy_f = 4.0;
	model.add_pcl(pcl);
	pcl.x = 2.5;
	pcl.y = 3.5;
	//pcl.vx_s = 2.0;
	//pcl.vy_s = 0.0;
	//pcl.vx_f = 3.0;
	//pcl.vy_f = 3.0;
	model.add_pcl(pcl);
	pcl.x = 3.5;
	pcl.y = 3.5;
	//pcl.vx_s = 2.0;
	//pcl.vy_s = 0.0;
	//pcl.vx_f = 3.0;
	//pcl.vy_f = 4.0;
	model.add_pcl(pcl);
	
	model.set_mesh(3, 3, 2.0);

	// velocity bc
	// solid phase
	model.set_vsy_num(8);
	VelocityBC vbc;
	vbc.v = 0.0;
	for (size_t i = 0; i < 8; i++)
	{
		vbc.node_id = i;
		model.add_vsy(vbc);
	}
	model.set_vsx_num(16);
	for (size_t i = 0; i < 16; i++)
	{
		vbc.node_id = i;
		model.add_vsx(vbc);
	}
	// fluid phase
	model.set_vfy_num(8);
	for (size_t i = 0; i < 8; i++)
	{
		vbc.node_id = i;
		model.add_vfy(vbc);
	}
	model.set_vsx_num(16);
	for (size_t i = 0; i < 16; i++)
	{
		vbc.node_id = i;
		model.add_vfx(vbc);
	}

	model.set_ty_num(2);
	TractionBC_MPM tbc;
	tbc.t = -1.0;
	tbc.pcl_id = 2;
	model.add_ty(tbc);
	tbc.pcl_id = 3;
	model.add_ty(tbc);

	model.update();

	ResultFile_Text res_file;
	res_file.set_filename("res_file");
	
	TimeHistory_Particle_2D_CHM_MPM_BSpline_s_AllPcl th1;
	th1.set_name("test_out1");
	th1.set_interval_num(100);
	th1.set_if_output_initial_state(false);
	Particle_Field_2D_CHM fld1[] = {
		Particle_Field_2D_CHM::x,
		Particle_Field_2D_CHM::y,
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

	//Step_1D_CHM_MPM_BSpline_s step1;
	Step_R2D_CHM_MPM_BSpline_APIC_s step1;
	step1.set_name("initial_step");
	step1.set_model(&model);
	step1.set_result_file(&res_file);
	step1.set_step_time(20.0); // total_time
	step1.set_dt(1.0e-3);
	step1.add_output(&th1);

	//step1.init_B_matrix();
	step1.solve();
}