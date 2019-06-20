#include "Test_pcp.h"

#include "Model_R2D_ME_MPM_BSpline_s.h"

#include "Step_R2D_ME_MPM_BSpline_APIC_s.h"

#include "TimeHistory_ConsoleProgressBar.h"
#include "TimeHistory_Particle_2D_ME_AllPcl_s.h"

#include "ResultFile_Text.h"

#include "test_sim_core.h"

// 2D bspline mechanics MPM
// 3 by 3 element (len = 2.0), 2 by 2 particles
void test_me_2D_bspline_mpm1(void)
{
	Model_R2D_ME_MPM_BSpline_s model;

	Particle_R2D_ME_Grid pcl;
	pcl.init();
	pcl.density = 2.0;
	pcl.m = 1.0 * 2.0;
	pcl.E = 100.0;
	pcl.niu = 0.3;
	model.set_pcl_num(4);
	pcl.x = 2.5;
	pcl.y = 2.5;
	pcl.vx = 3.0;
	pcl.vy = 2.0;
	model.add_pcl(pcl);
	pcl.x = 3.5;
	pcl.y = 2.5;
	pcl.vx = 3.0;
	pcl.vy = 3.0;
	model.add_pcl(pcl);
	pcl.x = 2.5;
	pcl.y = 3.5;
	pcl.vx = 3.0;
	pcl.vy = 3.0;
	model.add_pcl(pcl);
	pcl.x = 3.5;
	pcl.y = 3.5;
	pcl.vx = 3.0;
	pcl.vy = 4.0;
	model.add_pcl(pcl);
	
	model.set_mesh(3, 3, 2.0);

	// velocity bc
	model.set_vy_num(8);
	VelocityBC vbc;
	vbc.v = 0.0;
	for (size_t i = 0; i < 8; i++)
	{
		vbc.node_id = i;
		model.add_vy(vbc);
	}
	model.set_vx_num(16);
	for (size_t i = 0; i < 16; i++)
	{
		vbc.node_id = i;
		model.add_vx(vbc);
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
	
	TimeHistory_Particle_2D_ME_AllPcl_s th1;
	th1.set_name("test_out1");
	th1.set_interval_num(100);
	th1.set_if_output_initial_state(false);
	Particle_Field_2D_ME fld1[] = {
		Particle_Field_2D_ME::x,
		Particle_Field_2D_ME::y,
		Particle_Field_2D_ME::density,
		Particle_Field_2D_ME::vx,
		Particle_Field_2D_ME::vy,
		Particle_Field_2D_ME::s11
	};
	th1.set_field_num(sizeof(fld1) / sizeof(fld1[0]));
	for (size_t i = 0; i < sizeof(fld1) / sizeof(fld1[0]); i++)
		th1.add_field(fld1[i]);

	//Step_1D_ME_MPM_BSpline_s step1;
	Step_R2D_ME_MPM_BSpline_APIC_s step1;
	step1.set_name("initial_step");
	step1.set_model(&model);
	step1.set_result_file(&res_file);
	step1.set_step_time(20.0); // total_time
	step1.set_dt(1.0e-3);
	step1.add_output(&th1);

	//step1.solve();
	step1.init_B_matrix();
}