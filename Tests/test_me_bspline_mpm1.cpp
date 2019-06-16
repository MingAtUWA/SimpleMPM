#include "Test_pcp.h"

#include "Model_1D_ME_MPM_BSpline_s.h"

#include "Step_1D_ME_MPM_BSpline_s.h"

#include "TimeHistory_ConsoleProgressBar.h"
#include "TimeHistory_Particle_1D_ME_AllPcl.h"

#include "ResultFile_Text.h"

#include "test_sim_core.h"

// 1D bspline mechanics
// 3 element, 2 pcls
void test_me_bspline_mpm1(void)
{
	size_t i, j, k;
	Model_1D_ME_MPM_BSpline_s model;

	Particle_1D_ME pcl;
	pcl.init();
	pcl.density = 2.0;
	pcl.m = 0.5 * 2.0;
	pcl.E = 100.0;
	model.set_pcl_num(2);
	pcl.x = 1.25;
	model.add_pcl(pcl);
	pcl.x = 1.75;
	model.add_pcl(pcl);

	model.set_mesh(3, 1.0);

	model.set_vbc_num(2);
	VelocityBC vbc;
	vbc.node_id = 0;
	vbc.v = 0.0;
	model.add_vbc(vbc);
	vbc.node_id = 1;
	model.add_vbc(vbc);
	
	model.set_tbc_num(2);
	TractionBC_MPM tbc;
	tbc.pcl_id = 1;
	tbc.t = -1.0;
	model.add_tbc(tbc);

	model.update();

	ResultFile_Text res_file;
	res_file.set_filename("res_file");
	
	Step_1D_ME_MPM_BSpline_s step1;
	step1.set_name("initial_step");
	step1.set_model(&model);
	step1.set_result_file(&res_file);
	step1.set_step_time(1.0); // total_time
	step1.set_dt(1.0e-3);

	TimeHistory_Particle_1D_ME_AllPcl th1;
	th1.set_name("test_out1");
	th1.set_interval_num(10);
	th1.set_if_output_initial_state(false);
	Particle_Field_1D_ME fld1[] = {
		Particle_Field_1D_ME::x,
		Particle_Field_1D_ME::density,
		Particle_Field_1D_ME::v,
		Particle_Field_1D_ME::s11
	};
	th1.set_field_num(sizeof(fld1)/sizeof(fld1[0]));
	for (size_t i = 0; i < sizeof(fld1) / sizeof(fld1[0]); i++)
		th1.add_field(fld1[i]);
	step1.add_output(&th1);

	step1.solve();
}