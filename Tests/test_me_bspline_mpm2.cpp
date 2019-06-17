#include "Test_pcp.h"

#include "Model_1D_ME_MPM_BSpline_s.h"

#include "Step_1D_ME_MPM_BSpline_s.h"

#include "TimeHistory_ConsoleProgressBar.h"
#include "TimeHistory_Particle_1D_ME_AllPcl.h"

#include "ResultFile_Text.h"

#include "test_sim_core.h"

// 1D bspline mechanics
// compression
void test_me_bspline_mpm2(void)
{
	Model_1D_ME_MPM_BSpline_s model;
	size_t elem_num = 100;
	size_t pcl_per_elem_num = 5;
	
	double elem_len = 1.0 / double(elem_num);
	size_t pcl_num = elem_num * pcl_per_elem_num;
	double pcl_len = 1.0 / double(pcl_num);

	Particle_1D_ME pcl;
	pcl.init();
	pcl.density = 20.0;
	pcl.m = pcl_len * pcl.density;
	pcl.E = 100.0;
	model.set_pcl_num(pcl_num);
	for (size_t i = 0; i < pcl_num; i++)
	{
		pcl.x = elem_len + (0.5 + i) * pcl_len;
		model.add_pcl(pcl);
	}

	model.set_mesh(elem_num+3, elem_len);

	model.set_vbc_num(2);
	VelocityBC vbc;
	vbc.v = 0.0;
	vbc.node_id = 0;
	model.add_vbc(vbc);
	vbc.node_id = 1;
	model.add_vbc(vbc);
	
	model.set_tbc_num(1);
	TractionBC_MPM tbc;
	tbc.t = -20.0;
	tbc.pcl_id = pcl_num - 1;
	model.add_tbc(tbc);

	model.update();

	ResultFile_Text res_file;
	res_file.set_filename("res_file");
	
	TimeHistory_Particle_1D_ME_AllPcl th1;
	th1.set_name("test_out1");
	th1.set_interval_num(200);
	th1.set_if_output_initial_state(false);
	Particle_Field_1D_ME fld1[] = {
		Particle_Field_1D_ME::x,
		Particle_Field_1D_ME::density,
		Particle_Field_1D_ME::v,
		Particle_Field_1D_ME::s11
	};
	th1.set_field_num(sizeof(fld1) / sizeof(fld1[0]));
	for (size_t i = 0; i < sizeof(fld1) / sizeof(fld1[0]); i++)
		th1.add_field(fld1[i]);

	Step_1D_ME_MPM_BSpline_s step1;
	step1.set_name("initial_step");
	step1.set_model(&model);
	step1.set_result_file(&res_file);
	step1.set_step_time(10.0); // total_time
	step1.set_dt(1.0e-3);
	step1.add_output(&th1);

	step1.solve();
}