#include "Test_pcp.h"

#include "Model_R2D_ME_MPM_s.h"
#include "ResultFile_Text.h"
#include "TimeHistory_Particle_2D_ME.h"
#include "TimeHistory_ConsoleProgressBar.h"
#include "Step_R2D_ME_MPM_s.h"

#include "test_sim_core.h"

ResultFile_Text file;

void test_text_resultfile(void)
{
	size_t i;
	Model_R2D_ME_MPM_s model;

	model.pcl_num = 4;
	model.pcls = new Particle_2D_ME[model.pcl_num];
	Particle_2D_ME *ppcl;
	for (i = 0; i < model.pcl_num; i++)
	{
		ppcl = model.pcls + i;
		// initialize particles
		ppcl->x = (double)i;
		ppcl->y = (double)i;
		ppcl->density = 2.0;
		ppcl->m = 10.0;
		ppcl->mmx = 0.0;
		ppcl->mmy = 0.0;
		ppcl->s11 = 0.0;
		ppcl->s22 = 0.0;
		ppcl->s12 = 0.0;
		ppcl->s33 = 0.0;
		ppcl->s23 = 0.0;
		ppcl->s31 = 0.0;
		ppcl->e11 = 0.11;
		ppcl->e22 = 0.22;
		ppcl->e12 = 0.12;
	}

	ResultFile_Text res_file;
	res_file.set_filename("res_file");
	res_file.init();

	Step_R2D_ME_MPM_s *step1;
	step1 = new Step_R2D_ME_MPM_s;
	step1->set_name("initial_step");
	step1->set_model(&model);
	step1->set_result_file(&res_file);
	step1->set_step_time(10.0);
	step1->set_dt(0.1);

	TimeHistory_Particle_2D_ME *th1;
	th1 = new TimeHistory_Particle_2D_ME;
	th1->set_name("test_out1");
	th1->set_interval_num(2);
	th1->set_if_output_initial_state(false);
	Particle_2D_ME_Field fld1[5] = {
		Particle_2D_ME_Field::x,
		Particle_2D_ME_Field::y,
		Particle_2D_ME_Field::e11,
		Particle_2D_ME_Field::e12,
		Particle_2D_ME_Field::e22
	};
	size_t pcl_ids1[4] = { 0, 1, 2, 3 };
	th1->set_model_output(&model, fld1, 5, pcl_ids1, 4);

	TimeHistory_Particle_2D_ME *th2;
	th2 = new TimeHistory_Particle_2D_ME;
	th2->set_name("test_out2");
	th2->set_interval_num(2);
	th2->set_if_output_initial_state(false);
	Particle_2D_ME_Field fld2[3] = {
		Particle_2D_ME_Field::s11,
		Particle_2D_ME_Field::s12,
		Particle_2D_ME_Field::s22
	};
	size_t pcl_ids2[4] = { 0, 1, 2, 3 };
	th2->set_model_output(&model, fld2, 3, pcl_ids2, 4);

	TimeHistory_ConsoleProgressBar th3;

	step1->add_output(th1);
	//step1->add_output(th2);
	step1->add_output(&th3);

	step1->init_output();

	step1->output_time_history_anyway();

	for (i = 0; i < model.pcl_num; i++)
	{
		ppcl = model.pcls + i;
		// initialize particles
		ppcl->x = (double)i + 1;
		ppcl->y = (double)i + 2;
		ppcl->density = 2.0;
		ppcl->m = 10.0;
		ppcl->mmx = 0.0;
		ppcl->mmy = 0.0;
		ppcl->s11 = 0.0;
		ppcl->s22 = 0.0;
		ppcl->s12 = 0.0;
		ppcl->s33 = 0.0;
		ppcl->s23 = 0.0;
		ppcl->s31 = 0.0;
		ppcl->e11 = 0.1111;
		ppcl->e22 = 0.2222;
		ppcl->e12 = 0.1212;
	}
	step1->output_time_history_anyway();

	for (i = 0; i < model.pcl_num; i++)
	{
		ppcl = model.pcls + i;
		// initialize particles
		ppcl->x = (double)i + 2;
		ppcl->y = (double)i + 4;
		ppcl->density = 2.0;
		ppcl->m = 10.0;
		ppcl->mmx = 0.0;
		ppcl->mmy = 0.0;
		ppcl->s11 = 0.0;
		ppcl->s22 = 0.0;
		ppcl->s12 = 0.0;
		ppcl->s33 = 0.0;
		ppcl->s23 = 0.0;
		ppcl->s31 = 0.0;
		ppcl->e11 = 0.111111;
		ppcl->e22 = 0.222222;
		ppcl->e12 = 0.121212;
	}
	step1->output_time_history_anyway();

	step1->finalize_output();

	//step1->solve();
	delete step1;
	delete th1;
	delete th2;
}