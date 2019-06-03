#include "Test_pcp.h"

#include "Model_R2D_CHM_MPM_s.h"

#include "ResultFile_HDF5.h"
#include "ResultFile_Text.h"

#include "TimeHistory_ConsoleProgressBar.h"
#include "TimeHistory_Particle_R2D_CHM_s.h"
#include "Step_R2D_CHM_MPM_s.h"

#include "test_sim_core.h"

// 1 by 1 element with 4 particles CHM
void test_chm_mpm1(void)
{
	size_t i, j, k;
	Model_R2D_CHM_MPM_s model;

	model.node_x_num = 2;
	model.node_y_num = 2;
	model.node_coords_x = new double[model.node_x_num];
	for (i = 0; i < model.node_x_num; i++)
	{
		model.node_coords_x[i] = (double)i;
	}
	model.node_coords_y = new double[model.node_y_num];
	for (i = 0; i < model.node_y_num; i++)
	{
		model.node_coords_y[i] = (double)i;
	}
	model.node_num = model.node_x_num * model.node_y_num;
	model.nodes = new Node_R2D_CHM_s[model.node_num];
	k = 0;
	for (i = 0; i < model.node_y_num; i++)
		for (j = 0; j < model.node_x_num; j++)
		{
			model.nodes[k].index_x = j;
			model.nodes[k].index_y = i;
			++k;
		}

	model.elem_x_num = model.node_x_num - 1;
	model.elem_y_num = model.node_y_num - 1;
	model.elem_num = model.elem_x_num * model.elem_y_num;
	model.elems = new Element_R2D_CHM_MPM_s[model.elem_num];
	k = 0;
	for (i = 0; i < model.elem_y_num; i++)
		for (j = 0; j < model.elem_x_num; j++)
		{
			model.elems[k].index_x = j;
			model.elems[k].index_y = i;
			++k;
		}

	model.pcl_num = model.elem_x_num * model.elem_y_num * 4;
	model.pcls = new Particle_R2D_CHM_s[model.pcl_num];
	Particle_R2D_CHM_s *ppcl;
	k = 0;
	for (i = 0; i < model.elem_y_num * 2; i++)
		for (j = 0; j < model.elem_x_num * 2; j++)
		{
			ppcl = model.pcls + k;
			// initialize particles
			ppcl->x = 0.25 + (double)j * 0.5;
			ppcl->y = 0.25 + (double)i * 0.5;
			ppcl->vol = 0.5 * 0.5;
			ppcl->n = 0.3;
			ppcl->density_s = 2650.0;
			ppcl->density_f = 1000.0;
			ppcl->vx_s = 0.0;
			ppcl->vy_s = 0.0;
			ppcl->vx_f = 0.0;
			ppcl->vy_f = 0.0;
			ppcl->s11 = 0.0;
			ppcl->s22 = 0.0;
			ppcl->s12 = 0.0;
			ppcl->s33 = 0.0;
			ppcl->s23 = 0.0;
			ppcl->s31 = 0.0;
			ppcl->p = 0.0;
			// parameters blow does **not** have
			// much impact on calculation.
			ppcl->e11 = 0.0;
			ppcl->e22 = 0.0;
			ppcl->e12 = 0.0;
			ppcl->es11 = 0.0;
			ppcl->es22 = 0.0;
			ppcl->es12 = 0.0;
			ppcl->ps11 = 0.0;
			ppcl->ps22 = 0.0;
			ppcl->ps12 = 0.0;
			// constitutive models
			ppcl->E = 1.0e3;
			ppcl->niu = 0.3;
			ppcl->Kf = 50.0e3;
			ppcl->k = 1.0e-4;
			ppcl->miu = 1.0;

			++k;
		}

	model.bfx_num = 0;
	model.bfxs = nullptr;
	model.bfy_num = 0;
	model.bfys = nullptr;

	model.tx_bc_num = 0;
	model.tx_bcs = nullptr;
	model.ty_bc_num = model.elem_x_num * 2;
	model.ty_bcs = new TractionBC_MPM[model.ty_bc_num];
	for (i = 0; i < model.ty_bc_num; i++)
	{
		model.ty_bcs[i].pcl_id = (model.elem_y_num * 2 - 1) * model.elem_x_num * 2 + i;
		model.ty_bcs[i].t = -10.0 * 0.5;
	}

	model.vx_s_bc_num = 0;
	model.vx_s_bcs = nullptr;
	model.ax_s_bc_num = model.node_y_num * 2;
	model.ax_s_bcs = new AccelerationBC[model.ax_s_bc_num];
	for (i = 0; i < model.node_y_num; i++)
	{
		model.ax_s_bcs[i].node_id = i * model.node_x_num;
		model.ax_s_bcs[i].a = 0.0;
		model.ax_s_bcs[i + model.node_y_num].node_id = (i + 1) * model.node_x_num - 1;
		model.ax_s_bcs[i + model.node_y_num].a = 0.0;
	}

	model.vy_s_bc_num = 0;
	model.vy_s_bcs = nullptr;
	model.ay_s_bc_num = model.node_x_num;
	model.ay_s_bcs = new AccelerationBC[model.ay_s_bc_num];
	for (i = 0; i < model.node_x_num; i++)
	{
		model.ay_s_bcs[i].node_id = i;
		model.ay_s_bcs[i].a = 0.0;
	}

	model.vx_f_bc_num = 0;
	model.vx_f_bcs = nullptr;
	model.ax_f_bc_num = model.node_y_num * 2;
	model.ax_f_bcs = new AccelerationBC[model.ax_f_bc_num];
	for (i = 0; i < model.node_y_num; i++)
	{
		model.ax_f_bcs[i].node_id = i * model.node_x_num;
		model.ax_f_bcs[i].a = 0.0;
		model.ax_f_bcs[i + model.node_y_num].node_id = (i + 1) * model.node_x_num - 1;
		model.ax_f_bcs[i + model.node_y_num].a = 0.0;
	}

	model.vy_f_bc_num = 0;
	model.vy_f_bcs = nullptr;
	model.ay_f_bc_num = model.node_x_num * 2;
	model.ay_f_bcs = new AccelerationBC[model.ay_f_bc_num];
	for (i = 0; i < model.node_x_num; i++)
	{
		model.ay_f_bcs[i].node_id = i;
		model.ay_f_bcs[i].a = 0.0;
		model.ay_f_bcs[i + model.node_x_num].node_id = model.node_x_num * (model.node_y_num - 1) + i;
		model.ay_f_bcs[i + model.node_x_num].a = 0.0;
	}

	ResultFile_Text res_file;
	res_file.set_filename("res_file");

	res_file.output_model_state(model);

	TimeHistory_Particle_R2D_CHM_s *th1;
	th1 = new TimeHistory_Particle_R2D_CHM_s;
	th1->set_name("test_out1");
	th1->set_if_output_initial_state(true);
	Particle_Field_R2D_CHM_s fld1[10] = {
		Particle_Field_R2D_CHM_s::x,
		Particle_Field_R2D_CHM_s::y,
		Particle_Field_R2D_CHM_s::vol,
		Particle_Field_R2D_CHM_s::p,
		Particle_Field_R2D_CHM_s::n,
		Particle_Field_R2D_CHM_s::vx_s,
		Particle_Field_R2D_CHM_s::vy_s,
		Particle_Field_R2D_CHM_s::e11,
		Particle_Field_R2D_CHM_s::e12,
		Particle_Field_R2D_CHM_s::e22
	};
	size_t *pcl_ids1;
	pcl_ids1 = new size_t[model.pcl_num];
	for (i = 0; i < model.pcl_num; i++)
		pcl_ids1[i] = i;
	th1->set_model_output(&model, fld1, 9, pcl_ids1, model.pcl_num);
	delete[] pcl_ids1;

	TimeHistory_ConsoleProgressBar th2;

	Step_R2D_CHM_MPM_s *step1;
	step1 = new Step_R2D_CHM_MPM_s;
	step1->set_name("initial_step");
	step1->set_model(&model);
	step1->set_result_file(&res_file);
	step1->set_step_time(5.0e-4); // total_time
	step1->set_dt(1.0e-4);

	th1->set_interval_num(5);
	step1->add_output(th1);
	step1->add_output(&th2);

	step1->solve();

	Step_R2D_CHM_MPM_s *step2;
	step2 = new Step_R2D_CHM_MPM_s;
	step2->set_name("consolidation_step");
	step2->set_prev_step(step1);
	delete step1;
	step2->set_step_time(5.0e-4); // total_time
	step2->set_dt(1.0e-4);

	// free drainage bcs
	model.ay_f_bc_num = model.node_x_num;
	th1->set_interval_num(5);
	step2->add_output(th1);
	step2->add_output(&th2);

	step2->solve();
	
	delete step2;

	delete th1;
}