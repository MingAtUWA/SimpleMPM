#include "Test_pcp.h"

#include "Model_R2D_ME_MPM_s.h"

#include "ResultFile_HDF5.h"
#include "ResultFile_Text.h"

#include "TimeHistory_Particle_R2D_ME_s.h"
#include "TimeHistory_ConsoleProgressBar.h"
#include "Step_R2D_ME_MPM_s.h"

#include "test_sim_core.h"

// One diemnsional compression
// 16+ points in each element
void test_me_mpm5(void)
{
	size_t i, j, k;
	Model_R2D_ME_MPM_s model;
	double elem_len;
	size_t elem_div_num; // each element has elem_div_num * elem_div_num particles
	double pcl_len;

	model.node_x_num = 3;
	model.node_y_num = 16;
	elem_len = 1.0 / (model.node_y_num - 1);

	model.node_coords_x = new double[model.node_x_num];
	for (i = 0; i < model.node_x_num; i++)
		model.node_coords_x[i] = (double)i * elem_len;

	model.node_coords_y = new double[model.node_y_num];
	for (i = 0; i < model.node_y_num; i++)
		model.node_coords_y[i] = (double)i * elem_len;

	model.node_num = model.node_x_num * model.node_y_num;
	model.nodes = new Node_R2D_ME_s[model.node_num];
	k = 0;
	for (i = 0; i < model.node_y_num; ++i)
		for (j = 0; j < model.node_x_num; ++j)
		{
			model.nodes[k].index_x = j;
			model.nodes[k].index_y = i;
			++k;
		}

	model.elem_x_num = model.node_x_num - 1;
	model.elem_y_num = model.node_y_num - 1;
	model.elem_num = model.elem_x_num * model.elem_y_num;
	model.elems = new Element_R2D_ME_MPM_s[model.elem_num];
	k = 0;
	for (i = 0; i < model.elem_y_num; i++)
		for (j = 0; j < model.elem_x_num; j++)
		{
			model.elems[k].index_x = j;
			model.elems[k].index_y = i;
			++k;
		}

	elem_div_num = 8;
	pcl_len = elem_len / elem_div_num;
	model.pcl_num = model.elem_x_num * model.elem_y_num * elem_div_num * elem_div_num;
	model.pcls = new Particle_R2D_ME_s[model.pcl_num];
	Particle_R2D_ME_s *ppcl;
	k = 0;
	for (i = 0; i < model.elem_y_num * elem_div_num; i++)
		for (j = 0; j < model.elem_x_num * elem_div_num; j++)
		{
			ppcl = model.pcls + k;
			// initialize particles
			ppcl->x = (0.5 + (double)j) * pcl_len;
			ppcl->y = (0.5 + (double)i) * pcl_len;
			ppcl->density = 2650.0;
			ppcl->m = pcl_len * pcl_len * ppcl->density;
			ppcl->mmx = 0.0;
			ppcl->mmy = 0.0;
			ppcl->s11 = 0.0;
			ppcl->s22 = 0.0;
			ppcl->s12 = 0.0;
			ppcl->s33 = 0.0;
			ppcl->s23 = 0.0;
			ppcl->s31 = 0.0;
			ppcl->e11 = 0.0;
			ppcl->e22 = 0.0;
			ppcl->e12 = 0.0;
			ppcl->E   = 1000.0;
			ppcl->niu = 0.25;

			++k;
		}
	
	model.bfx_num = 0;
	model.bfxs = nullptr;
	model.bfy_num = 0;
	model.bfys = nullptr;

	model.tx_bc_num = 0;
	model.tx_bcs = nullptr;
	model.ty_bc_num = model.elem_x_num * elem_div_num;
	model.ty_bcs = new TractionBC_MPM[model.ty_bc_num];
	for (i = 0; i < model.ty_bc_num; i++)
	{
		model.ty_bcs[i].pcl_id = (model.elem_y_num * elem_div_num - 1) * model.elem_x_num * elem_div_num + i;
		model.ty_bcs[i].t = -400.0 * pcl_len;
	}

	// don't use acceleration bcs for fixed boundary condition
	model.ax_s_bc_num = 0;
	model.ax_s_bcs = nullptr;
	// don't use acceleration bcs for fixed boundary condition
	model.ay_s_bc_num = 0;
	model.ay_s_bcs = nullptr;

	model.vx_s_bc_num = model.node_y_num * 2;
	model.vx_s_bcs = new VelocityBC[model.vx_s_bc_num];
	for (i = 0; i < model.node_y_num; i++)
	{
		model.vx_s_bcs[i].node_id = i * model.node_x_num;
		model.vx_s_bcs[i].v = 0.0;
		model.vx_s_bcs[i + model.node_y_num].node_id = (i + 1) * model.node_x_num - 1;
		model.vx_s_bcs[i + model.node_y_num].v = 0.0;
	}

	model.vy_s_bc_num = model.node_x_num;
	model.vy_s_bcs = new VelocityBC[model.vy_s_bc_num];
	for (i = 0; i < model.vy_s_bc_num; i++)
	{
		model.vy_s_bcs[i].node_id = i;
		model.vy_s_bcs[i].v = 0.0;
	}

	ResultFile_Text res_file;
	res_file.set_filename("res_file");

	res_file.output_model_state(model);

	TimeHistory_Particle_R2D_ME_s th1;
	th1.set_name("test_out1");
	th1.set_interval_num(100);
	th1.set_if_output_initial_state(false);
	Particle_Field_R2D_ME_s fld1[8] = {
		Particle_Field_R2D_ME_s::x,
		Particle_Field_R2D_ME_s::y,
		Particle_Field_R2D_ME_s::vol,
		Particle_Field_R2D_ME_s::v_y,
		Particle_Field_R2D_ME_s::v_x,
		Particle_Field_R2D_ME_s::e11,
		Particle_Field_R2D_ME_s::e12,
		Particle_Field_R2D_ME_s::e22
	};
	size_t *pcl_ids1;
	pcl_ids1 = (size_t *)alloca(sizeof(size_t) * model.pcl_num);
	for (i = 0; i < model.pcl_num; i++) pcl_ids1[i] = i;
	th1.set_model_output(&model, fld1, 8, pcl_ids1, model.pcl_num);

	TimeHistory_ConsoleProgressBar th2;

	Step_R2D_ME_MPM_s step1;
	step1.set_name("initial_step");
	step1.set_model(&model);
	step1.set_result_file(&res_file);
	step1.set_step_time(100.0);
	step1.set_dt(1.0e-4);
	//step1.set_auto_dt(0.01);
	step1.add_output(&th1);
	step1.add_output(&th2);

	step1.solve();
}