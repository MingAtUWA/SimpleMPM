#include "SimulationCore_pcp.h"

#include "Step_1D_ME_MPM_BSpline_s.h"

Step_1D_ME_MPM_BSpline_s::Step_1D_ME_MPM_BSpline_s() :
	Step(&solve_substep_1D_ME_MPM_BSpline_s),
	model(nullptr), N_tol(1.0e-8) {}

Step_1D_ME_MPM_BSpline_s::~Step_1D_ME_MPM_BSpline_s() {}

int Step_1D_ME_MPM_BSpline_s::init()
{
	if (is_first_step)
	{
		for (size_t i = 0; i < model->pcl_num; ++i)
		{
			auto &pcl = model->pcls[i];
			pcl.is_in_mesh = true;
		}
	}

	for (size_t i = 0; i < model->pcl_num; i++)
	{
		auto &pcl = model->pcls[i];
		pcl.x_ori = pcl.x;
		pcl.u = 0.0;
	}

	return 0;
}

int Step_1D_ME_MPM_BSpline_s::finalize() { return 0; }

int solve_substep_1D_ME_MPM_BSpline_s(void *_self)
{
	Step_1D_ME_MPM_BSpline_s *self = static_cast<Step_1D_ME_MPM_BSpline_s *>(_self);
	Model_1D_ME_MPM_BSpline_s *model = self->model;

	// init nodes
	for (size_t i = 0; i < model->node_num; i++)
	{
		auto &pn = model->nodes[i];
		pn.cal_flag = 0;
		// particles to nodes
		pn.m = 0.0;
		pn.mv = 0.0;
		pn.f_ext = 0.0;
		pn.f_int = 0.0;
		// nodes to particles
		pn.a = 0.0;
		pn.v = 0.0;
		pn.du = 0.0;
	}

	// init particles
	for (size_t pcl_id = 0; pcl_id < model->pcl_num; ++pcl_id)
	{
		auto &pcl = model->pcls[pcl_id];
		if (pcl.is_in_mesh)
		{
			pcl.base_node_id = model->nearest_node_x_index(pcl.x);
			if (pcl.base_node_id)
			{
				--pcl.base_node_id;
			}
			else
			{
				pcl.is_in_mesh = false;
			}
		}
	}

	// map variables to node and cal internal force
	for (size_t pcl_id = 0; pcl_id < model->pcl_num; ++pcl_id)
	{
		auto &pcl = model->pcls[pcl_id];
		if (pcl.is_in_mesh)
		{
			double vol = pcl.m / pcl.density;
			for (size_t n_id = 0; n_id < 3; ++n_id)
			{
				double xi = model->xi(pcl.x, pcl.base_node_id + n_id);
				pcl.N[n_id] = model->N(xi);
				pcl.dN_dx[n_id] = model->dN_dx(xi);
				auto &pn = model->get_node_by_index(pcl.base_node_id + n_id);
				//if (pcl.N[n_id] > self->N_tol)
				if (pcl.N[n_id] != 0.0)
					pn.cal_flag = 1;
				// map to nodes
				pn.m  += pcl.m * pcl.N[n_id];
				pn.mv += pcl.m * pcl.v * pcl.N[n_id];
				pn.f_int += vol * (pcl.dN_dx[n_id] * pcl.s11);
			}
		}
	}

	// body force
	for (size_t bf_id = 0; bf_id < model->bf_num; ++bf_id)
	{
		auto &pcl = model->pcls[model->bfs[bf_id].pcl_id];
		if (pcl.is_in_mesh)
		{
			double bf_tmp = pcl.m * model->bfs[bf_id].bf;
			for (size_t n_id = 0; n_id < 3; ++n_id)
			{
				auto &pn = model->get_node_by_index(pcl.base_node_id + n_id);
				pn.f_ext += bf_tmp * pcl.N[n_id];
			}
		}
	}

	// surface force
	for (size_t sf_id = 0; sf_id < model->tbc_num; sf_id++)
	{
		auto &pcl = model->pcls[model->tbcs[sf_id].pcl_id];
		if (pcl.is_in_mesh)
		{
			double tf_tmp = model->tbcs[sf_id].t;
			for (size_t n_id = 0; n_id < 3; ++n_id)
			{
				auto &pn = model->get_node_by_index(pcl.base_node_id + n_id);
				pn.f_ext += tf_tmp * pcl.N[n_id];
			}
		}
	}
	
	// update nodal acceleration
	for (size_t i = 0; i < model->node_num; i++)
	{
		auto &pn = model->nodes[i];
		if (pn.cal_flag)
			pn.a = (pn.f_ext - pn.f_int) / pn.m;
	}
	// apply acceleration boundary conditions
	for (size_t i = 0; i < model->abc_num; i++)
	{
		auto &pn = model->nodes[model->abcs[i].node_id];
		pn.a = model->abcs[i].a;
	}

	// update nodal momentum
	for (size_t i = 0; i < model->node_num; i++)
	{
		auto &pn = model->nodes[i];
		if (pn.cal_flag)
			pn.v = pn.mv / pn.m + pn.a * self->dt;
	}
	// apply velocity boundary conditions
	for (size_t i = 0; i < model->vbc_num; i++)
	{
		auto &pn = model->nodes[model->vbcs[i].node_id];
		pn.v = model->vbcs[i].v;
	}
	
	// update displacement increment
	for (size_t i = 0; i < model->node_num; i++)
	{
		auto &pn = model->nodes[i];
		if (pn.cal_flag)
			pn.du = pn.v * self->dt;
	}

	// map variables back to and update variables particles
	for (size_t pcl_id = 0; pcl_id < model->pcl_num; ++pcl_id)
	{
		auto &pcl = model->pcls[pcl_id];
		if (pcl.is_in_mesh)
		{
			pcl.de11 = 0.0;

			auto &pn0 = model->get_node_by_index(pcl.base_node_id);
			auto &pn1 = model->get_node_by_index(pcl.base_node_id+1);
			auto &pn2 = model->get_node_by_index(pcl.base_node_id+2);

			// velocity combine PIC and FLIP
			double FLIP_portion = 0.95;
			pcl.v = (1.0 - FLIP_portion) * (pn0.v * pcl.N[0] + pn1.v * pcl.N[1] + pn2.v * pcl.N[2])
				  + FLIP_portion * (pcl.v + (pn0.a * pcl.N[0] + pn1.a * pcl.N[1] + pn2.a * pcl.N[2]) * self->dt);
			//pcl.v += (pn0.a * pcl.N[0] + pn1.a * pcl.N[1] + pn2.a * pcl.N[2]) * self->dt;
			// displacement
			pcl.u  += pn0.du * pcl.N[0] + pn1.du * pcl.N[1]+ pn2.du * pcl.N[2];
			// strain increment
			pcl.de11 += pn0.du * pcl.dN_dx[0] + pn1.du * pcl.dN_dx[1] + pn2.du * pcl.dN_dx[2];

			// update position
			pcl.x = pcl.x_ori + pcl.u;

			// update variables at particles
			double de_vol = pcl.de11;
			pcl.density /= (1.0 + de_vol);

			// update strain (also assume that strain increment is Jaumann rate)
			pcl.e11 += pcl.de11;

			// update stress
			pcl.s11 += pcl.E * pcl.de11;
		}
	}

	return 0;
}