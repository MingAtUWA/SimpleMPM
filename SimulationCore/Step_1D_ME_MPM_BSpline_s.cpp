#include "SimulationCore_pcp.h"

#include "Step_1D_ME_MPM_BSpline_s.h"

Step_1D_ME_MPM_BSpline_s::Step_1D_ME_MPM_BSpline_s() :
	Step(&solve_substep_1D_ME_MPM_BSpline_s),
	model(nullptr), N_tol(1.0e-8) {}

Step_1D_ME_MPM_BSpline_s::~Step_1D_ME_MPM_BSpline_s() {}

int Step_1D_ME_MPM_BSpline_s::init()
{
	Particle_1D_ME *ppcl;

	if (is_first_step)
	{
		for (size_t i = 0; i < model->pcl_num; ++i)
		{
			ppcl = model->pcls + i;
			ppcl->is_in_mesh = true;
		}
	}

	for (size_t i = 0; i < model->pcl_num; i++)
	{
		ppcl = model->pcls + i;
		ppcl->x_ori = ppcl->x;
		ppcl->u = 0.0;
	}

	return 0;
}

int Step_1D_ME_MPM_BSpline_s::finalize() {	return 0; }

int solve_substep_1D_ME_MPM_BSpline_s(void *_self)
{
	Step_1D_ME_MPM_BSpline_s *self = static_cast<Step_1D_ME_MPM_BSpline_s *>(_self);
	Model_1D_ME_MPM_BSpline_s *model = self->model;
	Element_1D_ME_Grid *pelem;
	Node_1D_ME_Grid *pn;
	Particle_1D_ME *ppcl;

	// init nodes
	for (size_t i = 0; i < model->node_num; i++)
	{
		pn = model->nodes + i;
		pn->cal_flag = 0;
		// particles to nodes
		pn->m = 0.0;
		pn->mv = 0.0;
		pn->f_ext = 0.0;
		pn->f_int = 0.0;
	}

	// init particles
	for (size_t i = 0; i < model->pcl_num; i++)
	{
		ppcl = model->pcls + i;
		if (ppcl->is_in_mesh)
		{
			ppcl->node_x_id = model->nearest_node_x_index(ppcl->x);
			if (!ppcl->node_x_id)
			{
				ppcl->is_in_mesh = false;
			}
		}
	}

	// map variables to node and cal internal force
	size_t x_index;
	double xi;
	double vol, N, dN_dx;
	for (size_t i = 0; i < model->pcl_num; i++)
	{
		ppcl = model->pcls + i;
		if (ppcl->is_in_mesh)
		{
			vol = ppcl->m / ppcl->density;
			for (size_t i = 0, x_index = ppcl->node_x_id - 1;
				i < 3; ++i, ++x_index)
			{
				xi = model->xi(ppcl->x, x_index);
				ppcl->N[i] = model->N(xi);
				if (ppcl->N[i] > self->N_tol)
				{
					ppcl->dN_dx[i] = model->dN_dx(xi);
					pn = model->get_node_by_index(x_index);
					pn->cal_flag = 1;
					pn->m += ppcl->m  * ppcl->N[i];
					pn->mv += ppcl->mv * ppcl->N[i];
					pn->f_int += vol * (ppcl->dN_dx[i] * ppcl->s11);
				}
				else
				{
					pn->a = 0.0;
					pn->v = 0.0;
					pn->du = 0.0;
				}
			}
		}
	}

	// body force
	double bf_tmp;
	for (size_t i = 0; i < model->bf_num; i++)
	{
		ppcl = model->pcls + model->bfs[i].pcl_id;
		bf_tmp = ppcl->m * model->bfs[i].bf;
		if (ppcl->is_in_mesh)
		{
			for (size_t i = 0, x_index = ppcl->node_x_id - 1;
				i < 3; ++i, ++x_index)
			{
				pn = model->get_node_by_index(x_index);
				pn->f_ext += bf_tmp * ppcl->N[i];
			}
		}
	}

	// surface force
	for (size_t i = 0; i < model->tbc_num; i++)
	{
		ppcl = model->pcls + model->tbcs[i].pcl_id;
		if (ppcl->is_in_mesh)
		{
			for (size_t i = 0, x_index = ppcl->node_x_id - 1;
				 i < 3; ++i, ++x_index)
			{
				pn = model->get_node_by_index(x_index);
				pn->f_ext += model->tbcs[i].t * ppcl->N[i];
			}
		}
	}
	
	// update nodal acceleration
	for (size_t i = 0; i < model->node_num; i++)
	{
		pn = model->nodes + i;
		if (pn->cal_flag) pn->a = (pn->f_ext - pn->f_int) / pn->m;
	}
	// apply acceleration boundary conditions
	for (size_t i = 0; i < model->abc_num; i++)
	{
		pn = model->nodes + model->abcs[i].node_id;
		if (pn->cal_flag) pn->a = model->abcs[i].a;
	}

	// update nodal momentum
	for (size_t i = 0; i < model->node_num; i++)
	{
		pn = model->nodes + i;
		if (pn->cal_flag) pn->v = pn->mv / pn->m + pn->a * self->dt;
	}
	// apply velocity boundary conditions
	for (size_t i = 0; i < model->vbc_num; i++)
	{
		pn = model->nodes + model->vbcs[i].node_id;
		if (pn->cal_flag) pn->v = model->vbcs[i].v;
	}
	
	// update displacement increment
	for (size_t i = 0; i < model->node_num; i++)
	{
		pn = model->nodes + i;
		if (pn->cal_flag) pn->du = pn->v * self->dt;
	}

	// map variables back to and update variables particles
	double N1_tmp, N2_tmp, N3_tmp, N4_tmp;
	double de_vol, ds11, ds22, ds12;
	double E_tmp;
	for (size_t i = 0; i < model->pcl_num; i++)
	{
		ppcl = model->pcls + i;
		if (ppcl->is_in_mesh)
		{
			ppcl->de11 = 0.0;

			for (size_t i = 0, x_index = ppcl->node_x_id - 1;
				i < 3; ++i, ++x_index)
			{
				pn = model->get_node_by_index(x_index);
				// momentum (velocity)
				ppcl->mv += ppcl->m * self->dt * pn->a * ppcl->N[i];

				// displacement
				ppcl->u += pn->du * ppcl->N[i];
			
				// strain increment
				ppcl->de11 += pn->du * ppcl->dN_dx[i];
			}

			// update position
			ppcl->x = ppcl->x_ori + ppcl->u;

			// update variables at particles
			de_vol = ppcl->de11;
			ppcl->density /= (1.0 + de_vol);

			// update strain (also assume that strain increment is Jaumann rate)
			ppcl->e11 += ppcl->de11;

			// update stress
			ppcl->s11 += ppcl->E * ppcl->de11;
		}
	}

	return 0;
}