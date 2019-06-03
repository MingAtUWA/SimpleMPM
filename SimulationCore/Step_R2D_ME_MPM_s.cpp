#include "SimulationCore_pcp.h"

#include "Step_R2D_ME_MPM_s.h"

Step_R2D_ME_MPM_s::Step_R2D_ME_MPM_s() :
	Step(&solve_substep_R2D_ME_MPM_s),
	model(nullptr),
	elem_len_min(std::numeric_limits<double>::max()),
	dt_adjust_factor(0.2) {}

Step_R2D_ME_MPM_s::~Step_R2D_ME_MPM_s() {}

int Step_R2D_ME_MPM_s::init()
{
	Particle_R2D_ME_s *ppcl;

	if (is_first_step)
	{
		for (size_t i = 0; i < model->pcl_num; ++i)
		{
			ppcl = model->pcls + i;
			ppcl->is_in_mesh = true;
			ppcl->elem = nullptr;
		}
	}

	for (size_t i = 0; i < model->pcl_num; i++)
	{
		ppcl = model->pcls + i;
		ppcl->x_ori = ppcl->x;
		ppcl->y_ori = ppcl->y;
		ppcl->ux = 0.0;
		ppcl->uy = 0.0;
	}

	// characteristic length of element
	for (size_t i = 0; i < model->elem_num; i++)
		model->cal_characteristic_length(model->elems + i);

	return 0;
}

int Step_R2D_ME_MPM_s::finalize() {	return 0; }

int solve_substep_R2D_ME_MPM_s(void *_self)
{
	Step_R2D_ME_MPM_s *self = static_cast<Step_R2D_ME_MPM_s *>(_self);
	Model_R2D_ME_MPM_s *model = self->model;
	Element_R2D_ME_MPM_s *pelem;
	Node_R2D_ME_s *pn, *pn1, *pn2, *pn3, *pn4;
	Particle_R2D_ME_s *ppcl;

	// init nodes
	for (size_t i = 0; i < model->node_num; i++)
	{
		pn = model->nodes + i;
		pn->cal_flag = 0;
		// for mapping to nodes
		pn->m = 0.0;
		pn->mmx = 0.0;
		pn->mmy = 0.0;
		pn->fx_ext_m = 0.0;
		pn->fx_int_m = 0.0;
		pn->fy_ext_m = 0.0;
		pn->fy_int_m = 0.0;
		// for mapping back to particles
		pn->ax = 0.0;
		pn->ay = 0.0;
		pn->dmmx = 0.0;
		pn->dmmy = 0.0;
		pn->dux = 0.0;
		pn->duy = 0.0;
	}

	// To decide whether dt needs to be recalculated
	double elem_len_min_tmp = std::numeric_limits<double>::max();
	// init particles
	for (size_t i = 0; i < model->pcl_num; i++)
	{
		ppcl = model->pcls + i;
		if (ppcl->is_in_mesh)
		{
			ppcl->elem = model->find_in_which_element(ppcl->x, ppcl->y, ppcl->elem);
			pelem = ppcl->elem;
			if (pelem)
			{
				model->cal_shape_function(ppcl);
				Get_Nodes_Of_Element_R2D(pelem, model, pn1, pn2, pn3, pn4);
				// node 1
				ppcl->node1 = pn1;
				pn1->cal_flag = 1;
				// node 2
				ppcl->node2 = pn2;
				pn2->cal_flag = 1;
				// node 3
				ppcl->node3 = pn3;
				pn3->cal_flag = 1;
				// node 4
				ppcl->node4 = pn4;
				pn4->cal_flag = 1;

				if (elem_len_min_tmp > pelem->char_len)
					elem_len_min_tmp = pelem->char_len;
			}
			else
			{
				ppcl->is_in_mesh = false;
			}
		}
	}

	// calculate critical time step
	if (self->elem_len_min > elem_len_min_tmp)
	{
		self->elem_len_min = elem_len_min_tmp;
		double cri_dt, cri_dt_tmp;
		cri_dt = std::numeric_limits<double>::max();
		for (size_t i = 0; i < model->pcl_num; i++)
		{
			ppcl = model->pcls + i;
			if (ppcl->elem)
			{
				cri_dt_tmp = ppcl->critical_time_step(ppcl->elem->char_len);
				if (cri_dt > cri_dt_tmp)
					cri_dt = cri_dt_tmp;
			}
		}
		self->dt = cri_dt_tmp * self->dt_adjust_factor;
		self->time_tol = self->dt * self->time_tol_ratio;
		//std::cout << self->id << " " << self->dt << "\n";
	}

	// map variables to node and cal internal force
	double vol_tmp;
	for (size_t i = 0; i < model->pcl_num; i++)
	{
		ppcl = model->pcls + i;
		if (ppcl->is_in_mesh)
		{
			vol_tmp = ppcl->m / ppcl->density;
			// node 1
			pn1 = ppcl->node1;
			pn1->m   += ppcl->m   * ppcl->N1;
			pn1->mmx += ppcl->mmx * ppcl->N1;
			pn1->mmy += ppcl->mmy * ppcl->N1;
			pn1->fx_int_m += vol_tmp * (ppcl->dN1_dx * ppcl->s11 + ppcl->dN1_dy * ppcl->s12);
			pn1->fy_int_m += vol_tmp * (ppcl->dN1_dx * ppcl->s12 + ppcl->dN1_dy * ppcl->s22);
			// node 2
			pn2 = ppcl->node2;
			pn2->m   += ppcl->m   * ppcl->N2;
			pn2->mmx += ppcl->mmx * ppcl->N2;
			pn2->mmy += ppcl->mmy * ppcl->N2;
			pn2->fx_int_m += vol_tmp * (ppcl->dN2_dx * ppcl->s11 + ppcl->dN2_dy * ppcl->s12);
			pn2->fy_int_m += vol_tmp * (ppcl->dN2_dx * ppcl->s12 + ppcl->dN2_dy * ppcl->s22);
			// node 3
			pn3 = ppcl->node3;
			pn3->m   += ppcl->m   * ppcl->N3;
			pn3->mmx += ppcl->mmx * ppcl->N3;
			pn3->mmy += ppcl->mmy * ppcl->N3;
			pn3->fx_int_m += vol_tmp * (ppcl->dN3_dx * ppcl->s11 + ppcl->dN3_dy * ppcl->s12);
			pn3->fy_int_m += vol_tmp * (ppcl->dN3_dx * ppcl->s12 + ppcl->dN3_dy * ppcl->s22);
			// node 4
			pn4 = ppcl->node4;
			pn4->m   += ppcl->m   * ppcl->N4;
			pn4->mmx += ppcl->mmx * ppcl->N4;
			pn4->mmy += ppcl->mmy * ppcl->N4;
			pn4->fx_int_m += vol_tmp * (ppcl->dN4_dx * ppcl->s11 + ppcl->dN4_dy * ppcl->s12);
			pn4->fy_int_m += vol_tmp * (ppcl->dN4_dx * ppcl->s12 + ppcl->dN4_dy * ppcl->s22);
		}
	}

	// body force
	double bf_tmp;
	for (size_t i = 0; i < model->bfx_num; i++)
	{
		ppcl = model->pcls + model->bfxs[i].pcl_id;
		bf_tmp = ppcl->m * model->bfxs[i].bf;
		ppcl->node1->fx_ext_m += bf_tmp * ppcl->N1;
		ppcl->node2->fx_ext_m += bf_tmp * ppcl->N2;
		ppcl->node3->fx_ext_m += bf_tmp * ppcl->N3;
		ppcl->node4->fx_ext_m += bf_tmp * ppcl->N4;
	}
	for (size_t i = 0; i < model->bfy_num; i++)
	{
		ppcl = model->pcls + model->bfys[i].pcl_id;
		bf_tmp = ppcl->m * model->bfys[i].bf;
		ppcl->node1->fy_ext_m += bf_tmp * ppcl->N1;
		ppcl->node2->fy_ext_m += bf_tmp * ppcl->N2;
		ppcl->node3->fy_ext_m += bf_tmp * ppcl->N3;
		ppcl->node4->fy_ext_m += bf_tmp * ppcl->N4;
	}

	// surface force
	for (size_t i = 0; i < model->tx_bc_num; i++)
	{
		ppcl = model->pcls + model->tx_bcs[i].pcl_id;
		ppcl->node1->fx_ext_m += model->tx_bcs[i].t * ppcl->N1;
		ppcl->node2->fx_ext_m += model->tx_bcs[i].t * ppcl->N2;
		ppcl->node3->fx_ext_m += model->tx_bcs[i].t * ppcl->N3;
		ppcl->node4->fx_ext_m += model->tx_bcs[i].t * ppcl->N4;
	}
	for (size_t i = 0; i < model->ty_bc_num; i++)
	{
		ppcl = model->pcls + model->ty_bcs[i].pcl_id;
		ppcl->node1->fy_ext_m += model->ty_bcs[i].t * ppcl->N1;
		ppcl->node2->fy_ext_m += model->ty_bcs[i].t * ppcl->N2;
		ppcl->node3->fy_ext_m += model->ty_bcs[i].t * ppcl->N3;
		ppcl->node4->fy_ext_m += model->ty_bcs[i].t * ppcl->N4;
	}

	// update nodal acceleration
	for (size_t i = 0; i < model->node_num; i++)
	{
		pn = model->nodes + i;
		if (pn->cal_flag)
		{
			pn->ax = (pn->fx_ext_m - pn->fx_int_m) / pn->m;
			pn->ay = (pn->fy_ext_m - pn->fy_int_m) / pn->m;
		}
	}
	// apply acceleration boundary conditions
	for (size_t i = 0; i < model->ax_s_bc_num; i++)
	{
		pn = model->nodes + model->ax_s_bcs[i].node_id;
		pn->ax = model->ax_s_bcs[i].a;
	}
	for (size_t i = 0; i < model->ay_s_bc_num; i++)
	{
		pn = model->nodes + model->ay_s_bcs[i].node_id;
		pn->ay = model->ay_s_bcs[i].a;
	}

	// update nodal momentum
	for (size_t i = 0; i < model->node_num; i++)
	{
		pn = model->nodes + i;
		if (pn->cal_flag)
		{
			pn->dmmx = pn->m * pn->ax * self->dt;
			pn->dmmy = pn->m * pn->ay * self->dt;
			pn->mmx += pn->dmmx;
			pn->mmy += pn->dmmy;
		}
	}
	// apply velocity boundary conditions
	for (size_t i = 0; i < model->vx_s_bc_num; i++)
	{
		pn = model->nodes + model->vx_s_bcs[i].node_id;
		pn->mmx = pn->m * model->vx_s_bcs[i].v;
		pn->dmmx = 0.0;
		pn->ax = 0.0;
	}
	for (size_t i = 0; i < model->vy_s_bc_num; i++)
	{
		pn = model->nodes + model->vy_s_bcs[i].node_id;
		pn->mmy = pn->m * model->vy_s_bcs[i].v;
		pn->dmmy = 0.0;
		pn->ay = 0.0;
	}

	// update displacement increment
	for (size_t i = 0; i < model->node_num; i++)
	{
		pn = model->nodes + i;
		if (pn->cal_flag)
		{
			pn->vx = pn->mmx / pn->m;
			pn->vy = pn->mmy / pn->m;
			pn->dux = pn->vx * self->dt;
			pn->duy = pn->vy * self->dt;
		}
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
			/* map variables back to particles */
			pn1 = ppcl->node1;
			pn2 = ppcl->node2;
			pn3 = ppcl->node3;
			pn4 = ppcl->node4;
			N1_tmp = ppcl->N1;
			N2_tmp = ppcl->N2;
			N3_tmp = ppcl->N3;
			N4_tmp = ppcl->N4;

			// momentum (velocity)
			ppcl->mmx += ppcl->m * self->dt * (pn1->ax * N1_tmp + pn2->ax * N2_tmp + pn3->ax * N3_tmp + pn4->ax * N4_tmp);
			ppcl->mmy += ppcl->m * self->dt * (pn1->ay * N1_tmp + pn2->ay * N2_tmp + pn3->ay * N3_tmp + pn4->ay * N4_tmp);
						
			// displacement
			ppcl->ux += pn1->dux * N1_tmp + pn2->dux * N2_tmp
					  + pn3->dux * N3_tmp + pn4->dux * N4_tmp;
			ppcl->uy += pn1->duy * N1_tmp + pn2->duy * N2_tmp
					  + pn3->duy * N3_tmp + pn4->duy * N4_tmp;
			// update position
			ppcl->x = ppcl->x_ori + ppcl->ux;
			ppcl->y = ppcl->y_ori + ppcl->uy;
			
			// strain increment
			ppcl->de11 = pn1->dux * ppcl->dN1_dx + pn2->dux * ppcl->dN2_dx
					   + pn3->dux * ppcl->dN3_dx + pn4->dux * ppcl->dN4_dx;
			ppcl->de22 = pn1->duy * ppcl->dN1_dy + pn2->duy * ppcl->dN2_dy
					   + pn3->duy * ppcl->dN3_dy + pn4->duy * ppcl->dN4_dy;
			ppcl->de12 = (pn1->dux * ppcl->dN1_dy + pn2->dux * ppcl->dN2_dy
						+ pn3->dux * ppcl->dN3_dy + pn4->dux * ppcl->dN4_dy
						+ pn1->duy * ppcl->dN1_dx + pn2->duy * ppcl->dN2_dx
						+ pn3->duy * ppcl->dN3_dx + pn4->duy * ppcl->dN4_dx) * 0.5;
			ppcl->dw12 = (pn1->dux * ppcl->dN1_dy + pn2->dux * ppcl->dN2_dy
						+ pn3->dux * ppcl->dN3_dy + pn4->dux * ppcl->dN4_dy
						- pn1->duy * ppcl->dN1_dx - pn2->duy * ppcl->dN2_dx
						- pn3->duy * ppcl->dN3_dx - pn4->duy * ppcl->dN4_dx) * 0.5;
			
			/* update variables at particles */
			de_vol = ppcl->de11 + ppcl->de22;
			ppcl->density /= (1.0 + de_vol);

			// update strain (also assume that strain increment is Jaumann rate)
			//ppcl->de11 +=  ppcl->dw12 * ppcl->e12 * 2.0;
			//ppcl->de22 += -ppcl->dw12 * ppcl->e12 * 2.0;
			//ppcl->de12 +=  ppcl->dw12 * (ppcl->e22 - ppcl->e11);
			ppcl->e11 += ppcl->de11;
			ppcl->e22 += ppcl->de22;
			ppcl->e12 += ppcl->de12;

			// update stress
			E_tmp = ppcl->E / (1.0 + ppcl->niu) / (1.0 - 2.0 * ppcl->niu);
			ds11 = E_tmp * ((1.0 - ppcl->niu) * ppcl->de11 + ppcl->niu * ppcl->de22);
			ds12 = 2.0 * ppcl->de12 * ppcl->E / (2.0 * (1.0 + ppcl->niu));
			ds22 = E_tmp * (ppcl->niu * ppcl->de11 + (1.0 - ppcl->niu) * ppcl->de22);

			/* ------------------------------------------------------------------
			Rotate as Jaumann rate:
				tensor_rate = tensor_Jaumann_rate + tensor * dW_T + dW * tensor
			  ------------------------------------------------------------------- */
/*			ds11 +=  ppcl->dw12 * ppcl->s12 * 2.0;
			ds22 += -ppcl->dw12 * ppcl->s12 * 2.0;
			ds12 +=  ppcl->dw12 * (ppcl->s22 - ppcl->s11);	*/	
			ppcl->s11 += ds11;
			ppcl->s22 += ds22;
			ppcl->s12 += ds12;
		}
	}

	return 0;
}