#include "SimulationCore_pcp.h"

#include "Step_R2D_ME_MPM_BSpline_APIC_s.h"

#define Mat_Set_Zero(mat) \
	mat[0][0] = 0.0;      \
	mat[0][1] = 0.0;      \
	mat[1][0] = 0.0;      \
	mat[1][1] = 0.0

#define Mat_Prod_Num(mat_res, mat, num) \
	mat_res[0][0] = mat[0][0] * num;    \
	mat_res[0][1] = mat[0][1] * num;    \
	mat_res[1][0] = mat[1][0] * num;    \
	mat_res[1][1] = mat[1][1] * num

Step_R2D_ME_MPM_BSpline_APIC_s::
	Step_R2D_ME_MPM_BSpline_APIC_s() :
	Step(&solve_substep_R2D_ME_MPM_BSpline_APIC_s),
	model(nullptr) {}

Step_R2D_ME_MPM_BSpline_APIC_s::
	~Step_R2D_ME_MPM_BSpline_APIC_s() {}

int Step_R2D_ME_MPM_BSpline_APIC_s::init()
{
	if (is_first_step)
	{
		invD = 4.0 / (model->h * model->h);
		for (size_t pcl_id = 0; pcl_id < model->pcl_num; ++pcl_id)
		{
			auto &pcl = model->pcls[pcl_id];
			pcl.is_in_mesh = model->is_in_mesh(pcl.x, pcl.y);
			if (pcl.is_in_mesh)
			{
				pcl.base_node_x_id = model->base_node_x_index(pcl.x);
				pcl.base_node_y_id = model->base_node_y_index(pcl.y);
				// init C matrix for APIC
				double B[2][2]; // B matrix
				Mat_Set_Zero(B);
				for (size_t nx_id = 0; nx_id < 3; ++nx_id)
					for (size_t ny_id = 0; ny_id < 3; ++ny_id)
					{
						double xi = model->xi(pcl.x, pcl.base_node_id + n_id);
						pcl.N[n_id] = model->N(xi);
						auto &pn = model->get_node_by_index(pcl.base_node_id + n_id);
						// map to nodes
						B += pcl.N[n_id] * pcl.v * (xi*model->h);
					}
				Mat_Prod_Num(pcl.C, B, invD);

			}
		}
	}

	for (size_t pcl_id = 0; pcl_id < model->pcl_num; ++pcl_id)
	{
		auto &pcl = model->pcls[pcl_id];
		pcl.x_ori = pcl.x;
		pcl.y_ori = pcl.y;
		pcl.ux = 0.0;
		pcl.uy = 0.0;
	}
	
	return 0;
}

int Step_R2D_ME_MPM_BSpline_APIC_s::finalize() { return 0; }

int solve_substep_R2D_ME_MPM_BSpline_APIC_s(void *_self)
{
	Step_R2D_ME_MPM_BSpline_APIC_s *self = static_cast<Step_R2D_ME_MPM_BSpline_APIC_s *>(_self);
	Model_R2D_ME_MPM_BSpline_s *model = self->model;

	// init nodes
	for (size_t n_id = 0; n_id < model->node_num; ++n_id)
	{
		auto &pn = model->nodes[n_id];
		pn.cal_flag = 0;
		// for mapping to nodes
		pn.m = 0.0;
		pn.ax = 0.0;
		pn.ay = 0.0;
		pn.vx = 0.0;
		pn.vy = 0.0;
		pn.dux = 0.0;
		pn.duy = 0.0;
		pn.fx_ext = 0.0;
		pn.fy_ext = 0.0;
		pn.fx_int = 0.0;
		pn.fy_int = 0.0;
	}
	
	// map variables to node and cal internal force
	for (size_t pcl_id = 0; pcl_id < model->pcl_num; ++pcl_id)
	{
		auto &pcl = model->pcls[pcl_id];
		if (pcl.is_in_mesh)
		{
			double vol_tmp = pcl.m / pcl.density;
			// node 1
			pn1 = ppcl->node1;
			pn1->m   += ppcl->m   * ppcl->N1;
			pn1->mmx += ppcl->mmx * ppcl->N1;
			pn1->mmy += ppcl->mmy * ppcl->N1;
			pn1->fx_int_m += vol_tmp * (ppcl->dN1_dx * ppcl->s11 + ppcl->dN1_dy * ppcl->s12);
			pn1->fy_int_m += vol_tmp * (ppcl->dN1_dx * ppcl->s12 + ppcl->dN1_dy * ppcl->s22);
		}
	}

	// body force
	for (size_t i = 0; i < model->bfx_num; i++)
	{
		auto &pcl = model->pcls[model->bfxs[i].pcl_id];
		double bf_tmp = pcl.m * model->bfxs[i].bf;
		if (pcl.is_in_mesh)
		{
			ppcl->node1->fx_ext_m += bf_tmp * ppcl->N1;
			ppcl->node2->fx_ext_m += bf_tmp * ppcl->N2;
			ppcl->node3->fx_ext_m += bf_tmp * ppcl->N3;
			ppcl->node4->fx_ext_m += bf_tmp * ppcl->N4;
		}
	}
	for (size_t i = 0; i < model->bfy_num; i++)
	{
		ppcl = model->pcls + model->bfys[i].pcl_id;
		bf_tmp = ppcl->m * model->bfys[i].bf;
		if (ppcl->is_in_mesh)
		{
			ppcl->node1->fy_ext_m += bf_tmp * ppcl->N1;
			ppcl->node2->fy_ext_m += bf_tmp * ppcl->N2;
			ppcl->node3->fy_ext_m += bf_tmp * ppcl->N3;
			ppcl->node4->fy_ext_m += bf_tmp * ppcl->N4;
		}
	}

	// surface force
	for (size_t i = 0; i < model->tx_bc_num; i++)
	{
		ppcl = model->pcls + model->tx_bcs[i].pcl_id;
		if (ppcl->is_in_mesh)
		{
			ppcl->node1->fx_ext_m += model->tx_bcs[i].t * ppcl->N1;
			ppcl->node2->fx_ext_m += model->tx_bcs[i].t * ppcl->N2;
			ppcl->node3->fx_ext_m += model->tx_bcs[i].t * ppcl->N3;
			ppcl->node4->fx_ext_m += model->tx_bcs[i].t * ppcl->N4;
		}
	}
	for (size_t i = 0; i < model->ty_bc_num; i++)
	{
		ppcl = model->pcls + model->ty_bcs[i].pcl_id;
		if (ppcl->is_in_mesh)
		{
			ppcl->node1->fy_ext_m += model->ty_bcs[i].t * ppcl->N1;
			ppcl->node2->fy_ext_m += model->ty_bcs[i].t * ppcl->N2;
			ppcl->node3->fy_ext_m += model->ty_bcs[i].t * ppcl->N3;
			ppcl->node4->fy_ext_m += model->ty_bcs[i].t * ppcl->N4;
		}
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
		if (pn->cal_flag)
			pn->ax = model->ax_s_bcs[i].a;
	}
	for (size_t i = 0; i < model->ay_s_bc_num; i++)
	{
		pn = model->nodes + model->ay_s_bcs[i].node_id;
		if (pn->cal_flag)
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
		if (pn->cal_flag)
		{
			pn->mmx = pn->m * model->vx_s_bcs[i].v;
			pn->dmmx = 0.0;
			pn->ax = 0.0;
		}
	}
	for (size_t i = 0; i < model->vy_s_bc_num; i++)
	{
		pn = model->nodes + model->vy_s_bcs[i].node_id;
		if (pn->cal_flag)
		{
			pn->mmy = pn->m * model->vy_s_bcs[i].v;
			pn->dmmy = 0.0;
			pn->ay = 0.0;
		}
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