#include "SimulationCore_pcp.h"

#include <fstream>
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
		// initialize B matrix of each particles
		init_B_matrix();
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
		pn.m = 0.0;
		pn.ax = 0.0;
		pn.ay = 0.0;
		pn.vx = 0.0;
		pn.vy = 0.0;
		pn.fx_ext = 0.0;
		pn.fy_ext = 0.0;
		pn.fx_int = 0.0;
		pn.fy_int = 0.0;
	}
	
	// map variables to node and cal internal force
	for (size_t pcl_id = 0; pcl_id < model->pcl_num; ++pcl_id)
	{
		auto &pcl = model->pcls[pcl_id];
		pcl.is_in_mesh = model->is_in_mesh(pcl.x, pcl.y);
		if (pcl.is_in_mesh)
		{
			model->cal_shape_func(pcl);
			double vol_tmp = pcl.m / pcl.density;
			for (size_t nx_id = 0; nx_id < 3; ++nx_id)
				for (size_t ny_id = 0; ny_id < 3; ++ny_id)
				{
					auto &n = *pcl.pn[ny_id][nx_id];
					if (pcl.N[ny_id][nx_id] != 0.0)
						n.cal_flag = 1;
					// map mass
					double m_prod_N = pcl.m * pcl.N[ny_id][nx_id];
					n.m += m_prod_N;
					// map momentum
					double Qx, Qy; // Q = C * (xi - xp)
					Qx = pcl.C[0][0] * pcl.x_dist[nx_id] + pcl.C[0][1] * pcl.y_dist[ny_id];
					Qy = pcl.C[1][0] * pcl.x_dist[nx_id] + pcl.C[1][1] * pcl.y_dist[ny_id];
					n.vx += m_prod_N * (pcl.vx + Qx);
					n.vy += m_prod_N * (pcl.vy + Qy);
					n.fx_int += (pcl.dN_dx[ny_id][nx_id] * pcl.s11 + pcl.dN_dy[ny_id][nx_id] * pcl.s12) * vol_tmp;
					n.fy_int += (pcl.dN_dx[ny_id][nx_id] * pcl.s12 + pcl.dN_dy[ny_id][nx_id] * pcl.s22) * vol_tmp;
				}
		}
	}

	// body force
	for (size_t i = 0; i < model->bfx_num; i++)
	{
		auto &pcl = model->pcls[model->bfxs[i].pcl_id];
		if (pcl.is_in_mesh)
		{
			double bf_tmp = pcl.m * model->bfxs[i].bf;
			for (size_t nx_id = 0; nx_id < 3; ++nx_id)
				for (size_t ny_id = 0; ny_id < 3; ++ny_id)
				{
					auto &n = *pcl.pn[ny_id][nx_id];
					n.fx_ext += bf_tmp * pcl.N[ny_id][nx_id];
				}
		}
	}
	for (size_t i = 0; i < model->bfy_num; i++)
	{
		auto &pcl = model->pcls[model->bfys[i].pcl_id];
		if (pcl.is_in_mesh)
		{
			double bf_tmp = pcl.m * model->bfys[i].bf;
			for (size_t nx_id = 0; nx_id < 3; ++nx_id)
				for (size_t ny_id = 0; ny_id < 3; ++ny_id)
				{
					auto &n = *pcl.pn[ny_id][nx_id];
					n.fy_ext += bf_tmp * pcl.N[ny_id][nx_id];
				}
		}
	}

	// surface force
	for (size_t i = 0; i < model->tx_num; i++)
	{
		auto &pcl = model->pcls[model->txs[i].pcl_id];
		if (pcl.is_in_mesh)
		{
			double tf_tmp = model->txs[i].t;
			for (size_t nx_id = 0; nx_id < 3; ++nx_id)
				for (size_t ny_id = 0; ny_id < 3; ++ny_id)
				{
					auto &n = *pcl.pn[ny_id][nx_id];
					n.fy_ext += tf_tmp * pcl.N[ny_id][nx_id];
				}
		}
	}
	for (size_t i = 0; i < model->ty_num; i++)
	{
		auto &pcl = model->pcls[model->tys[i].pcl_id];
		if (pcl.is_in_mesh)
		{
			double tf_tmp = model->tys[i].t;
			for (size_t nx_id = 0; nx_id < 3; ++nx_id)
				for (size_t ny_id = 0; ny_id < 3; ++ny_id)
				{
					auto &n = *pcl.pn[ny_id][nx_id];
					n.fy_ext += tf_tmp * pcl.N[ny_id][nx_id];
				}
		}
	}

	// update nodal acceleration
	for (size_t n_id = 0; n_id < model->node_num; ++n_id)
	{
		auto &n = model->nodes[n_id];
		if (n.cal_flag)
		{
			n.ax = (n.fx_ext - n.fx_int) / n.m;
			n.ay = (n.fy_ext - n.fy_int) / n.m;
		}
	}
	// apply acceleration boundary conditions
	for (size_t i = 0; i < model->ax_num; i++)
	{
		auto &n = model->nodes[model->axs[i].node_id];
		n.ax = model->axs[i].a;
	}
	for (size_t i = 0; i < model->ay_num; i++)
	{
		auto &n = model->nodes[model->ays[i].node_id];
		n.ay = model->ays[i].a;
	}

	// update nodal momentum
	for (size_t n_id = 0; n_id < model->node_num; ++n_id)
	{
		auto &n = model->nodes[n_id];
		if (n.cal_flag)
		{
			n.vx = n.vx / n.m + n.ax * self->dt;
			n.vy = n.vy / n.m + n.ay * self->dt;
		}
	}
	// apply velocity boundary conditions
	for (size_t i = 0; i < model->vx_num; i++)
	{
		auto &n = model->nodes[model->vxs[i].node_id];
		n.vx = model->vxs[i].v;
		n.ax = 0.0;
	}
	for (size_t i = 0; i < model->vy_num; i++)
	{
		auto &n = model->nodes[model->vys[i].node_id];
		n.vy = model->vys[i].v;
		n.ay = 0.0;
	}

	// map variables back to and update variables particles
	for (size_t pcl_id = 0; pcl_id < model->pcl_num; ++pcl_id)
	{
		auto &pcl = model->pcls[pcl_id];
		if (pcl.is_in_mesh)
		{
			pcl.vx = 0.0;
			pcl.vy = 0.0;
			double B[2][2]; // B matrix for APIC
			Mat_Set_Zero(B);
			for (size_t nx_id = 0; nx_id < 3; ++nx_id)
				for (size_t ny_id = 0; ny_id < 3; ++ny_id)
				{
					auto &n = *pcl.pn[ny_id][nx_id];
					// map velocity
					double vx_prod_N = n.vx * pcl.N[ny_id][nx_id];
					double vy_prod_N = n.vy * pcl.N[ny_id][nx_id];
					pcl.vx += vx_prod_N;
					pcl.vy += vy_prod_N;
					// B = sum (N * v * dist_T)
					B[0][0] += vx_prod_N * pcl.x_dist[nx_id];
					B[0][1] += vx_prod_N * pcl.y_dist[ny_id];
					B[1][0] += vy_prod_N * pcl.x_dist[nx_id];
					B[1][1] += vy_prod_N * pcl.y_dist[ny_id];
				}
			// update C matrix for APIC
			Mat_Prod_Num(pcl.C, B, self->invD);
			
			// displacement
			pcl.ux += pcl.vx * self->dt;
			pcl.uy += pcl.vy * self->dt;
			// update position
			pcl.x = pcl.x_ori + pcl.ux;
			pcl.y = pcl.y_ori + pcl.uy;
			
			// strain increment
			double dux, duy;
			double de11, de22, de12, dw12;
			de11 = 0.0;
			de22 = 0.0;
			de12 = 0.0;
			dw12 = 0.0;
			for (size_t nx_id = 0; nx_id < 3; ++nx_id)
				for (size_t ny_id = 0; ny_id < 3; ++ny_id)
				{
					auto &n = *pcl.pn[ny_id][nx_id];
					dux = n.vx * self->dt;
					duy = n.vy * self->dt;
					de11 += dux * pcl.dN_dx[ny_id][nx_id];
					de22 += duy * pcl.dN_dy[ny_id][nx_id];
					de12 += (dux * pcl.dN_dy[ny_id][nx_id]
						   + duy * pcl.dN_dx[ny_id][nx_id]) * 0.5;
					dw12 += (dux * pcl.dN_dy[ny_id][nx_id]
						   - duy * pcl.dN_dx[ny_id][nx_id]) * 0.5;
				}
			
			/* update variables at particles */
			double de_vol = de11 + de22;
			pcl.density /= (1.0 + de_vol);

			// update strain (also assume that strain increment is Jaumann rate)
			//ppcl->de11 +=  ppcl->dw12 * ppcl->e12 * 2.0;
			//ppcl->de22 += -ppcl->dw12 * ppcl->e12 * 2.0;
			//ppcl->de12 +=  ppcl->dw12 * (ppcl->e22 - ppcl->e11);
			pcl.e11 += de11;
			pcl.e22 += de22;
			pcl.e12 += de12;

			// update stress
			double ds11, ds22, ds12;
			double E_tmp = pcl.E / (1.0 + pcl.niu) / (1.0 - 2.0 * pcl.niu);
			ds11 = E_tmp * ((1.0 - pcl.niu) * de11 + pcl.niu * de22);
			ds22 = E_tmp * (pcl.niu * de11 + (1.0 - pcl.niu) * de22);
			ds12 = 2.0 * de12 * pcl.E / (2.0 * (1.0 + pcl.niu));

			/* ------------------------------------------------------------------
			Rotate as Jaumann rate:
				tensor_rate = tensor_Jaumann_rate + tensor * dW_T + dW * tensor
			  ------------------------------------------------------------------- */
/*			ds11 +=  ppcl->dw12 * ppcl->s12 * 2.0;
			ds22 += -ppcl->dw12 * ppcl->s12 * 2.0;
			ds12 +=  ppcl->dw12 * (ppcl->s22 - ppcl->s11);	*/	
			pcl.s11 += ds11;
			pcl.s22 += ds22;
			pcl.s12 += ds12;
		}
	}

	return 0;
}


void Step_R2D_ME_MPM_BSpline_APIC_s::init_B_matrix(void)
{
	invD = 4.0 / (model->h * model->h);

	for (size_t pcl_id = 0; pcl_id < model->pcl_num; ++pcl_id)
	{
		auto &pcl = model->pcls[pcl_id];
		pcl.is_in_mesh = model->is_in_mesh(pcl.x, pcl.y);
		model->cal_shape_func(pcl);
		for (size_t nx_id = 0; nx_id < 3; ++nx_id)
			for (size_t ny_id = 0; ny_id < 3; ++ny_id)
			{
				if (pcl.N[ny_id][nx_id] != 0.0)
					pcl.pn[ny_id][nx_id]->cal_flag = 1;
			}
		Mat_Set_Zero(pcl.C);
	}

	size_t cal_node_num = 0;
	for (size_t n_id = 0; n_id < model->node_num; ++n_id)
	{
		if (model->nodes[n_id].cal_flag)
			++cal_node_num;
	}
	double *nv1 = new double[cal_node_num * 2];
	double *nv2 = nv1 + cal_node_num;
	memset(nv1, 0, cal_node_num * 2 * sizeof(double));

	std::fstream file("ddd.txt", std::ios::out);
	size_t iter_id = 0;
	double vx_norm, dvx_norm;
	double vy_norm, dvy_norm;
	do
	{
		// init nodes
		for (size_t n_id = 0; n_id < model->node_num; ++n_id)
		{
			auto &pn = model->nodes[n_id];
			pn.cal_flag = 0;
			pn.m = 0.0;
			pn.vx = 0.0;
			pn.vy = 0.0;
		}

		// map mass and momentum
		for (size_t pcl_id = 0; pcl_id < model->pcl_num; ++pcl_id)
		{
			auto &pcl = model->pcls[pcl_id];
			if (pcl.is_in_mesh)
			{
				for (size_t nx_id = 0; nx_id < 3; ++nx_id)
					for (size_t ny_id = 0; ny_id < 3; ++ny_id)
					{
						auto &n = *pcl.pn[ny_id][nx_id];
						// map mass
						double m_prod_N = pcl.m * pcl.N[ny_id][nx_id];
						n.m += m_prod_N;
						// map momentum
						double Qx, Qy; // Q = C * (xi - xp)
						Qx = pcl.C[0][0] * pcl.x_dist[nx_id] + pcl.C[0][1] * pcl.y_dist[ny_id];
						Qy = pcl.C[1][0] * pcl.x_dist[nx_id] + pcl.C[1][1] * pcl.y_dist[ny_id];
						n.vx += m_prod_N * (pcl.vx + Qx);
						n.vy += m_prod_N * (pcl.vy + Qy);
					}
			}
		}

		vx_norm = 0.0;
		dvx_norm = 0.0;
		vy_norm = 0.0;
		dvy_norm = 0.0;
		for (size_t n_id = 0; n_id < model->node_num; ++n_id)
		{
			auto &pn = model->nodes[n_id];
			if (pn.cal_flag)
			{
				pn.vx /= pn.m;
				pn.vy /= pn.m;
				// convergence criteria
				vx_norm += pn.vx * pn.vx;
				vy_norm += pn.vy * pn.vy;
				double tmp;
				tmp = pn.vx - nv1[n_id];
				dvx_norm += tmp * tmp;
				tmp = pn.vy - nv2[n_id];
				dvy_norm += tmp * tmp;
			}
			file << pn.vx << ", ";
		}
		file << "\n";

		// form C matrix for APIC
		for (size_t pcl_id = 0; pcl_id < model->pcl_num; ++pcl_id)
		{
			auto &pcl = model->pcls[pcl_id];
			if (pcl.is_in_mesh)
			{
				double B[2][2]; // B matrix for APIC
				Mat_Set_Zero(B);
				for (size_t nx_id = 0; nx_id < 3; ++nx_id)
					for (size_t ny_id = 0; ny_id < 3; ++ny_id)
					{
						// B = sum (N * v * dist_T)
						auto &n = *pcl.pn[ny_id][nx_id];
						double vx_prod_N = pcl.N[ny_id][nx_id] * n.vx;
						double vy_prod_N = pcl.N[ny_id][nx_id] * n.vy;
						B[0][0] += vx_prod_N * pcl.x_dist[nx_id];
						B[0][1] += vx_prod_N * pcl.y_dist[ny_id];
						B[1][0] += vy_prod_N * pcl.x_dist[nx_id];
						B[1][1] += vy_prod_N * pcl.y_dist[ny_id];
					}
				// C = B * 1/D
				Mat_Prod_Num(pcl.C, B, invD);
			}
		}

		if (vx_norm == 0.0)
		{
			if (vy_norm == 0.0)
				break;
		}
			
		dvx_norm / vx_norm > 0.01 || dvy_norm / vy_norm > 0.01) // ||dv|| / ||v|| < 0.1
		
		++iter_id;
	} while (iter_id < 100); // max iteration num

	delete[] nv1;
}