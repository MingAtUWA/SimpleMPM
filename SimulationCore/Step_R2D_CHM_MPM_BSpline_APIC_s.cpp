#include "SimulationCore_pcp.h"

//#include <fstream>
#include "Step_R2D_CHM_MPM_BSpline_APIC_s.h"

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

Step_R2D_CHM_MPM_BSpline_APIC_s::
	Step_R2D_CHM_MPM_BSpline_APIC_s() :
	Step(&solve_substep_R2D_CHM_MPM_BSpline_APIC_s),
	model(nullptr) {}

Step_R2D_CHM_MPM_BSpline_APIC_s::
	~Step_R2D_CHM_MPM_BSpline_APIC_s() {}

int Step_R2D_CHM_MPM_BSpline_APIC_s::init()
{
	invD = 4.0 / (model->h * model->h);
	if (is_first_step)
	{
		init_B_matrix(); // for APIC
	}

	for (size_t pcl_id = 0; pcl_id < model->pcl_num; ++pcl_id)
	{
		auto &pcl = model->pcls[pcl_id];
		pcl.x_ori = pcl.x;
		pcl.y_ori = pcl.y;
		pcl.ux_s = 0.0;
		pcl.uy_s = 0.0;
	}

	return 0;
}

int Step_R2D_CHM_MPM_BSpline_APIC_s::finalize() { return 0; }


int solve_substep_R2D_CHM_MPM_BSpline_APIC_s(void *_self)
{
	Step_R2D_CHM_MPM_BSpline_APIC_s *self
		= static_cast<Step_R2D_CHM_MPM_BSpline_APIC_s *>(_self);
	Model_R2D_CHM_MPM_BSpline_s *model = self->model;

	// init nodes
	for (size_t n_id = 0; n_id < model->node_num; ++n_id)
		model->nodes[n_id].init();

	// init particles
	for (size_t pcl_id = 0; pcl_id < model->pcl_num; ++pcl_id)
	{
		auto &pcl = model->pcls[pcl_id];
		if (pcl.is_in_mesh)
		{
			pcl.is_in_mesh = model->is_in_mesh(pcl.x, pcl.y);
			model->cal_shape_func(pcl);
			double vol = pcl.m_s / ((1.0 - pcl.n) * pcl.density_s);
			pcl.vol = vol;
			double n_miu_div_k = pcl.n * pcl.miu / pcl.k;
			for (size_t nx_id = 0; nx_id < 3; ++nx_id)
				for (size_t ny_id = 0; ny_id < 3; ++ny_id)
				{
					auto &n = *pcl.pn[ny_id][nx_id];
					double N = pcl.N[ny_id][nx_id];
					double dN_dx = pcl.dN_dx[ny_id][nx_id];
					double dN_dy = pcl.dN_dy[ny_id][nx_id];
					double x_dist = pcl.x_dist[nx_id];
					double y_dist = pcl.y_dist[ny_id];
					if (N != 0.0)
						n.cal_flag = 1;
					double m_prod_N;
					double Qx, Qy;
					// solid phase
					m_prod_N = pcl.m_s * N;
					n.m_s += m_prod_N;
					Qx = pcl.C_s[0][0] * x_dist + pcl.C_s[0][1] * y_dist;
					Qy = pcl.C_s[1][0] * x_dist + pcl.C_s[1][1] * y_dist;
					n.vx_s += m_prod_N * (pcl.vx_s + Qx);
					n.vy_s += m_prod_N * (pcl.vy_s + Qy);
					n.fx_int_m += (dN_dx * (pcl.s11 - pcl.p) + dN_dy * pcl.s12) * vol;
					n.fy_int_m += (dN_dx * pcl.s12 + dN_dy * (pcl.s22 - pcl.p)) * vol;
					// fluid phase
					m_prod_N = pcl.density_f * vol * N;
					n.m_tf += m_prod_N;
					Qx = pcl.C_f[0][0] * x_dist + pcl.C_f[0][1] * y_dist;
					Qy = pcl.C_f[1][0] * x_dist + pcl.C_f[1][1] * y_dist;
					n.vx_f += m_prod_N * (pcl.vx_f + Qx);
					n.vy_f += m_prod_N * (pcl.vy_f + Qy);					
					n.fx_int_tf += (dN_dx * -pcl.p) * vol;
					n.fy_int_tf += (dN_dy * -pcl.p) * vol;
					n.fx_drag_tf += n_miu_div_k * (pcl.vx_f - pcl.vx_s) * vol * N;
					n.fy_drag_tf += n_miu_div_k * (pcl.vy_f - pcl.vy_s) * vol * N;
				}
		}
	}

	// body force
	for (size_t i = 0; i < model->bfx_num; i++)
	{
		auto &pcl = model->pcls[model->bfxs[i].pcl_id];
		double bf_m  = pcl.vol * ((1.0 - pcl.n) * pcl.density_s + pcl.n * pcl.density_f) * model->bfxs[i].bf;
		double bf_tf = pcl.vol * pcl.density_f * model->bfxs[i].bf;
		for (size_t nx_id = 0; nx_id < 3; ++nx_id)
			for (size_t ny_id = 0; ny_id < 3; ++ny_id)
			{
				auto &n     = *pcl.pn[ny_id][nx_id];
				n.fx_ext_m  += pcl.N[ny_id][nx_id] * bf_m;
				n.fx_ext_tf += pcl.N[ny_id][nx_id] * bf_tf;
			}
	}
	for (size_t i = 0; i < model->bfy_num; i++)
	{
		auto &pcl = model->pcls[model->bfys[i].pcl_id];
		double bf_m  = pcl.vol * ((1.0 - pcl.n) * pcl.density_s + pcl.n * pcl.density_f) * model->bfys[i].bf;
		double bf_tf = pcl.vol * pcl.density_f * model->bfys[i].bf;
		for (size_t nx_id = 0; nx_id < 3; ++nx_id)
			for (size_t ny_id = 0; ny_id < 3; ++ny_id)
			{
				auto &n     = *pcl.pn[ny_id][nx_id];
				n.fy_ext_m  += pcl.N[ny_id][nx_id] * bf_m;
				n.fy_ext_tf += pcl.N[ny_id][nx_id] * bf_tf;
			}
	}

	// surface force
	for (size_t i = 0; i < model->tx_num; i++)
	{
		auto &pcl = model->pcls[model->txs[i].pcl_id];
		double tf = model->txs[i].t;
		for (size_t nx_id = 0; nx_id < 3; ++nx_id)
			for (size_t ny_id = 0; ny_id < 3; ++ny_id)
			{
				auto &n = *pcl.pn[ny_id][nx_id];
				n.fx_ext_m += pcl.N[ny_id][nx_id] * tf;
			}
	}
	for (size_t i = 0; i < model->ty_num; i++)
	{
		auto &pcl = model->pcls[model->tys[i].pcl_id];
		double tf = model->tys[i].t;
		for (size_t nx_id = 0; nx_id < 3; ++nx_id)
			for (size_t ny_id = 0; ny_id < 3; ++ny_id)
			{
				auto &n = *pcl.pn[ny_id][nx_id];
				n.fy_ext_m += pcl.N[ny_id][nx_id] * tf;
			}
	}
	// pore pressure force...

	// update nodal acceleration of fluid pahse
	for (size_t n_id = 0; n_id < model->node_num; ++n_id)
	{
		auto &n = model->nodes[n_id];
		if (n.cal_flag)
		{
			n.ax_f = (n.fx_ext_tf - n.fx_int_tf - n.fx_drag_tf) / n.m_tf;
			n.ay_f = (n.fy_ext_tf - n.fy_int_tf - n.fy_drag_tf) / n.m_tf;
		}
	}

	// acceleration bcs
	for (size_t i = 0; i < model->afx_num; i++)
	{
		auto &n = model->nodes[model->afxs[i].node_id];
		n.ax_f = model->afxs[i].a;
	}
	for (size_t i = 0; i < model->afy_num; i++)
	{
		auto &n = model->nodes[model->afys[i].node_id];
		n.ay_f = model->afys[i].a;
	}

	// update nodal momentum of fluid phase
	for (size_t n_id = 0; n_id < model->node_num; ++n_id)
	{
		auto &n = model->nodes[n_id];
		if (n.cal_flag)
		{
			n.vx_f = n.vx_f / n.m_tf + n.ax_f * self->dt;
			n.vy_f = n.vy_f / n.m_tf + n.ay_f * self->dt;
		}
	}
	// apply velocity boundary conditions of fluid phase
	for (size_t i = 0; i < model->vfx_num; ++i)
	{
		auto &n = model->nodes[model->vfxs[i].node_id];
		n.vx_f = model->vfxs[i].v;
		n.ax_f = 0.0;
	}
	for (size_t i = 0; i < model->vfy_num; ++i)
	{
		auto &n = model->nodes[model->vfys[i].node_id];
		n.vy_f = model->vfys[i].v;
		n.ay_f = 0.0;
	}

	// map variables back to and update variables particles
	for (size_t pcl_id = 0; pcl_id < model->pcl_num; ++pcl_id)
	{
		auto &pcl = model->pcls[pcl_id];
		if (pcl.is_in_mesh)
		{
			double pcl_max_f = 0.0;
			double pcl_may_f = 0.0;
			for (size_t nx_id = 0; nx_id < 3; ++nx_id)
				for (size_t ny_id = 0; ny_id < 3; ++ny_id)
				{
					auto &n   = *pcl.pn[ny_id][nx_id];
					pcl_max_f += pcl.N[ny_id][nx_id] * n.ax_f;
					pcl_may_f += pcl.N[ny_id][nx_id] * n.ay_f;
				}
			double pcl_m_f = pcl.vol * pcl.n * pcl.density_f;
			pcl_max_f *= pcl_m_f;
			pcl_may_f *= pcl_m_f;
		
			for (size_t nx_id = 0; nx_id < 3; ++nx_id)
				for (size_t ny_id = 0; ny_id < 3; ++ny_id)
				{
					auto &n    = *pcl.pn[ny_id][nx_id];
					n.fx_kin_f += pcl.N[ny_id][nx_id] * pcl_max_f;
					n.fy_kin_f += pcl.N[ny_id][nx_id] * pcl_may_f;
				}
		}
	}

	// update nodal velocity of solid phase
	for (size_t n_id = 0; n_id < model->node_num; ++n_id)
	{
		auto &n = model->nodes[n_id];
		if (n.cal_flag)
		{
			n.ax_s = (n.fx_ext_m - n.fx_int_m - n.fx_kin_f) / n.m_s;
			n.ay_s = (n.fy_ext_m - n.fy_int_m - n.fy_kin_f) / n.m_s;
		}
	}
	// apply acceleration boundary conditions
	for (size_t i = 0; i < model->asx_num; ++i)
	{
		auto &n = model->nodes[model->asxs[i].node_id];
		n.ax_s = model->asxs[i].a;
	}
	for (size_t i = 0; i < model->asy_num; ++i)
	{
		auto &n = model->nodes[model->asys[i].node_id];
		n.ay_s = model->asys[i].a;
	}
	
	// update nodal momentum of fluid pahse
	for (size_t n_id = 0; n_id < model->node_num; ++n_id)
	{
		auto &n = model->nodes[n_id];
		if (n.cal_flag)
		{
			n.vx_s = n.vx_s / n.m_s + n.ax_s * self->dt;
			n.vy_s = n.vy_s / n.m_s + n.ay_s * self->dt;
		}
	}
	// apply velocity boundary conditions of solid phase
	for (size_t i = 0; i < model->vsx_num; ++i)
	{
		auto &n = model->nodes[model->vsxs[i].node_id];
		n.vx_s = model->vsxs[i].v;
		n.ax_s = 0.0;
	}
	for (size_t i = 0; i < model->vsy_num; ++i)
	{
		auto &n = model->nodes[model->vsys[i].node_id];
		n.vy_s = model->vsys[i].v;
		n.ay_s = 0.0;
	}

	// map variables back to and update variables particles
	for (size_t pcl_id = 0; pcl_id < model->pcl_num; ++pcl_id)
	{
		auto &pcl = model->pcls[pcl_id];
		if (pcl.is_in_mesh)
		{
			// map velocity (and B matrix)
			pcl.vx_f = 0.0;
			pcl.vy_f = 0.0;
			pcl.vx_s = 0.0;
			pcl.vy_s = 0.0;
			double B_f[2][2], B_s[2][2]; // B matrix for APIC
			Mat_Set_Zero(B_f);
			Mat_Set_Zero(B_s);
			for (size_t nx_id = 0; nx_id < 3; ++nx_id)
				for (size_t ny_id = 0; ny_id < 3; ++ny_id)
				{
					auto &n = *pcl.pn[ny_id][nx_id];
					double N = pcl.N[ny_id][nx_id];
					// map velocity
					double vx_f_prod_N = n.vx_f * N;
					pcl.vx_f += vx_f_prod_N;
					double vy_f_prod_N = n.vy_f * N;
					pcl.vy_f += vy_f_prod_N;
					double vx_s_prod_N = n.vx_s * N;
					pcl.vx_s += vx_s_prod_N;
					double vy_s_prod_N = n.vy_s * N;
					pcl.vy_s += vy_s_prod_N;
					// B = sum (N * v * dist_T)
					double x_dist = pcl.x_dist[nx_id];
					double y_dist = pcl.y_dist[ny_id];
					B_f[0][0] += vx_f_prod_N * x_dist;
					B_f[0][1] += vx_f_prod_N * y_dist;
					B_f[1][0] += vy_f_prod_N * x_dist;
					B_f[1][1] += vy_f_prod_N * y_dist;
					B_s[0][0] += vx_s_prod_N * x_dist;
					B_s[0][1] += vx_s_prod_N * y_dist;
					B_s[1][0] += vy_s_prod_N * x_dist;
					B_s[1][1] += vy_s_prod_N * y_dist;
				}
			// update C matrix for APIC
			Mat_Prod_Num(pcl.C_f, B_f, self->invD);
			Mat_Prod_Num(pcl.C_s, B_s, self->invD);

			// displacement
			pcl.ux_s += pcl.vx_s * self->dt;
			pcl.uy_s += pcl.vy_s * self->dt;
			// update position
			pcl.x = pcl.x_ori + pcl.ux_s;
			pcl.y = pcl.y_ori + pcl.uy_s;

			// strain increment
			double de11_s = 0.0;
			double de22_s = 0.0;
			double de12_s = 0.0;
			double dw12_s = 0.0;
			double de_vol_s; // volumetric strain of solid phase
			double de_vol_f = 0.0; // "volumetric strain" of fluid phase
			for (size_t nx_id = 0; nx_id < 3; ++nx_id)
				for (size_t ny_id = 0; ny_id < 3; ++ny_id)
				{
					auto &n = *pcl.pn[ny_id][nx_id];
					double dN_dx = pcl.dN_dx[ny_id][nx_id];
					double dN_dy = pcl.dN_dy[ny_id][nx_id];
					double dux_s = n.vx_s * self->dt;
					double duy_s = n.vy_s * self->dt;
					de11_s += dux_s * dN_dx;
					de22_s += duy_s * dN_dy;
					de12_s += (dux_s * dN_dy + duy_s * dN_dx) * 0.5;
					dw12_s += (dux_s * dN_dy - duy_s * dN_dx) * 0.5;
					double dux_f = n.vx_f * self->dt;
					double duy_f = n.vy_f * self->dt;
					de_vol_f -= dux_f * dN_dx + duy_f * dN_dy;
				}
			de_vol_s = de11_s + de22_s;
			de_vol_f -= (1.0 - pcl.n) / pcl.n * de_vol_s;


			// update strain (also assume that strain increment is Jaumann rate)
			//ppcl->de11 +=  dw12 * e12 * 2.0;
			//ppcl->de22 += -dw12 * e12 * 2.0;
			//ppcl->de12 +=  dw12 * (e22 - e11);
			pcl.e11 += de11_s;
			pcl.e22 += de22_s;
			pcl.e12 += de12_s;

			// update stress
			double ds11, ds22, ds12;
			double E_tmp = pcl.E / (1.0 + pcl.niu) / (1.0 - 2.0 * pcl.niu);
			ds11 = E_tmp * ((1.0 - pcl.niu) * de11_s + pcl.niu * de22_s);
			ds22 = E_tmp * (pcl.niu * de11_s + (1.0 - pcl.niu) * de22_s);
			ds12 = 2.0 * de12_s * pcl.E / (2.0 * (1.0 + pcl.niu));
			
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
			
			// porosity
			pcl.n = (de_vol_s + pcl.n) / (1.0 + de_vol_s);
			// pore pressure
			pcl.p += pcl.Kf * de_vol_f;
			// fluid density
			pcl.density_f += pcl.density_f * de_vol_f;
		}
	}

	return 0;
}


//****************************** Utility Functions *******************************
void Step_R2D_CHM_MPM_BSpline_APIC_s::init_B_matrix(void)
{
	// init nodes
	for (size_t n_id = 0; n_id < model->node_num; ++n_id)
		model->nodes[n_id].cal_flag = 0;
	
	size_t cal_node_num = 0;
	for (size_t pcl_id = 0; pcl_id < model->pcl_num; ++pcl_id)
	{
		auto &pcl = model->pcls[pcl_id];
		pcl.is_in_mesh = model->is_in_mesh(pcl.x, pcl.y);
		if (pcl.is_in_mesh)
		{
			model->cal_shape_func(pcl);
			pcl.vol = pcl.m_s / ((1.0 - pcl.n) * pcl.density_s);
			for (size_t nx_id = 0; nx_id < 3; ++nx_id)
				for (size_t ny_id = 0; ny_id < 3; ++ny_id)
				{
					if (pcl.N[ny_id][nx_id] != 0.0 &&
						!pcl.pn[ny_id][nx_id]->cal_flag)
					{
						++cal_node_num;
						pcl.pn[ny_id][nx_id]->cal_flag = 1;
					}
				}
			Mat_Set_Zero(pcl.C_f);
			Mat_Set_Zero(pcl.C_s);
		}
	}
	double *nvx_f = new double[cal_node_num * 4];
	double *nvy_f = nvx_f + cal_node_num;
	double *nvx_s = nvy_f + cal_node_num;
	double *nvy_s = nvx_s + cal_node_num;
	memset(nvx_f, 0, cal_node_num * 4 * sizeof(double));

	//std::fstream file("init_B_Matrix.txt", std::ios::out);
	size_t iter_id = 0;
	double vx_f_norm, dvx_f_norm, vy_f_norm, dvy_f_norm;
	double vx_s_norm, dvx_s_norm, vy_s_norm, dvy_s_norm;
	do
	{
		// init nodes
		for (size_t n_id = 0; n_id < model->node_num; ++n_id)
		{
			auto &n = model->nodes[n_id];
			n.m_s = 0.0;
			n.vx_s = 0.0;
			n.vy_s = 0.0;
			n.m_tf = 0.0;
			n.vx_f = 0.0;
			n.vy_f = 0.0;
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
						double N = pcl.N[ny_id][nx_id];
						double dN_dx = pcl.dN_dx[ny_id][nx_id];
						double dN_dy = pcl.dN_dy[ny_id][nx_id];
						double x_dist = pcl.x_dist[nx_id];
						double y_dist = pcl.y_dist[ny_id];
						double m_prod_N;
						double Qx, Qy;
						// fluid phase
						m_prod_N = pcl.density_f * pcl.vol * N;
						n.m_tf += m_prod_N;
						Qx = pcl.C_f[0][0] * x_dist + pcl.C_f[0][1] * y_dist;
						Qy = pcl.C_f[1][0] * x_dist + pcl.C_f[1][1] * y_dist;
						n.vx_f += m_prod_N * (pcl.vx_f + Qx);
						n.vy_f += m_prod_N * (pcl.vy_f + Qy);
						// solid phase
						m_prod_N = pcl.m_s * N;
						n.m_s += m_prod_N;
						Qx = pcl.C_s[0][0] * x_dist + pcl.C_s[0][1] * y_dist;
						Qy = pcl.C_s[1][0] * x_dist + pcl.C_s[1][1] * y_dist;
						n.vx_s += m_prod_N * (pcl.vx_s + Qx);
						n.vy_s += m_prod_N * (pcl.vy_s + Qy);
					}
			}
		}

		vx_f_norm = 0.0;
		dvx_f_norm = 0.0;
		vy_f_norm = 0.0;
		dvy_f_norm = 0.0;
		vx_s_norm = 0.0;
		dvx_s_norm = 0.0;
		vy_s_norm = 0.0;
		dvy_s_norm = 0.0;
		size_t cal_n_id = 0;
		for (size_t n_id = 0; n_id < model->node_num; ++n_id)
		{
			auto &pn = model->nodes[n_id];
			if (pn.cal_flag)
			{
				pn.vx_f /= pn.m_tf;
				pn.vy_f /= pn.m_tf;
				pn.vx_s /= pn.m_s;
				pn.vy_s /= pn.m_s;
				// convergence criteria
				double tmp;
				// vx_f
				vx_f_norm += pn.vx_f * pn.vx_f;
				tmp = pn.vx_f - nvx_f[cal_n_id];
				dvx_f_norm += tmp * tmp;
				nvx_f[cal_n_id] = pn.vx_f;
				// vy_f
				vy_f_norm += pn.vy_f * pn.vy_f;
				tmp = pn.vy_f - nvy_f[cal_n_id];
				dvy_f_norm += tmp * tmp;
				nvy_f[cal_n_id] = pn.vy_f;
				// vx_s
				vx_s_norm += pn.vx_s * pn.vx_s;
				tmp = pn.vx_s - nvx_s[cal_n_id];
				dvx_s_norm += tmp * tmp;
				nvx_s[cal_n_id] = pn.vx_s;
				// vy_s
				vy_s_norm += pn.vy_s * pn.vy_s;
				tmp = pn.vy_s - nvy_s[cal_n_id];
				dvy_s_norm += tmp * tmp;
				nvy_s[cal_n_id] = pn.vy_s;

				++cal_n_id;
			}
			//file << pn.vy_f << ", ";
		}
		//file << "\n";

		// form C matrix for APIC
		for (size_t pcl_id = 0; pcl_id < model->pcl_num; ++pcl_id)
		{
			auto &pcl = model->pcls[pcl_id];
			if (pcl.is_in_mesh)
			{
				double B_f[2][2], B_s[2][2]; // B matrix for APIC
				Mat_Set_Zero(B_f);
				Mat_Set_Zero(B_s);
				for (size_t nx_id = 0; nx_id < 3; ++nx_id)
					for (size_t ny_id = 0; ny_id < 3; ++ny_id)
					{
						// B = sum (N * v * dist_T)
						auto &n = *pcl.pn[ny_id][nx_id];
						double N = pcl.N[ny_id][nx_id];
						// map velocity
						double vx_f_prod_N = n.vx_f * N;
						double vy_f_prod_N = n.vy_f * N;
						double vx_s_prod_N = n.vx_s * N;
						double vy_s_prod_N = n.vy_s * N;
						// B = sum (N * v * dist_T)
						double x_dist = pcl.x_dist[nx_id];
						double y_dist = pcl.y_dist[ny_id];
						B_f[0][0] += vx_f_prod_N * x_dist;
						B_f[0][1] += vx_f_prod_N * y_dist;
						B_f[1][0] += vy_f_prod_N * x_dist;
						B_f[1][1] += vy_f_prod_N * y_dist;
						B_s[0][0] += vx_s_prod_N * x_dist;
						B_s[0][1] += vx_s_prod_N * y_dist;
						B_s[1][0] += vy_s_prod_N * x_dist;
						B_s[1][1] += vy_s_prod_N * y_dist;
					}
				// C = B * 1/D
				Mat_Prod_Num(pcl.C_f, B_f, invD);
				Mat_Prod_Num(pcl.C_s, B_s, invD);
			}
		}

		++iter_id;

		// Convergence criteria
#define VELOCITY_CONVERGENCE_LIMIT 0.0001
#define MAXIMUM_ITERACTION_NUMBER 100
		// ||dv|| / ||v|| < 0.01
		//if ((vx_f_norm == 0.0 || dvx_f_norm / vx_f_norm < 0.0001) &&
		//	  (vy_f_norm == 0.0 || dvy_f_norm / vy_f_norm < 0.0001) &&
		//    (vx_s_norm == 0.0 || dvx_s_norm / vx_f_norm < 0.0001) &&
		//	  (vy_s_norm == 0.0 || dvy_s_norm / vy_f_norm < 0.0001))
		//	break;
	} while (!((vx_f_norm == 0.0 || dvx_f_norm / vx_f_norm < VELOCITY_CONVERGENCE_LIMIT)
			&& (vy_f_norm == 0.0 || dvy_f_norm / vy_f_norm < VELOCITY_CONVERGENCE_LIMIT)
			&& (vx_s_norm == 0.0 || dvx_s_norm / vx_s_norm < VELOCITY_CONVERGENCE_LIMIT)
			&& (vy_s_norm == 0.0 || dvy_s_norm / vy_s_norm < VELOCITY_CONVERGENCE_LIMIT))
			&& iter_id < MAXIMUM_ITERACTION_NUMBER);

	delete[] nvx_f;
}