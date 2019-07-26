#include "SimulationCore_pcp.h"

#include <cmath>
#include "Step_S2D_CHM_MPM_s_GIMP.h"

Step_S2D_CHM_MPM_s_GIMP::Step_S2D_CHM_MPM_s_GIMP() :
	Step(&solve_substep_S2D_CHM_MPM_s_GIMP),
	h_elem_raio(0.05), h_pcl_ratio(0.1),
	model(nullptr) {}

Step_S2D_CHM_MPM_s_GIMP::~Step_S2D_CHM_MPM_s_GIMP() {}

int Step_S2D_CHM_MPM_s_GIMP::init()
{
	if (is_first_step)
	{
		for (size_t i = 0; i < model->pcl_num; ++i)
		{
			model->pcls[i].elem_num = 1;
		}
	}

	for (size_t i = 0; i < model->pcl_num; ++i)
	{
		Particle_S2D_CHM &pcl = model->pcls[i];
		pcl.x_ori = pcl.x;
		pcl.y_ori = pcl.y;
	}
	return 0;
}

int Step_S2D_CHM_MPM_s_GIMP::finalize() { return 0; }

inline void max(double &a, double b) noexcept { if (a < b) a = b; }

int solve_substep_S2D_CHM_MPM_s_GIMP(void *_self)
{
	Step_S2D_CHM_MPM_s_GIMP &self = *((Step_S2D_CHM_MPM_s_GIMP *)_self);
	Model_S2D_CHM_MPM_s &model = *(self.model);

	model.pcl_var_mem.reset_optimize();

	// init nodes
	for (size_t n_id = 0; n_id < model.node_num; ++n_id)
	{
		Node_S2D_CHM &n = model.nodes[n_id];
		// solid phase
		n.m_s = 0.0;
		n.ax_s = 0.0;
		n.ay_s = 0.0;
		n.vx_s = 0.0;
		n.vy_s = 0.0;
		n.fx_ext_m = 0.0;
		n.fy_ext_m = 0.0;
		n.fx_int_m = 0.0;
		n.fy_int_m = 0.0;
		n.fx_kin_f = 0.0;
		n.fy_kin_f = 0.0;
		// fluid phase
		n.m_tf = 0.0;
		n.ax_f = 0.0;
		n.ay_f = 0.0;
		n.vx_f = 0.0;
		n.vy_f = 0.0;
		n.fx_ext_tf = 0.0;
		n.fy_ext_tf = 0.0;
		n.fx_int_tf = 0.0;
		n.fy_int_tf = 0.0;
		n.fx_drag_tf = 0.0;
		n.fy_drag_tf = 0.0;
	}

	// map variables to node and cal internal force
	for (size_t pcl_id = 0; pcl_id < model.pcl_num; ++pcl_id)
	{
		Particle_S2D_CHM &pcl = model.pcls[pcl_id];
		if (pcl.elem_num)
		{
			model.get_elements_overlapped_by_particle(pcl);
			pcl.n_prod_k_div_miu = pcl.n * pcl.k / pcl.miu;
			for (size_t i = 0; i < pcl.elem_num; ++i)
			{
				ParticleVar_S2D_CHM &pcl_var = pcl.vars[i];
				double m_s = pcl_var.vol * pcl.density_s * (1.0 - pcl.n);
				double mvx_s = m_s * pcl.vx_s;
				double mvy_s = m_s * pcl.vy_s;
				double m_tf = pcl_var.vol * pcl.density_f;
				double mvx_tf = m_tf * pcl.vx_f;
				double mvy_tf = m_tf * pcl.vy_f;
				
				// ------------------- node 1 -------------------
				Node_S2D_CHM &n1 = *pcl_var.pn1;
				// mixture phase
				n1.m_s  += pcl_var.N1 * m_s;
				n1.vx_s += pcl_var.N1 * mvx_s;
				n1.vy_s += pcl_var.N1 * mvy_s;
				n1.fx_int_m += (pcl_var.dN1_dx * (pcl.s11 - pcl.p) + pcl_var.dN1_dy * pcl.s12) * pcl_var.vol;
				n1.fy_int_m += (pcl_var.dN1_dx * pcl.s12 + pcl_var.dN1_dy * (pcl.s22 - pcl.p)) * pcl_var.vol;
				// fluid phase
				n1.m_tf += pcl_var.N1 * m_tf;
				n1.vx_f += pcl_var.N1 * mvx_tf;
				n1.vy_f += pcl_var.N1 * mvy_tf;
				n1.fx_int_tf += (pcl_var.dN1_dx * -pcl.p) * pcl_var.vol;
				n1.fy_int_tf += (pcl_var.dN1_dy * -pcl.p) * pcl_var.vol;
				n1.fx_drag_tf += pcl_var.N1 * pcl.n_prod_k_div_miu * (pcl.vx_f - pcl.vx_s) * pcl_var.vol;
				n1.fy_drag_tf += pcl_var.N1 * pcl.n_prod_k_div_miu * (pcl.vy_f - pcl.vy_s) * pcl_var.vol;

				// ------------------- node 2 -------------------
				Node_S2D_CHM &n2 = *pcl_var.pn2;
				// mixture phase
				n2.m_s  += pcl_var.N2 * m_s;
				n2.vx_s += pcl_var.N2 * mvx_s;
				n2.vy_s += pcl_var.N2 * mvy_s;
				n2.fx_int_m += (pcl_var.dN2_dx * (pcl.s11 - pcl.p) + pcl_var.dN2_dy * pcl.s12) * pcl_var.vol;
				n2.fy_int_m += (pcl_var.dN2_dx * pcl.s12 + pcl_var.dN2_dy * (pcl.s22 - pcl.p)) * pcl_var.vol;
				// fluid phase
				n2.m_tf += pcl_var.N2 * m_tf;
				n2.vx_f += pcl_var.N2 * mvx_tf;
				n2.vy_f += pcl_var.N2 * mvy_tf;
				n2.fx_int_tf += (pcl_var.dN2_dx * -pcl.p) * pcl_var.vol;
				n2.fy_int_tf += (pcl_var.dN2_dy * -pcl.p) * pcl_var.vol;
				n2.fx_drag_tf += pcl_var.N2 * pcl.n_prod_k_div_miu * (pcl.vx_f - pcl.vx_s) * pcl_var.vol;
				n2.fy_drag_tf += pcl_var.N2 * pcl.n_prod_k_div_miu * (pcl.vy_f - pcl.vy_s) * pcl_var.vol;

				// ------------------- node 3 -------------------
				Node_S2D_CHM &n3 = *pcl_var.pn3;
				// mixture phase
				n3.m_s += pcl_var.N3 * m_s;
				n3.vx_s += pcl_var.N3 * mvx_s;
				n3.vy_s += pcl_var.N3 * mvy_s;
				n3.fx_int_m += (pcl_var.dN3_dx * (pcl.s11 - pcl.p) + pcl_var.dN3_dy * pcl.s12) * pcl_var.vol;
				n3.fy_int_m += (pcl_var.dN3_dx * pcl.s12 + pcl_var.dN3_dy * (pcl.s22 - pcl.p)) * pcl_var.vol;
				// fluid phase
				n3.m_tf += pcl_var.N3 * m_tf;
				n3.vx_f += pcl_var.N3 * mvx_tf;
				n3.vy_f += pcl_var.N3 * mvy_tf;
				n3.fx_int_tf += (pcl_var.dN3_dx * -pcl.p) * pcl_var.vol;
				n3.fy_int_tf += (pcl_var.dN3_dy * -pcl.p) * pcl_var.vol;
				n3.fx_drag_tf += pcl_var.N3 * pcl.n_prod_k_div_miu * (pcl.vx_f - pcl.vx_s) * pcl_var.vol;
				n3.fy_drag_tf += pcl_var.N3 * pcl.n_prod_k_div_miu * (pcl.vy_f - pcl.vy_s) * pcl_var.vol;

				// ------------------- node 4 -------------------
				Node_S2D_CHM &n4 = *pcl_var.pn4;
				// mixture phase
				n4.m_s += pcl_var.N4 * m_s;
				n4.vx_s += pcl_var.N4 * mvx_s;
				n4.vy_s += pcl_var.N4 * mvy_s;
				n4.fx_int_m += (pcl_var.dN4_dx * (pcl.s11 - pcl.p) + pcl_var.dN4_dy * pcl.s12) * pcl_var.vol;
				n4.fy_int_m += (pcl_var.dN4_dx * pcl.s12 + pcl_var.dN4_dy * (pcl.s22 - pcl.p)) * pcl_var.vol;
				// fluid phase
				n4.m_tf += pcl_var.N4 * m_tf;
				n4.vx_f += pcl_var.N4 * mvx_tf;
				n4.vy_f += pcl_var.N4 * mvy_tf;
				n4.fx_int_tf += (pcl_var.dN4_dx * -pcl.p) * pcl_var.vol;
				n4.fy_int_tf += (pcl_var.dN4_dy * -pcl.p) * pcl_var.vol;
				n4.fx_drag_tf += pcl_var.N4 * pcl.n_prod_k_div_miu * (pcl.vx_f - pcl.vx_s) * pcl_var.vol;
				n4.fy_drag_tf += pcl_var.N4 * pcl.n_prod_k_div_miu * (pcl.vy_f - pcl.vy_s) * pcl_var.vol;
			}
		}
	}

	// body force
	double bf_m, bf_tf;
	for (size_t bf_id = 0; bf_id < model.bfsx_num; ++bf_id)
	{
		ppcl = model->pcls + model->bfxs[i].pcl_id;
		// body force on particle
		bf_m = pcl_var.vol * ((1.0 - pcl_var.n) * pcl_var.density_s + pcl_var.n * pcl_var.density_f) * model->bfxs[i].bf;
		bf_tf = pcl_var.vol * pcl_var.density_f * model->bfxs[i].bf;
		// node 1
		pn1 = pcl_var.node1;
		n1.fx_ext_m += pcl_var.N1 * bf_m;
		n1.fx_ext_tf += pcl_var.N1 * bf_tf;
		// node 2
		pn2 = pcl_var.node2;
		n2.fx_ext_m += pcl_var.N2 * bf_m;
		n2.fx_ext_tf += pcl_var.N2 * bf_tf;
		// node 3
		pn3 = pcl_var.node3;
		n3.fx_ext_m += pcl_var.N3 * bf_m;
		n3.fx_ext_tf += pcl_var.N3 * bf_tf;
		// node 4
		pn4 = pcl_var.node4;
		n4.fx_ext_m += pcl_var.N4 * bf_m;
		n4.fx_ext_tf += pcl_var.N4 * bf_tf;
	}
	for (size_t i = 0; i < model->bfy_num; i++)
	{
		ppcl = model->pcls + model->bfys[i].pcl_id;
		// body force on particle
		bf_m = pcl_var.vol * ((1.0 - pcl_var.n) * pcl_var.density_s + pcl_var.n * pcl_var.density_f) * model->bfys[i].bf;
		bf_tf = pcl_var.vol * pcl_var.density_f * model->bfys[i].bf;
		// node 1
		pn1 = pcl_var.node1;
		n1.fy_ext_m += pcl_var.N1 * bf_m;
		n1.fy_ext_tf += pcl_var.N1 * bf_tf;
		// node 2
		pn2 = pcl_var.node2;
		n2.fy_ext_m += pcl_var.N2 * bf_m;
		n2.fy_ext_tf += pcl_var.N2 * bf_tf;
		// node 3
		pn3 = pcl_var.node3;
		n3.fy_ext_m += pcl_var.N3 * bf_m;
		n3.fy_ext_tf += pcl_var.N3 * bf_tf;
		// node 4
		pn4 = pcl_var.node4;
		n4.fy_ext_m += pcl_var.N4 * bf_m;
		n4.fy_ext_tf += pcl_var.N4 * bf_tf;
	}

	// surface force
	for (size_t i = 0; i < model->tx_bc_num; i++)
	{
		ppcl = model->pcls + model->tx_bcs[i].pcl_id;
		// node 1
		pn1 = pcl_var.node1;
		n1.fx_ext_m += pcl_var.N1 * model->tx_bcs[i].t;
		// node 2
		pn2 = pcl_var.node2;
		n2.fx_ext_m += pcl_var.N2 * model->tx_bcs[i].t;
		// node 3
		pn3 = pcl_var.node3;
		n3.fx_ext_m += pcl_var.N3 * model->tx_bcs[i].t;
		// node 4
		pn4 = pcl_var.node4;
		n4.fx_ext_m += pcl_var.N4 * model->tx_bcs[i].t;
	}
	for (size_t i = 0; i < model->ty_bc_num; i++)
	{
		ppcl = model->pcls + model->ty_bcs[i].pcl_id;
		// node 1
		pn1 = pcl_var.node1;
		n1.fy_ext_m += pcl_var.N1 * model->ty_bcs[i].t;
		// node 2
		pn2 = pcl_var.node2;
		n2.fy_ext_m += pcl_var.N2 * model->ty_bcs[i].t;
		// node 3
		pn3 = pcl_var.node3;
		n3.fy_ext_m += pcl_var.N3 * model->ty_bcs[i].t;
		// node 4
		pn4 = pcl_var.node4;
		n4.fy_ext_m += pcl_var.N4 * model->ty_bcs[i].t;
	}
	// pore pressure bc here...

	// update nodal acceleration of fluid pahse
	for (size_t n_id = 0; n_id < model.node_num; ++n_id)
	{
		Node_S2D_CHM &n = model.nodes[n_id];
		if (n.m_tf != 0.0)
		{
			n.ax_f = (n.fx_ext_tf - n.fx_int_tf - n.fx_drag_tf) / n.m_tf;
			n.ay_f = (n.fy_ext_tf - n.fy_int_tf - n.fy_drag_tf) / n.m_tf;
		}
	}
	for (size_t a_id = 0; a_id < model.afx_num; ++a_id)
		model.nodes[model.afxs[a_id].node_id].ax_f = model.afxs[a_id].a;
	for (size_t a_id = 0; a_id < model.afy_num; ++a_id)
		model.nodes[model.afys[a_id].node_id].ay_f = model.afys[a_id].a;

	// update nodal momentum of fluid phase
	for (size_t n_id = 0; n_id < model.node_num; ++n_id)
	{
		Node_S2D_CHM &n = model.nodes[n_id];
		if (n.m_tf != 0.0)
		{
			n.vx_f /= n.m_tf;
			n.vx_f += n.ax_f * self.dt;
			n.vy_f /= n.m_tf;
			n.vy_f += n.ay_f * self.dt;
		}
	}
	// apply velocity boundary conditions of fluid phase
	for (size_t v_id = 0; v_id < model.vfx_num; ++v_id)
	{
		Node_S2D_CHM &n = model.nodes[model.vfxs[v_id].node_id];
		n.vx_f = model.vfxs[v_id].v;
		n.ax_f = 0.0;
	}
	for (size_t v_id = 0; v_id < model.vfy_num; ++v_id)
	{
		Node_S2D_CHM &n = model.nodes[model.vfys[v_id].node_id];
		n.vy_f = model.vfys[v_id].v;
		n.ay_f = 0.0;
	}

	// calculate the inertial term of fluid in mixture formulation
	double pcl_ax_f, pcl_ay_f, pcl_m_f, pcl_max_f, pcl_may_f;
	for (size_t pcl_id = 0; pcl_id < model.pcl_num; ++pcl_id)
	{
		Particle_S2D_CHM &pcl = model.pcls[pcl_id];
		for (size_t i = 0; i < pcl.elem_num; i++)
		{
			ParticleVar_S2D_CHM &pcl_var = pcl.vars[i];
			Node_S2D_CHM &n1 = *pcl_var.pn1;
			Node_S2D_CHM &n2 = *pcl_var.pn2;
			Node_S2D_CHM &n3 = *pcl_var.pn3;
			Node_S2D_CHM &n4 = *pcl_var.pn4;
			// particle acceleration
			pcl_ax_f = pcl_var.N1 * n1.ax_f + pcl_var.N2 * n2.ax_f
					 + pcl_var.N3 * n3.ax_f + pcl_var.N4 * n4.ax_f;
			pcl_ay_f = pcl_var.N1 * n1.ay_f + pcl_var.N2 * n2.ay_f
					 + pcl_var.N3 * n3.ay_f + pcl_var.N4 * n4.ay_f;
			pcl_m_f = pcl.n * pcl.density_f * pcl_var.vol;
			pcl_max_f = pcl_m_f * pcl_ax_f;
			pcl_may_f = pcl_m_f * pcl_ay_f;
			// node 1
			n1.fx_kin_f += pcl_var.N1 * pcl_max_f;
			n1.fy_kin_f += pcl_var.N1 * pcl_may_f;
			// node 2
			n2.fx_kin_f += pcl_var.N2 * pcl_max_f;
			n2.fy_kin_f += pcl_var.N2 * pcl_may_f;
			// node 3
			n3.fx_kin_f += pcl_var.N3 * pcl_max_f;
			n3.fy_kin_f += pcl_var.N3 * pcl_may_f;
			// node 4
			n4.fx_kin_f += pcl_var.N4 * pcl_max_f;
			n4.fy_kin_f += pcl_var.N4 * pcl_may_f;
		}
	}

	// update nodal velocity of solid phase
	for (size_t n_id = 0; n_id < model.node_num; ++n_id)
	{
		Node_S2D_CHM &n = model.nodes[n_id];
		if (n.m_s != 0.0)
		{
			n.ax_s = (n.fx_ext_m - n.fx_int_m - n.fx_kin_f) / n.m_s;
			n.ay_s = (n.fy_ext_m - n.fy_int_m - n.fy_kin_f) / n.m_s;
		}
	}
	// apply acceleration boundary conditions
	for (size_t a_id = 0; a_id < model.asx_num; ++a_id)
		model.nodes[model.asxs[a_id].node_id].ax_s = model.asxs[a_id].a;
	for (size_t a_id = 0; a_id < model.asy_num; ++a_id)
		model.nodes[model.asys[a_id].node_id].ay_s = model.asys[a_id].a;

	// update solid phase nodal velocity
	for (size_t n_id = 0; n_id < model.node_num; ++n_id)
	{
		Node_S2D_CHM &n = model.nodes[n_id];
		if (n.m_s != 0.0)
		{
			n.vx_s /= n.m_s;
			n.vx_s += n.ax_s * self.dt;
			n.vy_s /= n.m_s;
			n.vy_s += n.ay_s * self.dt;
		}
	}
	// apply velocity boundary conditions of solid phase
	for (size_t v_id = 0; v_id < model.vsx_num; ++v_id)
	{
		Node_S2D_CHM &n = model.nodes[model.vsxs[v_id].node_id];
		n.vx_s = model.vsxs[v_id].v;
		n.ax_s = 0.0;
	}
	for (size_t v_id = 0; v_id < model.vsy_num; ++v_id)
	{
		Node_S2D_CHM &n = model.nodes[model.vsys[v_id].node_id];
		n.vy_s = model.vsys[v_id].v;
		n.ay_s = 0.0;
	}

	// map variables back to and update variables particles
	double de11_s, de22_s, de12_s, dw12, de_vol_s, de_vol_f;
	double E_tmp, ds11, ds22, ds12;
	for (size_t pcl_id = 0; pcl_id < model.pcl_num; ++pcl_id)
	{
		Particle_S2D_CHM &pcl = model.pcls[pcl_id];
		if (pcl.elem_num)
		{
			ParticleVar_S2D_CHM &pcl_var = pcl.var;
			Node_S2D_CHM &n1 = *pcl_var.pn1;
			Node_S2D_CHM &n2 = *pcl_var.pn2;
			Node_S2D_CHM &n3 = *pcl_var.pn3;
			Node_S2D_CHM &n4 = *pcl_var.pn4;

			pcl.vx_s += (pcl_var.N1 * n1.ax_s + pcl_var.N2 * n2.ax_s
					   + n3.ax_s * N3_tmp + n4.ax_s * N4_tmp) * self.dt;
			pcl.vy_s += (n1.ay_s * N1_tmp + n2.ay_s * N2_tmp
				+ n3.ay_s * N3_tmp + n4.ay_s * N4_tmp) * self.dt;
			pcl.vx_f += (n1.ax_f * N1_tmp + n2.ax_f * N2_tmp
				+ n3.ax_f * N3_tmp + n4.ax_f * N4_tmp) * self.dt;
			pcl.vy_f += (n1.ay_f * N1_tmp + n2.ay_f * N2_tmp
				+ n3.ay_f * N3_tmp + n4.ay_f * N4_tmp) * self.dt;

			pcl.ux_s += n1.dux_s * N1_tmp + n2.dux_s * N2_tmp
				+ n3.dux_s * N3_tmp + n4.dux_s * N4_tmp;
			pcl.uy_s += n1.duy_s * N1_tmp + n2.duy_s * N2_tmp
				+ n3.duy_s * N3_tmp + n4.duy_s * N4_tmp;
			pcl.ux_f += n1.dux_f * N1_tmp + n2.dux_f * N2_tmp
				+ n3.dux_f * N3_tmp + n4.dux_f * N4_tmp;
			pcl.uy_f += n1.duy_f * N1_tmp + n2.duy_f * N2_tmp
				+ n3.duy_f * N3_tmp + n4.duy_f * N4_tmp;
			pcl.x = pcl.x_ori + pcl.ux_s;
			pcl.y = pcl.y_ori + pcl.uy_s;

			// strain increment
			de11_s = n1.dux_s * pcl_var.dN1_dx + n2.dux_s * pcl_var.dN2_dx
				+ n3.dux_s * pcl_var.dN3_dx + n4.dux_s * pcl_var.dN4_dx;
			de22_s = n1.duy_s * pcl_var.dN1_dy + n2.duy_s * pcl_var.dN2_dy
				+ n3.duy_s * pcl_var.dN3_dy + n4.duy_s * pcl_var.dN4_dy;
			de12_s = (n1.dux_s * pcl_var.dN1_dy + n2.dux_s * pcl_var.dN2_dy
				+ n3.dux_s * pcl_var.dN3_dy + n4.dux_s * pcl_var.dN4_dy
				+ n1.duy_s * pcl_var.dN1_dx + n2.duy_s * pcl_var.dN2_dx
				+ n3.duy_s * pcl_var.dN3_dx + n4.duy_s * pcl_var.dN4_dx) * 0.5;
			dw12 = (n1.dux_s * pcl_var.dN1_dy + n2.dux_s * pcl_var.dN2_dy
				+ n3.dux_s * pcl_var.dN3_dy + n4.dux_s * pcl_var.dN4_dy
				- n1.duy_s * pcl_var.dN1_dx - n2.duy_s * pcl_var.dN2_dx
				- n3.duy_s * pcl_var.dN3_dx - n4.duy_s * pcl_var.dN4_dx) * 0.5;

			// update strain (also assume that strain increment is Jaumann rate)
			//pcl_var.de11 +=  dw12 * e12 * 2.0;
			//pcl_var.de22 += -dw12 * e12 * 2.0;
			//pcl_var.de12 +=  dw12 * (e22 - e11);
			pcl.e11 += de11_s;
			pcl.e22 += de22_s;
			pcl.e12 += de12_s;

			// update stress
			E_tmp = pcl.E / (1.0 + pcl.niu) / (1.0 - 2.0 * pcl.niu);
			ds11 = E_tmp * ((1.0 - pcl.niu) * de11_s + pcl.niu * de22_s);
			ds22 = E_tmp * (pcl.niu * de11_s + (1.0 - pcl.niu) * de22_s);
			ds12 = 2.0 * de12_s * pcl.E / (2.0 * (1.0 + pcl.niu));

			/* ------------------------------------------------------------------
			Rotate as Jaumann rate:
			tensor_rate = tensor_Jaumann_rate + tensor * dW_T + dW * tensor
			------------------------------------------------------------------- */
			/*			  ds11 +=  pcl_var.dw12 * pcl_var.s12 * 2.0;
			ds22 += -pcl_var.dw12 * pcl_var.s12 * 2.0;
			ds12 +=  pcl_var.dw12 * (pcl_var.s22 - pcl_var.s11);	*/
			pcl.s11 += ds11;
			pcl.s22 += ds22;
			pcl.s12 += ds12;

			// volumetric strain of solid phase
			de_vol_s = de11_s + de22_s;
			// porosity
			pcl.n = (de_vol_s + pcl.n) / (1.0 + de_vol_s);

			// "volumetric strain" of fluid phase
			de_vol_f = -(1.0 - pcl.n) / pcl.n * de_vol_s
					  - (n1.vx_f * pcl_var.dN1_dx + n2.vx_f * pcl_var.dN2_dx
					   + n3.vx_f * pcl_var.dN3_dx + n4.vx_f * pcl_var.dN4_dx
					   + n1.vy_f * pcl_var.dN1_dy + n2.vy_f * pcl_var.dN2_dy
					   + n3.vy_f * pcl_var.dN3_dy + n4.vy_f * pcl_var.dN4_dy) * self.dt;
			// pore pressure
			pcl.p += pcl.Kf * de_vol_f;
			// fluid density
			pcl.density_f += pcl.density_f * de_vol_f;
		}
	}

	return 0;
}