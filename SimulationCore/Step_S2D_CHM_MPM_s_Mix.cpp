#include "SimulationCore_pcp.h"

#include <cmath>
#include "Step_S2D_CHM_MPM_s_Mix.h"

Step_S2D_CHM_MPM_s_Mix::Step_S2D_CHM_MPM_s_Mix() :
	Step(&solve_substep_S2D_CHM_MPM_s_Mix),
	model(nullptr) {}

Step_S2D_CHM_MPM_s_Mix::~Step_S2D_CHM_MPM_s_Mix() {}

int Step_S2D_CHM_MPM_s_Mix::init()
{
	if (is_first_step)
	{
		for (size_t i = 0; i < model->pcl_num; ++i)
		{
			model->pcls[i].is_in_mesh = true;
		}
	}

	for (size_t i = 0; i < model->pcl_num; ++i)
	{
		Particle_S2D_CHM_Mix &pcl = model->pcls[i];
		pcl.x_ori = pcl.x;
		pcl.y_ori = pcl.y;
	}
	return 0;
}

int Step_S2D_CHM_MPM_s_Mix::finalize() { return 0; }

#define GET_ELEMENT_BY_ID(x_id, y_id) model.elems[(y_id) * model.elem_x_num + (x_id)];

int solve_substep_S2D_CHM_MPM_s_Mix(void *_self)
{
	Step_S2D_CHM_MPM_s_Mix &self = *((Step_S2D_CHM_MPM_s_Mix *)_self);
	Model_S2D_CHM_MPM_s_Mix &model = *(self.model);

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

	// init elements
	for (size_t e_id = 0; e_id < model.elem_num; ++e_id)
	{
		Element_S2D_CHM_Mix &elem = model.elems[e_id];
		elem.mat_vol = 0.0;
		elem.top = nullptr;
	}

	// map mass velocity and drag force
	for (size_t pcl_id = 0; pcl_id < model.pcl_num; ++pcl_id)
	{
		Particle_S2D_CHM_Mix &pcl = model.pcls[pcl_id];
		pcl.is_in_mesh = model.find_in_which_element(pcl);
		if (pcl.is_in_mesh)
		{
			Element_S2D_CHM_Mix &elem = GET_ELEMENT_BY_ID(pcl.elem_x_id, pcl.elem_y_id);
			elem.add_pcl(pcl);
			pcl.vol = pcl.m_s / ((1.0 - pcl.n) * pcl.density_s);
			elem.mat_vol += pcl.vol;
			IntegrationVar_S2D_CHM_Mix &pcl_var = pcl.var;
			model.cal_shape_func(pcl_var);
			double m_s  = pcl.m_s;
			double mvx_s = m_s * pcl.vx_s;
			double mvy_s = m_s * pcl.vy_s;
			double m_tf = pcl.density_f * pcl.vol;
			double mvx_tf = m_tf * pcl.vx_f;
			double mvy_tf = m_tf * pcl.vy_f;
			double n_miu_div_k = pcl.n * pcl.miu / pcl.k;
			double n_miu_div_k_vrx_vol = n_miu_div_k * (pcl.vx_f - pcl.vx_s) * pcl.vol;
			double n_miu_div_k_vry_vol = n_miu_div_k * (pcl.vy_f - pcl.vy_s) * pcl.vol;
			// ------------------- node 1 -------------------
			Node_S2D_CHM &n1 = *pcl_var.pn1;
			// mixture phase
			n1.m_s += pcl_var.N1 * m_s;
			n1.vx_s += pcl_var.N1 * mvx_s;
			n1.vy_s += pcl_var.N1 * mvy_s;
			//n1.fx_int_m += (pcl_var.dN1_dx * (pcl.s11 - pcl.p) + pcl_var.dN1_dy * pcl.s12) * pcl.vol;
			//n1.fy_int_m += (pcl_var.dN1_dx * pcl.s12 + pcl_var.dN1_dy * (pcl.s22 - pcl.p)) * pcl.vol;
			// fluid phase
			n1.m_tf += pcl_var.N1 * m_tf;
			n1.vx_f += pcl_var.N1 * mvx_tf;
			n1.vy_f += pcl_var.N1 * mvy_tf;
			//n1.fx_int_tf += (pcl_var.dN1_dx * -pcl.p) * pcl.vol;
			//n1.fy_int_tf += (pcl_var.dN1_dy * -pcl.p) * pcl.vol;
			n1.fx_drag_tf += pcl_var.N1 * n_miu_div_k_vrx_vol;
			n1.fy_drag_tf += pcl_var.N1 * n_miu_div_k_vry_vol;
			// ------------------- node 2 -------------------
			Node_S2D_CHM &n2 = *pcl_var.pn2;
			// mixture phase
			n2.m_s += pcl_var.N2 * m_s;
			n2.vx_s += pcl_var.N2 * mvx_s;
			n2.vy_s += pcl_var.N2 * mvy_s;
			//n2.fx_int_m += (pcl_var.dN2_dx * (pcl.s11 - pcl.p) + pcl_var.dN2_dy * pcl.s12) * pcl.vol;
			//n2.fy_int_m += (pcl_var.dN2_dx * pcl.s12 + pcl_var.dN2_dy * (pcl.s22 - pcl.p)) * pcl.vol;
			// fluid phase
			n2.m_tf += pcl_var.N2 * m_tf;
			n2.vx_f += pcl_var.N2 * mvx_tf;
			n2.vy_f += pcl_var.N2 * mvy_tf;
			//n2.fx_int_tf += (pcl_var.dN2_dx * -pcl.p) * pcl.vol;
			//n2.fy_int_tf += (pcl_var.dN2_dy * -pcl.p) * pcl.vol;
			n2.fx_drag_tf += pcl_var.N2 * n_miu_div_k_vrx_vol;
			n2.fy_drag_tf += pcl_var.N2 * n_miu_div_k_vry_vol;
			// ------------------- node 3 -------------------
			Node_S2D_CHM &n3 = *pcl_var.pn3;
			// mixture phase
			n3.m_s += pcl_var.N3 * m_s;
			n3.vx_s += pcl_var.N3 * mvx_s;
			n3.vy_s += pcl_var.N3 * mvy_s;
			//n3.fx_int_m += (pcl_var.dN3_dx * (pcl.s11 - pcl.p) + pcl_var.dN3_dy * pcl.s12) * pcl.vol;
			//n3.fy_int_m += (pcl_var.dN3_dx * pcl.s12 + pcl_var.dN3_dy * (pcl.s22 - pcl.p)) * pcl.vol;
			// fluid phase
			n3.m_tf += pcl_var.N3 * m_tf;
			n3.vx_f += pcl_var.N3 * mvx_tf;
			n3.vy_f += pcl_var.N3 * mvy_tf;
			//n3.fx_int_tf += (pcl_var.dN3_dx * -pcl.p) * pcl.vol;
			//n3.fy_int_tf += (pcl_var.dN3_dy * -pcl.p) * pcl.vol;
			n3.fx_drag_tf += pcl_var.N3 * n_miu_div_k_vrx_vol;
			n3.fy_drag_tf += pcl_var.N3 * n_miu_div_k_vry_vol;
			// ------------------- node 4 -------------------
			Node_S2D_CHM &n4 = *pcl_var.pn4;
			// mixture phase
			n4.m_s += pcl_var.N4 * m_s;
			n4.vx_s += pcl_var.N4 * mvx_s;
			n4.vy_s += pcl_var.N4 * mvy_s;
			//n4.fx_int_m += (pcl_var.dN4_dx * (pcl.s11 - pcl.p) + pcl_var.dN4_dy * pcl.s12) * pcl.vol;
			//n4.fy_int_m += (pcl_var.dN4_dx * pcl.s12 + pcl_var.dN4_dy * (pcl.s22 - pcl.p)) * pcl.vol;
			// fluid phase
			n4.m_tf += pcl_var.N4 * m_tf;
			n4.vx_f += pcl_var.N4 * mvx_tf;
			n4.vy_f += pcl_var.N4 * mvy_tf;
			//n4.fx_int_tf += (pcl_var.dN4_dx * -pcl.p) * pcl.vol;
			//n4.fy_int_tf += (pcl_var.dN4_dy * -pcl.p) * pcl.vol;
			n4.fx_drag_tf += pcl_var.N4 * n_miu_div_k_vrx_vol;
			n4.fy_drag_tf += pcl_var.N4 * n_miu_div_k_vry_vol;
		}
	}

	// Internal force (mix integration)
	for (size_t e_id = 0; e_id < model.elem_num; ++e_id)
	{
		Element_S2D_CHM_Mix &elem = model.elems[e_id];
		if (elem.mat_vol > model.elem_vol_threshold) // Gauss point integration
		//if (false)
		{
			elem.avg_p = 0.0;
			elem.avg_s11 = 0.0;
			elem.avg_s22 = 0.0;
			elem.avg_s12 = 0.0;
			for (Particle_S2D_CHM_Mix *ppcl = elem.top; ppcl; ppcl = ppcl->next)
			{
				elem.avg_p   += ppcl->vol * ppcl->p;
				elem.avg_s11 += ppcl->vol * ppcl->s11;
				elem.avg_s22 += ppcl->vol * ppcl->s22;
				elem.avg_s12 += ppcl->vol * ppcl->s12;
			}
			elem.avg_p   /= elem.mat_vol;
			elem.avg_s11 /= elem.mat_vol;
			elem.avg_s22 /= elem.mat_vol;
			elem.avg_s12 /= elem.mat_vol;
			IntegrationVar_S2D_CHM_Mix elem_var;
			elem_var.x = model.x0 + (double(elem.index_x) + 0.5) * model.h;
			elem_var.y = model.y0 + (double(elem.index_y) + 0.5) * model.h;
			elem_var.w = model.elem_vol;
			elem_var.elem_x_id = elem.index_x;
			elem_var.elem_y_id = elem.index_y;
			model.cal_shape_func(elem_var);
			// Node 1
			Node_S2D_CHM &n1 = *elem_var.pn1;
			// mixture phase
			n1.fx_int_m += (elem_var.dN1_dx * (elem.avg_s11 - elem.avg_p) + elem_var.dN1_dy * elem.avg_s12) * elem_var.w;
			n1.fy_int_m += (elem_var.dN1_dx * elem.avg_s12 + elem_var.dN1_dy * (elem.avg_s22 - elem.avg_p)) * elem_var.w;
			// fluid phase
			n1.fx_int_tf += (elem_var.dN1_dx * -elem.avg_p) * elem_var.w;
			n1.fy_int_tf += (elem_var.dN1_dy * -elem.avg_p) * elem_var.w;
			// Node 2
			Node_S2D_CHM &n2 = *elem_var.pn2;
			// mixture phase
			n2.fx_int_m += (elem_var.dN2_dx * (elem.avg_s11 - elem.avg_p) + elem_var.dN2_dy * elem.avg_s12) * elem_var.w;
			n2.fy_int_m += (elem_var.dN2_dx * elem.avg_s12 + elem_var.dN2_dy * (elem.avg_s22 - elem.avg_p)) * elem_var.w;
			// fluid phase
			n2.fx_int_tf += (elem_var.dN2_dx * -elem.avg_p) * elem_var.w;
			n2.fy_int_tf += (elem_var.dN2_dy * -elem.avg_p) * elem_var.w;
			// Node 3
			Node_S2D_CHM &n3 = *elem_var.pn3;
			// mixture phase
			n3.fx_int_m += (elem_var.dN3_dx * (elem.avg_s11 - elem.avg_p) + elem_var.dN3_dy * elem.avg_s12) * elem_var.w;
			n3.fy_int_m += (elem_var.dN3_dx * elem.avg_s12 + elem_var.dN3_dy * (elem.avg_s22 - elem.avg_p)) * elem_var.w;
			// fluid phase
			n3.fx_int_tf += (elem_var.dN3_dx * -elem.avg_p) * elem_var.w;
			n3.fy_int_tf += (elem_var.dN3_dy * -elem.avg_p) * elem_var.w;
			// Node 4
			Node_S2D_CHM &n4 = *elem_var.pn4;
			// mixture phase
			n4.fx_int_m += (elem_var.dN4_dx * (elem.avg_s11 - elem.avg_p) + elem_var.dN4_dy * elem.avg_s12) * elem_var.w;
			n4.fy_int_m += (elem_var.dN4_dx * elem.avg_s12 + elem_var.dN4_dy * (elem.avg_s22 - elem.avg_p)) * elem_var.w;
			// fluid phase
			n4.fx_int_tf += (elem_var.dN4_dx * -elem.avg_p) * elem_var.w;
			n4.fy_int_tf += (elem_var.dN4_dy * -elem.avg_p) * elem_var.w;
			//std::cout << "gauss inte\n";
		}
		else // Standard integration
		{
			for (Particle_S2D_CHM_Mix *ppcl = elem.top; ppcl; ppcl = ppcl->next)
			{
				Particle_S2D_CHM_Mix &pcl = *ppcl;
				IntegrationVar_S2D_CHM_Mix &pcl_var = pcl.var;
				// ------------------- node 1 -------------------
				Node_S2D_CHM &n1 = *pcl_var.pn1;
				// mixture phase
				n1.fx_int_m += (pcl_var.dN1_dx * (pcl.s11 - pcl.p) + pcl_var.dN1_dy * pcl.s12) * pcl.vol;
				n1.fy_int_m += (pcl_var.dN1_dx * pcl.s12 + pcl_var.dN1_dy * (pcl.s22 - pcl.p)) * pcl.vol;
				// fluid phase
				n1.fx_int_tf += (pcl_var.dN1_dx * -pcl.p) * pcl.vol;
				n1.fy_int_tf += (pcl_var.dN1_dy * -pcl.p) * pcl.vol;
				// ------------------- node 2 -------------------
				Node_S2D_CHM &n2 = *pcl_var.pn2;
				// mixture phase
				n2.fx_int_m += (pcl_var.dN2_dx * (pcl.s11 - pcl.p) + pcl_var.dN2_dy * pcl.s12) * pcl.vol;
				n2.fy_int_m += (pcl_var.dN2_dx * pcl.s12 + pcl_var.dN2_dy * (pcl.s22 - pcl.p)) * pcl.vol;
				// fluid phase
				n2.fx_int_tf += (pcl_var.dN2_dx * -pcl.p) * pcl.vol;
				n2.fy_int_tf += (pcl_var.dN2_dy * -pcl.p) * pcl.vol;
				// ------------------- node 3 -------------------
				Node_S2D_CHM &n3 = *pcl_var.pn3;
				// mixture phase
				n3.fx_int_m += (pcl_var.dN3_dx * (pcl.s11 - pcl.p) + pcl_var.dN3_dy * pcl.s12) * pcl.vol;
				n3.fy_int_m += (pcl_var.dN3_dx * pcl.s12 + pcl_var.dN3_dy * (pcl.s22 - pcl.p)) * pcl.vol;
				// fluid phase
				n3.fx_int_tf += (pcl_var.dN3_dx * -pcl.p) * pcl.vol;
				n3.fy_int_tf += (pcl_var.dN3_dy * -pcl.p) * pcl.vol;
				// ------------------- node 4 -------------------
				Node_S2D_CHM &n4 = *pcl_var.pn4;
				// mixture phase
				n4.fx_int_m += (pcl_var.dN4_dx * (pcl.s11 - pcl.p) + pcl_var.dN4_dy * pcl.s12) * pcl.vol;
				n4.fy_int_m += (pcl_var.dN4_dx * pcl.s12 + pcl_var.dN4_dy * (pcl.s22 - pcl.p)) * pcl.vol;
				// fluid phase
				n4.fx_int_tf += (pcl_var.dN4_dx * -pcl.p) * pcl.vol;
				n4.fy_int_tf += (pcl_var.dN4_dy * -pcl.p) * pcl.vol;
			}
		}
	}

	// body force
	double bf_m, bf_tf;
	for (size_t bf_id = 0; bf_id < model.bfx_num; ++bf_id)
	{
		Particle_S2D_CHM_Mix &pcl = model.pcls[model.bfxs[bf_id].pcl_id];
		if (pcl.is_in_mesh)
		{
			IntegrationVar_S2D_CHM_Mix &pcl_var = pcl.var;
			// body force on particle
			bf_m = ((1.0 - pcl.n) * pcl.density_s + pcl.n * pcl.density_f)
				* pcl.vol * model.bfxs[bf_id].bf;
			bf_tf = pcl.density_f * pcl.vol * model.bfxs[bf_id].bf;
			// node 1
			pcl_var.pn1->fx_ext_m += pcl_var.N1 * bf_m;
			pcl_var.pn1->fx_ext_tf += pcl_var.N1 * bf_tf;
			// node 2
			pcl_var.pn2->fx_ext_m += pcl_var.N2 * bf_m;
			pcl_var.pn2->fx_ext_tf += pcl_var.N2 * bf_tf;
			// node 3
			pcl_var.pn3->fx_ext_m += pcl_var.N3 * bf_m;
			pcl_var.pn3->fx_ext_tf += pcl_var.N3 * bf_tf;
			// node 4
			pcl_var.pn4->fx_ext_m += pcl_var.N4 * bf_m;
			pcl_var.pn4->fx_ext_tf += pcl_var.N4 * bf_tf;
		}
	}
	for (size_t bf_id = 0; bf_id < model.bfx_num; ++bf_id)
	{
		Particle_S2D_CHM_Mix &pcl = model.pcls[model.bfys[bf_id].pcl_id];
		if (pcl.is_in_mesh)
		{
			IntegrationVar_S2D_CHM_Mix &pcl_var = pcl.var;
			// body force on particle
			bf_m = ((1.0 - pcl.n) * pcl.density_s + pcl.n * pcl.density_f)
				* pcl.vol * model.bfys[bf_id].bf;
			bf_tf = pcl.density_f * pcl.vol * model.bfys[bf_id].bf;
			// node 1
			pcl_var.pn1->fy_ext_m += pcl_var.N1 * bf_m;
			pcl_var.pn1->fy_ext_tf += pcl_var.N1 * bf_tf;
			// node 2
			pcl_var.pn2->fy_ext_m += pcl_var.N2 * bf_m;
			pcl_var.pn2->fy_ext_tf += pcl_var.N2 * bf_tf;
			// node 3
			pcl_var.pn3->fy_ext_m += pcl_var.N3 * bf_m;
			pcl_var.pn3->fy_ext_tf += pcl_var.N3 * bf_tf;
			// node 4
			pcl_var.pn4->fy_ext_m += pcl_var.N4 * bf_m;
			pcl_var.pn4->fy_ext_tf += pcl_var.N4 * bf_tf;
		}
	}

	// surface force
	double t_tmp;
	for (size_t t_id = 0; t_id < model.tx_num; ++t_id)
	{
		Particle_S2D_CHM_Mix &pcl = model.pcls[model.txs[t_id].pcl_id];
		if (pcl.is_in_mesh)
		{
			IntegrationVar_S2D_CHM_Mix &pcl_var = pcl.var;
			t_tmp = model.txs[t_id].t;
			pcl_var.pn1->fx_ext_m += pcl_var.N1 * t_tmp;
			pcl_var.pn2->fx_ext_m += pcl_var.N2 * t_tmp;
			pcl_var.pn3->fx_ext_m += pcl_var.N3 * t_tmp;
			pcl_var.pn4->fx_ext_m += pcl_var.N4 * t_tmp;
		}
	}
	for (size_t t_id = 0; t_id < model.ty_num; ++t_id)
	{
		Particle_S2D_CHM_Mix &pcl = model.pcls[model.tys[t_id].pcl_id];
		if (pcl.is_in_mesh)
		{
			IntegrationVar_S2D_CHM_Mix &pcl_var = pcl.var;
			t_tmp = model.tys[t_id].t;
			pcl_var.pn1->fy_ext_m += pcl_var.N1 * t_tmp;
			pcl_var.pn2->fy_ext_m += pcl_var.N2 * t_tmp;
			pcl_var.pn3->fy_ext_m += pcl_var.N3 * t_tmp;
			pcl_var.pn4->fy_ext_m += pcl_var.N4 * t_tmp;
		}
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
			//std::cout << "m: fint_tf " << n.fx_int_tf << "\n";
		}
	}
	// apply acceleration bc of fluid phase
	for (size_t a_id = 0; a_id < model.afx_num; ++a_id)
		model.nodes[model.afxs[a_id].node_id].ax_f = model.afxs[a_id].a;
	for (size_t a_id = 0; a_id < model.afy_num; ++a_id)
		model.nodes[model.afys[a_id].node_id].ay_f = model.afys[a_id].a;
	// apply velocity bc (acceleration part) of fluid phase
	for (size_t v_id = 0; v_id < model.vfx_num; ++v_id)
		model.nodes[model.vfxs[v_id].node_id].ax_f = 0.0;
	for (size_t v_id = 0; v_id < model.vfy_num; ++v_id)
		model.nodes[model.vfys[v_id].node_id].ay_f = 0.0;
	
	// calculate inertial term of fluid phase in mixture formulation
	double pcl_ax_f, pcl_ay_f, pcl_m_f, pcl_max_f, pcl_may_f;
	for (size_t pcl_id = 0; pcl_id < model.pcl_num; ++pcl_id)
	{
		Particle_S2D_CHM_Mix &pcl = model.pcls[pcl_id];
		if (pcl.is_in_mesh)
		{
			IntegrationVar_S2D_CHM_Mix &pcl_var = pcl.var;
			Node_S2D_CHM &n1 = *pcl_var.pn1;
			Node_S2D_CHM &n2 = *pcl_var.pn2;
			Node_S2D_CHM &n3 = *pcl_var.pn3;
			Node_S2D_CHM &n4 = *pcl_var.pn4;
			// particle acceleration
			pcl_ax_f = pcl_var.N1 * n1.ax_f + pcl_var.N2 * n2.ax_f
					 + pcl_var.N3 * n3.ax_f + pcl_var.N4 * n4.ax_f;
			pcl_ay_f = pcl_var.N1 * n1.ay_f + pcl_var.N2 * n2.ay_f
					 + pcl_var.N3 * n3.ay_f + pcl_var.N4 * n4.ay_f;
			// Interial term of fluid phase
			pcl_m_f = pcl.n * pcl.density_f * pcl.vol;
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

	// update nodal acceleration of solid phase
	for (size_t n_id = 0; n_id < model.node_num; ++n_id)
	{
		Node_S2D_CHM &n = model.nodes[n_id];
		if (n.m_s != 0.0)
		{
			n.ax_s = (n.fx_ext_m - n.fx_int_m - n.fx_kin_f) / n.m_s;
			n.ay_s = (n.fy_ext_m - n.fy_int_m - n.fy_kin_f) / n.m_s;
		}
	}
	// apply acceleration boundary conditions of solid phase
	for (size_t a_id = 0; a_id < model.asx_num; ++a_id)
		model.nodes[model.asxs[a_id].node_id].ax_s = model.asxs[a_id].a;
	for (size_t a_id = 0; a_id < model.asy_num; ++a_id)
		model.nodes[model.asys[a_id].node_id].ay_s = model.asys[a_id].a;
	// apply velocity boundary conditions of solid phase
	for (size_t v_id = 0; v_id < model.vsx_num; ++v_id)
		model.nodes[model.vsxs[v_id].node_id].ax_s = 0.0;
	for (size_t v_id = 0; v_id < model.vsy_num; ++v_id)
		model.nodes[model.vsys[v_id].node_id].ay_s = 0.0;

	// update nodal velocity of both phase
	for (size_t n_id = 0; n_id < model.node_num; ++n_id)
	{
		Node_S2D_CHM &n = model.nodes[n_id];
		if (n.m_s != 0.0)
		{
			n.vx_f /= n.m_tf;
			n.vx_f += n.ax_f * self.dt;
			n.vy_f /= n.m_tf;
			n.vy_f += n.ay_f * self.dt;
			n.vx_s /= n.m_s;
			n.vx_s += n.ax_s * self.dt;
			n.vy_s /= n.m_s;
			n.vy_s += n.ay_s * self.dt;
		}
	}
	// apply velocity boundary conditions of both phases
	for (size_t v_id = 0; v_id < model.vfx_num; ++v_id)
		model.nodes[model.vfxs[v_id].node_id].vx_f = model.vfxs[v_id].v;
	for (size_t v_id = 0; v_id < model.vfy_num; ++v_id)
		model.nodes[model.vfys[v_id].node_id].vy_f = model.vfys[v_id].v;
	for (size_t v_id = 0; v_id < model.vsx_num; ++v_id)
		model.nodes[model.vsxs[v_id].node_id].vx_s = model.vsxs[v_id].v;
	for (size_t v_id = 0; v_id < model.vsy_num; ++v_id)
		model.nodes[model.vsys[v_id].node_id].vy_s = model.vsys[v_id].v;
	
	// map variables back to and update variables particles
	double de11, de22, de12, dw12, de_vol_s, de_vol_f;
	double E_tmp, ds11, ds22, ds12;
	for (size_t pcl_id = 0; pcl_id < model.pcl_num; ++pcl_id)
	{
		Particle_S2D_CHM_Mix &pcl = model.pcls[pcl_id];
		if (pcl.is_in_mesh)
		{
			IntegrationVar_S2D_CHM_Mix &pcl_var = pcl.var;
			Node_S2D_CHM &n1 = *pcl_var.pn1;
			Node_S2D_CHM &n2 = *pcl_var.pn2;
			Node_S2D_CHM &n3 = *pcl_var.pn3;
			Node_S2D_CHM &n4 = *pcl_var.pn4;

			pcl.vx_s += (n1.ax_s * pcl_var.N1 + n2.ax_s * pcl_var.N2
					   + n3.ax_s * pcl_var.N3 + n4.ax_s * pcl_var.N4) * self.dt;
			pcl.vy_s += (n1.ay_s * pcl_var.N1 + n2.ay_s * pcl_var.N2
					   + n3.ay_s * pcl_var.N3 + n4.ay_s * pcl_var.N4) * self.dt;
			pcl.vx_f += (n1.ax_f * pcl_var.N1 + n2.ax_f * pcl_var.N2
					   + n3.ax_f * pcl_var.N3 + n4.ax_f * pcl_var.N4) * self.dt;
			pcl.vy_f += (n1.ay_f * pcl_var.N1 + n2.ay_f * pcl_var.N2
					   + n3.ay_f * pcl_var.N3 + n4.ay_f * pcl_var.N4) * self.dt;

			pcl.ux_s += (n1.vx_s * pcl_var.N1 + n2.vx_s * pcl_var.N2
					   + n3.vx_s * pcl_var.N3 + n4.vx_s * pcl_var.N4) * self.dt;
			pcl.uy_s += (n1.vy_s * pcl_var.N1 + n2.vy_s * pcl_var.N2
					   + n3.vy_s * pcl_var.N3 + n4.vy_s * pcl_var.N4) * self.dt;
			pcl.ux_f += (n1.vx_f * pcl_var.N1 + n2.vx_f * pcl_var.N2
					   + n3.vx_f * pcl_var.N3 + n4.vx_f * pcl_var.N4) * self.dt;
			pcl.uy_f += (n1.vy_f * pcl_var.N1 + n2.vy_f * pcl_var.N2
					   + n3.vy_f * pcl_var.N3 + n4.vy_f * pcl_var.N4) * self.dt;
			pcl.x = pcl.x_ori + pcl.ux_s;
			pcl.y = pcl.y_ori + pcl.uy_s;

			de11 = (n1.vx_s * pcl_var.dN1_dx + n2.vx_s * pcl_var.dN2_dx
				  + n3.vx_s * pcl_var.dN3_dx + n4.vx_s * pcl_var.dN4_dx) * self.dt;
			de22 = (n1.vy_s * pcl_var.dN1_dy + n2.vy_s * pcl_var.dN2_dy
				  + n3.vy_s * pcl_var.dN3_dy + n4.vy_s * pcl_var.dN4_dy) * self.dt;
			de12 = (n1.vx_s * pcl_var.dN1_dy + n2.vx_s * pcl_var.dN2_dy
				  + n3.vx_s * pcl_var.dN3_dy + n4.vx_s * pcl_var.dN4_dy
				  + n1.vy_s * pcl_var.dN1_dx + n2.vy_s * pcl_var.dN2_dx
				  + n3.vy_s * pcl_var.dN3_dx + n4.vy_s * pcl_var.dN4_dx) * 0.5 * self.dt;
			dw12 = (n1.vx_s * pcl_var.dN1_dy + n2.vx_s * pcl_var.dN2_dy
				  + n3.vx_s * pcl_var.dN3_dy + n4.vx_s * pcl_var.dN4_dy
				  - n1.vy_s * pcl_var.dN1_dx - n2.vy_s * pcl_var.dN2_dx
				  - n3.vy_s * pcl_var.dN3_dx - n4.vy_s * pcl_var.dN4_dx) * 0.5 * self.dt;

			// update strain (also assume that strain increment is Jaumann rate)
			//pcl_var.de11 +=  dw12 * e12 * 2.0;
			//pcl_var.de22 += -dw12 * e12 * 2.0;
			//pcl_var.de12 +=  dw12 * (e22 - e11);
			pcl.e11 += de11;
			pcl.e22 += de22;
			pcl.e12 += de12;

			// update stress
			E_tmp = pcl.E / (1.0 + pcl.niu) / (1.0 - 2.0 * pcl.niu);
			ds11 = E_tmp * ((1.0 - pcl.niu) * de11 + pcl.niu * de22);
			ds22 = E_tmp * (pcl.niu * de11 + (1.0 - pcl.niu) * de22);
			ds12 = 2.0 * de12 * pcl.E / (2.0 * (1.0 + pcl.niu));

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
			de_vol_s = de11 + de22;
			// "volumetric strain" of fluid phase
			de_vol_f = -(1.0 - pcl.n) / pcl.n * de_vol_s
					   -(n1.vx_f * pcl_var.dN1_dx + n2.vx_f * pcl_var.dN2_dx
					   + n3.vx_f * pcl_var.dN3_dx + n4.vx_f * pcl_var.dN4_dx
					   + n1.vy_f * pcl_var.dN1_dy + n2.vy_f * pcl_var.dN2_dy
					   + n3.vy_f * pcl_var.dN3_dy + n4.vy_f * pcl_var.dN4_dy) * self.dt;
			// pore pressure
			pcl.p += pcl.Kf * de_vol_f;
			// fluid density
			pcl.density_f += pcl.density_f * de_vol_f;

			// porosity
			pcl.n = (de_vol_s + pcl.n) / (1.0 + de_vol_s);
		}
	}

	return 0;
}