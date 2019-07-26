#include "SimulationCore_pcp.h"

#include <cmath>
#include "Step_S2D_ME_MPM_s_GIMP.h"

Step_S2D_ME_MPM_s_GIMP::Step_S2D_ME_MPM_s_GIMP() :
	Step(&solve_substep_S2D_ME_MPM_s_GIMP),
	h_elem_raio(0.05), h_pcl_ratio(0.1),
	model(nullptr) {}

Step_S2D_ME_MPM_s_GIMP::~Step_S2D_ME_MPM_s_GIMP() {}

int Step_S2D_ME_MPM_s_GIMP::init()
{
	if (is_first_step)
	{
		for (size_t i = 0; i < model->pcl_num; ++i)
		{
			model->pcls[i].elem_num = 1;
		}
	}

	for (size_t i = 0; i < model->pcl_num; i++)
	{
		Particle_S2D_ME &pcl = model->pcls[i];
		pcl.x_ori = pcl.x;
		pcl.y_ori = pcl.y;
	}
	return 0;
}

int Step_S2D_ME_MPM_s_GIMP::finalize() { return 0; }

inline void max(double &a, double b) noexcept { if (a < b) a = b; }

int solve_substep_S2D_ME_MPM_s_GIMP(void *_self)
{
	Step_S2D_ME_MPM_s_GIMP &self = *((Step_S2D_ME_MPM_s_GIMP *)_self);
	Model_S2D_ME_MPM_s &model = *(self.model);

	model.pcl_var_mem.reset_optimize();

	// init nodes
	for (size_t n_id = 0; n_id < model.node_num; ++n_id)
	{
		Node_S2D_ME &n = model.nodes[n_id];
		n.m = 0.0;
		n.ax = 0.0;
		n.ay = 0.0;
		n.vx = 0.0;
		n.vy = 0.0;
		n.fx_ext_m = 0.0;
		n.fy_ext_m = 0.0;
		n.fx_int_m = 0.0;
		n.fy_int_m = 0.0;
	}

	// map variables to nodes and cal internal force
	for (size_t pcl_id = 0; pcl_id < model.pcl_num; ++pcl_id)
	{
		Particle_S2D_ME &pcl = model.pcls[pcl_id];
		if (pcl.elem_num) // is in mesh
		{
			// cal element overlap
			model.get_elements_overlapped_by_particle(pcl);
			for (size_t i = 0; i < pcl.elem_num; ++i)
			{
				ParticleVar_S2D_ME &pcl_var = pcl.vars[i];

				// Map to nodes
				double m = pcl_var.vol * pcl.density;
				double mvx = m * pcl.vx;
				double mvy = m * pcl.vy;
				// node 1
				Node_S2D_ME &n1 = *pcl_var.pn1;
				n1.m  += pcl_var.N1 * m;
				n1.vx += pcl_var.N1 * mvx;
				n1.vy += pcl_var.N1 * mvy;
				n1.fx_int_m += (pcl_var.dN1_dx * pcl.s11 + pcl_var.dN1_dy * pcl.s12) * pcl_var.vol;
				n1.fy_int_m += (pcl_var.dN1_dx * pcl.s12 + pcl_var.dN1_dy * pcl.s22) * pcl_var.vol;
				// node 2
				Node_S2D_ME &n2 = *pcl_var.pn2;
				n2.m  += pcl_var.N2 * m;
				n2.vx += pcl_var.N2 * mvx;
				n2.vy += pcl_var.N2 * mvy;
				n2.fx_int_m += (pcl_var.dN2_dx * pcl.s11 + pcl_var.dN2_dy * pcl.s12) * pcl_var.vol;
				n2.fy_int_m += (pcl_var.dN2_dx * pcl.s12 + pcl_var.dN2_dy * pcl.s22) * pcl_var.vol;
				// node 3
				Node_S2D_ME &n3 = *pcl_var.pn3;
				n3.m  += pcl_var.N3 * m;
				n3.vx += pcl_var.N3 * mvx;
				n3.vy += pcl_var.N3 * mvy;
				n3.fx_int_m += (pcl_var.dN3_dx * pcl.s11 + pcl_var.dN3_dy * pcl.s12) * pcl_var.vol;
				n3.fy_int_m += (pcl_var.dN3_dx * pcl.s12 + pcl_var.dN3_dy * pcl.s22) * pcl_var.vol;
				// node 4
				Node_S2D_ME &n4 = *pcl_var.pn4;
				n4.m  += pcl_var.N4 * m;
				n4.vx += pcl_var.N4 * mvx;
				n4.vy += pcl_var.N4 * mvy;
				n4.fx_int_m += (pcl_var.dN4_dx * pcl.s11 + pcl_var.dN4_dy * pcl.s12) * pcl_var.vol;
				n4.fy_int_m += (pcl_var.dN4_dx * pcl.s12 + pcl_var.dN4_dy * pcl.s22) * pcl_var.vol;
			}
		}
	}

	// body force
	for (size_t bf_id = 0; bf_id < model.bfx_num; ++bf_id)
	{
		Particle_S2D_ME &pcl = model.pcls[model.bfxs[bf_id].pcl_id];
		for (size_t i = 0; i < pcl.elem_num; ++i)
		{
			ParticleVar_S2D_ME &pcl_var = pcl.vars[i];
			double bf_tmp = pcl_var.vol * pcl.density * model.bfxs[bf_id].bf;
			pcl_var.pn1->fx_ext_m += pcl_var.N1 * bf_tmp;
			pcl_var.pn2->fx_ext_m += pcl_var.N2 * bf_tmp;
			pcl_var.pn3->fx_ext_m += pcl_var.N3 * bf_tmp;
			pcl_var.pn4->fx_ext_m += pcl_var.N4 * bf_tmp;
		}
	}
	for (size_t bf_id = 0; bf_id < model.bfy_num; ++bf_id)
	{
		Particle_S2D_ME &pcl = model.pcls[model.bfys[bf_id].pcl_id];
		for (size_t i = 0; i < pcl.elem_num; ++i)
		{
			ParticleVar_S2D_ME &pcl_var = pcl.vars[i];
			double bf_tmp = pcl_var.vol * pcl.density * model.bfys[bf_id].bf;
			pcl_var.pn1->fy_ext_m += pcl_var.N1 * bf_tmp;
			pcl_var.pn2->fy_ext_m += pcl_var.N2 * bf_tmp;
			pcl_var.pn3->fy_ext_m += pcl_var.N3 * bf_tmp;
			pcl_var.pn4->fy_ext_m += pcl_var.N4 * bf_tmp;
		}
	}

	// surface force
	for (size_t tf_id = 0; tf_id < model.tx_num; ++tf_id)
	{
		ParticleVar_S2D_ME &pcl_var = model.pcls[model.txs[tf_id].pcl_id].var;
		double tf_tmp = model.txs[tf_id].t;
		pcl_var.pn1->fx_ext_m += pcl_var.N1 * tf_tmp;
		pcl_var.pn2->fx_ext_m += pcl_var.N2 * tf_tmp;
		pcl_var.pn3->fx_ext_m += pcl_var.N3 * tf_tmp;
		pcl_var.pn4->fx_ext_m += pcl_var.N4 * tf_tmp;
	}
	for (size_t tf_id = 0; tf_id < model.ty_num; ++tf_id)
	{
		ParticleVar_S2D_ME &pcl_var = model.pcls[model.tys[tf_id].pcl_id].var;
		double tf_tmp = model.tys[tf_id].t;
		pcl_var.pn1->fy_ext_m += pcl_var.N1 * tf_tmp;
		pcl_var.pn2->fy_ext_m += pcl_var.N2 * tf_tmp;
		pcl_var.pn3->fy_ext_m += pcl_var.N3 * tf_tmp;
		pcl_var.pn4->fy_ext_m += pcl_var.N4 * tf_tmp;
	}

	// update nodal acceleration
	for (size_t n_id = 0; n_id < model.node_num; ++n_id)
	{
		Node_S2D_ME &n = model.nodes[n_id];
		if (n.m != 0.0)
		{
			n.ax = (n.fx_ext_m - n.fx_int_m) / n.m;
			n.ay = (n.fy_ext_m - n.fy_int_m) / n.m;
			//std::cout << "g " << n.fy_ext_m << " " << n.fy_int_m << " " << n.ay << "\n";
		}
	}
	// acceleration boundary conditions
	for (size_t ax_id = 0; ax_id < model.ax_num; ++ax_id)
		model.nodes[model.axs[ax_id].node_id].ax = model.axs[ax_id].a;
	for (size_t ay_id = 0; ay_id < model.ay_num; ++ay_id)
		model.nodes[model.ays[ay_id].node_id].ax = model.ays[ay_id].a;

	self.dt = self.max_dt;
	// adjust time step
	// particle shouldn't travel more than 1/10 element each steps
	double alpha_h = model.h * self.h_elem_raio;
	double v_max = 0.0;
	for (size_t n_id = 0; n_id < model.node_num; ++n_id)
	{
		Node_S2D_ME &n = model.nodes[n_id];
		max(v_max, (abs(n.vx) + sqrt(n.vx*n.vx + 4.0*abs(n.ax)*alpha_h)) / 2.0);
		max(v_max, (abs(n.vy) + sqrt(n.vy*n.vy + 4.0*abs(n.ay)*alpha_h)) / 2.0);
	}
	for (size_t vx_id = 0; vx_id < model.vx_num; ++vx_id)
		max(v_max, model.vxs[vx_id].v);
	for (size_t vy_id = 0; vy_id < model.vy_num; ++vy_id)
		max(v_max, model.vys[vy_id].v);
	if (v_max != 0.0)
	{
		double dt_tmp = alpha_h / v_max;
		//min_dt < dt_tmp < max_dt
		if (dt_tmp < self.dt)
		{
			self.dt = dt_tmp;
			//std::cout << "substep id: " << self.substep_num << " new dt: " << self.dt << "\n";
		}
	}

	// update nodal velocity
	for (size_t n_id = 0; n_id < model.node_num; ++n_id)
	{
		Node_S2D_ME &n = model.nodes[n_id];
		//std::cout << "g " << n.vy;
		if (n.m != 0.0)
		{
			n.vx /= n.m;
			n.vx += n.ax * self.dt;
			n.vy /= n.m;
			n.vy += n.ay * self.dt;
		}
		//std::cout << " " << n.vy << "\n";
	}
	// velocity boundary conditions
	for (size_t vx_id = 0; vx_id < model.vx_num; ++vx_id)
	{
		Node_S2D_ME &n = model.nodes[model.vxs[vx_id].node_id];
		n.vx = model.vxs[vx_id].v;
		n.ax = 0.0;
	}
	for (size_t vy_id = 0; vy_id < model.vy_num; ++vy_id)
	{
		Node_S2D_ME &n = model.nodes[model.vys[vy_id].node_id];
		n.vy = model.vys[vy_id].v;
		n.ay = 0.0;
	}

	for (size_t n_id = 0; n_id < model.node_num; ++n_id)
	{
		Node_S2D_ME &n = model.nodes[n_id];
		//std::cout << "g m: " << n.m << " " << n.vy << "\n";
	}

	// map variables back to particles
	double de11, de12, de22, dw12, de_vol;
	for (size_t pcl_id = 0; pcl_id < model.pcl_num; ++pcl_id)
	{
		Particle_S2D_ME &pcl = model.pcls[pcl_id];
		if (pcl.elem_num)
		{
			ParticleVar_S2D_ME &pcl_var = pcl.var;
			Node_S2D_ME &n1 = *pcl_var.pn1;
			Node_S2D_ME &n2 = *pcl_var.pn2;
			Node_S2D_ME &n3 = *pcl_var.pn3;
			Node_S2D_ME &n4 = *pcl_var.pn4;
			
			pcl.vx += (pcl_var.N1 * n1.ax + pcl_var.N2 * n2.ax + pcl_var.N3 * n3.ax + pcl_var.N4 * n4.ax) * self.dt;
			pcl.vy += (pcl_var.N1 * n1.ay + pcl_var.N2 * n2.ay + pcl_var.N3 * n3.ay + pcl_var.N4 * n4.ay) * self.dt;
			//std::cout << "g v: " << pcl.vx << " " << pcl.vy << "\n";

			pcl.ux += (pcl_var.N1 * n1.vx + pcl_var.N2 * n2.vx + pcl_var.N3 * n3.vx + pcl_var.N4 * n4.vx) * self.dt;
			pcl.uy += (pcl_var.N1 * n1.vy + pcl_var.N2 * n2.vy + pcl_var.N3 * n3.vy + pcl_var.N4 * n4.vy) * self.dt;
			pcl.x = pcl.x_ori + pcl.ux;
			pcl.y = pcl.y_ori + pcl.uy;
			//std::cout << "g u: " << pcl.ux << " " << pcl.uy << "\n";

			de11 = (pcl_var.dN1_dx * n1.vx + pcl_var.dN2_dx * n2.vx
				  + pcl_var.dN3_dx * n3.vx + pcl_var.dN4_dx * n4.vx) * self.dt;
			de22 = (pcl_var.dN1_dy * n1.vy + pcl_var.dN2_dy * n2.vy
				  + pcl_var.dN3_dy * n3.vy + pcl_var.dN4_dy * n4.vy) * self.dt;
			de12 = (pcl_var.dN1_dy * n1.vx + pcl_var.dN2_dy * n2.vx
				  + pcl_var.dN3_dy * n3.vx + pcl_var.dN4_dy * n4.vx
				  + pcl_var.dN1_dx * n1.vy + pcl_var.dN2_dx * n2.vy
				  + pcl_var.dN3_dx * n3.vy + pcl_var.dN4_dx * n4.vy) * 0.5 * self.dt;
			dw12 = (pcl_var.dN1_dy * n1.vx + pcl_var.dN2_dy * n2.vx
				  + pcl_var.dN3_dy * n3.vx + pcl_var.dN4_dy * n4.vx
				  - pcl_var.dN1_dx * n1.vy - pcl_var.dN2_dx * n2.vy
				  - pcl_var.dN3_dx * n3.vy - pcl_var.dN4_dx * n4.vy) * 0.5 * self.dt;
			
			// update strain (also assume that strain increment is Jaumann rate)
			//ppcl->de11 +=  ppcl->dw12 * ppcl->e12 * 2.0;
			//ppcl->de22 += -ppcl->dw12 * ppcl->e12 * 2.0;
			//ppcl->de12 +=  ppcl->dw12 * (ppcl->e22 - ppcl->e11);
			pcl.e11 += de11;
			pcl.e22 += de22;
			pcl.e12 += de12;

			// update stress
			double E_tmp = pcl.E / (1.0 + pcl.niu) / (1.0 - 2.0 * pcl.niu);
			double ds11, ds12, ds22;
			ds11 = E_tmp * ((1.0 - pcl.niu) * de11 + pcl.niu * de22);
			ds12 = 2.0 * pcl.E / (2.0 * (1.0 + pcl.niu)) * de12;
			ds22 = E_tmp * (pcl.niu * de11 + (1.0 - pcl.niu) * de22);
			//std::cout << "g " << ds11 << " " << ds12 << " " << ds22 << "\n";

			/* ------------------------------------------------------------------
			Rotate as Jaumann rate:
				tensor_rate = tensor_Jaumann_rate + tensor * dW_T + dW * tensor
			  ------------------------------------------------------------------- */
			/*	ds11 +=  ppcl->dw12 * ppcl->s12 * 2.0;
				ds22 += -ppcl->dw12 * ppcl->s12 * 2.0;
				ds12 +=  ppcl->dw12 * (ppcl->s22 - ppcl->s11);	*/
			pcl.s11 += ds11;
			pcl.s22 += ds22;
			pcl.s12 += ds12;

			de_vol = de11 + de22;
			pcl.density /= (1.0 + de_vol);
			//std::cout << pcl.density << "\n";
		}
	}

	return 0;
}