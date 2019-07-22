#include "SimulationCore_pcp.h"

#include <cmath>
#include "Step_S2D_ME_MPM_s_GIMP.h"

Step_S2D_ME_MPM_s_GIMP::Step_S2D_ME_MPM_s_GIMP() :
	Step(&solve_substep_S2D_ME_MPM_s_GIMP),
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
		pcl.ux = 0.0;
		pcl.uy = 0.0;
	}
	return 0;
}

int Step_S2D_ME_MPM_s_GIMP::finalize() { return 0; }

#define N_LOW(xi)  (1.0 - (xi)) / 2.0
#define N_HIGH(xi) (1.0 + (xi)) / 2.0
#define dN_dxi_LOW(xi) -0.5
#define dN_dxi_HIGH(xi) 0.5

int solve_substep_S2D_ME_MPM_s_GIMP(void *_self)
{
	Step_S2D_ME_MPM_s_GIMP &self = *((Step_S2D_ME_MPM_s_GIMP *)_self);
	Model_S2D_ME_MPM_s &model = *(self.model);

	// init nodes
	for (size_t i = 0; i < model.node_num; i++)
	{
		Node_S2D_ME &n = model.nodes[i];
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
				pcl_var.pn1 = model.nodes + model.node_x_num * pcl_var.elem_y_id + pcl_var.elem_x_id;
				pcl_var.pn2 = pcl_var.pn1 + 1;
				pcl_var.pn3 = pcl_var.pn2 + model.node_x_num;
				pcl_var.pn4 = pcl_var.pn3 - 1;

				// Cal shape function
				double xi  = 2.0 * ((pcl_var.x - model.x0) / model.h - double(pcl_var.elem_x_id)) - 1.0;
				double Nx_low  = N_LOW(xi);
				double Nx_high = N_HIGH(xi);
				double dNx_dxi_low  = dN_dxi_LOW(xi);
				double dNx_dxi_high = dN_dxi_HIGH(xi);
				double eta = 2.0 * ((pcl_var.y - model.y0) / model.h - double(pcl_var.elem_y_id)) - 1.0;
				double Ny_low  = N_LOW(eta);
				double Ny_high = N_HIGH(eta);
				double dNy_deta_low  = dN_dxi_LOW(eta);
				double dNy_deta_high = dN_dxi_HIGH(eta);
				pcl_var.N1 = Nx_low  * Ny_low;
				pcl_var.N2 = Nx_high * Ny_low;
				pcl_var.N3 = Nx_high * Ny_high;
				pcl_var.N4 = Nx_low  * Ny_high;
				double dxi_dx = 2.0 / model.h; // = deta_dy
				pcl_var.dN1_dx = dNx_dxi_low  * Ny_low  * dxi_dx;
				pcl_var.dN2_dx = dNx_dxi_high * Ny_low  * dxi_dx;
				pcl_var.dN3_dx = dNx_dxi_high * Ny_high * dxi_dx;
				pcl_var.dN4_dx = dNx_dxi_low  * Ny_high * dxi_dx;
				pcl_var.dN1_dy = Nx_low  * dNy_deta_low  * dxi_dx;
				pcl_var.dN2_dy = Nx_high * dNy_deta_low  * dxi_dx;
				pcl_var.dN3_dy = Nx_high * dNy_deta_high * dxi_dx;
				pcl_var.dN4_dy = Nx_low  * dNy_deta_high * dxi_dx;

				// Map to nodes
				double m_tmp = pcl_var.vol * pcl.density;
				// node 1
				Node_S2D_ME &n1 = *pcl_var.pn1;
				n1.m  += pcl_var.N1 * m_tmp;
				n1.vx += pcl_var.N1 * m_tmp * pcl.vx;
				n1.vy += pcl_var.N1 * m_tmp * pcl.vy;
				n1.fx_int_m += (pcl_var.dN1_dx * pcl.s11 + pcl_var.dN1_dy * pcl.s12) * pcl_var.vol;
				n1.fy_int_m += (pcl_var.dN1_dx * pcl.s12 + pcl_var.dN1_dy * pcl.s22) * pcl_var.vol;
				// node 2
				Node_S2D_ME &n2 = *pcl_var.pn2;
				n2.m  += pcl_var.N2 * m_tmp;
				n2.vx += pcl_var.N2 * m_tmp * pcl.vx;
				n2.vy += pcl_var.N2 * m_tmp * pcl.vy;
				n2.fx_int_m += (pcl_var.dN2_dx * pcl.s11 + pcl_var.dN2_dy * pcl.s12) * pcl_var.vol;
				n2.fy_int_m += (pcl_var.dN2_dx * pcl.s12 + pcl_var.dN2_dy * pcl.s22) * pcl_var.vol;
				// node 3
				Node_S2D_ME &n3 = *pcl_var.pn3;
				n3.m  += pcl_var.N3 * m_tmp;
				n3.vx += pcl_var.N3 * m_tmp * pcl.vx;
				n3.vy += pcl_var.N3 * m_tmp * pcl.vy;
				n3.fx_int_m += (pcl_var.dN3_dx * pcl.s11 + pcl_var.dN3_dy * pcl.s12) * pcl_var.vol;
				n3.fy_int_m += (pcl_var.dN3_dx * pcl.s12 + pcl_var.dN3_dy * pcl.s22) * pcl_var.vol;
				// node 4
				Node_S2D_ME &n4 = *pcl_var.pn4;
				n4.m  += pcl_var.N4 * m_tmp;
				n4.vx += pcl_var.N4 * m_tmp * pcl.vx;
				n4.vy += pcl_var.N4 * m_tmp * pcl.vy;
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
			pcl_var.pn1->fx_ext_m += bf_tmp * pcl_var.N1;
			pcl_var.pn2->fx_ext_m += bf_tmp * pcl_var.N2;
			pcl_var.pn3->fx_ext_m += bf_tmp * pcl_var.N3;
			pcl_var.pn4->fx_ext_m += bf_tmp * pcl_var.N4;
		}
	}
	for (size_t bf_id = 0; bf_id < model.bfy_num; ++bf_id)
	{
		Particle_S2D_ME &pcl = model.pcls[model.bfys[bf_id].pcl_id];
		for (size_t i = 0; i < pcl.elem_num; ++i)
		{
			ParticleVar_S2D_ME &pcl_var = pcl.vars[i];
			double bf_tmp = pcl_var.vol * pcl.density * model.bfys[bf_id].bf;
			pcl_var.pn1->fy_ext_m += bf_tmp * pcl_var.N1;
			pcl_var.pn2->fy_ext_m += bf_tmp * pcl_var.N2;
			pcl_var.pn3->fy_ext_m += bf_tmp * pcl_var.N3;
			pcl_var.pn4->fy_ext_m += bf_tmp * pcl_var.N4;
		}
	}
	// surface force
	for (size_t tf_id = 0; tf_id < model.tx_num; ++tf_id)
	{
		Particle_S2D_ME &pcl = model.pcls[model.txs[tf_id].pcl_id];
		for (size_t i = 0; i < pcl.elem_num; ++i)
		{
			ParticleVar_S2D_ME &pcl_var = pcl.vars[i];
			double tf_tmp = model.txs[tf_id].t;
			pcl_var.pn1->fx_ext_m += tf_tmp * pcl_var.N1;
			pcl_var.pn2->fx_ext_m += tf_tmp * pcl_var.N2;
			pcl_var.pn3->fx_ext_m += tf_tmp * pcl_var.N3;
			pcl_var.pn4->fx_ext_m += tf_tmp * pcl_var.N4;
		}
	}
	for (size_t tf_id = 0; tf_id < model.ty_num; ++tf_id)
	{
		Particle_S2D_ME &pcl = model.pcls[model.tys[tf_id].pcl_id];
		for (size_t i = 0; i < pcl.elem_num; ++i)
		{
			ParticleVar_S2D_ME &pcl_var = pcl.vars[i];
			double tf_tmp = model.tys[tf_id].t;
			pcl_var.pn1->fy_ext_m += tf_tmp * pcl_var.N1;
			pcl_var.pn2->fy_ext_m += tf_tmp * pcl_var.N2;
			pcl_var.pn3->fy_ext_m += tf_tmp * pcl_var.N3;
			pcl_var.pn4->fy_ext_m += tf_tmp * pcl_var.N4;
		}
	}

	// update nodal acceleration
	for (size_t n_id = 0; n_id < model.node_num; ++n_id)
	{
		Node_S2D_ME &n = model.nodes[n_id];
		if (n.m != 0.0)
		{
			n.ax = (n.fx_ext_m - n.fx_int_m) / n.m;
			n.ay = (n.fy_ext_m - n.fy_int_m) / n.m;
		}
	}
	// acceleration boundary conditions
	for (size_t ax_id = 0; ax_id < model.ax_num; ++ax_id)
		model.nodes[model.axs[ax_id].node_id].ax = model.axs[ax_id].a;
	for (size_t ay_id = 0; ay_id < model.ay_num; ++ay_id)
		model.nodes[model.ays[ay_id].node_id].ax = model.ays[ay_id].a;

	// update nodal velocity
	for (size_t n_id = 0; n_id < model.node_num; ++n_id)
	{
		Node_S2D_ME &n = model.nodes[n_id];
		if (n.m != 0.0)
		{
			n.vx /= n.m;
			n.vx += n.ax;
			n.vy /= n.m;
			n.vy += n.ay;
		}
	}
	// velocity boundary conditions
	for (size_t vx_id = 0; vx_id < model.vx_num; ++vx_id)
		model.nodes[model.vxs[vx_id].node_id].vx = model.vxs[vx_id].v;
	for (size_t vy_id = 0; vy_id < model.vy_num; ++vy_id)
		model.nodes[model.vys[vy_id].node_id].vy = model.vys[vy_id].v;

	// map variables back to particles
	for (size_t pcl_id = 0; pcl_id < model.pcl_num; ++pcl_id)
	{
		Particle_S2D_ME &pcl = model.pcls[pcl_id];
		for (size_t i = 0; i < pcl.elem_num; ++i)
		{
			ParticleVar_S2D_ME &pcl_var = pcl.vars[i];
			pcl_var.dvx = pcl_var.N1 * pcl_var.pn1->ax + pcl_var.N2 * pcl_var.pn2->ax
						+ pcl_var.N3 * pcl_var.pn3->ax + pcl_var.N4 * pcl_var.pn4->ax;
			pcl_var.dvx *= self.dt;
			pcl_var.dvy = pcl_var.N1 * pcl_var.pn1->ay + pcl_var.N2 * pcl_var.pn2->ay
						+ pcl_var.N3 * pcl_var.pn3->ay + pcl_var.N4 * pcl_var.pn4->ay;
			pcl_var.dvy *= self.dt;
			pcl_var.dux = pcl_var.N1 * pcl_var.pn1->vx + pcl_var.N2 * pcl_var.pn2->vx
						+ pcl_var.N3 * pcl_var.pn3->vx + pcl_var.N4 * pcl_var.pn4->vx;
			pcl_var.dux *= self.dt;
			pcl_var.duy = pcl_var.N1 * pcl_var.pn1->vy + pcl_var.N2 * pcl_var.pn2->vy
						+ pcl_var.N3 * pcl_var.pn3->vy + pcl_var.N4 * pcl_var.pn4->vy;
			pcl_var.duy *= self.dt;
			pcl_var.de11 = pcl_var.dN1_dx * pcl_var.pn1->vx + pcl_var.dN2_dx * pcl_var.pn2->vx
						 + pcl_var.dN3_dx * pcl_var.pn3->vx + pcl_var.dN4_dx * pcl_var.pn4->vx;
			pcl_var.de11 *= self.dt;
			pcl_var.de22 = pcl_var.dN1_dy * pcl_var.pn1->vy + pcl_var.dN2_dy * pcl_var.pn2->vy
						 + pcl_var.dN3_dy * pcl_var.pn3->vy + pcl_var.dN4_dy * pcl_var.pn4->vy;
			pcl_var.de22 *= self.dt;
			pcl_var.de12 = pcl_var.dN1_dy * pcl_var.pn1->vx + pcl_var.dN2_dy * pcl_var.pn2->vx
						 + pcl_var.dN3_dy * pcl_var.pn3->vx + pcl_var.dN4_dy * pcl_var.pn4->vx
						 + pcl_var.dN1_dx * pcl_var.pn1->vy + pcl_var.dN2_dx * pcl_var.pn2->vy
						 + pcl_var.dN3_dx * pcl_var.pn3->vy + pcl_var.dN4_dx * pcl_var.pn4->vy;
			pcl_var.de12 *= 0.5 * self.dt;
			pcl_var.dw12 = pcl_var.dN1_dy * pcl_var.pn1->vx + pcl_var.dN2_dy * pcl_var.pn2->vx
						 + pcl_var.dN3_dy * pcl_var.pn3->vx + pcl_var.dN4_dy * pcl_var.pn4->vx
						 - pcl_var.dN1_dx * pcl_var.pn1->vy - pcl_var.dN2_dx * pcl_var.pn2->vy
						 - pcl_var.dN3_dx * pcl_var.pn3->vy - pcl_var.dN4_dx * pcl_var.pn4->vy;
			pcl_var.dw12 *= 0.5 * self.dt;
		}

		if (pcl.elem_num > 1)
		{
			ParticleVar_S2D_ME &pcl_avg_var = pcl.var;
			pcl_avg_var.vol = 0.0;
			pcl_avg_var.dvx = 0.0;
			pcl_avg_var.dvy = 0.0;
			pcl_avg_var.dux = 0.0;
			pcl_avg_var.duy = 0.0;
			pcl_avg_var.de11 = 0.0;
			pcl_avg_var.de22 = 0.0;
			pcl_avg_var.de12 = 0.0;
			pcl_avg_var.dw12 = 0.0;
			for (size_t i = 0; i < pcl.elem_num; ++i)
			{
				ParticleVar_S2D_ME &pcl_var = pcl.vars[i];
				pcl_avg_var.vol += pcl_var.vol;
				pcl_avg_var.dvx += pcl_var.dvx * pcl_var.vol;
				pcl_avg_var.dvy += pcl_var.dvy * pcl_var.vol;
				pcl_avg_var.dux += pcl_var.dux * pcl_var.vol;
				pcl_avg_var.duy += pcl_var.duy * pcl_var.vol;
				pcl_avg_var.de11 += pcl_var.de11 * pcl_var.vol;
				pcl_avg_var.de22 += pcl_var.de22 * pcl_var.vol;
				pcl_avg_var.de12 += pcl_var.de12 * pcl_var.vol;
				pcl_avg_var.dw12 += pcl_var.dw12 * pcl_var.vol;
			}
			pcl_avg_var.dvx /= pcl_avg_var.vol;
			pcl_avg_var.dvy /= pcl_avg_var.vol;
			pcl_avg_var.dux /= pcl_avg_var.vol;
			pcl_avg_var.duy /= pcl_avg_var.vol;
			pcl_avg_var.de11 /= pcl_avg_var.vol;
			pcl_avg_var.de22 /= pcl_avg_var.vol;
			pcl_avg_var.de12 /= pcl_avg_var.vol;
			pcl_avg_var.dw12 /= pcl_avg_var.vol;
		}
	}
	
	// update particle variables
	for (size_t pcl_id = 0; pcl_id < model.pcl_num; ++pcl_id)
	{
		Particle_S2D_ME &pcl = model.pcls[pcl_id];
		if (pcl.elem_num)
		{
			ParticleVar_S2D_ME &pcl_var = pcl.var;

			pcl.x  += pcl_var.dux;
			pcl.y  += pcl_var.duy;
			pcl.vx += pcl_var.dvx;
			pcl.vy += pcl_var.dvy;

			// update strain (also assume that strain increment is Jaumann rate)
			//ppcl->de11 +=  ppcl->dw12 * ppcl->e12 * 2.0;
			//ppcl->de22 += -ppcl->dw12 * ppcl->e12 * 2.0;
			//ppcl->de12 +=  ppcl->dw12 * (ppcl->e22 - ppcl->e11);
			pcl.e11 += pcl_var.de11;
			pcl.e22 += pcl_var.de22;
			pcl.e12 += pcl_var.de12;

			// update stress
			double E_tmp = pcl.E / (1.0 + pcl.niu) / (1.0 - 2.0 * pcl.niu);
			double ds11, ds12, ds22;
			ds11 = E_tmp * ((1.0 - pcl.niu) * pcl_var.de11 + pcl.niu * pcl_var.de22);
			ds12 = 2.0 * pcl.E / (2.0 * (1.0 + pcl.niu)) * pcl_var.de12;
			ds22 = E_tmp * (pcl.niu * pcl_var.de11 + (1.0 - pcl.niu) * pcl_var.de22);

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

			double de_vol = pcl_var.de11 + pcl_var.de22;
			pcl.density /= (1.0 + de_vol);
		}
	}

	return 0;
}