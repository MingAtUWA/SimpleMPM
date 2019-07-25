#ifndef _MODEL_S2D_ME_MPM_S_H_
#define _MODEL_S2D_ME_MPM_S_H_

#include "BC.h"
#include "Model.h"

#include "StackLikeBuffer.hpp"
#include "ItemArray.hpp"

struct Node_S2D_ME
{
	size_t index_x, index_y;
	double m;
	double ax, ay;
	double vx, vy;
	double fx_ext_m, fy_ext_m;
	double fx_int_m, fy_int_m;
};

struct ParticleVar_S2D_ME
{
	double x, y;
	double vol;
	size_t elem_x_id, elem_y_id;
	
	Node_S2D_ME *pn1, *pn2, *pn3, *pn4;
	double N1, N2, N3, N4;
	double dN1_dx, dN2_dx, dN3_dx, dN4_dx;
	double dN1_dy, dN2_dy, dN3_dy, dN4_dy;
};

struct Particle_S2D_ME
{
	size_t index;
	double x, y;
	double x_ori, y_ori;
	double ux, uy;

	double vx, vy;
	double m, density;

	double s11, s22, s12;
	double e11, e22, e12;
	double es11, es22, es12;
	double ps11, ps22, ps12;

	// The number of elements that this particle covers
	// when elem_num == 0, the particle is out of mesh
	size_t elem_num; 
	union
	{
		struct
		{
			double x, y, vol;
			size_t elem_x_id, elem_y_id;
		};
		ParticleVar_S2D_ME var;
	};
	ParticleVar_S2D_ME *vars;

	// constitutive model
	double E, niu;
};

struct Model_S2D_ME_MPM_s : public Model
{
public:
	double h;
	double x0, xn, y0, yn;
	size_t node_x_num, node_y_num, node_num;
	Node_S2D_ME *nodes;

	size_t pcl_num;
	Particle_S2D_ME *pcls;

	size_t bfx_num, bfy_num;
	BodyForce *bfxs, *bfys;
	size_t tx_num, ty_num;
	TractionBC_MPM *txs, *tys;
	size_t ax_num, ay_num;
	AccelerationBC *axs, *ays;
	size_t vx_num, vy_num;
	VelocityBC *vxs, *vys;

public:
	Model_S2D_ME_MPM_s();
	~Model_S2D_ME_MPM_s();

	void init_mesh(double grid_size, size_t elem_x_num, size_t elem_y_num,
				   double x_start = 0.0, double y_start = 0.0)
	{
		clear_mesh();
		h = grid_size;
		x0 = x_start;
		xn = x0 + h * double(elem_x_num);
		y0 = y_start;
		yn = y0 + h * double(elem_y_num);
		node_x_num = elem_x_num + 1;
		node_y_num = elem_y_num + 1;
		node_num = node_x_num * node_y_num;
		nodes = new Node_S2D_ME[node_num];
		size_t i, j, k;
		k = 0;
		for (i = 0; i < node_y_num; ++i)
			for (j = 0; j < node_x_num; ++j)
			{
				nodes[k].index_x = j;
				nodes[k].index_y = i;
				++k;
			}
	}
	void clear_mesh(void)
	{
		if (nodes) delete[] nodes;
		nodes = nullptr;
		node_x_num = 0;
		node_y_num = 0;
	}

	void init_pcl(size_t num, double m, double density, double E, double niu)
	{
		pcl_num = num;
		pcls = new Particle_S2D_ME[num];
		for (size_t i = 0; i < num; ++i)
		{
			Particle_S2D_ME &pcl = pcls[i];
			pcl.index = i;
			pcl.x = 0.0;
			pcl.y = 0.0;
			pcl.vx = 0.0;
			pcl.vy = 0.0;
			pcl.m = m;
			pcl.density = density;
			pcl.E = E;
			pcl.niu = niu;
			pcl.e11 = 0.0;
			pcl.e12 = 0.0;
			pcl.e22 = 0.0;
			pcl.s11 = 0.0;
			pcl.s12 = 0.0;
			pcl.s22 = 0.0;
		}
		// init buffer for particle vars
		pcl_var_mem.reset();
		pcl_var_mem.set_default_pool_size(pcl_num);
	}
	void clear_pcl(void)
	{
		if (pcls) delete[] pcls;
		pcls = nullptr;
		pcl_num = 0;
		pcl_var_mem.clear();
	}

protected:
	friend int solve_substep_S2D_ME_MPM_s_GIMP(void *_self);
	MemoryUtilities::StackLikeBuffer<ParticleVar_S2D_ME> pcl_var_mem;
	MemoryUtilities::ItemArray<double> x_len_buf;
	MemoryUtilities::ItemArray<double> y_len_buf;
	
#define N_LOW(xi)  (1.0 - (xi)) / 2.0
#define N_HIGH(xi) (1.0 + (xi)) / 2.0
#define dN_dxi_LOW(xi) -0.5
#define dN_dxi_HIGH(xi) 0.5
	inline void cal_shape_func(ParticleVar_S2D_ME &pcl_var)
	{
		pcl_var.pn1 = nodes + node_x_num * pcl_var.elem_y_id + pcl_var.elem_x_id;
		pcl_var.pn2 = pcl_var.pn1 + 1;
		pcl_var.pn3 = pcl_var.pn2 + node_x_num;
		pcl_var.pn4 = pcl_var.pn3 - 1;
		double xi  = 2.0 * ((pcl_var.x - x0) / h - double(pcl_var.elem_x_id)) - 1.0;
		double Nx_low  = N_LOW(xi);
		double Nx_high = N_HIGH(xi);
		double dNx_dxi_low  = dN_dxi_LOW(xi);
		double dNx_dxi_high = dN_dxi_HIGH(xi);
		double eta = 2.0 * ((pcl_var.y - y0) / h - double(pcl_var.elem_y_id)) - 1.0;
		double Ny_low  = N_LOW(eta);
		double Ny_high = N_HIGH(eta);
		double dNy_deta_low  = dN_dxi_LOW(eta);
		double dNy_deta_high = dN_dxi_HIGH(eta);
		pcl_var.N1 = Nx_low  * Ny_low;
		pcl_var.N2 = Nx_high * Ny_low;
		pcl_var.N3 = Nx_high * Ny_high;
		pcl_var.N4 = Nx_low  * Ny_high;
		double dxi_dx = 2.0 / h; // = deta_dy
		pcl_var.dN1_dx = dNx_dxi_low  * Ny_low   * dxi_dx;
		pcl_var.dN2_dx = dNx_dxi_high * Ny_low   * dxi_dx;
		pcl_var.dN3_dx = dNx_dxi_high * Ny_high  * dxi_dx;
		pcl_var.dN4_dx = dNx_dxi_low  * Ny_high  * dxi_dx;
		pcl_var.dN1_dy = Nx_low  * dNy_deta_low  * dxi_dx;
		pcl_var.dN2_dy = Nx_high * dNy_deta_low  * dxi_dx;
		pcl_var.dN3_dy = Nx_high * dNy_deta_high * dxi_dx;
		pcl_var.dN4_dy = Nx_low  * dNy_deta_high * dxi_dx;
	}
#undef N_LOW
#undef N_HIGH
#undef dN_dxi_LOW
#undef dN_dxi_HIGH

public:
	// if coordinate of the particle is outside the mesh,
	// the whole particle is ignored
	// support large particle (allow more stable calculation)
	void get_elements_overlapped_by_particle(Particle_S2D_ME &pcl);
};

#endif