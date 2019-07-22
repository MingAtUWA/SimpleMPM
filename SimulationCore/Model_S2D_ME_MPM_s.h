#ifndef _MODEL_S2D_ME_MPM_S_H_
#define _MODEL_S2D_ME_MPM_S_H_

#include "BC.h"
#include "Model.h"
#include "StackLikeBuffer.hpp"

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
	double dvx, dvy, dux, duy;
	double de11, de22, de12, dw12;
};

struct Particle_S2D_ME
{
	size_t index;
	double m, density;
	double vx, vy;
	union // position and calculation variables
	{
		struct { double x, y, vol; };
		ParticleVar_S2D_ME var;
	};

	double x_ori, y_ori;
	double ux, uy;

	double s11, s22, s12;
	double e11, e22, e12;
	double es11, es22, es12;
	double ps11, ps22, ps12;

	size_t elem_num; // The number of elements that this particle covers
	ParticleVar_S2D_ME *vars;

	// constitutive model
	double E, niu;
};

class Model_S2D_ME_MPM_s : public Model
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

	MemoryUtilities::StackLikeBuffer<ParticleVar_S2D_ME> pcl_var_mem;

public:
	Model_S2D_ME_MPM_s() :
		nodes(nullptr), node_x_num(0), node_y_num(0),
		pcl_var_mem(10) {}
	~Model_S2D_ME_MPM_s()
	{
		clear_mesh();
		clear_pcl();
		if (bfxs) delete[] bfxs;
		bfxs = nullptr;
		if (bfys) delete[] bfys;
		bfys = nullptr;
		if (txs) delete[] txs;
		txs = nullptr;
		if (tys) delete[] tys;
		tys = nullptr;
		if (axs) delete[] axs;
		axs = nullptr;
		if (ays) delete[] ays;
		ays = nullptr;
		if (vxs) delete[] vxs;
		vxs = nullptr;
		if (vys) delete[] vys;
		vys = nullptr;
		pcl_var_mem.clear();
	}

	void init_mesh(double grid_size, double x_start, double y_start,
				   size_t elem_x_num, size_t elem_y_num)
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
		nodes = new Node_S2D_ME[node_x_num * node_y_num];
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
			pcl_var_mem.set_default_pool_size(pcl_num/2);
		}
	}
	void clear_pcl(void)
	{
		if (pcls) delete[] pcls;
		pcls = nullptr;
		pcl_num = 0;
	}

	// Only suitable for case when particle is smaller than element
	// The calculation may not be good with particles larger than element
	void get_elements_overlapped_by_particle(Particle_S2D_ME &pcl)
	{
		pcl.vol = pcl.m / pcl.density;
		double hlen = sqrt(pcl.vol) * 0.5; // half length
		double xl = pcl.x - hlen;
		if (xl < x0) xl = x0; // xl = max(xl, x0)
		double xu = pcl.x + hlen;
		if (xu > xn) xu = xn; // xu = min(xu, xn)
		double yl = pcl.y - hlen;
		if (yl < y0) yl = y0; // yl = max(yl, y0)
		double yu = pcl.y + hlen;
		if (yu > yn) yu = yn;
		// particle out of mesh
		if (xl >= xu || yl >= yu)
		{
			pcl.elem_num = 0;
			pcl.vars = nullptr;
			return;
		}

		size_t xl_id = size_t((xl - x0) / h);
		size_t xu_id = size_t((xu - x0) / h);
		if (xu - x0 > h * double(xu_id)) ++xu_id;
		size_t yl_id = size_t((yl - y0) / h);
		size_t yu_id = size_t((yu - y0) / h);
		if (yu - y0 > h * double(yu_id)) ++yu_id;
		size_t x_num = xu_id - xl_id;
		size_t y_num = yu_id - yl_id;
		pcl.elem_num = x_num * y_num;

		if (pcl.elem_num == 1)
		{
			pcl.vars = &pcl.var;
			pcl.var.elem_x_id = xl_id;
			pcl.var.elem_y_id = yl_id;
		}
		else
		{
			pcl.vars = pcl_var_mem.alloc(pcl.elem_num);
			double x_len1, x_len2, y_len1, y_len2;
			if (x_num == 1)
			{
				x_len1 = hlen * 2.0;
				// y_num must == 2
				y_len1 = y0 + double(yl_id + 1) * h - yl;
				y_len2 = yu - y0 - double(yu_id) * h;
				pcl.vars[0].x = pcl.x;
				pcl.vars[0].y = ;
				pcl.vars[0].vol = x_len1 * y_len1;
				pcl.vars[0].elem_x_id = xl_id;
				pcl.vars[0].elem_y_id = yl_id;
				pcl.vars[1].x = pcl.x;
				pcl.vars[1].y = ;
				pcl.vars[1].vol = x_len1 * y_len2;
				pcl.vars[1].elem_x_id = xl_id;
				pcl.vars[1].elem_y_id = yl_id + 1;
			}
			else
			{
				x_len1 = x0 + double(xl_id + 1) * h - xl;
				x_len2 = xu - x0 - double(xu_id) * h;
				if (y_num == 1)
				{
					y_len1 = hlen * 2.0;
					pcl.vars[0].x = ;
					pcl.vars[0].y = pcl.y;
					pcl.vars[0].vol = x_len1 * y_len1;
					pcl.vars[0].elem_x_id = xl_id;
					pcl.vars[0].elem_y_id = yl_id;
					pcl.vars[1].x = ;
					pcl.vars[1].y = pcl.y;
					pcl.vars[1].vol = x_len2 * y_len1;
					pcl.vars[1].elem_x_id = xl_id + 1;
					pcl.vars[1].elem_y_id = yl_id;
				}
				else
				{
					y_len1 = y0 + double(yl_id + 1) * h - yl;
					y_len2 = yu - y0 - double(yu_id) * h;
					pcl.vars[0].x = ;
					pcl.vars[0].y = ;
					pcl.vars[0].vol = x_len1 * y_len1;
					pcl.vars[0].elem_x_id = xl_id;
					pcl.vars[0].elem_y_id = yl_id;
					pcl.vars[1].x = ;
					pcl.vars[1].y = ;
					pcl.vars[1].vol = x_len2 * y_len1;
					pcl.vars[1].elem_x_id = xl_id + 1;
					pcl.vars[1].elem_y_id = yl_id;
					pcl.vars[2].x = ;
					pcl.vars[2].y = ;
					pcl.vars[2].vol = x_len1 * y_len2;
					pcl.vars[2].elem_x_id = xl_id;
					pcl.vars[2].elem_y_id = yl_id + 1;
					pcl.vars[3].x = ;
					pcl.vars[3].y = ;
					pcl.vars[3].vol = x_len2 * y_len2;
					pcl.vars[3].elem_x_id = xl_id + 1;
					pcl.vars[3].elem_y_id = yl_id + 1;
				}
			}
		}

	}
};

#endif