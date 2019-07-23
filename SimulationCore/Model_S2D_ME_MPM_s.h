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
	double dvx, dvy, dux, duy;
	double de11, de22, de12, dw12;
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

	size_t elem_num; // The number of elements that this particle covers
	ParticleVar_S2D_ME var;
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

	MemoryUtilities::StackLikeBuffer<ParticleVar_S2D_ME> pcl_var_mem;

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
		pcl_var_mem.set_default_pool_size(pcl_num);
	}
	void clear_pcl(void)
	{
		if (pcls) delete[] pcls;
		pcls = nullptr;
		pcl_num = 0;
		pcl_var_mem.clear();
	}

	// Only suitable for case when particle is smaller than element
	// The calculation may not be good with particles larger than element
	void get_elements_overlapped_by_particle(Particle_S2D_ME &pcl);

	MemoryUtilities::ItemArray<double> x_len_buf;
	MemoryUtilities::ItemArray<double> y_len_buf;
	// if coordinate of the particle is outside the mesh,
	// the whole particle is ignored
	// support large particle (allow more stable calculation)
	void get_elements_overlapped_by_particle2(Particle_S2D_ME &pcl);
};

#endif