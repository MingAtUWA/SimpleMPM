#ifndef _Model_S2D_CHM_MPM_S_MIX_H_
#define _Model_S2D_CHM_MPM_S_MIX_H_

#include "BC.h"
#include "Model.h"

#include "StackLikeBuffer.hpp"
#include "ItemArray.hpp"

#include "Node_S2D_CHM.h"

struct IntegrationVar_S2D_CHM_Mix
{
	double x, y, w;
	size_t elem_x_id, elem_y_id;

	Node_S2D_CHM *pn1, *pn2, *pn3, *pn4;
	double N1, N2, N3, N4;
	double dN1_dx, dN2_dx, dN3_dx, dN4_dx;
	double dN1_dy, dN2_dy, dN3_dy, dN4_dy;
};

struct Particle_S2D_CHM_Mix
{
	size_t index;

	double n;   // porosity
	double m_s; // mass of solid phase 
	double density_s, density_f;

	double ax_s, ay_s, ax_f, ay_f;
	double vx_s, vy_s, vx_f, vy_f;
	double ux_s, uy_s, ux_f, uy_f;

	double s11, s22, s12; // effective stress
	double p; // pore pressure

	double e11, e22, e12; // total strain
	double es11, es22, es12; // elastic strain
	double ps11, ps22, ps12; // plastic strain

	union
	{
		struct { double x, y, vol; size_t elem_x_id, elem_y_id; };
		IntegrationVar_S2D_CHM_Mix var;
	};
	
	double x_ori, y_ori;
	bool is_in_mesh;

	// Constitutive model
	double E;   // Elastic modulus
	double niu; // Poisson ratio
	double Kf;  // Bulk modulus of water
	double k;   // Permeability
	double miu; // Dynamic viscosity
	
	Particle_S2D_CHM_Mix *next;
};

struct Element_S2D_CHM_Mix
{
	size_t index_x, index_y;
	double mat_vol;
	// gauss point variable
	double avg_p;
	double avg_s11;
	double avg_s22;
	double avg_s12;
	// number of particle in this element
	Particle_S2D_CHM_Mix *top;
	inline void add_pcl(Particle_S2D_CHM_Mix &pcl) noexcept
	{
		pcl.next = top;
		top = &pcl;
	};
};

struct Model_S2D_CHM_MPM_s_Mix : public Model
{
public:
	double h, elem_vol, elem_vol_threshold;
	double x0, xn, y0, yn;
	size_t node_x_num, node_y_num, node_num;
	Node_S2D_CHM *nodes;
	size_t elem_x_num, elem_y_num, elem_num;
	Element_S2D_CHM_Mix *elems;

	size_t pcl_num;
	Particle_S2D_CHM_Mix *pcls;

	// boundary conditions
	size_t bfx_num, bfy_num;
	BodyForce *bfxs, *bfys;
	size_t tx_num, ty_num;
	TractionBC_MPM *txs, *tys;
	// solid phase
	size_t asx_num, asy_num;
	AccelerationBC *asxs, *asys;
	size_t vsx_num, vsy_num;
	VelocityBC *vsxs, *vsys;
	// fluid phase
	size_t afx_num, afy_num;
	AccelerationBC *afxs, *afys;
	size_t vfx_num, vfy_num;
	VelocityBC *vfxs, *vfys;

public:
	Model_S2D_CHM_MPM_s_Mix();
	~Model_S2D_CHM_MPM_s_Mix();

	void init_mesh(double grid_size, size_t e_x_num, size_t e_y_num,
		double x_start = 0.0, double y_start = 0.0)
	{
		size_t i, j, k;
		clear_mesh();
		h = grid_size;
		elem_vol = h * h;
		elem_vol_threshold = elem_vol * 0.9;
		x0 = x_start;
		xn = x0 + h * double(e_x_num);
		y0 = y_start;
		yn = y0 + h * double(e_y_num);
		// elements
		elem_x_num = e_x_num;
		elem_y_num = e_y_num;
		elem_num = elem_x_num * elem_y_num;
		elems = new Element_S2D_CHM_Mix[elem_num];
		k = 0;
		for (i = 0; i < elem_y_num; ++i)
			for (j = 0; j < elem_x_num; ++j)
			{
				elems[k].index_x = j;
				elems[k].index_y = i;
				++k;
			}
		// nodes
		node_x_num = elem_x_num + 1;
		node_y_num = elem_y_num + 1;
		node_num = node_x_num * node_y_num;
		nodes = new Node_S2D_CHM[node_num];
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
		node_num = 0;
		if (elems) delete[] elems;
		elems = nullptr;
		elem_x_num = 0;
		elem_y_num = 0;
		elem_num = 0;
	}

	void init_pcl(size_t num, double n, double m_s,
				  double density_s, double density_f,
				  double E, double niu, double Kf, double k, double miu)
	{
		clear_pcl();
		pcl_num = num;
		pcls = new Particle_S2D_CHM_Mix[num];
		for (size_t i = 0; i < num; ++i)
		{
			Particle_S2D_CHM_Mix &pcl = pcls[i];
			pcl.index = i;
			pcl.vx_s = 0.0;
			pcl.vy_s = 0.0;
			pcl.vx_f = 0.0;
			pcl.vy_f = 0.0;
			pcl.ux_s = 0.0;
			pcl.uy_s = 0.0;
			pcl.ux_f = 0.0;
			pcl.uy_f = 0.0;
			pcl.n = n;
			pcl.m_s = m_s;
			pcl.density_s = density_s;
			pcl.density_f = density_f;
			pcl.E = E;
			pcl.niu = niu;
			pcl.Kf = Kf;
			pcl.k = k;
			pcl.miu = miu;
			pcl.s11 = 0.0;
			pcl.s12 = 0.0;
			pcl.s22 = 0.0;
			pcl.p = 0.0;
			pcl.e11 = 0.0;
			pcl.e12 = 0.0;
			pcl.e22 = 0.0;
			pcl.es11 = 0.0;
			pcl.es12 = 0.0;
			pcl.es22 = 0.0;
			pcl.ps11 = 0.0;
			pcl.ps12 = 0.0;
			pcl.ps22 = 0.0;
		}
	}
	void clear_pcl(void)
	{
		if (pcls) delete[] pcls;
		pcls = nullptr;
		pcl_num = 0;
	}

public:
	inline bool find_in_which_element(Particle_S2D_CHM_Mix &pcl)
	{
		if (pcl.x < x0 || pcl.x >= xn || pcl.y < y0 || pcl.y >= yn)
			return false;
		pcl.elem_x_id = size_t((pcl.x - x0) / h);
		pcl.elem_y_id = size_t((pcl.y - y0) / h);
		return true;
	}

#define N_LOW(xi)  (1.0 - (xi)) / 2.0
#define N_HIGH(xi) (1.0 + (xi)) / 2.0
#define dN_dxi_LOW(xi) -0.5
#define dN_dxi_HIGH(xi) 0.5
	inline void cal_shape_func(IntegrationVar_S2D_CHM_Mix &inte_var)
	{
		inte_var.pn1 = nodes + node_x_num * inte_var.elem_y_id + inte_var.elem_x_id;
		inte_var.pn2 = inte_var.pn1 + 1;
		inte_var.pn3 = inte_var.pn2 + node_x_num;
		inte_var.pn4 = inte_var.pn3 - 1;
		double xi = 2.0 * ((inte_var.x - x0) / h - double(inte_var.elem_x_id)) - 1.0;
		double Nx_low  = N_LOW(xi);
		double Nx_high = N_HIGH(xi);
		double dNx_dxi_low  = dN_dxi_LOW(xi);
		double dNx_dxi_high = dN_dxi_HIGH(xi);
		double eta = 2.0 * ((inte_var.y - y0) / h - double(inte_var.elem_y_id)) - 1.0;
		double Ny_low  = N_LOW(eta);
		double Ny_high = N_HIGH(eta);
		double dNy_deta_low  = dN_dxi_LOW(eta);
		double dNy_deta_high = dN_dxi_HIGH(eta);
		inte_var.N1 = Nx_low  * Ny_low;
		inte_var.N2 = Nx_high * Ny_low;
		inte_var.N3 = Nx_high * Ny_high;
		inte_var.N4 = Nx_low  * Ny_high;
		double dxi_dx = 2.0 / h; // = deta_dy
		inte_var.dN1_dx = dNx_dxi_low  * Ny_low   * dxi_dx;
		inte_var.dN1_dy = Nx_low  * dNy_deta_low  * dxi_dx;
		inte_var.dN2_dx = dNx_dxi_high * Ny_low   * dxi_dx;
		inte_var.dN2_dy = Nx_high * dNy_deta_low  * dxi_dx;
		inte_var.dN3_dx = dNx_dxi_high * Ny_high  * dxi_dx;
		inte_var.dN3_dy = Nx_high * dNy_deta_high * dxi_dx;
		inte_var.dN4_dx = dNx_dxi_low  * Ny_high  * dxi_dx;
		inte_var.dN4_dy = Nx_low  * dNy_deta_high * dxi_dx;
	}
#undef N_LOW
#undef N_HIGH
#undef dN_dxi_LOW
#undef dN_dxi_HIGH

protected:
	friend int solve_substep_S2D_CHM_MPM_s_GIMP(void *_self);
};

#endif