#ifndef _MODEL_R2D_ME_MPM_BSPLINE_S_H_
#define _MODEL_R2D_ME_MPM_BSPLINE_S_H_

#include "ItemArray.hpp"

#include "Model.h"
#include "BC.h"

struct Node_R2D_ME_Grid
{
	// whether this node needs calculation
	unsigned short cal_flag;

	// acceleration, velocity
	// displacement increment
	double ax, vx, dux;
	double ay, vy, duy;
	// mass
	double m;
	// external force
	double fx_ext, fy_ext;
	// internal force
	double fx_int, fy_int;
};

struct Particle_R2D_ME_Grid
{
public:
	// location
	double x, y;

	// mass
	double m;
	// density
	double density;

	// velocity
	double vx, vy;

	// stress
	double s11, s22, s12;
	// total strain
	double e11, e22, e12;

	// constitutive relation
	double E;   // Elastic modulus
	double niu; // Poisson ratio
	
public:
	inline void init(void)
	{
		x = 0.0;
		y = 0.0;
		m = 0.0;
		density = 0.0;
		vx = 0.0;
		vy = 0.0;
		s11 = 0.0;
		s22 = 0.0;
		s12 = 0.0;
		e11 = 0.0;
		e22 = 0.0;
		e12 = 0.0;
		E = 0.0;
		niu = 0.0;
	}

public:
	// APIC C matrix, C = B * 1/D
	double C[2][2];

	// weight
	double N[3][3];
	double dN_dx[3][3];
	double dN_dy[3][3];

	double ux, uy;
	double x_ori, y_ori;

	bool is_in_mesh;
	size_t base_node_x_id;
	size_t base_node_y_id;
};

struct Model_R2D_ME_MPM_BSpline_s : public Model
{
	// background mesh
	double h; // grid size
	double x_start,  y_start; // boundary
	double x_bound1, x_bound2; // one node within boundary
	double y_bound1, y_bound2;
	size_t node_x_num, node_y_num;
	size_t node_num;
	Node_R2D_ME_Grid *nodes;
	MemoryUtilities::ItemArray<Node_R2D_ME_Grid> nodes_mem;

	// particles
	size_t pcl_num;
	Particle_R2D_ME_Grid *pcls;
	MemoryUtilities::ItemArray<Particle_R2D_ME_Grid> pcls_mem;

	// boundary conditions
	// body force boundary
	size_t bfx_num;
	BodyForce *bfxs;
	MemoryUtilities::ItemArray<BodyForce> bfxs_mem;
	size_t bfy_num;
	BodyForce *bfys;
	MemoryUtilities::ItemArray<BodyForce> bfys_mem;
	// traction force bc
	size_t tx_num;
	TractionBC_MPM *txs;
	MemoryUtilities::ItemArray<TractionBC_MPM> txs_mem;
	size_t ty_num;
	TractionBC_MPM *tys;
	MemoryUtilities::ItemArray<TractionBC_MPM> tys_mem;
	// acceleration bc
	size_t ax_num;
	AccelerationBC *axs;
	MemoryUtilities::ItemArray<AccelerationBC> axs_mem;
	size_t ay_num;
	AccelerationBC *ays;
	MemoryUtilities::ItemArray<AccelerationBC> ays_mem;
	// velocity bc
	size_t vx_num;
	VelocityBC *vxs;
	MemoryUtilities::ItemArray<VelocityBC> vxs_mem;
	size_t vy_num;
	VelocityBC *vys;
	MemoryUtilities::ItemArray<VelocityBC> vys_mem;

public:
	Model_R2D_ME_MPM_BSpline_s();
	~Model_R2D_ME_MPM_BSpline_s() { clear(); }

	void set_mesh(size_t x_elem_num, size_t y_elem_num,
		double grid_size, double x_st = 0.0, double y_st = 0.0)
	{
		h = grid_size;
		x_start = x_st;
		x_bound1 = x_st + h;
		x_bound2 = x_st + h * (x_elem_num - 1);
		y_start = y_st;
		y_bound1 = y_st + h;
		y_bound2 = y_st + h * (y_elem_num - 1);
		node_x_num = x_elem_num + 1;
		node_y_num = y_elem_num + 1;
		node_num = node_x_num * node_y_num;
	}

	void set_pcl_num(size_t num) { pcls_mem.reserve(num); }
	void add_pcl(Particle_R2D_ME_Grid &pcl) { pcls_mem.add(pcl); }

	void set_bfx_num(size_t num) { bfxs_mem.reserve(num); }
	void add_bfx(BodyForce &bf) { bfxs_mem.add(bf); }
	void set_bfy_num(size_t num) { bfys_mem.reserve(num); }
	void add_bfy(BodyForce &bf) { bfys_mem.add(bf); }

	void set_tx_num(size_t num) { txs_mem.reserve(num); }
	void add_tx(TractionBC_MPM &tbc) { txs_mem.add(tbc); }
	void set_ty_num(size_t num) { tys_mem.reserve(num); }
	void add_ty(TractionBC_MPM &tbc) { tys_mem.add(tbc); }

	void set_ax_num(size_t num) { axs_mem.reserve(num); }
	void add_ax(AccelerationBC &abc) { axs_mem.add(abc); }
	void set_ay_num(size_t num) { ays_mem.reserve(num); }
	void add_ay(AccelerationBC &abc) { ays_mem.add(abc); }

	void set_vx_num(size_t num) { vxs_mem.reserve(num); }
	void add_vx(VelocityBC &vbc) { vxs_mem.add(vbc); }
	void set_vy_num(size_t num) { vys_mem.reserve(num); }
	void add_vy(VelocityBC &vbc) { vys_mem.add(vbc); }

	int update(void);
	void clear(void);

public:
	inline bool is_in_mesh(double x, double y) const
	{
		return x < x_bound1 && x >= x_bound2
			&& y < y_bound1 && y >= y_bound2;
	}

	// return 0: invalide node
	inline size_t base_node_x_index(double x) const
	{
		return size_t((x - x_start) / h + 0.5);
	}

	inline size_t base_node_y_index(double y) const
	{
		return size_t((y - y_start) / h + 0.5);
	}

	inline Node_R2D_ME_Grid &get_node(size_t i, size_t j) const
	{
		return nodes[j * node_x_num + i];
	}

	//return (x - x_start) / h - double(i);
	//return (y - y_start) / h - double(j);

	// ================= shape function =================
	// (-1.5, -0.5)
	inline double N_left(double u) const { return (3.0 + 2.0*u) * (3.0 + 2.0*u) / 8.0; }
	// (-0.5, 0.5)
	inline double N_mid(double u) const { return 0.75 - u*u; }
	// (0.5, 1.5)
	inline double N_right(double u) const {	return (3.0 - 2.0*u) * (3.0 - 2.0*u) / 8.0; }

	// ========== derivative of shape function ==========
	// (-1.5, -0.5)
	inline double dN_dx_left(double u) const { return 0.5 * (2.0*u + 3.0) / h; }
	// (-0.5, 0.5)
	inline double dN_dx_mid(double u) const { return -2.0 * u / h; }
	// (0.5, 1.5)
	inline double dN_dx_right(double u) const { return 0.5 * (2.0*u - 3.0) / h; }
};

#endif