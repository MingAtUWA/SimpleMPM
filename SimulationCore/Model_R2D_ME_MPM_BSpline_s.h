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
	double ax, vx;
	double ay, vy;
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
	double x_dist[3], y_dist[3];
	// [y][x]
	Node_R2D_ME_Grid *pn[3][3]; // nodey0x0, nodey0x1, nodey0x2
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
		return x >= x_bound1 && x < x_bound2
			&& y >= y_bound1 && y < y_bound2;
	}

	// return 0: invalide node
	inline size_t base_node_x_index(double x) const
	{
		return size_t((x - x_start) / h - 0.5);
	}

	inline size_t base_node_y_index(double y) const
	{
		return size_t((y - y_start) / h - 0.5);
	}

	inline Node_R2D_ME_Grid &get_node(size_t i, size_t j) const
	{
		return nodes[j * node_x_num + i];
	}

	//return (x - x_start) / h - double(i);
	//return (y - y_start) / h - double(j);

	// ================= shape function =================
	// (-1.5, -0.5)
	#define N_left(u) ((3.0 + 2.0*(u)) * (3.0 + 2.0*(u)) / 8.0)
	// (-0.5, 0.5)
	#define N_mid(u) (0.75 - (u)*(u))
	// (0.5, 1.5)
	#define N_right(u) ((3.0 - 2.0*(u)) * (3.0 - 2.0*(u)) / 8.0)

	// ========== derivative of shape function ==========
	// (-1.5, -0.5)
	#define dN_dx_left(u) (0.5 * (2.0*(u) + 3.0) / h)
	// (-0.5, 0.5)
	#define dN_dx_mid(u) (-2.0 * (u) / h)
	// (0.5, 1.5)
	#define dN_dx_right(u) (0.5 * (2.0*(u) - 3.0) / h)

	inline void cal_shape_func(Particle_R2D_ME_Grid &pcl)
	{
		double dist, dist_norm;
		double Nx0, Nx1, Nx2;
		double dNx0_dx, dNx1_dx, dNx2_dx;
		double Ny0, Ny1, Ny2;
		double dNy0_dy, dNy1_dy, dNy2_dy;

		/* ------------------- x direction ------------------- */
		pcl.base_node_x_id = base_node_x_index(pcl.x);
		// Nx0
		dist = (pcl.x - x_start) - h * double(pcl.base_node_x_id);
		pcl.x_dist[0] = -dist;
		dist_norm = dist / h;
		Nx0 = N_right(dist_norm);
		dNx0_dx = dN_dx_right(dist_norm);
		// Nx1
		dist -= h;
		pcl.x_dist[1] = -dist;
		dist_norm -= 1.0;
		Nx1 = N_mid(dist_norm);
		dNx1_dx = dN_dx_mid(dist_norm);
		// Nx2
		dist -= h;
		pcl.x_dist[2] = -dist;
		dist_norm -= 1.0;
		Nx2 = N_left(dist_norm);
		dNx2_dx = dN_dx_left(dist_norm);

		/* ------------------- y direction ------------------- */
		pcl.base_node_y_id = base_node_y_index(pcl.y);
		// Ny0
		dist = (pcl.y - y_start) - h * double(pcl.base_node_y_id);
		pcl.y_dist[0] = -dist;
		dist_norm = dist / h;
		Ny0 = N_right(dist_norm);
		dNy0_dy = dN_dx_right(dist_norm);
		// Ny1
		dist -= h;
		pcl.y_dist[1] = -dist;
		dist_norm -= 1.0;
		Ny1 = N_mid(dist_norm);
		dNy1_dy = dN_dx_mid(dist_norm);
		// Ny2
		dist -= h;
		pcl.y_dist[2] = -dist;
		dist_norm -= 1.0;
		Ny2 = N_left(dist_norm);
		dNy2_dy = dN_dx_left(dist_norm);

		auto pn = nodes + node_x_num * pcl.base_node_y_id + pcl.base_node_x_id;
		pcl.pn[0][0] = pn;
		pcl.pn[0][1] = pn + 1;
		pcl.pn[0][2] = pn + 2;
		pn += node_x_num;
		pcl.pn[1][0] = pn;
		pcl.pn[1][1] = pn + 1;
		pcl.pn[1][2] = pn + 2;
		pn += node_x_num;
		pcl.pn[2][0] = pn;
		pcl.pn[2][1] = pn + 1;
		pcl.pn[2][2] = pn + 2;

		pcl.N[0][0] = Ny0 * Nx0;
		pcl.N[0][1] = Ny0 * Nx1;
		pcl.N[0][2] = Ny0 * Nx2;
		pcl.N[1][0] = Ny1 * Nx0;
		pcl.N[1][1] = Ny1 * Nx1;
		pcl.N[1][2] = Ny1 * Nx2;
		pcl.N[2][0] = Ny2 * Nx0;
		pcl.N[2][1] = Ny2 * Nx1;
		pcl.N[2][2] = Ny2 * Nx2;

		pcl.dN_dx[0][0] = Ny0 * dNx0_dx;
		pcl.dN_dx[0][1] = Ny0 * dNx1_dx;
		pcl.dN_dx[0][2] = Ny0 * dNx2_dx;
		pcl.dN_dx[1][0] = Ny1 * dNx0_dx;
		pcl.dN_dx[1][1] = Ny1 * dNx1_dx;
		pcl.dN_dx[1][2] = Ny1 * dNx2_dx;
		pcl.dN_dx[2][0] = Ny2 * dNx0_dx;
		pcl.dN_dx[2][1] = Ny2 * dNx1_dx;
		pcl.dN_dx[2][2] = Ny2 * dNx2_dx;

		pcl.dN_dy[0][0] = dNy0_dy * Nx0;
		pcl.dN_dy[0][1] = dNy0_dy * Nx1;
		pcl.dN_dy[0][2] = dNy0_dy * Nx2;
		pcl.dN_dy[1][0] = dNy1_dy * Nx0;
		pcl.dN_dy[1][1] = dNy1_dy * Nx1;
		pcl.dN_dy[1][2] = dNy1_dy * Nx2;
		pcl.dN_dy[2][0] = dNy2_dy * Nx0;
		pcl.dN_dy[2][1] = dNy2_dy * Nx1;
		pcl.dN_dy[2][2] = dNy2_dy * Nx2;
	}
};

#undef N_left
#undef N_mid
#undef N_right
#undef dN_dx_left
#undef dN_dx_mid
#undef dN_dx_right

#endif