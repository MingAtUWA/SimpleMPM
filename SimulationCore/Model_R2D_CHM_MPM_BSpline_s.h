#ifndef _MODEL_R2D_CHM_MPM_BSPLINE_S_H_
#define _MODEL_R2D_CHM_MPM_BSPLINE_S_H_

#include "ItemArray.hpp"

#include "Model.h"
#include "BC.h"

struct Node_R2D_CHM_Grid
{
	// ------------- fluid phase -------------
	// acceleration, velocity
	double ax_f, ay_f;
	double vx_f, vy_f;
	// mass
	double m_tf;
	// external force
	double fx_ext_tf, fy_ext_tf;
	// internal force
	double fx_int_tf, fy_int_tf;
	// drag force
	double fx_drag_tf, fy_drag_tf;

	// ------------- solid phase -------------
	// acceleration, velocity
	double ax_s, ay_s;
	double vx_s, vy_s;
	// mass
	double m_s;
	// kinetic term of fluid phase
	double fx_kin_f, fy_kin_f;
	// external force
	double fx_ext_m, fy_ext_m;
	// internal force
	double fx_int_m, fy_int_m;

	// whether this node needs calculation
	unsigned short cal_flag;

	inline void init(void)
	{
		ax_f = 0.0;
		ay_f = 0.0;
		vx_f = 0.0;
		vy_f = 0.0;
		m_tf = 0.0;
		fx_ext_tf = 0.0;
		fy_ext_tf = 0.0;
		fx_int_tf = 0.0;
		fy_int_tf = 0.0;
		fx_drag_tf = 0.0;
		fy_drag_tf = 0.0;
		ax_s = 0.0;
		ay_s = 0.0;
		vx_s = 0.0;
		vy_s = 0.0;
		m_s = 0.0;
		fx_kin_f = 0.0;
		fy_kin_f = 0.0;
		fx_ext_m = 0.0;
		fy_ext_m = 0.0;
		fx_int_m = 0.0;
		fy_int_m = 0.0;
		cal_flag = 0;
	}
};


struct Particle_R2D_CHM_Grid
{
public:
	// location
	double x, y;

	/* mass of solid phase
	 * use mass of solid phase instead
	 * of volume to ensure mass conservation
	 * of solid phase */
	double m_s;

	// porosity
	double n;
	// density
	double density_s;
	double density_f;

	// velocity
	double vx_s, vy_s;
	double vx_f, vy_f;

	// stress
	double s11, s22, s12;
	double p;
	// total strain
	double e11, e22, e12;

	// constitutive relation
	double E;   // Elastic modulus
	double niu; // Poisson ratio
	double Kf;  // Bulk modulus of water
	double k;   // Permeability
	double miu; // Dynamic viscosity

public:
	inline void init(void)
	{
		x = 0.0;
		y = 0.0;
		m_s = 0.0;
		density_s = 0.0;
		density_f = 0.0;
		vx_s = 0.0;
		vy_s = 0.0;
		vx_f = 0.0;
		vy_f = 0.0;
		s11 = 0.0;
		s22 = 0.0;
		s12 = 0.0;
		p = 0.0;
		e11 = 0.0;
		e22 = 0.0;
		e12 = 0.0;
		E = 0.0;
		niu = 0.0;
		Kf = 0.0;
		k = 0.0;
		miu = 0.0;
	}

public: // calculation variables
	double x_ori, y_ori;
	double ux_s, uy_s;

	double vol;

	bool is_in_mesh;
	size_t base_node_x_id;
	size_t base_node_y_id;

	// APIC C matrix, C = B * 1/D
	double C_s[2][2], C_f[2][2];

	// weight
	double x_dist[3], y_dist[3];
	// [y][x]
	Node_R2D_CHM_Grid *pn[3][3]; // nodey0x0, nodey0x1, nodey0x2
	double N[3][3];
	double dN_dx[3][3];
	double dN_dy[3][3];
};


struct Model_R2D_CHM_MPM_BSpline_s : public Model
{
	// Background mesh
	double h; // grid size
	double x_start; // boundary
	double x_bound1, x_bound2; // one node within boundary
	double y_start;
	double y_bound1, y_bound2;
	size_t node_x_num, node_y_num;
	size_t node_num;
	Node_R2D_CHM_Grid *nodes;
	MemoryUtilities::ItemArray<Node_R2D_CHM_Grid> nodes_mem;

	// Particle
	size_t pcl_num;
	Particle_R2D_CHM_Grid *pcls;
	MemoryUtilities::ItemArray<Particle_R2D_CHM_Grid> pcls_mem;

	// Body force (on both phase)
	size_t bfx_num;
	BodyForce *bfxs;
	MemoryUtilities::ItemArray<BodyForce> bfxs_mem;
	size_t bfy_num;
	BodyForce *bfys;
	MemoryUtilities::ItemArray<BodyForce> bfys_mem;
	
	// Traction (on both mixture phase, total stress)
	size_t tx_num;
	TractionBC_MPM *txs;
	MemoryUtilities::ItemArray<TractionBC_MPM> txs_mem;
	size_t ty_num;
	TractionBC_MPM *tys;
	MemoryUtilities::ItemArray<TractionBC_MPM> tys_mem;
	
	// Acceleration bc
	// solid phase acceleration bc
	size_t asx_num;
	AccelerationBC *asxs;
	MemoryUtilities::ItemArray<AccelerationBC> asxs_mem;
	size_t asy_num;
	AccelerationBC *asys;
	MemoryUtilities::ItemArray<AccelerationBC> asys_mem;
	// fluid phase acceleration bc
	size_t afx_num;
	AccelerationBC *afxs;
	MemoryUtilities::ItemArray<AccelerationBC> afxs_mem;
	size_t afy_num;
	AccelerationBC *afys;
	MemoryUtilities::ItemArray<AccelerationBC> afys_mem;

	// Velocity bc
	// solid phase velocity bc
	size_t vsx_num;
	VelocityBC *vsxs;
	MemoryUtilities::ItemArray<VelocityBC> vsxs_mem;
	size_t vsy_num;
	VelocityBC *vsys;
	MemoryUtilities::ItemArray<VelocityBC> vsys_mem;

	// fluid phase velocity bc
	size_t vfx_num;
	VelocityBC *vfxs;
	MemoryUtilities::ItemArray<VelocityBC> vfxs_mem;
	size_t vfy_num;
	VelocityBC *vfys;
	MemoryUtilities::ItemArray<VelocityBC> vfys_mem;

public:
	Model_R2D_CHM_MPM_BSpline_s();
	~Model_R2D_CHM_MPM_BSpline_s() { clear(); }

	// Background mesh
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

	// Particle
	void set_pcl_num(size_t num) { pcls_mem.reserve(num); }
	void add_pcl(Particle_R2D_CHM_Grid &pcl) { pcls_mem.add(pcl); }

	// Body force
	void set_bfx_num(size_t num) { bfxs_mem.reserve(num); }
	void add_bfx(BodyForce &bf) { bfxs_mem.add(bf); }
	void set_bfy_num(size_t num) { bfys_mem.reserve(num); }
	void add_bfy(BodyForce &bf) { bfys_mem.add(bf); }

	// Traction
	void set_tx_num(size_t num) { txs_mem.reserve(num); }
	void add_tx(TractionBC_MPM &tbc) { txs_mem.add(tbc); }
	void set_ty_num(size_t num) { tys_mem.reserve(num); }
	void add_ty(TractionBC_MPM &tbc) { tys_mem.add(tbc); }

	// Accleration bc
	void set_asx_num(size_t num) { asxs_mem.reserve(num); }
	void add_asx(AccelerationBC &abc) { asxs_mem.add(abc); }
	void clear_asx(void) { asxs_mem.reset(); }
	void set_asy_num(size_t num) { asys_mem.reserve(num); }
	void add_asy(AccelerationBC &abc) { asys_mem.add(abc); }
	void clear_asy(void) { asys_mem.reset(); }
	void set_afx_num(size_t num) { afxs_mem.reserve(num); }
	void add_afx(AccelerationBC &abc) { afxs_mem.add(abc); }
	void clear_afx(void) { afxs_mem.reset(); }
	void set_afy_num(size_t num) { afys_mem.reserve(num); }
	void add_afy(AccelerationBC &abc) { afys_mem.add(abc); }
	void clear_afy(void) { afys_mem.reset(); }

	// Velocity bc
	void set_vsx_num(size_t num) { vsxs_mem.reserve(num); }
	void add_vsx(VelocityBC &vbc) { vsxs_mem.add(vbc); }
	void clear_vsx(void) { vsxs_mem.reset(); }
	void set_vsy_num(size_t num) { vsys_mem.reserve(num); }
	void add_vsy(VelocityBC &vbc) { vsys_mem.add(vbc); }
	void clear_vsy(void) { vsys_mem.reset(); }
	void set_vfx_num(size_t num) { vfxs_mem.reserve(num); }
	void add_vfx(VelocityBC &vbc) { vfxs_mem.add(vbc); }
	void clear_vfx(void) { vfxs_mem.reset(); }
	void set_vfy_num(size_t num) { vfys_mem.reserve(num); }
	void add_vfy(VelocityBC &vbc) { vfys_mem.add(vbc); }
	void clear_vfy(void) { vfys_mem.reset(); }

	int update(void);
	void clear(void);

public:
	inline bool is_in_mesh(double x, double y) const
	{
		return x >= x_bound1 && x < x_bound2 && y >= y_bound1 && y < y_bound2;
	}

	inline size_t base_node_x_index(double x) const { return size_t((x - x_start) / h - 0.5); }
	inline size_t base_node_y_index(double y) const { return size_t((y - y_start) / h - 0.5); }
	inline Node_R2D_CHM_Grid &get_node(size_t i, size_t j) const { return nodes[j * node_x_num + i]; }
	inline double  xi(double x, size_t i) const { return (x - x_start) / h - double(i); }
	inline double eta(double y, size_t j) const { return (y - y_start) / h - double(j); }

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

	inline void cal_shape_func(Particle_R2D_CHM_Grid &pcl)
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