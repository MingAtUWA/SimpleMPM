#ifndef _MODEL_1D_ME_MPM_BSPLINE_S_H_
#define _MODEL_1D_ME_MPM_BSPLINE_S_H_

#include "ItemArray.hpp"

#include "Model.h"
#include "BC.h"

struct Node_1D_ME_Grid
{
	size_t index;

	unsigned short cal_flag;

	double m = 0.0;
	double mv = 0.0;
	double f_ext = 0.0;
	double f_int = 0.0;
	double a, v, du;
};

struct Element_1D_ME_Grid
{
	size_t index;
};

struct Particle_1D_ME
{
	// coordinate
	double x;
	double x_ori;
	double u;

	// mass
	double m;
	double density;

	// momentum
	double mv;

	// stress
	double s11;

	// total strain
	double e11;
	
	// elastic strain
	//double es11;

	// plastic strain
	//double ps11;

public:
	inline void init(void)
	{
		x = 0.0;
		m = 0.0;
		density = 0.0;
		mv = 0.0;
		s11 = 0.0;
		e11 = 0.0;
	}

public: // --------------- Obsoleted -----------------
		// constitutive relation
	double E;   // Elastic modulus

public: // calculation variables
	double N[3];
	double dN_dx[3];
	double de11;
	bool is_in_mesh;
	size_t node_x_id;
};


struct Model_1D_ME_MPM_BSpline_s : public Model
{
public:
	// background mesh
	double h; // grid size
	double x_start; // boundary
	double x_bound1, x_bound2; // one node within boundary
	size_t node_num;
	Node_1D_ME_Grid *nodes;
	MemoryUtilities::ItemArray<Node_1D_ME_Grid> nodes_mem;
	size_t elem_num;
	Element_1D_ME_Grid *elems;
	MemoryUtilities::ItemArray<Element_1D_ME_Grid> elems_mem;

	// particles
	size_t pcl_num;
	Particle_1D_ME *pcls;
	MemoryUtilities::ItemArray<Particle_1D_ME> pcls_mem;

	// boundary conditions
	size_t bf_num;
	BodyForce *bfs;
	MemoryUtilities::ItemArray<BodyForce> bfs_mem;
	size_t tbc_num;
	TractionBC_MPM *tbcs;
	MemoryUtilities::ItemArray<TractionBC_MPM> tbcs_mem;
	size_t abc_num;
	AccelerationBC *abcs;
	MemoryUtilities::ItemArray<AccelerationBC> abcs_mem;
	size_t vbc_num;
	VelocityBC *vbcs;
	MemoryUtilities::ItemArray<VelocityBC> vbcs_mem;

public:
	Model_1D_ME_MPM_BSpline_s();
	~Model_1D_ME_MPM_BSpline_s() { clear(); }

	void set_mesh(size_t x_num,
				  double grid_size,
				  double x_st = 0.0)
	{
		h = grid_size;
		x_start  = x_st;
		x_bound1 = x_st + h;
		x_bound2 = x_st + h * (x_num - 1);
		node_num = x_num + 1;
		elem_num = x_num;
	}

	void set_pcl_num(size_t num) { pcls_mem.reserve(num); }
	void add_pcl(Particle_1D_ME &pcl) { pcls_mem.add(pcl); }

	void set_bf_num(size_t num) { bfs_mem.reserve(num); }
	void add_bf(BodyForce &bf) { bfs_mem.add(bf); }

	void set_tbc_num(size_t num) { tbcs_mem.reserve(num); }
	void add_tbc(TractionBC_MPM &tbc) { tbcs_mem.add(tbc); }

	void set_abc_num(size_t num) { abcs_mem.reserve(num); }
	void add_abc(AccelerationBC &abc) { abcs_mem.add(abc); }

	void set_vbc_num(size_t num) { vbcs_mem.reserve(num); }
	void add_vbc(VelocityBC &vbc) { vbcs_mem.add(vbc); }

	int update(void);
	void clear(void);

public:
	// return 0 -> invalide node
	inline size_t nearest_node_x_index(double x)
	{
		if (x < x_bound1 && x >= x_bound2)
			return 0;
		return size_t((x - x_start) / h + 0.5);
	}

	inline Node_1D_ME_Grid *get_node_by_index(size_t i)
	{
		return nodes + i;
	}

	inline double xi(double x, size_t i /* node index */)
	{
		return (x - x_start) / h - double(i);
	}

	inline double N(double u)
	{
		if (u >= -1.5)
			if (u < -0.5)
				return (2.0*u+3.0) * (2.0*u+3.0) / 8.0;
			else if (u < 0.5)
				return 0.75 - u*u;
			else if (u <= 1.5)
				return (3.0-2.0*u) * (3.0-2.0*u) / 8.0;
		return 0.0;
	}

	inline double dN_dx(double u)
	{
		if (u >= -1.5)
			if (u < -0.5)
				return 0.5 * (2.0*u + 3.0) / h;
			else if (u < 0.5)
				return -2.0 * u / h;
			else if (u <= 1.5)
				return 0.5 * (2.0*u - 3.0) / h;
		return 0.0;
	}
};

#endif