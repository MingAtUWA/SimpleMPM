#ifndef _Model_S2D_CHM_MPM_S_
#define _Model_S2D_CHM_MPM_S_

struct Node_S2D_CHM
{
	size_t index_x, index_y;

	// for soil (mixture) phase
	double m_s;
	double ax_s, ay_s;
	double vx_s, vy_s;
	double fx_ext_m, fy_ext_m;
	double fx_int_m, fy_int_m;
	double fx_kin_f, fy_kin_f;
	
	// for fluid phase
	double m_f;
	double ax_f, ay_f;
	double vx_f, vy_f;
	double fx_ext_f, fy_ext_f;
	double fx_int_f, fy_int_f;
	double fx_drag_f, fy_drag_f;
};

struct Particle_Integration_Info
{
	double x, y;
	double vol;
};

struct Particle_S2D_CHM
{
	size_t index;

	size_t grid_num;
	Particle_Integration_Info pcl_info;
	Particle_Integration_Info *pcl_infos;

	double n;
	double m_s;
	double density_s;
	double density_f;

	double vx_s, vy_s;
	double vx_f, vy_f;

	// effective stress
	double s11;
	double s22;
	double s12;
	// pore pressure
	double p;

	// total strain
	double e11;
	double e22;
	double e12;
	// elastic strain
	double es11;
	double es22;
	double es12;
	// plastic strain
	double ps11;
	double ps22;
	double ps12;
};

class Model_S2D_CHM_MPM_s
{
public:
	double h;
	double x0, y0;
	size_t node_x_num, node_y_num;
	Node_S2D_CHM *nodes;

	size_t pcl_num;
	Particle_S2D_CHM *pcls;

public:
	Model_S2D_CHM_MPM_s() :
		nodes(nullptr), node_x_num(0), node_y_num(0) {}
	~Modle_S2D_CHM_MPM_s()
	{
		clear_mesh();
		clear_pcl();
	}

	void init_mesh(double grid_size, double x_start, double y_start,
				   size_t elem_x_num, size_t elem_y_num)
	{
		clear_mesh();
		h = grid_size;
		x0 = x_start;
		y0 = y_start;
		node_x_num = elem_x_num + 1;
		node_y_num = elem_y_num + 1;
		nodes = new Node_S2D_CHM[node_x_num * node_y_num];
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

	void init_pcl(size_t num)
	{
		pcl_num = num;
		pcls = Particle_S2D_CHM[num];
	}
	void clear_pcl(void)
	{
		if (pcls) delete[] pcls;
		pcls = nullptr;
		pcl_num = 0;
	}
};

#endif