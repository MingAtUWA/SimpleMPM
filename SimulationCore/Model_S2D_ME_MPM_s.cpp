#include "SimulationCore_pcp.h"

#include "Model_S2D_ME_MPM_s.h"

Model_S2D_ME_MPM_s::Model_S2D_ME_MPM_s() : 
	Model("Model_S2D_ME_MPM_s"),
	nodes(nullptr), node_x_num(0), node_y_num(0),
	bfx_num(0), bfy_num(0), bfxs(nullptr), bfys(nullptr),
	tx_num(0), ty_num(0), txs(nullptr), tys(nullptr),
	ax_num(0), ay_num(0), axs(nullptr), ays(nullptr),
	vx_num(0), vy_num(0), vxs(nullptr), vys(nullptr),
	pcl_var_mem(10), x_len_buf(20), y_len_buf(20) {}

Model_S2D_ME_MPM_s::~Model_S2D_ME_MPM_s()
{
	clear_mesh();
	clear_pcl();
	if (bfxs) delete[] bfxs;
	bfxs = nullptr;
	bfx_num = 0;
	if (bfys) delete[] bfys;
	bfys = nullptr;
	bfy_num = 0;
	if (txs) delete[] txs;
	txs = nullptr;
	tx_num = 0;
	if (tys) delete[] tys;
	tys = nullptr;
	ty_num = 0;
	if (axs) delete[] axs;
	axs = nullptr;
	ax_num = 0;
	if (ays) delete[] ays;
	ays = nullptr;
	ay_num = 0;
	if (vxs) delete[] vxs;
	vxs = nullptr;
	vx_num = 0;
	if (vys) delete[] vys;
	vys = nullptr;
	vy_num = 0;
}

void Model_S2D_ME_MPM_s::get_elements_overlapped_by_particle(Particle_S2D_ME &pcl)
{
	double hlen = sqrt(pcl.m / pcl.density) * 0.5; // half length
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
	if (xu - x0 > h * double(xu_id) && xu != xn) ++xu_id;
	size_t yl_id = size_t((yl - y0) / h);
	size_t yu_id = size_t((yu - y0) / h);
	if (yu - y0 > h * double(yu_id) && yu != yn) ++yu_id;
	size_t x_num = xu_id - xl_id;
	size_t y_num = yu_id - yl_id;
	pcl.elem_num = x_num * y_num;

	if (pcl.elem_num == 1)
	{
		pcl.vars = &pcl.var;
		pcl.var.x = (xl + xu) * 0.5;
		pcl.var.y = (yl + yu) * 0.5;
		pcl.var.vol = (xu - xl) * (yu - yl);
		pcl.var.elem_x_id = xl_id;
		pcl.var.elem_y_id = yl_id;
	}
	else
	{
		pcl.vars = pcl_var_mem.alloc(pcl.elem_num);
		double x_len1, x_len2, y_len1, y_len2;
		if (x_num == 1)
		{
			x_len1 = xu - xl;
			// y_num must == 2
			y_len1 = y0 + double(yl_id + 1) * h - yl;
			y_len2 = yu - yl - y_len1;
			pcl.vars[0].x = pcl.x;
			pcl.vars[0].y = yl + y_len1 * 0.5;
			pcl.vars[0].vol = x_len1 * y_len1;
			pcl.vars[0].elem_x_id = xl_id;
			pcl.vars[0].elem_y_id = yl_id;
			pcl.vars[1].x = pcl.x;
			pcl.vars[1].y = yu - y_len2 * 0.5;
			pcl.vars[1].vol = x_len1 * y_len2;
			pcl.vars[1].elem_x_id = xl_id;
			pcl.vars[1].elem_y_id = yl_id + 1;
		}
		else
		{
			x_len1 = x0 + double(xl_id + 1) * h - xl;
			x_len2 = xu - xl - x_len1;
			if (y_num == 1)
			{
				y_len1 = yu - yl;
				pcl.vars[0].x = xl + x_len1 * 0.5;
				pcl.vars[0].y = pcl.y;
				pcl.vars[0].vol = x_len1 * y_len1;
				pcl.vars[0].elem_x_id = xl_id;
				pcl.vars[0].elem_y_id = yl_id;
				pcl.vars[1].x = xu - x_len2 * 0.5;
				pcl.vars[1].y = pcl.y;
				pcl.vars[1].vol = x_len2 * y_len1;
				pcl.vars[1].elem_x_id = xl_id + 1;
				pcl.vars[1].elem_y_id = yl_id;
			}
			else
			{
				y_len1 = y0 + double(yl_id + 1) * h - yl;
				y_len2 = yu - yl - y_len1;
				pcl.vars[0].x = xl + x_len1 * 0.5;
				pcl.vars[0].y = yl + y_len1 * 0.5;
				pcl.vars[0].vol = x_len1 * y_len1;
				pcl.vars[0].elem_x_id = xl_id;
				pcl.vars[0].elem_y_id = yl_id;
				pcl.vars[1].x = xu - x_len2 * 0.5;
				pcl.vars[1].y = yl + y_len1 * 0.5;
				pcl.vars[1].vol = x_len2 * y_len1;
				pcl.vars[1].elem_x_id = xl_id + 1;
				pcl.vars[1].elem_y_id = yl_id;
				pcl.vars[2].x = xl + x_len1 * 0.5;
				pcl.vars[2].y = yu - y_len2 * 0.5;
				pcl.vars[2].vol = x_len1 * y_len2;
				pcl.vars[2].elem_x_id = xl_id;
				pcl.vars[2].elem_y_id = yl_id + 1;
				pcl.vars[3].x = xu - x_len2 * 0.5;
				pcl.vars[3].y = yu - y_len2 * 0.5;
				pcl.vars[3].vol = x_len2 * y_len2;
				pcl.vars[3].elem_x_id = xl_id + 1;
				pcl.vars[3].elem_y_id = yl_id + 1;
			}
		}
	}
}

void Model_S2D_ME_MPM_s::get_elements_overlapped_by_particle2(Particle_S2D_ME &pcl)
{
	if (pcl.x < x0 || pcl.x > xn || pcl.y < y0 || pcl.y > xn)
	{
		pcl.elem_num = 0;
		pcl.vars = nullptr;
		return;
	}

	double hlen = sqrt(pcl.m / pcl.density) * 0.5; // half length
	double xl = pcl.x - hlen;
	if (xl < x0) xl = x0; // xl = max(xl, x0)
	double xu = pcl.x + hlen;
	if (xu > xn) xu = xn; // xu = min(xu, xn)
	double yl = pcl.y - hlen;
	if (yl < y0) yl = y0; // yl = max(yl, y0)
	double yu = pcl.y + hlen;
	if (yu > yn) yu = yn;

	size_t xl_id = size_t((xl - x0) / h);
	size_t xu_id = size_t((xu - x0) / h);
	if (xu - x0 > h * double(xu_id) && xu != xn) ++xu_id;
	size_t yl_id = size_t((yl - y0) / h);
	size_t yu_id = size_t((yu - y0) / h);
	if (yu - y0 > h * double(yu_id) && yu != yn) ++yu_id;
	size_t x_num = xu_id - xl_id;
	size_t y_num = yu_id - yl_id;
	pcl.elem_num = x_num * y_num;
	
#define PCL_LEN_BUFFER_SIZE 6
	union
	{
		struct { double x_len1, x_len2; };
		double x_len[PCL_LEN_BUFFER_SIZE-1];
	};
	union
	{
		struct { double y_len1, y_len2; };
		double y_len[PCL_LEN_BUFFER_SIZE-1];
	};
	if (x_num == 1)
	{
		x_len1 = xu - xl;
	}
	else if (x_num == 2)
	{
		x_len1 = x0 + double(xl_id + 1) * h - xl;
		x_len2 = xu - xl - x_len1;
	}
	else if (x_num < PCL_LEN_BUFFER_SIZE)
	{
		x_len[0] = x0 + double(xl_id + 1) * h - xl;
		for (size_t i = 1; i < x_num - 1; ++i)
			x_len[i] = h;
		x_len[x_num - 1] = xu - double(xu_id - 1) * h - x0;
	}
	else
	{
		x_len_buf.reset();
		x_len_buf.reserve(x_num);
		double len_tmp;
		len_tmp = x0 + double(xl_id + 1) * h - xl;
		x_len_buf.add(len_tmp);
		for (size_t i = 1; i < x_num - 1; ++i)
			x_len_buf.add(h);
		len_tmp = xu - double(xu_id - 1) * h - x0;
		x_len_buf.add(len_tmp);
	}
	if (y_num == 1)
	{
		y_len1 = yu - yl;
	}
	else if (y_num == 2)
	{
		y_len1 = y0 + double(yl_id + 1) * h - yl;
		y_len2 = yu - yl - y_len1;
	}
	else if (y_num < PCL_LEN_BUFFER_SIZE)
	{
		y_len[0] = y0 + double(yl_id + 1) * h - yl;
		for (size_t i = 1; i < y_num - 1; ++i)
			y_len[i] = h;
		y_len[y_num - 1] = yu - double(yu_id - 1) * h - y0;
	}
	else
	{
		y_len_buf.reset();
		y_len_buf.reserve(y_num);
		double len_tmp;
		len_tmp = y0 + double(yl_id + 1) * h - yl;
		y_len_buf.add(len_tmp);
		for (size_t i = 1; i < y_num - 1; ++i)
			x_len_buf.add(h);
		len_tmp = yu - double(yu_id - 1) * h - y0;
		y_len_buf.add(len_tmp);
	}

	if (pcl.elem_num == 1)
	{
		pcl.vars = &pcl.var;
		pcl.var.x = (xl + xu) * 0.5;
		pcl.var.y = (yl + yu) * 0.5;
		pcl.var.vol = (xu - xl) * (yu - yl);
		pcl.var.elem_x_id = xl_id;
		pcl.var.elem_y_id = yl_id;
		return;
	}
	else
	{
		pcl.vars = pcl_var_mem.alloc(pcl.elem_num);
		if (x_num == 1)
		{
			if (y_num == 2)
			{
				pcl.vars[0].x = pcl.x;
				pcl.vars[0].y = yl + y_len1 * 0.5;
				pcl.vars[0].vol = x_len1 * y_len1;
				pcl.vars[0].elem_x_id = xl_id;
				pcl.vars[0].elem_y_id = yl_id;
				pcl.vars[1].x = pcl.x;
				pcl.vars[1].y = yu - y_len2 * 0.5;
				pcl.vars[1].vol = x_len1 * y_len2;
				pcl.vars[1].elem_x_id = xl_id;
				pcl.vars[1].elem_y_id = yl_id + 1;
				return;
			}
		}
		else if (x_num == 2)
		{
			if (y_num == 1)
			{
				pcl.vars[0].x = xl + x_len1 * 0.5;
				pcl.vars[0].y = pcl.y;
				pcl.vars[0].vol = x_len1 * y_len1;
				pcl.vars[0].elem_x_id = xl_id;
				pcl.vars[0].elem_y_id = yl_id;
				pcl.vars[1].x = xu - x_len2 * 0.5;
				pcl.vars[1].y = pcl.y;
				pcl.vars[1].vol = x_len2 * y_len1;
				pcl.vars[1].elem_x_id = xl_id + 1;
				pcl.vars[1].elem_y_id = yl_id;
				return;
			}
			else if (y_num == 2)
			{
				pcl.vars[0].x = xl + x_len1 * 0.5;
				pcl.vars[0].y = yl + y_len1 * 0.5;
				pcl.vars[0].vol = x_len1 * y_len1;
				pcl.vars[0].elem_x_id = xl_id;
				pcl.vars[0].elem_y_id = yl_id;
				pcl.vars[1].x = xu - x_len2 * 0.5;
				pcl.vars[1].y = yl + y_len1 * 0.5;
				pcl.vars[1].vol = x_len2 * y_len1;
				pcl.vars[1].elem_x_id = xl_id + 1;
				pcl.vars[1].elem_y_id = yl_id;
				pcl.vars[2].x = xl + x_len1 * 0.5;
				pcl.vars[2].y = yu - y_len2 * 0.5;
				pcl.vars[2].vol = x_len1 * y_len2;
				pcl.vars[2].elem_x_id = xl_id;
				pcl.vars[2].elem_y_id = yl_id + 1;
				pcl.vars[3].x = xu - x_len2 * 0.5;
				pcl.vars[3].y = yu - y_len2 * 0.5;
				pcl.vars[3].vol = x_len2 * y_len2;
				pcl.vars[3].elem_x_id = xl_id + 1;
				pcl.vars[3].elem_y_id = yl_id + 1;
				return;
			}
		}
		// multiple by multiple
		double *x_len_array, *y_len_array;
		x_len_array = x_num < PCL_LEN_BUFFER_SIZE ? x_len : x_len_buf.get_mem();
		y_len_array = y_num < PCL_LEN_BUFFER_SIZE ? y_len : y_len_buf.get_mem();
		size_t k = 0;
		for (size_t j = 0; j < y_num; ++j)
			for (size_t i = 0; i < x_num; ++i)
			{
				ParticleVar_S2D_ME &var = pcl.vars[k];
				var.x = ;
				var.y = ;
				var.vol = x_len_array[i] * y_len_array[j];
				var.elem_x_id = ;
				var.elem_y_id = ;
				++k;
			}

	}
}