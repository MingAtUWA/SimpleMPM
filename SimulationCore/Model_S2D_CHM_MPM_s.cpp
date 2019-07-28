#include "SimulationCore_pcp.h"

#include "Model_S2D_CHM_MPM_s.h"

Model_S2D_CHM_MPM_s::Model_S2D_CHM_MPM_s() :
	Model("Model_S2D_CHM_MPM_s"),
	nodes(nullptr), node_x_num(0), node_y_num(0),
	pcls(nullptr), pcl_num(0),
	bfx_num(0), bfy_num(0), bfxs(nullptr), bfys(nullptr),
	tx_num(0),  ty_num(0),  txs(nullptr),  tys(nullptr),
	asx_num(0), asy_num(0), asxs(nullptr), asys(nullptr),
	vsx_num(0), vsy_num(0), vsxs(nullptr), vsys(nullptr),
	afx_num(0), afy_num(0), afxs(nullptr), afys(nullptr),
	vfx_num(0), vfy_num(0), vfxs(nullptr), vfys(nullptr),
	pcl_var_mem(10), x_var_info_buf(20), y_var_info_buf(20) {}

Model_S2D_CHM_MPM_s::~Model_S2D_CHM_MPM_s()
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
	// solid
	if (asxs) delete[] asxs;
	asxs = nullptr;
	asx_num = 0;
	if (asys) delete[] asys;
	asys = nullptr;
	asy_num = 0;
	if (vsxs) delete[] vsxs;
	vsxs = nullptr;
	vsx_num = 0;
	if (vsys) delete[] vsys;
	vsys = nullptr;
	vsy_num = 0;
	// fluid
	if (afxs) delete[] afxs;
	afxs = nullptr;
	afx_num = 0;
	if (afys) delete[] afys;
	afys = nullptr;
	afy_num = 0;
	if (vfxs) delete[] vfxs;
	vfxs = nullptr;
	vfx_num = 0;
	if (vfys) delete[] vfys;
	vfys = nullptr;
	vfy_num = 0;
}

void Model_S2D_CHM_MPM_s::get_elements_overlapped_by_particle(Particle_S2D_CHM &pcl)
{
	if (pcl.x < x0 || pcl.x >= xn || pcl.y < y0 || pcl.y >= yn)
	{
		pcl.elem_num = 0;
		pcl.vars = nullptr;
		return;
	}

	pcl.vol = pcl.m_s / (pcl.density_s * (1.0 - pcl.n));
	pcl.elem_x_id = size_t((pcl.x - x0) / h);
	pcl.elem_y_id = size_t((pcl.y - y0) / h);
	cal_shape_func(pcl.var);

	// for debug purpose, no gimp
	//pcl.vars = &pcl.var;
	//pcl.elem_num = 1;
	//return;

	pcl.is_at_edge = 0;
	double hlen = sqrt(pcl.vol) * 0.5; // half length
	double xl = pcl.x - hlen;
	if (xl < x0)
	{
		xl = x0; // xl = max(xl, x0)
		pcl.is_at_edge = 1;
	}
	double xu = pcl.x + hlen;
	if (xu > xn)
	{
		xu = xn; // xu = min(xu, xn)
		pcl.is_at_edge = 1;
	}
	double yl = pcl.y - hlen;
	if (yl < y0)
	{
		yl = y0; // yl = max(yl, y0)
		pcl.is_at_edge = 1;
	}
	double yu = pcl.y + hlen;
	if (yu > yn)
	{
		yu = yn;
		pcl.is_at_edge = 1;
	}
	size_t xl_id = size_t((xl - x0) / h);
	size_t xu_id = size_t((xu - x0) / h);
	if (xu - x0 > h * double(xu_id) && xu < xn) ++xu_id;
	size_t yl_id = size_t((yl - y0) / h);
	size_t yu_id = size_t((yu - y0) / h);
	if (yu - y0 > h * double(yu_id) && yu < yn) ++yu_id;
	size_t x_num = xu_id - xl_id;
	size_t y_num = yu_id - yl_id;
	pcl.elem_num = x_num * y_num;

	if (pcl.elem_num == 1)
	{
		// part of the particle is outside the edge
		if (pcl.is_at_edge == 1)
		{
			pcl.vars = pcl_var_mem.alloc(1);
			ParticleVar_S2D_CHM &pcl_var = pcl.vars[0];
			pcl_var.x = (xl + xu) * 0.5;
			pcl_var.y = (yl + yu) * 0.5;
			pcl_var.vol = (xu - xl) * (yu - yl);
			pcl_var.elem_x_id = xl_id;
			pcl_var.elem_y_id = yl_id;
			cal_shape_func(pcl_var);
		}
		else
		{
			pcl.vars = &pcl.var;
		}
		return;
	}

	pcl.vars = pcl_var_mem.alloc(pcl.elem_num);
	double x_len1, x_len2, y_len1, y_len2;
	if (x_num == 1 && y_num == 2)
	{
		x_len1 = xu - xl;
		y_len1 = y0 + double(yl_id + 1) * h - yl;
		y_len2 = yu - yl - y_len1;
		ParticleVar_S2D_CHM &pcl_var1 = pcl.vars[0];
		pcl_var1.x = pcl.x;
		pcl_var1.y = yl + y_len1 * 0.5;
		pcl_var1.vol = x_len1 * y_len1;
		pcl_var1.elem_x_id = xl_id;
		pcl_var1.elem_y_id = yl_id;
		cal_shape_func(pcl_var1);
		ParticleVar_S2D_CHM &pcl_var2 = pcl.vars[1];
		pcl_var2.x = pcl.x;
		pcl_var2.y = yu - y_len2 * 0.5;
		pcl_var2.vol = x_len1 * y_len2;
		pcl_var2.elem_x_id = xl_id;
		pcl_var2.elem_y_id = yl_id + 1;
		cal_shape_func(pcl_var2);
		return;
	}

	// particle is at internal edge
	pcl.is_at_edge = 2;

	if (x_num == 2)
	{
		x_len1 = x0 + double(xl_id + 1) * h - xl;
		x_len2 = xu - xl - x_len1;
		if (y_num == 1)
		{
			y_len1 = yu - yl;
			ParticleVar_S2D_CHM &pcl_var1 = pcl.vars[0];
			pcl_var1.x = xl + x_len1 * 0.5;
			pcl_var1.y = pcl.y;
			pcl_var1.vol = x_len1 * y_len1;
			pcl_var1.elem_x_id = xl_id;
			pcl_var1.elem_y_id = yl_id;
			cal_shape_func(pcl_var1);
			ParticleVar_S2D_CHM &pcl_var2 = pcl.vars[1];
			pcl_var2.x = xu - x_len2 * 0.5;
			pcl_var2.y = pcl.y;
			pcl_var2.vol = x_len2 * y_len1;
			pcl_var2.elem_x_id = xl_id + 1;
			pcl_var2.elem_y_id = yl_id;
			cal_shape_func(pcl_var2);
		}
		else
		{
			y_len1 = y0 + double(yl_id + 1) * h - yl;
			y_len2 = yu - yl - y_len1;
			ParticleVar_S2D_CHM &pcl_var1 = pcl.vars[0];
			pcl_var1.x = xl + x_len1 * 0.5;
			pcl_var1.y = yl + y_len1 * 0.5;
			pcl_var1.vol = x_len1 * y_len1;
			pcl_var1.elem_x_id = xl_id;
			pcl_var1.elem_y_id = yl_id;
			cal_shape_func(pcl_var1);
			ParticleVar_S2D_CHM &pcl_var2 = pcl.vars[1];
			pcl_var2.x = xu - x_len2 * 0.5;
			pcl_var2.y = yl + y_len1 * 0.5;
			pcl_var2.vol = x_len2 * y_len1;
			pcl_var2.elem_x_id = xl_id + 1;
			pcl_var2.elem_y_id = yl_id;
			cal_shape_func(pcl_var2);
			ParticleVar_S2D_CHM &pcl_var3 = pcl.vars[2];
			pcl_var3.x = xl + x_len1 * 0.5;
			pcl_var3.y = yu - y_len2 * 0.5;
			pcl_var3.vol = x_len1 * y_len2;
			pcl_var3.elem_x_id = xl_id;
			pcl_var3.elem_y_id = yl_id + 1;
			cal_shape_func(pcl_var3);
			ParticleVar_S2D_CHM &pcl_var4 = pcl.vars[3];
			pcl_var4.x = xu - x_len2 * 0.5;
			pcl_var4.y = yu - y_len2 * 0.5;
			pcl_var4.vol = x_len2 * y_len2;
			pcl_var4.elem_x_id = xl_id + 1;
			pcl_var4.elem_y_id = yl_id + 1;
			cal_shape_func(pcl_var4);
		}
		return;
	}

	PclVarInfo *x_var_infos, *y_var_infos;
	// x
	x_var_info_buf.reset();
	if (x_num == 1)
	{
		x_var_infos = x_var_info_buf.get_mem();
		x_var_infos[0].len = xu - xl;
		x_var_infos[0].pos = (xu + xl) * 0.5;
		x_var_infos[0].elem_id = size_t((x_var_infos[0].pos - x0) / h);
	}
	else
	{
		x_var_info_buf.reserve(x_num);
		x_var_infos = x_var_info_buf.get_mem();
		x_var_infos[0].len = x0 + double(xl_id + 1) * h - xl;
		x_var_infos[0].pos = xl + x_var_infos[0].len * 0.5;
		x_var_infos[0].elem_id = size_t((x_var_infos[0].pos - x0) / h);
		for (size_t i = 1; i < x_num - 1; ++i)
		{
			PclVarInfo &var_info = x_var_infos[i];
			var_info.len = h;
			var_info.pos = x_var_infos[i - 1].pos + (x_var_infos[i - 1].len + h) * 0.5;
			var_info.elem_id = x_var_infos[i - 1].elem_id + 1;
		}
		x_var_infos[x_num - 1].len = xu - double(xu_id - 1) * h - x0;
		x_var_infos[x_num - 1].pos = xu - x_var_infos[x_num - 1].len * 0.5;
		x_var_infos[x_num - 1].elem_id = x_var_infos[x_num - 2].elem_id + 1;
	}
	// y
	y_var_info_buf.reset();
	if (y_num == 1)
	{
		y_var_infos = y_var_info_buf.get_mem();
		y_var_infos[0].len = yu - yl;
		y_var_infos[0].pos = (yu + yl) * 0.5;
		y_var_infos[0].elem_id = size_t((y_var_infos[0].pos - y0) / h);
	}
	else
	{
		y_var_info_buf.reserve(y_num);
		y_var_infos = y_var_info_buf.get_mem();
		y_var_infos[0].len = y0 + double(yl_id + 1) * h - yl;
		y_var_infos[0].pos = yl + y_var_infos[0].len * 0.5;
		y_var_infos[0].elem_id = size_t((y_var_infos[0].pos - y0) / h);
		for (size_t i = 1; i < y_num - 1; ++i)
		{
			PclVarInfo &var_info = y_var_infos[i];
			var_info.len = h;
			var_info.pos = y_var_infos[i - 1].pos + (y_var_infos[i - 1].len + h) * 0.5;
			var_info.elem_id = y_var_infos[i - 1].elem_id + 1;
		}
		y_var_infos[y_num - 1].len = yu - double(yu_id - 1) * h - x0;
		y_var_infos[y_num - 1].pos = yu - y_var_infos[y_num - 1].len * 0.5;
		y_var_infos[y_num - 1].elem_id = size_t((y_var_infos[y_num - 1].pos - y0) / h);
	}

	size_t k = 0;
	for (size_t j = 0; j < y_num; ++j)
		for (size_t i = 0; i < x_num; ++i)
		{
			ParticleVar_S2D_CHM &pcl_var = pcl.vars[k];
			pcl_var.x = x_var_infos[i].pos;
			pcl_var.y = y_var_infos[j].pos;
			pcl_var.vol = x_var_infos[i].len * y_var_infos[j].len;
			pcl_var.elem_x_id = x_var_infos[i].elem_id;
			pcl_var.elem_y_id = y_var_infos[j].elem_id;
			cal_shape_func(pcl_var);
			++k;
		}
}
