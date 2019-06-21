#include "SimulationCore_pcp.h"

#include "Model_R2D_CHM_MPM_BSpline_s.h"

Model_R2D_CHM_MPM_BSpline_s::
	Model_R2D_CHM_MPM_BSpline_s() :
	Model("R2D_CHM_MPM_BSpline_s"),
	nodes(nullptr), node_num(0),
	pcls(nullptr), pcl_num(0),
	bfxs(nullptr), bfx_num(0),
	bfys(nullptr), bfy_num(0),
	txs(nullptr),  tx_num(0),
	tys(nullptr),  ty_num(0),
	asxs(nullptr), asx_num(0),
	asys(nullptr), asy_num(0),
	afxs(nullptr), afx_num(0),
	afys(nullptr), afy_num(0),
	vsxs(nullptr), vsx_num(0),
	vsys(nullptr), vsy_num(0),
	vfxs(nullptr), vfx_num(0),
	vfys(nullptr), vfy_num(0) {}

int Model_R2D_CHM_MPM_BSpline_s::update(void)
{
	// mesh
	nodes_mem.clear();
	nodes_mem.reserve(node_num);
	nodes = nodes_mem.get_mem();
	
	// particles
	pcl_num = pcls_mem.get_num();
	pcls = pcls_mem.get_mem();

	// body force
	bfx_num = bfxs_mem.get_num();
	bfxs = bfxs_mem.get_mem();
	bfy_num = bfys_mem.get_num();
	bfys = bfys_mem.get_mem();

	// boundary traction
	tx_num = txs_mem.get_num();
	txs = txs_mem.get_mem();
	ty_num = tys_mem.get_num();
	tys = tys_mem.get_mem();

	// acceleration
	asx_num = asxs_mem.get_num();
	asxs = asxs_mem.get_mem();
	asy_num = asys_mem.get_num();
	asys = asys_mem.get_mem();
	afx_num = afxs_mem.get_num();
	afxs = afxs_mem.get_mem();
	afy_num = afys_mem.get_num();
	afys = afys_mem.get_mem();

	// velocity
	vsx_num = vsxs_mem.get_num();
	vsxs = vsxs_mem.get_mem();
	vsy_num = vsys_mem.get_num();
	vsys = vsys_mem.get_mem();
	vfx_num = vfxs_mem.get_num();
	vfxs = vfxs_mem.get_mem();
	vfy_num = vfys_mem.get_num();
	vfys = vfys_mem.get_mem();

	return 0;
}

void Model_R2D_CHM_MPM_BSpline_s::clear(void)
{
	// mesh
	node_x_num = 0;
	node_y_num = 0;
	node_num = 0;
	nodes = nullptr;
	nodes_mem.clear();

	// particles
	pcl_num = 0;
	pcls = nullptr;
	pcls_mem.clear();

	// body force
	bfx_num = 0;
	bfxs = nullptr;
	bfxs_mem.clear();
	bfy_num = 0;
	bfys = nullptr;
	bfys_mem.clear();

	// traction boundary
	tx_num = 0;
	txs = nullptr;
	txs_mem.clear();
	ty_num = 0;
	tys = nullptr;
	tys_mem.clear();

	// acceleration
	asx_num = 0;
	asxs = nullptr;
	asxs_mem.clear();
	asy_num = 0;
	asys = nullptr;
	asys_mem.clear();
	afx_num = 0;
	afxs = nullptr;
	afxs_mem.clear();
	afy_num = 0;
	afys = nullptr;
	afys_mem.clear();

	// velocity
	vsx_num = 0;
	vsxs = nullptr;
	vsxs_mem.clear();
	vsy_num = 0;
	vsys = nullptr;
	vsys_mem.clear();
	vfx_num = 0;
	vfxs = nullptr;
	vfxs_mem.clear();
	vfy_num = 0;
	vfys = nullptr;
	vfys_mem.clear();
}
