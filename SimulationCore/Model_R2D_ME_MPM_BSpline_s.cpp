#include "SimulationCore_pcp.h"

#include "Model_R2D_ME_MPM_BSpline_s.h"

Model_R2D_ME_MPM_BSpline_s::
	Model_R2D_ME_MPM_BSpline_s() :
	Model("R2D_ME_MPM_BSpline_s"),
	nodes(nullptr), node_num(0),
	pcls(nullptr), pcl_num(0),
	bfxs(nullptr), bfx_num(0),
	bfys(nullptr), bfy_num(0),
	txs(nullptr),  tx_num(0),
	tys(nullptr),  ty_num(0),
	axs(nullptr),  ax_num(0),
	ays(nullptr),  ay_num(0),
	vxs(nullptr),  vx_num(0),
	vys(nullptr),  vy_num(0) {}

int Model_R2D_ME_MPM_BSpline_s::update(void)
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
	ax_num = axs_mem.get_num();
	axs = axs_mem.get_mem();
	ay_num = ays_mem.get_num();
	ays = ays_mem.get_mem();

	// velocity
	vx_num = vxs_mem.get_num();
	vxs = vxs_mem.get_mem();
	vy_num = vys_mem.get_num();
	vys = vys_mem.get_mem();

	return 0;
}

void Model_R2D_ME_MPM_BSpline_s::clear(void)
{
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
	ax_num = 0;
	axs = nullptr;
	axs_mem.clear();
	ay_num = 0;
	ays = nullptr;
	ays_mem.clear();
	// velocity
	vx_num = 0;
	vxs = nullptr;
	vxs_mem.clear();
	vy_num = 0;
	vys = nullptr;
	vys_mem.clear();
}
