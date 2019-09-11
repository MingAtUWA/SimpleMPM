#include "SimulationCore_pcp.h"

#include "Model_S2D_CHM_MPM_s_Mix.h"

Model_S2D_CHM_MPM_s_Mix::Model_S2D_CHM_MPM_s_Mix() :
	Model("Model_S2D_CHM_MPM_s_Mix"),
	nodes(nullptr), node_x_num(0), node_y_num(0), node_num(0),
	elems(nullptr), elem_x_num(0), elem_y_num(0), elem_num(0),
	pcls(nullptr), pcl_num(0),
	bfx_num(0), bfy_num(0), bfxs(nullptr), bfys(nullptr),
	tx_num(0),  ty_num(0),  txs(nullptr),  tys(nullptr),
	asx_num(0), asy_num(0), asxs(nullptr), asys(nullptr),
	vsx_num(0), vsy_num(0), vsxs(nullptr), vsys(nullptr),
	afx_num(0), afy_num(0), afxs(nullptr), afys(nullptr),
	vfx_num(0), vfy_num(0), vfxs(nullptr), vfys(nullptr) {}

Model_S2D_CHM_MPM_s_Mix::~Model_S2D_CHM_MPM_s_Mix()
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
