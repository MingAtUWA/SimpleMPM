#include "SimulationCore_pcp.h"

#include "Model_1D_ME_MPM_BSpline_s.h"

Model_1D_ME_MPM_BSpline_s::
	Model_1D_ME_MPM_BSpline_s() :
	nodes(nullptr), node_num(0),
	elems(nullptr), elem_num(0),
	pcls(nullptr), pcl_num(0),
	bfs(nullptr), bf_num(0),
	tbcs(nullptr), tbc_num(0),
	abcs(nullptr), abc_num(0),
	vbcs(nullptr), vbc_num(0) {}

int Model_1D_ME_MPM_BSpline_s::update(void)
{
	// mesh
	nodes_mem.clear();
	nodes_mem.reserve(node_num);
	nodes = nodes_mem.get_mem();
	for (size_t i = 0; i < node_num; i++)
	{
		nodes[i].index = i;
	}
	elems_mem.clear();
	elems_mem.reserve(elem_num);
	elems = elems_mem.get_mem();
	for (size_t i = 0; i < elem_num; i++)
	{
		elems[i].index = i;
	}
	// particles
	pcl_num = pcls_mem.get_num();
	pcls = pcls_mem.get_mem();
	// body force
	bf_num = bfs_mem.get_num();
	bfs = bfs_mem.get_mem();
	// traction boundary
	tbc_num = tbcs_mem.get_num();
	tbcs = tbcs_mem.get_mem();
	// acceleration
	abc_num = abcs_mem.get_num();
	abcs = abcs_mem.get_mem();
	// velocity
	vbc_num = vbcs_mem.get_num();
	vbcs = vbcs_mem.get_mem();

	return 0;
}

void Model_1D_ME_MPM_BSpline_s::clear(void)
{
	node_num = 0;
	nodes = nullptr;
	nodes_mem.clear();
	elem_num = 0;
	elems = nullptr;
	elems_mem.clear();
	// particles
	pcl_num = 0;
	pcls = nullptr;
	pcls_mem.clear();
	// body force
	bf_num = 0;
	bfs = nullptr;
	bfs_mem.clear();
	// traction boundary
	tbc_num = 0;
	tbcs = nullptr;
	tbcs_mem.clear();
	// acceleration
	abc_num = 0;
	abcs = nullptr;
	abcs_mem.clear();
	// velocity
	vbc_num = 0;
	vbcs = nullptr;
	vbcs_mem.clear();
}
