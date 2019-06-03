#include "SimulationCore_pcp.h"

#include "Mesh_BG_R2D_ME.h"

Mesh_BG_R2D_ME::Mesh_BG_R2D_ME() :
	ax_bc_num(0), ax_bcs(nullptr),
	ay_bc_num(0), ay_bcs(nullptr),
	vx_bc_num(0), vx_bcs(nullptr),
	vy_bc_num(0), vy_bcs(nullptr) {}

Mesh_BG_R2D_ME::~Mesh_BG_R2D_ME() { clear(); }

void Mesh_BG_R2D_ME::update(void)
{
	Mesh_R2D<Node_BG_R2D_ME, Element_BG_R2D_ME>::update();

	// update acceleration boundary conditions
	ax_bc_num = ax_bcs_mem.get_num();
	ax_bcs = ax_bc_num ? ax_bcs_mem.get_mem() : nullptr;
	ay_bc_num = ay_bcs_mem.get_num();
	ay_bcs = ay_bc_num ? ay_bcs_mem.get_mem() : nullptr;
	// update vel
	vx_bc_num = vx_bcs_mem.get_num();
	vx_bcs = vx_bc_num ? vx_bcs_mem.get_mem() : nullptr;
	vy_bc_num = vy_bcs_mem.get_num();
	vy_bcs = vy_bc_num ? vy_bcs_mem.get_mem() : nullptr;
}

void Mesh_BG_R2D_ME::clear(void)
{
	Mesh_R2D<Node_BG_R2D_ME, Element_BG_R2D_ME>::clear();

	ax_bc_num = 0;
	ax_bcs = nullptr;
	ay_bc_num = 0;
	ay_bcs = nullptr;
	vx_bc_num = 0;
	vx_bcs = nullptr;
	vy_bc_num = 0;
	vy_bcs = nullptr;
	ax_bcs_mem.reset();
	ay_bcs_mem.reset();
	vx_bcs_mem.reset();
	vy_bcs_mem.reset();
}