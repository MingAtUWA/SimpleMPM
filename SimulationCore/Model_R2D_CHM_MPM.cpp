#include "SimulationCore_pcp.h"

#include "Model_R2D_CHM_MPM.h"

Model_R2D_CHM_MPM::Model_R2D_CHM_MPM() : 
	object_num(0),
	ax_s_bc_num(0), ax_s_bcs(nullptr),
	ay_s_bc_num(0), ay_s_bcs(nullptr),
	vx_s_bc_num(0), vx_s_bcs(nullptr),
	vy_s_bc_num(0), vy_s_bcs(nullptr),
	ax_f_bc_num(0), ax_f_bcs(nullptr),
	ay_f_bc_num(0), ay_f_bcs(nullptr),
	vx_f_bc_num(0), vx_f_bcs(nullptr),
	vy_f_bc_num(0), vy_f_bcs(nullptr) {}

Model_R2D_CHM_MPM::~Model_R2D_CHM_MPM()
{
	if (ax_s_bcs) delete[] ax_s_bcs;
	ax_s_bc_num = 0;
	ax_s_bcs = nullptr;
	if (ay_s_bcs) delete[] ay_s_bcs;
	ay_s_bc_num = 0;
	ay_s_bcs = nullptr;
	if (vx_s_bcs) delete[] vx_s_bcs;
	vx_s_bc_num = 0;
	vx_s_bcs = nullptr;
	if (vy_s_bcs) delete[] vy_s_bcs;
	vy_s_bc_num = 0;
	vy_s_bcs = nullptr;
	if (ax_f_bcs) delete[] ax_f_bcs;
	ax_f_bc_num = 0;
	ax_f_bcs = nullptr;
	if (ay_f_bcs) delete[] ay_f_bcs;
	ay_f_bc_num = 0;
	ay_f_bcs = nullptr;
	if (vx_f_bcs) delete[] vx_f_bcs;
	vx_f_bc_num = 0;
	vx_f_bcs = nullptr;
	if (vy_f_bcs) delete[] vy_f_bcs;
	vy_f_bc_num = 0;
	vy_f_bcs = nullptr;
}

void Model_R2D_CHM_MPM::cal_shape_function(ParticleVar_R2D_CHM *ppcl_var)
{
	Element_BG_R2D *pelem;
	Particle_2D_CHM *ppcl;
	double xLower, xUpper, yLower, yUpper;
	double xMiddle, xHalfLength, yMiddle, yHalfLength;
	double x1, x2, x3, x4, y1, y2, y3, y4;
	double dN1_dxi, dN1_deta, dN2_dxi, dN2_deta;
	double dN3_dxi, dN3_deta, dN4_dxi, dN4_deta;
	double dx_dxi, dx_deta, dy_dxi, dy_deta;
	double dxi_dx, dxi_dy, deta_dx, deta_dy;
	double Jdet;

	pelem = ppcl_var->elem;
	ppcl = ppcl_var->pcl;

	xLower = mesh.node_x_coords[pelem->index_x];
	xUpper = mesh.node_x_coords[pelem->index_x + 1];
	yLower = mesh.node_y_coords[pelem->index_y];
	yUpper = mesh.node_y_coords[pelem->index_y + 1];

	xHalfLength = (xUpper - xLower) / 2.0;
	yHalfLength = (yUpper - yLower) / 2.0;
	xMiddle = (xUpper + xLower) / 2.0;
	yMiddle = (yUpper + yLower) / 2.0;

	ppcl_var->xi = (ppcl->x - xMiddle) / xHalfLength;
	ppcl_var->eta = (ppcl->y - yMiddle) / yHalfLength;

	ppcl_var->N1 = _N1_R2D(ppcl_var->xi, ppcl_var->eta);
	if (ppcl_var->N1 < N_tol) ppcl_var->N1 = N_tol;
	ppcl_var->N2 = _N2_R2D(ppcl_var->xi, ppcl_var->eta);
	if (ppcl_var->N2 < N_tol) ppcl_var->N2 = N_tol;
	ppcl_var->N3 = _N3_R2D(ppcl_var->xi, ppcl_var->eta);
	if (ppcl_var->N3 < N_tol) ppcl_var->N3 = N_tol;
	ppcl_var->N4 = _N4_R2D(ppcl_var->xi, ppcl_var->eta);
	if (ppcl_var->N4 < N_tol) ppcl_var->N4 = N_tol;

	dN1_dxi = _dN1_dxi_R2D(ppcl_var->xi, ppcl_var->eta);
	dN1_deta = _dN1_deta_R2D(ppcl_var->xi, ppcl_var->eta);
	dN2_dxi = _dN2_dxi_R2D(ppcl_var->xi, ppcl_var->eta);
	dN2_deta = _dN2_deta_R2D(ppcl_var->xi, ppcl_var->eta);
	dN3_dxi = _dN3_dxi_R2D(ppcl_var->xi, ppcl_var->eta);
	dN3_deta = _dN3_deta_R2D(ppcl_var->xi, ppcl_var->eta);
	dN4_dxi = _dN4_dxi_R2D(ppcl_var->xi, ppcl_var->eta);
	dN4_deta = _dN4_deta_R2D(ppcl_var->xi, ppcl_var->eta);

	/*
	The Jacobian matrix:
	dx_dxi, dx_deta,
	dy_dxi, dy_deta,
	*/
	x1 = x4 = mesh.node_x_coords[pelem->index_x];
	x2 = x3 = mesh.node_x_coords[pelem->index_x + 1];
	y1 = y2 = mesh.node_y_coords[pelem->index_y];
	y3 = y4 = mesh.node_y_coords[pelem->index_y + 1];
	dx_dxi  = dN1_dxi  * x1 + dN2_dxi  * x2 + dN3_dxi  * x3 + dN4_dxi  * x4;
	dx_deta = dN1_deta * x1 + dN2_deta * x2 + dN3_deta * x3 + dN4_deta * x4;
	dy_dxi  = dN1_dxi  * y1 + dN2_dxi  * y2 + dN3_dxi  * y3 + dN4_dxi  * y4;
	dy_deta = dN1_deta * y1 + dN2_deta * y2 + dN3_deta * y3 + dN4_deta * y4;

	// determinant of Jacobian matrix
	Jdet = dx_dxi * dy_deta - dx_deta * dy_dxi;
	/* -----------------------------------
	Inverse of Jacobian matrix:
	dxi_dx,  dxi_dy,
	deta_dx, deta_dy,
	----------------------------------- */
	dxi_dx = dy_deta / Jdet;
	dxi_dy = -dx_deta / Jdet;
	deta_dx = -dy_dxi / Jdet;
	deta_dy = dx_dxi / Jdet;

	ppcl_var->dN1_dx = dN1_dxi * dxi_dx + dN1_deta * deta_dx;
	ppcl_var->dN1_dy = dN1_dxi * dxi_dy + dN1_deta * deta_dy;
	ppcl_var->dN2_dx = dN2_dxi * dxi_dx + dN2_deta * deta_dx;
	ppcl_var->dN2_dy = dN2_dxi * dxi_dy + dN2_deta * deta_dy;
	ppcl_var->dN3_dx = dN3_dxi * dxi_dx + dN3_deta * deta_dx;
	ppcl_var->dN3_dy = dN3_dxi * dxi_dy + dN3_deta * deta_dy;
	ppcl_var->dN4_dx = dN4_dxi * dxi_dx + dN4_deta * deta_dx;
	ppcl_var->dN4_dy = dN4_dxi * dxi_dy + dN4_deta * deta_dy;
}