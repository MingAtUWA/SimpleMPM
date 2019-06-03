#ifndef _NODEVAR_2D_CHM_H_
#define _NODEVAR_2D_CHM_H_

#include "Mesh.h"
#include "Mesh_BG_R2D.h"

// Nodal calculation variables
// For 2 dimensional coupled hydro-mechanics case
struct NodeVar_2D_CHM : public NodeVar
{
	Node_BG_R2D *node;

	Object *object;

	// for soil (mixture) phase
	double ax_s, vx_s, dux_s;
	double ay_s, vy_s, duy_s;
	double m_s;
	double mmx_s, mmy_s;
	double fx_kin_f, fy_kin_f;
	double fx_ext_m, fy_ext_m;
	double fx_int_m, fy_int_m;

	// for fluid phase
	double ax_f, vx_f, dux_f;
	double ay_f, vy_f, duy_f;
	double m_tf;
	double mmx_tf, mmy_tf;
	double fx_ext_tf, fy_ext_tf;
	double fx_int_tf, fy_int_tf;
	double fx_drag_tf, fy_drag_tf;

	// for damping calculation
	double vx_f_normalized, vy_f_normalized;
	double m_f;
	double fx_tf, fy_tf;
	double f_tf_norm;
};

#endif