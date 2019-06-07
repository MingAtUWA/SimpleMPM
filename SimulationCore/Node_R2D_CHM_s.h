#ifndef _NODE_R2D_CHM_S_H_
#define _NODE_R2D_CHM_S_H_

#include "Mesh.h"

struct Node_R2D_CHM_s : public Node
{
	size_t index_x;
	size_t index_y;

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

	unsigned char cal_flag;

	// for calculate damping
	double vx_f_normalized, vy_f_normalized;
	double m_f;
	double fx_tf, fy_tf;
};

#endif