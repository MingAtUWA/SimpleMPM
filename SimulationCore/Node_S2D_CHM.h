#ifndef _NODE_S2D_CHM_H_
#define _NODE_S2D_CHM_H_

struct Node_S2D_CHM
{
	size_t index_x, index_y;

	// for soil (mixture) phase
	double m_s;
	double ax_s, ay_s;
	double vx_s, vy_s;
	double fx_ext_m, fy_ext_m;
	double fx_int_m, fy_int_m;
	double fx_kin_f, fy_kin_f;
	
	// for fluid phase
	double m_tf;
	double ax_f, ay_f;
	double vx_f, vy_f;
	double fx_ext_tf, fy_ext_tf;
	double fx_int_tf, fy_int_tf;
	double fx_drag_tf, fy_drag_tf;
};

#endif