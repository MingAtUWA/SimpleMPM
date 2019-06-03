#ifndef _NODE_R2D_ME_S_H_
#define _NODE_R2D_ME_S_H_

#include "Mesh.h"

struct Node_R2D_ME_s : public Node
{
	size_t index_x;
	size_t index_y;

	double ax, vx, dux;
	double ay, vy, duy;
	// mass
	double m;
	// increment of momentum
	double dmmx, dmmy;
	// momentum
	double mmx, mmy;
	// external force
	double fx_ext_m, fy_ext_m;
	// internal force
	double fx_int_m, fy_int_m;
	
	unsigned char cal_flag;
};

#endif