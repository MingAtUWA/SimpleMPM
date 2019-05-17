#ifndef _NODE_R2D_ME_H_
#define _NODE_R2D_ME_H_

struct Node_R2D_ME
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

	/*
	cal_flag value and meaning:
		1) 0 : no need for calculation;
		2) 1 : calculate in normal way;
		3) 2 : calculate with equal weight.
	*/
	unsigned char cal_flag;
};

#endif