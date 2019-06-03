#ifndef _PARTICLEVAR_R2D_CHM_H_
#define _PARTICLEVAR_R2D_CHM_H_

#include "Object_Particle_2D_CHM.h"
#include "NodeVar_2D_CHM.h"

// particle calculation variables
// Use R2D as background mesh 
struct ParticleVar_R2D_CHM : public ParticleVar
{
	Particle_2D_CHM *pcl;

	Object_Particle_2D_CHM *object;

	double k_div_miu;

	Element_BG_R2D *elem;
	NodeVar_2D_CHM *node1_var;
	NodeVar_2D_CHM *node2_var;
	NodeVar_2D_CHM *node3_var;
	NodeVar_2D_CHM *node4_var;

	// natural coordinates
	union
	{
		double na_coords[2];
		struct { double xi, eta; };
	};

	// shape function
	double N1;
	double N2;
	double N3;
	double N4;

	// space directives of shape function
	double dN1_dx, dN1_dy;
	double dN2_dx, dN2_dy;
	double dN3_dx, dN3_dy;
	double dN4_dx, dN4_dy;
};

#endif