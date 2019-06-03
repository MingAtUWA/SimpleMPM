#ifndef _PARTICLEVAR_R2D_ME_H_
#define _PARTICLEVAR_R2D_ME_H_

#include "Object.h"

struct Particle_2D_ME;
struct Element_BG_R2D_ME;
struct NodeVar_2D_ME;

// particle calculation variables
// Use R2D as background mesh 
struct ParticleVar_R2D_ME : public ParticleVar
{
	Particle_2D_ME *pcl;

	// incremental total strain
	union
	{
		double dstrain[3];
		struct { double de11, de22, de12; };
	};
	// incremental  elastic strain
	union
	{
		double destrain[3];
		struct { double des11, des22, des12; };
	};
	// incremental plastic strain
	union
	{
		double dpstrain[3];
		struct { double dps11, dps22, dps12; };
	};
	// for Jaumann rate
	double dw12;

	// volume
	double vol;

	// In which element
	Element_BG_R2D_ME *elem;
	union
	{
		NodeVar_2D_ME *node_var[4];
		struct
		{
			NodeVar_2D_ME *node1_var, *node2_var;
			NodeVar_2D_ME *node3_var, *node4_var;
		};
	};
	// used by element
	ParticleVar_R2D_ME *next_by_elem;

	// natural coordinates
	union
	{
		double na_coords[2];
		struct { double xi, eta; };
	};

	// shape function
	union
	{
		double N[4];
		struct { double N1, N2, N3, N4; };
	};

	// space directives of shape function
	union
	{
		double dN_dx[4];
		struct { double dN1_dx, dN2_dx, dN3_dx, dN4_dx; };
	};
	union
	{
		double dN_dy[4];
		struct { double dN1_dy, dN2_dy, dN3_dy, dN4_dy; };
	};
};

#endif