#ifndef _PARTICLE_R2D_ME_S_H_
#define _PARTICLE_R2D_ME_S_H_

#include "Object.h"

#include "Element_R2D_ME_MPM_s.h"
#include "Node_R2D_ME_s.h"

struct Particle_R2D_ME_s : public Particle
{
public:
	union
	{
		double coords[2];
		struct
		{
			double x;
			double y;
		};
	};

	// mass
	double m;
	double density;

	// momentum
	union
	{
		double momentum[2];
		struct
		{
			double mmx;
			double mmy;
		};
	};
	union
	{
		double stress[6];
		struct
		{
			double s11;
			double s22;
			double s33;
			double s12;
			double s23;
			double s31;
		};
	};
	// total strain
	union
	{
		double strain[3];
		struct
		{
			double e11;
			double e22;
			double e12;
		};
	};
	// elastic strain
	union
	{
		double estrain[3];
		struct
		{
			double es11;
			double es22;
			double es12;
		};
	};
	// plastic strain
	union
	{
		double pstrain[3];
		struct
		{
			double ps11;
			double ps22;
			double ps12;
		};
	};
	
	// constitutive relation
	double E;   // Elastic modulus
	double niu; // Poisson ratio

	bool is_in_mesh;
	Element_R2D_ME_MPM_s *elem;
	Node_R2D_ME_s *node1, *node2, *node3, *node4;

	// total strain
	union
	{
		double dstrain[3];
		struct
		{
			double de11;
			double de22;
			double de12;
		};
	};
	// elastic strain
	union
	{
		double destrain[3];
		struct
		{
			double des11;
			double des22;
			double des12;
		};
	};
	// plastic strain
	union
	{
		double dpstrain[3];
		struct
		{
			double dps11;
			double dps22;
			double dps12;
		};
	};
	// for Jaumann rate
	double dw12;

	union // initial particle position
	{
		double coords_ori[2];
		struct { double x_ori, y_ori; };
	};
	union
	{
		double displacement[2];
		struct { double ux, uy; };
	};

	// natural coordinates
	union
	{
		double na_coords[2];
		struct
		{
			double xi;
			double eta;
		};
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

public: // critical time step at particle
	inline double critical_time_step(double elem_char_len)
	{
		double Ec = (1.0 - niu) / ((1.0 + niu) * (1.0 - 2.0 * niu)) * E;
		return elem_char_len / sqrt(Ec / density);
	}
};

#endif