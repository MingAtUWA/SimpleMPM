#ifndef _PARTICLE_2D_ME_H_
#define _PARTICLE_2D_ME_H_

#include "Object.h"
#include "ConstitutiveModel.h"

struct Object_Particle_2D_ME;

struct Particle_2D_ME : public Particle
{
public:
	// to which object this particle belongs
	Object_Particle_2D_ME *object;

	union
	{
		double coords[2];
		struct { double x, y; };
	};

	/* ------------------------------------------
	 * The two unions below are used to improve floating point 
	 * accuracy when ux << x uy << y.
	 * -----------------------------------------*/
	union
	{
		double coords_ori[2];
		struct { double x_ori, y_ori; };
	};
	union
	{
		double displacement[2];
		struct { double ux, uy; };
	};

	// mass
	double m;
	double density;

	// velocity
	union
	{
		double velocity[2];
		struct { double vx, vy; };
	};
	union
	{
		double stress[6];
		struct { double s11, s22, s33, s12, s23, s31; };
	};
	// total strain
	union
	{
		double strain[3];
		struct { double e11, e22, e12; };
	};
	// elastic strain
	union
	{
		double estrain[3];
		struct { double es11, es22, es12; };
	};
	// plastic strain
	union
	{
		double pstrain[3];
		struct { double ps11, ps22, ps12; };
	};

	// Constitutive Model
	ConstitutiveModel *cm;

	// Calculation variables
	ParticleVar *var;

public:
	void init(void)
	{
		x = 0.0;
		y = 0.0;
		x_ori = 0.0;
		y_ori = 0.0;
		ux = 0.0;
		uy = 0.0;
		m = 0.0;
		density = 0.0;
		vx = 0.0;
		vy = 0.0;
		s11 = 0.0;
		s22 = 0.0;
		s33 = 0.0;
		s12 = 0.0;
		s23 = 0.0;
		s31 = 0.0;
		e11 = 0.0;
		e22 = 0.0;
		e12 = 0.0;
		es11 = 0.0;
		es22 = 0.0;
		es12 = 0.0;
		ps11 = 0.0;
		ps22 = 0.0;
		ps12 = 0.0;
		cm = nullptr;
		var = nullptr;
	}

public: // critical time step at particle
	inline double critical_time_step(double elem_char_len)
	{
		double Ec = (1.0 - niu) / ((1.0 + niu) * (1.0 - 2.0 * niu)) * E;
		return elem_char_len / sqrt(Ec / density);
	}

public: // --------------- Obsoleted -----------------
	// constitutive relation
	double E;   // Elastic modulus
	double niu; // Poisson ratio
};

#endif