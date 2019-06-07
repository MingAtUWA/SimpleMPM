#ifndef _PARTICLE_R2D_CHM_S_H_
#define _PARTICLE_R2D_CHM_S_H_

#include "Object.h"

#include "Element_R2D_CHM_MPM_s.h"
#include "Node_R2D_CHM_s.h"

struct Particle_R2D_CHM_s : public Particle
{
public:
	union
	{
		double coords[2];
		struct { double x, y; };
	};

	// volume represented by this particle
	double vol;
	// porosity
	double n;
	// density
	double density_s;
	double density_f;

	// velocity
	union // solid phase
	{
		double velocity_s[2];
		struct { double vx_s, vy_s; };
	};
	union // fluid phase
	{
		double velocity_f[2];
		struct { double vx_f, vy_f; };
	};
	// effective stress
	// take tensile as positive
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
	// pore pressure
	// take compression as negative
	double p;
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

	union // initial particle position
	{
		double coords_ori[2];
		struct { double x_ori, y_ori; };
	};
	union
	{
		double displacement_s[2];
		struct { double ux_s, uy_s; };
	};
	union
	{
		double displacement_f[2];
		struct { double ux_f, uy_f; };
	};

public: // Critical time step at particle
	// Non-combined version by Mieremet (2015) report 
	inline double critical_time_step1(double elem_char_len)
	{
		double density_bar = (1.0 - n)*density_s + n*density_f + (1.0 / n - 2.0)*density_f;
		double Ec = (1.0 - niu) / ((1.0 + niu) * (1.0 - 2.0 * niu)) * E;
		double dt_cr1 = elem_char_len / sqrt((Ec + Kf / n) / density_bar);
		double dt_cr2 = 2.0 * density_bar * k/miu;
		return dt_cr1 < dt_cr2 ? dt_cr1 : dt_cr2;
	}

	// Non-combined version by Mieremet (2015) Master Thesis
	inline double critical_time_step2(double elem_char_len)
	{
		double density_avg, Ec, el2, tmp1, tmp2;
		double a, b, d;
		double dt_cr;
		density_avg = (1.0-n)*density_s + n*density_f;
		Ec = (1.0 - niu) / ((1.0 + niu) * (1.0 - 2.0 * niu)) * E;
		el2 = elem_char_len * elem_char_len;
		tmp1 = (1.0 - n) * density_s * density_f;
		a = n * density_avg * miu / (k * tmp1);
		b = 4.0 * (n*density_avg*Kf + (1.0 - 2.0*n)*density_f*Kf + n*density_f*Ec) / (n * el2 * tmp1);
		d = 16.0 * Ec * Kf / (el2*el2 * tmp1);
		tmp2 = b + sqrt(b*b - 4.0*d);
		dt_cr = (-2.0*a + sqrt(4.0*a*a + 8.0*tmp2)) / tmp2;
		return dt_cr;
	}

public:
	// Constitutive model
	double E;   // Elastic modulus
	double niu; // Poisson ratio
	double Kf;  // Bulk modulus of water
	double k;   // Permeability
	double miu; // Dynamic viscosity

	bool is_in_mesh;
	Element_R2D_CHM_MPM_s *elem;
	Node_R2D_CHM_s *node1, *node2, *node3, *node4;

	double k_div_miu;

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