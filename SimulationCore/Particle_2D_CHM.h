#ifndef _PARTICLE_2D_CHM_H_
#define _PARTICLE_2D_CHM_H_

#include "Object.h"
#include "ConstitutiveModel.h"
#include "Mesh_BG_R2D.h"

struct Particle_2D_CHM : public Particle
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

	// Constitutive Model
	ConstitutiveModel *cm;

	// Calculation variables
	ParticleVar *var;

public:
	void init(void)
	{
		x = 0.0;
		y = 0.0;
		vol = 0.0;
		n = 0.0;
		density_s = 0.0;
		density_f = 0.0;
		vx_s = 0.0;
		vy_s = 0.0;
		vx_f = 0.0;
		vy_f = 0.0;
		s11 = 0.0;
		s22 = 0.0;
		s33 = 0.0;
		s12 = 0.0;
		s23 = 0.0;
		s31 = 0.0;
		p = 0.0;
		e11 = 0.0;
		e22 = 0.0;
		e12 = 0.0;
		es11 = 0.0;
		es22 = 0.0;
		es12 = 0.0;
		ps11 = 0.0;
		ps22 = 0.0;
		ps12 = 0.0;
		x_ori = 0.0;
		y_ori = 0.0;
		ux_s = 0.0;
		uy_s = 0.0;
		ux_f = 0.0;
		uy_f = 0.0;
		cm = nullptr;
		var = nullptr;
	}

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

	/* ---------------- content below obsoleted ----------------- */
public:
	// Constitutive model
	double E;   // Elastic modulus
	double niu; // Poisson ratio
	double Kf;  // Bulk modulus of water
	double k;   // Permeability
	double miu; // Dynamic viscosity
};

#endif