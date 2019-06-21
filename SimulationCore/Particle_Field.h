#ifndef _PARTICLE_FIELD_H_
#define _PARTICLE_FIELD_H_

// 1D purely mechanical problem
enum class Particle_Field_1D_ME : unsigned short int
{
	x   = 1,
	density = 2,
	m   = 3,
	v   = 4,
	mv  = 5,
	// stress
	s11 = 6,
	// strain
	e11 = 7,
};


// 2D purely mechanical problem
enum class Particle_Field_2D_ME : unsigned short int
{
	x    = 1,
	y    = 2,
	vol  = 3,
	density = 4,
	m    = 5,
	vx   = 6,
	vy   = 7,
	mvx  = 8,
	mvy  = 9,
	//   = 10
	// stress
	s11  = 11,
	s22  = 12,
	s33  = 13,
	s12  = 14,
	s23  = 15,
	s31  = 16,
	//   = 17
	//   = 18
	//   = 19
	//   = 20
	// strain
	e11  = 21,
	e22  = 22,
	e12  = 23,
	// elastic strain
	es11 = 24,
	es22 = 25,
	es12 = 26,
	// plastic strain
	ps11 = 27,
	ps22 = 28,
	ps12 = 29
};


// 2D coupled hydro-mechanical problem
enum class Particle_Field_2D_CHM : unsigned short int
{
	x     = 1,
	y     = 2,
	vol   = 3,
	n     = 4,
	density_s = 5,
	density_f = 6,
	m_s   = 7,
	m_f   = 8,
	//    = 9
	//    = 10
	vx_s  = 11,
	vy_s  = 12,
	vx_f  = 13,
	vy_f  = 14,
	mvx_s = 15,
	mvy_s = 16,
	mvx_f = 17,
	mvy_f = 18,
	//    = 19,
	//    = 20,
	// stress
	s11   = 21,
	s22   = 22,
	s33   = 23,
	s12   = 24,
	s23   = 25,
	s31   = 26,
	// pore pressure
	p     = 27,
	//    = 28
	//    = 29
	//    = 30
	// strain
	e11   = 31,
	e22   = 32,
	e12   = 33,
	// elastic strain
	es11  = 34,
	es22  = 35,
	es12  = 36,
	// plastic strain
	ps11  = 37,
	ps22  = 38,
	ps12  = 39
};


#endif