#ifndef _OBJECT_PARTICLE_2D_CHM_H_
#define _OBJECT_PARTICLE_2D_CHM_H_

#include "Object.h"

#include "BC.h"
#include "ConstitutiveModel.h"
#include "Particle_2D_CHM.h"

struct Object_Particle_2D_CHM : public Object
{
public:
	// particles
	size_t pcl_num;
	Particle_2D_CHM *pcls;

	// boundary conditions
	size_t bfx_num;
	BodyForce *bfxs;
	size_t bfy_num;
	BodyForce *bfys;
	size_t tx_bc_num;
	TractionBC_MPM *tx_bcs;
	size_t ty_bc_num;
	TractionBC_MPM *ty_bcs;

public:
	Object_Particle_2D_CHM() :
		pcl_num(0), pcls(nullptr),
		bfx_num(0), bfxs(nullptr),
		bfy_num(0), bfys(nullptr),
		tx_bc_num(0), tx_bcs(nullptr),
		ty_bc_num(0), ty_bcs(nullptr) {}

	~Object_Particle_2D_CHM()
	{
		if (pcls) delete[] pcls;
		if (bfxs) delete[] bfxs;
		if (bfys) delete[] bfys;
		if (tx_bcs) delete[] tx_bcs;
		if (ty_bcs) delete[] ty_bcs;
		pcl_num = 0;
		pcls = nullptr;
		bfx_num = 0;
		bfxs = nullptr;
		bfy_num = 0;
		bfys = nullptr;
		tx_bc_num = 0;
		tx_bcs = nullptr;
		ty_bc_num = 0;
		ty_bcs = nullptr;
	}
};

#endif