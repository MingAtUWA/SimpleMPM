#ifndef _OBJECT_PARTICLE_2D_ME_H_
#define _OBJECT_PARTICLE_2D_ME_H_

#include "ItemArray.hpp"
#include "Object_Particle.hpp"

#include "BC.h"
#include "ConstitutiveModel.h"
#include "Particle_2D_ME.h"

struct Object_Particle_2D_ME : public Object_Particle<Particle_2D_ME>
{
public:
	// boundary conditions
	size_t bfx_num;
	BodyForce *bfxs;
	size_t bfy_num;
	BodyForce *bfys;
	size_t tx_bc_num;
	TractionBC_MPM *tx_bcs;
	size_t ty_bc_num;
	TractionBC_MPM *ty_bcs;

protected:
	MemoryUtilities::ItemArray<BodyForce> bfxs_mem;
	MemoryUtilities::ItemArray<BodyForce> bfys_mem;
	MemoryUtilities::ItemArray<TractionBC_MPM> tx_bcs_mem;
	MemoryUtilities::ItemArray<TractionBC_MPM> ty_bcs_mem;

public:
	Object_Particle_2D_ME() :
		bfx_num(0), bfxs(nullptr),
		bfy_num(0), bfys(nullptr),
		tx_bc_num(0), tx_bcs(nullptr),
		ty_bc_num(0), ty_bcs(nullptr) {}

	~Object_Particle_2D_ME() { clear(); }

	inline void set_bfx_num(size_t num) { bfxs_mem.reserve(num); }
	inline void add_bfx(BodyForce &bf) { bfxs_mem.add(bf); }
	inline void set_bfy_num(size_t num) { bfys_mem.reserve(num); }
	inline void add_bfy(BodyForce &bf) { bfys_mem.add(bf); }
	inline void set_tx_bc_num(size_t num) { tx_bcs_mem.reserve(num); }
	inline void add_tx_bc(TractionBC_MPM &t) { tx_bcs_mem.add(t); }
	inline void set_ty_bc_num(size_t num) { ty_bcs_mem.reserve(num); }
	inline void add_ty_bc(TractionBC_MPM &t) { ty_bcs_mem.add(t); }

	void update(void)
	{
		Object_Particle<Particle_2D_ME>::update();
		bfx_num = bfxs_mem.get_num();
		bfxs = bfx_num ? bfxs_mem.get_mem() : nullptr;
		bfy_num = bfys_mem.get_num();
		bfys = bfy_num ? bfys_mem.get_mem() : nullptr;
		tx_bc_num = tx_bcs_mem.get_num();
		tx_bcs = tx_bc_num ? tx_bcs_mem.get_mem() : nullptr;
		ty_bc_num = ty_bcs_mem.get_num();
		ty_bcs = ty_bc_num ? ty_bcs_mem.get_mem() : nullptr;
	}
	void clear(void)
	{
		Object_Particle<Particle_2D_ME>::clear();
		bfx_num = 0;
		bfxs = nullptr;
		bfxs_mem.reset();
		bfy_num = 0;
		bfys = nullptr;
		bfys_mem.reset();
		tx_bc_num = 0;
		tx_bcs = nullptr;
		tx_bcs_mem.reset();
		ty_bc_num = 0;
		ty_bcs = nullptr;
		ty_bcs_mem.reset();
	}
};

#endif