#ifndef _MODEL_R2D_CHM_MPM_H_
#define _MODEL_R2D_CHM_MPM_H_

#include "ItemStack.hpp"
#include "ItemList.hpp"
#include "Model.h"

#include "BC.h"
#include "Mesh_BG_R2D_ME.h"
#include "Object_Particle_2D_CHM.h"
#include "NodeVar_2D_CHM.h"
#include "ParticleVar_R2D_CHM.h"

class Step_R2D_CHM_MPM;

struct Model_R2D_CHM_MPM : public Model
{
	friend Step_R2D_CHM_MPM;

public:
	Mesh_BG_R2D mesh;

	// boundary conditions on mesh
	size_t ax_s_bc_num, ay_s_bc_num;
	AccelerationBC *ax_s_bcs, *ay_s_bcs;
	size_t vx_s_bc_num, vy_s_bc_num;
	VelocityBC *vx_s_bcs, *vy_s_bcs;

	size_t ax_f_bc_num, ay_f_bc_num;
	AccelerationBC *ax_f_bcs, *ay_f_bcs;
	size_t vx_f_bc_num, vy_f_bc_num;
	VelocityBC *vx_f_bcs, *vy_f_bcs;
	
	size_t object_num;
	MemoryUtilities::ItemList<Object_Particle_2D_CHM> objects;

public:
	Model_R2D_CHM_MPM();
	~Model_R2D_CHM_MPM();
	
	inline void set_object_num(size_t num) { objects.set_default_pool_item_num(num); }
	inline Object_Particle_2D_CHM *add_object(void)
	{
		++object_num;
		return objects.alloc();
	}

	void cal_shape_function(ParticleVar_R2D_CHM *ppcl_var);

protected:
	MemoryUtilities::ItemStack<NodeVar_2D_CHM> node_vars;
	MemoryUtilities::ItemList<ParticleVar_R2D_CHM> pcl_vars;
};

#endif