#ifndef _MODEL_R2D_ME_MPM_H_
#define _MODEL_R2D_ME_MPM_H_

#include "ItemStack.hpp"
#include "ItemList.hpp"
#include "ItemBuffer.hpp"
#include "Model.h"

#include "BC.h"
#include "Mesh_BG_R2D_ME.h"
#include "Object_Particle_2D_ME.h"
#include "NodeVar_2D_ME.h"
#include "ParticleVar_R2D_ME.h"

class Step_R2D_ME_MPM;

struct Model_R2D_ME_MPM : public Model
{
	friend Step_R2D_ME_MPM;

public:
	Mesh_BG_R2D_ME mesh;

	size_t object_num;
	typedef MemoryUtilities::ItemList<Object_Particle_2D_ME> ObjectList;
	ObjectList objects;

public:
	Model_R2D_ME_MPM();
	~Model_R2D_ME_MPM();
	
	inline void set_object_num(size_t num) { objects.set_default_pool_item_num(num); }
	inline Object_Particle_2D_ME *make_object(void)
	{
		Object_Particle_2D_ME *pobj = objects.alloc();
		new (pobj) Object_Particle_2D_ME;
		++object_num;
		return pobj;
	}

	void cal_shape_function(ParticleVar_R2D_ME *ppcl_var);

protected:
	typedef MemoryUtilities::ItemStack<NodeVar_2D_ME> NodeVarStack;
	NodeVarStack node_vars;
	typedef MemoryUtilities::ItemList<ParticleVar_R2D_ME> PclVarList;
	PclVarList pcl_vars;
	typedef MemoryUtilities::ItemBuffer<Node_Contact_Var_2D_ME> ContactVarBuffer;
	ContactVarBuffer contact_vars;
};

#endif