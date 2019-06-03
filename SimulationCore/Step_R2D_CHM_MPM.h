#ifndef _STEP_R2D_CHM_MPM_H_
#define _STEP_R2D_CHM_MPM_H_

#include "Step.h"
#include "Model_R2D_CHM_MPM.h"

struct Model_R2D_CHM_MPM;

int solve_substep_R2D_CHM_MPM(void *_self);

// for single object only
class Step_R2D_CHM_MPM : public Step
{
protected:
	Model_R2D_CHM_MPM *model;
	
public:
	Step_R2D_CHM_MPM();
	~Step_R2D_CHM_MPM();

	inline void set_model(Model_R2D_CHM_MPM *md)
	{
		Step::set_model(md);
		model = md;
	}

	// Restart from previous step
	inline void set_prev_step(Step_R2D_CHM_MPM *prev_step)
	{
		Step::set_prev_step(prev_step);
		model = prev_step->model;
	}

protected:
	Mesh_BG_R2D *mesh;
	MemoryUtilities::ItemList<Object_Particle_2D_CHM> *objects;
	MemoryUtilities::ItemStack<NodeVar_2D_CHM> *node_vars;
	MemoryUtilities::ItemList<ParticleVar_R2D_CHM> *pcl_vars;

protected:
	int init(void);	
	friend int solve_substep_R2D_CHM_MPM(void *_self);
	int finalize(void);

	int init_per_substep(void);
	int map_to_nodes_and_cal_internal_force(void);
	int cal_external_force(void);
	int cal_fluid_phase(void);
	int cal_mixture_phase(void);
	int contact_calculation(void);
	int map_to_particles_and_update_particle_variables(void);
};


#endif