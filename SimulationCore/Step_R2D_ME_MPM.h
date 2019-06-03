#ifndef _STEP_R2D_ME_MPM_H_
#define _STEP_R2D_ME_MPM_H_

#include "Step.h"
#include "Model_R2D_ME_MPM.h"

int solve_substep_R2D_ME_MPM(void *_self);

// for single object only
class Step_R2D_ME_MPM : public Step
{
protected:
	Model_R2D_ME_MPM *model;
	
public:
	Step_R2D_ME_MPM();
	~Step_R2D_ME_MPM();

	inline void set_model(Model_R2D_ME_MPM *md)
	{
		Step::set_model(md);
		model = md;
	}

	// Restart from previous step
	inline void set_prev_step(Step_R2D_ME_MPM *prev_step)
	{
		Step::set_prev_step(prev_step);
		model = prev_step->model;
	}

protected:
	Mesh_BG_R2D_ME *mesh;
	Model_R2D_ME_MPM::ObjectList *objects;
	Model_R2D_ME_MPM::NodeVarStack *node_vars;
	Model_R2D_ME_MPM::PclVarList *pcl_vars;
	Model_R2D_ME_MPM::ContactVarBuffer *contact_vars;

protected:
	int init(void);	
	friend int solve_substep_R2D_ME_MPM(void *_self);
	int finalize(void);

	int init_per_substep(void);
	int map_to_nodes_and_cal_internal_force(void);
	int cal_external_force(void);
	int update_nodal_variables(void);
	int contact_calculation(void);
	int map_to_particles_and_update_particle_variables(void);

	// helper functions
	NodeVar_2D_ME *init_node_variables(Particle_2D_ME *ppcl, Node_BG_R2D_ME *pnd);
	void contact_between_two_objects(Node_BG_R2D_ME *pnd);
	void contact_between_multiple_objects(Node_BG_R2D_ME *pnd);
};


#endif