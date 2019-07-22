#ifndef _STEP_S2D_CHM_MPM_S_GIMP_
#define _STEP_S2D_CHM_MPM_S_GIMP_

#include "Step.h"
#include "Model_S2D_CHM_MPM_s.h"

int solve_substep_S2D_CHM_MPM_s_GIMP(void *_self);

// for single object only
class Step_S2D_CHM_MPM_s_GIMP : public Step
{
public:
	Step_S2D_CHM_MPM_s_GIMP();
	~Step_S2D_CHM_MPM_s_GIMP();

	// Initialized as the first step
	int init_step(Model_S2D_CHM_MPM_s *md, double step_t);
	// Restart from previous step
	int init_step(Step_S2D_CHM_MPM_s_GIMP *prev_step, double step_t);

protected:
	int init_calculation(void);
	friend int solve_substep_S2D_CHM_MPM_s_GIMP(void *_self);

protected:
	// model data
	Model_S2D_CHM_MPM_s *model;
};

#endif