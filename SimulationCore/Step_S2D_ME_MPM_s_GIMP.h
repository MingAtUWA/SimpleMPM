#ifndef _STEP_S2D_ME_MPM_S_GIMP_H_
#define _STEP_S2D_ME_MPM_S_GIMP_H_

#include "Step.h"
#include "Model_S2D_ME_MPM_s.h"

int solve_substep_S2D_ME_MPM_s_GIMP(void *_self);

// for single object only
class Step_S2D_ME_MPM_s_GIMP : public Step
{
public:
	Step_S2D_ME_MPM_s_GIMP();
	~Step_S2D_ME_MPM_s_GIMP();

	inline void set_model(Model_S2D_ME_MPM_s *md)
	{
		Step::set_model(md);
		model = md;
	}

	// Restart from previous step
	inline void set_prev_step(Step_S2D_ME_MPM_s_GIMP *prev_step)
	{
		Step::set_prev_step(prev_step);
		model = prev_step->model;
	}

protected:
	int init(void);
	friend int solve_substep_S2D_ME_MPM_s_GIMP(void *_self);
	int finalize(void);

protected:
	Model_S2D_ME_MPM_s *model;
};

#endif