#ifndef _STEP_R2D_CHM_MPM_S_H_
#define _STEP_R2D_CHM_MPM_S_H_

#include "Model_R2D_CHM_MPM_s.h"
#include "Step.h"

struct Model_R2D_CHM_MPM_s;

int solve_substep_R2D_CHM_MPM_s(void *_self);

// for single object only
class Step_R2D_CHM_MPM_s : public Step
{
protected:
	Model_R2D_CHM_MPM_s *model;
	
public:
	Step_R2D_CHM_MPM_s();
	~Step_R2D_CHM_MPM_s();

	inline void set_model(Model_R2D_CHM_MPM_s *md)
	{
		Step::set_model(md);
		model = md;
	}

	// Restart from previous step
	inline void set_prev_step(Step_R2D_CHM_MPM_s *prev_step)
	{
		Step::set_prev_step(prev_step);
		model = prev_step->model;
	}

protected:
	int init(void);
	friend int solve_substep_R2D_CHM_MPM_s(void *_self);
	int finalize(void);
};


#endif