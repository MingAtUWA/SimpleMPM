#ifndef _STEP_S2D_CHM_MPM_S_MIX_H_
#define _STEP_S2D_CHM_MPM_S_MIX_H_

#include "Step.h"
#include "Model_S2D_CHM_MPM_s_Mix.h"

int solve_substep_S2D_CHM_MPM_s_Mix(void *_self);

// for single object only
class Step_S2D_CHM_MPM_s_Mix : public Step
{
public:
	Step_S2D_CHM_MPM_s_Mix();
	~Step_S2D_CHM_MPM_s_Mix();

	inline void set_model(Model_S2D_CHM_MPM_s_Mix *md)
	{
		Step::set_model(md);
		model = md;
	}

	// Restart from previous step
	inline void set_prev_step(Step_S2D_CHM_MPM_s_Mix *prev_step)
	{
		Step::set_prev_step(prev_step);
		model = prev_step->model;
	}

	inline void set_dt(double _dt, double t_tol_r = 0.01)
	{
		dt = _dt;
		time_tol_ratio = t_tol_r;
		time_tol = dt * t_tol_r;
	}

protected:
	int init(void);
	friend int solve_substep_S2D_CHM_MPM_s_Mix(void *_self);
	int finalize(void);

protected:
	// model data
	Model_S2D_CHM_MPM_s_Mix *model;
};

#endif