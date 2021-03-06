#ifndef _STEP_1D_ME_MPM_BSPLINE_S_H_
#define _STEP_1D_ME_MPM_BSPLINE_S_H_

#include "Model_1D_ME_MPM_BSpline_s.h"
#include "Step.h"

// Only use the sysmetric quadratic spline
// ignore spline at the boundary
// treat fix boundary coniditions by assigning zero coefficient to the two spline at the boundary 
// This approach is not correct, not successful

// but the blended method for node/particle mapping is OK

int solve_substep_1D_ME_MPM_BSpline_s(void *_self);

// for single object only
class Step_1D_ME_MPM_BSpline_s : public Step
{
protected:
	Model_1D_ME_MPM_BSpline_s *model;
	double N_tol;

public:
	Step_1D_ME_MPM_BSpline_s();
	~Step_1D_ME_MPM_BSpline_s();

	inline void set_model(Model_1D_ME_MPM_BSpline_s *md)
	{
		Step::set_model(md);
		model = md;
	}
	// Restart version
	inline void set_prev_step(Step_1D_ME_MPM_BSpline_s *prev_step)
	{
		Step::set_prev_step(prev_step);
		model = prev_step->model;
	}

protected:
	int init(void);
	friend int solve_substep_1D_ME_MPM_BSpline_s(void *_self);
	int finalize(void);

};

#endif