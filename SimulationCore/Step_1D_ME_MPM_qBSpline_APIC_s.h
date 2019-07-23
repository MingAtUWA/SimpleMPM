#ifndef _STEP_1D_ME_MPM_BSPLINE_APIC_S_H_
#define _STEP_1D_ME_MPM_BSPLINE_APIC_S_H_

#include "Model_1D_ME_MPM_BSpline_s.h"
#include "Step.h"

// use complete set of quadratic spline

int solve_substep_1D_ME_MPM_qBSpline_APIC_s(void *_self);

// for single object only
class Step_1D_ME_MPM_qBSpline_APIC_s : public Step
{
protected:
	Model_1D_ME_MPM_BSpline_s *model;
	double N_tol;
	// APIC inverse of D matrix
	double invD;

public:
	Step_1D_ME_MPM_qBSpline_APIC_s();
	~Step_1D_ME_MPM_qBSpline_APIC_s();

	inline void set_model(Model_1D_ME_MPM_BSpline_s *md)
	{
		Step::set_model(md);
		model = md;
	}
	// Restart version
	inline void set_prev_step(Step_1D_ME_MPM_qBSpline_APIC_s *prev_step)
	{
		Step::set_prev_step(prev_step);
		model = prev_step->model;
	}

protected:
	int init(void);
	friend int solve_substep_1D_ME_MPM_qBSpline_APIC_s(void *_self);
	int finalize(void);

};

#endif