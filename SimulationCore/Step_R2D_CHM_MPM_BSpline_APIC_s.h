#ifndef _STEP_R2D_CHM_MPM_BSPLINE_APIC_S_H_
#define _STEP_R2D_CHM_MPM_BSPLINE_APIC_S_H_

#include "Model_R2D_CHM_MPM_BSpline_s.h"
#include "Step.h"

int solve_substep_R2D_CHM_MPM_BSpline_APIC_s(void *_self);

// for single object only
class Step_R2D_CHM_MPM_BSpline_APIC_s : public Step
{
protected:
	Model_R2D_CHM_MPM_BSpline_s *model;
	double invD;

public:
	Step_R2D_CHM_MPM_BSpline_APIC_s();
	~Step_R2D_CHM_MPM_BSpline_APIC_s();

	inline void set_model(Model_R2D_CHM_MPM_BSpline_s *md)
	{
		Step::set_model(md);
		model = md;
	}

	// Restart from previous step
	inline void set_prev_step(Step_R2D_CHM_MPM_BSpline_APIC_s *prev_step)
	{
		Step::set_prev_step(prev_step);
		model = prev_step->model;
	}

protected:
	int init(void);
	friend int solve_substep_R2D_CHM_MPM_BSpline_APIC_s(void *_self);
	int finalize(void);

	void init_B_matrix(void);
};


#endif