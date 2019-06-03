#ifndef _STEP_R2D_CHM_MPM_S_VISDAMP_H_
#define _STEP_R2D_CHM_MPM_S_VISDAMP_H_

#include "Model_R2D_CHM_MPM_s.h"
#include "Step.h"

struct Model_R2D_CHM_MPM_s;
class Step_R2D_CHM_MPM_s_KinDamp;

int solve_substep_R2D_CHM_MPM_s_damp(void *_self);

// viscous damping
// for single object only
class Step_R2D_CHM_MPM_s_VisDamp : public Step
{
protected:
	Model_R2D_CHM_MPM_s *model;
	
public:
	Step_R2D_CHM_MPM_s_VisDamp();
	~Step_R2D_CHM_MPM_s_VisDamp();

	void set_model(Model_R2D_CHM_MPM_s *md);

	// continue from previous step
	void set_prev_step(Step_R2D_CHM_MPM_s_VisDamp *prev_step);
	void set_prev_step(Step_R2D_CHM_MPM_s_KinDamp *prev_step);

protected:
	int init(void);
	friend int solve_substep_R2D_CHM_MPM_s_damp(void *_self);
	int finalize(void);

protected:
	// global minimum element characteristic length
	// decide whether critical time step size should be calculated
	double elem_len_min;
	// adjust dt
	// dt = critical time step * dt_adjust_factor
	double dt_adjust_factor;

public:
	inline void set_dt(double d_t, double t_tol_r = 0.01)
	{
		Step::set_dt(d_t, t_tol_r);
		elem_len_min = -1.0; // prevent calling cal_global_critical_time_step()
	}
	inline void set_auto_dt(double dt_adj_fac, double t_tol_r = 0.01)
	{
		dt_adjust_factor = dt_adj_fac;
		time_tol_ratio = t_tol_r;
		elem_len_min = std::numeric_limits<double>::max();
	}
};


#endif