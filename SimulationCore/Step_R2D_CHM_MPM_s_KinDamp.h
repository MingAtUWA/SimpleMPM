#ifndef _STEP_R2D_CHM_MPM_S_KINDAMP_H_
#define _STEP_R2D_CHM_MPM_S_KINDAMP_H_

#include "Step.h"

#include "Model_R2D_CHM_MPM_s.h"

struct Model_R2D_CHM_MPM_s;

int solve_substep_R2D_CHM_MPM_s_KinDamp(void *_self);

// for single object only
class Step_R2D_CHM_MPM_s_KinDamp : public Step
{
protected:
	Model_R2D_CHM_MPM_s *model;
	
public:
	Step_R2D_CHM_MPM_s_KinDamp();
	~Step_R2D_CHM_MPM_s_KinDamp();

	inline void set_model(Model_R2D_CHM_MPM_s *md)
	{
		Step::set_model(md);
		model = md;
	}

	// Restart from previous step
	inline void set_prev_step(Step_R2D_CHM_MPM_s_KinDamp *prev_step)
	{
		Step::set_prev_step(prev_step);
		model = prev_step->model;
	}

protected:
	int init(void);
	friend int solve_substep_R2D_CHM_MPM_s_KinDamp(void *_self);
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

protected:
	double prev_kinetic_energy_s; // solid phase
	double prev_kinetic_energy_f; // fluid phase
};


#endif