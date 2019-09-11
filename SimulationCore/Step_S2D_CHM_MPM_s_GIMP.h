#ifndef _STEP_S2D_CHM_MPM_S_GIMP_H_
#define _STEP_S2D_CHM_MPM_S_GIMP_H_

#include "Step.h"
#include "Model_S2D_CHM_MPM_s.h"

int solve_substep_S2D_CHM_MPM_s_GIMP(void *_self);

// for single object only
class Step_S2D_CHM_MPM_s_GIMP : public Step
{
protected:
	double min_dt, max_dt;
	double h_elem_raio, h_pcl_ratio;

public:
	Step_S2D_CHM_MPM_s_GIMP();
	~Step_S2D_CHM_MPM_s_GIMP();

	inline void set_model(Model_S2D_CHM_MPM_s *md)
	{
		Step::set_model(md);
		model = md;
	}

	// Restart from previous step
	inline void set_prev_step(Step_S2D_CHM_MPM_s_GIMP *prev_step)
	{
		Step::set_prev_step(prev_step);
		model = prev_step->model;
	}

	inline void set_dt(double _dt,
		double dt_max_min_raio = 0.1 /*ad hoc number*/,
		double t_tol_r = 0.01)
	{
		max_dt = _dt;
		min_dt = max_dt * dt_max_min_raio;
		time_tol_ratio = t_tol_r;
		time_tol = max_dt * t_tol_r;
	}

	inline void set_dt_ratio(double _h_elem_ratio, double _h_pcl_ratio)
	{
		h_elem_raio = _h_elem_ratio;
		h_pcl_ratio = _h_pcl_ratio;
	}

protected:
	int init(void);
	friend int solve_substep_S2D_CHM_MPM_s_GIMP(void *_self);
	int finalize(void);

protected:
	// model data
	Model_S2D_CHM_MPM_s *model;
};

#endif