#ifndef __STEP_S2D_ME_S_RIGIDBODY_H__
#define __STEP_S2D_ME_S_RIGIDBODY_H__

#include "Step.h"
#include "Model_S2D_ME_s_RigidBody.h"

int solve_substep_S2D_ME_s_RigidBody(void *_self);

class Step_S2D_ME_s_RigidBody : public Step
{
protected:
	double min_dt, max_dt;
	double h_elem_raio, h_pcl_ratio;
	// pcl_ratio part not yet finished

public:
	Step_S2D_ME_s_RigidBody();
	~Step_S2D_ME_s_RigidBody();

	inline void set_model(Model_S2D_ME_s_RigidBody *md)
	{
		Step::set_model(md);
		model = md;
	}

	// Restart from previous step
	inline void set_prev_step(Step_S2D_ME_s_RigidBody *prev_step)
	{
		Step::set_prev_step(prev_step);
		model = prev_step->model;
	}

	inline void set_dt(double _dt,
		double dt_max_min_raio = 1.0e-3 /*ad hoc number*/,
		double t_tol_r = 0.01)
	{
		max_dt = _dt;
		min_dt = max_dt * dt_max_min_raio;
		time_tol_ratio = t_tol_r;
		time_tol = min_dt * t_tol_r;
	}

	inline void set_dt_ratio(double _h_elem_ratio, double _h_pcl_ratio)
	{
		h_elem_raio = _h_elem_ratio;
		h_pcl_ratio = _h_pcl_ratio;
	}

protected:
	int init(void);
	friend int solve_substep_S2D_ME_s_RigidBody(void *_self);
	int finalize(void);

protected:
	Model_S2D_ME_s_RigidBody *model;
};

#endif