#ifndef _STEP_H_
#define _STEP_H_

#include <string>
#include "TimeHistory.h"
#include "ResultFile.h"

struct Model;

// Avoid using virtual function
typedef int(*SolveSubstepFunc)(void *_self);

/* ========================================================
Class Step:
	Functions needs to be rewritten in children classes:
	1. init()
	2. solve_substep()
	3. finalize()
 =========================================================== */
int solve_substep_base(void *_self);

class Step
{
	friend ResultFile;
	friend void test_hdf5_resultfile(void);
	friend void test_text_resultfile(void);

protected:
	static size_t cur_id;
	size_t id;
	std::string name;

	Model *model;
	ResultFile *res_file;

public:
	Step(SolveSubstepFunc solve_substep_func = &solve_substep_base);
	~Step();

	// get parameters
	inline size_t get_id(void) const { return id; }
	inline const char *get_name(void) const { return name.c_str(); }
	// set parameters
	inline void set_name(const char *na) { name = na; }
	inline void set_step_time(double step_t) { step_time = step_t; }
	inline void set_dt(double d_t, double t_tol_r = 0.01)
	{
		dt = d_t;
		time_tol_ratio = t_tol_r;
		time_tol = dt * t_tol_r;
	}
	// for the first step
	inline void set_model(Model *md) { model = md; }
	inline void set_result_file(ResultFile *rf)
	{
		if (res_file && res_file->step != this)
			res_file->step->detach_result_file();
		res_file = rf; if (res_file) res_file->step = this;
	}
	inline void detach_result_file(void)
	{
		if (res_file && res_file->step == this)
			res_file->step = nullptr;
		res_file = nullptr;
	}
	// for other step
	inline void set_prev_step(Step *prev_step)
	{
		if (!prev_step) return;

		if (id <= prev_step->id)
		{
			id = prev_step->id + 1;
			cur_id = id;
		}

		is_first_step = false;
		start_substep_index = prev_step->get_total_substep_num();
		start_time = prev_step->get_total_time();

		set_model(prev_step->model);
		set_result_file(prev_step->res_file);
	}
	inline void add_output(TimeHistory *th)
	{
		if (!th) return;
		if (th->step != this)
		{
			if (th->step) th->step->del_output(th);
			th->step = this;
			add_th_to_list(th);
		}
	}
	inline void del_output(TimeHistory *th)
	{
		if (has_th_in_list(th))
		{
			del_th_from_list(th);
			th->step = nullptr;
		}
	}

	// main functions
	virtual int solve(void);

protected:
	bool is_first_step;
	// initialization before calculation
	virtual int init(void) { return 0; }
	// calculation of each substep
	SolveSubstepFunc solve_substep;
	// finalization after calculation
	virtual int finalize(void) { return 0; }

protected:
	/* ========= substep information ========= */
	// substep index at start of this step
	size_t start_substep_index;
	// number of substep from the start of this step
	size_t substep_num;
	// time length of this step
	double step_time;
	// start time for this step
	double start_time;
	// time from the start of this step
	double current_time;
	// time increment
	double dt;
	double time_tol_ratio;
	double time_tol;
public:
	// time from the start of this step
	inline double get_current_time(void) { return current_time; }
	// total time from the start of the whole simulation
	inline double get_total_time(void) { return start_time + current_time; }
	// number of substep from teh start of this step
	inline size_t get_substep_num(void) { return substep_num; }
	// total number of substep from the start of the whole simulation
	inline size_t get_total_substep_num(void) { return start_substep_index + substep_num; }
	// time length of this step
	inline double get_step_time(void) { return step_time; }
	// size of time increment
	inline double get_dt(void) { return dt; }

protected:
	// Output Utilities
	int init_output(void);
	int finalize_output(void);
	int output_time_history_anyway(void);
	int output_time_history_if_needed(void);

	// list of time histories
	size_t time_history_output_num;
	TimeHistory *time_history_list;
	// utility functions for the list
	inline TimeHistory *first_th(void) { return time_history_list; }
	inline TimeHistory *next_th(TimeHistory *th) { return th->next_for_step_class; }
	inline TimeHistory *prev_th(TimeHistory *th) { return th->prev_for_step_class; }
	void add_th_to_list(TimeHistory *th)
	{
		if (!th) return;
		th->prev_for_step_class = nullptr;
		th->next_for_step_class = time_history_list;
		time_history_list = th;
		++time_history_output_num;
	}
	bool has_th_in_list(TimeHistory *th)
	{
		for (TimeHistory *pth = time_history_list; pth;
			 pth = pth->next_for_step_class)
			if (th == pth) return true;
		return false;
	}
	void del_th_from_list(TimeHistory *th)
	{
		TimeHistory *prev_th, *next_th;
		prev_th = th->prev_for_step_class;
		next_th = th->next_for_step_class;
		if (prev_th)
			prev_th->next_for_step_class = next_th;
		else
			time_history_list = next_th;
		if (next_th) next_th->prev_for_step_class = prev_th;
		--time_history_output_num;

		th->prev_for_step_class = nullptr;
		th->next_for_step_class = nullptr;
	}

	struct InfoForTimeHistoryOutput
	{
		double next_output_time;
		TimeHistory *time_history;
	};
	InfoForTimeHistoryOutput *info_for_time_history_output;

protected: // tolerance for double
	double tol;
public:
	inline void set_tol(double t) { tol = t < 0 ? tol : t; }
	inline bool is_zero(double num) { return abs(num) < tol; }
	inline bool is_equal(double num1, double num2) { return abs(num1 - num2) < tol; }
};

#endif