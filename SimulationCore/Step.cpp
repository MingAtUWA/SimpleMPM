#include "SimulationCore_pcp.h"

#include <cassert>

#include "TimeHistory.h"
#include "ResultFile.h"
#include "Step.h"

size_t Step::max_index = 0;

Step::Step(SolveSubstepFunc solve_substep_func) : 
	solve_substep(solve_substep_func),
	name("Step"), index(++max_index), model(nullptr), res_file(nullptr),
	is_first_step(true), start_substep_index(0), substep_num(0),
	step_time(0.0), start_time(0.0), current_time(0.0),
	dt(0.0), time_tol_ratio(0.01), time_tol(0.0),
	time_history_output_num(0), time_history_list(nullptr),
	info_for_time_history_output(nullptr),
	tol(1.0e-10)
{
	char id_str[25];
	snprintf(id_str, 25, "%zu", index);
	name += id_str;
}

Step::~Step()
{
	if (info_for_time_history_output)
	{
		delete[] info_for_time_history_output;
		info_for_time_history_output = nullptr;
	}

	while (time_history_list)
		del_output(time_history_list);

	detach_result_file();
}

int Step::solve(void)
{
	substep_num = 0;
	current_time = 0.0;

	assert(res_file);
	if (is_first_step)
	{
		res_file->init();
	}

	init(); // initialize calculation
	init_output();

	double time_diff_tmp;
	do
	{
		(*solve_substep)(this);

		++substep_num;
		current_time += dt;
		time_diff_tmp = current_time - step_time;
		if (time_diff_tmp > time_tol)
		{
			dt -= time_diff_tmp;
			current_time = step_time;
		}

		output_time_history_if_needed();

	} while (-time_diff_tmp > time_tol);

	finalize(); // finalize calculation
	finalize_output();

	return 0;
}

int solve_substep_base(void *_self) { return 0; }

int Step::init_output(void)
{
	size_t i;
	
	// Initialize time history information
	TimeHistory *pth;
	if (time_history_output_num)
	{
		info_for_time_history_output = new InfoForTimeHistoryOutput[time_history_output_num];
		for (pth = time_history_list, i = 0; pth;
			 pth = pth->next_for_step_class, ++i) 
		{
			info_for_time_history_output[i].time_history = pth;
			info_for_time_history_output[i].next_output_time = step_time * pth->init_next_time_ratio();
			
			pth->step = this;
			// add time histories to result file
			res_file->add_output(pth);
		}
	}

	// Initialize time histories
	for (pth = time_history_list; pth; pth = pth->next_for_step_class)
		pth->init_per_step();

	// Initialize result file
	res_file->step = this;
	res_file->init_per_step(); // initialize handle for each output handle
	
	// output the initial state if needed
	output_time_history_if_needed();

	return 0;
}

int Step::finalize_output(void)
{
	// Finalize time history output
	TimeHistory *pth;
	for (size_t i = 0; i < time_history_output_num; ++i)
	{
		pth = info_for_time_history_output[i].time_history;
		pth->finalize_per_step();
		pth->step = nullptr;
	}
	if (info_for_time_history_output)
	{
		delete[] info_for_time_history_output;
		info_for_time_history_output = nullptr;
	}

	// Finalize result file
	res_file->finalize_per_step();
	res_file->step = nullptr;

	return 0;
}

int Step::output_time_history_anyway(void)
{
	for (size_t i = 0; i < time_history_output_num; i++)
		info_for_time_history_output[i].time_history->output();
	return 0;
}

int Step::output_time_history_if_needed(void)
{
	TimeHistory *pth;
	for (size_t i = 0; i < time_history_output_num; i++)
	{
		if (current_time > info_for_time_history_output[i].next_output_time)
		{
			pth = info_for_time_history_output[i].time_history;
			pth->output();
			info_for_time_history_output[i].next_output_time
				= step_time * pth->update_next_time_ratio(current_time / step_time);
		}
	}
	return 0;
}
