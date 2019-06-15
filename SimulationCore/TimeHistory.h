#ifndef _TIMEHISTORY_H_
#define _TIMEHISTORY_H_

#include <string>
#include <cassert>

enum class TimeHistoryType : unsigned short int
{
	InvalidType = 0,
	ConsoleProgressBar = 1,
	Node_R1D_CHM_s = 2,
	Node_R2D_CHM_s = 3,
	GaussPoint_2D_CHM = 4,
	Particle_R2D_ME_s = 5,
	Particle_R2D_CHM_s = 6,
	Particle_2D_ME_AllPcl = 7,
	Particle_1D_ME_AllPcl = 8
};

struct Model;
class Step;
struct TimeHistoryFileInfo;
class ResultFile;

/*=============================================================
Class TimeHistory
	1. Functions needs to be rewritten:
	   1) init_per_step();
	   2) output();
	   3) finalize_per_step().
 ==============================================================*/
class TimeHistory
{
	friend Step;
	friend ResultFile;

protected:
	static size_t cur_id;
	const size_t id;
	const TimeHistoryType type;
	const char *type_name;
	std::string name;
	size_t time_rcd_id;

	// res_file_type meaning
	// 0 don't need file
	// 1 need file
	size_t res_file_type;

public:
	TimeHistory(TimeHistoryType tp = TimeHistoryType::InvalidType,
				const char *tpna = "InvalidType");
	~TimeHistory();
	
	// get parameters
	inline size_t get_id(void) const { return id; }
	inline const char *get_name(void) const { return name.c_str(); }
	inline TimeHistoryType get_type(void) const { return type; }
	inline const char *get_type_name(void) const { return type_name; }
	inline size_t get_interval_num(void) const { return interval_num; }
	inline size_t get_time_rcd_id(void) const { return time_rcd_id; }
	// set parameters
	inline void set_name(const char *na) { name = na; }
	inline void set_interval_num(size_t num) { interval_num = num; }
	inline void set_if_output_initial_state(bool flag) { output_initial_state = flag; }

	// Initialize each steps
	virtual int init_per_step(void) { return 0; }
	// Finalize each steps
	virtual void finalize_per_step(void) {}
	// Output funtion
	virtual int output(void) { return 0; }

protected:
	// Used by list in Step class only.
	Step *step;
	TimeHistory *prev_for_step_class;
	TimeHistory *next_for_step_class;
	// Used by list in ResultFile class only.
	ResultFile *res_file;
	TimeHistoryFileInfo *file_info;
	TimeHistory *prev_for_resultfile_class;
	TimeHistory *next_for_resultfile_class;

protected:
	size_t field_num;
	size_t point_num;
public:
	inline size_t get_field_num(void) { return field_num; }
	inline size_t get_point_num(void) { return point_num; }
	inline size_t get_data_num(void) { return field_num * point_num; }

protected:
	// true if output the initial state
	bool output_initial_state;
	// Control the number of TimeHistory per step:
	size_t interval_num;
	// output schedule
	double next_time_ratio;
	double time_interval_ratio;
	double time_ratio_tol;
	// utility functions for ouput scheduling
	// used by Step class only
	inline double init_next_time_ratio(void)
	{
		assert(interval_num);
		time_interval_ratio = 1.0 / interval_num;
		if (output_initial_state)
			next_time_ratio = -time_ratio_tol;
		else
			next_time_ratio = time_interval_ratio - time_ratio_tol;
		return next_time_ratio;
	}
	inline double update_next_time_ratio(double cur_t_ratio)
	{
		if (cur_t_ratio > next_time_ratio)
			next_time_ratio += time_interval_ratio;
		if (cur_t_ratio > next_time_ratio)
			next_time_ratio = cur_t_ratio;
		return next_time_ratio;
	}
};

#endif