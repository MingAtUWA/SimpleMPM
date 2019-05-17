#include "SimulationCore_pcp.h"

#include <cassert>

#include "ResultFile.h"
#include "Step.h"
#include "TimeHistory.h"

size_t TimeHistory::cur_id = 0;

TimeHistory::TimeHistory(TimeHistoryType tp, const char *tpna) :
	id(++cur_id), type(tp), type_name(tpna), name("TimeHistory"),
	time_rcd_id(0), res_file_type(1),
	field_num(0), point_num(0),
	output_initial_state(true), interval_num(0),
	next_time_ratio(0.0), time_interval_ratio(0.0), time_ratio_tol(1.0e-8),
	step(nullptr),
	prev_for_step_class(nullptr), next_for_step_class(nullptr),
	res_file(nullptr), file_info(nullptr),
	prev_for_resultfile_class(nullptr), next_for_resultfile_class(nullptr)
{
	char id_str[25];
	snprintf(id_str, 25, "%zu", id);
	name += id_str;
}

TimeHistory::~TimeHistory()
{
	if (step) res_file->del_output(this);
	if (res_file) res_file->del_output(this);
	if (file_info) delete file_info;
}