#include "SimulationCore_pcp.h"

#include "Step.h"
#include "TimeHistory.h"

#include "ResultFile.h"

ResultFile::~ResultFile()
{
	close();
	detach_step();
}

void ResultFile::add_output(TimeHistory *th)
{
	if (!th || !th->res_file_type)
		return;
	if (th->res_file != this)
	{
		if (th->res_file)
			th->res_file->del_output(th);
		th->res_file = this;
		th->file_info = nullptr;
		add_th_to_list(th);
	}
}

void ResultFile::del_output(TimeHistory *th)
{
	if (has_th_in_list(th))
	{
		del_th_from_list(th);
		th->res_file = nullptr;
	}
	if (th->file_info)
	{
		delete th->file_info;
		th->file_info = nullptr;
	}
}

void ResultFile::set_step(Step *stp)
{
	if (step && step->res_file != this)
		step->res_file->detach_step();
	step = stp;
	if (step)
		step->res_file = this;
}

void ResultFile::detach_step(void)
{
	if (step && step->res_file == this)
		step->res_file = nullptr;
	step = nullptr;
}