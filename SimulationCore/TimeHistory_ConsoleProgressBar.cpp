#include "SimulationCore_pcp.h"

#include <cstdio> // for fprintf
#include <ctime>

#include "Step.h"
#include "TimeHistory_ConsoleProgressBar.h"

#define DEFAULT_WIDTH 80
#define DEFAULT_WINDTH_DIV_100 ((float)DEFAULT_WIDTH / 100.0f)

TimeHistory_ConsoleProgressBar::TimeHistory_ConsoleProgressBar() :
	TimeHistory(TimeHistoryType::ConsoleProgressBar, "ConsoleProgressBar"),
	width (DEFAULT_WIDTH), width_div_100(DEFAULT_WINDTH_DIV_100),
	prev_pos(0), cur_pos(0)
{
	interval_num = 100;
	res_file_type = 0;
}

TimeHistory_ConsoleProgressBar::~TimeHistory_ConsoleProgressBar() {}

#define PADDING_STR "##################################################" \
					"##################################################" \
					"##################################################" \
					"##################################################"
void TimeHistory_ConsoleProgressBar::print_progress(void)
{
	cur_pos = (size_t)(100.0 * (step->get_current_time() + step->get_dt())/ step->get_step_time());
	cur_pos = cur_pos > 100 ? 100 : cur_pos;
	if (cur_pos == prev_pos) return;
	prev_pos = cur_pos;
	int lpad = (int)(cur_pos * width_div_100);
	int rpad = width - lpad;
	printf("\r%3zd%% [%.*s%*s]", cur_pos, lpad, PADDING_STR, rpad, "");
	fflush(stdout);
}

int TimeHistory_ConsoleProgressBar::init_per_step(void)
{
	prev_pos = 0;
	print_progress();
	start_time = std::chrono::system_clock::now();
	return 0;
}

int TimeHistory_ConsoleProgressBar::output(void)
{
	print_progress();
	return 0;
}

void TimeHistory_ConsoleProgressBar::finalize_per_step(void)
{
	end_time = std::chrono::system_clock::now();
	std::chrono::milliseconds elapsed_time
		= std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);

	std::time_t start_time_t = std::chrono::system_clock::to_time_t(start_time);
	std::tm start_time_tm;
	localtime_s(&start_time_tm, &start_time_t);
	char start_time_str[50];
	std::strftime(start_time_str, sizeof(start_time_str) / sizeof(char),
				  "%Y-%m-%d %H:%M:%S", &start_time_tm);
	
	std::time_t end_time_t = std::chrono::system_clock::to_time_t(end_time);
	std::tm end_time_tm;
	localtime_s(&end_time_tm, &end_time_t);
	char end_time_str[50];
	std::strftime(end_time_str, sizeof(end_time_str) / sizeof(char),
				  "%Y-%m-%d %H:%M:%S", &end_time_tm);

	printf("\nStep completed in %.3lf s - from ** %s ** to ** %s **\n",
			double(elapsed_time.count()) / 1000.0, start_time_str, end_time_str);
}