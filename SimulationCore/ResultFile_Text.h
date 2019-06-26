#ifndef _RESULTFILE_TEXT_H_
#define _RESULTFILE_TEXT_H_

#include "ResultFile.h"

struct Model_R2D_CHM_MPM_s;
struct Model_R2D_ME_MPM_s;
struct Model_R2D_ME_MPM_BSpline_s;
struct Model_R2D_CHM_MPM_BSpline_s;

struct TimeHistoryFileInfo_Text : public TimeHistoryFileInfo
{
	char *buffer;
	size_t buffer_row_num;
	size_t buffer_used_row_num;

	char *text_buffer;
	size_t text_buffer_size;
	size_t text_buffer_used_size;

	size_t field_num;

	std::ofstream file;

public:
	TimeHistoryFileInfo_Text();
	~TimeHistoryFileInfo_Text();
};


/*=================================================
Class ResultFile_Text
=================================================*/
class ResultFile_Text : public ResultFile
{
public:
	ResultFile_Text();
	~ResultFile_Text();

	// Output Time History
	int init(void) override;
	void close(void);

	int init_per_step(void) override;
	int finalize_per_step(void) override;

	int init_time_record(TimeHistoryFileInfo *_pthfi) override;
	int finalize_time_record(TimeHistoryFileInfo *_pthfi) override;

	double *get_field_data_buffer(TimeHistoryFileInfo *_pthfi, size_t &buf_size) override;
	int flush_field_data_buffer(TimeHistoryFileInfo *_pthfi);

	// Output Model State
	int output_model_state(Model_R2D_ME_MPM_s &md);
	int output_model_state(Model_R2D_CHM_MPM_s &md);
	int output_model_state(Model_R2D_ME_MPM_BSpline_s &md);
	int output_model_state(Model_R2D_CHM_MPM_BSpline_s &md);
};

#endif