#ifndef _RESULTFILE_H_
#define _RESULTFILE_H_

#include <cassert>
#include <string>

#include "TimeHistory.h"

class Step;

struct TimeHistoryFileInfo
{
	TimeHistory *time_history;
public:
	TimeHistoryFileInfo() : time_history(nullptr) {}
	virtual ~TimeHistoryFileInfo() {}
};

/*==========================================================================
 * Class ResultFile
 *==========================================================================
 *  1. Data in buffer and binary file are 8 bytes long each, which may be
 *     unsigned long long, long long and double;
 *  2. Each ResultFile contain all the	handles of output whose data will
 *     be written in this file.
 *  3. Need to rewrite funtions:
 *	    1) init(), close();
 *	    2) init_per_step(), finalize_per_step();
 *	    3) init_time_record(), finalize_time_record();
 *      4) get_field_data_buffer(), flush_field_data_buffer().
 *==========================================================================*/
class ResultFile
{
	friend Step;
	friend TimeHistory;

protected:
	std::string file_name;
	bool is_init; // true if is initialized
	Step *step;

public:
	ResultFile(const char *default_file_name = "ResultFile") :
		file_name(default_file_name), is_init(false),
		step(nullptr),
		time_history_list(nullptr) {}
	virtual ~ResultFile();

	inline const char *get_filename(void) { return file_name.c_str(); }
	inline void set_filename(const char *fn) { file_name = fn; }

	virtual int init(void) { return 0; }
	void close(void) {}

public:
	void add_output(TimeHistory *th); // Used by TimeHistory class
	void del_output(TimeHistory *th); // Used by TimeHistory class

	virtual int init_per_step(void) { return 0; } // Used by Step class
	virtual int finalize_per_step(void) { return 0; } // Used by Step class

	/*------------------------------------------------------------- 
	Each time history output has a series of time records,
		init_time_record() function is called at the begining of each
		of this time records.
	  -------------------------------------------------------------*/
	virtual int init_time_record(TimeHistoryFileInfo *_pthfi) { return 0; }
	virtual int finalize_time_record(TimeHistoryFileInfo *_pthfi) { return 0; }
	/*-------------------------------------------------------------
	get_output_buffer() function
	The row_num parameter controls the buffer size as:
		buffer size = row_num * field_num
	which indicates:
		1) input: desirable row number;
		2) output: actual row number.
	  -------------------------------------------------------------*/
	virtual double *get_field_data_buffer(TimeHistoryFileInfo *_pthfi, size_t &row_num) { return nullptr; }

	inline void set_step(Step *stp);
	inline void detach_step(void);

protected:
	// list of time history output
	TimeHistory *time_history_list;
	// utility function for list of time history output
	inline TimeHistory *first_th(void) { return time_history_list; }
	inline TimeHistory *next_th(TimeHistory *th) { return th->next_for_resultfile_class; }
	inline TimeHistory *prev_th(TimeHistory *th) { return th->prev_for_resultfile_class; }
	inline void add_th_to_list(TimeHistory *th)
	{
		assert(th);
		th->prev_for_resultfile_class = nullptr;
		th->next_for_resultfile_class = time_history_list;
		time_history_list = th;
	}
	inline bool has_th_in_list(TimeHistory *th)
	{
		for (TimeHistory *pth = time_history_list; pth;
			pth = pth->next_for_resultfile_class)
			if (th == pth) return true;
		return false;
	}
	inline void del_th_from_list(TimeHistory *th)
	{
		assert(th);
		TimeHistory *prev_th, *next_th;
		prev_th = th->prev_for_resultfile_class;
		next_th = th->next_for_resultfile_class;
		if (prev_th)
			prev_th->next_for_resultfile_class = next_th;
		else
			time_history_list = next_th;
		if (next_th) next_th->prev_for_resultfile_class = prev_th;

		th->prev_for_resultfile_class = nullptr;
		th->next_for_resultfile_class = nullptr;
	}
	inline void set_th_file_info(TimeHistory *th, TimeHistoryFileInfo *thfo)
	{
		assert(th);
		if (th->file_info)
			delete th->file_info;
		th->file_info = thfo;
		if (thfo)
			thfo->time_history = th;
	}
	inline TimeHistoryFileInfo *get_th_file_info(TimeHistory *th) { return th->file_info; }
	inline void del_th_file_info(TimeHistory *th)
	{
		assert(th);
		if (th->file_info) delete th->file_info;
		th->file_info = nullptr;
	}
	inline Step *get_th_step(TimeHistory *th) { return th->step; }
};

#endif