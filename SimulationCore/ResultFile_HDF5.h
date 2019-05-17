#ifndef _RESULTFILE_HDF5_H_
#define _RESULTFILE_HDF5_H_

#include "hdf5.h"
#include "ResultFile.h"

// handle class for TimeHistory output
struct TimeHistoryFileInfo_HDF5 : public TimeHistoryFileInfo
{
	char *buffer;
	size_t buffer_row_num;
	size_t buffer_used_row_num;

	size_t field_num;
	size_t row_num; // total number of row that has been output
	size_t time_rcd_num;

	hid_t grp_id;
	hid_t fld_dset_id;
	hid_t time_rcd_dset_id;
public:
	TimeHistoryFileInfo_HDF5();
	~TimeHistoryFileInfo_HDF5();
};

/*=================================================
Class ResultFile_HDF5
  =================================================*/
class ResultFile_HDF5 : public ResultFile
{
protected:
	hid_t hdf5_file_id;
	hid_t time_history_grp_id;
	hid_t model_state_grp_id;
	// double data type
	hid_t double_datatype_id;
public:
	ResultFile_HDF5();
	~ResultFile_HDF5();

	int init(void);
	void close(void);

	int init_per_step(void);
	int finalize_per_step(void);

	int init_time_record(TimeHistoryFileInfo *_pthfi);
	double *get_field_data_buffer(TimeHistoryFileInfo *_pthfi, size_t &buf_size);
	int flush_field_data_buffer(TimeHistoryFileInfo *_pthfi);
};

#endif