#include "SimulationCore_pcp.h"

#include <cassert>

#include "Step.h"
#include "TimeHistory.h"
#include "ResultFile_HDF5.h"

union TimeRecord_HDF5
{
	struct
	{
		unsigned long long step_index;
		unsigned long long substep_num;
		unsigned long long total_substep_num;
		double current_time;
		double total_time;
		unsigned long long field_data_num;
	};
	double data_d[6];
};
#define TimeRecordDataNum (sizeof(TimeRecord_HDF5) / sizeof(double))

/*=================================================
Class TimeHistoryFileInfo_HDF5
  =================================================*/
TimeHistoryFileInfo_HDF5::TimeHistoryFileInfo_HDF5() :
	buffer(nullptr), buffer_row_num(0), buffer_used_row_num(0),
	field_num(0), row_num(0), time_rcd_num(0),
	grp_id(-1), fld_dset_id(-1), time_rcd_dset_id(-1) {}

TimeHistoryFileInfo_HDF5::~TimeHistoryFileInfo_HDF5()
{
	// output at the end of the time history
	if (row_num)
	{
		
	}

	if (buffer)
	{
		delete[] buffer;
		buffer = nullptr;
	}
	if (time_rcd_dset_id >= 0)
	{
		H5Dclose(time_rcd_dset_id);
		time_rcd_dset_id = -1;
	}
	if (fld_dset_id >= 0)
	{
		H5Dclose(fld_dset_id);
		fld_dset_id = -1;
	}
	if (grp_id >= 0)
	{
		H5Gclose(grp_id);
		grp_id = -1;
	}
}

/*=================================================
Class ResultFile_HDF5
  =================================================*/
ResultFile_HDF5::ResultFile_HDF5() :
	hdf5_file_id(-1), time_history_grp_id(-1), model_state_grp_id(-1),
	double_datatype_id(-1) {}

ResultFile_HDF5::~ResultFile_HDF5() {}

int ResultFile_HDF5::init(void)
{
	// only init once
	if (is_init) return 0;
	is_init = true;

	// create hdf5 file
	const char *hdf5_subfix = ".hdf5";
	const size_t hdf5_subfix_len = strlen(hdf5_subfix);
	size_t dd = file_name.length();
	size_t dd2 = file_name.size();
	if (file_name.length() < hdf5_subfix_len
		|| strcmp(file_name.c_str() + file_name.length() - hdf5_subfix_len, hdf5_subfix))
		file_name += hdf5_subfix;
	hdf5_file_id = H5Fcreate(file_name.c_str(), H5F_ACC_TRUNC, H5P_DEFAULT, H5P_DEFAULT);
	if (hdf5_file_id < 0) return -2;
	
	// create group for time history and model state
	time_history_grp_id = H5Gcreate(hdf5_file_id, "TimeHistory", H5P_DEFAULT, H5P_DEFAULT, H5P_DEFAULT);
	if (time_history_grp_id < 0) return -2;
	model_state_grp_id = H5Gcreate(hdf5_file_id, "ModelState", H5P_DEFAULT, H5P_DEFAULT, H5P_DEFAULT);
	if (model_state_grp_id < 0) return -2;

	// double datatype
	double_datatype_id = H5Tcopy(H5T_NATIVE_DOUBLE);
	H5Tset_order(double_datatype_id, H5T_ORDER_LE);

	return 0;
}

void ResultFile_HDF5::close(void)
{
	while (time_history_list) del_output(time_history_list);

	if (time_history_grp_id >= 0)
	{
		H5Fclose(time_history_grp_id);
		time_history_grp_id = -1;
	}
	if (model_state_grp_id >= 0)
	{
		H5Fclose(model_state_grp_id);
		model_state_grp_id = -1;
	}
	if (hdf5_file_id >= 0)
	{
		H5Fclose(hdf5_file_id);
		hdf5_file_id = -1;
	}

	H5Tclose(double_datatype_id);
}

int ResultFile_HDF5::init_per_step(void)
{
	TimeHistory *pth, *pth_tmp;
	TimeHistoryFileInfo_HDF5 *pthfi;
	hsize_t dims[2], max_dims[2], chunk_dims[2];
	hid_t time_rcd_dataspace_id, fld_dataspace_id, prop_id;

	for (pth = first_th(); pth; pth = next_th(pth))
	{
		// delete the output if it is not used anymore
		if (!get_th_step(pth))
		{
			pth_tmp = pth;
			pth = prev_th(pth);
			del_output(pth_tmp);
			continue;
		}
		// create handle for output without handle
		if (!get_th_file_info(pth))
		{
			pthfi = new TimeHistoryFileInfo_HDF5;
			set_th_file_info(pth, pthfi);
			// ************ inital file buffer ************
			pthfi->field_num = pth->get_field_num();
			/* may need better algorithm to select buffer_size in the future */
			pthfi->buffer_row_num = pth->get_point_num() * 2;
			pthfi->buffer = new char[pthfi->buffer_row_num * pthfi->field_num * sizeof(double)];
			// group for this time history
			pthfi->grp_id = H5Gcreate(time_history_grp_id, pth->get_name(), H5P_DEFAULT, H5P_DEFAULT, H5P_DEFAULT);
			// dataset for time record of this time history
			//   1) create dataspace
			dims[0] = 0;
			dims[1] = TimeRecordDataNum;
			max_dims[0] = H5S_UNLIMITED;
			max_dims[1] = dims[1];
			time_rcd_dataspace_id = H5Screate_simple(2, dims, max_dims);
			//   2) create dateset properties
			chunk_dims[0] = 1024; // 1K
			chunk_dims[1] = dims[1];
			prop_id = H5Pcreate(H5P_DATASET_CREATE);
			H5Pset_chunk(prop_id, 2, chunk_dims);
			//   3) create dataset and write data
			pthfi->time_rcd_dset_id = H5Dcreate(pthfi->grp_id, "TimeRecord", double_datatype_id,
											time_rcd_dataspace_id, H5P_DEFAULT, prop_id, H5P_DEFAULT);
			//   4) clear
			H5Sclose(time_rcd_dataspace_id);
			H5Pclose(prop_id);
			if (pthfi->time_rcd_dset_id < 0) return -2;
			// dataset for field data of this time history
			//   1) create dataspace
			dims[0] = 0;
			dims[1] = pthfi->field_num;
			max_dims[0] = H5S_UNLIMITED;
			max_dims[1] = dims[1];
			fld_dataspace_id = H5Screate_simple(2, dims, max_dims);
			//   2) create dataset properties
			chunk_dims[0] = 1024; // 1K
			chunk_dims[1] = dims[1];
			prop_id = H5Pcreate(H5P_DATASET_CREATE);
			H5Pset_chunk(prop_id, 2, chunk_dims);
			//   3) create dataset and write data
			pthfi->fld_dset_id = H5Dcreate(pthfi->grp_id, "FieldData", double_datatype_id,
										fld_dataspace_id, H5P_DEFAULT, prop_id, H5P_DEFAULT);
			//   4) clear
			H5Sclose(fld_dataspace_id);
			H5Pclose(prop_id);
			if (pthfi->fld_dset_id < 0)	return -2;
			//*********************************
		}
	}
	return 0;
}

int ResultFile_HDF5::finalize_per_step(void)
{
	TimeHistory *pth;

	// flush all file buffer
	for (pth = first_th(); pth; pth = next_th(pth))
		flush_field_data_buffer(get_th_file_info(pth));

	// output step info in the future here

	return 0;
}

int ResultFile_HDF5::init_time_record(TimeHistoryFileInfo *_pthfi)
{
	TimeHistoryFileInfo_HDF5 *pthfi = (TimeHistoryFileInfo_HDF5 *)_pthfi;
	hid_t mem_dataspace_id, file_dataspace_id;
	hsize_t dims[2], start[2], count[2];
	TimeRecord_HDF5 time_rcd;

	time_rcd.step_index = step->get_id();
	time_rcd.substep_num = step->get_substep_num();
	time_rcd.total_substep_num = step->get_total_substep_num();
	time_rcd.current_time = step->get_current_time();
	time_rcd.total_time = step->get_total_time();
	time_rcd.field_data_num = pthfi->time_history->get_data_num();

	// define dataset dimension in memory
	dims[0] = 1;
	dims[1] = TimeRecordDataNum;
	mem_dataspace_id = H5Screate_simple(2, dims, NULL);
	// params of hyperslab
	count[0] = dims[0];
	count[1] = dims[1];
	start[0] = pthfi->time_rcd_num;
	start[1] = 0;
	// expand dataset dimension in file
	dims[0] += pthfi->time_rcd_num;
	pthfi->time_rcd_num = dims[0];
	H5Dset_extent(pthfi->time_rcd_dset_id, dims);
	file_dataspace_id = H5Dget_space(pthfi->time_rcd_dset_id);
	// create hyperslab
	H5Sselect_hyperslab(file_dataspace_id, H5S_SELECT_SET, start, NULL, count, NULL);
	// write data to hdf5 file
	H5Dwrite(pthfi->time_rcd_dset_id, double_datatype_id,
		mem_dataspace_id, file_dataspace_id, H5P_DEFAULT, &time_rcd);
	// clear
	H5Sclose(mem_dataspace_id);
	H5Sclose(file_dataspace_id);

	return 0;
}

double *ResultFile_HDF5::get_field_data_buffer(TimeHistoryFileInfo *_pthfi, size_t &point_num)
{
	TimeHistoryFileInfo_HDF5 *pthfi = (TimeHistoryFileInfo_HDF5 *)_pthfi;
	size_t max_empty_row_line;
	double *res_buffer;

	max_empty_row_line = pthfi->buffer_row_num - pthfi->buffer_used_row_num;
	if (!max_empty_row_line)
	{
		flush_field_data_buffer(_pthfi);
		max_empty_row_line = pthfi->buffer_row_num - pthfi->buffer_used_row_num;
		// buffer may be two small (smaller than field_num)
		if (!max_empty_row_line) return nullptr;
	}
	
	res_buffer = (double *)pthfi->buffer + pthfi->buffer_used_row_num * pthfi->field_num;
	point_num = point_num < max_empty_row_line ? point_num : max_empty_row_line;
	pthfi->buffer_used_row_num += point_num;
	return res_buffer;
}

int ResultFile_HDF5::flush_field_data_buffer(TimeHistoryFileInfo *_pthfi)
{
	TimeHistoryFileInfo_HDF5 *pthfi = (TimeHistoryFileInfo_HDF5 *)_pthfi;
	hid_t mem_dataspace_id, file_dataspace_id;
	hsize_t dims[2], start[2], count[2];

	if (!pthfi->buffer_used_row_num) return 0;

	// define dataset dimension in memory
	dims[0] = pthfi->buffer_used_row_num;
	dims[1] = pthfi->field_num;
	mem_dataspace_id = H5Screate_simple(2, dims, NULL);
	// params of hyperslab
	count[0] = dims[0];
	count[1] = dims[1];
	start[0] = pthfi->row_num;
	start[1] = 0;
	// expand dataset dimension in file
	dims[0] += pthfi->row_num;
	pthfi->row_num = dims[0];
	H5Dset_extent(pthfi->fld_dset_id, dims);
	file_dataspace_id = H5Dget_space(pthfi->fld_dset_id);
	// create hyperslab
	H5Sselect_hyperslab(file_dataspace_id, H5S_SELECT_SET, start, NULL, count, NULL);
	// write data to hdf5 file
	H5Dwrite(pthfi->fld_dset_id, double_datatype_id, 
		mem_dataspace_id, file_dataspace_id, H5P_DEFAULT, pthfi->buffer);
	// clear
	H5Sclose(mem_dataspace_id);
	H5Sclose(file_dataspace_id);

	pthfi->buffer_used_row_num = 0;

	return 0;
}