#include "SimulationCore_pcp.h"

#include <cassert>
#include <string>
#include <fstream>

#include "PreAllocStringBuffer.hpp"
#include "common_utilities.h"

#include "Step.h"
#include "TimeHistory.h"
#include "ResultFile_Text.h"

#include "Model_R2D_ME_MPM_s.h"
#include "Model_R2D_CHM_MPM_s.h"
#include "Model_R2D_ME_MPM_BSpline_s.h"
#include "Model_R2D_CHM_MPM_BSpline_s.h"
#include "Model_S2D_ME_MPM_s.h"

// Information for step
struct Step_Text
{
	std::string name;
	unsigned long long index;
	// time at the beginning
	double start_time;
	// time length
	double step_time;
	// substep_index at the beginning
	unsigned long long start_substep_index;
	// number of substep of this step
	unsigned long long substep_num;
};

// Information for time record
struct TimeRecord_Text
{
	unsigned long long step_index;
	unsigned long long substep_index;
	unsigned long long total_substep_index;
	double current_time;
	double total_time;
	unsigned long long point_num;
};

/*=================================================
Class TimeHistoryFileInfo_Text
=================================================*/
TimeHistoryFileInfo_Text::TimeHistoryFileInfo_Text() :
	buffer(nullptr), buffer_row_num(0), buffer_used_row_num(0),
	text_buffer(nullptr), text_buffer_size(0), text_buffer_used_size(0),
	field_num(0) {}

TimeHistoryFileInfo_Text::~TimeHistoryFileInfo_Text()
{
	// output section end tag
	file << "*end\n";

	if (buffer)
	{
		delete[] buffer;
		buffer = nullptr;
	}
	if (text_buffer)
	{
		delete[] text_buffer;
		text_buffer = nullptr;
	}
	if (file.is_open())
	{
		file.flush();
		file.close();
	}
}

/*=================================================
Class ResultFile_Text
=================================================*/
ResultFile_Text::ResultFile_Text() {}

ResultFile_Text::~ResultFile_Text() { close(); }

int ResultFile_Text::init(void)
{
	// only init once
	if (is_init) return 0;
	is_init = true;
	
	make_dir(file_name.c_str(), nullptr);

	return 0;
}

void ResultFile_Text::close(void)
{
	while (time_history_list)
		del_output(time_history_list);
}

int ResultFile_Text::init_per_step(void)
{
	TimeHistory *pth, *pth_tmp;
	TimeHistoryFileInfo_Text *pthfi;

	for (pth = first_th(); pth; pth = next_th(pth))
	{
		// delete output that is not used anymore
		if (!get_th_step(pth))
		{
			// remove the time history
			pth_tmp = pth;
			pth = prev_th(pth);
			del_output(pth_tmp);
		}
		// create handle for output newly added (which is without handle)
		else if (!get_th_file_info(pth))
		{
			pthfi = new TimeHistoryFileInfo_Text;
			set_th_file_info(pth, pthfi);
			// ************ inital file buffer ************
			pthfi->field_num = pth->get_field_num();
			pthfi->buffer_row_num = pth->get_point_num() * 2;
			pthfi->buffer = new char[pthfi->buffer_row_num * pthfi->field_num * sizeof(double)];
			pthfi->text_buffer_size = pthfi->buffer_row_num * pthfi->field_num * 22;
			pthfi->text_buffer = new char[pthfi->text_buffer_size];

			MemoryUtilities::PreAllocStringBuffer<256> th_file_name;
			th_file_name << file_name << "\\TimeHistory"
						 << pth->get_id()   << "-"
						 << pth->get_name() << ".txt";
			pthfi->file.open(th_file_name.c_str());
			if (!pthfi->file) return -2;
			
			// output section header
			MemoryUtilities::PreAllocStringBuffer<200> output_header;
			output_header << "**\n\n"
							 "Time History\n"
							 "\n**\n\n";
			output_header << "*TimeHistory\n\n";
			// output name
			output_header << "  Name = " << pth->get_name() << "\n";
			// output index
			output_header << "  Index = " << pth->get_id() << "\n";
			// output type
			output_header << "  Type = " << pth->get_type_name() << "\n";
			// interval number
			output_header << "  IntervalNum = " << pth->get_interval_num() << "\n";
			// field number
			output_header << "  FieldNum = " << pth->get_field_num() << "\n";
			output_header << "\n";
			pthfi->file.write(output_header.get_string(), output_header.get_length());
			//*********************************
		}
	}
	return 0;
}

int ResultFile_Text::finalize_per_step(void)
{
	TimeHistory *pth;
	TimeHistoryFileInfo_Text *pthfi;
	char str_tmp[25];
#define str_tmp_size (sizeof(str_tmp)/sizeof(char))

	// output step info
	MemoryUtilities::PreAllocStringBuffer<200> step_info;
	step_info << "  *Step\n";
	// name
	step_info << "    Name = ";
	step_info << step->get_name() << "\n";
	// index
	step_info << "    Index = " << step->get_index() << "\n";
	// start_time
	snprintf(str_tmp, str_tmp_size, "%.3e", step->get_total_time() - step->get_step_time());
	step_info << "    StartTime = " << str_tmp << "\n";
	// step_time
	snprintf(str_tmp, str_tmp_size, "%.3e", step->get_step_time());
	step_info << "    StepTime = " << str_tmp << "\n";
	// start_iteration_index
	step_info << "    StartSubstepIndex = "
		<< step->get_total_substep_num() - step->get_substep_num() << "\n";
	// iteration_num
	step_info << "    SubstepNum = " << step->get_substep_num() << "\n";

	step_info << "  *end\n\n";

	for (pth = first_th(); pth; pth = next_th(pth))
	{
		pthfi = static_cast<TimeHistoryFileInfo_Text *>(get_th_file_info(pth));
		pthfi->file.write(step_info.get_string(), step_info.get_length());
	}
#undef str_tmp_size
	return 0;
}

int ResultFile_Text::init_time_record(TimeHistoryFileInfo *_pthfi)
{
	TimeHistoryFileInfo_Text *pthfi = static_cast<TimeHistoryFileInfo_Text *>(_pthfi);
	MemoryUtilities::PreAllocStringBuffer<250> time_rcd_header;
	char str_tmp[25];
#define str_tmp_size (sizeof(str_tmp)/sizeof(char))
	// section name
	time_rcd_header << "  *TimeRecord\n";
	// time record id
	time_rcd_header << "    Index = " << pthfi->time_history->get_time_rcd_id() << "\n";
	// step index
	time_rcd_header << "    StepIndex = " << step->get_index() << "\n";
	// substep number
	time_rcd_header << "    SubstepNum = " << step->get_substep_num() << "\n";
	// total substep number
	time_rcd_header << "    TotalSubstepNum = "
					<< step->get_total_substep_num() << "\n";
	// current time
	snprintf(str_tmp, str_tmp_size, "%.3e", step->get_current_time());
	time_rcd_header << "    CurrentTime = " << str_tmp << "\n";
	// total time
	snprintf(str_tmp, str_tmp_size, "%.3e", step->get_total_time());
	time_rcd_header << "    TotalTime = " << str_tmp << "\n";
	// point number
	time_rcd_header << "    PointNum = "
					<< pthfi->time_history->get_point_num() << "\n";

	time_rcd_header << "    *FieldData\n";
	
	pthfi->file.write(time_rcd_header.get_string(), time_rcd_header.get_length());
#undef str_tmp_size
	return 0;
}

int ResultFile_Text::finalize_time_record(TimeHistoryFileInfo *_pthfi)
{
	TimeHistoryFileInfo_Text *pthfi = static_cast<TimeHistoryFileInfo_Text *>(_pthfi);

	// flush buffer
	flush_field_data_buffer(pthfi);

	// output end tag of the time record
	pthfi->file << "    *end\n  *end\n\n";

	return 0;
}

double *ResultFile_Text::get_field_data_buffer(TimeHistoryFileInfo *_pthfi, size_t &point_num)
{
	TimeHistoryFileInfo_Text *pthfi = static_cast<TimeHistoryFileInfo_Text *>(_pthfi);
	size_t max_empty_row_line;
	double *res_buffer;

	max_empty_row_line = pthfi->buffer_row_num - pthfi->buffer_used_row_num;
	if (!max_empty_row_line)
	{
		flush_field_data_buffer(_pthfi);
		max_empty_row_line = pthfi->buffer_row_num - pthfi->buffer_used_row_num;
		// buffer may be too small (smaller than field_num)
		if (!max_empty_row_line) return nullptr;
	}

	res_buffer = reinterpret_cast<double *>(pthfi->buffer) + pthfi->buffer_used_row_num * pthfi->field_num;
	point_num = point_num < max_empty_row_line ? point_num : max_empty_row_line;
	pthfi->buffer_used_row_num += point_num;
	return res_buffer;
}

int ResultFile_Text::flush_field_data_buffer(TimeHistoryFileInfo *_pthfi)
{
	TimeHistoryFileInfo_Text *pthfi = static_cast<TimeHistoryFileInfo_Text *>(_pthfi);
	char str_tmp[32];
#define str_tmp_size (sizeof(str_tmp)/sizeof(char))
	size_t str_len;
	size_t buf_pos;

	if (!pthfi->buffer) return 0;

	pthfi->text_buffer_used_size = 0;
	buf_pos = 0;
	for (size_t i = 0; i < pthfi->buffer_used_row_num; i++)
	{
		for (size_t j = 1; j < pthfi->field_num; j++)
		{
			str_len = snprintf(str_tmp, str_tmp_size, "%26.10e,", (reinterpret_cast<double *>(pthfi->buffer))[buf_pos]);
			strcpy(pthfi->text_buffer + pthfi->text_buffer_used_size, str_tmp);
			pthfi->text_buffer_used_size += str_len;
			++buf_pos;
		}
		str_len = snprintf(str_tmp, str_tmp_size, "%26.10e\n", (reinterpret_cast<double *>(pthfi->buffer))[buf_pos]);
		strcpy(pthfi->text_buffer + pthfi->text_buffer_used_size, str_tmp);
		pthfi->text_buffer_used_size += str_len;
		++buf_pos;
	}
	pthfi->file.write(pthfi->text_buffer, pthfi->text_buffer_used_size);

	pthfi->buffer_used_row_num = 0;

	return 0;
}


int ResultFile_Text::output_model_state(Model_R2D_ME_MPM_s &md)
{
	init();

	MemoryUtilities::PreAllocStringBuffer<256> model_data_file_name;
	model_data_file_name << file_name << "\\ModelState.txt";
	std::ofstream model_data_file(model_data_file_name.c_str());

	model_data_file << "**\n\nModel State Data\n\n**\n\n";

	model_data_file << "*ModelState\n";
	if (md.get_name() && strlen(md.get_name()))
		model_data_file << "  Name = " << md.get_name() << "\n";
	model_data_file << "  Type = " << md.get_type() << "\n";

	// output background mesh
	model_data_file << "\n  *BackgroundMesh\n"
		<< "    Type = R2D\n"
		<< "    XCoordinateNum = " << md.node_x_num << "\n"
		<< "    YCoordinateNum = " << md.node_y_num << "\n";
	if (md.node_x_num)
	{
		model_data_file << "    XCoordinates = ";
		for (size_t i = 0; i < md.node_x_num - 1; i++)
			model_data_file << md.node_coords_x[i] << ", ";
		model_data_file << md.node_coords_x[md.node_x_num - 1];
	}
	if (md.node_y_num)
	{
		model_data_file << "\n    YCoordinates = ";
		for (size_t i = 0; i < md.node_y_num - 1; i++)
			model_data_file << md.node_coords_y[i] << ", ";
		model_data_file << md.node_coords_y[md.node_y_num - 1];
	}
	model_data_file << "\n  *end\n";

	// output particles
	model_data_file << "\n  *Object\n"
		<< "    Type = ME_MPM\n" // Object->get_type() in the future
		<< "    ParticleNum = " << md.pcl_num << "\n";
	if (md.pcl_num)
	{
		// x
		model_data_file << "    x = ";
		for (size_t i = 0; i < md.pcl_num - 1; i++)
			model_data_file << md.pcls[i].x << ", ";
		model_data_file << md.pcls[md.pcl_num - 1].x;
		// y
		model_data_file << "\n    y = ";
		for (size_t i = 0; i < md.pcl_num - 1; i++)
			model_data_file << md.pcls[i].y << ", ";
		model_data_file << md.pcls[md.pcl_num - 1].y;
		// vol
		model_data_file << "\n    vol = ";
		for (size_t i = 0; i < md.pcl_num - 1; i++)
			model_data_file << md.pcls[i].m / md.pcls[i].density << ", ";
		model_data_file << md.pcls[md.pcl_num - 1].m / md.pcls[md.pcl_num - 1].density;
		// density_s;
		model_data_file << "\n    density = ";
		for (size_t i = 0; i < md.pcl_num - 1; i++)
			model_data_file << md.pcls[i].density << ", ";
		model_data_file << md.pcls[md.pcl_num - 1].density;
		// constitutive model
	}
	model_data_file << "\n  *end\n";

	// output boundary conditions
	//...

	model_data_file << "\n*end\n";
	model_data_file.close();

	return 0;
}


int ResultFile_Text::output_model_state(Model_R2D_CHM_MPM_s &md)
{
	init();

	MemoryUtilities::PreAllocStringBuffer<256> model_data_file_name;
	model_data_file_name << file_name << "\\ModelState.txt";
	std::ofstream model_data_file(model_data_file_name.c_str());

	model_data_file << "**\n\nModel State Data\n\n**\n\n";

	model_data_file << "*ModelState\n";
	if (md.get_name() && strlen(md.get_name()))
		model_data_file << "  Name = " << md.get_name() << "\n";
	model_data_file << "  Type = " << md.get_type() << "\n";

	// output background mesh
	model_data_file << "\n  *BackgroundMesh\n"
					<< "    Type = R2D\n"
					<< "    XCoordinateNum = " << md.node_x_num << "\n"
					<< "    YCoordinateNum = " << md.node_y_num << "\n";
	if (md.node_x_num)
	{
		model_data_file << "    XCoordinates = ";
		for (size_t i = 0; i < md.node_x_num - 1; i++)
			model_data_file << md.node_coords_x[i] << ", ";
		model_data_file << md.node_coords_x[md.node_x_num - 1];
	}
	if (md.node_y_num)
	{
		model_data_file << "\n    YCoordinates = ";
		for (size_t i = 0; i < md.node_y_num - 1; i++)
			model_data_file << md.node_coords_y[i] << ", ";
		model_data_file << md.node_coords_y[md.node_y_num - 1];
	}
	model_data_file << "\n  *end\n";

	// output particles
	model_data_file << "\n  *Object\n"
					<< "    Type = CHM_MPM\n"
					<< "    ParticleNum = " << md.pcl_num << "\n";
	if (md.pcl_num)
	{
		// x
		model_data_file << "    x = ";
		for (size_t i = 0; i < md.pcl_num - 1; i++)
			model_data_file << md.pcls[i].x << ", ";
		model_data_file << md.pcls[md.pcl_num - 1].x;
		// y
		model_data_file << "\n    y = ";
		for (size_t i = 0; i < md.pcl_num - 1; i++)
			model_data_file << md.pcls[i].y << ", ";
		model_data_file << md.pcls[md.pcl_num - 1].y;
		// vol
		model_data_file << "\n    vol = ";
		for (size_t i = 0; i < md.pcl_num - 1; i++)
			model_data_file << md.pcls[i].vol << ", ";
		model_data_file << md.pcls[md.pcl_num - 1].vol;
		// n
		model_data_file << "\n    n = ";
		for (size_t i = 0; i < md.pcl_num - 1; i++)
			model_data_file << md.pcls[i].n << ", ";
		model_data_file << md.pcls[md.pcl_num - 1].n;
		// density_s;
		model_data_file << "\n    density_s = ";
		for (size_t i = 0; i < md.pcl_num - 1; i++)
			model_data_file << md.pcls[i].density_s << ", ";
		model_data_file << md.pcls[md.pcl_num - 1].density_s;
		// density_f;
		model_data_file << "\n    density_f = ";
		for (size_t i = 0; i < md.pcl_num - 1; i++)
			model_data_file << md.pcls[i].density_f << ", ";
		model_data_file << md.pcls[md.pcl_num - 1].density_f;
		// constitutive model
		// permeability
	}
	model_data_file << "\n  *end\n";

	// output boundary conditions
	//...

	model_data_file << "\n*end\n";
	model_data_file.close();

	return 0;
}


int ResultFile_Text::output_model_state(Model_R2D_ME_MPM_BSpline_s &md)
{
	init();

	MemoryUtilities::PreAllocStringBuffer<256> model_data_file_name;
	model_data_file_name << file_name << "\\ModelState.txt";
	std::ofstream model_data_file(model_data_file_name.c_str());

	model_data_file << "**\n\nModel State Data\n\n**\n\n";

	model_data_file << "*ModelState\n";
	if (md.get_name() && strlen(md.get_name()))
		model_data_file << "  Name = " << md.get_name() << "\n";
	model_data_file << "  Type = " << md.get_type() << "\n";

	// output background mesh
	model_data_file << "\n  *BackgroundMesh\n"
		<< "    Type = R2D\n"
		<< "    XCoordinateNum = " << md.node_x_num << "\n"
		<< "    YCoordinateNum = " << md.node_y_num << "\n";
	if (md.node_x_num)
	{
		model_data_file << "    XCoordinates = ";
		for (size_t i = 0; i < md.node_x_num - 1; i++)
			model_data_file << md.x_start + i * md.h << ", ";
		model_data_file << md.x_start + (md.node_x_num-1) * md.h;
	}
	if (md.node_y_num)
	{
		model_data_file << "\n    YCoordinates = ";
		for (size_t i = 0; i < md.node_y_num - 1; i++)
			model_data_file << md.y_start + i * md.h << ", ";
		model_data_file << md.y_start + (md.node_y_num-1) * md.h;
	}
	model_data_file << "\n  *end\n";

	// output particles
	model_data_file << "\n  *Object\n"
		<< "    Type = ME_MPM\n"
		<< "    ParticleNum = " << md.pcl_num << "\n";
	if (md.pcl_num)
	{
		// x
		model_data_file << "    x = ";
		for (size_t i = 0; i < md.pcl_num - 1; i++)
			model_data_file << md.pcls[i].x << ", ";
		model_data_file << md.pcls[md.pcl_num - 1].x;
		// y
		model_data_file << "\n    y = ";
		for (size_t i = 0; i < md.pcl_num - 1; i++)
			model_data_file << md.pcls[i].y << ", ";
		model_data_file << md.pcls[md.pcl_num - 1].y;
		// vol
		model_data_file << "\n    vol = ";
		for (size_t i = 0; i < md.pcl_num - 1; i++)
			model_data_file << md.pcls[i].m / md.pcls[i].density << ", ";
		model_data_file << md.pcls[md.pcl_num-1].m / md.pcls[md.pcl_num - 1].density;
		// density;
		model_data_file << "\n    density = ";
		for (size_t i = 0; i < md.pcl_num - 1; i++)
			model_data_file << md.pcls[i].density << ", ";
		model_data_file << md.pcls[md.pcl_num - 1].density;
		// constitutive model
		// permeability
	}
	model_data_file << "\n  *end\n";

	// output boundary conditions
	//...

	model_data_file << "\n*end\n";
	model_data_file.close();

	return 0;
}


int ResultFile_Text::output_model_state(Model_R2D_CHM_MPM_BSpline_s &md)
{
	init();

	MemoryUtilities::PreAllocStringBuffer<256> model_data_file_name;
	model_data_file_name << file_name << "\\ModelState.txt";
	std::ofstream model_data_file(model_data_file_name.c_str());

	model_data_file << "**\n\nModel State Data\n\n**\n\n";

	model_data_file << "*ModelState\n";
	if (md.get_name() && strlen(md.get_name()))
		model_data_file << "  Name = " << md.get_name() << "\n";
	model_data_file << "  Type = " << md.get_type() << "\n";

	// output background mesh
	model_data_file << "\n  *BackgroundMesh\n"
		<< "    Type = R2D\n"
		<< "    XCoordinateNum = " << md.node_x_num << "\n"
		<< "    YCoordinateNum = " << md.node_y_num << "\n";
	if (md.node_x_num)
	{
		model_data_file << "    XCoordinates = ";
		for (size_t i = 0; i < md.node_x_num - 1; i++)
			model_data_file << md.x_start + i * md.h << ", ";
		model_data_file << md.x_start + (md.node_x_num - 1) * md.h;
	}
	if (md.node_y_num)
	{
		model_data_file << "\n    YCoordinates = ";
		for (size_t i = 0; i < md.node_y_num - 1; i++)
			model_data_file << md.y_start + i * md.h << ", ";
		model_data_file << md.y_start + (md.node_y_num - 1) * md.h;
	}
	model_data_file << "\n  *end\n";

	// output particles
	model_data_file << "\n  *Object\n"
		<< "    Type = CHM_MPM\n"
		<< "    ParticleNum = " << md.pcl_num << "\n";
	if (md.pcl_num)
	{
		// x
		model_data_file << "    x = ";
		for (size_t i = 0; i < md.pcl_num - 1; i++)
			model_data_file << md.pcls[i].x << ", ";
		model_data_file << md.pcls[md.pcl_num - 1].x;
		// y
		model_data_file << "\n    y = ";
		for (size_t i = 0; i < md.pcl_num - 1; i++)
			model_data_file << md.pcls[i].y << ", ";
		model_data_file << md.pcls[md.pcl_num - 1].y;
		// vol
		double vol;
		model_data_file << "\n    vol = ";
		for (size_t i = 0; i < md.pcl_num - 1; i++)
		{
			vol = md.pcls[i].m_s / ((1.0 - md.pcls[i].n) * md.pcls[i].density_s);
			model_data_file << vol << ", ";
		}
		vol = md.pcls[md.pcl_num - 1].m_s / ((1.0 - md.pcls[md.pcl_num - 1].n) * md.pcls[md.pcl_num - 1].density_s);
		model_data_file << vol;
		// solid density;
		model_data_file << "\n    density_s = ";
		for (size_t i = 0; i < md.pcl_num - 1; i++)
			model_data_file << md.pcls[i].density_s << ", ";
		model_data_file << md.pcls[md.pcl_num - 1].density_s;
		// fluid density
		model_data_file << "\n    density_f = ";
		for (size_t i = 0; i < md.pcl_num - 1; i++)
			model_data_file << md.pcls[i].density_f << ", ";
		model_data_file << md.pcls[md.pcl_num - 1].density_f;
		// constitutive model
		// permeability
	}
	model_data_file << "\n  *end\n";

	// output boundary conditions
	//...

	model_data_file << "\n*end\n";
	model_data_file.close();

	return 0;
}

int ResultFile_Text::output_model_state(Model_S2D_ME_MPM_s &md)
{
	init();

	MemoryUtilities::PreAllocStringBuffer<256> model_data_file_name;
	model_data_file_name << file_name << "\\ModelState.txt";
	std::ofstream model_data_file(model_data_file_name.c_str());

	model_data_file << "**\n\nModel State Data\n\n**\n\n";

	model_data_file << "*ModelState\n";
	if (md.get_name() && strlen(md.get_name()))
		model_data_file << "  Name = " << md.get_name() << "\n";
	model_data_file << "  Type = " << md.get_type() << "\n";

	// output background mesh
	model_data_file << "\n  *BackgroundMesh\n"
		<< "    Type = R2D\n"
		<< "    XCoordinateNum = " << md.node_x_num << "\n"
		<< "    YCoordinateNum = " << md.node_y_num << "\n";
	if (md.node_x_num)
	{
		model_data_file << "    XCoordinates = ";
		for (size_t i = 0; i < md.node_x_num - 1; i++)
			model_data_file << md.x0 + i * md.h << ", ";
		model_data_file << md.x0 + (md.node_x_num - 1) * md.h;
	}
	if (md.node_y_num)
	{
		model_data_file << "\n    YCoordinates = ";
		for (size_t i = 0; i < md.node_y_num - 1; i++)
			model_data_file << md.y0 + i * md.h << ", ";
		model_data_file << md.y0 + (md.node_y_num - 1) * md.h;
	}
	model_data_file << "\n  *end\n";

	// output particles
	model_data_file << "\n  *Object\n"
		<< "    Type = ME_MPM\n"
		<< "    ParticleNum = " << md.pcl_num << "\n";
	if (md.pcl_num)
	{
		// x
		model_data_file << "    x = ";
		for (size_t i = 0; i < md.pcl_num - 1; i++)
			model_data_file << md.pcls[i].x << ", ";
		model_data_file << md.pcls[md.pcl_num - 1].x;
		// y
		model_data_file << "\n    y = ";
		for (size_t i = 0; i < md.pcl_num - 1; i++)
			model_data_file << md.pcls[i].y << ", ";
		model_data_file << md.pcls[md.pcl_num - 1].y;
		// vol
		model_data_file << "\n    vol = ";
		for (size_t i = 0; i < md.pcl_num - 1; i++)
			model_data_file << md.pcls[i].m / md.pcls[i].density << ", ";
		model_data_file << md.pcls[md.pcl_num - 1].m / md.pcls[md.pcl_num - 1].density;
		// density;
		model_data_file << "\n    density = ";
		for (size_t i = 0; i < md.pcl_num - 1; i++)
			model_data_file << md.pcls[i].density << ", ";
		model_data_file << md.pcls[md.pcl_num - 1].density;
		// constitutive model
		// permeability
	}
	model_data_file << "\n  *end\n";

	// output boundary conditions
	//...

	model_data_file << "\n*end\n";
	model_data_file.close();

	return 0;
}