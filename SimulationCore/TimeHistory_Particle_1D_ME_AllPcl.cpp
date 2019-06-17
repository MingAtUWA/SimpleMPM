#include "SimulationCore_pcp.h"

#include "Model_R2D_ME_MPM.h"
#include "Step_R2D_ME_MPM.h"
#include "ResultFile.h"

#include "TimeHistory_Particle_1D_ME_AllPcl.h"

/*====================================================================
TimeHistory_Particle_1D_ME_AllPcl Class
 =====================================================================*/
TimeHistory_Particle_1D_ME_AllPcl::TimeHistory_Particle_1D_ME_AllPcl() :
	TimeHistory(TimeHistoryType::Particle_1D_ME_AllPcl, "Particle_1D_ME_AllPcl"),
	fld_mem(10) {}

TimeHistory_Particle_1D_ME_AllPcl::~TimeHistory_Particle_1D_ME_AllPcl() {}

int TimeHistory_Particle_1D_ME_AllPcl::add_field(Particle_Field_1D_ME fld)
{
	unsigned short int fld_us = (unsigned short int)fld;
	if (fld_us > field_max_num || !output_field_funcs_map[fld_us])
		return 0;

	FieldInfo fld_info;
	fld_info.fld_id = fld;
	fld_info.out_func = output_field_funcs_map[fld_us];
	fld_mem.add(&fld_info);
	++field_num;
	
	return 0;
}

int TimeHistory_Particle_1D_ME_AllPcl::init_per_step(void)
{
	Model_1D_ME_MPM_BSpline_s *model = static_cast<Model_1D_ME_MPM_BSpline_s *>(step->get_model());
	Particle_1D_ME *ppcl;

	pcl_mem.reserve(model->pcl_num);
	for (size_t i = 0; i < model->pcl_num; i++)
	{
		ppcl = model->pcls + i;
		pcl_mem.add(ppcl);
	}
	pcls = pcl_mem.get_mem();
	point_num = pcl_mem.get_num();

	fld_infos = fld_mem.get_mem();
	field_num = fld_mem.get_num();

	return 0;
}

void TimeHistory_Particle_1D_ME_AllPcl::finalize_per_step(void) {}

int TimeHistory_Particle_1D_ME_AllPcl::output(void)
{
	// Create new time record and get the first data buffer
	res_file->init_time_record(file_info);

	size_t out_pcl_num;
	pcl_id_range_start = 0;
	while (pcl_id_range_start < point_num)
	{
		// get buffer
		out_pcl_num = point_num - pcl_id_range_start;
		buffer_pos = res_file->get_field_data_buffer(this->file_info, out_pcl_num);

		// fill field data to buffer
		pcl_id_range_end = pcl_id_range_start + out_pcl_num;
		for (size_t i = 0; i < field_num; ++i)
		{
			(this->*(fld_infos[i].out_func))();
			++buffer_pos;
		}
		pcl_id_range_start = pcl_id_range_end;
	}

	// Complete time record output
	res_file->finalize_time_record(file_info);

	++time_rcd_id;
	return 0;
}

// Map listing relative location of each variable at the data point.
#define FIELD_MAX_NUM 29
const unsigned short int TimeHistory_Particle_1D_ME_AllPcl::field_max_num = FIELD_MAX_NUM;
const TimeHistory_Particle_1D_ME_AllPcl::TimeHistoryFieldFunc
TimeHistory_Particle_1D_ME_AllPcl::output_field_funcs_map[FIELD_MAX_NUM] =
{
	nullptr,            // 0
	&output_x,          // 1
	&output_density,    // 2
	&output_m,          // 3
	&output_mv,         // 4
	&output_v,          // 5
	&output_s11,        // 6
	&output_e11,        // 7
};

void TimeHistory_Particle_1D_ME_AllPcl::output_x(void)
{
	double *pdata = buffer_pos;
	for (size_t i = pcl_id_range_start; i < pcl_id_range_end; ++i)
	{
		*pdata = pcls[i]->x;
		pdata += field_num;
	}
}

void TimeHistory_Particle_1D_ME_AllPcl::output_density(void)
{
	double *pdata = buffer_pos;
	for (size_t i = pcl_id_range_start; i < pcl_id_range_end; ++i)
	{
		*pdata = pcls[i]->density;
		pdata += field_num;
	}
}

void TimeHistory_Particle_1D_ME_AllPcl::output_m(void)
{
	double *pdata = buffer_pos;
	for (size_t i = pcl_id_range_start; i < pcl_id_range_end; ++i)
	{
		*pdata = pcls[i]->m;
		pdata += field_num;
	}
}

void TimeHistory_Particle_1D_ME_AllPcl::output_mv(void)
{
	double *pdata = buffer_pos;
	for (size_t i = pcl_id_range_start; i < pcl_id_range_end; ++i)
	{
		*pdata = pcls[i]->v * pcls[i]->m;
		pdata += field_num;
	}
}

void TimeHistory_Particle_1D_ME_AllPcl::output_v(void)
{
	double *pdata = buffer_pos;
	for (size_t i = pcl_id_range_start; i < pcl_id_range_end; ++i)
	{
		*pdata = pcls[i]->v;
		pdata += field_num;
	}
}

void TimeHistory_Particle_1D_ME_AllPcl::output_s11(void)
{
	double *pdata = buffer_pos;
	for (size_t i = pcl_id_range_start; i < pcl_id_range_end; ++i)
	{
		*pdata = pcls[i]->s11;
		pdata += field_num;
	}
}

void TimeHistory_Particle_1D_ME_AllPcl::output_e11(void)
{
	double *pdata = buffer_pos;
	for (size_t i = pcl_id_range_start; i < pcl_id_range_end; ++i)
	{
		*pdata = pcls[i]->e11;
		pdata += field_num;
	}
}
