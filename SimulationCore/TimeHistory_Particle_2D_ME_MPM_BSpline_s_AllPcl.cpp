#include "SimulationCore_pcp.h"

#include "Model_R2D_ME_MPM_BSpline_s.h"
#include "Step_R2D_ME_MPM_BSpline_APIC_s.h"
#include "ResultFile.h"

#include "TimeHistory_Particle_2D_ME_MPM_BSpline_s_AllPcl.h"

/*====================================================================
TimeHistory_Particle_2D_ME_MPM_BSpline_s_AllPcl Class
 =====================================================================*/
TimeHistory_Particle_2D_ME_MPM_BSpline_s_AllPcl::TimeHistory_Particle_2D_ME_MPM_BSpline_s_AllPcl() :
	TimeHistory(TimeHistoryType::Particle_2D_ME_AllPcl, "Particle_2D_ME_AllPcl"),
	fld_mem(10) {}

TimeHistory_Particle_2D_ME_MPM_BSpline_s_AllPcl::~TimeHistory_Particle_2D_ME_MPM_BSpline_s_AllPcl() {}

int TimeHistory_Particle_2D_ME_MPM_BSpline_s_AllPcl::add_field(Particle_Field_2D_ME fld)
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

int TimeHistory_Particle_2D_ME_MPM_BSpline_s_AllPcl::init_per_step(void)
{
	Model_R2D_ME_MPM_BSpline_s *model
		= static_cast<Model_R2D_ME_MPM_BSpline_s *>(step->get_model());
	Particle_R2D_ME_Grid *ppcl;

	pcl_mem.reset();
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

void TimeHistory_Particle_2D_ME_MPM_BSpline_s_AllPcl::finalize_per_step(void) {}

int TimeHistory_Particle_2D_ME_MPM_BSpline_s_AllPcl::output(void)
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
#define FIELD_MAX_NUM 24
const unsigned short int TimeHistory_Particle_2D_ME_MPM_BSpline_s_AllPcl::field_max_num = FIELD_MAX_NUM;
const TimeHistory_Particle_2D_ME_MPM_BSpline_s_AllPcl::TimeHistoryFieldFunc
TimeHistory_Particle_2D_ME_MPM_BSpline_s_AllPcl::output_field_funcs_map[FIELD_MAX_NUM] =
{
	nullptr,            // 0
	&output_x,          // 1
	&output_y,          // 2
	&output_vol,        // 3
	&output_density,    // 4
	&output_m,          // 5
	&output_vx,         // 6
	&output_vy,         // 7
	&output_mvx,        // 8
	&output_mvy,        // 9
	nullptr,            // 10
	&output_s11,        // 11
	&output_s22,        // 12
	nullptr,            // 13
	&output_s12,        // 14
	nullptr,            // 15
	nullptr,            // 16
	nullptr,            // 17
	nullptr,            // 18
	nullptr,            // 19
	nullptr,            // 20
	&output_e11,        // 21
	&output_e22,        // 22
	&output_e12,        // 23
};

void TimeHistory_Particle_2D_ME_MPM_BSpline_s_AllPcl::output_x(void)
{
	double *pdata = buffer_pos;
	for (size_t i = pcl_id_range_start; i < pcl_id_range_end; ++i)
	{
		*pdata = pcls[i]->x;
		pdata += field_num;
	}
}

void TimeHistory_Particle_2D_ME_MPM_BSpline_s_AllPcl::output_y(void)
{
	double *pdata = buffer_pos;
	for (size_t i = pcl_id_range_start; i < pcl_id_range_end; ++i)
	{
		*pdata = pcls[i]->y;
		pdata += field_num;
	}
}

void TimeHistory_Particle_2D_ME_MPM_BSpline_s_AllPcl::output_vol(void)
{
	double *pdata = buffer_pos;
	for (size_t i = pcl_id_range_start; i < pcl_id_range_end; ++i)
	{
		*pdata = pcls[i]->m / pcls[i]->density;
		pdata += field_num;
	}
}

void TimeHistory_Particle_2D_ME_MPM_BSpline_s_AllPcl::output_density(void)
{
	double *pdata = buffer_pos;
	for (size_t i = pcl_id_range_start; i < pcl_id_range_end; ++i)
	{
		*pdata = pcls[i]->density;
		pdata += field_num;
	}
}

void TimeHistory_Particle_2D_ME_MPM_BSpline_s_AllPcl::output_m(void)
{
	double *pdata = buffer_pos;
	for (size_t i = pcl_id_range_start; i < pcl_id_range_end; ++i)
	{
		*pdata = pcls[i]->m;
		pdata += field_num;
	}
}

void TimeHistory_Particle_2D_ME_MPM_BSpline_s_AllPcl::output_vx(void)
{
	double *pdata = buffer_pos;
	for (size_t i = pcl_id_range_start; i < pcl_id_range_end; ++i)
	{
		*pdata = pcls[i]->vx;
		pdata += field_num;
	}
}

void TimeHistory_Particle_2D_ME_MPM_BSpline_s_AllPcl::output_vy(void)
{
	double *pdata = buffer_pos;
	for (size_t i = pcl_id_range_start; i < pcl_id_range_end; ++i)
	{
		*pdata = pcls[i]->vy;
		pdata += field_num;
	}
}

void TimeHistory_Particle_2D_ME_MPM_BSpline_s_AllPcl::output_mvx(void)
{
	double *pdata = buffer_pos;
	for (size_t i = pcl_id_range_start; i < pcl_id_range_end; ++i)
	{
		*pdata = pcls[i]->vx * pcls[i]->m;
		pdata += field_num;
	}
}

void TimeHistory_Particle_2D_ME_MPM_BSpline_s_AllPcl::output_mvy(void)
{
	double *pdata = buffer_pos;
	for (size_t i = pcl_id_range_start; i < pcl_id_range_end; ++i)
	{
		*pdata = pcls[i]->vy * pcls[i]->m;
		pdata += field_num;
	}
}

void TimeHistory_Particle_2D_ME_MPM_BSpline_s_AllPcl::output_s11(void)
{
	double *pdata = buffer_pos;
	for (size_t i = pcl_id_range_start; i < pcl_id_range_end; ++i)
	{
		*pdata = pcls[i]->s11;
		pdata += field_num;
	}
}

void TimeHistory_Particle_2D_ME_MPM_BSpline_s_AllPcl::output_s22(void)
{
	double *pdata = buffer_pos;
	for (size_t i = pcl_id_range_start; i < pcl_id_range_end; ++i)
	{
		*pdata = pcls[i]->s22;
		pdata += field_num;
	}
}

void TimeHistory_Particle_2D_ME_MPM_BSpline_s_AllPcl::output_s12(void)
{
	double *pdata = buffer_pos;
	for (size_t i = pcl_id_range_start; i < pcl_id_range_end; ++i)
	{
		*pdata = pcls[i]->s12;
		pdata += field_num;
	}
}

void TimeHistory_Particle_2D_ME_MPM_BSpline_s_AllPcl::output_e11(void)
{
	double *pdata = buffer_pos;
	for (size_t i = pcl_id_range_start; i < pcl_id_range_end; ++i)
	{
		*pdata = pcls[i]->e11;
		pdata += field_num;
	}
}

void TimeHistory_Particle_2D_ME_MPM_BSpline_s_AllPcl::output_e22(void)
{
	double *pdata = buffer_pos;
	for (size_t i = pcl_id_range_start; i < pcl_id_range_end; ++i)
	{
		*pdata = pcls[i]->e22;
		pdata += field_num;
	}
}

void TimeHistory_Particle_2D_ME_MPM_BSpline_s_AllPcl::output_e12(void)
{
	double *pdata = buffer_pos;
	for (size_t i = pcl_id_range_start; i < pcl_id_range_end; ++i)
	{
		*pdata = pcls[i]->e12;
		pdata += field_num;
	}
}
