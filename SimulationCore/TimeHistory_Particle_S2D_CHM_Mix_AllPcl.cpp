#include "SimulationCore_pcp.h"

#include "Step_S2D_CHM_MPM_s_Mix.h"
#include "ResultFile.h"

#include "TimeHistory_Particle_S2D_CHM_Mix_AllPcl.h"

/*====================================================================
TimeHistory_Particle_S2D_CHM_Mix_AllPcl Class
 =====================================================================*/
TimeHistory_Particle_S2D_CHM_Mix_AllPcl::TimeHistory_Particle_S2D_CHM_Mix_AllPcl() :
	TimeHistory(TimeHistoryType::Particle_S2D_CHM_AllPcl, "Particle_S2D_CHM_AllPcl"),
	fld_mem(10) {}

TimeHistory_Particle_S2D_CHM_Mix_AllPcl::~TimeHistory_Particle_S2D_CHM_Mix_AllPcl() {}

int TimeHistory_Particle_S2D_CHM_Mix_AllPcl::add_field(Particle_Field_2D_CHM fld)
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

int TimeHistory_Particle_S2D_CHM_Mix_AllPcl::init_per_step(void)
{
	Model_S2D_CHM_MPM_s_Mix &model
		= static_cast<Model_S2D_CHM_MPM_s_Mix &>(*step->get_model());
	
	pcl_mem.reset();
	pcl_mem.reserve(model.pcl_num);
	Particle_S2D_CHM_Mix *ppcl;
	for (size_t i = 0; i < model.pcl_num; ++i)
	{
		ppcl = model.pcls + i;
		pcl_mem.add(ppcl);
	}
	point_num = pcl_mem.get_num();
	pcls = pcl_mem.get_mem();

	fld_infos = fld_mem.get_mem();

	return 0;
}

void TimeHistory_Particle_S2D_CHM_Mix_AllPcl::finalize_per_step(void) {}

int TimeHistory_Particle_S2D_CHM_Mix_AllPcl::output(void)
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
#define FIELD_MAX_NUM 40
const unsigned short int TimeHistory_Particle_S2D_CHM_Mix_AllPcl::field_max_num = FIELD_MAX_NUM;
const TimeHistory_Particle_S2D_CHM_Mix_AllPcl::TimeHistoryFieldFunc
TimeHistory_Particle_S2D_CHM_Mix_AllPcl::output_field_funcs_map[FIELD_MAX_NUM] =
{
	nullptr,            // 0
	&output_x,          // 1
	&output_y,          // 2
	&output_vol,        // 3
	&output_n,          // 4
	&output_density_s,  // 5
	&output_density_f,  // 6
	&output_m_s,        // 7
	&output_m_f,        // 8
	nullptr,            // 9
	nullptr,            // 10
	&output_vx_s,       // 11
	&output_vy_s,       // 12
	&output_vx_f,       // 13
	&output_vy_f,       // 14
	&output_mvx_s,      // 15
	&output_mvy_s,      // 16
	&output_mvx_f,      // 17
	&output_mvy_f,      // 18
	nullptr,            // 19
	nullptr,            // 20
	&output_s11,        // 21
	&output_s22,        // 22
	nullptr,            // 23
	&output_s12,        // 24
	nullptr,            // 25
	nullptr,            // 26
	&output_p,          // 27
	nullptr,            // 28
	nullptr,            // 29
	nullptr,            // 30
	&output_e11,        // 31
	&output_e22,        // 32
	&output_e12,        // 33
	&output_es11,       // 34
	&output_es22,       // 35
	&output_es12,       // 36
	&output_ps11,       // 37
	&output_ps22,       // 38
	&output_ps12,       // 39
};

void TimeHistory_Particle_S2D_CHM_Mix_AllPcl::output_x(void)
{
	double *pdata = buffer_pos;
	for (size_t i = pcl_id_range_start; i < pcl_id_range_end; ++i)
	{
		*pdata = pcls[i]->x;
		pdata += field_num;
	}
}

void TimeHistory_Particle_S2D_CHM_Mix_AllPcl::output_y(void)
{
	double *pdata = buffer_pos;
	for (size_t i = pcl_id_range_start; i < pcl_id_range_end; ++i)
	{
		*pdata = pcls[i]->y;
		pdata += field_num;
	}
}

void TimeHistory_Particle_S2D_CHM_Mix_AllPcl::output_vol(void)
{
	double *pdata = buffer_pos;
	for (size_t i = pcl_id_range_start; i < pcl_id_range_end; ++i)
	{
		*pdata = pcls[i]->m_s / (pcls[i]->density_s * (1.0 - pcls[i]->n));
		pdata += field_num;
	}
}

void TimeHistory_Particle_S2D_CHM_Mix_AllPcl::output_n(void)
{
	double *pdata = buffer_pos;
	for (size_t i = pcl_id_range_start; i < pcl_id_range_end; ++i)
	{
		*pdata = pcls[i]->n;
		pdata += field_num;
	}
}

void TimeHistory_Particle_S2D_CHM_Mix_AllPcl::output_density_s(void)
{
	double *pdata = buffer_pos;
	for (size_t i = pcl_id_range_start; i < pcl_id_range_end; ++i)
	{
		*pdata = pcls[i]->density_s;
		pdata += field_num;
	}
}

void TimeHistory_Particle_S2D_CHM_Mix_AllPcl::output_density_f(void)
{
	double *pdata = buffer_pos;
	for (size_t i = pcl_id_range_start; i < pcl_id_range_end; ++i)
	{
		*pdata = pcls[i]->density_f;
		pdata += field_num;
	}
}

void TimeHistory_Particle_S2D_CHM_Mix_AllPcl::output_m_s(void)
{
	double *pdata = buffer_pos;
	for (size_t i = pcl_id_range_start; i < pcl_id_range_end; ++i)
	{
		*pdata = pcls[i]->m_s;
		pdata += field_num;
	}
}

void TimeHistory_Particle_S2D_CHM_Mix_AllPcl::output_m_f(void)
{
	double *pdata = buffer_pos;
	for (size_t i = pcl_id_range_start; i < pcl_id_range_end; ++i)
	{
		*pdata = pcls[i]->m_s / (pcls[i]->density_s * (1.0 - pcls[i]->n)) * pcls[i]->density_f * pcls[i]->n;
		pdata += field_num;
	}
}

void TimeHistory_Particle_S2D_CHM_Mix_AllPcl::output_vx_s(void)
{
	double *pdata = buffer_pos;
	for (size_t i = pcl_id_range_start; i < pcl_id_range_end; ++i)
	{
		*pdata = pcls[i]->vx_s;
		pdata += field_num;
	}
}

void TimeHistory_Particle_S2D_CHM_Mix_AllPcl::output_vy_s(void)
{
	double *pdata = buffer_pos;
	for (size_t i = pcl_id_range_start; i < pcl_id_range_end; ++i)
	{
		*pdata = pcls[i]->vy_s;
		pdata += field_num;
	}
}

void TimeHistory_Particle_S2D_CHM_Mix_AllPcl::output_vx_f(void)
{
	double *pdata = buffer_pos;
	for (size_t i = pcl_id_range_start; i < pcl_id_range_end; ++i)
	{
		*pdata = pcls[i]->vx_f;
		pdata += field_num;
	}
}

void TimeHistory_Particle_S2D_CHM_Mix_AllPcl::output_vy_f(void)
{
	double *pdata = buffer_pos;
	for (size_t i = pcl_id_range_start; i < pcl_id_range_end; ++i)
	{
		*pdata = pcls[i]->vy_f;
		pdata += field_num;
	}
}

void TimeHistory_Particle_S2D_CHM_Mix_AllPcl::output_mvx_s(void)
{
	double *pdata = buffer_pos;
	for (size_t i = pcl_id_range_start; i < pcl_id_range_end; ++i)
	{
		*pdata = pcls[i]->vx_s * pcls[i]->m_s;
		pdata += field_num;
	}
}

void TimeHistory_Particle_S2D_CHM_Mix_AllPcl::output_mvy_s(void)
{
	double *pdata = buffer_pos;
	for (size_t i = pcl_id_range_start; i < pcl_id_range_end; ++i)
	{
		*pdata = pcls[i]->vy_s * pcls[i]->m_s;
		pdata += field_num;
	}
}

void TimeHistory_Particle_S2D_CHM_Mix_AllPcl::output_mvx_f(void)
{
	double *pdata = buffer_pos;
	for (size_t i = pcl_id_range_start; i < pcl_id_range_end; ++i)
	{
		double vol = pcls[i]->m_s / (pcls[i]->density_s * (1.0 - pcls[i]->n));
		*pdata = pcls[i]->vx_f * vol * pcls[i]->density_f * pcls[i]->n;
		pdata += field_num;
	}
}

void TimeHistory_Particle_S2D_CHM_Mix_AllPcl::output_mvy_f(void)
{
	double *pdata = buffer_pos;
	for (size_t i = pcl_id_range_start; i < pcl_id_range_end; ++i)
	{
		double vol = pcls[i]->m_s / (pcls[i]->density_s * (1.0 - pcls[i]->n));
		*pdata = pcls[i]->vy_f * vol * pcls[i]->density_f * pcls[i]->n;
		pdata += field_num;
	}
}

void TimeHistory_Particle_S2D_CHM_Mix_AllPcl::output_s11(void)
{
	double *pdata = buffer_pos;
	for (size_t i = pcl_id_range_start; i < pcl_id_range_end; ++i)
	{
		*pdata = pcls[i]->s11;
		pdata += field_num;
	}
}

void TimeHistory_Particle_S2D_CHM_Mix_AllPcl::output_s22(void)
{
	double *pdata = buffer_pos;
	for (size_t i = pcl_id_range_start; i < pcl_id_range_end; ++i)
	{
		*pdata = pcls[i]->s22;
		pdata += field_num;
	}
}

void TimeHistory_Particle_S2D_CHM_Mix_AllPcl::output_s12(void)
{
	double *pdata = buffer_pos;
	for (size_t i = pcl_id_range_start; i < pcl_id_range_end; ++i)
	{
		*pdata = pcls[i]->s12;
		pdata += field_num;
	}
}

void TimeHistory_Particle_S2D_CHM_Mix_AllPcl::output_p(void)
{
	double *pdata = buffer_pos;
	for (size_t i = pcl_id_range_start; i < pcl_id_range_end; ++i)
	{
		*pdata = pcls[i]->p;
		pdata += field_num;
	}
}

void TimeHistory_Particle_S2D_CHM_Mix_AllPcl::output_e11(void)
{
	double *pdata = buffer_pos;
	for (size_t i = pcl_id_range_start; i < pcl_id_range_end; ++i)
	{
		*pdata = pcls[i]->e11;
		pdata += field_num;
	}
}

void TimeHistory_Particle_S2D_CHM_Mix_AllPcl::output_e22(void)
{
	double *pdata = buffer_pos;
	for (size_t i = pcl_id_range_start; i < pcl_id_range_end; ++i)
	{
		*pdata = pcls[i]->e22;
		pdata += field_num;
	}
}

void TimeHistory_Particle_S2D_CHM_Mix_AllPcl::output_e12(void)
{
	double *pdata = buffer_pos;
	for (size_t i = pcl_id_range_start; i < pcl_id_range_end; ++i)
	{
		*pdata = pcls[i]->e12;
		pdata += field_num;
	}
}

void TimeHistory_Particle_S2D_CHM_Mix_AllPcl::output_es11(void)
{
	double *pdata = buffer_pos;
	for (size_t i = pcl_id_range_start; i < pcl_id_range_end; ++i)
	{
		*pdata = pcls[i]->es11;
		pdata += field_num;
	}
}

void TimeHistory_Particle_S2D_CHM_Mix_AllPcl::output_es22(void)
{
	double *pdata = buffer_pos;
	for (size_t i = pcl_id_range_start; i < pcl_id_range_end; ++i)
	{
		*pdata = pcls[i]->es22;
		pdata += field_num;
	}
}

void TimeHistory_Particle_S2D_CHM_Mix_AllPcl::output_es12(void)
{
	double *pdata = buffer_pos;
	for (size_t i = pcl_id_range_start; i < pcl_id_range_end; ++i)
	{
		*pdata = pcls[i]->es12;
		pdata += field_num;
	}
}

void TimeHistory_Particle_S2D_CHM_Mix_AllPcl::output_ps11(void)
{
	double *pdata = buffer_pos;
	for (size_t i = pcl_id_range_start; i < pcl_id_range_end; ++i)
	{
		*pdata = pcls[i]->ps11;
		pdata += field_num;
	}
}

void TimeHistory_Particle_S2D_CHM_Mix_AllPcl::output_ps22(void)
{
	double *pdata = buffer_pos;
	for (size_t i = pcl_id_range_start; i < pcl_id_range_end; ++i)
	{
		*pdata = pcls[i]->ps22;
		pdata += field_num;
	}
}

void TimeHistory_Particle_S2D_CHM_Mix_AllPcl::output_ps12(void)
{
	double *pdata = buffer_pos;
	for (size_t i = pcl_id_range_start; i < pcl_id_range_end; ++i)
	{
		*pdata = pcls[i]->ps12;
		pdata += field_num;
	}
}
