#include "SimulationCore_pcp.h"

#include <cassert>

#include "Model_R2D_ME_MPM_s.h"
#include "Step_R2D_ME_MPM_s.h"
#include "ResultFile.h"

#include "TimeHistory_Particle_R2D_ME_s.h"

/*====================================================================
TimeHistory_Particle_R2D_ME_s Class
 =====================================================================*/
TimeHistory_Particle_R2D_ME_s::TimeHistory_Particle_R2D_ME_s() :
	TimeHistory(TimeHistoryType::Particle_R2D_ME_s, "Particle_R2D_ME_s") {}

TimeHistory_Particle_R2D_ME_s::~TimeHistory_Particle_R2D_ME_s() {}

int TimeHistory_Particle_R2D_ME_s::set_model_output(Model_R2D_ME_MPM_s *md,
	Particle_Field_2D_ME *fld_ids, size_t fld_num, size_t *pcl_ids, size_t pcl_num)
{
	assert(md);

	set_field_num(fld_num);
	for (size_t i = 0; i < fld_num; i++)
		add_field(fld_ids[i]);

	set_particle_num(pcl_num);
	for (size_t i = 0; i < pcl_num; i++)
		if (pcl_ids[i] < md->pcl_num)
			add_particle(1, md->pcls + pcl_ids[i]);

	return 0;
}

int TimeHistory_Particle_R2D_ME_s::add_particle(size_t obj_id, Particle_R2D_ME_s *pcl)
{
	if (!pcl) return 0;

	ParticleInfo pcl_info;
	pcl_info.obj_id = obj_id;
	pcl_info.pcl = pcl;
	pcl_mem.add(&pcl_info);
	++point_num;

	return 0;
}

int TimeHistory_Particle_R2D_ME_s::add_field(Particle_Field_2D_ME fld)
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

int TimeHistory_Particle_R2D_ME_s::init_per_step(void)
{
	pcl_infos = pcl_mem.get_mem();
	fld_infos = fld_mem.get_mem();
	return 0;
}

void TimeHistory_Particle_R2D_ME_s::finalize_per_step(void) {}

int TimeHistory_Particle_R2D_ME_s::output(void)
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
#define FIELD_MAX_NUM 30
const unsigned short int TimeHistory_Particle_R2D_ME_s::field_max_num = FIELD_MAX_NUM;
const TimeHistory_Particle_R2D_ME_s::TimeHistoryFieldFunc
TimeHistory_Particle_R2D_ME_s::output_field_funcs_map[FIELD_MAX_NUM] =
{
	nullptr,            // 0
	&output_x,          // 1
	&output_y,          // 2
	&output_vol,        // 3
	&output_density,    // 4
	&output_m,          // 5
	&output_v_x,        // 6
	&output_v_y,        // 7
	&output_momentum_x, // 8
	&output_momentum_y, // 9
	nullptr,            // 10
	&output_s11,        // 11
	&output_s22,        // 12
	&output_s33,        // 13
	&output_s12,        // 14
	&output_s23,        // 15
	&output_s31,        // 16
	nullptr,            // 17
	nullptr,            // 18
	nullptr,            // 19
	nullptr,            // 20
	&output_e11,        // 21
	&output_e22,        // 22
	&output_e12,        // 23
	&output_es11,       // 24
	&output_es22,       // 25
	&output_es12,       // 26
	&output_ps11,       // 27
	&output_ps22,       // 28
	&output_ps12,       // 29
};

void TimeHistory_Particle_R2D_ME_s::output_x(void)
{
	double *pdata = buffer_pos;
	for (size_t i = pcl_id_range_start; i < pcl_id_range_end; ++i)
	{
		*pdata = pcl_infos[i].pcl->x;
		pdata += field_num;
	}
}

void TimeHistory_Particle_R2D_ME_s::output_y(void)
{
	double *pdata = buffer_pos;
	for (size_t i = pcl_id_range_start; i < pcl_id_range_end; ++i)
	{
		*pdata = pcl_infos[i].pcl->y;
		pdata += field_num;
	}
}

void TimeHistory_Particle_R2D_ME_s::output_vol(void)
{
	double *pdata = buffer_pos;
	Particle_R2D_ME_s *ppcl;
	for (size_t i = pcl_id_range_start; i < pcl_id_range_end; ++i)
	{
		ppcl = pcl_infos[i].pcl;
		*pdata = ppcl->m / ppcl->density;
		pdata += field_num;
	}
}

void TimeHistory_Particle_R2D_ME_s::output_density(void)
{
	double *pdata = buffer_pos;
	for (size_t i = pcl_id_range_start; i < pcl_id_range_end; ++i)
	{
		*pdata = pcl_infos[i].pcl->density;
		pdata += field_num;
	}
}

void TimeHistory_Particle_R2D_ME_s::output_m(void)
{
	double *pdata = buffer_pos;
	for (size_t i = pcl_id_range_start; i < pcl_id_range_end; ++i)
	{
		*pdata = pcl_infos[i].pcl->m;
		pdata += field_num;
	}
}

void TimeHistory_Particle_R2D_ME_s::output_v_x(void)
{
	double *pdata = buffer_pos;
	for (size_t i = pcl_id_range_start; i < pcl_id_range_end; ++i)
	{
		*pdata = pcl_infos[i].pcl->mmx / pcl_infos[i].pcl->m;
		pdata += field_num;
	}
}

void TimeHistory_Particle_R2D_ME_s::output_v_y(void)
{
	double *pdata = buffer_pos;
	for (size_t i = pcl_id_range_start; i < pcl_id_range_end; ++i)
	{
		*pdata = pcl_infos[i].pcl->mmy / pcl_infos[i].pcl->m;
		pdata += field_num;
	}
}

void TimeHistory_Particle_R2D_ME_s::output_momentum_x(void)
{
	double *pdata = buffer_pos;
	for (size_t i = pcl_id_range_start; i < pcl_id_range_end; ++i)
	{
		*pdata = pcl_infos[i].pcl->mmx;
		pdata += field_num;
	}
}

void TimeHistory_Particle_R2D_ME_s::output_momentum_y(void)
{
	double *pdata = buffer_pos;
	for (size_t i = pcl_id_range_start; i < pcl_id_range_end; ++i)
	{
		*pdata = pcl_infos[i].pcl->mmy;
		pdata += field_num;
	}
}

void TimeHistory_Particle_R2D_ME_s::output_s11(void)
{
	double *pdata = buffer_pos;
	for (size_t i = pcl_id_range_start; i < pcl_id_range_end; ++i)
	{
		*pdata = pcl_infos[i].pcl->s11;
		pdata += field_num;
	}
}

void TimeHistory_Particle_R2D_ME_s::output_s22(void)
{
	double *pdata = buffer_pos;
	for (size_t i = pcl_id_range_start; i < pcl_id_range_end; ++i)
	{
		*pdata = pcl_infos[i].pcl->s22;
		pdata += field_num;
	}
}

void TimeHistory_Particle_R2D_ME_s::output_s33(void)
{
	double *pdata = buffer_pos;
	for (size_t i = pcl_id_range_start; i < pcl_id_range_end; ++i)
	{
		*pdata = pcl_infos[i].pcl->s33;
		pdata += field_num;
	}
}

void TimeHistory_Particle_R2D_ME_s::output_s12(void)
{
	double *pdata = buffer_pos;
	for (size_t i = pcl_id_range_start; i < pcl_id_range_end; ++i)
	{
		*pdata = pcl_infos[i].pcl->s12;
		pdata += field_num;
	}
}

void TimeHistory_Particle_R2D_ME_s::output_s23(void)
{
	double *pdata = buffer_pos;
	for (size_t i = pcl_id_range_start; i < pcl_id_range_end; ++i)
	{
		*pdata = pcl_infos[i].pcl->s23;
		pdata += field_num;
	}
}

void TimeHistory_Particle_R2D_ME_s::output_s31(void)
{
	double *pdata = buffer_pos;
	for (size_t i = pcl_id_range_start; i < pcl_id_range_end; ++i)
	{
		*pdata = pcl_infos[i].pcl->s31;
		pdata += field_num;
	}
}

void TimeHistory_Particle_R2D_ME_s::output_e11(void)
{
	double *pdata = buffer_pos;
	for (size_t i = pcl_id_range_start; i < pcl_id_range_end; ++i)
	{
		*pdata = pcl_infos[i].pcl->e11;
		pdata += field_num;
	}
}

void TimeHistory_Particle_R2D_ME_s::output_e22(void)
{
	double *pdata = buffer_pos;
	for (size_t i = pcl_id_range_start; i < pcl_id_range_end; ++i)
	{
		*pdata = pcl_infos[i].pcl->e22;
		pdata += field_num;
	}
}

void TimeHistory_Particle_R2D_ME_s::output_e12(void)
{
	double *pdata = buffer_pos;
	for (size_t i = pcl_id_range_start; i < pcl_id_range_end; ++i)
	{
		*pdata = pcl_infos[i].pcl->e12;
		pdata += field_num;
	}
}

void TimeHistory_Particle_R2D_ME_s::output_es11(void)
{
	double *pdata = buffer_pos;
	for (size_t i = pcl_id_range_start; i < pcl_id_range_end; ++i)
	{
		*pdata = pcl_infos[i].pcl->es11;
		pdata += field_num;
	}
}

void TimeHistory_Particle_R2D_ME_s::output_es22(void)
{
	double *pdata = buffer_pos;
	for (size_t i = pcl_id_range_start; i < pcl_id_range_end; ++i)
	{
		*pdata = pcl_infos[i].pcl->es22;
		pdata += field_num;
	}
}

void TimeHistory_Particle_R2D_ME_s::output_es12(void)
{
	double *pdata = buffer_pos;
	for (size_t i = pcl_id_range_start; i < pcl_id_range_end; ++i)
	{
		*pdata = pcl_infos[i].pcl->es12;
		pdata += field_num;
	}
}

void TimeHistory_Particle_R2D_ME_s::output_ps11(void)
{
	double *pdata = buffer_pos;
	for (size_t i = pcl_id_range_start; i < pcl_id_range_end; ++i)
	{
		*pdata = pcl_infos[i].pcl->ps11;
		pdata += field_num;
	}
}

void TimeHistory_Particle_R2D_ME_s::output_ps22(void)
{
	double *pdata = buffer_pos;
	for (size_t i = pcl_id_range_start; i < pcl_id_range_end; ++i)
	{
		*pdata = pcl_infos[i].pcl->ps22;
		pdata += field_num;
	}
}

void TimeHistory_Particle_R2D_ME_s::output_ps12(void)
{
	double *pdata = buffer_pos;
	for (size_t i = pcl_id_range_start; i < pcl_id_range_end; ++i)
	{
		*pdata = pcl_infos[i].pcl->ps12;
		pdata += field_num;
	}
}
