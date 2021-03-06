#include "SimulationCore_pcp.h"

#include "Model_R2D_ME_MPM.h"
#include "Step_R2D_ME_MPM.h"
#include "ResultFile.h"

#include "TimeHistory_Particle_2D_ME_AllPcl.h"

/*====================================================================
TimeHistory_Particle_2D_ME_AllPcl Class
 =====================================================================*/
TimeHistory_Particle_2D_ME_AllPcl::TimeHistory_Particle_2D_ME_AllPcl() :
	TimeHistory(TimeHistoryType::Particle_2D_ME_AllPcl, "Particle_2D_ME_AllPcl"),
	fld_mem(10) {}

TimeHistory_Particle_2D_ME_AllPcl::~TimeHistory_Particle_2D_ME_AllPcl() {}

int TimeHistory_Particle_2D_ME_AllPcl::add_field(Particle_Field_2D_ME fld)
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

int TimeHistory_Particle_2D_ME_AllPcl::init_per_step(void)
{
	Model_R2D_ME_MPM *model = static_cast<Model_R2D_ME_MPM *>(step->get_model());
	Object_Particle_2D_ME *pobj, *pobj_end;
	Particle_2D_ME *ppcl;
	
	size_t total_pcl_num = 0;
	pobj = model->objects.first();
	pobj_end = model->objects.eoi();
	while (pobj != pobj_end)
	{
		total_pcl_num += pobj->pcl_num;
		pobj = model->objects.next(pobj);
	}
	pcl_mem.reserve(total_pcl_num);

	pobj = model->objects.first();
	pobj_end = model->objects.eoi();
	while (pobj != pobj_end)
	{
		for (size_t i = 0; i < pobj->pcl_num; i++)
		{
			ppcl = pobj->pcls + i;
			pcl_mem.add(ppcl);
		}
		pobj = model->objects.next(pobj);
	}
	pcls = pcl_mem.get_mem();

	fld_infos = fld_mem.get_mem();

	return 0;
}

void TimeHistory_Particle_2D_ME_AllPcl::finalize_per_step(void) {}

int TimeHistory_Particle_2D_ME_AllPcl::output(void)
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
const unsigned short int TimeHistory_Particle_2D_ME_AllPcl::field_max_num = FIELD_MAX_NUM;
const TimeHistory_Particle_2D_ME_AllPcl::TimeHistoryFieldFunc
TimeHistory_Particle_2D_ME_AllPcl::output_field_funcs_map[FIELD_MAX_NUM] =
{
	nullptr,            // 0
	&output_x,          // 1
	&output_y,          // 2
	&output_vol,        // 3
	&output_density,    // 4
	&output_m,          // 5
	&output_vx,         // 6
	&output_vy,         // 7
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

void TimeHistory_Particle_2D_ME_AllPcl::output_x(void)
{
	double *pdata = buffer_pos;
	for (size_t i = pcl_id_range_start; i < pcl_id_range_end; ++i)
	{
		*pdata = pcls[i]->x;
		pdata += field_num;
	}
}

void TimeHistory_Particle_2D_ME_AllPcl::output_y(void)
{
	double *pdata = buffer_pos;
	for (size_t i = pcl_id_range_start; i < pcl_id_range_end; ++i)
	{
		*pdata = pcls[i]->y;
		pdata += field_num;
	}
}

void TimeHistory_Particle_2D_ME_AllPcl::output_vol(void)
{
	double *pdata = buffer_pos;
	for (size_t i = pcl_id_range_start; i < pcl_id_range_end; ++i)
	{
		*pdata = pcls[i]->m / pcls[i]->density;
		pdata += field_num;
	}
}

void TimeHistory_Particle_2D_ME_AllPcl::output_density(void)
{
	double *pdata = buffer_pos;
	for (size_t i = pcl_id_range_start; i < pcl_id_range_end; ++i)
	{
		*pdata = pcls[i]->density;
		pdata += field_num;
	}
}

void TimeHistory_Particle_2D_ME_AllPcl::output_m(void)
{
	double *pdata = buffer_pos;
	for (size_t i = pcl_id_range_start; i < pcl_id_range_end; ++i)
	{
		*pdata = pcls[i]->m;
		pdata += field_num;
	}
}

void TimeHistory_Particle_2D_ME_AllPcl::output_vx(void)
{
	double *pdata = buffer_pos;
	for (size_t i = pcl_id_range_start; i < pcl_id_range_end; ++i)
	{
		*pdata = pcls[i]->vx;
		pdata += field_num;
	}
}

void TimeHistory_Particle_2D_ME_AllPcl::output_vy(void)
{
	double *pdata = buffer_pos;
	for (size_t i = pcl_id_range_start; i < pcl_id_range_end; ++i)
	{
		*pdata = pcls[i]->vy;
		pdata += field_num;
	}
}

void TimeHistory_Particle_2D_ME_AllPcl::output_momentum_x(void)
{
	double *pdata = buffer_pos;
	for (size_t i = pcl_id_range_start; i < pcl_id_range_end; ++i)
	{
		*pdata = pcls[i]->vx * pcls[i]->m;
		pdata += field_num;
	}
}

void TimeHistory_Particle_2D_ME_AllPcl::output_momentum_y(void)
{
	double *pdata = buffer_pos;
	for (size_t i = pcl_id_range_start; i < pcl_id_range_end; ++i)
	{
		*pdata = pcls[i]->vy * pcls[i]->m;
		pdata += field_num;
	}
}

void TimeHistory_Particle_2D_ME_AllPcl::output_s11(void)
{
	double *pdata = buffer_pos;
	for (size_t i = pcl_id_range_start; i < pcl_id_range_end; ++i)
	{
		*pdata = pcls[i]->s11;
		pdata += field_num;
	}
}

void TimeHistory_Particle_2D_ME_AllPcl::output_s22(void)
{
	double *pdata = buffer_pos;
	for (size_t i = pcl_id_range_start; i < pcl_id_range_end; ++i)
	{
		*pdata = pcls[i]->s22;
		pdata += field_num;
	}
}

void TimeHistory_Particle_2D_ME_AllPcl::output_s33(void)
{
	double *pdata = buffer_pos;
	for (size_t i = pcl_id_range_start; i < pcl_id_range_end; ++i)
	{
		*pdata = pcls[i]->s33;
		pdata += field_num;
	}
}

void TimeHistory_Particle_2D_ME_AllPcl::output_s12(void)
{
	double *pdata = buffer_pos;
	for (size_t i = pcl_id_range_start; i < pcl_id_range_end; ++i)
	{
		*pdata = pcls[i]->s12;
		pdata += field_num;
	}
}

void TimeHistory_Particle_2D_ME_AllPcl::output_s23(void)
{
	double *pdata = buffer_pos;
	for (size_t i = pcl_id_range_start; i < pcl_id_range_end; ++i)
	{
		*pdata = pcls[i]->s23;
		pdata += field_num;
	}
}

void TimeHistory_Particle_2D_ME_AllPcl::output_s31(void)
{
	double *pdata = buffer_pos;
	for (size_t i = pcl_id_range_start; i < pcl_id_range_end; ++i)
	{
		*pdata = pcls[i]->s31;
		pdata += field_num;
	}
}

void TimeHistory_Particle_2D_ME_AllPcl::output_e11(void)
{
	double *pdata = buffer_pos;
	for (size_t i = pcl_id_range_start; i < pcl_id_range_end; ++i)
	{
		*pdata = pcls[i]->e11;
		pdata += field_num;
	}
}

void TimeHistory_Particle_2D_ME_AllPcl::output_e22(void)
{
	double *pdata = buffer_pos;
	for (size_t i = pcl_id_range_start; i < pcl_id_range_end; ++i)
	{
		*pdata = pcls[i]->e22;
		pdata += field_num;
	}
}

void TimeHistory_Particle_2D_ME_AllPcl::output_e12(void)
{
	double *pdata = buffer_pos;
	for (size_t i = pcl_id_range_start; i < pcl_id_range_end; ++i)
	{
		*pdata = pcls[i]->e12;
		pdata += field_num;
	}
}

void TimeHistory_Particle_2D_ME_AllPcl::output_es11(void)
{
	double *pdata = buffer_pos;
	for (size_t i = pcl_id_range_start; i < pcl_id_range_end; ++i)
	{
		*pdata = pcls[i]->es11;
		pdata += field_num;
	}
}

void TimeHistory_Particle_2D_ME_AllPcl::output_es22(void)
{
	double *pdata = buffer_pos;
	for (size_t i = pcl_id_range_start; i < pcl_id_range_end; ++i)
	{
		*pdata = pcls[i]->es22;
		pdata += field_num;
	}
}

void TimeHistory_Particle_2D_ME_AllPcl::output_es12(void)
{
	double *pdata = buffer_pos;
	for (size_t i = pcl_id_range_start; i < pcl_id_range_end; ++i)
	{
		*pdata = pcls[i]->es12;
		pdata += field_num;
	}
}

void TimeHistory_Particle_2D_ME_AllPcl::output_ps11(void)
{
	double *pdata = buffer_pos;
	for (size_t i = pcl_id_range_start; i < pcl_id_range_end; ++i)
	{
		*pdata = pcls[i]->ps11;
		pdata += field_num;
	}
}

void TimeHistory_Particle_2D_ME_AllPcl::output_ps22(void)
{
	double *pdata = buffer_pos;
	for (size_t i = pcl_id_range_start; i < pcl_id_range_end; ++i)
	{
		*pdata = pcls[i]->ps22;
		pdata += field_num;
	}
}

void TimeHistory_Particle_2D_ME_AllPcl::output_ps12(void)
{
	double *pdata = buffer_pos;
	for (size_t i = pcl_id_range_start; i < pcl_id_range_end; ++i)
	{
		*pdata = pcls[i]->ps12;
		pdata += field_num;
	}
}
