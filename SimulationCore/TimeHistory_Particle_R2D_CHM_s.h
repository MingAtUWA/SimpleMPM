#ifndef _TIMEHISTORY_PARTICLE_R2D_CHM_S_H_
#define _TIMEHISTORY_PARTICLE_R2D_CHM_S_H_

#include "ItemArray.hpp"
#include "Particle_Field.h"
#include "TimeHistory.h"

#include "Particle_R2D_CHM_s.h"

struct Model_R2D_CHM_MPM_s;

/*===========================================================
Class TimeHistory_Particle_R2D_CHM_s
  ===========================================================*/
class TimeHistory_Particle_R2D_CHM_s : public TimeHistory
{
public:
	TimeHistory_Particle_R2D_CHM_s();
	~TimeHistory_Particle_R2D_CHM_s();

	int set_model_output(Model_R2D_CHM_MPM_s *md,
		Particle_Field_2D_CHM *fld_ids, size_t fld_num,
		size_t *pcl_ids, size_t pcl_num);

	// Initialize each steps
	int init_per_step(void);
	// Finalize each steps
	void finalize_per_step(void);
	// Output funtion
	int output(void) override;

protected:
	typedef void(TimeHistory_Particle_R2D_CHM_s::* TimeHistoryFieldFunc)(void);
	struct ParticleInfo
	{
		size_t obj_id;
		Particle_R2D_CHM_s *pcl;
	};
	struct FieldInfo
	{
		Particle_Field_2D_CHM fld_id;
		TimeHistoryFieldFunc out_func;
	};
	ParticleInfo *pcl_infos;
	MemoryUtilities::ItemArray<ParticleInfo> pcl_mem;
	FieldInfo *fld_infos;
	MemoryUtilities::ItemArray<FieldInfo> fld_mem;

	inline void set_particle_num(size_t num) { pcl_mem.reserve(num); }
	int add_particle(size_t obj_id, Particle_R2D_CHM_s *pcl);
	inline void set_field_num(size_t num) { fld_mem.reserve(num); }
	int add_field(Particle_Field_2D_CHM fld);

protected:
	const static unsigned short int field_max_num;
	const static TimeHistoryFieldFunc output_field_funcs_map[];
	// used by output functions
	size_t pcl_id_range_start, pcl_id_range_end;
	double *buffer_pos;
	// output functions
	void output_x(void);
	void output_y(void);
	void output_vol(void);
	void output_n(void);
	void output_density_s(void);
	void output_density_f(void);
	void output_vx_s(void);
	void output_vy_s(void);
	void output_vx_f(void);
	void output_vy_f(void);
	void output_s11(void);
	void output_s22(void);
	void output_s33(void);
	void output_s12(void);
	void output_s23(void);
	void output_s31(void);
	void output_p(void);
	void output_e11(void);
	void output_e22(void);
	void output_e12(void);
	void output_es11(void);
	void output_es22(void);
	void output_es12(void);
	void output_ps11(void);
	void output_ps22(void);
	void output_ps12(void);
};

#endif