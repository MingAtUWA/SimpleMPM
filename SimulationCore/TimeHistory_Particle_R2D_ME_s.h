#ifndef _TIMEHISTORY_PARTICLE_R2D_ME_S_H_
#define _TIMEHISTORY_PARTICLE_R2D_ME_S_H_

#include "ItemArray.hpp"
#include "TimeHistory.h"

#include "Particle_R2D_ME_s.h"

// Field variables that can be output from nodes
enum class Particle_Field_R2D_ME_s : unsigned short int
{
	x   = 1,
	y   = 2,
	vol = 3,
	density = 4,
	m   = 5,
	v_x = 6,
	v_y = 7,
	momentum_x = 8,
	momentum_y = 9,

	s11 = 11,
	s22 = 12,
	s33 = 13,
	s12 = 14,
	s23 = 15,
	s31 = 16,

	e11 = 20,
	e22 = 21,
	e12 = 22,
	es11 = 23,
	es22 = 24,
	es12 = 25,
	ps11 = 26,
	ps22 = 27,
	ps12 = 28
};

struct Model_R2D_ME_MPM_s;

/* ===========================================================
Class TimeHistory_Particle_R2D_ME_s
   =========================================================== */
class TimeHistory_Particle_R2D_ME_s : public TimeHistory
{
public:
	TimeHistory_Particle_R2D_ME_s();
	~TimeHistory_Particle_R2D_ME_s();
	
	int set_model_output(Model_R2D_ME_MPM_s *md,
		Particle_Field_R2D_ME_s *fld_ids, size_t fld_num,
		size_t *pcl_ids, size_t pcl_num);

	// Initialize each steps
	int init_per_step(void);
	// Finalize each steps
	void finalize_per_step(void);
	// Output funtion
	int output(void) override;

protected:
	typedef void(TimeHistory_Particle_R2D_ME_s::* TimeHistoryFieldFunc)(void);
	struct ParticleInfo
	{
		size_t obj_id;
		Particle_R2D_ME_s *pcl;
	};
	struct FieldInfo
	{
		Particle_Field_R2D_ME_s fld_id;
		TimeHistoryFieldFunc out_func;
	};
	ParticleInfo *pcl_infos;
	MemoryUtilities::ItemArray<ParticleInfo> pcl_mem;
	FieldInfo *fld_infos;
	MemoryUtilities::ItemArray<FieldInfo> fld_mem;

	inline void set_particle_num(size_t num) { pcl_mem.reserve(num); }
	int add_particle(size_t obj_id, Particle_R2D_ME_s *pcl);
	inline void set_field_num(size_t num) { fld_mem.reserve(num); }
	int add_field(Particle_Field_R2D_ME_s fld);

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
	void output_density(void);
	void output_m(void);
	void output_v_x(void);
	void output_v_y(void);
	void output_momentum_x(void);
	void output_momentum_y(void);
	void output_s11(void);
	void output_s22(void);
	void output_s33(void);
	void output_s12(void);
	void output_s23(void);
	void output_s31(void);
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