#ifndef _TIMEHISTORY_PARTICLE_2D_ME_ALLPCL_S_H_
#define _TIMEHISTORY_PARTICLE_2D_ME_ALLPCL_S_H_

#include "ItemArray.hpp"
#include "TimeHistory.h"

#include "Model_R2D_ME_MPM_BSpline_s.h"

// Field variables that can be output from nodes
enum class Particle_Field_2D_ME : unsigned short int
{
	x   = 1,
	y   = 2,
	vol = 3,
	density = 4,
	m   = 5,
	vx  = 6,
	vy  = 7,
	mvx = 8,
	mvy = 9,
	// stress
	s11 = 11,
	s22 = 12,
	s12 = 13,
	// strain
	e11 = 16,
	e22 = 17,
	e12 = 18,
};

/* ===========================================================
Class TimeHistory_Particle_2D_ME_AllPcl_s
   =========================================================== */
class TimeHistory_Particle_2D_ME_AllPcl_s : public TimeHistory
{
public:
	TimeHistory_Particle_2D_ME_AllPcl_s();
	~TimeHistory_Particle_2D_ME_AllPcl_s();
	
	// Initialize each steps
	int init_per_step(void);
	// Finalize each steps
	void finalize_per_step(void);
	// Output funtion
	int output(void) override;

protected:
	Particle_R2D_ME_Grid **pcls;
	MemoryUtilities::ItemArray<Particle_R2D_ME_Grid *> pcl_mem;

	typedef void(TimeHistory_Particle_2D_ME_AllPcl_s::* TimeHistoryFieldFunc)(void);
	struct FieldInfo
	{
		Particle_Field_2D_ME fld_id;
		TimeHistoryFieldFunc out_func;
	};
	FieldInfo *fld_infos;
	MemoryUtilities::ItemArray<FieldInfo> fld_mem;

public:
	inline void set_field_num(size_t num) { fld_mem.reserve(num); }
	int add_field(Particle_Field_2D_ME fld);

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
	void output_vx(void);
	void output_vy(void);
	void output_mvx(void);
	void output_mvy(void);
	void output_s11(void);
	void output_s22(void);
	void output_s12(void);
	void output_e11(void);
	void output_e22(void);
	void output_e12(void);
};

#endif