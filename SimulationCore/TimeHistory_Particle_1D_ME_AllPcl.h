#ifndef _TIMEHISTORY_PARTICLE_1D_ME_ALLPCL_H_
#define _TIMEHISTORY_PARTICLE_1D_ME_ALLPCL_H_

#include "ItemArray.hpp"
#include "TimeHistory.h"

#include "Model_1D_ME_MPM_BSpline_s.h"

// Field variables that can be output from nodes
enum class Particle_Field_1D_ME : unsigned short int
{
	x   = 1,
	density = 2,
	m   = 3,
	mv  = 4,
	v   = 5,
	s11 = 6,
	e11 = 7,
};

/* ===========================================================
Class TimeHistory_Particle_1D_ME_AllPcl
   =========================================================== */
class TimeHistory_Particle_1D_ME_AllPcl : public TimeHistory
{
public:
	TimeHistory_Particle_1D_ME_AllPcl();
	~TimeHistory_Particle_1D_ME_AllPcl();
	
	// Initialize each steps
	int init_per_step(void);
	// Finalize each steps
	void finalize_per_step(void);
	// Output funtion
	int output(void) override;

protected:
	Particle_1D_ME **pcls;
	MemoryUtilities::ItemArray<Particle_1D_ME *> pcl_mem;

	typedef void(TimeHistory_Particle_1D_ME_AllPcl::* TimeHistoryFieldFunc)(void);
	struct FieldInfo
	{
		Particle_Field_1D_ME fld_id;
		TimeHistoryFieldFunc out_func;
	};
	FieldInfo *fld_infos;
	MemoryUtilities::ItemArray<FieldInfo> fld_mem;

public:
	inline void set_field_num(size_t num) { fld_mem.reserve(num); }
	int add_field(Particle_Field_1D_ME fld);

protected:
	const static unsigned short int field_max_num;
	const static TimeHistoryFieldFunc output_field_funcs_map[];
	// used by output functions
	size_t pcl_id_range_start, pcl_id_range_end;
	double *buffer_pos;
	// output functions
	void output_x(void);
	void output_density(void);
	void output_m(void);
	void output_mv(void);
	void output_v(void);
	void output_s11(void);
	void output_e11(void);
};

#endif