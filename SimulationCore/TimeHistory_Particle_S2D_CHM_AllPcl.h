#ifndef _TIMEHISTORY_PARTICLE_S2D_CHM_ALLPCL_H_
#define _TIMEHISTORY_PARTICLE_S2D_CHM_ALLPCL_H_

#include "ItemArray.hpp"
#include "Particle_Field.h"
#include "TimeHistory.h"

#include "Model_S2D_CHM_MPM_s.h"

/* ===========================================================
Class TimeHistory_Particle_S2D_CHM_AllPcl
   =========================================================== */
class TimeHistory_Particle_S2D_CHM_AllPcl : public TimeHistory
{
public:
	TimeHistory_Particle_S2D_CHM_AllPcl();
	~TimeHistory_Particle_S2D_CHM_AllPcl();
	
	// Initialize each steps
	int init_per_step(void);
	// Finalize each steps
	void finalize_per_step(void);
	// Output funtion
	int output(void) override;

protected:
	Particle_S2D_CHM **pcls;
	MemoryUtilities::ItemArray<Particle_S2D_CHM *> pcl_mem;

	typedef void(TimeHistory_Particle_S2D_CHM_AllPcl::* TimeHistoryFieldFunc)(void);
	struct FieldInfo
	{
		Particle_Field_2D_CHM fld_id;
		TimeHistoryFieldFunc out_func;
	};
	FieldInfo *fld_infos;
	MemoryUtilities::ItemArray<FieldInfo> fld_mem;

public:
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
	void output_m_s(void);
	void output_m_f(void);
	void output_vx_s(void);
	void output_vy_s(void);
	void output_vx_f(void);
	void output_vy_f(void);
	void output_mvx_s(void);
	void output_mvy_s(void);
	void output_mvx_f(void);
	void output_mvy_f(void);
	void output_s11(void);
	void output_s22(void);
	void output_s12(void);
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