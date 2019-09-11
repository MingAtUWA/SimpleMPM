#ifndef __TIMEHISTORY_S2D_ME_S_RIGIDBODY_H__
#define __TIMEHISTORY_S2D_ME_S_RIGIDBODY_H__

#include "ItemArray.hpp"
#include "Particle_Field.h"
#include "TimeHistory.h"

#include "Model_S2D_ME_s_RigidBody.h"

/* ===========================================================
Class TimeHistory_S2D_ME_s_RigidBody
=========================================================== */
class TimeHistory_S2D_ME_s_RigidBody : public TimeHistory
{
protected:
	std::fstream &res_file;
	struct TimeHistoryHeader
	{
		unsigned long long index;
		unsigned long long step_index;
		unsigned long long substep_num;
		unsigned long long total_substep_num;
		double current_time;
		double total_time;
		unsigned long long point_num;
	};
	struct RigidBodyMotionHeader
	{
		double x, y, theta;
		double vx, vy, v_theta;
	};

public:
	TimeHistory_S2D_ME_s_RigidBody(std::fstream &file);
	~TimeHistory_S2D_ME_s_RigidBody();

	// Initialize each steps
	int init_per_step(void);
	// Finalize each steps
	void finalize_per_step(void);
	// Output funtion
	int output(void) override;
};

#endif