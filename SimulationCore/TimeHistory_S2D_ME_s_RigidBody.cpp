#include "SimulationCore_pcp.h"

#include "Model_S2D_ME_s_RigidBody.h"
#include "Step_S2D_ME_s_RigidBody.h"
#include "ResultFile.h"

#include <fstream>

#include "TimeHistory_S2D_ME_s_RigidBody.h"

/*====================================================================
TimeHistory_S2D_ME_s_RigidBody Class
=====================================================================*/
TimeHistory_S2D_ME_s_RigidBody::TimeHistory_S2D_ME_s_RigidBody(std::fstream &file) :
	TimeHistory(TimeHistoryType::S2D_ME_RigidBody, "S2D_ME_RigidBody"), res_file(file) {}

TimeHistory_S2D_ME_s_RigidBody::~TimeHistory_S2D_ME_s_RigidBody() {}

int TimeHistory_S2D_ME_s_RigidBody::init_per_step(void) { return 0; }

void TimeHistory_S2D_ME_s_RigidBody::finalize_per_step(void) {}

int TimeHistory_S2D_ME_s_RigidBody::output(void)
{
	TimeHistoryHeader thh;
	thh.index = time_rcd_id;
	thh.step_index = step->get_index();
	thh.substep_num = step->get_substep_num();
	thh.total_substep_num = step->get_total_substep_num();
	thh.current_time = step->get_current_time();
	thh.total_time = step->get_total_time();
	thh.point_num = point_num;
	res_file.write(reinterpret_cast<char *>(&thh), sizeof(thh));
	
	RigidBodyMotionHeader rbmh;
	Model_S2D_ME_s_RigidBody &md = 
		*static_cast<Model_S2D_ME_s_RigidBody *>(step->get_model());
	rbmh.x = md.rigid_body.x;
	rbmh.y = md.rigid_body.y;
	rbmh.vx = md.rigid_body.vx;
	rbmh.vy = md.rigid_body.vy;
	rbmh.v_theta = md.rigid_body.v_theta;
	res_file.write(reinterpret_cast<char *>(&rbmh), sizeof(rbmh));

	double *data = new double[point_num];
	size_t data_len = sizeof(double) * point_num;
	// x
	for (size_t pcl_id = 0; pcl_id < point_num; ++pcl_id)
		data[pcl_id] = md.pcls[pcl_id].x;
	res_file.write(reinterpret_cast<char *>(data), data_len);
	// y
	for (size_t pcl_id = 0; pcl_id < point_num; ++pcl_id)
		data[pcl_id] = md.pcls[pcl_id].y;
	res_file.write(reinterpret_cast<char *>(data), data_len);
	// vol
	for (size_t pcl_id = 0; pcl_id < point_num; ++pcl_id)
		data[pcl_id] = md.pcls[pcl_id].m / md.pcls[pcl_id].density;
	res_file.write(reinterpret_cast<char *>(data), data_len);
	delete[] data;

	++time_rcd_id;
	return 0;
}
