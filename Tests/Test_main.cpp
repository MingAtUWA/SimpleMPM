#include "Test_pcp.h"

#include <cstdlib>

#include "test_post_processor.h"
#include "test_sim_core.h"

int main(int argc, char *argv[])
{
	//test_me_mpm1();
	//test_me_mpm2();
	//test_me_mpm3();

	//test_me_kindamp_mpm1();
	//test_me_kindamp_mpm2();

	//test_chm_mpm1();
	//test_chm_mpm2();

	//test_chm_damp_mpm1();
	//test_chm_damp_mpm2();
	test_chm_damp_mpm3();

	//test_file_char_array();
	//test_item_array();
	//test_frame_pcl2d();
	//test_result_file_parser();
	//display_model();

	//test_animation();
	//test_animation_particle2d();

	system("pause");
	return 0;
}