#ifndef _ANIMATION_TEST_H_
#define _ANIMATION_TEST_H_

#include "Frame_Particle2D.h"

#include "Animation.h"

class AnimationTest : public Animation
{
public:
	AnimationTest(vtkRenderWindowInteractor *win_iter) :
		Animation(10.0, win_iter)
	{
		frame_num = 100;
		total_sim_time = 1.0;
		sim_time_per_frame = total_sim_time / frame_num;
	}
	~AnimationTest() {}

	double get_frame_time(size_t frame_index) override;
	vtkRenderer *render_frame(size_t frame_index) override;

	Frame_Particle2D frame;
	double sim_time_per_frame;
};

#endif