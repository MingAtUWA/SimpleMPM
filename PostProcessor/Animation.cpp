#include "PostProcessor_pcp.h"

#include "gif.h"

#include "Animation.h"

void Animation::PlayAnimationEvent::
	Execute(vtkObject *caller, unsigned long eventId, void *callData)
{
	animation->play_animation();
}

void Animation::RenderFrameTimerEvent::
	Execute(vtkObject * caller, unsigned long eventId, void * callData)
{
	// display previous frame (swap previous frame out of buffer)
	animation->display_frame();
	// render next frame
	animation->render_frame();
}

Animation::Animation(double animation_time, vtkRenderWindowInteractor *win_iter) :
	is_playing(false),
	cur_sim_time(0.0), cur_ani_time(0.0),
	total_sim_time(0.0), total_ani_time(animation_time),
	ani_sim_ratio(0.0), frame_num(0),
	window(win_iter->GetRenderWindow()), interactor(win_iter), frame(nullptr),
	need_export_animation(false),
	timer_id(0),
	gif_writer(nullptr)
{
	play_animation_event->animation = this;
	render_frame_timer_event->animation = this;

	interactor->AddObserver(Animation::PlayAnimation, 
		static_cast<vtkCommand *>(play_animation_event));
	interactor->AddObserver(vtkCommand::TimerEvent,
		static_cast<vtkCommand *>(render_frame_timer_event));
}

Animation::~Animation()
{
	complete_animation();
	interactor->RemoveObserver(static_cast<vtkCommand *>(play_animation_event));
	interactor->RemoveObserver(static_cast<vtkCommand *>(render_frame_timer_event));
}

/*-------------------------------------------------------------------------------
 * The coordinate system of gif is from left to right, from top to bottom, while
 * the coordiante system of VTK window is from left to right, from bottom tp top.
 * Therefore the buffer need to be reordered when transferred between different
 * source.
--------------------------------------------------------------------------------*/
void reorder_buffer(unsigned char *RGBA_data, int width, int height)
{
	// RGBA data are 4 bytes long
	long *data = reinterpret_cast<long *>(RGBA_data);
	long *line1 = data;
	long *line2 = data + (height - 1) * width;
	long data_tmp;
	while (line1 < line2)
	{
		for (size_t i = 0; i < width; i++)
		{
			data_tmp = line1[i];
			line1[i] = line2[i];
			line2[i] = data_tmp;
		}
		line1 += width;
		line2 -= width;
	}
}

int Animation::render_frame(void)
{
	remove_frame_from_window();

	if (!is_playing)
	{
		complete_animation();
		return 0;
	}

	// finish animation
	if (next_frame_index >= frame_num)
	{
		complete_animation();
		return 0;
	}
	
	start_rendering_time = std::chrono::system_clock::now();;
	cur_frame_index = next_frame_index;
	cur_ani_time = next_ani_time;
	cur_sim_time = next_sim_time;

	// create scenario to be rendered
	frame = render_frame(cur_frame_index);
	if (frame)
	{
		window->AddRenderer(frame);
		frame->ResetCamera();
	}

	// Render without displaying
	window->SwapBuffersOff();
	window->Render();
	window->SwapBuffersOn();

	//if (frame_index == 0)
	//	window->Frame();

	complete_rendering_time = std::chrono::system_clock::now();
	std::chrono::duration<double> rendering_time = complete_rendering_time - start_rendering_time;
	double diff_time = inc_ani_time - rendering_time.count();
	if (timer_id) interactor->DestroyTimer(timer_id);
	if (diff_time <= 0.0)
	{
		timer_id = interactor->CreateOneShotTimer(1);
	}
	else
	{
		timer_id = interactor->CreateOneShotTimer(diff_time * 1000); // from second to millisecond
	}

	// find the next frame needed to be exported
	next_frame_index = cur_frame_index;
	next_sim_time = cur_sim_time;
	inc_ani_time = 0.0;
	do
	{
		++next_frame_index;
		if (next_frame_index >= frame_num)
			break;
		next_sim_time = get_frame_time(next_frame_index);
		inc_ani_time = (next_sim_time - cur_sim_time) * ani_sim_ratio;

	}
	while (inc_ani_time < 1.0 / 100.0); // maximum rate of gif
	next_ani_time = cur_ani_time + inc_ani_time;

	if (need_export_animation)
	{
		// get from back buffer
		unsigned char *win_pixel_buffer = window->GetRGBACharPixelData(0, 0, win_width - 1, win_height - 1, 0);
		reorder_buffer(win_pixel_buffer, win_width, win_height);
		int delay = int(inc_ani_time * 100.0); // to hundredths second
		GifWriteFrame(gif_writer, win_pixel_buffer, win_width, win_height, (delay ? delay : 1));
		free(win_pixel_buffer);
	}

	// seems that there two back buffer in OpenGL
	// so skip the first unused one
	if (cur_frame_index == 0)
		window->Frame();

	return 0;
}


int Animation::play_animation(void)
{
	// init
	if (need_export_animation)
	{
		int *win_size = window->GetSize();
		win_width = win_size[0];
		win_height = win_size[1];
		gif_writer = new GifWriter;
		GifBegin(gif_writer, animation_name.c_str(), win_width, win_height, 1);
	}

	if (total_sim_time == 0.0) return -2;
	ani_sim_ratio = total_ani_time / total_sim_time;
	is_playing = true;
	cur_frame_index = 0;
	cur_ani_time = 0.0;
	cur_sim_time = get_frame_time(cur_frame_index);
	next_frame_index = 0;
	next_ani_time = 0.0;
	next_sim_time = cur_sim_time;
	inc_ani_time = 0.0;

	// start animation
	start_time = std::chrono::system_clock::now();
	render_frame();

	return 0;
}

void Animation::complete_animation(void)
{
	remove_frame_from_window();
	if (timer_id)
		interactor->DestroyTimer(timer_id);
	if (need_export_animation && gif_writer)
	{
		GifEnd(gif_writer);
		delete gif_writer;
		gif_writer = nullptr;
	}
}