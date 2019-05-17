#ifndef _ANIMATION_H_
#define _ANIMATION_H_

#include <chrono>

struct GifWriter;

/*----------------------------------------------------------------
Class Animation
	This class plays animation on an vtkRenderWindowInteractor.
Functions below must be rewritten:
	1. double get_frame_time(size_t frame_index)
	2. vtkRenderer *render_frame(size_t frame_index)
Functions below may need to be rewritten:
	1. int export_current_frame(void)
	2. int create_animation(void)
----------------------------------------------------------------*/
class Animation
{
public:
	enum EventIds
	{
		PlayAnimation = vtkCommand::UserEvent + 1,
	};

	/* 
	 * Animation()
	 * Parameter:
	 * 1. att - time length of animation.
	 * 2. dtt - time length of the simulation result data
	 * 3. tfn - number of simulation result data
	*/
	Animation(double animation_time, vtkRenderWindowInteractor *win_iter);
	~Animation();

	void set_animation_name(const char *ani_name = nullptr)
	{
		if (ani_name)
		{
			animation_name = ani_name;
			need_export_animation = true;
		}
		else
		{
			need_export_animation = false;
		}
	}

	int play_animation(void);
	void complete_animation(void);

protected:
	// functions needed to be rewritten
	virtual double get_frame_time(size_t frame_index) = 0;
	virtual vtkRenderer *render_frame(size_t frame_index) = 0;

	// render frame into back buffer
	int render_frame(void);
	// swap back buffer into front (to display)
	void display_frame(void) 
	{
		std::chrono::duration<double> elapsed_time = std::chrono::system_clock::now() - start_time;
		// Debug output
		std::cout << " frame_index: " << cur_frame_index
				  << " ani_time: " << cur_ani_time
				  << " real_time: " << elapsed_time.count() << std::endl;

		window->Frame();
	}

protected:
	class PlayAnimationEvent : public vtkCommand
	{
		friend Animation;
	protected:
		Animation *animation;
	public:
		static PlayAnimationEvent *New() { return new PlayAnimationEvent; }
		void Execute(vtkObject *caller, unsigned long eventId, void *callData) override;
	};

	class RenderFrameTimerEvent : public vtkCommand
	{
		friend Animation;
	protected:
		Animation *animation;
	public:
		static RenderFrameTimerEvent *New() { return new RenderFrameTimerEvent; }
		void Execute(vtkObject *caller, unsigned long eventId, void *callData) override;
	};

protected:
	bool is_playing;

	double total_sim_time;
	double total_ani_time;
	// = total_ani_time / total_data_time
	double ani_sim_ratio;
	size_t frame_num;

	double cur_sim_time;
	double cur_ani_time;
	size_t cur_frame_index;
	double next_sim_time;
	double next_ani_time;
	size_t next_frame_index;

	vtkRenderWindow *window;
	vtkRenderWindowInteractor *interactor;
	vtkRenderer *frame;
	inline void remove_frame_from_window(void)
	{
		if (frame)
		{
			window->RemoveRenderer(frame);
			frame = nullptr;
		}
	}
	vtkNew<PlayAnimationEvent> play_animation_event;
	vtkNew<RenderFrameTimerEvent> render_frame_timer_event;

	bool need_export_animation;
	std::string animation_name;

	// temporary variables
	int timer_id;
	std::chrono::system_clock::time_point start_rendering_time;
	std::chrono::system_clock::time_point complete_rendering_time;
	double inc_ani_time;
	// for gif generation
	GifWriter *gif_writer;
	int win_width, win_height;
	// for debug purpose
	std::chrono::system_clock::time_point start_time;
};

#endif