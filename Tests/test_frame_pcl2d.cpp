#include "Test_pcp.h"

#include "Frame_Particle2D.h"
#include "screenshot.h"
#include "test_post_processor.h"

// swap the backbuffer to the front
void swap_buffer(vtkRenderWindow *render_win)
{
	if (!render_win->GetSwapBuffers())
	{
		render_win->SwapBuffersOn();
		// Frame() only work with Swap Buffer being Set
		render_win->Frame();
		render_win->SwapBuffersOff();
	}
}

void test_frame_pcl2d(void)
{
	vtkSmartPointer<vtkRenderWindow> renderWindow = vtkRenderWindow::New();
	//renderWindow->SetAlphaBitPlanes(1); //enable usage of alpha channel
	vtkNew<vtkInteractorStyleTrackballCamera> interactor_style;
	vtkSmartPointer<vtkRenderWindowInteractor> renderWindowInteractor
		= vtkRenderWindowInteractor::New();
	renderWindowInteractor->SetRenderWindow(renderWindow);
	renderWindowInteractor->SetInteractorStyle(interactor_style);

	double x_coord[] = { 0.0, 1.0, 2.0, 3.0, 4.0 };
	double y_coord[] = { 0.0, 1.0, 2.0, 3.0, 4.0 };
	double vol[] = { 0.1, 0.2, 0.4, 0.7, 1.0 };
	double fld[] = { 0.5, 3.0, 1.5, 3.0, 2.0 };

	double mesh_x[] = { 0.0, 1.0, 2.0, 3.0, 4.0 };
	double mesh_y[] = { 1.0, 2.0, 3.0 };

	Frame_Particle2D frame;
	//frame.add_particles(sizeof(x_coord) / sizeof(double), x_coord, y_coord, vol);
	frame.add_particles(sizeof(x_coord) / sizeof(double), x_coord, y_coord, vol, fld);
	frame.add_mesh(sizeof(mesh_x) / sizeof(double), mesh_x, sizeof(mesh_y) / sizeof(double), mesh_y);
	renderWindow->AddRenderer(frame.create_scene());
	
	renderWindow->Render();

	screenshot(renderWindow, "test_shot.png");

	renderWindowInteractor->Start();
}
