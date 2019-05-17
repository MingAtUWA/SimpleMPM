#include "Test_pcp.h"

#include "Animation_Particle2D.h"
#include "test_post_processor.h"

void test_animation_particle2d(void)
{
	vtkNew<vtkRenderWindow> renderWindow;
	//renderWindow->SetAlphaBitPlanes(1); //enable usage of alpha channel
	vtkNew<vtkInteractorStyleTrackballCamera> interactor_style;

	// must use smartpointer, otherwise timer won't work normally... wtf
	vtkSmartPointer<vtkRenderWindowInteractor> interactor
		= vtkSmartPointer<vtkRenderWindowInteractor>::New();
	interactor->SetRenderWindow(renderWindow);
	interactor->SetInteractorStyle(interactor_style);

	// To use timer
	interactor->Initialize();

	Animation_Particle2D *ani = new Animation_Particle2D(10.0, interactor);
	
	ani->init("res_file\\TimeHistory1-test_out1.txt");
	ani->get_background_mesh("res_file\\ModelState.txt");
	ani->set_animation_name("sim_2d.gif");

	interactor->InvokeEvent(Animation::PlayAnimation);
	interactor->Start();

	delete ani;
}