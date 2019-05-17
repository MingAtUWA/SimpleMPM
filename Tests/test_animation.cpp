#include "Test_pcp.h"

#include "Animation_Test.h"
#include "test_post_processor.h"

void test_animation(void)
{
	vtkNew<vtkRenderWindow> renderWindow;
	//renderWindow->SetAlphaBitPlanes(1); //enable usage of alpha channel
	vtkNew<vtkInteractorStyleTrackballCamera> interactor_style;

	vtkSmartPointer<vtkRenderWindowInteractor> interactor 
		= vtkSmartPointer<vtkRenderWindowInteractor>::New();
	interactor->SetRenderWindow(renderWindow);
	interactor->SetInteractorStyle(interactor_style);

	// To use timer
	interactor->Initialize();

	AnimationTest *test_ani = new AnimationTest(interactor);
	test_ani->set_animation_name("test_ani.gif");

	interactor->InvokeEvent(Animation::PlayAnimation);
	interactor->Start();

	delete test_ani;
}
