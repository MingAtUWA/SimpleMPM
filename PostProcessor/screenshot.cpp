#include "PostProcessor_pcp.h"

#include "screenshot.h"

void screenshot(vtkRenderWindow *render_win, const char *file_name)
{
	// Screenshot
	vtkSmartPointer<vtkWindowToImageFilter> windowToImageFilter
		= vtkSmartPointer<vtkWindowToImageFilter>::New();
	windowToImageFilter->SetInput(render_win);
	// Set the resolution of the output image (3 times the current resolution of renderwindow)
	//windowToImageFilter->SetMagnification(3);
	// Also record the alpha (transparency) channel
	// need to turn on alpha channel frist in renderWindow
	//windowToImageFilter->SetInputBufferTypeToRGBA();
	// Turn off buffer swapping in render window
	// to guarantee image stay in back buffer after rendering
	//int is_swapping = render_win->GetSwapBuffers();
	//render_win->SwapBuffersOff();
	//render_win->Render();
	// read from the back buffer
	//windowToImageFilter->ReadFrontBufferOff();
	windowToImageFilter->Update();
	// resume buffer swapping state
	//render_win->SetSwapBuffers(is_swapping);

	// output as png
	vtkSmartPointer<vtkPNGWriter> writer
		= vtkSmartPointer<vtkPNGWriter>::New();
	writer->SetFileName(file_name);
	writer->SetInputConnection(windowToImageFilter->GetOutputPort());
	writer->Write();
}
