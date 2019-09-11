#ifndef _PCP_TEST_H_
#define _PCP_TEST_H_

#include <iostream>
#include <string>

// VTK package
#include <vtkSphereSource.h>
#include <vtkPolyData.h>
#include <vtkPolyDataMapper.h>
#include <vtkUnstructuredGrid.h>
#include <vtkGlyph3D.h>

#include <vtkActor.h>
#include <vtkRenderer.h>
#include <vtkRendererCollection.h>
#include <vtkRenderWindow.h>

#include <vtkInteractorStyleTrackballCamera.h>
#include <vtkRenderWindowInteractor.h>

#include <vtkSmartPointer.h>

#include <vtkPolyDataAlgorithm.h>
#include <vtkCommand.h>
#include <vtkCallbackCommand.h>

// output png
#include <vtkWindowToImageFilter.h>
#include <vtkPNGWriter.h>

#endif