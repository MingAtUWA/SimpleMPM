#include "PostProcessor_pcp.h"

#include "Frame_Particle2D.h"

#define PI 3.14159265358979323846264338327950288419716939937510

int Frame_Particle2D::add_particles(size_t num, 
	double *x_coords, double *y_coords, double *vol, double *fld)
{
	if (!num) return -2;

	pcls = nullptr;
	pcls = vtkSmartPointer<vtkPoints>::New();
	pcls_size = nullptr;
	pcls_size = vtkSmartPointer<vtkDoubleArray>::New();
	if (fld)
	{
		pcls_fld = nullptr;
		pcls_fld = vtkSmartPointer<vtkDoubleArray>::New();
	}
	pcl_set = nullptr;
	pcl_set = vtkSmartPointer<vtkPolyData>::New();
	//ball = nullptr;
	//ball = vtkSmartPointer<vtkSphereSource>::New();
	ball_glyph = nullptr;
	ball_glyph = vtkSmartPointer<vtkGlyph3D>::New();
	pcl_mapper = nullptr;
	pcl_mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
	pcl_actor = nullptr;
	pcl_actor = vtkSmartPointer<vtkActor>::New();

	pcls_size->SetName("particle_size");
	double fld_min = 0.0;
	double fld_max = 0.0;
	if (fld)
	{
		pcls_fld->SetName("particle_field");
		fld_min = fld[0];
		fld_max = fld[0];
		for (size_t i = 0; i < num; i++)
		{
			pcls->InsertNextPoint(x_coords[i], y_coords[i], 0.0);
			// radius
			pcls_size->InsertNextValue(sqrt(vol[i] / PI));
			pcls_fld->InsertNextValue(fld[i]);
			if (fld_max < fld[i])
				fld_max = fld[i];
			if (fld_min > fld[i])
				fld_min = fld[i];
		}
	}
	else
	{
		for (size_t i = 0; i < num; i++)
		{
			pcls->InsertNextPoint(x_coords[i], y_coords[i], 0.0);
			// radius
			pcls_size->InsertNextValue(sqrt(vol[i] / PI));
		}
	}

	pcl_set->SetPoints(pcls);
	pcl_set->GetPointData()->AddArray(pcls_size);
	pcl_set->GetPointData()->SetActiveScalars("particle_size");
	if (fld)
		pcl_set->GetPointData()->AddArray(pcls_fld);

	ball_glyph->SetInputData(pcl_set);
	ball_glyph->SetSourceConnection(ball->GetOutputPort());
	ball_glyph->SetScaleModeToScaleByScalar();

	pcl_mapper->SetInputConnection(ball_glyph->GetOutputPort());
	// color map
	if (fld)
	{
		color_table = nullptr;
		color_table = vtkSmartPointer<vtkLookupTable>::New();
		color_table->SetTableRange(fld_min, fld_max);
		color_table->SetHueRange(0.5, 1.0);
		color_table->SetSaturationRange(1.0, 1.0);
		color_table->SetValueRange(1.0, 1.0);
		color_table->Build();
		pcl_mapper->ScalarVisibilityOn();
		pcl_mapper->SetScalarModeToUsePointFieldData();
		pcl_mapper->SelectColorArray("particle_field");
		pcl_mapper->SetScalarRange(fld_min, fld_max);
		pcl_mapper->SetLookupTable(color_table);
	}
	else
	{
		pcl_mapper->ScalarVisibilityOff();
	}

	pcl_actor->SetMapper(pcl_mapper);

	return 0;
}

// the coordinates must be sorted
int Frame_Particle2D::add_mesh(size_t x_num,
	double *x_coords, size_t y_num, double *y_coords)
{
	if (!x_num || !y_num) return -2;

	mesh_points = nullptr;
	mesh_points = vtkSmartPointer<vtkPoints>::New();
	mesh_lines = nullptr;
	mesh_lines = vtkSmartPointer<vtkCellArray>::New();
	mesh = nullptr;
	mesh = vtkSmartPointer<vtkPolyData>::New();
	mesh_mapper = nullptr;
	mesh_mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
	mesh_actor = nullptr;
	mesh_actor = vtkSmartPointer<vtkActor>::New();

	double x_min, x_max, y_min, y_max;
	x_min = x_coords[0];
	x_max = x_coords[x_num - 1];
	y_min = y_coords[0];
	y_max = y_coords[y_num - 1];
	// Vertical lines
	for (size_t i = 0; i < x_num; i++)
	{
		mesh_points->InsertNextPoint(x_coords[i], y_min, 0.0);
		mesh_points->InsertNextPoint(x_coords[i], y_max, 0.0);
		mesh_lines->InsertNextCell(2);
		mesh_lines->InsertCellPoint(2 * i);
		mesh_lines->InsertCellPoint(2 * i + 1);
	}
	// Horizontal lines
	for (size_t i = 0; i < y_num; i++)
	{
		mesh_points->InsertNextPoint(x_min, y_coords[i], 0.0);
		mesh_points->InsertNextPoint(x_max, y_coords[i], 0.0);
		mesh_lines->InsertNextCell(2);
		mesh_lines->InsertCellPoint(x_num * 2 + 2 * i);
		mesh_lines->InsertCellPoint(x_num * 2 + 2 * i + 1);
	}

	mesh->SetPoints(mesh_points);
	mesh->SetLines(mesh_lines);

	mesh_mapper->SetInputData(mesh);

	mesh_actor->SetMapper(mesh_mapper);

	return 0;
}

vtkRenderer *Frame_Particle2D::create_scene(void)
{
	renderer = nullptr;
	renderer = vtkSmartPointer<vtkRenderer>::New();
	
	renderer->AddActor(pcl_actor);
	if (mesh_actor.GetPointer())
		renderer->AddActor(mesh_actor);

	renderer->SetBackground(0.0, 0.0, 0.0); // Background color white
	renderer->ResetCamera();

	return renderer;
}
