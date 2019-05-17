#ifndef _FRAME_PARTICLE2D_H_
#define _FRAME_PARTICLE2D_H_

class Frame_Particle2D
{
protected:
	vtkSmartPointer<vtkPoints> pcls;
	vtkSmartPointer<vtkDoubleArray> pcls_size;
	vtkSmartPointer<vtkDoubleArray> pcls_fld;
	vtkSmartPointer<vtkLookupTable> color_table;
	vtkSmartPointer<vtkPolyData> pcl_set;
	vtkSmartPointer<vtkSphereSource> ball;
	vtkSmartPointer<vtkGlyph3D> ball_glyph;
	vtkSmartPointer<vtkPolyDataMapper> pcl_mapper;
	vtkSmartPointer<vtkActor> pcl_actor;

	vtkSmartPointer<vtkPoints> mesh_points;
	vtkSmartPointer<vtkCellArray> mesh_lines;
	vtkSmartPointer<vtkPolyData> mesh;
	vtkSmartPointer<vtkPolyDataMapper> mesh_mapper;
	vtkSmartPointer<vtkActor> mesh_actor;

	vtkSmartPointer<vtkRenderer> renderer;

public:
	Frame_Particle2D() : ball(vtkSmartPointer<vtkSphereSource>::New())
	{
		ball->SetRadius(1.0);
		ball->SetThetaResolution(15);
		ball->SetPhiResolution(15);
	}
	~Frame_Particle2D() {}
	
	int add_particles(size_t num, double *x_coords, double *y_coords,
					  double *vol, double *fld = nullptr);

	// the coordinates must be sorted
	int add_mesh(size_t x_num, double *x_coords, size_t y_num, double *y_coords);

	vtkRenderer *create_scene(void);
};

#endif