#include "PostProcessor_pcp.h"

#include "Animation_Test.h"

double AnimationTest::get_frame_time(size_t cur_frame_index)
{
	// 0.1, 0.2, 0.3 ...
	return (double)cur_frame_index * sim_time_per_frame;
}

vtkRenderer *AnimationTest::render_frame(size_t frame_index)
{
	double x_coord[5];
	double y_coord[5];
	double vol[5];
	double fld[5];

	if (frame_index == 0)
	{
		for (size_t i = 0; i < 5; i++)
		{
			x_coord[i] = (double)i;
			y_coord[i] = (double)i;
			vol[i] = 1.0;
			fld[i] = double(i);
		}
		double mesh_x[] = { 0.0, 1.0, 2.0, 3.0, 4.0 };
		double mesh_y[] = { 1.0, 2.0, 3.0 };
		frame.add_particles(sizeof(x_coord) / sizeof(double), x_coord, y_coord, vol, fld);
		frame.add_mesh(sizeof(mesh_x) / sizeof(double), mesh_x, sizeof(mesh_y) / sizeof(double), mesh_y);
	}
	else
	{
		for (size_t i = 0; i < 5; i++)
		{
			x_coord[i] = (double)i * (1.0 + (double)frame_index * 0.05);
			y_coord[i] = (double)i * (1.0 - (double)frame_index * 0.05);
			vol[i] = 1.0 + (double)frame_index * 0.05;
			fld[i] = double((i + frame_index) % 5);
		}
		frame.add_particles(sizeof(x_coord) / sizeof(double), x_coord, y_coord, vol, fld);
	}

	return frame.create_scene();
}
