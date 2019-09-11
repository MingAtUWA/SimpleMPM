#include "Test_pcp.h"

#include "FileCharArray.h"
#include "TextResultFileInfo.h"
#include "TextResultFileParser.hpp"
#include "Frame_Particle2D.h"

#include "test_sim_core.h"

void display_model(void)
{
	TextResultFileInfo res_file_info;
	TextResultFile::Parser<PFileCharArray> parser;

	const char *ms_file1_path = "res_file\\ModelState.txt";
	FileCharArray ms_file1;
	ms_file1.open_file(ms_file1_path);
	if (!ms_file1.is_valid())
		return;

	parser.parse(&res_file_info, ms_file1);
	res_file_info.print(std::cout);

	ModelStateNode *ms_node = res_file_info.first_model_state();
	BackgroundMeshNode *bg_mesh = ms_node->background_mesh;
	ObjectNode *obj_node = ms_node->first_object();
	Frame_Particle2D frame;
	frame.add_mesh(bg_mesh->x_coord_num, bg_mesh->x_coords.get_mem(),
				   bg_mesh->y_coord_num, bg_mesh->y_coords.get_mem());
	frame.add_particles(obj_node->particle_num,
						obj_node->x.get_mem(), obj_node->y.get_mem(),
						obj_node->vol.get_mem());
	vtkSmartPointer<vtkRenderWindow> window = vtkSmartPointer<vtkRenderWindow>::New();
	window->AddRenderer(frame.create_scene());
	vtkSmartPointer<vtkRenderWindowInteractor> interactor
		= vtkSmartPointer<vtkRenderWindowInteractor>::New();
	interactor->SetRenderWindow(window);

	window->Render();
	interactor->Start();
}