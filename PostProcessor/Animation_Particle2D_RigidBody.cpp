#include <iostream>

#include "PostProcessor_pcp.h"

#include "Animation_Particle2D.h"
//
//Animation_Particle2D::
//	Animation_Particle2D(double animation_time, vtkRenderWindowInteractor *win_iter) :
//	Animation(animation_time, win_iter) {}
//
//Animation_Particle2D::~Animation_Particle2D() {}
//
//int Animation_Particle2D::init(const char *res_file_name)
//{
//	if (!res_file_name)
//		return -2;
//	
//	file.open_file(res_file_name);
//	if (!file.is_valid())
//		return -2;
//	parser.parse(&res_file_info, file);
//	//res_file_info.print(std::cout);
//	
//	// select output to be displayed
//	time_history = res_file_info.first_time_history();
//
//	// init frame_num and total_sim_time
//	TimeRecordNode *time_rcd = time_history->first_time_rcd();
//	frame_num = 0;
//	double max_sim_time = time_rcd->total_time;
//	double min_sim_time = time_rcd->total_time;
//	while (time_rcd)
//	{
//		if (max_sim_time < time_rcd->total_time)
//			max_sim_time = time_rcd->total_time;
//		if (min_sim_time > time_rcd->total_time)
//			min_sim_time = time_rcd->total_time;
//		++frame_num;
//		time_rcd = time_history->next_time_rcd(time_rcd);
//	}
//	total_sim_time = max_sim_time - min_sim_time;
//
//	if (!frame_num)
//		return -3;
//	time_rcds.reset();
//	time_rcds.reserve(frame_num);
//	
//	for (time_rcd = time_history->first_time_rcd(); time_rcd;
//		 time_rcd = time_history->next_time_rcd(time_rcd))
//		time_rcds.add(time_rcd);
//	//std::cout << time_rcds.get_num() << std::endl;
//
//	// add mesh data here
//	//frame.add_mesh();
//
//	return 0;
//}
//
//int  Animation_Particle2D::get_background_mesh(const char *mesh_file_name)
//{
//	if (!mesh_file_name)
//		return -2;
//
//	FileCharArray mesh_file(mesh_file_name);
//	if (!mesh_file.is_valid())
//		return -2;
//	parser.parse(&res_file_info, mesh_file);
//	//res_file_info.print(std::cout);
//
//	ModelStateNode *model_state = res_file_info.first_model_state();
//	BackgroundMeshNode *bg_mesh = model_state->background_mesh;
//	// create scene
//	frame.add_mesh(bg_mesh->x_coord_num, bg_mesh->x_coords.get_mem(),
//				   bg_mesh->y_coord_num, bg_mesh->y_coords.get_mem());
//	//for (size_t i = 0; i < bg_mesh->x_coord_num; i++)
//	//{
//	//	std::cout << bg_mesh->x_coords[i] << " ";
//	//}
//
//	return 0;
//}
//
//double Animation_Particle2D::get_frame_time(size_t frame_index)
//{
//	return time_rcds[frame_index]->total_time;
//}
//
//vtkRenderer *Animation_Particle2D::render_frame(size_t frame_index)
//{
//	size_t point_num;
//	
//	// Initialize parsing field data
//	fld_parser.init(time_rcds[frame_index]->field_data, 
//					file, time_history->get_field_num());
//	point_num = time_rcds[frame_index]->point_num;
//	pcl_x_coord.reset();
//	pcl_x_coord.reserve(point_num);
//	pcl_y_coord.reset();
//	pcl_y_coord.reserve(point_num);
//	pcl_vol.reset();
//	pcl_vol.reserve(point_num);
//	pcl_fld.reset();
//	pcl_fld.reserve(point_num);
//
//	// Begin parsing field data
//	double *fld_data;
//	size_t fld_data_num;
//	fld_data = fld_parser.parse_next_point();
//	point_num = 0;
//	while (fld_data)
//	{
//		++point_num;
//		fld_data_num = fld_parser.get_field_num();
//		// x
//		pcl_x_coord.add(fld_data[0]);
//		// y
//		pcl_y_coord.add(fld_data[1]);
//		// vol
//		pcl_vol.add(fld_data[2]);
//		// field
//		pcl_fld.add(fld_data[3]);
//		fld_data = fld_parser.parse_next_point();
//	}
//
//	// create scene
//	frame.add_particles(point_num, 
//						pcl_x_coord.get_mem(),
//						pcl_y_coord.get_mem(),
//						pcl_vol.get_mem(),
//						pcl_fld.get_mem());
//	return frame.create_scene();
//}
