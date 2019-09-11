#include "Test_pcp.h"

#include "Model_S2D_ME_s_RigidBody.h"
#include "test_sim_core.h"

void test_get_intersection_point(Model_S2D_ME_s_RigidBody &model)
{
	Model_S2D_ME_s_RigidBody::LongLongRange y_id_range;
	Model_S2D_ME_s_RigidBody::PreAllocIdArray x_ids_mem;
	if (model.get_intersect_points(-1.0, 6.0, 5.0, 0.0, y_id_range, x_ids_mem))
	{
		std::cout << "intersection point are: \n";
		long long *x_ids = x_ids_mem.get_mem();
		long long x_id_num = x_ids_mem.get_num();
		for (long long i = 0; i < x_id_num; ++i)
		{
			std::cout << "(" << x_ids[i] << ", " << y_id_range.lower + i << ") ";
			if (i % 5 == 4) std::cout << "\n";
		}
		std::cout << "\n";
	}
	else
	{
		std::cout << "y_id out of range.\n";
	}
}

void disp_bg_mesh(Model_S2D_ME_s_RigidBody &md)
{
	size_t elem_id = 0;
	std::cout << "\n";
	for (size_t line_id = 0; line_id < md.elem_y_num; ++line_id)
	{
		for (size_t col_id = 0; col_id < md.elem_x_num; ++col_id)
		{
			if (md.elems[elem_id].has_rigid_object)
				std::cout << "O ";
			else
				std::cout << "X ";
			++elem_id;
		}
		std::cout << "\n";
	}
	std::cout << "\n";
}

void test_mesh_resterization(void)
{
    Model_S2D_ME_s_RigidBody model;
    
	model.init_mesh(1.0, 5.0, 5.0, -2.0, -2.0);
	for (size_t i = 0; i < model.elem_num; ++i)
		model.elems[i].reset();

	//test_get_intersection_point(model);

	//model.rasterize_rect_on_grid(-0.5, -0.5, 1.5, -0.5, 1.5, 1.5, -0.5, 1.5);
	model.rasterize_rect_on_grid(-2.5, -2.5, 3.5, -2.5, 3.5, 3.5, -2.5, 2.5);

	disp_bg_mesh(model);
}
