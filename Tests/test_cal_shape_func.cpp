#include "Test_pcp.h"

#include "Model_R2D_CHM_MPM_s.h"

#include "test_sim_core.h"

void test_cal_shape_func(void)
{
	size_t i, j, k;
	Model_R2D_CHM_MPM_s model;

	model.node_x_num = 3;
	model.node_y_num = 3;
	model.node_coords_x = new double[model.node_x_num];
	for (i = 0; i < model.node_x_num; i++)
	{
		model.node_coords_x[i] = (double)i;
	}
	model.node_coords_y = new double[model.node_y_num];
	for (i = 0; i < model.node_y_num; i++)
	{
		model.node_coords_y[i] = (double)i;
	}
	model.node_num = model.node_x_num * model.node_y_num;
	model.nodes = new Node_R2D_CHM[model.node_num];
	k = 0;
	for (i = 0; i < model.node_y_num; i++)
		for (j = 0; j < model.node_x_num; j++)
		{
			model.nodes[k].index_x = j;
			model.nodes[k].index_y = i;
			++k;
		}

	model.elem_x_num = model.node_x_num - 1;
	model.elem_y_num = model.node_y_num - 1;
	model.elem_num = model.elem_x_num * model.elem_y_num;
	model.elems = new Element_R2D_CHM_MPM[model.elem_num];
	k = 0;
	for (i = 0; i < model.elem_y_num; i++)
		for (j = 0; j < model.elem_x_num; j++)
		{
			model.elems[k].index_x = j;
			model.elems[k].index_y = i;
			++k;
		}

	model.pcl_num = 1;
	model.pcls = new Particle_2D_CHM[model.pcl_num];

	model.pcls[0].x = 0.8;
	model.pcls[0].y = 1.2;

	model.pcls[0].elem = nullptr;
	model.pcls[0].elem = model.find_in_which_element(model.pcls[0].x, model.pcls[0].y, model.pcls[0].elem);
	std::cout << model.pcls[0].elem->index_x << " " << model.pcls[0].elem->index_y << std::endl;
	model.cal_shape_function(&model.pcls[0]);
}