#include "Test_pcp.h"

#include "test_sim_core.h"

#include "Model_R2D_ME_MPM_s.h"
#include "Model_R2D_CHM_MPM_s.h"

template<class Model, class Element>
void find_elem(Model &md, double x, double y, Element *&pelem)
{
	pelem = md.find_in_which_element(x, y, pelem);
	if (pelem)
	{
		std::cout << "(" << x << ", " << y << ") is in element ("
				<< pelem->index_x << ", " << pelem->index_y << ") "
				<< pelem->index_y * md.elem_x_num + pelem->index_x
				<< ".\n";
	}
	else
	{
		std::cout << "(" << x << ", " << y << ") is out of mesh.\n";
	}
}

template<class Model, class Element, class Node>
void test_mesh_template(void)
{
	size_t i, j, k;
	Model model;

	model.node_x_num = 3;
	model.node_y_num = 11;

	model.node_coords_x = new double[model.node_x_num];
	for (i = 0; i < model.node_x_num; i++)
		model.node_coords_x[i] = (double)i;

	model.node_coords_y = new double[model.node_y_num];
	for (i = 0; i < model.node_y_num; i++)
		model.node_coords_y[i] = (double)i;

	model.node_num = model.node_x_num * model.node_y_num;
	model.nodes = new Node[model.node_num];
	k = 0;
	for (i = 0; i < model.node_y_num; ++i)
		for (j = 0; j < model.node_x_num; ++j)
		{
			model.nodes[k].index_x = j;
			model.nodes[k].index_y = i;
			++k;
		}

	model.elem_x_num = model.node_x_num - 1;
	model.elem_y_num = model.node_y_num - 1;
	model.elem_num = model.elem_x_num * model.elem_y_num;
	model.elems = new Element[model.elem_num];
	k = 0;
	for (i = 0; i < model.elem_y_num; i++)
		for (j = 0; j < model.elem_x_num; j++)
		{
			model.elems[k].index_x = j;
			model.elems[k].index_y = i;
			++k;
		}

	Element *pelem1;
	pelem1 = nullptr;
	find_elem<Model, Element>(model, 0.5, 4.5, pelem1);
	find_elem<Model, Element>(model, 1.5, 4.5, pelem1);
	find_elem<Model, Element>(model, 0.5, 3.5, pelem1);
	find_elem<Model, Element>(model, 0.5, 4.5, pelem1);
	find_elem<Model, Element>(model, -0.5, 2.5, pelem1);
	find_elem<Model, Element>(model, 1.5, 2.5, pelem1);
	find_elem<Model, Element>(model, 0.5, 2.5, pelem1);
}

void test_mesh(void)
{
	std::cout << "ME:\n";
	test_mesh_template<Model_R2D_ME_MPM_s, Element_R2D_ME_MPM_s, Node_R2D_ME_s>();
	
	std::cout << "CHM:\n";
	test_mesh_template<Model_R2D_CHM_MPM_s, Element_R2D_CHM_MPM_s, Node_R2D_CHM_s>();
}