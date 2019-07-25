#include "Test_pcp.h"

#include "Model_S2D_ME_MPM_s.h"

#include "test_sim_core.h"

#include <cstdio>

void print_pcl(Particle_S2D_ME &pcl)
{
	std::cout << "pcl " << pcl.index << "\n";
	if (pcl.elem_num == 0)
		std::cout << "out of mesh.\n";
	for (size_t i = 0; i < pcl.elem_num; i++)
	{
		printf("coord: %.2lf, %.2lf, vol: %.4lf, elem: %zu, %zu\n",
			   pcl.vars[i].x, pcl.vars[i].y, pcl.vars[i].vol,
			   pcl.vars[i].elem_x_id, pcl.vars[i].elem_y_id);
	}
}

void test_Model_S2D_ME_s(void)
{
	Model_S2D_ME_MPM_s model;

	model.init_mesh(2.0, 3, 3);
	model.init_pcl(20, 1.0, 1.0, 100.0, 0.3);
	size_t k = 0;
	// out of mesh, left
	model.pcls[k].x = -1.0;
	model.pcls[k].y = 1.0;
	model.get_elements_overlapped_by_particle(model.pcls[k]);
	print_pcl(model.pcls[k]);
	++k;
	// out of mesh, up
	model.pcls[k].x = 3.0;
	model.pcls[k].y = 7.0;
	model.get_elements_overlapped_by_particle(model.pcls[k]);
	print_pcl(model.pcls[k]);
	++k;
	// out of mesh, left at edge
	model.pcls[k].x = -0.5;
	model.pcls[k].y = 1.0;
	model.get_elements_overlapped_by_particle(model.pcls[k]);
	print_pcl(model.pcls[k]);
	++k;
	// in one element
	model.pcls[k].x = 3.0;
	model.pcls[k].y = 3.0;
	model.get_elements_overlapped_by_particle(model.pcls[k]);
	print_pcl(model.pcls[k]);
	++k;
	// in one element, left edge
	model.pcls[k].x = 2.5;
	model.pcls[k].y = 3.0;
	model.get_elements_overlapped_by_particle(model.pcls[k]);
	print_pcl(model.pcls[k]);
	++k;
	// in one element, right edge
	model.pcls[k].x = 3.5;
	model.pcls[k].y = 3.0;
	model.get_elements_overlapped_by_particle(model.pcls[k]);
	print_pcl(model.pcls[k]);
	++k;
	// in one element, right and top edge
	model.pcls[k].x = 3.5;
	model.pcls[k].y = 3.5;
	model.get_elements_overlapped_by_particle(model.pcls[k]);
	print_pcl(model.pcls[k]);
	++k;
	// cover two element with same y
	model.pcls[k].x = 1.9;
	model.pcls[k].y = 3.0;
	model.get_elements_overlapped_by_particle(model.pcls[k]);
	print_pcl(model.pcls[k]);
	++k;
	// cover two element with same x
	model.pcls[k].x = 2.5;
	model.pcls[k].y = 3.9;
	model.get_elements_overlapped_by_particle(model.pcls[k]);
	print_pcl(model.pcls[k]);
	++k;
	// cover four element
	model.pcls[k].x = 1.9;
	model.pcls[k].y = 3.9;
	model.get_elements_overlapped_by_particle(model.pcls[k]);
	print_pcl(model.pcls[k]);
	++k;
	// cover four element, big element
	model.pcls[k].x = 4.0;
	model.pcls[k].y = 4.0;
	model.pcls[k].m = 16.0;
	model.get_elements_overlapped_by_particle(model.pcls[k]);
	print_pcl(model.pcls[k]);
	++k;
	// cover the edge
	model.pcls[k].x = 0.1;
	model.pcls[k].y = 5.5;
	model.get_elements_overlapped_by_particle(model.pcls[k]);
	print_pcl(model.pcls[k]);
	++k;
	// cover the edge
	model.pcls[k].x = 5.9;
	model.pcls[k].y = 5.9;
	model.get_elements_overlapped_by_particle(model.pcls[k]);
	print_pcl(model.pcls[k]);
	++k;

	// large particle
	model.pcls[k].x = 3.0;
	model.pcls[k].y = 3.0;
	model.pcls[k].m = 16.0;
	model.get_elements_overlapped_by_particle(model.pcls[k]);
	print_pcl(model.pcls[k]);
	++k;
	//
	model.pcls[k].x = 3.0;
	model.pcls[k].y = 3.0;
	model.pcls[k].m = 25.0;
	model.get_elements_overlapped_by_particle(model.pcls[k]);
	print_pcl(model.pcls[k]);
	++k;
}
