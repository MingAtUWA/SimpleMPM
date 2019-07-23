#include "Test_pcp.h"

#include "Model_S2D_ME_MPM_s.h"

#include "test_sim_core.h"

void print_pcl(Particle_S2D_ME &pcl)
{
	std::cout << "pcl " << pcl.index << "\n";
	if (pcl.elem_num == 0)
		std::cout << "out of mesh.\n";
	for (size_t i = 0; i < pcl.elem_num; i++)
	{
		std::cout << "coord: " << pcl.vars[i].x << ", " << pcl.vars[i].y
				<< ", vol: " << pcl.vars[i].vol
				<< ", elem: " << pcl.vars[i].elem_x_id << ", " << pcl.vars[i].elem_y_id << "\n";
	}
}

void test_Model_S2D_ME_s(void)
{
	Model_S2D_ME_MPM_s model;

	model.init_mesh(2.0, 3, 3);
	model.init_pcl(20, 1.0, 1.0, 100.0, 0.3);
	// out of mesh, left
	model.pcls[0].x = -1.0;
	model.pcls[0].y = 1.0;
	model.get_elements_overlapped_by_particle(model.pcls[0]);
	print_pcl(model.pcls[0]);
	// out of mesh, up
	model.pcls[1].x = 3.0;
	model.pcls[1].y = 7.0;
	model.get_elements_overlapped_by_particle(model.pcls[1]);
	print_pcl(model.pcls[1]);
	// out of mesh, left at edge
	model.pcls[2].x = -0.5;
	model.pcls[2].y = 1.0;
	model.get_elements_overlapped_by_particle(model.pcls[2]);
	print_pcl(model.pcls[2]);
	// in one element
	model.pcls[3].x = 3.0;
	model.pcls[3].y = 3.0;
	model.get_elements_overlapped_by_particle(model.pcls[3]);
	print_pcl(model.pcls[3]);
	// in one element, left edge
	model.pcls[4].x = 2.5;
	model.pcls[4].y = 3.0;
	model.get_elements_overlapped_by_particle(model.pcls[4]);
	print_pcl(model.pcls[4]);
	// in one element, right edge
	model.pcls[5].x = 3.5;
	model.pcls[5].y = 3.0;
	model.get_elements_overlapped_by_particle(model.pcls[5]);
	print_pcl(model.pcls[5]);
	// in one element, right and top edge
	model.pcls[6].x = 3.5;
	model.pcls[6].y = 3.5;
	model.get_elements_overlapped_by_particle(model.pcls[6]);
	print_pcl(model.pcls[6]);
	// cover two element with same y
	model.pcls[7].x = 1.9;
	model.pcls[7].y = 3.0;
	model.get_elements_overlapped_by_particle(model.pcls[7]);
	print_pcl(model.pcls[7]);
	// cover two element with same x
	model.pcls[8].x = 2.5;
	model.pcls[8].y = 3.9;
	model.get_elements_overlapped_by_particle(model.pcls[8]);
	print_pcl(model.pcls[8]);
	// cover four element
	model.pcls[9].x = 1.9;
	model.pcls[9].y = 3.9;
	model.get_elements_overlapped_by_particle(model.pcls[9]);
	print_pcl(model.pcls[9]);
	// cover four element, big element
	model.pcls[10].x = 4.0;
	model.pcls[10].y = 4.0;
	model.pcls[10].m = 16.0;
	model.get_elements_overlapped_by_particle(model.pcls[10]);
	print_pcl(model.pcls[10]);
	// cover the edge
	model.pcls[11].x = -0.1;
	model.pcls[11].y = 5.5;
	model.get_elements_overlapped_by_particle(model.pcls[11]);
	print_pcl(model.pcls[11]);
	// cover the edge
	model.pcls[12].x = 5.9;
	model.pcls[12].y = 5.9;
	model.get_elements_overlapped_by_particle(model.pcls[12]);
	print_pcl(model.pcls[12]);
}
