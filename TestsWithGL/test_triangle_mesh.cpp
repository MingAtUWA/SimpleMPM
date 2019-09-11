#include "TestsWithGL_pcp.h"

#include "test_sim_core.h"
#include "TriangleMesh.h"

void test_triangle_mesh(void)
{
	TriangleMesh mesh;
	mesh.load_mesh("square_mesh.mesh_data");
	mesh.get_bounding_box();
	mesh.find_edges_at_boundary();
	mesh.init_bg_grid();
#ifdef __DEBUG_TRIANGLE_MESH__
	//mesh.dump_mesh_info("square_mesh.txt", true);
#endif
	
//	Point pt1 = { 0.2, 1.1 };
//	double dist, nx, ny;
//	mesh.distance_to_boundary(pt1, dist, nx, ny);
//	std::cout << "dist " << dist << " normal (" << nx << "," << ny << ")\n";
//
	//display_triangle_mesh(mesh, true, true, true, mesh.closest_edge, &pt1);
	display_triangle_mesh(mesh, true, true, true);

	// point to edge distance
	//Edge e1 = { 7, 8 };
	//mesh.distance_to_edge(e1, pt1, dist, nx, ny);
	//std::cout << "dist " << dist << " normal (" << nx << "," << ny << ")\n";
}
