#include "Test_pcp.h"

#include "test_sim_core.h"

#include "Model_R2D_ME_MPM.h"
#include "Step_R2D_ME_MPM.h"

#include "ResultFile_Text.h"

#include "TimeHistory_ConsoleProgressBar.h"
#include "TimeHistory_Particle_2D_ME_AllPcl.h"

void test_multi_object_me1(void)
{
	size_t i, j;

	Model_R2D_ME_MPM model;
	
	Mesh_BG_R2D_ME &mesh = model.mesh;
	// geometry
	double node_x_coords[] = { 0.0, 1.0};
	double node_y_coords[] = { 0.0, 1.0, 2.0, 3.0, 4.0, 5.0};
	mesh.add_x_coord(sizeof(node_x_coords) / sizeof(node_x_coords[0]), node_x_coords);
	mesh.add_y_coord(sizeof(node_y_coords) / sizeof(node_y_coords[0]), node_y_coords);
	// boundary conidtions on mesh
	VelocityBC vbc;
	vbc.v = 0.0;
	mesh.set_vx_bcs_num(mesh.node_y_num * 2);
	for (i = 0; i < mesh.node_y_num; i++)
	{
		vbc.node_id = i * mesh.node_x_num;
		mesh.add_vx_bc(vbc);
	}
	for (i = mesh.node_y_num; i < mesh.node_y_num * 2; i++)
	{
		vbc.node_id = (i + 1) * mesh.node_x_num - 1;
		mesh.add_vx_bc(vbc);
	}
	mesh.set_vy_bcs_num(mesh.node_x_num);
	for (size_t i = 0; i < mesh.node_x_num; i++)
	{
		vbc.node_id = i;
		mesh.add_vy_bc(vbc);
	}
	mesh.update();
	
	Object_Particle_2D_ME *pobj;
	Particle_2D_ME pcl;
	model.set_object_num(2);
	// obj1
	pobj = model.make_object();
	pobj->set_pcl_num(4);
	for (i = 0; i < 2; i++)
		for (j = 0; j < 2; j++)
		{
			pcl.init();
			pcl.x = 0.25 + (double)j * 0.5;
			pcl.y = 0.25 + (double)i * 0.5;
			pcl.density = 2.0;
			pcl.m = 1.0 * 1.0 / 4.0 * pcl.density;
			pcl.E = 100.0;
			pcl.niu = 0.3;
			pobj->add_pcl(pcl);
		}
	pobj->update();
	// obj2
	pobj = model.make_object();
	pobj->set_pcl_num(4);
	for (i = 0; i < 2; i++)
		for (j = 0; j < 2; j++)
		{
			pcl.init();
			pcl.x = 0.25 + (double)j * 0.5;
			pcl.y = 3.25 + (double)i * 0.5;
			pcl.density = 2.0;
			pcl.m = 1.0 * 1.0 / 4.0 * pcl.density;
			pcl.E = 100.0;
			pcl.niu = 0.3;
			pobj->add_pcl(pcl);
		}
	// BCs
	BodyForce bf;
	pobj->set_bfy_num(2);
	for (i = 0; i < 2; i++)
	{
		bf.pcl_id = 3 - i;
		bf.bf = -0.05;
		pobj->add_bfy(bf);
	}
	pobj->update();
	
	ResultFile_Text res_file;
	res_file.set_filename("res_file");

	Step_R2D_ME_MPM step1;
	step1.set_name("initial_step");
	step1.set_model(&model);
	step1.set_result_file(&res_file);
	step1.set_step_time(1.0); // total_time
	step1.set_dt(1.0e-3);

	TimeHistory_Particle_2D_ME_AllPcl th1;
	th1.set_name("test_out1");
	th1.set_interval_num(20);
	th1.set_if_output_initial_state(false);
	Particle_Field_2D_ME fld1[8] = {
		Particle_Field_2D_ME::x,
		Particle_Field_2D_ME::y,
		Particle_Field_2D_ME::vol,
		Particle_Field_2D_ME::vx,
		Particle_Field_2D_ME::vy,
		Particle_Field_2D_ME::e11,
		Particle_Field_2D_ME::e12,
		Particle_Field_2D_ME::e22
	};
	th1.set_field_num(sizeof(fld1) / sizeof(fld1[0]));
	for (i = 0; i < sizeof(fld1) / sizeof(fld1[0]); i++)
		th1.add_field(fld1[i]);
	th1.set_if_output_initial_state(true);
	step1.add_output(&th1);

	TimeHistory_ConsoleProgressBar th2;
	step1.add_output(&th2);

	step1.solve();
}
