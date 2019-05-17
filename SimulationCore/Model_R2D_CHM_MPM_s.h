#ifndef _MODEL_R2D_CHM_MPM_S_H_
#define _MODEL_R2D_CHM_MPM_S_H_

#include "Node_R2D_CHM.h"
#include "Element_R2D_CHM_MPM.h"
#include "Particle_2D_CHM.h"
#include "BC.h"
#include "Model.h"

struct Model_R2D_CHM_MPM_s : public Model
{
public:
	// background mesh
	size_t node_x_num;
	double *node_coords_x;
	size_t node_y_num;
	double *node_coords_y;
	size_t node_num;
	Node_R2D_CHM *nodes;
	size_t elem_x_num;
	size_t elem_y_num;
	size_t elem_num;
	Element_R2D_CHM_MPM *elems;

	size_t pcl_num;
	Particle_2D_CHM *pcls;

	size_t bfx_num, bfy_num;
	BodyForce *bfxs, *bfys;
	size_t tx_bc_num, ty_bc_num;
	TractionBC_MPM *tx_bcs, *ty_bcs;

	size_t ax_s_bc_num, ay_s_bc_num;
	AccelerationBC *ax_s_bcs, *ay_s_bcs;
	size_t vx_s_bc_num, vy_s_bc_num;
	VelocityBC *vx_s_bcs, *vy_s_bcs;

	size_t ax_f_bc_num, ay_f_bc_num;
	AccelerationBC *ax_f_bcs, *ay_f_bcs;
	size_t vx_f_bc_num, vy_f_bc_num;
	VelocityBC *vx_f_bcs, *vy_f_bcs;

	// factor of local damping
	double alpha_s, alpha_f;
	// factor of mass scaling
	double mass_scaling_factor;

public:
	Model_R2D_CHM_MPM_s();
	~Model_R2D_CHM_MPM_s();

	void set_local_damping(double a_s, double a_f);
	
	Element_R2D_CHM_MPM *find_in_which_element(double x, double y);
	Element_R2D_CHM_MPM *find_in_which_element(double x, double y, Element_R2D_CHM_MPM *elem);

	double cal_characteristic_length(Element_R2D_CHM_MPM *elem);

	void cal_shape_function(Particle_2D_CHM *pcl);

protected:
	// for accelerating inWhichElement(double *coords;)
	Element_R2D_CHM_MPM *elem_buffer;
	// Find in which coords interval the coordinate lies
	// return -1 if coordinate lie out side the cordinates range
	int find_x_index(double x, size_t *index);
	int find_y_index(double y, size_t *index);
};

#endif