#ifndef _NODEVAR_2D_ME_H_
#define _NODEVAR_2D_ME_H_

#include "Mesh.h"

struct Node_BG_R2D_ME;
struct Object_Particle_2D_ME;

// Nodal calculation variables
// For 2 dimensional coupled hydro-mechanics case
struct NodeVar_2D_ME : public NodeVar
{
	Node_BG_R2D_ME *node;

	Object_Particle_2D_ME *object;

	// mass (for mapping)
	double m;
	// momentum (for mapping)
	double mmx, mmy;

	double ax, vx, dux;
	double ay, vy, duy;
	// external force
	double fx_ext, fy_ext;
	// internal force
	double fx_int, fy_int;

	NodeVar_2D_ME *next;
};

// variables for nodal contact calculation
struct Node_Contact_Var_2D_ME
{
	// surface normal vector
	double nx, ny;
	// relative velocity (with respect to centre of mass) 
	double vrx, vry;
	// normal relative velocity (with respect to centre of mass) 
	double vrn, vrnx, vrny;
	// tangential relative velocity (with respect to centre of mass) 
	double vrtx, vrty;
	// normal contact force
	double fcnx, fcny;
	// tangential contact force
	double fctx, fcty;
	// if this node is in contact
	// used only when multiple objects are in contact
	bool is_in_contact;

	Node_Contact_Var_2D_ME *next;
};

#endif