#ifndef _MESH_R2D_ME_H_
#define _MESH_R2D_ME_H_

#include "ItemArray.hpp"
#include "Mesh_R2D.hpp"

#include "BC.h"
#include "NodeVar_2D_ME.h"
#include "ParticleVar_R2D_ME.h"

struct Node_BG_R2D_ME : public Node_R2D
{
public:
	size_t object_num;
	NodeVar_2D_ME *var_list;
	Node_Contact_Var_2D_ME *contact_var_list;

	// centre of mass velocity
	bool is_in_contact;
	double vx_cm, vy_cm;

public:
	// Nodal Variables
	// (each nodal variables is associated with one object)
	inline void init(void)
	{
		var_list = nullptr;
		object_num = 0;
		is_in_contact = false;
	}
	inline void add(NodeVar_2D_ME *nv)
	{
		nv->next = var_list;
		var_list = nv;
		++object_num;
	}
	inline NodeVar_2D_ME *first(void) const { return var_list; }
	inline NodeVar_2D_ME *next(NodeVar_2D_ME *cur) const { return cur->next; }

	// Nodal Contact Variables
	inline void init_contact(void) { contact_var_list = nullptr; }
	// each nodal variables is associated with one object
	inline void add_contact_var(Node_Contact_Var_2D_ME *ncv)
	{
		ncv->next = contact_var_list;
		contact_var_list = ncv;
	}
	inline Node_Contact_Var_2D_ME *first_contact_var(void) const { return contact_var_list; }
	inline Node_Contact_Var_2D_ME *next_contact_var(Node_Contact_Var_2D_ME *cur) const { return cur->next; }
};

struct Element_BG_R2D_ME : public Element_R2D
{
public:
	ParticleVar_R2D_ME *pcl_var_list;

public:
	inline void init(void) { pcl_var_list = nullptr; }
	inline void add_pcl_var(ParticleVar_R2D_ME *ppcl_var)
	{

		ppcl_var->next_by_elem = pcl_var_list;
		pcl_var_list = ppcl_var;
	}
	inline ParticleVar_R2D_ME *first_pcl_var(void) const { return pcl_var_list; }
	inline ParticleVar_R2D_ME *next_pcl_var(ParticleVar_R2D_ME *cur) const { return cur->next_by_elem; }
};

struct Mesh_BG_R2D_ME : public Mesh_R2D<Node_BG_R2D_ME, Element_BG_R2D_ME>
{
public:
	// Boundary conditions on mesh
	size_t ax_bc_num, ay_bc_num;
	AccelerationBC *ax_bcs, *ay_bcs;
	size_t vx_bc_num, vy_bc_num;
	VelocityBC *vx_bcs, *vy_bcs;
	
public:
	Mesh_BG_R2D_ME();
	~Mesh_BG_R2D_ME();

	inline void set_ax_bcs_num(size_t num) { ax_bcs_mem.reserve(num); }
	inline void add_ax_bc(AccelerationBC &abc) { ax_bcs_mem.add(abc); }
	inline void set_ay_bcs_num(size_t num) { ay_bcs_mem.reserve(num); }
	inline void add_ay_bc(AccelerationBC &abc) { ay_bcs_mem.add(abc); }
	inline void set_vx_bcs_num(size_t num) { vx_bcs_mem.reserve(num); }
	inline void add_vx_bc(VelocityBC &vbc) { vx_bcs_mem.add(vbc); }
	inline void set_vy_bcs_num(size_t num) { vy_bcs_mem.reserve(num); }
	inline void add_vy_bc(VelocityBC &vbc) { vy_bcs_mem.add(vbc); }
	void update(void);
	void clear(void);

protected:
	// boundary conditions
	MemoryUtilities::ItemArray<AccelerationBC> ax_bcs_mem;
	MemoryUtilities::ItemArray<AccelerationBC> ay_bcs_mem;
	MemoryUtilities::ItemArray<VelocityBC> vx_bcs_mem;
	MemoryUtilities::ItemArray<VelocityBC> vy_bcs_mem;
};

#endif