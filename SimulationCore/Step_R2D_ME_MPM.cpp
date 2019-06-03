#include "SimulationCore_pcp.h"

#include "Step_R2D_ME_MPM.h"

inline size_t max(size_t a, size_t b) { return a > b ? a : b; }

Step_R2D_ME_MPM::Step_R2D_ME_MPM() :
	Step(&solve_substep_R2D_ME_MPM),
	model(nullptr) {}

Step_R2D_ME_MPM::~Step_R2D_ME_MPM() {}

int Step_R2D_ME_MPM::init()
{
	Object_Particle_2D_ME *pobj;
	Object_Particle_2D_ME *pobj_end = pobj_end = objects->eoi();
	Particle_2D_ME *ppcl;
	ParticleVar_R2D_ME *ppcl_var;

	mesh = &(model->mesh);
	objects = &(model->objects);
	node_vars = &(model->node_vars);
	pcl_vars = &(model->pcl_vars);
	contact_vars = &(model->contact_vars);

	// init particles
	size_t total_pcl_num = 0; // total number of particles (all objects)
	for (pobj = objects->first(); pobj != pobj_end; 
		 pobj = objects->next(pobj))
	{
		total_pcl_num += pobj->pcl_num;
		for (size_t i = 0; i < pobj->pcl_num; i++)
		{
			ppcl = pobj->pcls + i;
			ppcl->object = pobj;
			ppcl->x_ori = ppcl->x;
			ppcl->y_ori = ppcl->y;
			ppcl->ux = 0.0;
			ppcl->uy = 0.0;
		}
	}

	// Initialize buffer for particle & nodal & contact variables
	node_vars->set_default_pool_item_num(total_pcl_num);
	pcl_vars->set_default_pool_item_num(total_pcl_num);
	// assume 1 / 10 of particles are in contact 
	contact_vars->set_default_pool_item_num(max(total_pcl_num, 10) / 10);

	// Initialize particle variables
	if (is_first_step)
	{
		for (pobj = objects->first(); pobj != pobj_end;
			 pobj = objects->next(pobj))
		{
			for (size_t i = 0; i < pobj->pcl_num; i++)
			{
				ppcl = pobj->pcls + i;
				ppcl_var = pcl_vars->alloc();
				// init
				ppcl->var = ppcl_var;
				ppcl_var->pcl = ppcl;
				ppcl_var->elem = nullptr;
			}
		}
	}

	return 0;
}

int Step_R2D_ME_MPM::finalize() { return 0; }

int solve_substep_R2D_ME_MPM(void *_self)
{
	Step_R2D_ME_MPM *self = (Step_R2D_ME_MPM *)_self;
	
	self->init_per_substep();
	self->map_to_nodes_and_cal_internal_force();
	self->cal_external_force();
	self->update_nodal_variables();
	self->contact_calculation();
	self->map_to_particles_and_update_particle_variables();
	
	return 0;
}


// helper function of init_per_substep()
inline NodeVar_2D_ME *Step_R2D_ME_MPM::init_node_variables(Particle_2D_ME *ppcl, Node_BG_R2D_ME *pnd)
{
	NodeVar_2D_ME *pnd_var = static_cast<NodeVar_2D_ME *>(pnd->first());
	if (!pnd_var || pnd_var->object != static_cast<Object *>(ppcl->object))
	{
		pnd_var = node_vars->alloc();
		// add to stack at node
		pnd->add(pnd_var);
		// init node variable
		pnd_var->node = pnd;
		pnd_var->object = ppcl->object;
		pnd_var->m = 0.0;
		pnd_var->mmx = 0.0;
		pnd_var->mmy = 0.0;
		pnd_var->ax = 0.0;
		pnd_var->ay = 0.0;
		pnd_var->vx = 0.0;
		pnd_var->vy = 0.0;
		pnd_var->dux = 0.0;
		pnd_var->duy = 0.0;
		pnd_var->fx_ext = 0.0;
		pnd_var->fy_ext = 0.0;
		pnd_var->fx_int = 0.0;
		pnd_var->fy_int = 0.0;
	}
	return pnd_var;
}
int Step_R2D_ME_MPM::init_per_substep(void)
{
	ParticleVar_R2D_ME *ppcl_var, *ppcl_var_end, *ppcl_var_tmp;
	Particle_2D_ME *ppcl;
	Node_BG_R2D_ME *pnd1, *pnd2, *pnd3, *pnd4;
	Element_BG_R2D_ME *pelem;

	node_vars->reset();
	contact_vars->reset();
	for (size_t i = 0; i < mesh->node_num; i++)
		mesh->nodes[i].init();
	for (size_t i = 0; i < mesh->elem_num; i++)
		mesh->elems[i].init();

	ppcl_var_end = pcl_vars->eoi();
	ppcl_var = pcl_vars->first();
	while (ppcl_var != ppcl_var_end)
	{
		ppcl = ppcl_var->pcl;
		pelem = mesh->find_in_which_element(ppcl->x, ppcl->y, ppcl_var->elem);
		ppcl_var->elem = pelem;
		if (pelem) // particle is in background mesh
		{
			// init variables of this particle
			ppcl_var->vol = ppcl->m / ppcl->density;
			// shape functions and their derivatives
			model->cal_shape_function(ppcl_var);

			// init element associated with this particle
			// for contact calculation
			pelem->add_pcl_var(ppcl_var);

			// init nodes associated with this particle
			mesh->get_nodes_of_element(pelem, pnd1, pnd2, pnd3, pnd4);
			// node 1
			ppcl_var->node1_var = init_node_variables(ppcl, pnd1);
			// node 2
			ppcl_var->node2_var = init_node_variables(ppcl, pnd2);
			// node 3
			ppcl_var->node3_var = init_node_variables(ppcl, pnd3);
			// node 4
			ppcl_var->node4_var = init_node_variables(ppcl, pnd4);
			
			ppcl_var = pcl_vars->next(ppcl_var);
		}
		else // particle is out of background mesh
		{
			ppcl->var = nullptr;
			ppcl_var_tmp = ppcl_var;
			ppcl_var = pcl_vars->next(ppcl_var);
			pcl_vars->del(ppcl_var_tmp);
		}
	}
	return 0;
}


inline void map_to_node(Particle_2D_ME *ppcl,
						ParticleVar_R2D_ME *ppcl_var,
						size_t node_id)
{
	NodeVar_2D_ME *pnd_var = ppcl_var->node_var[node_id];
	double N     = ppcl_var->N[node_id];
	double dN_dx = ppcl_var->dN_dx[node_id];
	double dN_dy = ppcl_var->dN_dy[node_id];
	pnd_var->m   += ppcl->m * N;
	pnd_var->mmx += ppcl->m * ppcl->vx * N;
	pnd_var->mmy += ppcl->m * ppcl->vy * N;
	pnd_var->fx_int += (dN_dx * ppcl->s11 + dN_dy * ppcl->s12) * ppcl_var->vol;
	pnd_var->fy_int += (dN_dx * ppcl->s12 + dN_dy * ppcl->s22) * ppcl_var->vol;
}
int Step_R2D_ME_MPM::map_to_nodes_and_cal_internal_force(void)
{
	Particle_2D_ME *ppcl;
	ParticleVar_R2D_ME *ppcl_var, *ppcl_var_end;
	ppcl_var_end = pcl_vars->eoi();
	for(ppcl_var = pcl_vars->first(); ppcl_var != ppcl_var_end;
		ppcl_var = pcl_vars->next(ppcl_var))
	{
		ppcl = ppcl_var->pcl;
		// node 1
		map_to_node(ppcl, ppcl_var, 0);
		// node 2
		map_to_node(ppcl, ppcl_var, 1);
		// node 3
		map_to_node(ppcl, ppcl_var, 2);
		// node 4
		map_to_node(ppcl, ppcl_var, 3);
	}
	return 0;
}


int Step_R2D_ME_MPM::cal_external_force(void)
{
	Object_Particle_2D_ME *pobj, *pobj_end;
	Particle_2D_ME *ppcl;
	ParticleVar_R2D_ME *ppcl_var;
	double bf, tf;

	pobj_end = objects->eoi();
	for (pobj = objects->first(); pobj != pobj_end; 
		 pobj = objects->next(pobj))
	{
		// body force
		for (size_t i = 0; i < pobj->bfx_num; i++)
		{
			ppcl = pobj->pcls + pobj->bfxs[i].pcl_id;
			if (ppcl->var) // is in mesh
			{
				bf = ppcl->m * pobj->bfxs[i].bf;
				ppcl_var = static_cast<ParticleVar_R2D_ME *>(ppcl->var);
				ppcl_var->node1_var->fx_ext += ppcl_var->N1 * bf;
				ppcl_var->node2_var->fx_ext += ppcl_var->N2 * bf;
				ppcl_var->node3_var->fx_ext += ppcl_var->N3 * bf;
				ppcl_var->node4_var->fx_ext += ppcl_var->N4 * bf;
			}
		}
		for (size_t i = 0; i < pobj->bfy_num; i++)
		{
			ppcl = pobj->pcls + pobj->bfys[i].pcl_id;
			if (ppcl->var) // is in mesh
			{
				bf = ppcl->m * pobj->bfys[i].bf;
				ppcl_var = static_cast<ParticleVar_R2D_ME *>(ppcl->var);
				ppcl_var->node1_var->fy_ext += ppcl_var->N1 * bf;
				ppcl_var->node2_var->fy_ext += ppcl_var->N2 * bf;
				ppcl_var->node3_var->fy_ext += ppcl_var->N3 * bf;
				ppcl_var->node4_var->fy_ext += ppcl_var->N4 * bf;
			}
		}

		// surface force
		for (size_t i = 0; i < pobj->tx_bc_num; i++)
		{
			ppcl = pobj->pcls + pobj->tx_bcs[i].pcl_id;
			if (ppcl->var) // is in mesh
			{
				tf = pobj->tx_bcs[i].t;
				ppcl_var = static_cast<ParticleVar_R2D_ME *>(ppcl->var);
				ppcl_var->node1_var->fx_ext += ppcl_var->N1 * tf;
				ppcl_var->node2_var->fx_ext += ppcl_var->N2 * tf;
				ppcl_var->node3_var->fx_ext += ppcl_var->N3 * tf;
				ppcl_var->node4_var->fx_ext += ppcl_var->N4 * tf;
			}
		}
		for (size_t i = 0; i < pobj->ty_bc_num; i++)
		{
			ppcl = pobj->pcls + pobj->ty_bcs[i].pcl_id;
			if (ppcl->var) // is in mesh
			{
				tf = pobj->ty_bcs[i].t;
				ppcl_var = static_cast<ParticleVar_R2D_ME *>(ppcl->var);
				ppcl_var->node1_var->fy_ext += ppcl_var->N1 * tf;
				ppcl_var->node2_var->fy_ext += ppcl_var->N2 * tf;
				ppcl_var->node3_var->fy_ext += ppcl_var->N3 * tf;
				ppcl_var->node4_var->fy_ext += ppcl_var->N4 * tf;
			}
		}
		// pore pressure force here in the future...
	}

	return 0;
}


int Step_R2D_ME_MPM::update_nodal_variables(void)
{
	Node_BG_R2D_ME *pnd;
	NodeVar_2D_ME *pnd_var;

	// calculate acceleration
	for (pnd_var = node_vars->top(); pnd_var; pnd_var = node_vars->prev(pnd_var))
	{
		pnd_var->ax = (pnd_var->fx_ext - pnd_var->fx_int) / pnd_var->m;
		pnd_var->ay = (pnd_var->fy_ext - pnd_var->fy_int) / pnd_var->m;
	}
	
	// apply acceleration boundary conditions
	for (size_t i = 0; i < mesh->ax_bc_num; i++)
	{
		pnd = mesh->nodes + mesh->ax_bcs[i].node_id;
		for (pnd_var = static_cast<NodeVar_2D_ME *>(pnd->first()); pnd_var;
			 pnd_var = static_cast<NodeVar_2D_ME *>(pnd->next(pnd_var)))
		{
			pnd_var->ax = mesh->ax_bcs[i].a;
		}
	}
	for (size_t i = 0; i < mesh->ay_bc_num; i++)
	{
		pnd = mesh->nodes + mesh->ay_bcs[i].node_id;
		for (pnd_var = static_cast<NodeVar_2D_ME *>(pnd->first()); pnd_var;
			 pnd_var = static_cast<NodeVar_2D_ME *>(pnd->next(pnd_var)))
		{
			pnd_var->ay = mesh->ay_bcs[i].a;
		}
	}

	// update nodal velocity
	for (pnd_var = node_vars->top(); pnd_var; pnd_var = node_vars->prev(pnd_var))
	{
		pnd_var->vx = pnd_var->mmx / pnd_var->m;
		pnd_var->vx += pnd_var->ax * dt;
		pnd_var->vy = pnd_var->mmy / pnd_var->m;
		pnd_var->vy += pnd_var->ay * dt;
	}
	
	// apply velocity boundary conditions
	for (size_t i = 0; i < mesh->vx_bc_num; i++)
	{
		pnd = mesh->nodes + mesh->vx_bcs[i].node_id;
		for (pnd_var = static_cast<NodeVar_2D_ME *>(pnd->first()); pnd_var;
			 pnd_var = static_cast<NodeVar_2D_ME *>(pnd->next(pnd_var)))
		{
			pnd_var->vx = mesh->vx_bcs[i].v;
			pnd_var->ax = 0.0;
		}
	}
	for (size_t i = 0; i < mesh->vy_bc_num; i++)
	{
		pnd = mesh->nodes + mesh->vy_bcs[i].node_id;
		for (pnd_var = static_cast<NodeVar_2D_ME *>(pnd->first()); pnd_var;
			 pnd_var = static_cast<NodeVar_2D_ME *>(pnd->next(pnd_var)))
		{
			pnd_var->vy = mesh->vy_bcs[i].v;
			pnd_var->ay = 0.0;
		}
	}

	return 0;
}

int Step_R2D_ME_MPM::map_to_particles_and_update_particle_variables(void)
{
	// update displacement increment
	for (NodeVar_2D_ME *pnd_var = node_vars->top(); 
		 pnd_var; pnd_var = node_vars->prev(pnd_var))
	{
		pnd_var->dux = pnd_var->vx * dt;
		pnd_var->duy = pnd_var->vy * dt;
	}

	Particle_2D_ME *ppcl;
	ParticleVar_R2D_ME *ppcl_var, *ppcl_var_end;
	NodeVar_2D_ME *pnd1_var, *pnd2_var, *pnd3_var, *pnd4_var;
	double N1_tmp, N2_tmp, N3_tmp, N4_tmp;
	double de_vol, ds11, ds22, ds12;
	double E_tmp;

	ppcl_var_end = pcl_vars->eoi();
	for (ppcl_var = pcl_vars->first(); ppcl_var != ppcl_var_end;
		 ppcl_var = pcl_vars->next(ppcl_var))
	{
		ppcl = ppcl_var->pcl;
		pnd1_var = ppcl_var->node1_var;
		pnd2_var = ppcl_var->node2_var;
		pnd3_var = ppcl_var->node3_var;
		pnd4_var = ppcl_var->node4_var;
		N1_tmp = ppcl_var->N1;
		N2_tmp = ppcl_var->N2;
		N3_tmp = ppcl_var->N3;
		N4_tmp = ppcl_var->N4;

		// velocity
		ppcl->vx += (pnd1_var->ax * N1_tmp + pnd2_var->ax * N2_tmp
				   + pnd3_var->ax * N3_tmp + pnd4_var->ax * N4_tmp) * dt;
		ppcl->vy += (pnd1_var->ay * N1_tmp + pnd2_var->ay * N2_tmp
				   + pnd3_var->ay * N3_tmp + pnd4_var->ay * N4_tmp) * dt;

		// displacement
		ppcl->ux += pnd1_var->dux * N1_tmp + pnd2_var->dux * N2_tmp
				  + pnd3_var->dux * N3_tmp + pnd4_var->dux * N4_tmp;
		ppcl->uy += pnd1_var->duy * N1_tmp + pnd2_var->duy * N2_tmp
				  + pnd3_var->duy * N3_tmp + pnd4_var->duy * N4_tmp;
		// update position
		ppcl->x = ppcl->x_ori + ppcl->ux;
		ppcl->y = ppcl->y_ori + ppcl->uy;

		// strain increment
		ppcl_var->de11 = pnd1_var->dux * ppcl_var->dN1_dx + pnd2_var->dux * ppcl_var->dN2_dx
					   + pnd3_var->dux * ppcl_var->dN3_dx + pnd4_var->dux * ppcl_var->dN4_dx;
		ppcl_var->de22 = pnd1_var->duy * ppcl_var->dN1_dy + pnd2_var->duy * ppcl_var->dN2_dy
					   + pnd3_var->duy * ppcl_var->dN3_dy + pnd4_var->duy * ppcl_var->dN4_dy;
		ppcl_var->de12 = (pnd1_var->dux * ppcl_var->dN1_dy + pnd2_var->dux * ppcl_var->dN2_dy
						+ pnd3_var->dux * ppcl_var->dN3_dy + pnd4_var->dux * ppcl_var->dN4_dy
						+ pnd1_var->duy * ppcl_var->dN1_dx + pnd2_var->duy * ppcl_var->dN2_dx
						+ pnd3_var->duy * ppcl_var->dN3_dx + pnd4_var->duy * ppcl_var->dN4_dx) * 0.5;
		ppcl_var->dw12 = (pnd1_var->dux * ppcl_var->dN1_dy + pnd2_var->dux * ppcl_var->dN2_dy
						+ pnd3_var->dux * ppcl_var->dN3_dy + pnd4_var->dux * ppcl_var->dN4_dy
						- pnd1_var->duy * ppcl_var->dN1_dx - pnd2_var->duy * ppcl_var->dN2_dx
						- pnd3_var->duy * ppcl_var->dN3_dx - pnd4_var->duy * ppcl_var->dN4_dx) * 0.5;

		/* update variables at particles */
		de_vol = ppcl_var->de11 + ppcl_var->de22;
		ppcl->density /= (1.0 + de_vol);

		// update strain (also assume that strain increment is Jaumann rate)
		//ppcl->de11 +=  ppcl->dw12 * ppcl->e12 * 2.0;
		//ppcl->de22 += -ppcl->dw12 * ppcl->e12 * 2.0;
		//ppcl->de12 +=  ppcl->dw12 * (ppcl->e22 - ppcl->e11);
		ppcl->e11 += ppcl_var->de11;
		ppcl->e22 += ppcl_var->de22;
		ppcl->e12 += ppcl_var->de12;

		// update stress (Constitutive model)
		E_tmp = ppcl->E / (1.0 + ppcl->niu) / (1.0 - 2.0 * ppcl->niu);
		ds11 = E_tmp * ((1.0 - ppcl->niu) * ppcl_var->de11 + ppcl->niu * ppcl_var->de22);
		ds12 = 2.0 * ppcl_var->de12 * ppcl->E / (2.0 * (1.0 + ppcl->niu));
		ds22 = E_tmp * (ppcl->niu * ppcl_var->de11 + (1.0 - ppcl->niu) * ppcl_var->de22);

		/* ------------------------------------------------------------------
		Rotate as Jaumann rate:
		tensor_rate = tensor_Jaumann_rate + tensor * dW_T + dW * tensor
		------------------------------------------------------------------- */
		/*			ds11 +=  ppcl->dw12 * ppcl->s12 * 2.0;
		ds22 += -ppcl->dw12 * ppcl->s12 * 2.0;
		ds12 +=  ppcl->dw12 * (ppcl->s22 - ppcl->s11);	*/
		ppcl->s11 += ds11;
		ppcl->s22 += ds22;
		ppcl->s12 += ds12;
	}
	
	return 0;
}

int Step_R2D_ME_MPM::contact_calculation(void)
{
	Node_BG_R2D_ME *pnd;
	Element_BG_R2D_ME *pelem1, *pelem2, *pelem3, *pelem4;
	Object_Particle_2D_ME *pobj;
	Particle_2D_ME *ppcl1, *ppcl2, *ppcl3, *ppcl4;
	ParticleVar_R2D_ME *ppcl1_var, *ppcl2_var, *ppcl3_var, *ppcl4_var;
	NodeVar_2D_ME *pnd_var;
	Node_Contact_Var_2D_ME *con_var;
	double cm, cmmx, cmmy;
	double n_norm;

	for (size_t i = 0; i < mesh->node_num; i++)
	{
		pnd = mesh->nodes + i;
		if (pnd->object_num > 1) // detect possible contact
		{
			cm = 0.0;
			cmmx = 0.0;
			cmmy = 0.0;
			pnd->init_contact();
			for (pnd_var = pnd->first(); pnd_var; pnd_var = pnd->next(pnd_var))
			{
				// calculate velocity of "mass centre"
				cm += pnd_var->m;
				cmmx += pnd_var->vx * pnd_var->m;
				cmmy += pnd_var->vy * pnd_var->m;
				// alloc contact variables
				con_var = contact_vars->alloc();
				pnd->add_contact_var(con_var);
			}
			pnd->vx_cm = cmmx / cm;
			pnd->vy_cm = cmmy / cm;

			// calculate normal vector
			mesh->get_elements_by_node(pnd, pelem1, pelem2, pelem3, pelem4);
			ppcl1_var = pelem1->first_pcl_var();
			ppcl2_var = pelem2->first_pcl_var();
			ppcl3_var = pelem3->first_pcl_var();
			ppcl4_var = pelem4->first_pcl_var();
			for (pnd_var = pnd->first(), con_var = pnd->first_contact_var(); pnd_var;
				 pnd_var = pnd->next(pnd_var), con_var = pnd->next_contact_var(con_var))
			{
				pobj = pnd_var->object;
				con_var->nx = 0.0;
				con_var->ny = 0.0;
				// element1
				while (ppcl1_var->pcl->object == pobj)
				{
					con_var->nx += ppcl1_var->dN1_dx * ppcl1_var->vol;
					con_var->ny += ppcl1_var->dN1_dy * ppcl1_var->vol;
					ppcl1_var = pelem1->next_pcl_var(ppcl1_var);
				}
				// element2
				while (ppcl2_var->pcl->object == pobj)
				{
					con_var->nx += ppcl2_var->dN2_dx * ppcl2_var->vol;
					con_var->ny += ppcl2_var->dN2_dy * ppcl2_var->vol;
					ppcl2_var = pelem2->next_pcl_var(ppcl2_var);
				}
				// element3
				while (ppcl3_var->pcl->object == pobj)
				{
					con_var->nx += ppcl3_var->dN3_dx * ppcl3_var->vol;
					con_var->ny += ppcl3_var->dN3_dy * ppcl3_var->vol;
					ppcl3_var = pelem3->next_pcl_var(ppcl3_var);
				}
				// element4
				while (ppcl4_var->pcl->object == pobj)
				{
					con_var->nx += ppcl4_var->dN4_dx * ppcl4_var->vol;
					con_var->ny += ppcl4_var->dN4_dy * ppcl4_var->vol;
					ppcl4_var = pelem4->next_pcl_var(ppcl4_var);
				}
				n_norm = sqrt(con_var->nx * con_var->nx + con_var->ny * con_var->ny);
				con_var->nx /= n_norm;
				con_var->ny /= n_norm;
			}

			if (pnd->object_num == 2)
			{
				contact_between_two_objects(pnd);
			}
			else
			{
				/*----------------------------------------------
				 * Note:
				 *   Handle contact between more than 2 objects.
				 *   But this case is not accurate. Try to aoid 
				 *   this case with smaller mesh size.
				 *---------------------------------------------*/
				contact_between_multiple_objects(pnd);
			}
		}
	}
	
	// Reapply acceleration boundary conditions
	for (size_t i = 0; i < mesh->ax_bc_num; i++)
	{
		pnd = mesh->nodes + mesh->ax_bcs[i].node_id;
		if (pnd->is_in_contact)
		{
			for (pnd_var = static_cast<NodeVar_2D_ME *>(pnd->first()); pnd_var;
				 pnd_var = static_cast<NodeVar_2D_ME *>(pnd->next(pnd_var)))
			{
				 pnd_var->ax = mesh->ax_bcs[i].a;
			}
		}
	}
	for (size_t i = 0; i < mesh->ay_bc_num; i++)
	{
		pnd = mesh->nodes + mesh->ay_bcs[i].node_id;
		if (pnd->is_in_contact)
		{
			for (pnd_var = static_cast<NodeVar_2D_ME *>(pnd->first()); pnd_var;
				 pnd_var = static_cast<NodeVar_2D_ME *>(pnd->next(pnd_var)))
			{
				pnd_var->ay = mesh->ay_bcs[i].a;
			}
		}
	}
	// Reapply velocity boundary conditions
	for (size_t i = 0; i < mesh->vx_bc_num; i++)
	{
		pnd = mesh->nodes + mesh->vx_bcs[i].node_id;
		if (pnd->is_in_contact)
		{
			for (pnd_var = static_cast<NodeVar_2D_ME *>(pnd->first()); pnd_var;
				pnd_var = static_cast<NodeVar_2D_ME *>(pnd->next(pnd_var)))
			{
				pnd_var->vx = mesh->vx_bcs[i].v;
				pnd_var->ax = 0.0;
			}
		}
	}
	for (size_t i = 0; i < mesh->vy_bc_num; i++)
	{
		pnd = mesh->nodes + mesh->vy_bcs[i].node_id;
		if (pnd->is_in_contact)
		{
			for (pnd_var = static_cast<NodeVar_2D_ME *>(pnd->first()); pnd_var;
				pnd_var = static_cast<NodeVar_2D_ME *>(pnd->next(pnd_var)))
			{
				pnd_var->vy = mesh->vy_bcs[i].v;
				pnd_var->ay = 0.0;
			}
		}
	}

	return 0;
}


void Step_R2D_ME_MPM::contact_between_two_objects(Node_BG_R2D_ME *pnd)
{
	NodeVar_2D_ME *pnv_obj1, *pnv_obj2;
	Node_Contact_Var_2D_ME *pncv_obj1, *pncv_obj2;
	double n_norm;

	// Decide which nodes are in contact
	pnv_obj1 = pnd->first();
	pnv_obj2 = pnd->next(pnv_obj1);
	pncv_obj1 = pnd->first_contact_var();
	pncv_obj2 = pnd->next_contact_var(pncv_obj1);
	
	// get the "average" normal vector of two objects
	// (n1 - n2) / |n1 - n2|
	pncv_obj1->nx -= pncv_obj2->nx;
	pncv_obj1->ny -= pncv_obj2->ny;
	n_norm = sqrt(pncv_obj1->nx * pncv_obj1->nx + pncv_obj1->ny * pncv_obj1->ny);
	pncv_obj1->nx /= n_norm;
	pncv_obj1->ny /= n_norm;
	pncv_obj2->nx = -pncv_obj1->nx;
	pncv_obj2->ny = -pncv_obj1->ny;

	pncv_obj1->vrx = pnv_obj1->vx - pnd->vx_cm;
	pncv_obj1->vry = pnv_obj2->vy - pnd->vy_cm;
	pncv_obj1->vrn = pncv_obj1->vrx * pncv_obj1->nx + pncv_obj1->vry * pncv_obj1->ny;
	// Check if two objects are in contact
	if (pncv_obj1->vrn > 0.0)
	{

	}
	// normal contact force

	// tangential contact force


	// adjust a += f / m * dt
	// re-apply boundary condtions
	// adjust v += a * dt
	// re-apply boundary condition
}

void Step_R2D_ME_MPM::contact_between_multiple_objects(Node_BG_R2D_ME *pnd)
{
	// Hmmmmk ..., I don't wanna code this .... :(


}