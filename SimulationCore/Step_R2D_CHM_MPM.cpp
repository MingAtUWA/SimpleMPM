#include "SimulationCore_pcp.h"

#include "Step_R2D_CHM_MPM.h"

Step_R2D_CHM_MPM::Step_R2D_CHM_MPM() :
	Step(&solve_substep_R2D_CHM_MPM),
	model(nullptr) {}

Step_R2D_CHM_MPM::~Step_R2D_CHM_MPM() {}

int Step_R2D_CHM_MPM::init()
{
	Object_Particle_2D_CHM *pobj, *pobj_end;
	Particle_2D_CHM *ppcl;
	ParticleVar_R2D_CHM *ppcl_var;

	mesh = &(model->mesh);
	objects = &(model->objects);
	node_vars = &(model->node_vars);
	pcl_vars = &(model->pcl_vars);

	// initialize buffer for particle & nodal calculation variables
	size_t total_pcl_num = 0;
	pobj = objects->first();
	pobj_end = objects->eoi();
	while (pobj != pobj_end)
	{
		total_pcl_num += pobj->pcl_num;
		pobj = objects->next(pobj);
	}
	node_vars->set_default_pool_item_num(total_pcl_num);
	pcl_vars->set_default_pool_item_num(total_pcl_num);

	if (is_first_step)
	{
		pobj = objects->first();
		pobj_end = objects->eoi();
		while (pobj != pobj_end)
		{
			for (size_t i = 0; i < pobj->pcl_num; i++)
			{
				ppcl = pobj->pcls + i;
				ppcl_var = pcl_vars->alloc();
				// init
				ppcl->var = ppcl_var;
				ppcl_var->pcl = ppcl;
				ppcl_var->object = pobj;
				ppcl_var->elem = nullptr;
			}
			pobj = objects->next(pobj);
		}
	}

	pobj = objects->first();
	pobj_end = objects->eoi();
	while (pobj != pobj_end)
	{
		for (size_t i = 0; i < pobj->pcl_num; i++)
		{
			ppcl = pobj->pcls + i;
			ppcl->x_ori = ppcl->x;
			ppcl->y_ori = ppcl->y;
			ppcl->ux_s = 0.0;
			ppcl->uy_s = 0.0;
			ppcl->ux_f = 0.0;
			ppcl->uy_f = 0.0;
		}
		pobj = objects->next(pobj);
	}

	return 0;
}

int Step_R2D_CHM_MPM::finalize() { return 0; }

int solve_substep_R2D_CHM_MPM(void *_self)
{
	Step_R2D_CHM_MPM *self = (Step_R2D_CHM_MPM *)_self;
	
	self->init_per_substep();
	self->map_to_nodes_and_cal_internal_force();
	self->cal_external_force();
	self->cal_fluid_phase();
	self->cal_mixture_phase();
	self->contact_calculation();
	self->map_to_particles_and_update_particle_variables();
	
	return 0;
}

inline void init_node_variables(NodeVar_2D_CHM *pnode_var)
{
	// for mapping from nodes
	pnode_var->m_s = 0.0;
	pnode_var->mmx_s = 0.0;
	pnode_var->mmy_s = 0.0;
	pnode_var->fx_kin_f = 0.0;
	pnode_var->fy_kin_f = 0.0;
	pnode_var->fx_ext_m = 0.0;
	pnode_var->fy_ext_m = 0.0;
	pnode_var->fx_int_m = 0.0;
	pnode_var->fy_int_m = 0.0;
	// for mapping back to particles
	pnode_var->ax_s = 0.0;
	pnode_var->ay_s = 0.0;
	pnode_var->vx_s = 0.0;
	pnode_var->vy_s = 0.0;
	pnode_var->dux_s = 0.0;
	pnode_var->duy_s = 0.0;
	// for mapping from nodes
	pnode_var->m_tf = 0.0;
	pnode_var->mmx_tf = 0.0;
	pnode_var->mmy_tf = 0.0;
	pnode_var->fx_ext_tf = 0.0;
	pnode_var->fy_ext_tf = 0.0;
	pnode_var->fx_int_tf = 0.0;
	pnode_var->fy_int_tf = 0.0;
	pnode_var->fx_drag_tf = 0.0;
	pnode_var->fy_drag_tf = 0.0;
	// for mapping back to particles
	pnode_var->ax_f = 0.0;
	pnode_var->ay_f = 0.0;
	pnode_var->vx_f = 0.0;
	pnode_var->vy_f = 0.0;
	pnode_var->dux_f = 0.0;
	pnode_var->duy_f = 0.0;
}
int Step_R2D_CHM_MPM::init_per_substep(void)
{
	ParticleVar_R2D_CHM *ppcl_var, *ppcl_var_end;
	Particle_2D_CHM *ppcl;
	Node_BG_R2D *pnd1, *pnd2, *pnd3, *pnd4;
	NodeVar_2D_CHM *pnd_var;

	node_vars->reset();
	for (size_t i = 0; i < mesh->node_num; i++)
		mesh->nodes[i].init();

	ppcl_var = pcl_vars->first();
	ppcl_var_end = pcl_vars->eoi();
	while (ppcl_var != ppcl_var_end)
	{
		ppcl = ppcl_var->pcl;
		ppcl_var->elem = mesh->find_in_which_element(ppcl->x, ppcl->y, ppcl_var->elem);
		if (ppcl_var->elem) // particle is in background mesh
		{
			// init variables on particles
			ppcl_var->k_div_miu = ppcl->k / ppcl->miu;

			// cal shape functions and their derivatives
			model->cal_shape_function(ppcl_var);

			// init the four nodes associated with this particle
			Get_Nodes_Of_Element_R2D(ppcl_var->elem, mesh, pnd1, pnd2, pnd3, pnd4);
			// node 1
			pnd_var = static_cast<NodeVar_2D_CHM *>(pnd1->var);
			if (!pnd1->var || pnd_var->object != ppcl_var->object)
			{
				pnd_var = node_vars->alloc();
				// add to stack at node
				pnd1->add(pnd_var);
				pnd_var->node = pnd1;
				pnd_var->object = ppcl_var->object;
				// init node variable
				init_node_variables(pnd_var);
			}
			ppcl_var->node1_var = pnd_var;
			// node 2
			pnd_var = static_cast<NodeVar_2D_CHM *>(pnd2->var);
			if (!pnd2->var || pnd_var->object != ppcl_var->object)
			{
				pnd_var = node_vars->alloc();
				// add to stack at node
				pnd2->add(pnd_var);
				pnd_var->node = pnd2;
				pnd_var->object = ppcl_var->object;
				// init node variable
				init_node_variables(pnd_var);
			}
			ppcl_var->node2_var = pnd_var;
			// node 3
			pnd_var = static_cast<NodeVar_2D_CHM *>(pnd3->var);
			if (!pnd3->var || pnd_var->object != ppcl_var->object)
			{
				pnd_var = node_vars->alloc();
				// add to stack at node
				pnd3->add(pnd_var);
				pnd_var->node = pnd3;
				pnd_var->object = ppcl_var->object;
				// init node variable
				init_node_variables(pnd_var);
			}
			ppcl_var->node3_var = pnd_var;
			// node 4
			pnd_var = static_cast<NodeVar_2D_CHM *>(pnd4->var);
			if (!pnd4->var || pnd_var->object != ppcl_var->object)
			{
				pnd_var = node_vars->alloc();
				// add to stack at node
				pnd4->add(pnd_var);
				pnd_var->node = pnd4;
				pnd_var->object = ppcl_var->object;
				// init node variable
				init_node_variables(pnd_var);
			}
			ppcl_var->node4_var = pnd_var;
		}
		else // particle is out of background mesh
		{
			ppcl->var = nullptr;
			pcl_vars->del(ppcl_var);
		}
		ppcl_var = pcl_vars->next(ppcl_var);
	}
	return 0;
}


int Step_R2D_CHM_MPM::map_to_nodes_and_cal_internal_force(void)
{
	Particle_2D_CHM *ppcl;
	ParticleVar_R2D_CHM *ppcl_var, *ppcl_var_end;
	NodeVar_2D_CHM *pnd_var;

	ppcl_var = pcl_vars->first();
	ppcl_var_end = pcl_vars->eoi();
	while (ppcl_var != ppcl_var_end)
	{
		ppcl = ppcl_var->pcl;

		// node 1
		pnd_var = ppcl_var->node1_var;
		// mixture phase
		pnd_var->m_s   += ppcl_var->N1 * (1.0 - ppcl->n) * ppcl->density_s * ppcl->vol;
		pnd_var->mmx_s += ppcl_var->N1 * (1.0 - ppcl->n) * ppcl->density_s * ppcl->vol * ppcl->vx_s;
		pnd_var->mmy_s += ppcl_var->N1 * (1.0 - ppcl->n) * ppcl->density_s * ppcl->vol * ppcl->vy_s;
		pnd_var->fx_int_m += (ppcl_var->dN1_dx * (ppcl->s11 - ppcl->p) + ppcl_var->dN1_dy * ppcl->s12) * ppcl->vol;
		pnd_var->fy_int_m += (ppcl_var->dN1_dx * ppcl->s12 + ppcl_var->dN1_dy * (ppcl->s22 - ppcl->p)) * ppcl->vol;
		// fluid phase
		pnd_var->m_tf   += ppcl_var->N1 * ppcl->density_f * ppcl->vol;
		pnd_var->mmx_tf += ppcl_var->N1 * ppcl->density_f * ppcl->vol * ppcl->vx_f;
		pnd_var->mmy_tf += ppcl_var->N1 * ppcl->density_f * ppcl->vol * ppcl->vy_f;
		pnd_var->fx_int_tf += (ppcl_var->dN1_dx * -ppcl->p) * ppcl->vol;
		pnd_var->fy_int_tf += (ppcl_var->dN1_dy * -ppcl->p) * ppcl->vol;
		pnd_var->fx_drag_tf += ppcl_var->N1 * ppcl->n / ppcl_var->k_div_miu * (ppcl->vx_f - ppcl->vx_s) * ppcl->vol;
		pnd_var->fy_drag_tf += ppcl_var->N1 * ppcl->n / ppcl_var->k_div_miu * (ppcl->vy_f - ppcl->vy_s) * ppcl->vol;
		
		// node 2
		pnd_var = ppcl_var->node2_var;
		// mixture phase
		pnd_var->m_s   += ppcl_var->N2 * (1.0 - ppcl->n) * ppcl->density_s * ppcl->vol;
		pnd_var->mmx_s += ppcl_var->N2 * (1.0 - ppcl->n) * ppcl->density_s * ppcl->vol * ppcl->vx_s;
		pnd_var->mmy_s += ppcl_var->N2 * (1.0 - ppcl->n) * ppcl->density_s * ppcl->vol * ppcl->vy_s;
		pnd_var->fx_int_m += (ppcl_var->dN2_dx * (ppcl->s11 - ppcl->p) + ppcl_var->dN2_dy * ppcl->s12) * ppcl->vol;
		pnd_var->fy_int_m += (ppcl_var->dN2_dx * ppcl->s12 + ppcl_var->dN2_dy * (ppcl->s22 - ppcl->p)) * ppcl->vol;
		// fluid phase
		pnd_var->m_tf   += ppcl_var->N2 * ppcl->density_f * ppcl->vol;
		pnd_var->mmx_tf += ppcl_var->N2 * ppcl->density_f * ppcl->vol * ppcl->vx_f;
		pnd_var->mmy_tf += ppcl_var->N2 * ppcl->density_f * ppcl->vol * ppcl->vy_f;
		pnd_var->fx_int_tf += (ppcl_var->dN2_dx * -ppcl->p) * ppcl->vol;
		pnd_var->fy_int_tf += (ppcl_var->dN2_dy * -ppcl->p) * ppcl->vol;
		pnd_var->fx_drag_tf += ppcl_var->N2 * ppcl->n / ppcl_var->k_div_miu * (ppcl->vx_f - ppcl->vx_s) * ppcl->vol;
		pnd_var->fy_drag_tf += ppcl_var->N2 * ppcl->n / ppcl_var->k_div_miu * (ppcl->vy_f - ppcl->vy_s) * ppcl->vol;

		// node 3
		pnd_var = ppcl_var->node3_var;
		// mixture phase
		pnd_var->m_s   += ppcl_var->N3 * (1.0 - ppcl->n) * ppcl->density_s * ppcl->vol;
		pnd_var->mmx_s += ppcl_var->N3 * (1.0 - ppcl->n) * ppcl->density_s * ppcl->vol * ppcl->vx_s;
		pnd_var->mmy_s += ppcl_var->N3 * (1.0 - ppcl->n) * ppcl->density_s * ppcl->vol * ppcl->vy_s;
		pnd_var->fx_int_m += (ppcl_var->dN3_dx * (ppcl->s11 - ppcl->p) + ppcl_var->dN3_dy * ppcl->s12) * ppcl->vol;
		pnd_var->fy_int_m += (ppcl_var->dN3_dx * ppcl->s12 + ppcl_var->dN3_dy * (ppcl->s22 - ppcl->p)) * ppcl->vol;
		// fluid phase
		pnd_var->m_tf   += ppcl_var->N3 * ppcl->density_f * ppcl->vol;
		pnd_var->mmx_tf += ppcl_var->N3 * ppcl->density_f * ppcl->vol * ppcl->vx_f;
		pnd_var->mmy_tf += ppcl_var->N3 * ppcl->density_f * ppcl->vol * ppcl->vy_f;
		pnd_var->fx_int_tf += (ppcl_var->dN3_dx * -ppcl->p) * ppcl->vol;
		pnd_var->fy_int_tf += (ppcl_var->dN3_dy * -ppcl->p) * ppcl->vol;
		pnd_var->fx_drag_tf += ppcl_var->N3 * ppcl->n / ppcl_var->k_div_miu * (ppcl->vx_f - ppcl->vx_s) * ppcl->vol;
		pnd_var->fy_drag_tf += ppcl_var->N3 * ppcl->n / ppcl_var->k_div_miu * (ppcl->vy_f - ppcl->vy_s) * ppcl->vol;

		// node 4
		pnd_var = ppcl_var->node4_var;
		// mixture phase
		pnd_var->m_s   += ppcl_var->N4 * (1.0 - ppcl->n) * ppcl->density_s * ppcl->vol;
		pnd_var->mmx_s += ppcl_var->N4 * (1.0 - ppcl->n) * ppcl->density_s * ppcl->vol * ppcl->vx_s;
		pnd_var->mmy_s += ppcl_var->N4 * (1.0 - ppcl->n) * ppcl->density_s * ppcl->vol * ppcl->vy_s;
		pnd_var->fx_int_m += (ppcl_var->dN4_dx * (ppcl->s11 - ppcl->p) + ppcl_var->dN4_dy * ppcl->s12) * ppcl->vol;
		pnd_var->fy_int_m += (ppcl_var->dN4_dx * ppcl->s12 + ppcl_var->dN4_dy * (ppcl->s22 - ppcl->p)) * ppcl->vol;
		// fluid phase
		pnd_var->m_tf   += ppcl_var->N4 * ppcl->density_f * ppcl->vol;
		pnd_var->mmx_tf += ppcl_var->N4 * ppcl->density_f * ppcl->vol * ppcl->vx_f;
		pnd_var->mmy_tf += ppcl_var->N4 * ppcl->density_f * ppcl->vol * ppcl->vy_f;
		pnd_var->fx_int_tf += (ppcl_var->dN4_dx * -ppcl->p) * ppcl->vol;
		pnd_var->fy_int_tf += (ppcl_var->dN4_dy * -ppcl->p) * ppcl->vol;
		pnd_var->fx_drag_tf += ppcl_var->N4 * ppcl->n / ppcl_var->k_div_miu * (ppcl->vx_f - ppcl->vx_s) * ppcl->vol;
		pnd_var->fy_drag_tf += ppcl_var->N4 * ppcl->n / ppcl_var->k_div_miu * (ppcl->vy_f - ppcl->vy_s) * ppcl->vol;

		ppcl_var = pcl_vars->next(ppcl_var);
	}
	return 0;
}

int Step_R2D_CHM_MPM::cal_external_force(void)
{
	Object_Particle_2D_CHM *pobj, *pobj_end;
	Particle_2D_CHM *ppcl;
	ParticleVar_R2D_CHM *ppcl_var;
	NodeVar_2D_CHM *pnd_var;
	double bf_m, bf_tf;

	pobj = objects->first();
	pobj_end = objects->eoi();
	while (pobj != pobj_end)
	{
		// body force
		for (size_t i = 0; i < pobj->bfx_num; i++)
		{
			ppcl = pobj->pcls + pobj->bfxs[i].pcl_id;
			if (ppcl->var) // is in mesh
			{
				ppcl_var = static_cast<ParticleVar_R2D_CHM *>(ppcl->var);
				// body force on particle
				bf_m = ppcl->vol * ((1.0 - ppcl->n) * ppcl->density_s + ppcl->n * ppcl->density_f) * pobj->bfxs[i].bf;
				bf_tf = ppcl->vol * ppcl->density_f * pobj->bfxs[i].bf;
				// node 1
				pnd_var = ppcl_var->node1_var;
				pnd_var->fx_ext_m += ppcl_var->N1 * bf_m;
				pnd_var->fx_ext_tf += ppcl_var->N1 * bf_tf;
				// node 2
				pnd_var = ppcl_var->node2_var;
				pnd_var->fx_ext_m += ppcl_var->N2 * bf_m;
				pnd_var->fx_ext_tf += ppcl_var->N2 * bf_tf;
				// node 3
				pnd_var = ppcl_var->node3_var;
				pnd_var->fx_ext_m += ppcl_var->N3 * bf_m;
				pnd_var->fx_ext_tf += ppcl_var->N3 * bf_tf;
				// node 4
				pnd_var = ppcl_var->node4_var;
				pnd_var->fx_ext_m += ppcl_var->N4 * bf_m;
				pnd_var->fx_ext_tf += ppcl_var->N4 * bf_tf;
			}
		}
		for (size_t i = 0; i < pobj->bfy_num; i++)
		{
			ppcl = pobj->pcls + pobj->bfys[i].pcl_id;
			if (ppcl->var) // is in mesh
			{
				ppcl_var = static_cast<ParticleVar_R2D_CHM *>(ppcl->var);
				// body force on particle
				bf_m = ppcl->vol * ((1.0 - ppcl->n) * ppcl->density_s + ppcl->n * ppcl->density_f) * pobj->bfys[i].bf;
				bf_tf = ppcl->vol * ppcl->density_f * pobj->bfys[i].bf;
				// node 1
				pnd_var = ppcl_var->node1_var;
				pnd_var->fy_ext_m += ppcl_var->N1 * bf_m;
				pnd_var->fy_ext_tf += ppcl_var->N1 * bf_tf;
				// node 2
				pnd_var = ppcl_var->node2_var;
				pnd_var->fy_ext_m += ppcl_var->N2 * bf_m;
				pnd_var->fy_ext_tf += ppcl_var->N2 * bf_tf;
				// node 3
				pnd_var = ppcl_var->node3_var;
				pnd_var->fy_ext_m += ppcl_var->N3 * bf_m;
				pnd_var->fy_ext_tf += ppcl_var->N3 * bf_tf;
				// node 4
				pnd_var = ppcl_var->node4_var;
				pnd_var->fy_ext_m += ppcl_var->N4 * bf_m;
				pnd_var->fy_ext_tf += ppcl_var->N4 * bf_tf;
			}
		}
		// surface force
		for (size_t i = 0; i < pobj->tx_bc_num; i++)
		{
			ppcl = pobj->pcls + pobj->tx_bcs[i].pcl_id;
			if (ppcl->var) // is in mesh
			{
				ppcl_var = static_cast<ParticleVar_R2D_CHM *>(ppcl->var);
				// node 1
				pnd_var = ppcl_var->node1_var;
				pnd_var->fx_ext_m += ppcl_var->N1 * pobj->tx_bcs[i].t;
				// node 2
				pnd_var = ppcl_var->node2_var;
				pnd_var->fx_ext_m += ppcl_var->N2 * pobj->tx_bcs[i].t;
				// node 3
				pnd_var = ppcl_var->node3_var;
				pnd_var->fx_ext_m += ppcl_var->N3 * pobj->tx_bcs[i].t;
				// node 4
				pnd_var = ppcl_var->node4_var;
				pnd_var->fx_ext_m += ppcl_var->N4 * pobj->tx_bcs[i].t;
			}
		}
		for (size_t i = 0; i < pobj->ty_bc_num; i++)
		{
			ppcl = pobj->pcls + pobj->ty_bcs[i].pcl_id;
			if (ppcl->var) // is in mesh
			{
				ppcl_var = static_cast<ParticleVar_R2D_CHM *>(ppcl->var);
				// node 1
				pnd_var = ppcl_var->node1_var;
				pnd_var->fy_ext_m += ppcl_var->N1 * pobj->ty_bcs[i].t;
				// node 2
				pnd_var = ppcl_var->node2_var;
				pnd_var->fy_ext_m += ppcl_var->N2 * pobj->ty_bcs[i].t;
				// node 3
				pnd_var = ppcl_var->node3_var;
				pnd_var->fy_ext_m += ppcl_var->N3 * pobj->ty_bcs[i].t;
				// node 4
				pnd_var = ppcl_var->node4_var;
				pnd_var->fy_ext_m += ppcl_var->N4 * pobj->ty_bcs[i].t;
			}
		}
		// pore pressure force here in the future...
	}

	return 0;
}


int Step_R2D_CHM_MPM::cal_fluid_phase(void)
{
	Node_BG_R2D *pnd;
	NodeVar_2D_CHM *pnd_var;

	// calculation acceleration
	for (pnd_var = node_vars->top(); pnd_var; pnd_var = node_vars->prev(pnd_var))
	{
		pnd_var->ax_f = (pnd_var->fx_ext_tf - pnd_var->fx_int_tf - pnd_var->fx_drag_tf) / pnd_var->m_tf;
		pnd_var->ay_f = (pnd_var->fy_ext_tf - pnd_var->fy_int_tf - pnd_var->fy_drag_tf) / pnd_var->m_tf;
	}
	
	// apply acceleration boundary conditions
	for (size_t i = 0; i < model->ax_f_bc_num; i++)
	{
		pnd = mesh->nodes + model->ax_f_bcs[i].node_id;
		for (pnd_var = static_cast<NodeVar_2D_CHM *>(pnd->first()); pnd_var;
			 pnd_var = static_cast<NodeVar_2D_CHM *>(pnd->next(pnd_var)))
		{
			pnd_var->ax_f = model->ax_f_bcs[i].a;
		}
	}
	for (size_t i = 0; i < model->ay_f_bc_num; i++)
	{
		pnd = mesh->nodes + model->ay_f_bcs[i].node_id;
		for (pnd_var = static_cast<NodeVar_2D_CHM *>(pnd->first()); pnd_var;
			pnd_var = static_cast<NodeVar_2D_CHM *>(pnd->next(pnd_var)))
		{
			pnd_var->ay_f = model->ay_f_bcs[i].a;
		}
	}

	// update nodal momentum of fluid phase
	for (pnd_var = node_vars->top(); pnd_var; pnd_var = node_vars->prev(pnd_var))
	{
		pnd_var->vx_f = pnd_var->mmx_tf / pnd_var->m_tf;
		pnd_var->vx_f += pnd_var->ax_f * dt;
		pnd_var->vy_f = pnd_var->mmy_tf / pnd_var->m_tf;
		pnd_var->vy_f += pnd_var->ay_f * dt;
	}
	
	// apply velocity boundary conditions
	for (size_t i = 0; i < model->vx_f_bc_num; i++)
	{
		pnd = mesh->nodes + model->vx_f_bcs[i].node_id;
		for (pnd_var = static_cast<NodeVar_2D_CHM *>(pnd->first()); pnd_var;
			pnd_var = static_cast<NodeVar_2D_CHM *>(pnd->next(pnd_var)))
		{
			pnd_var->vx_f = model->vx_f_bcs[i].v;
			pnd_var->ax_f = 0.0;
		}
	}
	for (size_t i = 0; i < model->vy_f_bc_num; i++)
	{
		pnd = mesh->nodes + model->vy_f_bcs[i].node_id;
		for (pnd_var = static_cast<NodeVar_2D_CHM *>(pnd->first()); pnd_var;
			pnd_var = static_cast<NodeVar_2D_CHM *>(pnd->next(pnd_var)))
		{
			pnd_var->vy_f = model->vy_f_bcs[i].v;
			pnd_var->ay_f = 0.0;
		}
	}

	return 0;
}


int Step_R2D_CHM_MPM::cal_mixture_phase(void)
{
	Particle_2D_CHM *ppcl;
	ParticleVar_R2D_CHM *ppcl_var, *ppcl_var_end;
	NodeVar_2D_CHM *pnd1_var, *pnd2_var, *pnd3_var, *pnd4_var;
	Node_BG_R2D *pnd;
	NodeVar_2D_CHM *pnd_var;
	double pcl_ax_f, pcl_ay_f;
	double pcl_max_f, pcl_may_f;

	// calculate the inertial term of fluid in mixture formulation
	ppcl_var = pcl_vars->first();
	ppcl_var_end = pcl_vars->eoi();
	while (ppcl_var != ppcl_var_end)
	{
		ppcl = ppcl_var->pcl;
		pnd1_var = ppcl_var->node1_var;
		pnd2_var = ppcl_var->node2_var;
		pnd3_var = ppcl_var->node3_var;
		pnd4_var = ppcl_var->node4_var;

		// particle acceleration
		pcl_ax_f = ppcl_var->N1 * pnd1_var->ax_f + ppcl_var->N2 * pnd2_var->ax_f
			+ ppcl_var->N3 * pnd3_var->ax_f + ppcl_var->N4 * pnd4_var->ax_f;
		pcl_ay_f = ppcl_var->N1 * pnd1_var->ay_f + ppcl_var->N2 * pnd2_var->ay_f
			+ ppcl_var->N3 * pnd3_var->ay_f + ppcl_var->N4 * pnd4_var->ay_f;
		pcl_max_f = ppcl->n * ppcl->density_f * ppcl->vol * pcl_ax_f;
		pcl_may_f = ppcl->n * ppcl->density_f * ppcl->vol * pcl_ay_f;
		// node 1
		pnd1_var->fx_kin_f += ppcl_var->N1 * pcl_max_f;
		pnd1_var->fy_kin_f += ppcl_var->N1 * pcl_may_f;
		// node 2
		pnd2_var->fx_kin_f += ppcl_var->N2 * pcl_max_f;
		pnd2_var->fy_kin_f += ppcl_var->N2 * pcl_may_f;
		// node 3
		pnd3_var->fx_kin_f += ppcl_var->N3 * pcl_max_f;
		pnd3_var->fy_kin_f += ppcl_var->N3 * pcl_may_f;
		// node 4
		pnd4_var->fx_kin_f += ppcl_var->N4 * pcl_max_f;
		pnd4_var->fy_kin_f += ppcl_var->N4 * pcl_may_f;

		ppcl_var = pcl_vars->next(ppcl_var);
	}

	// update nodal velocity of solid phase
	for (pnd_var = node_vars->top(); pnd_var;
		pnd_var = node_vars->prev(pnd_var))
	{
		pnd_var->ax_s = (pnd_var->fx_ext_m - pnd_var->fx_int_m - pnd_var->fx_kin_f) / pnd_var->m_s;
		pnd_var->ay_s = (pnd_var->fy_ext_m - pnd_var->fy_int_m - pnd_var->fy_kin_f) / pnd_var->m_s;
	}

	// apply acceleration boundary conditions
	for (size_t i = 0; i < model->ax_s_bc_num; i++)
	{
		pnd = mesh->nodes + model->ax_s_bcs[i].node_id;
		for (pnd_var = static_cast<NodeVar_2D_CHM *>(pnd->first()); pnd_var;
			pnd_var = static_cast<NodeVar_2D_CHM *>(pnd->next(pnd_var)))
		{
			pnd_var->ax_s = model->ax_s_bcs[i].a;
		}
	}
	for (size_t i = 0; i < model->ay_s_bc_num; i++)
	{
		pnd = mesh->nodes + model->ay_s_bcs[i].node_id;
		for (pnd_var = static_cast<NodeVar_2D_CHM *>(pnd->first()); pnd_var;
			pnd_var = static_cast<NodeVar_2D_CHM *>(pnd->next(pnd_var)))
		{
			pnd_var->ay_s = model->ay_s_bcs[i].a;
		}
	}

	// update nodal momentum of fluid pahse
	for (pnd_var = node_vars->top(); pnd_var;
		pnd_var = node_vars->prev(pnd_var))
	{
		pnd_var->vx_s  = pnd_var->mmx_s / pnd_var->m_s;
		pnd_var->vx_s += pnd_var->ax_s * dt;
		pnd_var->vy_s  = pnd_var->mmy_s / pnd_var->m_s;
		pnd_var->vy_s += pnd_var->ay_s * dt;
	}

	// apply velocity boundary conditions of solid phase
	for (size_t i = 0; i < model->vx_s_bc_num; i++)
	{
		pnd = mesh->nodes + model->vx_s_bcs[i].node_id;
		for (pnd_var = static_cast<NodeVar_2D_CHM *>(pnd->first()); pnd_var;
			 pnd_var = static_cast<NodeVar_2D_CHM *>(pnd->next(pnd_var)))
		{
			pnd_var->vx_s = model->vx_s_bcs[i].v;
			pnd_var->ax_s = 0.0;
		}
	}
	for (size_t i = 0; i < model->vy_s_bc_num; i++)
	{
		pnd = mesh->nodes + model->vy_s_bcs[i].node_id;
		for (pnd_var = static_cast<NodeVar_2D_CHM *>(pnd->first()); pnd_var;
			 pnd_var = static_cast<NodeVar_2D_CHM *>(pnd->next(pnd_var)))
		{
			pnd_var->vy_s = model->vy_s_bcs[i].v;
			pnd_var->ay_s = 0.0;
		}
	}
	
	// update displacement increment of both phases
	for (pnd_var = node_vars->top(); pnd_var;
		 pnd_var = node_vars->prev(pnd_var))
	{
		// solid phase
		pnd_var->dux_s = pnd_var->vx_s * dt;
		pnd_var->duy_s = pnd_var->vy_s * dt;
		// fluid phase
		pnd_var->dux_f = pnd_var->vx_f * dt;
		pnd_var->duy_f = pnd_var->vy_f * dt;
	}

	return 0;
}


int Step_R2D_CHM_MPM::contact_calculation(void)
{
	
	return 0;
}

int Step_R2D_CHM_MPM::map_to_particles_and_update_particle_variables(void)
{
	Particle_2D_CHM *ppcl;
	ParticleVar_R2D_CHM *ppcl_var, *ppcl_var_end;
	NodeVar_2D_CHM *pnd1_var, *pnd2_var, *pnd3_var, *pnd4_var;
	double N1_tmp, N2_tmp, N3_tmp, N4_tmp;
	double E_tmp;
	double de11_s, de22_s, de12_s, dw12;
	double ds11, ds22, ds12;
	double de_vol_s, de_vol_f;

	ppcl_var = pcl_vars->first();
	ppcl_var_end = pcl_vars->eoi();
	while (ppcl_var != ppcl_var_end)
	{
		pnd1_var = ppcl_var->node1_var;
		pnd2_var = ppcl_var->node2_var;
		pnd3_var = ppcl_var->node3_var;
		pnd4_var = ppcl_var->node4_var;
		N1_tmp = ppcl_var->N1;
		N2_tmp = ppcl_var->N2;
		N3_tmp = ppcl_var->N3;
		N4_tmp = ppcl_var->N4;

		// velocity
		ppcl->vx_s += (pnd1_var->ax_s * N1_tmp + pnd2_var->ax_s * N2_tmp
					 + pnd3_var->ax_s * N3_tmp + pnd4_var->ax_s * N4_tmp) * dt;
		ppcl->vy_s += (pnd1_var->ay_s * N1_tmp + pnd2_var->ay_s * N2_tmp
					 + pnd3_var->ay_s * N3_tmp + pnd4_var->ay_s * N4_tmp) * dt;
		ppcl->vx_f += (pnd1_var->ax_f * N1_tmp + pnd2_var->ax_f * N2_tmp
					 + pnd3_var->ax_f * N3_tmp + pnd4_var->ax_f * N4_tmp) * dt;
		ppcl->vy_f += (pnd1_var->ay_f * N1_tmp + pnd2_var->ay_f * N2_tmp
					 + pnd3_var->ay_f * N3_tmp + pnd4_var->ay_f * N4_tmp) * dt;

		// displacement
		ppcl->ux_s += pnd1_var->dux_s * N1_tmp + pnd2_var->dux_s * N2_tmp
					+ pnd3_var->dux_s * N3_tmp + pnd4_var->dux_s * N4_tmp;
		ppcl->uy_s += pnd1_var->duy_s * N1_tmp + pnd2_var->duy_s * N2_tmp
					+ pnd3_var->duy_s * N3_tmp + pnd4_var->duy_s * N4_tmp;
		ppcl->ux_f += pnd1_var->dux_f * N1_tmp + pnd2_var->dux_f * N2_tmp
					+ pnd3_var->dux_f * N3_tmp + pnd4_var->dux_f * N4_tmp;
		ppcl->uy_f += pnd1_var->duy_f * N1_tmp + pnd2_var->duy_f * N2_tmp
					+ pnd3_var->duy_f * N3_tmp + pnd4_var->duy_f * N4_tmp;

		// update position
		ppcl->x = ppcl->x_ori + ppcl->ux_s;
		ppcl->y = ppcl->y_ori + ppcl->uy_s;

		// strain increment
		de11_s = pnd1_var->dux_s * ppcl_var->dN1_dx + pnd2_var->dux_s * ppcl_var->dN2_dx
			   + pnd3_var->dux_s * ppcl_var->dN3_dx + pnd4_var->dux_s * ppcl_var->dN4_dx;
		de22_s = pnd1_var->duy_s * ppcl_var->dN1_dy + pnd2_var->duy_s * ppcl_var->dN2_dy
			   + pnd3_var->duy_s * ppcl_var->dN3_dy + pnd4_var->duy_s * ppcl_var->dN4_dy;
		de12_s = (pnd1_var->dux_s * ppcl_var->dN1_dy + pnd2_var->dux_s * ppcl_var->dN2_dy
				+ pnd3_var->dux_s * ppcl_var->dN3_dy + pnd4_var->dux_s * ppcl_var->dN4_dy
				+ pnd1_var->duy_s * ppcl_var->dN1_dx + pnd2_var->duy_s * ppcl_var->dN2_dx
				+ pnd3_var->duy_s * ppcl_var->dN3_dx + pnd4_var->duy_s * ppcl_var->dN4_dx) * 0.5;
		dw12 = (pnd1_var->dux_s * ppcl_var->dN1_dy + pnd2_var->dux_s * ppcl_var->dN2_dy
			  + pnd3_var->dux_s * ppcl_var->dN3_dy + pnd4_var->dux_s * ppcl_var->dN4_dy
			  - pnd1_var->duy_s * ppcl_var->dN1_dx - pnd2_var->duy_s * ppcl_var->dN2_dx
			  - pnd3_var->duy_s * ppcl_var->dN3_dx - pnd4_var->duy_s * ppcl_var->dN4_dx) * 0.5;

		// update strain (also assume that strain increment is Jaumann rate)
		//ppcl->de11 +=  dw12 * e12 * 2.0;
		//ppcl->de22 += -dw12 * e12 * 2.0;
		//ppcl->de12 +=  dw12 * (e22 - e11);
		ppcl->e11 += de11_s;
		ppcl->e22 += de22_s;
		ppcl->e12 += de12_s;

		// update stress
		E_tmp = ppcl->E / (1.0 + ppcl->niu) / (1.0 - 2.0 * ppcl->niu);
		ds11 = E_tmp * ((1.0 - ppcl->niu) * de11_s + ppcl->niu * de22_s);
		ds22 = E_tmp * (ppcl->niu * de11_s + (1.0 - ppcl->niu) * de22_s);
		ds12 = 2.0 * de12_s * ppcl->E / (2.0 * (1.0 + ppcl->niu));

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

		// volumetric strain of solid phase
		de_vol_s = de11_s + de22_s;
		// volume
		ppcl->vol *= (1.0 + de_vol_s);
		// porosity
		ppcl->n = (de_vol_s + ppcl->n) / (1.0 + de_vol_s);
		// "volumetric strain" of fluid phase
		de_vol_f = -(1.0 - ppcl->n) / ppcl->n * de_vol_s
				   - (pnd1_var->dux_f * ppcl_var->dN1_dx + pnd2_var->dux_f * ppcl_var->dN2_dx 
					+ pnd3_var->dux_f * ppcl_var->dN3_dx + pnd4_var->dux_f * ppcl_var->dN4_dx)
				   - (pnd1_var->duy_f * ppcl_var->dN1_dy + pnd2_var->duy_f * ppcl_var->dN2_dy
					+ pnd3_var->duy_f * ppcl_var->dN3_dy + pnd4_var->duy_f * ppcl_var->dN4_dy);

		// pore pressure
		ppcl->p += ppcl->Kf * de_vol_f;
		// fluid density
		ppcl->density_f += ppcl->density_f * de_vol_f;

		ppcl_var = pcl_vars->next(ppcl_var);
	}
	
	return 0;
}