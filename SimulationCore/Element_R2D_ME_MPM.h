#ifndef _ELEMENT_R2D_ME_MPM_H_
#define _ELEMENT_R2D_ME_MPM_H_

struct Particle_2D_ME;

/* ===============================================
Class Element_R2D_ME_MPM

local coordinate:
	eta
	/\
	|
	O ---> xi

local node number for each element:
	4________________3
	|                |
	|                |
	|                |
	|                |
	|________________|
	1                2
edge 1: node 1 -- node 2;
edge 2: node 2 -- node 3;
edge 3: node 3 -- node 4;
edge 4: node 4 -- node 1;
 =============================================== */
struct Element_R2D_ME_MPM
{
	// Topology
	size_t index_x, index_y;
		
	// particles in this element
	Particle_2D_ME *head;

	// characteristic length of element
	double char_len;
};

#endif

// Input element and mesh and return four nodes 
// of the element: pn1, pn2, pn3 and pn4.
#ifndef Get_Nodes_Of_Element_R2D
#define Get_Nodes_Of_Element_R2D(pelem, mesh, pn1, pn2, pn3, pn4)                     \
	do {                                                                              \
	(pn1) = (mesh)->nodes + (mesh)->node_x_num * (pelem)->index_y + (pelem)->index_x; \
	(pn2) = (pn1) + 1;                                                                \
	(pn3) = (pn2) + (mesh)->node_x_num;                                               \
	(pn4) = (pn3) - 1;                                                                \
	} while(false)
#endif

#ifndef _SHAPE_FUNCTION_R2D_
#define _SHAPE_FUNCTION_R2D_

// shape function
#define _N1_R2D(xi, eta) ((1.0 - (xi)) * (1.0 - (eta)) / 4.0)
#define _N2_R2D(xi, eta) ((1.0 + (xi)) * (1.0 - (eta)) / 4.0)
#define _N3_R2D(xi, eta) ((1.0 + (xi)) * (1.0 + (eta)) / 4.0)
#define _N4_R2D(xi, eta) ((1.0 - (xi)) * (1.0 + (eta)) / 4.0)
// one order derivative of shape function
#define _dN1_dxi_R2D(xi, eta) (-(1.0 - (eta)) / 4.0)
#define _dN1_deta_R2D(xi, eta) (-(1.0 - (xi)) / 4.0)
#define _dN2_dxi_R2D(xi, eta) ( (1.0 - (eta)) / 4.0)
#define _dN2_deta_R2D(xi, eta) (-(1.0 + (xi)) / 4.0)
#define _dN3_dxi_R2D(xi, eta) ( (1.0 + (eta)) / 4.0)
#define _dN3_deta_R2D(xi, eta) ( (1.0 + (xi)) / 4.0)
#define _dN4_dxi_R2D(xi, eta) (-(1.0 + (eta)) / 4.0)
#define _dN4_deta_R2D(xi, eta) ( (1.0 - (xi)) / 4.0)

#endif