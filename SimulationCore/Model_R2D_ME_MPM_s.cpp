#include "SimulationCore_pcp.h"

#include "Model_R2D_ME_MPM_s.h"

Model_R2D_ME_MPM_s::Model_R2D_ME_MPM_s() :
	Model("R2D_ME_MPM_s"),
	node_x_num(0), node_coords_x(nullptr),
	node_y_num(0), node_coords_y(nullptr),
	node_num(0), nodes(nullptr),
	elem_x_num(0), elem_y_num(0),
	elem_num(0), elems(nullptr),
	pcl_num(0), pcls(nullptr),
	bfx_num(0), bfxs(nullptr),
	bfy_num(0), bfys(nullptr),
	tx_bc_num(0), tx_bcs(nullptr),
	ty_bc_num(0), ty_bcs(nullptr),
	ax_s_bc_num(0), ax_s_bcs(nullptr),
	ay_s_bc_num(0), ay_s_bcs(nullptr),
	vx_s_bc_num(0), vx_s_bcs(nullptr),
	vy_s_bc_num(0), vy_s_bcs(nullptr),
	elem_buffer(nullptr) {}

Model_R2D_ME_MPM_s::~Model_R2D_ME_MPM_s()
{
	if(node_coords_x) delete[] node_coords_x;
	if(node_coords_y) delete[] node_coords_y;
	if(nodes) delete[] nodes;
	if(elems) delete[] elems;
	if(pcls) delete[] pcls;
	if(bfxs) delete[] bfxs;
	if(bfys) delete[] bfys;
	if(tx_bcs) delete[] tx_bcs;
	if(ty_bcs) delete[] ty_bcs;
	if(ax_s_bcs) delete[] ax_s_bcs;
	if(ay_s_bcs) delete[] ay_s_bcs;
	if(vx_s_bcs) delete[] vx_s_bcs;
	if(vy_s_bcs) delete[] vy_s_bcs;
}

Element_R2D_ME_MPM_s *Model_R2D_ME_MPM_s::find_in_which_element(double x, double y)
{
	size_t elem_x_id, elem_y_id;

	// check if coords lies in elem_buffer
	if (elem_buffer)
	{
		elem_x_id = elem_buffer->index_x;
		elem_y_id = elem_buffer->index_y;

		if (x >= node_coords_x[elem_x_id] &&
			x <  node_coords_x[elem_x_id + 1] &&
			y >= node_coords_y[elem_y_id] &&
			y <  node_coords_y[elem_y_id + 1])
			return elem_buffer;
	}

	if (find_x_index(x, &elem_x_id) == -1 || find_y_index(y, &elem_y_id) == -1)
		return nullptr;

	elem_buffer = elems + elem_x_num * elem_y_id + elem_x_id;

	return elem_buffer;
}

Element_R2D_ME_MPM_s *Model_R2D_ME_MPM_s::find_in_which_element(
	double x, double y, Element_R2D_ME_MPM_s *elem)
{
	size_t elem_x_id, elem_y_id;
	long long int elem_id_offset;

	// first check if coords is in elem and its adjacent elements
	if (elem)
	{
		elem_x_id = elem->index_x;
		elem_y_id = elem->index_y;
		elem_id_offset = 0;

		// x direction
		if (x >= node_coords_x[elem_x_id])
		{
			if (x >= node_coords_x[elem_x_id + 1])
			{
				// check if coords lie outside mesh
				if (elem_x_id + 1 == elem_x_num)
					return nullptr;

				if (x >= node_coords_x[elem_x_id + 2])
					goto PointNotInOrNearElem;

				elem_id_offset++;
			}
		}
		else
		{
			// check if coords lie outside mesh
			if (elem_x_id == 0)
				return nullptr;

			if (x < node_coords_x[elem_x_id - 1])
				goto PointNotInOrNearElem;

			elem_id_offset--;
		}

		// y direction
		if (y >= node_coords_y[elem_y_id])
		{
			if (y >= node_coords_y[elem_y_id + 1])
			{
				// check if coords lie outside mesh
				if (elem_y_id + 1 == elem_y_num)
					return nullptr;

				if (y >= node_coords_y[elem_y_id + 2])
					goto PointNotInOrNearElem;

				elem_id_offset += (long int)elem_x_num;
			}
		}
		else
		{
			// check if coords lie outside mesh
			if (elem_y_id == 0)
				return nullptr;

			if (y < node_coords_y[elem_y_id - 1])
				goto PointNotInOrNearElem;

			elem_id_offset -= (long int)elem_x_num;
		}

		return &elems[elem_y_id * elem_x_num + elem_x_id + elem_id_offset];
	}

PointNotInOrNearElem:
	return find_in_which_element(x, y);
}

inline double get_min(double a, double b) { return a < b ? a : b; }
double Model_R2D_ME_MPM_s::cal_characteristic_length(Element_R2D_ME_MPM_s *elem)
{
	elem->char_len = get_min(abs(node_coords_x[elem->index_x + 1] - node_coords_x[elem->index_x]),
							 abs(node_coords_y[elem->index_y + 1] - node_coords_y[elem->index_y]));
	return elem->char_len;
}

void Model_R2D_ME_MPM_s::cal_shape_function(Particle_R2D_ME_s *pcl)
{
	Element_R2D_ME_MPM_s *pelem;
	double xLower, xUpper, yLower, yUpper;
	double xMiddle, xHalfLength, yMiddle, yHalfLength;
	double x1, x2, x3, x4, y1, y2, y3, y4;
	double dN1_dxi, dN1_deta, dN2_dxi, dN2_deta;
	double dN3_dxi, dN3_deta, dN4_dxi, dN4_deta;
	double dx_dxi, dx_deta, dy_dxi, dy_deta;
	double dxi_dx, dxi_dy, deta_dx, deta_dy;
	double Jdet;

	pelem = pcl->elem;
	xLower = node_coords_x[pelem->index_x];
	xUpper = node_coords_x[pelem->index_x + 1];
	yLower = node_coords_y[pelem->index_y];
	yUpper = node_coords_y[pelem->index_y + 1];

	xHalfLength = (xUpper - xLower) / 2.0;
	yHalfLength = (yUpper - yLower) / 2.0;
	xMiddle = (xUpper + xLower) / 2.0;
	yMiddle = (yUpper + yLower) / 2.0;
	
	pcl->xi  = (pcl->x - xMiddle) / xHalfLength;
	pcl->eta = (pcl->y - yMiddle) / yHalfLength;
	
	pcl->N1 = _N1_R2D(pcl->xi, pcl->eta);
	pcl->N1 = pcl->N1 < N_tol ? N_tol : pcl->N1;
	pcl->N2 = _N2_R2D(pcl->xi, pcl->eta);
	pcl->N2 = pcl->N2 < N_tol ? N_tol : pcl->N2;
	pcl->N3 = _N3_R2D(pcl->xi, pcl->eta);
	pcl->N3 = pcl->N3 < N_tol ? N_tol : pcl->N3;
	pcl->N4 = _N4_R2D(pcl->xi, pcl->eta);
	pcl->N4 = pcl->N4 < N_tol ? N_tol : pcl->N4;

	dN1_dxi  = _dN1_dxi_R2D(pcl->xi,  pcl->eta);
	dN1_deta = _dN1_deta_R2D(pcl->xi, pcl->eta);
	dN2_dxi  = _dN2_dxi_R2D(pcl->xi,  pcl->eta);
	dN2_deta = _dN2_deta_R2D(pcl->xi, pcl->eta);
	dN3_dxi  = _dN3_dxi_R2D(pcl->xi,  pcl->eta);
	dN3_deta = _dN3_deta_R2D(pcl->xi, pcl->eta);
	dN4_dxi  = _dN4_dxi_R2D(pcl->xi,  pcl->eta);
	dN4_deta = _dN4_deta_R2D(pcl->xi, pcl->eta);

	/* --------------------------------
			    Jacobian matrix:
				dx_dxi, dx_deta,
				dy_dxi, dy_deta,
	  --------------------------------*/
	x1 = x4 = node_coords_x[pelem->index_x];
	x2 = x3 = node_coords_x[pelem->index_x + 1];
	y1 = y2 = node_coords_y[pelem->index_y];
	y3 = y4 = node_coords_y[pelem->index_y + 1];
	dx_dxi  = dN1_dxi  * x1 + dN2_dxi  * x2 + dN3_dxi  * x3 + dN4_dxi  * x4;
	dx_deta = dN1_deta * x1 + dN2_deta * x2 + dN3_deta * x3 + dN4_deta * x4;
	dy_dxi  = dN1_dxi  * y1 + dN2_dxi  * y2 + dN3_dxi  * y3 + dN4_dxi  * y4;
	dy_deta = dN1_deta * y1 + dN2_deta * y2 + dN3_deta * y3 + dN4_deta * y4;

	// determinant of Jacobian matrix
	Jdet = dx_dxi * dy_deta - dx_deta * dy_dxi;
	/* -----------------------------------
		   Inverse of Jacobian matrix:
				dxi_dx,  dxi_dy,
				deta_dx, deta_dy,
	  ----------------------------------- */
	dxi_dx  =  dy_deta / Jdet;
	dxi_dy  = -dx_deta / Jdet;
	deta_dx = -dy_dxi  / Jdet;
	deta_dy =  dx_dxi  / Jdet;

	pcl->dN1_dx = dN1_dxi * dxi_dx + dN1_deta * deta_dx;
	pcl->dN1_dy = dN1_dxi * dxi_dy + dN1_deta * deta_dy;
	pcl->dN2_dx = dN2_dxi * dxi_dx + dN2_deta * deta_dx;
	pcl->dN2_dy = dN2_dxi * dxi_dy + dN2_deta * deta_dy;
	pcl->dN3_dx = dN3_dxi * dxi_dx + dN3_deta * deta_dx;
	pcl->dN3_dy = dN3_dxi * dxi_dy + dN3_deta * deta_dy;
	pcl->dN4_dx = dN4_dxi * dxi_dx + dN4_deta * deta_dx;
	pcl->dN4_dy = dN4_dxi * dxi_dy + dN4_deta * deta_dy;
}


// return index of left side of interval
// note that tail is the last index + 1
int Model_R2D_ME_MPM_s::find_x_index(double x, size_t *index)
{
	size_t head = 0, tail = node_x_num - 1;
	size_t middle;

	// check if in range
	if (x < node_coords_x[head] || x >= node_coords_x[tail])
		return -1;

	// use bisection for searching
	do
	{
		middle = (head + tail) / 2;
		if (x < node_coords_x[middle])
			tail = middle;
		else if (x > node_coords_x[middle])
			head = middle;
		else
		{
			// but this situation is rare.
			*index = middle;
			return 0;
		}
	} while (head != (tail - 1));
	*index = head;

	return 0;
}

// return index of left side of interval
// note that tail is the last index + 1
int Model_R2D_ME_MPM_s::find_y_index(double y, size_t *index)
{
	size_t head = 0, tail = node_y_num - 1;
	size_t middle;

	// check if in range
	if (y < node_coords_y[head] || y >= node_coords_y[tail])
		return -1;

	// use bisection for searching
	do
	{
		middle = (head + tail) / 2;
		if (y < node_coords_y[middle])
			tail = middle;
		else if (y > node_coords_y[middle])
			head = middle;
		else
		{
			// but this situation is rare.
			*index = middle;
			return 0;
		}
	} while (head != (tail - 1));
	*index = head;

	return 0;
}