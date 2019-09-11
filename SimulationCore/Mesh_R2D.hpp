#ifndef _MESH_R2D_HPP_
#define _MESH_R2D_HPP_

#include "Mesh.h"
#include "ItemArray.hpp"

namespace
{
	inline double get_min(double a, double b) { return a < b ? a : b; }
}

struct Node_R2D : public Node
{
	size_t index_x, index_y;
};

struct Element_R2D : public Element
{
	// Topology
	size_t index_x, index_y;
	// characteristic length of element
	double min_len;
};

template<class NodeType, class ElementType>
struct Mesh_R2D : public Mesh
{
public:
	// Mesh geometry
	size_t node_x_num;
	double *node_x_coords;
	size_t node_y_num;
	double *node_y_coords;
	size_t node_num;
	NodeType *nodes;
	size_t elem_x_num;
	size_t elem_y_num;
	size_t elem_num;
	ElementType *elems;

protected:
	// coordinates
	MemoryUtilities::ItemArray<double> node_x_coords_mem;
	MemoryUtilities::ItemArray<double> node_y_coords_mem;

public:
	Mesh_R2D():
		node_x_num(0), node_x_coords(nullptr),
		node_y_num(0), node_y_coords(nullptr),
		node_num(0), nodes(nullptr),
		elem_x_num(0), elem_y_num(0),
		elem_num(0), elems(nullptr) {}
	~Mesh_R2D() { clear(); }

	inline void set_x_coord_num(size_t num) { node_x_coords_mem.reserve(num); }
	inline void add_x_coord(double coord) { node_x_coords_mem.add(coord); }
	inline void add_x_coord(size_t num, double *coords)
	{
		double *node_x_coords = node_x_coords_mem.alloc(num);
		memcpy(node_x_coords, coords, sizeof(double) * num);
	}
	inline void set_y_coord_num(size_t num) { node_y_coords_mem.reserve(num); }
	inline void add_y_coord(double coord) { node_y_coords_mem.add(coord); }
	inline void add_y_coord(size_t num, double *coords)
	{
		double *node_y_coords = node_y_coords_mem.alloc(num);
		memcpy(node_y_coords, coords, sizeof(double) * num);
	}
	void update(void)
	{
		size_t i, j, k;
		// update nodes
		node_x_num = node_x_coords_mem.get_num();
		node_x_coords = node_x_num ? node_x_coords_mem.get_mem() : nullptr;
		node_y_num = node_y_coords_mem.get_num();
		node_y_coords = node_y_num ? node_y_coords_mem.get_mem() : nullptr;
		node_num = node_x_num * node_y_num;
		if (nodes) delete[] nodes;
		nodes = nullptr;
		if (node_num)
		{
			k = 0;
			nodes = new Node_BG_R2D_ME[node_num];
			for (i = 0; i < node_y_num; i++)
				for (j = 0; j < node_x_num; j++)
				{
					nodes[k].index_x = j;
					nodes[k].index_y = i;
					++k;
				}
		}
		// udpate elements
		elem_x_num = node_x_num > 1 ? node_x_num - 1 : 0;
		elem_y_num = node_y_num > 1 ? node_y_num - 1 : 0;
		elem_num = elem_x_num * elem_y_num;
		if (elems) delete[] elems;
		elems = nullptr;
		if (elem_num)
		{
			k = 0;
			elems = new Element_BG_R2D_ME[elem_num];
			for (i = 0; i < elem_y_num; i++)
				for (j = 0; j < elem_x_num; j++)
				{
					elems[k].index_x = j;
					elems[k].index_y = i;
					++k;
				}
		}
	}
	void clear(void)
	{
		node_x_num = 0;
		node_x_coords = nullptr;
		node_y_num = 0;
		node_y_coords = nullptr;
		node_num = 0;
		if (nodes) delete[] nodes;
		nodes = nullptr;
		elem_x_num = 0;
		elem_y_num = 0;
		elem_num = 0;
		if (elems) delete[] elems;
		elems = nullptr;
		node_x_coords_mem.reset();
		node_y_coords_mem.reset();
	}

	double min_length(ElementType *pelem)
	{
		elem->min_len = get_min(abs(node_x_coords[elem->index_x + 1] - node_x_coords[elem->index_x]),
								abs(node_y_coords[elem->index_y + 1] - node_y_coords[elem->index_y]));
		return elem->min_len;
	}

public:
	inline void get_nodes_of_element(
		ElementType *pelem,
		NodeType *&pnode1, NodeType *&pnode2,
		NodeType *&pnode3, NodeType *&pnode4)
	{
		pnode1 = nodes + node_x_num * pelem->index_y + pelem->index_x;
		pnode2 = pnode1 + 1;
		pnode3 = pnode2 + node_x_num;
		pnode4 = pnode3 - 1;
	}

	/* | elem2 | elem1 |
	 * |-------|-------|
	 * | elem3 | elem4 |
	 */
	inline void get_elements_by_node(
		NodeType *pnode,
		ElementType *&pelem1, ElementType *&pelem2,
		ElementType *&pelem3, ElementType *&pelem4)
	{
		size_t x_id = pnode->index_x;
		size_t y_id = pnode->index_y;
		size_t e_x1_id, e_x2_id;
		size_t e_y1_id, e_y2_id;

		e_x1_id = x_id;
		e_x2_id = x_id;
		if (x_id == 0) e_x1_id = node_x_num + 1;
		if (x_id == node_x_num - 1)	e_x2_id = node_x_num;
		--e_x1_id;

		e_y1_id = y_id;
		e_y2_id = y_id;
		if (y_id == 0) e_y1_id = node_y_num + 1;
		if (y_id == node_y_num - 1) e_y2_id = node_y_num;
		--e_y1_id;

		if (e_x2_id < node_x_num && e_y2_id < node_y_num)
			pelem1 = elems + elem_x_num * e_y2_id  + e_x2_id;
		else
			pelem1 = nullptr;

		if (e_x1_id < node_x_num && e_y2_id < node_y_num)
			pelem2 = elems + elem_x_num * e_y2_id + e_x1_id;
		else
			pelem2 = nullptr;

		if (e_x1_id < node_x_num && e_y1_id < node_y_num)
			pelem3 = elems + elem_x_num * e_y1_id + e_x1_id;
		else
			pelem3 = nullptr;

		if (e_x2_id < node_x_num && e_y1_id < node_y_num)
			pelem4 = elems + elem_x_num * e_y1_id + e_x2_id;
		else
			pelem4 = nullptr;
	}

	ElementType *find_in_which_element(double x, double y)
	{
		size_t elem_x_id, elem_y_id;

		// check if coords lies in elem_buffer
		if (elem_buffer)
		{
			elem_x_id = elem_buffer->index_x;
			elem_y_id = elem_buffer->index_y;

			if (x >= node_x_coords[elem_x_id] &&
				x <  node_x_coords[elem_x_id + 1] &&
				y >= node_y_coords[elem_y_id] &&
				y <  node_y_coords[elem_y_id + 1])
				return elem_buffer;
		}

		if (find_x_index(x, &elem_x_id) == -1 || find_y_index(y, &elem_y_id) == -1)
			return nullptr;

		elem_buffer = elems + elem_x_num * elem_y_id + elem_x_id;

		return elem_buffer;
	}
	ElementType *find_in_which_element(double x, double y, ElementType *pelem)
	{
		size_t elem_x_id, elem_y_id;
		long long int elem_id_offset;

		// first check if coords is in elem and its adjacent elements
		if (pelem)
		{
			elem_x_id = pelem->index_x;
			elem_y_id = pelem->index_y;
			elem_id_offset = 0;

			// x direction
			if (x >= node_x_coords[elem_x_id])
			{
				if (x >= node_x_coords[elem_x_id + 1])
				{
					// check if coords lie outside mesh
					if (elem_x_id + 1 == elem_x_num)
						return nullptr;

					if (x >= node_x_coords[elem_x_id + 2])
						goto PointNotInOrNearElem;

					elem_id_offset++;
				}
			}
			else
			{
				// check if coords lie outside mesh
				if (elem_x_id == 0)
					return nullptr;

				if (x < node_x_coords[elem_x_id - 1])
					goto PointNotInOrNearElem;

				elem_id_offset--;
			}

			// y direction
			if (y >= node_y_coords[elem_y_id])
			{
				if (y >= node_y_coords[elem_y_id + 1])
				{
					// check if coords lie outside mesh
					if (elem_y_id + 1 == elem_y_num)
						return nullptr;

					if (y >= node_y_coords[elem_y_id + 2])
						goto PointNotInOrNearElem;

					elem_id_offset += (long int)elem_x_num;
				}
			}
			else
			{
				// check if coords lie outside mesh
				if (elem_y_id == 0)
					return nullptr;

				if (y < node_y_coords[elem_y_id - 1])
					goto PointNotInOrNearElem;

				elem_id_offset -= (long int)elem_x_num;
			}

			return &elems[elem_y_id * elem_x_num + elem_x_id + elem_id_offset];
		}

	PointNotInOrNearElem:
		return find_in_which_element(x, y);
	}

protected:
	// for accelerating inWhichElement(double *coords;)
	ElementType *elem_buffer;
	// Find in which coords interval the coordinate lies
	// return -1 if coordinate lie out side the cordinates range
	int find_x_index(double x, size_t *index)
	{
		size_t head = 0, tail = node_x_num - 1;
		size_t middle;

		// check if in range
		if (x < node_x_coords[head] || x >= node_x_coords[tail])
			return -1;

		// use bisection for searching
		do
		{
			middle = (head + tail) / 2;
			if (x < node_x_coords[middle])
				tail = middle;
			else if (x > node_x_coords[middle])
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
	int find_y_index(double y, size_t *index)
	{
		size_t head = 0, tail = node_y_num - 1;
		size_t middle;

		// check if in range
		if (y < node_y_coords[head] || y >= node_y_coords[tail])
			return -1;

		// use bisection for searching
		do
		{
			middle = (head + tail) / 2;
			if (y < node_y_coords[middle])
				tail = middle;
			else if (y > node_y_coords[middle])
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
};

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