#ifndef __RIGID_BODY_H__
#define __RIGID_BODY_H__

#include "TriangleMesh.h"

#define PI 3.14159265359
// limit theta to [-pi, pi]
inline void trim_to_pi(double &theta)
{
	if (theta > PI)
		theta -= (2.0*PI) * long long((theta + PI) / (2.0*PI));
	else if (theta < -PI)
		theta -= (2.0*PI) * long long((theta - PI) / (2.0*PI));
}
#undef PI

class RigidBody
{
public:
	double m, L;

	// mass centre is (x, y)
	// theta (in radians) takes counter-clockwise as positive
	double x, y, theta;
	double vx, vy, v_theta;
	// x = x_ori + ux, y = y_ori + uy
	double x_ori, y_ori, theta_ori;
	double ux, uy, u_theta;

	// external force
	double Fx_ext, Fy_ext, M_ext;
	// contact force
	double Fx_con, Fy_con, M_con;

	TriangleMesh mesh;

public:
	RigidBody() : m(0.0), L(0.0),
		x_ori(0.0), y_ori(0.0), theta_ori(0.0),
		x(0.0), y(0.0), theta(0.0),
		ux(0.0), uy(0.0), u_theta(0.0),
		vx(0.0), vy(0.0), v_theta(0.0),
		Fx_ext(0.0), Fy_ext(0.0), M_ext(0.0),
		Fx_con(0.0), Fy_con(0.0), M_con(0.0) {}
	~RigidBody() {}
	int load_and_init_mesh(const char *filename, double bg_grid_size = -1.0);
	// called after load_and_init_mesh()
	inline void set_params(double density,
		double _x = 0.0, double _y = 0.0, double _theta = 0.0,
		double _vx = 0.0, double _vy = 0.0, double _v_theta = 0.0) noexcept
	{
		m = mesh.area * density;
		L = mesh.moi_area * density;
		x = _x, y = _y, theta = _theta, trim_to_pi(theta);
		_vx = vx, _vy = vy, v_theta = _v_theta;
	}
	// add force at mass centre
	inline void add_ext_force(double fx, double fy)
	{
		Fx_ext += fx, Fy_ext += fy;
	}
	inline void add_ext_force(double fx, double fy, double posx, double posy)
	{
		Fx_ext += fx, Fy_ext += fy;
		M_ext += (posx - x) * fy - (posy - y) * fx;
	}

public: // for calculation
	inline void init_calculation(void)
	{
		x_ori = x;
		y_ori = y;
		trim_to_pi(theta);
		theta_ori = theta;
		ux = 0.0;
		uy = 0.0;
		u_theta = 0.0;
		Fx_con = 0.0;
		Fy_con = 0.0;
		M_con = 0.0;
	}
	inline void add_con_force(double fx, double fy, double posx, double posy)
	{
		Fx_con += fx;
		Fy_con += fy;
		M_con  += (posx - x) * fy - (posy - y) * fx;
	}
	inline void predict_motion_from_ext_force(double dt) // Euler - Cromer
	{
		vx += Fx_ext / m * dt;
		vy += Fy_ext / m * dt;
		v_theta += M_ext / L * dt;
		ux += vx * dt;
		uy += vy * dt;
		u_theta += v_theta * dt;
		x = x_ori + ux;
		y = y_ori + uy;
		theta = theta_ori + u_theta;
		trim_to_pi(theta);
	}
	inline void correct_motion_by_con_force(double dt) // Euler - Cromer
	{
		double dt_2 = dt * dt;
		double ax_corr, ay_corr, a_theta_corr;
		ax_corr = Fx_con / m;
		ay_corr = Fy_con / m;
		a_theta_corr = M_con / L;
		vx += ax_corr * dt;
		vy += ay_corr * dt;
		v_theta += a_theta_corr * dt;
		ux += ax_corr * dt_2;
		uy += ay_corr * dt_2;
		u_theta += a_theta_corr * dt_2;
		x = x_ori + ux;
		y = y_ori + uy;
		theta = theta_ori + u_theta;
		trim_to_pi(theta);

		// reset for the next substep
		Fx_con = 0.0;
		Fy_con = 0.0;
		M_con = 0.0;
	}

protected: // helper data and functions
		   // for transformation
	double sin_theta, cos_theta;
public:
	// called after set_init_pos()
	inline void init_transformation(void) noexcept
	{
		sin_theta = sin(theta), cos_theta = cos(theta);
	}
	// functions below should be called after init_transformation()
	void get_bounding_box(double &x1, double &y1, double &x2, double &y2,
						  double &x3, double &y3, double &x4, double &y4,
						  double expand_size = 0.0);
	inline int distance_from_boundary(Point &p, double &dist, double &nx, double &ny,
									  double dist_max = std::numeric_limits<double>::max())
	{
		// transform into local coordinates
		double x_tmp = p.x - x, y_tmp = p.y - y;
		Point lp;
		lp.x = x_tmp *  cos_theta + y_tmp * sin_theta + mesh.x_mc;
		lp.y = x_tmp * -sin_theta + y_tmp * cos_theta + mesh.y_mc;
		
		int res = mesh.distance_to_boundary(lp, dist, nx, ny, dist_max);

		// transform nx, ny into glbal coordinates
		double nx_tmp = nx, ny_tmp = ny;
		nx = nx_tmp * cos_theta - ny_tmp * sin_theta;
		ny = nx_tmp * sin_theta + ny_tmp * cos_theta;
		//std::cout << "\nPoint in global coords: (" << p.x << ", " << p.y << ").\n"
		//		  "Distance: " << dist << ", normal: (" << nx << ", " << ny << ")\n";

		return res;
	}
	inline Point to_local_coord(Point &gp) const noexcept
	{
		double x_tmp = gp.x - x, y_tmp = gp.y - y;
		Point lp;
		lp.x = x_tmp *  cos_theta + y_tmp * sin_theta + mesh.x_mc;
		lp.y = x_tmp * -sin_theta + y_tmp * cos_theta + mesh.y_mc;
		return lp;
	}
	inline Point to_global_coord(Point &lp) const noexcept
	{
		double x_tmp = lp.x - mesh.x_mc, y_tmp = lp.y - mesh.y_mc;
		Point gp;
		gp.x = x_tmp * cos_theta + y_tmp * -sin_theta + x;
		gp.y = x_tmp * sin_theta + y_tmp *  cos_theta + y;
		return gp;
	}
};

#endif