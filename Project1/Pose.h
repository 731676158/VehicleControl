#pragma once
#include <iostream>
#include <vector>
#include <functional>
#include "Eigen/Dense"
using namespace std;
using namespace Eigen;

class Parameter
{
public:
	Parameter(double len = 2.9, double t = 0.1, double refd = 0, double refs = 40 / 3.6)
	{
		L = len;
		dt = t;
		ref_delta = refd;
		ref_speed = refs;
	}
public:
	double L;
	double dt;
	double ref_delta, ref_speed;
};

class refPos
{
public:
	void setfun(function<double(double)>& fun, int x_rg);
	double refposition(double pos_x, double pos_y);   //ªπŒ¥∂®“Â
public:
	double idx;
	int nums;
	vector<double> ref_x;
	vector<double> ref_y;
	vector<double> ref_yaw;
	vector<double> ref_pos_d;
	vector<double> ref_pos_dd;
	vector<double> ref_pos_k;
private:
	void refderiCalcu();
};

class Pose
{
public:
	Pose(double x = 0,double y = 2,double yaw = 0.5,double velo = 10.0,double del = 0)
	{
		pos_x = x;
		pos_y = y;
		pos_yaw = yaw;
		v = velo;
		delta = del;
	}
	void update(double dt, double v_inc, double delta_inc);
	void update(double dt, Vector2d& U);

public:
	double pos_x, pos_y, pos_yaw;
	double v;
	double delta;
	Parameter para;
	refPos refpos;
};

