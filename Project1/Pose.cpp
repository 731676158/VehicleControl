#pragma once
#include <Eigen/Dense>
#include "Pose.h"
using namespace std;
using namespace Eigen;

void Pose::update(double dt, double v_inc, double delta_inc)
{
	delta = para.ref_delta + delta_inc;
	pos_x += v * cos(pos_yaw) * dt;
	pos_y += v * sin(pos_yaw) * dt;
	pos_yaw += v * tan(delta) * dt / para.L;
	v = para.ref_speed + v_inc;
}

void Pose::update(double dt,Vector2d& U)
{
	double v_inc = U(0);
	double delta_inc = U(1);
	delta = para.ref_delta + delta_inc;
	pos_x += v * cos(pos_yaw) * dt;
	pos_y += v * sin(pos_yaw) * dt;
	pos_yaw += v * tan(delta) * dt / para.L;
	v = para.ref_speed + v_inc;
}

void refPos::setfun(function<double(double)>& fun,int x_rg)
{
	nums = x_rg;
	ref_x.resize(x_rg);
	ref_y.resize(x_rg);
	for (int i = 0; i < x_rg; ++i)
	{
		ref_x[i] = i + 1;
		ref_y[i] = fun(i + 1);
	}
	refderiCalcu();
}

double refPos::refposition(double pos_x, double pos_y)
{
	int min_idx = 0;
	double min_dis = DBL_MAX;
	double this_dis = 0;
	for (int i = 0; i < nums; ++i)
	{
		this_dis = sqrt(pow(ref_x[i] - pos_x, 2) + pow(ref_y[i] - pos_y, 2));
		if (this_dis < min_dis)
		{
			min_idx = i;
			min_dis = this_dis;
		}
	}
	idx = min_idx;
	return this_dis;
}

void refPos::refderiCalcu()
{
	ref_pos_d.resize(nums);
	ref_pos_dd.resize(nums);
	ref_yaw.resize(nums);
	ref_pos_k.resize(nums);
	for (int i = 0; i < (nums - 1); ++i)
	{
		ref_pos_d[i] = (ref_y[i + 1] - ref_y[i]) / (ref_x[i + 1] - ref_x[i]);
		if (i != 0)
		{
			ref_pos_dd[i] = (ref_y[i + 1] - 2 * ref_y[i] + ref_y[i - 1]) / (pow((0.5 * (ref_x[i + 1] - ref_x[i - 1])), 2));
		}
		ref_yaw[i] = atan(ref_pos_d[i]);
		ref_pos_k[i] = (ref_pos_d[i]) / (pow(1 + pow(ref_pos_d[i], 2), 1.5));
	}
	ref_pos_dd[0] = ref_pos_dd[1];
	ref_pos_d[nums - 1] = ref_pos_d[nums - 2];
	ref_pos_dd[nums - 1] = ref_pos_dd[nums - 2];
	ref_yaw[nums - 1] = ref_yaw[nums - 2];
	ref_pos_k[nums - 1] = ref_pos_k[nums - 2];
}

