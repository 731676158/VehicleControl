#pragma once
#include <Eigen/Dense>
#include <stdexcept>
#define _USE_MATH_DEFINES
#include <math.h>
#include "Pose.h"
#include "LQR.h"
using namespace Eigen;


bool LQRControl::Calcu_K(Matrix<double, 3, 3>& A, Matrix<double, 3, 2>& B)
{
	int iter_max = 500;
	double epsilon = 0.01;

	//
	Matrix3d P_old = Q;
	Matrix3d P_new;
	try
	{
		for (int i = 0; i < iter_max; ++i)
		{
			P_new = A.transpose() * P_old * A - (A.transpose() * P_old * B) * ((R + B.transpose() * P_old * B).inverse()) * (B.transpose() * P_old * A) + Q;
			if (cmpEpsl(P_new - P_old, epsilon)) break;
			else P_old = P_new;
		}

		P = P_new;
		K = (B.transpose() * P * B + R).inverse() * (B.transpose() * P * A);
	}
	catch(std::runtime_error& e)
	{
		std::cout << e.what() << std::endl;
		return false;
	}
	return true;
}

void LQRControl::actOnce(Pose& p)
{
	Vector3d X = Vector3d::Zero();
	X(0) = p.pos_x - p.refpos.ref_x[p.refpos.idx];
	X(1) = p.pos_y - p.refpos.ref_y[p.refpos.idx];
	X(2) = angle_lmt(p.pos_yaw - p.refpos.ref_yaw[p.refpos.idx]);
	Matrix3d A = Matrix3d::Identity();
	A(0, 2) = -p.para.ref_speed * p.para.dt * sin(p.refpos.ref_yaw[p.refpos.idx]);
	A(1,2)= p.para.ref_speed * p.para.dt * cos(p.refpos.ref_yaw[p.refpos.idx]);
	Matrix<double, 3, 2> B = Matrix<double, 3, 2>::Zero();   //一定要初始化！！！！！！！！！！！！！！！！！！！！！
	B(0, 0) = p.para.dt * cos(p.refpos.ref_yaw[p.refpos.idx]);
	B(1, 0) = p.para.dt * sin(p.refpos.ref_yaw[p.refpos.idx]);
	B(2, 0) = p.para.dt * tan(p.para.ref_delta) / p.para.L;
	B(2, 1) = p.para.ref_speed * p.para.dt / (p.para.L * pow(cos(p.para.ref_delta), 2));
	if (!Calcu_K(A, B))
	{
		cout << "ERROR OCCURED!" << endl;
	}
	U= -K * X;
	U(1) = angle_lmt(U(1));
}

bool LQRControl::cmpEpsl(const Matrix3d&& P, double epsilon)
{
	for (int i = 0; i < 3; ++i)
	{
		for (int j = 0; j < 3; ++j)
		{
			if (P(i, j) > epsilon) return false;
		}
	}
	return true;
}

double LQRControl::angle_lmt(double angle_in)
{
	if (angle_in > M_PI) return angle_in - 2 * M_PI;
	else if (angle_in < -M_PI) return angle_in + 2 * M_PI;
	else return angle_in;
}
