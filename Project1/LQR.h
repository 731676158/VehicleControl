#pragma once
#include <iostream>
#include <Eigen/Dense>
#include "Pose.h"

using namespace Eigen;

class LQRControl
{
public:

	LQRControl():Q(Matrix3d::Identity()),R(Matrix2d::Identity()*2){}
	bool Calcu_K(Matrix<double, 3, 3>& A, Matrix<double, 3, 2>& B);
	void actOnce(Pose& p);

private:
	bool cmpEpsl(const Matrix3d&& P, double epsilon);
	double angle_lmt(double angle_in);

public:
	Matrix<double, 2, 3> K;
	Matrix3d Q;
	Matrix2d R;
	Vector2d U;
private:
	Matrix3d P;

};