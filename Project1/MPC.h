#pragma once
#include <iostream>
#include <Eigen/Dense>
#define _USE_MATH_DEFINES
#include <math.h>
#include "Pose.h"

using namespace std;
using namespace Eigen;



template <int Nx = 3,int Nu = 2,int Np = 60,int Nc = 30>
class MPCControl
{
public:
	MPCControl();
	void actOnce(Pose& p);

public:
	double row;
	Matrix<double, Np* Nx, Np* Nx> Q;
	Matrix<double, Nc* Nu, Nc* Nu> R;
	Matrix<double, Nu, 1> U;
	Matrix<double, Nu, 1> umin;
	Matrix<double, Nu, 1> umax;
	Matrix<double, Nu, 1> delta_umin;
	Matrix<double, Nu, 1> delta_umax;

private:
	template<int a_n,int a_m,int b_n,int b_m, int r_n,int r_m>
	Matrix<double,r_n,r_m> kron(Matrix<double,a_n,a_m>& a, Matrix<double,b_n,b_m>& b);
};