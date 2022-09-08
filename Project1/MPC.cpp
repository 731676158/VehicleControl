#include "MPC.h"


template <int Nx, int Nu, int Np, int Nc>
MPCControl<Nx, Nu, Np, Nc>::MPCControl()
{
	row = 10;
	Q = Matrix<double, Np* Nx, Np* Nx>::Identity() * 100;
	R = Matrix<double, Nc* Nu, Nc* Nu>::Identity();
	U << 0, 0;
	umin << -0.2, -0.54;
	umax << 0.2, 0.332;
	delta_umin << -0.05, -0.64;
	delta_umax << 0.05, 0.64;
}

template <int Nx, int Nu, int Np, int Nc>
void MPCControl<Nx, Nu, Np, Nc>::actOnce(Pose& p)
{
	/************原运动学误差状态空间方程的相关矩阵计算**************/
	p.refpos.refposition(p.pos_x, p.pos_y);
	p.para.ref_delta = atan(p.para.L * p.refpos.ref_pos_k[p.refpos.idx]);
	//矩阵计算
	Matrix<double, 3, 1> X = Matrix<double, 3, 1>::Zero();
	X(0, 0) = p.pos_x - p.refpos.ref_x[p.refpos.idx];
	X(1, 0) = p.pos_y - p.refpos.ref_y[p.refpos.idx];
	X(2, 0) = angle_lmt(p.pos_yaw - p.refpos.ref_yaw[p.refpos.idx]);
	Matrix3d a = Matrix3d::Identity();
	a(0, 2) = -p.para.ref_speed * p.para.dt * sin(p.refpos.ref_yaw[p.refpos.idx]);
	a(1, 2) = p.para.ref_speed * p.para.dt * cos(p.refpos.ref_yaw[p.refpos.idx]);
	Matrix<double, 3, 2> b = Matrix<double, 3, 2>::Zero();   //一定要初始化！！！！！！！！！！！！！！！！！！！！！
	b(0, 0) = p.para.dt * cos(p.refpos.ref_yaw[p.refpos.idx]);
	b(1, 0) = p.para.dt * sin(p.refpos.ref_yaw[p.refpos.idx]);
	b(2, 0) = p.para.dt * tan(p.para.ref_delta) / p.para.L;
	b(2, 1) = p.para.ref_speed * p.para.dt / (p.para.L * pow(cos(p.para.ref_delta), 2));

	/*************新的状态空间方程的相关矩阵*****************************/
	//涉及到切块、vector转matrix的操作
	Matrix<double, Nx + Nu, 1> kesi;
	kesi.block < Nx, 1 >(0, 0) = X;
	kesi.block<Nu, 1>(Nx, 0) = U;

	Matrix<double, Nx + Nu, Nx + Nu> A;
	A.block<Nx, Nx>(0, 0) = a;
	A.block<Nx, Nu>(0, Nx) = b;
	A.block<Nu, Nx>(Nx, 0) = Matrix<double, Nu, Nx>::Zero();
	A.block<Nu, Nu>(Nx, Nx) = Matrix<double, Nu, Nu>::Identity();

	Matrix<double, Nx + Nu, Nu> B;
	B.block<Nx, Nu>(0, 0) = b;
	B.block<Nu, Nu>(Nx, 0) = Matrix<double, Nu, Nu>::Identity();

	Matrix<double, Nx, Nx + Nu> C;
	C.block<Nx, Nx>(0, 0) = Matrix<double, Nx, Nx>::Identity();
	C.block<Nx, Nu>(0, Nx) = Matrix<double, Nx, Nu>::Zero();

	Matrix<double, Nx * Np, Nx + Nu> PHI;
	for (int i = 0; i < Np; ++i)
	{
		PHI.block<Nx, Nx + Nu>(i * Nx, 0) = C * (pow(A, i));
	}

	Matrix<double, Nx* Np, Nc* Nu> THETA;
	for (int i = 0; i < Np; ++i)
	{
		for (int j = 0; j < Nc; ++j)
		{
			if (j <= i)
			{
				THETA.block<Nx, Nu>(i * Nx, j * Nu) = C * (pow(A, (i - j)) * B);
			}
			else
			{
				THETA.block<Nx, Nu>(i * Nx, j * Nu) = Matrix<double, Nx, Nu>::Zero();
			}
		}
	}

	/******************二次型相关********************/
	Matrix<double, Nc* Nu + 1, Nc* Nu + 1> H;
	H.block<Nc* Nu, Nc* Nu>(0, 0) = THETA.transpose() * Q * THETA + R;
	H.block<Nc* Nu, 1>(0, Nc * Nu) = Matrix<double, Nc* Nu, 1>::Zero();
	H.block<1, Nc* Nu>(Nc * Nu, 0) = Matrix<double, 1, Nc* Nu>::Zero();
	H.block<1, 1>(Nc * Nu, Nc * Nu) = row;

	Matrix<double, Nx* Np, 1> E;
	E = PHI * kesi;

	Matrix<double, 1, Nc* Nu + 1> g;
	g.block<1, Nc* Nu>(0, 0) = E.transpose() * Q * THETA;
	g.block<1, 1>(0, Nc * Nu) = 0;

	/******************约束相关**********************/
	Matrix<double, Nc, Nc> A_t = Matrix<double, Nc, Nc>::Zero();
	for (int i = 0; i < Nc; ++i)
	{
		for (int j = 0; j <= i; ++j)
		{
			A_t(i, j) = 1;
		}
	}
	Matrix<double, Nc* Nu, Nc* Nu> A_I = kron<Nc, Nc, Nu, Nu, Nc * Nu, Nc * Nu>(A_t, Matrix<double, Nu, Nu>::Identity());

	Matrix<double, Nc* Nu, 1> U_t = kron<Nc, 1, Nu, 1, Nc* Nu, 1>(Matrix<double, Nc, 1>::Ones(), U);

	//约束
	Matrix<double, Nc* Nu, 1> U_min = kron<Nc, 1, Nu, 1, Nc* Nu, 1>(Matrix<double, Nc, 1>::Ones(), umin);
	Matrix<double, Nc* Nu, 1> U_max = kron<Nc, 1, Nu, 1, Nc* Nu, 1>(Matrix<double, Nc, 1>::Ones(), umax);
	Matrix<double, Nc* Nu, 1> d_U_min = kron<Nc, 1, Nu, 1, Nc* Nu, 1>(Matrix<double, Nc, 1>::Ones(), delta_umin);
	Matrix<double, Nc* Nu, 1> d_U_max = kron<Nc, 1, Nu, 1, Nc* Nu, 1>(Matrix<double, Nc, 1>::Ones(), delta_umax);

	//不等式约束的矩阵A和b
	Matrix<double, 2 * Nc * Nu, Nc* Nu + 1> A_cons;
	A_cons.block<Nc* Nu, Nc* Nu>(0, 0) = A_I;
	A_cons.block<Nc* Nu, 1>(0, Nc * Nu) = Matrix<double, Nc* Nu, 1>::Zero();
	A_cons.block<Nc* Nu, Nc* Nu>(Nc * Nu, 0) = -A_I;
	A_cons.block<Nc* Nu, 1>(Nc * Nu, Nc * Nu) = Matrix<double, Nc* Nu, 1>::Zero();

	//?
	Matrix<double, 2 * Nc * Nu, 1> b_cons;
	b_cons.block<Nc* Nu, 1>(0, 0) = U_max - U_t;
	b_cons.block<Nc* Nu, 1>(Nc * Nu, 0) = -U_min + U_t;

	Matrix<double, Nc* Nu + 1, 1> lb;
	lb.block<Nc* Nu, 1>(0, 0) = d_U_min;
	lb.block<1, 1>(Nc * Nu, 0) = 0;

	Matrix<double, Nc* Nu + 1, 1>ub;
	ub.block<Nc* Nu, 1>(0, 0) = d_U_max;
	ub.block<1, 1>(Nc * Nu, 0) = 1;

	//开始求解

}

template <int Nx, int Nu, int Np, int Nc>
template<int a_n, int a_m, int b_n, int b_m, int r_n, int r_m>
Matrix<double, r_n, r_m> MPCControl<Nx, Nu, Np, Nc>::kron(Matrix<double, a_n, a_m>& a, Matrix<double, b_n, b_m>& b)
{
	Matrix<double, r_n, r_m> res;
	for (int i = 0; i < a_n; ++i)
	{
		for (int j = 0; j < a_m; ++j)
		{
			res.block<b_n, b_m>(i * b_n, j * b_m) = a(i, j) * b;
		}
	}
	return res;
}