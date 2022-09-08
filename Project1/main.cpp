#include <iostream>
#include "LQR.h"
#include "Pose.h"
#include"graph2d.h"
using namespace std;

void LQRtest(int control_time);

int main()
{
	LQRtest(100);


}

void LQRtest(int control_time)
{
	Pose pos;
	LQRControl ctrl;
	//给定refpos的回调函数和控制时间
	function<double(double)> fct = [](double x) {return 4 / (exp(-0.1 * (x - 50)) + 1); };
	pos.refpos.setfun(fct, control_time);

	//存放参考轨迹与每一时刻的轨迹，供画图使用
	vector<Point> actual_pos(pos.refpos.nums);
	vector<Point> ref_pos(pos.refpos.nums);

	//开始循环
	for (int i = 0; i < control_time; ++i)
	{
		pos.refpos.refposition(pos.pos_x, pos.pos_y);
		ctrl.actOnce(pos);
		pos.update(pos.para.dt, ctrl.U);

		actual_pos[i].x = pos.pos_x;
		actual_pos[i].y = pos.pos_y;
		ref_pos[i].x = pos.refpos.ref_x[i];
		ref_pos[i].y = pos.refpos.ref_y[i];
	}

	//画图
	graph2d g2d(700, 590, { 0,-5 }, { 125,5 });
	g2d.xlabel("x轴");
	g2d.ylabel("y轴");
	g2d.title("LQR控制情况");
	g2d.plot(actual_pos, BLUE);
	g2d.plot(ref_pos, RED);
	g2d.waitKey();
}

void MPCtest(int control_time)
{

}