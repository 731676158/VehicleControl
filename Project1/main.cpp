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
	//����refpos�Ļص������Ϳ���ʱ��
	function<double(double)> fct = [](double x) {return 4 / (exp(-0.1 * (x - 50)) + 1); };
	pos.refpos.setfun(fct, control_time);

	//��Ųο��켣��ÿһʱ�̵Ĺ켣������ͼʹ��
	vector<Point> actual_pos(pos.refpos.nums);
	vector<Point> ref_pos(pos.refpos.nums);

	//��ʼѭ��
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

	//��ͼ
	graph2d g2d(700, 590, { 0,-5 }, { 125,5 });
	g2d.xlabel("x��");
	g2d.ylabel("y��");
	g2d.title("LQR�������");
	g2d.plot(actual_pos, BLUE);
	g2d.plot(ref_pos, RED);
	g2d.waitKey();
}

void MPCtest(int control_time)
{

}