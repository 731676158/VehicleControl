#pragma once
#include <iostream>
#include <graphics.h>
#include <conio.h>
#include <vector>
#include <math.h>
#define f(_lambda) ([](double x) {return (_lambda); })

typedef struct mypoint {
	double x;
	double y;
}Point;
class graph2d
{
private:
	double height;											// ����߶�
	double width;											// ������
	Point pointlb;											// ���������½ǵĵ�
	Point pointrt;											// ���������Ͻǵĵ�
	int x_len;												// x���ֵĿ��
	int y_len;												// y���ֵĿ��
public:
	/*********************************
	 *
	 *	��ʼ��������ת������������ȵ�
	 *
	 *********************************/
	graph2d();												// ��ʼ��ΪĬ��ֵ
	graph2d(double _width, double _height);					// ��ʼ��������
	graph2d(double _width, double _height, Point _pointlb, Point _pointrt);		// ��ʼ�������߼�����
	~graph2d();												// ��������
	void waitKey(int _delay = 0);							// �ȴ��ر�
private:
	/*********************************
	 *
	 *	����ת������������ȵ�
	 *
	 *********************************/
	wchar_t* ctow(const char* str);							// char* to wchar_t*
	void setlen(int _len = 3);								// ���������ı�����
	std::string dtos(double _num, char _axis);				// string to double
	void setGrid(Point _pointlb, Point _pointrt);			// ��������
	void drawGrid();										// ��������
	void drawAxisX();										// ����X��
	void drawAxisY();										// ����Y��
	bool isBorder(Point _point);							// �Ƿ��ڱ߽���
	int numConversion(double _num, char _axis);				// ��y��ߵ�
	Point fucCSDataToAbsCSData(Point _point);				// ���̵ĵ�ת��������ĵ�
	Point absCSDataToFucCSData(Point _point);				// ����ĵ�ת�������̵ĵ�
	void showError(std::string _err);						// ��ʾ����
	void drawRectangle(Point _pointlb, Point _pointrt, COLORREF _colorl, COLORREF _colorf, int _style = BS_SOLID);	//��������
	void setBackgroundColor(COLORREF _color = 0xEAEAEA);	// ���û��屳����ɫ
	void setAxisColor();									// ��������ϵ������ɫ
	void initAxis();										// ��ʼ���������ڵ���Ϣ
	void mouseClick();										// ���������ȡ�õ㺯������
public:
	/*********************************
	 *
	 *	�������귽�̺���
	 *
	 *********************************/
	void plot(Point _point, COLORREF _color = RED, int _size = 3, int _type = BS_SOLID);		// ���Ƶ�
	void plot(std::vector<Point> _point, COLORREF _color = BLACK, int _thickness = 3, int _type = PS_SOLID);		// ����һ��������
	void plot(double _start, double _end, double (*_f)(double), double _step = 0.1, COLORREF _color = BLACK, int _thickness = 3, int _type = PS_SOLID);		// ����f(x)����
	void title(std::string _str);							// ����
	void xlabel(std::string _str);							// x���ı�ע��
	void ylabel(std::string _str);							// y���ı�ע��,�����ÿո���
	void text(std::string _str);							// �����������ע��
	void legend(std::string _str, COLORREF _color, int _num);// ��ǩ
};

