#pragma once
#include "FeatureNetwork.h"
//#include "SplineVolume.h"
#include "NurbsTrans.h"
#include "Option.h"

//新数据结构头文件
#include "WingStruct.h"
#include "Nurbs.h"
#include <time.h>
//using namespace YN;

//！三角网格模型处理头文件
#include <iomanip>
#include <cmath>
#include "Mesh.h"
#include "FormTrait.h"
#include "LSCM.h"
#include <iostream>

/*
 *倍数
 *进行曲面展开时，得到的平面尺寸过小，会影响剖分
 *故需要先放大图形，然后在进行曲面映射时再缩小图形
 */
#define multiple 100
using namespace std;
using namespace MeshLib;

//线(包括圆弧，用于四边剖分)
class Spline0 {
public:
	Vec4 v1, v2, v3;

	Spline0() {};
	Spline0(Vec4& v1, Vec4& v2) {
		this->v1 = v1;
		this->v2 = v2;
	};
	Spline0(Vec4& v1, Vec4& v2, Vec4& v3) {
		this->v1 = v1;
		this->v2 = v2;
		this->v3 = v3;
	};
	~Spline0() {
	}

	Spline getSpline() {
		Spline SL;
		varray<double> knots;
		knots.push_back(0);
		knots.push_back(0);
		knots.push_back(0);
		knots.push_back(1);
		knots.push_back(1);
		knots.push_back(1);
		SL.m_Knots = knots;
		SL.m_Degree = 2;

		SL.m_CtrlPts.push_back(v1);
		SL.m_CtrlPts.push_back((v1 + v2) / 2);
		SL.m_CtrlPts.push_back(v2);

		return SL;
	}

	//线段（参数：两个点坐标）
	Spline getSpline(Vec4 v1, Vec4 v2) {
		Spline SL;
		varray<double> knots;
		knots.push_back(0);
		knots.push_back(0);
		knots.push_back(0);
		knots.push_back(1);
		knots.push_back(1);
		knots.push_back(1);
		SL.m_Knots = knots;
		SL.m_Degree = 2;

		SL.m_CtrlPts.push_back(v1);
		SL.m_CtrlPts.push_back((v1 + v2) / 2);
		SL.m_CtrlPts.push_back(v2);

		return SL;
	}

	Spline getSpline(Vec3& v1, Vec3& v2) {
		Vec4 p1 = v1;
		Vec4 p2 = v2;
		p1.w = 1;
		p2.w = 1;
		Spline SL;
		varray<double> knots;
		knots.push_back(0);
		knots.push_back(0);
		knots.push_back(0);
		knots.push_back(1);
		knots.push_back(1);
		knots.push_back(1);
		SL.m_Knots = knots;
		SL.m_Degree = 2;

		SL.m_CtrlPts.push_back(p1);
		SL.m_CtrlPts.push_back((p1 + p2) / 2);
		SL.m_CtrlPts.push_back(p2);

		return SL;
	}

	//弧线段（参数：三个点坐标）
	Spline getSpline(Vec4 v1, Vec4 v3, Vec4 v2) {
		Spline SL;
		varray<double> knots;
		knots.push_back(0);
		knots.push_back(0);
		knots.push_back(0);
		knots.push_back(1);
		knots.push_back(1);
		knots.push_back(1);
		SL.m_Knots = knots;
		SL.m_Degree = 2;

		SL.m_CtrlPts.push_back(v1);
		SL.m_CtrlPts.push_back(v3);
		SL.m_CtrlPts.push_back(v2);

		return SL;
	}

	//圆弧（参数：三个点坐标 默认九十度）
	/*Spline getArcSpline(Vec4 v1, Vec4 v2, Vec4 v3) {
		Spline SL;
		varray<double> knots;
		knots.push_back(0);
		knots.push_back(0);
		knots.push_back(0);
		knots.push_back(1);
		knots.push_back(1);
		knots.push_back(1);
		SL.m_Knots = knots;
		SL.m_Degree = 2;

		SL.m_CtrlPts.push_back(v1);
		SL.m_CtrlPts.push_back(v2);
		SL.m_CtrlPts.push_back(v3);

		return SL;
	}*/

	//圆弧（参数：半径、角度、首末控制点）圆心为坐标原点
	Spline getArcSpline(double r, double angle, Vec4 v1, Vec4 v3) {
		double l1 = r / cos(angle / 2);//中间控制点到圆心的距离
		double w = cos(angle / 2);//权重
		Vec4 v = v1.Normalize();
		Vec3 v2 = v.RotateZ(-angle / 2);//v2单位向量
		Vec4 p02;
		p02.x = v2.x * l1;
		p02.y = v2.y * l1;
		p02.z = v2.z * l1;
		p02.w = w;
		Spline SL;
		varray<double> knots;
		knots.push_back(0);
		knots.push_back(0);
		knots.push_back(0);
		knots.push_back(1);
		knots.push_back(1);
		knots.push_back(1);
		SL.m_Knots = knots;
		SL.m_Degree = 2;

		SL.m_CtrlPts.push_back(v1);
		SL.m_CtrlPts.push_back(p02);
		SL.m_CtrlPts.push_back(v3);

		return SL;
	}

	/**
	* @brief   : 给定三点，求夹角
	* @param[I]: Vec4 p1
	* @param[I]: Vec4 p2 交点(中间点)
	* @param[I]: Vec4 p3
	* @param[0]: none
	* @return  : double 夹角的弧度
	* @note    :
	**/
	double calculateAngle3D(const Vec4& p01, const Vec4& p02, const Vec4& p03)
	{
		Vec4 s1 = p01 - p02;
		Vec4 s2 = p03 - p02;

		double l1 = sqrt(pow(s1.x, 2) + pow(s1.y, 2) + pow(s1.z, 2));
		double l2 = sqrt(pow(s2.x, 2) + pow(s2.y, 2) + pow(s2.z, 2));

		double d = s1.x * s2.x + s1.y * s2.y + s1.z * s2.z;

		double angle = acos(d / (l1 * l2));
		return angle;
	}

	/**
	* @brief   : 给出圆弧的两个端点，以及圆心，构建一段圆弧
	* @param[I]: 端点1
	* @param[I]: 端点2
	* @param[I]: 圆心
	* @param[0]: none
	* @return  : 最终构建的曲线
	* @note    :
	**/
	Spline getArcSpline(Vec4 v1, Vec4 v3, Vec4 O)
	{
		Spline s;
		//方向向量
		Vec4 op = (v1 + v3) / 2 - O;

		//获得向量的长度
		double lop = sqrt(pow(op.x, 2) + pow(op.y, 2) + pow(op.z, 2));

		//向量单位化
		op /= lop;

		//计算曲线的弧度
		double angle = calculateAngle3D(v1, O, v3);
		//cout << "夹角为：" << angle / PI * 180 << endl;

		//计算某一端点到圆心的长度
		double l1 = sqrt(pow(v1.x - O.x, 2) + pow(v1.y - O.y, 2) + pow(v1.z - O.z, 2));

		//计算中间控制点到圆心的距离
		double l2 = l1 / cos(angle / 2);

		//方向向量与长度相乘，得到等长向量
		op *= l2;

		//获得中间控制点坐标
		op += O;

		//设置权重
		op.w = cos(angle / 2);

		varray<double> knots;
		knots.push_back(0);
		knots.push_back(0);
		knots.push_back(0);
		knots.push_back(1);
		knots.push_back(1);
		knots.push_back(1);
		s.m_Knots = knots;
		s.m_Degree = 2;

		s.m_CtrlPts.push_back(v1);
		s.m_CtrlPts.push_back(op);
		s.m_CtrlPts.push_back(v3);

		return s;
	}

	//圆（参数：圆半径）
	varray<Spline> arcSplines(double r) {
		Model_Solution m;
		varray<Spline> SL;
		double w = cos(PI / 4);
		Spline sl;
		Vec4 v1 = { -r,0,0,1 };
		Vec4 v2 = { -r,r,0,w };
		Vec4 v3 = { 0,r,0,1 };
		sl = getArcSpline(v1, v2, v3);
		SL.push_back(sl);
		m.Rolate(sl, PI / 2, 3);
		SL.push_back(sl);
		m.Rolate(sl, PI / 2, 3);
		SL.push_back(sl);
		m.Rolate(sl, PI / 2, 3);
		SL.push_back(sl);

		return SL;
	}

	//长方形（参数：长 宽 默认左下角为原点）
	varray<Spline> recSplines(double a, double b) {
		double tmp;
		if (a > b) {
			tmp = a;
			a = b;
			b = tmp;
		}
		Model_Solution m;
		varray<Spline> SL;
		Spline sl;
		Vec4 v1 = { 0,0,0,1 };
		Vec4 v2 = { 0,a,0,1 };
		Vec4 v3 = { b,a,0,1 };
		Vec4 v4 = { b,0,0,1 };
		sl = getSpline(v1, v2);
		SL.push_back(sl);
		sl = getSpline(v2, v3);
		SL.push_back(sl);
		sl = getSpline(v3, v4);
		SL.push_back(sl);
		sl = getSpline(v4, v1);
		SL.push_back(sl);

		return SL;
	}
};

//任意角度圆弧
class Circle0 {
public:
	Circle0() {};

	//输入半径和角度
	Circle0(double r, double a) :r(r), a(a) {};

	Vec4 getP1() {
		Vec4 p01;
		p01.x = v0.x * l0;
		p01.y = v0.y * l0;
		p01.z = v0.z * l0;
		p01.w = 1;
		return p01;
	}
	Vec4 getP2() {
		Vec4 p02;
		p02.x = v1.x * l1;
		p02.y = v1.y * l1;
		p02.z = v1.z * l1;
		p02.w = w;
		return p02;
	}
	Vec4 getP3() {
		Vec4 p03;
		p03.x = v2.x * l2;
		p03.y = v2.y * l2;
		p03.z = v2.z * l2;
		p03.w = 1;
		return p03;
	}

	Spline getCircle() {
		Spline SL;
		varray<double> knots;
		knots.push_back(0);
		knots.push_back(0);
		knots.push_back(0);
		knots.push_back(1);
		knots.push_back(1);
		knots.push_back(1);
		SL.m_Knots = knots;
		SL.m_Degree = 2;
		p01.x = v0.x * l0;
		p01.y = v0.y * l0;
		p01.z = v0.z * l0;
		p01.w = 1;
		p02.x = v1.x * l1;
		p02.y = v1.y * l1;
		p02.z = v1.z * l1;
		p02.w = w;
		p03.x = v2.x * l2;
		p03.y = v2.y * l2;
		p03.z = v2.z * l2;
		p03.w = 1;
		SL.m_CtrlPts.push_back(p01);
		SL.m_CtrlPts.push_back(p02);
		SL.m_CtrlPts.push_back(p03);

		return SL;
	}

public:
	double r;//半径
	double a;//角度
	Vec4 p01, p02, p03;//控制点
private:
	double a1 = a / 2;
	double a2 = a / 2;

	double l0 = r;
	double l1 = r / cos(a1);
	double l2 = r;
	double w = cos(a / 2);//权重
	Vec3 v1 = { 0,1,0 };//+y轴单位向量
	Vec3 v0 = v1.RotateZ(-a / 2);
	Vec3 v2 = v1.RotateZ(a / 2);
};

//任意角度圆环
class Annulus0 {
public:
	double r1;//内径
	double r2;//外径
	double a;//角度

	Annulus0() {};

	Annulus0(double r1, double r2, double a) :r1(r1), r2(r2), a(a) {}

	SplineSurface getSurface() {
		varray<Spline> SL1;
		varray<double> knots;
		knots.push_back(0);
		knots.push_back(0);
		knots.push_back(0);
		knots.push_back(1);
		knots.push_back(1);
		knots.push_back(1);
		SL1.resize(4);
		for (int i = 0; i < 4; i++) {
			SL1[i].m_Degree = 2;
			SL1[i].m_Knots = knots;
		}

		Circle0 c1(r1, a), c2(r2, a);
		/*Spline s1, s2;
		s1 = c1.getCircle();
		s2 = c2.getCircle();*/

		Vec4 p01 = c1.getP1();
		Vec4 p02 = c2.getP1();

		Vec4 p03 = c1.getP3();
		Vec4 p04 = c2.getP3();

		SL1[0].m_CtrlPts.push_back(c1.getP1());
		SL1[0].m_CtrlPts.push_back(c1.getP2());
		SL1[0].m_CtrlPts.push_back(c1.getP3());

		SL1[1].m_CtrlPts.push_back(p01);
		SL1[1].m_CtrlPts.push_back((p02 + p01) / 2);
		SL1[1].m_CtrlPts.push_back(p02);

		SL1[2].m_CtrlPts.push_back(c2.getP1());
		SL1[2].m_CtrlPts.push_back(c2.getP2());
		SL1[2].m_CtrlPts.push_back(c2.getP3());

		SL1[3].m_CtrlPts.push_back(p03);
		SL1[3].m_CtrlPts.push_back((p03 + p04) / 2);
		SL1[3].m_CtrlPts.push_back(p04);

		SplineSurface ss1;
		ss1.CoonsInterpolate(SL1);

		return ss1;
	}
};

//长方形
class Rectangle0 {
public:
	double l;//长
	double h;//宽
	Vec4 v1, v2, v3, v4;

	Rectangle0() {};
	Rectangle0(double l, double h) :l(l), h(h) {
		Vec4 p1 = { -l / 2,-h / 2,0,1 };
		Vec4 p2 = { l / 2,-h / 2,0,1 };
		Vec4 p3 = { -l / 2,h / 2,0,1 };
		Vec4 p4 = { l / 2,h / 2,0,1 };

		this->v1 = p1;
		this->v2 = p2;
		this->v3 = p3;
		this->v4 = p4;
	};

	//由长和宽得到长方形
	varray<Spline> getRectangle() {
		varray<Spline> SL;
		SL.resize(4);
		varray<double> knots;
		knots.push_back(0);
		knots.push_back(0);
		knots.push_back(0);
		knots.push_back(1);
		knots.push_back(1);
		knots.push_back(1);
		for (int i = 0; i < SL.size(); i++) {
			SL[i].m_Knots = knots;
			SL[i].m_Degree = 2;
		}
		SL[0].m_CtrlPts.push_back(v1);
		SL[0].m_CtrlPts.push_back((v1 + v2) / 2);
		SL[0].m_CtrlPts.push_back(v2);

		SL[1].m_CtrlPts.push_back(v1);
		SL[1].m_CtrlPts.push_back((v1 + v3) / 2);
		SL[1].m_CtrlPts.push_back(v3);

		SL[2].m_CtrlPts.push_back(v3);
		SL[2].m_CtrlPts.push_back((v3 + v4) / 2);
		SL[2].m_CtrlPts.push_back(v4);

		SL[3].m_CtrlPts.push_back(v2);
		SL[3].m_CtrlPts.push_back((v4 + v2) / 2);
		SL[3].m_CtrlPts.push_back(v4);

		return SL;
	}

	//由四点形成长方形（剖分算法中使用，按一个方向传参）
	varray<Spline> getRectangle(Vec4 v1, Vec4 v2, Vec4 v3, Vec4 v4) {
		varray<Spline> SL;
		SL.resize(4);
		varray<double> knots;
		knots.push_back(0);
		knots.push_back(0);
		knots.push_back(0);
		knots.push_back(1);
		knots.push_back(1);
		knots.push_back(1);
		for (int i = 0; i < SL.size(); i++) {
			SL[i].m_Knots = knots;
			SL[i].m_Degree = 2;
		}
		SL[0].m_CtrlPts.push_back(v1);
		SL[0].m_CtrlPts.push_back((v1 + v2) / 2);
		SL[0].m_CtrlPts.push_back(v2);

		SL[1].m_CtrlPts.push_back(v2);
		SL[1].m_CtrlPts.push_back((v2 + v3) / 2);
		SL[1].m_CtrlPts.push_back(v3);

		SL[2].m_CtrlPts.push_back(v3);
		SL[2].m_CtrlPts.push_back((v3 + v4) / 2);
		SL[2].m_CtrlPts.push_back(v4);

		SL[3].m_CtrlPts.push_back(v4);
		SL[3].m_CtrlPts.push_back((v4 + v1) / 2);
		SL[3].m_CtrlPts.push_back(v1);

		return SL;
	}

	SplineSurface getSurface() {
		varray<Spline> sl;
		sl = getRectangle();
		SplineSurface ss;
		ss.CoonsInterpolate(sl);
		return ss;
	}
};

//任意形状（目前有：任意四边形、圆、圆环）
class RandomModel {
private:
	double w = cos(PI / 4);
	//任意四边形的四个顶点
	Vec4 v1, v2, v3, v4;

	//圆的半径、圆环内外半径
	double r, r1;

	//四边形
	double a, b;//长宽（小者为宽）
public:
	Model_Solution m;

	RandomModel() {};

	//圆 r:半径
	RandomModel(double r) :r(r) {};

	//长和宽
	RandomModel(double a, double b) :a(a), b(b) {};

	//由四个点得到四边形
	RandomModel(Vec4 v1, Vec4 v2, Vec4 v3, Vec4 v4) :v1(v1), v2(v2), v3(v3), v4(v4) {};

	//四边形(由四个点得到)
	SplineSurface getSurface() {
		varray<Spline> SL1;
		varray<double> knots;
		knots.push_back(0);
		knots.push_back(0);
		knots.push_back(0);
		knots.push_back(1);
		knots.push_back(1);
		knots.push_back(1);
		SL1.resize(4);
		for (int i = 0; i < 4; i++) {
			SL1[i].m_Degree = 2;
			SL1[i].m_Knots = knots;
		}

		SL1[0].m_CtrlPts.push_back(v1);
		SL1[0].m_CtrlPts.push_back((v1 + v4) / 2);
		SL1[0].m_CtrlPts.push_back(v4);
		SL1[1].m_CtrlPts.push_back(v1);
		SL1[1].m_CtrlPts.push_back((v1 + v2) / 2);
		SL1[1].m_CtrlPts.push_back(v2);
		SL1[2].m_CtrlPts.push_back(v2);
		SL1[2].m_CtrlPts.push_back((v2 + v3) / 2);
		SL1[2].m_CtrlPts.push_back(v3);
		SL1[3].m_CtrlPts.push_back(v4);
		SL1[3].m_CtrlPts.push_back((v4 + v3) / 2);
		SL1[3].m_CtrlPts.push_back(v3);
		SplineSurface ss1;
		ss1.CoonsInterpolate(SL1);

		return ss1;
	}

	//四边形（由长宽得到,中心为原点）
	SplineSurface getSurfaceTwo() {
		varray<Spline> SL1;
		varray<double> knots;
		knots.push_back(0);
		knots.push_back(0);
		knots.push_back(0);
		knots.push_back(1);
		knots.push_back(1);
		knots.push_back(1);
		SL1.resize(4);
		for (int i = 0; i < 4; i++) {
			SL1[i].m_Degree = 2;
			SL1[i].m_Knots = knots;
		}
		Vec4 v1 = { -a / 2,-b / 2,0,1 };
		Vec4 v2 = { -a / 2,b / 2,0,1 };
		Vec4 v3 = { a / 2,b / 2,0,1 };
		Vec4 v4 = { a / 2,-b / 2,0,1 };

		SL1[0].m_CtrlPts.push_back(v1);
		SL1[0].m_CtrlPts.push_back((v1 + v4) / 2);
		SL1[0].m_CtrlPts.push_back(v4);
		SL1[1].m_CtrlPts.push_back(v1);
		SL1[1].m_CtrlPts.push_back((v1 + v2) / 2);
		SL1[1].m_CtrlPts.push_back(v2);
		SL1[2].m_CtrlPts.push_back(v2);
		SL1[2].m_CtrlPts.push_back((v2 + v3) / 2);
		SL1[2].m_CtrlPts.push_back(v3);
		SL1[3].m_CtrlPts.push_back(v4);
		SL1[3].m_CtrlPts.push_back((v4 + v3) / 2);
		SL1[3].m_CtrlPts.push_back(v3);
		SplineSurface ss1;
		ss1.CoonsInterpolate(SL1);

		return ss1;
	}

	//获取圆截面
	varray<SplineSurface> getArcSurface() {
		varray<SplineSurface> SS;
		varray<Spline> SL1, SL2;
		varray<double> knots;
		knots.push_back(0);
		knots.push_back(0);
		knots.push_back(0);
		knots.push_back(1);
		knots.push_back(1);
		knots.push_back(1);
		SL1.resize(4);
		SL2.resize(4);
		for (int i = 0; i < 4; i++) {
			SL1[i].m_Degree = 2;
			SL1[i].m_Knots = knots;
			SL2[i].m_Degree = 2;
			SL2[i].m_Knots = knots;
		}

		Vec4 v1 = { -r,0,0,1 };
		Vec4 v2 = { 0,r,0,1 };
		Vec4 v3 = { r,0,0,1 };
		Vec4 v4 = { 0,-r,0,1 };

		Vec4 p1 = { v1.x,v4.y,0,w };
		Vec4 p2 = { v1.x,v2.y,0,w };
		Vec4 p3 = { v3.x,v2.y,0,w };
		Vec4 p4 = { v3.x,v4.y,0,w };

		Vec4 v5 = { -r / 2,0,0,1 };
		Vec4 v6 = { 0,r / 2,0,1 };
		Vec4 v7 = { r / 2,0,0,1 };
		Vec4 v8 = { 0,-r / 2,0,1 };

		SL1[0].m_CtrlPts.push_back(v1);
		SL1[0].m_CtrlPts.push_back((v1 + v5) / 2);
		SL1[0].m_CtrlPts.push_back(v5);
		SL1[1].m_CtrlPts.push_back(v1);
		SL1[1].m_CtrlPts.push_back(p2);
		SL1[1].m_CtrlPts.push_back(v2);
		SL1[2].m_CtrlPts.push_back(v2);
		SL1[2].m_CtrlPts.push_back((v2 + v6) / 2);
		SL1[2].m_CtrlPts.push_back(v6);
		SL1[3].m_CtrlPts.push_back(v5);
		SL1[3].m_CtrlPts.push_back((v6 + v5) / 2);
		SL1[3].m_CtrlPts.push_back(v6);
		SplineSurface ss1, temp;
		ss1.CoonsInterpolate(SL1);
		SS.push_back(ss1);
		temp = ss1;
		Model_Solution m;
		m.Rolate(temp, PI / 2, 3);
		SS.push_back(temp);
		m.Rolate(temp, PI / 2, 3);
		SS.push_back(temp);
		m.Rolate(temp, PI / 2, 3);
		SS.push_back(temp);

		SL2[0].m_CtrlPts.push_back(v5);
		SL2[0].m_CtrlPts.push_back((v8 + v5) / 2);
		SL2[0].m_CtrlPts.push_back(v8);
		SL2[1].m_CtrlPts.push_back(v5);
		SL2[1].m_CtrlPts.push_back((v6 + v5) / 2);
		SL2[1].m_CtrlPts.push_back(v6);
		SL2[2].m_CtrlPts.push_back(v6);
		SL2[2].m_CtrlPts.push_back((v7 + v6) / 2);
		SL2[2].m_CtrlPts.push_back(v7);
		SL2[3].m_CtrlPts.push_back(v8);
		SL2[3].m_CtrlPts.push_back((v8 + v7) / 2);
		SL2[3].m_CtrlPts.push_back(v7);
		SplineSurface ss2;
		ss2.CoonsInterpolate(SL2);
		SS.push_back(ss2);
		return SS;
	}

	//获取四边形+内空圆
	varray<SplineSurface> getRecArcSurface();

	//圆环(参数：内径、外径)
	varray<SplineSurface> getAnnulus(double r, double r1) {
		varray<SplineSurface> SS;
		SplineSurface ss;

		Annulus0 annu(r, r1, PI / 2);
		ss = annu.getSurface();
		SS.push_back(ss);
		m.Rolate(ss, PI / 2, 3);
		SS.push_back(ss);
		m.Rolate(ss, PI / 2, 3);
		SS.push_back(ss);
		m.Rolate(ss, PI / 2, 3);
		SS.push_back(ss);
		m.Rolate(SS, PI / 4, 3);

		return SS;
	}
};

//连接线连接的两条曲线，以及连接线端点所在的曲线参数值
struct AddLineData
{
	pair<int, double> pa1;
	pair<int, double> pa2;
	Spline s;
};

//连接线对应的轮廓
struct ContourData
{
	AddLineData adl;
	//轮廓线编号
	int contour1;
	int contour2;
};

//公共功能函数类
class PublicSolution {
public:

	//检查曲线的点
	void chackPoint(string path, varray<varray<Spline>>vecSL) {
		varray<Vec4> temp1;
		varray<varray<Vec4>> temp2;
		for (auto& i : vecSL) {
			orderEdges(i);
			for (auto j : i) {
				//为了一个点一个点的看，方便查看曲线的方向
				temp1.clear();
				temp1.push_back(j.m_CtrlPts[0]);
				temp2.push_back(temp1);

				temp1.clear();
				temp1.push_back(j.m_CtrlPts[j.m_CtrlPts.size() - 1]);
				temp2.push_back(temp1);
			}
		}
		RWGeometric rwg;
		rwg.WritePoint(path, temp2);
	}

	void chackSpline(string path, varray<varray<Spline>>vecSL) {
		varray<Spline> chackTemp;
		for (auto i : vecSL) {
			for (auto j : i) {
				chackTemp.push_back(j);
			}
		}
		RWGeometric rwg;
		rwg.WriteSpline(path, chackTemp);
	}
	void chackSpline(string path, varray<Spline>vecSL) {
		RWGeometric rwg;
		rwg.WriteSpline(path, vecSL);
	}

	/**
	 *	施加载荷和力接口
	 *	str1：需要施加约束和力的体模型文件路径（加.txt）
	 *	str2: 输出的分析文件路径（不加.txt）
	 */
	void setWCandWF(const string str1, const string str2)
	{
		string stringPath1 = "E:\\Model\\WcWfFile\\newSurface.txt";//生成面片表示模型，用于查看面片序号
		string stringPath2 = "E:\\Model\\WcWfFile\\WcWf.txt";	  //存放需要施加力跟约束的面片序号（以WC、WF开头，各占一行）
		string stringPath3 = "E:\\Model\\WcWfFile\\WcWfLog.txt";   //存放记录
		RWGeometric rwg;
		Model_Solution M;
		varray<SplineVolume> NVs;
		varray<varray<SplineSurface>> NFs;
		varray<SplineSurface> NF;
		TestBolcks::pList plist;

		rwg.ReadSplineVolume(str1, NVs);
		NFs = M.GetSurfaces(NVs);
		for (auto& SF : NFs) {
			for (auto& i : SF) {
				NF.push_back(i);
			}
		}
		rwg.WriteSplineSurface(stringPath1, NF);
		string str, modelName;
		cout << "散面文件已经生成，请到" << stringPath1 << "查看......" << endl;
		cout << "请输入模型名称：";
		cin >> modelName;
		cout << "将序号放到文件中，是否已记录施加约束和力面的序号信息？";

		varray<int> wc;
		varray<int> wf;
		wc.clear();
		wf.clear();
		while (1) {
			cin >> str;
			if (str == "y" || str == "Y") {
				ifstream wcWfFile;
				string temp = "";
				int flag = 0;
				wcWfFile.open(stringPath2, ios::binary);
				if (!wcWfFile) {
					cout << "文件打开失败!" << endl;
				}
				while (!wcWfFile.eof()) {
					wcWfFile >> temp;
					if (temp == "WC") {
						flag = 1;
						continue;
					}
					else if (temp == "WF") {
						flag = 2;
						continue;
					}
					switch (flag) {
					case 1: {
						int c = atoi(temp.c_str());
						wc.push_back(c);
						break;
					}
					case 2: {
						int f = atoi(temp.c_str());
						wf.push_back(f);
						break;
					}
					default:
						break;
					}
				}
				cout << "约束面个数：" << wc.size() << endl;
				cout << "载荷面个数：" << wf.size() << endl;
				wcWfFile.close();

				//记录模型约束和载荷的数据
				//时间
				time_t tm;
				time(&tm);
				char tmp[64];
				strftime(tmp, sizeof(tmp), "%Y-%m-%d %H:%M:%S", localtime(&tm));
				string str = tmp;
				ofstream wcWfLog;
				wcWfLog.open(stringPath3, ios::binary | ios::app);
				wcWfLog << endl;
				wcWfLog << str.c_str() << "  " << modelName << endl;
				wcWfLog << "WC ";
				for (auto& i : wc) {
					wcWfLog << i << " ";
				}
				wcWfLog << endl;
				wcWfLog << "WF ";
				for (auto& i : wf) {
					wcWfLog << i << " ";
				}
				wcWfLog << endl;

				plist.OutputParaVolumeDataTxt(NVs, str2);

				//施加约束和力
				varray<SplineVolume> NVS;
				rwg.ReadSplineVolume(str1, NVS);
				TestBolcks::pList plt;
				ofstream wcWf;
				varray<varray<SplineSurface>> NS = M.GetSurfaces(NVS);
				varray<SplineSurface> NSf;
				int ssNum = 0;
				for (int i = 0; i < NS.size(); i++)
				{
					for (int j = 0; j < NS[i].size(); j++)
					{
						NSf.push_back(NS[i][j]);
						ssNum++;
					}
				}
				cout << "面的片数：" << ssNum << endl;
				rwg.WriteSplineSurface(str2 + "排序面.txt", NSf);

				wcWf.open(str2 + "ctrlptsIdx.txt", ios::binary | ios::app);

				varray<SplineSurface> WC;
				for (int i = 0; i < wc.size(); i++) {
					WC.push_back(NSf[wc[i]]);
				}
				varray<varray<int>>WCidx = plt.getfaceidx(NVS, WC);
				//plt.showdata();
				int cnum = 0;
				cout << "WC" << " ";
				wcWf << "WC" << " ";
				for (int i = 0; i < WCidx.size(); i++)
				{
					for (int j = 0; j < WCidx[i].size(); j++)
					{
						cnum++;
						cout << WCidx[i][j] << " ";
						wcWf << WCidx[i][j] << " ";
					}
				}
				cout << endl;
				wcWf << endl;
				varray<SplineSurface> WF;
				for (int i = 0; i < wf.size(); i++) {
					WF.push_back(NSf[wf[i]]);
				}
				varray<varray<int>>WFidx = plt.getfaceidx(NVS, WF);
				int fnum = 0;
				cout << "WF" << " ";
				wcWf << "WF" << " ";
				for (int i = 0; i < WFidx.size(); i++)
				{
					for (int j = 0; j < WFidx[i].size(); j++)
					{
						fnum++;
						cout << WFidx[i][j] << " ";
						wcWf << WFidx[i][j] << " ";
					}
				}
				wcWf.close();
				cout << endl;
				cout << "约束控制点数：" << cnum << endl;
				cout << "载荷控制点数：" << fnum << endl;

				wcWfLog << "约束面个数：" << wc.size() << endl;
				wcWfLog << "载荷面个数：" << wf.size() << endl;
				wcWfLog << "约束控制点数：" << cnum << endl;
				wcWfLog << "载荷控制点数：" << fnum << endl;
				wcWfLog.close();

				break;
			}
			else
				cout << "错误输入！请重新输入：";
		}
	}

	//对体控制点添加约束
	void setWC(const string str1, const string str2) {
		TestBolcks::pList plist;
		TestBolcks::pList plt;
		string str;
		RWGeometric rwg;
		Model_Solution M;
		varray<SplineVolume> NVs, NV;
		rwg.ReadSplineVolume(str1, NVs);

		string modelName;
		cout << "输入模型名称：";
		cin >> modelName;

		ifstream wcWfFile;
		string temp = "";
		int flag = 0;
		wcWfFile.open("E:\\kuang_models\\WcWfFile\\WcWf.txt", ios::binary);
		cout << "读取文件中需要添加约束的指定体序号......" << endl;
		if (!wcWfFile) {
			cout << "文件打开失败!" << endl;
		}
		varray<int> wc;			 //保存施加约束的序号
		while (!wcWfFile.eof()) {//若文件最后有换行，会多读一个数
			wcWfFile >> temp;
			if (temp == "WC") {
				flag = 1;
				continue;
			}
			else {
				int c = atoi(temp.c_str());
				cout << "体序号：" << c << endl;
				wc.push_back(c);
			}
		}
		cout << "约束体个数：" << wc.size() << endl;
		cout << "读取成功" << endl;
		wcWfFile.close();

		plt.getVolidx(NVs);
		//对指定体进行全控制点施加约束
		for (int i = 0; i < wc.size(); i++) {
			NV.push_back(NVs[wc[i]]);
		}

		varray<varray<int>> res;//指定体序号
		varray<int>cur_res;
		map<Vec4, int> tidx = plt.ptMap;
		int num = 0;
		for (auto nv : NV) {
			for (auto nvpt : nv.m_CtrlPts)
			{
				if (tidx.find(nvpt) != tidx.end())
				{
					num = tidx[nvpt];
					cur_res.push_back(num);
					tidx.erase(nvpt);
				}
			}
			res.push_back(cur_res);
			cur_res.clear();
		}

		//记录模型约束的数据
		//时间
		time_t tm;
		time(&tm);
		char tmp[64];
		strftime(tmp, sizeof(tmp), "%Y-%m-%d %H:%M:%S", localtime(&tm));
		string strTime = tmp;

		ofstream wcLog;
		wcLog.open("E:\\kuang_models\\WcWfFile\\WcWfLog.txt", ios::binary | ios::app);
		wcLog << endl;
		wcLog << strTime.c_str() << "  " << modelName << endl;
		wcLog << "WC ";
		for (auto& i : wc) {
			wcLog << i << " ";
		}
		wcLog << endl;

		//输入到分析文件中
		ofstream WC;
		WC.open(str2 + "ctrlptsIdx.txt", ios::binary | ios::app);
		cout << "输出控制点序号：" << endl;
		cout << "WC" << " ";
		WC << "WC" << " ";
		int cnum = 0;
		for (int i = 0; i < res.size(); i++) {
			for (auto& j : res[i]) {
				cnum++;
				cout << j << " ";
				WC << j << " ";
			}
		}

		WC.close();
		cout << endl;
		cout << "约束控制点数：" << cnum << endl;
		wcLog << "约束控制点数：" << cnum << endl;
		wcLog.close();
	}

	void setBO(const string str1, const string str2) {
		string stringPath1 = "E:\\kuang_models\\WcWfFile\\newSurface.txt";//生成面片表示模型，用于查看面片序号
		string stringPath2 = "E:\\kuang_models\\WcWfFile\\BO.txt";	  //存放需要施加力跟约束的面片序号（以WC、WF开头，各占一行）
		string stringPath3 = "E:\\kuang_models\\WcWfFile\\WcWfLog.txt";   //存放记录
		TestBolcks::pList plt;
		string str;
		RWGeometric rwg;
		Model_Solution M;
		varray<SplineVolume> NVs, NV;
		varray<varray<SplineSurface>> NFs;
		varray<SplineSurface> NF;
		rwg.ReadSplineVolume(str1, NVs);
		NFs = M.GetSurfaces(NVs);
		for (auto& SF : NFs) {
			for (auto& i : SF) {
				NF.push_back(i);
			}
		}
		rwg.WriteSplineSurface(stringPath1, NF);

		string modelName;
		cout << "散面文件已经生成，请到" << stringPath1 << "查看......" << endl;
		cout << "请输入模型名称：";
		cin >> modelName;
		cout << "将序号放到文件中，是否已记录边界序号信息？";
		while (1) {
			cin >> str;
			if (str == "y" || str == "Y") {
				ifstream wcWfFile;
				string temp = "";
				int flag = 0;
				wcWfFile.open("E:\\kuang_models\\WcWfFile\\BO.txt", ios::binary);
				cout << "读取文件中边界序号......" << endl;
				if (!wcWfFile) {
					cout << "文件打开失败!" << endl;
				}
				varray<int> wc;			 //保存施加约束的序号
				while (!wcWfFile.eof()) {//若文件最后有换行，会多读一个数
					wcWfFile >> temp;
					if (temp == "BO") {
						flag = 1;
						continue;
					}
					else {
						int c = atoi(temp.c_str());
						wc.push_back(c);
					}
				}
				cout << "边界面个数：" << wc.size() << endl;
				cout << "读取成功" << endl;
				wcWfFile.close();

				//记录模型约束的数据
				//时间
				time_t tm;
				time(&tm);
				char tmp[64];
				strftime(tmp, sizeof(tmp), "%Y-%m-%d %H:%M:%S", localtime(&tm));
				string strTime = tmp;
				ofstream wcLog;
				wcLog.open("E:\\kuang_models\\WcWfFile\\WcWfLog.txt", ios::binary | ios::app);
				wcLog << endl;
				wcLog << strTime.c_str() << "  " << modelName << endl;
				wcLog << "BO ";
				for (auto& i : wc) {
					wcLog << i << " ";
				}
				wcLog << endl;

				//！输出分析用的文件
				plt.OutputParaVolumeDataTxt(NVs, str2);

				ofstream wcWf;
				wcWf.open(str2 + "ctrlptsIdx.txt", ios::binary | ios::app);
				varray<SplineSurface> WC;
				for (int i = 0; i < wc.size(); i++) {
					WC.push_back(NF[wc[i]]);
				}
				varray<varray<int>>WCidx = plt.getfaceidx(NVs, WC);
				int cnum = 0;

				for (int i = 0; i < WCidx.size(); i++) {
					cout << "BO" << endl;
					wcWf << "BO" << endl;
					for (int j = 0; j < WCidx[i].size(); j++) {
						cnum++;
						cout << WCidx[i][j] << " ";
						wcWf << WCidx[i][j] << " ";
					}
					cout << endl;
					wcWf << endl;
				}
				cout << endl;
				wcWf << endl;
				wcWf.close();

				cout << endl;
				cout << "约束控制点数：" << cnum << endl;
				wcLog << "约束面个数：" << wc.size() << endl;
				wcLog << "约束控制点数：" << cnum << endl;
				wcLog.close();

				break;
			}
			else {
				cout << "输入有误" << endl;
			}
		}
	}

	//曲面拉伸成体
	SplineVolume stretch(SplineSurface& ss1, double distance, int mode) {
		Model_Solution m;
		RWGeometric rwg;
		SplineSurface ss2;
		ss2 = ss1;
		m.Trans(ss2, distance, mode);

		Spline sl;
		Vec4 v1 = ss1.m_CtrlPts[0];
		Vec4 v2 = ss2.m_CtrlPts[0];
		Spline0 sl0;
		sl = sl0.getSpline(v1, v2);

		varray<double> slknots;
		for (int i = 0; i < ss1.m_uKnots.size(); i++) {
			if (ss1.m_uKnots[i] != 0 && ss1.m_uKnots[i] != 1) {
				slknots.push_back(ss1.m_uKnots[i]);
			}
		}
		sl.KnotsRefine(slknots);

		varray<Spline> SL;
		SL.push_back(sl);
		rwg.WriteSpline("TmpSpline.txt", SL);

		varray<Vec4> cptVecs;
		for (int i = 1; i < sl.m_CtrlPts.size() - 1; i++) {
			SplineSurface ss3;
			ss3 = ss1;
			if (mode == 1 || mode == -1) {
				m.Trans(ss3, sl.m_CtrlPts[i].x - sl.m_CtrlPts[0].x, mode);
			}
			else if (mode == 2 || mode == -2) {
				m.Trans(ss3, sl.m_CtrlPts[i].y - sl.m_CtrlPts[0].y, mode);
			}
			else if (mode == 3 || mode == -3) {
				m.Trans(ss3, sl.m_CtrlPts[i].z - sl.m_CtrlPts[0].z, mode);
			}

			for (auto i : ss3.m_CtrlPts) {
				cptVecs.push_back(i);
			}
		}

		SplineVolume SV;
		varray<double> knots;
		knots = ss1.m_uKnots;
		SV.m_uNum = ss1.m_uNum;//控制点个数
		SV.m_vNum = ss1.m_uNum;
		SV.m_wNum = ss1.m_uNum;
		SV.m_uKnots = knots;
		SV.m_uDegree = ss1.m_uDegree;
		SV.m_vKnots = knots;
		SV.m_vDegree = ss1.m_uDegree;
		SV.m_wKnots = knots;
		SV.m_wDegree = ss1.m_uDegree;

		for (int i = 0; i < ss1.m_CtrlPts.size(); i++) {
			SV.m_CtrlPts.push_back(ss1.m_CtrlPts[i]);
		}
		for (int i = 0; i < cptVecs.size(); i++) {
			SV.m_CtrlPts.push_back(cptVecs[i]);
		}
		for (int i = 0; i < ss2.m_CtrlPts.size(); i++) {
			SV.m_CtrlPts.push_back(ss2.m_CtrlPts[i]);
		}
		varray<SplineVolume> SVT;
		SVT.push_back(SV);
		rwg.WriteSplineVolume("SVT.txt", SVT);
		return SV;
	}

	//多面拉伸成体
	varray<SplineVolume> stretch(varray<SplineSurface> ss1, double distance, int mode) {
		varray<SplineVolume> SV;
		for (int i = 0; i < ss1.size(); i++) {
			SplineVolume sv;
			sv = stretch(ss1[i], distance, mode);
			SV.push_back(sv);
		}
		return SV;
	}

	//两面放样 适用于3*3*3控制点平面之间的放样
	SplineVolume loft(SplineSurface& ss1, SplineSurface& ss2) {
		varray<Vec4> vec, vecTmp;
		for (int i = 0; i < ss1.m_CtrlPts.size(); i++) {
			Vec4 tmp = ss1.m_CtrlPts[i] + ss2.m_CtrlPts[i];
			Vec4 v = tmp / 2;
			vec.push_back(v);
		}
		SplineVolume SV;
		varray<double> knots;

		knots = ss1.m_uKnots;
		SV.m_uNum = ss1.m_uNum;//控制点个数
		SV.m_vNum = ss1.m_uNum;
		SV.m_wNum = ss1.m_uNum;
		SV.m_uKnots = knots;
		SV.m_uDegree = ss1.m_uDegree;
		SV.m_vKnots = knots;
		SV.m_vDegree = ss1.m_uDegree;
		SV.m_wKnots = knots;
		SV.m_wDegree = ss1.m_uDegree;

		for (int i = 0; i < ss1.m_CtrlPts.size(); i++) {
			SV.m_CtrlPts.push_back(ss1.m_CtrlPts[i]);
		}
		for (int i = 0; i < vec.size(); i++) {
			SV.m_CtrlPts.push_back(vec[i]);
		}
		for (int i = 0; i < ss2.m_CtrlPts.size(); i++) {
			SV.m_CtrlPts.push_back(ss2.m_CtrlPts[i]);
		}
		return SV;
	}

	//多面放样 适用于3*3*3控制点平面面之间的放样
	varray<SplineVolume> loft(varray<SplineSurface> ss1, varray<SplineSurface> ss2) {
		varray<SplineVolume> SV;
		for (int i = 0; i < ss1.size(); i++) {
			SplineVolume sv;
			sv = loft(ss1[i], ss2[i]);
			SV.push_back(sv);
		}
		return SV;
	}

	//两面放样，适用于多控制点曲面之间的放样（大于3*3*3）
	SplineVolume loft2(SplineSurface& ss1, SplineSurface& ss2) {
		RWGeometric rwg;
		Spline sl;
		Vec4 v1 = ss1.m_CtrlPts[0];
		Vec4 v2 = ss2.m_CtrlPts[0];
		Spline0 sl0;
		sl = sl0.getSpline(v1, v2);

		varray<double> slknots;
		for (int i = 0; i < ss1.m_uKnots.size(); i++) {
			if (ss1.m_uKnots[i] != 0 && ss1.m_uKnots[i] != 1) {
				slknots.push_back(ss1.m_uKnots[i]);
			}
		}
		sl.KnotsRefine(slknots);

		varray<Spline> SL;
		SL.push_back(sl);
		rwg.WriteSpline("TmpSpline.txt", SL);

		varray<Vec4> cptVecs;
		for (int i = 1; i < sl.m_CtrlPts.size() - 1; i++) {
			//
			double n;
			if (sl.m_CtrlPts[0].x != sl.m_CtrlPts[i].x) {
				n = sl.m_CtrlPts[0].x / sl.m_CtrlPts[i].x;
			}
			else if (sl.m_CtrlPts[0].z != sl.m_CtrlPts[i].z) {
				n = sl.m_CtrlPts[0].z / sl.m_CtrlPts[i].z;
			}
			else {
				n = sl.m_CtrlPts[0].y / sl.m_CtrlPts[i].y;
			}

			SplineSurface ss3;
			ss3 = ss1;
			littler(ss3, n, 2);
			for (auto i : ss3.m_CtrlPts) {
				cptVecs.push_back(i);
			}
		}

		SplineVolume SV;
		varray<double> knots;
		knots = ss1.m_uKnots;
		SV.m_uNum = ss1.m_uNum;//控制点个数
		SV.m_vNum = ss1.m_uNum;
		SV.m_wNum = ss1.m_uNum;
		SV.m_uKnots = knots;
		SV.m_uDegree = ss1.m_uDegree;
		SV.m_vKnots = knots;
		SV.m_vDegree = ss1.m_uDegree;
		SV.m_wKnots = knots;
		SV.m_wDegree = ss1.m_uDegree;

		for (int i = 0; i < ss1.m_CtrlPts.size(); i++) {
			SV.m_CtrlPts.push_back(ss1.m_CtrlPts[i]);
		}
		for (int i = 0; i < cptVecs.size(); i++) {
			SV.m_CtrlPts.push_back(cptVecs[i]);
		}
		for (int i = 0; i < ss2.m_CtrlPts.size(); i++) {
			SV.m_CtrlPts.push_back(ss2.m_CtrlPts[i]);
		}
		varray<SplineVolume> SVT;
		SVT.push_back(SV);
		rwg.WriteSplineVolume("SVT.txt", SVT);
		return SV;
	}

	//多面放样 适用于多控制点曲面之间的放样（大于3*3*3）
	varray<SplineVolume> loft2(varray<SplineSurface> ss1, varray<SplineSurface> ss2) {
		varray<SplineVolume> SV;
		for (int i = 0; i < ss1.size(); i++) {
			SplineVolume sv;
			sv = loft2(ss1[i], ss2[i]);
			SV.push_back(sv);
		}
		return SV;
	}

	//点容器反转
	void reverse(varray<Vec4> v1) {
		stack<Vec4> st;
		for (int i = 0; i < v1.size(); i++) {
			st.push(v1[i]);
		}
		for (int i = 0; i < st.size(); i++) {
			v1.push_back(st.top());
			st.pop();
		}
	}

	/**
	*	针对自动剖分： Spline镜像(xy平面)
	*	axis:镜像轴x,y:(1,2)
	*	是否包含原图形 1：是 2：否
	*
	*/

	//点旋转(默认绕坐标原点)
	Vec4 rolate(Vec4 v1, double angle, int mode) {
		if (mode == 1) {
			//绕x轴旋转
			v1 = v1.RotateX(angle);
		}
		else if (mode == 2) {
			//绕Y轴旋转
			v1 = v1.RotateY(angle);
		}
		else if (mode == 3) {
			//绕Z轴旋转
			v1 = v1.RotateZ(angle);
		}
		return v1;
	}

	//点镜像
	varray<Vec4> mirror(varray<Vec4> v1, int axis) {
		varray<Vec4> V;

		if (axis == 1) {
			for (auto& v : v1) {
				v.y = -v.y;
			}
		}
		if (axis == 2) {
			for (auto& v : v1) {
				v.x = -v.x;
			}
		}
		for (auto& v : v1) {
			V.push_back(v);
		}

		return V;
	}

	varray<Spline> mirror(Spline sl, int axis, int choice) {
		varray<Spline> SL;
		if (choice == 1) {
			SL.push_back(sl);
		}
		if (axis == 1) {
			for (int i = 0; i < sl.m_CtrlPts.size(); i++) {
				if (sl.m_CtrlPts[i].y != 0) {
					sl.m_CtrlPts[i].y = -sl.m_CtrlPts[i].y;
				}
			}
			reverse(sl.m_CtrlPts);
			SL.push_back(sl);
		}
		if (axis == 2) {
			for (int i = 0; i < sl.m_CtrlPts.size(); i++) {
				if (sl.m_CtrlPts[i].x != 0) {
					sl.m_CtrlPts[i].x = -sl.m_CtrlPts[i].x;
				}
			}
			reverse(sl.m_CtrlPts);
			SL.push_back(sl);
		}

		return SL;
	}

	varray<Spline> mirror(varray<Spline> sl, int axis, int choice) {
		varray<Spline> SL;
		if (axis == 1) {
			for (int i = 0; i < sl.size(); i++) {
				varray<Spline> SL1;
				SL1 = mirror(sl[i], axis, choice);
				for (int j = 0; j < SL1.size(); j++) {
					SL.push_back(SL1[j]);
				}
			}
		}
		if (axis == 2) {
			for (int i = 0; i < sl.size(); i++) {
				varray<Spline> SL1;
				SL1 = mirror(sl[i], axis, choice);
				for (int j = 0; j < SL1.size(); j++) {
					SL.push_back(SL1[j]);
				}
			}
		}

		return SL;
	}

	SplineSurface mirror(SplineSurface ss, int axis) {
		ss.m_CtrlPts = mirror(ss.m_CtrlPts, axis);
		return ss;
	}

	varray<SplineSurface> mirror(varray<SplineSurface> ss, int axis) {
		varray<SplineSurface> SS;
		SplineSurface SS1;
		for (auto& s : ss) {
			SS1 = mirror(s, axis);
			SS.push_back(SS1);
		}
		return SS;
	}

	varray<SplineVolume> mirror(SplineVolume& sv, int axis) {
		varray<SplineVolume> SV;
		SV.push_back(sv);
		sv.m_CtrlPts = mirror(sv.m_CtrlPts, axis);
		SV.push_back(sv);

		return SV;
	}

	varray<SplineVolume> mirror(varray<SplineVolume>& sv, int axis) {
		varray<SplineVolume> SV, SV1;
		for (auto& s : sv) {
			SV1 = mirror(s, axis);
			for (auto& s1 : SV1) {
				SV.push_back(s1);
			}
		}
		return SV;
	}

	/*
	 *文件输出
	 */
	 //！输出VTK格式
	void outPutVTK(varray<SplineVolume>& SV, string path) {
		varray<NurbsVol> NV;
		NV = NurbsTrans::SplinevolsToCvols(SV);
		CPolyParaVolume cp;       //输出vtk文件的类对象
		cp = NV;
		cp.OutputParaVolumeDataVTK(path);
	}

	//！输出分析用文件,不用加后缀
	void outPutTXT(varray<SplineSurface>& ss, string path) {
		TestBolcks::pList plist;
		plist.OutputParaSurfaceDataTxt(ss, path);
	}
	//！输出分析用文件,不用加后缀
	void outPutTXT(varray<SplineVolume>& sv, string path) {
		TestBolcks::pList plist;
		plist.OutputParaVolumeDataTxt(sv, path);
	}

	//冒泡排序(找出最小点)
	void bubbleSort(varray<double>& points) {
		int n = points.size();
		int flag = n;
		int stop_pos;
		for (int i = 0; i < n; i++) {
			stop_pos = flag - 1;
			flag = 0;
			for (int j = 0; j < stop_pos; j++) {
				if (points[j] > points[j + 1]) {
					swap(points[j], points[j + 1]);
					flag = j + 1;
				}
			}
		}
	}

	//计算两向量夹角（弧度）
	double calAngle(Vec4 v1, Vec4 v2) {
		double dotP = v1.Dot(v2);
		double MA = v1.Magnitude();
		double MB = v2.Magnitude();
		double w = dotP / (MA * MB);
		double tmp = acos(w);
		return acos(w);
	}

	//计算中间点的权重
	void calWeight(Spline& sl) {
		Vec4 v1, v2, v3;
		varray<double> var;
		for (int i = 0; i < sl.m_CtrlPts.size() - 2; i++) {
			v1 = sl.m_CtrlPts[i];
			v2 = sl.m_CtrlPts[i + 1];
			v3 = sl.m_CtrlPts[i + 2];
			Vec4 p1 = v2 - v1;
			Vec4 p2 = v3 - v1;
			double dotP = p1.Dot(p2);
			double MA = p1.Magnitude();
			double MB = p2.Magnitude();
			double w = dotP / (MA * MB);
			var.push_back(w);
		}
		sl.m_CtrlPts[0].w = 1;
		for (int i = 1, j = 0; j < var.size(); i++, j++) {
			sl.m_CtrlPts[i].w = var[j];
		}
		int cptLenth = sl.m_CtrlPts.size() - 1;
		sl.m_CtrlPts[cptLenth].w = 1;
	}

	//调整平面模型权重(包括调整每片uv方向一致)
	void adjustSurfaceW(varray<SplineSurface>& S) {
		varray<SplineSurface> SS;
		for (auto& i : S) {
			varray<Spline> sl;
			i.GetEdgeLines(sl);
			sortEdg(sl);
			SplineSurface ss;
			ss.CoonsInterpolate(sl);
			SS.push_back(ss);
		}
		S.clear();
		S = SS;
	}

	//两点之间的距离
	static double distance(Vec4& p1, Vec4& p2)
	{
		return sqrt((p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y) + (p1.z - p2.z) * (p1.z - p2.z));
	}
	static double distance(Vec3& p1, Vec3& p2)
	{
		return sqrt((p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y) + (p1.z - p2.z) * (p1.z - p2.z));
	}

	//查找最最小Y值
	double calMinPointY(const varray<Spline>& sl) {
		varray<Vec4> v;
		for (auto i : sl) {
			for (auto j : i.m_CtrlPts) {
				v.push_back(j);
			}
		}
		varray<double> var;
		for (auto i : v) {
			var.push_back(i.y);
		}
		bubbleSort(var);

		return var[0];
	}
	//查找最最小X值
	double calMinPointX(const varray<Spline>& sl) {
		varray<Vec4> v;
		for (auto i : sl) {
			for (auto j : i.m_CtrlPts) {
				v.push_back(j);
			}
		}
		varray<double> var;
		for (auto i : v) {
			var.push_back(i.x);
		}
		bubbleSort(var);

		return var[0];
	}

	//找出距离指定点最近的点
	Vec4 calMinPoint(varray<Spline>& sl) {
		double dist, temp, tmp1, tmp2;
		Vec4 vecDistMin;//距离参考点最近的点
		tmp1 = calMinPointX(sl);
		tmp2 = calMinPointY(sl);
		Vec4 P0;//参考点
		if (tmp1 < 0 && tmp2 < 0) {
			P0 = { tmp1 * 5,tmp2 * 2,0,0 };
		}
		else if (tmp1 < 0 && tmp2 > 0) {
			P0 = { tmp1 * 5,-tmp2 * 2,0,0 };
		}
		else if (tmp1 > 0 && tmp2 < 0) {
			P0 = { -tmp1 * 5,tmp2 * 2,0,0 };
		}
		else {
			P0 = { -tmp1 * 5,-tmp2 * 2,0,0 };
		}
		//只考虑端点（这样可以排除圆弧等特殊情况）
		temp = distance(sl[0].m_CtrlPts[0], P0);
		vecDistMin = sl[0].m_CtrlPts[0];
		for (auto& i : sl) {
			int cptLenth = i.m_CtrlPts.size();//控制点个数
			dist = distance(i.m_CtrlPts[0], P0);
			if (dist < temp) {
				temp = dist;
				vecDistMin = i.m_CtrlPts[0];
			}
			dist = distance(i.m_CtrlPts[cptLenth - 1], P0);
			if (dist < temp) {
				temp = dist;
				vecDistMin = i.m_CtrlPts[cptLenth - 1];
			}
		}

		return vecDistMin;
	}

	//对边界进行排序，用于coons插值（包括控制点权重调整）
	void sortEdg(varray<Spline>& sl) {
		Vec4 vMin = calMinPoint(sl);
		varray<Spline> SL;//存放排好序的边界线
		varray<Spline> SL1;
		Spline temp, Line4, Line2, Line3;
		int cptLenth = sl[0].m_CtrlPts.size() - 1;
		//找到与距离最小点有关的两个边界线
		for (auto& i : sl) {
			int cptLenth = i.m_CtrlPts.size() - 1;
			if (JudgeTwoPointsCoincide(vMin, i.m_CtrlPts[0])) {
				SL.push_back(i);
			}
			else if (JudgeTwoPointsCoincide(vMin, i.m_CtrlPts[cptLenth])) {
				varray<Vec4> vec;
				for (int j = i.m_CtrlPts.size() - 1; j >= 0; j--) {
					vec.push_back(i.m_CtrlPts[j]);
				}
				i.m_CtrlPts = vec;
				SL.push_back(i);
			}
			else {
				SL1.push_back(i);
			}
		}
		//确定边界顺序
		Vec4 v1 = SL[0].m_CtrlPts[cptLenth] - SL[0].m_CtrlPts[0];
		Vec4 v2 = SL[1].m_CtrlPts[cptLenth] - SL[1].m_CtrlPts[0];
		v1.Normalize();
		v2.Normalize();

		Vec4 cross = v1.Cross(v2);//两向量叉乘
		//u-v
		if (cross.z < 0)
		{
			temp = SL[0];
			SL[0] = SL[1];
			SL[1] = temp;
		}

		for (auto& i : SL1) {
			if (JudgeTwoPointsCoincide(SL[1].m_CtrlPts[cptLenth], i.m_CtrlPts[0])) {
				SL.push_back(i);
			}
			else if (JudgeTwoPointsCoincide(SL[1].m_CtrlPts[cptLenth], i.m_CtrlPts[cptLenth])) {
				varray<Vec4> vec;
				for (int j = i.m_CtrlPts.size() - 1; j >= 0; j--) {
					vec.push_back(i.m_CtrlPts[j]);
				}
				i.m_CtrlPts = vec;
				SL.push_back(i);
			}
			else if (JudgeTwoPointsCoincide(SL[0].m_CtrlPts[cptLenth], i.m_CtrlPts[0])) {
				Line4 = i;
			}
			else if (JudgeTwoPointsCoincide(SL[0].m_CtrlPts[cptLenth], i.m_CtrlPts[cptLenth])) {
				varray<Vec4> vec;
				for (int j = i.m_CtrlPts.size() - 1; j >= 0; j--) {
					vec.push_back(i.m_CtrlPts[j]);
				}
				i.m_CtrlPts = vec;
				Line4 = i;
			}
		}
		SL.push_back(Line4);
		//调整权重
		for (auto& i : SL) {
			calWeight(i);
		}
		sl = SL;
	}

	//对边界顺时针排序
	void clockWiseSortEdg(varray<Spline>& sl) {
		Vec4 vMin = calMinPoint(sl);
		varray<Spline> SL;//存放排好序的边界线
		varray<Spline> tmp;//存放与最小点无关的边
		int cptLenth = sl[0].m_CtrlPts.size() - 1;
		int splineNum = sl.size() - 1;

		//找到与距离最小点有关的两个边界线
		for (auto& i : sl) {
			int cptLenth = i.m_CtrlPts.size() - 1;
			if (JudgeTwoPointsCoincide(vMin, i.m_CtrlPts[0])) {
				SL.push_back(i);
			}
			else if (JudgeTwoPointsCoincide(vMin, i.m_CtrlPts[cptLenth])) {
				varray<Vec4> vec;
				for (int j = i.m_CtrlPts.size() - 1; j >= 0; j--) {
					vec.push_back(i.m_CtrlPts[j]);
				}
				i.m_CtrlPts = vec;
				SL.push_back(i);
			}
			else {
				tmp.push_back(i);//存放无关的线段
			}
		}
		//确定边界顺序
		Vec4 v1 = SL[0].m_CtrlPts[cptLenth] - SL[0].m_CtrlPts[0];
		Vec4 v2 = SL[1].m_CtrlPts[cptLenth] - SL[1].m_CtrlPts[0];

		Vec4 p1 = v1.Normalize();
		Vec4 p2 = v2.Normalize();
		Vec4 cross = p1.Cross(p2);//两向量叉乘
		if (cross.z < 0)
		{
			tmp.push_back(SL[1]);
			SL.erase(SL.begin() + 1);
		}
		else if (cross.z > 0) {
			swap(SL[0], SL[1]);
			tmp.push_back(SL[1]);
			SL.erase(SL.begin() + 1);
		}

		queue<Spline> que;
		for (auto& i : tmp) {
			que.push(i);
		}
		int num = 0;
		while (!que.empty()) {
			if (JudgeTwoPointsCoincide(SL[num].m_CtrlPts[cptLenth], que.front().m_CtrlPts[0])) {
				SL.push_back(que.front());
				que.pop();
				num++;
			}
			else if (JudgeTwoPointsCoincide(SL[num].m_CtrlPts[cptLenth], que.front().m_CtrlPts[cptLenth])) {
				varray<Vec4> vec;
				for (int j = cptLenth; j >= 0; j--) {
					vec.push_back(que.front().m_CtrlPts[j]);
				}
				que.front().m_CtrlPts = vec;
				SL.push_back(que.front());
				que.pop();
				num++;
			}
			else {
				que.push(que.front());
				que.pop();
			}
		}
		sl = SL;
	}

	//对面片排序
	void sortSurf(varray<SplineSurface>& ss) {
		varray<SplineSurface> SS;
		varray<double> points;
		unordered_map<double, SplineSurface> map;
		for (auto& i : ss) {
			points.push_back(i.m_CtrlPts[4].x);
		}
		//冒泡排序
		bubbleSort(points);

		for (auto& i : ss) {
			map.insert(pair<double, SplineSurface>(i.m_CtrlPts[4].x, i));
		}
		for (auto& i : points) {
			auto iter = map.find(i);
			SS.push_back(iter->second);
		}
		ss = SS;
	}

	//调整面片uv排序，使整体uv一致（包括控制点权重调整）(放样的时候使用会导致边边不对应)
	void quadAdjustUV(varray<SplineSurface>& ss) {
		varray<SplineSurface> SS;
		for (auto& i : ss) {
			SplineSurface ss;
			varray<Spline> sl;
			i.GetEdgeLines(sl);
			sortEdg(sl);
			ss.CoonsInterpolate(sl);
			SS.push_back(ss);
		}
		ss.clear();
		/*sortSurf(SS);*/
		for (auto& i : SS) {
			ss.push_back(i);
		}
	}

	//由线组生成面
	void splineCreatSurf(varray<Spline>& sl, varray<SplineSurface>& SS) {
		int num = sl.size();
		if (num % 4 == 0) {
			//线分组
			varray<Spline> s;
			for (int i = 0; i < sl.size(); i = i + 4) {
				s.clear();
				s.push_back(sl[i]);
				s.push_back(sl[i + 1]);
				s.push_back(sl[i + 2]);
				s.push_back(sl[i + 3]);
				SplineSurface ss;
				sortEdg(s);
				ss.CoonsInterpolate(s);
				SS.push_back(ss);
			}
		}
	}

	//判断两线段是否相交
	bool judgeSplineCross(Vec4 v1, Vec4 v2, Vec4 v3, Vec4 v4) {
		int ptFlag1, ptFlag2;
		ptFlag1 = JudgeTwoPointsCoincide(v1, v2);
		ptFlag2 = JudgeTwoPointsCoincide(v3, v4);

		if (!ptFlag1 && !ptFlag2) {
			Vec4 V1 = v2 - v1;//向量1
			Vec4 V2 = v3 - v1;//向量2
			Vec4 V3 = v4 - v1;//向量3

			Vec4 V4 = v4 - v3;//向量1
			Vec4 V5 = v1 - v3;//向量2
			Vec4 V6 = v2 - v3;//向量3

			Vec4 Vc1 = V1.Cross(V2);
			Vec4 Vc2 = V1.Cross(V3);

			Vec4 Vc3 = V4.Cross(V5);
			Vec4 Vc4 = V4.Cross(V6);

			//判断向量方向是否一致
			bool dir1, dir2, dir3, dir4;
			Vec4 P1 = V1.Normalize();
			Vec4 P2 = V2.Normalize();
			Vec4 P3 = V3.Normalize();
			dir1 = JudgeTwoPointsCoincide(P1, P2);
			dir2 = JudgeTwoPointsCoincide(P1, P3);

			Vec4 P4 = V4.Normalize();
			Vec4 P5 = V5.Normalize();
			Vec4 P6 = V6.Normalize();
			dir3 = JudgeTwoPointsCoincide(P4, P5);
			dir4 = JudgeTwoPointsCoincide(P4, P6);

			//端点是否重合
			bool flag1, flag2, flag3, flag4;
			flag1 = JudgeTwoPointsCoincide(v1, v3);
			flag2 = JudgeTwoPointsCoincide(v1, v4);
			flag3 = JudgeTwoPointsCoincide(v2, v3);
			flag4 = JudgeTwoPointsCoincide(v2, v4);

			//叉乘异号肯定相交
			if ((Vc1.z < 0 && Vc2.z>0 && Vc3.z < 0 && Vc4.z>0)
				|| (Vc1.z > 0 && Vc2.z < 0) && (Vc3.z < 0 && Vc4.z>0)
				|| (Vc1.z < 0 && Vc2.z>0) && (Vc3.z > 0 && Vc4.z < 0)
				|| (Vc1.z > 0 && Vc2.z < 0) && (Vc3.z > 0 && Vc4.z < 0)) {
				return true;
			}
			//一个端点在线段上（不包括端点）
			else if ((!flag1 && !flag2 && !flag3 && !flag4) &&
				((Vc1.z == 0 && dir1) || (Vc2.z == 0 && dir2) ||
					(Vc3.z == 0 && dir3) || (Vc4.z == 0 && dir4))) {
				return true;
			}
			//两线段端点有重合的情况(肯定不相交)
			else if (flag1 || flag2 || flag3 || flag4) {
				return false;
			}
		}
		return false;
	}

	//判断几何域包含关系（不包含辅助线）
	//利用PolygonsRelationship来判断两个几何域的包含关系
	void ReSetTreeNode(SfCtainTreeNode* father)
	{
		if (!father->childs.empty())
		{
			list<SfCtainTreeNode*>::iterator left = father->childs.begin();
			list<SfCtainTreeNode*>::iterator right;

			//首先判断父节点下所有子节点的互相包含关系
			for (; left != (father->childs.end());)
			{
				right = left;
				bool flag1 = false, flag2 = false;
				for (++right; right != father->childs.end();)
				{
					//先判定right是否在left内部
					flag1 = PolygonsRelationship((*right)->outLines, (*left)->outLines);
					if (flag1) {
						//若right在left内部
						list<SfCtainTreeNode*>::iterator tmpit = right;
						tmpit++;
						//将right放入left的子节点list中
						(*left)->childs.push_back(*right);
						//删除right
						father->childs.erase(right);
						right = tmpit;
						continue;
					}

					//若right不在left内部,判断left是否在right内部
					flag2 = PolygonsRelationship((*left)->outLines, (*right)->outLines);
					if (flag2) {
						//若left在right内部
						list<SfCtainTreeNode*>::iterator tmpit = left;
						tmpit++;
						//将left放入right子节点list中
						(*right)->childs.push_back(*left);
						//删除left
						father->childs.erase(left);
						left = tmpit;
						break;
					}
					right++;
				}
				if (!flag2)
					left++;
			}
		}
	}

	//构建包含树（不包含辅助线）
	SfCtainTreeNode* CreateTree(const varray<varray<Spline>>& surf)
	{
		queue< SfCtainTreeNode*> nodes;

		//设置树根，树根为外轮廓
		SfCtainTreeNode* root = new SfCtainTreeNode(surf[0]);
		root->num = -1;

		//所有的内轮廓设置为外轮廓的孩子
		SfCtainTreeNode* cur = nullptr;
		for (int i = 1; i < surf.size(); ++i)
		{
			cur = new SfCtainTreeNode(surf[i]);
			cur->num = i - 1;//设定序号
			//把其余曲面轮廓都设为根节点的子节点
			root->childs.push_back(cur);
		}

		//层序遍历处理所有节点
		nodes.push(root);
		while (!nodes.empty())
		{
			cur = nodes.front();
			nodes.pop();
			if (!cur->childs.empty()) {
				//构建几何域包含树
				//判断几何域包含关系
				ReSetTreeNode(cur);

				//存入cur节点的所有子节点
				list< SfCtainTreeNode*>::iterator it = cur->childs.begin();
				for (; it != cur->childs.end();) {
					nodes.push(*it);
					it++;
				}
			}
		}
		return root;//返回根结点
	}

	//线段与多边形是否有交点
	bool SplinePolygonIfCross(Spline sl, varray<Spline> pol)
	{
		int num = sl.m_CtrlPts.size() - 1;
		Vec3 v1 = sl.m_CtrlPts[0];
		Vec3 v2 = sl.m_CtrlPts[num];

		//情况一：直线段
		varray<Vec3> ptVec;//存放多边形端点
		//这里有问题

		for (auto i : pol) {
			//判断是否为曲线
			bool flag;
			Vec4 v1, v2, v3;
			for (int j = 0; j < i.m_CtrlPts.size() - 2; j++) {
				v1 = i.m_CtrlPts[j];
				v2 = i.m_CtrlPts[j + 1];
				v3 = i.m_CtrlPts[j + 2];
				Vec4 p1 = v2 - v1;
				Vec4 p2 = v3 - v1;
				double dotP = p1.Dot(p2);
				double MA = p1.Magnitude();
				double MB = p2.Magnitude();
				double w = dotP / (MA * MB);
				if (abs(w) < 1) {
					flag = true;
					break;
				}
			}
			if (flag) {
				for (int j = 0; j < i.m_CtrlPts.size() - 1; ++j) {
					ptVec.push_back(i.m_CtrlPts[j]);
				}
				flag = false;
			}
			else {//直线
				for (auto i : pol) {
					ptVec.push_back(i.m_CtrlPts[0]);
				}
			}
		}
		//cout << "ptVec大小：" << ptVec.size() << endl;
		/*for (auto i : pol) {
			ptVec.push_back(i.m_CtrlPts[0]);
		}*/
		bool crossFlag;
		for (int i = 0; i < ptVec.size(); ++i) {
			if (i == ptVec.size() - 1) {
				crossFlag = judgeSplineCross(v1, v2, ptVec[i], ptVec[0]);
				if (crossFlag) {
					return true;
				}
			}
			else {
				crossFlag = judgeSplineCross(v1, v2, ptVec[i], ptVec[i + 1]);
				if (crossFlag) {
					return true;
				}
			}
		}
		return false;
	}

	void test_SplinePolygonIfCross0()
	{
		Spline s;
		varray<Spline> S;
		varray<Spline> temp;
		RWGeometric rwg;
		rwg.ReadSpline("E:\\Model\\PlaneQuad\\Chack.txt", temp);
		for (int i = 0; i < 8; i++)
		{
			S.push_back(temp[i]);
		}
		s = temp[12];

		if (SplinePolygonIfCross(s, S))
		{
			cout << "相交" << endl;
		}
		else
		{
			cout << "不相交" << endl;
		}
		temp = S;

		temp.push_back(s);
		rwg.WriteSpline("E:\\Model\\PlaneQuad\\ChackSplinePolygonIfCross.txt", temp);
	}

	//创建内外连接线(外轮廓、单个内轮廓、全部内轮廓、结果) 返回的是所有可能的连接线
	varray<Spline> creatSplineOfPolgons(varray<Spline> outLine, varray<Spline> inLine, varray<varray<Spline>> inLines, varray<Spline>& result) {
		RWGeometric rwg;
		varray<Spline> tmp;
		clockWiseSortEdg(outLine);					//轮廓线顺时针排序
		rwg.WriteSpline("E:\\Model\\PlaneQuad\\aoutLine.txt", outLine);	//后续可注释掉，下同
		for (auto& i : inLines) {
			clockWiseSortEdg(i);					//轮廓线顺时针排序
			for (auto& j : i) {
				tmp.push_back(j);
			}
		}
		clockWiseSortEdg(inLine);					//轮廓线顺时针排序
		rwg.WriteSpline("E:\\Model\\PlaneQuad\\ainLine.txt", inLine);
		varray<Vec4> outs, outs1;					//控制点
		varray<Vec4> ins, ins1;						//控制点
		for (auto i : outLine) {
			//outs.push_back(i.m_CtrlPts[0]);		//这里是取首端点，不包含样条中间点,用于生成连接线

			if (i.m_CtrlPts[1].w != 1) {			//这里是取曲线的首端点和中间点，改进后续曲线夹角的判断方式
				outs1.push_back(i.m_CtrlPts[0]);
				outs1.push_back(i.m_CtrlPts[1]);
			}
			else {
				outs1.push_back(i.m_CtrlPts[0]);
			}
		}
		for (auto i : inLine) {
			//ins.push_back(i.m_CtrlPts[0]);		//这里是取首端点，不包含样条中间点,用于生成连接线

			if (i.m_CtrlPts[1].w != 1) {			//这里是取曲线的首端点和中间点，改进后续曲线夹角的判断方式
				ins1.push_back(i.m_CtrlPts[0]);
				ins1.push_back(i.m_CtrlPts[1]);
			}
			else {
				ins1.push_back(i.m_CtrlPts[0]);
			}
		}
		varray<double> bestAngle;					//存放各个外轮廓点对应连接线最好的角度
		unordered_map<double, Spline> bestMap;		//用于查找最好的连接线
		varray<Spline> resultSL;					//存放所有可能的连接线
		resultSL.clear();

		//创建合适连接线
		for (int j = 0; j < outs1.size(); j++) {
			if (outs1[j].w != 1) {					//曲线中间控制点不参与连接线生成
				continue;
			}
			//cout << "外轮廓端点大小："<< outs.size() << endl;
			varray<double> allAngle;				//存放单个轮廓点对应连接线最好的角度
			unordered_map<double, Spline> Map;		//用于查找单个轮廓点最好的连接线
			allAngle.clear();
			Map.clear();
			varray<Spline> tmp;

			for (int i = 0; i < ins1.size(); ++i) {
				if (ins1[i].w != 1) {				//曲线中间控制点不参与连接线生成
					continue;
				}
				//cout << "内轮廓端点大小：" << ins.size() << endl;
				Spline0 sl;
				Spline tmpSL;

				tmpSL = sl.getSpline(ins1[i], outs1[j]);
				int cptLenth = tmpSL.m_CtrlPts.size() - 1;
				bool flag;
				int insLenth = ins1.size() - 1;
				//判断是否相交
				for (auto& k : inLines) {
					flag = SplinePolygonIfCross(tmpSL, k);
					if (flag) {
						break;
					}
				}
				tmp.push_back(tmpSL);
				//没有相交的情况，收集最符合要求的连接线
				if (!flag) {
					//再排除与曲线相交的连接线 排除连接线位于内轮廓内部的情况
					Vec3 mid1 = tmpSL.GetLinePoint(0.01);
					Vec3 mid2 = tmpSL.GetLinePoint(0.99);
					int mflag1, mflag2;
					mflag1 = PointRelatePolygon(mid1, inLine);
					if (mflag1 == 1) {
						continue;
					}
					mflag2 = PointRelatePolygon(mid2, inLine);
					if (mflag2 == 1) {
						continue;
					}

					//用于判断是否满足角度要求 夹角相差最小
					//这里只有对内部线的夹角判断 要加上外部判断来完善
					Vec4 v1 = tmpSL.m_CtrlPts[cptLenth] - tmpSL.m_CtrlPts[0];
					Vec4 v2 = tmpSL.m_CtrlPts[0] - tmpSL.m_CtrlPts[cptLenth];
					Vec4 v3, v4;//内部
					Vec4 v5, v6;//外部

					//内部
					if (i == 0) {
						v3 = ins1[i + 1] - ins1[i];
						v4 = ins1[insLenth] - ins1[i];
					}
					else if (i == ins1.size() - 1) {
						v3 = ins1[0] - ins1[i];
						v4 = ins1[i - 1] - ins1[i];
					}
					else {
						v3 = ins1[i + 1] - ins1[i];
						v4 = ins1[i - 1] - ins1[i];
					}

					//外部
					if (j == 0) {
						v5 = outs1[j + 1] - outs1[j];
						v6 = outs1[outs1.size() - 1] - outs1[j];
					}
					else if (j == outs1.size() - 1) {
						v5 = outs1[0] - outs1[j];
						v6 = outs1[j - 1] - outs1[j];
					}
					else {
						v5 = outs1[j + 1] - outs1[j];
						v6 = outs1[j - 1] - outs1[j];
					}

					//记录夹角之差 内部
					double angle, angle1, angle2;
					angle1 = calAngle(v1, v3);
					angle2 = calAngle(v1, v4);
					angle = abs(angle1 - angle2);

					//记录夹角之差 外部
					double outAngle, outAngle1, outAngle2;
					outAngle1 = calAngle(v2, v5);
					outAngle2 = calAngle(v2, v6);
					outAngle = abs(outAngle1 - outAngle2);

					double ioAngle;
					ioAngle = angle + outAngle;

					allAngle.push_back(ioAngle);
					Map.insert(pair<double, Spline>(ioAngle, tmpSL));
					/*auto iter = Map.find(allAngle[0]);
					if (iter == Map.end()) {
						cout << "没有找到对像" << endl;
					}*/
					resultSL.push_back(tmpSL);
				}
			}
			rwg.WriteSpline("ak.txt", tmp);
			if (allAngle.size() == 0) {
				cout << "没有合适连接线" << endl;
				continue;
			}
			//找出对于外轮廓一点最合适的连接线
			bubbleSort(allAngle);
			auto iter = Map.find(allAngle[0]);
			if (iter == Map.end()) {
				cout << "没有找到对像" << endl;
				continue;
			}
			else {
				bestAngle.push_back(allAngle[0]);
				bestMap.insert(pair<double, Spline>(iter->first, iter->second));
			}
		}
		if (bestAngle.size() != 0) {
			bubbleSort(bestAngle);
			//选取第一个连接线作为最合适的连接线
			//有些情况，连接线的不同会影响剖分成功与否，这里可以改变选取的连接线，如选择第二条连接线
			cout << "连接线数量：" << bestAngle.size() << endl;
			cout << "选择连接线序号(从0开始)：";

			varray<Spline> temp;
			rwg.ReadSpline("E:\\Model\\PlaneQuad\\AllBoundary.txt", temp);
			for (const auto& i : bestMap)
			{
				temp.push_back(i.second);
			}
			rwg.WriteSpline("E:\\Model\\PlaneQuad\\AllLineAndTwoBoundary.txt", temp);
			int num = 0;
			cin >> num;
			while (num > bestAngle.size()) {
				cout << "选择的连接线序号超出连接线总数量，请重新输入:" << endl;
				cin >> num;
			}
			auto iter = bestMap.find(bestAngle[num]);
			result.push_back(iter->second);
			rwg.WriteSpline("E:\\Model\\PlaneQuad\\ak.txt", resultSL);
			return resultSL;
		}
		else {
			cout << "没有合适的连接线" << endl;
		}

		//所有候选连接线
		rwg.WriteSpline("E:\\Model\\PlaneQuad\\ak.txt", resultSL);
		return resultSL;
	}

	/**
	* @brief   : 判断曲线是否跨越边界线所围成的区域
	* @param[I]: 曲线
	* @param[I]: 边界曲线
	* @param[O]: none
	* @return  : bool，是否跨越
	* @note    :通过截断两端点，判断截断后的曲线s1是否与边界曲线相交，进而判断曲线s是否跨越边界线S所围成的区域
	**/
	bool SplinePolygonIfCross0(const Spline& s, const varray<Spline>& S)
	{
		Spline0 s0;
		//截断两端点获取新的连接线
		Spline s1 = s0.getSpline(s.GetLinePoint(0.01), s.GetLinePoint(0.99));

		//分别与所有边界线进行求交判断
		for (auto& i : S)
		{
			if (if_Intersect(s1, i) != 0)
			{
				return true;
			}
		}
		return false;
	}

	/**
	* @brief   : 判断是否为直线
	* @param[I]: 待判断的线段
	* @return  : bool 为直线时返回true
	* @note    :
	**/
	bool if_StraightLine(const Spline& s)
	{
		//分别获得两端点及中间点
		Vec3 p1 = s.GetLinePoint(0);
		Vec3 p2 = s.GetLinePoint(0.5);
		Vec3 p3 = s.GetLinePoint(1);

		//获得方向向量
		Vec3 t1 = p2 - p1;
		Vec3 t2 = p3 - p2;
		Vec3 t3 = p3 - p1;

		//单位化方向向量
		t1 = t1.Normalize();
		t2 = t2.Normalize();
		t3 = t3.Normalize();

		if (JudgeTwoPointsCoincide(t1, t2) && JudgeTwoPointsCoincide(t1, t3) && JudgeTwoPointsCoincide(t2, t3))
		{
			return true;
		}
		return false;
	}

	/**
	* @brief   : 用折线逼近曲线
	* @param[I]: 要逼近的曲线
	* @return  : 折线
	* @note    :在参数区间0~1之间取点，并构建直线，从而逼近曲线
	**/
	varray<Spline> CurveToLines(const Spline& s)
	{
		varray<Spline> S;
		if (if_StraightLine(s))
		{
			S.push_back(s);
			return S;
		}

		//v1为各个逼近直线的起点
		Vec4 v1;

		//v1为各个逼近直线的终点
		Vec4 v2;
		Spline0 s0;

		//取点间隔
		double epsge = 0.05;

		//生成折线
		v1 = s.GetLinePoint(0);
		for (double i = epsge; i <= 1; )
		{
			v2 = s.GetLinePoint(i);
			S.push_back(s0.getSpline(v1, v2));
			v1 = v2;
			i += epsge;
		}
		return S;
	}

	/**
	* @brief   : 获取两点之间的距离
	* @param[I]: 点p1
	* @param[I]: 点p2
	* @return  : 两点之间的距离
	* @note    :
	**/
	double twoPointDistence(const Vec4& p1, const Vec4& p2)
	{
		double x = p1.x - p2.x;
		double y = p1.y - p2.y;
		double z = p1.z - p2.z;
		x = pow(x, 2);
		y = pow(y, 2);
		z = pow(z, 2);
		return sqrt(x + y + z);
	}

	/**
	* @brief   : 利用SISL库函数判断两曲线是否相交
	* @param[I]: 第一条曲线
	* @param[I]: 第二条曲线
	* @return  : 交点个数
	* @note    :
	**/
	int SplinesInterNum(const Spline& s1, const Spline& s2)
	{
		SISLCurve* curve1 = NurbsLineToSislLine(s1); /* Must be defined */
		SISLCurve* curve2 = NurbsLineToSislLine(s2); /* Must be defined */
		double epsco = 1.0e-9; /* Not used */
		double epsge = 1.0e-6;
		int numintpt = 0;
		double* intpar1 = NULL;
		double* intpar2 = NULL;
		int numintcu = 0;
		SISLIntcurve** intcurve = NULL;
		int stat = 0;
		s1857(curve1, curve2, epsco, epsge, &numintpt, &intpar1, &intpar2, &numintcu, &intcurve, &stat);

		//numintpt为相交的个数
		return numintpt;
	}

	/**
	* @brief   : 用折线逼近曲线，从而判断两曲线是否相交
	* @param[I]: 第一条曲线
	* @param[I]: 第二条曲线
	* @return  : 相交的次数
	* @note    :
	**/
	int if_Intersect(const Spline& s1, const Spline& s2)
	{
		if (JudgeTwoLinesCoincide(s1, s2))
		{
			return 1;
		}

		if (JudgeTwoPointsCoincide(s1.m_CtrlPts[0], s2.m_CtrlPts[0]) || JudgeTwoPointsCoincide(s1.m_CtrlPts[s1.m_CtrlPts.size() - 1], s2.m_CtrlPts[s1.m_CtrlPts.size() - 1]))
		{
			return 0;
		}

		if (JudgeTwoPointsCoincide(s1.m_CtrlPts[0], s2.m_CtrlPts[s2.m_CtrlPts.size() - 1]) || JudgeTwoPointsCoincide(s1.m_CtrlPts[s1.m_CtrlPts.size() - 1], s2.m_CtrlPts[0]))
		{
			return 0;
		}

		//用直线逼近曲线之后，用直线判断是否相交更准确
		varray<Spline> S1 = this->CurveToLines(s1);
		varray<Spline> S2 = this->CurveToLines(s2);
		int n = 0;
		for (auto& i : S1)
		{
			for (auto& j : S2)
			{
				if (SplinesInterNum(i, j) != 0)
				{
					n++;
				}
			}
		}
		return n;
	}

	/**
	* @brief   : 两曲线之间最近的点
	* @param[I]: 第一条曲线
	* @param[I]: 第二条曲线
	* @param[O]: 第一条曲线上的参数值
	* @param[O]: 第二条曲线上的参数值
	* @return  : none
	* @note    :
	**/
	bool ClosePoint(const Spline& s1, const Spline& s2, double& u1, double& u2)
	{
		//两曲线相交，不考虑
		if (if_Intersect(s1, s2) > 0)
		{
			return false;
		}

		u1 = 0;
		u2 = 0;
		double temp;
		double min = twoPointDistence(s1.GetLinePoint(u1), s2.GetLinePoint(u2));
		//步长，参数每次增长的长度
		double step = 0.01;
		for (double i = 0.0; i <= 1;)
		{
			for (double j = 0.0; j <= 1;)
			{
				temp = twoPointDistence(s1.GetLinePoint(i), s2.GetLinePoint(j));
				if (temp < min)
				{
					u1 = i;
					u2 = j;
					min = temp;
				}
				j += step;
			}
			i += step;
		}
	}

	/**
	* @brief   : 找出曲线中所有控制点的最小x值
	* @param[I]: 轮廓线
	* @param[O]: 最小的x坐标
	* @return  :
	* @note    :通过找出所有轮廓曲线中的最小x坐标，并按照x的值进行排序
	**/
	bool minimum_X(Spline s, double& min)
	{
		if (s.m_CtrlPts.empty())
		{
			return false;
		}
		min = s.m_CtrlPts[0].x;

		for (auto& i : s.m_CtrlPts)
		{
			if (min > i.x)
			{
				min = i.x;
			}
		}
		return true;
	}

	/**
	* @brief   : 找出轮廓线中控制顶点x最小的曲线
	* @param[I]: 轮廓线
	* @param[O]: 最小的x坐标
	* @return  :
	* @note    :通过找出所有轮廓曲线中的最小x坐标，并按照x的值进行排序
	**/
	bool minimum_X(varray<Spline> S, double& min)
	{
		double temp = 0;
		if (S.empty())
		{
			return false;
		}
		min = S[0].m_CtrlPts[0].x;
		for (auto& i : S)
		{
			//判断该曲线的控制点是否为空
			if (!minimum_X(i, temp))
			{
				return false;
			}
			if (min > temp)
			{
				min = temp;
			}
		}
	}

	/**
	* @brief   : 从左至右排序轮廓线
	* @param[I]: 节点数组
	* @param[O]: none
	* @return  :
	* @note    :通过找出所有轮廓曲线中的最小x坐标，并按照x的值进行排序
	**/
	bool Order_Nodes(varray<SfCtainTreeNode*>& Nodes)
	{
		//轮廓为空，排序失败
		if (Nodes.empty())
		{
			return false;
		}
		double min = 0;
		multimap<double, SfCtainTreeNode*>mp;
		for (auto& i : Nodes)
		{
			minimum_X(i->outLines, min);
			mp.insert(pair<double, SfCtainTreeNode*>(min, i));
		}

		Nodes.clear();
		for (auto it = mp.begin(); it != mp.end(); it++)
		{
			Nodes.push_back(it->second);
		}
	}

	/**
	* @brief   : 控制u的精度，防止在截断时出现小曲线
	* @param[I]: 参数值 u
	* @param[O]: none
	* @return  :
	* @note    :
	**/
	void control_Accuracy(double& u)
	{
		//若u与1之间相差小于0.02，则将u设置为1
		if (1.0 - u <= 0.1)
		{
			u = 1;
		}
		if (u - 0.0 <= 0.1)
		{
			u = 0;
		}
	}

	/**
	* @brief   : 去除中间结果中的重复曲线
	* @param[I]: 带去除数组
	* @param[O]: none
	* @return  :
	* @note    :通过找出所有轮廓曲线中的最小x坐标，并按照x的值进行排序
	**/
	void DeduplicateLines(varray<ContourData>& Lines)
	{
		bool flag;
		varray<ContourData> result;
		for (auto& i : Lines)
		{
			//标志作用，记录曲线i是否与result中的某条曲线重合
			flag = false;
			for (auto& j : result)
			{
				//曲线i与result中的某条曲线重合，
				if (JudgeTwoLinesCoincide(i.adl.s, j.adl.s))
				{
					flag = true;
					break;
				}
			}
			//flag为false，说明曲线i不与result中任何一条曲线重合
			if (!flag)
			{
				result.push_back(i);
			}
		}
		Lines = result;
	}
	void DeduplicateLines(varray<AddLineData>& Lines)
	{
		bool flag;
		varray<AddLineData> result;
		for (auto& i : Lines)
		{
			//标志作用，记录曲线i是否与result中的某条曲线重合
			flag = false;
			for (auto& j : result)
			{
				//曲线i与result中的某条曲线重合，
				if (JudgeTwoLinesCoincide(i.s, j.s))
				{
					flag = true;
					break;
				}
			}
			//flag为false，说明曲线i不与result中任何一条曲线重合
			if (!flag)
			{
				result.push_back(i);
			}
		}
		Lines = result;
	}

	/**
	* @brief   : 找出两边界之间最合适的连接线（最短，且不跨越两区域）
	* @param[I]: 第一个边界线
	* @param[I]: 第二个边界线
	* @param[O]: none
	* @return  :varray<AddLineData>	两边界之间的连接线
	* @note    :通过截断两端点，判断截断后的曲线s1是否与边界曲线相交
				进而判断曲线s是否跨越边界线S所围成的区域
				从而判断该连接线是是否符合要求
	**/
	varray<AddLineData> BestAddLine(SfCtainTreeNode* Node1, SfCtainTreeNode*& Node2)
	{
		varray<Spline> chack;
		RWGeometric rwg;
		varray<Spline> S1 = Node1->outLines;
		varray<Spline> S2 = Node2->outLines;
		multimap<double, AddLineData> bestMap;
		multimap<double, AddLineData> single_bestMap;
		varray<AddLineData> result;
		AddLineData adl;
		double u1, u2;
		Vec4 v1, v2;
		Spline s;
		Spline0 s0;
		varray<Spline> all = S1;
		for (auto& i : S2)
		{
			all.push_back(i);
		}

		/*
			依次取出S1中的每条曲线i，找出最合适的曲线sl
			1、sl是最短的曲线
			2、sl不与S2相交
		*/
		for (auto i = S1.begin(); i != S1.end(); i++)
		{
			single_bestMap.clear();
			for (auto j = S2.begin(); j != S2.end(); j++)
			{
				//两曲线i,j之间最近点的参数值
				ClosePoint(*i, *j, u1, u2);

				if (u1 == -1)
				{
					continue;
				}

				control_Accuracy(u1);
				control_Accuracy(u2);

				v1 = i->GetLinePoint(u1);
				v2 = j->GetLinePoint(u2);

				//两最近点生成一条线段
				s = s0.getSpline(v1, v2);
				adl.s = s;
				adl.pa1.first = i - S1.begin();
				adl.pa1.second = u1;

				adl.pa2.first = j - S2.begin();
				adl.pa2.second = u2;

				chack.clear();
				chack.push_back(s);
				chack.push_back(*i);
				chack.push_back(*j);
				rwg.WriteSpline("E:\\Model\\PlaneQuad\\BestAddLineChack.txt", chack);
				//判断s是否与S1、S2相交
				if (!SplinePolygonIfCross0(s, S1) && !SplinePolygonIfCross0(s, S2))
				{
					//不跨越区域S1、S2，可以存入候选线
					single_bestMap.insert(pair<double, AddLineData>(s.GetLength(s.m_CtrlPts.size()), adl));
					cout << "不相交" << endl;
					//system("pause");
				}
			}

			//在所有符合要求的连接线中
			if (!single_bestMap.empty())
			{
				for (auto& i : single_bestMap)
				{
					result.push_back(i.second);
				}
			}
		}

		//查看所有连接线
		chack = all;
		for (auto& i : result)
		{
			chack.push_back(i.s);
		}
		rwg.WriteSpline("E:\\Model\\PlaneQuad\\Chack.txt", chack);
		return result;
	}

	/**
	* @brief   : 依据长度最短的原则构建连接线
	* @param[I]: 外轮廓
	* @param[I]: 内轮廓
	* @param[O]: 结果连接线
	* @return  :
	* @note    :依据连接线最短的原则，在两轮廓之间构建连接线，并且连接线不能跨越其他区域以及不与其他连接线相交
	**/
	void createAddline(varray<Spline>& outer, varray<varray<Spline>>& inner, varray<ContourData>& result)
	{
		//所有轮廓线
		varray<varray<Spline>> surf;

		//初始化surf
		surf.push_back(outer);
		for (auto& i : inner)
		{
			surf.push_back(i);
		}

		//所有边界线
		varray<Spline> allSpline;
		for (auto& i : surf)
		{
			for (auto& j : i)
			{
				allSpline.push_back(j);
			}
		}

		//辅助查看的数组
		varray<Spline> chack;

		//临时变量
		ContourData cd;

		//层次遍历所用的队列
		queue<SfCtainTreeNode*> Qnodes;

		//层次遍历后根遍历所用队列
		stack<SfCtainTreeNode*>Snode;
		//初略构建几何域包含树
		SfCtainTreeNode* root = CreateTree(surf);

		//层次遍历指针
		SfCtainTreeNode* cur = nullptr;

		//辅助工具
		RWGeometric rwg;
		varray<Spline> tempChackLine;

		Qnodes.push(root);
		//cur节点的子节点
		varray<SfCtainTreeNode*> allNode;
		varray<SfCtainTreeNode*> childnodes;

		//层次遍历几何域包含树，
		while (!Qnodes.empty())
		{
			cur = Qnodes.front();
			Qnodes.pop();

			//用栈记录后根遍历的顺序
			Snode.push(cur);

			//实现层次遍历
			for (auto it = cur->childs.begin(); it != cur->childs.end(); ++it)
			{
				Qnodes.push(*it);
			}
		}
		//根据后根遍历顺序，实现由内至外创建连接线
		while (!Snode.empty())
		{
			cur = Snode.top();
			Snode.pop();

			//为叶子节点，无内轮廓，不添加连接线
			if (cur->childs.empty())
			{
				continue;
			}

			allNode.clear();
			allNode.push_back(cur);
			childnodes.clear();

			//以cur为外轮廓，子节点为内轮廓，创建连接线
			for (auto it = cur->childs.begin(); it != cur->childs.end(); ++it)
			{
				childnodes.push_back(*it);
			}

			//排序内轮廓顺序
			Order_Nodes(childnodes);

			//将排序后子节点数组存入allNode中
			for (auto it = childnodes.begin(); it != childnodes.end(); ++it)
			{
				allNode.push_back(*it);
			}
			if (childnodes.size() > 1)
			{
				allNode.push_back(cur);
			}

			//记录allNode数组中相邻的两个节点
			auto front = allNode.begin();
			auto back = allNode.begin() + 1;

			//存放每两个边界区域之间的候选连接线
			//存放两轮廓间合适的连接线<连接线长度，连接线相关曲线的信息>
			multimap<double, ContourData> bestLine;

			//中间结果，用于存放不与其他轮廓线以及连接线相交的曲线
			varray<ContourData> mid_result;
			for (; back != allNode.end(); ++front, ++back)
			{
				bestLine.clear();
				varray<AddLineData> temp = BestAddLine(*front, *back);

				chack.clear();
				for (auto& i : temp)
				{
					chack.push_back(i.s);
				}
				rwg.WriteSpline("E:\\Model\\PlaneQuad\\BeforeDeduplicate.txt", chack);
				DeduplicateLines(temp);

				chack.clear();
				for (auto& i : temp)
				{
					chack.push_back(i.s);
				}
				rwg.WriteSpline("E:\\Model\\PlaneQuad\\AfterDeduplicate.txt", chack);

				//判断候选线是否与其他区域以及连接线相交
				for (auto& j : temp)
				{
					//记录连接线
					Spline s = j.s;

					//记录是否与其他边界区域相交
					bool flag = true;

					//对各个候选连接线判断是否跨域边界区域
					for (auto& k : surf)
					{
						//是否跨越轮廓线围成的区域
						if (this->SplinePolygonIfCross0(s, k))
						{
							flag = false;
						}
					}

					//将候选线分别与已选择的连接线进行判断是否相交
					for (auto it = result.begin(); it != result.end(); it++)
					{
						if (this->if_Intersect(it->adl.s, s) > 0)
						{
							flag = false;
						}
					}

					//不与任何边界或连接线相交，则存入辅助线相关的信息如：辅助线、长度、相关边界的序号（在operate_array数组中的）
					if (flag)
					{
						cd.adl = j;
						cd.contour1 = (*front)->num;
						cd.contour2 = (*back)->num;
						double length = s.GetLength(s.m_CtrlPts.size());
						if (length > 0.1)
						{
							bestLine.insert(pair<double, ContourData>(length, cd));
						}
					}
				}
				mid_result.clear();
				int len;
				int n;
				//将符合要求的连接线存入mid_result数组中，再从mid_result数组中选择连接线
				if (!bestLine.empty())
				{
					for (auto it = bestLine.begin(); it != bestLine.end(); it++)
					{
						mid_result.push_back(it->second);
					}

					//去掉重复的曲线
					DeduplicateLines(mid_result);

					//候选线不为空，选择合适得连接线(可以多选)
					if (!mid_result.empty())
					{
						len = mid_result.end() - mid_result.begin();
						cout << "共" << len << "条候选线，输入需要保留的连接线：";
						{
							chack.clear();
							for (auto& i : mid_result)
							{
								chack.push_back(i.adl.s);
							}
							rwg.WriteSpline("E:\\Model\\PlaneQuad\\OnlyAddLine.txt", chack);
							for (auto& i : allSpline)
							{
								chack.push_back(i);
							}
							rwg.WriteSpline("E:\\Model\\PlaneQuad\\AllLineAndTwoBoundary.txt", chack);
							//可以选择多条连接线
							while (cin >> n) {
								if (n >= 0 || n < len)
								{
									result.push_back(mid_result[n]);
								}
								char ch = getchar();//读取下一个字符，为换行符，则break
								if (ch == '\n')
									break;
							}
						}
					}
				}
			}
		}
	}

	void testNewMeasure(varray<Spline>& outer, varray<varray<Spline>>& inner, varray<ContourData>& result)
	{
		//所有轮廓线
		varray<varray<Spline>> surf;

		//初始化surf
		surf.push_back(outer);
		for (auto& i : inner)
		{
			surf.push_back(i);
		}

		//所有边界线
		varray<Spline> allSpline;
		for (auto& i : surf)
		{
			for (auto& j : i)
			{
				allSpline.push_back(j);
			}
		}

		//临时变量
		ContourData cd;

		//层次遍历所用的队列
		queue<SfCtainTreeNode*> Qnodes;

		//层次遍历后根遍历所用队列
		stack<SfCtainTreeNode*>Snode;
		//初略构建几何域包含树
		SfCtainTreeNode* root = CreateTree(surf);

		//层次遍历指针
		SfCtainTreeNode* cur = nullptr;

		//辅助工具
		RWGeometric rwg;
		varray<Spline> tempChackLine;

		Qnodes.push(root);
		//cur节点的子节点
		varray<SfCtainTreeNode*> allNode;
		varray<SfCtainTreeNode*> childnodes;

		//层次遍历几何域包含树，
		while (!Qnodes.empty())
		{
			cur = Qnodes.front();
			Qnodes.pop();

			//用栈记录后根遍历的顺序
			Snode.push(cur);

			//实现层次遍历
			for (auto it = cur->childs.begin(); it != cur->childs.end(); ++it)
			{
				Qnodes.push(*it);
			}
		}
		//根据后根遍历顺序，实现由内至外创建连接线
		while (!Snode.empty())
		{
			cur = Snode.top();
			Snode.pop();

			//为叶子节点，无内轮廓，不添加连接线
			if (cur->childs.empty())
			{
				continue;
			}

			allNode.clear();
			allNode.push_back(cur);
			childnodes.clear();

			//以cur为外轮廓，子节点为内轮廓，创建连接线
			for (auto it = cur->childs.begin(); it != cur->childs.end(); ++it)
			{
				childnodes.push_back(*it);
			}

			//排序内轮廓顺序
			Order_Nodes(childnodes);

			//将排序后子节点数组存入allNode中
			for (auto it = childnodes.begin(); it != childnodes.end(); ++it)
			{
				allNode.push_back(*it);
			}
			if (childnodes.size() > 1)
			{
				allNode.push_back(cur);
			}

			//记录allNode数组中相邻的两个节点

			auto back = allNode.begin() + 1;

			//存放每两个边界区域之间的候选连接线
			//存放两轮廓间合适的连接线<连接线长度，连接线相关曲线的信息>
			multimap<double, ContourData> bestLine;
			varray<AddLineData> temp;
			//中间结果，用于存放不与其他轮廓线以及连接线相交的曲线
			varray<ContourData> mid_result;
			for (auto front = allNode.begin(); front != allNode.end(); ++front)
			{
				bestLine.clear();
				for (auto back = allNode.begin(); back != allNode.end(); ++back)
				{
					temp = BestAddLine(*front, *back);
					DeduplicateLines(temp);

					//判断候选线是否与其他区域以及连接线相交
					for (auto& j : temp)
					{
						//记录连接线
						Spline s = j.s;

						//记录是否与其他边界区域相交
						bool flag = true;

						//对各个候选连接线判断是否跨域边界区域
						for (auto& k : surf)
						{
							//是否跨越轮廓线围成的区域
							if (this->SplinePolygonIfCross0(s, k))
							{
								flag = false;
							}
						}

						//将候选线分别与已选择的连接线进行判断是否相交
						for (auto it = result.begin(); it != result.end(); it++)
						{
							if (this->if_Intersect(it->adl.s, s) > 0)
							{
								flag = false;
							}
						}

						//不与任何边界或连接线相交，则存入辅助线相关的信息如：辅助线、长度、相关边界的序号（在operate_array数组中的）
						if (flag)
						{
							cd.adl = j;
							cd.contour1 = (*front)->num;
							cd.contour2 = (*back)->num;
							double len = s.GetLength(s.m_CtrlPts.size());
							if (len != 0)
							{
								bestLine.insert(pair<double, ContourData>(len, cd));
							}
						}
					}
				}
				mid_result.clear();
				int len;
				int n;
				//将符合要求的连接线存入mid_result数组中，再从mid_result数组中选择连接线
				if (!bestLine.empty())
				{
					for (auto it = bestLine.begin(); it != bestLine.end(); it++)
					{
						mid_result.push_back(it->second);
					}

					//去掉重复的曲线
					DeduplicateLines(mid_result);

					//候选线不为空，选择合适得连接线(可以多选)
					if (!mid_result.empty())
					{
						len = mid_result.end() - mid_result.begin();
						cout << "共" << len << "条候选线，输入需要保留的连接线：";
						{
							varray<Spline> chack;
							chack.clear();
							for (auto& i : mid_result)
							{
								chack.push_back(i.adl.s);
							}
							rwg.WriteSpline("E:\\Model\\PlaneQuad\\ChooseLine.txt", chack);
							for (auto& i : allSpline)
							{
								chack.push_back(i);
							}
							rwg.WriteSpline("E:\\Model\\PlaneQuad\\Chack.txt", chack);
							//可以选择多条连接线
							while (cin >> n) {
								if (n >= 0 || n < len)
								{
									result.push_back(mid_result[n]);
								}
								char ch = getchar();//读取下一个字符，为换行符，则break
								if (ch == '\n')
									break;
							}
						}
					}
				}
			}
		}
	}

	/**
	* @brief   : 对surf二维数组中的轮廓两两之间创建连接线
	* @param[I]: 所有轮廓
	* @param[0]: 创建的连接线
	* @return  :
	* @note    :
	**/
	void createAddline(varray<varray<Spline>>& surf, varray<Spline>& result) {
		queue<SfCtainTreeNode*> nodes;
		SfCtainTreeNode* cur = nullptr;//临时指向当前包含树节点

		RWGeometric rwg;
		varray<Spline> res;
		varray<Spline> temp;

		varray<varray<Spline>> re;						//存放各层连接线
		SfCtainTreeNode* root = CreateTree(surf);
		nodes.push(root);
		//层次遍历包含树
		while (!nodes.empty())
		{
			cur = nodes.front();
			nodes.pop();

			varray<Spline> outline;//当前节点的轮廓线
			varray<Spline> inLine;//当前节点的某一个内轮廓线
			varray<varray<Spline>> inlines;//当前节点所有的内轮廓线

			outline.clear();
			inLine.clear();
			inlines.clear();
			//外轮廓线
			outline = cur->outLines;
			varray<SfCtainTreeNode*> childnodes;
			childnodes.clear();
			if (!cur->childs.empty()) {
				//存入cur节点的所有子节点
				list< SfCtainTreeNode*>::iterator it = cur->childs.begin();
				for (; it != cur->childs.end();) {
					nodes.push(*it);
					childnodes.push_back(*it);				//存放子节点，进行后续处理
					it++;
				}
			}
			for (auto& i : childnodes) {
				inLine = i->outLines;
				inlines.push_back(inLine);
			}
			//补充：后续可以添加内轮廓的排序，实现由内向外的顺序，便于连接线的生成
			varray<Spline> outlineTmp;
			for (int i = 0; i < childnodes.size(); ++i) {
				//若内部只有两个内轮廓
				if (childnodes.size() == 2) {
					//最后一个内轮廓与外轮廓相连接、内轮廓之间生成连接线
					if (i == childnodes.size() - 1) {
						inLine = childnodes[i]->outLines;
						re.push_back(creatSplineOfPolgons(outline, inLine, inlines, result));
						//内轮廓之间生成连接线
						inLine = childnodes[i]->outLines;
						outlineTmp = childnodes[i - 1]->outLines;
						re.push_back(creatSplineOfPolgons(outlineTmp, inLine, inlines, result));
					}
					if (i == 0) {
						//第一个内轮廓与外轮廓相连接
						inLine = childnodes[i]->outLines;
						re.push_back(creatSplineOfPolgons(outline, inLine, inlines, result));
					}
				}

				//内轮廓不为2
				else {
					//第一个和外轮廓相连接
					if (i == 0) {
						inLine = childnodes[i]->outLines;
						re.push_back(creatSplineOfPolgons(outline, inLine, inlines, result));
					}
					else if (i == childnodes.size() - 1) {
						//最后一个内轮廓与其前面轮廓生成连接线
						inLine = childnodes[i]->outLines;
						varray<Spline> tmpSL;
						//判断最后一个内轮廓与前面内轮廓是否存在连接线
						for (int i = childnodes.size() - 1; i > 0; i--) {
							outlineTmp = childnodes[i - 1]->outLines;
							tmpSL = creatSplineOfPolgons(outlineTmp, inLine, inlines, result);
							if (tmpSL.size() != 0) {
								re.push_back(tmpSL);
								break;						//若与某个内轮廓存在连接线，则跳出
							}
						}
						//最后一个内轮廓与外轮廓相连接
						re.push_back(creatSplineOfPolgons(outline, inLine, inlines, result));
					}
					else {
						//内轮廓之间生成连接线
						inLine = childnodes[i]->outLines;
						outlineTmp = childnodes[i - 1]->outLines;
						varray<Spline> tmpSL;
						tmpSL = creatSplineOfPolgons(outlineTmp, inLine, inlines, result);
						//若该内轮廓与前一个内轮廓存在连接线
						if (tmpSL.size() != 0) {
							re.push_back(tmpSL);
						}
						//若该内轮廓与前一个内轮廓不存在连接线，则将其与外轮廓之间生成连接线
						else {
							re.push_back(creatSplineOfPolgons(outline, inLine, inlines, result));
						}
					}
				}
			}
			//res存放所有
			for (auto i : re) {
				for (auto j : i) {
					res.push_back(j);
				}
			}
		}

		//连接线
		rwg.WriteSpline("E:\\Model\\PlaneQuad\\all.txt", res);
		for (auto i : surf) {
			for (auto j : i) {
				res.push_back(j);
			}
		}
		//所有可能结果
		rwg.WriteSpline("E:\\Model\\PlaneQuad\\allAddlines.txt", res);
	}

	/**
	* @brief   : 向索引表中插入一个元组
	* @param[I]: curveNumber	曲线编号
	* @param[I]: contourNumber	轮廓编号
	* @param[I]: value_U		参数值
	* @param[O]: Map			索引表
	* @return  :
	* @note    :
	**/
	void addTuple(const int& curveNumber, const int& contourNumber, const double& value_U, map<int, map<int, set<double>>>& Map)
	{
		map<int, set<double>> temp_map;
		set<double> temp_set;
		temp_map.clear();
		temp_set.clear();
		//一级索引表中缺少contourNumber对应的元组
		if (Map.find(contourNumber) == Map.end())
		{
			//创建set数组
			temp_set.insert(value_U);

			//创建二级索引表，并插入创建一条元组
			temp_map.insert(pair<int, set<double>>(curveNumber, temp_set));

			//向一级索引表中插入元组
			Map.insert(pair<int, map<int, set<double>>>(contourNumber, temp_map));
		}

		//一级索引中存在contourNumber对应的元组
		else
		{
			//记录一级索引表中contourNumber对应元组的迭代器
			auto it1 = Map.find(contourNumber);

			//获取迭代器it1对应的二级索引表
			temp_map = it1->second;

			//二级索引表中不存在索引号为curveNumber的元组、
			if (temp_map.find(curveNumber) == temp_map.end())
			{
				//创建set数组
				temp_set.insert(value_U);

				//向二级索引中添加元组
				it1->second.insert(pair<int, set<double>>(curveNumber, temp_set));
			}
			//二级索引表中存在索引号为curveNumber的元组
			else
			{
				//通过it1，获得二级索引表中curveNumber对应元组迭代器
				auto it2 = it1->second.find(curveNumber);
				//向curveNumber对应元组
				it2->second.insert(value_U);
			}
		}
	}

	/**
	* @brief   : 通过结果信息result创建截断曲线所用的索引表
	* @param[I]: 结果信息		result
	* @param[O]: 索引表		mp
	* @return  :
	* @note    :
	**/
	void operateResult(const varray<ContourData>& result, map<int, map<int, set<double>>>& mp)
	{
		map<int, set<double>> temp_map;
		set<double> temp_set;
		double u;
		int curveNumber;
		int contourNumber;
		double value_U;
		//对每一条结果考虑是否有轮廓号相同，曲线号相同
		//若都相同，说明是对同一轮廓的同一曲线进行截断
		//若只有轮廓线相同，则说明是对同一轮廓线的不同曲线进行截断
		//以下程序遵循二级索引表的逻辑，构建map
		//元组的定义参考关系数据库
		for (auto& contourData : result)
		{
			//记录轮廓编号
			contourNumber = contourData.contour1;
			//记录轮廓线编号
			curveNumber = contourData.adl.pa1.first;
			//记录对应的u值
			value_U = contourData.adl.pa1.second;

			//参数值为0或1，不需要截断
			if (value_U != 0 && value_U != 1)
			{
				addTuple(curveNumber, contourNumber, value_U, mp);
			}

			//记录轮廓编号
			contourNumber = contourData.contour2;
			//记录轮廓线编号
			curveNumber = contourData.adl.pa2.first;
			//记录对应的u值
			value_U = contourData.adl.pa2.second;
			//参数值为0或1，不需要截断
			if (value_U != 0 && value_U != 1)
			{
				addTuple(curveNumber, contourNumber, value_U, mp);
			}
		}

		/*for (auto &i : mp)
		{
			cout << "轮廓号：" << i.first << endl;
			for (auto &j : i.second)
			{
				cout << "轮廓线号：" << j.first << endl;
				cout << "参数值：";
				for (auto &k : j.second)
				{
					cout << k << " " << endl;
				}
			}
			cout << endl;
		}*/
	}

	/**
	* @brief   : 某条线段垂直x轴，导致斜率无穷大，需要特俗处理
	* @param[I]: 端点p1
	* @param[I]: 圆弧中间点p2
	* @param[I]: 端点p3
	* @return  : none
	* @note    : p1p2构成的曲线垂直x轴，说明圆心与p1构成一条与x轴平行的线段，因此圆心与p1等高
	**/
	Vec4 getCenterByThreePoint(Vec4 p1, Vec4 p2, Vec4 p3)
	{
		Vec4 center;
		Vec4 p21 = p2 - p1;
		Vec4 p23 = p2 - p3;
		//确保p21垂直x轴
		if (p23.x == 0)
		{
			return getCenterByThreePoint(p3, p2, p1);
		}

		center.y = p1.y;
		double k = p23.y / p23.x;
		center.x = p3.x + k * (center.y - p3.y);
		return center;
	}

	/**
	* @brief   : 给定圆弧，求圆弧对应圆心坐标
	* @param[I]: 给定圆弧
	* @param[O]: 圆弧对应圆心坐标
	* @return  : 是否存在圆心
	* @note    : 利用p2p1,p2p3所构成的直线分别与经过p1,p3的半径垂直这一特性，建立方程，求解圆心坐标
	**/
	bool getCenter(const Spline& S, Vec4& center)
	{
		Vec4 p1 = S.m_CtrlPts[0];
		Vec4 p2 = S.m_CtrlPts[1];
		Vec4 p3 = S.m_CtrlPts[2];
		//曲线上的三个点
		Vec4 p21 = p2 - p1;				//p1p2的方向向量
		Vec4 p23 = p2 - p3;				//p3p2的方向向量
		if (JudgeTwoPointsCoincide(p21.Normalize(), p23.Normalize()))
		{
			return false;
		}

		//方向向量的x坐标为0，导致线段的斜率无穷大，需要特殊处理
		if (p21.x == 0 || p23.x == 0)
		{
			center = getCenterByThreePoint(p1, p2, p3);
			return true;
		}

		double k21 = p21.y / p21.x;//直线p1p2的斜率
		double k23 = p23.y / p23.x;//直线p3p2的斜率

		center.y = (p3.x - p1.x + k23 * p3.y - k21 * p1.y) / (k23 - k21);//圆心计算公式
		center.x = p1.x + k21 * p1.y - k21 * center.y;
		return true;
	}

	/**
	* @brief   : 利用有序数组分割曲线
	* @param[I]: 保存参数的有序数组
	* @param[i]: 待分割的曲线
	* @return  :
	* @note    :
	**/
	varray<Spline> segmentCurve(const set<double>& value, const Spline& S)
	{
		Spline0 s0;
		Vec4 center;
		varray<Vec4> points;
		varray<Spline> result;
		Spline s;
		result.clear();
		//将两端点及截断位置的坐标存入points数组中
		points.push_back(S.GetLinePoint(0));
		for (auto& i : value)
		{
			points.push_back(S.GetLinePoint(i));
		}
		points.push_back(S.GetLinePoint(1));

		//相邻两节点之间构建线段
		auto front = points.begin();
		auto back = points.begin() + 1;
		//判断是否为直线
		if (if_StraightLine(S))
		{
			for (; back != points.end(); ++back, ++front)
			{
				s = s0.getSpline(*front, *back);
				result.push_back(s);
			}
		}
		//若为曲线需要找出圆心的位置
		else
		{
			getCenter(S, center);
			cout << center.x << " " << center.y << " " << center.z << endl;
			for (; back != points.end(); ++back, ++front)
			{
				s = s0.getArcSpline(*front, *back, center);
				result.push_back(s);
			}
		}
		return result;
	}

	/**
	* @brief   : 利用二级索引表重构轮廓线
	* @param[I]: 外轮廓
	* @param[I]: 内轮廓
	* @param[O]: 二级索引表
	* @return  :
	* @note    :利用对result处理之后的二级索引表，处理轮廓曲线，主要是截断一些曲线
	**/
	void refineBoundaryByMap(varray<Spline>& outer, varray<varray<Spline>>& inner, const map<int, map<int, set<double>>>& mp)
	{
		map<int, set<double>> temp_map;

		//某条曲线截断位置数组
		set<double> temp_set;

		///保存某个轮廓截断后生成的曲线
		varray<Spline> result;

		//保存某条曲线截断后生成的曲线
		varray<Spline> temp;

		//记录坐标的栈
		stack<int> index;

		//轮廓号
		int boundaryNum;

		//曲线号
		int curveNum;
		for (auto it1 = mp.begin(); it1 != mp.end(); ++it1)
		{
			result.clear();
			boundaryNum = it1->first;
			temp_map = it1->second;
			for (auto it2 = temp_map.begin(); it2 != temp_map.end(); ++it2)
			{
				temp.clear();
				curveNum = it2->first;
				temp_set = it2->second;
				//外轮廓
				if (boundaryNum == -1)
				{
					temp = segmentCurve(temp_set, outer[curveNum]);
				}

				//内轮廓
				else
				{
					temp = segmentCurve(temp_set, inner[boundaryNum][curveNum]);
				}

				//存储截断生成的曲线
				for (auto& i : temp)
				{
					result.push_back(i);
				}
				index.push(curveNum);
			}

			//因为删除某个元素会影响后面元素的坐标
			//因此利用栈实现从后往前删除
			//外轮廓
			if (boundaryNum == -1)
			{
				while (!index.empty())
				{
					outer.erase(outer.begin() + index.top());
					index.pop();
				}
				for (auto& i : result)
				{
					outer.push_back(i);
				}
			}
			//内轮廓
			else
			{
				while (!index.empty())
				{
					inner[boundaryNum].erase(inner[boundaryNum].begin() + index.top());
					index.pop();
				}

				for (auto& i : result)
				{
					inner[boundaryNum].push_back(i);
				}
			}
		}
	}

	/**
	* @brief   : 利用几何域包含树获取剖分完成之后的所有曲面
	* @param[I]: 几何域包含树树根
	* @param[O]: 剖分所得曲面
	* @return  :
	* @note    :
	**/
	void getAllSurface(SfCtainTreeNode* root, varray<SplineSurface>& allSurf)
	{
		varray<Spline> allLines;
		queue<SfCtainTreeNode*> q;
		q.push(root);
		allLines.clear();
		while (!q.empty())
		{
			varray<SplineSurface> tmpsf;
			SfCtainTreeNode* cur = q.front();
			q.pop();
			//存入当前节点所有曲线
			for (auto i : cur->quadPolNumber)
			{
				for (auto j : i) {
					allLines.push_back(cur->allLines[j]);
				}
			}
			//存入当前节点所有曲面
			cur->GetSurfs(tmpsf);
			for (auto& s : tmpsf) {
				allSurf.push_back(s);
			}

			list<SfCtainTreeNode*>::iterator it = cur->childs.begin();
			for (; it != cur->childs.end(); it++) {
				q.push(*it);
			}
		}
		RWGeometric rwg;
		rwg.WriteSpline("E:\\Model\\PlaneQuad\\AllLinesAfterQuad.txt", allLines);
	}

	/**
	* @brief   : 无辅助线，自动剖分
	* @param[I]: 外轮廓
	* @param[I]: 内轮廓数组
	* @param[I]: 亏格属性数组
	* @param[O]: 剖分所得曲面
	* @return  :
	* @note    : 无额外辅助先，按照最短距离的原则构建连接线
	**/
	void quad_Lee(varray<Spline>& outer, varray<varray<Spline>>& inner, varray<bool>& genus, varray<SplineSurface>& allSurf)
	{
		RWGeometric rwg;
		varray<varray<Spline>>allBoundary;
		varray<Spline> addlines;
		varray<ContourData> result;
		varray<Spline> allLines;
		map<int, map<int, set<double>>>mp;
		//存储所有轮廓
		allBoundary.push_back(outer);
		for (auto& i : inner)
		{
			allBoundary.push_back(i);
		}

		//获取连接线结果
		createAddline(outer, inner, result);

		//存储连接线
		for (auto& i : result)
		{
			addlines.push_back(i.adl.s);
		}

		//提取所有轮廓线以及连接线
		allLines.clear();
		for (auto& i : addlines)
		{
			allLines.push_back(i);
		}

		for (auto& i : allBoundary)
		{
			for (auto& j : i)
			{
				allLines.push_back(j);
			}
		}
		rwg.WriteSpline("E:\\Model\\PlaneQuad\\AllLinesBeforeRefine.txt", allLines);

		//处理连接线结果
		operateResult(result, mp);

		//根据连接线结果处理边界线
		refineBoundaryByMap(outer, inner, mp);

		//提取所有轮廓线以及连接线
		allLines.clear();
		for (auto& i : addlines)
		{
			allLines.push_back(i);
		}
		allBoundary.clear();
		allBoundary.push_back(outer);
		for (auto& i : inner)
		{
			allBoundary.push_back(i);
		}
		for (auto& i : allBoundary)
		{
			for (auto& j : i)
			{
				allLines.push_back(j);
			}
		}
		rwg.WriteSpline("E:\\Model\\PlaneQuad\\AllLinesAfterRefine.txt", allLines);

		//记录每个轮廓的可分割性，0为不可分，1为可分割
		varray<varray<int>>seg;

		//默认设置为都可分割
		seg.push_back(varray<int>(outer.size(), 1));
		for (int i = 0; i < inner.size(); i++)
		{
			seg.push_back(varray<int>(inner[i].size(), 1));
		}

		//建立几何域包含树
		SfCtainTreeNode* root = CreateSurfContainTree(allBoundary, addlines, seg, genus);

		//剖分
		QuadWithContainTree(root);

		//以下部分是从包含树中取出截面，可以单独封装成函数
		getAllSurface(root, allSurf);

		//剖分调整
		quadAdjustUV(allSurf);
	}
	/**
	* @brief   : 有辅助线，自动剖分
	* @param[I]: 外轮廓
	* @param[I]: 内轮廓数组
	* @param[I]: 亏格属性数组
	* @param[O]: 剖分所得曲面
	* @param[I]: 辅助线
	* @return  :
	* @note    : 有额外辅助先，按照最短距离的原则构建连接线
	**/
	void quad_Lee(varray<Spline>& outer, varray<varray<Spline>>& inner, varray<bool>& genus, varray<SplineSurface>& allSurf, const varray<Spline>& extraAddLine)
	{
		RWGeometric rwg;
		varray<varray<Spline>>allBoundary;
		varray<Spline> addlines;
		varray<ContourData> result;
		varray<Spline> allLines;
		map<int, map<int, set<double>>>mp;
		//存储所有轮廓
		allBoundary.push_back(outer);
		for (auto& i : inner)
		{
			allBoundary.push_back(i);
		}

		//获取连接线结果
		createAddline(outer, inner, result);

		//存储连接线
		for (auto& i : result)
		{
			addlines.push_back(i.adl.s);
		}

		for (auto& i : extraAddLine)
		{
			addlines.push_back(i);
		}

		//提取所有轮廓线以及连接线
		allLines.clear();
		for (auto& i : addlines)
		{
			allLines.push_back(i);
		}

		for (auto& i : allBoundary)
		{
			for (auto& j : i)
			{
				allLines.push_back(j);
			}
		}
		rwg.WriteSpline("E:\\Model\\PlaneQuad\\AllLinesBeforeRefine.txt", allLines);

		//处理连接线结果
		operateResult(result, mp);

		//根据连接线结果处理边界线
		refineBoundaryByMap(outer, inner, mp);

		//提取所有轮廓线以及连接线
		allLines.clear();
		for (auto& i : addlines)
		{
			allLines.push_back(i);
		}
		allBoundary.clear();
		allBoundary.push_back(outer);
		for (auto& i : inner)
		{
			allBoundary.push_back(i);
		}
		for (auto& i : allBoundary)
		{
			for (auto& j : i)
			{
				allLines.push_back(j);
			}
		}
		rwg.WriteSpline("E:\\Model\\PlaneQuad\\AllLinesAfterRefine.txt", allLines);

		//记录每个轮廓的可分割性，0为不可分，1为可分割
		varray<varray<int>>seg;

		//默认设置为都可分割
		seg.push_back(varray<int>(outer.size(), 1));
		for (int i = 0; i < inner.size(); i++)
		{
			seg.push_back(varray<int>(inner[i].size(), 1));
		}

		//建立几何域包含树
		SfCtainTreeNode* root = CreateSurfContainTree(allBoundary, addlines, seg, genus);

		//剖分
		QuadWithContainTree(root);

		//以下部分是从包含树中取出截面，可以单独封装成函数
		getAllSurface(root, allSurf);

		//剖分调整
		quadAdjustUV(allSurf);
	}

	/**
	* @brief   : 逆时针排序闭合曲线
	* @param[I]: 闭合轮廓线
	* @return  :
	* @note    :
	**/
	void orderEdgeAntiClock0(varray<Spline>& sl)
	{
		Model_Solution m;
		m.OrderLinesAntioclock(sl);
		decltype(sl.begin()) back;

		Vec4 p;
		Vec4 p_begin;
		Vec4 p_end;

		int j;
		for (int i = 0; i < sl.size(); i++)
		{
			if (i == sl.size() - 1)
			{
				j = 0;
			}
			else
			{
				j = i + 1;
			}
			p = sl[i].m_CtrlPts[0];
			p_begin = sl[j].m_CtrlPts[0];
			p_end = sl[j].m_CtrlPts[sl[j].m_CtrlPts.size() - 1];

			if (JudgeTwoPointsCoincide(p, p_begin) || JudgeTwoPointsCoincide(p, p_end))
			{
				sl[i].m_CtrlPts[0] = sl[i].m_CtrlPts[sl[i].m_CtrlPts.size() - 1];
				sl[i].m_CtrlPts[sl[i].m_CtrlPts.size() - 1] = p;
			}
		}
	}


	void orderEdgeClockwise(varray<Spline>& sl)
	{
		orderEdgeAntiClock0(sl);
		stack<Spline> m_stack;
		Spline s;
		Vec4 temp;
		for (int i = 0; i < sl.size(); ++i)
		{
			temp = sl[i].m_CtrlPts[0];
			sl[i].m_CtrlPts[0] = sl[i].m_CtrlPts[2];
			sl[i].m_CtrlPts[2] = temp;
			m_stack.push(sl[i]);
		}
		sl.clear();
		while (!m_stack.empty())
		{
			sl.push_back(m_stack.top());
			m_stack.pop();
		}
	}
	//凸点判断
	void FindBump(varray<Spline>& boundary)
	{
		//对曲线进行排序
		this->orderEdgeAntiClock0(boundary);

		//获取所有的顶点
		varray<Vec4> points;
		for (auto& i : boundary)
		{
			points.push_back(i.m_CtrlPts[0]);
			if (!this->if_StraightLine(i))
			{
				points.push_back(i.GetLinePoint(0.5));
			}
		}

		Vec4 line1;
		Vec4 line2;
		int front;
		int back;
		double result;
		int size = points.size();
		for (int i = 0; i < size; i++)
		{
			front = (i - 1 + size) % size;
			back = (i + 1) % size;

			line1 = points[i] - points[front];
			line2 = points[back] - points[i];

			result = line1.x * line2.y - line2.x * line1.y;
			if (result < 0)
			{
				cout << "p" << i + 1 << "是凹顶点" << endl;
			}
			else
			{
				cout << "p" << i + 1 << "是凸顶点" << endl;
			}
		}
	}

	void test_FindBump()
	{
		double r1 = 4;
		double r2 = 10;
		double l1 = 3;
		double l2 = 10;
		double l3 = 15;
		double l = r1 + l1 + l2 + l3;
		double h1 = 10;
		double h2 = 20;
		Vec4 p1 = { 0,r1,0,1 };
		Vec4 p2 = { r1,0,0,1 };
		Vec4 p3 = { r1 + l1,0,0,1 };
		Vec4 p4 = { r1 + l1,h1,0,1 };
		Vec4 p5 = { r1 + l1 + l2,h1,0,1 };
		Vec4 p6 = { r1 + l1 + l2,0,0,1 };
		Vec4 p7 = { l,0,0,1 };
		Vec4 p8 = { l,h2,0,1 };
		Vec4 p9 = { r2,h2,0,1 };
		Vec4 p10 = { 0,r2 + h2,0,1 };
		Vec4 o1 = { 0,h2,0,1 };
		Vec4 o2 = { 0,0,0,1 };

		varray<Spline> S;
		varray<Spline> temp;
		Model_Solution m;
		Spline0 s0;
		S.push_back(s0.getArcSpline(p1, p2, o2));
		S.push_back(s0.getArcSpline(p10, p9, o1));
		S.push_back(s0.getSpline(p9, p8));
		S.push_back(s0.getSpline(p2, p3));
		S.push_back(s0.getSpline(p4, p3));
		S.push_back(s0.getSpline(p5, p4));
		S.push_back(s0.getSpline(p5, p6));
		S.push_back(s0.getSpline(p6, p7));
		S.push_back(s0.getSpline(p8, p7));
		m.MirrorLines(S, temp, 1);

		for (const auto& i : temp)
		{
			S.push_back(i);
		}
		RWGeometric rwg;
		this->FindBump(S);
		rwg.WriteSpline("E:\\Model\\PlaneQuad\\test_FindBump.txt", S);
	}

	//剖分函数 outer为外轮廓，inner为内轮廓，addlines为连接线， genus为每个轮廓的是否为孔， allSurf为剖分结果
	void quad(varray<Spline>& outer, varray<varray<Spline>>& inner, varray<Spline>& addlines, varray<bool>& genus, varray<SplineSurface>& allSurf) {
		varray<Spline>allLines;
		//分别读取外部轮廓曲线，内部轮廓曲线和额外辅助线
		//其中辅助线的作用是将孔和边界连接起来
		varray<SplineSurface> NS;
		varray<varray<Spline>> surf;					//将每个轮廓分别装入数组
		surf.push_back(outer);
		for (auto& suf : inner)
		{
			surf.push_back(suf);
		}

		varray<varray<int>>seg;							//将每个轮廓的可分割性记录下来，0为不可分，1为可分割
		seg.push_back(varray<int>(outer.size(), 1));    //默认设置为都可分割
		for (int i = 0; i < inner.size(); i++)
		{
			seg.push_back(varray<int>(inner[i].size(), 1));
		}

		SfCtainTreeNode* root = CreateSurfContainTree(surf, addlines, seg, genus);//建立几何域包含树
		QuadWithContainTree(root);						//剖分

		//以下部分是从包含树中取出截面，可以单独封装成函数
		//queue<SfCtainTreeNode*> q;
		//q.push(root);
		//allLines.clear();
		//while (!q.empty())
		//{
		//	varray<SplineSurface> tmpsf;
		//	SfCtainTreeNode*cur = q.front();
		//	q.pop();
		//	//存入当前节点所有曲线
		//	for (auto i : cur->quadPolNumber)
		//	{
		//		for (auto j : i) {
		//			allLines.push_back(cur->allLines[j]);
		//		}
		//	}
		//	//存入当前节点所有曲面
		//	cur->GetSurfs(tmpsf);
		//	for (auto& s : tmpsf) {
		//		allSurf.push_back(s);
		//	}

		//	list<SfCtainTreeNode*>::iterator it = cur->childs.begin();
		//	for (; it != cur->childs.end(); it++) {
		//		q.push(*it);
		//	}
		//}
		getAllSurface(root, allSurf);
		//剖分调整
		quadAdjustUV(allSurf);
	}

	//剖分函数（无需辅助线参数）
	void quad(varray<Spline>& outer, varray<varray<Spline>>& inner, varray<bool>& genus, varray<SplineSurface>& allSurf) {
		RWGeometric rwg;
		varray<Spline>allLines;

		//分别读取外部轮廓曲线，内部轮廓曲线和额外辅助线
		//其中辅助线的作用是将孔和边界连接起来,构成零亏格的几何域
		varray<SplineSurface> NS;
		varray<varray<Spline>> surf;					//将每个轮廓分别装入数组

		//将外轮廓存入surf中
		surf.push_back(outer);

		//将内轮廓存入suf中
		for (auto& suf : inner)
			surf.push_back(suf);

		//提取所有内轮廓轮廓线
		for (auto& i : surf) {
			for (auto& j : i) {
				allLines.push_back(j);
			}
		}

		//为调试查看用
		rwg.WriteSpline("E:\\Model\\PlaneQuad\\AllBoundary.txt", allLines);

		//将每个轮廓域的可分割性记录下来，0为不可分，1为可分割
		//默认设置为都可分割
		varray<varray<int>>seg;
		seg.push_back(varray<int>(outer.size(), 1));    //默认设置为都可分割
		for (int i = 0; i < inner.size(); i++)
			seg.push_back(varray<int>(inner[i].size(), 1));

		//存放连接线
		varray<Spline> addLines;

		//自动生成连接线
		createAddline(surf, addLines);
		//createAddline0(outer, inner, addLines);

		//输出轮廓和连接线(用于自己查看，可注释)
		varray<Spline> res;
		for (auto i : surf) {
			for (auto j : i) {
				res.push_back(j);
			}
		}
		for (auto i : addLines) {
			res.push_back(i);
		}
		rwg.WriteSpline("E:\\Model\\PlaneQuad\\AddSplines.txt", res);
		rwg.WriteSpline("E:\\Model\\PlaneQuad\\OnlyAddSplines.txt", addLines);

		SfCtainTreeNode* root = CreateSurfContainTree(surf, addLines, seg, genus);//建立几何域包含树

		QuadWithContainTree(root);	//剖分

		//以下部分是从包含树中取出截面，可以单独封装成函数
		queue<SfCtainTreeNode*> q;
		q.push(root);
		allLines.clear();
		while (!q.empty())
		{
			varray<SplineSurface> tmpsf;
			SfCtainTreeNode* cur = q.front();
			q.pop();
			//存入当前节点所有曲线
			for (auto i : cur->quadPolNumber)
			{
				for (auto j : i) {
					allLines.push_back(cur->allLines[j]);
				}
			}
			//存入当前节点所有曲面
			cur->GetSurfs(tmpsf);
			for (auto& s : tmpsf) {
				allSurf.push_back(s);
			}

			list<SfCtainTreeNode*>::iterator it = cur->childs.begin();
			for (; it != cur->childs.end(); it++) {
				q.push(*it);
			}
		}

		//剖分调整
		quadAdjustUV(allSurf);

		rwg.WriteSplineSurface("E:\\Model\\PlaneQuad\\allSurf.txt", allSurf);
	}

	//体模型去重
	static void removeRepeatVols(varray<SplineVolume>& Vols) {
		varray<varray<Vec4>> Rmaps;
		int len = Vols.size();
		int j = 0;
		for (int i = 0; i < len; i++) {
			varray<Vec4> cpts;
			cpts.push_back(Vols[i].GetVolPoint(0, 0, 0));
			cpts.push_back(Vols[i].GetVolPoint(0, 1, 0));
			cpts.push_back(Vols[i].GetVolPoint(1, 0, 0));
			cpts.push_back(Vols[i].GetVolPoint(0, 1, 1));
			bool isFind = false;
			for (auto& var : Rmaps) {
				if (distance(cpts[0], var[0]) < 0.001
					&& distance(cpts[1], var[1]) < 0.001
					&& distance(cpts[2], var[2]) < 0.001
					&& distance(cpts[3], var[3]) < 0.001) {
					isFind = true;
					break;
				}
			}
			if (isFind) {
				continue;
			}
			else {
				Rmaps.push_back(cpts);
				Vols[j++] = Vols[i];
			}
		}
		Vols.resize(j);
	}

	//该函数是让所有边朝着一个方向
	//边的排序（达到从角点开始）参数：单个轮廓边界集合，排序之前轮廓曲线要求已经是有序存储的
	void orderEdges(varray<Spline>& sl) {
		//对于全曲线的情况，就从夹角最大的地方开始（夹角最大处肯定是个角点）
		//而对于存在曲线和直线的情况，目前没有进行考虑，后面需要添加上相应处理方式
		//起始样条应该从直线上开始（若存在直线）
		bool flag1 = 0;//设置标志位，判断是否存在直线

		varray<Spline> tmp1;//临时存放排序好的曲线
		int num = sl.size() - 1;

		Spline SL;//起点曲线
		SL = sl[0];

		double tmpAng = 0;//当轮廓为全曲线的情况，记录两样条夹角大小，最后选择最大处的夹角作为角点
		double ang = 0;

		//找到夹角最大的边
		for (int i = 1; i <= num; ++i) {
			//v1，v2为相邻曲线的向量
			Vec4 v1, v2;
			v1 = (sl[i - 1].m_CtrlPts[2] - sl[i - 1].m_CtrlPts[0]);
			v2 = (sl[i].m_CtrlPts[2] - sl[i].m_CtrlPts[0]);
			ang = calAngle(v1, v2);

			//找到直线上的边
			if (ang == 0) {//共线的情况
				SL = sl[i - 1];
				flag1 = 1;
				break;//原先的代码在这里将break注释掉，但我感觉这里加上break更好
				//因为找到了直线上的边，并且加上break时间复杂度更低，2023.05.09
			}
			else {//不共线（）
				if (ang > tmpAng) {//找夹角最大的那条边
					tmpAng = ang;
					SL = sl[i];//这里为什么不是sl[i-1]，2023.05.09（可能是因为i-1与i条曲线是一定不共线，且第i条之后可能与别的曲线共线）
				}
				//continue;
			}
		}

		//***************
		//查看找到的夹角最大的边
		varray<Spline> t;
		t.push_back(SL);
		RWGeometric rwg;
		rwg.WriteSpline("aOrderSL.txt", t);
		//***************

		//存在直线边界，则从该直线边界开始排序
		if (flag1) {
			//先将目标样条及其后面的样条存入容器，再将其前面的样条存入容器，达到逆时针排序，并且是从最大夹角边处开始
			varray<Spline> tmpSL1;//存放目标样条前面的样条
			bool flag = false;

			//因为之前已经逆序的内轮廓，所以找到SL后，之后的都是有序的
			//这部分代码是为了让SL对应的曲线置于数组第一个位置上
			for (auto& i : sl) {
				if (JudgeTwoLinesCoincide(SL, i)) {
					flag = true;
				}
				if (flag) {
					tmp1.push_back(i);
				}
				else {
					tmpSL1.push_back(i);
				}
			}
			for (auto& i : tmpSL1) {
				tmp1.push_back(i);
			}
			sl.clear();
			sl = tmp1;

			//寻找存在角点的边界
			varray<Spline> tmp;
			SL = sl[0];
			for (int i = 0; i <= num; ++i) {
				Vec3 v1, v2;//表示曲线的向量

				//v1为第i条曲线的前一条曲线的方向向量
				//即若i=0，则v1为第num-1条曲线
				//若i!=0，则v1为第i-1条曲线
				//v2为第i条曲线的方向向量
				v1 = (sl[(i - 1 + num) % num].m_CtrlPts[2] - sl[(i - 1 + num) % num].m_CtrlPts[0]).Normalize();
				v2 = (sl[i].m_CtrlPts[2] - sl[i].m_CtrlPts[0]).Normalize();

				//找到存在角点的边，即两方向向量不一致
				if (!JudgeTwoPointsCoincide(v1, v2)) {
					SL = sl[i];
					break;
				}
				else {
					continue;
				}
			}
		}

		//全曲线的情况
		//从存在角点的边界开始排序
		//先将目标样条及其后面的样条存入容器，再将其前面的样条存入容器，达到逆时针排序，并且从角点处开始
		varray<Spline> tmpSL;//存放目标样条前面的样条
		varray<Spline> tmp;
		bool flag = false;

		//将以SL放在第一个位置上
		for (auto& i : sl) {
			if (JudgeTwoLinesCoincide(SL, i)) {
				flag = true;
			}
			if (flag) {
				tmp.push_back(i);
			}
			else {
				tmpSL.push_back(i);
			}
		}
		for (auto& i : tmpSL) {
			tmp.push_back(i);
		}

		//存在样条方向不一致的情况，这里进行调整

		Vec4 v1;
		v1 = tmp[0].m_CtrlPts[0];

		//找到起始点，之后进行排序，使得每条曲线的方向与曲线的排序方向一致
		//判断第0条曲线是否存在一下两种情况：
		//1、第0条曲线的起始点与第1条曲线的起始点重合
		//2、第0条曲线的起始点与第1条曲线的终点重合
		//出现上述情况则需要改变方向，因为该曲线不应与下一条曲线的任意端点重合
		if (JudgeTwoPointsCoincide(v1, tmp[1].m_CtrlPts[0]) || JudgeTwoPointsCoincide(v1, tmp[1].m_CtrlPts[2])) {
			tmp[0].m_CtrlPts[0] = tmp[0].m_CtrlPts[2];
			tmp[0].m_CtrlPts[2] = v1;
		}

		//判断第i条曲线的方向是否为一致的方向
		//若该条曲线的尾端点，与上一条曲线的尾端点重合，则不是一致的
		//因此要改变第i条曲线的方向
		for (int i = 1; i < tmp.size(); i++) {
			//若该条曲线的尾端点与上一条曲线的尾端点重合
			if (JudgeTwoPointsCoincide(tmp[i - 1].m_CtrlPts[2], tmp[i].m_CtrlPts[2])) {
				Vec4 vTmp = tmp[i].m_CtrlPts[2];
				tmp[i].m_CtrlPts[2] = tmp[i].m_CtrlPts[0];
				tmp[i].m_CtrlPts[0] = vTmp;
			}
		}
		sl.clear();
		sl = tmp;
	}

	//都改变一下方向就是逆时针了？什么？
	//逆时针排序内轮廓
	void orderEdgeAntiClock(varray<varray<Spline>>& sl) {
		//划分网格后外轮廓是逆时针，内轮廓可能是顺时针，只需要改变内轮廓就行
		varray<varray<Spline>> tmp;
		for (int i = 0; i < sl.size(); i++) {
			if (i == 0) {//第一个为外轮廓，不需要重置为逆时针
				tmp.push_back(sl[0]);
			}
			else {
				varray<Spline> tmpSL;

				//将内轮廓的每条曲线改变一下方向
				for (auto& i : sl[i]) {
					Vec4 tmpV = i.m_CtrlPts[2];
					i.m_CtrlPts[2] = i.m_CtrlPts[0];
					i.m_CtrlPts[0] = tmpV;
				}
				for (int j = sl[i].size() - 1; j >= 0; j--) {
					tmpSL.push_back(sl[i][j]);
				}
				tmp.push_back(tmpSL);
			}
		}
		sl.clear();
		sl = tmp;
	}

	//外轮廓逆时针排序，内轮廓顺时针，需要将外轮廓转换为顺时针，便于圆弧段的生成（圆弧段的控制点顺序默认顺时针，后续可以增加参数，指定顺序）
	void orderEdgeAntiClock1(varray<varray<Spline>>& sl) {
		//划分网格后外轮廓是逆时针，内轮廓是顺时针，只需要改变内轮廓就行
		varray<varray<Spline>> tmp;
		for (int i = 0; i < sl.size(); i++) {
			if (i == 0) {
				varray<Spline> tmpSL;
				for (auto& i : sl[i]) {
					Vec4 tmpV = i.m_CtrlPts[2];
					i.m_CtrlPts[2] = i.m_CtrlPts[0];
					i.m_CtrlPts[0] = tmpV;
				}
				for (int j = sl[i].size() - 1; j >= 0; j--) {
					tmpSL.push_back(sl[i][j]);
				}
				tmp.push_back(tmpSL);
			}
			else {
				tmp.push_back(sl[i]);
			}
		}
		sl.clear();
		sl = tmp;
	}

	//获取轮廓按照一个方向排序的点
	void getOritientPoint(varray<Spline>& sl, varray<Vec4>& vec) {
		//使得所有边朝着一个方向
		orderEdges(sl);

		//***************
		//输出用于查看
		RWGeometric rwg;
		rwg.WriteSpline("orderEdges.txt", sl);
		//***************

		//记录曲线控制点数
		int ptNum = sl[0].m_CtrlPts.size();

		Vec4 p2, p1;//p1可以不用

		//这个判断有必要嘛，前面已经有了orderEdges(sl)
		//判断第一条曲线是否存在方向不一致的情况
		if (JudgeTwoPointsCoincide(sl[0].m_CtrlPts[0], sl[1].m_CtrlPts[0]) ||
			JudgeTwoPointsCoincide(sl[0].m_CtrlPts[0], sl[1].m_CtrlPts[ptNum - 1])) {
			p2 = sl[0].m_CtrlPts[0];
			//p1 = sl[0].m_CtrlPts[ptNum - 1];
			vec.push_back(sl[0].m_CtrlPts[ptNum - 1]);
		}
		else {
			p2 = sl[0].m_CtrlPts[ptNum - 1];
			//p1 = sl[0].m_CtrlPts[0];
			vec.push_back(sl[0].m_CtrlPts[0]);
		}

		//p1记录曲线的首控制点，用于存储到数组中
		//p2记录曲线的未控制点，用于查找下一条曲线
		//p2可能是下一条曲线的首控制点，也可能是下一条曲线的尾控制点
		//vec.push_back(p1);

		for (int i = 1; i < sl.size(); i++) {
			if (JudgeTwoPointsCoincide(sl[i].m_CtrlPts[0], p2)) {
				vec.push_back(p2);
				p2 = sl[i].m_CtrlPts[ptNum - 1];
			}
			else {
				vec.push_back(sl[i].m_CtrlPts[ptNum - 1]);
				p2 = sl[i].m_CtrlPts[0];
			}
		}
	}

	//平面展开后提取的边界，都是由弧线组成的情况，需要施加约束条件，简化过程中尽可能保持原形状
	//参数：组成弧线边界的点、简化后的点
	void dealCirBoundary(varray<varray<Vec4>>& vecs, varray<varray<Vec4>>& vecss) {
	}

	//简化一些小弧度曲线为直线
	void dealCirBoundary(varray<Spline>& SLS) {
		varray<Spline> Big;//区分大小弧度上的曲线
		int num = SLS.size() - 1;
		varray<Spline> result;
		Spline SL;

		//先把曲线区分出来
		//设置角度约束，超过一定角度就认为是出现曲线分段(可以改变角度，处理不同情况)
		double angle = PI / 6;//30度

		//设置角度约束，判断曲线弯曲程度
		double angle1 = PI / 8;//22.5度

		for (int i = 1; i < SLS.size(); i++) {
			double ang = 0;
			Vec4 v1, v2;

			//v1、v2分别表示第i-1、i条曲线的方向向量
			v1 = (SLS[i - 1].m_CtrlPts[2] - SLS[i - 1].m_CtrlPts[0]);
			v2 = (SLS[i].m_CtrlPts[2] - SLS[i].m_CtrlPts[0]);

			//求相邻两样条对应向量的夹角，作为角度判断
			ang = calAngle(v1, v2);

			//小于angle，说明没有分段
			if (ang < angle) {
				Big.push_back(SLS[i - 1]);
			}

			//若大于angle度，说明轮廓曲线分段了，对前面这一段曲线进行处理；或者是最后一个样条，也将其存放到Big中，用于判断是否是整圆曲线
			if ((ang > angle) || (i == SLS.size() - 1)) {
				Big.push_back(SLS[i - 1]);//将对应样条存到大弧度容器中

				//将最后一个也样条存到大弧度容器中
				if (i == SLS.size() - 1) {
					Big.push_back(SLS[i]);
				}

				//若大容器中的所有样条是首尾相连的
				if (JudgeTwoPointsCoincide(Big[0].m_CtrlPts[0], Big[Big.size() - 1].m_CtrlPts[2])) {
					//说明没有分段，是个整圆曲线
					result = SLS;
					break;
				}

				//出现拐角点，进行曲线处理
				double angFlag = 0;//一段曲线中最大角度标志
				Vec4 v3;
				v3 = (Big[Big.size() - 1].m_CtrlPts[2] - Big[0].m_CtrlPts[0]);//大弧度曲线的首末控制点形成的向量
				bool flag = 0;
				//找到曲线中最大角度
				for (int i = 0; i < Big.size(); i++) {
					Vec4 v4;
					v4 = (Big[i].m_CtrlPts[2] - Big[0].m_CtrlPts[0]);//每个样条末端端点与大弧度曲线首端点形成的向量
					double ang1 = 0;
					ang1 = calAngle(v3, v4);
					if (angFlag < ang1) {
						angFlag = ang1;
					}
				}

				//判断曲线弯曲的弧度是否超过设定的值
				if (angFlag < angle1) {
					//说明是小弧度曲线、简化为一条直线，但是后续还会有直线轮廓的简化，所以这里不能只出现一条直线，至少两条
					Vec4 v1, v2;
					v1 = (Big[0].m_CtrlPts[0] + Big[Big.size() - 1].m_CtrlPts[2]) / 2;//两端的中点
					v1.w = 1;
					//前一段曲面
					Spline0 sl(Big[0].m_CtrlPts[0], v1);
					SL = sl.getSpline();
					result.push_back(SL);

					//后一段曲线
					Spline0 sl2(v1, Big[Big.size() - 1].m_CtrlPts[2]);
					SL = sl2.getSpline();
					result.push_back(SL);

					////说明是小弧度曲线、重建为两条直线
					//Vec4 vTmp;
					//vTmp = (Big[0].m_CtrlPts[0] + Big[Big.size() - 1].m_CtrlPts[2]) / 2;
					//Spline0 sl(Big[0].m_CtrlPts[0], vTmp);
					//SL = sl.getSpline();
					//result.push_back(SL);
					//Spline0 sl1(vTmp, Big[Big.size() - 1].m_CtrlPts[2]);
					//SL = sl1.getSpline();
					//result.push_back(SL);
				}
				else {
					//大弧度曲线不处理，放到结果容器中
					for (auto& i : Big) {
						result.push_back(i);
					}
				}
				Big.clear();
			}
		}
		SLS.clear();
		SLS = result;
	}

	//！提取二维网格面的边界
	void getMeshModelBoundary(string path, varray<varray<Spline>>& boundRes) {
		RWGeometric rwg;
		std::cout << "--> Reading mesh..." << std::endl;
		std::cout << path << std::endl;
		MeshLib::Mesh mesh;

		//读取指定路径上了obj文件的边界
		mesh.read_obj(path.c_str());

		//用于存放提取出的所有边界曲线
		varray<Spline> SLS;
		varray<Spline> chackTemp;

		//提取网格模型边界
		for (std::list<Edge*>::iterator eiter = mesh.edges().begin(); eiter != mesh.edges().end(); ++eiter) {
			//is_boundary用来判断是否为边界
			if (mesh.is_boundary(*eiter)) {
				//如果是边界，p、p1为该段曲线的两端
				//edge_vertex_1用于返回一条边的第一个端点（即起点）对应的顶点
				//edge_vertex_2()用于函数获取一条边的另外一个端点对应的顶点
				Point p = mesh.edge_vertex_1(*eiter)->point();
				Point p1 = mesh.edge_vertex_2(*eiter)->point();
				Vec4 v1 = { p.x(),p.y(),0,1 };
				Vec4 v2 = { p1.x(),p1.y(),0,1 };
				Spline SL;

				//生成一条nurbs曲线
				Spline0 sl(v1, v2);
				SL = sl.getSpline();
				SLS.push_back(SL);
			}
		}

		//尺寸放大（尺寸过小会导致计算失误，后续需要恢复原样）
		bigger(SLS, multiple);

		//输出提取到的模型轮廓(用于自己查看，可注释)
		rwg.WriteSpline("SimpleSplineAdjust.txt", SLS);
		rwg.ReadSpline("SimpleSplineAdjust.txt", SLS);
		//区分轮廓
		varray<varray<Spline>> vecSL;
		selectWire(SLS, vecSL);

		//将轮廓区分后的曲线输出，用于查看
		chackSpline("ChackSelectWire.txt", vecSL);

		//构建圆轮廓的时候会出现方向相反的情况
		//在这里可以修正(具体看函数内部)
		//orderEdgeAntiClock1(vecSL);
		orderEdgeAntiClock(vecSL);
		cout << "轮廓数量：" << vecSL.size() << endl;

		//用于查看轮廓区分后并排序内轮廓后的结果(可注释)
		chackSpline("SimpleSpline1.txt", vecSL);

		//需要找到拐点，再进行轮廓排序
		for (auto& i : vecSL) {
			orderEdges(i);
		}

		//输出点，用于检查
		chackPoint("ChackPoint.txt", vecSL);

		//简化一些小弧度曲线为直线(改进：后续可用曲线拟合弧度曲线，提高模型精确度)
		for (auto& i : vecSL) {
			dealCirBoundary(i);
		}

		//查看初步简化结果(可注释)
		chackSpline("SimpleSpline2.txt", vecSL);

		//进行简化
		for (int i = 0; i < vecSL.size(); i++) {
			//应该分三种情况 <180° =180° 整圆（目前不考虑大于180°的圆弧）
			varray<Vec4> vec;
			vec.clear();

			//按照一个方向获取所有的控制点
			getOritientPoint(vecSL[i], vec);

			varray<Spline> SL;
			varray<Vec4> vecs;//存放圆弧上的点
			varray<varray<Vec4>> vecss;//存放多个圆弧数组
			bool cirflag = 0;//标志圆弧上的点
			bool cirFlag = 0;//标志直线上的点
			for (int i = 0; i < vec.size(); ++i) {
				//根据向量方向的相同和不同进行区分直线上的点和圆弧上的点（通过比较中间线及其前后两个线的方向）
				//对于处在开头和末尾的几个点需要进行特别处理
				Vec4 v1, v2, v3, v4, direVec, tmpV, tmpV1;
				//确保v4-v1-v2-v3的顺序
				int len = vec.size();
				v1 = vec[i];
				v2 = vec[(i + 1) % len];
				v3 = vec[(i + 2) % len];
				v4 = vec[(i - 1 + len) % len];

				//中间线及其前后两线的向量方向
				tmpV1 = (v1 - v4).Normalize();
				direVec = (v2 - v1).Normalize();
				tmpV = (v3 - v2).Normalize();

				//若是圆弧上的点，收集起来进行后续处理
				//意思是既不与前驱曲线共线，又不与后继曲线共线
				//这里用并，那如果与某一条曲线共线呢？—————这里只是将圆弧上的点分离开
				if (!JudgeTwoPointsCoincide(direVec, tmpV) && !JudgeTwoPointsCoincide(direVec, tmpV1)) {
					vecs.push_back(v1);
					vecs.push_back(v2);
					/*cout << v1.x <<" " << v1.y << endl;
					cout << v2.x << " " << v2.y << endl;*/
					cirflag = 1;//标记此线段属于圆弧段
					cirFlag = 0;//标记此线段非直线
				}
				//若是直线上的点，直接生成样条，后续再进行简化处理
				else {
					Spline slTmp;
					Spline0 sl(v1, v2);
					slTmp = sl.getSpline();
					SL.push_back(slTmp);
					cirFlag = 1;//标记此线段为直线
				}
				//从圆弧段跳到直线段时、直线段跳到圆弧段时、已到最后一个点时,提取各个圆弧段的点，后续分开处理
				if ((cirFlag && cirflag) || i == vec.size() - 1) {
					cirflag = 0;
					cirFlag = 0;
					if (vecs.size() != 0) {
						vecss.push_back(vecs);
						vecs.clear();
					}
				}
			}

			//简化直线
			simpleBoundary(SL);
			chackSpline("simpleSpline3.txt", SL);

			//简化圆弧
			varray<Spline> tempCheck;
			Spline0 s0;
			for (auto i : vecss)
			{
				for (int j = 0; j < i.size() - 1; j++)
				{
					tempCheck.push_back(s0.getSpline(i[j], i[j + 1]));
				}
			}
			rwg.WriteSpline("ForCircleSpline.txt", tempCheck);

			if (vecss.size() != 0) {
				for (int i = 0; i < vecss.size(); i++) {
					//半圆弧情况 重建半圆弧
					//通过判断首末两点是否为同一个点，是则为整圆，否为半圆
					//1、获得圆心、半径
					//2、以原点为圆心，生成圆弧，然后再移动到对应位置上
					if (!JudgeTwoPointsCoincide(vecss[i][0], vecss[i][vecss[i].size() - 1])) {
						//首尾两点不重合
						Vec4 cirPoint;//圆弧圆心
						double r;//圆弧半径

						//通过圆弧上的点，来获取圆弧的圆心和半径
						getCirclePara(vecss[i], cirPoint, r);

						//计算首末两点与圆心之间的向量夹角，判断圆弧角度
						Vec4 p1, p2, dirP1, dirP2, dirP3;
						double angle;
						p1 = vecss[i][0];
						p2 = vecss[i][vecss[i].size() - 1];

						//这里相当于求出以圆弧圆心为坐标原点的圆弧，后续在经过平移得到对应位置的圆弧
						dirP1 = p1 - cirPoint;
						dirP2 = p2 - cirPoint;
						dirP3 = dirP2.Cross(dirP1);

						//计算圆弧弧度
						angle = calAngle(dirP1, dirP2);

						/*if (dirP3.z < 0) {
							angle = calAngle(dirP1, dirP2);
						}
						else {
							angle = 2*PI-calAngle(dirP1, dirP2);
						}*/

						Model_Solution m;
						/*m.MovePoint(midP, cirPoint.x, 1);
						m.MovePoint(midP, cirPoint.y, 2);*/
						//180° 两个四分之一圆弧
						if (abs(angle - PI) < 0.2) {
							//得到半圆中间点
							Vec4 midP;
							midP = rolate(dirP1, -PI / 2, 3);
							//重建
							Spline cirSL;
							Spline0 cirTmp;
							//由于读取的三角网格轮廓未实现顺时针排序，故重建弧线的时候需要根据情况改变下面函数中角度的方向（正负）下同。
							cirSL = cirTmp.getArcSpline(r, -angle / 2, dirP2, midP);
							//平移圆弧 下面同样步骤
							m.Trans(cirSL, cirPoint.x, 1);
							m.Trans(cirSL, cirPoint.y, 2);

							//两端点权重为1，中间权重通过函数计算
							calWeight(cirSL);
							SL.push_back(cirSL);

							cirSL = cirTmp.getArcSpline(r, -angle / 2, midP, dirP1);
							m.Trans(cirSL, cirPoint.x, 1);
							m.Trans(cirSL, cirPoint.y, 2);
							calWeight(cirSL);

							SL.push_back(cirSL);
						}
						//小于180°
						else if (angle < PI) {
							//重建
							Spline cirSL1;
							Spline0 cirTmp1;
							cirSL1 = cirTmp1.getArcSpline(r, -angle, dirP2, dirP1);
							m.Trans(cirSL1, cirPoint.x, 1);
							m.Trans(cirSL1, cirPoint.y, 2);
							calWeight(cirSL1);
							SL.push_back(cirSL1);
						}
						//大于180°小于360°，分成两个小于180°的圆弧
						else {
							Vec4 midP;
							midP = rolate(dirP1, -angle / 2, 3);
							//重建
							Spline cirSL1;
							Spline0 cirTmp1;

							//第一段
							cirSL1 = cirTmp1.getArcSpline(r, -angle / 2, dirP2, midP);
							m.Trans(cirSL1, cirPoint.x, 1);
							m.Trans(cirSL1, cirPoint.y, 2);
							calWeight(cirSL1);
							SL.push_back(cirSL1);

							//第二段
							cirSL1 = cirTmp1.getArcSpline(r, -angle / 2, midP, dirP1);
							m.Trans(cirSL1, cirPoint.x, 1);
							m.Trans(cirSL1, cirPoint.y, 2);
							calWeight(cirSL1);
							SL.push_back(cirSL1);
						}
					}
					//整圆情况 重建整圆
					else if (JudgeTwoPointsCoincide(vecss[i][0], vecss[i][vecss[i].size() - 1]) && (vecss[i].size() - 1) != 0) {
						varray<Spline> Tmp1;
						Spline0 cirTmp;
						Spline cirSL;
						varray<Vec4> cirRes;
						Vec4 cirPoint;
						double cirR;

						getCirclePara(vecss[i], cirPoint, cirR);
						Vec4 mV1 = { -cirR,0,0,1 };
						Vec4 mV2 = { 0,cirR,0 ,1 };
						Vec4 mV3 = { cirR,0,0 ,1 };
						Vec4 mV4 = { 0,-cirR,0,1 };
						cirRes.push_back(mV1);
						cirRes.push_back(mV2);
						cirRes.push_back(mV3);
						cirRes.push_back(mV4);
						for (int i = 0; i < cirRes.size(); ++i) {
							if (i == cirRes.size() - 1) {
								cirSL = cirTmp.getArcSpline(cirR, PI / 2, cirRes[cirRes.size() - 1], cirRes[0]);
								Tmp1.push_back(cirSL);
							}
							else {
								cirSL = cirTmp.getArcSpline(cirR, PI / 2, cirRes[i], cirRes[i + 1]);
								Tmp1.push_back(cirSL);
							}
						}
						Model_Solution m;
						m.Trans(Tmp1, cirPoint.x, 1);
						m.Trans(Tmp1, cirPoint.y, 2);

						for (auto& i : Tmp1) {
							SL.push_back(i);
						}
					}
				}
			}
			vecSL[i].clear();
			orderEg(SL);
			vecSL[i] = SL;
			boundRes.push_back(SL);
		}

		//简化后边界,打印出来查看简化结果，用于调整
		chackSpline("resSpline.txt", boundRes);

		chackPoint("ChackPoint.txt", boundRes);
	}

	//图形放大
	void bigger(varray<Spline>& boundRes, int n) {
		//放大
		for (auto& i : boundRes) {
			for (auto& k : i.m_CtrlPts) {
				k.x *= n;
				k.y *= n;
			}
		}
	}
	void bigger(varray<SplineSurface>& ss, int n) {
		//放大
		for (int i = 0; i < ss.size(); i++) {
			for (int j = 0; j < ss[i].m_CtrlPts.size(); j++) {
				ss[i].m_CtrlPts[j].x *= n;
				ss[i].m_CtrlPts[j].y *= n;
				ss[i].m_CtrlPts[j].z *= n;
			}
		}
	}
	//图形缩小
	void littler(varray<Spline>& boundRes, double n) {
		//缩小
		for (auto& i : boundRes) {
			for (auto& k : i.m_CtrlPts) {
				k.x /= n;
				k.y /= n;
			}
		}
	}
	//点坐标缩小
	void littler(varray<varray<Vec4>>& vec4, double n) {
		//缩小
		for (auto& i : vec4) {
			for (auto& k : i) {
				k.x /= n;
				k.y /= n;
			}
		}
	}
	//曲面缩小（参数：曲面，缩小倍数，不变方向(0:xyz 1:x 2:y 3:z)）
	void littler(varray<SplineSurface>& ss, double n, int choice) {
		for (int i = 0; i < ss.size(); i++) {
			if (choice == 1) {
				for (int j = 0; j < ss[i].m_CtrlPts.size(); j++) {
					//ss[i].m_CtrlPts[j].x /= n;
					ss[i].m_CtrlPts[j].y /= n;
					ss[i].m_CtrlPts[j].z /= n;
				}
			}
			else if (choice == 2) {
				for (int j = 0; j < ss[i].m_CtrlPts.size(); j++) {
					ss[i].m_CtrlPts[j].x /= n;
					//ss[i].m_CtrlPts[j].y /= n;
					ss[i].m_CtrlPts[j].z /= n;
				}
			}
			else if (choice == 3) {
				for (int j = 0; j < ss[i].m_CtrlPts.size(); j++) {
					ss[i].m_CtrlPts[j].x /= n;
					ss[i].m_CtrlPts[j].y /= n;
					//ss[i].m_CtrlPts[j].z /= n;
				}
			}
			else if (choice == 0) {
				//全坐标都缩小
				for (int j = 0; j < ss[i].m_CtrlPts.size(); j++) {
					ss[i].m_CtrlPts[j].x /= n;
					ss[i].m_CtrlPts[j].y /= n;
					ss[i].m_CtrlPts[j].z /= n;
				}
			}
			else {
				cout << "输入错误！" << endl;
			}
		}
	}

	//曲面缩小（参数：曲面，缩小倍数，不变方向(0:xyz 1:x 2:y 3:z)）
	void littler(SplineSurface& ss, double n, int choice) {
		if (choice == 1) {
			for (int j = 0; j < ss.m_CtrlPts.size(); j++) {
				//ss.m_CtrlPts[j].x /= n;
				ss.m_CtrlPts[j].y /= n;
				ss.m_CtrlPts[j].z /= n;
			}
		}
		else if (choice == 2) {
			for (int j = 0; j < ss.m_CtrlPts.size(); j++) {
				ss.m_CtrlPts[j].x /= n;
				//ss.m_CtrlPts[j].y /= n;
				ss.m_CtrlPts[j].z /= n;
			}
		}
		else if (choice == 3) {
			for (int j = 0; j < ss.m_CtrlPts.size(); j++) {
				ss.m_CtrlPts[j].x /= n;
				ss.m_CtrlPts[j].y /= n;
				//ss.m_CtrlPts[j].z /= n;
			}
		}
		else if (choice == 0) {
			//全坐标都缩小
			for (int j = 0; j < ss.m_CtrlPts.size(); j++) {
				ss.m_CtrlPts[j].x /= n;
				ss.m_CtrlPts[j].y /= n;
				ss.m_CtrlPts[j].z /= n;
			}
		}
		else {
			cout << "输入错误！" << endl;
		}
	}
	//平面全自动四边剖分
	//boundRes:	内外轮廓线集合
	//genus:	亏格
	//bV:		各个面片控制点集合(用于后续曲线曲面拟合)
	void quadPlane(varray<varray<Spline>>& boundRes, varray<bool>& genus, varray<varray<Vec4>>& bV) {
		RWGeometric rwg;
		varray<Spline> outer;
		varray<varray<Spline>> inner;
		Model_Solution m;
		varray<Spline>temp;

		//首个轮廓为外轮廓，其余为内部轮廓
		for (auto& i : boundRes[0]) {
			outer.push_back(i);
			temp.push_back(i);
		}
		for (int i = 1; i < boundRes.size(); i++) {
			inner.push_back(boundRes[i]);
			for (auto& j : boundRes[i])
			{
				temp.push_back(j);
			}
		}
		rwg.WriteSpline("E:\\Model\\PlaneQuad\\ChackLine.txt", temp);
		//平面全四边剖分，输出平面剖分结果(便于查看剖分结果正确与否)
		varray<SplineSurface> SS;
		/*CDT_Operate cdto(outer, inner);
		quad(outer, inner, cdto.addLine, genus, SS);*/

		quad(outer, inner, genus, SS);

		rwg.WriteSplineSurface("E:\\Model\\SurfaceQuad\\PlaneplaneQuadSurface.txt", SS);

		//剖分结果细化，获取较多的数据点用于拟合(一般细化3~4次就行，过多的话，结果可能变差)
		for (auto& i : SS) {
			i.KnotsRefineNum(4);
		}

		//提取各个面片所有控制点，用于后续曲面拟合
		for (auto& i : SS) {
			bV.push_back(i.m_CtrlPts);
		}

		//提取剖分结果的各个轮廓边界，用于后续曲面拟合
		varray<varray<Spline>> boundsl;
		for (auto& i : SS) {
			varray<Spline> sl;
			i.GetEdgeLines(sl);
			sortEdg(sl);
			SplineSurface ss;
			varray<SplineSurface> SS;
			ss.CoonsInterpolate(sl);
			SS.push_back(ss);
			boundsl.push_back(sl);
		}
		boundRes.clear();

		//缩小(因为前面获取网格模型轮廓时，对模型进行了放大，这里需要还原)
		for (auto& i : boundsl) {
			littler(i, multiple);
		}

		//输出平面剖分轮廓(便于查看剖分结果,可注释)
		varray<Spline> resSL;
		for (auto i : boundsl) {
			for (auto j : i) {
				resSL.push_back(j);
			}
		}
		rwg.WriteSpline("planeQuadSplines.txt", resSL);

		boundRes = boundsl;
	}

	//两线是否有公共点
	bool ifHasCommonPoint(Spline& sl1, Spline& sl2) {
		Vec4 v1 = sl1.m_CtrlPts[0];
		Vec4 v2 = sl1.m_CtrlPts[sl1.m_CtrlPts.size() - 1];
		Vec4 v3 = sl2.m_CtrlPts[0];
		Vec4 v4 = sl2.m_CtrlPts[sl2.m_CtrlPts.size() - 1];
		if (distance(v1, v3) < 0.001 || distance(v1, v4) < 0.001 ||
			distance(v2, v3) < 0.001 || distance(v2, v4) < 0.001) {
			return true;
		}
		return false;
	}
	bool ifHasCommonPoint1(Spline& sl1, Spline& sl2) {
		Vec4 v1 = sl1.m_CtrlPts[0];
		Vec4 v2 = sl1.m_CtrlPts[sl1.m_CtrlPts.size() - 1];
		Vec4 v3 = sl2.m_CtrlPts[0];
		Vec4 v4 = sl2.m_CtrlPts[sl2.m_CtrlPts.size() - 1];
		cout << distance(v1, v3) << endl;
		cout << distance(v1, v4) << endl;
		cout << distance(v2, v3) << endl;
		cout << distance(v2, v4) << endl;
		if (distance(v1, v3) < 0.0015 || distance(v1, v4) < 0.0015 ||
			distance(v2, v3) < 0.0015 || distance(v2, v4) < 0.0015) {
			return true;
		}
		return false;
	}

	//轮廓按一个方向排序
	void orderEg(varray<Spline>& sl) {
		varray<Spline> tmpSL;
		Spline SL = sl[0];
		sl.erase(sl.begin());
		tmpSL.clear();
		tmpSL.push_back(SL);
		for (int i = 0; i < sl.size(); i++) {
			cout << SL.m_CtrlPts[0].x << " " << SL.m_CtrlPts[0].y << endl;
			cout << SL.m_CtrlPts[2].x << " " << SL.m_CtrlPts[2].y << endl;
			cout << sl[i].m_CtrlPts[0].x << " " << sl[i].m_CtrlPts[0].y << endl;
			cout << sl[i].m_CtrlPts[2].x << " " << sl[i].m_CtrlPts[2].y << endl;
			if (ifHasCommonPoint(SL, sl[i])) {
				SL = sl[i];
				tmpSL.push_back(SL);
				sl.erase(sl.begin() + i);
				i--;
				//如果找到了与首个样条有公共点的边，说明找到了整个轮廓，后面进行下一个轮廓区分
				if (tmpSL.size() > 2 && ifHasCommonPoint(tmpSL[0], tmpSL[tmpSL.size() - 1])) {
					break;
				}
			}
			else {
				sl.push_back(sl[i]);
				sl.erase(sl.begin() + i);
				i--;
			}
		}
		sl.clear();
		sl = tmpSL;
	}
	//区分轮廓
	void selectWire(varray<Spline>& SLS, varray<varray<Spline>>& vecSL) {
		RWGeometric rwg;
		varray<Spline> tmpSL;

		//实现了内外轮廓区分，并且轮廓按照一个方向排序，但方向是随机的，主要与数据有关
		while (SLS.size() != 0) {
			Spline SL = SLS[0];
			SLS.erase(SLS.begin());
			tmpSL.clear();
			tmpSL.push_back(SL);

			////找到曲线SL所在的轮廓
			//for (int i = 0; i < SLS.size(); i++) {
			//
			//	//与SL有交点就放入tempSL中，并在SLS中删除该线段
			//	if (ifHasCommonPoint(SL, SLS[i])) {
			//		SL = SLS[i];
			//		tmpSL.push_back(SL);
			//		SLS.erase(SLS.begin() + i);
			//		i--;
			//		//如果找到了与首个样条有公共点的边，说明找到了整个轮廓，后面进行下一个轮廓区分
			//		if (tmpSL.size() > 2 && ifHasCommonPoint(tmpSL[0], tmpSL[tmpSL.size() - 1])) {
			//			break;
			//		}

			//	}
			//	else {
			//		//没有交点的线段往后放
			//		SLS.push_back(SLS[i]);
			//		SLS.erase(SLS.begin() + i);
			//		i--;

			//	}
			//}

			////找到曲线SL所在的轮廓(改，上面为原来的代码)
			while (1) {
				//与SL有交点就放入tempSL中，并在SLS中删除该线段
				if (ifHasCommonPoint(SL, SLS[0])) {
					SL = SLS[0];
					tmpSL.push_back(SL);
					SLS.erase(SLS.begin());

					//如果找到了与首个样条有公共点的边，说明找到了整个轮廓，后面进行下一个轮廓区分
					if (tmpSL.size() > 2 && ifHasCommonPoint(tmpSL[0], tmpSL[tmpSL.size() - 1])) {
						break;
					}
				}
				else {
					//没有交点的线段往后放放
					SLS.push_back(SLS[0]);
					SLS.erase(SLS.begin());
				}
			}
			vecSL.push_back(tmpSL);
		}

		//将外轮廓放到第一位，作为几何域包含树的根
		//外轮廓曲线
		varray<Spline> outSplines;
		varray<varray<Spline>> resTmp;
		outSplines = vecSL[0];

		//找出外轮廓，并将外轮廓放在第一的位置
		//因为外轮廓为最外围的，所以calMinPointX(vecSL[i])的值是最小的
		double minX = calMinPointX(vecSL[0]);
		double tmpX;
		for (int i = 1; i < vecSL.size(); i++) {
			tmpX = calMinPointX(vecSL[i]);
			if (tmpX < minX) {
				outSplines = vecSL[i];
				minX = tmpX;
			}
		}

		//将所有内轮廓存入resTmp中
		for (auto& i : vecSL) {
			if (JudgeTwoLinesCoincide(outSplines[0], i[0])) {
				continue;
			}
			else {
				resTmp.push_back(i);
			}
		}

		//将最终区分好的轮廓存入vecSL中
		varray<varray<Spline>> res;
		res.push_back(outSplines);
		for (auto& i : resTmp) {
			res.push_back(i);
		}
		vecSL = res;
	}

	//简化直线边界
	void simpleBoundary(varray<Vec4>& vec, varray<Vec4>& vecs) {
		////设置角度阈值
		//double ang1 = 0;
		//double ang2 = PI / 16;
		//角点集合
		Vec3 tmpV = {};
		for (int i = 0; i < vec.size(); ++i) {
			Vec3 v1, v2, direVec;
			if (i == vec.size() - 1) {
				v1 = vec[i];
				v2 = vec[0];
				direVec = (v2 - v1).Normalize();
			}
			else {
				v1 = vec[i];
				v2 = vec[i + 1];
				direVec = (v2 - v1).Normalize();
			}
			if (!JudgeTwoPointsCoincide(direVec, tmpV)) {
				vecs.push_back(v1);
				tmpV = direVec;
			}
			else {
				continue;
			}
		}
	}
	//简化直线边界
	void simpleBoundary(varray<Vec4>& vec, varray<Spline>& sl) {
		////设置角度阈值
		//double ang1 = 0;
		//double ang2 = PI / 16;
		//角点集合
		Vec3 tmpV = {};
		for (int i = 0; i < vec.size(); ++i) {
			Vec3 v1, v2, direVec;
			if (i == vec.size() - 1) {
				v1 = vec[i];
				v2 = vec[0];
				direVec = (v2 - v1).Normalize();
			}
			else {
				v1 = vec[i];
				v2 = vec[i + 1];
				direVec = (v2 - v1).Normalize();
			}
			if (!JudgeTwoPointsCoincide(direVec, tmpV)) {
				//vecs.push_back(v1);
				tmpV = direVec;
			}
			else {
				continue;
			}
		}
	}
	//简化直线边界
	void simpleBoundary(varray<Spline>& SL) {
		////设置角度阈值
		//double ang1 = 0;
		//double ang2 = PI / 16;
		//角点集合
		//同一方向的且有公共点的放在一起，简化成一条

		//判断是否存在直线
		if (SL.size() == 0) {
			cout << "不存在直线" << endl;
			return;
		}
		varray<varray<Spline>> varSL;//第一维存放一条属于一条直线的线段集合
		varray<Spline> var, sl;
		var.push_back(SL[0]);

		Vec4 p1 = SL[0].m_CtrlPts[0];//首端点
		Vec4 p2 = SL[0].m_CtrlPts[2];//尾端点
		Vec3 tmpV = (p2 - p1).Normalize();//方向向量

		for (int i = 1; i < SL.size(); ++i) {
			Vec3 v1, v2, direVec;
			v1 = SL[i].m_CtrlPts[0];
			v2 = SL[i].m_CtrlPts[2];
			direVec = (v2 - v1).Normalize();
			//方向相同，且有公共端点，所以简化成一条直线
			if (JudgeTwoPointsCoincide(direVec, tmpV) && ifHasCommonPoint(SL[i], SL[i - 1])) {
				var.push_back(SL[i]);
			}
			else {
				varSL.push_back(var);
				var.clear();
				var.push_back(SL[i]);
				tmpV = (SL[i].m_CtrlPts[2] - SL[i].m_CtrlPts[0]).Normalize();
			}
			if (i == SL.size() - 1) {
				varSL.push_back(var);
			}
		}
		//由第一个点与最后一个点新构建出一条样条曲线
		//varSL每一个元素都是属于一条直线的集合，所以取第一个点与最后一个点即可简化为一条直线
		for (int i = 0; i < varSL.size(); i++) {
			Vec4 p3, p4;
			p3 = varSL[i][0].m_CtrlPts[0];
			p4 = varSL[i][varSL[i].size() - 1].m_CtrlPts[2];
			Spline slTmp;
			Spline0 sl0(p3, p4);
			slTmp = sl0.getSpline();
			sl.push_back(slTmp);
		}
		SL = sl;
	}
	//判断是否是圆弧
	bool isCircleBoundary(varray<Vec4> vec) {
		//若是圆弧，相邻两样条方向不一致
		//对于直接生成的obj文件，其圆弧部分可能出现两样条在同一直线上的情况
		//ansys划分的网格应该不会出现这种情况
		Vec4 tmpV = {};
		for (int i = 0; i < vec.size() - 1; ++i) {
			Vec4 v1, v2, direVec;
			v1 = vec[i];
			v2 = vec[i + 1];
			direVec = (v2 - v1).Normalize();

			if (JudgeTwoPointsCoincide(direVec, tmpV)) {
				return false;
			}
		}
		return true;
	}

	//获取圆弧参数：圆心坐标、半径 输入：圆弧上的点
	void getCirclePara(varray<Vec4>& vec, Vec3& cirPoint, double& cirR) {
		varray<Vec4> vecTmp;
		vecTmp.push_back(vec[0]);

		//点去重
		for (int i = 1; i < vec.size(); i++) {
			//去掉重合的点
			if (JudgeTwoPointsCoincide(vec[i - 1], vec[i])) {
				continue;
			}
			vecTmp.push_back(vec[i]);
		}
		vec = vecTmp;
		if (vec.size() < 3) {
			cout << "getCirclePara函数中，vec容器中顶点数小于3，无法计算圆弧半径、圆心等数据，说明组成该弧线的数量少于2个" << endl;
			assert(0);
		}

		//查看去重后的点
		//************
		RWGeometric rwg;
		varray<varray<Vec4>> temp;
		varray<Vec4> temp1;
		for (auto i : vec) {
			temp1.clear();
			temp1.push_back(i);
			temp.push_back(temp1);
		}
		rwg.WritePoint("ChackPoint.txt", temp);
		//************

		if (this->isCircleBoundary(vec)) {
			double x, y, r;
			int num = 0;
			double x1, x2, x3, y1, y2, y3, A, B, C, D;
			x1 = vec[0].x;
			y1 = vec[0].y;

			//找到三个点来构建圆弧
			if (vec.size() > 4) {
				//圆弧中间点坐标
				x2 = vec[vec.size() / 2].x;
				y2 = vec[vec.size() / 2].y;

				//圆弧尾端点坐标
				//为什么这里改成1会出错————因为第一个点与最后一个点重合
				x3 = vec[vec.size() - 2].x;
				y3 = vec[vec.size() - 2].y;
			}
			else {
				x2 = vec[1].x;
				y2 = vec[1].y;
				x3 = vec[2].x;
				y3 = vec[2].y;
			}

			//****************
			//这里是三点确定圆心的数学公式
			//利用圆心与端点确定半径
			A = x1 * (y2 - y3) - y1 * (x2 - x3) + x2 * y3 - x3 * y2;
			B = (x1 * x1 + y1 * y1) * (y3 - y2) + (x2 * x2 + y2 * y2) * (y1 - y3) + (x3 * x3 + y3 * y3) * (y2 - y1);
			C = (x1 * x1 + y1 * y1) * (x2 - x3) + (x2 * x2 + y2 * y2) * (x3 - x1) + (x3 * x3 + y3 * y3) * (x1 - x2);
			D = (x1 * x1 + y1 * y1) * (x3 * y2 - x2 * y3) + (x2 * x2 + y2 * y2) * (x1 * y3 - x3 * y1) + (x3 * x3 + y3 * y3) * (x2 * y1 - x1 * y2);

			x = (-B / (2 * A));
			y = (-C / (2 * A));
			r = (sqrt((B * B + C * C - 4 * A * D) / (4 * A * A)));
			//****************
			if (A == 0) {
				cout << "计算圆弧圆心时，所用到的三个点共线，导致圆心坐标有误！" << endl;
				assert(A);
			}

			//求坐标和半径的平均值
			//x /= num;
			//y /= num;
			//r /= num;
			cirPoint = { x,y,0 };
			cirR = r;
		}
	}

	//计算多变形轮廓重心，用于排序（考虑到模型情况不同，目前还没用上）
	Vec4 calGravity(varray<Spline> SL)
	{
		varray<Vec4> polygenPoints;
		for (int i = 0; i < SL.size(); i++) {
			polygenPoints.push_back(SL[i].m_CtrlPts[0]);
		}
		double martix, xaddx, yaddy, gx, gy, gz, denominator, x_numerator, y_numerator;
		varray<double> martixCollect, xCollect, yCollect;
		Vec4 gcoordinate;
		int pointsNumber = polygenPoints.size();
		for (int i = 0; i < pointsNumber; i++)
		{
			if (i == (pointsNumber - 1))
			{
				martix = (polygenPoints[i].x * polygenPoints[0].y) - (polygenPoints[i].y * polygenPoints[0].x);
				xaddx = polygenPoints[i].x + polygenPoints[0].x;
				yaddy = polygenPoints[i].y + polygenPoints[0].y;
				martixCollect.push_back(martix);
				xCollect.push_back(xaddx);
				yCollect.push_back(yaddy);
			}

			else
			{
				martix = (polygenPoints[i].x * polygenPoints[i + 1].y) - (polygenPoints[i].y * polygenPoints[i + 1].x);
				xaddx = polygenPoints[i].x + polygenPoints[i + 1].x;
				yaddy = polygenPoints[i].y + polygenPoints[i + 1].y;
				martixCollect.push_back(martix);
				xCollect.push_back(xaddx);
				yCollect.push_back(yaddy);
			}
		}

		x_numerator = 0;
		y_numerator = 0;
		denominator = 0;

		for (int i = 0; i < martixCollect.size(); i++)
		{
			x_numerator += (xCollect[i] * martixCollect[i]);
			y_numerator += (yCollect[i] * martixCollect[i]);
			denominator += martixCollect[i];
		}
		denominator = denominator * 3;
		gx = x_numerator / denominator;
		gy = y_numerator / denominator;
		gz = polygenPoints[0].z;
		gcoordinate = Vec4(gx, gy, gz, 1);
		return gcoordinate;
	}

	unordered_map<int, Point> map3d;
	unordered_map<int, Point> map2d;
	//！展开曲面
	void unfoldMeshSurface(string path, varray<Vec4>& vecs) {
		std::cout << "--> Reading mesh..." << std::endl;
		MeshLib::Mesh mesh;

		//读取原始网格曲面
		mesh.read_obj(path.c_str());

		//FormTrait 是MeshLib库中的一个类，用于实现网格数据和表单数据之间的转换
		MeshLib::FormTrait traits(&mesh);
		//收集三角网格空间曲面上的点,用于后续曲面拟合
		//MeshV

			//判断所读取的点是ertexIterator可以依次访问网格中的每个顶点，并对其进行操作。
		for (MeshVertexIterator viter(&mesh); !viter.end(); ++viter) {
			Vertex* v = *viter;//否为一开始设置的固定点
			if (v->string().substr(0, 3) != "fix") {
				Point p = v->point();
				Vec4 vec = { p.x(), p.y(), p.z(),1 };
				vecs.push_back(vec);
			}
		}

		//计算共形映射
		std::cout << "--> Computing conformal map..." << std::endl;
		MeshLib::LSCM lscm(&mesh);
		lscm.project();
		map3d = lscm.map3d;
		map2d = lscm.map2d;
		//m = mesh;
		std::cout << "--> Writing mesh..." << std::endl;
		mesh.write_obj("obj\\Plane.obj");
	}

	Vec3 pointTransToVec(MeshLib::Point& p) {
		Vec3 v;
		v.x = p.x();
		v.y = p.y();
		v.z = p.z();
		return v;
	}
	//在三维面上选择最近的点
	vector<int> selectCloserPoint(Spline& sl, unordered_map<int, Point>& map) {
		vector<int> res;
		for (int i = 0; i < sl.m_CtrlPts.size(); ++i) {
			double dist = MAXINT;
			int id = -1;
			for (auto& j : map) {
				Vec3 tmp = pointTransToVec(j.second);
				double tmpDist = distance(tmp, sl.m_CtrlPts[i]);
				if (tmpDist < dist) {
					dist = tmpDist;
					id = j.first;
				}
			}
			res.push_back(id);
			cout << "序号" << id << endl;
		}

		return res;
	}

	//在三维面上选择最近的点
	vector<int> selectCloserPoint(varray<Vec4> v, unordered_map<int, Point>& map) {
		vector<int> res;
		for (int i = 0; i < v.size(); ++i) {
			double dist = MAXINT;
			int id = -1;
			for (auto& j : map) {
				Vec3 tmp = pointTransToVec(j.second);
				double tmpDist = distance(tmp, v[i]);
				if (tmpDist < dist) {
					dist = tmpDist;
					id = j.first;
				}
			}
			res.push_back(id);
			cout << "序号" << id << endl;
		}
		return res;
	}

	//曲线曲面拟合
	//boundRes:	每个面片轮廓的空间数据点
	//uDegree:	曲线次数(自己设置)
	//uNum:		曲线控制点数(自己设置)
	varray<SplineSurface> fittingSurface(varray<varray<Spline>>& boundRes, int uDegree, int uNum) {
		RWGeometric rwg;
		//轮廓曲线拟合
		vector<int> res;										//空间点序号集合
		varray<Vec4> tmpV;										//数据点集
		varray<varray<Vec4>> bV2;
		varray<Spline> SL, SL1;									//SL用于Coons插值、SL1存放拟合后空间样条
		varray<SplineSurface> SS;								//拟合后的空间曲面
		for (int i = 0; i < boundRes.size(); i++) {
			for (int j = 0; j < boundRes[i].size(); j++) {
				res = selectCloserPoint(boundRes[i][j], map2d);
				for (int k = 0; k < res.size(); k++) {
					auto iter = map3d.find(res[k]);
					Vec3 v = pointTransToVec(iter->second);
					tmpV.push_back(v);
				}
				//样条拟合类对象
				FitBSpline fbL;
				fbL.FittingBspl(tmpV, uDegree, uNum);			//调用拟合函数，可得到拟合样条的节点矢量和控制点数据

				//利用得到的样条数据构建拟合样条
				Spline sl;
				sl.m_Degree = uDegree;
				sl.m_Knots = fbL.m_Knots;
				sl.m_CtrlPts = fbL.m_CtrlPts;
				SL.push_back(sl);
				SL1.push_back(sl);
				tmpV.clear();
			}
			SplineSurface ss;
			ss.CoonsInterpolate(SL);
			SL.clear();
			SS.push_back(ss);
		}
		bigger(SS, 100);
		rwg.WriteSpline("fitBSpline.txt", SL1);
		rwg.WriteSplineSurface("fitBSurface.txt", SS);
		return SS;
	}

	//曲线曲面拟合
	//bV:				存放的是各个面片的点集
	//boundRes:			存放各个面片轮廓上的点集(与bV中的面片顺序一致)
	//uGegree和vDegree:	拟合曲面的次数(自己设置)
	//uNum和vNum:		拟合曲面两个方向的控制点数量(自己设置)
	varray<SplineSurface> fittingSurface(varray<varray<Vec4>>& bV, varray<varray<Spline>>& boundRes, int uDegree, int vDegree, int uNum, int vNum) {
		RWGeometric rwg;
		varray<SplineSurface> SS;
		//寻找控制点对应原始曲面上的空间点
		vector<int> res;									//存放空间点对应序号
		varray<Vec4> bV1;									//存放空间点(原始曲面上的点)
		varray<varray<Vec4>> bV0;
		for (int j = 0; j < bV.size(); j++) {
			res = selectCloserPoint(bV[j], map2d);
			for (int i = 0; i < res.size(); i++) {
				auto iter = map3d.find(res[i]);
				Vec3 v = pointTransToVec(iter->second);
				bV1.push_back(v);
			}
			bV0.push_back(bV1);
		}
		cout << "空间点集点的数量：" << bV.size() << endl;
		varray<Vec4> tmpV;									//临时空间点容器
		varray<varray<Vec4>> bV2;							//存放边界处的空间点

		for (int i = 0; i < boundRes.size(); i++) {
			for (int j = 0; j < boundRes[i].size(); j++) {
				res = selectCloserPoint(boundRes[i][j], map2d);
				for (int k = 0; k < res.size(); k++) {
					auto iter = map3d.find(res[k]);
					Vec3 v = pointTransToVec(iter->second);
					tmpV.push_back(v);
				}
				bV2.push_back(tmpV);
				tmpV.clear();
			}
			FitBSplineSurface fb;
			fb.FittingBsurface(bV0[i], bV2, uDegree, vDegree, uNum, vNum);

			varray<Vec4> fbVec;
			for (auto& i : fb.m_uvCtrlPts) {
				for (auto& j : i) {
					fbVec.push_back(j);
				}
			}

			SplineSurface ss;
			ss.m_uDegree = uDegree;
			ss.m_vDegree = vDegree;
			ss.m_uKnots = fb.m_uKnots;
			ss.m_vKnots = fb.m_vKnots;
			ss.m_uNum = uNum;
			ss.m_vNum = vNum;
			ss.m_CtrlPts = fbVec;

			SS.push_back(ss);
			bV2.clear();
		}
		rwg.WriteSplineSurface("Surface.txt", SS);
		return SS;
	}

	varray<SplineSurface> quadSurface() {
		RWGeometric rwg;
		//亏格设置
		varray<bool> genus;
		genus.resize(3);
		genus[0] = false;
		genus[1] = true;
		genus[2] = true;
		/*genus[3] = true;*/
		/*genus[4] = true;
		genus[5] = true;*/
		/*genus[2] = true;
		genus[3] = true;*/
		/*genus[6] = true;*/
		MeshLib::Mesh mesh;
		varray<Vec4> vecs;

		//曲面展开
		unfoldMeshSurface("obj\\midfac.obj", vecs);

		//获取边界（带有简化）
		varray<varray<Spline>> boundRes;
		getMeshModelBoundary("obj\\Plane.obj", boundRes);

		//平面全自动四边剖分
		varray<varray<Vec4>> bV;							//存放各个面的所有控制点

		if (0)
		{
			varray<Spline> temp_Splines;
			rwg.ReadSpline("InputrResSpline.txt", temp_Splines);
			selectWire(temp_Splines, boundRes);
		}
		quadPlane(boundRes, genus, bV);

		//曲线曲面空间映射和拟合
		varray<SplineSurface> SS;
		SS = fittingSurface(boundRes, 2, 3);
		//SS = fittingSurface(bV, boundRes, 2, 2, 3, 3);

		////原始方法 当上述两个拟合方法得到的结果不对时，退而求其次选择原始方法(该方法精度不够)
		//varray<SplineSurface> SS;
		//varray<Spline> SLS;
		//for (int i = 0; i < boundRes.size(); i++) {
		//	vector<int> res;
		//	varray<Spline> SL;
		//	for (int j = 0; j < boundRes[i].size(); j++) {
		//		res = selectCloserPoint(boundRes[i][j], map2d);
		//		Spline SL1;
		//		SL1.m_Knots = boundRes[i][j].m_Knots;
		//		SL1.m_Degree = boundRes[i][j].m_Degree;
		//		for (int i = 0; i < res.size(); i++) {
		//			auto iter = map3d.find(res[i]);
		//			Vec3 v = pointTransToVec(iter->second);
		//			SL1.m_CtrlPts.push_back(v);
		//		}
		//		calWeight(SL1);
		//		SL.push_back(SL1);
		//	}
		//	SplineSurface ss;
		//	ss.CoonsInterpolate(SL);
		//	SS.push_back(ss);
		//
		//	rwg.WriteSplineSurface("Surface.txt", SS);
		//	for (auto i : SL) {
		//		SLS.push_back(i);
		//	}
		//
		//}
		//
		////空间轮廓线
		//rwg.WriteSpline("Splines1.txt", SLS);

		return SS;
	}

	/*
	 *	转换接口，与燕楠所写新数据结构配合
	 */
	void Transform(Vec4& v, point4d& p) {
		p.x = v.x;
		p.y = v.y;
		p.z = v.z;
		p.w = v.w;
	}

	void Transform(point4d& p, Vec4& v) {
		v.x = p.x;
		v.y = p.y;
		v.z = p.z;
		v.w = p.w;
	}

	void Transform(varray<Vec4>& v, vector<point4d>& p) {
		for (int i = 0; i < v.size(); i++) {
			Transform(v[i], p[i]);
		}
	}

	void Transform(vector<point4d>& p, varray<Vec4>& v) {
		v.resize(p.size());
		for (int i = 0; i < p.size(); i++) {
			Transform(p[i], v[i]);
		}
	}

	void Transform(SplineSurface& SS, YN::NurbsSurface& NS) {
		NS._u_Degree = SS.m_uDegree;
		NS._v_Degree = SS.m_vDegree;
		NS._u_Num = SS.m_uNum;
		NS._v_Num = SS.m_vNum;
		vector<point4d> p;
		Transform(SS.m_CtrlPts, p);
		auto p1 = make_shared<vector<point4d>>(p);
		NS._ControlPts = p1;

		for (auto& i : SS.m_uKnots) {
			NS._u_Knots->push_back(i);
		}
		for (auto& i : SS.m_vKnots) {
			NS._v_Knots->push_back(i);
		}
	}

	void Transform(YN::NurbsSurface& NS, SplineSurface& SS) {
		SS.m_uDegree = NS._u_Degree;
		SS.m_vDegree = NS._v_Degree;
		SS.m_uNum = NS._u_Num;
		SS.m_vNum = NS._v_Num;
		vector<double> v = *NS._u_Knots;//智能指针，指针呗
		for (auto& i : v) {
			SS.m_uKnots.push_back(i);
		}

		v = *NS._v_Knots;
		for (auto& i : v) {
			SS.m_vKnots.push_back(i);
		}

		Transform(*NS._ControlPts, SS.m_CtrlPts);
	}

	void Transform(varray<SplineSurface>& SS, vector<YN::NurbsSurface>& NS) {
		for (int i = 0; i < SS.size(); i++) {
			Transform(SS[i], NS[i]);
		}
	}

	void Transform(vector<YN::NurbsSurface>& NS, varray<SplineSurface>& SS) {
		SS.resize(NS.size());
		for (int i = 0; i < NS.size(); i++) {
			Transform(NS[i], SS[i]);
		}
	}

	void Transform(SplineVolume& SV, YN::NurbsVol& NV) {
		NV._u_Degree = SV.m_uDegree;
		NV._v_Degree = SV.m_vDegree;
		NV._w_Degree = SV.m_wDegree;
		NV._u_Num = SV.m_uNum;
		NV._v_Num = SV.m_vNum;
		NV._w_Num = SV.m_wNum;
		for (auto& i : SV.m_uKnots) {
			NV._u_Knots->push_back(i);
		}
		for (auto& i : SV.m_vKnots) {
			NV._v_Knots->push_back(i);
		}
		for (auto& i : SV.m_wKnots) {
			NV._w_Knots->push_back(i);
		}
		vector<point4d> p;
		Transform(SV.m_CtrlPts, p);
		for (auto& i : p) {
			NV._ControlPts->push_back(i);
		}
	}

	void Transform(varray<SplineVolume>& SV, vector<YN::NurbsVol>& NV) {
		for (int i = 0; i < SV.size(); i++) {
			Transform(SV[i], NV[i]);
		}
	}
};

//--------------------模型--------------------//

//汽车零件类
class CarPart {
public:

	//两翼尺寸参数
	double r = 1.5;//空洞倒角
	double angle = 102.53 * PI / 180;//斜边角度
	double angle1 = (90 - 75.07) * PI / 180;//两翼上偏移角度1
	double angle2 = (90 - 71.07) * PI / 180;//两翼上偏移角度2,其与后面四个为五层面的角度
	double angle3 = (90 - 70.07) * PI / 180;//两翼上偏移角度3
	double angle4 = (90 - 70.07) * PI / 180;//两翼上偏移角度4
	double angle5 = (90 - 69.07) * PI / 180;//两翼上偏移角度5
	double angle6 = (90 - 73.07) * PI / 180;//两翼上偏移角度6
	double Y_L = 58.22 * cos(angle1);//大小平面之间距离
	double Y_L1 = 24.21;//空洞轮廓长度
	double Y_L2;//侧翼斜长
	double Y_L3 = 40;//底边长度
	double Y_L4 = 2;//第三轮廓顶边长度
	double Y_L5 = 15;//顶面长度
	double Y_H = 5;//相邻连接部分宽度
	double Y_H1 = 10, Y_H3 = 13;//三个空洞高度
	double Y_H2;

	//与主体连接部分
	double Y_L7 = 17.6;//两翼底边中间部分长
	double Y_L8 = 37.5;//主体部分正视图边长
	double Y_L9 = 30;//主体部分正视图高

	//连接处替换部分
	double Y_L6 = (Y_L3 / 2 - cos(PI - angle) * Y_L) * 2;//与圆柱连接部分外围宽度

	//翼边平面平移距离
	double s1 = 5 - 5 * sqrt(2) / 4;
	double s2 = 5 + 5 * sqrt(2) / 4;
	double s3 = 20;
	double s4 = 30;
	//两侧圆柱半径
	double r1 = ((s3 - s2) > Y_L6) ? Y_L6 / 4 : (s3 - s2) / 4, r2 = Y_L4 / 4;
	//圆柱放样长度、零件长度、圆柱长度
	double h1 = sin(angle3) * Y_H, h2 = 10;

	Model_Solution m;
	PublicSolution ps;
	RWGeometric rwg;

	void initialize() {
		Y_H2 = Y_L2 - 3 * Y_H - Y_H1 - Y_H3 - Y_H / 2;//第三个空洞高度
	}

	//与圆柱连接的替换部分
	varray<SplineVolume> part() {
		varray<SplineSurface> SS, SS1;
		//内
		RandomModel rm((s3 - s2), Y_L4);
		SS = rm.getRecArcSurface();
		//外
		RandomModel rm1((s3 - s2), Y_L6);
		SS1 = rm1.getRecArcSurface();

		m.Trans(SS1, (sin(PI - angle) * Y_L / cos(angle3) - (Y_L - Y_H)) * cos(angle3), 3);
		m.Trans(SS1, (sin(PI - angle) * Y_L / cos(angle3) - (Y_L - Y_H)) * sin(angle3), -2);
		varray<SplineVolume> SV;
		rwg.WriteSplineSurface("E:\\kuang_models\\carPartSS.txt", SS);
		rwg.WriteSplineSurface("E:\\kuang_models\\carPartSS1.txt", SS1);
		for (auto i : SS1) {
			SS.push_back(i);
		}
		rwg.WriteSplineSurface("E:\\kuang_models\\carPartSS2.txt", SS);
		SV = ps.loft(SS1, SS);

		rwg.WriteSplineVolume("E:\\kuang_models\\carPartVolume1.txt", SV);
		return SV;
	}

	//圆柱和连接部分
	varray<SplineVolume> cylinderLoft() {
		varray<SplineVolume> SV, temp;
		SplineVolume tmp;
		PublicSolution ps;

		//内
		RandomModel rm(r2);
		varray<SplineSurface> SS;
		SS = rm.getArcSurface();
		//旋转45°
		m.Rolate(SS, PI / 4, 3);
		//外
		RandomModel rm1(r1);
		varray<SplineSurface> SS1;
		SS1 = rm1.getArcSurface();
		m.Rolate(SS1, PI / 4, 3);

		m.Trans(SS1, (sin(PI - angle) * Y_L / cos(angle3) - (Y_L - Y_H)) * cos(angle3), 3);
		m.Trans(SS1, (sin(PI - angle) * Y_L / cos(angle3) - (Y_L - Y_H)) * sin(angle3), -2);
		SV = ps.loft(SS, SS1);
		rwg.WriteSplineVolume("E:\\kuang_models\\carPartVolume3.txt", SV);
		temp = m.CreatSweepVol(SS1, h2, 3);
		rwg.WriteSplineVolume("E:\\kuang_models\\carPartVolume4.txt", temp);
		for (int i = 0; i < temp.size(); i++) {
			SV.push_back(temp[i]);
		}

		temp = part();
		for (int i = 0; i < temp.size(); i++) {
			SV.push_back(temp[i]);
		}
		m.Rolate(SV, -PI / 2, 1);
		m.Trans(SV, s2 + (s3 - s2) / 2, -3);
		m.Trans(SV, cos(angle3) * (Y_L - Y_H), 2);
		m.Trans(SV, sin(angle3) * (Y_L - Y_H), 3);

		rwg.WriteSplineSurface("E:\\kuang_models\\carPartSurface2.txt", SS);
		rwg.WriteSplineVolume("E:\\kuang_models\\carPartVolume2.txt", SV);
		return SV;
	}

	//翼---内外轮廓、连接线
	void carPartLine(varray<Spline>& outLine, varray<varray<Spline>>& inLine, varray<Spline>& addLine) {
		initialize();
		Vec4 v1 = { Y_L1 / 2,Y_H / 2,0,1 };
		Vec4 v2 = { -Y_L1 / 2,Y_H / 2,0,1 };
		Vec4 v3 = { -v2.x - Y_H1 / tan(PI - angle),Y_H / 2 + Y_H1,0,1 };
		Vec4 v4 = { v2.x + Y_H1 / tan(PI - angle),Y_H / 2 + Y_H1,0,1 };

		Vec4 v6 = { v2.x + (Y_H1 + Y_H) / tan(PI - angle),v2.y + Y_H1 + Y_H,0,1 };
		Vec4 v5 = { -v6.x,v6.y,0,1 };
		Vec4 v7 = { v6.x + Y_H2 / tan(PI - angle),v6.y + Y_H2 ,0,1 };
		Vec4 v8 = { -v7.x,v7.y,0,1 };

		Vec4 v10 = { -Y_L4 / 2,Y_L - Y_H,0,1 };
		Vec4 v9 = { v10.x - Y_H3 / tan(PI - angle),v10.y - Y_H3,0,1 };
		Vec4 v11 = { -v10.x,v10.y ,0,1 };
		Vec4 v12 = { -v9.x,v9.y,0,1 };

		//外轮廓坐标
		Vec4 p1 = { -Y_L3 / 2,0,0,1 };
		Vec4 p2 = { -Y_L3 / 2 + cos(PI - angle) * Y_L,sin(PI - angle) * Y_L2,0,1 };
		Vec4 p3 = { -p2.x,p2.y,0,1 };
		Vec4 p4 = { -p1.x,p1.y,0,1 };

		Vec4 p5 = { -Y_L7 / 2,0,0,1 };
		Vec4 p6 = { Y_L7 / 2,0,0,1 };

		varray<Spline> SL, inner1, inner2, inner3;//存储所有内轮廓线

		Spline sl, temp;
		Spline sl_1, temp_1;
		Spline sl_2, temp_2;

		//下方圆弧
		Circle0 cc1(r, angle);
		sl = cc1.getCircle();
		m.Rolate(sl, PI - angle / 2, 3);
		sl_2 = sl;
		m.Trans(sl, r + Y_H / 2, 2);
		m.Trans(sl, v2.x, 1);
		inner1.push_back(sl);
		sl_1 = sl;
		m.Trans(sl_1, Y_H1 + Y_H, 2);
		m.Trans(sl_1, (Y_H1 + Y_H) / tan(PI - angle), 1);
		inner2.push_back(sl_1);

		m.Trans(sl_2, Y_L - Y_H - Y_H3 + r, 2);
		m.Trans(sl_2, Y_L4 / 2 + Y_H3 / tan(PI - angle), -1);
		inner3.push_back(sl_2);

		Circle0 cc2(r, PI - angle);
		temp = cc2.getCircle();
		m.Rolate(temp, (PI - angle) / 2, 3);
		temp_2 = temp;
		m.Trans(temp, Y_H1 + Y_H / 2 - r, 2);
		m.Trans(temp, v4.x, 1);
		inner1.push_back(temp);
		temp_1 = temp;
		m.Trans(temp_1, Y_H2 + Y_H, 2);
		m.Trans(temp_1, (Y_H2 + Y_H) / tan(PI - angle), 1);
		inner2.push_back(temp_1);

		m.Trans(temp_2, Y_L - Y_H - r, 2);
		m.Trans(temp_2, Y_L4 / 2, -1);
		inner3.push_back(temp_2);

		Spline0 sl3(sl.m_CtrlPts[0], temp.m_CtrlPts[2]);
		sl = sl3.getSpline();
		inner1.push_back(sl);

		Spline0 sl4(sl_1.m_CtrlPts[0], temp_1.m_CtrlPts[2]);
		sl = sl4.getSpline();
		inner2.push_back(sl);

		Spline0 sl5(sl_2.m_CtrlPts[0], temp_2.m_CtrlPts[2]);
		sl = sl5.getSpline();
		inner3.push_back(sl);

		//镜像
		inner1 = ps.mirror(inner1, 2, 1);
		inner2 = ps.mirror(inner2, 2, 1);
		inner3 = ps.mirror(inner3, 2, 1);

		Spline0 sl1(v1, v2);
		sl = sl1.getSpline();
		inner1.push_back(sl);
		Spline0 sl2(v4, v3);
		sl = sl2.getSpline();
		inner1.push_back(sl);
		Spline0 sl11(v5, v6);
		sl = sl11.getSpline();
		inner2.push_back(sl);
		Spline0 sl12(v7, v8);
		sl = sl12.getSpline();
		inner2.push_back(sl);
		Spline0 sl13(v12, v9);
		sl = sl13.getSpline();
		inner3.push_back(sl);
		Spline0 sl14(v10, v11);
		sl = sl14.getSpline();
		inner3.push_back(sl);

		//内轮廓线
		inLine.push_back(inner1);
		inLine.push_back(inner2);
		inLine.push_back(inner3);

		varray<Spline> inSL;
		for (auto& i : inLine) {
			for (auto& j : i) {
				inSL.push_back(j);
			}
		}

		//外轮廓
		Spline0 nl1(p1, p2);
		sl = nl1.getSpline();
		SL.push_back(sl);

		/*Spline0 nl2(p2, v10);
		sl = nl2.getSpline();
		SL.push_back(sl);

		Spline0 nl3(v11, p3);
		sl = nl3.getSpline();
		SL.push_back(sl);*/

		Spline0 nl3(p2, p3);
		sl = nl3.getSpline();
		SL.push_back(sl);

		Spline0 nl4(p3, p4);
		sl = nl4.getSpline();
		SL.push_back(sl);

		Spline0 nl5(p4, p6);
		sl = nl5.getSpline();
		SL.push_back(sl);

		Spline0 nl12(p6, p5);
		sl = nl12.getSpline();
		SL.push_back(sl);

		Spline0 nl13(p5, p1);
		sl = nl13.getSpline();
		SL.push_back(sl);

		outLine = SL;
		SL.clear();

		Spline0 nl6(v4, v6);
		sl = nl6.getSpline();
		SL.push_back(sl);

		Spline0 nl7(v7, v9);
		sl = nl7.getSpline();
		SL.push_back(sl);

		Spline0 nl8(p2, v10);
		sl = nl8.getSpline();
		SL.push_back(sl);

		Spline0 nl9(v11, p3);
		sl = nl9.getSpline();
		SL.push_back(sl);

		Spline0 nl10(v2, p5);
		sl = nl10.getSpline();
		SL.push_back(sl);

		Spline0 nl11(v1, p6);
		sl = nl11.getSpline();
		SL.push_back(sl);
		addLine = SL;

		/*for (auto&i : addLine) {
			inSL.push_back(i);
		}*/
		for (auto& i : outLine) {
			inSL.push_back(i);
		}
		rwg.WriteSpline("E:\\kuang_models\\CarpartinSplines.txt", inSL);
		for (auto& i : addLine) {
			inSL.push_back(i);
		}
		rwg.WriteSpline("E:\\kuang_models\\CarpartinSplineAll.txt", inSL);
		//rwg.WriteSpline("E:\\kuang_models\\CarpartinSpline.txt", SL);
		/*rwg.WriteSpline("E:\\kuang_models\\CarpartoutSpline.txt", outLine);
		rwg.WriteSpline("E:\\kuang_models\\CarpartinSpline.txt", inSL);
		rwg.WriteSpline("E:\\kuang_models\\CarpartaddSpline.txt", addLine);*/
	}

	//汽车零件
	void carPart() {
		Model_Solution m;
		varray<Spline> outLine, outLine1, outLine2, outLine3, outLine4, outLine5;
		varray<varray<Spline>> inLine, inLine1, inLine2, inLine3, inLine4, inLine5;
		varray<Spline> addLine, addLine1, addLine2, addLine3, addLine4, addLine5;

		Y_L2 = Y_L / cos(angle2);
		carPartLine(outLine1, inLine1, addLine1);
		Y_L2 = Y_L / cos(angle3);
		carPartLine(outLine2, inLine2, addLine2);
		Y_L2 = Y_L / cos(angle4);
		carPartLine(outLine3, inLine3, addLine3);
		Y_L2 = Y_L / cos(angle5);
		carPartLine(outLine4, inLine4, addLine4);
		Y_L2 = Y_L / cos(angle6);
		carPartLine(outLine5, inLine5, addLine5);
		varray<bool> genus, genus1, genus2, genus3, genus4, genus5;
		varray<SplineSurface> allSurf, allSurf1, allSurf2, allSurf3, allSurf4, allSurf5;//存放剖分结果

		/*genus1.resize(4);
		genus1[0] = false;
		genus1[1] = true;
		genus1[2] = true;
		genus1[3] = true;*/
		genus2.resize(4);
		genus2[0] = false;
		genus2[1] = true;
		genus2[2] = true;
		genus2[3] = true;
		/*genus3.resize(4);
		genus3[0] = false;
		genus3[1] = true;
		genus3[2] = true;
		genus3[3] = true;*/
		/*genus4.resize(4);
		genus4[0] = false;
		genus4[1] = true;
		genus4[2] = true;
		genus4[3] = true;*/
		/*genus5.resize(4);
		genus5[0] = false;
		genus5[1] = true;
		genus5[2] = true;
		genus5[3] = true;*/

		//转移到第一象限
		for (auto& i : outLine2) {
			m.Trans(i, 50, 1);
		}
		for (auto& i : inLine2) {
			m.Trans(i, 50, 1);
		}
		for (auto& i : addLine2) {
			m.Trans(i, 50, 1);
		}

		//ps.quad(outLine1, inLine1, addLine1, genus1, allSurf1);
		ps.quad(outLine2, inLine2, addLine2, genus2, allSurf2);
		//ps.quad(outLine3, inLine3, addLine3, genus3, allSurf3);
		//ps.quad(outLine4, inLine4, addLine4, genus4, allSurf4);
		//ps.quad(outLine5, inLine5, addLine5, genus5, allSurf5);
		for (auto& i : allSurf2) {
			m.Trans(i, 50, -1);
		}
		rwg.WriteSplineSurface("E:\\kuang_models\\CarpartQuadsurf.txt", allSurf2);

		/*m.Rolate(allSurf1, angle2, 1);
		m.Trans(allSurf1, s1, -3);*/
		m.Rolate(allSurf2, angle3, 1);
		m.Trans(allSurf2, s2, -3);
		allSurf3 = allSurf2;
		allSurf1 = allSurf2;
		m.Trans(allSurf3, s3 - s2, -3);
		m.Trans(allSurf1, s2 - s1, 3);
		allSurf5 = allSurf1;
		m.Trans(allSurf5, s1, 3);
		allSurf4 = allSurf3;
		m.Trans(allSurf4, s4 - s3, -3);

		varray<SplineSurface> SS;

		for (int j = 0; j < allSurf1.size(); j++) {
			allSurf.push_back(allSurf1[j]);
		}
		for (int j = 0; j < allSurf2.size(); j++) {
			allSurf.push_back(allSurf2[j]);
		}
		for (int j = 0; j < allSurf3.size(); j++) {
			allSurf.push_back(allSurf3[j]);
		}
		for (int j = 0; j < allSurf4.size(); j++) {
			allSurf.push_back(allSurf4[j]);
		}
		for (int j = 0; j < allSurf4.size(); j++) {
			allSurf.push_back(allSurf5[j]);
		}
		rwg.WriteSplineSurface("E:\\kuang_models\\CarpartSurface1.txt", allSurf);

		varray<Spline> SL;
		for (int i = 0; i < inLine.size(); i++) {
			for (int j = 0; j < inLine[i].size(); j++) {
				SL.push_back(inLine[i][j]);
			}
		}
		varray<SplineVolume> SV, SV1, SV2, SV3, SV4, SV5;
		SV1 = m.CreatSweepVol(allSurf5, s1, -3);
		SV2 = m.CreatSweepVol(allSurf1, s2 - s1, -3);
		SV3 = m.CreatSweepVol(allSurf2, s3 - s2, -3);
		SV4 = m.CreatSweepVol(allSurf3, s4 - s3, -3);
		/*SV1 = ps.loft(allSurf5, allSurf1);
		SV2 = ps.loft(allSurf1, allSurf2);
		SV3 = ps.loft(allSurf2, allSurf3);
		SV4 = ps.loft(allSurf3, allSurf4);*/
		for (auto& i : SV1) {
			SV.push_back(i);
		}
		for (auto& i : SV2) {
			SV.push_back(i);
		}
		for (auto& i : SV3) {
			SV.push_back(i);
		}

		for (auto& i : SV4) {
			SV.push_back(i);
		}
		for (auto& i : SV) {
			i.OrderCtrlPts(i);
		}

		SV.erase(SV.begin() + 49);
		rwg.WriteSplineVolume("E:\\kuang_models\\CarpartVolumeYi.txt", SV);
		SV5 = cylinderLoft();
		for (int i = 0; i < SV5.size(); i++) {
			SV.push_back(SV5[i]);
		}
		SV[106].OrderCtrlPts(SV[106]);
		SV[108].OrderCtrlPts(SV[108]);
		SV[105].OrderCtrlPts(SV[105]);
		SV[107].OrderCtrlPts(SV[107]);
		for (auto& i : SV) {
			i.OrderCtrlPts(i);
		}
		cout << SV.size() << endl;
		m.Trans(SV, Y_L8 / 2, 2);

		cout << "零件左边翼......" << endl;
		rwg.WriteSplineVolume("E:\\kuang_models\\CarpartVolumeYiAll.txt", SV);

		varray<SplineVolume> SVtemp;
		m.MirrorVols(SV, SVtemp, 2);
		for (auto& i : SVtemp) {
			i.OrderCtrlPts(i);
		}
		for (auto& i : SVtemp) {
			SV.push_back(i);
		}
		rwg.WriteSplineVolume("E:\\kuang_models\\CarpartVolumeYiAll2.txt", SV);

		//rwg.WriteSplineSurface("E:\\kuang_models\\CarpartSurface.txt", allSurf);
		//rwg.WriteSplineVolume("E:\\kuang_models\\CarpartVolume.txt", SV);
		//rwg.WriteSpline("E:\\kuang_models\\CarpartoutSpline.txt", outLine);
		//rwg.WriteSpline("E:\\kuang_models\\CarpartinSpline.txt", SL);
		//rwg.WriteSpline("E:\\kuang_models\\CarpartaddSpline.txt", addLine);

		m.Rolate(SV, PI / 2, 3);
		m.Trans(SV, Y_L3 / 2, 2);
		m.Trans(SV, Y_L8 / 2, 1);
		m.Trans(SV, Y_L9, 3);

		SVtemp.clear();
		//读取主体部分
		rwg.ReadSplineVolume("E:\\kuang_models\\kuang\\carpart\\AllcarModel.txt", SVtemp);
		for (auto& i : SVtemp) {
			SV.push_back(i);
		}

		/*YN::WingStruct ws("E:\\kuang_models\\kuang\\carpartctrlpts.txt", true);
		vector<YN::NurbsSurface> YNSS;
		varray<SplineSurface> YNSS1;
		YNSS = ws.getModelSurface();
		ps.Transform(YNSS, YNSS1);*/

		for (int i = 0; i < SV.size(); i++) {
			SV[i].KnotsRefineNum(1);
		}
		ps.outPutVTK(SV, "E:\\kuang_models\\CarpartVolume.vtk");
		rwg.WriteSplineVolume("E:\\kuang_models\\CarpartVolumeAll.txt", SV);
		//ps.outPutTXT(SV, "E:\\kuang_models\\CarpartVolume444");
		//rwg.WriteNurbsSurface("E:\\kuang_models\\CarpartVolume.txt", YNSS1);
		//rwg.WriteSplineSurface("E:\\kuang_models\\CarpartVolume.txt", YNSS1);
		//rwg.WriteNurbsVol("E:\\kuang_models\\CarpartVolume.txt", NVtemp);
	}
};

//机床
class MachineTool {
private:
	//缩放
	double x = (1.1 * 1.1 * 11.9) / 92;

	//y轴床身
	double L1 = 80 * x, L2 = 25 * x, L3 = 15 * x, H1 = 40 * x, H2 = 12 * x;//截面尺寸
	double M1 = 200 * x, M2 = 50 * x, M3 = 18 * x, M4 = 18 * x, M5 = 15 * x, M6 = 15 * x;//床身长度等

	//底座
	double a1 = 180 * x, b1 = 100 * x, b2 = 40 * x, c1 = 240 * x;
	double a2 = 2 * M3 / 3;

	//支撑件
	double c2 = c1 / 30;//拉伸长度

	//连接部分参数
	double u = 1.1 * 1.1 * 11.9;//长
	double v = 10;//宽
	double w = 12;//高

	Model_Solution m;
	RWGeometric rwg;

public:
	//y轴床身
	varray<SplineVolume> machineTool1() {
		/*
			可以使用PublicModels中的RandomModel函数创建四边形
			后面可以更改，以减少代码重复量
		*/
		varray<Spline> SL1;
		varray<Spline> SL2;
		varray<Spline> SL3;
		varray<Spline> SL4;
		varray<Spline> SL5;
		varray<Spline> SL6;
		varray<Spline> SL7;
		varray<Spline> SL8;
		varray<Spline> SL9;
		varray<double> knots;
		knots.push_back(0);
		knots.push_back(0);
		knots.push_back(0);
		knots.push_back(1);
		knots.push_back(1);
		knots.push_back(1);
		SL1.resize(4);
		SL2.resize(4);
		SL3.resize(4);
		SL4.resize(4);
		SL5.resize(4);
		SL6.resize(4);
		SL7.resize(4);
		SL8.resize(4);
		SL9.resize(4);
		for (int i = 0; i < 4; i++) {
			SL1[i].m_Degree = 2;
			SL1[i].m_Knots = knots;
			SL2[i].m_Degree = 2;
			SL2[i].m_Knots = knots;
			SL3[i].m_Degree = 2;
			SL3[i].m_Knots = knots;
			SL4[i].m_Degree = 2;
			SL4[i].m_Knots = knots;
			SL5[i].m_Degree = 2;
			SL5[i].m_Knots = knots;
			SL6[i].m_Degree = 2;
			SL6[i].m_Knots = knots;
			SL7[i].m_Degree = 2;
			SL7[i].m_Knots = knots;
			SL8[i].m_Degree = 2;
			SL8[i].m_Knots = knots;
			SL9[i].m_Degree = 2;
			SL9[i].m_Knots = knots;
		}
		//截面外轮廓点坐标
		Vec4 v1 = { -L1 / 2,-H1 / 2,0,1 };
		Vec4 v2 = { -L1 / 2,0,0,1 };
		Vec4 v3 = { -L1 / 4,0,0,1 };
		Vec4 v4 = { -L1 / 4,H2,0,1 };
		Vec4 v5 = { -L2 / 2 - L3,H2,0,1 };
		Vec4 v6 = { -L2 / 2 - L3,H1 / 2,0,1 };
		Vec4 v7 = { -L2 / 2 ,H1 / 2,0,1 };
		Vec4 v8 = { -L2 / 2 ,0,0,1 };
		Vec4 v9 = { L2 / 2 ,0,0,1 };
		Vec4 v10 = { L2 / 2 ,H1 / 2,0,1 };
		Vec4 v11 = { L2 / 2 + L3,H1 / 2,0,1 };
		Vec4 v12 = { L2 / 2 + L3,H2,0,1 };
		Vec4 v13 = { L1 / 4,H2,0,1 };
		Vec4 v14 = { L1 / 4,0,0,1 };
		Vec4 v15 = { L1 / 2,0,0,1 };
		Vec4 v16 = { L1 / 2,-H1 / 2,0,1 };
		Vec4 v17 = { L2 / 2,-H1 / 2,0,1 };
		Vec4 v18 = { -L2 / 2,-H1 / 2,0,1 };
		Vec4 v19 = (v1 + v18) / 2;
		Vec4 v20 = (v17 + v16) / 2;

		SL1[0].m_CtrlPts.push_back(v1);
		SL1[0].m_CtrlPts.push_back((v1 + v19) / 2);
		SL1[0].m_CtrlPts.push_back(v19);
		SL1[1].m_CtrlPts.push_back(v1);
		SL1[1].m_CtrlPts.push_back((v1 + v2) / 2);
		SL1[1].m_CtrlPts.push_back(v2);
		SL1[2].m_CtrlPts.push_back(v2);
		SL1[2].m_CtrlPts.push_back((v2 + v3) / 2);
		SL1[2].m_CtrlPts.push_back(v3);
		SL1[3].m_CtrlPts.push_back(v19);
		SL1[3].m_CtrlPts.push_back((v19 + v3) / 2);
		SL1[3].m_CtrlPts.push_back(v3);
		SplineSurface ss1;
		ss1.CoonsInterpolate(SL1);

		SL2[0].m_CtrlPts.push_back(v19);
		SL2[0].m_CtrlPts.push_back((v18 + v19) / 2);
		SL2[0].m_CtrlPts.push_back(v18);
		SL2[1].m_CtrlPts.push_back(v19);
		SL2[1].m_CtrlPts.push_back((v19 + v3) / 2);
		SL2[1].m_CtrlPts.push_back(v3);
		SL2[2].m_CtrlPts.push_back(v3);
		SL2[2].m_CtrlPts.push_back((v8 + v3) / 2);
		SL2[2].m_CtrlPts.push_back(v8);
		SL2[3].m_CtrlPts.push_back(v18);
		SL2[3].m_CtrlPts.push_back((v18 + v8) / 2);
		SL2[3].m_CtrlPts.push_back(v8);
		SplineSurface ss2;
		ss2.CoonsInterpolate(SL2);

		SL3[0].m_CtrlPts.push_back(v18);
		SL3[0].m_CtrlPts.push_back((v18 + v17) / 2);
		SL3[0].m_CtrlPts.push_back(v17);
		SL3[1].m_CtrlPts.push_back(v18);
		SL3[1].m_CtrlPts.push_back((v18 + v8) / 2);
		SL3[1].m_CtrlPts.push_back(v8);
		SL3[2].m_CtrlPts.push_back(v8);
		SL3[2].m_CtrlPts.push_back((v8 + v9) / 2);
		SL3[2].m_CtrlPts.push_back(v9);
		SL3[3].m_CtrlPts.push_back(v17);
		SL3[3].m_CtrlPts.push_back((v17 + v9) / 2);
		SL3[3].m_CtrlPts.push_back(v9);
		SplineSurface ss3;
		ss3.CoonsInterpolate(SL3);

		SL4[0].m_CtrlPts.push_back(v17);
		SL4[0].m_CtrlPts.push_back((v17 + v20) / 2);
		SL4[0].m_CtrlPts.push_back(v20);
		SL4[1].m_CtrlPts.push_back(v17);
		SL4[1].m_CtrlPts.push_back((v17 + v9) / 2);
		SL4[1].m_CtrlPts.push_back(v9);
		SL4[2].m_CtrlPts.push_back(v9);
		SL4[2].m_CtrlPts.push_back((v9 + v14) / 2);
		SL4[2].m_CtrlPts.push_back(v14);
		SL4[3].m_CtrlPts.push_back(v20);
		SL4[3].m_CtrlPts.push_back((v20 + v14) / 2);
		SL4[3].m_CtrlPts.push_back(v14);
		SplineSurface ss4;
		ss4.CoonsInterpolate(SL4);

		SL5[0].m_CtrlPts.push_back(v20);
		SL5[0].m_CtrlPts.push_back((v20 + v16) / 2);
		SL5[0].m_CtrlPts.push_back(v16);
		SL5[1].m_CtrlPts.push_back(v20);
		SL5[1].m_CtrlPts.push_back((v20 + v14) / 2);
		SL5[1].m_CtrlPts.push_back(v14);
		SL5[2].m_CtrlPts.push_back(v14);
		SL5[2].m_CtrlPts.push_back((v14 + v15) / 2);
		SL5[2].m_CtrlPts.push_back(v15);
		SL5[3].m_CtrlPts.push_back(v16);
		SL5[3].m_CtrlPts.push_back((v16 + v15) / 2);
		SL5[3].m_CtrlPts.push_back(v15);
		SplineSurface ss5;
		ss5.CoonsInterpolate(SL5);

		SL6[0].m_CtrlPts.push_back(v3);
		SL6[0].m_CtrlPts.push_back((v3 + v8) / 2);
		SL6[0].m_CtrlPts.push_back(v8);
		SL6[1].m_CtrlPts.push_back(v3);
		SL6[1].m_CtrlPts.push_back((v3 + v4) / 2);
		SL6[1].m_CtrlPts.push_back(v4);
		SL6[2].m_CtrlPts.push_back(v4);
		SL6[2].m_CtrlPts.push_back((v4 + v7) / 2);
		SL6[2].m_CtrlPts.push_back(v7);
		SL6[3].m_CtrlPts.push_back(v8);
		SL6[3].m_CtrlPts.push_back((v8 + v7) / 2);
		SL6[3].m_CtrlPts.push_back(v7);
		SplineSurface ss6;
		ss6.CoonsInterpolate(SL6);

		SL7[0].m_CtrlPts.push_back(v5);
		SL7[0].m_CtrlPts.push_back((v5 + v4) / 2);
		SL7[0].m_CtrlPts.push_back(v4);
		SL7[1].m_CtrlPts.push_back(v5);
		SL7[1].m_CtrlPts.push_back((v5 + v6) / 2);
		SL7[1].m_CtrlPts.push_back(v6);
		SL7[2].m_CtrlPts.push_back(v6);
		SL7[2].m_CtrlPts.push_back((v6 + v7) / 2);
		SL7[2].m_CtrlPts.push_back(v7);
		SL7[3].m_CtrlPts.push_back(v4);
		SL7[3].m_CtrlPts.push_back((v7 + v4) / 2);
		SL7[3].m_CtrlPts.push_back(v7);
		SplineSurface ss7;
		ss7.CoonsInterpolate(SL7);

		SL8[0].m_CtrlPts.push_back(v9);
		SL8[0].m_CtrlPts.push_back((v14 + v9) / 2);
		SL8[0].m_CtrlPts.push_back(v14);
		SL8[1].m_CtrlPts.push_back(v9);
		SL8[1].m_CtrlPts.push_back((v9 + v10) / 2);
		SL8[1].m_CtrlPts.push_back(v10);
		SL8[2].m_CtrlPts.push_back(v10);
		SL8[2].m_CtrlPts.push_back((v10 + v13) / 2);
		SL8[2].m_CtrlPts.push_back(v13);
		SL8[3].m_CtrlPts.push_back(v14);
		SL8[3].m_CtrlPts.push_back((v14 + v13) / 2);
		SL8[3].m_CtrlPts.push_back(v13);
		SplineSurface ss8;
		ss8.CoonsInterpolate(SL8);

		SL9[0].m_CtrlPts.push_back(v13);
		SL9[0].m_CtrlPts.push_back((v13 + v12) / 2);
		SL9[0].m_CtrlPts.push_back(v12);
		SL9[1].m_CtrlPts.push_back(v13);
		SL9[1].m_CtrlPts.push_back((v10 + v13) / 2);
		SL9[1].m_CtrlPts.push_back(v10);
		SL9[2].m_CtrlPts.push_back(v10);
		SL9[2].m_CtrlPts.push_back((v10 + v11) / 2);
		SL9[2].m_CtrlPts.push_back(v11);
		SL9[3].m_CtrlPts.push_back(v12);
		SL9[3].m_CtrlPts.push_back((v12 + v11) / 2);
		SL9[3].m_CtrlPts.push_back(v11);
		SplineSurface ss9;
		ss9.CoonsInterpolate(SL9);

		varray<SplineSurface> SS;
		SS.push_back(ss1);
		SS.push_back(ss2);
		SS.push_back(ss3);
		SS.push_back(ss4);
		SS.push_back(ss5);
		SS.push_back(ss6);
		SS.push_back(ss7);
		SS.push_back(ss8);
		SS.push_back(ss9);

		Model_Solution m;
		double M7 = M1 / 2 - M2 / 2 - M3 - M4 - M5;
		varray<SplineVolume> SV;
		varray<SplineVolume> SV1;
		varray<SplineVolume> SV2;
		varray<SplineVolume> SV3;
		varray<SplineVolume> SV4;
		varray<SplineVolume> SV5;
		varray<SplineVolume> SV6;
		varray<SplineVolume> SV7;
		varray<SplineVolume> SV8;
		varray<SplineVolume> SV9;
		SV1 = m.CreatSweepVol(SS, M7, 3);
		m.Trans(SS, M7, 3);
		SV2 = m.CreatSweepVol(SS, M5, 3);
		m.Trans(SS, M5, 3);
		SV3 = m.CreatSweepVol(SS, M4, 3);
		m.Trans(SS, M4, 3);
		SV4 = m.CreatSweepVol(SS, M3, 3);
		m.Trans(SS, M3, 3);
		SV5 = m.CreatSweepVol(SS, M2, 3);
		m.Trans(SS, M2, 3);
		SV6 = m.CreatSweepVol(SS, M3, 3);
		m.Trans(SS, M3, 3);
		SV7 = m.CreatSweepVol(SS, M4, 3);
		m.Trans(SS, M4, 3);
		SV8 = m.CreatSweepVol(SS, M5, 3);
		m.Trans(SS, M5, 3);
		SV9 = m.CreatSweepVol(SS, M7, 3);
		m.Trans(SS, M7, 3);
		for (int i = 0; i < 9; i++) {
			SV.push_back(SV1[i]);
			SV.push_back(SV2[i]);
			SV.push_back(SV3[i]);
			SV.push_back(SV4[i]);
			SV.push_back(SV5[i]);
			SV.push_back(SV6[i]);
			SV.push_back(SV7[i]);
			SV.push_back(SV8[i]);
			SV.push_back(SV9[i]);
		}

		////rwg.WriteSplineSurface("E:\\kuang_models\\machineToolSurface.txt", SS);
		rwg.WriteSplineVolume("E:\\kuang_models\\machineToolVolume1.txt", SV);

		return SV;
	}
	varray<SplineVolume> machineTool2() {
		/*
			可以使用PublicModels中的RandomModel函数创建四边形
			后面可以更改，以减少代码重复量
		*/
		varray<Spline> SL1;
		varray<Spline> SL2;
		varray<Spline> SL3;
		varray<Spline> SL4;
		varray<Spline> SL5;
		varray<Spline> SL6;
		varray<double> knots;
		knots.push_back(0);
		knots.push_back(0);
		knots.push_back(0);
		knots.push_back(1);
		knots.push_back(1);
		knots.push_back(1);
		SL1.resize(4);
		SL2.resize(4);
		SL3.resize(4);
		SL4.resize(4);
		SL5.resize(4);
		SL6.resize(4);

		for (int i = 0; i < 4; i++) {
			SL1[i].m_Degree = 2;
			SL1[i].m_Knots = knots;
			SL2[i].m_Degree = 2;
			SL2[i].m_Knots = knots;
			SL3[i].m_Degree = 2;
			SL3[i].m_Knots = knots;
			SL4[i].m_Degree = 2;
			SL4[i].m_Knots = knots;
			SL5[i].m_Degree = 2;
			SL5[i].m_Knots = knots;
			SL6[i].m_Degree = 2;
			SL6[i].m_Knots = knots;
		}
		//截面外轮廓点坐标
		Vec4 v1 = { -M2 / 2 - M3 - M4 - M5,-M6,0,1 };
		Vec4 v2 = { -M2 / 2 - M3 - M4 - M5,0,0,1 };
		Vec4 v3 = { -M2 / 2 - M3 - M4,0,0,1 };
		Vec4 v4 = { -M2 / 2 - M3 - M4,-M6,0,1 };
		Vec4 v5 = { -M2 / 2 - M3,-M6 + a2,0,1 };
		Vec4 v6 = { -M2 / 2 - M3,0,0,1 };
		Vec4 v7 = { -M2 / 2 ,0,0,1 };
		Vec4 v8 = { -M2 / 2 - 2 * M3 / 3 ,-M6 + a2,0,1 };
		Vec4 v9 = { -M2 / 2 - 2 * M3 / 3 ,-M6,0,1 };
		Vec4 v10 = { -M2 / 2 ,-M6 ,0,1 };
		Vec4 v11 = { M2 / 2 ,-M6 ,0,1 };
		Vec4 v12 = { M2 / 2 ,0,0,1 };
		Vec4 v13 = { M2 / 2 + M3,0,0,1 };
		Vec4 v14 = { M2 / 2 + M3,-M6 + a2,0,1 };
		Vec4 v15 = { M2 / 2 + 2 * M3 / 3 ,-M6 + a2,0,1 };
		Vec4 v16 = { M2 / 2 + 2 * M3 / 3 ,-M6,0,1 };
		Vec4 v17 = { M2 / 2 + M3 + M4,-M6,0,1 };
		Vec4 v18 = { M2 / 2 + M3 + M4,0,0,1 };
		Vec4 v19 = { M2 / 2 + M3 + M4 + M5,0,0,1 };
		Vec4 v20 = { M2 / 2 + M3 + M4 + M5,-M6,0,1 };

		SL1[0].m_CtrlPts.push_back(v1);
		SL1[0].m_CtrlPts.push_back((v1 + v4) / 2);
		SL1[0].m_CtrlPts.push_back(v4);
		SL1[1].m_CtrlPts.push_back(v1);
		SL1[1].m_CtrlPts.push_back((v1 + v2) / 2);
		SL1[1].m_CtrlPts.push_back(v2);
		SL1[2].m_CtrlPts.push_back(v2);
		SL1[2].m_CtrlPts.push_back((v2 + v3) / 2);
		SL1[2].m_CtrlPts.push_back(v3);
		SL1[3].m_CtrlPts.push_back(v4);
		SL1[3].m_CtrlPts.push_back((v4 + v3) / 2);
		SL1[3].m_CtrlPts.push_back(v3);
		SplineSurface ss1;
		ss1.CoonsInterpolate(SL1);

		SL2[0].m_CtrlPts.push_back(v5);
		SL2[0].m_CtrlPts.push_back((v5 + v8) / 2);
		SL2[0].m_CtrlPts.push_back(v8);
		SL2[1].m_CtrlPts.push_back(v5);
		SL2[1].m_CtrlPts.push_back((v5 + v6) / 2);
		SL2[1].m_CtrlPts.push_back(v6);
		SL2[2].m_CtrlPts.push_back(v6);
		SL2[2].m_CtrlPts.push_back((v6 + v7) / 2);
		SL2[2].m_CtrlPts.push_back(v7);
		SL2[3].m_CtrlPts.push_back(v8);
		SL2[3].m_CtrlPts.push_back((v7 + v8) / 2);
		SL2[3].m_CtrlPts.push_back(v7);
		SplineSurface ss2;
		ss2.CoonsInterpolate(SL2);

		SL3[0].m_CtrlPts.push_back(v9);
		SL3[0].m_CtrlPts.push_back((v9 + v10) / 2);
		SL3[0].m_CtrlPts.push_back(v10);
		SL3[1].m_CtrlPts.push_back(v9);
		SL3[1].m_CtrlPts.push_back((v9 + v8) / 2);
		SL3[1].m_CtrlPts.push_back(v8);
		SL3[2].m_CtrlPts.push_back(v8);
		SL3[2].m_CtrlPts.push_back((v8 + v7) / 2);
		SL3[2].m_CtrlPts.push_back(v7);
		SL3[3].m_CtrlPts.push_back(v10);
		SL3[3].m_CtrlPts.push_back((v10 + v7) / 2);
		SL3[3].m_CtrlPts.push_back(v7);
		SplineSurface ss3;
		ss3.CoonsInterpolate(SL3);

		SL4[0].m_CtrlPts.push_back(v11);
		SL4[0].m_CtrlPts.push_back((v11 + v16) / 2);
		SL4[0].m_CtrlPts.push_back(v16);
		SL4[1].m_CtrlPts.push_back(v11);
		SL4[1].m_CtrlPts.push_back((v11 + v12) / 2);
		SL4[1].m_CtrlPts.push_back(v12);
		SL4[2].m_CtrlPts.push_back(v12);
		SL4[2].m_CtrlPts.push_back((v12 + v15) / 2);
		SL4[2].m_CtrlPts.push_back(v15);
		SL4[3].m_CtrlPts.push_back(v16);
		SL4[3].m_CtrlPts.push_back((v16 + v15) / 2);
		SL4[3].m_CtrlPts.push_back(v15);
		SplineSurface ss4;
		ss4.CoonsInterpolate(SL4);

		SL5[0].m_CtrlPts.push_back(v15);
		SL5[0].m_CtrlPts.push_back((v15 + v14) / 2);
		SL5[0].m_CtrlPts.push_back(v14);
		SL5[1].m_CtrlPts.push_back(v15);
		SL5[1].m_CtrlPts.push_back((v15 + v12) / 2);
		SL5[1].m_CtrlPts.push_back(v12);
		SL5[2].m_CtrlPts.push_back(v12);
		SL5[2].m_CtrlPts.push_back((v12 + v13) / 2);
		SL5[2].m_CtrlPts.push_back(v13);
		SL5[3].m_CtrlPts.push_back(v14);
		SL5[3].m_CtrlPts.push_back((v14 + v13) / 2);
		SL5[3].m_CtrlPts.push_back(v13);
		SplineSurface ss5;
		ss5.CoonsInterpolate(SL5);

		SL6[0].m_CtrlPts.push_back(v17);
		SL6[0].m_CtrlPts.push_back((v17 + v20) / 2);
		SL6[0].m_CtrlPts.push_back(v20);
		SL6[1].m_CtrlPts.push_back(v17);
		SL6[1].m_CtrlPts.push_back((v17 + v18) / 2);
		SL6[1].m_CtrlPts.push_back(v18);
		SL6[2].m_CtrlPts.push_back(v18);
		SL6[2].m_CtrlPts.push_back((v18 + v19) / 2);
		SL6[2].m_CtrlPts.push_back(v19);
		SL6[3].m_CtrlPts.push_back(v20);
		SL6[3].m_CtrlPts.push_back((v20 + v19) / 2);
		SL6[3].m_CtrlPts.push_back(v19);
		SplineSurface ss6;
		ss6.CoonsInterpolate(SL6);

		varray<SplineSurface> SS;
		SS.push_back(ss1);
		SS.push_back(ss2);
		SS.push_back(ss3);
		SS.push_back(ss4);
		SS.push_back(ss5);
		SS.push_back(ss6);

		Model_Solution m;
		varray<SplineVolume> SV;
		SV = m.CreatSweepVol(SS, L1, 3);

		rwg.WriteSplineSurface("E:\\kuang_models\\machineToolSurface2.txt", SS);
		rwg.WriteSplineVolume("E:\\kuang_models\\machineToolVolume2.txt", SV);

		return SV;
	}

	//支撑件
	varray<SplineVolume> machineTool4() {
		double d1 = b2 / 2, d2 = b2 / 3;
		double d3 = 2 * a2 / 3;

		Vec4 v1 = { -d2,0,0,1 };
		Vec4 v2 = { -d2,d1 / 4,0,1 };
		Vec4 v3 = { -d2,d1 / 2,0,1 };
		Vec4 v4 = { -d2,d1,0,1 };
		Vec4 v5 = { 0,b2,0,1 };
		Vec4 v6 = { -d2 / 3,d1 / 2,0,1 };
		Vec4 v7 = { -d2 / 3,d1 / 4,0,1 };
		Vec4 v8 = { 0,0,0,1 };

		varray<SplineSurface> SS;
		SplineSurface ss;
		SplineSurface ss1;
		RandomModel rm1(v1, v2, v7, v8);
		ss = rm1.getSurface();
		SS.push_back(ss);

		RandomModel rm2(v2, v3, v6, v7);
		ss = rm2.getSurface();
		ss1 = ss;
		SS.push_back(ss);

		RandomModel rm3(v3, v4, v5, v6);
		ss = rm3.getSurface();
		SS.push_back(ss);

		RandomModel rm4(v7, v6, v5, v8);
		ss = rm4.getSurface();
		SS.push_back(ss);

		varray<SplineVolume> SV;
		SV = m.CreatSweepVol(SS, c2, 3);

		varray<SplineVolume> SV1;
		SV1 = SV;
		m.Trans(SV1, c2 * 4, 3);
		for (int i = 0; i < SV1.size(); i++) {
			SV.push_back(SV1[i]);
		}

		SS.clear();
		SS.push_back(ss1);
		varray<SplineVolume> SV2;
		SV2 = m.CreatSweepVol(SS, c2 * 3, 3);
		m.Trans(SV2, c2, 3);
		for (int i = 0; i < SV2.size(); i++) {
			SV.push_back(SV2[i]);
		}

		SV1 = SV;
		SV.clear();
		m.Trans(SV1, a1 / 2, -1);
		m.Trans(SV1, 5 * c2, -3);
		for (int i = 0; i < SV1.size(); i++) {
			SV.push_back(SV1[i]);
		}
		m.Trans(SV1, 3 * c1 / 4 - 5 * c2, -3);
		for (int i = 0; i < SV1.size(); i++) {
			SV.push_back(SV1[i]);
		}
		SV1 = SV;
		m.Rolate(SV1, PI, 2);
		m.Trans(SV1, 3 * c1 / 4, -3);
		for (int i = 0; i < SV1.size(); i++) {
			SV.push_back(SV1[i]);
		}
		m.Trans(SV, 3 * c1 / 4, 3);

		rwg.WriteSplineVolume("E:\\kuang_models\\machineToolVolume3.txt", SV);

		return SV;
	}

	//底座
	varray<SplineVolume> machineTool3() {
		/*
			可以使用PublicModels中的RandomModel函数创建四边形
			后面可以更改，以减少代码重复量
		*/
		varray<Spline> SL1;
		varray<Spline> SL2;
		varray<double> knots;
		knots.push_back(0);
		knots.push_back(0);
		knots.push_back(0);
		knots.push_back(1);
		knots.push_back(1);
		knots.push_back(1);
		SL1.resize(4);
		SL2.resize(4);
		for (int i = 0; i < 4; i++) {
			SL1[i].m_Degree = 2;
			SL1[i].m_Knots = knots;
			SL2[i].m_Degree = 2;
			SL2[i].m_Knots = knots;
		}
		//外轮廓点坐标
		Vec4 v1 = { -a1 / 2 ,0,0,1 };
		Vec4 v2 = { -a1 / 2 ,b2,0,1 };
		Vec4 v3 = { -M2 / 2 - a2 * 3,b1 - a2 * 2,0,1 };
		Vec4 v4 = { -M2 / 2 - a2 * 3,0,0,1 };
		Vec4 v5 = { M2 / 2 + a2 * 3,0,0,1 };
		Vec4 v6 = { M2 / 2 + a2 * 3,b1 - a2 * 2,0,1 };
		Vec4 v7 = { a1 / 2 ,b2,0,1 };
		Vec4 v8 = { a1 / 2 ,0,0,1 };

		Vec4 v9 = { -M2 / 2 - a2 * 2,0,0,1 };
		Vec4 v10 = { -M2 / 2 - a2 * 2 ,a2,0,1 };
		Vec4 v11 = { -M2 / 2 - a2 * 3 ,a2,0,1 };
		Vec4 v12 = { -M2 / 2 - a2 * 3,M6,0,1 };
		Vec4 v13 = { -M2 / 2 - M3,M6,0,1 };
		Vec4 v14 = { -M2 / 2 - M3,a2,0,1 };
		Vec4 v15 = { -M2 / 2 - a2,a2,0,1 };
		Vec4 v16 = { -M2 / 2 - a2,0,0,1 };
		Vec4 v17 = { M2 / 2 + a2,0,0,1 };
		Vec4 v18 = { M2 / 2 + a2,a2,0,1 };
		Vec4 v19 = { M2 / 2 + M3,a2,0,1 };
		Vec4 v20 = { M2 / 2 + M3,M6,0,1 };
		Vec4 v21 = { M2 / 2 + a2 * 3,M6,0,1 };
		Vec4 v22 = { M2 / 2 + a2 * 3 ,a2,0,1 };
		Vec4 v23 = { M2 / 2 + a2 * 2 ,a2,0,1 };
		Vec4 v24 = { M2 / 2 + a2 * 2,0,0,1 };

		//底座最底层部分
		varray<SplineSurface> SS;
		varray<SplineSurface> SS3;//底座补充两边面片
		varray<SplineSurface> ss1;
		varray<SplineSurface> ss2;
		Rectangle0 rt(a2, a2);
		SplineSurface ss;
		ss = rt.getSurface();
		m.Trans(ss, M2 / 2 + a2 + a2 / 2, -1);
		m.Trans(ss, b1 + a2 / 2, 2);
		ss1.push_back(ss);
		m.Trans(ss, a2, 1);
		ss1.push_back(ss);
		m.Trans(ss, M2 + a2, 1);
		ss1.push_back(ss);
		m.Trans(ss, a2, 1);
		ss1.push_back(ss);
		m.Trans(ss1, a2, -2);
		for (int i = 0; i < ss1.size(); i++) {
			SS.push_back(ss1[i]);
		}
		ss2 = ss1;
		m.Trans(ss1, a2, -2);
		for (int i = 0; i < ss1.size(); i++) {
			SS.push_back(ss1[i]);
		}

		Rectangle0 rt1(a2, b1 - a2 * 2);
		ss = rt1.getSurface();
		m.Trans(ss, M2 / 2 + 5 * a2 / 2, -1);
		m.Trans(ss, (b1 - a2 * 2) / 2, 2);
		SS.push_back(ss);
		SS3.push_back(ss);
		m.Trans(ss, a2, 1);
		SS.push_back(ss);
		SS3.push_back(ss);
		m.Trans(ss, a2, 1);
		SS.push_back(ss);
		SS3.push_back(ss);
		m.Trans(ss, a2 + M2, 1);
		SS.push_back(ss);
		SS3.push_back(ss);
		m.Trans(ss, a2, 1);
		SS.push_back(ss);
		SS3.push_back(ss);
		m.Trans(ss, a2, 1);
		SS.push_back(ss);
		SS3.push_back(ss);

		Rectangle0 rt2(M2, b1 - a2 * 2);
		ss = rt2.getSurface();
		m.Trans(ss, (b1 - a2 * 2) / 2, 2);
		SS.push_back(ss);
		SS3.push_back(ss);

		Rectangle0 rt3(M2, a2);
		ss = rt3.getSurface();
		m.Trans(ss, b1 - a2 * 2 + a2 / 2, 2);
		SS.push_back(ss);
		SplineSurface s;//后面用
		s = ss;

		SL1[0].m_CtrlPts.push_back(v1);
		SL1[0].m_CtrlPts.push_back((v1 + v4) / 2);
		SL1[0].m_CtrlPts.push_back(v4);
		SL1[1].m_CtrlPts.push_back(v1);
		SL1[1].m_CtrlPts.push_back((v1 + v2) / 2);
		SL1[1].m_CtrlPts.push_back(v2);
		SL1[2].m_CtrlPts.push_back(v2);
		SL1[2].m_CtrlPts.push_back((v2 + v3) / 2);
		SL1[2].m_CtrlPts.push_back(v3);
		SL1[3].m_CtrlPts.push_back(v4);
		SL1[3].m_CtrlPts.push_back((v4 + v3) / 2);
		SL1[3].m_CtrlPts.push_back(v3);
		SplineSurface ss3;
		ss3.CoonsInterpolate(SL1);
		SS.push_back(ss3);
		SS3.push_back(ss3);

		SL2[0].m_CtrlPts.push_back(v5);
		SL2[0].m_CtrlPts.push_back((v5 + v8) / 2);
		SL2[0].m_CtrlPts.push_back(v8);
		SL2[1].m_CtrlPts.push_back(v5);
		SL2[1].m_CtrlPts.push_back((v5 + v6) / 2);
		SL2[1].m_CtrlPts.push_back(v6);
		SL2[2].m_CtrlPts.push_back(v6);
		SL2[2].m_CtrlPts.push_back((v6 + v7) / 2);
		SL2[2].m_CtrlPts.push_back(v7);
		SL2[3].m_CtrlPts.push_back(v8);
		SL2[3].m_CtrlPts.push_back((v8 + v7) / 2);
		SL2[3].m_CtrlPts.push_back(v7);
		SplineSurface ss4;
		ss4.CoonsInterpolate(SL2);
		SS.push_back(ss4);
		SS3.push_back(ss4);

		//中间镂空部分
		varray<SplineSurface> SS1;
		SS1.push_back(ss2[1]);
		SS1.push_back(ss2[2]);
		m.Trans(SS1, a2, 2);
		m.Trans(s, a2, 2);
		SS1.push_back(s);
		m.Trans(s, a2, 2);
		SS1.push_back(s);

		Rectangle0 rt4(M3 / 3, M6 - a2);
		ss = rt4.getSurface();
		m.Trans(ss, b1 + a2 + (M6 - a2) / 2, 2);
		m.Trans(ss, M2 / 2 + a2 + M3 / 6, -1);
		SS1.push_back(ss);
		m.Trans(ss, 2 * (M2 / 2 + a2 + M3 / 6), 1);
		SS1.push_back(ss);

		Rectangle0 rt5(a2, M6 - a2);
		ss = rt5.getSurface();
		m.Trans(ss, b1 + a2 + (M6 - a2) / 2, 2);
		m.Trans(ss, M2 / 2 + a2 / 2, -1);
		SS1.push_back(ss);
		m.Trans(ss, 2 * (M2 / 2 + a2 / 2), 1);
		SS1.push_back(ss);

		Rectangle0 rt6(M2, M6 - a2);
		ss = rt6.getSurface();
		m.Trans(ss, b1 + a2 + (M6 - a2) / 2, 2);
		SS1.push_back(ss);

		//滑轨
		varray<SplineSurface> SS2;
		SplineSurface ss5;

		RandomModel rm(v17, v18, v19, v24);
		ss5 = rm.getSurface();
		SS2.push_back(ss5);
		RandomModel rm1(v19, v20, v23, v24);
		ss5 = rm1.getSurface();
		SS2.push_back(ss5);
		RandomModel rm2(v23, v20, v21, v22);
		ss5 = rm2.getSurface();
		SS2.push_back(ss5);
		RandomModel rm3(v9, v14, v15, v16);
		ss5 = rm3.getSurface();
		SS2.push_back(ss5);
		RandomModel rm4(v9, v10, v13, v14);
		ss5 = rm4.getSurface();
		SS2.push_back(ss5);
		RandomModel rm5(v11, v12, v13, v10);
		ss5 = rm5.getSurface();
		SS2.push_back(ss5);

		////底座补充部分 暂时不要
		//Vec4 p1 = { a1 / 2 + a1 / 10,0,0,1 };
		//Vec4 p2 = { a1 / 2 + a1 / 10,0,0,1 };
		//Vec4 p1 = { a1 / 2 + a1 / 10,0,0,1 };
		//Vec4 p1 = { a1 / 2 + a1 / 10,0,0,1 };
		//RandomModel();

		varray<SplineVolume> SV;
		varray<SplineVolume> SV1;
		SV = m.CreatSweepVol(SS, c2, 3);
		m.Trans(SS, c2, 3);
		SV1 = m.CreatSweepVol(SS, c2 * 3, 3);
		m.Trans(SS, c2 * 3, 3);
		for (int i = 0; i < SV1.size(); i++) {
			SV.push_back(SV1[i]);
		}
		SV1 = m.CreatSweepVol(SS, c2, 3);
		m.Trans(SS, c2, 3);
		for (int i = 0; i < SV1.size(); i++) {
			SV.push_back(SV1[i]);
		}
		SV1 = m.CreatSweepVol(SS, c1 * 3 / 4 - 10 * c2, 3);
		m.Trans(SS, c1 * 3 / 4 - 10 * c2, 3);
		for (int i = 0; i < SV1.size(); i++) {
			SV.push_back(SV1[i]);
		}
		SV1 = m.CreatSweepVol(SS, c2, 3);
		m.Trans(SS, c2, 3);
		for (int i = 0; i < SV1.size(); i++) {
			SV.push_back(SV1[i]);
		}
		SV1 = m.CreatSweepVol(SS, c2 * 3, 3);
		m.Trans(SS, c2 * 3, 3);
		for (int i = 0; i < SV1.size(); i++) {
			SV.push_back(SV1[i]);
		}
		SV1 = m.CreatSweepVol(SS, c2, 3);
		m.Trans(SS, c2, 3);
		for (int i = 0; i < SV1.size(); i++) {
			SV.push_back(SV1[i]);
		}

		SV1 = m.CreatSweepVol(SS, v, 3);//与立柱连接部分
		for (int i = 0; i < SV1.size(); i++) {
			SV.push_back(SV1[i]);
		}
		SV1 = m.CreatSweepVol(ss2, c1 / 10, -3);//伸出部分
		for (int i = 0; i < SV1.size(); i++) {
			SV.push_back(SV1[i]);
		}
		m.Trans(SS1, 3 * c1 / 4, 3);
		SV1 = m.CreatSweepVol(SS1, v, 3);//与立柱连接部分
		for (int i = 0; i < SV1.size(); i++) {
			SV.push_back(SV1[i]);
		}

		SV1 = m.CreatSweepVol(SS2, c2, 3);
		m.Trans(SS2, c2, 3);
		m.Trans(SV1, b1, 2);
		for (int i = 0; i < SV1.size(); i++) {
			SV.push_back(SV1[i]);
		}
		SV1 = m.CreatSweepVol(SS2, 3 * c2, 3);
		m.Trans(SS2, 3 * c2, 3);
		m.Trans(SV1, b1, 2);
		for (int i = 0; i < SV1.size(); i++) {
			SV.push_back(SV1[i]);
		}
		SV1 = m.CreatSweepVol(SS2, c2, 3);
		m.Trans(SS2, c2, 3);
		m.Trans(SV1, b1, 2);
		for (int i = 0; i < SV1.size(); i++) {
			SV.push_back(SV1[i]);
		}
		SV1 = m.CreatSweepVol(SS2, 3 * c1 / 4 - 10 * c2, 3);
		m.Trans(SS2, 3 * c1 / 4 - 10 * c2, 3);
		m.Trans(SV1, b1, 2);
		for (int i = 0; i < SV1.size(); i++) {
			SV.push_back(SV1[i]);
		}
		SV1 = m.CreatSweepVol(SS2, c2, 3);
		m.Trans(SS2, c2, 3);
		m.Trans(SV1, b1, 2);
		for (int i = 0; i < SV1.size(); i++) {
			SV.push_back(SV1[i]);
		}
		SV1 = m.CreatSweepVol(SS2, 3 * c2, 3);
		m.Trans(SS2, c2 * 3, 3);
		m.Trans(SV1, b1, 2);
		for (int i = 0; i < SV1.size(); i++) {
			SV.push_back(SV1[i]);
		}
		SV1 = m.CreatSweepVol(SS2, c2, 3);
		m.Trans(SS2, c2, 3);
		m.Trans(SV1, b1, 2);
		for (int i = 0; i < SV1.size(); i++) {
			SV.push_back(SV1[i]);
		}

		SV1 = machineTool4();
		for (int i = 0; i < SV1.size(); i++) {
			SV.push_back(SV1[i]);
		}

		rwg.WriteSplineSurface("E:\\kuang_models\\machineToolSurface.txt", SS);
		rwg.WriteSplineSurface("E:\\kuang_models\\machineToolSurface1.txt", SS1);
		rwg.WriteSplineSurface("E:\\kuang_models\\machineToolSurface2.txt", SS2);
		rwg.WriteSplineVolume("E:\\kuang_models\\machineToolVolume6.txt", SV);

		return SV;
	}

	//y轴床身
	varray<SplineVolume> getVolume1() {
		varray<SplineVolume> SV;
		varray<SplineVolume> SV1;
		varray<SplineVolume> SV2;
		SV1 = machineTool1();
		SV2 = machineTool2();
		m.Rolate(SV2, PI / 2, 2);
		m.Trans(SV2, H1 / 2, -2);
		m.Trans(SV2, M1 / 2, 3);
		m.Trans(SV2, L1 / 2, -1);
		for (int i = 0; i < SV1.size(); i++) {
			SV.push_back(SV1[i]);
		}
		for (int i = 0; i < SV2.size(); i++) {
			SV.push_back(SV2[i]);
		}

		rwg.WriteSplineVolume("E:\\kuang_models\\MachineTool1.txt", SV);

		return SV;
	}

	//y轴床身,底座和支撑件
	varray<SplineVolume> getVolume2() {
		varray<SplineVolume> SV;
		varray<SplineVolume> SV1;
		SV = getVolume1();
		m.Trans(SV, M1 / 2, -3);
		m.Rolate(SV, PI / 2, 2);
		m.Trans(SV, b1 + M6 + H1 / 2, 2);
		m.Trans(SV, c1 / 2, 3);
		SV1 = machineTool3();
		for (int i = 0; i < SV1.size(); i++) {
			SV.push_back(SV1[i]);
		}

		m.Rolate(SV, PI, 2);
		m.Trans(SV, b1 + M6 + w, -2);
		m.Trans(SV, c1 * 3 / 4, 3);
		m.Trans(SV, 11.0 / 2, 1);
		rwg.WriteSplineVolume("E:\\kuang_models\\machineTool2.txt", SV);

		return SV;
	}

	//y轴床身、底座和立柱
	varray<SplineVolume> getVolume3() {
		varray<SplineVolume> SV;
		varray<SplineVolume> SV1;
		MachineTool mt;
		SV = mt.getVolume2();

		rwg.ReadSplineVolume("E:\\kuang_models\\buningyuan\\MachineTool\\jichuang.txt", SV1);

		for (int i = 0; i < SV1.size(); i++) {
			SV.push_back(SV1[i]);
		}
		rwg.WriteSplineVolume("E:\\kuang_models\\machineTool4.txt", SV);

		rwg.ReadSplineVolume("E:\\kuang_models\\buningyuan\\MachineTool\\jichuang_model1.txt", SV1);
		//调整雅可比值
		for (auto& i : SV1) {
			i.OrderCtrlPts(i);
		}
		SV1[46].OrderCtrlPts(SV1[46]);
		for (int i = 59; i < 91; i++) {
			SV1[i].OrderCtrlPts(SV1[i]);
		}
		SV1[98].OrderCtrlPts(SV1[98]);
		for (int i = 332; i < 336; i++) {
			SV1[i].OrderCtrlPts(SV1[i]);
		}
		for (int i = 464; i < 479; i++) {
			SV1[i].OrderCtrlPts(SV1[i]);
		}
		for (int i = 423; i < 454; i++) {
			SV1[i].OrderCtrlPts(SV1[i]);
		}

		SV1[475].OrderCtrlPts(SV1[475]);
		SV1[484].OrderCtrlPts(SV1[484]);
		SV1[486].OrderCtrlPts(SV1[486]);
		SV1[487].OrderCtrlPts(SV1[487]);
		SV1[490].OrderCtrlPts(SV1[490]);
		SV1[491].OrderCtrlPts(SV1[491]);
		for (int i = 493; i < 499; i++) {
			SV1[i].OrderCtrlPts(SV1[i]);
		}
		for (int i = 499; i < 507; i++) {
			SV1[i].OrderCtrlPts(SV1[i]);
		}
		SV1[512].OrderCtrlPts(SV1[512]);
		SV1[513].OrderCtrlPts(SV1[513]);
		SV1[511].OrderCtrlPts(SV1[511]);
		SV1[510].OrderCtrlPts(SV1[510]);
		SV1[519].OrderCtrlPts(SV1[519]);
		SV1[518].OrderCtrlPts(SV1[518]);
		SV1[517].OrderCtrlPts(SV1[517]);
		SV1[521].OrderCtrlPts(SV1[521]);
		SV1[524].OrderCtrlPts(SV1[524]);
		SV1[526].OrderCtrlPts(SV1[526]);
		for (int i = 532; i < 536; i++) {
			SV1[i].OrderCtrlPts(SV1[i]);
		}
		for (int i = 548; i < 552; i++) {
			SV1[i].OrderCtrlPts(SV1[i]);
		}
		for (int i = 564; i < 568; i++) {
			SV1[i].OrderCtrlPts(SV1[i]);
		}
		for (int i = 575; i < SV1.size(); i++) {
			SV1[i].OrderCtrlPts(SV1[i]);
		}
		for (int i = 580; i < 584; i++) {
			SV1[i].OrderCtrlPts(SV1[i]);
		}
		/*
		for (int i = 580; i < 584; i++) {
			SV1[i].OrderCtrlPts(SV1[i]);
		}

		SV1[526].OrderCtrlPts(SV1[526]);

		SV1[481].OrderCtrlPts(SV1[481]);

		*/
		PublicSolution ps;
		ps.outPutVTK(SV1, "E:\\kuang_models\\machineTool.vtk");
		return SV;
	}

	//剖分出现问题模型，进行检查测试，y轴床身截面
	void testQuad() {
		//坐标点
		Vec4 v1 = { 0,0,0,1 };
		Vec4 v2 = { 0,H1 / 2,0,1 };
		Vec4 v3 = { L1 / 4,H1 / 2,0,1 };
		Vec4 v4 = { L1 / 4,H1 / 2 + H2,0,1 };
		Vec4 v5 = { L1 / 2 - L2 / 2 - L3 ,H1 / 2 + H2,0,1 };
		Vec4 v6 = { L1 / 2 - L2 / 2 - L3,H1,0,1 };
		Vec4 v7 = { L1 / 2 - L2 / 2,H1,0,1 };
		Vec4 v8 = { L1 / 2 - L2 / 2,H1 / 2,0,1 };
		Vec4 v9 = { L1 / 2 + L2 / 2 ,H1 / 2,0,1 };
		Vec4 v10 = { L1 / 2 + L2 / 2,H1,0,1 };
		Vec4 v11 = { L1 / 2 + L2 / 2 + L3,H1,0,1 };
		Vec4 v12 = { L1 / 2 + L2 / 2 + L3,H1 - (H1 / 2 - H2),0,1 };
		Vec4 v13 = { L1 / 2 + L1 / 4,H1 - (H1 / 2 - H2),0,1 };
		Vec4 v14 = { L1 / 2 + L1 / 4,H1 / 2,0,1 };
		Vec4 v15 = { L1,H1 / 2,0,1 };
		Vec4 v16 = { L1,0,0,1 };

		varray<Spline> sps;
		Spline0 sl;
		Spline sp;
		sp = sl.getSpline(v1, v2);
		sps.push_back(sp);

		sp = sl.getSpline(v2, v3);
		sps.push_back(sp);

		sp = sl.getSpline(v3, v4);
		sps.push_back(sp);

		sp = sl.getSpline(v4, v5);
		sps.push_back(sp);

		sp = sl.getSpline(v5, v6);
		sps.push_back(sp);

		sp = sl.getSpline(v6, v7);
		sps.push_back(sp);

		sp = sl.getSpline(v7, v8);
		sps.push_back(sp);

		sp = sl.getSpline(v8, v9);
		sps.push_back(sp);

		sp = sl.getSpline(v9, v10);
		sps.push_back(sp);

		sp = sl.getSpline(v10, v11);
		sps.push_back(sp);

		sp = sl.getSpline(v11, v12);
		sps.push_back(sp);

		sp = sl.getSpline(v12, v13);
		sps.push_back(sp);

		sp = sl.getSpline(v13, v14);
		sps.push_back(sp);

		sp = sl.getSpline(v14, v15);
		sps.push_back(sp);

		sp = sl.getSpline(v15, v16);
		sps.push_back(sp);

		sp = sl.getSpline(v16, v1);
		sps.push_back(sp);

		RWGeometric rwg;
		rwg.WriteSpline("E:\\kuang_models\\outSpline.txt", sps);

		varray<Spline>outer;//存放外轮廓曲线
		varray<Spline>inner1;//存放内轮廓曲线
		varray<varray<Spline>> inner;
		varray<Spline>addlines, allLines;//辅助线 将内外轮廓连接起来变为零亏格
		varray<bool> genus;
		varray<SplineSurface> allSurf;//存放剖分结果

		rwg.ReadSpline("E:\\kuang_models\\outSpline.txt", outer);
		genus.resize(1);
		genus[0] = false;//我之前用的是push_back,会出现问题
		PublicSolution ps;
		ps.quad(outer, inner, addlines, genus, allSurf);

		rwg.WriteSplineSurface("E:\\kuang_models\\newSurface.txt", allSurf);
	}
};

class Part {
public:
	Model_Solution m;
	PublicSolution ps;
	RWGeometric rwg;

	double m_r = 0.25;
	double m_s1 = 5;
	double m_s2 = 0.6;
	double m_s3 = 1;

	double mGao = 0.05;
	double mGao1 = 0.025;
	double mLenth = 0.1476;
	double mLen1 = 0.0138;
	double mLen2 = 0.046;
	double mR = 0.0082;

	double mLenth1 = 0.1;
	double mLenth2 = 0.05;
	double mLenth3 = mLenth2 * 2 / 5;

	double w = cos(PI / 4);

	//自动剖分
	void maoPartOne() {
		Vec4 v1 = { 0,m_r,0,1 };
		Vec4 v2 = { 0,3 * m_s2 + m_r,0,1 };
		Vec4 v3 = { m_s1,3 * m_s2 + m_r,0,1 };
		Vec4 v4 = { m_s1,m_r,0,1 };
		Vec4 v5 = { m_s3 * 2,3 * m_s2 + m_r,0,1 };
		Vec4 v6 = { m_s3 * 4,3 * m_s2 + m_r,0,1 };
		Vec4 v7 = { m_s3 * 3,m_r,0,1 };
		Vec4 v8 = { m_s3,m_r,0,1 };
		Vec4 v9 = { m_s3 * 3,3 * m_s2 + m_r,0,1 };

		Vec4 p1 = { m_s3,m_s2,0,1 };
		Vec4 p2 = { m_s3 * 2,2 * m_s2 + m_r + m_r,0,1 };
		Vec4 p3 = { m_s3 * 3,m_s2,0,1 };
		Vec4 p4 = { m_s3 * 4,2 * m_s2 + m_r + m_r,0,1 };
		Vec4 p5 = { m_s3 * 2,2 * m_s2,0,1 };
		Vec4 p6 = { m_s3 + m_r,m_s2 + m_r,0,1 };
		Vec4 p7 = { m_s3 * 3,m_s2 + m_r + m_r,0,1 };

		varray<Spline> outSpline;
		varray<Spline> subSpline;
		varray<varray<Spline>> inSpline;
		varray<Spline> addSpline;
		varray<SplineSurface> allSurf;
		varray<Spline> allSpline;

		Spline0 sl(v1, v2);
		outSpline.push_back(sl.getSpline());

		Spline0 sl1(v2, v5);
		outSpline.push_back(sl1.getSpline());

		Spline0 sl2(v5, v9);
		outSpline.push_back(sl2.getSpline());
		Spline0 SL5(v9, v6);
		outSpline.push_back(SL5.getSpline());

		Spline0 sl3(v6, v3);
		outSpline.push_back(sl3.getSpline());

		Spline0 SL(v3, v4);
		outSpline.push_back(SL.getSpline());

		Spline0 SL1(v4, v7);
		outSpline.push_back(SL1.getSpline());

		Spline0 SL2(v7, v8);
		outSpline.push_back(SL2.getSpline());

		Spline0 SL4(v8, v1);
		outSpline.push_back(SL4.getSpline());

		Spline0 sl4;
		subSpline = sl4.arcSplines(m_r);
		m.Trans(subSpline, m_s3, 1);
		m.Trans(subSpline, m_s2 + m_r, 2);
		inSpline.push_back(subSpline);

		m.Trans(subSpline, m_s3, 1);
		m.Trans(subSpline, m_s2, 2);
		inSpline.push_back(subSpline);

		m.Trans(subSpline, m_s3, 1);
		m.Trans(subSpline, m_s2, -2);
		inSpline.push_back(subSpline);

		m.Trans(subSpline, m_s3, 1);
		m.Trans(subSpline, m_s2, 2);
		inSpline.push_back(subSpline);

		Spline0 sl5(p1, v8);
		addSpline.push_back(sl5.getSpline());
		Spline0 sl6(p2, v5);
		addSpline.push_back(sl6.getSpline());
		Spline0 sl7(p3, v7);
		addSpline.push_back(sl7.getSpline());
		Spline0 sl8(p4, v6);
		addSpline.push_back(sl8.getSpline());
		Spline0 sl9(p5, p6);
		addSpline.push_back(sl9.getSpline());
		Spline0 sl10(p7, v9);
		addSpline.push_back(sl10.getSpline());

		for (auto& i : outSpline) {
			allSpline.push_back(i);
		}
		for (auto& i : inSpline) {
			for (auto& j : i) {
				allSpline.push_back(j);
			}
		}
		for (auto& i : addSpline) {
			allSpline.push_back(i);
		}

		varray<bool> genus;
		genus.resize(5);
		genus[0] = false;
		genus[1] = true;
		genus[2] = true;
		genus[3] = true;
		genus[4] = true;
		rwg.WriteSpline("E:\\kuang_models\\maoSplines.txt", allSpline);
		ps.quad(outSpline, inSpline, addSpline, genus, allSurf);
		varray<SplineVolume> SV;
		SV = m.CreatSweepVol(allSurf, 1, 3);
		ps.outPutTXT(allSurf, "E:\\kuang_models\\maoSurf");

		rwg.WriteSplineSurface("E:\\kuang_models\\maoSurfaces.txt", allSurf);
		rwg.WriteSplineVolume("E:\\kuang_models\\maoSPlineVolume.txt", SV);
		ps.outPutVTK(SV, "E:\\kuang_models\\maoSPlineVolume.vtk");
	}

	Spline maoSpline(Vec4& v1, Vec4& v2) {
		Spline SL;
		varray<double> knots;
		knots.push_back(0);
		knots.push_back(0);
		knots.push_back(0);
		knots.push_back(0.5);
		knots.push_back(0.5);
		knots.push_back(1);
		knots.push_back(1);
		knots.push_back(1);
		SL.m_Knots = knots;
		SL.m_Degree = 2;
		Vec4 p1 = v2 - v1;

		SL.m_CtrlPts.push_back(v1);
		SL.m_CtrlPts.push_back(v1 + p1 / 4);
		SL.m_CtrlPts.push_back(v1 + 2 * p1 / 4);
		SL.m_CtrlPts.push_back(v1 + 3 * p1 / 4);
		SL.m_CtrlPts.push_back(v2);
		for (int i = 1; i < SL.m_CtrlPts.size() - 1; i++) {
			SL.m_CtrlPts[i].w = 1;
		}
		return SL;
	}

	void maoPartTwo() {
		Vec4 v1 = { 0,0,0,1 };
		Vec4 v2 = { mLen1,0,0,1 };
		Vec4 v3 = { 0,mGao,0,1 };
		Vec4 v4 = { mLen1 + mLen2 / 2 - mR,mGao,0,1 };

		Vec4 v5 = { mLen1 + mLen2 / 2 - mR,mGao - mR,0,w };
		Vec4 v6 = { mLen1 + mLen2 / 2,mGao - mR,0,1 };
		Vec4 v7 = { mLen1 + mLen2 / 2 + mR,mGao - mR,0,w };
		Vec4 v8 = { mLen1 + mLen2 / 2 + mR,mGao,0,1 };
		Vec4 v9 = { mLen1 + mLen2,0,0,1 };
		Vec4 v11 = { -mGao,0,0,1 };
		Vec4 v12 = { -mGao,mGao,0,1 };

		varray<SplineSurface> SS;
		SplineSurface ss, temp;
		varray<Spline> SL;
		Spline sl, sl1, sl2;
		sl = maoSpline(v1, v2);
		SL.push_back(sl);
		sl = maoSpline(v1, v3);
		SL.push_back(sl);
		sl = maoSpline(v3, v4);
		SL.push_back(sl);
		sl = maoSpline(v2, v4);
		SL.push_back(sl);
		ss.CoonsInterpolate(SL);
		SS.push_back(ss);

		m.Rolate(ss, PI, 3);
		m.Trans(ss, mGao, 2);
		m.Trans(ss, mLen1 * 2 + mLen2 / 2 - mR + mLen2 * 2 + 4 * mR, 1);
		SS.push_back(ss);

		SL.clear();
		sl1 = maoSpline(v2, v9);
		SL.push_back(sl1);
		sl1 = maoSpline(v2, v4);
		SL.push_back(sl1);

		varray<double> knots;
		knots.push_back(0);
		knots.push_back(0);
		knots.push_back(0);
		knots.push_back(0.5);
		knots.push_back(0.5);
		knots.push_back(1);
		knots.push_back(1);
		knots.push_back(1);
		sl2.m_Knots = knots;
		sl2.m_Degree = 2;

		sl2.m_CtrlPts.push_back(v4);
		sl2.m_CtrlPts.push_back(v5);
		sl2.m_CtrlPts.push_back(v6);
		sl2.m_CtrlPts.push_back(v7);
		sl2.m_CtrlPts.push_back(v8);
		SL.push_back(sl2);

		sl1 = maoSpline(v9, v8);
		SL.push_back(sl1);

		ss.CoonsInterpolate(SL);
		SS.push_back(ss);
		m.Trans(ss, mLen2 + mR * 2, 1);
		SS.push_back(ss);
		temp = ps.mirror(ss, 1);
		m.Trans(temp, mGao, 2);
		m.Trans(temp, mLen2 / 2 + mR, 1);
		SS.push_back(temp);
		m.Trans(temp, -2 * (mLen2 / 2 + mR), 1);
		SS.push_back(temp);

		SL.clear();
		sl = maoSpline(v11, v1);
		SL.push_back(sl);
		sl = maoSpline(v11, v12);
		SL.push_back(sl);
		sl = maoSpline(v12, v3);
		SL.push_back(sl);
		sl = maoSpline(v1, v3);
		SL.push_back(sl);
		ss.CoonsInterpolate(SL);
		SS.push_back(ss);

		m.Trans(ss, mGao, 2);
		SS.push_back(ss);
		m.Trans(ss, 2 * mGao, -2);
		SS.push_back(ss);
		m.Trans(SS, mGao, 1);
		m.Trans(SS, mGao, 2);

		ps.quadAdjustUV(SS);
		for (auto& i : SS) {
			i.KnotsRefineNum(3);
		}
		rwg.WriteSplineSurface("E:\\kuang_models\\maoSurface1.txt", SS);

		ps.outPutTXT(SS, "E:\\kuang_models\\maoSurf");
	}
	void maoPartThree() {
		Vec4 v1 = { 0,0,0,1 };
		Vec4 v2 = { mLen1,0,0,1 };
		Vec4 v3 = { 0,mGao,0,1 };
		Vec4 v4 = { mLen1 + mLen2 / 2 - mR,mGao,0,1 };

		Vec4 v5 = { mLen1 + mLen2 / 2 - mR,mGao - mR,0,w };
		Vec4 v6 = { mLen1 + mLen2 / 2,mGao - mR,0,1 };
		Vec4 v7 = { mLen1 + mLen2 / 2 + mR,mGao - mR,0,w };
		Vec4 v8 = { mLen1 + mLen2 / 2 + mR,mGao,0,1 };
		Vec4 v9 = { mLen1 + mLen2,0,0,1 };
		Vec4 v10 = { mLen1 + mLen2 / 2,0,0,1 };

		varray<SplineSurface> SS, SS1, temp;
		SplineSurface ss;
		varray<Spline> SL;

		Spline sl;
		Spline0 sp(v1, v2);
		sl = sp.getSpline();
		SL.push_back(sl);
		Spline0 sp1(v1, v3);
		sl = sp1.getSpline();
		SL.push_back(sl);
		Spline0 sp2(v3, v4);
		sl = sp2.getSpline();
		SL.push_back(sl);
		Spline0 sp3(v2, v4);
		sl = sp3.getSpline();
		SL.push_back(sl);

		ss.CoonsInterpolate(SL);
		SS.push_back(ss);

		m.Rolate(ss, PI, 3);
		m.Trans(ss, mGao, 2);
		m.Trans(ss, mLen1 * 2 + mLen2 / 2 - mR + mLen2 * 2 + 4 * mR, 1);
		SS.push_back(ss);

		SL.clear();
		Spline0 sp4(v2, v10);
		sl = sp4.getSpline();
		SL.push_back(sl);
		Spline0 sp5(v2, v4);
		sl = sp5.getSpline();
		SL.push_back(sl);
		Spline0 sp6;
		sl = sp6.getArcSpline(v4, v5, v6);
		SL.push_back(sl);
		Spline0 sp7(v10, v6);
		sl = sp7.getSpline();
		SL.push_back(sl);
		ss.CoonsInterpolate(SL);
		SS.push_back(ss);
		SS1.push_back(ss);

		SL.clear();
		Spline0 sp8(v10, v9);
		sl = sp8.getSpline();
		SL.push_back(sl);
		Spline0 sp9(v10, v6);
		sl = sp9.getSpline();
		SL.push_back(sl);
		Spline0 sp10;
		sl = sp10.getArcSpline(v6, v7, v8);
		SL.push_back(sl);
		Spline0 sp11(v9, v8);
		sl = sp11.getSpline();
		SL.push_back(sl);
		ss.CoonsInterpolate(SL);
		SS1.push_back(ss);
		SS.push_back(ss);

		RandomModel  rm(mR);
		temp = rm.getArcSurface();
		m.Trans(temp, mGao, 2);
		m.Trans(temp, mLen2 / 2 + mLen1, 1);
		SS1.push_back(temp[1]);
		SS1.push_back(temp[2]);
		SS.push_back(temp[1]);
		SS.push_back(temp[2]);

		m.Trans(SS1, mLen2 + mR * 2, 1);
		for (auto& i : SS1) {
			SS.push_back(i);
		}
		temp = ps.mirror(SS1, 1);
		m.Trans(temp, mGao, 2);
		m.Trans(temp, mLen2 / 2 + mR, 1);
		for (auto& i : temp) {
			SS.push_back(i);
		}
		m.Trans(temp, -2 * (mLen2 / 2 + mR), 1);
		for (auto& i : temp) {
			SS.push_back(i);
		}

		ps.quadAdjustUV(SS);
		for (auto& i : SS) {
			i.KnotsRefineNum(3);
		}
		rwg.WriteSplineSurface("E:\\kuang_models\\maoSurface1.txt", SS);

		ps.outPutTXT(SS, "E:\\kuang_models\\maoSurf");
	}

	void maoPartFour() {
		Vec4 v1 = { 0,0,0,1 };
		Vec4 v2 = { 0,mGao,0,1 };
		Vec4 v3 = { mLenth1,mGao,0,1 };
		Vec4 v4 = { mLenth2,0,0,1 };

		Vec4 v5 = { mLenth1,mLenth3,0,1 };
		Vec4 v6 = { mLenth1 - mLenth3,0,0,1 };
		Vec4 v7 = { mLenth1 + mLenth3,0,0,1 };
		Vec4 v8 = { mLenth1 + mLenth2,0,0,1 };
		Vec4 v9 = { mLenth1 + mLenth2,mGao,0,1 };
		Vec4 v10 = { mLenth1 * 2,0,0,1 };
		Vec4 v11 = { mLenth1 * 2 - mLenth3,mGao,0,1 };
		Vec4 v12 = { mLenth1 * 2,mGao - mLenth3,0,1 };

		Vec4 v13 = { -mLenth2,0,0,1 };
		Vec4 v14 = { -mLenth2,mGao,0,1 };
		Vec4 v15 = { mLenth1,0,0,1 };
		Vec4 v16 = { mLenth1,-mLenth3,0,1 };

		Vec4 v17 = { -mLenth3,0,0,1 };
		Vec4 v18 = { mLenth3,0,0,1 };
		Vec4 v19 = { 0,mLenth3,0,1 };
		Vec4 v20 = { -mLenth3 / 4,mLenth3 / 2 + mLenth3 / 4,0,1 };
		Vec4 v21 = { mLenth3 / 4,mLenth3 / 2 + mLenth3 / 4,0,1 };
		Vec4 v22 = { 0,mLenth3 / 2,0,1 };

		//change
		Vec4 v23 = { mLenth1,-mGao,0,1 };
		Vec4 v24 = { mLenth1 * 2,-mGao,0,1 };

		Vec4 p1 = { -mGao,0,0,1 };
		Vec4 p2 = { 0,0,0,1 };
		Vec4 p3 = { -mGao,mGao1,0,1 };
		Vec4 p4 = { 0,mGao1,0,1 };
		Vec4 p5 = { mLenth1,mGao1,0,1 };
		Vec4 p6 = { mLenth1,0,0,1 };

		varray<SplineSurface> SS, tmp, SS1, SS2, SS3, SS4;
		varray<SplineSurface> SS5, SS6;
		SplineSurface ss, temp;
		varray<Spline> SL;
		Spline sl, sl1, sl2;
		sl = maoSpline(v1, v4);
		SL.push_back(sl);
		sl = maoSpline(v1, v2);
		SL.push_back(sl);
		sl = maoSpline(v2, v3);
		SL.push_back(sl);
		sl = maoSpline(v4, v3);
		SL.push_back(sl);
		ss.CoonsInterpolate(SL);
		SS.push_back(ss);

		//复用
		SS1.push_back(ss);

		SL.clear();
		sl = maoSpline(v4, v6);
		SL.push_back(sl);
		sl = maoSpline(v4, v3);
		SL.push_back(sl);
		sl = maoSpline(v3, v5);
		SL.push_back(sl);
		sl = maoSpline(v6, v5);
		SL.push_back(sl);
		ss.CoonsInterpolate(SL);
		SS.push_back(ss);
		//复用
		SS1.push_back(ss);

		SL.clear();
		sl = maoSpline(v5, v7);
		SL.push_back(sl);
		sl = maoSpline(v5, v3);
		SL.push_back(sl);
		sl = maoSpline(v3, v8);
		SL.push_back(sl);
		sl = maoSpline(v7, v8);
		SL.push_back(sl);
		ss.CoonsInterpolate(SL);
		SS.push_back(ss);

		SL.clear();
		sl = maoSpline(v8, v10);
		SL.push_back(sl);
		sl = maoSpline(v8, v3);
		SL.push_back(sl);
		sl = maoSpline(v3, v9);
		SL.push_back(sl);
		sl = maoSpline(v10, v9);
		SL.push_back(sl);
		ss.CoonsInterpolate(SL);
		SS.push_back(ss);

		SL.clear();
		sl = maoSpline(v10, v12);
		SL.push_back(sl);
		sl = maoSpline(v10, v9);
		SL.push_back(sl);
		sl = maoSpline(v9, v11);
		SL.push_back(sl);
		sl = maoSpline(v12, v11);
		SL.push_back(sl);
		ss.CoonsInterpolate(SL);
		SS.push_back(ss);

		//黄色小方块
		SL.clear();
		sl = maoSpline(v13, v1);
		SL.push_back(sl);
		sl = maoSpline(v13, v14);
		SL.push_back(sl);
		sl = maoSpline(v14, v2);
		SL.push_back(sl);
		sl = maoSpline(v1, v2);
		SL.push_back(sl);
		ss.CoonsInterpolate(SL);
		SS.push_back(ss);
		SS2.push_back(ss);

		m.Trans(SS, mLenth1 * 2, -1);

		tmp.clear();
		tmp = ps.mirror(SS, 2);
		for (auto& i : tmp) {
			SS.push_back(i);
		}
		tmp.clear();
		tmp = ps.mirror(SS, 1);
		for (auto& i : tmp) {
			SS.push_back(i);
		}
		m.Trans(SS, mLenth1 * 2, 1);
		m.Trans(SS, mGao, 1);

		m.Trans(SS1, mLenth1, -1);
		tmp.clear();
		tmp = ps.mirror(SS1, 2);
		for (auto& i : tmp) {
			SS1.push_back(i);
		}
		m.Trans(SS1, mLenth1 * 2 + mGao, 1);
		m.Trans(SS1, mGao, 2);
		for (auto& i : SS1) {
			SS.push_back(i);
		}

		SL.clear();
		sl = maoSpline(v1, v15);
		SL.push_back(sl);
		sl = maoSpline(v1, v2);
		SL.push_back(sl);
		sl = maoSpline(v2, v3);
		SL.push_back(sl);
		sl = maoSpline(v15, v3);
		SL.push_back(sl);
		ss.CoonsInterpolate(SL);
		SS2.push_back(ss);

		m.Trans(SS2, mLenth1, -1);
		tmp = ps.mirror(SS2, 2);
		m.Trans(SS2, mLenth1 + mGao, 1);
		m.Trans(tmp, mLenth1 * 4 - mGao, 1);
		for (auto& i : tmp) {
			SS2.push_back(i);
		}
		m.Trans(SS2, mGao, 2);
		for (auto& i : SS2) {
			SS.push_back(i);
		}

		//尺寸改变 最上层
		SL.clear();
		sl = maoSpline(p1, p2);
		SL.push_back(sl);
		sl = maoSpline(p1, p3);
		SL.push_back(sl);
		sl = maoSpline(p3, p4);
		SL.push_back(sl);
		sl = maoSpline(p2, p4);
		SL.push_back(sl);
		ss.CoonsInterpolate(SL);
		SS5.push_back(ss);

		SL.clear();
		sl = maoSpline(p2, p6);
		SL.push_back(sl);
		sl = maoSpline(p2, p4);
		SL.push_back(sl);
		sl = maoSpline(p4, p5);
		SL.push_back(sl);
		sl = maoSpline(p6, p5);
		SL.push_back(sl);
		ss.CoonsInterpolate(SL);
		temp = ss;
		SS5.push_back(ss);
		m.Trans(temp, mLenth1, 1);
		SS5.push_back(temp);

		m.Trans(SS5, mLenth1 * 2, -1);
		tmp = ps.mirror(SS5, 2);
		for (auto& i : tmp) {
			SS5.push_back(i);
		}

		SS6 = ps.mirror(SS5, 1);
		m.Trans(SS5, mGao * 2, 2);
		m.Trans(SS5, mLenth1 * 2 + mGao, 1);
		for (auto& i : SS5) {
			SS.push_back(i);
		}

		//菱形
		SL.clear();
		sl = maoSpline(v6, v16);
		SL.push_back(sl);
		sl = maoSpline(v6, v5);
		SL.push_back(sl);
		sl = maoSpline(v5, v7);
		SL.push_back(sl);
		sl = maoSpline(v16, v7);
		SL.push_back(sl);
		ss.CoonsInterpolate(SL);
		m.Trans(ss, mGao, 1);
		SS3.push_back(ss);
		m.Trans(ss, mLenth1 * 2, 1);
		SS3.push_back(ss);
		m.Trans(ss, mLenth1, -1);
		m.Trans(ss, mGao, 2);
		SS3.push_back(ss);
		for (auto& i : SS3) {
			SS.push_back(i);
		}

		//删除三角形后续操作
		SS.erase(SS.begin() + 15);
		SS.erase(SS.begin() + 15);
		SS.erase(SS.begin() + 19);
		SS.erase(SS.begin() + 19);

		SL.clear();
		sl = maoSpline(v23, v24);
		SL.push_back(sl);
		sl = maoSpline(v23, v8);
		SL.push_back(sl);
		sl = maoSpline(v8, v10);
		SL.push_back(sl);
		sl = maoSpline(v24, v10);
		SL.push_back(sl);
		ss.CoonsInterpolate(SL);

		m.Trans(ss, mLenth1 * 2, -1);
		temp = ps.mirror(ss, 2);
		m.Trans(ss, mLenth1 * 2 + mGao, 1);
		SS.push_back(ss);
		m.Trans(temp, mLenth1 * 2 + mGao, 1);
		SS.push_back(temp);

		m.Trans(SS6, mLenth1 * 2 + mGao, 1);
		m.Trans(SS6, mGao, -2);
		for (auto& i : SS6) {
			SS.push_back(i);
		}
		tmp = SS6;
		m.Trans(tmp, mGao1, -2);
		for (auto& i : tmp) {
			SS.push_back(i);
		}
		m.Trans(tmp, mGao1 * 3 + mGao * 3, 2);
		for (auto& i : tmp) {
			SS.push_back(i);
		}

		m.Trans(SS, mGao * 2, 2);

		ps.quadAdjustUV(SS);
		for (auto& i : SS) {
			i.KnotsRefineNum(3);
		}

		//调整雅可比
		for (auto& i : SS) {
			i.OrderCtrlPts(i);
		}

		varray<SplineVolume> SV;
		SV = m.CreatSweepVol(SS, 1, 3);

		rwg.WriteSplineVolume("E:\\kuang_models\\maoSplineVolume.txt", SV);
		rwg.WriteSplineSurface("E:\\kuang_models\\maoSurface1.txt", SS);
		ps.outPutVTK(SV, "E:\\kuang_models\\maoSplineVolume.vtk");

		ps.outPutTXT(SS, "E:\\kuang_models\\maoSurf");
	}
};

//机械臂模型
class Arm {
public:
	Model_Solution m;
	PublicSolution ps;
	RWGeometric rwg;
	double w = cos(PI * 45 / 180);
	//参数
	double arm_r1;//底座圆半径
	double armR1 = 10;//下表面圆半径
	double armR2 = 8; //上表面圆半径
	double armH = 12;//底座高度

	//计算底座圆周上控制点
	void calPoints(varray<Vec4>& v)
	{
		cout << endl;
		cout << "半径为：" << arm_r1 << endl;
		//存放底座圆周上八个控制点
		Vec4 v1 = { -cos(30 * PI / 180) * arm_r1,sin(30 * PI / 180) * arm_r1,0,1 };
		v.push_back(v1);
		v1 = ps.rolate(v1, -30 * PI / 180, 3);
		v.push_back(v1);
		v1 = ps.rolate(v1, -60 * PI / 180, 3);
		v.push_back(v1);
		v1 = ps.rolate(v1, -30 * PI / 180, 3);
		v.push_back(v1);

		v1 = ps.rolate(v1, -60 * PI / 180, 3);
		v.push_back(v1);
		v1 = ps.rolate(v1, -30 * PI / 180, 3);
		v.push_back(v1);

		v1 = ps.rolate(v1, -60 * PI / 180, 3);
		v.push_back(v1);
		v1 = ps.rolate(v1, -30 * PI / 180, 3);
		v.push_back(v1);

		//正方形控制点
		double rec_l = 4;//正方形边长

		Vec4 v2 = { -rec_l / 2,-rec_l / 2,0,1 };
		v.push_back(v2);
		v2 = ps.rolate(v2, -90 * PI / 180, 3);
		v.push_back(v2);
		v2 = ps.rolate(v2, -90 * PI / 180, 3);
		v.push_back(v2);
		v2 = ps.rolate(v2, -90 * PI / 180, 3);
		v.push_back(v2);
	}

	//底座-圆部分
	void creatSpline(varray<Spline>& SLs) {
		varray<Vec4> v;
		Spline SL;
		calPoints(v);
		Spline0 sl;

		SL = sl.getArcSpline(arm_r1, PI / 6, v[0], v[1]);
		SLs.push_back(SL);
		Spline0 sl1(v[1], v[2]);
		SL = sl1.getSpline();
		SLs.push_back(SL);
		Spline0 sl2;
		SL = sl2.getArcSpline(arm_r1, PI / 6, v[2], v[3]);
		SLs.push_back(SL);
		Spline0 adsl1(v[0], v[3]);
		SL = adsl1.getSpline();
		SLs.push_back(SL);

		Spline0 sl3;
		SL = sl3.getArcSpline(arm_r1, PI / 3, v[3], v[4]);
		SLs.push_back(SL);
		Spline0 adsl2(v[11], v[4]);
		SL = adsl2.getSpline();
		SLs.push_back(SL);
		Spline0 adsl3(v[11], v[10]);
		SL = adsl3.getSpline();
		SLs.push_back(SL);
		Spline0 adsl4(v[10], v[3]);
		SL = adsl4.getSpline();
		SLs.push_back(SL);

		Spline0 sl4;
		SL = sl4.getArcSpline(arm_r1, PI / 6, v[4], v[5]);
		SLs.push_back(SL);
		Spline0 sl5;
		SL = sl5.getArcSpline(arm_r1, PI / 3, v[5], v[6]);
		SLs.push_back(SL);
		Spline0 sl6;
		SL = sl6.getArcSpline(arm_r1, PI / 6, v[6], v[7]);
		SLs.push_back(SL);
		Spline0 adsl5(v[7], v[4]);
		SL = adsl5.getSpline();
		SLs.push_back(SL);

		Spline0 sl7;
		SL = sl7.getSpline(v[7], v[8]);
		SLs.push_back(SL);
		Spline0 adsl6;
		SL = adsl6.getArcSpline(arm_r1, PI / 3, v[7], v[0]);
		SLs.push_back(SL);
		Spline0 adsl7(v[0], v[9]);
		SL = adsl7.getSpline();
		SLs.push_back(SL);
		Spline0 adsl8(v[8], v[9]);
		SL = adsl8.getSpline();
		SLs.push_back(SL);

		Spline0 sl8;
		SL = sl8.getSpline(v[9], v[10]);
		SLs.push_back(SL);
		Spline0 sl9;
		SL = sl9.getSpline(v[9], v[0]);
		SLs.push_back(SL);
		Spline0 sl10;
		SL = sl10.getSpline(v[0], v[3]);
		SLs.push_back(SL);
		Spline0 sl11;
		SL = sl11.getSpline(v[10], v[3]);
		SLs.push_back(SL);

		Spline0 sl12;
		SL = sl12.getSpline(v[8], v[9]);
		SLs.push_back(SL);
		Spline0 sl13;
		SL = sl13.getSpline(v[9], v[10]);
		SLs.push_back(SL);
		Spline0 sl14;
		SL = sl14.getSpline(v[10], v[11]);
		SLs.push_back(SL);
		Spline0 sl15;
		SL = sl15.getSpline(v[11], v[8]);
		SLs.push_back(SL);

		Spline0 sl16;
		SL = sl16.getSpline(v[7], v[8]);
		SLs.push_back(SL);
		Spline0 sl17;
		SL = sl17.getSpline(v[7], v[4]);
		SLs.push_back(SL);
		Spline0 sl18;
		SL = sl18.getSpline(v[11], v[8]);
		SLs.push_back(SL);
		Spline0 sl19;
		SL = sl19.getSpline(v[11], v[4]);
		SLs.push_back(SL);
	}
	//底座-方形部分
	void creatRec(varray<Spline>& sls) {
		varray<varray<Spline>> sl;
		varray<Spline> SL;
		double a = arm_r1 * sin(PI / 6) * 2;
		double b = a / 4;

		Spline0 sl1;
		SL = sl1.recSplines(a, b);
		sl.push_back(SL);

		Spline0 sl2;
		SL = sl2.recSplines(a, b);
		m.Trans(SL, b, 2);
		sl.push_back(SL);

		Spline0 sl3;
		SL = sl3.recSplines(b, b);
		m.Trans(SL, b, 2);
		m.Trans(SL, b, -1);
		sl.push_back(SL);

		Spline0 sl4;
		SL = sl4.recSplines(b, b);
		m.Trans(SL, b, 2);
		m.Trans(SL, a, 1);
		sl.push_back(SL);

		varray<Spline> tmp;
		for (auto i : sl) {
			for (auto j : i) {
				tmp.push_back(j);
			}
		}
		m.Trans(tmp, a / 2, -1);
		m.Trans(tmp, arm_r1 * cos(PI / 6), 2);
		for (auto i : tmp) {
			sls.push_back(i);
		}
	}

	void part() {
		varray<Spline> SL;
		varray<varray<Spline>> SL1;
		varray<SplineSurface> SS, SS1;//下表面、上表面
		varray<SplineVolume> SV, temp;
		double distance = armR1;
		arm_r1 = armR1;//上表面半径
		SL.clear();
		creatSpline(SL);
		creatRec(SL);
		ps.splineCreatSurf(SL, SS);

		varray<SplineSurface> tmp;
		tmp = SS;
		m.Trans(SS, armH / 10, 3);
		SV = ps.loft(tmp, SS);

		arm_r1 = armR2;//下表面半径
		SL.clear();
		creatSpline(SL);
		creatRec(SL);
		ps.splineCreatSurf(SL, SS1);
		m.Trans(SS1, armH, 3);

		temp = ps.loft(SS, SS1);
		for (auto i : temp) {
			SV.push_back(i);
		}

		//rwg.WriteSplineSurface("E:\\kuang_models\\aSurf.txt", SS);
		rwg.WriteSplineVolume("E:\\kuang_models\\aSV.txt", SV);
	}
};




