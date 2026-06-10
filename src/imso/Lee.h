#pragma once
#include "FeatureNetwork.h"

#include "SplineVolume.h"
#include "NurbsTrans.h"
#include "Option.h"
#include <math.h>
#include"PublicModels.h"
#include <ctime>
#include "PublicModels.h"

#include <CGAL/Constrained_Delaunay_triangulation_2.h>
#include <CGAL/Simple_cartesian.h> //笛卡尔坐标相关头文件

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Delaunay_triangulation_2.h>


// 定义使用的Kernel类型，这里使用的是具有精确构造但在计算中允许轻微浮点误差的内核
typedef CGAL::Exact_predicates_inexact_constructions_kernel K;

// 使用定义的Kernel类型创建2D Delaunay三角剖分类型
typedef CGAL::Delaunay_triangulation_2<K> DT;

// 定义点的类型，基于我们选择的Kernel
typedef DT::Point Point_DT;

typedef CGAL::Constrained_Delaunay_triangulation_2<K> CDT;
typedef CGAL::Simple_cartesian<double> Kernel0; // 内核使用双精度浮点数作为该点的笛卡尔坐标
typedef Kernel0::Point_2 Point_2D;               // 二维点
typedef CDT::Point Point_CDT;
typedef Kernel0::Segment_2 Line_2D;           // 二维线段
typedef CDT::Vertex_handle Vertex_handle;

#include <iostream>
#include <fstream>
#include <string>
#include <CGAL/Simple_cartesian.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/Surface_mesh/IO.h>

// 定义点和三角形类型
typedef K::Point_3 Point_3D;
typedef K::Triangle_3 Triangle;

// 定义几何内核
typedef CGAL::Simple_cartesian<double> PM;
typedef CGAL::Surface_mesh<PM::Point_3> ProctedMesh;
class projectPointToMesh
{
public:
	projectPointToMesh(const std::string& filename)
	{
		load(filename);
	}
	// 加载OBJ文件到mesh中
	bool load(const std::string& filename)
	{
		std::ifstream input(filename);
		if (!input)
		{
			std::cerr << "Cannot open file " << filename << std::endl;
			return false;
		}
		if (!CGAL::read_mesh(mesh,filename))
		{
			std::cerr << "Error: Cannot read file " << filename << std::endl;
			return false;
		}

		std::cout << "Successfully read the OBJ file." << std::endl;
		std::cout << "Number of vertices: " << mesh.number_of_vertices() << std::endl;
		std::cout << "Number of faces: " << mesh.number_of_faces() << std::endl;
		return true;
	}

	// 获取网格信息
	const ProctedMesh& get_mesh() const
	{
		return mesh;
	}
private:
	ProctedMesh mesh;  // 存储加载的三角网格
};

//	四分之一圆筒
class Circle {
public:
	/*  左边上半圆筒 r:内圆半径 R1:外圆半径*/
	SplineSurface rec_circle(double r, double R1) {
		double R = R1;
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
		Vec4 p01 = { -R,0,0,1 };
		Vec4 p02 = { 0,R,0,1 };
		Vec4 p03 = { 0,r,0,1 };
		Vec4 p04 = { -r,0,0,1 };
		Vec4 p05 = { -R,R,0,w };
		Vec4 p06 = { -r,r,0,w };

		SL1[0].m_CtrlPts.push_back(p04);
		SL1[0].m_CtrlPts.push_back(p06);
		SL1[0].m_CtrlPts.push_back(p03);

		SL1[1].m_CtrlPts.push_back(p04);
		SL1[1].m_CtrlPts.push_back((p04 + p01) / 2);
		SL1[1].m_CtrlPts.push_back(p01);

		SL1[2].m_CtrlPts.push_back(p01);
		SL1[2].m_CtrlPts.push_back(p05);
		SL1[2].m_CtrlPts.push_back(p02);

		SL1[3].m_CtrlPts.push_back(p03);
		SL1[3].m_CtrlPts.push_back((p02 + p03) / 2);
		SL1[3].m_CtrlPts.push_back(p02);
		SplineSurface ss1;
		ss1.CoonsInterpolate(SL1);

		return ss1;
	}

	/* 左边半圆筒 r:内圆半径 R1:外圆半径 h:拉伸厚度*/
	varray<SplineVolume> get_vol2(double r, double R1, double h) {
		varray<SplineSurface> SS;
		varray<SplineVolume> SV;
		SplineSurface ss;
		ss = rec_circle(r, R1);
		SS.push_back(ss);

		m.Rolate(ss, PI / 2, 3);//将ss绕3（即z轴旋转PI/2）
		SS.push_back(ss);

		SV = m.CreatSweepVol(SS, h, 3);

		return SV;
	}

	/* 右边半圆筒 r:内圆半径 R1:外圆半径 h:拉伸厚度*/
	varray<SplineVolume> get_vol3(double r, double R1, double h) {
		varray<SplineSurface> SS;
		varray<SplineVolume> SV;
		SplineSurface ss;

		ss = rec_circle(r, R1);
		m.Rolate(ss, -PI / 2, 3);
		SS.push_back(ss);
		m.Rolate(ss, -PI / 2, 3);
		SS.push_back(ss);

		SV = m.CreatSweepVol(SS, h, 3);

		return SV;
	}

private:
	Model_Solution m;
	double w = cos(PI / 4);
};

//矩形
class Rec {
public:

	varray<SplineSurface> rec(double a, double b) {
		varray<SplineSurface> SS;
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
		Vec4 p00 = { 0,0,0,1 };
		Vec4 p01 = { -a / 2,-b / 2,0,w };
		Vec4 p02 = { -a / 2,b / 2,0,w };
		Vec4 p03 = { a / 2,b / 2,0,w };
		Vec4 p04 = { a / 2,-b / 2,0,w };
		Vec4 p05 = { -a / 2,0,0,1 };
		Vec4 p06 = { 0,b / 2,0,1 };
		Vec4 p07 = { a / 2,0,0,1 };
		Vec4 p08 = { 0,-b / 2,0,1 };

		SL1[0].m_CtrlPts.push_back(p05);
		SL1[0].m_CtrlPts.push_back((p05 + p06) / 2);
		SL1[0].m_CtrlPts.push_back(p06);

		SL1[1].m_CtrlPts.push_back(p06);
		SL1[1].m_CtrlPts.push_back((p06 + p07) / 2);
		SL1[1].m_CtrlPts.push_back(p07);

		SL1[2].m_CtrlPts.push_back(p08);
		SL1[2].m_CtrlPts.push_back((p08 + p07) / 2);
		SL1[2].m_CtrlPts.push_back(p07);

		SL1[3].m_CtrlPts.push_back(p05);
		SL1[3].m_CtrlPts.push_back((p05 + p08) / 2);
		SL1[3].m_CtrlPts.push_back(p08);

		SplineSurface ss1;
		ss1.CoonsInterpolate(SL1);//Coons插值生成面
		SS.push_back(ss1);
		return SS;
	}
	/*
		a:矩形长
		b:矩形宽
		h:厚度
		矩形中心为坐标原点
	*/
	varray<SplineVolume> get_vol(double a, double b, double h) {
		varray<SplineSurface> SS;
		SS = rec(a, b);//生成面
		return m.CreatSweepVol(SS, h, 3);//通过面生成体
	}
private:
	Model_Solution m;
	double w = cos(PI / 4);
};

//	两半圆弧+矩形
class RecCircle {
public:
	//	左半圆弧+矩形
	SplineSurface rec_circle1(double r, double a) {
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
		Vec4 p01 = { 0,-a / 2,0,1 };
		Vec4 p02 = { -r,-a / 2,0,w };
		Vec4 p03 = { -r,-a / 2 + r,0,1 };
		Vec4 p04 = { -r,a / 2 - r,0,1 };
		Vec4 p05 = { -r,a / 2,0,w };
		Vec4 p06 = { 0,a / 2,0,1 };

		SL1[0].m_CtrlPts.push_back(p03);
		SL1[0].m_CtrlPts.push_back((p03 + p04) / 2);
		SL1[0].m_CtrlPts.push_back(p04);

		SL1[1].m_CtrlPts.push_back(p04);
		SL1[1].m_CtrlPts.push_back(p05);
		SL1[1].m_CtrlPts.push_back(p06);

		SL1[2].m_CtrlPts.push_back(p01);
		SL1[2].m_CtrlPts.push_back((p01 + p06) / 2);
		SL1[2].m_CtrlPts.push_back(p06);

		SL1[3].m_CtrlPts.push_back(p03);
		SL1[3].m_CtrlPts.push_back(p02);
		SL1[3].m_CtrlPts.push_back(p01);
		SplineSurface ss1;
		ss1.CoonsInterpolate(SL1);
		return ss1;
	}
	//	右半圆弧+矩形
	SplineSurface rec_circle2(double r, double a) {
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
		Vec4 p01 = { 0,-a / 2,0,1 };
		Vec4 p02 = { r,-a / 2,0,w };
		Vec4 p03 = { r,-a / 2 + r,0,1 };
		Vec4 p04 = { r,a / 2 - r,0,1 };
		Vec4 p05 = { r,a / 2,0,w };
		Vec4 p06 = { 0,a / 2,0,1 };

		SL1[0].m_CtrlPts.push_back(p01);
		SL1[0].m_CtrlPts.push_back((p01 + p06) / 2);
		SL1[0].m_CtrlPts.push_back(p06);

		SL1[1].m_CtrlPts.push_back(p06);
		SL1[1].m_CtrlPts.push_back(p05);
		SL1[1].m_CtrlPts.push_back(p04);

		SL1[2].m_CtrlPts.push_back(p03);
		SL1[2].m_CtrlPts.push_back((p03 + p04) / 2);
		SL1[2].m_CtrlPts.push_back(p04);

		SL1[3].m_CtrlPts.push_back(p01);
		SL1[3].m_CtrlPts.push_back(p02);
		SL1[3].m_CtrlPts.push_back(p03);
		SplineSurface ss1;
		ss1.CoonsInterpolate(SL1);
		return ss1;
	}
	varray<SplineVolume> get_rec_circle1(double r, double a, double h) {
		varray<SplineSurface> SS;
		SS.push_back(rec_circle1(r, a));//生成面
		SS.push_back(rec_circle2(r, a));//生成面
		return m.CreatSweepVol(SS, h, 3);//通过面生成体
	}

private:
	double w = cos(PI / 4);
	Model_Solution m;
};

//	长方体-内圆柱
class Cube_Cylinder {
public:
	//正方体-内圆柱
	varray<SplineSurface> Cube_cylinder(double l, double r) {
		varray<Spline> SL1;
		varray<Spline> SL2;
		varray<Spline> SL3;
		varray<Spline> SL4;
		varray<SplineSurface> SS;
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
		for (int i = 0; i < 4; i++)
		{
			SL1[i].m_Degree = 2;
			SL1[i].m_Knots = knots;
			SL2[i].m_Degree = 2;
			SL2[i].m_Knots = knots;
			SL3[i].m_Degree = 2;
			SL3[i].m_Knots = knots;
			SL4[i].m_Degree = 2;
			SL4[i].m_Knots = knots;
		}
		Vec4 p01 = { -l / 2,-l / 2,0,1 };
		Vec4 p02 = { -l / 2,l / 2,0,1 };
		Vec4 p03 = { l / 2,l / 2,0,1 };
		Vec4 p04 = { l / 2,-l / 2,0,1 };
		Vec4 p05 = { -sqrt(2)*r / 2,-sqrt(2)*r / 2,0,1 };
		Vec4 p06 = { -sqrt(2)*r / 2,sqrt(2)*r / 2,0,1 };
		Vec4 p07 = { sqrt(2)*r / 2,sqrt(2)*r / 2,0,1 };
		Vec4 p08 = { sqrt(2)*r / 2,-sqrt(2)*r / 2,0,1 };
		Vec4 p09 = { -sqrt(2)*r ,0,0,w };
		Vec4 p10 = { 0,sqrt(2)*r,0,w };
		Vec4 p11 = { sqrt(2)*r ,0,0,w };
		Vec4 p12 = { 0,-sqrt(2)*r,0,w };

		SL1[0].m_CtrlPts.push_back(p01);
		SL1[0].m_CtrlPts.push_back((p05 + p01) / 2);
		SL1[0].m_CtrlPts.push_back(p05);

		SL1[1].m_CtrlPts.push_back(p01);
		SL1[1].m_CtrlPts.push_back((p02 + p01) / 2);
		SL1[1].m_CtrlPts.push_back(p02);

		SL1[2].m_CtrlPts.push_back(p02);
		SL1[2].m_CtrlPts.push_back((p02 + p06) / 2);
		SL1[2].m_CtrlPts.push_back(p06);

		SL1[3].m_CtrlPts.push_back(p05);
		SL1[3].m_CtrlPts.push_back(p09);
		SL1[3].m_CtrlPts.push_back(p06);
		SplineSurface S1;
		S1.CoonsInterpolate(SL1);
		SS.push_back(S1);

		SL2[0].m_CtrlPts.push_back(p06);
		SL2[0].m_CtrlPts.push_back(p10);
		SL2[0].m_CtrlPts.push_back(p07);

		SL2[1].m_CtrlPts.push_back(p06);
		SL2[1].m_CtrlPts.push_back((p06 + p02) / 2);
		SL2[1].m_CtrlPts.push_back(p02);

		SL2[2].m_CtrlPts.push_back(p02);
		SL2[2].m_CtrlPts.push_back((p02 + p03) / 2);
		SL2[2].m_CtrlPts.push_back(p03);

		SL2[3].m_CtrlPts.push_back(p07);
		SL2[3].m_CtrlPts.push_back((p07 + p03) / 2);
		SL2[3].m_CtrlPts.push_back(p03);
		SplineSurface S2;
		S2.CoonsInterpolate(SL2);
		SS.push_back(S2);

		SL3[0].m_CtrlPts.push_back(p08);
		SL3[0].m_CtrlPts.push_back((p04 + p08) / 2);
		SL3[0].m_CtrlPts.push_back(p04);

		SL3[1].m_CtrlPts.push_back(p08);
		SL3[1].m_CtrlPts.push_back(p11);
		SL3[1].m_CtrlPts.push_back(p07);

		SL3[2].m_CtrlPts.push_back(p07);
		SL3[2].m_CtrlPts.push_back((p07 + p03) / 2);
		SL3[2].m_CtrlPts.push_back(p03);

		SL3[3].m_CtrlPts.push_back(p04);
		SL3[3].m_CtrlPts.push_back((p04 + p03) / 2);
		SL3[3].m_CtrlPts.push_back(p03);
		SplineSurface S3;
		S3.CoonsInterpolate(SL3);
		SS.push_back(S3);

		SL4[0].m_CtrlPts.push_back(p01);
		SL4[0].m_CtrlPts.push_back((p01 + p04) / 2);
		SL4[0].m_CtrlPts.push_back(p04);

		SL4[1].m_CtrlPts.push_back(p01);
		SL4[1].m_CtrlPts.push_back((p01 + p05) / 2);
		SL4[1].m_CtrlPts.push_back(p05);

		SL4[2].m_CtrlPts.push_back(p05);
		SL4[2].m_CtrlPts.push_back(p12);
		SL4[2].m_CtrlPts.push_back(p08);

		SL4[3].m_CtrlPts.push_back(p04);
		SL4[3].m_CtrlPts.push_back((p04 + p08) / 2);
		SL4[3].m_CtrlPts.push_back(p08);
		SplineSurface S4;
		S4.CoonsInterpolate(SL4);
		SS.push_back(S4);

		return SS;
	}

	//上方四分之一正方体-内圆柱
	varray<SplineSurface> Cube_cylinder1(double l, double r) {
		varray<Spline> SL1;
		varray<SplineSurface> SS;
		varray<double> knots;
		knots.push_back(0);
		knots.push_back(0);
		knots.push_back(0);
		knots.push_back(1);
		knots.push_back(1);
		knots.push_back(1);
		SL1.resize(4);
		for (int i = 0; i < 4; i++)
		{
			SL1[i].m_Degree = 2;
			SL1[i].m_Knots = knots;
		}
		Vec4 p01 = { -sqrt(2)*r / 2,sqrt(2)*r / 2,0,1 };
		Vec4 p02 = { -l / 2,l / 2,0,1 };
		Vec4 p03 = { l / 2,l / 2,0,1 };
		Vec4 p04 = { sqrt(2)*r / 2,sqrt(2)*r / 2,0,1 };
		Vec4 p05 = { 0,sqrt(2)*r,0,w };

		SL1[0].m_CtrlPts.push_back(p01);
		SL1[0].m_CtrlPts.push_back(p05);
		SL1[0].m_CtrlPts.push_back(p04);

		SL1[1].m_CtrlPts.push_back(p01);
		SL1[1].m_CtrlPts.push_back((p02 + p01) / 2);
		SL1[1].m_CtrlPts.push_back(p02);

		SL1[2].m_CtrlPts.push_back(p02);
		SL1[2].m_CtrlPts.push_back((p02 + p03) / 2);
		SL1[2].m_CtrlPts.push_back(p03);

		SL1[3].m_CtrlPts.push_back(p04);
		SL1[3].m_CtrlPts.push_back((p03 + p04) / 2);
		SL1[3].m_CtrlPts.push_back(p03);
		SplineSurface S1;
		S1.CoonsInterpolate(SL1);
		SS.push_back(S1);

		return SS;
	}

	//下方四分之一正方体-内圆柱
	varray<SplineSurface> Cube_cylinder2(double l, double r) {
		varray<Spline> SL1;
		varray<SplineSurface> SS;
		varray<double> knots;
		knots.push_back(0);
		knots.push_back(0);
		knots.push_back(0);
		knots.push_back(1);
		knots.push_back(1);
		knots.push_back(1);
		SL1.resize(4);
		for (int i = 0; i < 4; i++)
		{
			SL1[i].m_Degree = 2;
			SL1[i].m_Knots = knots;
		}
		Vec4 p01 = { -l / 2,-l / 2,0,1 };
		Vec4 p02 = { -sqrt(2)*r / 2,-sqrt(2)*r / 2,0,1 };
		Vec4 p03 = { sqrt(2)*r / 2,-sqrt(2)*r / 2,0,1 };
		Vec4 p04 = { l / 2,-l / 2, 0,1 };
		Vec4 p05 = { 0,-sqrt(2)*r,0,w };

		SL1[0].m_CtrlPts.push_back(p01);
		SL1[0].m_CtrlPts.push_back((p04 + p01) / 2);
		SL1[0].m_CtrlPts.push_back(p04);

		SL1[1].m_CtrlPts.push_back(p01);
		SL1[1].m_CtrlPts.push_back((p02 + p01) / 2);
		SL1[1].m_CtrlPts.push_back(p02);

		SL1[2].m_CtrlPts.push_back(p02);
		SL1[2].m_CtrlPts.push_back(p05);
		SL1[2].m_CtrlPts.push_back(p03);

		SL1[3].m_CtrlPts.push_back(p04);
		SL1[3].m_CtrlPts.push_back((p04 + p03) / 2);
		SL1[3].m_CtrlPts.push_back(p03);
		SplineSurface S1;
		S1.CoonsInterpolate(SL1);
		SS.push_back(S1);

		return SS;
	}

	/*  正方体-内圆柱
		l:正方形边长 r:内圆半径 h:厚度*/
	varray<SplineVolume> get_vol(double l, double r, double h) {
		varray<SplineSurface> SS;
		SS = Cube_cylinder(l, r);
		return m.CreatSweepVol(SS, h, 3);
	}

	/*  上方四分之一正方体-内圆柱
		l:正方形边长 r:内圆半径 h:厚度*/
	varray<SplineVolume> get_vol1(double l, double r, double h) {
		varray<SplineSurface> SS;
		SS = Cube_cylinder1(l, r);
		return m.CreatSweepVol(SS, h, 3);
	}

	/*  下方四分之一正方体-内圆柱
		l:正方形边长 r:内圆半径 h:厚度*/
	varray<SplineVolume> get_vol2(double l, double r, double h) {
		varray<SplineSurface> SS;
		SS = Cube_cylinder2(l, r);
		return m.CreatSweepVol(SS, h, 3);
	}
private:
	Model_Solution m;
	double w = cos(PI / 4);
};

class CircleVolum
{
public:
	SplineSurface rec_circle(double r, double l) {
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
		Vec4 p01 = { -sqrt(2)*r,-sqrt(2)*r,0,1 };
		Vec4 p02 = { -sqrt(2)*r,sqrt(2)*r,0,1 };
		Vec4 p05 = { -r,-r,0,1 };
		Vec4 p06 = { -r,r,0,1 };
		Vec4 p09 = { -sqrt(2)*r,0,0,w };

		SL1[0].m_CtrlPts.push_back(p01);
		SL1[0].m_CtrlPts.push_back((p01 + p05) / 2);
		SL1[0].m_CtrlPts.push_back(p05);

		SL1[1].m_CtrlPts.push_back(p01);
		SL1[1].m_CtrlPts.push_back(p09);
		SL1[1].m_CtrlPts.push_back(p02);

		SL1[2].m_CtrlPts.push_back(p02);
		SL1[2].m_CtrlPts.push_back((p02 + p06) / 2);
		SL1[2].m_CtrlPts.push_back(p06);

		SL1[3].m_CtrlPts.push_back(p05);
		SL1[3].m_CtrlPts.push_back((p05 + p06) / 2);
		SL1[3].m_CtrlPts.push_back(p06);

		SplineSurface ss1;
		ss1.CoonsInterpolate(SL1);
		return ss1;
	}

	varray<SplineVolume> get_vol(double r, double h) {
		varray<SplineSurface> SS;
		SS.push_back(rec_circle(r, h));
		return m.CreatSweepVol(SS, h, 3);
	}

private:
	Model_Solution m;
	double w = cos(PI / 4);
};

//双圆环
class DoubleCircle
{
public:

	//XY平面圆弧
	SplineSurface double_Circle_XY(double R, double r)
	{
		varray<Spline> SL1;
		varray<double> knots;
		knots.push_back(0);
		knots.push_back(0);
		knots.push_back(0);
		knots.push_back(1);
		knots.push_back(1);
		knots.push_back(1);
		SL1.resize(4);
		for (int i = 0; i < 4; i++)
		{
			SL1[i].m_Degree = 2;
			SL1[i].m_Knots = knots;
		}
		Vec4 p01 = { -R,-R,0,w };
		Vec4 p02 = { -R,0,0,1 };
		Vec4 p03 = { -r,0,0,1 };
		Vec4 p04 = { 0,-r,0,1 };
		Vec4 p05 = { 0,-R,0,1 };
		Vec4 p06 = { -r,-r,0,w };

		SL1[0].m_CtrlPts.push_back(p05);
		SL1[0].m_CtrlPts.push_back(p01);
		SL1[0].m_CtrlPts.push_back(p02);

		SL1[1].m_CtrlPts.push_back(p02);
		SL1[1].m_CtrlPts.push_back((p02 + p03) / 2);
		SL1[1].m_CtrlPts.push_back(p03);

		SL1[2].m_CtrlPts.push_back(p04);
		SL1[2].m_CtrlPts.push_back(p06);
		SL1[2].m_CtrlPts.push_back(p03);

		SL1[3].m_CtrlPts.push_back(p05);
		SL1[3].m_CtrlPts.push_back((p05 + p04) / 2);
		SL1[3].m_CtrlPts.push_back(p04);

		SplineSurface ss;
		ss.CoonsInterpolate(SL1);
		return ss;
	}

	//沿着指定路径扫掠
	varray<SplineVolume> get_vol_Incline(double R, double r)
	{
		varray<SplineVolume> SS;
		SplineSurface ss = double_Circle_XY(R, r);//生成第二象限四分之一圆弧

		varray<SplineSurface> SSL;
		SSL.push_back(ss);

		SSL.resize(1);
		//绕圆心旋转，获得完整圆弧
		SSL.push_back(ss);

		m.Rolate(ss, PI / 2, 3);
		SSL.push_back(ss);

		m.Rolate(ss, PI / 2, 3);
		SSL.push_back(ss);

		m.Rolate(ss, PI / 2, 3);
		SSL.push_back(ss);

		//生成路径
		varray<double> knots;
		Spline Path;
		knots.push_back(0);
		knots.push_back(0);
		knots.push_back(0);
		knots.push_back(1);
		knots.push_back(1);
		knots.push_back(1);
		Path.resize(1);
		for (int i = 0; i < Path.size(); i++)
		{
			Path.m_Degree = 2;
			Path.m_Knots = knots;
		}

		Vec4 p01 = { 0,-40,40,1 };
		Vec4 p02 = { 0,-20,20,1 };
		Vec4 p03 = { 0,0,0,1 };

		Path.m_CtrlPts.push_back(p03);
		Path.m_CtrlPts.push_back(p02);
		Path.m_CtrlPts.push_back(p01);

		//扫掠生成nurbs体
		SplineVolume sv1;
		for (int i = 0; i < SSL.size(); i++)
		{
			sv1.CreateTransSweepSplineVolume(Path, SSL[i]);
			SS.push_back(sv1);
		}

		return SS;
	}

	//YZ平面圆弧
	varray<SplineSurface> double_Circle_YZ(double R, double r)
	{
		varray<Spline> SL1;
		varray<double> knots;
		knots.push_back(0);
		knots.push_back(0);
		knots.push_back(0);
		knots.push_back(1);
		knots.push_back(1);
		knots.push_back(1);
		SL1.resize(4);
		for (int i = 0; i < 4; i++)
		{
			SL1[i].m_Degree = 2;
			SL1[i].m_Knots = knots;
		}
		Vec4 p01 = { 0,-R,R,w };
		Vec4 p02 = { 0,0,R,1 };
		Vec4 p03 = { 0,0,r,1 };
		Vec4 p04 = { 0,-r,0,1 };
		Vec4 p05 = { 0,-R,0,1 };
		Vec4 p06 = { 0,-r,r,w };

		SL1[0].m_CtrlPts.push_back(p05);
		SL1[0].m_CtrlPts.push_back(p01);
		SL1[0].m_CtrlPts.push_back(p02);

		SL1[1].m_CtrlPts.push_back(p02);
		SL1[1].m_CtrlPts.push_back((p02 + p03) / 2);
		SL1[1].m_CtrlPts.push_back(p03);

		SL1[2].m_CtrlPts.push_back(p04);
		SL1[2].m_CtrlPts.push_back(p06);
		SL1[2].m_CtrlPts.push_back(p03);

		SL1[3].m_CtrlPts.push_back(p05);
		SL1[3].m_CtrlPts.push_back((p05 + p04) / 2);
		SL1[3].m_CtrlPts.push_back(p04);

		SplineSurface ss;
		ss.CoonsInterpolate(SL1);

		varray<SplineSurface> SS;
		SS.push_back(ss);

		m.Rolate(ss, PI / 2, 1);
		SS.push_back(ss);

		m.Rolate(ss, PI / 2, 1);
		SS.push_back(ss);

		m.Rolate(ss, PI / 2, 1);
		SS.push_back(ss);
		return SS;
	}

	//YZ平面圆弧
	SplineSurface double_Circle_YZ_0(double R, double r)
	{
		varray<Spline> SL1;
		varray<double> knots;
		knots.push_back(0);
		knots.push_back(0);
		knots.push_back(0);
		knots.push_back(1);
		knots.push_back(1);
		knots.push_back(1);
		SL1.resize(4);
		for (int i = 0; i < 4; i++)
		{
			SL1[i].m_Degree = 2;
			SL1[i].m_Knots = knots;
		}
		Vec4 p01 = { 0,-R,R,w };
		Vec4 p02 = { 0,0,R,1 };
		Vec4 p03 = { 0,0,r,1 };
		Vec4 p04 = { 0,-r,0,1 };
		Vec4 p05 = { 0,-R,0,1 };
		Vec4 p06 = { 0,-r,r,w };

		SL1[0].m_CtrlPts.push_back(p05);
		SL1[0].m_CtrlPts.push_back(p01);
		SL1[0].m_CtrlPts.push_back(p02);

		SL1[1].m_CtrlPts.push_back(p02);
		SL1[1].m_CtrlPts.push_back((p02 + p03) / 2);
		SL1[1].m_CtrlPts.push_back(p03);

		SL1[2].m_CtrlPts.push_back(p04);
		SL1[2].m_CtrlPts.push_back(p06);
		SL1[2].m_CtrlPts.push_back(p03);

		SL1[3].m_CtrlPts.push_back(p05);
		SL1[3].m_CtrlPts.push_back((p05 + p04) / 2);
		SL1[3].m_CtrlPts.push_back(p04);

		SplineSurface ss;
		ss.CoonsInterpolate(SL1);
		return ss;
	}

	//生成垂直于XY平面的圆筒
	varray<SplineVolume> get_vol_XY(double R, double r, double h)
	{
		varray<SplineSurface> SS;
		SplineSurface ss = double_Circle_XY(R, r);

		SS.push_back(ss);
		m.Rolate(ss, PI / 2, 3);

		SS.push_back(ss);
		m.Rolate(ss, PI / 2, 3);

		SS.push_back(ss);
		m.Rolate(ss, PI / 2, 3);

		SS.push_back(ss);

		return m.CreatSweepVol(SS, h, 3);
	}

	//生成垂直于YZ平面的圆筒
	varray<SplineVolume> get_vol_YZ(double R, double r, double h)
	{
		varray<SplineSurface> SS = double_Circle_YZ(R, r);

		varray<SplineVolume> SV;
		SV = m.CreatSweepVol(SS, h, 1);
		return SV;
	}

	//按照路径旋转获得nurbs体
	varray<SplineVolume> get_vol_Path(double R, double r, double h)
	{
		varray<SplineSurface> SS = double_Circle_YZ(R, r);
		varray<SplineVolume> SV;
		SplineVolume T;
		SV.resize(SS.size());
		//构建路径
		Spline pathT;
		varray<double> knots;

		knots.push_back(0);
		knots.push_back(0);
		knots.push_back(0);
		knots.push_back(1);
		knots.push_back(1);
		knots.push_back(1);

		pathT.m_Knots = knots;
		pathT.m_Degree = 2;

		Vec4 p01 = { 1,0,0,1 };
		Vec4 p02 = { 5,0,0,1 };
		Vec4 p03 = { 10,0,0,1 };

		pathT.m_CtrlPts.push_back(p01);
		pathT.m_CtrlPts.push_back(p02);
		pathT.m_CtrlPts.push_back(p03);

		SplineVolume v;
		v.CreateTransSweepSplineVolume(pathT, SS[0]);
		SV.push_back(v);
		SV.push_back(T);
		return SV;
	}

private:
	double w = cos(PI / 4);
	Model_Solution m;
};

//菱形
class Rhombus
{
public:
	SplineSurface rec_circle(double a) {
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
		Vec4 p01 = { 0,-a,0,1 };
		Vec4 p02 = { -a,0,0,1 };
		Vec4 p03 = { 0,a,0,1 };
		Vec4 p04 = { a,0,0,1 };

		SL1[0].m_CtrlPts.push_back(p02);
		SL1[0].m_CtrlPts.push_back((p02 + p01) / 2);
		SL1[0].m_CtrlPts.push_back(p01);

		SL1[1].m_CtrlPts.push_back(p02);
		SL1[1].m_CtrlPts.push_back((p02 + p03) / 2);
		SL1[1].m_CtrlPts.push_back(p03);

		SL1[2].m_CtrlPts.push_back(p03);
		SL1[2].m_CtrlPts.push_back((p03 + p04) / 2);
		SL1[2].m_CtrlPts.push_back(p04);

		SL1[3].m_CtrlPts.push_back(p01);
		SL1[3].m_CtrlPts.push_back((p01 + p04) / 2);
		SL1[3].m_CtrlPts.push_back(p04);

		SplineSurface ss1;
		ss1.CoonsInterpolate(SL1);
		return ss1;
	}

	varray<SplineVolume> get_vol(double a, double h) {
		varray<SplineSurface> SS;
		SS.push_back(rec_circle(a));
		return m.CreatSweepVol(SS, h, 3);
	}

private:
	Model_Solution m;
	double w = cos(PI / 4);
};

//李瑞阁的模型1
class LeeRuiGe {
public:
	SplineSurface double_Circle_XY()
	{
		varray<Spline> SL1;
		varray<double> knots;
		knots.push_back(0);
		knots.push_back(0);
		knots.push_back(0);
		knots.push_back(1);
		knots.push_back(1);
		knots.push_back(1);
		SL1.resize(4);
		for (int i = 0; i < 4; i++)
		{
			SL1[i].m_Degree = 2;
			SL1[i].m_Knots = knots;
		}
		Vec4 p01 = { -12,0,0,1 };
		Vec4 p02 = { -2,0,0,w };
		Vec4 p03 = { 0,0,0,1 };
		Vec4 p04 = { 0,8,0,1 };
		Vec4 p05 = { -2,8,0,1 };

		SL1[0].m_CtrlPts.push_back(p05);
		SL1[0].m_CtrlPts.push_back((p05 + p04) / 2);
		SL1[0].m_CtrlPts.push_back(p04);

		SL1[1].m_CtrlPts.push_back(p03);
		SL1[1].m_CtrlPts.push_back((p03 + p04) / 2);
		SL1[1].m_CtrlPts.push_back(p04);

		SL1[2].m_CtrlPts.push_back(p01);
		SL1[2].m_CtrlPts.push_back((p01 + p03) / 2);
		SL1[2].m_CtrlPts.push_back(p03);

		SL1[3].m_CtrlPts.push_back(p01);
		SL1[3].m_CtrlPts.push_back(p02);
		SL1[3].m_CtrlPts.push_back(p05);

		SplineSurface ss;
		ss.CoonsInterpolate(SL1);
		return ss;
	}
	void set_01(double d, double l, double r, double L, double h, double H)
	{
		this->d = d;
		this->l = l;
		this->r = r;
		this->L = L;
		this->h = h;
		this->H = H;
	}
	void set_02(double d1, double l1, double r1)
	{
		this->d1 = d1;
		this->l1 = l1;
		this->r1 = r1;
	}

	//长方形
	varray<SplineSurface> oblong()
	{
		varray<Spline> SL1;

		SplineSurface ss;
		varray<SplineSurface> SS;
		varray<double> knots;
		knots.push_back(0);
		knots.push_back(0);
		knots.push_back(0);
		knots.push_back(1);
		knots.push_back(1);
		knots.push_back(1);
		SL1.resize(4);
		for (int i = 0; i < 4; i++)
		{
			SL1[i].m_Degree = 2;
			SL1[i].m_Knots = knots;
		}

		Vec4 p01 = { 0,l + L / 2,0,1 };
		Vec4 p02 = { H,l + L / 2,0,1 };
		Vec4 p03 = { H,-(l + L / 2),0,1 };
		Vec4 p04 = { 0,-(l + L / 2) ,0,1 };

		SL1[0].m_CtrlPts.push_back(p01);
		SL1[0].m_CtrlPts.push_back((p01 + p02) / 2);
		SL1[0].m_CtrlPts.push_back(p02);

		SL1[1].m_CtrlPts.push_back(p04);
		SL1[1].m_CtrlPts.push_back((p04 + p01) / 2);
		SL1[1].m_CtrlPts.push_back(p01);

		SL1[2].m_CtrlPts.push_back(p04);
		SL1[2].m_CtrlPts.push_back((p04 + p03) / 2);
		SL1[2].m_CtrlPts.push_back(p03);

		SL1[3].m_CtrlPts.push_back(p03);
		SL1[3].m_CtrlPts.push_back((p03 + p02) / 2);
		SL1[3].m_CtrlPts.push_back(p02);

		ss.CoonsInterpolate(SL1);
		SS.push_back(ss);
		return SS;
	}

	//创建梯形带圆弧缺口平面
	varray<SplineSurface> ladder_Arc(double d, double r, double l)
	{
		varray<Spline> SL1;
		varray<Spline> SL2;

		SplineSurface ss;
		varray<SplineSurface> SS1;
		varray<SplineSurface> SS;
		varray<double> knots;
		knots.push_back(0);
		knots.push_back(0);
		knots.push_back(0);
		knots.push_back(1);
		knots.push_back(1);
		knots.push_back(1);
		SL1.resize(4);
		for (int i = 0; i < 4; i++)
		{
			SL1[i].m_Degree = 2;
			SL1[i].m_Knots = knots;
		}

		SL2.resize(4);
		for (int i = 0; i < 4; i++)
		{
			SL2[i].m_Degree = 2;
			SL2[i].m_Knots = knots;
		}
		Vec4 p01 = { -l,d / 2,0,1 };
		Vec4 p02 = { 0,r,0,1 };
		Vec4 p03 = { 0,-r,0,1 };
		Vec4 p04 = { -l,-d / 2,0,1 };
		Vec4 p05 = { -r,0,0,1 };
		Vec4 p06 = { -r,r,0,w };
		Vec4 p07 = { -r,-r,0,w };
		Vec4 p08 = { -l,0,0,1 };

		SL1[0].m_CtrlPts.push_back(p08);
		SL1[0].m_CtrlPts.push_back((p08 + p05) / 2);
		SL1[0].m_CtrlPts.push_back(p05);

		SL1[1].m_CtrlPts.push_back(p05);
		SL1[1].m_CtrlPts.push_back(p06);
		SL1[1].m_CtrlPts.push_back(p02);

		SL1[2].m_CtrlPts.push_back(p02);
		SL1[2].m_CtrlPts.push_back((p02 + p01) / 2);
		SL1[2].m_CtrlPts.push_back(p01);

		SL1[3].m_CtrlPts.push_back(p08);
		SL1[3].m_CtrlPts.push_back((p08 + p01) / 2);
		SL1[3].m_CtrlPts.push_back(p01);

		ss.CoonsInterpolate(SL1);
		SS.push_back(ss);

		SL2[0].m_CtrlPts.push_back(p04);
		SL2[0].m_CtrlPts.push_back((p04 + p03) / 2);
		SL2[0].m_CtrlPts.push_back(p03);

		SL2[1].m_CtrlPts.push_back(p03);
		SL2[1].m_CtrlPts.push_back(p07);
		SL2[1].m_CtrlPts.push_back(p05);

		SL2[2].m_CtrlPts.push_back(p08);
		SL2[2].m_CtrlPts.push_back((p08 + p05) / 2);
		SL2[2].m_CtrlPts.push_back(p05);

		SL2[3].m_CtrlPts.push_back(p04);
		SL2[3].m_CtrlPts.push_back((p04 + p08) / 2);
		SL2[3].m_CtrlPts.push_back(p08);
		ss.Clear();
		ss.CoonsInterpolate(SL2);
		SS.push_back(ss);
		return SS;
	}

	varray<SplineSurface> square_Circle()
	{
		varray<SplineSurface> SS;

		varray<Spline> SL1;

		SplineSurface ss;
		varray<SplineSurface> SS1;
		varray<double> knots;
		knots.push_back(0);
		knots.push_back(0);
		knots.push_back(0);
		knots.push_back(1);
		knots.push_back(1);
		knots.push_back(1);
		SL1.resize(4);
		for (int i = 0; i < 4; i++)
		{
			SL1[i].m_Degree = 2;
			SL1[i].m_Knots = knots;
		}

		Vec4 p01 = { 0,-l - L / 2,-l - L / 2,1 };
		Vec4 p02 = { 0,-l - L / 2,l + L / 2,1 };
		Vec4 p03 = { 0,-sqrt(2)* r1 / 2,sqrt(2) * r1 / 2,1 };
		Vec4 p04 = { 0,-sqrt(2) * r1,0,w };
		Vec4 p05 = { 0,-sqrt(2) * r1 / 2,-sqrt(2) * r1 / 2,1 };

		SL1[2].m_CtrlPts.push_back(p01);
		SL1[2].m_CtrlPts.push_back((p01 + p05) / 2);
		SL1[2].m_CtrlPts.push_back(p05);

		SL1[3].m_CtrlPts.push_back(p05);
		SL1[3].m_CtrlPts.push_back(p04);
		SL1[3].m_CtrlPts.push_back(p03);

		SL1[0].m_CtrlPts.push_back(p03);
		SL1[0].m_CtrlPts.push_back((p03 + p02) / 2);
		SL1[0].m_CtrlPts.push_back(p02);

		SL1[1].m_CtrlPts.push_back(p01);
		SL1[1].m_CtrlPts.push_back((p01 + p02) / 2);
		SL1[1].m_CtrlPts.push_back(p02);

		ss.CoonsInterpolate(SL1);
		SS.push_back(ss);

		return SS;
	}

	//创建梯形平面
	varray<SplineSurface> ladder(double d, double r, double L, double l)
	{
		varray<Spline> SL1;

		SplineSurface ss;
		varray<SplineSurface> SS1;
		varray<SplineSurface> SS;
		varray<double> knots;
		knots.push_back(0);
		knots.push_back(0);
		knots.push_back(0);
		knots.push_back(1);
		knots.push_back(1);
		knots.push_back(1);
		SL1.resize(4);
		for (int i = 0; i < 4; i++)
		{
			SL1[i].m_Degree = 2;
			SL1[i].m_Knots = knots;
		}

		Vec4 p01 = { -l,-d / 2,0,1 };
		Vec4 p02 = { 0,-r,0,1 };
		Vec4 p03 = { L,-r,0,1 };
		Vec4 p04 = { L + l,-d / 2,0,1 };

		SL1[0].m_CtrlPts.push_back(p01);
		SL1[0].m_CtrlPts.push_back((p01 + p04) / 2);
		SL1[0].m_CtrlPts.push_back(p04);

		SL1[1].m_CtrlPts.push_back(p04);
		SL1[1].m_CtrlPts.push_back((p04 + p03) / 2);
		SL1[1].m_CtrlPts.push_back(p03);

		SL1[2].m_CtrlPts.push_back(p02);
		SL1[2].m_CtrlPts.push_back((p02 + p03) / 2);
		SL1[2].m_CtrlPts.push_back(p03);

		SL1[3].m_CtrlPts.push_back(p01);
		SL1[3].m_CtrlPts.push_back((p01 + p02) / 2);
		SL1[3].m_CtrlPts.push_back(p02);

		ss.CoonsInterpolate(SL1);
		SS.push_back(ss);
		return SS;
	}

	//创建nurbs曲线，为了调试用
	varray<Spline> creatSpline()
	{
		varray<double> knots;
		varray<Spline> SL1;
		knots.push_back(0);
		knots.push_back(0);
		knots.push_back(0);
		knots.push_back(1);
		knots.push_back(1);
		knots.push_back(1);
		SL1.resize(4);
		for (int i = 0; i < 4; i++)
		{
			SL1[i].m_Degree = 2;
			SL1[i].m_Knots = knots;
		}
		Vec4 p01 = { -l,-d / 2,0,1 };
		Vec4 p02 = { 0,-r,0,1 };
		Vec4 p03 = { L,-r,0,1 };
		Vec4 p04 = { L + l,-d / 2,0,1 };

		SL1[0].m_CtrlPts.push_back(p01);
		SL1[0].m_CtrlPts.push_back((p01 + p04) / 2);
		SL1[0].m_CtrlPts.push_back(p04);

		SL1[1].m_CtrlPts.push_back(p04);
		SL1[1].m_CtrlPts.push_back((p04 + p03) / 2);
		SL1[1].m_CtrlPts.push_back(p03);

		SL1[2].m_CtrlPts.push_back(p02);
		SL1[2].m_CtrlPts.push_back((p02 + p03) / 2);
		SL1[2].m_CtrlPts.push_back(p03);

		SL1[3].m_CtrlPts.push_back(p01);
		SL1[3].m_CtrlPts.push_back((p01 + p02) / 2);
		SL1[3].m_CtrlPts.push_back(p02);

		return SL1;
	}

	//拉伸成体
	varray<SplineVolume> get_Model01()
	{
		varray<SplineVolume> SV;
		varray<SplineVolume> SV1;
		//构建底座
		SV1 = m.CreatSweepVol(ladder_Arc(d, r, l), H, 3);
		SV = SV1;

		SV1 = m.CreatSweepVol(ladder(d, r, L, l), H, 3);

		for (int i = 0; i < SV1.size(); i++) {
			SV.push_back(SV1[i]);
		}

		m.Trans(SV, L / 2, -1);

		SV1 = SV;

		m.Rolate(SV1, PI, 3);
		for (int i = 0; i < SV1.size(); i++) {
			SV.push_back(SV1[i]);
		}
		return SV;
	}

	varray<SplineVolume> get_Model02()
	{
		varray<SplineVolume> SV;
		varray<SplineVolume> SV1;
		varray<SplineVolume> SV2;

		SV1 = m.CreatSweepVol(square_Circle(), H, 1);
		for (int i = 0; i < SV1.size(); i++) {
			SV.push_back(SV1[i]);
		}

		m.Rolate(SV1, PI / 2, 1);
		for (int i = 0; i < SV1.size(); i++) {
			SV.push_back(SV1[i]);
		}

		m.Rolate(SV1, PI / 2, 1);
		for (int i = 0; i < SV1.size(); i++) {
			SV.push_back(SV1[i]);
		}

		m.Rolate(SV1, PI / 2, 1);
		for (int i = 0; i < SV1.size(); i++) {
			SV.push_back(SV1[i]);
		}

		m.Rolate(SV, PI / 2, 3);
		m.Trans(SV, d / 2, 2);
		m.Trans(SV, l + L / 2 + H, 3);
		return SV;
	}

	varray<SplineVolume> get_Model03()
	{
		varray<SplineVolume> SV;
		varray<SplineVolume> SV1;
		//构建小底座
		SV1.clear();
		SV1 = m.CreatSweepVol(oblong(), H, 3);
		for (int i = 0; i < SV1.size(); i++) {
			SV.push_back(SV1[i]);
		}
		m.Rolate(SV, PI / 2, 3);
		m.Trans(SV, d / 2, 2);
		return SV;
	}

	varray<SplineVolume> assembly_Model()
	{
		varray<SplineVolume> SV;
		varray<SplineVolume> SV1;
		varray<SplineVolume> SV2;
		//构建底座

		SV1 = get_Model01();
		for (int i = 0; i < SV1.size(); i++) {
			SV.push_back(SV1[i]);
		}

		//构建侧体
		SV1 = get_Model02();
		for (int i = 0; i < SV1.size(); i++) {
			SV.push_back(SV1[i]);
		}

		//构建小底座
		SV1 = get_Model03();
		for (int i = 0; i < SV1.size(); i++) {
			SV.push_back(SV1[i]);
		}
		return SV;
	}

private:
	double H;
	double d;
	double l;
	double r;
	double L;
	double h;
	Model_Solution m;
	double w = cos(PI / 4);
	double r1;
	double d1;
	double l1;
};

class CreateSurface {
private:
	Model_Solution m;
	double w = cos(PI / 4);
public:

	varray<SplineVolume> CreateSweepVolumeTest(Spline Path, varray<SplineSurface> SS)
	{
		//由于改变不完全的原因，需要进行格式转换，才能用放样以及垂直于路径扫描的功能
		//Spline与Cnurbslines等之间的转换
		//包含在NurbsTrans.h的头文件中
		NurbsLine nl;
		varray<NurbsSurface>sfs;
		NurbsVol vol;
		varray<NurbsVol> vols;
		//这个还需要指定大小，否则会报错
		vols.resize(SS.size());

		nl = NurbsTrans::SplineToCnurbsline(Path);
		sfs = NurbsTrans::SplinesurfsToCsurfs(SS);

		for (int i = 0; i < sfs.size(); i++)
		{
			vols[i].CreateSweepNurbsVol(nl, sfs[i], 2);
		}

		//体格式再转换回来
		varray<SplineVolume> SV;
		SV = NurbsTrans::CvolsToSplinevols(vols);
		return SV;
	}
	varray<SplineSurface> createSurface(double r, double R)
	{
		varray<double> knots;
		varray<Spline> SL1;
		knots.push_back(0);
		knots.push_back(0);
		knots.push_back(0);
		knots.push_back(1);
		knots.push_back(1);
		knots.push_back(1);
		SL1.resize(4);
		for (auto &i : SL1)
		{
			i.m_Degree = 2;
			i.m_Knots = knots;
		}
		Vec4 p01 = { 0, -r,0,1 };
		Vec4 p02 = { 0,0,r,1 };
		Vec4 p03 = { 0,r,0,1 };
		Vec4 p04 = { 0, 0,-r,1 };
		Vec4 p05 = { 0,-r,r,w };
		Vec4 p06 = { 0,r,r,w };
		Vec4 p07 = { 0,r,-r,w };
		Vec4 p08 = { 0,-r,-r,w };

		SL1[0].m_CtrlPts.push_back(p04);
		SL1[0].m_CtrlPts.push_back(p08);
		SL1[0].m_CtrlPts.push_back(p01);

		SL1[1].m_CtrlPts.push_back(p04);
		SL1[1].m_CtrlPts.push_back(p07);
		SL1[1].m_CtrlPts.push_back(p03);

		SL1[2].m_CtrlPts.push_back(p03);
		SL1[2].m_CtrlPts.push_back(p06);
		SL1[2].m_CtrlPts.push_back(p02);

		SL1[3].m_CtrlPts.push_back(p01);
		SL1[3].m_CtrlPts.push_back(p05);
		SL1[3].m_CtrlPts.push_back(p02);
		SplineSurface ss;
		varray<SplineSurface> SS;
		ss.CoonsInterpolate(SL1);
		SS.push_back(ss);

		m.Trans(SS, R, 2);
		m.Trans(SS, R, -2);
		m.Rolate(SS, PI / 2, 2);
		RWGeometric rwg;
		rwg.WriteSpline("E:\\Model\\TestModel\\ChackSpline.txt", SL1);
		rwg.WriteSplineSurface("E:\\Model\\TestModel\\ChackSurface.txt", SS);
		return SS;
	}

	varray<Spline> createPath(double R)
	{
		varray<double> knots;
		varray<Spline> SL1;
		knots.push_back(0);
		knots.push_back(0);
		knots.push_back(0);
		knots.push_back(1);
		knots.push_back(1);
		knots.push_back(1);
		SL1.resize(1);
		for (auto &i : SL1)
		{
			i.m_Degree = 2;
			i.m_Knots = knots;
		}
		Vec4 p01 = { 0,0,0,1 };
		Vec4 p02 = { 0,0,R / 2,1 };
		Vec4 p03 = { 0,0,R,1 };

		SL1[0].m_CtrlPts.push_back(p01);
		SL1[0].m_CtrlPts.push_back(p02);
		SL1[0].m_CtrlPts.push_back(p03);

		RWGeometric rwg;
		rwg.WriteSpline("E:\\Model\\TestModel\\ChackSpline.txt", SL1);
		return SL1;
	}

	void test(double r, double R) {
		varray<Spline> path;
		varray<SplineSurface> ss;
		varray<SplineVolume> sv;
		varray<SplineVolume> temp;
		ss = this->createSurface(r, R);
		path = this->createPath(R);
		for (auto &i : path)
		{
			//temp = this->CreateSweepVolumeTest(i, ss);
			for (auto &j : temp)
			{
				sv.push_back(j);
			}
			m.Rolate(ss, -(PI / 2), 3);
		}

		RWGeometric rwg;
		rwg.WriteSplineSurface("E:\\Model\\WcWfFile\\ChackSurface.txt", ss);
		rwg.WriteSplineVolume("E:\\Model\\WcWfFile\\ChackVolume.txt", sv);
	}
};

class CreatePolygons
{
private:
	Model_Solution m;
	RWGeometric rwg;
	Spline0 s0;
	varray<Spline> S;
public:
	void createPolygons1()
	{
		double l1 = 10;
		double l2 = 4;
		Vec4 p1 = { -l1,l1,0,1 };
		Vec4 p2 = { l1,l1,0,1 };
		Vec4 p3 = { -l1,-l1,0,1 };
		Vec4 p4 = { l1,-l1,0,1 };
		Vec4 p5 = { -l2,0,0,1 };
		Vec4 p6 = { l2,0,0,1 };
		S.push_back(s0.getSpline(p1, p2));
		S.push_back(s0.getSpline(p4, p2));
		S.push_back(s0.getSpline(p3, p4));
		S.push_back(s0.getSpline(p1, p3));
		rwg.WriteSpline("E:\\Model\\PlaneQuad\\OuterBoundry.txt", S);

		S.clear();
		S.push_back(s0.getSpline(p1, p5));
		S.push_back(s0.getSpline(p5, p6));
		S.push_back(s0.getSpline(p6, p4));
		S.push_back(s0.getSpline(p3, p4));
		S.push_back(s0.getSpline(p1, p3));
		rwg.WriteSpline("E:\\Model\\PlaneQuad\\InnerBoundry1.txt", S);

		S.clear();
		S.push_back(s0.getSpline(p1, p5));
		S.push_back(s0.getSpline(p5, p6));
		S.push_back(s0.getSpline(p6, p4));
		S.push_back(s0.getSpline(p4, p2));
		S.push_back(s0.getSpline(p1, p2));
		rwg.WriteSpline("E:\\Model\\PlaneQuad\\InnerBoundry2.txt", S);
	}
};

//提取函数
class Extract {
private:
	RWGeometric rwg;
	Model_Solution m;
public:
	//提取曲面的边界曲线
	varray<Spline> getBoundry(varray<SplineSurface> SS) {
		varray<varray<Spline>> s0;
		varray<Spline> s;
		s0 = m.GetBoundryLines(SS);
		for (auto it = s0.begin(); it != s0.end(); it++) {
			for (auto &i : *it) {
				s.push_back(i);
			}
		}

		return s;
	}
	//提取体的所有面
	varray<SplineSurface> getSurface(varray<SplineVolume> SV) {
		varray<SplineSurface> SS;
		varray<varray<SplineSurface>> S0;
		S0 = m.GetSurfaces(SV);
		for (auto &i : S0) {
			for (auto &j : i) {
				SS.push_back(j);
			}
		}
		return SS;
	}

	//Vec4转Point4d
	point4d vec4ToPoint4d(Vec4 v)
	{
		point4d t;
		t.Set(v.x, v.y, v.z);
		t.w = v.w;
		return t;
	}

	//提取曲线控制点
	varray<varray<point4d>> getPoint(varray<Spline> s) {
		varray<Vec4> p;
		varray<point4d> pp;
		varray<varray<point4d>> point;

		//提取每条边上的控制点
		for (auto&i : s) {
			for (auto &j : i.m_CtrlPts) {
				p.push_back(j);
			}
		}

		////去除重复的点
		//for (int i = 0; i < p.size() - 1; ++i) {
		//	for (int j = i + 1; j < p.size(); ++j) {
		//		if (m.JudgeTwoPointsCoincide(p[i], p[j])) {
		//			p.erase(p.begin() + j);
		//		}
		//	}

		//}

		//将Vec4转换才point4d
		for (auto &i : p) {
			pp.clear();
			pp.push_back(this->vec4ToPoint4d(i));
			point.push_back(pp);
		}

		return point;
	}

	//提取曲面的控制点
	varray<varray<point4d>> getPoint(varray<SplineSurface> SS) {
		varray<Spline> S = this->getBoundry(SS);
		return this->getPoint(S);
	}

	//提取体控制点
	varray<varray<point4d>> getPoint(varray<SplineVolume> SV) {
		varray<SplineSurface> SS = this->getSurface(SV);
		varray<Spline> S = this->getBoundry(SS);
		return this->getPoint(S);
	}
};

class LHX
{
public:
	//四分之一扇
	//r为内圆环半径，R1为外圆环半径
	varray<SplineSurface> quadrant(double r) {
		varray<SplineSurface>ss;
		varray<Spline> SL1;
		varray<Spline> SL2;
		varray<Spline> SL3;
		Vec4 p01 = { -r,0,0,1 };
		Vec4 p02 = { -sqrt(2) / 2 * r,sqrt(2) / 2 * r,0,1 };
		Vec4 p03 = { 0,r,0,1 };
		Vec4 p04 = { 0,0,0,1 };
		Vec4 p05 = { -r / 2,0,0,1 };
		Vec4 p06 = { -r / 2,r / 2,0,1 };
		Vec4 p07 = { 0,r / 2,0,1 };

		SL1.push_back(s0.getSpline(p01, p05));
		SL1.push_back(s0.getArcSpline(r, PI / 4, p01, p02));
		SL1.push_back(s0.getSpline(p02, p06));
		SL1.push_back(s0.getSpline(p05, p06));
		SplineSurface ss1;
		ss1.CoonsInterpolate(SL1);
		ss.push_back(ss1);

		SL2.push_back(s0.getSpline(p02, p06));
		SL2.push_back(s0.getSpline(p06, p07));
		SL2.push_back(s0.getSpline(p07, p03));
		SL2.push_back(s0.getArcSpline(r, PI / 4, p02, p03));
		ss1.CoonsInterpolate(SL2);
		ss.push_back(ss1);

		RWGeometric rwg;
		varray<NurbsLine> nl;

		nl.push_back(NurbsTrans::SplineToCnurbsline(s0.getSpline(p02, p03)));
		nl.push_back(NurbsTrans::SplineToCnurbsline(s0.getSpline(p02, p06)));
		nl.push_back(NurbsTrans::SplineToCnurbsline(s0.getSpline(p06, p07)));
		nl.push_back(NurbsTrans::SplineToCnurbsline(s0.getSpline(p07, p03)));
		rwg.WriteNurbsLine("SplineTest.txt", nl);

		SL3.push_back(s0.getSpline(p05, p04));
		SL3.push_back(s0.getSpline(p04, p07));
		SL3.push_back(s0.getSpline(p06, p07));
		SL3.push_back(s0.getSpline(p05, p06));
		ss1.CoonsInterpolate(SL3);
		ss.push_back(ss1);

		return ss;
	}

	//扇形拉伸
	varray<SplineVolume> get_Model01(double r)
	{
		varray<SplineVolume> SV;
		SV = m.CreatSweepVol(quadrant(r), r / 4, 3);
		return SV;
	}

	varray<SplineSurface> rec(double d, double l)
	{
		Vec4 p01 = { -l / 2 - d,-l / 2,0,1 };
		Vec4 p02 = { -l / 2 - d,l / 2,0,1 };
		Vec4 p03 = { -l / 2,l / 2,0,1 };
		Vec4 p04 = { -l / 2,-l / 2,0,1 };

		varray<SplineSurface>ss;
		varray<Spline> SL1;
		SL1.push_back(s0.getSpline(p01, p02));
		SL1.push_back(s0.getSpline(p02, p03));
		SL1.push_back(s0.getSpline(p04, p03));
		SL1.push_back(s0.getSpline(p01, p04));
		SplineSurface ss1;
		ss1.CoonsInterpolate(SL1);
		ss.push_back(ss1);
		return ss;
	}
	//拉伸
	varray<SplineVolume> get_Model02(double d, double l)
	{
		varray<SplineVolume> SV1;
		varray<SplineVolume> SV2;
		varray<SplineVolume> SV;
		SV1 = m.CreatSweepVol(rec(d, l), d, 3);
		SV2 = m.CreatSweepVol(rec(d, d), d, 3);
		for (varray<SplineVolume>::iterator it = SV1.begin(); it != SV1.end(); it++) {
			SV.push_back(*it);
		}

		m.Trans(SV2, l / 2 - d / 2, -1);
		m.Trans(SV2, l / 2 + d / 2, -2);
		for (varray<SplineVolume>::iterator it = SV2.begin(); it != SV2.end(); it++) {
			SV.push_back(*it);
		}

		m.Rolate(SV1, PI / 2, 3);
		for (varray<SplineVolume>::iterator it = SV1.begin(); it != SV1.end(); it++) {
			SV.push_back(*it);
		}

		m.Rolate(SV1, PI / 2, 3);
		for (varray<SplineVolume>::iterator it = SV1.begin(); it != SV1.end(); it++) {
			SV.push_back(*it);
		}

		m.Trans(SV2, l + d, 1);
		for (varray<SplineVolume>::iterator it = SV2.begin(); it != SV2.end(); it++) {
			SV.push_back(*it);
		}
		m.Rolate(SV, PI / 2 * 3, 1);
		m.Trans(SV, l / 2 + d, 1);
		m.Trans(SV, l / 2, 3);
		return SV;
	}

private:

	Model_Solution m;
	double w = cos(PI / 4);
	Spline0 s0;
};

class YuYan {
public:
	Gear_Straight gs;
	RWGeometric rwg;
	Model_Solution m;

public:
	//完整齿轮
	void gear() {
		rwg.WriteSplineVolume("E:\\Model\\YuYanModel\\CompleteGear.txt", gs.getGear());
	}
	//单个齿轮啮合
	void singleGear()
	{
		rwg.WriteSplineSurface("E:\\Model\\WcWfFile\\GearSurface.txt", gs.test());
	}

	//节点细化
	void resetKnots() {
		varray<double> u;
		varray<double> v;
		varray<double> w;
		u.resize(2);
		v.resize(2);
		w.resize(2);
		varray<SplineVolume> sv;
		rwg.ReadSplineVolume("E:\\Model\\YuYanModel\\Model_yy.txt", sv);
		for (auto &i : sv) {
			u.clear();
			v.clear();
			w.clear();
			u.push_back(0.5);
			u.push_back(0.5);
			v.push_back(0.5);
			v.push_back(0.5);
			w.push_back(0.5);
			w.push_back(0.5);
			i.KnotsRefine(u, v, w);
		}
		rwg.WriteSplineVolume("E:\\Model\\YuYanModel\\Model_yy.txt1", sv);
	}

	//双齿轮啮合
	void doubleGear()
	{
		double len1;
		double len2;
		double len3;
		double len4;
		gs.singleGear(len1, len2, len3, len4);
		varray<SplineSurface> ss = gs.Gear;
		varray<SplineSurface> temp = ss;
		m.Rolate(temp, gs.getBetak(), 3);
		m.Trans(ss, gs.getDk() / 2, -2);
		m.Trans(temp, gs.getDk() / 2, 2);
		m.Trans(ss, len1 + len2, -2);
		m.Trans(temp, len1 + len2, 2);
		m.Trans(temp, len1, -2);
		m.Trans(temp, 0.5, -2);
		for (auto &i : temp) {
			ss.push_back(i);
		}
		//将齿轮移动到原点位置

		rwg.WriteSplineSurface("E:\\Model\\WcWfFile\\ChackSurface.txt", ss);
	}

	void completeGear()
	{
		double len1;
		double len2;
		double len3;
		double len4;
		gs.singleGear(len1, len2, len3, len4);
		varray<SplineVolume> SV = gs.getGear();
		varray<SplineVolume> temp;
		temp = SV;
		m.Rolate(SV, gs.getBetak(), 3);
		m.Trans(SV, len1 + len2, -2);
		m.Trans(SV, gs.getDk() / 2, -2);
		m.Trans(SV, 0.1, -1);

		m.Trans(temp, gs.getDk() / 2, 2);
		//m.Rolate(temp, gs.getBetak(), 3);
		m.Trans(temp, len1 + len2, 2);
		m.Trans(temp, len1, -2);
		m.Trans(temp, 0.5, -2);
		for (auto &i : temp) {
			SV.push_back(i);
		}
		rwg.WriteSplineVolume("E:\\Model\\YuYanModel\\CompleteGear.txt", SV);
	}

	varray<SplineSurface> InnerCircle(double R, double r)
	{
		double angle1;
		double angle2;

		varray<SplineVolume> SV;
		varray<SplineVolume> temp;
		varray<Spline> S;
		SplineSurface ss;
		Spline0 s;
		double len1, len2, len3, len4, len5;

		double angle = gs.getBetak();

		angle1 = angle * 2;
		angle2 = angle * 3;
		varray<SplineSurface> SS1;
		varray<SplineSurface> SS;
		varray<SplineSurface> TempSurface;
		//获得一些齿轮的尺寸数据
		Vec4 p0 = { 0,0,0,1 };
		Vec4 p1 = { 0,R,0,1 };
		Vec4 p2 = { 0,r,0,1 };
		Vec4 p3;
		Vec4 p4;
		Vec4 p5;
		Vec4 p6;
		Vec4 p7;
		Vec4 p8;
		Vec4 p9;
		Vec4 p10;
		Vec4 p11;
		Vec4 p12;
		Spline s1;
		Spline s2;
		Spline s3;
		Spline s4;
		Spline s5;
		Spline s6;
		Spline s7;
		Spline s8;
		Spline s9;
		Spline s10;
		Spline s11;
		Spline s12;
		Spline s13;
		Spline s14;
		Spline s15;
		Spline s16;

		//获得关键点
		s1 = s.getSpline(p2, p1);

		m.Rolate(s1, angle, 3);
		p3 = s1.m_CtrlPts[s1.m_CtrlPts.size() - 1];
		p4 = s1.m_CtrlPts[0];

		m.Rolate(s1, angle1, 3);
		p5 = s1.m_CtrlPts[s1.m_CtrlPts.size() - 1];
		p6 = s1.m_CtrlPts[0];

		m.Rolate(s1, angle2, 3);
		p7 = s1.m_CtrlPts[s1.m_CtrlPts.size() - 1];
		p8 = s1.m_CtrlPts[0];

		m.Rolate(s1, PI / 2 - angle - angle1 - angle2, 3);
		p9 = s1.m_CtrlPts[s1.m_CtrlPts.size() - 1];
		p10 = s1.m_CtrlPts[0];

		m.Rolate(s1, PI / 2, 3);
		p11 = s1.m_CtrlPts[s1.m_CtrlPts.size() - 1];
		p12 = s1.m_CtrlPts[0];

		//生成必要曲线
		s1 = s.getSpline(p2, p1);
		s2 = s.getSpline(p4, p3);
		s3 = s.getSpline(p6, p5);
		s4 = s.getSpline(p8, p7);
		s5 = s.getSpline(p9, p10);
		s6 = s.getSpline(p11, p12);

		s7 = s.getArcSpline(r, angle, p4, p2);
		s9 = s.getArcSpline(r, angle1, p6, p4);
		s11 = s.getArcSpline(r, angle1 + angle2, p8, p4);
		//s13 = s.getArcSpline(r, PI / 2 - angle - angle1 - angle2, p10, p8);
		s13 = s.getArcSpline(r, PI / 2 - angle, p10, p4);
		s15 = s.getArcSpline(r, PI / 2, p12, p10);

		s8 = s.getArcSpline(R, angle, p3, p1);
		s10 = s.getArcSpline(R, angle1, p5, p3);
		s12 = s.getArcSpline(R, angle1 + angle2, p7, p3);
		//s14 = s.getArcSpline(R, PI / 2 - angle - angle1 - angle2, p9, p7);

		s14 = s.getArcSpline(R, PI / 2 - angle, p9, p3);
		s16 = s.getArcSpline(R, PI / 2, p11, p9);

		S.clear();
		S.push_back(s1);
		S.push_back(s8);
		S.push_back(s2);
		S.push_back(s7);
		ss.CoonsInterpolate(S);
		SS.push_back(ss);

		/*S.clear();
		S.push_back(s2);
		S.push_back(s10);
		S.push_back(s3);
		S.push_back(s9);
		ss.CoonsInterpolate(S);
		SS.push_back(ss);*/

		/*S.clear();
		S.push_back(s2);
		S.push_back(s12);
		S.push_back(s4);
		S.push_back(s11);
		ss.CoonsInterpolate(S);
		SS.push_back(ss);*/

		S.clear();
		S.push_back(s2);
		S.push_back(s14);
		S.push_back(s5);
		S.push_back(s13);
		ss.CoonsInterpolate(S);
		SS.push_back(ss);

		S.clear();
		S.push_back(s5);
		S.push_back(s16);
		S.push_back(s6);
		S.push_back(s15);
		ss.CoonsInterpolate(S);
		SS.push_back(ss);

		m.MirrorSufaces(SS, TempSurface, 1);

		for (auto &i : TempSurface) {
			SS.push_back(i);
		}

		return SS;
	}

	varray<Spline> getPath()
	{
		Spline path;
		varray<Spline> S;
		Spline s;
		Spline0 s0;
		Vec4 p;
		Vec4 p1 = { 0,0,0,1 };
		Vec4 p2 = { -446.4,0,0,1 };
		double angle = PI / 180.0*5.8;
		double angle1 = angle / 5 * 2;
		double angle2 = angle / 5 * 2;
		double angle3 = angle / 5;
		s = s0.getSpline(p1, p2);
		//S.push_back(s);

		m.Rolate(s, angle1, 2);
		p = s.m_CtrlPts[2];
		path = s0.getArcSpline(p2, p, p1);
		m.Trans(path, 446.4, 1);
		m.Rolate(path, PI / 2, 3);
		S.push_back(path);

		m.Rolate(s, angle2, 2);
		p = s.m_CtrlPts[2];
		path = s0.getArcSpline(p2, p, p1);
		m.Trans(path, 446.4, 1);
		m.Rolate(path, PI / 2, 3);
		S.push_back(path);

		m.Rolate(s, angle3, 2);
		p = s.m_CtrlPts[2];
		path = s0.getArcSpline(p2, p, p1);
		m.Trans(path, 446.4, 1);
		m.Rolate(path, PI / 2, 3);
		S.push_back(path);

		rwg.WriteSpline("E:\\Model\\YuYanModel\\ChackLine.txt", S);
		return S;
	}

	//齿轮轴
	varray<SplineVolume> gearShaft(double r)
	{
		varray<SplineVolume> SV;
		varray<SplineVolume> SV1;
		varray<SplineVolume> temp;
		varray<Spline> S;
		SplineSurface ss;
		Spline0 s;
		double len1, len2, len3, len4, len5;
		varray<SplineSurface> SS1;
		varray<SplineSurface> SS;
		varray<SplineSurface> TempSurface;
		varray<SplineSurface> TempSurface0;
		//获得一些齿轮的尺寸数据
		gs.getData(len1, len2, len3, len4, len5);

		//获得单齿轮
		SS = gs.singleGear(len1, len2, len3, len4);

		rwg.WriteSplineSurface("E:\\Model\\YuYanModel\\ChackSurface1.txt", SS);
		TempSurface = this->InnerCircle(gs.getDk() / 2 + len5,  gs.getDk() / 2);
		rwg.WriteSplineSurface("E:\\Model\\YuYanModel\\ChackSurface1.txt", TempSurface);

		//最外层
		for (int i = 1; i < TempSurface.size(); i++) {
			if (i == TempSurface.size() / 2) {
				continue;
			}
			else {
				SS.push_back(TempSurface[i]);
			}
		}

		//中间层(靠近外层)
		TempSurface = this->InnerCircle(gs.getDk() / 2, r);
		for (auto &i : TempSurface) {
			SS.push_back(i);
		}

		//中间层(靠近内层)
		TempSurface0 = this->InnerCircle(r, r - 4);
		for (auto &i : TempSurface0) {
			SS.push_back(i);
		}
		//最内层
		TempSurface = this->InnerCircle(r - 1, 4);
		for (auto &i : TempSurface) {
			//SS.push_back(i);
		}

		rwg.WriteSplineSurface("E:\\Model\\YuYanModel\\GearShaftSurface.txt", SS);

		varray<SplineVolume> chack;
		//拉伸中间部分
		temp = m.CreatSweepVol(SS, 10, 3);
		for (auto &i : temp) {
			SV.push_back(i);
			chack.push_back(i);
		}

		double  l = 5;

		//拉伸下面部分

		varray<Spline> sweepPath = this->getPath();

		varray<SplineSurface> tempSurface;

		temp = this->CreatSweepVol1(TempSurface, sweepPath[0]);

		temp = m.CreatSweepVol(TempSurface, l * 9 / 2, -3);
		for (auto &i : temp) {
			SV.push_back(i);
			chack.push_back(i);
		}
		m.Trans(TempSurface, l * 9 / 2, -3);

		temp = m.CreatSweepVol(TempSurface, l * 9 / 2, -3);
		for (auto &i : temp) {
			SV.push_back(i);
			chack.push_back(i);
		}
		m.Trans(TempSurface, l * 9 / 2, -3);

		//第二段
		temp = m.CreatSweepVol(TempSurface, l, -3);
		for (auto &i : temp) {
			SV.push_back(i);
			chack.push_back(i);
		}
		m.Trans(TempSurface, l, -3);

		//第三段
		temp = m.CreatSweepVol(TempSurface, l * 2, -3);
		for (auto &i : temp) {
			SV.push_back(i);
			chack.push_back(i);
		}

		//拉伸下面部分（外壳）

		//第一段
		temp = m.CreatSweepVol(TempSurface0, l * 9 / 2, -3);
		for (auto &i : temp) {
			SV.push_back(i);
			chack.push_back(i);
		}
		m.Trans(TempSurface0, l * 9 / 2, -3);

		temp = m.CreatSweepVol(TempSurface0, l * 9 / 2, -3);
		for (auto &i : temp) {
			SV.push_back(i);
			chack.push_back(i);
		}
		m.Trans(TempSurface0, l * 9 / 2, -3);

		//第二段
		//m.Trans(TempSurface0, l, -3);

		temp = m.CreatSweepVol(TempSurface0, l, -3);
		for (auto &i : temp) {
			SV.push_back(i);
			chack.push_back(i);
		}

		//拉伸上面部分
		m.Trans(TempSurface, l * 10 + 10, 3); //移动到拉伸位置

		//第一段
		temp = m.CreatSweepVol(TempSurface, l * 9 / 2, 3);
		for (auto &i : temp) {
			SV.push_back(i);
			chack.push_back(i);
		}
		m.Trans(TempSurface, l * 9 / 2, 3);

		temp = m.CreatSweepVol(TempSurface, l * 9 / 2, 3);
		for (auto &i : temp) {
			SV.push_back(i);
			chack.push_back(i);
		}
		m.Trans(TempSurface, l * 9 / 2, 3);

		//第二段
		temp = m.CreatSweepVol(TempSurface, l, 3);
		for (auto &i : temp) {
			SV.push_back(i);
			chack.push_back(i);
		}
		m.Trans(TempSurface, l, 3);

		//第三段
		temp = m.CreatSweepVol(TempSurface, l * 2, 3);
		for (auto &i : temp) {
			SV.push_back(i);
			chack.push_back(i);
		}

		//拉伸上面面部分（外壳）
		m.Trans(TempSurface0, l * 9 + 10, 3);//移动到拉伸位置

		//第一段
		temp = m.CreatSweepVol(TempSurface0, l * 9 / 2, 3);
		for (auto &i : temp) {
			SV.push_back(i);
			chack.push_back(i);
		}
		m.Trans(TempSurface0, l * 9 / 2, 3);

		temp = m.CreatSweepVol(TempSurface0, l * 9 / 2, 3);
		for (auto &i : temp) {
			SV.push_back(i);
			chack.push_back(i);
		}
		m.Trans(TempSurface0, l * 9 / 2, 3);

		//第二段
		//m.Trans(TempSurface0, l, 3);

		temp = m.CreatSweepVol(TempSurface0, l, 3);
		for (auto &i : temp) {
			SV.push_back(i);
			chack.push_back(i);
		}

		rwg.WriteSplineVolume("E:\\Model\\YuYanModel\\ChackGearShaftVolume.txt", chack);

		////****另一个齿轮轴
		//m.Trans(TempSurface0, l * 10, -3);
		//m.Trans(TempSurface, l * 10, -3);
		////拉伸中间部分
		//temp = m.CreatSweepVol(SS, 10, 3);
		//for (auto &i : temp) {
		//	SV1.push_back(i);
		//	chack.push_back(i);
		//}

		////拉伸短轴部分

		//for (int i = 0; i < 6; i++) {
		//	temp = m.CreatSweepVol(TempSurface, l, 3);
		//	for (auto &i : temp) {
		//		SV1.push_back(i);
		//		chack.push_back(i);
		//	}
		//	m.Trans(TempSurface, l, 3);
		//}
		////拉伸短轴部分（外壳）
		//for (int i = 0; i < 3; i++) {
		//	temp = m.CreatSweepVol(TempSurface0, l, 3);
		//	for (auto &i : temp) {
		//		SV1.push_back(i);
		//		chack.push_back(i);
		//	}
		//	m.Trans(TempSurface0, l, 3);
		//}

		//m.Trans(TempSurface0, l, 3);

		//for (int i = 0; i < 2; i++) {
		//	temp = m.CreatSweepVol(TempSurface0, l, 3);
		//	for (auto &i : temp) {
		//		SV1.push_back(i);
		//		chack.push_back(i);
		//	}
		//	m.Trans(TempSurface0, l, 3);
		//}

		////拉伸上面部分
		//m.Trans(TempSurface, l * 6 + 10, -3);
		//for (int i = 0; i < 12; i++) {
		//	temp = m.CreatSweepVol(TempSurface, l, -3);
		//	for (auto &i : temp) {
		//		SV1.push_back(i);
		//		chack.push_back(i);
		//	}
		//	m.Trans(TempSurface, l, -3);
		//}

		//m.Trans(TempSurface0, l * 6 + 10, -3);

		////拉伸上面面部分（外壳）
		//for (int i = 0; i < 9; i++) {
		//	temp = m.CreatSweepVol(TempSurface0, l, -3);
		//	for (auto &i : temp) {
		//		SV1.push_back(i);
		//		chack.push_back(i);
		//	}
		//	m.Trans(TempSurface0, l, -3);
		//}

		//m.Trans(TempSurface0, l, -3);

		//for (int i = 0; i < 2; i++) {
		//	temp = m.CreatSweepVol(TempSurface0, l, -3);
		//	for (auto &i : temp) {
		//		SV1.push_back(i);
		//		chack.push_back(i);
		//	}
		//	m.Trans(TempSurface0, l, -3);
		//}
		//temp = SV1;

		temp = SV;
		m.Rolate(temp, PI, 2);
		m.Trans(temp, 10, 3);

		//m.Rolate(SV, PI / 1800 * 9.7, 3);
		m.Rolate(temp, PI / 1800 * 9.7, 3);

		m.Rolate(temp, PI, 3);
		m.Trans(temp, gs.getDk() + len2 * 2 + len1, 2);
		m.Trans(temp, 0.181, 2);
		m.Trans(temp, len3, 1);

		for (auto &i : temp) {
			SV.push_back(i);
		}

		cout << len5 + gs.getDk() << endl;
		rwg.WriteSplineVolume("E:\\Model\\YuYanModel\\GearShaftVolume.txt", SV);
		//this->resetKnots();
		return SV;
	}
	varray<SplineVolume> gearShaft1(double r)
	{
		varray<SplineVolume> SV;
		varray<SplineVolume> SV1;
		varray<SplineVolume> temp;
		varray<Spline> S;
		SplineSurface ss;
		Spline0 s;
		double len1, len2, len3, len4, len5;
		varray<SplineSurface> SS1;
		varray<SplineSurface> SS;
		varray<SplineSurface> TempSurface;
		varray<SplineSurface> TempSurface0;
		//获得一些齿轮的尺寸数据
		gs.getData(len1, len2, len3, len4, len5);

		//获得但齿轮
		SS = gs.singleGear(len1, len2, len3, len4);

		rwg.WriteSplineSurface("E:\\Model\\YuYanModel\\ChackSurface1.txt", SS);
		TempSurface = this->InnerCircle(gs.getDk() / 2 + len5, gs.getDk() / 2);
		rwg.WriteSplineSurface("E:\\Model\\YuYanModel\\ChackSurface1.txt", TempSurface);

		//最外层
		for (int i = 1; i < TempSurface.size(); i++) {
			if (i == TempSurface.size() / 2) {
				continue;
			}
			else {
				SS.push_back(TempSurface[i]);
			}
		}

		//最内层
		TempSurface = this->InnerCircle(r - 1, 4);
		for (auto &i : TempSurface) {
			//SS.push_back(i);
		}

		rwg.WriteSplineSurface("E:\\Model\\YuYanModel\\GearShaftSurface.txt", SS);

		varray<SplineVolume> chack;
		//拉伸中间部分
		temp = m.CreatSweepVol(SS, 5, 3);
		for (auto &i : temp) {
			SV.push_back(i);
			chack.push_back(i);
		}

		m.Trans(SS, 5, 3);
		temp = m.CreatSweepVol(SS, 5, 3);
		for (auto &i : temp) {
			SV.push_back(i);
			chack.push_back(i);
		}

		double  l = 5;

		//拉伸下面部分

		//第一段
		temp = m.CreatSweepVol(TempSurface0, l * 5, -3);
		for (auto &i : temp) {
			SV.push_back(i);
			chack.push_back(i);
		}
		m.Trans(TempSurface0, l * 5, -3);

		temp = m.CreatSweepVol(TempSurface0, l * 5, -3);
		for (auto &i : temp) {
			SV.push_back(i);
			chack.push_back(i);
		}
		

		//拉伸上面部分
		m.Trans(TempSurface0, l * 5 + 10, 3);//移动到拉伸位置
		//第一段
		temp = m.CreatSweepVol(TempSurface0, l * 5, 3);
		for (auto &i : temp) {
			SV.push_back(i);
			chack.push_back(i);
		}
		m.Trans(TempSurface0, l * 5, 3);

		temp = m.CreatSweepVol(TempSurface0, l * 5, 3);
		for (auto &i : temp) {
			SV.push_back(i);
			chack.push_back(i);
		}

		rwg.WriteSplineVolume("E:\\Model\\YuYanModel\\ChackGearShaftVolume.txt", chack);

		////****另一个齿轮轴
		//m.Trans(TempSurface0, l * 10, -3);
		//m.Trans(TempSurface, l * 10, -3);
		////拉伸中间部分
		//temp = m.CreatSweepVol(SS, 10, 3);
		//for (auto &i : temp) {
		//	SV1.push_back(i);
		//	chack.push_back(i);
		//}

		////拉伸短轴部分

		//for (int i = 0; i < 6; i++) {
		//	temp = m.CreatSweepVol(TempSurface, l, 3);
		//	for (auto &i : temp) {
		//		SV1.push_back(i);
		//		chack.push_back(i);
		//	}
		//	m.Trans(TempSurface, l, 3);
		//}
		////拉伸短轴部分（外壳）
		//for (int i = 0; i < 3; i++) {
		//	temp = m.CreatSweepVol(TempSurface0, l, 3);
		//	for (auto &i : temp) {
		//		SV1.push_back(i);
		//		chack.push_back(i);
		//	}
		//	m.Trans(TempSurface0, l, 3);
		//}

		//m.Trans(TempSurface0, l, 3);

		//for (int i = 0; i < 2; i++) {
		//	temp = m.CreatSweepVol(TempSurface0, l, 3);
		//	for (auto &i : temp) {
		//		SV1.push_back(i);
		//		chack.push_back(i);
		//	}
		//	m.Trans(TempSurface0, l, 3);
		//}

		////拉伸上面部分
		//m.Trans(TempSurface, l * 6 + 10, -3);
		//for (int i = 0; i < 12; i++) {
		//	temp = m.CreatSweepVol(TempSurface, l, -3);
		//	for (auto &i : temp) {
		//		SV1.push_back(i);
		//		chack.push_back(i);
		//	}
		//	m.Trans(TempSurface, l, -3);
		//}

		//m.Trans(TempSurface0, l * 6 + 10, -3);

		////拉伸上面面部分（外壳）
		//for (int i = 0; i < 9; i++) {
		//	temp = m.CreatSweepVol(TempSurface0, l, -3);
		//	for (auto &i : temp) {
		//		SV1.push_back(i);
		//		chack.push_back(i);
		//	}
		//	m.Trans(TempSurface0, l, -3);
		//}

		//m.Trans(TempSurface0, l, -3);

		//for (int i = 0; i < 2; i++) {
		//	temp = m.CreatSweepVol(TempSurface0, l, -3);
		//	for (auto &i : temp) {
		//		SV1.push_back(i);
		//		chack.push_back(i);
		//	}
		//	m.Trans(TempSurface0, l, -3);
		//}
		//temp = SV1;

		temp = SV;
		m.Rolate(temp, PI, 2);
		m.Trans(temp, 10, 3);

		m.Rolate(SV, PI / 1800 * 9.7, 3);
		m.Rolate(temp, PI / 1800 * 9.7, 3);

		m.Rolate(temp, PI, 3);
		m.Trans(temp, gs.getDk() + len2 * 2 + len1, 2);
		m.Trans(temp, 0.181, 2);
		m.Trans(temp, len3, 1);

		for (auto &i : temp) {
			SV.push_back(i);
		}

		cout << len5 + gs.getDk() << endl;
		rwg.WriteSplineVolume("E:\\Model\\YuYanModel\\GearShaftVolume.txt", SV);
		//this->resetKnots();
		return SV;
	}


	//全齿齿轮
	varray<SplineVolume> completeGearShaft(double r)
	{
		varray<SplineVolume> SV;
		varray<SplineVolume> temp;
		varray<SplineSurface>SS;
		varray<SplineSurface>allSurf;
		double len1, len2, len3, len4, len5;
		gs.setDk(2 * r);
		
		varray<SplineSurface>TempSurface = this->InnerCircle(r, r-2);
		SplineSurface s = TempSurface[0];
		double angle = gs.getBetak();
		TempSurface.clear();
		for (int i = 0; i < PI / angle*2; i++)
		{
			TempSurface.push_back(s);
			m.Rolate(s, angle, 3);
		}
		for (auto &i : TempSurface)
		{
			//allSurf.push_back(i);
		}
		//获得单齿轮
		SS = gs.singleGear(len1, len2, len3, len4);
		for (int i = 0; i < PI / angle; i++)
		{
			for (auto &i : SS)
			{
				allSurf.push_back(i);
				m.Rolate(i, angle * 2, 3);
			}
			
		}

		rwg.WriteSplineSurface("E:\\Model\\YuYanModel\\CompleteGear.txt", allSurf);
		double  l = 5;
		//拉伸中间部分
		varray<SplineVolume> SV1;
		temp = m.CreatSweepVol(allSurf, 10, 3);
		for (auto &i : temp) {
			SV.push_back(i);
			SV1.push_back(i);
		}
		rwg.WriteSplineVolume("E:\\Model\\YuYanModel\\GearShaftVolumeMidPart1.txt", temp);
		temp = m.CreatSweepVol(TempSurface, 10, 3);
		for (auto &i : temp) {
			SV.push_back(i);
			SV1.push_back(i);
		}
		rwg.WriteSplineVolume("E:\\Model\\YuYanModel\\GearShaftVolumeMidPart2.txt", temp);
		//拉伸下面部分
		//第一段
		varray<SplineVolume> SV2;
		temp = m.CreatSweepVol(TempSurface, l * 5, -3);
		for (auto &i : temp) {
			SV.push_back(i);
			SV2.push_back(i);
		}
		m.Trans(TempSurface, l * 5, -3);

		temp = m.CreatSweepVol(TempSurface, l * 5, -3);
		for (auto &i : temp) {
			/*SV.push_back(i);
			SV2.push_back(i);*/
		}
		rwg.WriteSplineVolume("E:\\Model\\YuYanModel\\GearShaftVolumePart2.txt", SV2);

		//拉伸上面部分
		varray<SplineVolume> SV3;
		m.Trans(TempSurface, l * 5 + 10, 3);//移动到拉伸位置
		//第一段
		temp = m.CreatSweepVol(TempSurface, l * 5, 3);
		for (auto &i : temp) {
			SV.push_back(i);
			SV3.push_back(i);
		}
		m.Trans(TempSurface, l * 5, 3);

		temp = m.CreatSweepVol(TempSurface, l * 5, 3);
		for (auto &i : temp) {
			/*SV.push_back(i);
			SV3.push_back(i);*/
		}
		rwg.WriteSplineVolume("E:\\Model\\YuYanModel\\GearShaftVolumePart3.txt", SV3);
		temp = SV;
		m.Rolate(temp, PI, 2);
		m.Trans(temp, 10, 3);

		m.Rolate(SV, PI / 1800 * 9.7, 3);
		m.Rolate(temp, PI / 1800 * 9.7, 3);

		m.Rolate(temp, PI, 3);
		m.Trans(temp, gs.getDk() + len2 * 2 + len1, 2);
		m.Trans(temp, 0.181, 2);
		m.Trans(temp, len3, 1);

		//另一个啮合的此轮轴
		for (auto &i : temp) {
			//SV.push_back(i);
		}

		cout << len5 + gs.getDk() << endl;
		rwg.WriteSplineVolume("E:\\Model\\YuYanModel\\GearShaftVolume.txt", SV);
		rwg.WriteSplineVolume("E:\\Model\\YuYanModel\\CompleteGearShaftVolume.txt", SV);
		return SV;
	}

	varray<SplineVolume> CreatSweepVol1(const varray<SplineSurface>& NSf, Spline p)
	{
		varray<SplineVolume> NVS;

		for (int i = 0; i < NSf.size(); i++)
		{
			Creat_Vol CV;
			CV.NL.ClearCtrlPoint();
			for (auto point : p.m_CtrlPts) {
				CV.NL.m_CtrlPts.push_back(point);
			}
			CV.NL.m_Degree = 2;
			CV.NL.m_Knots.push_back(0);
			CV.NL.m_Knots.push_back(0);
			CV.NL.m_Knots.push_back(0);
			CV.NL.m_Knots.push_back(1);
			CV.NL.m_Knots.push_back(1);
			CV.NL.m_Knots.push_back(1);

			CV.NV.CreateTransSweepSplineVolume(CV.NL, NSf[i]);

			NVS.push_back(CV.NV);
		}
		return NVS;
	}

	void Change() {
		varray<SplineVolume> SV;
		varray<SplineVolume> SV1;
		varray<SplineVolume> SV2;
		rwg.ReadSplineVolume("E:\\Model\\YuYanModel\\gear.txt", SV);
		for (int i = 0; i < SV.size() / 2; ++i)
		{
			SV1.push_back(SV[i]);
		}

		m.Rolate(SV1, PI / 1800 * 4, 3);
		for (int i = SV.size() / 2; i < SV.size(); ++i)
		{
			SV2.push_back(SV[i]);
		}

		m.Trans(SV2, 0.15, -1);
		m.Trans(SV2, 0.08, -2);
		SV.clear();
		for (auto &i : SV1) {
			SV.push_back(i);
		}
		for (auto &i : SV2) {
			SV.push_back(i);
		}
		rwg.WriteSplineVolume("E:\\Model\\YuYanModel\\ChackVolume.txt", SV);
	}
};


//操作测试类
class OperateTest {
public:
	Spline0 s0;
	Model_Solution m;
	RWGeometric rwg;
	PublicSolution ps;

public:
	void createVolume() 
	{
		SplineSurface S1;
		SplineSurface S2;
	}
	void quadSurface1()
	{
		varray<SplineSurface> SS, SS1;
		varray<SplineVolume> SV, SV1, SVS;

		rwg.ReadSplineSurface("E:\\Model\\TestModel\\fitBSurface.txt", SS1);
		for (auto &i : SS1) {
			i.KnotsRefineNum(1);
		}
		//圆柱
		for (auto i : SS1) {
			SS.push_back(i);
		}
		ps.littler(SS, 1.1, 0);
		rwg.WriteSplineSurface("E:\\Model\\TestModel\\ballSplineSurface2.txt", SS);

		SV = ps.loft2(SS1, SS);
		rwg.WriteSplineVolume("E:\\Model\\TestModel\\ballSplineVolume1.txt", SV);

		//镜像
		m.MirrorVols(SV, SV1, 3);
		for (auto i : SV1) {
			i.OrderCtrlPts(i);
			SV.push_back(i);
		}

		rwg.WriteSplineVolume("E:\\Model\\TestModel\\ballSplineVolumeAll.txt", SV);
		ps.outPutVTK(SV, "E:\\Model\\TestModel\\ballSplineVolumeAll.vtk");

		for (auto i : SS) {
			SS1.push_back(i);
		}
		rwg.WriteSplineSurface("E:\\Model\\TestModel\\ballSplineSurfaceAll.txt", SS1);
	}

	void MoveSurface()
	{
		varray<SplineSurface> SS;
		//SS
	}

	bool truncationStraightLine(Spline s, Vec4 point, varray<Spline>& result) {
		//获得方向向量
		Vec4 v1 = s.m_CtrlPts[0] - point;
		Vec4 v2 = point - s.m_CtrlPts[s.m_CtrlPts.size() - 1];

		//方向向量单位化
		v1 = v1.Normalize();
		v2 = v2.Normalize();

		//判断方向是否相同
		if (!JudgeTwoPointsCoincide(v1, v2)) {
			return false;
		}

		//方向相同说明该截断点在直线上
		result.push_back(s0.getSpline(s.m_CtrlPts[0], point));
		result.push_back(s0.getSpline(point, s.m_CtrlPts[s.m_CtrlPts.size() - 1]));
		return true;
	}

	bool truncationCurves(Spline s, Vec4 point, varray<Spline>& result) {
		return true;
	}

	bool nurbsCurveIntersect(const Spline crv1, const Spline crv2)
	{
		return false;
	}
};

/**
* @brief   : 给定曲线，确定曲线的相关参数
* @note    :
**/
class ResetSpline
{
public:

	/**
	* @brief   : 显示曲线的信息
	* @param[I]: none
	* @param[O]: none
	* @return  : none
	* @note    :
	**/
	void showData()
	{
		cout << "圆心角为：" << this->deta << endl;
		cout << "圆心坐标为：（" << this->o.x << " ," << this->o.y << " ," << this->o.z << " ," << "）" << endl;
		cout << "圆弧对应的半径为：" << this->r << endl;
	}

	/**
	* @brief   : 输入三点
	* @param[I]: p1 (x1,y1)
	* @param[I]: p2 (x2,y2)
	* @param[I]: p3 (x3,y3)
	* @param[O]: none
	* @return  : none
	* @note    :
	**/
	void setThreePoints(Vec4 &p1, Vec4 &p2, Vec4 &p3)
	{
		//判断是否三点共线
		if ((p2.x - p1.x)*(p3.y - p1.y) - (p3.x - p1.x)*(p2.y - p1.y) == 0)
		{
			cout << "->三点共线，无法确定一个圆" << endl;;
			system("pause");
			abort();
		}
		m_p1 = p1;
		m_p2 = p2;
		m_p3 = p3;
		is_set3Points = true;
	}

	/**
	* @brief   : 三点定圆
	* @param[I]: none
	* @param[O]: none
	* @return  : none
	* @note    : 输出圆心、半径、圆方程
	**/
	void defineCircle()
	{
		if (!is_set3Points)
		{
			cout << "->请输入三个点！" << endl;
			system("pause");
			abort();
		}

		float kab, kbc, kac;	//三边斜率
		kab = (m_p2.y - m_p1.y) / (m_p2.x - m_p1.x);
		kbc = (m_p3.y - m_p2.y) / (m_p3.x - m_p2.x);
		kac = (m_p3.y - m_p1.y) / (m_p3.x - m_p1.x);

		Vec4 Pab, Pbc, Pac;	//三边中点
		Pab.x = (m_p1.x + m_p2.x) / 2;
		Pab.y = (m_p1.y + m_p2.y) / 2;
		Pbc.x = (m_p2.x + m_p3.x) / 2;
		Pbc.y = (m_p2.y + m_p3.y) / 2;
		Pac.x = (m_p1.x + m_p3.x) / 2;
		Pac.y = (m_p1.y + m_p3.y) / 2;

		//矩阵方程Ga=d
		Matrix2f G;
		G << 1 / kab, 1, 1 / kbc, 1;
		Vector2f d;
		d << (Pab.y + 1 / kab * Pab.x), (Pbc.y + 1 / kbc * Pbc.x);
		Vector2f a;
		a = G.colPivHouseholderQr().solve(d);
		//圆半径
		r = sqrt(pow((a[0, 0] - m_p1.x), 2) + pow((a[1, 0] - m_p1.y), 2));
		o.x = a[0, 0];
		o.y = a[1, 0];
		cout << "圆心：" << "(" << o.x << ", " << o.y << ")" << endl;

		cout << "->圆方程：" << "(x - " << o.x << " )^2 + (y - " << o.y << " )^2 = " << r << "^2" << endl;
	}

	/**
	* @brief   : 给定曲线、圆心，求圆心角（xy平面曲线）
	* @param[I]: none
	* @param[0]: none
	* @return  : double 圆心角的弧度
	* @note    :
	**/
	double calculateAngle()
	{
		Vec4 s1 = p1 - o;
		Vec4 s2 = p3 - o;

		double angle1 = atan2(s1.y, s1.x);
		double angle2 = atan2(s2.y, s2.x);

		if (angle1 < 0)
		{
			angle1 = angle1 + PI * 2;
		}
		if (angle2 < 0)
		{
			angle2 = angle2 + PI * 2;
		}

		this->deta = abs(angle1 - angle2);

		return this->deta;
	}

	/**
	* @brief   : 给定三点，求夹角(二维)
	* @param[I]: Vec4 p1
	* @param[I]: Vec4 p2 交点
	* @param[I]: Vec4 p3
	* @param[0]: none
	* @return  : double 夹角的弧度
	* @note    :
	**/
	double calculateAngle(const Vec4 &p01, const Vec4 &p02, const Vec4 &p03)
	{
		Vec4 s1 = p01 - p02;
		Vec4 s2 = p03 - p02;

		double angle1 = atan2(s1.y, s1.x);
		double angle2 = atan2(s2.y, s2.x);
		if (angle1 < 0)
		{
			angle1 = angle1 + PI * 2;
		}
		if (angle2 < 0)
		{
			angle2 = angle2 + PI * 2;
		}

		return abs(angle1 - angle2);
	}

	/**
	* @brief   : 给定三点，求夹角（三维）
	* @param[I]: Vec4 p1
	* @param[I]: Vec4 p2 交点(中间点)
	* @param[I]: Vec4 p3
	* @param[0]: none
	* @return  : double 夹角的弧度
	* @note    :
	**/
	double calculateAngle3D(const Vec4 &p01, const Vec4 &p02, const Vec4 &p03)
	{
		Vec4 s1 = p01 - p02;
		Vec4 s2 = p03 - p02;

		double l1 = sqrt(pow(s1.x, 2) + pow(s1.y, 2) + pow(s1.z, 2));
		double l2 = sqrt(pow(s2.x, 2) + pow(s2.y, 2) + pow(s2.z, 2));

		double d = s1.x*s2.x + s1.y*s2.y + s1.z*s2.z;

		double angle = acos(d / (l1 * l2));
		return angle;
	}

	/**
	* @brief   : 给定曲线，求出圆心（xy平面曲线）
	* @param[I]: const Spline &S
	* @param[0]: none
	* @return  : bool
	* @note    :
	**/
	bool calculateCircleCenter()
	{
		//曲线上的三个点
		Vec4 p21 = p2 - p1;				//p1p2的方向向量
		Vec4 p23 = p2 - p3;				//p3p2的方向向量

		if (JudgeTwoPointsCoincide(p21.Normalize(), p23.Normalize()))
		{
			cout << "直线不存在圆心" << endl;
			return false;
		}

		double k21 = p21.y / p21.x;//直线p1p2的斜率
		double k23 = p23.y / p23.x;//直线p3p2的斜率

		o.y = (p3.x - p1.x + k23 * p3.y - k21 * p1.y) / (k23 - k21);//圆心计算公式
		o.x = p1.x + k21 * p1.y - k21 * o.y;
		return true;
	}

	/**
	* @brief   : 判断一点是否为在圆上
	* @param[I]: point 判断的点
	* @return  : true 在圆上 false 不在圆上
	* @note    : 根据园的方程判断是否在圆上
	**/
	bool is_PointOfCirlce(const Vec4 &point)
	{
		//是否在完整的圆上
		if (pow(point.x - o.x, 2) + pow(point.y - o.y, 2) - pow(r, 2) > 1e-3)
		{
			cout << "pow(point.x - o.x, 2) + pow(point.y-o.y, 2) - pow(r, 2) = " << pow(point.x - o.x, 2) + pow(point.y - o.y, 2) - pow(r, 2) << endl;
			return false;
		}

		//截断位置形成的两个曲线的弧度
		angle1 = calculateAngle(p1, o, point);
		angle2 = calculateAngle(point, o, p3);

		cout << "angle1：" << angle1 / PI * 180 << "°" << endl;
		cout << "angle2：" << angle2 / PI * 180 << "°" << endl;

		//是否在圆弧上
		if (angle1 > deta || angle2 > deta)
		{
			return false;
		}

		return true;
	}

	/**
	* @brief   : 移动曲线的位置
	* @param[I]: const Vec4 &point
	* @param[0]: none
	* @return  : void
	* @note    : 因Spline0构造的是以原点为圆心的曲线，因此需要吧各个点移动到以原点为圆心的对应位置上
	**/
	Vec4 MoveTo(const Vec4 &point)
	{
		Vec4 p;
		p.x = point.x - o.x;
		p.y = point.y - o.y;
		p.z = point.z - o.z;
		p.w = point.w;
		return p;
	}

	/**
	* @brief   : 指定位置重构截断曲线
	* @param[I]: const Vec4 &point		截断的位置
	* @param[0]: none
	* @return  : varray<Spline>			截断之后的两条（也可能是0、1条）
	* @note    :重构之前要判断该点的位置，位置不同，处理的方法也不同
	**/
	varray<Spline> cutSpline(const Vec4 &point)
	{
		varray<Spline> SS;
		//判断是否在曲线上
		if (!is_PointOfCirlce(point))
		{
			return SS;
		}

		//判断是否为曲线的两端
		if (JudgeTwoPointsCoincide(p1, point) || JudgeTwoPointsCoincide(p3, point))
		{
			SS.push_back(S);
			return SS;
		}

		//重构曲线
		SS.push_back(s0.getArcSpline(r, angle1, MoveTo(p1), MoveTo(point)));
		SS.push_back(s0.getArcSpline(r, angle2, MoveTo(point), MoveTo(p3)));
		Model_Solution m;
		m.Trans(SS, o.x, 1);
		m.Trans(SS, o.y, 2);
		return SS;
	}

	/**
	* @brief   : 含参构造函数
	* @param[I]: const Spline &S
	* @note    :
	**/
	ResetSpline(const Spline &S)
	{
		this->S = S;
		p1 = S.m_CtrlPts[0];
		p2 = S.m_CtrlPts[1];
		p3 = S.m_CtrlPts[2];
		calculateCircleCenter();
		calculateAngle();
		r = sqrt(pow(p1.x - o.x, 2) + pow(p1.y - o.y, 2));
	}

	/**
	* @brief   : 默认构造函数
	* @param[I]: none
	* @note    :
	**/
	ResetSpline() {}

	/**
	* @brief   : 计算矩阵的秩
	* @param[I]: MatrixXd G
	* @param[0]: none
	* @return  : int 矩阵的秩
	* @note    :
	**/
	int GetRank(MatrixXd G)
	{
		JacobiSVD<Eigen::MatrixXd> svd(G);
		std::cout << "A :\n" << G << std::endl;
		std::cout << "rank:\n" << svd.rank() << std::endl;
		return svd.rank();
	}

	/**
	* @brief   : 计算两点之间的距离
	* @param[I]: Vec4 v1, Vec4 v2 两个点
	* @param[0]: none
	* @return  : double 两点之间的距离
	* @note    :
	**/
	double getLength(Vec4 v1, Vec4 v2)
	{
		return sqrt(pow(v1.x - v2.x, 2) + pow(v1.y - v2.y, 2) + pow(v1.z - v2.z, 2));
	}

	/**
	* @brief   : 计算向量的长度
	* @param[I]: Vec4 v1 向量
	* @param[0]: none
	* @return  : double 向量的长度
	* @note    :
	**/
	double getLength(Vec4 v1)
	{
		return sqrt(pow(v1.x, 2) + pow(v1.y, 2) + pow(v1.z, 2));
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
		double lop = getLength(op);

		//向量单位化
		op /= lop;

		//计算曲线的弧度
		double angle = calculateAngle3D(v1, O, v3);
		cout << "夹角为：" << angle / PI * 180 << endl;

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

private:
	Spline S;
	Spline0 s0;
	double deta;				//圆心角
	double r;					//半径
	Vec4 o;						//圆心
	Vec4 p1, p2, p3;			//曲线的两个端点
	double angle1, angle2;
	Vec4 m_p1, m_p2, m_p3;		//输入的三点
	bool is_set3Points = false;	//是否输入三点
};

//边界类
class Boundary
{
public:
	varray<Spline> getSquare(double l, double d)
	{
		Vec4 p1 = { -l / 2,d / 2,0,1 };
		Vec4 p2 = { l / 2,d / 2,0,1 };
		Vec4 p3 = { l / 2,-d / 2,0,1 };
		Vec4 p4 = { -l / 2,-d / 2,0,1 };

		Spline s;
		Spline0 s0;

		varray<Spline> S;

		s = s0.getSpline(p1, p2);
		S.push_back(s);
		s = s0.getSpline(p2, p3);
		S.push_back(s);
		s = s0.getSpline(p3, p4);
		S.push_back(s);
		s = s0.getSpline(p4, p1);
		S.push_back(s);
		return S;
	}

	varray<Spline> getCircle(double r)
	{
		Vec4 p1 = { -r,0,0,1 };
		Vec4 p2 = { 0,r,0,1 };
		Vec4 p3 = { r,0,0,1 };
		Vec4 p4 = { 0,-r,0,1 };

		Spline s;
		Spline0 s0;

		varray<Spline> S;

		s = s0.getArcSpline(r, PI / 2, p1, p2);
		S.push_back(s);

		s = s0.getArcSpline(r, PI / 2, p2, p3);
		S.push_back(s);

		s = s0.getArcSpline(r, PI / 2, p3, p4);
		S.push_back(s);

		s = s0.getArcSpline(r, PI / 2, p4, p1);
		S.push_back(s);

		return S;
	}
};

//线(包括圆弧，用于四边剖分)
class Curve {
public:
	Vec4 v1, v2, v3;

	Curve() {};
	Curve(Vec4 &v1, Vec4 &v2) {
		this->v1 = v1;
		this->v2 = v2;
	};
	Curve(Vec4& v1, Vec4& v2, Vec4& v3) {
		this->v1 = v1;
		this->v2 = v2;
		this->v3 = v3;
	};
	~Curve() {
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

	/**
	* @brief   : 给定三点，求夹角
	* @param[I]: Vec4 p1
	* @param[I]: Vec4 p2 交点(中间点)
	* @param[I]: Vec4 p3
	* @param[0]: none
	* @return  : double 夹角的弧度
	* @note    :
	**/
	double calculateAngle3D(const Vec4 &p01, const Vec4 &p02, const Vec4 &p03)
	{
		Vec4 s1 = p01 - p02;
		Vec4 s2 = p03 - p02;

		double l1 = sqrt(pow(s1.x, 2) + pow(s1.y, 2) + pow(s1.z, 2));
		double l2 = sqrt(pow(s2.x, 2) + pow(s2.y, 2) + pow(s2.z, 2));

		double d = s1.x*s2.x + s1.y*s2.y + s1.z*s2.z;

		double angle = acos(d / (l1 * l2));
		return angle;
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

	Spline getSpline(Vec3 &v1, Vec3 &v2) {
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

class Model_Chong
{
public:
	varray<SplineSurface> getSquare(double l, double d)
	{
		Vec4 p1 = { 0,0,0,1 };
		Vec4 p2 = { 0,d,0,1 };
		Vec4 p3 = { l,d,0,1 };
		Vec4 p4 = { l,0,0,1 };

		varray<Spline> S;
		Spline0 s0;
		S.push_back(s0.getSpline(p1, p4));
		S.push_back(s0.getSpline(p4, p3));
		S.push_back(s0.getSpline(p2, p3));
		S.push_back(s0.getSpline(p1, p2));

		SplineSurface ss;
		ss.CoonsInterpolate(S);
		varray<SplineSurface> SS;
		SS.push_back(ss);
		return SS;
	}

	varray<SplineSurface> getSS1()
	{
		varray<SplineSurface> SS1;
		varray<SplineSurface> temp;
		temp = this->getSquare(5, 20);
		for (auto &i : temp)
		{
			SS1.push_back(i);
		}

		temp = this->getSquare(5, 15);
		m.Trans(temp, 20, 2);
		for (auto &i : temp)
		{
			SS1.push_back(i);
		}

		temp = this->getSquare(5, 20);
		m.Trans(temp, 35, 2);
		for (auto &i : temp)
		{
			SS1.push_back(i);
		}
		return SS1;
	}

	varray<SplineSurface> getSS2()
	{
		Model_Solution m;
		varray<SplineSurface> SS2;
		varray<SplineSurface> temp;
		temp = this->getSquare(2.5, 17.5);
		for (auto &i : temp)
		{
			SS2.push_back(i);
		}

		temp = this->getSquare(2.5, 20);
		m.Trans(temp, 17.5, 2);
		for (auto &i : temp)
		{
			SS2.push_back(i);
		}

		temp = this->getSquare(2.5, 17.5);
		m.Trans(temp, 37.5, 2);
		for (auto &i : temp)
		{
			SS2.push_back(i);
		}
		return SS2;
	}

	varray<SplineSurface> getAllSurface()
	{
		varray<SplineSurface> AllSurface;
		varray<SplineSurface> SS1 = this->getSS1();
		varray<SplineSurface> SS2 = this->getSS2();

		for (int i = 0; i < 11; i++)
		{
			for (auto &i : SS1)
			{
				AllSurface.push_back(i);
			}
			m.Trans(SS1, 5, 1);
		}

		return AllSurface;
	}

	varray<SplineSurface> getPartSurface()
	{
		varray<SplineSurface> PartSurface;
		varray<SplineSurface> SS1 = this->getSS1();

		for (int i = 0; i < 6; i++)
		{
			for (auto &i : SS1)
			{
				PartSurface.push_back(i);
			}
			m.Trans(SS1, 10, 1);
		}
		return PartSurface;
	}

	varray<SplineSurface> getBottomSurface()
	{
		varray<SplineSurface> BottomSurface;

		varray<SplineSurface> SS = this->getSquare(5, 15);

		m.Trans(SS, 20, 1);
		m.Trans(SS, 20, 2);
		for (int i = 0; i < 3; i++)
		{
			for (auto &i : SS)
			{
				BottomSurface.push_back(i);
			}
			m.Trans(SS, 5, 1);
		}
		return BottomSurface;
	}

	varray<SplineVolume>  getVolumeModel()
	{
		varray<SplineVolume>SV;
		varray<SplineVolume>temp;
		varray<SplineSurface> AllSurface = this->getAllSurface();
		varray<SplineSurface> PartSurface = this->getPartSurface();
		varray<SplineSurface> BottomSurface = this->getBottomSurface();

		temp = m.CreatSweepVol(AllSurface, 5, 3);
		for (auto &i : temp)
		{
			SV.push_back(i);
		}
		m.Trans(temp, 5, 3);
		for (auto &i : temp)
		{
			SV.push_back(i);
		}

		m.Trans(PartSurface, 10, 3);
		for (int i = 0; i < 4; i++)
		{
			temp = m.CreatSweepVol(PartSurface, 10, 3);
			for (auto &i : temp)
			{
				SV.push_back(i);
			}
			m.Trans(PartSurface, 10, 3);
		}

		temp = m.CreatSweepVol(BottomSurface, 5, -3);

		for (auto &i : temp)
		{
			SV.push_back(i);
		}

		return SV;
	}
public:
	Model_Solution m;
};

////连接线连接的两条曲线，以及连接线端点所在的曲线参数值
//struct AddLineData
//{
//	pair<int, double> pa1;
//	pair<int, double> pa2;
//	Spline s;
//};
//
////连接线对应的轮廓
//struct ContourData
//{
//	AddLineData adl;
//	//轮廓线编号
//	int contour1;
//	int contour2;
//};
class AddLine_Close
{
public:

	/**
	* @brief   : 获取两点之间的距离
	* @param[I]: 点p1
	* @param[I]: 点p2
	* @return  : 两点之间的距离
	* @note    :
	**/
	double towPointDistence(const Vec4 &p1, const Vec4 &p2)
	{
		double x = p1.x - p2.x;
		double y = p1.y - p2.y;
		double z = p1.z - p2.z;
		x = pow(x, 2);
		y = pow(y, 2);
		z = pow(z, 2);
		return sqrt(x + y + z);
	}

	bool if_StraightLine(const Spline &s)
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

	//测试用，可以删除
	void test_if_StraightLine()
	{
		Vec4 p3 = { 10,5,0,1 };
		Vec4 p4 = { 10,-5,0,1, };
		Spline0 s0;
		Spline s = s0.getSpline(p3, p4);
		if_StraightLine(s);
	}

	/**
	* @brief   : 用折线逼近曲线
	* @param[I]: 要逼近的曲线
	* @return  : 折线
	* @note    :在参数区间0~1之间取点，并构建直线，从而逼近曲线
	**/
	varray<Spline> CurveToLines(const Spline &s)
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
	* @brief   : 利用SISL库函数判断两曲线是否相交
	* @param[I]: 第一条曲线
	* @param[I]: 第二条曲线
	* @return  : 交点个数
	* @note    :
	**/
	int SplinesInterNum(const Spline &s1, const Spline &s2)
	{
		SISLCurve *curve1 = NurbsLineToSislLine(s1); /* Must be defined */
		SISLCurve *curve2 = NurbsLineToSislLine(s2); /* Must be defined */
		double epsco = 1.0e-9; /* Not used */
		double epsge = 1.0e-6;
		int numintpt = 0;
		double *intpar1 = NULL;
		double *intpar2 = NULL;
		int numintcu = 0;
		SISLIntcurve **intcurve = NULL;
		int stat = 0;
		s1857(curve1, curve2, epsco, epsge, &numintpt, &intpar1, &intpar2, &numintcu, &intcurve, &stat);

		//numintpt为相交的个数
		return numintpt;
	}

	/**
	* @brief   : 用折线逼近曲线，从而判断两曲线是否相交，并返回相交的次数
	* @param[I]: 第一条曲线
	* @param[I]: 第二条曲线
	* @return  : 相交的次数
	* @note    :
	**/
	int if_Intersect(const Spline &s1, const Spline &s2)
	{
		//用直线逼近曲线之后，用直线判断是否相交更准确
		varray<Spline> S1 = this->CurveToLines(s1);
		varray<Spline> S2 = this->CurveToLines(s2);
		int n = 0;
		for (auto &i : S1)
		{
			for (auto &j : S2)
			{
				if (SplinesInterNum(i, j) != 0)
				{
					n++;
				}
			}
		}
		return n;
	}

	//测试用，可以删除
	void test_IfInter()
	{
		Spline s1, s2;

		varray<Spline> temp;
		rwg.ReadSpline("E:\\Model\\ModelTest\\test_IfInter.txt", temp);

		s1 = temp[0];
		s2 = temp[1];
		rwg.WriteSpline("E:\\Model\\ModelTest\\test_IfInter.txt", temp);

		if (if_Intersect(s1, s2))
		{
			cout << "相交" << endl;
		}
		else
		{
			cout << "不相交" << endl;
		}
	}

	/**
	* @brief   : 判断点v是否为曲线s的端点
	* @param[I]: 一个点的坐标
	* @param[O]: 一条曲线
	* @return  : bool，表示是否为端点
	* @note    :
	**/
	bool if_EndOfLine(Vec4 v, const Spline &s)
	{
		if (JudgeTwoPointsCoincide(v, s.m_CtrlPts[0]) || JudgeTwoPointsCoincide(v, s.m_CtrlPts[s.m_CtrlPts.size() - 1]))
		{
			return true;
		}
		return false;
	}

	/**
	* @brief   : 判断曲线是否跨越边界线所围成的区域
	* @param[I]: 曲线
	* @param[I]: 边界曲线
	* @param[O]: none
	* @return  : bool，是否跨越
	* @note    :通过截断两端点，判断截断后的曲线s1是否与边界曲线相交，进而判断曲线s是否跨越边界线S所围成的区域
	**/
	bool SplinePolygonIfCross(const Spline &s, const varray<Spline> &S)
	{
		Spline0 s0;
		//截断两端点获取新的连接线
		Spline s1 = s0.getSpline(s.GetLinePoint(0.01), s.GetLinePoint(0.99));

		//分别与所有边界线进行求交判断
		for (auto &i : S)
		{
			if (if_Intersect(s1, i) != 0)
			{
				return true;
			}
		}
		return false;
	}

	void test_SplinePolygonIfCross()
	{
		varray<Spline> temp;
		varray<Spline> S;
		Spline s;
		Boundary bo;
		S = bo.getCircle(5);
		rwg.ReadSpline("E:\\Model\\ModelTest\\OneTestLine.txt", temp);

		s = *temp.begin();

		if (SplinePolygonIfCross(s, S))
		{
			cout << "跨越区域" << endl;
		}
		else
		{
			cout << "不跨越区域" << endl;
		}
	}

	bool minimum_X(Spline s, double &min)
	{
		if (s.m_CtrlPts.empty())
		{
			return false;
		}
		min = s.m_CtrlPts[0].x;

		for (auto &i : s.m_CtrlPts)
		{
			if (min > i.x)
			{
				min = i.x;
			}
		}
		return true;
	}

	bool minimum_X(varray<Spline> S, double &min)
	{
		double temp = 0;
		if (S.empty())
		{
			return false;
		}
		min = S[0].m_CtrlPts[0].x;
		for (auto &i : S)
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
	* @param[I]: 轮廓线数组
	* @param[O]: none
	* @return  :
	* @note    :通过找出所有轮廓曲线中的最小x坐标，并按照x的值进行排序
	**/
	bool Order_Spline(varray<pair<int, varray<Spline>>> &S)
	{
		//轮廓为空，排序失败
		if (S.empty())
		{
			return false;
		}
		double min = 0;
		multimap<double, pair<int, varray<Spline>>>mp;
		for (auto &i : S)
		{
			minimum_X(i.second, min);
			mp.insert(pair<double, pair<int, varray<Spline>>>(min, i));
		}

		S.clear();
		for (auto it = mp.begin(); it != mp.end(); it++)
		{
			S.push_back(it->second);
		}
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
	bool ClosePoint(const Spline &s1, const Spline &s2, double &u1, double &u2)
	{
		//两曲线相交，不考虑
		if (if_Intersect(s1, s2) > 0)
		{
			return false;
		}

		u1 = 0;
		u2 = 0;
		double temp;
		double min = towPointDistence(s1.GetLinePoint(u1), s2.GetLinePoint(u2));
		//步长，参数每次增长的长度
		double step = 0.01;
		for (double i = 0.0; i <= 1;)
		{
			for (double j = 0.0; j <= 1;)
			{
				temp = towPointDistence(s1.GetLinePoint(i), s2.GetLinePoint(j));
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

	void control_Accuracy(double &u)
	{
		//若u与1之间相差小于0.02，则将u设置为1
		if (1.0 - u <= 0.05)
		{
			u = 1;
		}
		if (u - 0.0 <= 0.05)
		{
			u = 0;
		}
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
	varray<AddLineData> BestAddLine(varray<Spline> &S1, varray<Spline> &S2)
	{
		multimap<double, AddLineData> bestMap;
		multimap<double, AddLineData> single_bestMap;
		varray<AddLineData> result;

		AddLineData adl;
		double u1, u2;
		Vec4 v1, v2;
		Spline s;
		Spline0 s0;

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
				//判断s是否与S1、S2相交
				if (!SplinePolygonIfCross(s, S1) && !SplinePolygonIfCross(s, S2))
				{
					//不跨越区域S1、S2，可以存入候选线
					single_bestMap.insert(pair<double, AddLineData>(s.GetLength(s.m_CtrlPts.size()), adl));
				}
			}

			//在所有符合要求的连接线中
			if (!single_bestMap.empty())
			{
				for (auto &i : single_bestMap)
				{
					result.push_back(i.second);
				}
			}
		}
		return result;
	}

	varray<ContourData> DeduplicateLines(varray<ContourData> Lines)
	{
		bool flag;
		varray<ContourData> result;
		for (auto &i : Lines)
		{
			//标志作用，记录曲线i是否与result中的某条曲线重合
			flag = false;
			for (auto &j : result)
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
		return result;
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
	void addTuple(int curveNumber, int contourNumber, double value_U, map<int, map<int, set<double>>> &Map)
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
	void operateResult(varray<ContourData> result, map<int, map<int, set<double>>> &mp)
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
		for (auto &contourData : result)
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

		for (auto &i : mp)
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
		}
	}

	bool truncateCurve(pair<Spline, double> pa, varray<Spline> &S)
	{
	}

	varray<ContourData> AddLine(varray<Spline> &outLines, varray<varray<Spline>> &inLines)
	{
		ContourData cd;
		Spline s;
		varray<Spline> chack;
		varray<ContourData> result;
		varray<ContourData> mid_result;
		varray<AddLineData> temp;
		varray<pair<int, varray<Spline>>> operate_array;
		varray<pair<int, varray<Spline>>> temp_operate;
		bool flag;
		operate_array.push_back(pair<int, varray<Spline>>(-1, outLines));
		//将所有的内轮廓放入temp_operate数组，用于对内轮廓排序
		int num = 0;
		for (auto &i : inLines)
		{
			temp_operate.push_back(pair<int, varray<Spline>>(num++, i));
		}
		//将内轮廓从左至右的排序
		Order_Spline(temp_operate);

		//将排序后的数组存入operate_array数组中
		for (auto &i : temp_operate)
		{
			operate_array.push_back(i);
		}
		//实现循环数组的功能
		if (inLines.size() > 1)
		{
			operate_array.push_back(pair<int, varray<Spline>>(-1, outLines));
		}

		//存放每两个边界区域之间的候选连接线
		multimap<double, ContourData> bestLine;

		//用于判断连接线是否跨越轮廓区域
		varray<varray<Spline>> allBoudary = inLines;
		allBoudary.push_back(outLines);

		auto front = operate_array.begin();
		auto back = operate_array.begin() + 1;
		//获得所有的候选连接线
		for (; back != operate_array.end();)
		{
			bestLine.clear();

			//得到边界front和边界back之间的所有的候选连接线
			temp = BestAddLine(front->second, back->second);

			//判断候选线是否与其他区域相交
			for (auto&j : temp)
			{
				s = j.s;
				flag = true;
				//对各个候选连接线判断是否跨域边界区域
				for (auto &k : allBoudary)
				{
					//是否跨越轮廓线围成的区域
					if (this->SplinePolygonIfCross(s, k))
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
					cd.contour1 = front->first;
					cd.contour2 = back->first;
					bestLine.insert(pair<double, ContourData>(s.GetLength(s.m_CtrlPts.size()), cd));
				}
			}

			mid_result.clear();
			int len;
			int n;
			//将符合要求的最短的线存入mid_result数组中，再从mid_result数组中选择一条连接线
			if (!bestLine.empty())
			{
				double min = bestLine.begin()->first;
				auto beg = bestLine.lower_bound(min);
				auto end = bestLine.upper_bound(min);
				for (auto it = beg; it != end; it++)
				{
					mid_result.push_back(it->second);
				}
				//去掉重复的曲线
				mid_result = DeduplicateLines(mid_result);

				//候选线不为空，选择合适得连接线
				if (!mid_result.empty())
				{
					len = mid_result.end() - mid_result.begin();
					cout << "共" << len << "条候选线，请输入编号进行选择(编号从0开始)：";
					{
						chack.clear();
						for (auto &i : mid_result)
						{
							chack.push_back(i.adl.s);
						}

						for (auto &i : allBoudary)
						{
							for (auto &j : i)
							{
								chack.push_back(j);
							}
						}
						rwg.WriteSpline("E:\\Model\\ModelTest\\ChooseLine.txt", chack);
					}
					cin >> n;
					while (n >= len)
					{
						cout << "请输入正确的编号，";
						cout << "共" << len << "条候选线，请输入编号进行选择(编号从0开始)：";
						cin >> n;
					}
					result.push_back(mid_result[n]);
				}
			}
			front = back;
			back++;
		}

		//出去中间结果中的重复曲线
		result = DeduplicateLines(result);

		//可能会对同一曲线进行多次截断，所以要特殊处理
		map<int, map<int, set<double>>> mp;
		operateResult(result, mp);
		return result;
	}

	//测试用，可以删除
	void testFunc()
	{
		Model_Solution m;
		Boundary bo;
		varray<Spline> S;
		varray<Spline> chack;
		varray<varray<Spline>> inLines;

		S = bo.getCircle(5);
		m.Trans(S, 20, 1);
		inLines.push_back(S);

		double l = 20;
		double r = 5;
		Spline0 s0;

		Vec4 p1 = { -r - l / 2,0,0,1 };
		Vec4 p2 = { -l / 2,r,0,1 };
		Vec4 p3 = { l / 2,r,0,1 };
		Vec4 p4 = { r + l / 2,0,0,1 };
		Vec4 p5 = { l / 2,-r,0,1 };
		Vec4 p6 = { -l / 2,-r,0,1 };

		Vec4 O1 = { -l / 2,0,0,1 };
		Vec4 O2 = { l / 2,0,0,1 };

		S = bo.getSquare(10, 10);
		m.Trans(S, 40, 1);
		inLines.push_back(S);

		S.clear();
		S.push_back(s0.getArcSpline(p1, p2, O1));
		S.push_back(s0.getSpline(p2, p3));
		S.push_back(s0.getArcSpline(p3, p4, O2));
		S.push_back(s0.getArcSpline(p5, p4, O2));
		S.push_back(s0.getSpline(p6, p5));
		S.push_back(s0.getArcSpline(p6, p1, O1));
		m.Trans(S, 10, -1);
		inLines.push_back(S);

		S = bo.getSquare(90, 40);
		m.Trans(S, 10, 1);
		varray<Spline> temp;
		varray<ContourData> result = AddLine(S, inLines);
		//查看result里面保存的信息
		chack.clear();
		for (auto &i : result)
		{
			chack.push_back(i.adl.s);
			if (i.contour1 == -1)
			{
				chack.push_back(S[i.adl.pa1.first]);
			}
			else
			{
				chack.push_back(*((inLines.begin() + i.contour1)->begin() + i.adl.pa1.first));
			}

			if (i.contour2 == -1)
			{
				chack.push_back(S[i.adl.pa2.first]);
			}
			else
			{
				(inLines.begin() + i.contour2)->begin();
				chack.push_back(*((inLines.begin() + i.contour2)->begin() + i.adl.pa2.first));
			}
		}
		rwg.WriteSpline("E:\\Model\\ModelTest\\ChackLine.txt", chack);
		for (auto &i : result)
		{
			temp.push_back(i.adl.s);
		}
		rwg.WriteSpline("E:\\Model\\ModelTest\\ResultLine.txt", temp);
		for (auto &i : S)
		{
			temp.push_back(i);
		}

		for (auto &i : inLines)
		{
			for (auto &j : i)
			{
				temp.push_back(j);
			}
		}
		rwg.WriteSpline("E:\\Model\\ModelTest\\AllResultLine.txt", temp);
	}

	void testFunc1()
	{
		Model_Solution m;
		Boundary bo;
		varray<Spline> S;
		varray<varray<Spline>> inLines;

		double unit = 10;
		double l = unit * 4;
		double r = unit;
		Spline0 s0;

		Vec4 p1 = { -r - l,0,0,1 };
		Vec4 p2 = { -l,r,0,1 };
		Vec4 p3 = { -r,r,0,1 };
		Vec4 p4 = { -r,-r,0,1 };
		Vec4 p5 = { -l,-r,0,1 };

		Vec4 O = { -l,0,0,1 };

		S.clear();

		S.push_back(s0.getArcSpline(p1, p2, O));
		S.push_back(s0.getSpline(p2, p3));
		S.push_back(s0.getSpline(p5, p4));
		S.push_back(s0.getArcSpline(p5, p1, O));

		varray<Spline>temp = S;
		for (int i = 0; i < 3; i++)
		{
			for (auto &i : temp)
			{
				m.Rolate(i, PI / 2, 3);
				S.push_back(i);
			}
		}
		inLines.push_back(S);

		S = bo.getSquare(12 * unit, 12 * unit);
		varray<ContourData> result = AddLine(S, inLines);
		temp.clear();
		for (auto &i : result)
		{
			temp.push_back(i.adl.s);
		}
		rwg.WriteSpline("E:\\Model\\ModelTest\\ResultLine.txt", temp);

		for (auto &i : S)
		{
			temp.push_back(i);
		}

		for (auto &i : inLines)
		{
			for (auto &j : i)
			{
				temp.push_back(j);
			}
		}
		rwg.WriteSpline("E:\\Model\\ModelTest\\AllResultLine.txt", temp);
	}

	void testFunc2()
	{
		double r1 = 6;
		double r2 = 8;
		double r3 = 4;
		double r4 = 1;
		double r5 = 2;
		double l1 = 24;
		double l2 = 24;
		double l3 = 12;
		double l4 = 12;

		Vec4 o1 = { -l1,0,0,1 };
		Vec4 o2 = { l2,0,0,1 };
		Vec4 o3 = { -l3,0,0,1 };
		Vec4 o4 = { l4,0,0,1 };

		Vec4 v1 = { -l1 - r2,0,0,1 };
		Vec4 v2 = { -l1,r2,0,1 };
		Vec4 v3 = { l2,r3,0,1 };
		Vec4 v4 = { l2 + r3,0,0,1 };
		Vec4 v5 = { l2,-r3,0,1 };
		Vec4 v6 = { -l1,-r2,0,1 };
		Vec4 v7 = { -l1 - r1,0,0,1 };
		Vec4 v8 = { -l1,r1,0,1 };
		Vec4 v9 = { -l1 + r1,0,0,1 };
		Vec4 v10 = { -l1,-r1,0,1 };
		Vec4 v11 = { -l3 - r4,0,0,1 };
		Vec4 v12 = { -l3,r4,0,1 };
		Vec4 v13 = { l4,r4,0,1 };
		Vec4 v14 = { l4 + r4,0,0,1 };
		Vec4 v15 = { l4,-r4,0,1 };
		Vec4 v16 = { -l3,-r4,0,1 };
		Vec4 v17 = { l2 - r5,0,0,1 };
		Vec4 v18 = { l2,r5,0,1 };
		Vec4 v19 = { l2 + r5,0,0,1 };
		Vec4 v20 = { l2,-r5,0,1 };

		varray<Spline> S;
		varray<Spline> S1;
		varray<Spline> S2;
		varray<Spline> S3;
		varray<Spline> S4;

		Spline0 s0;
		Spline s;

		s = s0.getArcSpline(v1, v2, o1);
		S.push_back(s);

		s = s0.getSpline(v2, v3);
		S.push_back(s);

		s = s0.getArcSpline(v3, v4, o2);
		S.push_back(s);

		s = s0.getArcSpline(v5, v4, o2);
		S.push_back(s);

		s = s0.getSpline(v6, v5);
		S.push_back(s);

		s = s0.getArcSpline(v6, v1, o1);
		S.push_back(s);

		RWGeometric rwg;
		s = s0.getArcSpline(v7, v8, o1);
		S1.push_back(s);

		s = s0.getArcSpline(v9, v8, o1);
		S1.push_back(s);

		s = s0.getArcSpline(v10, v9, o1);
		S1.push_back(s);

		s = s0.getArcSpline(v10, v7, o1);
		S1.push_back(s);

		s = s0.getArcSpline(v11, v12, o3);
		S2.push_back(s);

		s = s0.getSpline(v12, v13);
		S2.push_back(s);

		s = s0.getArcSpline(v14, v13, o4);
		S2.push_back(s);

		s = s0.getArcSpline(v15, v14, o4);
		S2.push_back(s);

		s = s0.getSpline(v16, v15);
		S2.push_back(s);

		s = s0.getArcSpline(v16, v11, o3);
		S2.push_back(s);

		s = s0.getArcSpline(v17, v18, o2);
		S3.push_back(s);

		s = s0.getArcSpline(v19, v18, o2);
		S3.push_back(s);

		s = s0.getArcSpline(v20, v19, o2);
		S3.push_back(s);

		s = s0.getArcSpline(v20, v17, o2);
		S3.push_back(s);

		varray<varray<Spline>> inLines;
		inLines.push_back(S1);
		inLines.push_back(S2);
		inLines.push_back(S3);

		varray<Spline> temp;
		varray<ContourData> result = AddLine(S, inLines);
		for (auto &i : result)
		{
			temp.push_back(i.adl.s);
		}
		rwg.WriteSpline("E:\\Model\\ModelTest\\ResultLine.txt", temp);
		for (auto &i : S)
		{
			temp.push_back(i);
		}

		for (auto &i : inLines)
		{
			for (auto &j : i)
			{
				temp.push_back(j);
			}
		}
		rwg.WriteSpline("E:\\Model\\ModelTest\\AllResultLine.txt", temp);
	}
public:
	RWGeometric rwg;
};

//边表节点
struct EdgeNode0
{
	int adjvex;						//邻接点域，存储该顶点的下标
	Spline EdgeLine;				//对应的Nurbs边曲线
	int adjEdge;					//对应的Nurbs边曲线在all_NurbsLines中的下标
	EdgeNode0* next;				//链域，指向下一个邻接点
};
//顶点表节点
struct VertexNode0
{
	Vec4 vexdata;				//顶点域，存储顶点信息
	EdgeNode0* firstedge;			//边表头指针
};

//有向图
struct GraphAdjList0
{
	varray<VertexNode0> adjlist;		//顶点集合
	int numNodes, numEdges;			//图中当前顶点和边数
};

//多边形轮廓
class MyPolygon0
{
public:
	//获得一点的前后点编号
	void GetFBpNumber(int pointNumber, int &frontP, int &behindP)
	{
	}

	//获得某点的前序点编号
	int GetFrontNumber(int pointNumber)
	{
	}

public:
	varray<Vec4> p_Points;					//顶点
	varray<Spline> p_NurbsLines;			//边曲线
	varray<int> objectInAllLines;			//对应曲线在all_NurbsLines数组中的位置
	varray<int> convexHull;					//0代表凸点，1代表凹点，180°共线的也算凹点
};

class CreatePolygon
{
public:
	CreatePolygon(varray<Spline> Outer, varray<Spline> AddLine, varray<varray<Spline>> Inner)
	{
		g_Adjlist.numEdges = 0;
		g_Adjlist.numNodes = 0;
		getGraph(Outer, AddLine, Inner);
		book.resize(g_Adjlist.numNodes, 0);
		DFSPolygon();
		getPolygon();
	}

	bool path_Exit(const vector<int> &path)
	{
		if (allPath.size() == 0)
		{
			return false;
		}
		vector<int> temp_path1 = path;
		vector<int> temp_path2;
		//顶点号排序，使得等价路径对应位置相同
		sort(temp_path1.begin(), temp_path1.end());

		//记录是否有等价路径的存在，默认不存在
		bool exit = false;

		for (const auto &i : allPath)
		{
			//大小不相同，不是等价路径
			if (temp_path1.size() != i.size())
			{
				continue;
			}

			temp_path2 = i;
			sort(temp_path2.begin(), temp_path2.end());
			bool same = true;
			//字符串匹配原理
			for (int j = 0; j < temp_path1.size(); ++j)
			{
				//对应位置不相同，说明不是等价路径
				if (temp_path1[j] != temp_path2[j])
				{
					same = false;
					break;
				}
			}
			if (same)
			{
				exit = true;
			}
		}
		return exit;
	}

	void dfs(int curNum)
	{
		//最少三个顶点构成轮廓区域
		if (path.size() >= 3)
		{
			if (curNum == *path.begin())
			{
				for (const auto &i : path)
				{
					cout << i << " ";
				}
				cout << endl;
				if (!path_Exit(path))
				{
					allPath.push_back(path);
				}
			}
		}
		if (book[curNum] == 1)
		{
			return;
		}
		path.push_back(curNum);
		book[curNum] = 1;
		for (auto next = g_Adjlist.adjlist[curNum].firstedge; next != NULL;)
		{
			int n = next->adjvex;
			dfs(n);
			next = next->next;
		}
		path.pop_back();
		book[curNum] = 0;
	}

	//广度优先搜索获取区域
	void DFSPolygon()
	{
		for (int i = 0; i < g_Adjlist.adjlist.size(); ++i)
		{
			dfs(i);
		}
	}

	void getPolygon()
	{
		EdgeNode0* next;
		int nextIndex;//轮廓下一个顶点的编号
		int size;//轮廓大小
		MyPolygon0 mPolygon;//临时轮廓

		for (const auto &path : allPath)
		{
			size = path.size();
			mPolygon.p_NurbsLines.clear();
			mPolygon.p_Points.clear();
			mPolygon.objectInAllLines.clear();
			mPolygon.convexHull.resize(size, 1);

			for (int j = 0; j < path.size(); j++)
			{
				mPolygon.p_Points.push_back(g_Adjlist.adjlist[path[j]].vexdata);

				//获取相邻边链表
				next = g_Adjlist.adjlist[path[j]].firstedge;
				nextIndex = path[(j + 1) % size];

				while (next != NULL)
				{
					if (next->adjvex == nextIndex)
					{
						//存入边
						mPolygon.p_NurbsLines.push_back(next->EdgeLine);

						//存入对应曲线在all_NurbsLines数组中的位置
						mPolygon.objectInAllLines.push_back(next->adjEdge);
						break;
					}
					next = next->next;
				}
			}
			allPolygon.push_back(mPolygon);
		}
		removeCoincidentPolygon();
	}

	void removeCoincidentPolygon()
	{
		int size = allPolygon.size();
		vector<int> remove;
		remove.resize(size, 0);

		for (int i = 0; i < size; ++i)
		{
			if (remove[i] == 1)
			{
				continue;
			}
			//判断第i个多边形是否被别的多边形包含
			for (int j = 0; j < size; j++)
			{
				if (j == i || remove[j] == 1)
				{
					continue;
				}
				if (PolygonsRelationship(allPolygon[i].p_NurbsLines, allPolygon[j].p_NurbsLines))
				{
					remove[j] = 1;
				}
			}
		}
		for (int i = size - 1; i >= 0; --i)
		{
			if (remove[i] == 1)
			{
				allPolygon.erase(allPolygon.begin() + i);
			}
		}
	}

	//在邻接表中寻找顶点的索引
	int searchVertexIndex(Vec4 point)
	{
		for (int i = 0; i < g_Adjlist.numNodes; ++i)
		{
			bool flag = JudgeTwoPointsCoincide(point, g_Adjlist.adjlist[i].vexdata);
			if (flag)
			{
				return i;
			}
		}
		return -1;
	}

	//无向图构建
	void getGraph(varray<Spline> Outer, varray<Spline> AddLine, varray<varray<Spline>> Inner)
	{
		ps.orderEdgeAntiClock0(Outer);
		for (auto & i : Inner)
		{
			ps.orderEdgeAntiClock0(i);
		}

		for (const auto &i : Outer)
		{
			all_NurbsLines.push_back(i);
		}
		for (const auto &i : Inner)
		{
			for (const auto &j : i)
			{
				all_NurbsLines.push_back(j);
			}
		}
		for (const auto &i : AddLine)
		{
			all_NurbsLines.push_back(i);
		}

		Vec4 p1;
		Vec4 p2;
		int index1;
		int index2;
		int len;
		int edgeIndex = 0;
		EdgeNode0* tempEdgeNode0;
		VertexNode0 tempVertexNode0;
		for (const auto &i : all_NurbsLines)
		{
			len = i.m_CtrlPts.size();
			p1 = i.m_CtrlPts[0];
			p2 = i.m_CtrlPts[len - 1];

			index1 = searchVertexIndex(p1);
			index2 = searchVertexIndex(p2);
			//没有顶点p1的顶点数据,需要添加顶点,下同
			if (index1 == -1)
			{
				//创建顶点信息
				tempVertexNode0.firstedge = NULL;
				tempVertexNode0.vexdata = p1;
				g_Adjlist.adjlist.push_back(tempVertexNode0);

				//重置顶点位置
				index1 = g_Adjlist.numNodes;
				++g_Adjlist.numNodes;
			}
			if (index2 == -1)
			{
				//创建顶点信息
				tempVertexNode0.firstedge = NULL;
				tempVertexNode0.vexdata = p2;
				g_Adjlist.adjlist.push_back(tempVertexNode0);

				//重置顶点位置
				index2 = g_Adjlist.numNodes;
				++g_Adjlist.numNodes;
			}

			//创建邻接边
			tempEdgeNode0 = new EdgeNode0();
			tempEdgeNode0->adjvex = index2;
			tempEdgeNode0->EdgeLine = i;
			tempEdgeNode0->adjEdge = edgeIndex;

			//头接法，加入顶点的邻接链表
			tempEdgeNode0->next = g_Adjlist.adjlist[index1].firstedge;
			g_Adjlist.adjlist[index1].firstedge = tempEdgeNode0;

			//创建邻接边
			tempEdgeNode0 = new EdgeNode0();
			tempEdgeNode0->adjvex = index1;
			tempEdgeNode0->EdgeLine = i;
			tempEdgeNode0->adjEdge = edgeIndex;
			//头接法，加入顶点的邻接链表
			tempEdgeNode0->next = g_Adjlist.adjlist[index2].firstedge;
			g_Adjlist.adjlist[index2].firstedge = tempEdgeNode0;

			++edgeIndex;
		}
		g_Adjlist.numEdges = all_NurbsLines.size();
	}

public:
	GraphAdjList0 g_Adjlist;
	varray<MyPolygon0> allPolygon;
	PublicSolution ps;
	varray<int> book;
	varray<Vec4> allPoint;
	varray<Spline> all_NurbsLines;
	RWGeometric rwg;
	vector<int> path;
	vector<vector<int>> allPath;
};

struct SingleFace
{
	varray<int> index;
};

struct SinglePoint
{
	Vec4 center;
	bool isCurve;//是否为曲线的标志
	int pointIndex;//点在allPoint中的序号
	int point_of_Line;//点所在的曲线编号
	int Point_of_Polygon;//点所在的轮廓编号
	int front;// 前驱点
	int back;//后继点
	Vec4 point;//点的坐标
	double u;//点的参数值
};

struct CircumCenter
{
	Vec4 cent;
	SinglePoint point1;
	SinglePoint point2;
	SinglePoint point3;
};

struct TheCurve
{
	//该曲线编号
	int index_Curve;

	//该曲线需要截断位置的u值
	set<double> value_U;
};

struct TheBoundary
{
	//轮廓编号
	int index_Boundary;

	//需要处理的曲线数组
	map<int, TheCurve> curveNeedOperate;
};

struct OperatePoint
{
	map<int, TheBoundary> pointNeedOperate;
};

class DomainTool
{
public:
	/**
	* @brief   : 逆时针排序闭合曲线
	* @param[I]: 闭合轮廓线
	* @return  :
	* @note    :
	**/
	static void orderEdgeAntiClock(varray<Spline>&sl)
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

	static void orderEdgeClockwise(varray<Spline> &sl)
	{
		orderEdgeAntiClock(sl);
		Vec4 temp;
		stack<Spline> s;
		for (auto &i : sl)
		{
			temp = i.m_CtrlPts[0];
			i.m_CtrlPts[0] = i.m_CtrlPts[2];
			i.m_CtrlPts[2] = temp;
			s.push(i);
		}
		sl.clear();
		while (!s.empty())
		{
			sl.push_back(s.top());
			s.pop();
		}
	}
};

//曲线的工具
class CurveTool
{
public:
	CurveTool() {};

	//判断点是否在线段为斜对角的矩形内
	bool inScope(Vec4 p, Spline s)
	{
		int x1 = s.m_CtrlPts[0].x;
		int x2 = s.m_CtrlPts[s.m_CtrlPts.size() - 1].x;

		int y1 = s.m_CtrlPts[0].y;
		int y2 = s.m_CtrlPts[s.m_CtrlPts.size() - 1].y;

		int x_left = x1 < x2 ? x1 : x2;
		int x_right = x1 > x2 ? x1 : x2;

		int y_bottom = y1 < y2 ? y1 : y2;
		int y_top = y1 > y2 ? y1 : y2;

		if (x_left - p.x > this->precision || p.x - x_right > this->precision)
		{
			return false;
		}
		else
		{
			if (y_bottom - p.y > this->precision || p.y - y_top > this->precision)
			{
				return false;
			}
		}
		return true;
	}

	//判断是否为直线
	bool if_StraightLine(const Spline &s)
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
	* @brief   : 利用SISL库函数判断两曲线是否相交
	* @param[I]: 第一条曲线
	* @param[I]: 第二条曲线
	* @return  : 交点个数
	* @note    :
	**/
	int SplinesInterNum(const Spline &s1, const Spline &s2)
	{
		SISLCurve *curve1 = NurbsLineToSislLine(s1); /* Must be defined */
		SISLCurve *curve2 = NurbsLineToSislLine(s2); /* Must be defined */
		double epsco = 1.0e-9; /* Not used */
		double epsge = 1.0e-6;
		int numintpt = 0;
		double *intpar1 = NULL;
		double *intpar2 = NULL;
		int numintcu = 0;
		SISLIntcurve **intcurve = NULL;
		int stat = 0;
		s1857(curve1, curve2, epsco, epsge, &numintpt, &intpar1, &intpar2, &numintcu, &intcurve, &stat);

		//numintpt为相交的个数
		return numintpt;
	}

	/**
	* @brief   : 用折线逼近曲线，从而判断两曲线是否相交，并返回相交的次数
	* @param[I]: 第一条曲线
	* @param[I]: 第二条曲线
	* @return  : 相交的次数
	* @note    :
	**/
	int if_Intersect(const Spline &line1, const Spline &line2)
	{
		//判断是否共线
		if (areLinesCollinear(line1, line2))
		{
			//若共线，判断两直线是否有重合的部分
			if (inScope(line1.m_CtrlPts[0], line2) || inScope(line1.m_CtrlPts[line1.m_CtrlPts.size() - 1], line2))
			{
				return 1;
			}

			if (inScope(line2.m_CtrlPts[0], line1) || inScope(line2.m_CtrlPts[line2.m_CtrlPts.size() - 1], line1))
			{
				return 1;
			}
		}
		//用直线逼近曲线之后，用直线判断是否相交更准确
		varray<Spline> S1 = this->CurveToLines(line1);
		varray<Spline> S2 = this->CurveToLines(line2);
		int n = 0;
		for (auto &i : S1)
		{
			for (auto &j : S2)
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
	* @brief   : 用折线逼近曲线
	* @param[I]: 要逼近的曲线
	* @return  : 折线
	* @note    :在参数区间0~1之间取点，并构建直线，从而逼近曲线
	**/
	varray<Spline> CurveToLines(const Spline &s)
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

	//正负法判断点的位置
	//F（x，y）＝x（YB－YA）＋y（XA－XB）＋YA·XB－XA·YB
	//两点构成直线
	//F(x,y)<0 说明点在直线左边
	//F(x,y)>0说明点在直线右边
	//F(x,y)=0说明点在直线上
	double PositiveOrNegative(Spline s, Vec4 P)
	{
		Vec4 Pa = s.m_CtrlPts[0];
		Vec4 Pb = s.m_CtrlPts[s.m_CtrlPts.size() - 1];
		double A = Pb.y - Pa.y;
		double B = Pa.x - Pb.x;
		double C = Pa.y * Pb.x;
		double D = Pa.x * Pb.y;
		double F = A * P.x + B * P.y + C - D;
		return F;
	}

	//判断是否垂直与x轴
	bool isVertical(Spline s)
	{
		Vec4 point1 = s.m_CtrlPts[0];
		Vec4 point2 = s.m_CtrlPts[s.m_CtrlPts.size() - 1];
		if (point1.x - point2.x < this->precision) {
			return true;
		}
		else
		{
			return false;
		}
	}
	bool isVertical(Vec4 point1, Vec4 point2)
	{
		if (abs(point1.x - point2.x) < this->precision) {
			return true;
		}
		else
		{
			return false;
		}
	}

	//计算斜率
	double calculateSlope(Spline s)
	{
		Vec4 point1 = s.m_CtrlPts[0];
		Vec4 point2 = s.m_CtrlPts[s.m_CtrlPts.size() - 1];

		// 如果直线垂直于x轴，返回无穷大
		if (isVertical(point1, point2))
		{
			return INFINITY;
		}

		// 计算斜率
		return (point2.y - point1.y) / (point2.x - point1.x);
	}

	//判断两直线是否共线
	bool areLinesCollinear(const Spline& line1, const Spline& line2)
	{
		double slope1 = calculateSlope(line1);
		double slope2 = calculateSlope(line2);
		//斜率相等
		if (slope1 == slope2 | abs(slope1 - slope2) < this->precision)
		{
			double F = PositiveOrNegative(line1, line2.m_CtrlPts[0]);
			//line2上一点在line1上
			if (F < this->precision)
			{
				return true;
			}
			else
			{
				return false;
			}
		}
		return false;
	}

	/**
	* @brief   : 判断曲线是否跨越边界线所围成的区域
	* @param[I]: 曲线
	* @param[I]: 边界曲线
	* @param[O]: none
	* @return  : bool，是否跨越
	* @note    :通过截断两端点，判断截断后的曲线s1是否与边界曲线相交，进而判断曲线s是否跨越边界线S所围成的区域
	**/
	bool SplinePolygonIfCross(const Spline &s, const varray<Spline> &S)
	{
		Spline0 s0;
		//截断两端点获取新的连接线
		Spline s1 = s0.getSpline(s.GetLinePoint(0.01), s.GetLinePoint(0.99));

		//分别与所有边界线进行求交判断
		for (auto &i : S)
		{
			if (if_Intersect(s1, i) != 0)
			{
				return true;
			}
		}
		return false;
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
	double calculateAngle3D(const Vec4 &p01, const Vec4 &p02, const Vec4 &p03)
	{
		Vec4 s1 = p01 - p02;
		Vec4 s2 = p03 - p02;

		double l1 = sqrt(pow(s1.x, 2) + pow(s1.y, 2) + pow(s1.z, 2));
		double l2 = sqrt(pow(s2.x, 2) + pow(s2.y, 2) + pow(s2.z, 2));

		double d = s1.x*s2.x + s1.y*s2.y + s1.z*s2.z;

		double angle = acos(d / (l1 * l2));
		return angle;
	}

	double calculateLength(Spline s)
	{
		Vec4 p1 = s.m_CtrlPts[0];
		Vec4 p2 = s.m_CtrlPts[s.m_CtrlPts.size() - 1];
		return sqrt(pow(p2.x - p1.x, 2) + pow(p2.y - p1.y, 2));
	}
	double calculateLength(const Vec4 &p1, const Vec4 &p2)
	{
		return sqrt(pow(p2.x - p1.x, 2) + pow(p2.y - p1.y, 2));
	}

private:
	double precision = 1e-3;//计算精度
};

//三角形的工具
class TriangleTool
{
public:

	// 静态方法计算外接圆心
	static Vec4 calculateCircumcenter(const Vec4& p1, const Vec4& p2, const Vec4& p3) {
		if (p1.x == p2.x || p1.x == p3.x)
		{
			return calculateCircumcenter(p2, p3, p1);
		}
		double D = 2 * (p1.x * (p2.y - p3.y) + p2.x * (p3.y - p1.y) + p3.x * (p1.y - p2.y));

		double Ux = (p1.x * p1.x + p1.y * p1.y) * (p2.y - p3.y) +
			(p2.x * p2.x + p2.y * p2.y) * (p3.y - p1.y) +
			(p3.x * p3.x + p3.y * p3.y) * (p1.y - p2.y);

		double Uy = (p1.x * p1.x + p1.y * p1.y) * (p3.x - p2.x) +
			(p2.x * p2.x + p2.y * p2.y) * (p1.x - p3.x) +
			(p3.x * p3.x + p3.y * p3.y) * (p2.x - p1.x);

		double circumcenterX = Ux / D;
		double circumcenterY = Uy / D;

		return Vec4(circumcenterX, circumcenterY, 0.0, 1);
	}

	// 计算两点之间的中点
	static Vec4 calculateMidpoint(const Vec4& p1, const Vec4& p2)
	{
		double midX = (p1.x + p2.x) / 2;
		double midY = (p1.y + p2.y) / 2;
		return { midX, midY ,0,1 };
	}

	// 计算两点之间的斜率
	static double calculateSlope(const Vec4& p1, const Vec4& p2)
	{
		return (p2.y - p1.y) / (p2.x - p1.x);
	}

	// 计算中垂线的斜率
	static double calculatePerpendicularSlope(double slope)
	{
		// 两条直线的斜率乘积为 -1
		return -1 / slope;
	}

	// 计算中垂线的方程
	static pair<double, double> calculatePerpendicularLine(const Vec4& p1, const Vec4& p2) {
		Vec4 midPoint = calculateMidpoint(p1, p2);
		double slope = calculatePerpendicularSlope(calculateSlope(p1, p2));

		if (std::isinf(slope)) {
			// 处理斜率无穷大的情况
			return make_pair(midPoint.x, 0.0);
		}

		double b = midPoint.y - slope * midPoint.x;
		return make_pair(slope, b);
	}

	// 计算两条直线的交点
	static Vec4 calculateIntersectionPoint(double m1, double b1, double m2, double b2)
	{
		double x;
		double y;
		if (std::isinf(m1)) {
			// 处理斜率无穷大的情况
			x = b1;
			y = m2 * x + b2;
		}
		else if (std::isinf(m2)) {
			// 处理斜率无穷大的情况
			x = b2;
			y = m1 * x + b1;
		}
		else {
			x = (b2 - b1) / (m1 - m2);
			y = m1 * x + b1;
		}

		return{ x,y,0,1 };
	}

	// 计算两点之间的距离
	static double distance(double x1, double y1, double x2, double y2)
	{
		return sqrt(pow(x2 - x1, 2) + pow(y2 - y1, 2));
	}

	// 静态方法计算两点之间的距离
	static double distance(const Vec4& p1, const Vec4& p2) {
		return sqrt(pow(p2.x - p1.x, 2) + pow(p2.y - p1.y, 2));
	}

	// 计算三角形的面积
	static double calculateTriangleArea2D(Vec4 p1, Vec4 p2, Vec4 p3)
	{
		// 检查三个点是否共线
		if ((p2.x - p1.x) * (p3.y - p1.y) - (p3.x - p1.x) * (p2.y - p1.y) < 1e-3) {
			return 0.0;  // 返回零表示面积无效
		}
		// 计算三条边的长度
		double side1 = distance(p1.x, p1.y, p2.x, p2.y);
		double side2 = distance(p2.x, p2.y, p3.x, p3.y);
		double side3 = distance(p3.x, p3.y, p1.x, p1.y);

		// 使用海伦公式计算面积
		double s = (side1 + side2 + side3) / 2;
		double area = sqrt(s * (s - side1) * (s - side2) * (s - side3));

		return area;
	}

	// 静态方法计算外接圆的半径
	static double circumcircleRadius(const Vec4& p1, const Vec4& p2, const Vec4& p3) {
		double side1 = distance(p1, p2);
		double side2 = distance(p2, p3);
		double side3 = distance(p3, p1);

		double s = (side1 + side2 + side3) / 2;
		double area = sqrt(s * (s - side1) * (s - side2) * (s - side3));
		double radius = (side1 * side2 * side3) / (4 * area);
		return radius;
	}
};

class CDT_Operate
{
public:

	CDT_Operate() {};

	CDT_Operate(varray<Spline> outer, varray<varray<Spline>> inner)
	{
		varray<Spline> S;
		rwg.ReadSpline("E:\\Model\\PlaneQuad\\Delaunay\\ChackLines.txt", S);
		int n = curveTool.if_Intersect(S[0], S[1]);
		//角度控制因子
		this->p = 1;

		//长度控制因子
		this->q = 0.5;

		//位置控制因子
		this->n = 100;
		this->m = 1;

		this->outer = outer;
		this->inner = inner;

		PublicSolution ps;
		ps.orderEdgeAntiClock0(this->outer);
		for (auto &i : this->inner)
		{
			ps.orderEdgeAntiClock0(i);
		}
		//离散化轮廓，生成CDT的输入
		Discretization(this->outer, this->inner);

		//输出CDT的结果到文件夹中
		outPut();

		//Delaynay的结果，去除一些无用结果
		readFace();

		//构建Delaunay三角网格，用于查看
		buid_Delaunay();

		//寻找与三个三角形相邻的三角形
		searchTriangle();

		//获取三角形外接圆的圆心
		getCircumcenter();

		//连接线的端点可能不在轮廓的端点，需要处理轮廓线
		//refineBoundaryByMap();

		//获取最终的连接线
		//getAddLine();
		//getAddLine();
		getAddLine(2.0);
	}
	/**
	* @brief   : 获取轮廓中最短曲线的长度
	* @param[I]: 内轮廓曲线
	* @return  : 长度值
	* @note    :
	**/
	double minLength(varray<varray<Spline>> inner)
	{
		if (inner.size() == 0)
		{
			return -1;
		}
		double len = inner[0][0].GetLength(inner[0][0].m_CtrlPts.size());
		for (auto &l : inner)
		{
			for (auto &i : l)
			{
				if (len > i.GetLength(i.m_CtrlPts.size()))
				{
					len = i.GetLength(i.m_CtrlPts.size());
				}
			}
		}
		return len;
	}

	/**
	* @brief   : 利用最短长度等比例离散曲线
	* @param[I]: 外轮廓
	* @param[I]: 内轮廓
	* @return  : none
	* @note    : 点对应的轮廓编号中，0代表外轮廓，>=1代表内轮廓
	**/
	void Discretization(varray<Spline> outer, varray<varray<Spline>> inner)
	{
		tempS.clear();
		if (outer.size() == 0)
		{
			return;
		}
		double minLen = minLength(inner);

		if (minLen == -1)
		{
			minLen = outer[0].GetLength(outer[0].m_CtrlPts.size());
			for (auto &i : outer)
			{
				if (minLen > i.GetLength(i.m_CtrlPts.size()))
				{
					minLen = i.GetLength(i.m_CtrlPts.size());
				}
			}
		}

		SinglePoint sp;

		int begin = 0;
		int end = 0;
		double path;
		int num;
		bool isCurve = false;
		for (auto it = outer.begin(); it != outer.end(); ++it)
		{
			isCurve = !curveTool.if_StraightLine(*it);
			num = it->GetLength(it->m_CtrlPts.size()) / minLen * 10;
			path = 1.0 / num;//计算出步长
			for (double i = 0.0; i < 1.0;)
			{
				//由于精度为小数点后两位，因此为了防止控制点重合，需要处理重合的点
				if (1.0 - i < path / 2.0)
				{
					break;
				}
				sp.point = it->GetLinePoint(i);
				sp.u = i;
				sp.Point_of_Polygon = 0;
				sp.point_of_Line = it - outer.begin();
				sp.pointIndex = allPoint.size();
				sp.isCurve = isCurve;
				allPoint.push_back(sp);
				end++;
				i += path;
			}
		}
		getDCT(begin, end);
		for (auto it1 = inner.begin(); it1 != inner.end(); ++it1)
		{
			for (auto it2 = it1->begin(); it2 != it1->end(); ++it2)
			{
				num = it2->GetLength(it2->m_CtrlPts.size()) / minLen * 10;
				path = 1.0 / num;
				for (double i = 0.0; i < 1.0;)
				{
					//由于精度为小数点后两位，因此为了防止控制点重合，需要处理重合的点
					if (1.0 - i < path / 2.0)
					{
						break;
					}
					sp.point = it2->GetLinePoint(i);
					sp.u = i;
					//所属轮廓号
					sp.Point_of_Polygon = it1 - inner.begin() + 1;
					//所属曲线号
					sp.point_of_Line = it2 - it1->begin();
					//在allPoints中的索引号
					sp.pointIndex = allPoint.size();
					allPoint.push_back(sp);
					end++;
					i += path;
				}
			}
			getDCT(begin, end);
		}
	}

	/**
	* @brief   : 滑动窗口机制，在将begin与end区间内的离散点存入CDT中,并为每个带信息的点设置前驱与后继，便于后续计算剖分线角度权值
	* @param[I]: 开始下标
	* @param[I]: 结束下标
	* @return  : none
	* @note    :
	**/
	void getDCT(int &begin, int &end)
	{
		if (begin == end)
		{
			return;
		}
		// construct a constrained triangulation
		Vertex_handle v1;
		Vertex_handle v2;
		Vertex_handle vBegin;
		varray<SinglePoint>::iterator it = this->allPoint.begin() + begin;
		it->back = begin + 1;
		it->front = end - 1;
		v1 = cdt.insert(Point_CDT(it->point.x, it->point.y));
		vBegin = v1;
		it++;
		int n;

		for (; it != this->allPoint.begin() + end; )
		{
			v2 = cdt.insert(Point_CDT(it->point.x, it->point.y));
			if (it == this->allPoint.begin() + end - 1)
			{
				it->front = it->pointIndex - 1;
				it->back = begin;
			}
			else
			{
				it->front = it->pointIndex - 1;
				it->back = it->pointIndex + 1;
			}
			cdt.insert_constraint(v1, v2);
			v1 = v2;
			it++;
			n = it - this->allPoint.begin();
		}
		cdt.insert_constraint(v1, vBegin);
		begin = end;
	}

	/**
	* @brief   : 若三角形的三个点均在同一轮廓上，需要特殊处理
	* @param[I]: 三角形顶点下标
	* @param[I]: 三角形顶点下标
	* @param[I]: 三角形顶点下标
	* @return  : none
	* @note    : 三角形三个顶点均在外轮廓上时，需要特殊判断，
				 1、三点在同一曲线上时，不满足要求
				 2、三点构成的几何域在外轮廓之外时，不满足要求
	**/
	bool ifSatisfactory(int p1, int p2, int p3)
	{
		//三点在同一曲线上时，不满足要求
		if (allPoint[p1].point_of_Line == allPoint[p2].point_of_Line&&allPoint[p1].point_of_Line == allPoint[p3].point_of_Line)
		{
			return false;
		}
		Spline s1;
		Spline s2;
		Spline s3;
		varray<Spline> S;

		//对应轮廓曲线编号
		int index1 = allPoint[p1].point_of_Line;
		int index2 = allPoint[p2].point_of_Line;
		int index3 = allPoint[p3].point_of_Line;

		bool flag1 = curveTool.if_StraightLine(outer[index1]);
		bool flag2 = curveTool.if_StraightLine(outer[index2]);
		bool flag3 = curveTool.if_StraightLine(outer[index3]);

		//三个点均在曲线上时，不满住要求
		if (!(flag1 || flag2 || flag3))
		{
			//return false;
		}

		s1 = outer[index1];
		s2 = outer[index2];
		s3 = outer[index3];

		s1 = curve.getSpline(allPoint[p1].point, allPoint[p2].point);
		S.push_back(s1);
		s2 = curve.getSpline(allPoint[p2].point, allPoint[p3].point);
		S.push_back(s2);
		s3 = curve.getSpline(allPoint[p3].point, allPoint[p1].point);
		S.push_back(s3);

		varray<Spline> temp = outer;
		for (const auto &i : S)
		{
			temp.push_back(i);
		}
		rwg.WriteSpline("E:\\Model\\PlaneQuad\\Delaunay\\ifSatisfactory.txt", temp);
		//三点构成的几何域在外轮廓之外时，不满足要求
		if (!PolygonsRelationship(S, outer))
		{
			return false;
		}
		return true;
	}

	//读取所有三角形面的信息
	void readFace()
	{
		//读取文件内容
		ifstream ifs;
		ifs.open("E:\\Model\\PlaneQuad\\Delaunay\\vertax.txt", ios::in);
		if (!ifs.is_open())
		{
			cout << "文件打开失败" << endl;
			return;
		}

		ifs >> verNum;
		ifs >> faceNum;
		ifs >> num;
		int p1, p2, p3;
		Vec4 point;
		for (int i = 0; i < verNum - 1; ++i)
		{
			ifs >> point.x;
			ifs >> point.y;
			point.z = 0;
			point.w = 1;
		}
		for (int i = 0; i < faceNum; ++i)
		{
			ifs >> p1;
			ifs >> p2;
			ifs >> p3;
			if (p1*p2*p3 == 0)
			{
				continue;
			}

			int polygon1 = allPoint[p1 - 1].Point_of_Polygon;
			int polygon2 = allPoint[p2 - 1].Point_of_Polygon;
			int polygon3 = allPoint[p3 - 1].Point_of_Polygon;
			//若为三角形的顶点在一个轮廓上
			if (polygon1 == polygon2 && polygon1 == polygon3)
			{
				//三点均在非外轮廓上，则不考虑
				if (allPoint[p1 - 1].Point_of_Polygon != 0)
				{
					continue;
				}
				//若三点均在外轮廓上，则需要判断三角形是否在几何域外面
				else
				{
					if (!ifSatisfactory(p1 - 1, p2 - 1, p3 - 1))
					{
						continue;
					}
				}
			}

			//计算三角形的面积，若三角形的面积为零，则不考虑
			if (triangleTool.calculateTriangleArea2D(allPoint[p1 - 1].point, allPoint[p2 - 1].point, allPoint[p3 - 1].point) == 0)
			{
				continue;
			}
			SingleFace tempFace;
			tempFace.index.push_back(p1 - 1);
			tempFace.index.push_back(p2 - 1);
			tempFace.index.push_back(p3 - 1);
			allFace.push_back(tempFace);
		}

		ifs.close();

		/*ofstream ofs;
		ofs.open("E:\\Model\\PlaneQuad\\Delaunay\\NewFace.txt", ios::out);
		if (!ofs.is_open())
		{
			cout << "文件打开失败" << endl;
			return;
		}

		for (auto &face : allFace)
		{
			ofs << face.index[0] << " " << face.index[1] << " " << face.index[2] << endl;
		}*/
	}

	//构建Delaunay三角网格，用于查看
	void buid_Delaunay()
	{
		Spline s;
		varray<Spline> S;
		int p1, p2, p3;
		for (const auto &face : allFace)
		{
			p1 = face.index[0];
			p2 = face.index[1];
			p3 = face.index[2];
			s = curve.getSpline(allPoint[p1].point, allPoint[p2].point);
			S.push_back(s);
			s = curve.getSpline(allPoint[p2].point, allPoint[p3].point);
			S.push_back(s);
			s = curve.getSpline(allPoint[p3].point, allPoint[p1].point);
			S.push_back(s);
		}
		for (const auto &i : outer)
		{
			S.push_back(i);
		}

		for (const auto &i : inner)
		{
			for (const auto &j : i)
			{
				S.push_back(j);
			}
		}
		rwg.WriteSpline("E:\\Model\\PlaneQuad\\Delaunay\\Delaunay.txt", S);
	}

	//输出CDT结果到记事本中
	void outPut()
	{
		filebuf buf;
		if (buf.open("E:\\Model\\PlaneQuad\\Delaunay\\vertax.txt", ios::out) == nullptr)
		{
			cerr << "stdout open failed" << endl;
			return;
		}
		ostream out(&buf);
		cdt.file_output(out);
		buf.close();
	}

	//两三角形是否有公共边
	bool ifCommonEdges(const SingleFace &face1, const SingleFace &face2)
	{
		//两个面公共顶点个数
		int num = 0;
		for (const auto &p1 : face1.index)
		{
			for (const auto &p2 : face2.index)
			{
				if (p1 == p2)
				{
					++num;
					break;
				}
			}
		}
		if (num > 1)
		{
			return true;
		}
		return false;
	}

	//找到需要连接外接圆心的三角形
	void searchTriangle()
	{
		//相邻三角形个数
		int num = 0;
		int p1;
		int p2;
		int p3;
		varray<int> adj;

		for (auto i = 0; i < allFace.size(); ++i)
		{
			num = 0;
			adj.clear();
			adj.push_back(i);
			for (auto j = 0; j < allFace.size(); ++j)
			{
				if (i == j)
				{
					continue;
				}
				if (ifCommonEdges(allFace[i], allFace[j]))
				{
					adj.push_back(j);
					++num;
				}
			}
			//相邻三角形有三个
			if (num == 3)
			{
				p1 = allFace[i].index[0];
				p2 = allFace[i].index[1];
				p3 = allFace[i].index[2];

				if (allPoint[p1].Point_of_Polygon == allPoint[p2].Point_of_Polygon&&
					allPoint[p1].Point_of_Polygon == allPoint[p3].Point_of_Polygon)
				{
					continue;
				}
				processedFaces.push_back(allFace[i]);
			}
		}
	}

	//计算并存储所有需要连接外接圆心的三角形的外接圆心
	void getCircumcenter()
	{
		for (const auto &face : processedFaces)
		{
			//构建外接圆心
			CircumCenter temp;
			temp.point1 = allPoint[face.index[0]];
			temp.point2 = allPoint[face.index[1]];
			temp.point3 = allPoint[face.index[2]];
			temp.cent = TriangleTool::calculateCircumcenter(temp.point1.point, temp.point2.point, temp.point3.point);
			circumCenter.push_back(temp);
		}
	}

	//计算出位于该区域的点
	varray<SinglePoint> calculateAllPointsInThisArea(const Vec4 &cent, const double &R)
	{
		varray<SinglePoint> thePoints;
		for (const auto &p : allPoint)
		{
			//if(TriangleTool::distance(cent,))
		}
	}

	//获取指定轮廓指定线上指定区域的点
	varray<SinglePoint> getThePoint(int polygon, Vec4 center, double R)
	{
		double len;
		varray<SinglePoint> thePoint;
		for (const auto &p : allPoint)
		{
			//判断点p的轮廓编号是否相等
			if (p.Point_of_Polygon == polygon)
			{
				//判断p是否在以center为圆心，半径为R的范围内
				len = TriangleTool::distance(p.point, center);
				if (len <= R)
				{
					thePoint.push_back(p);
				}
			}
		}
		return thePoint;
	}

	//判断连接线是否满足初步要求
	bool preliminaryRequirements(const SinglePoint &p1, const SinglePoint &p2)
	{
		if (p1.u != 0.0 && p1.u != 1.0)
		{
			return false;
		}
		if (p2.u != 0.0 && p2.u != 1.0)
		{
			return false;
		}
		Spline s;
		s = this->curve.getSpline(p1.point, p2.point);

		varray<Spline>temp;
		temp = this->outer;
		for (const auto &i : this->inner)
		{
			for (const auto &j : i)
			{
				temp.push_back(j);
			}
		}
		temp.push_back(s);
		RWGeometric rwg;
		rwg.WriteSpline("E:\\Model\\PlaneQuad\\Delaunay\\ChackLines.txt", temp);
		int n;
		//是否与轮廓线相交
		if (curveTool.SplinePolygonIfCross(s, outer))
		{
			return false;
		}
		for (const auto &i : inner)
		{
			if (curveTool.SplinePolygonIfCross(s, i))
			{
				return false;
			}
		}
		//将候选线分别与已选择的连接线进行判断是否相交
		Spline temps = curve.getSpline(s.GetLinePoint(0.1), s.GetLinePoint(0.9));
		for (const auto &i : addLine)
		{
			temp.clear();
			temp.push_back(s);
			temp.push_back(i);
			rwg.WriteSpline("E:\\Model\\PlaneQuad\\Delaunay\\ChackLines.txt", temp);
			int n = curveTool.if_Intersect(s, i);
			rwg.ReadSpline("E:\\Model\\PlaneQuad\\Delaunay\\ChackLines.txt", temp);
			n = curveTool.if_Intersect(temp[0], temp[1]);
			if (n)
			{
				return false;
			}

			rwg.ReadSpline("E:\\Model\\PlaneQuad\\Delaunay\\ChackLines.txt", temp);
		}

		return true;
	}

	//计算角度的权重
	double calculateAngleWeight(const SinglePoint &p1, const SinglePoint &p2)
	{
		double angleAlpha1;
		double angleBeta1;

		double angleAlpha2;
		double angleBeta2;

		Vec4 front1 = allPoint[p1.front].point;
		Vec4 back1 = allPoint[p1.back].point;

		Vec4 front2 = allPoint[p2.front].point;
		Vec4 back2 = allPoint[p2.back].point;

		angleAlpha1 = curveTool.calculateAngle3D(front1, p1.point, p2.point);
		angleBeta1 = curveTool.calculateAngle3D(p2.point, p1.point, back1);

		angleAlpha2 = curveTool.calculateAngle3D(front2, p2.point, p1.point);
		angleBeta2 = curveTool.calculateAngle3D(p1.point, p2.point, back2);

		double deta1 = abs(cos(angleAlpha1) - cos(angleBeta1));
		double deta2 = abs(cos(angleAlpha2) - cos(angleBeta2));
		return m * deta1 + m * deta2;
	}

	//计算连接线权重
	double calculateWeightOfLine(const SinglePoint &p1, const SinglePoint &p2, const double &R,const bool &allNode)
	{
		//首先判断是否满足初步要求，即是否与轮廓、或者已选择的连接线相交
		if (!preliminaryRequirements(p1, p2))
		{
			return -1;
		}

		double angleWeight = calculateAngleWeight(p1, p2);
		double lengthWeight = curveTool.calculateLength(p1.point, p2.point) / R;
		if (allNode)
		{
			return angleWeight;
		}
		return angleWeight * p + lengthWeight * q;
	}

	//获得单个区域中合适的连接线
	bool areaAddline(const varray<varray<SinglePoint>> &area_Points, varray<Spline> &S, double R,const bool &allNode)
	{
		varray<Spline> chooseLine;
		varray<SinglePoint> points1 = *(area_Points.begin());
		varray<SinglePoint> points2 = *(area_Points.begin() + 1);
		varray<SinglePoint> points3;

		multimap<double, pair<SinglePoint, SinglePoint>> candidateLine;
		double weight;
		//一个区域可能涉及三个轮廓，所以要分为三个组，两两之间
		if (area_Points.size() == 3)
		{
			points3 = *(area_Points.begin() + 2);
		}
		for (const auto &p1 : points1)
		{
			if (p1.u != 0.0 && p1.u != 1.0)
			{
				continue;
			}
			for (const auto &p2 : points2)
			{
				weight = calculateWeightOfLine(p1, p2, R, allNode);
				if (weight > 0)
				{
					candidateLine.insert(pair<double, pair<SinglePoint, SinglePoint>>(weight, pair<SinglePoint, SinglePoint>(p1, p2)));
				}
			}
		}
		for (const auto &p3 : points3)
		{
			if (p3.u != 0.0 && p3.u != 1.0)
			{
				continue;
			}
			for (const auto &p1 : points1)
			{
				weight = calculateWeightOfLine(p1, p3, R, allNode);
				if (weight > 0)
				{
					candidateLine.insert(pair<double, pair<SinglePoint, SinglePoint>>(weight, pair<SinglePoint, SinglePoint>(p1, p3)));
				}
			}
			for (const auto &p2 : points2)
			{
				weight = calculateWeightOfLine(p3, p2, R, allNode);
				if (weight > 0)
				{
					candidateLine.insert(pair<double, pair<SinglePoint, SinglePoint>>(weight, pair<SinglePoint, SinglePoint>(p3, p2)));
				}
			}
		}
		//用于查看*******************
		//候选连接线
		////调试使用**************************
		point4d t;
		varray<point4d> tempP;
		varray<varray<point4d>>pp;
		for (const auto &i : area_Points)
		{
			for (const auto &p : i)
			{
				t.Set(p.point.x, p.point.y, p.point.z);
				t.w = p.point.w;
				tempP.push_back(t);
			}
		}
		pp.push_back(tempP);
		rwg.WritePoint("E:\\Model\\PlaneQuad\\Delaunay\\Point4D.txt", pp);
		////调试使用***************************


		for (const auto &i : candidateLine)
		{
			chooseLine.push_back(curve.getSpline(i.second.first.point, i.second.second.point));
			cout << i.first << endl;
		}
		if (chooseLine.size() == 0)
		{
			return false;
		}
		cout << "共" << chooseLine.size() << "个候选线，请选择（编号从零开始,-1表示不选择）：" << endl;
		varray<Spline> temp = chooseLine;
		for (const auto &i : outer)
		{
			temp.push_back(i);
		}
		for (const auto &i : inner)
		{
			for (const auto &j : i)
			{
				temp.push_back(j);
			}
		}
		rwg.WriteSpline("E:\\Model\\PlaneQuad\\Delaunay\\ChooseLine.txt", temp);
		//int index;
		//double len;
		//len = chooseLine.size();
		//while (cin >> index)
		//{
		//	if (index >= 0 && index < len)
		//	{
		//		S.push_back(chooseLine[index]);
		//	}
		//	char ch = getchar();//读取下一个字符，为换行符，则break
		//	if (ch == '\n')
		//	{
		//		break;
		//	}
		//}
		//用于查看*******************

		if (candidateLine.empty())
		{
			return false;
		}
		else
		{
			double weight = candidateLine.begin()->first;
			auto begin = candidateLine.lower_bound(weight);
			auto end = candidateLine.upper_bound(weight);
			for (; begin != end; begin++)
			{
				S.push_back(curve.getSpline(begin->second.first.point, begin->second.second.point));
			}
			return true;
		}
	}

	//获得区域中的点
	varray<varray<SinglePoint>> areaPoints(CircumCenter circumCenter, const double &len)
	{
		varray<varray<SinglePoint>> all_Area_Points;
		varray<SinglePoint> area_Points1;
		varray<SinglePoint> area_Points2;
		varray<SinglePoint> area_Points3;
		int polygon1 = circumCenter.point1.Point_of_Polygon;
		int polygon2 = circumCenter.point2.Point_of_Polygon;
		int polygon3 = circumCenter.point3.Point_of_Polygon;

		//三个点位于同一区域，返回空的数组
		if (polygon1 == polygon2 && polygon1 == polygon3)
		{
			return all_Area_Points;
		}
		int line1 = circumCenter.point1.point_of_Line;
		int line2 = circumCenter.point2.point_of_Line;
		int line3 = circumCenter.point3.point_of_Line;
		area_Points1 = getThePoint(polygon1, circumCenter.cent, len);
		area_Points2 = getThePoint(polygon2, circumCenter.cent, len);
		area_Points3 = getThePoint(polygon3, circumCenter.cent, len);

		if (polygon1 == polygon2)
		{
			area_Points1 = getThePoint(polygon1, circumCenter.cent, len);
			area_Points3 = getThePoint(polygon3, circumCenter.cent, len);
			all_Area_Points.push_back(area_Points1);
			all_Area_Points.push_back(area_Points3);
			return all_Area_Points;
		}
		if (polygon1 == polygon3)
		{
			area_Points1 = getThePoint(polygon1, circumCenter.cent, len);
			area_Points2 = getThePoint(polygon2, circumCenter.cent, len);
			all_Area_Points.push_back(area_Points1);
			all_Area_Points.push_back(area_Points2);
			return all_Area_Points;
		}
		if (polygon2 == polygon3)
		{
			area_Points1 = getThePoint(polygon1, circumCenter.cent, len);
			area_Points2 = getThePoint(polygon2, circumCenter.cent, len);
			all_Area_Points.push_back(area_Points1);
			all_Area_Points.push_back(area_Points2);
			return all_Area_Points;
		}
		area_Points1 = getThePoint(polygon1, circumCenter.cent, len);
		area_Points2 = getThePoint(polygon2, circumCenter.cent, len);
		area_Points3 = getThePoint(polygon3, circumCenter.cent, len);
		all_Area_Points.push_back(area_Points1);
		all_Area_Points.push_back(area_Points2);
		all_Area_Points.push_back(area_Points3);
		return all_Area_Points;
	}
	//获得区域中的所有点，轮廓不止三个
	varray<varray<SinglePoint>> areaAllPoints(const CircumCenter &circumCenter, const double &len)
	{
		map<int,varray<SinglePoint>> allPointInThisArea;
		varray<varray<SinglePoint>> all_Area_Points;
		varray<SinglePoint> temp;

		double length;
		int boundaryNum = -2;
		for (const auto &p : allPoint)
		{
			//判断p是否在以center为圆心，半径为R的范围内
			length = TriangleTool::distance(p.point, circumCenter.cent);
			if (length <= len)
			{
				if (boundaryNum == p.Point_of_Polygon)
				{
					temp.push_back(p);
				}
				else
				{
					if (boundaryNum == -2)
					{
						temp.push_back(p);
						boundaryNum = p.Point_of_Polygon;
					}
					else
					{
						allPointInThisArea.insert(pair<int, varray<SinglePoint>>(boundaryNum, temp));
						temp.clear();
						boundaryNum = p.Point_of_Polygon;
						temp.push_back(p);
					}
					
				}
			}
		}
		for (const auto & points : allPointInThisArea)
		{
			all_Area_Points.push_back(points.second);
		}
		return all_Area_Points;
	}


	varray<varray<SinglePoint>> areaPoints()
	{
		varray<varray<SinglePoint>> all_Area_Points;
		varray<SinglePoint> area_Points1;
		varray<SinglePoint> area_Points2;

		for (const auto &i : allPoint)
		{
			if (i.Point_of_Polygon == 0)
			{
				if (i.u == 0 || i.u == 1)
				{
					area_Points1.push_back(i);
				}
			}
			else
			{
				if (i.u == 0 || i.u == 1)
				{
					area_Points2.push_back(i);
				}
			}
		}
		all_Area_Points.push_back(area_Points1);
		all_Area_Points.push_back(area_Points2);
		return all_Area_Points;
	}
	varray<varray<SinglePoint>> allAreaPoints()
	{
		varray<varray<SinglePoint>> all_Area_Points;
		varray<SinglePoint> area_Points1;
		varray<SinglePoint> area_Points2;


		for (const auto &i : allPoint)
		{
			if (i.Point_of_Polygon == 0)
			{
				if (i.u == 0 || i.u == 1)
				{
					area_Points1.push_back(i);
				}
			}
			else
			{
				if (i.u == 0 || i.u == 1)
				{
					area_Points2.push_back(i);
				}
			}
		}
		all_Area_Points.push_back(area_Points1);
		all_Area_Points.push_back(area_Points2);
		return all_Area_Points;
	}

	double getDomainRadius(varray<Spline> domain)
	{
		return 1;
	}

	//连接外接圆心，获取连接线
	void getAddLine()
	{
		Boundary bo;
		Model_Solution m;
		varray<Spline> tempCircum;

		for (const auto &o : circumCenter)
		{
			addLine.push_back(curve.getSpline(o.cent, o.point1.point));
			addLine.push_back(curve.getSpline(o.cent, o.point2.point));
			addLine.push_back(curve.getSpline(o.cent, o.point3.point));

			double len = TriangleTool::distance(o.cent, o.point1.point);

			tempCircum = bo.getCircle(len*1.5);
			m.Trans(tempCircum, o.cent.x, 1);
			m.Trans(tempCircum, o.cent.y, 2);

			for (const auto &i : tempCircum)
			{
				addLine.push_back(i);
			}
		}
		varray<Spline> temp = addLine;
		for (const auto &i : outer)
		{
			temp.push_back(i);
		}
		for (const auto &i : inner)
		{
			for (const auto &j : i)
			{
				temp.push_back(j);
			}
		}
		addLine.clear();
		rwg.WriteSpline("E:\\Model\\PlaneQuad\\Delaunay\\CircleWithAddLines.txt", temp);
	}

	//获取区域连接线
	void getAddLine(double expend)
	{
		varray<Spline> S;
		varray<Spline> tempS;
		double len;
		varray<varray<SinglePoint>>  area_Points;
		multimap<double, CircumCenter> area;

		for (const auto &o : circumCenter)
		{
			len = TriangleTool::distance(o.cent, { 0,0,0,1 });
			area.insert(pair<double, CircumCenter>(len, o));
		}

		{
			//调试使用**************************
			point4d t;
			varray<point4d> tempP;
			varray<varray<point4d>>pp;
			for (const auto &o : circumCenter)
			{
				pp.clear();
				len = TriangleTool::distance(o.cent, o.point1.point);
				area_Points = areaAllPoints(o, len * expend);
				tempP.clear();
				t.Set(o.cent.x, o.cent.y, o.cent.z);
				t.w = o.cent.w;
				tempP.push_back(t);
				for (const auto &i : area_Points)
				{
					for (const auto &p : i)
					{
						t.Set(p.point.x, p.point.y, p.point.z);
						t.w = p.point.w;
						tempP.push_back(t);
					}
				}
				pp.push_back(tempP);
				rwg.WritePoint("E:\\Model\\PlaneQuad\\Delaunay\\Point4D.txt", pp);
			}

			//调试使用***************************
		}

		for (const auto&o : area)
		{
			S.clear();
			len = TriangleTool::distance(o.second.cent, o.second.point1.point);
			area_Points = areaPoints(o.second, len * expend);
			if (areaAddline(area_Points, S, len * expend,false))
			{
				for (const auto &i : S)
				{
					addLine.push_back(i);
				}
			}
		}
		////打个补丁，提高程序鲁棒性
		////例如外圆内方CDT无法生成与三个三角形相邻的三角形
		if (area.size() == 0)
		{
			area_Points = allAreaPoints();
			if (areaAddline(area_Points, S, len * expend, true))
			{
				for (const auto &i : S)
				{
					addLine.push_back(i);
				}
			}
		}


		/*for (const auto &o : circumCenter)
		{
			S.clear();
			len = TriangleTool::distance(o.cent, o.point1.point);
			area_Points = areaPoints(o, len * expend);
			if (areaAddline(area_Points, S, len * expend))
			{
				for (const auto &i : S)
				{
					addLine.push_back(i);
				}
			}
		}*/

		////打个补丁，提高程序鲁棒性
		////例如外圆内方CDT无法生成与三个三角形相邻的三角形
		//if (circumCenter.size() == 0)
		//{
		//	area_Points = areaPoints();
		//	len = getDomainRadius(outer);
		//	if (areaAddline(area_Points, S, len))
		//	{
		//		for (const auto &i : S)
		//		{
		//			addLine.push_back(i);
		//		}
		//	}
		//}

		S = addLine;
		for (const auto &i : outer)
		{
			S.push_back(i);
		}

		for (const auto &i : inner)
		{
			for (const auto &j : i)
			{
				S.push_back(j);
			}
		}
		rwg.WriteSpline("E:\\Model\\PlaneQuad\\Delaunay\\AddLine.txt", S);
	}

	/**
	* @brief   : 向索引表中插入一个元组
	* @param[I]: curve	曲线编号
	* @param[I]: boundary	轮廓编号
	* @param[I]: value_U		参数值
	* @param[O]: Map			索引表
	* @return  :
	* @note    :
	**/
	void addTuple(const int &index)
	{
		if (allPoint[index].u == 0 || allPoint[index].u == 1)
		{
			return;
		}
		double value_U = allPoint[index].u;
		int curveIndex = allPoint[index].point_of_Line;
		int boundaryIndex = allPoint[index].Point_of_Polygon;

		map<int, set<double>> temp_map;
		set<double> temp_set;
		temp_map.clear();
		temp_set.clear();
		//一级索引表中缺少boundary对应的元组
		if (Map.find(boundaryIndex) == Map.end())
		{
			//创建set数组
			temp_set.insert(value_U);

			//创建二级索引表，并插入创建一条元组
			temp_map.insert(pair<int, set<double>>(curveIndex, temp_set));

			//向一级索引表中插入元组
			Map.insert(pair<int, map<int, set<double>>>(boundaryIndex, temp_map));
		}

		//一级索引中存在boundary对应的元组
		else
		{
			//记录一级索引表中boundary对应元组的迭代器
			auto it1 = Map.find(boundaryIndex);

			//获取迭代器it1对应的二级索引表
			temp_map = it1->second;

			//二级索引表中不存在索引号为curve的元组、
			if (temp_map.find(curveIndex) == temp_map.end())
			{
				//创建set数组
				temp_set.insert(value_U);

				//向二级索引中添加元组
				it1->second.insert(pair<int, set<double>>(curveIndex, temp_set));
			}
			//二级索引表中存在索引号为curve的元组
			else
			{
				//通过it1，获得二级索引表中curve对应元组迭代器
				auto it2 = it1->second.find(curveIndex);
				//向curve对应元组
				it2->second.insert(value_U);
			}
		}
	}
	void addTuple(const SinglePoint &point)
	{
		if (point.u == 0 || point.u == 1)
		{
			return;
		}
		double value_U = point.u;
		int curveIndex = point.point_of_Line;
		int boundaryIndex = point.Point_of_Polygon;

		map<int, set<double>> temp_map;
		set<double> temp_set;
		temp_map.clear();
		temp_set.clear();
		//一级索引表中缺少boundary对应的元组
		if (Map.find(boundaryIndex) == Map.end())
		{
			//创建set数组
			temp_set.insert(value_U);

			//创建二级索引表，并插入创建一条元组
			temp_map.insert(pair<int, set<double>>(curveIndex, temp_set));

			//向一级索引表中插入元组
			Map.insert(pair<int, map<int, set<double>>>(boundaryIndex, temp_map));
		}

		//一级索引中存在boundary对应的元组
		else
		{
			//记录一级索引表中boundary对应元组的迭代器
			auto it1 = Map.find(boundaryIndex);

			//获取迭代器it1对应的二级索引表
			temp_map = it1->second;

			//二级索引表中不存在索引号为curve的元组、
			if (temp_map.find(curveIndex) == temp_map.end())
			{
				//创建set数组
				temp_set.insert(value_U);

				//向二级索引中添加元组
				it1->second.insert(pair<int, set<double>>(curveIndex, temp_set));
			}
			//二级索引表中存在索引号为curve的元组
			else
			{
				//通过it1，获得二级索引表中curve对应元组迭代器
				auto it2 = it1->second.find(curveIndex);
				//向curve对应元组
				it2->second.insert(value_U);
			}
		}
	}

	void getOperatePoint()
	{
		for (const auto &face : processedFaces)
		{
			for (const auto &point : face.index)
			{
				addTuple(point);
			}
		}

		for (auto &i : Map)
		{
			cout << "轮廓编号：" << i.first << endl;
			for (auto &j : i.second)
			{
				cout << "曲线编号：" << j.first << endl;
				cout << "参数值：";
				for (auto &k : j.second)
				{
					cout << k << " ";
				}
				cout << endl;
			}
			cout << endl;
		}
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
	bool getCenter(const Spline &S, Vec4 &center)
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
	varray<Spline> segmentCurve(const set<double> &value, const Spline &S)
	{
		Vec4 center;
		varray<Vec4> points;
		varray<Spline> result;
		Spline s;
		result.clear();
		//将两端点及截断位置的坐标存入points数组中
		points.push_back(S.GetLinePoint(0));
		for (auto &i : value)
		{
			points.push_back(S.GetLinePoint(i));
		}
		points.push_back(S.GetLinePoint(1));

		//相邻两节点之间构建线段
		auto front = points.begin();
		auto back = points.begin() + 1;
		//判断是否为直线
		if (curveTool.if_StraightLine(S))
		{
			for (; back != points.end(); ++back, ++front)
			{
				s = curve.getSpline(*front, *back);
				result.push_back(s);
			}
		}
		//若为曲线需要找出圆心的位置
		else
		{
			getCenter(S, center);
			//cout << center.x << " " << center.y << " " << center.z << endl;
			for (; back != points.end(); ++back, ++front)
			{
				s = curve.getArcSpline(*front, *back, center);
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
	void refineBoundaryByMap()
	{
		getOperatePoint();

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
		for (auto it1 = Map.begin(); it1 != Map.end(); ++it1)
		{
			result.clear();
			boundaryNum = it1->first - 1;
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
				for (auto &i : temp)
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
				for (auto &i : result)
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

				for (auto &i : result)
				{
					inner[boundaryNum].push_back(i);
				}
			}
		}
	}

	//测试用
	void test_getCircumCenter()
	{
		varray<Spline> S;
		varray<Spline>S1;
		rwg.ReadSpline("E:\\Model\\PlaneQuad\\Delaunay\\Boundary.txt", S1);
		rwg.ReadSpline("E:\\Model\\PlaneQuad\\Delaunay\\searchTriangle.txt", S);
		for (const auto &o : circumCenter)
		{
			S.push_back(curve.getSpline(o.cent, o.point1.point));
			S.push_back(curve.getSpline(o.cent, o.point2.point));
			S.push_back(curve.getSpline(o.cent, o.point3.point));
		}
		for (const auto &i : S1)
		{
			S.push_back(i);
		}
		rwg.WriteSpline("E:\\Model\\PlaneQuad\\Delaunay\\CircumCenter.txt", S);
	}
	void test_searchTriangle()
	{
		searchTriangle();
		Spline s;
		varray<Spline> S;
		int p1, p2, p3;
		for (const auto &face : processedFaces)
		{
			p1 = face.index[0];
			p2 = face.index[1];
			p3 = face.index[2];
			s = curve.getSpline(allPoint[p1].point, allPoint[p2].point);
			tempS.push_back(s);
			s = curve.getSpline(allPoint[p2].point, allPoint[p3].point);
			tempS.push_back(s);
			s = curve.getSpline(allPoint[p3].point, allPoint[p1].point);
			tempS.push_back(s);
		}
		rwg.WriteSpline("E:\\Model\\PlaneQuad\\Delaunay\\searchTriangle.txt", tempS);
	}
	void test_refineBoundaryByMap()
	{
		refineBoundaryByMap();
		varray<Spline> S;
		for (const auto &i : outer)
		{
			S.push_back(i);
		}

		for (const auto &i : inner)
		{
			for (const auto &j : i)
			{
				S.push_back(j);
			}
		}
		rwg.WriteSpline("E:\\Model\\PlaneQuad\\Delaunay\\Boundary.txt", S);
	}

public:
	CDT cdt;
	varray<SinglePoint> allPoint;//带信息的点
	varray<SingleFace> allFace;//CDT获得的所有三角面片
	varray<SingleFace> processedFaces;//需要连接外接圆心的三角形
	varray<CircumCenter> circumCenter;//外接圆心
	varray<Spline> addLine;
	varray<pair<SinglePoint, SinglePoint>> twoPoint;
	map<int, map<int, set<double>>> Map;
	int verNum;//顶点个数
	int faceNum;//三角面片数
	int num;//咱也不知道是啥，不影响
	RWGeometric rwg;
	TriangleTool triangleTool;//三角形计算工具
	CurveTool curveTool;
	Curve curve;
	varray<Spline> tempS;
	varray<Spline> outer;
	varray<varray<Spline>> inner;
	varray<Spline> allBoundary;
	//以下为控制函数
	double p;
	double q;
	double m;
	double n;
};

class DT_Operate
{
public:

	DT_Operate() 
	{
		// 准备一组点来构建三角剖分，这里简单地创建了一个矩形的四个顶点
		varray<Point_DT> points;
		//{ Point_DT(0, 0), Point_DT(1, 0), Point_DT(0, 1), Point_DT(1, 1) };
		points.push_back(Point_DT(0, 0));
		points.push_back(Point_DT(0, 1));
		points.push_back(Point_DT(1, 0));
		points.push_back(Point_DT(1, 1));

		// 循环插入点到Delaunay三角剖分中
		for (const auto& p : points) {
			dt.insert(p); // 插入单个点
		}

		// 输出三角剖分中顶点的数量
		std::cout << "Number of vertices: " << dt.number_of_vertices() << std::endl;

		// 遍历所有的有限三角形面
		DT::Finite_faces_iterator fit;
		for (fit = dt.finite_faces_begin(); fit != dt.finite_faces_end(); ++fit) {
			// 获取当前面的句柄
			DT::Face_handle f = fit;

			// 输出构成三角形的三个顶点的坐标
			std::cout << "Triangle: ";
			for (int i = 0; i < 3; ++i) {
				DT::Vertex_handle vertex = f->vertex(i); // 获取第i个顶点的句柄
				std::cout << "(" << vertex->point() << ") ";
			}
			std::cout << std::endl; // 换行
		}
	};

	DT_Operate(varray<Spline> outer, varray<varray<Spline>> inner)
	{
		Discretization(outer, inner);
		buid_Delaunay();
	}
	/**
	* @brief   : 获取轮廓中最短曲线的长度
	* @param[I]: 内轮廓曲线
	* @return  : 长度值
	* @note    :
	**/
	double minLength(varray<varray<Spline>> inner)
	{
		if (inner.size() == 0)
		{
			return -1;
		}
		double len = inner[0][0].GetLength(inner[0][0].m_CtrlPts.size());
		for (auto &l : inner)
		{
			for (auto &i : l)
			{
				if (len > i.GetLength(i.m_CtrlPts.size()))
				{
					len = i.GetLength(i.m_CtrlPts.size());
				}
			}
		}
		return len;
	}
	/**
	* @brief   : 利用最短长度等比例离散曲线
	* @param[I]: 外轮廓
	* @param[I]: 内轮廓
	* @return  : none
	* @note    : 点对应的轮廓编号中，0代表外轮廓，>=1代表内轮廓
	**/
	void Discretization(varray<Spline> outer, varray<varray<Spline>> inner)
	{
		tempS.clear();
		if (outer.size() == 0)
		{
			return;
		}
		double minLen = minLength(inner);

		if (minLen == -1)
		{
			minLen = outer[0].GetLength(outer[0].m_CtrlPts.size());
			for (auto &i : outer)
			{
				if (minLen > i.GetLength(i.m_CtrlPts.size()))
				{
					minLen = i.GetLength(i.m_CtrlPts.size());
				}
			}
		}

		SinglePoint sp;

		int begin = 0;
		int end = 0;
		double path;
		int num;
		bool isCurve = false;
		for (auto it = outer.begin(); it != outer.end(); ++it)
		{
			isCurve = !curveTool.if_StraightLine(*it);
			num = it->GetLength(it->m_CtrlPts.size()) / minLen * 10;
			path = 1.0 / num;//计算出步长
			for (double i = 0.0; i < 1.0;)
			{
				//由于精度为小数点后两位，因此为了防止控制点重合，需要处理重合的点
				if (1.0 - i < path / 2.0)
				{
					break;
				}
				sp.point = it->GetLinePoint(i);
				sp.u = i;
				sp.Point_of_Polygon = 0;
				sp.point_of_Line = it - outer.begin();
				sp.pointIndex = allPoint.size();
				sp.isCurve = isCurve;
				allPoint.push_back(sp);
				end++;
				i += path;
			}
		}
		getCT(begin, end);
		for (auto it1 = inner.begin(); it1 != inner.end(); ++it1)
		{
			for (auto it2 = it1->begin(); it2 != it1->end(); ++it2)
			{
				num = it2->GetLength(it2->m_CtrlPts.size()) / minLen * 10;
				path = 1.0 / num;
				for (double i = 0.0; i < 1.0;)
				{
					//由于精度为小数点后两位，因此为了防止控制点重合，需要处理重合的点
					if (1.0 - i < path / 2.0)
					{
						break;
					}
					sp.point = it2->GetLinePoint(i);
					sp.u = i;
					//所属轮廓号
					sp.Point_of_Polygon = it1 - inner.begin() + 1;
					//所属曲线号
					sp.point_of_Line = it2 - it1->begin();
					//在allPoints中的索引号
					sp.pointIndex = allPoint.size();
					allPoint.push_back(sp);
					end++;
					i += path;
				}
			}
			getCT(begin, end);
		}
	}
	void getCT(int &begin, int &end)
	{

		if (begin == end)
		{
			return;
		}
		varray<SinglePoint>::iterator it = this->allPoint.begin() + begin;
		it->back = begin + 1;
		it->front = end - 1;
		dt.insert(Point_CDT(it->point.x, it->point.y));
		++it;
		int n;
		for (; it != this->allPoint.begin() + end; )
		{
			dt.insert(Point_CDT(it->point.x, it->point.y));
			if (it == this->allPoint.begin() + end - 1)
			{
				it->front = it->pointIndex - 1;
				it->back = begin;
			}
			else
			{
				it->front = it->pointIndex - 1;
				it->back = it->pointIndex + 1;
			}
			it++;
			n = it - this->allPoint.begin();
		}
		begin = end;
	}

	//构建Delaunay三角网格，用于查看
	void buid_Delaunay()
	{
		Spline s;
		varray<Spline> S;
		// 输出三角剖分中顶点的数量
		std::cout << "Number of vertices: " << dt.number_of_vertices() << std::endl;

		// 遍历所有的有限三角形面
		DT::Finite_faces_iterator fit;
		for (fit = dt.finite_faces_begin(); fit != dt.finite_faces_end(); ++fit) {
			// 获取当前面的句柄
			DT::Face_handle f = fit;

			// 输出构成三角形的三个顶点的坐标
			std::cout << "Triangle: ";
			for (int i = 0; i < 3; ++i) {
				DT::Vertex_handle vertex = f->vertex(i); // 获取第i个顶点的句柄
				std::cout << "(" << vertex->point() << ") ";
			}
			std::cout << std::endl; // 换行
			Vec4 p1 = { 0,0,0,1 };
			Vec4 p2 = { 0,0,0,1 };
			Vec4 p3 = { 0,0,0,1 };
			DT::Vertex_handle vertex1 = f->vertex(0);
			DT::Vertex_handle vertex2 = f->vertex(1);
			DT::Vertex_handle vertex3 = f->vertex(2);
			p1.x = vertex1->point().x();
			p1.y = vertex1->point().y();
			p2.x = vertex2->point().x();
			p2.y = vertex2->point().y();
			p3.x = vertex3->point().x();
			p3.y = vertex3->point().y();
			S.push_back(curve.getSpline(p1, p2));
			S.push_back(curve.getSpline(p1, p3));
			S.push_back(curve.getSpline(p2, p3));
		}
		rwg.WriteSpline("E:\\Model\\PlaneQuad\\Delaunay\\Delaunay.txt", S);
	}
	
public:
	DT dt;
	varray<SinglePoint> allPoint;//带信息的点
	varray<SingleFace> allFace;//CDT获得的所有三角面片
	varray<SingleFace> processedFaces;//需要连接外接圆心的三角形
	varray<CircumCenter> circumCenter;//外接圆心
	varray<Spline> addLine;
	varray<pair<SinglePoint, SinglePoint>> twoPoint;
	map<int, map<int, set<double>>> Map;
	int verNum;//顶点个数
	int faceNum;//三角面片数
	int num;//咱也不知道是啥，不影响
	RWGeometric rwg;
	TriangleTool triangleTool;//三角形计算工具
	CurveTool curveTool;
	Curve curve;
	varray<Spline> tempS;
	varray<Spline> outer;
	varray<varray<Spline>> inner;
	varray<Spline> allBoundary;
	//以下为控制函数
	double p;
	double q;
	double m;
	double n;

};

class ConvexDecomposition
{
public:
	//正负法判断点的位置
	//F（x，y）＝x（YB－YA）＋y（XA－XB）＋YA·XB－XA·YB
	//两点构成直线
	//F(x,y)<0 说明点在直线左边
	//F(x,y)>0说明点在直线右边
	//F(x,y)=0说明点在直线上
	double PositiveOrNegative(Vec4 Pa, Vec4 Pb, Vec4 P)
	{
		double A = Pb.y - Pa.y;
		double B = Pa.x - Pb.x;
		double C = Pa.y * Pb.x;
		double D = Pa.x * Pb.y;
		double F = A * P.x + B * P.y + C - D;
		return F;
	}

	void test_PositiveOrNegative()
	{
		Boundary bo;
		varray<Spline> S = bo.getSquare(10, 10);
		PublicSolution ps;
		ps.orderEdgeAntiClock0(S);
		varray<Vec4> P;
		P.push_back({ 0,0,0,1 });
		P.push_back({ 5,0,0,1 });
		P.push_back({ 10,0,0,1 });
		for (const auto &i : S)
		{
			for (const auto &p : P)
			{
				cout << PositiveOrNegative(i.m_CtrlPts[0], i.m_CtrlPts[2], p) << endl;
			}
			cout << endl;
		}
	}

	//凹凸点判断
	void test_BumpPiont()
	{
		PublicSolution ps;
		Spline0 curve;
		varray<Spline> S;
		RWGeometric rwg;
		rwg.ReadSpline("E:\\Model\\PlaneQuad\\OuterBoundary.txt", S);
		ps.orderEdgeAntiClock0(S);
		varray<Vec4> points;
		for (auto&i : S)
		{
			points.push_back(i.m_CtrlPts[0]);
			if (!ps.if_StraightLine(i))
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

		rwg.WriteSpline("E://Model/PlaneQuad//BumpPiontModel.txt", S);
	}
};

struct Edge_Node
{
	int adjvex;						//邻接点域
	Spline EdgeLine;				//对应的Nurbs边曲线
	int adjEdge;					//对应的Nurbs边曲线在allLines中的下标
	Edge_Node* next;				//链域，指向下一个邻接点
	bool visited;					//是否为剖分线的标志
	bool isAddLine;
	int  num;						//边界点的序号
};

struct Vertex_Node
{
	Vec4 vertex;					//顶点坐标信息
	int vertexIndex;				//顶点索引
	Edge_Node* next;				//邻接边链表
	varray<int> front;				//前驱
	varray<int> back;				//后继
};

class Subdomain
{
public:
	Subdomain() {};
	Subdomain(varray<Spline> outer, varray<varray<Spline>>inner, varray<Spline> addLines)
	{
		initialize(outer, inner, addLines);

		createGraph();

		getPath();
	}

	void initialize(const varray<Spline> &outer, const varray<varray<Spline>> &inner, const varray<Spline> &addLines)
	{
		this->addLines = addLines;
		this->outer = outer;
		this->inner = inner;
		//排序轮廓线
		//外轮廓逆时针，内轮廓顺时针
		DomainTool::orderEdgeClockwise(this->outer);
		for (auto &i : this->inner)
		{
			DomainTool::orderEdgeAntiClock(i);
		}
	}

	//在邻接表中寻找顶点的索引
	int searchVertexIndex(Vec4 point)
	{
		for (int i = 0; i < directedGraph.size(); ++i)
		{
			bool flag = JudgeTwoPointsCoincide(point, directedGraph[i].vertex);
			if (flag)
			{
				return i;
			}
		}
		return -1;
	}

	void createGraph()
	{
		Vec4 p1;
		Vec4 p2;
		int index1;
		int index2;
		Vertex_Node* vertex;
		Edge_Node* edge;

		//构建外轮廓图
		for (const auto &i : outer)
		{
			p1 = i.m_CtrlPts[0];
			p2 = i.m_CtrlPts[2];
			index1 = searchVertexIndex(p1);
			index2 = searchVertexIndex(p2);
			if (index1 == -1)
			{
				index1 = directedGraph.size();
				vertex = new Vertex_Node();
				vertex->next = NULL;
				vertex->vertex = p1;
				vertex->vertexIndex = index1;
				directedGraph.push_back(*vertex);
				delete vertex;
			}

			if (index2 == -1)
			{
				index2 = directedGraph.size();
				vertex = new Vertex_Node();
				vertex->next = NULL;
				vertex->vertex = p2;
				vertex->vertexIndex = index2;
				directedGraph.push_back(*vertex);
				delete vertex;
			}

			allLines.push_back(i);
			edge = new Edge_Node();
			edge->adjEdge = allLines.size() - 1;
			edge->adjvex = index2;
			edge->EdgeLine = i;
			edge->visited = false;
			edge->isAddLine = false;
			edge->next = directedGraph[index1].next;
			directedGraph[index1].next = edge;

			directedGraph[index1].back.push_back(index2);
			directedGraph[index2].front.push_back(index1);
		}
		//构建内轮廓图
		for (const auto &i : this->inner)
		{
			for (const auto &j : i)
			{
				p1 = j.m_CtrlPts[0];
				p2 = j.m_CtrlPts[2];
				index1 = searchVertexIndex(p1);
				index2 = searchVertexIndex(p2);
				if (index1 == -1)
				{
					index1 = directedGraph.size();
					vertex = new Vertex_Node();
					vertex->next = NULL;
					vertex->vertex = p1;
					vertex->vertexIndex = index1;
					directedGraph.push_back(*vertex);
					delete vertex;
				}

				if (index2 == -1)
				{
					index2 = directedGraph.size();
					vertex = new Vertex_Node();
					vertex->next = NULL;
					vertex->vertex = p2;
					vertex->vertexIndex = index2;
					directedGraph.push_back(*vertex);
					delete vertex;
				}

				allLines.push_back(j);
				edge = new Edge_Node();
				edge->adjEdge = allLines.size() - 1;
				edge->adjvex = index2;
				edge->EdgeLine = j;
				edge->visited = false;
				edge->isAddLine = false;
				edge->next = directedGraph[index1].next;
				directedGraph[index1].next = edge;
				directedGraph[index1].back.push_back(index2);
				directedGraph[index2].front.push_back(index1);
			}
		}

		for (const auto &i : this->addLines)
		{
			p1 = i.m_CtrlPts[0];
			p2 = i.m_CtrlPts[2];
			index1 = searchVertexIndex(p1);
			index2 = searchVertexIndex(p2);
			if (index1 == -1)
			{
				index1 = directedGraph.size();
				vertex = new Vertex_Node();
				vertex->next = NULL;
				vertex->vertex = p1;
				vertex->vertexIndex = index1;
				directedGraph.push_back(*vertex);
				delete vertex;
			}

			if (index2 == -1)
			{
				index2 = directedGraph.size();
				vertex = new Vertex_Node();
				vertex->next = NULL;
				vertex->vertex = p2;
				vertex->vertexIndex = index2;
				directedGraph.push_back(*vertex);
				delete vertex;
			}

			allLines.push_back(i);
			edge = new Edge_Node();
			edge->adjEdge = allLines.size() - 1;
			edge->adjvex = index2;
			edge->EdgeLine = i;
			edge->visited = false;
			edge->isAddLine = true;
			edge->next = directedGraph[index1].next;
			directedGraph[index1].next = edge;

			edge = new Edge_Node();
			edge->adjEdge = allLines.size() - 1;
			edge->adjvex = index1;
			edge->EdgeLine = i;
			edge->visited = false;
			edge->isAddLine = true;
			edge->next = directedGraph[index2].next;
			directedGraph[index2].next = edge;

			directedGraph[index1].back.push_back(index2);
			directedGraph[index2].front.push_back(index1);
			directedGraph[index2].back.push_back(index1);
			directedGraph[index1].front.push_back(index2);
		}


		rwg.WriteSpline("E:\\Model\\PlaneQuad\\Delaunay\\ChackLines.txt", this->allLines);
		
		for (const auto &i : directedGraph)
		{
			cout << "顶点：" << i.vertexIndex << endl;
			cout << "出:";
			for (const auto &j : i.back)
			{
				cout << j << ", ";
			}
			cout << endl;
			cout << "入:";
			for (const auto &j : i.front)
			{
				cout << j << ", ";
			}
			cout << endl;
			cout << endl;
		}
	}

	bool JudgeTwoLinesCoincide(const Spline &line1, const Spline &line2)
	{
		Vec4 p1 = line1.m_CtrlPts[0];
		Vec4 p2 = line1.m_CtrlPts[2];
		Vec4 p3 = line2.m_CtrlPts[0];
		Vec4 p4 = line2.m_CtrlPts[2];

		if (JudgeTwoPointsCoincide(p1, p3) && JudgeTwoPointsCoincide(p2, p4))
		{
			return true;
		}
		if (JudgeTwoPointsCoincide(p1, p2) && JudgeTwoPointsCoincide(p3, p3))
		{
			return true;
		}
		return false;
	}

	//深度优先搜索
	bool dfs(int index,Edge_Node* theEdge)
	{
		
		//判断该顶点的出度是否为零
		if (outOfDegree[index] == 0)
		{
			path.push_back(index);
			return false;
		}
		//与起始点重合,构成完整的环
		if (path.size() >= 3 && index == path[0])
		{
			for (const auto &i : path)
			{
				cout << i << " ";
			}
			cout << endl;
			path.push_back(index);
			return true;
		}
		else
		{
			//若与子路径构成环，则返回false
			for (const auto &i : path)
			{
				if (i == index)
				{
					path.push_back(index);
					return false;
				}
			}
		}

		bool hasAddline = false;
		bool hasCircle = false;
		auto next = directedGraph[index].next;
		path.push_back(index);
		decltype(next) first;


		//判断是否有连接线，优先走存在连接线的路
		while (next != NULL)
		{
			//next未被访问过，且next为连接线
			if (!next->visited && next->isAddLine)
			{
				hasAddline = true;
				first = next;
				break;
			}
			next = next->next;
		}
		
		//有相邻连接线,优先走连接线
		if (hasAddline)
		{
			first->visited = true;
			outOfDegree[index]--;
			//防止走回头路
			if (theEdge == NULL || !JudgeTwoLinesCoincide(theEdge->EdgeLine, first->EdgeLine))
			{
				hasCircle = dfs(first->adjvex, first);
				//无合格的环
				if (!hasCircle)
				{
					//回退
					first->visited = false;
					outOfDegree[index]++;
				}
				path.pop_back();
			}
			
		}
		first = directedGraph[index].next;
		//没有相邻连接线
		while (first!=NULL)
		{
			if (first->isAddLine)
			{
				first = first->next;
				continue;
			}
			//该边是否访问过
			if (!first->visited)
			{
				outOfDegree[index]--;
				bool circle = false;
				first->visited = true;
				circle = dfs(first->adjvex, first);

				//这条路上不存在合格的环
				if (!circle)
				{	
					//回退
					first->visited = false;
					outOfDegree[index]++;
				}
				//这条路上存在合格的环
				else
				{
					hasCircle = true;
				}
				path.pop_back();
			}
			first = first->next;

		}
		return hasCircle;
	}
	void getPath()
	{
		
		bool hasCircle = false;
		inOfDegree.resize(directedGraph.size(), 0);
		outOfDegree.resize(directedGraph.size(), 0);

		//统计地点的入度与出度
		for (const auto &i : directedGraph)
		{
			inOfDegree[i.vertexIndex] = i.front.size();
			outOfDegree[i.vertexIndex] = i.back.size();
		}
		for (const auto &i : directedGraph)
		{
			if (outOfDegree.size())
			{
				dfs(i.vertexIndex, NULL);
			}
		}
	}
private:
	varray<Vertex_Node> directedGraph;
	int vertexNum;
	varray<Spline> outer;
	varray<varray<Spline>>inner;
	varray<Spline>addLines;
	varray<Edge_Node> allEdges;
	varray<Vertex_Node> allVertexts;
	varray<Spline> allLines;
	varray<int> inOfDegree;
	varray<int> outOfDegree;
	varray<int> path;
	varray<Edge_Node*>pathOfEdge;
	varray<bool> visited;
	varray<varray<int>> allPolygon;
	//调试用
	RWGeometric rwg;
	varray<Spline> tempLines;
	Spline tempLine;
};
class CreateAddlineByDelaunay
{
public:
	CreateAddlineByDelaunay(const varray<Spline> &outer, const varray<varray<Spline>> &inner)
	{
		m_allBoundary.push_back(outer);
		for (const auto &i : inner)
		{
			m_allBoundary.push_back(i);
		}
		createAddline();
	}
	void createAddline()
	{
		SfCtainTreeNode * root = ps.CreateTree(m_allBoundary);
		queue<SfCtainTreeNode *> nodes;
		SfCtainTreeNode* cur = nullptr;//临时指向当前包含树节点
		varray<Spline>temp;

		nodes.push(root);
		varray<Spline> outLine;//当前节点的轮廓线
		varray<varray<Spline>> inLines;//当前节点所有的内轮廓线
		//层次遍历包含树
		while (!nodes.empty())
		{

			outLine.clear();
			inLines.clear();
			cur = nodes.front();
			nodes.pop();
			if (!cur->childs.empty())
			{
				outLine = cur->outLines;
				temp = outLine;
				for (auto it = cur->childs.begin(); it != cur->childs.end(); it++)
				{
					nodes.push(*it);
					inLines.push_back((*it)->outLines);
					for (const auto &i : (*it)->outLines)
					{
						temp.push_back(i);
					}
				}
				rwg.WriteSpline("E:\\Model\\PlaneQuad\\Delaunay\\ChackLines.txt", temp);
				
				CDT_Operate cdto(outLine, inLines);
				for (const auto &i : cdto.addLine)
				{
					m_addLine.push_back(i);
				}
			}
		}
	}

public:
	varray<varray<Spline>> m_allBoundary;
	varray<Spline>m_addLine;
	varray<Spline> outer;
	varray<varray<Spline>> inner;
	PublicSolution ps;
	RWGeometric rwg;
};

class TheEdge {
public:
	int target_index; // 邻接顶点的索引
	double angle; // 与x轴的夹角
	bool visited;
	int boundary_index;
	int edge_index;
	TheEdge(int target_index, const Vec4& source, const Vec4& target, const int &boundary_index, const int &edge_index) :
		target_index(target_index),
		boundary_index(boundary_index),
		edge_index(edge_index)
	{
		// 计算边与x轴正方向的夹角，取值范围为[0,2π]
		double dx = target.x - source.x;
		double dy = target.y - source.y;
		angle = atan2(dy, dx);
		if (angle < 0) angle += 2 * PI; // 调整为在0到2π之间
		visited = false;
	}
};
class Graph {
private:
	vector<Vec4> vertices; // 顶点表
	vector<vector<TheEdge>> adj_list; // 邻接表
	vector<int> path;
	vector<pair<int, int>> edge_index;
	vector<vector<int>>all_boundary;
	int num_vertices;
	RWGeometric rwg;
	varray<Spline> area;
	varray<Spline> addLines;
	varray<Spline> outer;
	varray<varray<Spline>>inner;
	varray<varray<Spline>> all_area;

public:
	Graph(varray<Spline> outer, varray<varray<Spline>>inner, varray<Spline> addLines) :
		outer(outer),
		inner(inner),
		addLines(addLines)
	{
		PublicSolution ps;
		int start = 0;
		int end = -1;
		this->num_vertices = 0;
		ps.orderEdgeClockwise(this->outer);
		if (outer.size() > 0)
		{
			add_vertex(outer[0].m_CtrlPts[0]);
			end++;
		}
		for (int i = 0; i < outer.size()-1; ++i)
		{
			add_vertex(outer[i].m_CtrlPts[outer[i].m_CtrlPts.size()-1]);
			++end;
			add_edge(end - 1, end,-1,i);
			
		}
		add_edge(end, start,-1, outer.size()-1);
		start = end+1;

		for (int i = 0; i < inner.size(); ++i)
		{
			ps.orderEdgeAntiClock0(this->inner[i]);
			if (this->inner[i].size() > 0)
			{
				add_vertex(this->inner[i][0].m_CtrlPts[0]);
				end++;
			}
			for (int j = 0; j < this->inner[i].size()-1; j++)
			{
				add_vertex(this->inner[i][j].m_CtrlPts[this->inner[i][j].m_CtrlPts.size() - 1]);
				++end;
				add_edge(end - 1, end,i,j);
			}
			add_edge(end, start,i, this->inner[i].size() - 1);
			start = end + 1;
		}

		for (int i = 0; i < this->addLines.size(); ++i)
		{
			decltype(vertices.begin()) start_index = find(vertices.begin(), vertices.end(), this->addLines[i].m_CtrlPts[0]);
			decltype(vertices.begin()) end_index = find(vertices.begin(), vertices.end(), this->addLines[i].m_CtrlPts[this->addLines[i].m_CtrlPts.size() - 1]);
			if (start_index == vertices.end() || end_index == vertices.end())
			{
				exit(0);
			}
			else
			{
				start = start_index - vertices.begin();
				end = end_index - vertices.begin();
			}
			add_edge(start, end, -2,i);
			add_edge(end, start,-2,i);
		}
		varray<Spline> temp;
		for (const auto &i : this->outer)
		{
			temp.push_back(i);
		}

		for (const auto &i : this->inner)
		{
			for (const auto &j : i)
			{
				temp.push_back(j);
			}
		}
		for (auto &i : addLines)
		{
			temp.push_back(i);
		}
		rwg.WriteSpline("E:\\Model\\PlaneQuad\\AllSpline.txt", temp);
	}

	//添加顶点
	void add_vertex(const Vec4& v) {
		vertices.push_back(v);
		adj_list.push_back(vector<TheEdge>());
	}
	//添加边
	void add_edge(int start_vertex_index, int end_vertex_index, int boundary_index,int edge_index) {
		adj_list[start_vertex_index].push_back(TheEdge(end_vertex_index, vertices[start_vertex_index], vertices[end_vertex_index], boundary_index, edge_index));
	}

	const vector<TheEdge>& get_outgoing_edges(int vertex_index) const {
		return adj_list[vertex_index];
	}

	const Vec4& get_vertex(int vertex_index)  const {
		return vertices[vertex_index];
	}

	const vector<Vec4>& get_vertices()  const {
		return vertices;
	}

	const vector<TheEdge>& get_adj_list(int vertex_index)
	{
		return adj_list[vertex_index];
	}
	vector<int>& get_path()
	{
		return path;
	}
	void print_adj_list() const {
		for (int i = 0; i < vertices.size(); ++i) {
			cout << "顶点 " << i << " (" << vertices[i].x << ", " << vertices[i].y << ") 的出边: ";
			for (const TheEdge& edge : adj_list[i]) {
				if (edge.visited)
				{
					continue;
				}
				cout << edge.target_index /*<< "（" << edge.angle / M_PI * 180 << "°) "*/ << " ";
			}
			cout << endl;
		}
	}

	//输出为遍历的边
	void print_unvisited_edges(const unordered_set<int>& visited_edges) const {
		cout << "未访问的边: " << endl;
		for (int i = 0; i < adj_list.size(); ++i) {
			for (const TheEdge& edge : adj_list[i]) {
				if (visited_edges.find(edge.target_index) == visited_edges.end()) {
					cout << "顶点 " << i << " 到顶点 " << edge.target_index << endl;
				}
			}
		}
	}

	bool if_boundary()
	{
		bool isBoundary = false;
		bool flag;
		for (auto& i : all_boundary)
		{
			flag = true;
			for (auto& j : path)
			{
				if (find(i.begin(), i.end(), j) == i.end())
				{
					flag = false;
					break;
				}
			}
			if (flag)
			{
				return true;
			}

		}
		return false;
	}

	//获取所有未访问边的旋转角度
	multimap<double, int, greater<double>> get_rotation_angle(int vertex_index, const double& lastAngle)
	{
		multimap<double, int, greater<double>> all_angle;
		multimap<double, int, greater<double>> all_angle_than_PI;
		const vector<TheEdge>& outgoing_edges = adj_list[vertex_index];
		for (int i = 0; i < outgoing_edges.size(); ++i) {
			if (outgoing_edges[i].visited)
			{
				continue;
			}
			//计算出顺时针旋转的角度
			double rotation_angle = outgoing_edges[i].angle - lastAngle;

			if (rotation_angle < 0)
			{
				rotation_angle *= -1;
			}
			else
			{
				rotation_angle = 2 * PI - rotation_angle;
			}
			if (rotation_angle < PI)
			{
				all_angle.insert(make_pair(rotation_angle, i));
			}
			else
			{
				rotation_angle = 2 * PI - rotation_angle;
				all_angle_than_PI.insert(make_pair(rotation_angle, i));
			}
		}
		if (all_angle.size() == 0)
		{
			return all_angle_than_PI;
		}
		return all_angle;
	}

	void setVisited(const int &vertex_index, const int &edge_index, bool value)
	{
		adj_list[vertex_index][edge_index].visited = value;

	}

	bool dfs(int current_vertex_index, const double &last_angle, int start_vertex_index)
	{
		//构成环，输出结果
		if (current_vertex_index == start_vertex_index)
		{
			area.clear();
			if (if_boundary())
			{
				return false;
			}
			for (const auto& i : path)
			{
				cout << i << " ";
			}
			rwg.WriteSpline("E:\\Model\\PlaneQuad\\Area.txt", getArea());
			all_area.push_back(getArea());
			cout << endl;
			return true;
		}

		bool ret = false;
		bool flag = false;
		double angle = last_angle;
		multimap<double, int, greater<double>> all_angle = get_rotation_angle(current_vertex_index, last_angle);

		//排序好边的优先级，按优先级旋转下一个边
		for (auto it = all_angle.begin(); it != all_angle.end(); ++it)
		{
			//记录角度
			angle = get_adj_list(current_vertex_index).at(it->second).angle;

			//记录下一个顶点的序号
			int next_vertex_index = get_adj_list(current_vertex_index).at(it->second).target_index;

			//防止走回头路，提高程序效率
			decltype(path.begin()) temp = find(path.begin(), path.end(), next_vertex_index);
			if (temp == path.begin() && path.size() == 2)
			{
				continue;
			}
			if (path.end() != temp && temp != path.begin())
			{
				continue;
			}
			//存入路径
			path.push_back(next_vertex_index);
			edge_index.push_back(make_pair(get_adj_list(current_vertex_index).at(it->second).boundary_index, get_adj_list(current_vertex_index).at(it->second).edge_index));
			//路径设为已访问
			setVisited(current_vertex_index, it->second, true);

			//以下一点为起点，继续深度优先搜索，并返回结果
			//若结果为true则存在回路，若为false,则不存在回路，访问过的要撤回
			ret = dfs(next_vertex_index, angle, start_vertex_index);
			if (!flag)
			{
				flag = ret;
			}
			setVisited(current_vertex_index, it->second, ret);
			path.pop_back();
			edge_index.pop_back();
			if (flag)
			{
				return flag;
			}
		}
		return flag;
	}

	varray<Spline> getArea()
	{
		varray<Spline> area;
		for (int i = 0; i < edge_index.size();++i)
		{
			if (edge_index[i].first == -1)
			{
				area.push_back(outer[edge_index[i].second]);
			}
			else if (edge_index[i].first == -2)
			{
				area.push_back(addLines[edge_index[i].second]);
			}
			else
			{
				area.push_back(inner[edge_index[i].first][edge_index[i].second]);
			}
		}
		return area;
	}

	varray<varray<Spline>>get_all_area()
	{
		cout << "有向图的邻接表：" << endl;
		print_adj_list();
		double angle = 0;
		// 从第一个顶点开始遍历，选择下一条边时选择逆时针旋转角度最小的边
		for (int i = 0; i < vertices.size(); ++i)
		{
			path.push_back(i);

			for (int j = 0; j < adj_list[i].size(); ++j)
			{
				if (adj_list[i].at(j).visited)
				{
					continue;
				}
				//存入路径
				path.push_back(adj_list[i].at(j).target_index);
				edge_index.push_back(make_pair(adj_list[i].at(j).boundary_index, adj_list[i].at(j).edge_index));
				//路径设为已访问
				setVisited(i, j, true);
				bool ret = dfs(adj_list[i].at(j).target_index, adj_list[i].at(j).angle, i);
				//路径设为已访问
				setVisited(i, j, ret);
				edge_index.pop_back();
				path.pop_back();
			}
			path.pop_back();
		}
		return all_area;
	}
};