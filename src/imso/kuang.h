#pragma once
#include "FeatureNetwork.h"

#include "SplineVolume.h"
#include "NurbsTrans.h"
#include "Option.h"w

#include "PublicModels.h"
#include <ctime>
#include "PublicModels.h"

/*
  加约束和力
  path: 文件路径 
  wc: 第几片施加约束 
  wf: 第几片施加力
  输入生成的控制点序号就可以将力加在对应的控制点上
  直接在模型函数中调用下面的test函数就行
*/
////齿轮用的
//void setWCandWF0(string path, int wc, int wf)
//{
//	RWGeometric rwg;
//	varray<SplineVolume> NVS;
//	rwg.ReadSplineVolume(path + ".txt", NVS);
//	TestBolcks::pList plt;
//	Model_Solution M;
//	varray<varray<SplineSurface>> NS = M.GetSurfaces(NVS);
//	varray<SplineSurface> NSf;
//	for (int i = 0; i < NS.size(); i++)
//	{
//		for (int j = 0; j < NS[i].size(); j++)
//		{
//			NSf.push_back(NS[i][j]);
//		}
//	}
//	rwg.WriteSplineSurface(path + "排序面.txt", NSf);
//	//创建存放施加约束面的容器
//	varray<SplineSurface> WC;
//	WC.push_back(NSf[wc]);
//	//六齿
//	int temp = 31;
//	WC.push_back(NSf[temp]);
//	int temp1 = 132;
//	WC.push_back(NSf[temp1]);
//	int temp2 = 138;
//	WC.push_back(NSf[temp2]);
//	for (int i = 0;i < 2;i++) {
//		WC.push_back(NSf[temp+=36]);
//		WC.push_back(NSf[temp1+=36]);
//		WC.push_back(NSf[temp2+=36]);
//	}
//	
//	//全齿
//	int temp = wc;
//	int temp1 = 31;
//	WC.push_back(NSf[31]);
//	int temp2 = 1104;
//	WC.push_back(NSf[1104]);
//	int temp3 = 1110;
//	WC.push_back(NSf[1110]);
//	for (int i = 0;i < 29;i++) {
//		temp+=36;
//		temp1+=36;
//		temp2+=36;
//		temp3+=36;
//		WC.push_back(NSf[temp]);
//		WC.push_back(NSf[temp1]);
//		WC.push_back(NSf[temp2]);
//		WC.push_back(NSf[temp3]);
//	}
//	//创建并初始化存放施加约束控制点的容器对象
//	varray<varray<int>>WCidx = plt.getfaceidx(NVS, WC);
//	//plt.showdata();
//	cout << "WC" << " ";
//	int num = 0;
//	for (int i = 0; i < WCidx.size(); i++)
//	{
//		for (int j = 0; j < WCidx[i].size(); j++)
//		{
//			num++;
//			cout << WCidx[i][j] << " ";
//		}
//	}
//	cout << endl;
//
//	//创建存放施加力的面片容器
//	varray<SplineSurface> WF;
//	WF.push_back(NSf[wf]);
//	WF.push_back(NSf[146]);
//	//创建并初始化存放施加力控制点的容器对象
//	varray<varray<int>>WFidx = plt.getfaceidx(NVS, WF);
//	cout << "WF" << " ";
//	for (int i = 0; i < WFidx.size(); i++)
//	{
//		for (int j = 0; j < WFidx[i].size(); j++)
//		{
//			cout << WFidx[i][j] << " ";
//		}
//	}
//}
//
//void test0()
//{
//	RWGeometric rwg;
//	varray<SplineVolume> NVs;
//	rwg.ReadSplineVolume("E:\\kuang_models\\part1.txt", NVs);
//	Model_Solution M;
//	varray<varray<SplineSurface>> NFs;
//	NFs = M.GetSurfaces(NVs);
//	varray<SplineSurface> NF;
//	for (auto& SF : NFs) {
//		for (auto& i : SF) {
//			NF.push_back(i);
//		}
//	}
//	//rwg.WriteSplineSurface("E:\\kuang_models\\part1_surfaces.txt", NF);
//	TestBolcks::pList plist;
//	plist.OutputParaVolumeDataTxt(NVs, "E:\\kuang_models\\part1.txt");
//	setWCandWF0("E:\\kuang_models\\part1", 4, 1048);
//}


/*************************************************************************************/
									/*公用模型*/
/*************************************************************************************/

//	四分之一圆筒
class Circle {
public:
	/*  左边上半圆筒 r:内圆半径 R1:外圆半径*/
	SplineSurface rec_circle(double r,double R1) {
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

		m.Rolate(ss, PI / 2, 3);
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

//	外圆柱-内长方体
class Cylinder_rec {
public:
	/*l:内部正方形边长 r:外圆半径 */
	varray<SplineSurface> cylinder_rec(double r, double l) {
		varray<Spline> SL1;
		varray<Spline> SL2;
		varray<Spline> SL3;
		varray<Spline> SL4;
		varray<Spline> SL5;
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
		SL5.resize(4);
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
			SL5[i].m_Degree = 2;
			SL5[i].m_Knots = knots;
		}
	
		Vec4 p01 = { -l / 2,-l / 2,0,1 };
		Vec4 p02 = { -l / 2,l / 2,0,1 };
		Vec4 p03 = { l / 2,l / 2,0,1 };
		Vec4 p04 = { l / 2,-l / 2,0,1 };

		Vec4 p05 = { -sqrt(2)*r / 2 ,-sqrt(2)*r / 2 ,0,1 };
		Vec4 p06 = { -sqrt(2)*r / 2 ,sqrt(2)*r / 2 ,0,1 };
		Vec4 p07 = { sqrt(2)*r / 2 ,sqrt(2)*r / 2 ,0,1 };
		Vec4 p08 = { sqrt(2)*r / 2 ,-sqrt(2)*r / 2 ,0,1 };
		
		Vec4 p09 = { -sqrt(2)*r, 0,0,w };
		Vec4 p10 = { 0 ,sqrt(2)*r,0,w };
		Vec4 p11 = { sqrt(2)*r, 0,0,w };
		Vec4 p12 = { 0,-sqrt(2)*r,0,w };

		SL1[0].m_CtrlPts.push_back(p05);
		SL1[0].m_CtrlPts.push_back(p09);
		SL1[0].m_CtrlPts.push_back(p06);

		SL1[1].m_CtrlPts.push_back(p06);
		SL1[1].m_CtrlPts.push_back((p06 + p02) / 2);
		SL1[1].m_CtrlPts[1].w = 1;
		SL1[1].m_CtrlPts.push_back(p02);

		SL1[2].m_CtrlPts.push_back(p01);
		SL1[2].m_CtrlPts.push_back((p01 + p02) / 2);
		SL1[2].m_CtrlPts[1].w = 1;
		SL1[2].m_CtrlPts.push_back(p02);

		SL1[3].m_CtrlPts.push_back(p05);
		SL1[3].m_CtrlPts.push_back((p05 + p01) / 2);
		SL1[3].m_CtrlPts[1].w = 1;
		SL1[3].m_CtrlPts.push_back(p01);

		SplineSurface S1;
		S1.CoonsInterpolate(SL1);
		SS.push_back(S1);

		SL2[0].m_CtrlPts.push_back(p02);
		SL2[0].m_CtrlPts.push_back((p02 + p06) / 2);
		SL2[0].m_CtrlPts[1].w = 1;
		SL2[0].m_CtrlPts.push_back(p06);

		SL2[1].m_CtrlPts.push_back(p06);
		SL2[1].m_CtrlPts.push_back(p10);
		SL2[1].m_CtrlPts.push_back(p07);

		SL2[2].m_CtrlPts.push_back(p03);
		SL2[2].m_CtrlPts.push_back((p03 + p07) / 2);
		SL2[2].m_CtrlPts[1].w = 1;
		SL2[2].m_CtrlPts.push_back(p07);

		SL2[3].m_CtrlPts.push_back(p02);
		SL2[3].m_CtrlPts.push_back((p02 + p03) / 2);
		SL2[3].m_CtrlPts[1].w = 1;
		SL2[3].m_CtrlPts.push_back(p03);

		SplineSurface S2;
		S2.CoonsInterpolate(SL2);
		SS.push_back(S2);

		SL3[0].m_CtrlPts.push_back(p04);
		SL3[0].m_CtrlPts.push_back((p04 + p03) / 2);
		SL3[0].m_CtrlPts[1].w = 1;
		SL3[0].m_CtrlPts.push_back(p03);

		SL3[1].m_CtrlPts.push_back(p03);
		SL3[1].m_CtrlPts.push_back((p07 + p03) / 2);
		SL3[1].m_CtrlPts[1].w = 1;
		SL3[1].m_CtrlPts.push_back(p07);

		SL3[2].m_CtrlPts.push_back(p08);
		SL3[2].m_CtrlPts.push_back(p11);
		SL3[2].m_CtrlPts.push_back(p07);

		SL3[3].m_CtrlPts.push_back(p04);
		SL3[3].m_CtrlPts.push_back((p04 + p08) / 2);
		SL3[3].m_CtrlPts[1].w = 1;
		SL3[3].m_CtrlPts.push_back(p08);

		SplineSurface S3;
		S3.CoonsInterpolate(SL3);
		SS.push_back(S3);

		SL4[0].m_CtrlPts.push_back(p05);
		SL4[0].m_CtrlPts.push_back((p05 + p01) / 2);
		SL4[0].m_CtrlPts[1].w = 1;
		SL4[0].m_CtrlPts.push_back(p01);

		SL4[1].m_CtrlPts.push_back(p01);
		SL4[1].m_CtrlPts.push_back((p04 + p01) / 2);
		SL4[1].m_CtrlPts[1].w = 1;
		SL4[1].m_CtrlPts.push_back(p04);

		SL4[2].m_CtrlPts.push_back(p08);
		SL4[2].m_CtrlPts.push_back((p08 + p04) / 2);
		SL4[2].m_CtrlPts[1].w = 1;
		SL4[2].m_CtrlPts.push_back(p04);

		SL4[3].m_CtrlPts.push_back(p05);
		SL4[3].m_CtrlPts.push_back(p12);
		SL4[3].m_CtrlPts.push_back(p08);

		SplineSurface S4;
		S4.CoonsInterpolate(SL4);
		SS.push_back(S4);

		SL5[0].m_CtrlPts.push_back(p01);
		SL5[0].m_CtrlPts.push_back((p02 + p01) / 2);
		SL5[0].m_CtrlPts[1].w = 1;
		SL5[0].m_CtrlPts.push_back(p02);

		SL5[1].m_CtrlPts.push_back(p02);
		SL5[1].m_CtrlPts.push_back((p02 + p03) / 2);
		SL5[1].m_CtrlPts[1].w = 1;
		SL5[1].m_CtrlPts.push_back(p03);

		SL5[2].m_CtrlPts.push_back(p04);
		SL5[2].m_CtrlPts.push_back((p04 + p03) / 2);
		SL5[2].m_CtrlPts[1].w = 1;
		SL5[2].m_CtrlPts.push_back(p03);

		SL5[3].m_CtrlPts.push_back(p01);
		SL5[3].m_CtrlPts.push_back((p01 + p04) / 2);
		SL5[3].m_CtrlPts[1].w = 1;
		SL5[3].m_CtrlPts.push_back(p04);

		SplineSurface S5;
		S5.CoonsInterpolate(SL5);
		SS.push_back(S5);

		return SS;
	}
	/*r:外圆半径 l:内部正方形边长  h:高度*/
	varray<SplineVolume> get_vol(double r, double l, double h) {
		varray<SplineSurface> SS;
		SS = cylinder_rec(r, l);
		return m.CreatSweepVol(SS, h, 3);
	}
private:
	Model_Solution m;
	double w = cos(PI / 4);
};

//	矩形
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
		Vec4 p01 = { -a / 2,-b / 2,0,1 };
		Vec4 p02 = { -a / 2,b / 2,0,1 };
		Vec4 p03 = { a / 2,b / 2,0,1 };
		Vec4 p04 = { a / 2,-b / 2,0,1 };
		
		SL1[0].m_CtrlPts.push_back(p01);
		SL1[0].m_CtrlPts.push_back((p01 + p02) / 2);
		SL1[0].m_CtrlPts.push_back(p02);

		SL1[1].m_CtrlPts.push_back(p02);
		SL1[1].m_CtrlPts.push_back((p02 + p03) / 2);
		SL1[1].m_CtrlPts.push_back(p03);

		SL1[2].m_CtrlPts.push_back(p04);
		SL1[2].m_CtrlPts.push_back((p04 + p03) / 2);
		SL1[2].m_CtrlPts.push_back(p03);

		SL1[3].m_CtrlPts.push_back(p01);
		SL1[3].m_CtrlPts.push_back((p01 + p04) / 2);
		SL1[3].m_CtrlPts.push_back(p04);
		SplineSurface ss1;
		ss1.CoonsInterpolate(SL1);
		SS.push_back(ss1);
		return SS;
	}
	/*
		a:矩形长
		b:矩形宽
		h:厚度
		矩形中心为坐标原点
	*/
	varray<SplineVolume> get_vol(double a, double b,double h) {
		varray<SplineSurface> SS;
		SS = rec(a, b); 
		return m.CreatSweepVol(SS, h, 3);
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
private:
	double w = cos(PI / 4);
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
		Vec4 p05 = { 0,sqrt(2)*r,0,w};

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
		Vec4 p05 = {  0,-sqrt(2)*r,0,w};

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

//	梯形
class Cube_slice {
public:

	//等腰梯形
	varray<SplineSurface> cube_slice3(double a, double b, double h) {
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
		Vec4 p01 = { -b / 2,0,0,1 };
		Vec4 p02 = { -a / 2,h,0,1 };
		Vec4 p03 = { a / 2,h,0,1 };
		Vec4 p04 = { b / 2,0,0,1 };

		SL1[0].m_CtrlPts.push_back(p01);
		SL1[0].m_CtrlPts.push_back((p01 + p04) / 2);
		SL1[0].m_CtrlPts.push_back(p04);

		SL1[1].m_CtrlPts.push_back(p01);
		SL1[1].m_CtrlPts.push_back((p01 + p02) / 2);
		SL1[1].m_CtrlPts.push_back(p02);

		SL1[2].m_CtrlPts.push_back(p02);
		SL1[2].m_CtrlPts.push_back((p02 + p03) / 2);
		SL1[2].m_CtrlPts.push_back(p03);

		SL1[3].m_CtrlPts.push_back(p04);
		SL1[3].m_CtrlPts.push_back((p03 + p04) / 2);
		SL1[3].m_CtrlPts.push_back(p03);
		SplineSurface ss1;
		ss1.CoonsInterpolate(SL1);
		SS.push_back(ss1);

		return SS;
	}

	//等腰梯形
	/*a:上边长度 b:底边长度 h:高度 s:拉伸长度*/
	varray<SplineVolume> get_vol3(double a, double b, double h, double s) {
		varray<SplineSurface> SS;
		SS = cube_slice3(a, b, h);
		return m.CreatSweepVol(SS, s, 3);
	}

private:
	Model_Solution m;
};

//任意角度圆弧
class Circle1 {
public:
	Circle1() {};

	Circle1(double r,double a):r(r),a(a) {};

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
	double w=cos(a/2);//权重
	Vec3 v1 = { 0,1,0 };//+y轴单位向量
	Vec3 v0 = v1.RotateZ(-a / 2);
	Vec3 v2 = v1.RotateZ(a / 2);
};

//任意角度圆环
class Annulus {
public:
	double r1;//半径
	double r2;
	double a;//角度

	Annulus() {};

	Annulus(double r1,double r2,double a):r1(r1),r2(r2),a(a) {}

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
		for (int i = 0;i < 4;i++) {
			SL1[i].m_Degree = 2;
			SL1[i].m_Knots = knots;
		}
		
		Circle1 c1(r1,a), c2(r2,a);
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

/*************************************************************************************/
								 /*模型类*/
/*************************************************************************************/


/*
	燃料棒
*/
class Fuel_rod {
public:
	/*	
		l: 内部正方形边长 
		h: 燃料棒长度
		angle: 旋转角度 默认90° 逆时针旋转
		
	*/

	//7X3 7X7
	varray<SplineVolume> fuel_rod1(double l, double h,double angle) {
		varray<Spline> SL1;
		varray<Spline> SL2;
		varray<SplineSurface> SS;
		varray<SplineSurface> SS2;
		varray<double> knots1;
		varray<double> knots2;
		knots1.push_back(0);
		knots1.push_back(0);
		knots1.push_back(0);
		knots1.push_back(0.25);
		knots1.push_back(0.25);
		knots1.push_back(0.75);
		knots1.push_back(0.75);
		knots1.push_back(1);
		knots1.push_back(1);
		knots1.push_back(1);

		knots2.push_back(0);
		knots2.push_back(0);
		knots2.push_back(0);
		knots2.push_back(1);
		knots2.push_back(1);
		knots2.push_back(1);

		SL1.resize(4);
		SL2.resize(4);
		SL1[0].m_Degree = 2;
		SL1[0].m_Knots = knots2;
		SL1[1].m_Degree = 2;
		SL1[1].m_Knots = knots1;
		SL1[2].m_Degree = 2;
		SL1[2].m_Knots = knots2;
		SL1[3].m_Degree = 2;
		SL1[3].m_Knots = knots1;

		SL2[0].m_Degree = 2;
		SL2[0].m_Knots = knots1;
		SL2[1].m_Degree = 2;
		SL2[1].m_Knots = knots1;
		SL2[2].m_Degree = 2;
		SL2[2].m_Knots = knots1;
		SL2[3].m_Degree = 2;
		SL2[3].m_Knots = knots1;

		double w = sqrt(2) / 2;
		double a = sqrt(2)*l / 2;
		Vec4 p01 = { 0,6.5,0,1 };
		Vec4 p02 = { 2,6.5,0,w };
		Vec4 p03 = { 2,3.2504,0,1 };
		Vec4 p04 = { 2,2,0,w };
		Vec4 p05 = { 3.2504,2,0,1 };
		Vec4 p06 = { 6.5,2,0,w };
		Vec4 p07 = { 6.5,0,0,1 };

		Vec4 p08 = { sqrt(2)*l / 2,0,0,1 };
		Vec4 p09 = { sqrt(2)*l / 4 + 6.5 / 2,0,0,1 };


		Vec4 p10 = { 0,a,0,1 };
		Vec4 p11 = { a / 6,5 * a / 6,0,1 };
		Vec4 p12 = { 2 * a / 6,4 * a / 6,0,1 };
		Vec4 p13 = { 3 * a / 6,3 * a / 6,0,1 };
		Vec4 p14 = { 4 * a / 6,2 * a / 6,0,1 };
		Vec4 p15 = { 5 * a / 6,a / 6,0,1 };

		Vec4 p16 = { 0,sqrt(2)*l / 4 + 6.5 / 2,0,1 };

		Vec4 p17 = { 0,-a,0,1 };
		Vec4 p18 = { a / 6,-5 * a / 6,0,1 };
		Vec4 p19 = { 2 * a / 6,-4 * a / 6,0,1 };
		Vec4 p20 = { 3 * a / 6,-3 * a / 6,0,1 };
		Vec4 p21 = { 4 * a / 6,-2 * a / 6,0,1 };
		Vec4 p22 = { 5 * a / 6,-a / 6,0,1 };

		Vec4 p23 = { -a,0,0,1 };
		Vec4 p24 = { -5 * a / 6,-a / 6,0,1 };
		Vec4 p25 = { -4 * a / 6,-2 * a / 6,0,1 };
		Vec4 p26 = { -3 * a / 6,-3 * a / 6,0,1 };
		Vec4 p27 = { -2 * a / 6,-4 * a / 6,0,1 };
		Vec4 p28 = { -a / 6,-5 * a / 6,0,1 };

		Vec4 p29 = { -5 * a / 6,a / 6,0,1 };
		Vec4 p30 = { -4 * a / 6,2 * a / 6,0,1 };
		Vec4 p31 = { -3 * a / 6,3 * a / 6,0,1 };
		Vec4 p32 = { -2 * a / 6,4 * a / 6,0,1 };
		Vec4 p33 = { -a / 6,5 * a / 6,0,1 };

		SL1[0].m_CtrlPts.push_back(p10);
		SL1[0].m_CtrlPts.push_back(p16);
		SL1[0].m_CtrlPts.push_back(p01);

		SL1[1].m_CtrlPts.push_back(p01);
		SL1[1].m_CtrlPts.push_back(p02);
		SL1[1].m_CtrlPts.push_back(p03);
		SL1[1].m_CtrlPts.push_back(p04);
		SL1[1].m_CtrlPts.push_back(p05);
		SL1[1].m_CtrlPts.push_back(p06);
		SL1[1].m_CtrlPts.push_back(p07);

		SL1[2].m_CtrlPts.push_back(p08);
		SL1[2].m_CtrlPts.push_back(p09);
		SL1[2].m_CtrlPts.push_back(p07);

		SL1[3].m_CtrlPts.push_back(p10);
		SL1[3].m_CtrlPts.push_back(p11);
		SL1[3].m_CtrlPts.push_back(p12);
		SL1[3].m_CtrlPts.push_back(p13);
		SL1[3].m_CtrlPts.push_back(p14);
		SL1[3].m_CtrlPts.push_back(p15);
		SL1[3].m_CtrlPts.push_back(p08);

		SplineSurface ss1;
		ss1.CoonsInterpolate(SL1);
		SS.push_back(ss1);

		SL2[0].m_CtrlPts.push_back(p23);
		SL2[0].m_CtrlPts.push_back(p29);
		SL2[0].m_CtrlPts.push_back(p30);
		SL2[0].m_CtrlPts.push_back(p31);
		SL2[0].m_CtrlPts.push_back(p32);
		SL2[0].m_CtrlPts.push_back(p33);
		SL2[0].m_CtrlPts.push_back(p10);

		SL2[1].m_CtrlPts.push_back(p10);
		SL2[1].m_CtrlPts.push_back(p11);
		SL2[1].m_CtrlPts.push_back(p12);
		SL2[1].m_CtrlPts.push_back(p13);
		SL2[1].m_CtrlPts.push_back(p14);
		SL2[1].m_CtrlPts.push_back(p15);
		SL2[1].m_CtrlPts.push_back(p08);

		SL2[2].m_CtrlPts.push_back(p17);
		SL2[2].m_CtrlPts.push_back(p18);
		SL2[2].m_CtrlPts.push_back(p19);
		SL2[2].m_CtrlPts.push_back(p20);
		SL2[2].m_CtrlPts.push_back(p21);
		SL2[2].m_CtrlPts.push_back(p22);
		SL2[2].m_CtrlPts.push_back(p08);

		SL2[3].m_CtrlPts.push_back(p23);
		SL2[3].m_CtrlPts.push_back(p24);
		SL2[3].m_CtrlPts.push_back(p25);
		SL2[3].m_CtrlPts.push_back(p26);
		SL2[3].m_CtrlPts.push_back(p27);
		SL2[3].m_CtrlPts.push_back(p28);
		SL2[3].m_CtrlPts.push_back(p17);
		SplineSurface ss2;
		ss2.CoonsInterpolate(SL2);
		SS2.push_back(ss2);

		Model_Solution M;
		M.Rolate(ss1, PI, 2);
		M.Trans(ss1, h, -3);
		M.Rolate(ss1, angle, 3);
		SS.push_back(ss1);
		//中间正方形的变换
		M.Rolate(ss2, PI, 2);
		M.Trans(ss2, h, -3);
		M.Rolate(ss2, angle, 3);
		SS2.push_back(ss2);

		//放样路径是两面上的连线
		Spline NL;
		varray<double> knots;

		knots.push_back(0);
		knots.push_back(0);
		knots.push_back(0);
		knots.push_back(1);
		knots.push_back(1);
		knots.push_back(1);

		NL.m_Degree = 2;
		NL.m_Knots = knots;

		Vec4 u1 = { a / 2,a / 2,0,1 };
		Vec4 u2 = { a / 2,a / 2,h / 2,1 };
		Vec4 u3 = { a / 2,a / 2,h ,1 };

		NL.m_CtrlPts.push_back(u1);
		NL.m_CtrlPts.push_back(u2);
		NL.m_CtrlPts.push_back(u3);

		varray<Spline> SL;
		SL.push_back(NL);

		NurbsLine nl;
		varray<NurbsSurface> ns;
		varray<NurbsSurface> ns2;
		NurbsVol nv;
		NurbsVol nv2;
		nl = NurbsTrans::SplineToCnurbsline(NL);
		ns = NurbsTrans::SplinesurfsToCsurfs(SS);
		ns2 = NurbsTrans::SplinesurfsToCsurfs(SS2);
		nv.LoftingNurbsVol(nl, ns[0], ns[1]);
		nv2.LoftingNurbsVol(nl, ns2[0], ns2[1]);

		SplineVolume sv;
		sv = NurbsTrans::CnurbsvolToSplinevol(nv);
		SplineVolume sv2;
		sv2 = NurbsTrans::CnurbsvolToSplinevol(nv2);

		varray<SplineVolume> SV;
		varray<SplineVolume> SV1;
		SV.push_back(sv);
		SV1.push_back(sv);
		M.Rolate(SV, PI / 2, 3);
		SV1.push_back(SV[0]);
		M.Rolate(SV, PI / 2, 3);
		SV1.push_back(SV[0]);
		M.Rolate(SV, PI / 2, 3);
		SV1.push_back(SV[0]);
		SV1.push_back(sv2);

		return SV1;
	}
	//3X3
	varray<SplineVolume> fuel_rod2(double h, double angle) {
		varray<Spline> SL1;
		varray<Spline> SL2;
		varray<Spline> SL3;
		varray<SplineSurface> SS;
		varray<double> knots1;
		knots1.push_back(0);
		knots1.push_back(0);
		knots1.push_back(0);
		knots1.push_back(1);
		knots1.push_back(1);
		knots1.push_back(1);

		SL1.resize(4);
		SL2.resize(4);
		SL3.resize(4);
		
		for (int i = 0; i < 4; i++) {
			SL1[i].m_Degree = 2;
			SL1[i].m_Knots = knots1;
			SL2[i].m_Degree = 2;
			SL2[i].m_Knots = knots1;
			SL3[i].m_Degree = 2;
			SL3[i].m_Knots = knots1;
		}

		double w = cos(PI / 4);

		Vec4 p01 = { 0,0,0,1 };
		Vec4 p02 = { -2,3.25,0,1 };
		Vec4 p03 = { -2,6.5,0,w };
		Vec4 p04 = { 0,6.5,0,1 };
		Vec4 p05 = { 2,6.5,0,w };
		Vec4 p06 = { 2,3.25,0,1 };
		Vec4 p07 = { 2,2,0,w };
		Vec4 p08 = { 3.25,2,0,1 };
		Vec4 p09 = { 6.5,2,0,w };

		Vec4 p10 = { 6.5,0,0,1 };
		Vec4 p11 = { -6.5,0,0,1 };
		Vec4 p12 = { -6.5,2,0,w };
		Vec4 p13 = { -3.25,2,0,1 };
		Vec4 p14 = { -2,2,0,w };

		SL1[0].m_CtrlPts.push_back(p01);
		SL1[0].m_CtrlPts.push_back((p02 + p01 )/ 2);
		SL1[0].m_CtrlPts.push_back(p02);

		SL1[1].m_CtrlPts.push_back(p02);
		SL1[1].m_CtrlPts.push_back(p03);
		SL1[1].m_CtrlPts.push_back(p04);

		SL1[2].m_CtrlPts.push_back(p06);
		SL1[2].m_CtrlPts.push_back(p05);
		SL1[2].m_CtrlPts.push_back(p04);

		SL1[3].m_CtrlPts.push_back(p01);
		SL1[3].m_CtrlPts.push_back((p06 + p01 )/ 2);
		SL1[3].m_CtrlPts.push_back(p06);

		SplineSurface ss1;
		ss1.CoonsInterpolate(SL1);
		SS.push_back(ss1);

		SL2[0].m_CtrlPts.push_back(p01);
		SL2[0].m_CtrlPts.push_back((p06 + p01 )/ 2);
		SL2[0].m_CtrlPts.push_back(p06);

		SL2[1].m_CtrlPts.push_back(p06);
		SL2[1].m_CtrlPts.push_back(p07);
		SL2[1].m_CtrlPts.push_back(p08);

		SL2[2].m_CtrlPts.push_back(p10);
		SL2[2].m_CtrlPts.push_back(p09);
		SL2[2].m_CtrlPts.push_back(p08);

		SL2[3].m_CtrlPts.push_back(p01);
		SL2[3].m_CtrlPts.push_back((p01 + p10 )/ 2);
		SL2[3].m_CtrlPts.push_back(p10);

		SplineSurface ss2;
		ss2.CoonsInterpolate(SL2);
		SS.push_back(ss2);

		SL3[0].m_CtrlPts.push_back(p01);
		SL3[0].m_CtrlPts.push_back((p01 + p11 )/ 2);
		SL3[0].m_CtrlPts.push_back(p11);

		SL3[1].m_CtrlPts.push_back(p11);
		SL3[1].m_CtrlPts.push_back(p12);
		SL3[1].m_CtrlPts.push_back(p13);

		SL3[2].m_CtrlPts.push_back(p02);
		SL3[2].m_CtrlPts.push_back(p14);
		SL3[2].m_CtrlPts.push_back(p13);

		SL3[3].m_CtrlPts.push_back(p01);
		SL3[3].m_CtrlPts.push_back((p01 + p02 )/ 2);
		SL3[3].m_CtrlPts.push_back(p02);

		SplineSurface ss3;
		ss3.CoonsInterpolate(SL3);
		SS.push_back(ss3);

		varray<SplineSurface> SS1;
		for (int i = 0; i < SS.size(); i++) {
			SS1.push_back(SS[i]);
		}
		Model_Solution M;
		M.Rolate(SS1, PI, 3);
		/*M.Trans(SS1, h, -3);
		M.Rolate(SS1, angle, 3);*/
		for (int i = 0; i < SS1.size(); i++) {
			SS.push_back(SS1[i]);
		}
		varray<SplineVolume> SV;
		SV = M.CreatSweepVol(SS, h, 3);

		/*//放样路径是两面上的连线
		Spline NL;
		varray<double> knots;

		knots.push_back(0);
		knots.push_back(0);
		knots.push_back(0);
		knots.push_back(1);
		knots.push_back(1);
		knots.push_back(1);

		NL.m_Degree = 2;
		NL.m_Knots = knots;

		Vec4 u1 = { 1,1,0,1 };
		Vec4 u2 = { 1,1,h / 2,1 };
		Vec4 u3 = { 1,1,h ,1 };

		NL.m_CtrlPts.push_back(u1);
		NL.m_CtrlPts.push_back(u2);
		NL.m_CtrlPts.push_back(u3);

		varray<Spline> SL;
		SL.push_back(NL);

		NurbsLine nl;
		varray<NurbsSurface> ns;
		varray<NurbsVol> nv;
		varray<NurbsVol> sv;
		nl = NurbsTrans::SplineToCnurbsline(NL);
		ns = NurbsTrans::SplinesurfsToCsurfs(SS);
		nv[0].LoftingNurbsVol(nl, ns[0], ns[3]);

		nv[1].LoftingNurbsVol(nl, ns[1], ns[5]);
		nv[2].LoftingNurbsVol(nl, ns[2], ns[4]);
		for (int i = 0; i < nv.size(); i++) {
			sv.push_back(nv[i]);
		}
		varray<SplineVolume> SV;
		varray<SplineVolume> SV1;
		SV = NurbsTrans::CvolsToSplinevols(sv);
		
		for (int i = 0; i < SV.size(); i++) {
			SV1.push_back(SV[i]);
		}
		M.Rolate(SV1, PI, 3);
		for (int i = 0; i < SV1.size(); i++) {
			SV.push_back(SV1[i]);
		}*/

		return SV;
	}

	////原始状态
	//varray<SplineVolume> fuelRod3(double r1, double r2) {
	//	Circle0 c(r1, PI / 2);
	//	Spline sl;
	//	sl = c.getCircle();
	//	m.Rolate(sl, PI / 4, 3);

	//	Spline sl1;
	//	sl1 = sl;

	//	m.Trans(sl, r1 + r2, -1);
	//	m.Trans()


	//	Circle0 c1(r2, PI / 2);
	//	Spline sl1;
	//	sl1 = c1.getCircle();
	//	m.Rolate(sl1, PI * 3 / 4, 3);

	//	
	//}
	
	void setWCandWF(string path, int wc, int wf)
	{
		RWGeometric rwg;
		varray<SplineVolume> NVS;
		rwg.ReadSplineVolume(path + ".txt", NVS);
		TestBolcks::pList plt;
		Model_Solution M;
		varray<varray<SplineSurface>> NS = M.GetSurfaces(NVS);
		varray<SplineSurface> NSf;
		for (int i = 0; i < NS.size(); i++)
		{
			for (int j = 0; j < NS[i].size(); j++)
			{
				NSf.push_back(NS[i][j]);
			}
		}
		rwg.WriteSplineSurface(path + "排序面.txt", NSf);
		//创建存放施加约束面的容器
		varray<SplineSurface> WC;
		WC.push_back(NSf[wc]);
		WC.push_back(NSf[10]);
		WC.push_back(NSf[16]);
		WC.push_back(NSf[22]);
		WC.push_back(NSf[28]);
		WC.push_back(NSf[34]);
		//创建并初始化存放施加约束控制点的容器对象
		varray<varray<int>>WCidx = plt.getfaceidx(NVS, WC);
		//plt.showdata();
		cout << "WC" << " ";
		int num = 0;
		for (int i = 0; i < WCidx.size(); i++)
		{
			for (int j = 0; j < WCidx[i].size(); j++)
			{
				num++;
				cout << WCidx[i][j] << " ";
			}
		}
		cout << endl;

		//创建存放施加力的面片容器
		varray<SplineSurface> WF;
		WF.push_back(NSf[wf]);
		WF.push_back(NSf[11]);
		WF.push_back(NSf[17]);
		WF.push_back(NSf[23]);
		WF.push_back(NSf[29]);
		WF.push_back(NSf[35]);
		//创建并初始化存放施加力控制点的容器对象
		varray<varray<int>>WFidx = plt.getfaceidx(NVS, WF);
		cout << "WF" << " ";
		for (int i = 0; i < WFidx.size(); i++)
		{
			for (int j = 0; j < WFidx[i].size(); j++)
			{
				cout << WFidx[i][j] << " ";
			}
		}
	}
	void test()
	{
		RWGeometric rwg;
		varray<SplineVolume> NVs;
		rwg.ReadSplineVolume("E:\\kuang_models\\fuel_rod3.txt", NVs);
		Model_Solution M;
		varray<varray<SplineSurface>> NFs;
		NFs = M.GetSurfaces(NVs);
		varray<SplineSurface> NF;
		for (auto& SF : NFs) {
			for (auto& i : SF) {
				NF.push_back(i);
			}
		}
		//rwg.WriteSplineSurface("E:\\kuang_models\\getSurface1.txt", NF);
		TestBolcks::pList plist;
		plist.OutputParaVolumeDataTxt(NVs, "E:\\kuang_models\\fuel3");
		setWCandWF("E:\\kuang_models\\fuel_rod3", 4, 5);

	}

	Model_Solution m;
};

/*
	两个相对四分之一圆柱
*/
class Cylinder {
public:
	/*原始版本 两个相对的 四分之一圆柱 r: 圆弧半径 h: 高度 num: 细化次数*/
	varray<SplineVolume> cylinder1(double r, double h, int num) {
		//创建容器，存放线
		//存放正方形的四条边
		varray<Spline> SL1;
		varray<Spline> SL2;
		varray<Spline> SL3;
		varray<Spline> SL4;
		varray<Spline> SL5;
		varray<Spline> SL6;
		//创建容器，存放曲面,用于多面拉伸
		varray<SplineSurface> SS;
		//存放节点 p=2
		varray<double> knots;
		knots.push_back(0);
		knots.push_back(0);
		knots.push_back(0);
		knots.push_back(1);
		knots.push_back(1);
		knots.push_back(1);
		//指定大小，可存放四条线，用于Coons插值成面
		SL1.resize(4);
		SL2.resize(4);
		SL3.resize(4);
		SL4.resize(4);
		SL5.resize(4);
		SL6.resize(4);
		for (int i = 0; i < 4; i++)
		{
			//给定次数、节点矢量
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
		//下半部分
		double w = cos(PI / 8);
		Vec4 p01 = { 0,0,0,1 };
		Vec4 p02 = { 0,r / 2,0,1 };
		Vec4 p03 = { r / 2,r / 2,0,1 };	
		Vec4 p04 = { r / 2,0,0,1 };
		Vec4 p05 = { sqrt(2)*r / 2,sqrt(2)*r / 2,0,1 };
		Vec4 p06 = { r,tan(PI / 8)*r,0,w };
		Vec4 p07 = { r,0,0,1 };
		Vec4 p08 = { 0,r,0,1 };
		Vec4 p09 = { tan(PI / 8)*r,r,0,w };

		Vec4 p10 = { 0,-r,0,1 };
		Vec4 p11 = { 0,-r / 2,0,1 };
		Vec4 p12 = { r / 2,-r / 2,0,1 };

		Vec4 p13 = { sqrt(2)*r / 2,-sqrt(2)*r / 2,0,1 };
		Vec4 p14 = { tan(PI / 8)*r,-r,0,w };
		Vec4 p15 = { r,-tan(PI / 8)*r,0,w };

		//将点放到线容器中
		//用coons插值成正方形面，点的存放是有顺序的，顺时针，从左到右，从下到上
		SL1[0].m_CtrlPts.push_back(p01);
		SL1[0].m_CtrlPts.push_back((p02 + p01) / 2);
		SL1[0].m_CtrlPts.push_back(p02);

		SL1[1].m_CtrlPts.push_back(p02);
		SL1[1].m_CtrlPts.push_back((p02 + p03) / 2);
		SL1[1].m_CtrlPts.push_back(p03);

		SL1[2].m_CtrlPts.push_back(p04);
		SL1[2].m_CtrlPts.push_back((p04 + p03) / 2);
		SL1[2].m_CtrlPts.push_back(p03);

		SL1[3].m_CtrlPts.push_back(p01);
		SL1[3].m_CtrlPts.push_back((p04 + p01) / 2);
		SL1[3].m_CtrlPts.push_back(p04);

		SplineSurface S1;
		S1.CoonsInterpolate(SL1);

		//插值成右旁边的面
		SL2[0].m_CtrlPts.push_back(p04);
		SL2[0].m_CtrlPts.push_back((p04 + p03) / 2);
		SL2[0].m_CtrlPts.push_back(p03);

		SL2[1].m_CtrlPts.push_back(p03);
		SL2[1].m_CtrlPts.push_back((p03 + p05) / 2);
		SL2[1].m_CtrlPts.push_back(p05);

		SL2[2].m_CtrlPts.push_back(p07);
		SL2[2].m_CtrlPts.push_back(p06);
		SL2[2].m_CtrlPts.push_back(p05);

		SL2[3].m_CtrlPts.push_back(p04);
		SL2[3].m_CtrlPts.push_back((p07 + p04) / 2);
		SL2[3].m_CtrlPts.push_back(p07);

		SplineSurface S2;
		S2.CoonsInterpolate(SL2);

		//插值成上边的面
		SL3[0].m_CtrlPts.push_back(p02);
		SL3[0].m_CtrlPts.push_back((p02 + p08) / 2);
		SL3[0].m_CtrlPts.push_back(p08);

		SL3[1].m_CtrlPts.push_back(p08);
		SL3[1].m_CtrlPts.push_back(p09);
		SL3[1].m_CtrlPts.push_back(p05);

		SL3[2].m_CtrlPts.push_back(p03);
		SL3[2].m_CtrlPts.push_back((p03 + p05) / 2);
		SL3[2].m_CtrlPts.push_back(p05);

		SL3[3].m_CtrlPts.push_back(p02);
		SL3[3].m_CtrlPts.push_back((p02 + p03) / 2);
		SL3[3].m_CtrlPts.push_back(p03);

		SplineSurface S3;
		S3.CoonsInterpolate(SL3);

		//上边体的最下面
		SL4[0].m_CtrlPts.push_back(p10);
		SL4[0].m_CtrlPts.push_back((p11 + p10) / 2);
		SL4[0].m_CtrlPts.push_back(p11);

		SL4[1].m_CtrlPts.push_back(p11);
		SL4[1].m_CtrlPts.push_back((p11 + p12) / 2);
		SL4[1].m_CtrlPts.push_back(p12);

		SL4[2].m_CtrlPts.push_back(p13);
		SL4[2].m_CtrlPts.push_back((p13 + p12) / 2);
		SL4[2].m_CtrlPts.push_back(p12);

		SL4[3].m_CtrlPts.push_back(p10);
		SL4[3].m_CtrlPts.push_back(p14);
		SL4[3].m_CtrlPts.push_back(p13);

		SplineSurface S4;
		S4.CoonsInterpolate(SL4);

		SL5[0].m_CtrlPts.push_back(p12);
		SL5[0].m_CtrlPts.push_back((p12 + p04) / 2);
		SL5[0].m_CtrlPts.push_back(p04);

		SL5[1].m_CtrlPts.push_back(p04);
		SL5[1].m_CtrlPts.push_back((p04 + p07) / 2);
		SL5[1].m_CtrlPts.push_back(p07);

		SL5[2].m_CtrlPts.push_back(p13);
		SL5[2].m_CtrlPts.push_back(p15);
		SL5[2].m_CtrlPts.push_back(p07);

		SL5[3].m_CtrlPts.push_back(p12);
		SL5[3].m_CtrlPts.push_back((p12 + p13) / 2);
		SL5[3].m_CtrlPts.push_back(p13);

		SplineSurface S5;
		S5.CoonsInterpolate(SL5);

		SL6[0].m_CtrlPts.push_back(p11);
		SL6[0].m_CtrlPts.push_back((p01 + p11) / 2);
		SL6[0].m_CtrlPts.push_back(p01);

		SL6[1].m_CtrlPts.push_back(p01);
		SL6[1].m_CtrlPts.push_back((p01 + p04) / 2);
		SL6[1].m_CtrlPts.push_back(p04);

		SL6[2].m_CtrlPts.push_back(p12);
		SL6[2].m_CtrlPts.push_back((p04 + p12) / 2);
		SL6[2].m_CtrlPts.push_back(p04);

		SL6[3].m_CtrlPts.push_back(p11);
		SL6[3].m_CtrlPts.push_back((p11 + p12) / 2);
		SL6[3].m_CtrlPts.push_back(p12);

		SplineSurface S6;
		S6.CoonsInterpolate(SL6);

		Model_Solution M;
		varray<SplineSurface> NSS;
		varray<SplineSurface> NSS1;
		NSS.push_back(S1);
		NSS.push_back(S2);
		NSS.push_back(S3);

		NSS1.push_back(S4);
		NSS1.push_back(S5);
		NSS1.push_back(S6);
		M.Trans(NSS1, 2 * r - 0.002, 2);

		for (int i = 0; i < 3; i++) {
			NSS.push_back(NSS1[i]);
		}

		//多面拉伸成体
		varray<SplineVolume> SV;
		SV = M.CreatSweepVol(NSS, h, 3);

		for (int i = 0; i < SV.size(); i++) {
			SV[i].KnotsRefineNum(num);
		}

		return SV;
		/*
		//嵌套类调用
		//第一种方式
		TestBolcks::pList p;
		//第二种方式
		//输出的文件适用于分析
		TestBolcks::pList().OutputParaVolumeDataTxt(SV, "E:\\kuang_models\\SplineVolumePara2.txt");
		*/
	}

	varray<SplineVolume> cylinder2(double r, double h, int num) {
		//创建容器，存放线
		//存放正方形的四条边
		varray<Spline> SL1;
		varray<Spline> SL2;
		varray<Spline> SL3;
		varray<Spline> SL4;
		varray<Spline> SL5;
		varray<Spline> SL6;
		//创建容器，存放曲面,用于多面拉伸
		varray<SplineSurface> SS;
		//存放节点 p=2
		varray<double> knots;
		knots.push_back(0);
		knots.push_back(0);
		knots.push_back(0);
		knots.push_back(1);
		knots.push_back(1);
		knots.push_back(1);
		//指定大小，可存放四条线，用于Coons插值成面
		SL1.resize(4);
		SL2.resize(4);
		SL3.resize(4);
		SL4.resize(4);
		SL5.resize(4);
		SL6.resize(4);
		for (int i = 0; i < 4; i++)
		{
			//给定次数、节点矢量
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
		//下半部分
		double w = cos(PI / 8);
		Vec4 p01 = { 0,0,0,1 };
		Vec4 p02 = { 0,r / 2,0,1 };
		Vec4 p03 = { r / 2,r / 2,0,1 };
		Vec4 p04 = { r / 2,0,0,1 };
		Vec4 p05 = { sqrt(2)*r / 2,sqrt(2)*r / 2,0,1 };
		Vec4 p06 = { r,tan(PI / 8)*r,0,w };
		Vec4 p07 = { r,0,0,1 };
		Vec4 p08 = { 0,r,0,1 };
		Vec4 p09 = { tan(PI / 8)*r,r,0,w };

		Vec4 p10 = { 0,-r,0,1 };
		Vec4 p11 = { 0,-r / 2,0,1 };
		Vec4 p12 = { r / 2,-r / 2,0,1 };

		Vec4 p13 = { sqrt(2)*r / 2,-sqrt(2)*r / 2,0,1 };
		Vec4 p14 = { tan(PI / 8)*r,-r,0,w };
		Vec4 p15 = { r,-tan(PI / 8)*r,0,w };

		//将点放到线容器中
		//用coons插值成正方形面，点的存放是有顺序的，顺时针，从左到右，从下到上
		SL1[0].m_CtrlPts.push_back(p11);
		SL1[0].m_CtrlPts.push_back((p11 + p01) / 2);
		SL1[0].m_CtrlPts.push_back(p01);

		SL1[1].m_CtrlPts.push_back(p01);
		SL1[1].m_CtrlPts.push_back((p01 + p04) / 2);
		SL1[1].m_CtrlPts.push_back(p04);

		SL1[2].m_CtrlPts.push_back(p12);
		SL1[2].m_CtrlPts.push_back((p12 + p04) / 2);
		SL1[2].m_CtrlPts.push_back(p04);

		SL1[3].m_CtrlPts.push_back(p11);
		SL1[3].m_CtrlPts.push_back((p11 + p12) / 2);
		SL1[3].m_CtrlPts.push_back(p12);

		SplineSurface S1;
		S1.CoonsInterpolate(SL1);

		//插值成右旁边的面
		SL2[0].m_CtrlPts.push_back(p12);
		SL2[0].m_CtrlPts.push_back((p12 + p04) / 2);
		SL2[0].m_CtrlPts.push_back(p04);

		SL2[1].m_CtrlPts.push_back(p04);
		SL2[1].m_CtrlPts.push_back((p04 + p07) / 2);
		SL2[1].m_CtrlPts.push_back(p07);

		SL2[2].m_CtrlPts.push_back(p13);
		SL2[2].m_CtrlPts.push_back(p15);
		SL2[2].m_CtrlPts.push_back(p07);

		SL2[3].m_CtrlPts.push_back(p12);
		SL2[3].m_CtrlPts.push_back((p12 + p13) / 2);
		SL2[3].m_CtrlPts.push_back(p13);

		SplineSurface S2;
		S2.CoonsInterpolate(SL2);

		//插值成上边的面
		SL3[0].m_CtrlPts.push_back(p10);
		SL3[0].m_CtrlPts.push_back((p10 + p11) / 2);
		SL3[0].m_CtrlPts.push_back(p11);

		SL3[1].m_CtrlPts.push_back(p11);
		SL3[1].m_CtrlPts.push_back((p12 + p11) / 2);
		SL3[1].m_CtrlPts.push_back(p12);

		SL3[2].m_CtrlPts.push_back(p13);
		SL3[2].m_CtrlPts.push_back((p13 + p12) / 2);
		SL3[2].m_CtrlPts.push_back(p12);

		SL3[3].m_CtrlPts.push_back(p10);
		SL3[3].m_CtrlPts.push_back(p14);
		SL3[3].m_CtrlPts.push_back(p13);

		SplineSurface S3;
		S3.CoonsInterpolate(SL3);

		SL4[0].m_CtrlPts.push_back(p02);
		SL4[0].m_CtrlPts.push_back((p02 + p08) / 2);
		SL4[0].m_CtrlPts.push_back(p08);

		SL4[1].m_CtrlPts.push_back(p08);
		SL4[1].m_CtrlPts.push_back(p09);
		SL4[1].m_CtrlPts.push_back(p05);

		SL4[2].m_CtrlPts.push_back(p03);
		SL4[2].m_CtrlPts.push_back((p03 + p05) / 2);
		SL4[2].m_CtrlPts.push_back(p05);

		SL4[3].m_CtrlPts.push_back(p02);
		SL4[3].m_CtrlPts.push_back((p02 + p03) / 2);
		SL4[3].m_CtrlPts.push_back(p03);

		SplineSurface S4;
		S4.CoonsInterpolate(SL4);

		SL5[0].m_CtrlPts.push_back(p04);
		SL5[0].m_CtrlPts.push_back((p03 + p04) / 2);
		SL5[0].m_CtrlPts.push_back(p03);

		SL5[1].m_CtrlPts.push_back(p03);
		SL5[1].m_CtrlPts.push_back((p03 + p05) / 2);
		SL5[1].m_CtrlPts.push_back(p05);

		SL5[2].m_CtrlPts.push_back(p07);
		SL5[2].m_CtrlPts.push_back(p06);
		SL5[2].m_CtrlPts.push_back(p05);

		SL5[3].m_CtrlPts.push_back(p04);
		SL5[3].m_CtrlPts.push_back((p04 + p07) / 2);
		SL5[3].m_CtrlPts.push_back(p07);

		SplineSurface S5;
		S5.CoonsInterpolate(SL5);

		SL6[0].m_CtrlPts.push_back(p01);
		SL6[0].m_CtrlPts.push_back((p01 + p02) / 2);
		SL6[0].m_CtrlPts.push_back(p02);

		SL6[1].m_CtrlPts.push_back(p02);
		SL6[1].m_CtrlPts.push_back((p02 + p03) / 2);
		SL6[1].m_CtrlPts.push_back(p03);

		SL6[2].m_CtrlPts.push_back(p04);
		SL6[2].m_CtrlPts.push_back((p04 + p03) / 2);
		SL6[2].m_CtrlPts.push_back(p03);

		SL6[3].m_CtrlPts.push_back(p01);
		SL6[3].m_CtrlPts.push_back((p04 + p01) / 2);
		SL6[3].m_CtrlPts.push_back(p04);

		SplineSurface S6;
		S6.CoonsInterpolate(SL6);

		Model_Solution M;
		varray<SplineSurface> NSS;
		varray<SplineSurface> NSS1;
		NSS.push_back(S1);
		NSS.push_back(S2);
		NSS.push_back(S3);

		NSS1.push_back(S4);
		NSS1.push_back(S5);
		NSS1.push_back(S6);
		M.Trans(NSS, 2 * r - 0.002, 2);

		for (int i = 0; i < 3; i++) {
			NSS.push_back(NSS1[i]);
		}

		//多面拉伸成体
		varray<SplineVolume> SV;
		SV = M.CreatSweepVol(NSS, h, 3);

		for (int i = 0; i < SV.size(); i++) {
			SV[i].KnotsRefineNum(num);
		}

		return SV;
		/*
		//嵌套类调用
		//第一种方式
		TestBolcks::pList p;
		//第二种方式
		//输出的文件适用于分析
		TestBolcks::pList().OutputParaVolumeDataTxt(SV, "E:\\kuang_models\\SplineVolumePara2.txt");
		*/
	}

	varray<SplineVolume> cylinder3(double r, double h, int num) {
		//创建容器，存放线
		//存放正方形的四条边
		varray<Spline> SL1;
		varray<Spline> SL2;
		varray<Spline> SL3;
		varray<Spline> SL4;
		varray<Spline> SL5;
		varray<Spline> SL6;
		//创建容器，存放曲面,用于多面拉伸
		varray<SplineSurface> SS;
		//存放节点 p=2
		varray<double> knots;
		knots.push_back(0);
		knots.push_back(0);
		knots.push_back(0);
		knots.push_back(1);
		knots.push_back(1);
		knots.push_back(1);
		//指定大小，可存放四条线，用于Coons插值成面
		SL1.resize(4);
		SL2.resize(4);
		SL3.resize(4);
		SL4.resize(4);
		SL5.resize(4);
		SL6.resize(4);
		for (int i = 0; i < 4; i++)
		{
			//给定次数、节点矢量
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
		//下半部分
		double w = cos(PI / 8);
		Vec4 p01 = { 0,0,0,1 };
		Vec4 p02 = { 0,r / 2,0,1 };
		Vec4 p03 = { r / 2,r / 2,0,1 };
		Vec4 p04 = { r / 2,0,0,1 };
		Vec4 p05 = { sqrt(2)*r / 2,sqrt(2)*r / 2,0,1 };
		Vec4 p06 = { r,tan(PI / 8)*r,0,w };
		Vec4 p07 = { r,0,0,1 };
		Vec4 p08 = { 0,r,0,1 };
		Vec4 p09 = { tan(PI / 8)*r,r,0,w };

		Vec4 p10 = { 0,-r,0,1 };
		Vec4 p11 = { 0,-r / 2,0,1 };
		Vec4 p12 = { r / 2,-r / 2,0,1 };

		Vec4 p13 = { sqrt(2)*r / 2,-sqrt(2)*r / 2,0,1 };
		Vec4 p14 = { tan(PI / 8)*r,-r,0,w };
		Vec4 p15 = { r,-tan(PI / 8)*r,0,w };

		//将点放到线容器中
		//用coons插值成正方形面，点的存放是有顺序的，顺时针，从左到右，从下到上
		SL1[0].m_CtrlPts.push_back(p11);
		SL1[0].m_CtrlPts.push_back((p11 + p01) / 2);
		SL1[0].m_CtrlPts.push_back(p01);

		SL1[1].m_CtrlPts.push_back(p01);
		SL1[1].m_CtrlPts.push_back((p01 + p04) / 2);
		SL1[1].m_CtrlPts.push_back(p04);

		SL1[2].m_CtrlPts.push_back(p12);
		SL1[2].m_CtrlPts.push_back((p12 + p04) / 2);
		SL1[2].m_CtrlPts.push_back(p04);

		SL1[3].m_CtrlPts.push_back(p11);
		SL1[3].m_CtrlPts.push_back((p11 + p12) / 2);
		SL1[3].m_CtrlPts.push_back(p12);

		SplineSurface S1;
		S1.CoonsInterpolate(SL1);

		//插值成右旁边的面
		SL2[0].m_CtrlPts.push_back(p12);
		SL2[0].m_CtrlPts.push_back((p12 + p04) / 2);
		SL2[0].m_CtrlPts.push_back(p04);

		SL2[1].m_CtrlPts.push_back(p04);
		SL2[1].m_CtrlPts.push_back((p04 + p07) / 2);
		SL2[1].m_CtrlPts.push_back(p07);

		SL2[2].m_CtrlPts.push_back(p13);
		SL2[2].m_CtrlPts.push_back(p15);
		SL2[2].m_CtrlPts.push_back(p07);

		SL2[3].m_CtrlPts.push_back(p12);
		SL2[3].m_CtrlPts.push_back((p12 + p13) / 2);
		SL2[3].m_CtrlPts.push_back(p13);

		SplineSurface S2;
		S2.CoonsInterpolate(SL2);

		//插值成上边的面
		SL3[0].m_CtrlPts.push_back(p10);
		SL3[0].m_CtrlPts.push_back((p10 + p11) / 2);
		SL3[0].m_CtrlPts.push_back(p11);

		SL3[1].m_CtrlPts.push_back(p11);
		SL3[1].m_CtrlPts.push_back((p12 + p11) / 2);
		SL3[1].m_CtrlPts.push_back(p12);

		SL3[2].m_CtrlPts.push_back(p13);
		SL3[2].m_CtrlPts.push_back((p13 + p12) / 2);
		SL3[2].m_CtrlPts.push_back(p12);

		SL3[3].m_CtrlPts.push_back(p10);
		SL3[3].m_CtrlPts.push_back(p14);
		SL3[3].m_CtrlPts.push_back(p13);

		SplineSurface S3;
		S3.CoonsInterpolate(SL3);

		SL4[0].m_CtrlPts.push_back(p01);
		SL4[0].m_CtrlPts.push_back((p01 + p02) / 2);
		SL4[0].m_CtrlPts.push_back(p02);

		SL4[1].m_CtrlPts.push_back(p02);
		SL4[1].m_CtrlPts.push_back((p03 + p02) / 2);
		SL4[1].m_CtrlPts.push_back(p03);

		SL4[2].m_CtrlPts.push_back(p04);
		SL4[2].m_CtrlPts.push_back((p04 + p04) / 2);
		SL4[2].m_CtrlPts.push_back(p04);

		SL4[3].m_CtrlPts.push_back(p01);
		SL4[3].m_CtrlPts.push_back((p01 + p04) / 2);
		SL4[3].m_CtrlPts.push_back(p04);

		SplineSurface S4;
		S4.CoonsInterpolate(SL4);

		SL5[0].m_CtrlPts.push_back(p04);
		SL5[0].m_CtrlPts.push_back((p03 + p04) / 2);
		SL5[0].m_CtrlPts.push_back(p03);

		SL5[1].m_CtrlPts.push_back(p03);
		SL5[1].m_CtrlPts.push_back((p03 + p05) / 2);
		SL5[1].m_CtrlPts.push_back(p05);

		SL5[2].m_CtrlPts.push_back(p07);
		SL5[2].m_CtrlPts.push_back(p06);
		SL5[2].m_CtrlPts.push_back(p05);

		SL5[3].m_CtrlPts.push_back(p04);
		SL5[3].m_CtrlPts.push_back((p04 + p07) / 2);
		SL5[3].m_CtrlPts.push_back(p07);

		SplineSurface S5;
		S5.CoonsInterpolate(SL5);

		SL6[0].m_CtrlPts.push_back(p02);
		SL6[0].m_CtrlPts.push_back((p08 + p02) / 2);
		SL6[0].m_CtrlPts.push_back(p08);

		SL6[1].m_CtrlPts.push_back(p08);
		SL6[1].m_CtrlPts.push_back(p09);
		SL6[1].m_CtrlPts.push_back(p05);

		SL6[2].m_CtrlPts.push_back(p03);
		SL6[2].m_CtrlPts.push_back((p05 + p03) / 2);
		SL6[2].m_CtrlPts.push_back(p05);

		SL6[3].m_CtrlPts.push_back(p02);
		SL6[3].m_CtrlPts.push_back((p03 + p02) / 2);
		SL6[3].m_CtrlPts.push_back(p03);

		SplineSurface S6;
		S6.CoonsInterpolate(SL6);

		Model_Solution M;
		varray<SplineSurface> NSS;
		varray<SplineSurface> NSS1;
		NSS.push_back(S1);
		NSS.push_back(S2);
		NSS.push_back(S3);

		NSS1.push_back(S4);
		NSS1.push_back(S5);
		NSS1.push_back(S6);
		M.Trans(NSS, 2 * r - 0.002, 2);

		for (int i = 0; i < 3; i++) {
			NSS.push_back(NSS1[i]);
		}

		//多面拉伸成体
		varray<SplineVolume> SV;
		SV = M.CreatSweepVol(NSS, h, 3);

		for (int i = 0; i < SV.size(); i++) {
			SV[i].KnotsRefineNum(num);
		}

		return SV;
		/*
		//嵌套类调用
		//第一种方式
		TestBolcks::pList p;
		//第二种方式
		//输出的文件适用于分析
		TestBolcks::pList().OutputParaVolumeDataTxt(SV, "E:\\kuang_models\\SplineVolumePara2.txt");
		*/
	}

	void setWCandWF(string path, int wc, int wf)
	{
		RWGeometric rwg;
		varray<SplineVolume> NVS;
		rwg.ReadSplineVolume(path + ".txt", NVS);
		TestBolcks::pList plt;
		Model_Solution M;
		varray<varray<SplineSurface>> NS = M.GetSurfaces(NVS);
		varray<SplineSurface> NSf;
		for (int i = 0; i < NS.size(); i++)
		{
			for (int j = 0; j < NS[i].size(); j++)
			{
				NSf.push_back(NS[i][j]);
			}
		}
		rwg.WriteSplineSurface(path + "排序面.txt", NSf);
		varray<SplineSurface> WC;
		WC.push_back(NSf[wc]);
		WC.push_back(NSf[9]);
		varray<varray<int>>WCidx = plt.getfaceidx(NVS, WC);
		//plt.showdata();
		cout << "WC" << " ";
		int num = 0;
		for (int i = 0; i < WCidx.size(); i++)
		{
			for (int j = 0; j < WCidx[i].size(); j++)
			{
				num++;
				cout << WCidx[i][j] << " ";
			}
		}
		cout << endl;
		varray<SplineSurface> WF;
		WF.push_back(NSf[wf]);
		WF.push_back(NSf[26]);
		varray<varray<int>>WFidx = plt.getfaceidx(NVS, WF);
		cout << "WF" << " ";
		for (int i = 0; i < WFidx.size(); i++)
		{
			for (int j = 0; j < WFidx[i].size(); j++)
			{
				cout << WFidx[i][j] << " ";
			}
		}
	}

	void test()
	{
		RWGeometric rwg;
		varray<SplineVolume> NVs;
		rwg.ReadSplineVolume("E:\\kuang_models\\linzhikai\\cylinder1-3.txt", NVs);
		Model_Solution M;
		varray<varray<SplineSurface>> NFs;
		NFs = M.GetSurfaces(NVs);
		varray<SplineSurface> NF;
		for (auto& SF : NFs) {
			for (auto& i : SF) {
				NF.push_back(i);
			}
		}
		//rwg.WriteSplineSurface("E:\\kuang_models\\getSurface1.txt", NF);
		TestBolcks::pList plist;
		plist.OutputParaVolumeDataTxt(NVs, "E:\\kuang_models\\cylinder1-10");
		setWCandWF("E:\\kuang_models\\cylinder1-10", 3, 20);
	}
};

/*
	测试类
*/
class Kuang_test {
public:
	//读取文本文件
	void readtest() {
		RWGeometric rw;
		varray<NurbsLine> nls;
		varray<NurbsSurface> sf;
		varray<NurbsVol> vols;
		//读文件，然后可以对文本文件里的模型进行操作 用法与写文件函数是一样的
		rw.ReadNurbsLine("D:\\冯文斌大论文\\素材\\示意图txt\\5.2\\Spline_v3.txt", nls);
		rw.ReadNurbsSurface("D:\\冯文斌大论文\\素材\\示意图txt\\5.2\\SplineSurface_v3.txt", sf);
		for (auto& s : sf) {
			NurbsVol v;
			v.CreateSweepNurbsVol(nls[0], s, 2);
			vols.push_back(v);
		}
		//写到文件中
		rw.WriteNurbsVol("D:\\冯文斌大论文\\素材\\示意图txt\\5.2\\SplineVolume_v3.txt", vols);
	}

	//二维线圆
	void Circle()
	{
		NurbsLine NL;
		NL.m_Degree = 2;
		NL.m_Knots.push_back(0);
		NL.m_Knots.push_back(0);
		NL.m_Knots.push_back(0);
		NL.m_Knots.push_back(0.25);
		NL.m_Knots.push_back(0.25);
		NL.m_Knots.push_back(0.5);
		NL.m_Knots.push_back(0.5);
		NL.m_Knots.push_back(0.75);
		NL.m_Knots.push_back(0.75);
		NL.m_Knots.push_back(1);
		NL.m_Knots.push_back(1);
		NL.m_Knots.push_back(1);

		//sqrt() 开根号
		double a = sqrt(2) / 2;
		point4d u1 = { 1,0,0,1 };
		point4d u2 = { 1,1,0,a };
		point4d u3 = { 0,1,0,1 };
		point4d u4 = { -1,1,0,a };
		point4d u5 = { -1,0,0,1 };
		point4d u6 = { -1,-1,0,a };
		point4d u7 = { 0,-1,0,1 };
		point4d u8 = { 1,-1,0,a };

		NL.m_CtrlPts.push_back(u1);
		NL.m_CtrlPts.push_back(u2);
		NL.m_CtrlPts.push_back(u3);
		NL.m_CtrlPts.push_back(u4);
		NL.m_CtrlPts.push_back(u5);
		NL.m_CtrlPts.push_back(u6);
		NL.m_CtrlPts.push_back(u7);
		NL.m_CtrlPts.push_back(u8);
		NL.m_CtrlPts.push_back(u1);

		varray<NurbsLine> NLS;
		NLS.push_back(NL);

		RWGeometric rwg;
		rwg.WriteNurbsLine("E:\\models\\nurbsline2.txt", NLS);
	}

	//二维圆面
	void Circle_Surface()
	{
		varray<double> u;
		varray<double> v;

		u.push_back(0);
		u.push_back(0);
		u.push_back(0);
		u.push_back(0.25);
		u.push_back(0.25);
		u.push_back(0.5);
		u.push_back(0.5);
		u.push_back(0.75);
		u.push_back(0.75);
		u.push_back(1);
		u.push_back(1);
		u.push_back(1);

		v.push_back(0);
		v.push_back(0);
		v.push_back(1);
		v.push_back(1);

		NurbsSurface NS;                //创建面对象
		NS.SetSurface(2, 1, 9, 2, u, v);//初始化

		//添加控制点
		//sqrt() 开根号
		double a = sqrt(2) / 2;
		point4d p1 = { 1,0,0,1 };
		point4d p2 = { 1,1,0,a };
		point4d p3 = { 0,1,0,1 };
		point4d p4 = { -1,1,0,a };
		point4d p5 = { -1,0,0,1 };
		point4d p6 = { -1,-1,0,a };
		point4d p7 = { 0,-1,0,1 };
		point4d p8 = { 1,-1,0,a };

		NS.m_CtrlPts.push_back(p1);
		NS.m_CtrlPts.push_back(p2);
		NS.m_CtrlPts.push_back(p3);
		NS.m_CtrlPts.push_back(p4);
		NS.m_CtrlPts.push_back(p5);
		NS.m_CtrlPts.push_back(p6);
		NS.m_CtrlPts.push_back(p7);
		NS.m_CtrlPts.push_back(p8);
		NS.m_CtrlPts.push_back(p1);

		varray<NurbsSurface> NSS;
		NSS.push_back(NS);

		RWGeometric rwg;
		rwg.WriteNurbsSurface("E:\\models\\nurbssurface0.txt", NSS);
	}

	//4X4X4立方体模型
	void Cube()
	{
		//建立面
		NurbsSurface NS;
		NS.m_uDegree = 2;
		NS.m_vDegree = 2;
		//控制点个数 
		NS.m_uNum = 4;
		NS.m_vNum = 4;


		NS.m_uKnots.push_back(0);
		NS.m_uKnots.push_back(0);
		NS.m_uKnots.push_back(0);
		NS.m_uKnots.push_back(0.5);
		NS.m_uKnots.push_back(1);
		NS.m_uKnots.push_back(1);
		NS.m_uKnots.push_back(1);

		NS.m_vKnots.push_back(0);
		NS.m_vKnots.push_back(0);
		NS.m_vKnots.push_back(0);
		NS.m_vKnots.push_back(0.5);
		NS.m_vKnots.push_back(1);
		NS.m_vKnots.push_back(1);
		NS.m_vKnots.push_back(1);

		//控制点
		point4d uv1 = { 0,0,0,1 };
		point4d uv2 = { 0,1,0,1 };
		point4d uv3 = { 0,2,0,1 };
		point4d uv4 = { 0,3,0,1 };

		point4d uv5 = { 1,0,0,1 };
		point4d uv6 = { 1,1,0,1 };
		point4d uv7 = { 1,2,0,1 };
		point4d uv8 = { 1,3,0,1 };

		point4d uv9 = { 2,0,0,1 };
		point4d uv10 = { 2,1,0,1 };
		point4d uv11 = { 2,2,0,1 };
		point4d uv12 = { 2,3,0,1 };

		point4d uv13 = { 3,0,0,1 };
		point4d uv14 = { 3,1,0,1 };
		point4d uv15 = { 3,2,0,1 };
		point4d uv16 = { 3,3,0,1 };



		NS.m_CtrlPts.push_back(uv1);
		NS.m_CtrlPts.push_back(uv2);
		NS.m_CtrlPts.push_back(uv3);
		NS.m_CtrlPts.push_back(uv4);
		NS.m_CtrlPts.push_back(uv5);
		NS.m_CtrlPts.push_back(uv6);
		NS.m_CtrlPts.push_back(uv7);
		NS.m_CtrlPts.push_back(uv8);
		NS.m_CtrlPts.push_back(uv9);
		NS.m_CtrlPts.push_back(uv10);
		NS.m_CtrlPts.push_back(uv11);
		NS.m_CtrlPts.push_back(uv12);
		NS.m_CtrlPts.push_back(uv13);
		NS.m_CtrlPts.push_back(uv14);
		NS.m_CtrlPts.push_back(uv15);
		NS.m_CtrlPts.push_back(uv16);

		//建立线 即扫描路径
		NurbsLine NL;
		NL.m_Degree = 2;
		NL.m_Knots.push_back(0);
		NL.m_Knots.push_back(0);
		NL.m_Knots.push_back(0);
		NL.m_Knots.push_back(0.5);
		NL.m_Knots.push_back(1);
		NL.m_Knots.push_back(1);
		NL.m_Knots.push_back(1);
		point4d p1 = { 0,0,0,1 };
		point4d p2 = { 0,0,1,1 };
		point4d p3 = { 0,0,2,1 };
		point4d p4 = { 0,0,3,1 };

		NL.m_CtrlPts.push_back(p1);
		NL.m_CtrlPts.push_back(p2);
		NL.m_CtrlPts.push_back(p3);
		NL.m_CtrlPts.push_back(p4);

		NurbsVol NV;
		NV.CreateTransSweepNurbsVol(NL, NS);
		NV.KnotsRefineNum(2);

		varray<NurbsVol> NVS;
		NVS.push_back(NV);

		RWGeometric rwg;
		rwg.WriteNurbsVol("E:\\models\\nurbsvol3.txt", NVS);
	}

	/*
		两个相对的 四分之一圆柱——给楷
		r: 圆弧半径
		h: 高度
		num: 细化次数
	*/
	void kai_vol(double r,double h,int num) {
		//创建容器，存放线
		//存放正方形的四条边
		varray<Spline> SL1;
		varray<Spline> SL2;
		varray<Spline> SL3;
		varray<Spline> SL4;
		varray<Spline> SL5;
		varray<Spline> SL6;
		//创建容器，存放曲面,用于多面拉伸
		varray<SplineSurface> SS;
		//存放节点 p=2
		varray<double> knots;
		knots.push_back(0);
		knots.push_back(0);
		knots.push_back(0);
		knots.push_back(1);
		knots.push_back(1);
		knots.push_back(1);
		//指定大小，可存放四条线，用于Coons插值成面
		SL1.resize(4);
		SL2.resize(4);
		SL3.resize(4);
		SL4.resize(4);
		SL5.resize(4);
		SL6.resize(4);
		for (int i = 0; i < 4; i++)
		{
			//给定次数、节点矢量
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
		//下半部分
		double w = cos(PI / 8);
		Vec4 p01 = { 0,0,0,1 };
		Vec4 p03 = { 0,r / 2,0,1 };
		Vec4 p02 = (p01 + p03) / 2;
		Vec4 p05 = { r / 2,r / 2,0,1 };
		Vec4 p04 = (p03 + p05) / 2;
		Vec4 p07 = { r / 2,0,0,1 };
		Vec4 p06 = (p05 + p07) / 2;
		Vec4 p08 = (p01 + p07) / 2;
		Vec4 p10 = { sqrt(2)*r / 2,sqrt(2)*r / 2,0,1 };
		Vec4 p09 = (p05 + p10) / 2;
		Vec4 p11 = { r,tan(PI / 8)*r,0,w };

		Vec4 p12 = { r,0,0,1 };
		Vec4 p13 = { 3 * r / 4,0,0,1 };
		Vec4 p15 = { 0,r,0,1 };
		Vec4 p14 = (p03 + p15) / 2;
		Vec4 p16 = { tan(PI / 8)*r,r,0,w };
		
		Vec4 p17 = { 0,-r,0,1 };
		Vec4 p19 = { 0,-r/2,0,1 };
		Vec4 p18 = (p17 + p19) / 2;
		Vec4 p21 = { r/2,-r/2,0,1 };
		Vec4 p20 = (p19 + p21) / 2;
	
		Vec4 p23 = { sqrt(2)*r / 2,-sqrt(2)*r / 2,0,1 };
		Vec4 p22 = (p21 + p23) / 2;
		Vec4 p24 = { tan(PI / 8)*r,-r,0,w };
		Vec4 p26 = { r / 2,0,0,1 };
		Vec4 p25 = (p21 + p26) / 2;
		Vec4 p28 = { r,0,0,1 };
		Vec4 p27 = (p26 + p28) / 2;
		Vec4 p29 = { r,-tan(PI / 8)*r,0,w };
		Vec4 p31 = { 0,0,0,1 };
		Vec4 p30 = (p31 + p19) / 2;
		Vec4 p32 = (p31 + p26) / 2;

		//将点放到线容器中
		//用coons插值成正方形面，点的存放是有顺序的，顺时针，从左到右，从下到上
		SL1[0].m_CtrlPts.push_back(p01);
		SL1[0].m_CtrlPts.push_back(p02);
		SL1[0].m_CtrlPts.push_back(p03);

		SL1[1].m_CtrlPts.push_back(p03);
		SL1[1].m_CtrlPts.push_back(p04);
		SL1[1].m_CtrlPts.push_back(p05);

		SL1[2].m_CtrlPts.push_back(p07);
		SL1[2].m_CtrlPts.push_back(p06);
		SL1[2].m_CtrlPts.push_back(p05);

		SL1[3].m_CtrlPts.push_back(p01);
		SL1[3].m_CtrlPts.push_back(p08);
		SL1[3].m_CtrlPts.push_back(p07);

		//实例化曲面对象
		SplineSurface S1;
		//Coons插值成正方形面
		S1.CoonsInterpolate(SL1);

		//插值成右旁边的面
		SL2[0].m_CtrlPts.push_back(p07);
		SL2[0].m_CtrlPts.push_back(p06);
		SL2[0].m_CtrlPts.push_back(p05);

		SL2[1].m_CtrlPts.push_back(p05);
		SL2[1].m_CtrlPts.push_back(p09);
		SL2[1].m_CtrlPts.push_back(p10);

		SL2[2].m_CtrlPts.push_back(p12);
		SL2[2].m_CtrlPts.push_back(p11);
		SL2[2].m_CtrlPts.push_back(p10);

		SL2[3].m_CtrlPts.push_back(p07);
		SL2[3].m_CtrlPts.push_back(p13);
		SL2[3].m_CtrlPts.push_back(p12);

		//实例化曲面对象
		SplineSurface S2;
		//Coons插值成右边面
		S2.CoonsInterpolate(SL2);

		//插值成上边的面
		SL3[0].m_CtrlPts.push_back(p03);
		SL3[0].m_CtrlPts.push_back(p14);
		SL3[0].m_CtrlPts.push_back(p15);

		SL3[1].m_CtrlPts.push_back(p15);
		SL3[1].m_CtrlPts.push_back(p16);
		SL3[1].m_CtrlPts.push_back(p10);

		SL3[2].m_CtrlPts.push_back(p05);
		SL3[2].m_CtrlPts.push_back(p09);
		SL3[2].m_CtrlPts.push_back(p10);

		SL3[3].m_CtrlPts.push_back(p03);
		SL3[3].m_CtrlPts.push_back(p04);
		SL3[3].m_CtrlPts.push_back(p05);

		//实例化曲面对象
		SplineSurface S3;
		//Coons插值成上边的面
		S3.CoonsInterpolate(SL3);

		SL4[0].m_CtrlPts.push_back(p17);
		SL4[0].m_CtrlPts.push_back(p18);
		SL4[0].m_CtrlPts.push_back(p19);

		SL4[1].m_CtrlPts.push_back(p19);
		SL4[1].m_CtrlPts.push_back(p20);
		SL4[1].m_CtrlPts.push_back(p21);

		SL4[2].m_CtrlPts.push_back(p23);
		SL4[2].m_CtrlPts.push_back(p22);
		SL4[2].m_CtrlPts.push_back(p21);

		SL4[3].m_CtrlPts.push_back(p17);
		SL4[3].m_CtrlPts.push_back(p24);
		SL4[3].m_CtrlPts.push_back(p23);

		//实例化曲面对象
		SplineSurface S4;
		//Coons插值成正方形面
		S4.CoonsInterpolate(SL4);

		//插值成右旁边的面
		SL5[0].m_CtrlPts.push_back(p21);
		SL5[0].m_CtrlPts.push_back(p25);
		SL5[0].m_CtrlPts.push_back(p26);

		SL5[1].m_CtrlPts.push_back(p26);
		SL5[1].m_CtrlPts.push_back(p27);
		SL5[1].m_CtrlPts.push_back(p28);

		SL5[2].m_CtrlPts.push_back(p23);
		SL5[2].m_CtrlPts.push_back(p29);
		SL5[2].m_CtrlPts.push_back(p28);

		SL5[3].m_CtrlPts.push_back(p21);
		SL5[3].m_CtrlPts.push_back(p22);
		SL5[3].m_CtrlPts.push_back(p23);

		//实例化曲面对象
		SplineSurface S5;
		//Coons插值成右边面
		S5.CoonsInterpolate(SL5);

		//插值成上边的面
		SL6[0].m_CtrlPts.push_back(p19);
		SL6[0].m_CtrlPts.push_back(p30);
		SL6[0].m_CtrlPts.push_back(p31);

		SL6[1].m_CtrlPts.push_back(p31);
		SL6[1].m_CtrlPts.push_back(p32);
		SL6[1].m_CtrlPts.push_back(p26);

		SL6[2].m_CtrlPts.push_back(p21);
		SL6[2].m_CtrlPts.push_back(p25);
		SL6[2].m_CtrlPts.push_back(p26);

		SL6[3].m_CtrlPts.push_back(p19);
		SL6[3].m_CtrlPts.push_back(p20);
		SL6[3].m_CtrlPts.push_back(p21);

		//实例化曲面对象
		SplineSurface S6;
		//Coons插值成上边的面
		S6.CoonsInterpolate(SL6);

		Model_Solution M;

		varray<SplineSurface> NSS;
		varray<SplineSurface> NSS1;
		NSS.push_back(S1);
		NSS.push_back(S2);
		NSS.push_back(S3);

		NSS1.push_back(S4);
		NSS1.push_back(S5);
		NSS1.push_back(S6);
		M.Trans(NSS1, 2 * r - 0.002, 2);
		
		for (int i = 0; i < 3; i++) {
			NSS.push_back(NSS1[i]);
		}
		
		//多面拉伸成体
		varray<SplineVolume> SV;
		SV = M.CreatSweepVol(NSS, h, 3);


		RWGeometric rwg;
		

		//节点细化
		for (int i = 0; i < SV.size(); i++) {
			SV[i].KnotsRefineNum(num);
		}

		//rwg.WriteSplineVolume("E:\\kuang_models\\SplineVolumekai2.txt", SV);

		/*
		//控制点排序
		CPolyParaVolume cp;
		cp = S;
		cp.SetAdjacentHexIdx();
		cp.Order();
		*/

		//test0();
		
		/*
		//嵌套类调用
		//第一种方式
		TestBolcks::pList p; 
		//第二种方式
		//输出的文件适用于分析
		TestBolcks::pList().OutputParaVolumeDataTxt(SV, "E:\\kuang_models\\SplineVolumePara2.txt");
		*/
	}

	/*
		三维圆筒
		r: 内圆半径 
		R: 外圆半径 
		h: 高度
	*/
	void cylinder(double r,double R,double h) {
		//创建容器，存放线
		//存放四片图形的四条边
		varray<Spline> SL1;
		varray<Spline> SL2;
		varray<Spline> SL3;
		varray<Spline> SL4;
		//创建容器，存放曲面,用于多面拉伸
		varray<SplineSurface> SS;
		//存放节点 p=2
		varray<double> knots;
		knots.push_back(0);
		knots.push_back(0);
		knots.push_back(0);
		knots.push_back(1);
		knots.push_back(1);
		knots.push_back(1);
		//指定大小，可存放四条线，用于Coons插值成面
		SL1.resize(4);
		SL2.resize(4);
		SL3.resize(4);
		SL4.resize(4);
		//对每条边进行曲线的次数和节点矢量初始化
		for (int i = 0; i < 4; i++)
		{
			//给定次数、节点矢量
			SL1[i].m_Degree = 2;
			SL1[i].m_Knots = knots;

			SL2[i].m_Degree = 2;
			SL2[i].m_Knots = knots;

			SL3[i].m_Degree = 2;
			SL3[i].m_Knots = knots;

			SL4[i].m_Degree = 2;
			SL4[i].m_Knots = knots;
		}
		//给出控制点
		//右边的扇形面坐标点
		//v=0
		Vec4 v1 = { sqrt(2)*r / 2,-sqrt(2)*r / 2,0,1 };
		Vec4 v2 = { sqrt(2)*(r + R) / 4,-sqrt(2)*(r + R) / 4,0,1 };
		Vec4 v3 = { sqrt(2)*R / 2,-sqrt(2)*R / 2,0,1 };
		//u=0
		Vec4 v4 = { sqrt(2)*r,0,0,0.707 };
		Vec4 v5 = { sqrt(2)*r / 2,sqrt(2)*r / 2,0,1 };
		//v=1
		Vec4 v6 = { sqrt(2)*(r + R) / 4,sqrt(2)*(r + R) / 4,0,1 };
		Vec4 v7 = { sqrt(2)*R / 2,sqrt(2)*R / 2,0,1 };

		Vec4 v8 = { sqrt(2)*R,0,0,0.707 };
		//上边的
		Vec4 u1 = { -sqrt(2)*r / 2,sqrt(2)*r / 2,0,1 };
		Vec4 u2 = {0,sqrt(2)*r,0,0.707 };
		
		Vec4 u3 = { -sqrt(2)*(r + R) / 4,sqrt(2)*(r + R) / 4,0,1 };
		Vec4 u4 = { -sqrt(2)*R / 2,sqrt(2)*R / 2,0,1 };

		Vec4 u5 = {0,sqrt(2)*R,0,0.707 };

		//左边的
		Vec4 w1 = { -sqrt(2)*R / 2,-sqrt(2)*R / 2,0,1 };
		Vec4 w3 = { -sqrt(2)*r / 2,-sqrt(2)*r / 2,0,1 };
		Vec4 w2 = { -sqrt(2)*(r + R) / 4,-sqrt(2)*(r + R) / 4,0,1 };

		Vec4 w4 = { -sqrt(2)*R,0,0,0.707 };
		Vec4 w5 = { -sqrt(2)*r,0,0,0.707 };

		//下边的
		Vec4 p1 = { 0,-sqrt(2)*R,0,0.707 };
		Vec4 p2 = { 0,-sqrt(2)*r,0,0.707 };


		SL1[0].m_CtrlPts.push_back(v1);
		SL1[0].m_CtrlPts.push_back(v2);
		SL1[0].m_CtrlPts.push_back(v3);

		SL1[1].m_CtrlPts.push_back(v1);
		SL1[1].m_CtrlPts.push_back(v4);
		SL1[1].m_CtrlPts.push_back(v5);

		SL1[2].m_CtrlPts.push_back(v5);
		SL1[2].m_CtrlPts.push_back(v6);
		SL1[2].m_CtrlPts.push_back(v7);

		SL1[3].m_CtrlPts.push_back(v3);
		SL1[3].m_CtrlPts.push_back(v8);
		SL1[3].m_CtrlPts.push_back(v7);
		//实例化曲面对象
		SplineSurface S1;
		//Coons插值成正方形面
		S1.CoonsInterpolate(SL1);


		//上边的
		SL2[0].m_CtrlPts.push_back(u1);
		SL2[0].m_CtrlPts.push_back(u2);
		SL2[0].m_CtrlPts.push_back(v5);

		SL2[1].m_CtrlPts.push_back(u1);
		SL2[1].m_CtrlPts.push_back(u3);
		SL2[1].m_CtrlPts.push_back(u4);

		SL2[2].m_CtrlPts.push_back(u4);
		SL2[2].m_CtrlPts.push_back(u5);
		SL2[2].m_CtrlPts.push_back(v7);

		SL2[3].m_CtrlPts.push_back(v5);
		SL2[3].m_CtrlPts.push_back(v6);
		SL2[3].m_CtrlPts.push_back(v7);

		//实例化曲面对象
		SplineSurface S2;
		//Coons插值成右边面
		S2.CoonsInterpolate(SL2);

		//左边
		SL3[0].m_CtrlPts.push_back(w1);
		SL3[0].m_CtrlPts.push_back(w2);
		SL3[0].m_CtrlPts.push_back(w3);

		SL3[1].m_CtrlPts.push_back(w1);
		SL3[1].m_CtrlPts.push_back(w4);
		SL3[1].m_CtrlPts.push_back(u4);

		SL3[2].m_CtrlPts.push_back(u4);
		SL3[2].m_CtrlPts.push_back(u3);
		SL3[2].m_CtrlPts.push_back(u1);

		SL3[3].m_CtrlPts.push_back(w3);
		SL3[3].m_CtrlPts.push_back(w5);
		SL3[3].m_CtrlPts.push_back(u1);

		//实例化曲面对象
		SplineSurface S3;
		//Coons插值成上边的面
		S3.CoonsInterpolate(SL3);

		//下边
		SL4[0].m_CtrlPts.push_back(w1);
		SL4[0].m_CtrlPts.push_back(p1);
		SL4[0].m_CtrlPts.push_back(v3);

		SL4[1].m_CtrlPts.push_back(w1);
		SL4[1].m_CtrlPts.push_back(w2);
		SL4[1].m_CtrlPts.push_back(w3);

		SL4[2].m_CtrlPts.push_back(w3);
		SL4[2].m_CtrlPts.push_back(p2);
		SL4[2].m_CtrlPts.push_back(v1);

		SL4[3].m_CtrlPts.push_back(v3);
		SL4[3].m_CtrlPts.push_back(v2);
		SL4[3].m_CtrlPts.push_back(v1);

		//实例化曲面对象
		SplineSurface S4;
		//Coons插值成上边的面
		S4.CoonsInterpolate(SL4);

		varray<SplineSurface> NSS;
		NSS.push_back(S1);
		NSS.push_back(S2);
		NSS.push_back(S3);
		NSS.push_back(S4);

		
		/*RWGeometric rwg;
		rwg.WriteSplineSurface("E:\\kuang_models\\SplineSurface1.txt", NSS);*/

		//公用函数类实例化
		Model_Solution M;
		//多面拉伸成体
		varray<SplineVolume> SV;
		SV = M.CreatSweepVol(NSS, h, 3);

		RWGeometric rwg;
		rwg.WriteSplineVolume("E:\\kuang_models\\SplineVolumecyclinder.txt", SV);

		//嵌套类调用
		//第一种方式
		//TestBolcks::pList p; 
		//第二种方式
		//输出的文件适用于分析
		/*TestBolcks::pList().OutputParaVolumeDataTxt(SV, "E:\\kuang_models\\SplineVolumePara.txt");*/

	}
	/*
		四分之一圆筒
		r: 内圆半径 
		R: 外圆半径
	*/
	void cylinder1(double r,double R) {
		//创建容器，存放线
		//存放四条边
		varray<Spline> SL1;

		//存放节点 次数p=21
		varray<double> knots;
		knots.push_back(0);
		knots.push_back(0);
		knots.push_back(0);
		knots.push_back(1);
		knots.push_back(1);
		knots.push_back(1);
		//指定大小，可存放四条线，用于Coons插值成面
		SL1.resize(4);
		for (int i = 0; i < 4; i++)
		{
			//给定次数、节点矢量
			SL1[i].m_Degree = 2;
			SL1[i].m_Knots = knots;
		}
		//给出控制点
		//v=0
		Vec4 v1 = { sqrt(2)*r/2,-sqrt(2)*r / 2,0,1 };
		Vec4 v2 = { sqrt(2)*(r+R) / 4,-sqrt(2)*(r + R) / 4,0,1 };
		Vec4 v3 = { sqrt(2)*R / 2,-sqrt(2)*R / 2,0,1 };
		//u=0
		Vec4 v4 = { sqrt(2)*r,0,0,0.707 };
		Vec4 v5 = { sqrt(2)*r / 2,sqrt(2)*r / 2,0,1 };
		//v=1
		Vec4 v6 = { sqrt(2)*(r + R) / 4,sqrt(2)*(r + R) / 4,0,1 };
		Vec4 v7 = { sqrt(2)*R / 2,sqrt(2)*R / 2,0,1 };

		Vec4 v8 = { sqrt(2)*R,0,0,0.707 };

		SL1[0].m_CtrlPts.push_back(v1);
		SL1[0].m_CtrlPts.push_back(v2);
		SL1[0].m_CtrlPts.push_back(v3);

		SL1[1].m_CtrlPts.push_back(v1);
		SL1[1].m_CtrlPts.push_back(v4);
		SL1[1].m_CtrlPts.push_back(v5);

		SL1[2].m_CtrlPts.push_back(v5);
		SL1[2].m_CtrlPts.push_back(v6);
		SL1[2].m_CtrlPts.push_back(v7);

		SL1[3].m_CtrlPts.push_back(v3);
		SL1[3].m_CtrlPts.push_back(v8);
		SL1[3].m_CtrlPts.push_back(v7);
		//实例化曲面对象
		SplineSurface S1;
		//Coons插值成正方形面
		S1.CoonsInterpolate(SL1);

		
		////放到曲面容器中
		//varray<SplineSurface> SS;
		//SS.push_back(S1);

		////输出面
		//RWGeometric rwg;
		//rwg.WriteSplineSurface("E:\\kuang_models\\SplineSurfacecylinder1.txt", SS);

		//单面拉伸成体
		Creat_Vol cv;
		//这一部分修改了扫描路径函数里面的一些东西，以达到可以产生四个控制点（默认写的路径上只有三个控制点）
		cv.InitVol(S1, 4, 3);

		//放到体容器中
		varray<SplineVolume> SV;
		SV.push_back(cv.NV);
		//输出体文件
		RWGeometric rwg;
		rwg.WriteSplineVolume("E:\\kuang_models\\SplineVolumecylinder1.txt", SV);
	}

	/*
		正方体---卜
		L：底面正方形边长
	*/
	void kuang_bu_vol(double L) {
		//创建容器，存放线
		//存放正方形的四条边
		varray<Spline> SL1;

		//存放节点 次数p=1
		//一条边上只有两个控制点，只能用一次曲线 ——————要注意！
		varray<double> knots;
		knots.push_back(0);
		knots.push_back(0);

		knots.push_back(1);
		knots.push_back(1);

		//指定大小，可存放四条线，用于Coons插值成面
		SL1.resize(4);
		for (int i = 0; i < 4; i++)
		{
			//给定次数、节点矢量
			SL1[i].m_Degree = 1;
			SL1[i].m_Knots = knots;
		}
		//给出控制点
		//用于正方形的
		//v=0
		Vec4 v1 = { 0,0,0,1 };
		Vec4 v2 = { L,0,0,1 };
		//u=0
		Vec4 v3 = { 0,L,0,1 };
		//v=1
		Vec4 v4 = { L,L,0,1 };

		SL1[0].m_CtrlPts.push_back(v1);
		SL1[0].m_CtrlPts.push_back(v2);

		SL1[1].m_CtrlPts.push_back(v1);
		SL1[1].m_CtrlPts.push_back(v3);

		SL1[2].m_CtrlPts.push_back(v3);
		SL1[2].m_CtrlPts.push_back(v4);

		SL1[3].m_CtrlPts.push_back(v2);
		SL1[3].m_CtrlPts.push_back(v4);

		//实例化曲面对象
		SplineSurface S1;
		//Coons插值成正方形面
		S1.CoonsInterpolate(SL1);

		/*
		//放到曲面容器中
		varray<SplineSurface> SS;
		SS.push_back(S1);

		//输出面
		RWGeometric rwg;
		rwg.WriteSplineSurface("E:\\kuang_models\\SplineSurfaceKB.txt", SS);*/

		//单面拉伸成体
		Creat_Vol cv;
		//这一部分修改了扫描路径函数里面的一些东西，以达到可以产生四个或两个控制点（默认写的路径上只有三个控制点）
		cv.InitVol(S1, L/3, 3);

		//放到体容器中
		varray<SplineVolume> SV;
		SV.push_back(cv.NV);

		for (int i = 0; i < cv.NV.m_CtrlPts.size(); i++)
		{
			cv.NV.m_CtrlPts[i].z += L/3;
		}
		SV.push_back(cv.NV);

		for (int i = 0; i < cv.NV.m_CtrlPts.size(); i++)
		{
			cv.NV.m_CtrlPts[i].z += L/ 3;
		}
		SV.push_back(cv.NV);

		//输出体文件
		RWGeometric rwg;
		rwg.WriteSplineVolume("E:\\kuang_models\\SplineVolumek-bu4.txt", SV);
	}
	/*
		给卜的模型——线2
	*/
	void kuang_bu_lines()
	{
		varray<Spline> SLS;

		varray<double> knots;
		knots.push_back(0);
		knots.push_back(0);
		knots.push_back(0);
		knots.push_back(1);
		knots.push_back(1);
		knots.push_back(1);
		
		Spline SL1;
		//给定次数、节点矢量
		SL1.m_Degree = 2;
		SL1.m_Knots = knots;

		Spline SL2;
		//给定次数、节点矢量
		SL2.m_Degree = 2;
		SL2.m_Knots = knots;

		Spline SL3;
		//给定次数、节点矢量
		SL3.m_Degree = 2;
		SL3.m_Knots = knots;

		Spline SL4;
		//给定次数、节点矢量
		SL4.m_Degree = 2;
		SL4.m_Knots = knots;

		Spline SL5;
		//给定次数、节点矢量
		SL5.m_Degree = 2;
		SL5.m_Knots = knots;

		Spline SL6;
		//给定次数、节点矢量
		SL6.m_Degree = 2;
		SL6.m_Knots = knots;
		
		////以下是a图形的三条线的坐标
		//Vec4 p1 = { 0,0.8,0.2,1 };
	 //   Vec4 p2 = { -0.5,1.3,-0.9,1 };
		//Vec4 p3 = { -2,3,-2,1 };

		//Vec4 p4 = { 0,0.7,0.85,1 };
		//Vec4 p5 = { -0.5,1,1.2,1 };
		//Vec4 p6 = { -2,1.3,2,1 };

		//Vec4 p7 = { 0,0.25,0.86,1 };
		//Vec4 p8 = { -0.5,-0.4,1.2,1 };
		//Vec4 p9 = { -2,-1.5,1.5,1 };

		//以下是b图形的三条线的坐标
		Vec4 p1 = { 0.5,0.6,0,1 };
		Vec4 p2 = { 0.5,1,-0.9,1 };
		Vec4 p3 = { 0.5,2,-2,1 };

		Vec4 p4 = { 0.5,1,0.5,1 };
		Vec4 p5 = { 0.5,1.5,0.2,1 };
		Vec4 p6 = { 0.5,3,0.5,1 };

		Vec4 p7 = { 0,0.9,0.7,1 };
		Vec4 p8 = { -0.5,1.5,1,1 };
		Vec4 p9 = { -2,3,1.4,1 };

		////以下是六条线的坐标
		//Vec4 p1 = { 0.5,1,0.2,1 };
		//Vec4 p2 = { 0.5,2,-0.9,1 };
		//Vec4 p3 = { 0.5,3,-1.2,1 };

		//Vec4 p4 = { 0.85,1,0.5,1 };
		//Vec4 p5 = { 1.8,2,0.5,1 };
		//Vec4 p6 = { 2,3,0.5,1 };

		//Vec4 p7 = { 0.5,1,0.8,1 };
		//Vec4 p8 = { 0.5,2,1.7,1 };
		//Vec4 p9 = { 0.5,3,2,1 };

		//Vec4 v1 = { 0.1,0.8,0,1 };
		//Vec4 v2 = { -0.3,0.8,-0.8,1 };
		//Vec4 v3 = {-1.3,0.8,-1.6,1 };

		//Vec4 v4 = { 0,0.8,0.5,1 };
		//Vec4 v5 = { -0.6,1.8,0.5,1 };
		//Vec4 v6 = { -1.3,2,0.5,1 };
		//
		//Vec4 v7 = { 0.1,0.8,1,1 };
		//Vec4 v8 = { -0.2,0.8,1.8,1 };
		//Vec4 v9 = { -1.3,0.8,2.6,1 };

		SL1.m_CtrlPts.push_back(p1);
		SL1.m_CtrlPts.push_back(p2);
		SL1.m_CtrlPts.push_back(p3);

		SL2.m_CtrlPts.push_back(p4);
		SL2.m_CtrlPts.push_back(p5);
		SL2.m_CtrlPts.push_back(p6);

		SL3.m_CtrlPts.push_back(p7);
		SL3.m_CtrlPts.push_back(p8);
		SL3.m_CtrlPts.push_back(p9);

		/*SL4.m_CtrlPts.push_back(v1);
		SL4.m_CtrlPts.push_back(v2);
		SL4.m_CtrlPts.push_back(v3);

		SL5.m_CtrlPts.push_back(v4);
		SL5.m_CtrlPts.push_back(v5);
		SL5.m_CtrlPts.push_back(v6);

		SL6.m_CtrlPts.push_back(v7);
		SL6.m_CtrlPts.push_back(v8);
		SL6.m_CtrlPts.push_back(v9);*/

		SLS.push_back(SL1);
		SLS.push_back(SL2);
		SLS.push_back(SL3);
		/*SLS.push_back(SL4);
		SLS.push_back(SL5);
		SLS.push_back(SL6);*/

		RWGeometric rwg;
		rwg.WriteSpline("E:\\kuang_models\\nurbslinebu1.txt", SLS);
	}

	/*
		01.给冯的模型---边界
	*/
	void kuang_lines1()
	{
		varray<Spline> SL;
	    varray<double> knots;
		knots.push_back(0);
		knots.push_back(0);
		knots.push_back(0);
		knots.push_back(1);
		knots.push_back(1);
		knots.push_back(1);
		//圆弧半径
		double r=1;

		//底边边界
		Spline SL1;
		SL1.m_Degree = 2;
		SL1.m_Knots = knots;
		
		Vec4 p1 = { 0,r,0,1 };
		Vec4 p2 = { r,r,0,0.707 };
		Vec4 p3 = { r,0,0,1 };

		SL1.m_CtrlPts.push_back(p1);
		SL1.m_CtrlPts.push_back(p2);
		SL1.m_CtrlPts.push_back(p3);
		SL.push_back(SL1);

		Spline SL2;
		SL2.m_Degree = 2;
		SL2.m_Knots = knots;
		Vec4 p4 = { 3*r,0,0,1 };
		Vec4 p5 = { 5*r,0,0,1 };

		SL2.m_CtrlPts.push_back(p3);
		SL2.m_CtrlPts.push_back(p4);
		SL2.m_CtrlPts.push_back(p5);
		SL.push_back(SL2);

		Spline SL3;
		SL3.m_Degree = 2;
		SL3.m_Knots = knots;
		Vec4 p6 = { 5*r,r,0,0.707 };
		Vec4 p7 = { 6 * r,r,0,1 };

		SL3.m_CtrlPts.push_back(p5);
		SL3.m_CtrlPts.push_back(p6);
		SL3.m_CtrlPts.push_back(p7);
		SL.push_back(SL3);
		//向右移动四次
		for (int i = 0; i < 4; i++) {
			for (int j = 0; j < SL1.m_CtrlPts.size(); j++) {
				SL1.m_CtrlPts[j].x += 6 * r;
				SL2.m_CtrlPts[j].x += 6 * r;
				SL3.m_CtrlPts[j].x += 6 * r;
			}
			SL.push_back(SL1);
			SL.push_back(SL2);
			SL.push_back(SL3);
		}

		//左边边界
		Spline SL4;
		SL4.m_Degree = 2;
		SL4.m_Knots = knots;
		Vec4 v1 = { 0,3 * r,0,1 };
		Vec4 v2 = { 0,5 * r,0,1 };

		SL4.m_CtrlPts.push_back(p1);
		SL4.m_CtrlPts.push_back(v1);
		SL4.m_CtrlPts.push_back(v2);
		SL.push_back(SL4);

		Spline SL5;
		SL5.m_Degree = 2;
		SL5.m_Knots = knots;
		Vec4 v3 = { r,5 * r,0,0.707 };
		Vec4 v4 = { r,6 * r,0,1 };

		SL5.m_CtrlPts.push_back(v2);
		SL5.m_CtrlPts.push_back(v3);
		SL5.m_CtrlPts.push_back(v4);
		SL.push_back(SL5);
		//向上移动四次
		//SL1之前移动了，要先变回来
		for (int i = 0; i < 3; i++)
		{
			SL1.m_CtrlPts[i].x -= 24 * r;
		}
		
		for (int i = 0; i < 4; i++) {
			for (int j = 0; j < 3; j++) {
				
				SL1.m_CtrlPts[j].y += 6 * r;
				SL4.m_CtrlPts[j].y += 6 * r;
				SL5.m_CtrlPts[j].y += 6 * r;
			}
			SL.push_back(SL1);
			SL.push_back(SL4);
			SL.push_back(SL5);
		}
		//右边边界
		//SL4向下移动24*r
		for (int i = 0; i < 3; i++)
		{
			SL4.m_CtrlPts[i].y -= 24 * r;
		}
		//SL4右移动30*r
		for (int i = 0; i < 3; i++)
		{
			SL4.m_CtrlPts[i].x += 30 * r;
		}
		SL.push_back(SL4);

		Spline SL6;
		SL6.m_Degree = 2;
		SL6.m_Knots = knots;

		Vec4 w1 = { 29*r,5 * r,0,0.707 };
		Vec4 w2 = { 29*r,6 * r,0,1 };

		SL6.m_CtrlPts.push_back(SL4.m_CtrlPts[2]);
		SL6.m_CtrlPts.push_back(w1);
		SL6.m_CtrlPts.push_back(w2);
		SL.push_back(SL6);

		for (int i = 0; i < 4; i++) {
			for (int j = 0; j < 3; j++) {

				SL3.m_CtrlPts[j].y += 6 * r;
				SL4.m_CtrlPts[j].y += 6 * r;
				SL6.m_CtrlPts[j].y += 6 * r;
			}
			SL.push_back(SL3);
			SL.push_back(SL4);
			SL.push_back(SL6);
		}

		//上边边界
		//SL2向上移动30 * r
		for (int i = 0; i < 3; i++)
		{
			SL2.m_CtrlPts[i].y += 30 * r;
		}
		//左移24 * r
		for (int i = 0; i < 3; i++)
		{
			SL2.m_CtrlPts[i].x -= 24 * r;
		}
		SL.push_back(SL2);
		Spline SL7;
		SL7.m_Degree = 2;
		SL7.m_Knots = knots;

		Vec4 u1 = { 5 * r,29 * r,0,0.707 };
		Vec4 u2 = { 6 * r,29 * r,0,1 };

		SL7.m_CtrlPts.push_back(SL2.m_CtrlPts[2]);
		SL7.m_CtrlPts.push_back(u1);
		SL7.m_CtrlPts.push_back(u2);
		SL.push_back(SL7);

		for (int i = 0; i < 4; i++) {
			for (int j = 0; j < 3; j++) {

				SL5.m_CtrlPts[j].x += 6 * r;
				SL2.m_CtrlPts[j].x += 6 * r;
				SL7.m_CtrlPts[j].x += 6 * r;
			}
			SL.push_back(SL5);
			SL.push_back(SL2);
			SL.push_back(SL7);
		}

		RWGeometric rwg;
		rwg.WriteSpline("E:\\kuang_models\\nurbslinefeng1.txt", SL);
	}
	/*
		02.给冯的模型---内圆
	*/
	void kuang_lines2() {
		varray<Spline> SL;
		varray<double> knots;
		knots.push_back(0);
		knots.push_back(0);
		knots.push_back(0);
		knots.push_back(1);
		knots.push_back(1);
		knots.push_back(1);
		Spline SL1;
		SL1.m_Degree = 2;
		SL1.m_Knots = knots;
		//圆弧半径
		double r = 1;
		Vec4 p1 = { 0,r,0,1 };
		Vec4 p2 = { r,r,0,0.707 };
		Vec4 p3 = { r,0,0,1 };

		SL1.m_CtrlPts.push_back(p1);
		SL1.m_CtrlPts.push_back(p2);
		SL1.m_CtrlPts.push_back(p3);

		Spline SL3;
		SL3.m_Degree = 2;
		SL3.m_Knots = knots;
		Vec4 p5 = { 5 * r,0,0,1 };
		Vec4 p6 = { 5 * r,r,0,0.707 };
		Vec4 p7 = { 6 * r,r,0,1 };

		SL3.m_CtrlPts.push_back(p5);
		SL3.m_CtrlPts.push_back(p6);
		SL3.m_CtrlPts.push_back(p7);

		Spline SL5;
		SL5.m_Degree = 2;
		SL5.m_Knots = knots;
		Vec4 v2 = { 0,5 * r,0,1 };
		Vec4 v3 = { r,5 * r,0,0.707 };
		Vec4 v4 = { r,6 * r,0,1 };

		SL5.m_CtrlPts.push_back(v2);
		SL5.m_CtrlPts.push_back(v3);
		SL5.m_CtrlPts.push_back(v4);

		Spline SL7;
		SL7.m_Degree = 2;
		SL7.m_Knots = knots;
		Vec4 u0 = { 5 * r,30 * r,0,1 };
		Vec4 u1 = { 5 * r,29 * r,0,0.707 };
		Vec4 u2 = { 6 * r,29 * r,0,1 };

		SL7.m_CtrlPts.push_back(u0);
		SL7.m_CtrlPts.push_back(u1);
		SL7.m_CtrlPts.push_back(u2);

		
		for (int i = 0; i < 3; i++)
		{
			SL7.m_CtrlPts[i].y -= 24* r;
			SL5.m_CtrlPts[i].x += 6 * r;
			SL3.m_CtrlPts[i].y += 6 * r;
			SL1.m_CtrlPts[i].y += 6 * r;
			SL1.m_CtrlPts[i].x += 6 * r;
		}
		double a = 18;
		for (int i = 0; i < 3; i++)
		{
			SL7.m_CtrlPts[i].y += a * r;
			SL5.m_CtrlPts[i].y += a * r;
			SL3.m_CtrlPts[i].y += a * r;
			SL1.m_CtrlPts[i].y += a * r;
		}
		double b = 18;
		for (int i = 0; i < 3; i++)
		{
			SL7.m_CtrlPts[i].x += b * r;
			SL5.m_CtrlPts[i].x += b * r;
			SL3.m_CtrlPts[i].x += b * r;
			SL1.m_CtrlPts[i].x += b * r;
		}
		SL.push_back(SL7);
		SL.push_back(SL5);
		SL.push_back(SL3);
		SL.push_back(SL1);

		RWGeometric rwg;
		rwg.WriteSpline("E:\\kuang_models\\nurbslinefeng2-16.txt", SL);
	}
	//03.给冯的模型---内圆+连线
	void kuang_lines3() {
		varray<Spline> SL;
		varray<double> knots;
		knots.push_back(0);
		knots.push_back(0);
		knots.push_back(0);
		knots.push_back(1);
		knots.push_back(1);
		knots.push_back(1);
		Spline SL1;
		SL1.m_Degree = 2;
		SL1.m_Knots = knots;
		//圆弧半径
		double r = 1;
		Vec4 p1 = { (6 - sqrt(2) / 2)*r,(6 - sqrt(2) / 2)*r,0,1 };
		Vec4 p2 = { 6 * r,(6 - sqrt(2))*r,0,0.707 };
		Vec4 p3 = { (6 + sqrt(2) / 2)*r,(6 - sqrt(2) / 2)*r,0,1 };

		SL1.m_CtrlPts.push_back(p1);
		SL1.m_CtrlPts.push_back(p2);
		SL1.m_CtrlPts.push_back(p3);
		/*SL.push_back(SL1);*/

		Spline SL2;
		SL2.m_Degree = 2;
		SL2.m_Knots = knots;

		Vec4 p4 = { (6 +sqrt(2))* r,6*r,0,0.707 };
		Vec4 p5 = { (6 + sqrt(2) / 2)*r,(6 + sqrt(2) / 2)*r,0,1 };

		SL2.m_CtrlPts.push_back(p3);
		SL2.m_CtrlPts.push_back(p4);
		SL2.m_CtrlPts.push_back(p5);
		/*SL.push_back(SL2);*/

		Spline SL3;
		SL3.m_Degree = 2;
		SL3.m_Knots = knots;

		Vec4 p6 = { 6* r,(6 + sqrt(2))* r,0,0.707 };
		Vec4 p7 = { (6 - sqrt(2) / 2)*r,(6 + sqrt(2) / 2)*r,0,1 };

		SL3.m_CtrlPts.push_back(p5);
		SL3.m_CtrlPts.push_back(p6);
		SL3.m_CtrlPts.push_back(p7);
		/*SL.push_back(SL3);*/

		Spline SL4;
		SL4.m_Degree = 2;
		SL4.m_Knots = knots;

		Vec4 p8 = { (6 - sqrt(2))*r,6* r,0,0.707 };

		SL4.m_CtrlPts.push_back(p7);
		SL4.m_CtrlPts.push_back(p8);
		SL4.m_CtrlPts.push_back(p1);
		/*SL.push_back(SL4);*/

		Spline SL5;
		SL5.m_Degree = 2;
		SL5.m_Knots = knots;
		
		Vec4 u1 = { 9*r,(6 - sqrt(2) / 2)*r,0,1 };
		Vec4 u2 = { (12 - sqrt(2) / 2)*r,(6 - sqrt(2) / 2)*r,0,1 };
		

		SL5.m_CtrlPts.push_back(p3);
		SL5.m_CtrlPts.push_back(u1);
		SL5.m_CtrlPts.push_back(u2);
		SL.push_back(SL5);

		//上移动
		double a = 6;
		for (int i = 0; i < 3; i++) {
			if (i == 2) {
				for (int j = 0; j < 3; j++)
				{
					SL5.m_CtrlPts[j].y += (a + sqrt(2)) * r;
				}
				SL.push_back(SL5);
			}
			else {
				for (int j = 0; j < 3; j++)
				{
					SL5.m_CtrlPts[j].y += a * r;
				}
				SL.push_back(SL5);
			}
		}
		
		//右移动
		for (int i = 0; i < 3; i++)
		{
			SL5.m_CtrlPts[i].x += a * r;
			SL.push_back(SL5);
		}
		//下移
		for (int i = 0; i < 3; i++)
		{
			if (i == 0) {
				for (int j = 0; j < 3; j++)
				{
					SL5.m_CtrlPts[j].y -= (a + sqrt(2)) * r;
				}
				SL.push_back(SL5);
			}
			else {
				for (int j = 0; j < 3; j++)
				{
					SL5.m_CtrlPts[j].y -= a * r;
				}
				SL.push_back(SL5);
			}
		}
		//右移动
		for (int i = 0; i < 3; i++)
		{
			SL5.m_CtrlPts[i].x += a * r;
			SL.push_back(SL5);
		}
		//上移动
		for (int i = 0; i < 3; i++) {
			if (i == 2) {
				for (int j = 0; j < 3; j++)
				{
					SL5.m_CtrlPts[j].y += (a + sqrt(2)) * r;
				}
				SL.push_back(SL5);
			}
			else {
				for (int j = 0; j < 3; j++)
				{
					SL5.m_CtrlPts[j].y += a * r;
				}
				SL.push_back(SL5);
			}
		}

		Spline SL6;
		SL6.m_Degree = 2;
		SL6.m_Knots = knots;

		Vec4 v1 = { (6 - sqrt(2) / 2)*r,9 * r,0,1 };
		Vec4 v2 = { (6 - sqrt(2) / 2)*r,(12 - sqrt(2) / 2)*r,0,1 };
		

		SL6.m_CtrlPts.push_back(p7);
		SL6.m_CtrlPts.push_back(v1);
		SL6.m_CtrlPts.push_back(v2);
		SL.push_back(SL6);
		//上移动
		for (int i = 0; i < 2; i++) {
			for (int j = 0; j < 3; j++)
			{
				SL6.m_CtrlPts[j].y += a * r;
			}
			SL.push_back(SL6);
		}
		//右移动
		for (int j = 0; j < 3; j++)
		{
			SL6.m_CtrlPts[j].x += (18+sqrt(2)) * r;
		}
		SL.push_back(SL6);
		//下移动
		for (int i = 0; i < 2; i++) {
			for (int j = 0; j < 3; j++)
			{
				SL6.m_CtrlPts[j].y -= a * r;
			}
			SL.push_back(SL6);
		}
		

		RWGeometric rwg;
		rwg.WriteSpline("E:\\kuang_models\\nurbslinefeng3.txt", SL);
	}

	//04.给冯的模型---线->矩形
	void kuang_line4()
	{
		varray<Spline> SL;

		varray<double> knots;
		knots.push_back(0);
		knots.push_back(0);
		knots.push_back(0);
		knots.push_back(1);
		knots.push_back(1);
		knots.push_back(1);

		//第一条
		Spline SL1;
		SL1.m_Degree = 2;
		SL1.m_Knots = knots;

		Vec4 p1 = { 4.93934,0,0,1 };
		Vec4 p2 = { 6.46967,0,0,1 };
		Vec4 p3 = { 8,0,0,1 };

		SL1.m_CtrlPts.push_back(p1);
		SL1.m_CtrlPts.push_back(p2);
		SL1.m_CtrlPts.push_back(p3);
		SL.push_back(SL1);

		Spline SL2;
		SL2.m_Degree = 2;
		SL2.m_Knots = knots;
		Vec4 p4 = { 0,0,0,1 };
		Vec4 p5 = { 1.375,0,0,1 };
		Vec4 p6 = { 2.75,0,0,1 };

		SL2.m_CtrlPts.push_back(p4);
		SL2.m_CtrlPts.push_back(p5);
		SL2.m_CtrlPts.push_back(p6);
		SL.push_back(SL2);

		Spline SL3;
		SL3.m_Degree = 2;
		SL3.m_Knots = knots;
		Vec4 p7 = { 3.84467,0,0,0 };
		

		SL3.m_CtrlPts.push_back(p6);
		SL3.m_CtrlPts.push_back(p7);
		SL3.m_CtrlPts.push_back(p1);
		SL.push_back(SL3);
		//三条线向上移动3.5	
		for (int j = 0; j < SL1.m_CtrlPts.size(); j++) {
			SL1.m_CtrlPts[j].y += 3.5;
			SL2.m_CtrlPts[j].y += 3.5;
			SL3.m_CtrlPts[j].y += 3.5;
		}
		//移动之后再放到线容器中，再显示一下
		SL.push_back(SL1);
		SL.push_back(SL2);
		SL.push_back(SL3);

		Spline SL4;
		SL4.m_Degree = 2;
		SL4.m_Knots = knots;

		Vec4 u2 = { 0,3.5/2,0,1 };
		Vec4 u3 = { 0,3.5,0,1 };
		
		SL4.m_CtrlPts.push_back(p4);
		SL4.m_CtrlPts.push_back(u2);
		SL4.m_CtrlPts.push_back(u3);
		SL.push_back(SL4);

		Spline SL5;
		SL5.m_Degree = 2;
		SL5.m_Knots = knots;
	
		Vec4 u5 = { 8,3.5 / 2,0,1 };
		Vec4 u6 = { 8,3.5,0,1 };

		SL5.m_CtrlPts.push_back(p3);
		SL5.m_CtrlPts.push_back(u5);
		SL5.m_CtrlPts.push_back(u6);
		SL.push_back(SL5);

		//写出文件
		RWGeometric rwg;
		rwg.WriteSpline("E:\\kuang_models\\nurbslinefeng0.txt", SL);
	}
	//05.给冯的模型---s1体 箱体
	void kuang_box() {
		//创建容器，存放线
		//存放十四个片四条边
		varray<Spline> SL1;
		varray<Spline> SL2;
		varray<Spline> SL3;
		varray<Spline> SL4;
		varray<Spline> SL5;
		varray<Spline> SL6;
		varray<Spline> SL7;
		varray<Spline> SL8;
		varray<Spline> SL9;
		varray<Spline> SL10;
		varray<Spline> SL11;
		varray<Spline> SL12;
		varray<Spline> SL13;
		varray<Spline> SL14;
		
		//创建容器，存放曲面,用于多面拉伸
		varray<SplineSurface> SS;
		//存放节点 p=2
		varray<double> knots;
		knots.push_back(0);
		knots.push_back(0);
		knots.push_back(0);
		knots.push_back(1);
		knots.push_back(1);
		knots.push_back(1);
		//指定大小，可存放四条线，用于Coons插值成面
		SL1.resize(4);
		SL2.resize(4);
		SL3.resize(4);
		SL4.resize(4);
		SL5.resize(4);
		SL6.resize(4);
		SL7.resize(4);
		SL8.resize(4);
		SL9.resize(4);
		SL10.resize(4);
		SL11.resize(4);
		SL12.resize(4);
		SL13.resize(4);
		SL14.resize(4);
		
		for (int i = 0; i < 4; i++)
		{
			//给定四条边次数、节点矢量
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

			SL10[i].m_Degree = 2;
			SL10[i].m_Knots = knots;

			SL11[i].m_Degree = 2;
			SL11[i].m_Knots = knots;

			SL12[i].m_Degree = 2;
			SL12[i].m_Knots = knots;

			SL13[i].m_Degree = 2;
			SL13[i].m_Knots = knots;

			SL14[i].m_Degree = 2;
			SL14[i].m_Knots = knots;
		}
		

		double w = sqrt(2) / 2;
		Vec4 p01 = { 0,0,0,1 };
		Vec4 p02 = { 0,2,0,1 };
		Vec4 p03 = { 0,4,0,1 };
		Vec4 p04 = { 0.625,3.375,0,1 };
		Vec4 p05 = { 1.25,2.75,0,1 };
		Vec4 p06 = { 1.25,2,0,1 };
		Vec4 p07 = { 1.25,1.25,0,1 };
		Vec4 p08 = { 0.625,0.625,0,1 };
		Vec4 p09 = { 1.375,0,0,1 };
		Vec4 p10 = { 2.75,0,0,1 };
		Vec4 p11 = { 2.75,0.625,0,1 };
		Vec4 p12 = { 2.75,1.25,0,1 };
		Vec4 p13 = { 2,1.25,0,1 };
		Vec4 p14 = { 2.75,2,0,1 };
		Vec4 p15 = { 2.75,2.75,0,1 };
		Vec4 p16 = { 2,2.75,0,1 };

		Vec4 p17 = { 2.75,3.375,0,1 };
		Vec4 p18 = { 2.75,4,0,1 };
		Vec4 p19 = { 1.375,4,0,1 };
		Vec4 p20 = { 3.84467,4,0,1 };
		Vec4 p21 = { 4.93934,4,0,1 };
		Vec4 p22 = { 4.93934,3.53033,0,1 };
		Vec4 p23 = { 4.93934,3.06066,0,1 };

		Vec4 p24 = { 3.84467,2.90533,0,1 };
		Vec4 p25 = { 3.84467,1.09467,0,1 };
		Vec4 p26 = { 4.93934,0.93934,0,1 };
		Vec4 p27 = { 4.93934,0.46967,0,1 };
		Vec4 p28 = { 4.93934,0,0,1 };
		Vec4 p29 = { 3.84467,0,0,1 };
		Vec4 p30 = { 6.46967,4,0,1 };

		Vec4 p31 = { 8,4,0,1 };
		Vec4 p32 = { 7.53033,3.53033,0,1 };
		Vec4 p33 = { 7.06066,3.06066,0,1 };
		Vec4 p34 = { 6,4.12132,0,w };
		Vec4 p35 = { 8,2,0,1 };
		Vec4 p36 = { 8,0,0,1 };
		Vec4 p37 = { 7.53033,0.46967,0,1 };

		Vec4 p38 = { 7.06066,0.93934,0,1 };
		Vec4 p39 = { 8.12132,2,0,w };
		Vec4 p40 = { 6.46967,0,0,1 };
		Vec4 p41 = { 6,-0.12132,0,w };
		Vec4 p42 = { 3.87867,2,0,w };
		Vec4 p43 = { 5.11612,2.88388,0,1 };
		Vec4 p44 = { 5.2929,2.7071,0,1 };

		Vec4 p45 = { 6,3.4142,0,w };
		Vec4 p46 = { 6.7071,2.7071,0,1 };
		Vec4 p47 = { 6.88388,2.88388,0,1 };
		Vec4 p48 = { 7.4142,2,0,w };
		Vec4 p49 = { 6.7071,1.2929,0,1 };
		Vec4 p50 = { 6.88388,1.11612,0,1 };
		Vec4 p51 = { 6,0.5858,0,w };
		Vec4 p52 = { 5.11612,1.11612,0,1 };
		Vec4 p53 = { 5.2929,1.2929,0,1 };
		Vec4 p54 = { 4.5858,2,0,w };
		
		
		//将点放到线容器中
		//用coons插值成正方形面，点的存放是有顺序的，顺时针，从左到右，从下到上
		SL1[0].m_CtrlPts.push_back(p01);
		SL1[0].m_CtrlPts.push_back(p02);
		SL1[0].m_CtrlPts.push_back(p03);

		SL1[1].m_CtrlPts.push_back(p03);
		SL1[1].m_CtrlPts.push_back(p04);
		SL1[1].m_CtrlPts.push_back(p05);

		SL1[2].m_CtrlPts.push_back(p07);
		SL1[2].m_CtrlPts.push_back(p06);
		SL1[2].m_CtrlPts.push_back(p05);


		SL1[3].m_CtrlPts.push_back(p01);
		SL1[3].m_CtrlPts.push_back(p08);
		SL1[3].m_CtrlPts.push_back(p07);

		//实例化曲面对象
		SplineSurface S1;
		//Coons插值成正方形面
		S1.CoonsInterpolate(SL1);
		SS.push_back(S1);

		SL2[0].m_CtrlPts.push_back(p01);
		SL2[0].m_CtrlPts.push_back(p08);
		SL2[0].m_CtrlPts.push_back(p07);

		SL2[1].m_CtrlPts.push_back(p07);
		SL2[1].m_CtrlPts.push_back(p13);
		SL2[1].m_CtrlPts.push_back(p12);

		SL2[2].m_CtrlPts.push_back(p10);
		SL2[2].m_CtrlPts.push_back(p11);
		SL2[2].m_CtrlPts.push_back(p12);


		SL2[3].m_CtrlPts.push_back(p01);
		SL2[3].m_CtrlPts.push_back(p09);
		SL2[3].m_CtrlPts.push_back(p10);
		//实例化曲面对象
		SplineSurface S2;
		//Coons插值成正方形面
		S2.CoonsInterpolate(SL2);
		SS.push_back(S2);

		SL3[0].m_CtrlPts.push_back(p07);
		SL3[0].m_CtrlPts.push_back(p06);
		SL3[0].m_CtrlPts.push_back(p05);

		SL3[1].m_CtrlPts.push_back(p05);
		SL3[1].m_CtrlPts.push_back(p16);
		SL3[1].m_CtrlPts.push_back(p15);

		SL3[2].m_CtrlPts.push_back(p12);
		SL3[2].m_CtrlPts.push_back(p14);
		SL3[2].m_CtrlPts.push_back(p15);


		SL3[3].m_CtrlPts.push_back(p07);
		SL3[3].m_CtrlPts.push_back(p13);
		SL3[3].m_CtrlPts.push_back(p12);

		//实例化曲面对象
		SplineSurface S3;
		//Coons插值成正方形面
		S3.CoonsInterpolate(SL3);
		SS.push_back(S3);

		SL4[0].m_CtrlPts.push_back(p05);
		SL4[0].m_CtrlPts.push_back(p04);
		SL4[0].m_CtrlPts.push_back(p03);

		SL4[1].m_CtrlPts.push_back(p03);
		SL4[1].m_CtrlPts.push_back(p19);
		SL4[1].m_CtrlPts.push_back(p18);

		SL4[2].m_CtrlPts.push_back(p15);
		SL4[2].m_CtrlPts.push_back(p17);
		SL4[2].m_CtrlPts.push_back(p18);


		SL4[3].m_CtrlPts.push_back(p05);
		SL4[3].m_CtrlPts.push_back(p16);
		SL4[3].m_CtrlPts.push_back(p15);

		//实例化曲面对象
		SplineSurface S4;
		//Coons插值成正方形面
		S4.CoonsInterpolate(SL4);
		SS.push_back(S4);

		SL5[0].m_CtrlPts.push_back(p15);
		SL5[0].m_CtrlPts.push_back(p17);
		SL5[0].m_CtrlPts.push_back(p18);

		SL5[1].m_CtrlPts.push_back(p18);
		SL5[1].m_CtrlPts.push_back(p20);
		SL5[1].m_CtrlPts.push_back(p21);

		SL5[2].m_CtrlPts.push_back(p23);
		SL5[2].m_CtrlPts.push_back(p22);
		SL5[2].m_CtrlPts.push_back(p21);


		SL5[3].m_CtrlPts.push_back(p15);
		SL5[3].m_CtrlPts.push_back(p24);
		SL5[3].m_CtrlPts.push_back(p23);

		//实例化曲面对象
		SplineSurface S5;
		//Coons插值成正方形面
		S5.CoonsInterpolate(SL5);
		SS.push_back(S5);

		SL6[0].m_CtrlPts.push_back(p12);
		SL6[0].m_CtrlPts.push_back(p14);
		SL6[0].m_CtrlPts.push_back(p15);

		SL6[1].m_CtrlPts.push_back(p15);
		SL6[1].m_CtrlPts.push_back(p24);
		SL6[1].m_CtrlPts.push_back(p23);

		SL6[2].m_CtrlPts.push_back(p26);
		SL6[2].m_CtrlPts.push_back(p42);
		SL6[2].m_CtrlPts.push_back(p23);


		SL6[3].m_CtrlPts.push_back(p12);
		SL6[3].m_CtrlPts.push_back(p25);
		SL6[3].m_CtrlPts.push_back(p26);

		//实例化曲面对象
		SplineSurface S6;
		//Coons插值成正方形面
		S6.CoonsInterpolate(SL6);
		SS.push_back(S6);

		SL7[0].m_CtrlPts.push_back(p10);
		SL7[0].m_CtrlPts.push_back(p11);
		SL7[0].m_CtrlPts.push_back(p12);

		SL7[1].m_CtrlPts.push_back(p12);
		SL7[1].m_CtrlPts.push_back(p25);
		SL7[1].m_CtrlPts.push_back(p26);

		SL7[2].m_CtrlPts.push_back(p28);
		SL7[2].m_CtrlPts.push_back(p27);
		SL7[2].m_CtrlPts.push_back(p26);


		SL7[3].m_CtrlPts.push_back(p10);
		SL7[3].m_CtrlPts.push_back(p29);
		SL7[3].m_CtrlPts.push_back(p28);

		//实例化曲面对象
		SplineSurface S7;
		//Coons插值成正方形面
		S7.CoonsInterpolate(SL7);
		SS.push_back(S7);

		SL8[0].m_CtrlPts.push_back(p23);
		SL8[0].m_CtrlPts.push_back(p22);
		SL8[0].m_CtrlPts.push_back(p21);

		SL8[1].m_CtrlPts.push_back(p21);
		SL8[1].m_CtrlPts.push_back(p30);
		SL8[1].m_CtrlPts.push_back(p31);

		SL8[2].m_CtrlPts.push_back(p33);
		SL8[2].m_CtrlPts.push_back(p32);
		SL8[2].m_CtrlPts.push_back(p31);


		SL8[3].m_CtrlPts.push_back(p23);
		SL8[3].m_CtrlPts.push_back(p34);
		SL8[3].m_CtrlPts.push_back(p33);

		//实例化曲面对象
		SplineSurface S8;
		//Coons插值成正方形面
		S8.CoonsInterpolate(SL8);
		SS.push_back(S8);

		SL9[0].m_CtrlPts.push_back(p44);
		SL9[0].m_CtrlPts.push_back(p43);
		SL9[0].m_CtrlPts.push_back(p23);

		SL9[1].m_CtrlPts.push_back(p23);
		SL9[1].m_CtrlPts.push_back(p34);
		SL9[1].m_CtrlPts.push_back(p33);

		SL9[2].m_CtrlPts.push_back(p46);
		SL9[2].m_CtrlPts.push_back(p47);
		SL9[2].m_CtrlPts.push_back(p33);


		SL9[3].m_CtrlPts.push_back(p44);
		SL9[3].m_CtrlPts.push_back(p45);
		SL9[3].m_CtrlPts.push_back(p46);

		//实例化曲面对象
		SplineSurface S9;
		//Coons插值成正方形面
		S9.CoonsInterpolate(SL9);
		SS.push_back(S9);

		SL10[0].m_CtrlPts.push_back(p26);
		SL10[0].m_CtrlPts.push_back(p42);
		SL10[0].m_CtrlPts.push_back(p23);

		SL10[1].m_CtrlPts.push_back(p23);
		SL10[1].m_CtrlPts.push_back(p43);
		SL10[1].m_CtrlPts.push_back(p44);

		SL10[2].m_CtrlPts.push_back(p53);
		SL10[2].m_CtrlPts.push_back(p54);
		SL10[2].m_CtrlPts.push_back(p44);


		SL10[3].m_CtrlPts.push_back(p26);
		SL10[3].m_CtrlPts.push_back(p52);
		SL10[3].m_CtrlPts.push_back(p53);

		//实例化曲面对象
		SplineSurface S10;
		//Coons插值成正方形面
		S10.CoonsInterpolate(SL10);
		SS.push_back(S10);

		SL11[0].m_CtrlPts.push_back(p26);
		SL11[0].m_CtrlPts.push_back(p52);
		SL11[0].m_CtrlPts.push_back(p53);

		SL11[1].m_CtrlPts.push_back(p53);
		SL11[1].m_CtrlPts.push_back(p51);
		SL11[1].m_CtrlPts.push_back(p49);

		SL11[2].m_CtrlPts.push_back(p38);
		SL11[2].m_CtrlPts.push_back(p50);
		SL11[2].m_CtrlPts.push_back(p49);


		SL11[3].m_CtrlPts.push_back(p26);
		SL11[3].m_CtrlPts.push_back(p41);
		SL11[3].m_CtrlPts.push_back(p38);

		//实例化曲面对象
		SplineSurface S11;
		//Coons插值成正方形面
		S11.CoonsInterpolate(SL11);
		SS.push_back(S11);

		SL12[0].m_CtrlPts.push_back(p49);
		SL12[0].m_CtrlPts.push_back(p48);
		SL12[0].m_CtrlPts.push_back(p46);

		SL12[1].m_CtrlPts.push_back(p46);
		SL12[1].m_CtrlPts.push_back(p47);
		SL12[1].m_CtrlPts.push_back(p33);

		SL12[2].m_CtrlPts.push_back(p38);
		SL12[2].m_CtrlPts.push_back(p39);
		SL12[2].m_CtrlPts.push_back(p33);


		SL12[3].m_CtrlPts.push_back(p49);
		SL12[3].m_CtrlPts.push_back(p50);
		SL12[3].m_CtrlPts.push_back(p38);

		//实例化曲面对象
		SplineSurface S12;
		//Coons插值成正方形面
		S12.CoonsInterpolate(SL12);
		SS.push_back(S12);

		SL13[0].m_CtrlPts.push_back(p38);
		SL13[0].m_CtrlPts.push_back(p39);
		SL13[0].m_CtrlPts.push_back(p33);

		SL13[1].m_CtrlPts.push_back(p33);
		SL13[1].m_CtrlPts.push_back(p32);
		SL13[1].m_CtrlPts.push_back(p31);

		SL13[2].m_CtrlPts.push_back(p36);
		SL13[2].m_CtrlPts.push_back(p35);
		SL13[2].m_CtrlPts.push_back(p31);


		SL13[3].m_CtrlPts.push_back(p38);
		SL13[3].m_CtrlPts.push_back(p37);
		SL13[3].m_CtrlPts.push_back(p36);

		//实例化曲面对象
		SplineSurface S13;
		//Coons插值成正方形面
		S13.CoonsInterpolate(SL13);
		SS.push_back(S13);


		SL14[0].m_CtrlPts.push_back(p28);
		SL14[0].m_CtrlPts.push_back(p27);
		SL14[0].m_CtrlPts.push_back(p26);

		SL14[1].m_CtrlPts.push_back(p26);
		SL14[1].m_CtrlPts.push_back(p41);
		SL14[1].m_CtrlPts.push_back(p38);

		SL14[2].m_CtrlPts.push_back(p36);
		SL14[2].m_CtrlPts.push_back(p37);
		SL14[2].m_CtrlPts.push_back(p38);


		SL14[3].m_CtrlPts.push_back(p28);
		SL14[3].m_CtrlPts.push_back(p40);
		SL14[3].m_CtrlPts.push_back(p36);

		//实例化曲面对象
		SplineSurface S14;
		//Coons插值成正方形面
		S14.CoonsInterpolate(SL14);
		SS.push_back(S14);

		//公用函数类实例化
		Model_Solution M;
		////绕x轴旋转90°,参数中是用弧度表示
		//M.Rolate(SS, PI/2, 1);

		////新创建一个容器s，用于中间存储，之后再放到SS中，用于一起显示
		//varray<SplineSurface> s;
		//for (int i = 0; i < 14; i++) {
		//	s.push_back(SS[i]);
		//}
		//
		////多面旋转 绕z轴逆时针转180°
		//M.Rolate(SS, PI, 3);
		////多面再向右平移8
		//M.Trans(SS, 8, 1);
		////再像上平移3.5
		//M.Trans(SS, 3.5, 2);
		//
		////将中间存储的面放到SS中
		//for (int i = 0; i < 14; i++) {
		//	SS.push_back(s[i]);
		//}

		////多面拉伸成体
		//varray<SplineVolume> SV;
		////这个是我又添加的一个函数，多增加了一个拉伸方向参数，下面这个用来相对方向拉伸
		//SV=M.CreatSweepVol(SS, 0.5, 2,-2);

		varray<SplineVolume> SV;
		SV = M.CreatSweepVol(SS, 0.5, 3);

		

		//体细化函数是针对一个体，所以要用循环
		/*for (int i = 0; i < SV.size(); i++)
		{
			SV[i].KnotsRefineNum(1);
		}*/

		NurbsTrans n;
		varray<NurbsVol> ns;
		ns = n.SplinevolsToCvols(SV);

		//输出体文件
		RWGeometric rwg;
		rwg.WriteNurbsVol("E:\\kuang_models\\nurbsvolbox0.txt", ns);
	}
	//06.给冯的模型---s2体
	void kuang_s2()
	{
		//创建容器，存放线
		//存放三个片的四条边
		varray<Spline> SL1;
		varray<Spline> SL2;
		varray<Spline> SL3;
		//创建容器，存放曲面,用于多面拉伸
		varray<SplineSurface> SS;
		//存放节点 p=2
		varray<double> knots;
		knots.push_back(0);
		knots.push_back(0);
		knots.push_back(0);
		knots.push_back(1);
		knots.push_back(1);
		knots.push_back(1);
		//指定大小，可存放四条线，用于Coons插值成面
		SL1.resize(4);
		SL2.resize(4);
		SL3.resize(4);
		for (int i = 0; i < 4; i++)
		{
			//给定四条边次数、节点矢量
			SL1[i].m_Degree = 2;
			SL1[i].m_Knots = knots;

			SL2[i].m_Degree = 2;
			SL2[i].m_Knots = knots;

			SL3[i].m_Degree = 2;
			SL3[i].m_Knots = knots;
		}
		//给出长方体1控制点
		Vec4 p01 = { 0,0,0,1 };
		Vec4 p02 = { 0,1.75,0,1 };
		Vec4 p03 = { 0,3.5,0,1 };
		Vec4 p04 = { 1.375,3.5,0,1 };
		Vec4 p05 = { 2.75,3.5,0,1 };
		Vec4 p06 = { 2.75,1.75,0,1 };
		Vec4 p07 = { 2.75,0,0,1 };
		Vec4 p08 = { 1.375,0,0,1 };
		Vec4 p09 = { 3.84467,3.5,0,1 };
		Vec4 p10 = { 4.93934,3.5,0,1 };
		Vec4 p11 = { 4.93934,1.75,0,1 };
		Vec4 p12 = { 4.93934,0,0,1 };
		Vec4 p13 = { 3.84467,0,0,1 };
		Vec4 p14 = { 6.46967,3.5,0,1 };
		Vec4 p15 = { 8 ,3.5,0,1 };
		Vec4 p16 = { 8,1.75,0,1 };
		Vec4 p17 = { 8,0,0,1 };
		Vec4 p18 = { 6.46967 ,0,0,1 };

		//将点放到线容器中
		//用coons插值成正方形面，点的存放是有顺序的，顺时针，从左到右，从下到上
		SL1[0].m_CtrlPts.push_back(p01);
		SL1[0].m_CtrlPts.push_back(p02);
		SL1[0].m_CtrlPts.push_back(p03);

		SL1[1].m_CtrlPts.push_back(p03);
		SL1[1].m_CtrlPts.push_back(p04);
		SL1[1].m_CtrlPts.push_back(p05);

		SL1[2].m_CtrlPts.push_back(p07);
		SL1[2].m_CtrlPts.push_back(p06);
		SL1[2].m_CtrlPts.push_back(p05);


		SL1[3].m_CtrlPts.push_back(p01);
		SL1[3].m_CtrlPts.push_back(p08);
		SL1[3].m_CtrlPts.push_back(p07);

		//实例化曲面对象
		SplineSurface S1;
		//Coons插值成正方形面
		S1.CoonsInterpolate(SL1);
		SS.push_back(S1);

		SL2[0].m_CtrlPts.push_back(p07);
		SL2[0].m_CtrlPts.push_back(p06);
		SL2[0].m_CtrlPts.push_back(p05);

		SL2[1].m_CtrlPts.push_back(p05);
		SL2[1].m_CtrlPts.push_back(p09);
		SL2[1].m_CtrlPts.push_back(p10);

		SL2[2].m_CtrlPts.push_back(p12);
		SL2[2].m_CtrlPts.push_back(p11);
		SL2[2].m_CtrlPts.push_back(p10);


		SL2[3].m_CtrlPts.push_back(p07);
		SL2[3].m_CtrlPts.push_back(p13);
		SL2[3].m_CtrlPts.push_back(p12);

		//实例化曲面对象
		SplineSurface S2;
		//Coons插值成正方形面
		S2.CoonsInterpolate(SL2);
		SS.push_back(S2);

		SL3[0].m_CtrlPts.push_back(p12);
		SL3[0].m_CtrlPts.push_back(p11);
		SL3[0].m_CtrlPts.push_back(p10);

		SL3[1].m_CtrlPts.push_back(p10);
		SL3[1].m_CtrlPts.push_back(p14);
		SL3[1].m_CtrlPts.push_back(p15);

		SL3[2].m_CtrlPts.push_back(p17);
		SL3[2].m_CtrlPts.push_back(p16);
		SL3[2].m_CtrlPts.push_back(p15);


		SL3[3].m_CtrlPts.push_back(p12);
		SL3[3].m_CtrlPts.push_back(p18);
		SL3[3].m_CtrlPts.push_back(p17);

		//实例化曲面对象
		SplineSurface S3;
		//Coons插值成正方形面
		S3.CoonsInterpolate(SL3);
		SS.push_back(S3);

		//公用函数类实例化
		Model_Solution M;
		//多面拉伸成体
		varray<SplineVolume> SV;
		SV = M.CreatSweepVol(SS, 0.5, -3);

		//输出体文件
		RWGeometric rwg;
		rwg.WriteSplineVolume("E:\\kuang_models\\SplineVolume_s2.txt", SV);

		//TestBolcks::pList().OutputParaVolumeDataTxt(SV, "E:\\kuang_models\\SplineVolume_mao_s2.txt");
	}
	//07.给冯的模型---s3、s4体	
	void kuang_s3() {
		//创建容器，存放线
		//存放正方形的四条边
		varray<Spline> SL1;
		varray<Spline> SL2;
		varray<Spline> SL3;
		
		//存放节点 p=2
		varray<double> knots;
		knots.push_back(0);
		knots.push_back(0);
		knots.push_back(0);
		knots.push_back(1);
		knots.push_back(1);
		knots.push_back(1);
		//指定大小，可存放四条线，用于Coons插值成面
		SL1.resize(4);
		SL2.resize(4);
		SL3.resize(4);
		for (int i = 0; i < 4; i++)
		{
			//给定次数、节点矢量
			SL1[i].m_Degree = 2;
			SL1[i].m_Knots = knots;

			SL2[i].m_Degree = 2;
			SL2[i].m_Knots = knots;

			SL3[i].m_Degree = 2;
			SL3[i].m_Knots = knots;
		}
		//给出长方体控制点
		double m = -0.25, n = -0.5;
		Vec4 p01 = { 0,0,0,1 };
		Vec4 p02 = { 0,0,m,1 };
		Vec4 p03 = { 0,0,n,1 };
		Vec4 p04 = { 0,m,n,1 };
		Vec4 p05 = { 0,n,n,1 };
		Vec4 p06 = { 0,n,m,1 };
		Vec4 p07 = { 0,n,0,1 };
		Vec4 p08 = { 0,m,0,1 };

		Vec4 p09 = { 2.75,0,0,1 };
		Vec4 p10 = { 2.75,0,m,1 };
		Vec4 p11 = { 2.75,0,n,1 };
		Vec4 p12 = { 2.75,m,n,1 };
		Vec4 p13 = { 2.75,n,n,1 };
		Vec4 p14 = { 2.75,n,m,1 };
		Vec4 p15 = { 2.75,n,0,1 };
		Vec4 p16 = { 2.75,m,0,1 };

		Vec4 p17 = { 4.93934,0,0,1 };
		Vec4 p18 = { 4.93934,0,m,1 };
		Vec4 p19 = { 4.93934,0,n,1 };
		Vec4 p20 = { 4.93934,m,n,1 };
		Vec4 p21 = { 4.93934,n,n,1 };
		Vec4 p22 = { 4.93934,n,m,1 };
		Vec4 p23 = { 4.93934,n,0,1 };
		Vec4 p24 = { 4.93934,m,0,1 };
		//将点放到线容器中
		//用coons插值成正方形面，点的存放是有顺序的，顺时针，从左到右，从下到上
		SL1[0].m_CtrlPts.push_back(p01);
		SL1[0].m_CtrlPts.push_back(p02);
		SL1[0].m_CtrlPts.push_back(p03);

		SL1[1].m_CtrlPts.push_back(p05);
		SL1[1].m_CtrlPts.push_back(p04);
		SL1[1].m_CtrlPts.push_back(p03);

		SL1[2].m_CtrlPts.push_back(p05);
		SL1[2].m_CtrlPts.push_back(p06);
		SL1[2].m_CtrlPts.push_back(p07);


		SL1[3].m_CtrlPts.push_back(p07);
		SL1[3].m_CtrlPts.push_back(p08);
		SL1[3].m_CtrlPts.push_back(p01);

		//实例化曲面对象
		SplineSurface S1;
		//Coons插值成正方形面
		S1.CoonsInterpolate(SL1);

		SL2[0].m_CtrlPts.push_back(p09);
		SL2[0].m_CtrlPts.push_back(p10);
		SL2[0].m_CtrlPts.push_back(p11);

		SL2[1].m_CtrlPts.push_back(p13); 
		SL2[1].m_CtrlPts.push_back(p12);
		SL2[1].m_CtrlPts.push_back(p11);

		SL2[2].m_CtrlPts.push_back(p13);
		SL2[2].m_CtrlPts.push_back(p14);
		SL2[2].m_CtrlPts.push_back(p15);


		SL2[3].m_CtrlPts.push_back(p15);
		SL2[3].m_CtrlPts.push_back(p16);
		SL2[3].m_CtrlPts.push_back(p09);

		//实例化曲面对象
		SplineSurface S2;
		//Coons插值成正方形面
		S2.CoonsInterpolate(SL2);

		SL3[0].m_CtrlPts.push_back(p17);
		SL3[0].m_CtrlPts.push_back(p18);
		SL3[0].m_CtrlPts.push_back(p19);

		SL3[1].m_CtrlPts.push_back(p21);
		SL3[1].m_CtrlPts.push_back(p20);
		SL3[1].m_CtrlPts.push_back(p19);

		SL3[2].m_CtrlPts.push_back(p21);
		SL3[2].m_CtrlPts.push_back(p22);
		SL3[2].m_CtrlPts.push_back(p23);


		SL3[3].m_CtrlPts.push_back(p23);
		SL3[3].m_CtrlPts.push_back(p24);
		SL3[3].m_CtrlPts.push_back(p17);

		//实例化曲面对象
		SplineSurface S3;
		//Coons插值成正方形面
		S3.CoonsInterpolate(SL3);


		//放到体容器中
		varray<SplineVolume> SV;

		//公用函数类实例化
		Model_Solution M;

		//单面拉伸成体
		Creat_Vol cv1;
		Creat_Vol cv2;
		Creat_Vol cv3;
		//这一部分修改了扫描路径函数里面的一些东西，默认写的路径上只有三个控制点
		cv1.InitVol(S1, 2.75, 1);//生成体对象NV
		SV.push_back(cv1.NV);
		
		cv2.InitVol(S2, 2.18934, 1);
		SV.push_back(cv2.NV);

		cv3.InitVol(S3, 3.06066, 1);
		SV.push_back(cv3.NV);

		//下面是第四个体模型
		//向+y方向平移4 得到原型
		M.Trans(SV, 4, 2);
		//体模型的中间存储---原型
		varray<SplineVolume> SV1;
		for (int i = 0; i < SV.size(); i++) {
			SV1.push_back(SV[i]);
		}

		//向+y方向平移0.5 得到第二个
		M.Trans(SV, 0.5, 2);
		varray<SplineVolume> SV2;
		for (int i = 0; i < SV.size(); i++) {
			SV2.push_back(SV[i]);
		}

		//向+z方向移动4.5得到第三个
		M.Trans(SV, 4.5, 3);

		//将前面两个再放到体容器中
		for (int i = 0; i < 3; i++) {
			SV.push_back(SV1[i]);
		}
		for (int i = 0; i < 3; i++) {
			SV.push_back(SV2[i]);
		}

		//下面是第三个体模型
		////体模型的中间存储---原型
		//varray<SplineVolume> SV1;
		//for (int i = 0; i < SV.size(); i++) {
		//	SV1.push_back(SV[i]);
		//}

		////向-y方向平移0.5
		//M.Trans(SV, 0.5, -2);

		////移动后的模型，存起来
		//varray<SplineVolume> SV2;
		//for (int i = 0; i < SV.size(); i++) {
		//	SV2.push_back(SV[i]);
		//}

		////向+y方向平移0.5
		//M.Trans(SV, 0.5, 2);
		////再向+z方向平移4.5
		//M.Trans(SV, 4.5, 3);
		//

		//for (int i = 0; i < 3; i++) {
		//	SV.push_back(SV1[i]);
		//}
		//for (int i = 0; i < 3; i++) {
		//	SV.push_back(SV2[i]);
		//}
		
		
		//输出体文件
		RWGeometric rwg;
		rwg.WriteSplineVolume("E:\\kuang_models\\SplineVolume_s4.txt", SV);
	}

	//08.给冯的模型---v1体 l:边长 r:内圆半径
	void kuang_v1(double l,double r) {
		//创建容器，存放线
		//存放正方形的四条边
		varray<Spline> SL1;
		//存放多面
		varray<SplineSurface> SS;
		//存放节点 p=2
		varray<double> knots;
		knots.push_back(0);
		knots.push_back(0);
		knots.push_back(0);
		knots.push_back(1);
		knots.push_back(1);
		knots.push_back(1);
		//指定大小，可存放四条线，用于Coons插值成面
		SL1.resize(4);
		for (int i = 0; i < 4; i++)
		{
			//给定次数、节点矢量
			SL1[i].m_Degree = 2;
			SL1[i].m_Knots = knots;
		}
		
		Vec4 p01 = { -l / 2,-l / 2,0,1 };
		Vec4 p02 = { -l / 2,0,0,1 };
		Vec4 p03 = { -l / 2,l / 2,0,1 };
		Vec4 p04 = { -(l + sqrt(2)*r) / 4,l / 2 - (l - sqrt(2)*r) / 4,0,1 };
		Vec4 p05 = { -(l / 2 - (l - sqrt(2)*r) / 2),l / 2 - (l - sqrt(2)*r) / 2,0,1 };
		Vec4 p06 = { -sqrt(2)*r,0,0,0.707 };
		Vec4 p07 = { -(l / 2 - (l - sqrt(2)*r) / 2), -(l / 2 - (l - sqrt(2)*r) / 2),0,1 };
		Vec4 p08 = { -(l / 2 - (l - sqrt(2)*r) / 4),-(l / 2 - (l - sqrt(2)*r) / 4),0,1 };

		//将点放到线容器中
		//用coons插值成正方形面，点的存放是有顺序的，顺时针，从左到右，从下到上
		SL1[0].m_CtrlPts.push_back(p01);
		SL1[0].m_CtrlPts.push_back(p02);
		SL1[0].m_CtrlPts.push_back(p03);

		SL1[1].m_CtrlPts.push_back(p03);
		SL1[1].m_CtrlPts.push_back(p04);
		SL1[1].m_CtrlPts.push_back(p05);

		SL1[2].m_CtrlPts.push_back(p07);
		SL1[2].m_CtrlPts.push_back(p06);
		SL1[2].m_CtrlPts.push_back(p05);


		SL1[3].m_CtrlPts.push_back(p01);
		SL1[3].m_CtrlPts.push_back(p08);
		SL1[3].m_CtrlPts.push_back(p07);

		//实例化曲面对象
		SplineSurface S1;
		//Coons插值成正方形面
		S1.CoonsInterpolate(SL1);
		SS.push_back(S1);

		Model_Solution m;
		m.Rolate(S1, PI / 2, 3);
		SS.push_back(S1);
		m.Rolate(S1, PI / 2, 3);
		SS.push_back(S1);
		m.Rolate(S1, PI / 2, 3);
		SS.push_back(S1);

		varray<SplineVolume> SV;
		SV = m.CreatSweepVol(SS, 4, 3);

		
		//输出体文件
		RWGeometric rwg;
		rwg.WriteSplineSurface("E:\\Model\\SplineSurface_v3.txt", SS);
		rwg.WriteSplineVolume("E:\\Model\\SplineVolume_v3.txt", SV);
		////转换格式文件
		//TestBolcks::pList().OutputParaVolumeDataTxt(SV, "E:\\kuang_models\\SplineVolume_kuang_xie5.txt");

	}
	//09.给冯的模型---v2体 l:内部正方形边长 r:外圆半径 h:高度
	void kuang_v2(double r,double h) {
		//创建容器，存放线
		//存放正方形的四条边
		varray<Spline> SL1;
		varray<Spline> SL2;
		//存放多面
		varray<SplineSurface> SS;
		//存放节点 p=2
		varray<double> knots;
		knots.push_back(0);
		knots.push_back(0);
		knots.push_back(0);
		knots.push_back(1);
		knots.push_back(1);
		knots.push_back(1);
		//指定大小，可存放四条线，用于Coons插值成面
		SL1.resize(4);
		SL2.resize(4);
		for (int i = 0; i < 4; i++)
		{
			//给定次数、节点矢量
			SL1[i].m_Degree = 2;
			SL1[i].m_Knots = knots;
			SL2[i].m_Degree = 2;
			SL2[i].m_Knots = knots;
			
		}
		double l = sqrt(2)*r / 2;
		Vec4 p01 = { -sqrt(2)*r / 2,-sqrt(2)*r / 2,0,1 };
		Vec4 p02 = { -sqrt(2)*r,0,0,0.707 };
		Vec4 p03 = { -sqrt(2)*r / 2,sqrt(2)*r / 2,0,1 };
		Vec4 p04 = { -sqrt(2)*r * 3 / 8,sqrt(2)*r * 3 / 8,0,1 };
		Vec4 p05 = { -l / 2 ,l / 2 ,0,1 };
		Vec4 p06 = { -l / 2,0,0,1 };
		Vec4 p07 = { -l / 2, -l / 2,0,1 };
		Vec4 p08 = { -sqrt(2)*r * 3 / 8,-sqrt(2)*r * 3 / 8,0,1 };
		
		Vec4 p12 = { 0 ,l / 2 ,0,1 };
		Vec4 p13 = { l / 2 ,l / 2 ,0,1 };
		Vec4 p14 = { l / 2 ,0 ,0,1 };
		Vec4 p15 = { l / 2 ,-l / 2 ,0,1 };
		Vec4 p16 = { 0 ,-l / 2 ,0,1 };
		
		//将点放到线容器中
		//用coons插值成正方形面，点的存放是有顺序的，顺时针，从左到右，从下到上
		SL1[0].m_CtrlPts.push_back(p01);
		SL1[0].m_CtrlPts.push_back(p02);
		SL1[0].m_CtrlPts.push_back(p03);

		SL1[1].m_CtrlPts.push_back(p03);
		SL1[1].m_CtrlPts.push_back(p04);
		SL1[1].m_CtrlPts.push_back(p05);

		SL1[2].m_CtrlPts.push_back(p07);
		SL1[2].m_CtrlPts.push_back(p06);
		SL1[2].m_CtrlPts.push_back(p05);


		SL1[3].m_CtrlPts.push_back(p01);
		SL1[3].m_CtrlPts.push_back(p08);
		SL1[3].m_CtrlPts.push_back(p07);

		//实例化曲面对象
		SplineSurface S1;
		//Coons插值成正方形面
		S1.CoonsInterpolate(SL1);
		SS.push_back(S1);

		SL2[0].m_CtrlPts.push_back(p07);
		SL2[0].m_CtrlPts.push_back(p06);
		SL2[0].m_CtrlPts.push_back(p05);

		SL2[1].m_CtrlPts.push_back(p05);
		SL2[1].m_CtrlPts.push_back(p12);
		SL2[1].m_CtrlPts.push_back(p13);

		SL2[2].m_CtrlPts.push_back(p15);
		SL2[2].m_CtrlPts.push_back(p14);
		SL2[2].m_CtrlPts.push_back(p13);


		SL2[3].m_CtrlPts.push_back(p07);
		SL2[3].m_CtrlPts.push_back(p16);
		SL2[3].m_CtrlPts.push_back(p15);

		//实例化曲面对象
		SplineSurface S2;
		//Coons插值成正方形面
		S2.CoonsInterpolate(SL2);
		SS.push_back(S2);

		Model_Solution m;
		m.Rolate(S1, PI / 2, 3);
		SS.push_back(S1);
		m.Rolate(S1, PI / 2, 3);
		SS.push_back(S1);
		m.Rolate(S1, PI / 2, 3);
		SS.push_back(S1);
		
		varray<SplineVolume> SV;
		SV = m.CreatSweepVol(SS,h,4);



		//输出体文件
		RWGeometric rwg;
		rwg.WriteSplineSurface("E:\\Model\\SplineSurface_v3.txt", SS);
		rwg.WriteSplineVolume("E:\\Model\\SplineVolume_v3.txt", SV);
		////转换格式文件
		//TestBolcks::pList().OutputParaVolumeDataTxt(SV, "E:\\kuang_models\\SplineVolume_kuang_xie5.txt");

	}
	//10.给冯的模型---v3体 r:外圆半径 r1：扫描路径的半径   **这里面有格式转换**
	void kuang_v3(double r, double r1) {
		//创建容器，存放线
		//存放正方形的四条边
		varray<Spline> SL1;
		varray<Spline> SL2;
		//存放多面
		varray<SplineSurface> SS;
		//存放节点 p=2
		varray<double> knots;
		knots.push_back(0);
		knots.push_back(0);
		knots.push_back(0);
		knots.push_back(1);
		knots.push_back(1);
		knots.push_back(1);
		//指定大小，可存放四条线，用于Coons插值成面
		SL1.resize(4);
		SL2.resize(4);
		for (int i = 0; i < 4; i++)
		{
			//给定次数、节点矢量
			SL1[i].m_Degree = 2;
			SL1[i].m_Knots = knots;
			SL2[i].m_Degree = 2;
			SL2[i].m_Knots = knots;

		}
		double l = sqrt(2)*r / 2;
		Vec4 p01 = { -sqrt(2)*r / 2,-sqrt(2)*r / 2,0,1 };
		Vec4 p02 = { -sqrt(2)*r,0,0,0.707 };
		Vec4 p03 = { -sqrt(2)*r / 2,sqrt(2)*r / 2,0,1 };
		Vec4 p04 = { -sqrt(2)*r * 3 / 8,sqrt(2)*r * 3 / 8,0,1 };
		Vec4 p05 = { -l / 2 ,l / 2 ,0,1 };
		Vec4 p06 = { -l / 2,0,0,1 };
		Vec4 p07 = { -l / 2, -l / 2,0,1 };
		Vec4 p08 = { -sqrt(2)*r * 3 / 8,-sqrt(2)*r * 3 / 8,0,1 };

		Vec4 p12 = { 0 ,l / 2 ,0,1 };
		Vec4 p13 = { l / 2 ,l / 2 ,0,1 };
		Vec4 p14 = { l / 2 ,0 ,0,1 };
		Vec4 p15 = { l / 2 ,-l / 2 ,0,1 };
		Vec4 p16 = { 0 ,-l / 2 ,0,1 };

		//将点放到线容器中
		//用coons插值成正方形面，点的存放是有顺序的，顺时针，从左到右，从下到上
		SL1[0].m_CtrlPts.push_back(p01);
		SL1[0].m_CtrlPts.push_back(p02);
		SL1[0].m_CtrlPts.push_back(p03);

		SL1[1].m_CtrlPts.push_back(p03);
		SL1[1].m_CtrlPts.push_back(p04);
		SL1[1].m_CtrlPts.push_back(p05);

		SL1[2].m_CtrlPts.push_back(p07);
		SL1[2].m_CtrlPts.push_back(p06);
		SL1[2].m_CtrlPts.push_back(p05);


		SL1[3].m_CtrlPts.push_back(p01);
		SL1[3].m_CtrlPts.push_back(p08);
		SL1[3].m_CtrlPts.push_back(p07);

		//实例化曲面对象
		SplineSurface S1;
		//Coons插值成正方形面
		S1.CoonsInterpolate(SL1);
		SS.push_back(S1);

		SL2[0].m_CtrlPts.push_back(p07);
		SL2[0].m_CtrlPts.push_back(p06);
		SL2[0].m_CtrlPts.push_back(p05);

		SL2[1].m_CtrlPts.push_back(p05);
		SL2[1].m_CtrlPts.push_back(p12);
		SL2[1].m_CtrlPts.push_back(p13);

		SL2[2].m_CtrlPts.push_back(p15);
		SL2[2].m_CtrlPts.push_back(p14);
		SL2[2].m_CtrlPts.push_back(p13);


		SL2[3].m_CtrlPts.push_back(p07);
		SL2[3].m_CtrlPts.push_back(p16);
		SL2[3].m_CtrlPts.push_back(p15);

		//实例化曲面对象
		SplineSurface S2;
		//Coons插值成正方形面
		S2.CoonsInterpolate(SL2);
		SS.push_back(S2);

		Model_Solution m;
		m.Rolate(S1, PI / 2, 3);
		SS.push_back(S1);
		m.Rolate(S1, PI / 2, 3);
		SS.push_back(S1);
		m.Rolate(S1, PI / 2, 3);
		SS.push_back(S1);



		//创建两条圆弧路径
		Spline L1;
		Spline L2;

		varray<double> knots1;
		knots1.push_back(0);
		knots1.push_back(0);
		knots1.push_back(0);
		knots1.push_back(1);
		knots1.push_back(1);
		knots1.push_back(1);

		L1.m_Degree = 2;
		L1.m_Knots = knots1;

		L2.m_Degree = 2;
		L2.m_Knots = knots1;
		
		Vec4 p1 = { 0,0,0,1 };
		Vec4 p2 = { 0,0,r1,0.707 };
		Vec4 p3 = { 0,-r1,r1,1 };

		Vec4 p4 = { 0,-2 * r1,r1,0.707 };
		Vec4 p5 = { 0,-2 * r1,2 * r1,0.707 };
		

		L1.m_CtrlPts.push_back(p1);
		L1.m_CtrlPts.push_back(p2);
		L1.m_CtrlPts.push_back(p3);
		
		L2.m_CtrlPts.push_back(p3);
		L2.m_CtrlPts.push_back(p4);
		L2.m_CtrlPts.push_back(p5);

		varray<Spline> L;
		L.push_back(L1);
		L.push_back(L2);

		//由于改变不完全的原因，需要进行格式转换，才能用放样以及垂直于路径扫描的功能
		//Spline与Cnurbslines等之间的转换
		//包含在NurbsTrans.h的头文件中
		NurbsLine nl;
		varray<NurbsSurface>sfs;
		NurbsVol vol;
		varray<NurbsVol> vols;
		//这个还需要指定大小，否则会报错
		vols.resize(5);

		nl=NurbsTrans::SplineToCnurbsline(L1);
		sfs = NurbsTrans::SplinesurfsToCsurfs(SS);
		
		for (int i = 0; i < sfs.size(); i++)
		{
			vols[i].CreateSweepNurbsVol(nl, sfs[i], 2);
		}
		
		//再转换回来
		varray<SplineVolume> SV;
		varray<SplineVolume> SV1;

		SV=NurbsTrans::CvolsToSplinevols(vols);
		//存放原型  容器要用push_back进行加入元素，用“=”会报错的
		for (int i = 0; i < 5; i++) {
			SV1.push_back(SV[i]);
		}

		//旋转
		m.Rolate(SV, PI, 2);
		m.Rolate(SV, PI, 3);
		m.Trans(SV, 2 * r1, -2);
		m.Trans(SV, 2 * r1, 3);

		for (int i = 0; i < 5; i++) {
			SV.push_back(SV1[i]);
		}

		//输出体文件
		RWGeometric rwg;
		rwg.WriteSpline("E:\\Model\\Spline_v3.txt", L);
		rwg.WriteSplineSurface("E:\\Model\\SplineSurface_v3.txt", SS);
		rwg.WriteSplineVolume("E:\\Model\\SplineVolume_v3.txt", SV);
		////转换格式文件
		//TestBolcks::pList().OutputParaVolumeDataTxt(SV, "E:\\kuang_models\\SplineVolume_kuang_xie5.txt");

	}
	//11.给冯的模型---v4体 三角形旋转 l:三角形边长 r:圆形扫描路径的半径
	void kuang_v4(double l,double r) {
		//创建容器，存放线
		//存放正方形的四条边
		varray<Spline> SL1;
		//存放多面、多线，用于最后写出
		varray<SplineSurface> SS;
		varray<Spline> SL;
		//存放节点 p=2
		varray<double> knots;
		knots.push_back(0);
		knots.push_back(0);
		knots.push_back(0);
		knots.push_back(1);
		knots.push_back(1);
		knots.push_back(1);
		//指定大小，可存放四条线，用于Coons插值成面
		SL1.resize(4);

		for (int i = 0; i < 4; i++)
		{
			//给定次数、节点矢量
			SL1[i].m_Degree = 2;
			SL1[i].m_Knots = knots;


		}
		double H = sqrt(3)* l / 2;
		double h = sqrt(3) * l / 6;
		Vec4 p01 = { -l / 2,-h,0,1 };
		Vec4 p02 = { -l * 3 / 8,-(h - H / 4),0,1 };
		Vec4 p03 = { -l / 4,H / 2 - h,0,1 };
		Vec4 p04 = { -l / 8,(H / 2 - h) / 2,0,1 };
		Vec4 p05 = { 0,0,0,1 };
		Vec4 p06 = { 0,-h / 2,0,1 };
		Vec4 p07 = { 0, -h,0,1 };
		Vec4 p08 = { -l / 4,-h,0,1 };

		

		//将点放到线容器中
		//用coons插值成正方形面，点的存放是有顺序的，顺时针，从左到右，从下到上
		SL1[0].m_CtrlPts.push_back(p01);
		SL1[0].m_CtrlPts.push_back(p02);
		SL1[0].m_CtrlPts.push_back(p03);

		SL1[1].m_CtrlPts.push_back(p03);
		SL1[1].m_CtrlPts.push_back(p04);
		SL1[1].m_CtrlPts.push_back(p05);

		SL1[2].m_CtrlPts.push_back(p07);
		SL1[2].m_CtrlPts.push_back(p06);
		SL1[2].m_CtrlPts.push_back(p05);


		SL1[3].m_CtrlPts.push_back(p01);
		SL1[3].m_CtrlPts.push_back(p08);
		SL1[3].m_CtrlPts.push_back(p07);

		//实例化曲面对象
		SplineSurface S1;
		//Coons插值成正方形面
		S1.CoonsInterpolate(SL1);
		SS.push_back(S1);

		Model_Solution m;
		m.Rolate(S1, PI * 2 / 3, 3);
		SS.push_back(S1);
		m.Rolate(S1, PI * 2 / 3, 3);
		SS.push_back(S1);

		//创建扫描路径——圆形
		varray<Spline> NLS;
		varray<double> Knots;
		Knots.push_back(0);
		Knots.push_back(0);
		Knots.push_back(0);
		Knots.push_back(1);
		Knots.push_back(1);
		Knots.push_back(1);
		NLS.resize(4);
		for (int i = 0; i < 4; i++) {
			NLS[i].m_Degree = 2;
			NLS[i].m_Knots = Knots;
		}

		//sqrt() 开根号 圆形扫描路径半径
		double a = sqrt(2) / 2;
		Vec4 u1 = { 0,0,0,1 };
		Vec4 u2 = { 0,0,r,a };
		Vec4 u3 = { 0,-r,r,1 };

		Vec4 u4 = { 0,-2*r,r,a };
		Vec4 u5 = { 0,-2*r,0,1 };
		Vec4 u6 = { 0,-2*r,-r,a };
		Vec4 u7 = { 0,-r,-r,1 };
		Vec4 u8 = { 0,0,-r,a };

		NLS[0].m_CtrlPts.push_back(u1);
		NLS[0].m_CtrlPts.push_back(u2);
		NLS[0].m_CtrlPts.push_back(u3);

		NLS[1].m_CtrlPts.push_back(u3);
		NLS[1].m_CtrlPts.push_back(u4);
		NLS[1].m_CtrlPts.push_back(u5);

		NLS[2].m_CtrlPts.push_back(u5);
		NLS[2].m_CtrlPts.push_back(u6);
		NLS[2].m_CtrlPts.push_back(u7);

		NLS[3].m_CtrlPts.push_back(u7);
		NLS[3].m_CtrlPts.push_back(u8);
		NLS[3].m_CtrlPts.push_back(u1);
		for (int i = 0; i < 4; i++) {
			SL.push_back(NLS[i]);
		}
		

		//由于改变不完全的原因，需要进行格式转换，才能用放样以及垂直于路径扫描的功能
		//Spline与Cnurbslines等之间的转换
		//包含在NurbsTrans.h的头文件中
		NurbsLine nl;
		varray<NurbsSurface>sfs;
		NurbsVol vol;
		varray<NurbsVol> vols;
		//这个还需要指定大小，否则会报错
		vols.resize(3);

		nl = NurbsTrans::SplineToCnurbsline(SL[0]);
		sfs = NurbsTrans::SplinesurfsToCsurfs(SS);

		for (int i = 0; i < sfs.size(); i++)
		{
			vols[i].CreateSweepNurbsVol(nl, sfs[i], 2);
		}

		//体格式再转换回来
		varray<SplineVolume> SV;
		SV = NurbsTrans::CvolsToSplinevols(vols);
		
		varray<SplineVolume> SV1;
		varray<SplineVolume> SV2;
		varray<SplineVolume> SV3;
		//原型
		for (int i = 0; i < SV.size(); i++) {
			SV1.push_back(SV[i]);
		}
		
		m.Rolate(SV, PI, 3);
		m.Trans(SV, 2*r, -2);
		for (int i = 0; i < SV.size(); i++) {
			SV2.push_back(SV[i]);
		}
		m.Rolate(SV, PI, 2);
		for (int i = 0; i < SV.size(); i++) {
			SV3.push_back(SV[i]);
		}
		m.Rolate(SV, PI, 3);
		m.Trans(SV, 2*r, -2);

		for (int i = 0; i < 3; i++) {
			SV.push_back(SV1[i]);
		}
		for (int i = 0; i < 3; i++) {
			SV.push_back(SV2[i]);
		}
		for (int i = 0; i < 3; i++) {
			SV.push_back(SV3[i]);
		}

		varray<NurbsVol> S;
		S = NurbsTrans::SplinevolsToCvols(SV);
		RWGeometric rwg;
		rwg.WriteNurbsVol("E:\\Model\\SplineVolume_v3.txt", S);
	
		\
		//输出体文件
		/*RWGeometric rwg;
		rwg.WriteSpline("E:\\kuang_models\\Spline_kuang_v4.txt", SL);
		rwg.WriteSplineSurface("E:\\kuang_models\\SplineSurface_kuang_v4.txt", SS);
		rwg.WriteSplineVolume("E:\\kuang_models\\SplineVolume_kuang_v4.txt", SV);*/
		////转换格式文件
		//TestBolcks::pList().OutputParaVolumeDataTxt(SV, "E:\\kuang_models\\SplineVolume_kuang_xie5.txt");

	}
	//12.给冯的模型---v5体 两面放样 l1:大面边长 l2:小面边长 h:两面之间的距离
	void kuang_v5(double l1,double l2,double h) {
		//创建容器，存放线
		//存放正方形的四条边
		varray<Spline> SL1;
		varray<Spline> SL2;
		//创建容器，存放曲面,用于多面拉伸
		varray<SplineSurface> SS;
		//存放节点 p=2
		varray<double> knots;
		knots.push_back(0);
		knots.push_back(0);
		knots.push_back(0);
		knots.push_back(1);
		knots.push_back(1);
		knots.push_back(1);
		//指定大小，可存放四条线，用于Coons插值成面
		SL1.resize(4);
		SL2.resize(4);
		for (int i = 0; i < 4; i++)
		{
			//给定次数、节点矢量
			SL1[i].m_Degree = 2;
			SL1[i].m_Knots = knots;
			SL2[i].m_Degree = 2;
			SL2[i].m_Knots = knots;
		}
		double w=cos(65*PI/180);
		Vec4 p01 = { 0,0,0,1 };
		Vec4 p02 = { l1 / 2 * sin(25 * PI / 180),l1/2,0,w };
		Vec4 p03 = { 0,l1,0,1 };
		Vec4 p04 = { l1/2,-(l1 / 2 * sin(25 * PI / 180)-l1),0,w };
		Vec4 p05 = { l1,l1,0,1 };
		Vec4 p06 = { -(l1 / 2 * sin(25 * PI / 180) - l1),l1/2,0,w };
		Vec4 p07 = { l1,0,0,1 };
		Vec4 p08 = { l1/2,l1 / 2 * sin(25 * PI / 180),0,w };

		Vec4 p09 = { 0,0,h,1 };
		Vec4 p10 = { -(l2 / 2 * sin(25 * PI / 180)),l2 / 2,h,w };
		Vec4 p11 = { 0,l2,h,1 };
		Vec4 p12 = { l2 / 2,(l2 / 2 * sin(25 * PI / 180) + l2),h,w };
		Vec4 p13 = { l2,l2,h,1 };
		Vec4 p14 = { (l2 / 2 * sin(25 * PI / 180) + l2),l2 / 2,h,w };
		Vec4 p15 = { l2,0,h,1 };
		Vec4 p16 = { l2 / 2,-(l2 / 2 * sin(25 * PI / 180)),h,w };
		//将点放到线容器中
		//用coons插值成正方形面，点的存放是有顺序的，顺时针，从左到右，从下到上
		SL1[0].m_CtrlPts.push_back(p01);
		SL1[0].m_CtrlPts.push_back(p02);
		SL1[0].m_CtrlPts.push_back(p03);

		SL1[1].m_CtrlPts.push_back(p03);
		SL1[1].m_CtrlPts.push_back(p04);
		SL1[1].m_CtrlPts.push_back(p05);

		SL1[2].m_CtrlPts.push_back(p07);
		SL1[2].m_CtrlPts.push_back(p06);
		SL1[2].m_CtrlPts.push_back(p05);


		SL1[3].m_CtrlPts.push_back(p01);
		SL1[3].m_CtrlPts.push_back(p08);
		SL1[3].m_CtrlPts.push_back(p07);

		//实例化曲面对象
		SplineSurface S1;
		//Coons插值成正方形面
		S1.CoonsInterpolate(SL1);
		SS.push_back(S1);

		SL2[0].m_CtrlPts.push_back(p09);
		SL2[0].m_CtrlPts.push_back(p10);
		SL2[0].m_CtrlPts.push_back(p11);

		SL2[1].m_CtrlPts.push_back(p11);
		SL2[1].m_CtrlPts.push_back(p12);
		SL2[1].m_CtrlPts.push_back(p13);

		SL2[2].m_CtrlPts.push_back(p15);
		SL2[2].m_CtrlPts.push_back(p14);
		SL2[2].m_CtrlPts.push_back(p13);


		SL2[3].m_CtrlPts.push_back(p09);
		SL2[3].m_CtrlPts.push_back(p16);
		SL2[3].m_CtrlPts.push_back(p15);

		//实例化曲面对象
		SplineSurface S2;
		//Coons插值成正方形面
		S2.CoonsInterpolate(SL2);

		//平移 放样也有方向的要求，所以对小面进行了旋转平移
		Model_Solution m;
		m.Rolate(S2, PI, 2);
		m.Trans(S2, l1/2+l2/2, 1);
		m.Trans(S2, l1/2-l2/2 , 2);
	
		SS.push_back(S2);

		//放样路径是两面中心的连线
		Spline NL;
		NL.m_Degree = 2;
		NL.m_Knots.push_back(0);
		NL.m_Knots.push_back(0);
		NL.m_Knots.push_back(0);
		NL.m_Knots.push_back(1);
		NL.m_Knots.push_back(1);
		NL.m_Knots.push_back(1);
		
		Vec4 u1 = { l1 / 2,l1 / 2,0,1 };
		Vec4 u2 = { l1 / 2,l1 / 2,-h/2,1 };
		Vec4 u3 = { l1 / 2,l1 / 2,-h,1 };
		
		NL.m_CtrlPts.push_back(u1);
		NL.m_CtrlPts.push_back(u2);
		NL.m_CtrlPts.push_back(u3);

		varray<Spline> SL;
		SL.push_back(NL);
		
		NurbsLine nl;
		varray<NurbsSurface> ns;
		NurbsVol nv;
		nl = NurbsTrans::SplineToCnurbsline(NL);
		ns = NurbsTrans::SplinesurfsToCsurfs(SS);
		nv.LoftingNurbsVol(nl, ns[0], ns[1]);

		SplineVolume sv;
		sv = NurbsTrans::CnurbsvolToSplinevol(nv);
		varray<SplineVolume> SV;
		SV.push_back(sv);
		
		//输出体文件
		RWGeometric rwg;
		//rwg.WriteSpline("E:\\kuang_models\\Spline_v5.txt", SL);
		//rwg.WriteSplineSurface("E:\\kuang_models\\SplineSurface_v5.txt", SS);
		//rwg.WriteSplineVolume("E:\\kuang_models\\SV1.txt", SV);
		////转换格式文件
		TestBolcks::pList().OutputParaVolumeDataTxt(SV, "E:\\kuang_models\\SV1.txt");

	}




	//给谢的模型---板子——单片
	void kuang_xie_banzi() {
		//创建容器，存放线
		//存放正方形的四条边
		varray<Spline> SL1;
		//创建容器，存放曲面,用于多面拉伸
		varray<SplineSurface> SS;
		//存放节点 p=2
		varray<double> knots;
		knots.push_back(0);
		knots.push_back(0);
		knots.push_back(0);
		knots.push_back(1);
		knots.push_back(1);
		knots.push_back(1);
		//指定大小，可存放四条线，用于Coons插值成面
		SL1.resize(4);
		for (int i = 0; i < 4; i++)
		{
			//给定次数、节点矢量
			SL1[i].m_Degree = 2;
			SL1[i].m_Knots = knots;
		}
		//给出长方体控制点
		//v=0
		Vec4 u1 = { 0,0,0,1 };
		Vec4 u2 = { 127,0,0,1 };
		Vec4 u3 = { 254,0,0,1 };
		//u=0
		Vec4 v1 = { 0,15.24,0,1 };
		Vec4 v2 = { 0,30.48,0,1 };
		//v=1
		Vec4 u4 = { 127,30.48,0,1 };
		Vec4 u5 = { 254,30.48,0,1 };
		//u=1
		Vec4 v3 = { 254,15.24,0,1 };
		//将点放到线容器中
		//用coons插值成正方形面，点的存放是有顺序的，顺时针，从左到右，从下到上
		SL1[0].m_CtrlPts.push_back(u1);
		SL1[0].m_CtrlPts.push_back(u2);
		SL1[0].m_CtrlPts.push_back(u3);

		SL1[1].m_CtrlPts.push_back(u1);
		SL1[1].m_CtrlPts.push_back(v1);
		SL1[1].m_CtrlPts.push_back(v2);
		
		SL1[2].m_CtrlPts.push_back(v2);
		SL1[2].m_CtrlPts.push_back(u4);
		SL1[2].m_CtrlPts.push_back(u5);
		

		SL1[3].m_CtrlPts.push_back(u3);
		SL1[3].m_CtrlPts.push_back(v3);
		SL1[3].m_CtrlPts.push_back(u5);

		//实例化曲面对象
		SplineSurface S1;
		//Coons插值成正方形面
		S1.CoonsInterpolate(SL1);

		//单面拉伸成体
		Creat_Vol cv;
		//这一部分修改了扫描路径函数里面的一些东西，以达到可以产生四个控制点（默认写的路径上只有三个控制点）
		cv.InitVol(S1, 15.24, 3);//生成体对象NV

		//体细化
		cv.NV.KnotsRefineNum(5);
	
		//放到体容器中
		varray<SplineVolume> SV;
		SV.push_back(cv.NV);
		//输出体文件
		RWGeometric rwg;
		rwg.WriteSplineVolume("E:\\kuang_models\\SplineVolume_kuang_xie5.txt", SV);
		////转换格式文件
		//TestBolcks::pList().OutputParaVolumeDataTxt(SV, "E:\\kuang_models\\SplineVolume_kuang_xie5.txt");

	}
	//给谢的模型---板子——三片
	void kuang_xie_banzi1() {
		//创建容器，存放线
		//存放三个片四条边
		varray<Spline> SL1;
		varray<Spline> SL2;
		varray<Spline> SL3;
		//创建容器，存放曲面,用于多面拉伸
		varray<SplineSurface> SS;
		//存放节点 p=2
		varray<double> knots;
		knots.push_back(0);
		knots.push_back(0);
		knots.push_back(0);
		knots.push_back(1);
		knots.push_back(1);
		knots.push_back(1);
		//指定大小，可存放四条线，用于Coons插值成面
		SL1.resize(4);
		SL2.resize(4);
		SL3.resize(4);
		for (int i = 0; i < 4; i++)
		{
			//给定四条边次数、节点矢量
			SL1[i].m_Degree = 2;
			SL1[i].m_Knots = knots;

			SL2[i].m_Degree = 2;
			SL2[i].m_Knots = knots;

			SL3[i].m_Degree = 2;
			SL3[i].m_Knots = knots;
		}
		//给出长方体1控制点
		//v=0
		Vec4 u1 = { 0,0,0,1 };
		Vec4 u2 = { 254/6,0,0,1 };
		Vec4 u3 = { 254/3,0,0,1 };
		//u=0
		Vec4 v1 = { 0,15.24,0,1 };
		Vec4 v2 = { 0,30.48,0,1 };
		//v=1
		Vec4 u4 = { 254 / 6,30.48,0,1 };
		Vec4 u5 = { 254/3,30.48,0,1 };
		//u=1
		Vec4 v3 = { 254/3,15.24,0,1 };

		//给出长方体2控制点
		//v=0
		Vec4 uu1 = { 254 / 2,0,0,1 };
		Vec4 uu2 = { 254 *2/ 3,0,0,1 };
		//u=0
		//v=1
		Vec4 uu3 = { 254 / 2,30.48,0,1 };
		Vec4 uu4 = { 254 *2/ 3,30.48,0,1 };
		//u=1
		Vec4 v4 = { 254 *2/ 3,15.24,0,1 };

		//给出长方体3控制点
		//v=0
		Vec4 uuu1 = { 254 *5/ 6,0,0,1 };
		Vec4 uuu2 = { 254 ,0,0,1 };
		//u=0
		//v=1
		Vec4 uuu3 = { 254 *5/ 6,30.48,0,1 };
		Vec4 uuu4 = { 254,30.48,0,1 };
		//u=1
		Vec4 v5 = { 254 ,15.24,0,1 };

		//将点放到线容器中
		//用coons插值成正方形面，点的存放是有顺序的，顺时针，从左到右，从下到上
		SL1[0].m_CtrlPts.push_back(u1);
		SL1[0].m_CtrlPts.push_back(u2);
		SL1[0].m_CtrlPts.push_back(u3);

		SL1[1].m_CtrlPts.push_back(u1);
		SL1[1].m_CtrlPts.push_back(v1);
		SL1[1].m_CtrlPts.push_back(v2);

		SL1[2].m_CtrlPts.push_back(v2);
		SL1[2].m_CtrlPts.push_back(u4);
		SL1[2].m_CtrlPts.push_back(u5);


		SL1[3].m_CtrlPts.push_back(u3);
		SL1[3].m_CtrlPts.push_back(v3);
		SL1[3].m_CtrlPts.push_back(u5);

		//实例化曲面对象
		SplineSurface S1;
		//Coons插值成正方形面
		S1.CoonsInterpolate(SL1);
		SS.push_back(S1);

		SL2[0].m_CtrlPts.push_back(u3);
		SL2[0].m_CtrlPts.push_back(uu1);
		SL2[0].m_CtrlPts.push_back(uu2);

		SL2[1].m_CtrlPts.push_back(u3);
		SL2[1].m_CtrlPts.push_back(v3);
		SL2[1].m_CtrlPts.push_back(u5);

		SL2[2].m_CtrlPts.push_back(u5);
		SL2[2].m_CtrlPts.push_back(uu3);
		SL2[2].m_CtrlPts.push_back(uu4);


		SL2[3].m_CtrlPts.push_back(uu2);
		SL2[3].m_CtrlPts.push_back(v4);
		SL2[3].m_CtrlPts.push_back(uu4);

		//实例化曲面对象
		SplineSurface S2;
		//Coons插值成正方形面
		S2.CoonsInterpolate(SL2);
		SS.push_back(S2);

		SL3[0].m_CtrlPts.push_back(uu2);
		SL3[0].m_CtrlPts.push_back(uuu1);
		SL3[0].m_CtrlPts.push_back(uuu2);

		SL3[1].m_CtrlPts.push_back(uu2);
		SL3[1].m_CtrlPts.push_back(v4);
		SL3[1].m_CtrlPts.push_back(uu4);

		SL3[2].m_CtrlPts.push_back(uu4);
		SL3[2].m_CtrlPts.push_back(uuu3);
		SL3[2].m_CtrlPts.push_back(uuu4);


		SL3[3].m_CtrlPts.push_back(uuu2);
		SL3[3].m_CtrlPts.push_back(v5);
		SL3[3].m_CtrlPts.push_back(uuu4);

		//实例化曲面对象
		SplineSurface S3;
		//Coons插值成正方形面
		S3.CoonsInterpolate(SL3);
		SS.push_back(S3);

		//公用函数类实例化
		Model_Solution M;
		//多面拉伸成体
		varray<SplineVolume> SV;
		SV = M.CreatSweepVol(SS, 15.24, 3);

		//体细化
		for (int i = 0; i < SV.size(); i++)
		{
			SV[i].KnotsRefineNum(3);
			SV[i].OrderCtrlPts(SV[i]);
		}

		

		//输出体文件
		RWGeometric rwg;
		rwg.WriteSplineVolume("E:\\kuang_models\\kuang\\SplineVolume3.txt", SV);

		PublicSolution ps;
		//ps.outPutVTK(SV, "E:\\kuang_models\\kuang\\SplineVolume0.vtk");
		//转换格式文件
		TestBolcks::pList().OutputParaVolumeDataTxt(SV, "E:\\kuang_models\\kuang\\SplineVolume3");

	}
};

/*	
	回转压板
*/
class Press_plate {
public:
	/*
		矩形被圆所截剩余部分
		x和y: 确定位置的坐标
		r: 圆形半径
	*/
	void rec(double x,double y,double r) {
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
		double w = sqrt(2) / 2;
		double R = 2 * r;
		Vec4 p01 = { x,y,0,1 };
		Vec4 p02 = { x,y + R,0,1 };
		Vec4 p03 = { x,y + 2 * R,0,1 };
		Vec4 p05 = { x + R - cos(PI/4)*r,y + R + cos(PI / 4)*r,0,1 };
		Vec4 p04 = (p03 + p05) / 2;
		Vec4 p06 = { x + R - sqrt(2)*r,y + R,0,w };
		Vec4 p07 = { x + R - cos(PI / 4)*r,y + R - cos(PI / 4)*r,0,1 };
		Vec4 p08 = (p07 + p01) / 2;

		SL1[0].m_CtrlPts.push_back(p01);
		SL1[0].m_CtrlPts.push_back(p02);
		SL1[0].m_CtrlPts.push_back(p03);

		SL1[1].m_CtrlPts.push_back(p03);
		SL1[1].m_CtrlPts.push_back(p04);
		SL1[1].m_CtrlPts.push_back(p05);

		SL1[2].m_CtrlPts.push_back(p07);
		SL1[2].m_CtrlPts.push_back(p06);
		SL1[2].m_CtrlPts.push_back(p05);

		SL1[3].m_CtrlPts.push_back(p01);
		SL1[3].m_CtrlPts.push_back(p08);
		SL1[3].m_CtrlPts.push_back(p07);
		SplineSurface ss1;
		ss1.CoonsInterpolate(SL1);

		
		varray<SplineSurface> SS1;
		SS1.push_back(ss1);
		m.Rolate(ss1, PI / 2, 3);
		SS1.push_back(ss1);
		m.Rolate(ss1, PI / 2, 3);
		SS1.push_back(ss1);
		m.Rolate(ss1, PI / 2, 3);
		SS1.push_back(ss1);
		m.Trans(SS1, R * 2, 1);

		for (int i = 0; i < SS1.size(); i++) {
			SS.push_back(SS1[i]);
		}
	}

	/*
		左边圆弧部分+中间部分
	*/
	void rec_circle(double r) {
		double R = 2 * r;
		varray<SplineSurface> SS1;
		varray<Spline> SL1;
		varray<Spline> SL2;
		varray<Spline> SL3;
		varray<Spline> SL4;
		varray<Spline> SL5;
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
		}
		Vec4 p01 = { 0,-R,0,1 };
		Vec4 p03 = { 0,-r,0,1 };
		Vec4 p02 = (p01 + p03) / 2;
		Vec4 p04 = { r,-r,0,w };
		Vec4 p05 = { r,0,0,1 };
		Vec4 p07 = { 3 * r,0,0,1 };
		Vec4 p06 = (p05 + p07) / 2;
		Vec4 p08 = (p01 + p07) / 2;

		Vec4 p10 = { 2 * R,-R,0,1 };
		Vec4 p09 = (p01 + p10) / 2;
		Vec4 p12 = { 2 * R,-r,0,1 };
		Vec4 p11 = (p10 + p12) / 2;
		Vec4 p13 = { 3*r,-r,0,w };

		Vec4 p14 = { r,r,0,w };
		Vec4 p15 = { 0,r,0,1 };
		Vec4 p17 = { 0,R,0,1 };
		Vec4 p16 = (p15 + p17) / 2;

		
		Vec4 p18 = (p17 + p07) / 2;
		Vec4 p19 = { 3*r,r,0,w };
		Vec4 p20 = { 2 * R,r,0,1 };

		Vec4 p22 = { 2 * R,R,0,1 };
		Vec4 p21 = (p22 + p20) / 2;
		Vec4 p23 = (p17 + p22) / 2;


		//圆弧
		Vec4 p24 = { -R,-R,0,w };
		Vec4 p25 = { -R,0,0,1 };
		Vec4 p27 = { -r,0,0,1 };
		Vec4 p26 = (p25 + p27) / 2;
		Vec4 p28 = { -r,-r,0,w };
		

		SL1[0].m_CtrlPts.push_back(p01);
		SL1[0].m_CtrlPts.push_back(p02);
		SL1[0].m_CtrlPts.push_back(p03);

		SL1[1].m_CtrlPts.push_back(p03);
		SL1[1].m_CtrlPts.push_back(p04);
		SL1[1].m_CtrlPts.push_back(p05);

		SL1[2].m_CtrlPts.push_back(p07);
		SL1[2].m_CtrlPts.push_back(p06);
		SL1[2].m_CtrlPts.push_back(p05);

		SL1[3].m_CtrlPts.push_back(p01);
		SL1[3].m_CtrlPts.push_back(p08);
		SL1[3].m_CtrlPts.push_back(p07);
		SplineSurface ss1;
		ss1.CoonsInterpolate(SL1);
		SS1.push_back(ss1);

		SL2[0].m_CtrlPts.push_back(p01);
		SL2[0].m_CtrlPts.push_back(p08);
		SL2[0].m_CtrlPts.push_back(p07);

		SL2[1].m_CtrlPts.push_back(p07);
		SL2[1].m_CtrlPts.push_back(p13);
		SL2[1].m_CtrlPts.push_back(p12);

		SL2[2].m_CtrlPts.push_back(p10);
		SL2[2].m_CtrlPts.push_back(p11);
		SL2[2].m_CtrlPts.push_back(p12);

		SL2[3].m_CtrlPts.push_back(p01);
		SL2[3].m_CtrlPts.push_back(p09);
		SL2[3].m_CtrlPts.push_back(p10);
		SplineSurface ss2;
		ss2.CoonsInterpolate(SL2);
		SS1.push_back(ss2);

		SL3[0].m_CtrlPts.push_back(p05);
		SL3[0].m_CtrlPts.push_back(p14);
		SL3[0].m_CtrlPts.push_back(p15);

		SL3[1].m_CtrlPts.push_back(p15);
		SL3[1].m_CtrlPts.push_back(p16);
		SL3[1].m_CtrlPts.push_back(p17);

		SL3[2].m_CtrlPts.push_back(p07);
		SL3[2].m_CtrlPts.push_back(p18);
		SL3[2].m_CtrlPts.push_back(p17);

		SL3[3].m_CtrlPts.push_back(p05);
		SL3[3].m_CtrlPts.push_back(p06);
		SL3[3].m_CtrlPts.push_back(p07);
		SplineSurface ss3;
		ss3.CoonsInterpolate(SL3);
		SS1.push_back(ss3);

		SL4[0].m_CtrlPts.push_back(p07);
		SL4[0].m_CtrlPts.push_back(p18);
		SL4[0].m_CtrlPts.push_back(p17);

		SL4[1].m_CtrlPts.push_back(p17);
		SL4[1].m_CtrlPts.push_back(p23);
		SL4[1].m_CtrlPts.push_back(p22);

		SL4[2].m_CtrlPts.push_back(p20);
		SL4[2].m_CtrlPts.push_back(p21);
		SL4[2].m_CtrlPts.push_back(p22);

		SL4[3].m_CtrlPts.push_back(p07);
		SL4[3].m_CtrlPts.push_back(p19);
		SL4[3].m_CtrlPts.push_back(p20);
		SplineSurface ss4;
		ss4.CoonsInterpolate(SL4);
		SS1.push_back(ss4);

		SL5[0].m_CtrlPts.push_back(p01);
		SL5[0].m_CtrlPts.push_back(p24);
		SL5[0].m_CtrlPts.push_back(p25);

		SL5[1].m_CtrlPts.push_back(p25);
		SL5[1].m_CtrlPts.push_back(p26);
		SL5[1].m_CtrlPts.push_back(p27);

		SL5[2].m_CtrlPts.push_back(p03);
		SL5[2].m_CtrlPts.push_back(p28);
		SL5[2].m_CtrlPts.push_back(p27);

		SL5[3].m_CtrlPts.push_back(p01);
		SL5[3].m_CtrlPts.push_back(p02);
		SL5[3].m_CtrlPts.push_back(p03);
		SplineSurface ss5;
		ss5.CoonsInterpolate(SL5);
		SS1.push_back(ss5);
		m.Rolate(ss5, -PI / 2, 3);
		SS1.push_back(ss5);
		for (int i = 0; i < SS1.size(); i++) {
			SS.push_back(SS1[i]);
		}
	}

	/*
		右边两部分
	*/
	void rec_c2(double r) {
		double R = 2 * r;
		varray<SplineSurface> SS1;
		varray<Spline> SL1;
		varray<Spline> SL2;
		varray<Spline> SL3;
		varray<Spline> SL4;
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
		for (int i = 0; i < 4; i++) {
			SL1[i].m_Degree = 2;
			SL1[i].m_Knots = knots;
			SL2[i].m_Degree = 2;
			SL2[i].m_Knots = knots;
			SL3[i].m_Degree = 2;
			SL3[i].m_Knots = knots;
			SL4[i].m_Degree = 2;
			SL4[i].m_Knots = knots;
		}
		Vec4 p01 = { 0,-R,0,1 };
		Vec4 p03 = { 0,-r,0,1 };
		Vec4 p02 = (p01 + p03) / 2;
		Vec4 p05 = { 5 * r / 2,-3 * r / 2,0,1 };
		Vec4 p04 = (p03 + p05) / 2;
		Vec4 p06 = { 5 * r / 2,-R,0,w };
		Vec4 p07 = { 2 * r,-R,0,1 };
		Vec4 p08 = (p01 + p07) / 2;
		Vec4 p09 = { r,-r,0,w };
		Vec4 p10 = { r,0,0,1 };
		
		Vec4 p12 = { 5 * r / 2,0,0,1 };
		Vec4 p11 = (p10 + p12) / 2;
		Vec4 p13 = (p12 + p05) / 2;

		Vec4 p14 = { r,r,0,w };
		Vec4 p15 = { 0,r,0,1 };
		Vec4 p17 = { 0,R,0,1 };
		Vec4 p16 = (p15 + p17) / 2;


		Vec4 p18 = (p17 + p12) / 2;
		Vec4 p20 = { 7*r/2,R,0,1 };
		Vec4 p19 = (p17 + p20) / 2;
		Vec4 p22 = { 7 * r / 2,r,0,1 };
		Vec4 p21 = (p22 + p20) / 2;
		Vec4 p23 = { 5 * r / 2,r,0,w };

		Vec4 p24 = { -R,-R,0,w };
		Vec4 p25 = { -R,0,0,1 };
		Vec4 p27 = { -r,0,0,1 };
		Vec4 p26 = (p19 + p21) / 2;
		Vec4 p28 = { -r,-r,0,w };


		SL1[0].m_CtrlPts.push_back(p01);
		SL1[0].m_CtrlPts.push_back(p02);
		SL1[0].m_CtrlPts.push_back(p03);

		SL1[1].m_CtrlPts.push_back(p03);
		SL1[1].m_CtrlPts.push_back(p04);
		SL1[1].m_CtrlPts.push_back(p05);

		SL1[2].m_CtrlPts.push_back(p07);
		SL1[2].m_CtrlPts.push_back(p06);
		SL1[2].m_CtrlPts.push_back(p05);

		SL1[3].m_CtrlPts.push_back(p01);
		SL1[3].m_CtrlPts.push_back(p08);
		SL1[3].m_CtrlPts.push_back(p07);
		SplineSurface ss1;
		ss1.CoonsInterpolate(SL1);
		SS1.push_back(ss1);

		SL2[0].m_CtrlPts.push_back(p03);
		SL2[0].m_CtrlPts.push_back(p09);
		SL2[0].m_CtrlPts.push_back(p10);

		SL2[1].m_CtrlPts.push_back(p10);
		SL2[1].m_CtrlPts.push_back(p11);
		SL2[1].m_CtrlPts.push_back(p12);

		SL2[2].m_CtrlPts.push_back(p05);
		SL2[2].m_CtrlPts.push_back(p13);
		SL2[2].m_CtrlPts.push_back(p12);

		SL2[3].m_CtrlPts.push_back(p03);
		SL2[3].m_CtrlPts.push_back(p04);
		SL2[3].m_CtrlPts.push_back(p05);
		SplineSurface ss2;
		ss2.CoonsInterpolate(SL2);
		SS1.push_back(ss2);

		SL3[0].m_CtrlPts.push_back(p10);
		SL3[0].m_CtrlPts.push_back(p14);
		SL3[0].m_CtrlPts.push_back(p15);

		SL3[1].m_CtrlPts.push_back(p15);
		SL3[1].m_CtrlPts.push_back(p16);
		SL3[1].m_CtrlPts.push_back(p17);

		SL3[2].m_CtrlPts.push_back(p12);
		SL3[2].m_CtrlPts.push_back(p18);
		SL3[2].m_CtrlPts.push_back(p17);

		SL3[3].m_CtrlPts.push_back(p10);
		SL3[3].m_CtrlPts.push_back(p11);
		SL3[3].m_CtrlPts.push_back(p12);
		SplineSurface ss3;
		ss3.CoonsInterpolate(SL3);
		SS1.push_back(ss3);

		SL4[0].m_CtrlPts.push_back(p12);
		SL4[0].m_CtrlPts.push_back(p18);
		SL4[0].m_CtrlPts.push_back(p17);

		SL4[1].m_CtrlPts.push_back(p17);
		SL4[1].m_CtrlPts.push_back(p19);
		SL4[1].m_CtrlPts.push_back(p20);

		SL4[2].m_CtrlPts.push_back(p22);
		SL4[2].m_CtrlPts.push_back(p21);
		SL4[2].m_CtrlPts.push_back(p20);

		SL4[3].m_CtrlPts.push_back(p12);
		SL4[3].m_CtrlPts.push_back(p23);
		SL4[3].m_CtrlPts.push_back(p22);
		SplineSurface ss4;
		ss4.CoonsInterpolate(SL4);
		SS1.push_back(ss4);

		m.Trans(SS1, 2 * R, 1);

		for (int i = 0; i < SS1.size(); i++) {
			SS.push_back(SS1[i]);
		}
	}

	void rec_c3(double r) {
		double R = 2 * r;
		varray<SplineSurface> SS1;
		varray<Spline> SL1;
		varray<Spline> SL2;
		varray<Spline> SL3;
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
		for (int i = 0; i < 4; i++) {
			SL1[i].m_Degree = 2;
			SL1[i].m_Knots = knots;
			SL2[i].m_Degree = 2;
			SL2[i].m_Knots = knots;
			SL3[i].m_Degree = 2;
			SL3[i].m_Knots = knots;
		}
		Vec4 p01 = { r,0,0,1 };
		Vec4 p02 = { r,r,0,w };
		Vec4 p03 = { 0,r,0,1 };
		Vec4 p05 = { 0,R,0,1 };
		Vec4 p04 = (p03 + p05) / 2;
		Vec4 p06 = { R,R,0,w };
		Vec4 p07 = { R,0,0,1 };
		Vec4 p08 = (p01 + p07) / 2;
		Vec4 p10 = { r,-r,0,1 };
		Vec4 p09 = (p10 + p01) / 2;
		

		Vec4 p12 = { R,-r,0,1 };
		Vec4 p13 = (p07 + p12) / 2;
		Vec4 p11 = (p10 + p12) / 2;

		Vec4 p14 = { r,-5 * r / 4,0,w };
		Vec4 p15 = { 5 * r / 4,-5 * r / 4,0,1 };
		Vec4 p17 = { 7 * r / 4,-5 * r / 4,0,1 };
		Vec4 p16 = (p15 + p17) / 2;
		Vec4 p18 = { R,-5 * r / 4,0,w };


		SL1[0].m_CtrlPts.push_back(p01);
		SL1[0].m_CtrlPts.push_back(p02);
		SL1[0].m_CtrlPts.push_back(p03);

		SL1[1].m_CtrlPts.push_back(p03);
		SL1[1].m_CtrlPts.push_back(p04);
		SL1[1].m_CtrlPts.push_back(p05);

		SL1[2].m_CtrlPts.push_back(p07);
		SL1[2].m_CtrlPts.push_back(p06);
		SL1[2].m_CtrlPts.push_back(p05);

		SL1[3].m_CtrlPts.push_back(p01);
		SL1[3].m_CtrlPts.push_back(p08);
		SL1[3].m_CtrlPts.push_back(p07);
		SplineSurface ss1;
		ss1.CoonsInterpolate(SL1);
		SS1.push_back(ss1);

		SL2[0].m_CtrlPts.push_back(p10);
		SL2[0].m_CtrlPts.push_back(p09);
		SL2[0].m_CtrlPts.push_back(p01);

		SL2[1].m_CtrlPts.push_back(p01);
		SL2[1].m_CtrlPts.push_back(p08);
		SL2[1].m_CtrlPts.push_back(p07);

		SL2[2].m_CtrlPts.push_back(p12);
		SL2[2].m_CtrlPts.push_back(p13);
		SL2[2].m_CtrlPts.push_back(p07);

		SL2[3].m_CtrlPts.push_back(p10);
		SL2[3].m_CtrlPts.push_back(p11);
		SL2[3].m_CtrlPts.push_back(p12);
		SplineSurface ss2;
		ss2.CoonsInterpolate(SL2);
		SS1.push_back(ss2);

		SL3[0].m_CtrlPts.push_back(p15);
		SL3[0].m_CtrlPts.push_back(p14);
		SL3[0].m_CtrlPts.push_back(p10);

		SL3[1].m_CtrlPts.push_back(p10);
		SL3[1].m_CtrlPts.push_back(p11);
		SL3[1].m_CtrlPts.push_back(p12);

		SL3[2].m_CtrlPts.push_back(p17);
		SL3[2].m_CtrlPts.push_back(p18);
		SL3[2].m_CtrlPts.push_back(p12);

		SL3[3].m_CtrlPts.push_back(p15);
		SL3[3].m_CtrlPts.push_back(p16);
		SL3[3].m_CtrlPts.push_back(p17);
		SplineSurface ss3;
		ss3.CoonsInterpolate(SL3);
		SS1.push_back(ss3);

		m.Trans(SS1, 3 * R + 3 * r / 2, 1);

		for (int i = 0; i < SS1.size(); i++) {
			SS.push_back(SS1[i]);
		}
	}

	/*
		多面拉伸成体
		r:内圆半径
		h:厚度
	*/
	void get_vol(double r,double h) {
		rec_circle(r);
		rec_c2(r);
		rec_c3(r);
		varray<SplineVolume> SV1;
		SV1 = m.CreatSweepVol(SS, h, 3);
		for (int i = 0; i < SV1.size(); i++) {
			SV.push_back(SV1[i]);
		}
	}

	void writeSplineSurface() {
		rwg.WriteSplineSurface("E:\\kuang_models\\Surfaces.txt", SS);
	}

	void writeSplineVolume() {
		rwg.WriteSplineVolume("E:\\kuang_models\\Volume.txt", SV);
	}
private:
	varray<SplineSurface> SS; //创建容器，存放曲面,用于多面拉伸
	varray<SplineVolume> SV;
	RWGeometric rwg;          //读写文件
	Model_Solution m;		  //通用函数类对象	 
	double w = cos(PI / 4);   //权重
};

/*
	医疗精密配件
*/
class Precise_part {
public:
	/*
		r:倒角半径
		h:拉伸厚度
		a:外部矩形长
		b:外部矩形宽
		c:内部矩形长
		d:内部矩形宽
	*/
	varray<SplineSurface> rec_circle(double r,double a,double b,double c,double d) {
		varray<SplineSurface> SS1;
		varray<Spline> SL1;
		varray<Spline> SL2;
		varray<Spline> SL3;
		varray<Spline> SL4;
		varray<Spline> SL5;
		varray<Spline> SL6;
		varray<Spline> SL7;
		varray<Spline> SL8;
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
		}
		Vec4 p01 = { -(a / 2 - r),-b / 2,0,1 };
		Vec4 p02 = { -a / 2,-b / 2,0,w };
		Vec4 p03 = { -a / 2,-(b / 2 - r),0,1 };

		Vec4 p06 = { -(c / 2 - r),-d / 2,0,1 };
		Vec4 p05 = { -c / 2,-d / 2,0,w };
		Vec4 p04 = { -c / 2,-(d / 2 - r),0,1 };

		Vec4 p09 = { -(a / 2 - r),b / 2,0,1 };
		Vec4 p08 = { -a / 2,b / 2,0,w };
		Vec4 p07 = { -a / 2,b / 2 - r,0,1 };

		Vec4 p10 = { -(c / 2 - r),d / 2,0,1 };
		Vec4 p11 = { -c / 2,d / 2,0,w };
		Vec4 p12 = { -c / 2,d / 2 - r,0,1 };

		Vec4 p13 = { c / 2 - r,d / 2,0,1 };
		Vec4 p14 = { c / 2,d / 2,0,w };
		Vec4 p15 = { c / 2,d / 2 - r,0,1 };

		Vec4 p16 = { a / 2 - r,b / 2,0,1 };
		Vec4 p17 = { a / 2,b / 2,0,w };
		Vec4 p18 = { a / 2,b / 2 - r,0,1 };

		Vec4 p19 = { a / 2 ,-b / 2+r,0,1 };
		Vec4 p20 = { a / 2,-b / 2,0,w };
		Vec4 p21 = { a / 2 - r,-b / 2,0,1 };

		Vec4 p22 = { c / 2,-d / 2 +r,0,1 };
		Vec4 p23 = { c / 2,-d / 2,0,w };
		Vec4 p24 = { c / 2 - r,-d / 2,0,1 };

		SL1[0].m_CtrlPts.push_back(p01);
		SL1[0].m_CtrlPts.push_back(p02);
		SL1[0].m_CtrlPts.push_back(p03);

		SL1[1].m_CtrlPts.push_back(p03);
		SL1[1].m_CtrlPts.push_back((p04 + p03) / 2);
		SL1[1].m_CtrlPts.push_back(p04);

		SL1[2].m_CtrlPts.push_back(p04);
		SL1[2].m_CtrlPts.push_back(p05);
		SL1[2].m_CtrlPts.push_back(p06);

		SL1[3].m_CtrlPts.push_back(p01);
		SL1[3].m_CtrlPts.push_back((p01 + p06) / 2);
		SL1[3].m_CtrlPts.push_back(p06);
		SplineSurface ss1;
		ss1.CoonsInterpolate(SL1);
		SS1.push_back(ss1);

		SL2[0].m_CtrlPts.push_back(p03);
		SL2[0].m_CtrlPts.push_back((p03 + p07) / 2);
		SL2[0].m_CtrlPts.push_back(p07);

		SL2[1].m_CtrlPts.push_back(p07);
		SL2[1].m_CtrlPts.push_back((p12 + p07) / 2);
		SL2[1].m_CtrlPts.push_back(p12);

		SL2[2].m_CtrlPts.push_back(p04);
		SL2[2].m_CtrlPts.push_back((p04 + p12) / 2);
		SL2[2].m_CtrlPts.push_back(p12);

		SL2[3].m_CtrlPts.push_back(p03);
		SL2[3].m_CtrlPts.push_back((p03 + p04) / 2);
		SL2[3].m_CtrlPts.push_back(p04);
		SplineSurface ss2;
		ss2.CoonsInterpolate(SL2);
		SS1.push_back(ss2);

		SL3[0].m_CtrlPts.push_back(p07);
		SL3[0].m_CtrlPts.push_back(p08);
		SL3[0].m_CtrlPts.push_back(p09);

		SL3[1].m_CtrlPts.push_back(p09);
		SL3[1].m_CtrlPts.push_back((p09 + p10) / 2);
		SL3[1].m_CtrlPts.push_back(p10);

		SL3[2].m_CtrlPts.push_back(p12);
		SL3[2].m_CtrlPts.push_back(p11);
		SL3[2].m_CtrlPts.push_back(p10);

		SL3[3].m_CtrlPts.push_back(p07);
		SL3[3].m_CtrlPts.push_back((p07 + p12) / 2);
		SL3[3].m_CtrlPts.push_back(p12);
		SplineSurface ss3;
		ss3.CoonsInterpolate(SL3);
		SS1.push_back(ss3);

		SL4[0].m_CtrlPts.push_back(p10);
		SL4[0].m_CtrlPts.push_back((p10 + p09) / 2);
		SL4[0].m_CtrlPts.push_back(p09);

		SL4[1].m_CtrlPts.push_back(p09);
		SL4[1].m_CtrlPts.push_back((p09 + p16) / 2);
		SL4[1].m_CtrlPts.push_back(p16);

		SL4[2].m_CtrlPts.push_back(p13);
		SL4[2].m_CtrlPts.push_back((p13 + p16) / 2);
		SL4[2].m_CtrlPts.push_back(p16);

		SL4[3].m_CtrlPts.push_back(p10);
		SL4[3].m_CtrlPts.push_back((p10 + p13) / 2);
		SL4[3].m_CtrlPts.push_back(p13);
		SplineSurface ss4;
		ss4.CoonsInterpolate(SL4);
		SS1.push_back(ss4);

		SL5[0].m_CtrlPts.push_back(p13);
		SL5[0].m_CtrlPts.push_back((p13 + p16) / 2);
		SL5[0].m_CtrlPts.push_back(p16);

		SL5[1].m_CtrlPts.push_back(p16);
		SL5[1].m_CtrlPts.push_back(p17);
		SL5[1].m_CtrlPts.push_back(p18);

		SL5[2].m_CtrlPts.push_back(p15);
		SL5[2].m_CtrlPts.push_back((p15 + p18) / 2);
		SL5[2].m_CtrlPts.push_back(p18);

		SL5[3].m_CtrlPts.push_back(p13);
		SL5[3].m_CtrlPts.push_back(p14);
		SL5[3].m_CtrlPts.push_back(p15);
		SplineSurface ss5;
		ss5.CoonsInterpolate(SL5);
		SS1.push_back(ss5);

		SL6[0].m_CtrlPts.push_back(p22);
		SL6[0].m_CtrlPts.push_back((p22 + p15) / 2);
		SL6[0].m_CtrlPts.push_back(p15);

		SL6[1].m_CtrlPts.push_back(p15);
		SL6[1].m_CtrlPts.push_back((p15 + p18) / 2);
		SL6[1].m_CtrlPts.push_back(p18);

		SL6[2].m_CtrlPts.push_back(p19);
		SL6[2].m_CtrlPts.push_back((p19 + p18) / 2);
		SL6[2].m_CtrlPts.push_back(p18);

		SL6[3].m_CtrlPts.push_back(p22);
		SL6[3].m_CtrlPts.push_back((p22 + p19) / 2);
		SL6[3].m_CtrlPts.push_back(p19);
		SplineSurface ss6;
		ss6.CoonsInterpolate(SL6);
		SS1.push_back(ss6);

		SL7[0].m_CtrlPts.push_back(p21);
		SL7[0].m_CtrlPts.push_back((p21 + p24) / 2);
		SL7[0].m_CtrlPts.push_back(p24);

		SL7[1].m_CtrlPts.push_back(p24);
		SL7[1].m_CtrlPts.push_back(p23);
		SL7[1].m_CtrlPts.push_back(p22);

		SL7[2].m_CtrlPts.push_back(p19);
		SL7[2].m_CtrlPts.push_back((p19 + p22) / 2);
		SL7[2].m_CtrlPts.push_back(p22);

		SL7[3].m_CtrlPts.push_back(p21);
		SL7[3].m_CtrlPts.push_back(p20);
		SL7[3].m_CtrlPts.push_back(p19);
		SplineSurface ss7;
		ss7.CoonsInterpolate(SL7);
		SS1.push_back(ss7);

		SL8[0].m_CtrlPts.push_back(p01);
		SL8[0].m_CtrlPts.push_back((p01 + p06) / 2);
		SL8[0].m_CtrlPts.push_back(p06);

		SL8[1].m_CtrlPts.push_back(p06);
		SL8[1].m_CtrlPts.push_back((p06 + p24) / 2);
		SL8[1].m_CtrlPts.push_back(p24);

		SL8[2].m_CtrlPts.push_back(p21);
		SL8[2].m_CtrlPts.push_back((p21 + p24) / 2);
		SL8[2].m_CtrlPts.push_back(p24);

		SL8[3].m_CtrlPts.push_back(p01);
		SL8[3].m_CtrlPts.push_back((p01 + p21) / 2);
		SL8[3].m_CtrlPts.push_back(p21);
		SplineSurface ss8;
		ss8.CoonsInterpolate(SL8);
		SS1.push_back(ss8);
		return SS1;
	}

	//面片
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

	//体
	varray<SplineVolume> get_vol(double r,double h, double a, double b, double c, double d) {
		varray<SplineVolume> SV;
		varray<SplineSurface> SS;
		varray<SplineSurface> SS1;
		SS1 = rec_circle(r, a, b, c, d);
		for (int i = 0; i < SS1.size(); i++) {
			SS.push_back(SS1[i]);
		}
		SplineSurface s1;
		s1 = rec_circle1(r, d);
		m.Trans(s1, c / 2 - r, -1);
		SplineSurface s2;
		s2 = rec_circle2(r, d);
		m.Trans(s2, c / 2 - r, 1);
		SS.push_back(s1);
		SS.push_back(s2);
		SV = m.CreatSweepVol(SS, h, 3);
		return SV;
	}

	varray<SplineVolume> get_vol1(double r, double h, double a, double b, double c, double d) {
		varray<SplineVolume> SV;
		varray<SplineSurface> SS;
		SS = rec_circle(r, a, b, c, d);
		SV = m.CreatSweepVol(SS, h, 3);
		return SV;
	}
private:
	double w = cos(PI / 4);
	Model_Solution m;
};

//车辆零件
class Car_part {
public:
	double H_l1 = 180;	//零件总长
	double H_l2 = 60;	//两边边翼长
	double H_l3 = 40;	//中间正方体边长
	double H_l4 = 15;	//中间挖空圆柱半径
	double H_l5 = 30;	//中间正方体厚度
	double H_l6 = 92;	//零件宽度
	double H_r1 = 1;	//倒角半径
	double H_r2 = 3;	//凸台圆孔半径
	double H_r3 = 5;	//凸台平滑圆弧半径
	double H_t = 3;		//中间方孔壁厚
private:
	//中层参数：
	double M_l1 = (H_l6 - H_l3) / 2;//凸台宽度
	double M_l2 = 11;				//H_l5 / 3;//凸台厚
	double M_l3 = (H_l1 - H_l3 ) / 2- H_l2;//两侧圆柱长
	double M_r1 = H_l5 / 8;			//两侧圆柱半径
	double M_alph = atan(H_l2 / (H_l3 - H_l5)); //角度，蓝色圆圈
	double M_l4 = (H_l2 - 3 * H_t) / 4;			//最小方孔长
	double M_l5 = (H_l2 - 3 * H_t) / 3;			//中间方孔长
	double M_l6 = 5 * (H_l2 - 3 * H_t) / 12;	//最大方孔长
	double M_l7 = 1.3*H_l3;			//凸台长度

	double H_l14 = 6;//放样体的高度

public:	
	//放样的那一部分
	//简单放样函数
	/*v:存放体的控制点*/
	SplineVolume loft(varray<Vec4> &v) {
		SplineVolume SV;
		SV.m_uNum = 3;//控制点个数
		SV.m_vNum = 3;
		SV.m_wNum = 3;
		SV.m_uDegree = 2;//次数
		SV.m_vDegree = 2;
		SV.m_wDegree = 2;
		varray<double> knots;//节点矢量
		knots.push_back(0);
		knots.push_back(0);
		knots.push_back(0);
		knots.push_back(1);
		knots.push_back(1);
		knots.push_back(1);
		SV.m_uKnots = knots;
		SV.m_vKnots = knots;
		SV.m_wKnots = knots;
		//控制点
		/*将面片的控制点放到体容器中*/
		for (int i = 0;i < v.size();i++) {
			SV.m_CtrlPts.push_back(v[i]);
		}

		return SV;

	}
	
	//放样部分的体
	varray<SplineVolume> part3() {
		varray<SplineVolume> SV;
		SV.resize(9);
		varray<Vec4> v1;
		varray<Vec4> v2;
		varray<Vec4> v3;
		varray<Vec4> v4;
		varray<Vec4> v5;
		varray<Vec4> v6;
		varray<Vec4> v7;
		varray<Vec4> v8;
		varray<Vec4> v9;
		//控制点
		//s1
		double w = cos(PI / 4);
		Vec4 p01 = { -6,9.5,-9.5,1 };
		Vec4 p02 = { -6,9.5,0,1 };
		Vec4 p03 = { -6,9.5,9.5,1 };

		Vec4 p07 = { -6,sqrt(2)*7.5 / 4,-sqrt(2)*7.5 / 4,1 };
		Vec4 p08 = { -6,sqrt(2)*7.5 / 2,0,w };
		Vec4 p09 = { -6,sqrt(2)*7.5 / 4,sqrt(2)*7.5 / 4,1 };

		Vec4 p04 = (p01 + p07) / 2;
		Vec4 p05 = (p02 + p08) / 2;
		p05.w = 1;
		Vec4 p06 = (p03 + p09) / 2;
		//s2
		Vec4 p10 = { 0,3 - 1.5,-9.5,1 };
		Vec4 p11 = { 0,3 - 1.5,0,1 };
		Vec4 p12 = { 0,3 - 1.5,9.5,1 };

		Vec4 p16 = { 0,cos(atan(9.5 / 3)) * 2 - 1.5,-sin(atan(9.5 / 3)) * 2,1 };
		Vec4 p17 = { 0,2.4 - 1.5,0,w };
		Vec4 p18 = { 0,cos(atan(9.5 / 3)) * 2 - 1.5,sin(atan(9.5 / 3)) * 2,1 };

		Vec4 p13 = (p10 + p16) / 2;
		Vec4 p14 = (p11 + p17) / 2;
		p14.w = 1;
		Vec4 p15 = (p12 + p18) / 2;

		//s3
		Vec4 p19 = (p01 + p10) / 2;
		Vec4 p20 = (p02 + p11) / 2;
		Vec4 p21 = (p03 + p12) / 2;
		Vec4 p22 = (p04 + p13) / 2;
		Vec4 p23 = (p05 + p14) / 2;
		Vec4 p24 = (p06 + p15) / 2;
		Vec4 p25 = (p07 + p16) / 2;
		Vec4 p26 = (p08 + p17) / 2;
		Vec4 p27 = (p09 + p18) / 2;

		v1.push_back(p01);
		v1.push_back(p02);
		v1.push_back(p03);
		v1.push_back(p04);
		v1.push_back(p05);
		v1.push_back(p06);
		v1.push_back(p07);
		v1.push_back(p08);
		v1.push_back(p09);
		v1.push_back(p19);
		v1.push_back(p20);
		v1.push_back(p21);
		v1.push_back(p22);
		v1.push_back(p23);
		v1.push_back(p24);
		v1.push_back(p25);
		v1.push_back(p26);
		v1.push_back(p27);
		v1.push_back(p10);
		v1.push_back(p11);
		v1.push_back(p12);
		v1.push_back(p13);
		v1.push_back(p14);
		v1.push_back(p15);
		v1.push_back(p16);
		v1.push_back(p17);
		v1.push_back(p18);

		Vec4 p28 = { -6,0,sqrt(2)*7.5 / 2,w };
		Vec4 p30 = { -6,0,9.5,1 };
		Vec4 p29 = (p28 + p30) / 2;
		p29.w = 1;

		Vec4 p31 = { -6,-sqrt(2)*7.5 / 4,sqrt(2)*7.5 / 4,1 };
		Vec4 p33 = { -6,-9.5,9.5,1 };
		Vec4 p32 = (p33 + p31) / 2;

		Vec4 p34 = { 0,0 - 1.5,2.4,w };
		Vec4 p36 = { 0,0 - 1.5,9.5,1 };
		Vec4 p35 = (p34 + p36) / 2;
		p35.w = 1;

		Vec4 p37 = { 0,-cos(atan(9.5 / 3)) * 2 - 1.5,sin(atan(9.5 / 3)) * 2,1 };
		Vec4 p39 = { 0,-3 - 1.5,9.5,1 };
		Vec4 p38 = (p37 + p39) / 2;

		Vec4 p40 = (p09 + p18) / 2;
		Vec4 p41 = (p06 + p15) / 2;
		Vec4 p42 = (p03 + p12) / 2;
		Vec4 p43 = (p28 + p34) / 2;
		Vec4 p44 = (p29 + p35) / 2;
		Vec4 p45 = (p30 + p36) / 2;
		Vec4 p46 = (p31 + p37) / 2;
		Vec4 p47 = (p32 + p38) / 2;
		Vec4 p48 = (p33 + p39) / 2;

		v2.push_back(p09);
		v2.push_back(p06);
		v2.push_back(p03);
		v2.push_back(p28);
		v2.push_back(p29);
		v2.push_back(p30);
		v2.push_back(p31);
		v2.push_back(p32);
		v2.push_back(p33);
		v2.push_back(p40);
		v2.push_back(p41);
		v2.push_back(p42);
		v2.push_back(p43);
		v2.push_back(p44);
		v2.push_back(p45);
		v2.push_back(p46);
		v2.push_back(p47);
		v2.push_back(p48);
		v2.push_back(p18);
		v2.push_back(p15);
		v2.push_back(p12);
		v2.push_back(p34);
		v2.push_back(p35);
		v2.push_back(p36);
		v2.push_back(p37);
		v2.push_back(p38);
		v2.push_back(p39);

		Vec4 p49 = { -6,-sqrt(2)*7.5 / 4,-sqrt(2)*7.5 / 4,1 };
		Vec4 p50 = { -6,-sqrt(2)*7.5 / 2,0,w };
		Vec4 p53 = { -6,-9.5,-9.5,1 };
		Vec4 p54 = { -6,-9.5,0,1 };

		Vec4 p51 = (p49 + p53) / 2;
		Vec4 p52 = (p50 + p54) / 2;
		p52.w = 1;
		//s2
		Vec4 p55 = { 0,-cos(atan(9.5 / 3)) * 2 - 1.5,-sin(atan(9.5 / 3)) * 2,1 };
		Vec4 p56 = { 0,-2.4 - 1.5,0,w };
		Vec4 p59 = { 0,-3 - 1.5,-9.5,1 };
		Vec4 p60 = { 0,-3 - 1.5,0,1 };

		Vec4 p57 = (p55 + p59) / 2;
		Vec4 p58 = (p56 + p60) / 2;
		p58.w = 1;

		//s3
		Vec4 p61 = (p49 + p55) / 2;
		Vec4 p62 = (p50 + p56) / 2;
		Vec4 p63 = (p31 + p37) / 2;
		Vec4 p64 = (p57 + p51) / 2;
		Vec4 p65 = (p58 + p52) / 2;
		Vec4 p66 = (p38 + p32) / 2;
		Vec4 p67 = (p59 + p53) / 2;
		Vec4 p68 = (p60 + p54) / 2;
		Vec4 p69 = (p39 + p33) / 2;

		v3.push_back(p49);
		v3.push_back(p50);
		v3.push_back(p31);
		v3.push_back(p51);
		v3.push_back(p52);
		v3.push_back(p32);
		v3.push_back(p53);
		v3.push_back(p54);
		v3.push_back(p33);
		v3.push_back(p61);
		v3.push_back(p62);
		v3.push_back(p63);
		v3.push_back(p64);
		v3.push_back(p65);
		v3.push_back(p66);
		v3.push_back(p67);
		v3.push_back(p68);
		v3.push_back(p69);
		v3.push_back(p55);
		v3.push_back(p56);
		v3.push_back(p37);
		v3.push_back(p57);
		v3.push_back(p58);
		v3.push_back(p38);
		v3.push_back(p59);
		v3.push_back(p60);
		v3.push_back(p39);

		Vec4 p70 = { -6,0,-9.5,1 };
		Vec4 p72 = { -6,0,-sqrt(2)*7.5 / 2,w };
		Vec4 p71 = (p70 + p72) / 2;
		p71.w = 1;

		Vec4 p73 = { 0,0 - 1.5,-9.5,1 };
		Vec4 p75 = { 0,0 - 1.5,-2.4,w };
		Vec4 p74 = (p73 + p75) / 2;
		p74.w = 1;
		Vec4 p76 = (p70 + p73) / 2;
		Vec4 p77 = (p71 + p74) / 2;
		Vec4 p78 = (p72 + p75) / 2;

		v4.push_back(p01);
		v4.push_back(p04);
		v4.push_back(p07);
		v4.push_back(p70);
		v4.push_back(p71);
		v4.push_back(p72);
		v4.push_back(p53);
		v4.push_back(p51);
		v4.push_back(p49);

		v4.push_back(p19);
		v4.push_back(p22);
		v4.push_back(p25);
		v4.push_back(p76);
		v4.push_back(p77);
		v4.push_back(p78);
		v4.push_back(p67);
		v4.push_back(p64);
		v4.push_back(p61);

		v4.push_back(p10);
		v4.push_back(p13);
		v4.push_back(p16);
		v4.push_back(p73);
		v4.push_back(p74);
		v4.push_back(p75);
		v4.push_back(p59);
		v4.push_back(p57);
		v4.push_back(p55);

		//圆柱部分放样
		//中间正方形
		Vec4 c01 = { -6,2,-2,1 };
		Vec4 c02 = { -6,2,2,1 };
		Vec4 c03 = { -6,-2,2,1 };
		Vec4 c04 = { -6,-2,-2,1 };

		Vec4 c05 = (c01 + c02) / 2;
		Vec4 c06 = (c01 + c04) / 2;
		Vec4 c09 = (c03 + c04) / 2;
		Vec4 c07 = (c05 + c09) / 2;
		Vec4 c08 = (c03 + c02) / 2;

		Vec4 c10 = { 0,0.5 - 1.5,-1,1 };
		Vec4 c11 = { 0,0.5 - 1.5,1,1 };
		Vec4 c12 = { 0,-0.5 - 1.5,1,1 };
		Vec4 c13 = { 0,-0.5 - 1.5,-1,1 };

		Vec4 c14 = (c10 + c11) / 2;
		Vec4 c15 = (c10 + c13) / 2;
		Vec4 c18 = (c12 + c13) / 2;
		Vec4 c16 = (c14 + c18) / 2;
		Vec4 c17 = (c11 + c12) / 2;

		Vec4 c19 = (c01 + c10) / 2;
		Vec4 c20 = (c05 + c14) / 2;
		Vec4 c21 = (c02 + c11) / 2;
		Vec4 c22 = (c06 + c15) / 2;
		Vec4 c23 = (c07 + c16) / 2;
		Vec4 c24 = (c08 + c17) / 2;
		Vec4 c25 = (c04 + c13) / 2;
		Vec4 c26 = (c09 + c18) / 2;
		Vec4 c27 = (c03 + c12) / 2;

		v5.push_back(c01);
		v5.push_back(c05);
		v5.push_back(c02);
		v5.push_back(c06);
		v5.push_back(c07);
		v5.push_back(c08);
		v5.push_back(c04);
		v5.push_back(c09);
		v5.push_back(c03);
		v5.push_back(c19);
		v5.push_back(c20);
		v5.push_back(c21);
		v5.push_back(c22);
		v5.push_back(c23);
		v5.push_back(c24);
		v5.push_back(c25);
		v5.push_back(c26);
		v5.push_back(c27);
		v5.push_back(c10);
		v5.push_back(c14);
		v5.push_back(c11);
		v5.push_back(c15);
		v5.push_back(c16);
		v5.push_back(c17);
		v5.push_back(c13);
		v5.push_back(c18);
		v5.push_back(c12);
		//大圆面
		Vec4 c28 = (p07 + c01) / 2;
		Vec4 c29 = (p08 + c05) / 2;
		c29.w = 1;
		Vec4 c30 = (p09 + c02) / 2;
		Vec4 c31 = (p28 + c08) / 2;
		c31.w = 1;
		Vec4 c32 = (p31 + c03) / 2;
		Vec4 c33 = (p50 + c09) / 2;
		c33.w = 1;
		Vec4 c34 = (p49 + c04) / 2;
		Vec4 c35 = (p72 + c06) / 2;
		c35.w = 1;
		//小圆面
		Vec4 c72 = (p16 + c10) / 2;
		Vec4 c73 = (p17 + c14) / 2;
		c73.w = 1;
		Vec4 c74 = (p18 + c11) / 2;
		Vec4 c75 = (p34 + c17) / 2;
		c75.w = 1;
		Vec4 c76 = (p37 + c12) / 2;
		Vec4 c77 = (p56 + c18) / 2;
		c77.w = 1;
		Vec4 c78 = (p55 + c13) / 2;
		Vec4 c79 = (p75 + c15) / 2;
		c79.w = 1;
		//放样圆柱中间面的点
		Vec4 c36 = (p07 + p16) / 2;
		Vec4 c37 = (p08 + p17) / 2;
		Vec4 c38 = (p09 + p18) / 2;
		Vec4 c39 = (c28 + c72) / 2;
		Vec4 c40 = (c29 + c73) / 2;
		Vec4 c41 = (c30 + c74) / 2;
		Vec4 c42 = (c10 + c01) / 2;
		Vec4 c43 = (c14 + c05) / 2;
		Vec4 c44 = (c11 + c02) / 2;

		Vec4 c45 = (c17 + c08) / 2;
		Vec4 c46 = (c75 + c31) / 2;
		Vec4 c47 = (p34 + p28) / 2;
		Vec4 c48 = (c12 + c03) / 2;
		Vec4 c49 = (c76 + c32) / 2;
		Vec4 c50 = (p37 + p31) / 2;

		Vec4 c51 = (c09 + c18) / 2;
		Vec4 c52 = (c33 + c77) / 2;
		Vec4 c53 = (p50 + p56) / 2;

		Vec4 c54 = (c13 + c04) / 2;
		Vec4 c55 = (c78 + c34) / 2;
		Vec4 c56 = (p55 + p49) / 2;

		Vec4 c57 = (c06 + c15) / 2;
		Vec4 c58 = (c35 + c79) / 2;
		Vec4 c59 = (p72 + p75) / 2;

		v6.push_back(p07);
		v6.push_back(p08);
		v6.push_back(p09);

		v6.push_back(c28);
		v6.push_back(c29);
		v6.push_back(c30);
		v6.push_back(c01);
		v6.push_back(c05);
		v6.push_back(c02);
		//中间面点
		v6.push_back(c36);
		v6.push_back(c37);
		v6.push_back(c38);
		v6.push_back(c39);
		v6.push_back(c40);
		v6.push_back(c41);
		v6.push_back(c42);
		v6.push_back(c43);
		v6.push_back(c44);
		//小面点
		v6.push_back(p16);
		v6.push_back(p17);
		v6.push_back(p18);
		v6.push_back(c72);
		v6.push_back(c73);
		v6.push_back(c74);
		v6.push_back(c10);
		v6.push_back(c14);
		v6.push_back(c11);
	
		//大面点
		v7.push_back(c02);
		v7.push_back(c30);
		v7.push_back(p09);
		v7.push_back(c08);
		v7.push_back(c31);
		v7.push_back(p28);
		v7.push_back(c03);
		v7.push_back(c32);
		v7.push_back(p31);

		v7.push_back(c44);
		v7.push_back(c41);
		v7.push_back(c38);
		v7.push_back(c45);
		v7.push_back(c46);
		v7.push_back(c47);
		v7.push_back(c48);
		v7.push_back(c49);
		v7.push_back(c50);

		v7.push_back(c11);
		v7.push_back(c74);
		v7.push_back(p18);

		v7.push_back(c17);
		v7.push_back(c75);
		v7.push_back(p34);
		v7.push_back(c12);
		v7.push_back(c76);
		v7.push_back(p37);
		

		//大面点
		v8.push_back(c04);
		v8.push_back(c09);
		v8.push_back(c03);
		v8.push_back(c34);
		v8.push_back(c33);
		v8.push_back(c32);
		v8.push_back(p49);
		v8.push_back(p50);
		v8.push_back(p31);
		//中间面点
		v8.push_back(c54);
		v8.push_back(c51);
		v8.push_back(c48);
		v8.push_back(c55);
		v8.push_back(c52);
		v8.push_back(c49);
		v8.push_back(c56);
		v8.push_back(c53);
		v8.push_back(c50);
		//小面点
		v8.push_back(c13);
		v8.push_back(c18);
		v8.push_back(c12);
		v8.push_back(c78);
		v8.push_back(c77);
		v8.push_back(c76);
		v8.push_back(p55);
		v8.push_back(p56);
		v8.push_back(p37);

		//大面点
		v9.push_back(p07);
		v9.push_back(c28);
		v9[1].w = 1;
		v9.push_back(c01);
		v9.push_back(p72);
		v9.push_back(c35);
		v9.push_back(c06);
		v9.push_back(p49);
		v9.push_back(c34);
		v9.push_back(c04);
		//中间面点
		v9.push_back(c36);
		v9.push_back(c39);
		v9.push_back(c42);
		v9.push_back(c59);
		v9.push_back(c58);
		v9.push_back(c57);
		v9.push_back(c56);
		v9.push_back(c55);
		v9.push_back(c54);
		//小面点
		v9.push_back(p16);
		v9.push_back(c72);
		v9.push_back(c10);
		v9.push_back(p75);
		v9.push_back(c79);
		v9.push_back(c15);
		v9.push_back(p55);
		v9.push_back(c78);
		v9.push_back(c13);

		varray<varray<Vec4>> v;
		v.push_back(v1);
		v.push_back(v2);
		v.push_back(v3);
		v.push_back(v4);
		v.push_back(v5);
		v.push_back(v6);
		v.push_back(v7);
		v.push_back(v8);
		v.push_back(v9);
		for (int i = 0;i < 9;i++) {
			SV[i] = loft(v[i]);
		}

		/*RWGeometric rwg;
		rwg.WriteSplineVolume("E:\\kuang_models\\part3.txt", SV);*/
		return SV;
	}

	//圆柱
	varray<SplineVolume> part2(double r,double l,double h) {
		Cylinder_rec cr;
		varray<SplineVolume> SV;
		SV = cr.get_vol(r,l,h);
		return SV;
	}

	//车辆零件中的不规则体
	varray<SplineVolume> Temp1() {
		varray<SplineSurface> SS;
		varray<Spline> SL1;
		varray<Spline> SL2;
		varray<Spline> SL3;
		varray<Spline> SL4;
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
		for (int i = 0; i < 4; i++) {
			SL1[i].m_Degree = 2;
			SL1[i].m_Knots = knots;
			SL2[i].m_Degree = 2;
			SL2[i].m_Knots = knots;
			SL3[i].m_Degree = 2;
			SL3[i].m_Knots = knots;
			SL4[i].m_Degree = 2;
			SL4[i].m_Knots = knots;
		}
		Vec4 p01 = { -H_l3 / 2,-H_r3,0,1 };
		Vec4 p02 = { -H_l3 / 2 + H_r3,-H_r3,0,w };
		Vec4 p03 = { -H_l3 / 2 + H_r3,0,0,1 };
		Vec4 p04 = { -H_l3 / 2+(M_l1-3*H_r3),0,0,1 };
		Vec4 p05 = { -H_l3 / 2 + H_r3,-2*H_r3,0,1 };

		Vec4 p06 = { -H_l3 / 2 + H_r3,0,0,1 };
		Vec4 p07 = { -H_l3 / 2 + H_r3,H_r3,0,w };
		Vec4 p08 = { -H_l3 / 2,H_r3,0,1 };
		Vec4 p09 = { -H_l3 / 2 + H_r3,2 * H_r3,0,1 };
		Vec4 p10 = { -H_l3 / 2 + (M_l1 - 3 * H_r3),0,0,1 };

		Vec4 p11 = { H_l3 / 2 - (M_l1 - 3 * H_r3),0,0,1 };
		Vec4 p12 = { H_l3 / 2 - H_r3,2 * H_r3,0,1 };
		Vec4 p13 = { H_l3 / 2,H_r3,0,1 };
		Vec4 p14 = { H_l3 / 2 - H_r3,H_r3,0,w };
		Vec4 p15 = { H_l3 / 2 - H_r3,0,0,1 };

		Vec4 p16 = { H_l3 / 2 - H_r3,-2 * H_r3,0,1 };
		Vec4 p17 = { H_l3 / 2 - (M_l1 - 3 * H_r3),0,0,1 };
		Vec4 p18 = { H_l3 / 2 - H_r3,0,0,1 };
		Vec4 p19 = { H_l3 / 2 - H_r3,-H_r3,0,w };
		Vec4 p20 = { H_l3 / 2,-H_r3,0,1 };

		SL1[0].m_CtrlPts.push_back(p01);
		SL1[0].m_CtrlPts.push_back((p01 + p05) / 2);
		SL1[0].m_CtrlPts.push_back(p05);

		SL1[1].m_CtrlPts.push_back(p01);
		SL1[1].m_CtrlPts.push_back(p02);
		SL1[1].m_CtrlPts.push_back(p03);

		SL1[2].m_CtrlPts.push_back(p03);
		SL1[2].m_CtrlPts.push_back((p03 + p04) / 2);
		SL1[2].m_CtrlPts.push_back(p04);

		SL1[3].m_CtrlPts.push_back(p05);
		SL1[3].m_CtrlPts.push_back((p04 + p05) / 2);
		SL1[3].m_CtrlPts.push_back(p04);
		SplineSurface ss1;
		ss1.CoonsInterpolate(SL1);
		m.Trans(ss1, H_l3 / 2 + (M_l1 - 3 * H_r3), -2);
		SS.push_back(ss1);

		SL2[0].m_CtrlPts.push_back(p06);
		SL2[0].m_CtrlPts.push_back((p06 + p10) / 2);
		SL2[0].m_CtrlPts.push_back(p10);

		SL2[1].m_CtrlPts.push_back(p06);
		SL2[1].m_CtrlPts.push_back(p07);
		SL2[1].m_CtrlPts.push_back(p08);

		SL2[2].m_CtrlPts.push_back(p08);
		SL2[2].m_CtrlPts.push_back((p08+ p09) / 2);
		SL2[2].m_CtrlPts.push_back(p09);

		SL2[3].m_CtrlPts.push_back(p10);
		SL2[3].m_CtrlPts.push_back((p09 + p10) / 2);
		SL2[3].m_CtrlPts.push_back(p09);
		SplineSurface ss2;
		ss2.CoonsInterpolate(SL2);
		m.Trans(ss2,H_l3 / 2 + (M_l1 - 3 * H_r3), 2);
		SS.push_back(ss2);
		
		SL3[0].m_CtrlPts.push_back(p11);
		SL3[0].m_CtrlPts.push_back((p11 + p15) / 2);
		SL3[0].m_CtrlPts.push_back(p15);

		SL3[1].m_CtrlPts.push_back(p11);
		SL3[1].m_CtrlPts.push_back((p11 + p12) / 2);
		SL3[1].m_CtrlPts.push_back(p12);

		SL3[2].m_CtrlPts.push_back(p12);
		SL3[2].m_CtrlPts.push_back((p13 + p12) / 2);
		SL3[2].m_CtrlPts.push_back(p13);

		SL3[3].m_CtrlPts.push_back(p15);
		SL3[3].m_CtrlPts.push_back(p14);
		SL3[3].m_CtrlPts.push_back(p13);
		SplineSurface ss3;
		ss3.CoonsInterpolate(SL3);
		m.Trans(ss3, H_l3 / 2 + (M_l1 - 3 * H_r3), 2);
		SS.push_back(ss3);

		SL4[0].m_CtrlPts.push_back(p16);
		SL4[0].m_CtrlPts.push_back((p20 + p16) / 2);
		SL4[0].m_CtrlPts.push_back(p20);

		SL4[1].m_CtrlPts.push_back(p16);
		SL4[1].m_CtrlPts.push_back((p17 + p16) / 2);
		SL4[1].m_CtrlPts.push_back(p17);

		SL4[2].m_CtrlPts.push_back(p17);
		SL4[2].m_CtrlPts.push_back((p17 + p18) / 2);
		SL4[2].m_CtrlPts.push_back(p18);

		SL4[3].m_CtrlPts.push_back(p20);
		SL4[3].m_CtrlPts.push_back(p19);
		SL4[3].m_CtrlPts.push_back(p18);
		SplineSurface ss4;
		ss4.CoonsInterpolate(SL4);
		m.Trans(ss4, H_l3 / 2 + (M_l1 - 3 * H_r3), -2);
		SS.push_back(ss4);

		return m.CreatSweepVol(SS, M_l2, 3);
	}

	//车辆零件整体
	void part1() {
		Model_Solution m;
		//正方体-内空圆柱
		Cube_Cylinder cc;
		/*
			以下是存放车辆零件中间部分的体容器
		*/
		varray<SplineVolume> SV;//存放所有的体
		varray<SplineVolume> SV1;//存放底层正方体-内圆柱
		varray<SplineVolume> SV2;//存放上层正方体-内圆柱
		varray<SplineVolume> SV3;//存放梯形
		varray<SplineVolume> SV4;//存放梯形
		//存放四分之一正方体-内圆柱
		varray<SplineVolume> SV5;//下方---左
		varray<SplineVolume> SV6;//下方---右
		varray<SplineVolume> SV7;//上方---左
		varray<SplineVolume> SV8;//上方---右

		varray<SplineVolume> SV9; //最下方---左---上
		varray<SplineVolume> SV10;//最下方的---左---下
		varray<SplineVolume> SV11; //最上方的---左---上
		varray<SplineVolume> SV12;//最上方的---左---下
		varray<SplineVolume> SV13; //最下方的---右---上
		varray<SplineVolume> SV14;//最下方的---右---下
		varray<SplineVolume> SV15; //最上方的---右---上
		varray<SplineVolume> SV16;//最上方的---右---下

		varray<SplineVolume> SV17;//存放梯形---最下方两个
		varray<SplineVolume> SV18;//存放梯形
		varray<SplineVolume> SV19;//存放梯形---最下方两个
		varray<SplineVolume> SV20;//存放梯形

		SV1 = cc.get_vol(H_l3, H_l4, M_l2);
		SV2 = cc.get_vol(H_l3, H_l4, H_l5-M_l2);
		m.Trans(SV2, M_l2, 3);
		//等腰梯形
		Cube_slice cs;
		//下方第一层梯形
		SV3 = cs.get_vol3(H_l3, H_l3-2*(M_l1-3*H_r3), M_l1-3*H_r3, M_l2);
		m.Trans(SV3, H_l3/2+(M_l1-3*H_r3), -2);

		//上方第一层
		SV4 = cs.get_vol3(H_l3 - 2 * (M_l1 - 3 * H_r3), H_l3, M_l1 - 3 * H_r3, M_l2);
		m.Trans(SV4, H_l3 / 2, 2);

		//下方第二层梯形
		SV17 = cs.get_vol3(H_l3 - 2 * (M_l1 - 3 * H_r3), H_l3 -2* H_r3, 2 * H_r3, M_l2);
		m.Trans(SV17, H_l3 / 2 + M_l1-H_r3, -2);

		//下方第三层梯形
		SV18 = cs.get_vol3(H_l3 - 2 * H_r3, H_l3, H_r3, M_l2);
		m.Trans(SV18, H_l3 / 2 + M_l1, -2);

		//上方第二层
		SV19 = cs.get_vol3(H_l3 - 2 * H_r3, H_l3 - 2 * (M_l1 - 3 * H_r3), 2 * H_r3, M_l2);
		m.Trans(SV19, H_l3 / 2 + (M_l1 - 3 * H_r3), 2);

		//上方第三层
		SV20 = cs.get_vol3(H_l3, H_l3 - 2 * H_r3, H_r3, M_l2);
		m.Trans(SV20, H_l3 / 2 + M_l1-H_r3, 2);

		//下方四分之一正方体-内空圆柱-左
		SV5 = cc.get_vol1(sqrt(2)*(M_l1 - 3 * H_r3), H_r3, M_l2);
		m.Rolate(SV5, -PI / 4, 3);
		m.Trans(SV5, H_l3 / 2 + M_l1-3*H_r3, -2);
		m.Trans(SV5, H_l3/2, -1);

		//下方四分之一正方体-内空圆柱-右
		SV6 = cc.get_vol1(sqrt(2)*(M_l1 - 3 * H_r3), H_r3, M_l2);
		m.Rolate(SV6, PI / 4, 3);
		m.Trans(SV6, H_l3 / 2 + M_l1 - 3 * H_r3, -2);
		m.Trans(SV6, H_l3 / 2, 1);

		//上方四分之一正方体-内空圆柱-左
		SV7 = cc.get_vol2(sqrt(2)*(M_l1 - 3 * H_r3), H_r3, M_l2);
		m.Rolate(SV7, PI / 4, 3);
		m.Trans(SV7, H_l3 / 2 + M_l1 - 3 * H_r3, 2);
		m.Trans(SV7, H_l3 / 2, -1);

		//上方四分之一正方体-内空圆柱-右
		SV8 = cc.get_vol2(sqrt(2)*(M_l1 - 3 * H_r3), H_r3, M_l2);
		m.Rolate(SV8, -PI / 4, 3);
		m.Trans(SV8, H_l3 / 2 + M_l1 - 3 * H_r3, 2);
		m.Trans(SV8, H_l3 / 2, 1);

		//最下方---左---上
		SV9 = cc.get_vol1(sqrt(2) * H_r3, H_r2, M_l2);
		m.Rolate(SV9, -PI / 4, 3);
		m.Trans(SV9, H_l3 / 2 + M_l1-H_r3, -2);
		m.Trans(SV9, H_l3 / 2, -1);

		//最下方---左---下
		SV10 = cc.get_vol2(sqrt(2) * H_r3, H_r2, M_l2);
		m.Rolate(SV10, PI / 4, 3);
		m.Trans(SV10, H_l3 / 2 + M_l1 - H_r3, -2);
		m.Trans(SV10, H_l3 / 2, -1);

		//最上方---左---下
		SV11 = cc.get_vol2(sqrt(2) * H_r3, H_r2, M_l2);
		m.Rolate(SV11, PI / 4, 3);
		m.Trans(SV11, H_l3 / 2 + M_l1 - H_r3, 2);
		m.Trans(SV11, H_l3 / 2, -1);

		//最上方---左---上
		SV12 = cc.get_vol1(sqrt(2) * H_r3, H_r2, M_l2);
		m.Rolate(SV12, -PI / 4, 3);
		m.Trans(SV12, H_l3 / 2 + M_l1 - H_r3, 2);
		m.Trans(SV12, H_l3 / 2, -1);

		//最下方---右---下
		SV13 = cc.get_vol2(sqrt(2) * H_r3, H_r2, M_l2);
		m.Rolate(SV13, -PI / 4, 3);
		m.Trans(SV13, H_l3 / 2 + M_l1 - H_r3, -2);
		m.Trans(SV13, H_l3 / 2, 1);

		//最下方---右---上
		SV14 = cc.get_vol1(sqrt(2) * H_r3, H_r2, M_l2);
		m.Rolate(SV14, PI / 4, 3);
		m.Trans(SV14, H_l3 / 2 + M_l1 - H_r3, -2);
		m.Trans(SV14, H_l3 / 2, 1);

		//最上方---右---下
		SV15 = cc.get_vol2(sqrt(2) * H_r3, H_r2, M_l2);
		m.Rolate(SV15, -PI / 4, 3);
		m.Trans(SV15, H_l3 / 2 + M_l1 - H_r3, 2);
		m.Trans(SV15, H_l3 / 2, 1);

		//最上方---右---上
		SV16 = cc.get_vol1(sqrt(2) * H_r3, H_r2, M_l2);
		m.Rolate(SV16, PI / 4, 3);
		m.Trans(SV16, H_l3 / 2 + M_l1 - H_r3, 2);
		m.Trans(SV16, H_l3 / 2, 1);

		//不规则零件
		varray<SplineVolume> sv;
		sv = Temp1();

		//左右两边半圆筒
		Circle c;
		varray<SplineVolume> sv1;
		varray<SplineVolume> sv2;
		sv1 = c.get_vol2(H_r2, H_r3, M_l2);
		sv2 = c.get_vol3(H_r2, H_r3, M_l2);
		m.Trans(sv1, H_l3 / 2 + M_l1 - H_r3, -2);
		m.Trans(sv1, H_l3 / 2, -1);
		m.Trans(sv2, H_l3 / 2 + M_l1 - H_r3, -2);
		m.Trans(sv2, H_l3 / 2, 1);
		varray<SplineVolume> sv3;
		varray<SplineVolume> sv4;
		sv3 = c.get_vol2(H_r2, H_r3, M_l2);
		sv4 = c.get_vol3(H_r2, H_r3, M_l2);
		m.Trans(sv3, H_l3 / 2 + M_l1 - H_r3, 2);
		m.Trans(sv3, H_l3 / 2, -1);
		m.Trans(sv4, H_l3 / 2 + M_l1 - H_r3, 2);
		m.Trans(sv4, H_l3 / 2, 1);

		//两边边翼零件
		RWGeometric rwg;
		varray<SplineVolume> vols;//底层
		varray<SplineVolume> vols1;//上层
		rwg.ReadSplineVolume("E:\\kuang_models\\volume_model1.txt", vols);
		m.Trans(vols, H_l3 / 2, -2);

		//装到输出容器中
		for (int i = 0;i < vols.size();i++) {
			SV.push_back(vols[i]);
		}

		rwg.ReadSplineVolume("E:\\kuang_models\\volume_model1-1.txt", vols1);
		m.Trans(vols1, H_l3 / 2, -2);

		//装到输出容器中
		for (int i = 0;i < vols1.size();i++) {
			SV.push_back(vols1[i]);
		}

		//放样体
		varray<SplineVolume> csv;
		varray<SplineVolume> csv1;
		csv = part3();
		m.Trans(csv, H_l3/2+H_l2-6, -1);
		m.Trans(csv, H_l3 / 2 - 9.5, 2);
		m.Trans(csv, 20.5, 3);//H_l5 * 2 / 3
		csv1 = part3();
		m.Rolate(csv1, PI, 2);
		m.Trans(csv1, H_l3 / 2 + H_l2-6, 1);
		m.Trans(csv1, H_l3 / 2 - 9.5, 2);//H_l3 / 2 - H_l5 / 3
		m.Trans(csv1, 20.5, 3);//H_l5 * 2 / 3

		//两边圆柱
		varray<SplineVolume> csv2;
		varray<SplineVolume> csv3;
		csv2 = part2(M_r1, 4, M_l3);
		m.Rolate(csv2, -PI / 2, 2);
		m.Trans(csv2, H_l3/2+H_l2, -1);
		m.Trans(csv2, H_l3 / 2 - 9.5, 2);//H_l3 / 2 - H_l5 / 3
		m.Trans(csv2, 20.5, 3);//H_l5*2/3
		csv3 = csv2;
		m.Trans(csv3, H_l1-M_l3, 1);

		//装四个体
		for (int i = 0;i < SV1.size();i++) {
			SV.push_back(SV1[i]);//中间底层部分
		}
		for (int i = 0;i < SV1.size();i++) {
			SV.push_back(sv[i]);//不规则体部分
		}

		//装两个体 上下左右两边半圆筒
		for (int i = 0;i < sv1.size();i++) {
			SV.push_back(sv1[i]);
			SV.push_back(sv2[i]);
			SV.push_back(sv3[i]);
			SV.push_back(sv4[i]);
		}

		//装一个体 梯形和四分之一正-圆体
		for (int i = 0;i < SV3.size();i++) {
			SV.push_back(SV3[i]);
			SV.push_back(SV4[i]);
			SV.push_back(SV5[i]);
			SV.push_back(SV6[i]);
			SV.push_back(SV7[i]);
			SV.push_back(SV8[i]);
			SV.push_back(SV9[i]);
			SV.push_back(SV12[i]);
			SV.push_back(SV14[i]);
			SV.push_back(SV16[i]);
			SV.push_back(SV10[i]);
			SV.push_back(SV11[i]);
			SV.push_back(SV13[i]);
			SV.push_back(SV15[i]);
			SV.push_back(SV17[i]);
			SV.push_back(SV18[i]);
			SV.push_back(SV19[i]);
			SV.push_back(SV20[i]);
		}

		//装四个体 中间上层部分
		for (int i = 0;i < SV2.size();i++) {
			SV.push_back(SV2[i]);
		}

		//放样体
		//装九个体
		for (int i = 0;i < csv.size();i++) {
			csv[i].OrderCtrlPts1(csv[i], 4);
			SV.push_back(csv[i]);
			csv1[i].OrderCtrlPts1(csv1[i], 1);
			SV.push_back(csv1[i]);
		}

		//圆柱
		//装五个体
		for (int i = 0;i < csv2.size();i++) {
			csv2[i].OrderCtrlPts1(csv2[i],5);
			SV.push_back(csv2[i]);
		}
		for (int i = 0;i < csv2.size();i++) {
			csv3[i].OrderCtrlPts1(csv3[i], 5);
			SV.push_back(csv3[i]);
		}
		SV[37].OrderCtrlPts(SV[37]);
		SV[96].OrderCtrlPts(SV[96]);
		for (int i = 0;i < 118;i++) {
			SV[i].OrderCtrlPts(SV[i]);
		}
		for (int i = 118;i < 156;i++) {
			SV[i].OrderCtrlPts(SV[i]);
		}
		/*SV[153].OrderCtrlPts1(SV[153], 3);
		SV[153].OrderCtrlPts(SV[153]);*/

		//细化
		for (int i = 0;i < SV.size();++i) {
			SV[i].KnotsRefineNum(3);
		}
		
		//varray<NurbsVol> NV;
		//NV = NurbsTrans::SplinevolsToCvols(SV);
		//CPolyParaVolume cp;       //输出vtk文件的类对象
		//cp = NV;
		///////*cp.SetAdjacentHexIdx();
		/////*cp.Order();*/
		//cp.OutputParaVolumeDataVTK("E:\\kuang_models\\Part1Volumevtk1.vtk");

		TestBolcks::get_Align(SV);//检测控制点是否对齐

		rwg.WriteSplineVolume("E:\\kuang_models\\part1-3.txt", SV);
	}

	//车辆零件施加约束和力
	void setWCandWF0(string path, int wc, int wf)
	{
		RWGeometric rwg;
		varray<SplineVolume> NVS;
		rwg.ReadSplineVolume(path + ".txt", NVS);
		TestBolcks::pList plt;
		Model_Solution M;
		varray<varray<SplineSurface>> NS = M.GetSurfaces(NVS);
		varray<SplineSurface> NSf;
		for (int i = 0; i < NS.size(); i++)
		{
			for (int j = 0; j < NS[i].size(); j++)
			{
				NSf.push_back(NS[i][j]);
			}
		}
		rwg.WriteSplineSurface(path + "排序面.txt", NSf);
		//创建存放施加约束面的容器
		varray<SplineSurface> WC;
		WC.push_back(NSf[wc]);

		//底面全施加约束
		
		for (int i = 0;i < 59;i++) {
			WC.push_back(NSf[wc += 6]);
		}
		int temp1 = 712;
		WC.push_back(NSf[712]);
		for (int i = 0;i < 33;i++) {
			WC.push_back(NSf[temp1 += 6]);
		}

		//两边耳朵施加约束 712开头
		/*for (int i = 0;i < 30;i++) {
			WC.push_back(NSf[wc += 6]);
		}*/

		////耳朵上圆孔施加约束 758开始
		//for (int i = 0;i < 7;i++) {
		//	WC.push_back(NSf[wc += 6]);
		//}
		//int temp1 = 842;
		//WC.push_back(NSf[temp1]);
		//for (int i = 0;i < 3;i++) {
		//	WC.push_back(NSf[temp1 += 6]);
		//}
		//int temp2 = 867;
		//WC.push_back(NSf[temp2]);
		//for (int i = 0;i < 3;i++) {
		//	WC.push_back(NSf[temp2 += 6]);
		//}

		//创建并初始化存放施加约束控制点的容器对象
		varray<varray<int>>WCidx = plt.getfaceidx(NVS, WC);
		//plt.showdata();
		cout << "WC" << " ";
		int num = 0;
		for (int i = 0; i < WCidx.size(); i++)
		{
			for (int j = 0; j < WCidx[i].size(); j++)
			{
				num++;
				cout << WCidx[i][j] << " ";
			}
		}
		cout << endl;

		//创建存放施加力的面片容器
		varray<SplineSurface> WF;
		WF.push_back(NSf[wf]);
		for (int i = 0;i < 4;i++) {
			WF.push_back(NSf[wf += 6]);
		}
		int temp = 1074;
		WF.push_back(NSf[1074]);
		for (int i = 0;i < 4;i++) {
			WF.push_back(NSf[temp += 6]);
		}
		//创建并初始化存放施加力控制点的容器对象
		varray<varray<int>>WFidx = plt.getfaceidx(NVS, WF);
		cout << "WF" << " ";
		for (int i = 0; i < WFidx.size(); i++)
		{
			for (int j = 0; j < WFidx[i].size(); j++)
			{
				cout << WFidx[i][j] << " ";
			}
		}
	}

	void test0()
	{
		RWGeometric rwg;
		varray<SplineVolume> NVs;
		rwg.ReadSplineVolume("E:\\kuang_models\\part1-3.txt", NVs);
		Model_Solution M;
		varray<varray<SplineSurface>> NFs;
		NFs = M.GetSurfaces(NVs);
		varray<SplineSurface> NF;
		for (auto& SF : NFs) {
			for (auto& i : SF) {
				NF.push_back(i);
			}
		}
		//rwg.WriteSplineSurface("E:\\kuang_models\\part1_surfaces.txt", NF);
		TestBolcks::pList plist;
		plist.OutputParaVolumeDataTxt(NVs, "E:\\kuang_models\\part1-3.txt");
		setWCandWF0("E:\\kuang_models\\part1-3", 4, 1045);//全底面----758, 1045
	}

private:
		Model_Solution m;
		double w = cos(PI / 4);
};

/*
	齿轮
*/
//class Gear {
//public:
//	//3X3X3
//	void gear1(int z, int num) {
//		Model_Solution m;
//		Gear_Straight g;
//		varray<SplineVolume> SV1;
//		varray<SplineVolume> SV2;
//		varray<SplineSurface> SS;
//		varray<SplineVolume> SV;
//
//		SV1 = g.getGear1();
//		m.Rolate(SV1, PI / 2, 3);
//		m.Rolate(SV1, -PI / 105, 3);
//		//全齿
//		SV = SV1;
//		SV1.clear();
//		if (z > 1) {
//			m.Rolate(SV, PI / 15, 3);
//		}
//		for (int i = 0;i < z;i++) {
//			for (int i = 0;i < SV.size();i++) {
//				SV1.push_back(SV[i]);
//			}
//			m.Rolate(SV, -PI / 15, 3);
//		}
//
//		SV2 = g.getGear2();
//		SV.clear();
//		SV = SV2;
//		SV2.clear();
//		if (z > 1) {
//			m.Rolate(SV, -PI / 15, 3);
//		}
//		for (int i = 0;i < z;i++) {
//			for (int i = 0;i < SV.size();i++) {
//				SV2.push_back(SV[i]);
//			}
//			m.Rolate(SV, PI / 15, 3);
//		}
//		m.Rolate(SV2, PI / 2, 3);
//		m.Rolate(SV2, PI / 24.9, 3);//这个是3X3X3旋转角度
//		m.Trans(SV2, 60, 2);
//
//		for (int i = 0; i < SV2.size(); i++) {
//			SV1.push_back(SV2[i]);
//		}
//		//细化
//		for (int i = 0; i < SV1.size(); i++) {
//			SV1[i].KnotsRefineNum(num);
//		}
//		stringstream stream;
//		stream << 3 << "-"<< z << "-" << num;
//		string r = stream.str();
//		RWGeometric rwg;
//		rwg.WriteSplineVolume("E:\\kuang_models\\gear" + r + ".txt", SV1);
//
//	}
//	//5X5X5
//	void gear2(int z,int num) {
//		Model_Solution m;
//		Gear_Straight g;
//		varray<SplineVolume> SV1;
//		varray<SplineVolume> SV2;
//		varray<SplineSurface> SS;
//		varray<SplineVolume> SV;
//
//		SV1 = g.getGear4();
//		m.Rolate(SV1, PI / 2, 3);
//		m.Rolate(SV1, -PI / 105, 3);
//		//全齿
//		SV = SV1;
//		SV1.clear();
//		if (z > 1) {
//			m.Rolate(SV, PI / 15, 3);
//		}
//		for (int i = 0;i < z;i++) {
//			for (int i = 0;i < SV.size();i++) {
//				SV1.push_back(SV[i]);
//			}
//			m.Rolate(SV, -PI / 15, 3);
//		}
//
//		SV2 = g.getGear3();
//		SV.clear();
//		SV = SV2;
//		SV2.clear();
//		if (z > 1) {
//			m.Rolate(SV, -PI / 15, 3);
//		}
//		for (int i = 0;i < z;i++) {
//			for (int i = 0;i < SV.size();i++) {
//				SV2.push_back(SV[i]);
//			}
//			m.Rolate(SV, PI / 15, 3);
//		}
//		m.Rolate(SV2, PI / 2, 3);
//		m.Rolate(SV2, PI / 24.3, 3);//这个是5X5X5旋转角度
//		m.Trans(SV2, 60, 2);
//
//		for (int i = 0; i < SV2.size(); i++) {
//			SV1.push_back(SV2[i]);
//		}
//		//细化
//		for (int i = 0; i < SV1.size(); i++) {
//			SV1[i].KnotsRefineNum(num);
//		}
//		stringstream stream;
//		stream << 5 <<"-"<< z<<"-"<<num;
//		string r = stream.str();
//		RWGeometric rwg;
//		rwg.WriteSplineVolume("E:\\kuang_models\\gear" + r + ".txt", SV1);
//
//	}
//	//齿轮
//	/*
//		z:单个齿轮显示的齿数
//		num:细化次数
//		select:若为1，表示控制点为3X3X3
//			   若为2，表示控制点为5X5X5
//	*/
//	void gear(int z,int num,int select) {
//		if (select == 1) {
//			gear1(z,num);
//		}
//		else if (select == 2) {
//			gear2(z, num);
//		}
//		
//	}
//	void setWCandWF(string path, int wc, int wf)
//	{
//		RWGeometric rwg;
//		varray<SplineVolume> NVS;
//		rwg.ReadSplineVolume(path + ".txt", NVS);
//		TestBolcks::pList plt;
//		Model_Solution M;
//		varray<varray<SplineSurface>> NS = M.GetSurfaces(NVS);
//		varray<SplineSurface> NSf;
//		for (int i = 0; i < NS.size(); i++)
//		{
//			for (int j = 0; j < NS[i].size(); j++)
//			{
//				NSf.push_back(NS[i][j]);
//			}
//		}
//		rwg.WriteSplineSurface(path + "排序面.txt", NSf);
//		//创建存放施加约束面的容器
//		varray<SplineSurface> WC;
//		WC.push_back(NSf[wc]);
//		////六齿
//		//int temp = 31;
//		//WC.push_back(NSf[temp]);
//		//for (int i = 0;i < 2;i++) {
//		//	WC.push_back(NSf[wc += 36]);
//		//	WC.push_back(NSf[temp += 36]);
//		//}
//
//		//全齿
//		//wc=25
//		int temp = 31;
//		WC.push_back(NSf[temp]);
//		for (int i = 0;i < 29;i++) {
//			WC.push_back(NSf[wc += 36]);
//			WC.push_back(NSf[temp += 36]);
//		}
//		//创建并初始化存放施加约束控制点的容器对象
//		varray<varray<int>>WCidx = plt.getfaceidx(NVS, WC);
//		//plt.showdata();
//		cout << "WC" << " ";
//		int num = 0;
//		for (int i = 0; i < WCidx.size(); i++)
//		{
//			for (int j = 0; j < WCidx[i].size(); j++)
//			{
//				num++;
//				cout << WCidx[i][j] << " ";
//			}
//		}
//		cout << endl;
//
//		//创建存放施加力的面片容器
//		varray<SplineSurface> WF;
//		WF.push_back(NSf[wf]);
//		WF.push_back(NSf[1118]);
//		//创建并初始化存放施加力控制点的容器对象
//		varray<varray<int>>WFidx = plt.getfaceidx(NVS, WF);
//		cout << "WF" << " ";
//		for (int i = 0; i < WFidx.size(); i++)
//		{
//			for (int j = 0; j < WFidx[i].size(); j++)
//			{
//				cout << WFidx[i][j] << " ";
//			}
//		}
//	}
//	void test() {
//		RWGeometric rwg;
//		varray<SplineVolume> NVs;
//		rwg.ReadSplineVolume("E:\\kuang_models\\gear3-30-0.txt", NVs);
//		Model_Solution M;
//		varray<varray<SplineSurface>> NFs;
//		NFs = M.GetSurfaces(NVs);
//		varray<SplineSurface> NF;
//		for (auto& SF : NFs) {
//			for (auto& i : SF) {
//				NF.push_back(i);
//			}
//		}
//		//rwg.WriteSplineSurface("E:\\kuang_models\\gear_surfaces3.txt", NF);
//		TestBolcks::pList plist;
//		plist.OutputParaVolumeDataTxt(NVs, "E:\\kuang_models\\gear3-30-0.txt");
//		setWCandWF("E:\\kuang_models\\gear3-30-0", 25, 45);
//	}
//};


/****************************************************************************************/
										/*模型函数*/
/****************************************************************************************/



//辜
//void guPart() {
//	double l = 100, h = 150, r = 5;
//	Model_Solution m;
//	double w = cos(PI / 4);
//	//坐标点
//	Vec4 v1 = { 0,0,0,1 };
//	Vec4 v2 = { 0,h,0,1 };
//	Vec4 v3 = { l, h,0,1 };
//	Vec4 v4 = { l,0,0,1 };
//	Vec4 p1 = { 0,h / 2,0,1 };
//	Vec4 p2 = { l/2,h ,0,1 };
//	Vec4 p3 = { l,h / 2,0,1 };
//	Vec4 p4 = { l/2,0,0,1 };
//
//	Vec4 v5 = { l/6-r,h/6,0,1 };
//	Vec4 v6 = { l / 6 - r,h / 6+r,0,w };
//	Vec4 v7 = { l / 6 ,h / 6+r,0,1 };
//	Vec4 v8 = { l / 6 +r,h / 6+r,0,w };
//	Vec4 v9 = { l / 6 + r,h / 6,0,1 };
//	Vec4 v10 = { l / 6 + r,h / 6 - r,0,w };
//	Vec4 v11 = { l / 6 ,h / 6 - r,0,1 };
//	Vec4 v12= { l / 6 - r,h / 6 - r,0,w };
//
//	Vec4 v13 = { l / 2 -r,h / 6,0,1 };
//	Vec4 v14 = { l*5 / 6,h / 6+r,0,1 };
//	Vec4 v15 = { l / 3,h / 2 - r,0,1 };
//	Vec4 v16 = { l*2 / 3,h / 2 - r,0,1 };
//	Vec4 v17 = { l / 3,h / 2 + r,0,1 };
//	Vec4 v18 = { l*2/ 3,h / 2 +r,0,1 };
//	Vec4 v19 = { l / 6 ,h*5 / 6 - r,0,1 };
//	Vec4 v20 = { l*5 / 6 ,h * 5 / 6 - r,0,1 };
//	Vec4 v21 = { l / 6 ,h * 5 / 6 + r,0,1 };
//	Vec4 v22 = { l * 5 / 6 ,h * 5 / 6 + r,0,1 };
//
//	varray<Spline> sps, sn1,sn2,sn3,sn4,sn5,sn6,sn7,sn8,sps2, temp,temp1;
//	Spline0 sl;
//	Spline sp,sp1;
//	//内轮廓
//	sp = sl.getArcSpline(v5, v6, v7);
//	sn1.push_back(sp);
//	sp = sl.getArcSpline(v7, v8, v9);
//	sn1.push_back(sp);
//	sp = sl.getArcSpline(v9, v10, v11);
//	sn1.push_back(sp);
//	sp = sl.getArcSpline(v11, v12, v5);
//	sn1.push_back(sp);
//
//	sn2 = sn1;
//	m.Trans(sn2, l / 3, 1);
//	sn3 = sn2;
//	m.Trans(sn3, l / 3, 1);
//
//	sn4 = sn1;
//	m.Trans(sn4, h*2 / 3, 2);
//	sn5 = sn2;
//	m.Trans(sn5, h*2 / 3, 2);
//	sn6 = sn3;
//	m.Trans(sn6, h * 2 / 3, 2);
//
//	sn7 = sn1;
//	m.Trans(sn7, l / 6, 1);
//	m.Trans(sn7, h / 3, 2);
//	sn8 = sn7;
//	m.Trans(sn8, l / 3, 1);
//	//外轮廓
//	sp = sl.getSpline(v1, p1);
//	sps.push_back(sp);
//	sp = sl.getSpline(p1, v2);
//	sps.push_back(sp);
//	sp = sl.getSpline(v2, p2);
//	sps.push_back(sp);
//	sp = sl.getSpline(p2, v3);
//	sps.push_back(sp);
//	sp = sl.getSpline(v3, p3);
//	sps.push_back(sp);
//	sp = sl.getSpline(p3, v4);
//	sps.push_back(sp);
//	sp = sl.getSpline(v4, p4);
//	sps.push_back(sp);
//	sp = sl.getSpline(p4, v1);
//	sps.push_back(sp);
//
//	//连接线
//	sp = sl.getSpline(v9, v13);
//	temp.push_back(sp);
//	sp1 = sp;
//	m.Trans(sp1, h / 3, 2);
//	m.Trans(sp1, l / 6, 1);
//	sps2.push_back(sp1);
//	
//	m.Trans(sp, l / 3, 1);
//	temp.push_back(sp);
//
//	temp1 = temp;
//	m.Trans(temp1, 2 * h / 3, 2);
//
//	for (int i = 0; i < temp1.size(); i++) {
//		temp.push_back(temp1[i]);
//	}
//	for (int i = 0; i < temp.size(); i++) {
//		sps2.push_back(temp[i]);
//	}
//
//	sp = sl.getSpline(v7, p1);
//	sps2.push_back(sp);
//	sp = sl.getSpline(v14, p3);
//	sps2.push_back(sp);
//	sp = sl.getSpline(v17, v19);
//	sps2.push_back(sp);
//	/*sp = sl.getSpline(v18, v20);
//	sps2.push_back(sp);*/
//	sp = sl.getSpline(v2, v21);
//	sps2.push_back(sp);
//	sp = sl.getSpline(v3, v22);
//	sps2.push_back(sp);
//
//
//
//	RWGeometric rwg;
//	rwg.WriteSpline("E:\\kuang_models\\outSpline.txt", sps);
//	rwg.WriteSpline("E:\\kuang_models\\inSpline1.txt", sn1);
//	rwg.WriteSpline("E:\\kuang_models\\inSpline2.txt", sn2);
//	rwg.WriteSpline("E:\\kuang_models\\inSpline3.txt", sn3);
//	rwg.WriteSpline("E:\\kuang_models\\inSpline4.txt", sn4);
//	rwg.WriteSpline("E:\\kuang_models\\inSpline5.txt", sn5);
//	rwg.WriteSpline("E:\\kuang_models\\inSpline6.txt", sn6);
//	rwg.WriteSpline("E:\\kuang_models\\inSpline7.txt", sn7);
//	rwg.WriteSpline("E:\\kuang_models\\inSpline8.txt", sn8);
//	rwg.WriteSpline("E:\\kuang_models\\addSpline.txt", sps2);
//
//	varray<Spline>outer;//存放外轮廓曲线
//	varray<Spline>inner1, inner2, inner3, inner4, inner5, inner6, inner7, inner8;//存放内轮廓曲线
//	varray<varray<Spline>> inner;
//	varray<Spline>addlines, allLines;//辅助线 将内外轮廓连接起来变为零亏格
//	varray<bool> genus;
//	varray<SplineSurface> allSurf;//存放剖分结果
//
//	rwg.ReadSpline("E:\\kuang_models\\outSpline.txt", outer);
//	rwg.ReadSpline("E:\\kuang_models\\inSpline1.txt", inner1);
//	rwg.ReadSpline("E:\\kuang_models\\inSpline2.txt", inner2);
//	rwg.ReadSpline("E:\\kuang_models\\inSpline3.txt", inner3);
//	rwg.ReadSpline("E:\\kuang_models\\inSpline4.txt", inner4);
//	rwg.ReadSpline("E:\\kuang_models\\inSpline5.txt", inner5);
//	rwg.ReadSpline("E:\\kuang_models\\inSpline6.txt", inner6);
//	rwg.ReadSpline("E:\\kuang_models\\inSpline7.txt", inner7);
//	rwg.ReadSpline("E:\\kuang_models\\inSpline8.txt", inner8);
//	rwg.ReadSpline("E:\\kuang_models\\addSpline.txt", addlines);
//	inner.push_back(inner1);
//	inner.push_back(inner2);
//	inner.push_back(inner3);
//	inner.push_back(inner4);
//	inner.push_back(inner5);
//	inner.push_back(inner6); 
//	inner.push_back(inner7);
//	inner.push_back(inner8);
//	genus.resize(9);
//	genus[0] = false;//我之前用的是push_back,会出现问题
//	genus[1] = true;
//	genus[2] = true;
//	genus[3] = true;
//	genus[4] = true;
//	genus[5] = true;
//	genus[6] = true;
//	genus[7] = true;
//	genus[8] = true;
//
//	quad(outer, inner, addlines, genus, allSurf);
//
//	rwg.WriteSplineSurface("E:\\kuang_models\\newSurface.txt", allSurf);
//	/*varray<SplineVolume> SV;
//	SV = m.CreatSweepVol(allSurf, 5, 3);*/
//
//	
//	//rwg.WriteSplineSurface("E:\\kuang_models\\newSurface.txt", SS);
//	/*rwg.WriteSplineVolume("E:\\kuang_models\\newVolume.txt", SV);
//
//	TestBolcks::pList plist;
//	plist.OutputParaVolumeDataTxt(SV, "E:\\kuang_models\\guPart");*/
//
//}


////吴
//void wuPart() {
//	double l = 20, h = 10,n=15;
//	Model_Solution m;
//	//坐标点
//	Vec4 v1 = { 0,0,0,1 };
//	Vec4 v2 = { 0,0,h / 2,1 };
//	Vec4 v3 = { 0, 0,h,1 };
//	Vec4 v4 = { 20,0,h,1 };
//	Vec4 v5 = { 20,0,h/2,1 };
//	Vec4 v6 = { 20,0,0,1 };
//	
//
//	varray<SplineSurface> SS, SS1;
//	SplineSurface ss;
//	RandomModel rm(v1, v2, v5, v6);
//	ss = rm.getSurface();
//	SS.push_back(ss);
//
//	RandomModel rm1(v2, v3, v4, v5);
//	ss = rm1.getSurface();
//	SS.push_back(ss);
//
//	varray<SplineVolume> SV;
//	SV = m.CreatSweepVol(SS, n, 2);
//
//	RWGeometric rwg;
//	//rwg.WriteSplineSurface("E:\\kuang_models\\newSurface.txt", SS);
//	rwg.WriteSplineVolume("E:\\kuang_models\\newVolume.txt", SV);
//
//
//}
//毛

void maoPart() {
	double l = 0.5, h = 0.25;
	Model_Solution m;
	//坐标点
	Vec4 v1 = { 0,0,0,1 };
	Vec4 v2 = { 0,h/5,0,1 };
	Vec4 v3 = { 0,h / 2,0,1 };
	Vec4 v4 = { 0,h,0,1 };
	Vec4 v5 = { l/10,0,0,1 };
	Vec4 v6 = { l / 10,h / 5,0,1 };
	Vec4 v7 = { l / 10,h / 2,0,1 };
	Vec4 v8 = { l / 10,h,0,1 };
	Vec4 v9 = { l / 2,0,0,1 };
	Vec4 v10 = { l / 2,h / 5,0,1 };
	Vec4 v11 = { l / 2,h / 2,0,1 };
	Vec4 v12 = { l / 2,h,0,1 };
	
	varray<SplineSurface> SS,SS1;
	SplineSurface ss;
	RandomModel rm(v1, v2, v6, v5);
	ss = rm.getSurface();
	SS.push_back(ss);
	
	RandomModel rm1(v2, v3, v7, v6);
	ss = rm1.getSurface();
	SS.push_back(ss);

	RandomModel rm2(v3, v4, v8, v7);
	ss = rm2.getSurface();
	SS.push_back(ss);

	SS1 = SS;
	//平移
	m.Trans(SS1, l - l / 10, 1);
	for (auto i : SS1) {
		SS.push_back(i);
	}

	RandomModel rm3(v5, v6, v10, v9);
	ss = rm3.getSurface();
	SS.push_back(ss);

	RandomModel rm4(v6, v7, v11, v10);
	ss = rm4.getSurface();
	SS.push_back(ss);

	RandomModel rm5(v7, v8, v12, v11);
	ss = rm5.getSurface();
	SS.push_back(ss);

	SS1 = SS;
	for (int i = 6; i < SS1.size(); i++) {
		m.Trans(SS1[i], l / 2 - l / 10, 1);
		SS.push_back(SS1[i]);
	}

	RWGeometric rwg;
	rwg.WriteSplineSurface("E:\\kuang_models\\newSurface.txt", SS);

}
void maoPart1() {
	double r = 0.07/2;//圆半径
	double L = 0.4;//外轮廓上边长
	double H = 0.4;//外轮廓高
	double L1 = 0.05;//小正方形边长

	Model_Solution m;
	PublicSolution ps;
	varray<Spline> inner,temp;
	varray<varray<Spline>> inners;//内轮廓
	varray<Spline> outer;//外轮廓
	varray<Spline> addLine;//连接线


	Spline0 sl;
	Spline s;
	inner = sl.arcSplines(r);
	temp = inner;
	m.Trans(inner, L / 4, -1);
	m.Trans(inner, H / 4, 2);
	inners.push_back(inner);
	inner = ps.mirror(inner, 2, 2);
	inners.push_back(inner);
	inner = ps.mirror(inner, 1, 2);
	inners.push_back(inner);
	inner = ps.mirror(inner, 2, 2);
	inners.push_back(inner);
	inners.push_back(temp);

	Vec4 v1 = { -(L / 2 + L1),L1 / 2,0,1 };
	Vec4 v2 = { -L / 2 ,L1 / 2,0,1 };
	Vec4 v3 = { -L / 2,H/2,0,1 };
	Vec4 v4 = { -L/4,H / 2,0,1 };
	Vec4 v5 = { 0,H / 2,0,1 };

	Vec4 v6 = { -(L / 2 + L1),-L1 / 2,0,1 };
	Vec4 v7 = { -L / 4,H / 4+r,0,1 };
	Vec4 v8 = { 0,r,0,1 };

	Vec4 v9 = { -L / 4-r,H / 4 ,0,1 };
	Vec4 v10 = { -L / 4+r,H / 4 ,0,1 };
	Vec4 v11 = { -L / 2,-L1 / 2,0,1 };

	s = sl.getSpline(v1, v2);
	outer.push_back(s);
	s = sl.getSpline(v2, v3);
	outer.push_back(s);
	s = sl.getSpline(v3, v4);
	outer.push_back(s);
	s = sl.getSpline(v4, v5);
	outer.push_back(s);

	outer = ps.mirror(outer, 1, 1);
	outer = ps.mirror(outer, 2, 1);
	s = sl.getSpline(v6, v1);
	temp = ps.mirror(s, 2, 1);
	outer.push_back(temp[0]);
	outer.push_back(temp[1]);

	temp.clear();
	s = sl.getSpline(v4, v7);
	temp.push_back(s);
	s = sl.getSpline(v3, v9);
	temp.push_back(s);
	s = sl.getSpline(v10, v5);
	temp.push_back(s);
	temp = ps.mirror(temp, 2, 1);
	addLine = temp;
	s = sl.getSpline(v8, v5);
	addLine.push_back(s);
	addLine = ps.mirror(addLine, 1, 1);
	temp.clear();
	s = sl.getSpline(v2, v11);
	temp = ps.mirror(s, 2, 1);
	addLine.push_back(temp[0]);
	addLine.push_back(temp[1]);

	varray<bool> genus;
	varray<SplineSurface> allSurf;//存放剖分结果
	genus.resize(6);
	genus[0] = false;
	genus[1] = false;
	genus[2] = false;
	genus[3] = false;
	genus[4] = false;
	genus[5] = false;
	ps.quad(outer, inners, addLine, genus, allSurf);


	//周围面片
	Vec4 v12 = { -(L / 2 + L1),H / 2,0,1 };
	Vec4 v13 = { -(L / 2 + L1) ,H / 2+L1,0,1 };
	Vec4 v14 = { -L / 2,H / 2+L1,0,1 };
	Vec4 v15 = { -L/4,H / 2+L1,0,1 };
	Vec4 v16 = { 0,H / 2 + L1,0,1 };
	varray<SplineSurface> SS;
	SplineSurface ss;
	RandomModel rm(v1,v12,v3,v2);
	ss = rm.getSurface();
	SS.push_back(ss);

	RandomModel rm1(v12, v13, v14, v3);
	ss = rm1.getSurface();
	SS.push_back(ss);

	RandomModel rm2(v3, v14, v15, v4);
	ss = rm2.getSurface();
	SS.push_back(ss);

	RandomModel rm3(v4, v15, v16, v5);
	ss = rm3.getSurface();
	SS.push_back(ss);

	SS = ps.mirror(SS, 2);
	SS = ps.mirror(SS, 1);

	for (auto& ss : SS) {
		allSurf.push_back(ss);
	}

	for (int i = 0; i < 5; i++) {
		allSurf.erase(allSurf.begin() + 35);
	}
	

	//补充中间孔
	RandomModel rm0(r);
	varray<SplineSurface> stemp;
	stemp = rm0.getArcSurface();
	for (auto& s : stemp) {
		allSurf.push_back(s);
	}
	m.Trans(stemp, L / 4, -1);
	m.Trans(stemp, H / 4, 2);
	for (auto& s : stemp) {
		allSurf.push_back(s);
	}
	m.Trans(stemp, L / 2, 1);
	for (auto& s : stemp) {
		allSurf.push_back(s);
	}
	m.Trans(stemp, H / 2, -2);
	for (auto& s : stemp) {
		allSurf.push_back(s);
	}
	m.Trans(stemp, L / 2, -1);
	for (auto& s : stemp) {
		allSurf.push_back(s);
	}
	varray<SplineVolume> SV;
	SV = m.CreatSweepVol(allSurf, 0.05, 3);

	
	SV[4].OrderCtrlPts(SV[4]);
	SV[5].OrderCtrlPts(SV[5]);
	SV[9].OrderCtrlPts(SV[9]);
	SV[11].OrderCtrlPts(SV[11]);
	SV[13].OrderCtrlPts(SV[13]);
	SV[16].OrderCtrlPts(SV[16]);
	SV[17].OrderCtrlPts(SV[17]);
	SV[19].OrderCtrlPts(SV[19]);
	SV[23].OrderCtrlPts(SV[23]);
	SV[25].OrderCtrlPts(SV[25]);
	SV[29].OrderCtrlPts(SV[29]);
	SV[32].OrderCtrlPts(SV[32]); 
	SV[34].OrderCtrlPts(SV[34]);
	SV[35].OrderCtrlPts(SV[35]);
	SV[38].OrderCtrlPts(SV[38]);
	SV[39].OrderCtrlPts(SV[39]);
	SV[42].OrderCtrlPts(SV[42]);
	SV[43].OrderCtrlPts(SV[43]);
	SV[46].OrderCtrlPts(SV[46]);
	SV[47].OrderCtrlPts(SV[47]);
	for (int i = 50; i < 76; i++) {
		SV[i].OrderCtrlPts(SV[i]);
	}

	ps.outPutVTK(SV,"E:\\kuang_models\\MaoPartVolume.vtk");

	ps.outPutTXT(allSurf, "E:\\kuang_models\\MaoPart");
	

	
	varray<Spline> SL;
	for (int i = 0; i < inners.size(); i++) {
		for (int j = 0; j < inners[i].size(); j++) {
			SL.push_back(inners[i][j]);
		}
	}

	RWGeometric rwg;
	rwg.WriteSplineSurface("E:\\kuang_models\\MaoSurface.txt", allSurf);
	rwg.WriteSplineVolume("E:\\kuang_models\\MaoVolume.txt", SV);
	rwg.WriteSpline("E:\\kuang_models\\MaooutSpline.txt", outer);
	rwg.WriteSpline("E:\\kuang_models\\MaoinSpline.txt", SL);
	rwg.WriteSpline("E:\\kuang_models\\MaoaddSpline.txt", addLine);

}






//爆炸图
void testBaozha() {
	varray<SplineVolume> SV;
	Model_Solution m;
	RWGeometric rwg;
	rwg.ReadSplineVolume("E:\\kuang_models\\buningyuan\\Box.txt", SV);
	m.Trans(SV[24], 2, -1);
	m.Trans(SV[25], 2, 1);
	m.Trans(SV[37], 2, -1);
	m.Trans(SV[38], 2, 1);
	m.Trans(SV[57], 2, -1);
	m.Trans(SV[58], 2, 1);
	m.Trans(SV[29], 2, -2);
	m.Trans(SV[28], 2, -2);
	m.Trans(SV[27], 2, -2);
	m.Trans(SV[26], 2, -2);
	m.Trans(SV[34], 2, -2);
	m.Trans(SV[35], 2, -2);
	m.Trans(SV[36], 2, -2);
	m.Trans(SV[39], 2, -2);
	m.Trans(SV[41], 2, -2);
	m.Trans(SV[44], 2, -2);
	m.Trans(SV[47], 2, -2);
	m.Trans(SV[50], 2, -2);
	for (int i = 52;i < 57;i++) {
		m.Trans(SV[i], 2, -2);
	}
	m.Trans(SV[59], 2, -2);
	for (int i = 76;i < 100;i++) {
		m.Trans(SV[i], 2, -2);
	}
	for (int i = 100;i < 148;i++) {
		m.Trans(SV[i], 2, 3);
	}
	for (int i = 148;i < 196;i++) {
		m.Trans(SV[i], 2, -3);
	}

	//细化
	for (int i = 0;i < SV.size();i++) {
		SV[i].KnotsRefineNum(1);
	}

	varray<NurbsVol> NV;
	NV = NurbsTrans::SplinevolsToCvols(SV);
	CPolyParaVolume cp;       //输出vtk文件的类对象
	cp = NV;
	cp.OutputParaVolumeDataVTK("E:\\kuang_models\\buningyuan\\BoxBaozha.vtk");
	rwg.WriteSplineVolume("E:\\kuang_models\\buningyuan\\BoxBaozha.txt", SV);
}

//小论文模型
void testPart0() {

	//尺寸一
	double l = 44, h = 22, r1 = 8, r2 = 2;//外轮廓长、宽、大半圆半径、小半圆半径
	double l1 = 8;//l1内轮廓边长
	double h1 = 5;//长方形的宽

	//////尺寸二
	//double l = 55, h = 33, r1 = 8, r2 = 2;//外轮廓长、宽、大半圆半径、小半圆半径
	//double l1 = 8;//l1内轮廓边长
	//double h1 = 5;//长方形的宽


	double w = cos(PI / 4);

	//外轮廓点坐标
	Vec4 v1 = { -l / 2 ,h / 2,0,1 };
	Vec4 v2 = { -r1 ,h / 2,0,1 };
	Vec4 v3 = { -r1 ,h / 2 + r1,0,w };
	Vec4 v4 = { 0 ,h / 2 + r1,0,1 };
	Vec4 v5 = { r1 ,h / 2 + r1,0,w };
	Vec4 v6 = { r1 ,h / 2,0,1 };
	Vec4 v7 = { l / 2 ,h / 2,0,1 };
	Vec4 v8 = { l / 2 ,-h / 2,0,1 };
	Vec4 v9 = { r1+r2 ,-h / 2,0,1 };
	Vec4 v10 = { r1 + r2 ,-h / 2+r2,0,w };
	Vec4 v11 = { r1 ,-h / 2 + r2,0,1 };
	Vec4 v12 = { r1 - r2 ,-h / 2 + r2,0,w };
	Vec4 v13 = { r1 - r2 ,-h / 2,0,1 };
	Vec4 v14 = { -r1 + r2 ,-h / 2,0,1 };
	Vec4 v15 = { -r1 + r2 ,-h / 2+r2,0,w };
	Vec4 v16 = { -r1 ,-h / 2+r2,0,1 };
	Vec4 v17 = { -r1 - r2 ,-h / 2 + r2,0,1 };
	Vec4 v18 = { -r1 - r2 ,-h / 2,0,1 };
	Vec4 v19 = { -l / 2 ,-h / 2,0,1 };

	//内轮廓点坐标
	Vec4 p1 = { -l1 / 2,l1 / 2 - r2,0,1 };
	Vec4 p2 = { -l1 / 2-r2,l1 / 2 - r2,0,w };
	Vec4 p3 = { -l1 / 2-r2,l1 / 2,0,1 };
	Vec4 p4 = { -l1 / 2 - r2,l1 / 2 + r2,0,w };
	Vec4 p5 = { -l1 / 2,l1 / 2 + r2,0,1 };
	Vec4 p6 = { -l1 / 2+r2,l1 / 2 + r2,0,w };
	Vec4 p7 = { -l1 / 2+r2,l1 / 2,0,1 };
	Vec4 p8 = { l1 / 2-r2,l1 / 2,0,1 };
	Vec4 p9 = { l1 / 2-r2,l1 / 2 + r2,0,w };
	Vec4 p10 = { l1 / 2,l1 / 2 + r2,0,1 };
	Vec4 p11 = { l1 / 2+r2,l1 / 2+r2,0,w };
	Vec4 p12 = { l1 / 2+r2,l1 / 2,0,1 };
	Vec4 p13 = { l1 / 2+r2,l1 / 2 - r2,0,1 };
	Vec4 p14 = { l1 / 2,l1 / 2 - r2,0,1 };
	Vec4 p15 = { l1 / 2,-l1 / 2 + r2,0,1 };
	Vec4 p16 = { l1 / 2+r2,-l1 / 2 + r2,0,w };
	Vec4 p17 = { l1 / 2+r2,-l1 / 2,0,1 };
	Vec4 p18 = { l1 / 2+r2,-l1 / 2 - r2,0,w };
	Vec4 p19 = { l1 / 2,-l1 / 2 - r2,0,1 };
	Vec4 p20 = { l1 / 2-r2,-l1 / 2 - r2,0,w };
	Vec4 p21 = { l1 / 2 - r2,-l1 / 2,0,1 };
	Vec4 p22 = { -l1 / 2 + r2,-l1 / 2,0,1 };
	Vec4 p23 = { -l1 / 2 + r2,-l1 / 2 - r2,0,w };
	Vec4 p24 = { -l1 / 2,-l1 / 2 - r2,0,1 };
	Vec4 p25 = { -l1 / 2 - r2,-l1 / 2-r2,0,w };
	Vec4 p26 = { -l1 / 2 - r2,-l1 / 2,0,1 };
	Vec4 p27 = { -l1 / 2 - r2,-l1 / 2 + r2,0,w };
	Vec4 p28 = { -l1 / 2,-l1 / 2 + r2,0,1 };

	//内轮廓---长方形
	Vec4 p29 = { -l1-h1,-l1 / 2,0,1 };
	Vec4 p30 = { -l1 - h1,l1 / 2,0,1 };
	Vec4 p31 = { -l1,l1 / 2,0,1 };
	Vec4 p32 = { -l1,-l1 / 2,0,1 };
	Vec4 p33 = { l1,-l1 / 2,0,1 };
	Vec4 p34 = { l1,l1 / 2,0,1 };
	Vec4 p35 = { l1 + h1,l1 / 2,0,1 };
	Vec4 p36 = { l1 + h1,-l1 / 2,0,1 };


	varray<Spline> sps,sps1,sps2,sps3,sps7,sps4;//分别存放外轮廓、内轮廓、内外连接线
	Spline sp;
	Spline0 sl;

	//外轮廓
	sp = sl.getSpline(v1,v2);
	sps.push_back(sp);

	sp = sl.getArcSpline(v2, v3, v4);
	sps.push_back(sp);

	sp = sl.getArcSpline(v4, v5, v6);
	sps.push_back(sp);

	sp = sl.getSpline(v6, v7);
	sps.push_back(sp);

	sp = sl.getSpline(v7, v8);
	sps.push_back(sp);

	sp = sl.getSpline(v8, v9);
	sps.push_back(sp);

	sp = sl.getArcSpline(v9, v10, v11);
	sps.push_back(sp);

	sp = sl.getArcSpline(v11, v12, v13);
	sps.push_back(sp);

	sp = sl.getSpline(v13, v14);
	sps.push_back(sp);

	sp = sl.getArcSpline(v14, v15, v16);
	sps.push_back(sp);

	sp = sl.getArcSpline(v16, v17, v18);
	sps.push_back(sp);

	sp = sl.getSpline(v18, v19);
	sps.push_back(sp);

	sp = sl.getSpline(v19, v1);
	sps.push_back(sp);

	//内轮廓线
	sp = sl.getArcSpline(p3, p2, p1);
	sps1.push_back(sp);

	sp = sl.getArcSpline(p5, p4, p3);
	sps1.push_back(sp);

	sp = sl.getArcSpline(p7, p6, p5);
	sps1.push_back(sp);

	sp = sl.getSpline(p8, p7);
	sps1.push_back(sp);

	sp = sl.getArcSpline(p10, p9, p8);
	sps1.push_back(sp);

	sp = sl.getArcSpline(p12, p11, p10);
	sps1.push_back(sp);

	sp = sl.getArcSpline(p14, p13, p12);
	sps1.push_back(sp);

	sp = sl.getSpline(p15, p14);
	sps1.push_back(sp);

	sp = sl.getArcSpline(p17, p16, p15);
	sps1.push_back(sp);

	sp = sl.getArcSpline(p19, p18, p17);
	sps1.push_back(sp);

	sp = sl.getArcSpline(p21, p20, p19);
	sps1.push_back(sp);

	sp = sl.getSpline(p22, p21);
	sps1.push_back(sp);

	sp = sl.getArcSpline(p24, p23, p22);
	sps1.push_back(sp);

	sp = sl.getArcSpline(p26, p25, p24);
	sps1.push_back(sp);

	sp = sl.getArcSpline(p28, p27, p26);
	sps1.push_back(sp);

	sp = sl.getSpline(p1, p28);
	sps1.push_back(sp);

	//长方形
	sp = sl.getSpline(p29, p32);
	sps2.push_back(sp);
	sp = sl.getSpline(p32, p31);
	sps2.push_back(sp);
	sp = sl.getSpline(p31, p30);
	sps2.push_back(sp);
	sp = sl.getSpline(p30, p29);
	sps2.push_back(sp);

	sp = sl.getSpline(p33, p36);
	sps3.push_back(sp);
	sp = sl.getSpline(p36, p35);
	sps3.push_back(sp);
	sp = sl.getSpline(p35, p34);
	sps3.push_back(sp);
	sp = sl.getSpline(p34, p33);
	sps3.push_back(sp);

	

	//内外连接线
	//连接方式一
	sp = sl.getSpline(p30, v1);
	sps4.push_back(sp);
	sp = sl.getSpline(p3, p31);
	sps4.push_back(sp);
	sp = sl.getSpline(v7, p35);
	sps4.push_back(sp);
	
	////////连接方式二
	//sp = sl.getSpline(v8, p36);
	//sps4.push_back(sp);
	//sp = sl.getSpline(p31, p3);
	//sps4.push_back(sp);
	//sp = sl.getSpline(p17, p33);
	//sps4.push_back(sp);
	
	
	varray<Spline> sps5, sps6;
	for (auto& s : sps) {
		sps5.push_back(s);
	}
	for (auto& s : sps1) {
		sps5.push_back(s);
	}
	for (auto& s : sps2) {
		sps5.push_back(s);
	}
	for (auto& s : sps3) {
		sps5.push_back(s);
	}
	sps6 = sps5;
	for (auto& s : sps4) {
		sps5.push_back(s);
	}
	RWGeometric rwg;
	rwg.WriteSpline("E:\\kuang_models\\outSpline.txt", sps);
	rwg.WriteSpline("E:\\kuang_models\\inSpline0.txt", sps1);
	rwg.WriteSpline("E:\\kuang_models\\inSpline1.txt", sps2);
	rwg.WriteSpline("E:\\kuang_models\\inSpline2.txt", sps3);
	rwg.WriteSpline("E:\\kuang_models\\subSpline.txt", sps4);
	rwg.WriteSpline("E:\\kuang_models\\allSpline1.txt", sps5);
	rwg.WriteSpline("E:\\kuang_models\\allSpline2.txt", sps6);

}

void quad1() {
	PublicSolution ps;

	varray<Spline>outer;//存放外轮廓曲线
	varray<Spline>inner1, inner2, inner3;//存放内轮廓曲线
	varray<varray<Spline>> inner;
	varray<Spline>addlines, allLines;//辅助线 将内外轮廓连接起来变为零亏格
	varray<bool> genus;
	varray<SplineSurface> allSurf;//存放剖分结果

	RWGeometric rwg;
	rwg.ReadSpline("E:\\kuang_models\\outSpline.txt", outer);
	rwg.ReadSpline("E:\\kuang_models\\inSpline0.txt", inner1);
	rwg.ReadSpline("E:\\kuang_models\\inSpline1.txt", inner2);
	rwg.ReadSpline("E:\\kuang_models\\inSpline2.txt", inner3);
	rwg.ReadSpline("E:\\kuang_models\\subSpline.txt", addlines);
	inner.push_back(inner1);
	inner.push_back(inner2);
	inner.push_back(inner3);
	genus.resize(4);
	genus[0] = false;//我之前用的是push_back,会出现问题
	genus[1] = true;
	genus[2] = true;
	genus[3] = true;

	//统计剖分程序时间
	time_t begin, end;
	double ret;
	begin = clock();
	ps.quad(outer, inner, addlines, genus, allSurf);
	end = clock();
	ret = double(end - begin) / CLOCKS_PER_SEC;
	cout << "运行时间：" << ret << endl;

	rwg.WriteSplineSurface("E:\\kuang_models\\newSurface.txt", allSurf);

	Model_Solution m;
	varray<SplineVolume> SV;
	SV = m.CreatSweepVol(allSurf, 5, 3);
	////尺寸一和连接线一
	//SV[0].OrderCtrlPts(SV[0]);
	//SV[7].OrderCtrlPts(SV[7]);
	//SV[8].OrderCtrlPts(SV[8]);
	//SV[10].OrderCtrlPts(SV[10]);
	//SV[15].OrderCtrlPts(SV[15]);
	//SV[19].OrderCtrlPts(SV[19]);
	//SV[20].OrderCtrlPts(SV[20]);
	/*尺寸二和连接线二
	SV[0].OrderCtrlPts(SV[0]);
	SV[5].OrderCtrlPts(SV[5]);
	SV[10].OrderCtrlPts(SV[10]);
	SV[12].OrderCtrlPts(SV[12]);
	SV[21].OrderCtrlPts(SV[21]);*/

	////尺寸一和连接线二
	//SV[2].OrderCtrlPts(SV[2]);
	//SV[5].OrderCtrlPts(SV[5]);
	//SV[9].OrderCtrlPts(SV[9]);
	//SV[10].OrderCtrlPts(SV[10]);
	//SV[14].OrderCtrlPts(SV[14]);
	//SV[16].OrderCtrlPts(SV[16]);
	//SV[18].OrderCtrlPts(SV[18]);
	//SV[21].OrderCtrlPts(SV[21]);
	
	////尺寸二和连接线二
	//SV[2].OrderCtrlPts(SV[2]);
	//SV[10].OrderCtrlPts(SV[10]);
	//SV[20].OrderCtrlPts(SV[20]);
	//SV[24].OrderCtrlPts(SV[24]);
	//SV[21].OrderCtrlPts(SV[21]);
	//SV[25].OrderCtrlPts(SV[25]);
	rwg.WriteSplineVolume("E:\\kuang_models\\newVolume.txt", SV);

	
	varray<NurbsVol> NV;
	NV = NurbsTrans::SplinevolsToCvols(SV);
	CPolyParaVolume cp;       //输出vtk文件的类对象
	cp = NV;
	cp.OutputParaVolumeDataVTK("E:\\kuang_models\\newVolume.vtk");

}

void testPart2() {
	//尺寸一
	//double r1 = 5, r2=10,r3=8, l1 = 40, l2 = 15;//倒角半径、半圆半径、内圆半径、轮廓长度、内部正方形边长

	//尺寸二
	double r1 = 8, r2 = 10, r3 = 12, l1 = 50, l2 = 10;//倒角半径、半圆半径、内圆半径、轮廓长度、内部正方形边长

	double l3 = (l1 - 2 * r1);//轮廓正方形边长
	double w = cos(PI / 4);

	//外轮廓
	Vec4 v1 = { -l1 / 2,-l1 / 2 ,0,1 };
	Vec4 v2 = { -l1 / 2,l1 / 2 - r1,0,1 };
	Vec4 v3 = { -l1 / 2,l1 / 2,0,w };
	Vec4 v4 = { -l1 / 2+r1,l1 / 2,0,1 };

	Vec4 v5 = { -l3/2,l1 / 2+l3,0,1 };
	Vec4 v6 = { l3/2,l1 / 2+l3,0,1 };
	Vec4 v7 = { l3/2,l1 / 2,0,1 };
	Vec4 v8= { l1 / 2,l1 / 2,0,w };
	Vec4 v9 = { l1 / 2,l1 / 2-r1,0,1 };
	Vec4 v10 = { l1 / 2,-l1 / 2,0,1 };
	Vec4 v11 = { r2,-l1 / 2,0,1 };
	Vec4 v12 = { r2,-l1 / 2 - r2,0,w };
	Vec4 v13 = { 0,-l1 / 2-r2,0,1 };
	Vec4 v14 = { -r2,-l1 / 2 - r2,0,w };
	Vec4 v15 = { -r2,-l1 / 2,0,1 };
	
	//内轮廓---正方形
	Vec4 p1 = { -l2/2 ,l1/2 ,0,1 };
	Vec4 p2 = { -l2/2 ,l1 / 2+l2 ,0,1 };
	Vec4 p3 = { l2/2 ,l1 / 2 + l2 ,0,1 };
	Vec4 p4 = { l2/2 ,l1 / 2 ,0,1 };
	
	//内轮廓---圆
	Vec4 p17 = { -r3 ,0 ,0,1 };
	Vec4 p18 = { -r3 ,r3 ,0,w };
	Vec4 p19 = { 0 ,r3 ,0,1 };
	Vec4 p20 = { r3 ,r3 ,0,w };
	Vec4 p21 = { r3 ,0 ,0,1 };
	Vec4 p22 = { r3 ,-r3 ,0,w };
	Vec4 p23 = { 0 ,-r3 ,0,1 };
	Vec4 p24 = { -r3 ,-r3 ,0,w };

	varray<Spline> sps, sps1, sps2, sps3;//分别存放外轮廓、内轮廓、内外连接线
	Spline sp;
	Spline0 sl;

	//外轮廓
	sp = sl.getSpline(v1, v15);
	sps.push_back(sp);
	sp = sl.getArcSpline(v15, v14, v13);
	sps.push_back(sp);
	sp = sl.getArcSpline(v13, v12, v11);
	sps.push_back(sp);
	sp = sl.getSpline(v11, v10);
	sps.push_back(sp);
	sp = sl.getSpline(v10, v9);
	sps.push_back(sp);
	sp = sl.getArcSpline(v9, v8, v7);
	sps.push_back(sp);
	sp = sl.getSpline(v7, v6);
	sps.push_back(sp);
	sp = sl.getSpline(v6, v5);
	sps.push_back(sp);
	sp = sl.getSpline(v5, v4);
	sps.push_back(sp);
	sp = sl.getArcSpline(v4, v3, v2);
	sps.push_back(sp);
	sp = sl.getSpline(v2, v1);
	sps.push_back(sp);
	//内轮廓---正方形
	sp = sl.getSpline(p1, p2);
	sps1.push_back(sp);
	sp = sl.getSpline(p2, p3);
	sps1.push_back(sp);
	sp = sl.getSpline(p3, p4);
	sps1.push_back(sp);
	sp = sl.getSpline(p4, p1);
	sps1.push_back(sp);
	
	//内轮廓---圆
	sp = sl.getArcSpline(p17, p18, p19);
	sps2.push_back(sp);
	sp = sl.getArcSpline(p19, p20, p21);
	sps2.push_back(sp);
	sp = sl.getArcSpline(p21, p22, p23);
	sps2.push_back(sp);
	sp = sl.getArcSpline(p23, p24, p17);
	sps2.push_back(sp);
	//辅助线---1 与尺寸二搭配不成功
	sp = sl.getSpline(v4, p1);
	sps3.push_back(sp);
	sp = sl.getSpline(v11, p23);
	sps3.push_back(sp);
	////辅助线---2与尺寸一二都可
	//sp = sl.getSpline(v5, p2);
	//sps3.push_back(sp);
	//sp = sl.getSpline(v11, p23);
	//sps3.push_back(sp);

	////辅助线---3
	//sp = sl.getSpline(v6, p3);
	//sps3.push_back(sp);
	//sp = sl.getSpline(p1, p19);
	//sps3.push_back(sp);

	varray<Spline> sps4, sps5;
	for (auto& s : sps) {
		sps4.push_back(s);
	}
	for (auto& s : sps1) {
		sps4.push_back(s);
	}
	for (auto& s : sps2) {
		sps4.push_back(s);
	}
	sps5 = sps4;
	for (auto& s : sps3) {
		sps5.push_back(s);
	}
	RWGeometric rwg;
	rwg.WriteSpline("E:\\kuang_models\\outSpline.txt", sps);
	rwg.WriteSpline("E:\\kuang_models\\inSpline0.txt", sps1);
	rwg.WriteSpline("E:\\kuang_models\\inSpline1.txt", sps2);
	rwg.WriteSpline("E:\\kuang_models\\subSpline.txt", sps3);
	rwg.WriteSpline("E:\\kuang_models\\allSpline1.txt", sps4);
	rwg.WriteSpline("E:\\kuang_models\\allSpline2.txt", sps5);
}

void quad2() {
	PublicSolution ps;

	varray<Spline>outer;//存放外轮廓曲线
	varray<Spline>inner1, inner2;//存放内轮廓曲线
	varray<varray<Spline>> inner;
	varray<Spline>addlines, allLines;//辅助线 将内外轮廓连接起来变为零亏格
	varray<bool> genus;
	varray<SplineSurface> allSurf;//存放剖分结果

	RWGeometric rwg;
	rwg.ReadSpline("E:\\kuang_models\\outSpline.txt", outer);
	rwg.ReadSpline("E:\\kuang_models\\inSpline0.txt", inner1);
	rwg.ReadSpline("E:\\kuang_models\\inSpline1.txt", inner2);
	rwg.ReadSpline("E:\\kuang_models\\subSpline.txt", addlines);
	inner.push_back(inner1);
	inner.push_back(inner2);
	genus.resize(3);
	genus[0] = false;//我之前用的是push_back,会出现问题
	genus[1] = true;
	genus[2] = true;

	//统计剖分程序时间
	time_t begin, end;
	double ret;
	begin = clock();
	ps.quad(outer, inner, addlines, genus, allSurf);
	end = clock();
	ret = double(end - begin) / CLOCKS_PER_SEC;
	cout << "运行时间：" << ret << endl;

	rwg.WriteSplineSurface("E:\\kuang_models\\newSurface.txt", allSurf);

	Model_Solution m;
	varray<SplineVolume> SV;
	SV = m.CreatSweepVol(allSurf, 5, 3);
	//////尺寸一、二和连接线二
	//SV[6].OrderCtrlPts(SV[6]);
	//SV[7].OrderCtrlPts(SV[7]);
	//SV[8].OrderCtrlPts(SV[8]);
	//SV[10].OrderCtrlPts(SV[10]);
	//SV[11].OrderCtrlPts(SV[11]);
	//SV[12].OrderCtrlPts(SV[12]);
	//for (int i = 0;i < SV.size();i++) {
	//	SV[i].OrderCtrlPts(SV[i]);
	//}
	//尺寸一和连接线三
	SV[1].OrderCtrlPts(SV[1]);
	SV[0].OrderCtrlPts(SV[0]);
	SV[9].OrderCtrlPts(SV[9]);
	SV[10].OrderCtrlPts(SV[10]);
	for (int i = 0;i < SV.size();i++) {
		SV[i].OrderCtrlPts(SV[i]);
	}
	//细化一次
	for (int i = 0;i < SV.size();i++) {
		SV[i].KnotsRefineNum(1);
	}
	rwg.WriteSplineVolume("E:\\kuang_models\\newVolume.txt", SV);

	varray<NurbsVol> NV;
	NV = NurbsTrans::SplinevolsToCvols(SV);
	CPolyParaVolume cp;       //输出vtk文件的类对象
	cp = NV;
	cp.OutputParaVolumeDataVTK("E:\\kuang_models\\newVolume.vtk");

}


//圆环
varray<SplineVolume> annulus(double r1,double r2,double angle,double angle1, double h) {
	Annulus ann1(r1, r2, angle);
	varray<SplineSurface> ss;
	SplineSurface ss1,ss2,ss3;
	ss1 = ann1.getSurface();
	Model_Solution m;
	m.RotateSurface(ss1, -angle / 2, 3);
	ss.push_back(ss1);
	m.RotateSurface(ss1, angle, 3);
	ss.push_back(ss1);

	Annulus ann3(r1, r2, angle1);
	ss3 = ann3.getSurface();
	m.RotateSurface(ss3, PI, 3);
	ss.push_back(ss3);

	double a = (PI * 2 - 2 * angle-angle1) / 2;
	Annulus ann2(r1, r2, a);
	ss2 = ann2.getSurface();
	m.RotateSurface(ss2, PI - angle1 /2 - a / 2, 3);
	ss.push_back(ss2);
	m.RotateSurface(ss2, angle1 + a, 3);
	ss.push_back(ss2);

	varray<SplineVolume> SV;
	SV = m.CreatSweepVol(ss, h, 3);

	return SV;

	//varray<NurbsVol> NV;
	//NV = NurbsTrans::SplinevolsToCvols(SV);
	//CPolyParaVolume cp;       //输出vtk文件的类对象
	//cp = NV;
	///////*cp.SetAdjacentHexIdx();
	/////*cp.Order();*/
	//cp.OutputParaVolumeDataVTK("E:\\kuang_models\\annulus.vtk");

	//RWGeometric rwg;
	////rwg.WriteSplineSurface("E:\\kuang_models\\circlespline.txt", ss);
	//rwg.WriteSplineVolume("E:\\kuang_models\\circleSV.txt", SV);

}

void newPart() {
	//高层参数
	double r1=0.75;//内径
	double r2 = 2.75;//外径
	double h = 4;//厚度

	double c1=5; //B部分上边长
	double c2 = 34;//B部分下边长
	double c3 = 38;//B部分斜边长
	double t1 = 9;
	double l4 = 5;
	double t2 = 4;

	double a2 = PI * 14.5 / 180;//顶部圆环的下部分两边片的弧度
	double a0 = asin((c1 / 2) / r2);//圆弧部分夹角
	double a1 = asin(((c2 - c1) / 2) / c3);//B部分斜边与竖直方向夹角
	double s = cos(a0)*r2 + cos(a1)*c3;//上圆弧圆心平移距离

	varray<SplineVolume> SV;

	varray<SplineVolume> SV1;
	SV1 = annulus(r1, r2, PI - a0, (a0 - a2) * 2, h);
	Model_Solution m;
	m.Rolate(SV1, -PI / 2, 1);
	m.Rolate(SV1, -PI, 2);
	m.Trans(SV1, c2 / 2, 1);
	m.Trans(SV1, s, 3);
	m.Trans(SV1, (t1-h)/2, 2);
	

	varray<SplineVolume> SV2;
	varray<SplineVolume> SV3;
	RWGeometric rwg;
	rwg.ReadSplineVolume("E:\\kuang_models\\kuang\\volm.txt", SV2);
	rwg.ReadSplineVolume("E:\\kuang_models\\kuang\\Wheel_model.txt", SV3);

	for (int i = 0;i < SV1.size();i++) {
		SV.push_back(SV1[i]);
	}
	for (int i = 0;i < SV2.size();i++) {
		SV.push_back(SV2[i]);
	}
	for (int i = 0;i < SV3.size();i++) {
		SV3[i].OrderCtrlPts(SV3[i]);
		SV.push_back(SV3[i]);

	}

	for (int i = 0;i < SV.size();++i) {
		SV[i].KnotsRefineNum(2);
	}

	TestBolcks::get_Align(SV);//检测控制点是否对齐

	varray<NurbsVol> NV;
	NV = NurbsTrans::SplinevolsToCvols(SV);
	CPolyParaVolume cp;       //输出vtk文件的类对象
	cp = NV;
	cp.OutputParaVolumeDataVTK("E:\\kuang_models\\newpart.vtk");

	rwg.WriteSplineVolume("E:\\kuang_models\\kuang\\newPart.txt", SV);
}




//精密医疗零件
void precise_p(double r, double h, double a, double b, double c, double d) {
	Precise_part p;
	varray<SplineVolume> SV1;
	SV1 = p.get_vol(r, h, a, b, c, d);
	varray<SplineVolume> SV2;
	SV2 = p.get_vol1(r, h, a, b, c, d);
	Model_Solution m;
	m.Trans(SV2, h, 3);
	Rec re;
	varray<SplineVolume> SV3;
	SV3 = re.get_vol(c - 2 * r, d, h);

	varray<SplineVolume> SV;
	for (int i = 0; i < SV1.size(); i++) {
		SV.push_back(SV1[i]);
	}
	for (int i = 0; i < SV2.size(); i++) {
		SV.push_back(SV2[i]);
	}
	for (int i = 0; i < SV3.size(); i++) {
		SV.push_back(SV3[i]);
	}
	RWGeometric rwg;
	rwg.WriteSplineVolume("E:\\kuang_models\\SV.txt", SV);
}

//燃料棒
void fuel_rod() {
	varray<SplineVolume> SV;
	Fuel_rod f;
	SV = f.fuel_rod2(80, 0);
	for (int i = 0; i < SV.size(); i++) {
		SV[i].KnotsRefineNum(0);
	}
	RWGeometric rwg;
	rwg.WriteSplineVolume("E:\\kuang_models\\fuel_rod4.txt", SV);

	//f.test();
}

//圆柱
void cylinder() {
	Cylinder c;
	varray<SplineVolume> SV;
	SV = c.cylinder1(2, 0.5, 0);
	RWGeometric rwg;
	/*rwg.WriteSplineVolume("E:\\kuang_models\\cylinder1-3.txt", SV);*/

	c.test();

}

////三个组合圆柱
//void c_cylinder() {
//	Model_Solution m;
//	Circle c;
//	varray<SplineSurface> SS;
//	varray<SplineSurface> SS1;
//	varray<SplineSurface> SS2;
//	varray<SplineVolume> SV;
//	varray<SplineVolume> SV1;
//	varray<SplineVolume> SV2;
//	varray<SplineVolume> SV3;
//	SS = c.rec_circle(26, 34.65);
//
//	SS1 = c.rec_circle(34.65, 48.725);
//	SS2 = c.rec_circle(26, 34.65);
//	SV = m.CreatSweepVol(SS, 10, 3);
//	SV1 = m.CreatSweepVol(SS1, 10, 3);
//	SV2 = m.CreatSweepVol(SS2, 20, 3);
//	m.Trans(SV2, 10, 3);
//	for (int i = 0; i < SV1.size(); i++) {
//		SV.push_back(SV1[i]);
//	}
//	for (int i = 0; i < SV2.size(); i++) {
//		SV.push_back(SV2[i]);
//	}
//	for (int i = 0; i < SV.size(); i++) {
//		SV3.push_back(SV[i]);
//	}
//	m.Rolate(SV, PI, 3);
//	for (int i = 0; i < SV.size(); i++) {
//		SV3.push_back(SV[i]);
//	}
//	varray<NurbsVol> NV;
//	NV = NurbsTrans::SplinevolsToCvols(SV3);
//	CPolyParaVolume cp;       //输出vtk文件的类对象
//	cp = NV;
//	cp.OutputParaVolumeDataVTK("E:\\kuang_models\\c_cylinderVolumevtk.vtk");
//	RWGeometric rwg;
//	rwg.WriteSplineVolume("E:\\kuang_models\\c_cylinderVolume.txt", SV3);
//	/*rwg.WriteSplineSurface("E:\\kuang_models\\c_cylinder.txt", SS2);*/
//}

////承受装置
//void part() {
//	//中间梯形
//	Model_Solution m;
//	//存放相对梯形
//	varray<SplineVolume> SV;
//	varray<SplineVolume> SV3;
//	varray<SplineVolume> SV4;
//	//存放正方体-圆柱孔
//	varray<SplineVolume> SV1;
//	varray<SplineVolume> SV2;
//
//	varray<SplineVolume> SV0;
//	varray<SplineVolume> SV5;
//	varray<SplineVolume> SV6;
//	varray<SplineVolume> SV7;
//	varray<SplineVolume> SV8;
//	varray<SplineVolume> SV9;
//	//相对梯形
//	Cube_slice cs;
//	SV = cs.get_vol2(36, 50, 7, 7);
//	SV3 = cs.get_vol(3, 10, 7, 7);
//	m.Trans(SV3, 35, -1);
//	SV4 = cs.get_vol1(3, 10, 7, 7);
//	m.Trans(SV4, 35, 1);
//	for (int i = 0; i < SV3.size(); i++) {
//		SV.push_back(SV3[i]);
//		SV.push_back(SV4[i]);
//	}
//	//正方体-圆柱孔
//	Cube_Cylinder cc;
//	SV1 = cc.get_vol(sqrt(2) * 7, 4, 7);
//	for (int i = 0; i < SV1.size(); i++) {
//		SV2.push_back(SV1[i]);
//	}
//	m.Rolate(SV1, -PI / 4, 3);
//	m.Rolate(SV2, -PI / 4, 3);
//	m.Trans(SV1, 25, 1);
//	m.Trans(SV2, 25, -1);
//	for (int i = 0; i < SV1.size(); i++) {
//		SV.push_back(SV1[i]);
//		SV.push_back(SV2[i]);
//	}
//
//
//	//前四个体复制一份，用于移动
//	for (int i = 0; i < SV.size(); i++) {
//		SV0.push_back(SV[i]);
//	}
//	m.Trans(SV0, 86, 2);
//	for (int i = 0; i < SV0.size(); i++) {
//		SV.push_back(SV0[i]);
//	}
//	//长方体
//	Rec rec;
//	//左边体
//	SV5 = rec.get_vol(10, 8, 7);
//	m.Trans(SV5, 11, 2);
//	m.Trans(SV5, 30, -1);
//	//右边体
//	SV6 = rec.get_vol(10, 8, 7);
//	m.Trans(SV6, 11, 2);
//	m.Trans(SV6, 30, 1);
//	//中间体
//	SV7 = rec.get_vol(50, 8, 7);
//	m.Trans(SV7, 11, 2);
//	//中间偏上体
//	SV8 = rec.get_vol(50, 8, 40);
//	m.Trans(SV8, 11, 2);
//	m.Trans(SV8, 7, 3);
//	for (int i = 0; i < SV5.size(); i++) {
//		SV9.push_back(SV5[i]);
//		SV9.push_back(SV6[i]);
//		SV9.push_back(SV7[i]);
//		SV9.push_back(SV8[i]);
//		SV.push_back(SV5[i]);
//		SV.push_back(SV6[i]);
//		SV.push_back(SV7[i]);
//		SV.push_back(SV8[i]);
//	}
//	m.Trans(SV9, 64, 2);
//	for (int i = 0; i < SV9.size(); i++) {
//		SV.push_back(SV9[i]);
//	}
//
//
//	RWGeometric rwg;
//	rwg.WriteSplineVolume("E:\\kuang_models\\part.txt", SV);
//}