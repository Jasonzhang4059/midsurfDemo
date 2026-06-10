//////////////////体模型的切片类////////////////////////
///////////////2020-3-1 193780714///////////////////////
#pragma once

#include"stdafx.h"

using std::min;
using std::max;
using std::vector;

class Fragment
{
public:
	Fragment();
	Fragment(float a, float b, float c,float begin,float end,float degree);//a,b,c对应平面的一般参数方程，begin，end对应平面的开始和起始位置，degree对应细分度
	
	~Fragment();
	//vector<float> ReturnCurve();//返回等参线的点
	vector<float> ReturnSupportPoints();//返回支撑点
	//vector<float> ReturnFourBoundary();//返回截面边界
	//vector<float> ReturnSurface();//截面点和截面

	bool intersectionLinePlane(point3d p1, point3d p2, vector<float> & temp,  float D);  //计算交点
	vector<vector<float>> getPoints(varray < varray<varray<point3d>>> points); //获得切片点
	vector<float> getSupportPoints(); //获得支撑点
	bool SetInOrOut(point3d p,int n);//点是否落在平面内
	point3d getShadow(point3d p, float D);
protected:
	//辅助计算函数
	point3d alf_mult_point(double alf, point3d p);

	//步骤
	//void QuadsAndHexs();                    //构建多片体参数化模型
	//void SolvingSection();                  //等参单元截面
	//void SolvingSupport();                  //支撑点求解

	//
	//void ReturnSupport();                                             //返还支撑点
	//
	//void GetSupporPoints();                                           //求解支撑点
	//
	//void FindBoundaryCurve();                                         //求解边界轮廓
	//void ClearHex();                                                  //清空单元
	//void FindBoundaryPoints();                                         //查找边界
	//void CoordinateTransformation();                                  //空间坐标转换
	//void ChangePoints(point3d & p1, point3d & p2);                     //交换顶点
	//double Angle(point3d pt1, point3d pt2, point3d pt3);                //计算向量夹角
	//
	//void SurfaceFitting();                                            //平面拟合
	//
	//void CIntersecPoints();                                           //直线与平面相交点计算
	//void GetAllCell(int Unum, int Vnum, int Wnum, int Uorder, int Vorder, int Worder, varray<double> &uknots, varray<double> &vknots, varray<double> &wknots, varray<point3d> &bpts);//计算等参线单元
	//point3d GetCubicPoint(int Unum, int Vnum, int Wnum, int uid, int vid, int wid, int uorder, int vorder, int worder, varray<double> Nu, varray<double> Nv, varray<double> Nw, varray<point3d> &bpt, double u, double v, double w);//计算体上的点
	//int Findnum(int left, int right, double x, varray<double> kont);
private:
	//多片体参数化模型存储
	varray<double>  u, v, w;                           //u,v,w三方向的节点矢量
	int u_num, v_num, w_num;                           //u,v,w三方向的节点个数
	int u_order, v_order, w_order;                     //u,v,w三个方向的阶数
	int Patchnum;                                    //体参数化模型的片数
	varray<point4d> m_allpoints;                     //单片控制点坐标
	varray<varray<point3d>> PatchControlPoints;      //所有控制点坐标

	varray<varray<varray<point3d>>> BoundaryCurve;   //边界点

	//任意平面切片
	float _A, _B, _C, _D,_begin,_end,_each;                                  //构建任意平面
	int _degree;   //总共切分了多少个平面
	vector<vector<float>> eachSurfacePoints;  //每一个切平面的所有点
	vector<vector<point3d>> eachSurfacePoints_point3d;

	//空间坐标转换
	double sinrx, cosrx, sinry, cosry;                  //转换夹角

	//切片截面存储
	varray<point3d> m_allIntersetPoints;
	varray<varray<point3d>> m_allSupportPoints;
	varray<point3d> SupportPointsCube;
	varray<varray<double>> Tran;

	//输出网格信息
	double Wdegree, Vdegree, Udegree;//细分精度
	varray<point3d> m_allcubepoints;
	int Cubenum;
};

