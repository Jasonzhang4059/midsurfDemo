#pragma once
#include<iostream>
#include <vector>
#include"sisl.h"
#include "CNurbs.h"
#include "SplineVolume.h"

//三维点坐标转化为SISL格式点
double* Point3dToSislPoint(const Vec3 & inputpoint, int length = 3);

//NURBS曲线转化为SISL格式曲线
SISLCurve* NurbsLineToSislLine(const Spline &inputLine);
SISLCurve* NurbsLineToSislLine(const Spline &inputLine, int dim);

//sisl曲线转cnurbs格式
Spline SislLineToNurbsLine(SISLCurve* cruve);

//曲线参数u处点坐标及切向量
//返回曲线维度(如果求解或失败曲线为空返回-1)
//u:曲线参数值
//derive:各阶切向量数组
//der:求导次数
int CalLinePoint(SISLCurve *&curve, double u, std::vector<std::vector<double>> &mderive, int der = 1);

//一点和一Nurbs曲线求交,返回曲线上的u值
double PointIntersectNurbsLine(double* &pt, SISLCurve* &pc, int length = 3);


//仅判定两NURBS曲线是否存在交点
bool ISTwoNurbsLineIntersect(SISLCurve *&curve1, SISLCurve *&curve2);

bool ISTwoNurbsLineIntersect(const Spline &line1, const Spline &line2, int dim = 2);

//两Nurbs曲线求交，各自返回曲线上的u值
int TwoNurbsLineIntersect(SISLCurve *&curve1, SISLCurve *&curve2, double *&intpar1, double *&intpar2);

//两Nurbs曲线求交,返回求交情况
//各自返回曲线上的u值
//返回相交点个数及重合曲线数目
int TwoNurbsLineIntersectVer2(SISLCurve *&curve1, SISLCurve *&curve2, double *&intpar1, double *&intpar2, int &numintpt, int &numintcu);

//直线与Spline求交
int StraLineIntersectNurbsLine(SISLCurve *&curve, const varray<Vec3> &strLine, double *&intpar, int &numintpt, int &numintcu);

//判定一交点是否符合CNurbs中曲线的要求(SISL曲线Nurbs情况有差异),此函数暂时不需使用
bool isInterPoint(double &u,SISLCurve * curve, const Spline* nurbsCurve);

//B样条拟合
SISLCurve* FitBspline(const varray<Vec3>&p, int degree, int dim = 3);
Spline FitBsplineCnurbs(const varray<Vec3>&p, int degree, int dim = 3);

class NurbsTrans
{
public:
	//Spline类型和Cnurbs类型格式转换
	static Spline CnurbslineToSpline(const NurbsLine& nl);
	static NurbsLine SplineToCnurbsline(const Spline& sl);
	static varray<Spline> ClinesToSplines(const varray<NurbsLine>& nls);
	static varray<NurbsLine> SplinesToClines(const varray<Spline>& sls);
	static SplineSurface CnurbssurfToSplinesurf(const NurbsSurface& nsf);
	static NurbsSurface SplinesurfToCnurbssurf(const SplineSurface& sf);
	static varray<SplineSurface> CsurfsToSplinesurfs(const varray<NurbsSurface>& nsfs);
	static varray<NurbsSurface> SplinesurfsToCsurfs(const varray<SplineSurface>& sfs);
	static SplineVolume CnurbsvolToSplinevol(const NurbsVol& nvol);
	static NurbsVol SplinevolToCnurbsvol(const SplineVolume& vol);
	static varray<SplineVolume> CvolsToSplinevols(const varray<NurbsVol>& nvols);
	static varray<NurbsVol> SplinevolsToCvols(const varray<SplineVolume>& vols);

	//部分曲线操作
	//曲线降维
	//mode:0表示清空原容器，1表示不清空
	static void DimReduceNurbsLines(varray<Spline>& nl, varray<varray<Spline>>& allL, int mode = 0);

	//曲面降维
	//mode:0表示清空原容器，1表示不清空
	static void DimReduceNurbsSurfs(varray<SplineSurface>& nsf, varray<varray<SplineSurface>>& allsf, int mode = 0);

};