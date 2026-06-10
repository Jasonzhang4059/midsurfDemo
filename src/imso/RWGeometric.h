#pragma once
#include "CNurbs.h"
#include <string>
#include"spline.h"
#include"SplineSurface.h"
#include"SplineVolume.h"
#include"XVec.h"

#include "Nurbs.h"

using namespace std;
class RWGeometric
{
public:
	//读取3d离散点
	int ReadPoint(const string& path, varray<varray<point3d>>& allPts);

	//读取4d离散点
	int ReadPoint(const string& path, varray<varray<point4d>>& allPts);

	//读取Bezier曲线
	//返回曲线数量
	int ReadBezierLine(const string& path, varray<BezierLine>& lines);

	//读取NURBS曲线
	//返回曲线数量
	int ReadNurbsLine(const string& path, varray<NurbsLine>& lines);

	//读取NURBS曲面
	//返回曲面数量
	int ReadNurbsSurface(const string& path, varray<NurbsSurface>& surfaces);

	//读取NURBS体
	//返回体模型数量
	int ReadNurbsVol(const string& path, varray<NurbsVol>& vols);

	int ReadNurbsVol(const string& path, varray<YN::NurbsVol>& vols);

	//读取3d离散点
	int WritePoint(const string& path, const varray<varray<point3d>>& allPts);

	//读取4d离散点
	int WritePoint(const string& path, const varray<varray<point4d>>& allPts);
	
	//读取4d离散点(Vec4类型)
	int WritePoint(const string & path, const varray<varray<Vec4>>& allPts);

	//写出NURBS曲线
	//返回写出的曲线数量
	int WriteBezierLine(const string& path, const varray<BezierLine>& lines);

	//写出NURBS曲线
	//返回写出的曲线数量
	int WriteNurbsLine(const string& path, const varray<NurbsLine>& lines);

	//写出NURBS曲面
	//返回写出的曲面数量
	int WriteNurbsSurface(const string& path, const varray<NurbsSurface>& surfaces);

	//写出NURBS体
	//返回写出的体模型数量
	int WriteNurbsVol(const string& path, const varray<NurbsVol>& vols);

	//-------------------------------------------------------------//
		//读取NURBS曲线
	//返回曲线数量
	int ReadSpline(const string& path, varray<Spline>& lines);

	//读取NURBS曲面
	//返回曲面数量
	int ReadSplineSurface(const string& path, varray<SplineSurface>& surfaces);

	//读取NURBS体
	//返回体模型数量
	int ReadSplineVolume(const string& path, varray<SplineVolume>& vols);

	//写出NURBS曲线
	//返回写出的曲线数量
	int WriteSpline(const string& path, const varray<Spline>& lines);

	//写出NURBS曲面
	//返回写出的曲面数量
	int WriteSplineSurface(const string& path, const varray<SplineSurface>& surfaces);

	//写出NURBS体
	//返回写出的体模型数量
	int WriteSplineVolume(const string& path, const varray<SplineVolume>& vols);


};