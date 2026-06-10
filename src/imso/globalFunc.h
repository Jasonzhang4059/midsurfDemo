//全局函数
#ifndef _globalFunc_H_
#define _globalFunc_H_

#include "varray.h"
#include "pointXd.h"
#include "../../3rdParty/eigen-3.3.8/Eigen/Dense"
#include <string>
#include "XVec.h"


using Eigen::Vector4d;
using Eigen::Matrix4d;
using std::string;

using namespace base;
/////////////////////////////////////////////何文彬    2018///////////////////////////////

template<class _T>
varray<_T> transVectorToVarray(const std::vector<_T> vector)
{
	varray<_T> varray;
	for (auto&i : vector) {
		varray.push_back(i);
	}
	return varray;
}


/*维度降一
hightDimVarray：高维varray
lowDimVarray：低维varray
clear：是否清除低维varray原有内容
*/
template<class _T>
void ReduceVarrayDim(const varray<varray<_T>>& hightDimVarray, varray<_T>& lowDimVarray, const bool clear)
{
	if (clear)
		lowDimVarray.clear();
	if (hightDimVarray.size() == 0)return;

	for (int i = 0; i < hightDimVarray.size(); ++i)
		for (int j = 0; j < hightDimVarray[i].size(); ++j)
			lowDimVarray.push_back(hightDimVarray[i][j]);
}

//二维数组转置
template<class _T>
void Transpose(varray<varray<_T>>& Varr)
{
	varray<varray<_T>> tvarr;
	tvarr.resize(Varr[0].size());
	for (int i = 0; i < Varr.size(); ++i)
		for (int j = 0; j < Varr[i].size(); ++j)
			tvarr[j].push_back(Varr[i][j]);
	Varr = tvarr;
}

/*二项式系数
Bin(x,y)=x!/(y!*(x-y)!)
*/
double Bin(const int x, const int y);

//显示点规范化至[a，b]
void Normalization(varray<point3d>& pts, const double a, const double b);

//显示点规范化至[a，b]
void Normalization(varray<varray<point3d>>& pts, const double a, const double b);

//计算点集中心
void GetCenter(varray<point3d>& pts, point3d &center);

//点集中心平移
//mode=1 vec为平移终点,此时需要传入点集中心center
//mode=0 vec为平移向量
template<class T>
void TransPts(varray<T> &pts, const T &vec, bool mode, const T &center = T())
{
	if (mode)
	{
		for (int i = 0; i < pts.size(); ++i)
			pts[i] = pts[i] + (vec - center);
	}
	else
	{
		for (int i = 0; i < pts.size(); ++i)
			pts[i] = pts[i] + vec;
	}
}

//点格式转换
//point4d转Vector4d
Vector4d P4dToV4d(const point4d P4d);

//点格式转换
//Vector4d转point4d
point4d V4dToP4d(const Vector4d V4d);

Vector4d P4dToV4d(const Vec4 P4d);


//Vector4d单位化
void Vector4dToUnit(Vector4d &v);

//Vector4d叉乘
void AcrossB(const Vector4d& a, const Vector4d& b, Vector4d& c);

//计算夹角
double angleAB(const Vector4d& a, const Vector4d& b);

//计算曲线长度,L为曲线坐标点
double CurveLength(const varray<point3d>& L);

//判断a是否在S内
template<class _T>
bool IsInSet(const _T a, const varray<_T>& S)
{
	for (int i = 0; i < S.size(); ++i)
	{
		if (a == S[i])
			return true;
	}
	return false;
}

//计算坐标系obj到target的过渡矩阵
//Coordinates：X轴，Y轴，Z轴, 坐标原点
void CalCoordinatesTransMat(const varray<point3d>& objCoordinates, const varray<point3d>& targetCoordinates, Matrix4d& mat);

//计算矢量变换矩阵
void CalTransMat(const Vector4d& targetPts, const Vector4d& targetVec,
	const Vector4d& objPts, const Vector4d& objVec, Matrix4d& mat);
void CalTransMat(const point4d& targetPts, const point4d& targetVec,
	const point4d& objPts, const point4d& objVec, Matrix4d& mat);
void CalTransMat(const Vec4& targetPts, const Vec4& targetVec,
	const Vec4& objPts, const Vec4& objVec, Matrix4d& mat);

//坐标变换
void TransByMat(varray<Vector4d>& varr, const Matrix4d& mat);
void TransByMat(varray<point4d>& varr, const Matrix4d& mat);
void TransByMat(varray<Vec4>& varr, const Matrix4d& mat);

//取符号函数
int Sgn(double a);

/*更改容器起点及方向
beginIdx:设置为起点元素的当前下标
dir：是否按当前顺序储存
*/
template<class _T>
void SetBeginDir(varray<_T>& varr, const size_t beginIdx, const bool dir = true)
{
	varray<_T> res;
	varray<_T>::iterator it0, it;
	it = it0 = varr.begin() + beginIdx;
	do
	{
		res.push_back(*it);
		if (dir)
		{
			++it;
			if (it == varr.end())
				it = varr.begin();
		}
		else
		{
			--it;
			if (it == varr.begin())
			{
				res.push_back(*it);
				it = varr.end() - 1;
			}
		}
	} while (it != it0);
	varr = res;
}

//最大公约数
int GCD(int a, int b);

//并集
template<class _T>
void KnotUnify(const varray<_T>& KnotsA, const varray<_T>& KnotsB, varray<_T>& NewKnots)
{
	NewKnots.clear();
	int Anum = 0, Bnum = 0;
	// 如果Anum或Bnum有一个遍历完了就停止
	while (Anum < KnotsA.size() && Bnum < KnotsB.size())
	{
		if (KnotsA[Anum] < KnotsB[Bnum])
		{
			NewKnots.push_back(KnotsA[Anum++]);
			continue;
		}
		else if (KnotsA[Anum] > KnotsB[Bnum])
		{
			NewKnots.push_back(KnotsB[Bnum++]);
			continue;
		}
		else
		{
			NewKnots.push_back(KnotsB[Bnum++]);
			Anum++;
		}
	}
	// 如果 Anum< KnotsA.size()，说明Bnum都添加进数组NewKnots了，只剩下比Bnum大的Anum，逐个添加
	while (Anum < KnotsA.size())
	{
		NewKnots.push_back(KnotsA[Anum++]);
	}
	while (Bnum < KnotsB.size())
	{
		NewKnots.push_back(KnotsB[Bnum++]);
	}
}
//差集
//diffKnots=KnotsL-KnotsS
//KnotsL大小不小于KnotsS
template<class _T>
void KnotsDiff(const varray<_T>& KnotsL, const varray<_T>& KnotsS, varray<_T>& diffKnots)
{
	diffKnots.clear();
	if (KnotsL.size() <= KnotsS.size())
		return;

	int Ln = 0, Sn = 0;
	while (Ln < KnotsL.size() && Sn < KnotsS.size())
	{
		if (KnotsS[Sn] != KnotsL[Ln])
			diffKnots.push_back(KnotsL[Ln]);
		else
			++Sn;
		++Ln;
	}
	while (Ln < KnotsL.size())
	{
		diffKnots.push_back(KnotsL[Ln++]);
	}
}

//点pts沿向量v1投影至平面(v,p)
//v1默认为0向量，此时为正投影，相当于v1 = -v
//平行返回0,否则返回1
int Project2Plane(const point3d& v, const point3d& p, const point3d& pts, point3d& res, const point3d& v1 = point3d());

//点集Cpts沿向量v1投影至平面(v,p)
//v1默认为0向量，此时为正投影，相当于v1 = -v
int Project2Plane(const point3d & v, const point3d & p,
	const varray<point4d>& Cpts, varray<point4d>& ProPts, const point3d & v1 = point3d());

#endif