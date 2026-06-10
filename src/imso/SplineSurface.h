#pragma once

//#include "XFunction.h"
#include "spline.h"
#include "XBaseMesh.h"
#include "stdafx.h"
using namespace base;

namespace base {

class SplineSurface:public SplineBase
{
public:	
	enum SPLINESURFACE_MODE {
		SURFACEMODE_BSPLINE = 0,
		SURFACEMODE_BZIER,
		SURFACEMODE_NONUNI_BSPLINE,
	};
public:
	int					  m_uDegree;
	int					  m_vDegree;
	varray<Spline>        m_4sideSpline;  //four sided spline indexed u0,v0,u1,v1; 四条边界线首尾相接。

public:
    varray<XBaseMesh>     m_vShowMeshes;
	varray<varray<Vec4>>  m_UVCtrlPts; //set all the control points.
	varray<Vec4>		  m_CtrlPts;
    varray<double>        m_uKnots;
	varray<double>        m_vKnots;
	int					  m_uNum;
	int					  m_vNum;
private:
    DWORD                 m_iSurfaceMode;

public:
	SplineSurface(void);
	SplineSurface(const SplineSurface& surface);
	SplineSurface& operator = (const SplineSurface& src);
	enum{ CLSID = 0xd4fe5339 };
	virtual DWORD ClassID() const  {return (DWORD)CLSID;}
	SplineSurface* SplineSurface::CreateMe() const;
public:
	~SplineSurface(void);
public:
	void AddCtrlPts(varray<varray<Vec4> >& CtrlPts);  //访问按照ij序号来，i表示V向行号，j表示U向j列。
	void AddKnotsVector(varray<double>& knots,bool uvflag);
	void AddKnotsVector(varray<float>& knots,bool uvflag);
	void ChangeSurfaceMode(DWORD iMode);
	void SetSurfaceDegree(int uDegree,int vDegree);
	void Generate4SidedBoudaryCurve();
	Vec4 GetUVPoint(float u,float v);
	void CreateShowMesh(bool bClear= false);
	void CopySurfaceWithoutMesh(SplineSurface& sp);
	void SetSurface(const int uDegree, const int vDegree, const int uNum, const int vNum,
		const varray<double>& uKnots, const varray<double>& vKnots);

	//计算（u，v）对应的曲面上的点
	Vec3 GetSurFacePoint(const double u, const double v)const;

	//计算四边形面片显示数据
	//num:该方向采样点数量
	//quads:面片数据
	//lines:等参线数据
	threadParamSpline CalQuads(const int Unum, const int Vnum)const;

	/*Coons插值
	EndgCtrlPts:边界控制点（e0,e1,e2,e3顺序）*/
	void CoonsInterpolate(const varray<varray<Vec4>>& EdgeCtrlPts);

	/*Coons插值
	EdgeLines:边界曲线（e0,e1,e2,e3顺序）*/
	void CoonsInterpolate(const varray<Spline>& EdgeLines);

	//曲面升阶
	//Udegree,Vdegree:升阶后次数
	void DegreeElevate(const int Udegree, const int Vdegree);

	//曲面节点插入
	//Uknot,Vknot:需要插入的节点
	void KnotsRefine(const varray<double>& Uknot, const varray<double>& Vknot);

	void KnotsRefineNum(int refinenum);

	//曲面分段为Bezier曲面
	//dir:0=U方向，1=V方向
	//QW：Bezier曲面控制点
	void Decompose(const bool dir, varray<varray<varray<Vec4>>>& QW);

	//根据控制点二维序号计算一维序号
	int CtrlPtsIdx(const int uIdx, const int vIdx);

	//控制点排序转换为U-V
	void OrderCtrlPts();

	//控制点排序转换为U-V
	void OrderCtrlPts(SplineSurface& sf);

	//提取边界线,排序已经是U-V情况下
	void GetEdgeLines(varray<Spline>& EdgeLines);

//display
    /*void DisplaySplineSurface(int renderMode,bool bShowBoundary = false);
	void DrawSurfaceCtrlPts(COLORREF clr);*/
	void Clear();


public:
	Vec4 GetCtrlPt(int vi,int uj) const {return m_UVCtrlPts.at(vi).at(uj);}
	void SetCtrlPt(int vi,int uj,Vec4 vt)  {m_UVCtrlPts.at(vi).at(uj) = vt;}
	int  GetUCtrlPtNum(){return m_uNum;}
	int  GetVCtrlPtNum(){return m_vNum;}
    int  GetUDegree(){return m_uDegree;}
	int  GetVDegree(){return m_vDegree;}
//////////////////////////////////////////////////////////////////////
public:   //以下函数主要用于polyIGA中的曲面排序。
	bool IstwoSurfaceSame(SplineSurface& sf, bool notConsiderUVDirection = true);
	int  RotateTheWholeSurfaceAccordingtoBasesurface(int istate); //同时返回一个值，该值反映输入面和基准面相对位置，以0,1,2,3位置排列。
	void RotateBaseSurface();  
	int  RotateLastSuface(int istate);  //旋转封顶曲面。要求istate是该面和u0面的相对位置，此时u0面已经旋转到位。因此需要特殊处理。
	bool IsTwoVec4arraySameinOrder(const varray<Vec4>& arr1, const varray<Vec4>& arr2);
	int  GetTwoSurfaceEdgeshareState(const SplineSurface& sf);
	int  GetTwoSurfaceEdgeshareStateNew(const SplineSurface& sf);
	void RotateControlPoints(int imode);
	//以上函数主要用于polyIGA中的曲面排序。
/////////////////////////////////////////////////////////////////////
//B样条曲面操作。
public:
	void CoonsInterpolate(const varray<varray<Vec4>>& EdgeCtrlPts, varray<varray<Vec4>>& coonsPatchCtrlpts);
	float GetDistanceBetweenTwoOrderSurface(SplineSurface& sf);
	float GetDistanceBetweenTwoSurface(SplineSurface& sf);
};
}