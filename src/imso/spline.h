// Spline.h: Spline 僋儔僗偺僀儞僞乕僼僃僀僗
//
//////////////////////////////////////////////////////////////////////

#ifndef _SPLINE_H__INCLUDED_
#define _SPLINE_H__INCLUDED_

#include <Windows.h>
#include "varray.h"
#include "XVec.h"
#include "MathUtil.h"

using namespace std;
using namespace base;
namespace base {
//////////////////////////////////
	class SplineBase
	{
	public:

		/*计算节点下标
		x：节点值
		degree：次数
		CtrlPtsNum：控制点数量
		knots：节点矢量*/
		int FindSpan(const double x, const int degree, const int CtrlPtsNum, const varray<double>& knots)const;

		/*根据参数值，节点下标，计算基函数
		u：参数值
		k：节点下标
		degree：次数
		knots：节点矢量
		N：返回的基函数*/
		void BasisFuns(const double u, const int k, const int degree, const varray<double>& knots,
			varray<double>& N)const;

		/*u处所有基函数
		u：参数值
		k：节点下标
		degree：次数
		knots：节点矢量
		ndu：返回的所有基函数*/
		void AllBasisFuns(const double u, const int k, const int degree,
			const varray<double>& knots, varray<varray<double>>& ndu)const;

		/*基函数n阶导
		u：参数值
		k：节点下标
		degree：次数
		n：导矢阶数
		knots：节点矢量
		basisDu：基函数n阶导*/
		void DerBasisFuns(const double u, const int k, const int degree, const int n, const varray<double>& knots,
			varray<varray<double>>& basisDu)const;
	};
class  Spline: public SplineBase
{
public:
	varray<Vec4> m_CtrlPts;
	DWORD m_mode;
	int m_Degree;
	varray<double> m_Knots;
public:
	
	enum SPLINE_MODE {
		//the segment between two control points in a spline with the following mode is decided by the two control points and its tangent.
		SPLMODE_LINER = 0,
        //t = [0,1] for the whole spline. 
        SPLMODE_BEZIER,          
        SPLMODE_CLOSED_BEZIER,	//not realized.
        //Spline. t = [0,1] for the every segment between two control points of the whole spline.
		SPLMODE_SPLINE,         
        SPLMODE_CLOSED_SPLINE,		
		//uniform Bspline.
		SPLMODE_2BSPLINE,
		SPLMODE_CLOSED_2BSPLINE,
        SPLMODE_3BSPLINE,
        SPLMODE_CLOSED_3BSPLINE,
		//rational Bspline
        SPLMODE_NBSPLINE,  //n = 2,3,4,......  N次均匀有理B样条   
		SPLMODE_CLOSED_NBSPLINE,  //封闭均匀有理B样条
		//non-uniform Bspline
        SPLMODE_NONUNI_NBSPLINE,
	};
public:
	bool				m_bSplineIsEdit;
	int					m_nSelected;
	int					m_nMaped;
	float				m_yCrossSection;
protected:
	int				m_bSplineGenerateMode;	//generate mode: 0:normal mode; 1:giving tangent of the control point directly for spline;2 for boundary spline					
	varray<Vec4>	m_vTangent;				//arry for tangent
	varray<float>   m_vTangentMagnitude0;   //the tangent of each control point i have two magnitude. magnitude0 for (i-1, i) and magnitude1 for(i, i+1)
	varray<float>   m_vTangentMagnitude1;   //		
	varray<Vec4>    m_RenderPt;    //点用于显示。
	
		
public:
	/*计算u节点对应的曲线坐标点
		u：节点参数*/
	Vec3 GetLinePoint(const double u)const;

	/*获取线上的点
	Unum：曲线点数量
	linePts：曲线点	*/
	void CalLinePoint(const int Unum, varray<Vec3>& linePts)const;

	/*曲线矢值函数A(u)所有p阶导，矢值函数A(u)即NURBS曲线分子式
	u：参数
	n：导矢阶数
	Der：A(u)所有p阶导*/
	void PtsDerivsAu(const double u, const int n, varray<Vec4>& Der)const;

	/*曲线上u处所有n阶导
	u：参数
	n：导矢阶数
	Der：所有n阶导 */
	void PtsDerivs(const double u, const int n, varray<Vec4>& Der)const;

	/*曲线所有n阶导
	step：参数域等分步进量
	Der：曲线点对应的n阶导*/
	void CurveDerivs(const int n, const double step, varray<varray<Vec3>>& Der);

	/*节点插入
	u：需要插入的节点
	r：插入次数*/
	void KnotInsert(const double u, const int r);

	/*节点去除
	u：需要删除的节点
	r：删除次数*/
	int KnotRemove(const double u, const int r);

	/*节点细化
	u:需要插入的节点*/
	void KnotsRefine(const varray<double>& u);

	/* 曲线升阶
	degree：升阶后次数*/
	void DegreeElevate(const int degree);

	//在节点u处对曲线进行分割
	void Segmentation(const double u, varray<Spline>& lines);

	/*NURBS曲线分解为BEZIER
	Qw：输出的BEZIER曲线段控制点*/
	void Decompose(varray<varray<Vec4>>& Qw);



public:
	//曲线生成
	Spline();
	Spline(const Spline& spl);
	virtual ~Spline();
	enum{ CLSID = 0xd4fe5009 };

	virtual DWORD ClassID() const  {return (DWORD)CLSID;}
	Spline* Spline::CreateMe() const;//把这个曲线复制一个出来
	void ClearStatus();//清理函数
    
	//曲线的存储和读取
	bool Save(BaseOStream& bs) const;
	bool Load(BaseIStream& bs);	
	bool Save0010(BaseOStream& bs) const;
	bool Load0010(BaseIStream& bs,float fCurVersion);  
	//bool Load0010(FILE* fp,float fCurVersion);  //just  version 1.005   没有实现
	
    //B样条曲线拟合
	void    FitSpline(varray<Vec4>& inputVecs,varray<float>& knots, int degreeNum,DWORD splineMode = SPLMODE_NBSPLINE);
	//设置和获取操作
	void	SetTangentOfCtrlPnt(Vec4& vTangent, int i);
	void	SetSplineGenerateMd(int iMode);	
	void	SetMagnitudeOfTangent(int i, int j, float fMagnitude); //get the magnitude of control point, 20050125	
    void    SetCtrlPoint(int index,const Vec4& v);
	void    SetCtrlPointArr(const varray<Vec4>& ptsArr)    { m_CtrlPts = ptsArr;}
	void    SetSplineDegree(int n = 3);
	void    SetSplineKnot(const varray<double>& knots){m_Knots = knots;}

    DWORD   GetMode() const {return m_mode;}   
	int     GetSplineDegree() const {return m_Degree;}
	Vec4&   GetCtrlPoint(int i) {return m_CtrlPts[i];}
	const Vec4& GetCtrlPoint(int i) const {return m_CtrlPts[i];}
	int     GetCtrlPointCount() const {return static_cast<int>(m_CtrlPts.size());}
	Vec4    GetPoint(int i, double t, float tension=0.0);
	float   GetLength(int n);
	Vec4			GetTangentOfCtrlPnt(int i);
	varray<Vec4>&   GetCtrlPointArr()     {return m_CtrlPts;}
	int				GetSplineGenerateMd(void)const;
	int				GetTangentCount() const { return static_cast<int>(m_vTangent.size()); }
	float			GetMagnitudeOfTangent(int i, int j);
	void CruveReverse();//曲线反转
	void CreatLineWithTwoPoints(const Vec3 &p1, const Vec3 &p2, int degree = 2);

    //控制点的一些其他操作
	void    AddTangent(Vec4 vTangent, float fMagnitude0, float fMagnitude1);
	void    ChangeMode(DWORD mode,int degree = 0);		
	void    AddCtrlPoint(const Vec4& v);
	void    AddKnots(const double knot);
	void    DelCtrlPoint(int index);    
	void    InsertCtrlPoint(int idx,Vec4 vt);	
	void    ClearCtrlPoint();  	

	void    resize(int sz) {m_CtrlPts.resize(sz);}
	int     size() const {return static_cast<int>(m_CtrlPts.size());}
	Vec4&   p(int i) {return m_CtrlPts[i];}//返回控制点的第i个点
	const Vec4& p(int i) const {return m_CtrlPts[i];}
	Vec4&   pBack() {return m_CtrlPts.back();}
	const Vec4&  pBack()const {return m_CtrlPts.back();} 
};
}
#endif
