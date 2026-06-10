#pragma once
//toric曲面
//USST 何文彬 2018/11

#include "varray.h"
#include "pointXd.h"

class toric
{
public:
	varray<point2d> m_ConerPts;//二维参数域的顶点,所有顶点坐标应大于等于0,按逆时针排列
	varray<varray<double>> m_cm;//格点对应的基函数系数，大于0。如果需要为贝塞尔曲面，则取二项式系数
	varray<varray<point4d>> m_CtrlPts;//控制点

	//以下成员由构造函数内完成
	varray<varray<point2d>> m_LatticePts;//二维参数域格点
	int m_LineN;//参数域边的数量
	varray<varray<int>> m_BoudaryLine;//参数域边界直线方程参数h(u,v)=au+bv+c=0
	int m_xmin, m_xmax, m_ymin, m_ymax;//参数域极值

public:
	//默认构造函数，仅用于分配空间
	toric();

	//构造函数
	toric(const varray<point2d>& ConerPts);

	//构造函数
	//ConerPts：二维参数域的顶点，所有顶点坐标应大于等于0,按逆时针排列
	//CtrlPts：控制点
	//cm：格点对应的基函数系数，如果需要为贝塞尔曲面，则取二项式系数
	toric(const varray<point2d>& ConerPts, const varray<varray<point4d>>& CtrlPts, const varray<varray<double>>& cm);

	//析构函数
	~toric();

	//计算Toric曲面点
	point3d GetToricPts(const double u, const double v)const;

	//计算等参线
	void CalSurfacePts(const int UptsNum, const int VptsNum, varray<varray<point3d>>& uPts, varray<varray<point3d>>& vPts);

	//计算边界曲线
	void CalBondaryPts(const int ptsNum, varray<varray<point3d>>& BondaryPts);

	//判断是否为toric边界格点, 是则返回边界号，否则返回-1
	int InEdge(const point2d &p);

	//判断参数是否合法
	bool IsInSF(const double u, const double v);

	//COONS插值理论进行曲面插值
	//BondaryCtrlPts：边界曲线控制点，按toric参数域方向顺序排列
	void CoonsToric(const varray<varray<point4d>>& BondaryCtrlPts);

protected:
	varray<varray<point4d>> m_BondaryCtrlPts;//边界曲线控制点，按toric参数域方向顺序排列

	//Coons直纹面,混合曲面控制点
	void CoonsS12TCtrlPts(varray<varray<point4d>>& UCtrlPts, varray<varray<point4d>>& VCtrlPts, varray<varray<point4d>>& TCtrlPts);

	//曲线首点法矢
	//center：结点坐标
	//point3d FirstDir(const varray<point4d>& cur, const point3d center);

	//寻找等值线与边界交点
	//val:等值线的值
	//UorV:true为U等值线，false为V等值线
	//startVal：等值线与首条边界的交点
	//endVal：等值线与最后一条边界的交点
	//void FindStartAndEndVal(const double val, const bool UorV, double& startVal, double& endVal);


	//根据格点参数域计算参数域边界方程
	void CalBoudaryLine();

	//计算参数域格点
	void CalParaLattice();

	//计算(u,v)对应的二元边界方程值
	double CalhiVal(const double u, const double v, const int lineIdx)const;
	double CalhiVal(const point2d& m, const int lineIdx)const;

	//计算(u,v)对应的基函数值
	/*m：参数域格点
	cm：基函数系数
	*/
	double ToricBasic(const double u, const double v, const point2d& m, const double cm)const;

	//参数域边界方程非负性校正
	void LineNonNeg(varray<int>& line, int idx);

	//(u,v)处格点m对应的基函数一阶导数
	//dir：true表示对U求导，false表示对V求导
	double CalhiDer1(const double u, const double v, const point2d& m, const double cm, const bool dir);

	//(u,v)处格点m对应的基函数二阶导数
	//dir：【0，1，2，3】=【uu,vv,uv,vu】
	double CalhiDer2(const double u, const double v, const point2d& m, const double cm, const int dir);

public:
	//参数域U等值线的端点,返回端点在m_LatticePts的坐标
	void FindEndpointV(const point2d& pts, point2d& stIdx, point2d& edIdx);

	//toric分子式
	point3d CalM(const double u, const double v);

	//toric分子式偏导
	//dir：true表示对U求导，false表示对V求导
	point3d CalMDir1(const double u, const double v, const bool dir);

	//(u,v)处一阶导数
	//dir：true表示对U求导，false表示对V求导
	point3d CalPtsDir1(const double u, const double v, const bool dir);

	//(u,v)处二阶导数
	//dir：【0，1，2，3】=【uu,vv,uv,vu】
	point3d CalPtsDir2(const double u, const double v, const int dir);
};
