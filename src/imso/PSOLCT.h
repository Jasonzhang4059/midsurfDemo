
#pragma once
#include "PSO.h"
#include "pointXd.h"
#include "CToric.h"
#include "CNurbs.h"

//PSO计算直线与toric曲面边界交点
class psoLCT :public PSO
{
public:
	psoLCT(const toric& mtoric,const int Eidx);

	virtual ~psoLCT() {}

	//PSO计算直线与toric曲面交点的参数域值，及其适应度
	void CalPSOLCT(point2d& para,double& dis);

private:
	point2d m_p0;//边界直线起点
	point2d m_v0;//边界直线方向向量
	toric m_toric;
	double m_len;

	//重写目标函数
	double CalDis(const varray<double>& P);
};

//计算与向量组的角平分线
class psoCenLine :public PSO
{
public:
	psoCenLine();

	virtual ~psoCenLine(){}

	//计算与向量组的角平分线
	void CalPsoCenLine(const varray<point3d>& lineDu, point3d& Cenline, double& dis);

private:
	varray<point3d> m_lineDu;

	//重写目标函数
	double CalDis(const varray<double>& P);
};

class psoPtsDis :public PSO
{
public:
	psoPtsDis();
	virtual ~psoPtsDis() {}

	//曲线上与点P0距离为dis的点
	void FindPtsWithDis(const NurbsLine line, const point3d P0, const double dis, double & u,double& err);

private:
	NurbsLine m_line;
	point3d m_P0;
	double m_dis;
	//重写目标函数
	double CalDis(const varray<double>& P);
};