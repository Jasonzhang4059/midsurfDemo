#include "stdafx.h"
#include "PSOLCT.h"

psoLCT::psoLCT(const toric& mtoric, const int Eidx)
	:m_toric(mtoric)
{
	int nextE = Eidx + 1;
	if (nextE >= m_toric.m_ConerPts.size())nextE = 0;
	m_p0 = mtoric.m_ConerPts[Eidx];
	m_v0 = m_toric.m_ConerPts[nextE] - m_p0;
	m_len = m_v0.Magnitude();
	m_v0 = m_v0.Normalize();

	m_Dim = 1;
	m_Pmin.clear();
	m_Pmax.clear();
	m_Vmin.clear();
	m_Vmax.clear();

	m_Pmin.push_back(0);
	m_Pmax.push_back(1);

	m_Vmin.push_back(-1);
	m_Vmax.push_back(1);
}

void psoLCT::CalPSOLCT(point2d& para, double& dis)
{
	SetPSOPara(10, 50, 0.5, 0.5, 0.2, 0.00001);
	PSOUpdate();
	para = m_p0 + m_PGlobal[0] * m_len*m_v0;
	dis = m_DisGol;
}

double psoLCT::CalDis(const varray<double>& P)
{
	point2d res = m_p0 + P[0]*m_len*m_v0;
	point3d pt = m_toric.GetToricPts(res.x, res.y);
	double ang = pt.Angle(point3d(0, 0, 1));
	return ang;
}


psoCenLine::psoCenLine()
{
	m_Dim = 2;
}

void psoCenLine::CalPsoCenLine(const varray<point3d>& lineDu, point3d & Cenline, double & dis)
{
	m_lineDu = lineDu;
	m_Pmin.push_back(0);
	m_Pmin.push_back(0);
	m_Pmax.push_back(PI);
	m_Pmax.push_back(2 * PI);
	m_Vmin.push_back(-PI);
	m_Vmin.push_back(-2 * PI);
	m_Vmax.push_back(PI);
	m_Vmax.push_back(2 * PI);
	PSOUpdate();
	Cenline.x = sin(m_PGlobal[0])*cos(m_PGlobal[1]);
	Cenline.y = sin(m_PGlobal[0])*sin(m_PGlobal[1]);
	Cenline.z = cos(m_PGlobal[0]);
	dis = m_DisGol;
}

double psoCenLine::CalDis(const varray<double>& P)
{
	double avang = 0;
	point3d p;
	p.x = sin(P[0])*cos(P[1]);
	p.y = sin(P[0])*sin(P[1]);
	p.z = cos(P[0]);
	varray<double> ang;
	for (int i = 0; i < m_lineDu.size(); ++i)
	{
		ang.push_back(p.Angle(m_lineDu[i]));
		avang += ang[i];
	}
	avang /= ang.size();
	double res = 0;
	for (int i = 0; i < ang.size(); ++i)
		res += pow(ang[i] - avang, 2);
	return res;
}

psoPtsDis::psoPtsDis()
{
	m_Dim = 1;
}

void psoPtsDis::FindPtsWithDis(const NurbsLine line, const point3d P0, const double dis, double & u, double & err)
{
	m_line = line;
	m_P0 = P0;
	m_dis = dis;
	m_iteraNum = 200;
	m_PNum = 16;
	m_Pmin.push_back(0);
	m_Pmax.push_back(1);
	m_Vmin.push_back(-1);
	m_Vmax.push_back(1);
	PSOUpdate();
	u = m_PGlobal[0];
	err = m_DisGol;
}

double psoPtsDis::CalDis(const varray<double>& P)
{
	point3d pts = m_line.GetLinePoint(P[0]);
	double dp = (pts - m_P0).Magnitude();
	return abs(dp - m_dis);
}
