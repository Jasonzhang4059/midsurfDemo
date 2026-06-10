#include "stdafx.h"
#include "CToric.h"
#include "globalFunc.h"
#include <cmath>

//默认构造函数，仅用于分配空间
toric::toric()
{
}

//默认构造函数
toric::toric(const varray<point2d>& ConerPts)
{
	m_ConerPts = ConerPts;
	m_LineN = m_ConerPts.size();
	m_xmin = m_ConerPts[0].x, m_xmax = m_ConerPts[0].x, m_ymin = m_ConerPts[0].y, m_ymax = m_ConerPts[0].y;
	for (int i = 1; i < m_ConerPts.size(); ++i)
	{
		if (m_ConerPts[i].x < m_xmin)
			m_xmin = m_ConerPts[i].x;
		else if (m_ConerPts[i].x > m_xmax)
			m_xmax = m_ConerPts[i].x;

		if (m_ConerPts[i].y < m_ymin)
			m_ymin = m_ConerPts[i].y;
		else if (m_ConerPts[i].y > m_ymax)
			m_ymax = m_ConerPts[i].y;
	}
	CalBoudaryLine();
	CalParaLattice();
	m_cm.clear();
	m_cm.resize(m_LatticePts.size());
	for (int i = 0; i < m_LatticePts.size(); ++i)
	{
		m_cm[i].resize(m_LatticePts[i].size());
		for (int j = 0; j < m_LatticePts[i].size(); ++j)
		{
			m_cm[i][j] = 1;
		}
	}
}

//ConerPts：二维参数域的顶点，所有顶点坐标应大于等于0,按逆时针排列
//CtrlPts：控制点
//cm：格点对应的基函数系数，如果需要为贝塞尔曲面，则取二项式系数
toric::toric(const varray<point2d>& ConerPts, const varray<varray<point4d>>& CtrlPts, const varray<varray<double>>& cm)
{
	m_ConerPts = ConerPts;
	m_cm = cm;
	m_CtrlPts = CtrlPts;
	m_LineN = m_ConerPts.size();

	m_xmin = m_ConerPts[0].x, m_xmax = m_ConerPts[0].x, m_ymin = m_ConerPts[0].y, m_ymax = m_ConerPts[0].y;
	for (int i = 1; i < m_ConerPts.size(); ++i)
	{
		if (m_ConerPts[i].x < m_xmin)
			m_xmin = m_ConerPts[i].x;
		else if (m_ConerPts[i].x > m_xmax)
			m_xmax = m_ConerPts[i].x;

		if (m_ConerPts[i].y < m_ymin)
			m_ymin = m_ConerPts[i].y;
		else if (m_ConerPts[i].y > m_ymax)
			m_ymax = m_ConerPts[i].y;
	}

	CalBoudaryLine();
	CalParaLattice();
}

//析构函数
toric::~toric()
{
}

//计算Toric曲面点
point3d toric::GetToricPts(const double u, const double v)const
{
	point3d sumPts;
	double sw = 0;
	double t1 = u, t2 = v;

	if (t1 > m_xmax)t1 = m_xmax;
	else if (t1 < m_xmin)t1 = m_xmin;

	if (t2 > m_ymax)t2 = m_ymax;
	else if (t2 < m_ymin)t2 = m_ymin;

	for (int i = 0; i < m_LatticePts.size(); ++i)
	{
		for (int j = 0; j < m_LatticePts[i].size(); ++j)
		{
			double t = m_CtrlPts[i][j].w*ToricBasic(t1, t2, m_LatticePts[i][j], m_cm[i][j]);
			sumPts += t*m_CtrlPts[i][j];
			sw += t;
		}
	}
	return sumPts / sw;
}

//计算等参线
void toric::CalSurfacePts(const int UptsNum, const int VptsNum, varray<varray<point3d>>& uPts, varray<varray<point3d>>& vPts)
{
	uPts.clear();
	vPts.clear();
	double ustep = 1.0*(m_xmax - m_xmin) / UptsNum;
	double vstep = 1.0*(m_ymax - m_ymin) / VptsNum;

	//U方向等参线
	for (double v = m_ymin; v < m_ymax + vstep; v += vstep)
	{
		if (v > m_ymax)v = m_ymax;
		varray<point3d> line;
		for (double u = m_xmin; u < m_xmax + ustep; u += ustep)
		{
			if (u > m_xmax)u = m_xmax;
			if (!IsInSF(u, v))
				continue;

			point3d pts = GetToricPts(u, v);
			line.push_back(pts);
		}
		uPts.push_back(line);
	}

	//V方向等参线
	for (double u = m_xmin; u < m_xmax + ustep; u += ustep)
	{
		varray<point3d> line;
		for (double v = m_ymin; v < m_ymax + vstep; v += vstep)
		{
			if (!IsInSF(u, v))
				continue;
			point3d pts = GetToricPts(u, v);
			line.push_back(pts);
		}
		vPts.push_back(line);
	}
}

//计算边界曲线
void toric::CalBondaryPts(const int ptsNum, varray<varray<point3d>>& BondaryPts)
{
	BondaryPts.clear();

	for (int i = 0; i < m_LineN; ++i)
	{
		varray<point3d> edge;
		point2d st = m_ConerPts[i];
		point2d ed = m_ConerPts[(i + 1) % m_ConerPts.size()];
		point2d dp = (ed - st) / (ptsNum - 1);
		for (int i = 0; i < ptsNum; ++i)
		{
			point2d para = st + i*dp;
			point3d pts = GetToricPts(para.x, para.y);
			edge.push_back(pts);
		}
		BondaryPts.push_back(edge);
	}
}

//根据格点参数域计算参数域边界方程h(u,v)=au+bv+c
//paraBoudary:参数域的边界格点
//linePara:边界直线方程参数
void toric::CalBoudaryLine()
{
	m_BoudaryLine.clear();
	for (int i = 0; i < m_LineN; ++i)
	{
		point2d st = m_ConerPts[i],
			ed = m_ConerPts[(i + 1) % m_LineN],
			dp = ed - st;
		varray<int> line(3, 0);
		if (dp.x == 0)
		{
			line[0] = 1;
			line[2] = -st.x;
			LineNonNeg(line, i);
			m_BoudaryLine.push_back(line);
			continue;
		}
		else if (dp.y == 0)
		{
			line[1] = 1;
			line[2] = -st.y;
			LineNonNeg(line, i);
			m_BoudaryLine.push_back(line);
			continue;
		}
		else
		{
			int dy = dp.y,
				dx = -dp.x;
			int gcd = GCD(dy, dx);
			line[0] = dy / gcd;
			line[1] = dx / gcd;
			line[2] = -(line[0] * st.x + line[1] * st.y);
			LineNonNeg(line, i);
			m_BoudaryLine.push_back(line);
			continue;
		}
	}
}

//计算参数域格点
void toric::CalParaLattice()
{
	m_LatticePts.clear();
	for (int y = m_ymin; y <= m_ymax; ++y)
	{
		varray<point2d> xVec;
		for (int x = m_xmin; x <= m_xmax; ++x)
		{
			if (!IsInSF(x, y))
				continue;
			xVec.push_back(point2d(x, y));
		}
		m_LatticePts.push_back(xVec);
	}
}

//计算(u,v)对应的二元边界方程值
inline
double toric::CalhiVal(const double u, const double v, const int lineIdx)const
{
	double res = m_BoudaryLine[lineIdx][0] * u + m_BoudaryLine[lineIdx][1] * v + m_BoudaryLine[lineIdx][2];
	return res;
}

inline
double toric::CalhiVal(const point2d& m, const int lineIdx)const
{
	double res = m_BoudaryLine[lineIdx][0] * m.x + m_BoudaryLine[lineIdx][1] * m.y + m_BoudaryLine[lineIdx][2];
	return res;
}

//计算(u,v)对应的基函数值
/*m：参数域格点
cm：基函数系数
*/
double toric::ToricBasic(const double u, const double v, const point2d& m, const double cm)const
{
	double tol = cm;
	for (int i = 0; i < m_BoudaryLine.size(); ++i)
	{
		tol *= pow(CalhiVal(u, v, i), CalhiVal(m, i));
		if (tol == 0)
			break;
	}
	return tol;
}

//判断参数是否合法
bool toric::IsInSF(const double u, const double v)
{
	for (int i = 0; i < m_LineN; ++i)
	{
		double d = m_BoudaryLine[i][0] * u + m_BoudaryLine[i][1] * v + m_BoudaryLine[i][2];
		if (d < 0)
			return false;
	}
	return true;
}

//COONS插值理论进行曲面插值
//BondaryCtrlPts：边界曲线控制点，按toric参数域方向顺序排列
void toric::CoonsToric(const varray<varray<point4d>>& BondaryCtrlPts)
{
	m_BondaryCtrlPts = BondaryCtrlPts;

	//将边界控制点放入m_CtrlPts的对应位置
	m_CtrlPts.resize(m_LatticePts.size());
	for (int i = 0; i < m_LatticePts.size(); ++i)
	{
		m_CtrlPts[i].resize(m_LatticePts[i].size());
		for (int j = 0; j < m_LatticePts[i].size(); ++j)
		{
			//边界号
			int isE = InEdge(m_LatticePts[i][j]);
			if (isE == -1)continue;

			//isE边界上的第几号点
			double a = (m_LatticePts[i][j] - m_ConerPts[isE]).Magnitude();
			double b = (m_ConerPts[(isE + 1) % m_ConerPts.size()] - m_ConerPts[isE]).Magnitude();
			int pIdx = a / (b / (m_BondaryCtrlPts[isE].size() - 1)) + 0.5;
			point4d cpts = m_BondaryCtrlPts[isE][pIdx];
			m_CtrlPts[i][j] = cpts;
		}
	}

	varray<varray<point4d>> UCtrlPts, VCtrlPts, TCtrlPts;
	CoonsS12TCtrlPts(UCtrlPts, VCtrlPts, TCtrlPts);

	for (int i = 1; i < UCtrlPts.size() - 1; ++i)
		for (int j = 1; j < UCtrlPts[i].size() - 1; ++j)
		{
			m_CtrlPts[i][j] = UCtrlPts[i][j] + VCtrlPts[i][j] - TCtrlPts[i][j];
			m_CtrlPts[i][j].w = UCtrlPts[i][j].w + VCtrlPts[i][j].w - TCtrlPts[i][j].w;
		}
}

//Coons直纹面,混合曲面控制点
void toric::CoonsS12TCtrlPts(varray<varray<point4d>>& UCtrlPts, varray<varray<point4d>>& VCtrlPts, varray<varray<point4d>>& TCtrlPts)
{
	UCtrlPts = m_CtrlPts;
	VCtrlPts = m_CtrlPts;
	TCtrlPts = m_CtrlPts;

	for (int i = 1; i < m_LatticePts.size() - 1; ++i)
	{
		int sz = m_LatticePts[i].size();
		point4d stu = UCtrlPts[i][0];
		point4d edu = UCtrlPts[i][sz - 1];
		point4d dpu = (edu - stu) / (sz - 1);
		dpu.w = (edu.w - stu.w) / (sz - 1);
		int lu0 = InEdge(m_LatticePts[i][0]);
		int lu1 = InEdge(m_LatticePts[i][sz - 1]);
		point4d p01 = m_BondaryCtrlPts[lu0][0];
		point4d p10 = m_BondaryCtrlPts[lu1][0];

		for (int j = 1; j < sz - 1; ++j)
		{
			//U直纹面
			UCtrlPts[i][j] = stu + j*dpu;
			UCtrlPts[i][j].w = stu.w + j*dpu.w;

			//V直纹面
			point2d stIdx, edIdx;
			FindEndpointV(m_LatticePts[i][j], stIdx, edIdx);
			point4d stv = VCtrlPts[stIdx.x][stIdx.y];
			point4d edv = VCtrlPts[edIdx.x][edIdx.y];
			point4d dpv = (edv - stv) / (edIdx.x - stIdx.x);
			dpv.w = (edv.w - stv.w) / (edIdx.x - stIdx.x);
			int n = m_LatticePts[i][j].y - stIdx.y;
			VCtrlPts[i][j] = stv + n*dpv;
			VCtrlPts[i][j].w = stv.w + n*dpv.w;

			//混合面
			double u = (m_LatticePts[i][j].x - m_xmin) / (m_xmax - m_xmin);
			double v = (m_LatticePts[i][j].y - m_ymin) / (m_ymax - m_ymin);
			int lv0 = InEdge(m_LatticePts[stIdx.x][stIdx.y]);
			int lv1 = InEdge(m_LatticePts[edIdx.x][edIdx.y]);
			point4d p00 = m_BondaryCtrlPts[lv0][0];
			point4d p11 = m_BondaryCtrlPts[lv1][0];
			TCtrlPts[i][j] = (1 - u)*(1 - v)*p00 + u*(1 - v)*p10 + (1 - u)*v*p01 + u*v*p11;
			TCtrlPts[i][j].w = (1 - u)*(1 - v)*p00.w + u*(1 - v)*p10.w + (1 - u)*v*p01.w + u*v*p11.w;
		}
	}
}

//参数域边界方程非负性校正
void toric::LineNonNeg(varray<int>& line, int idx)
{
	point2d check = m_ConerPts[(idx + 2) % m_LineN];
	double a = line[0] * check.x + line[1] * check.y + line[2];
	if (a < 0)
	{
		line[0] *= -1;
		line[1] *= -1;
		line[2] *= -1;
	}
}

//(u,v)处格点m对应的基函数一阶导数
//dir：true表示对U求导，false表示对V求导
double toric::CalhiDer1(const double u, const double v, const point2d& m, const double cm, const bool dir)
{
	int uv = 0;
	if (!dir)
		uv = 1;
	double res = 0;
	for (int i = 0; i < m_LineN; ++i)
	{
		double a = 0;
		if (CalhiVal(u, v, i) == 0 && CalhiVal(m, i) < 1)
			continue;
		else
			a = m_BoudaryLine[i][uv] * CalhiVal(m, i)*pow(CalhiVal(u, v, i), CalhiVal(m, i) - 1);

		if (a == 0)
			continue;

		double b = 1;
		for (int j = 0; j < m_LineN; ++j)
		{
			if (i == j)
				continue;

			if (CalhiVal(u, v, j) == 0 && CalhiVal(m, i) < 0)
			{
				b = 0;
				break;
			}
			else
				b *= pow(CalhiVal(u, v, j), CalhiVal(m, j));
		}
		res += a*b;
	}
	return cm*res;
}

//(u,v)处格点m对应的基函数二阶导数
//dir：【0，1，2，3】=【uu,vv,uv,vu】
double toric::CalhiDer2(const double u, const double v, const point2d & m, const double cm, const int dir)
{
	int uv[2] = { 0,0 };
	switch (dir)
	{
	case 1:uv[0] = 1; uv[1] = 1; break;
	case 2:uv[1] = 1; break;
	case 3:uv[0] = 1; break;
	default: break;
	}

	varray<double> der2, der1, hival;
	double resT = 0;
	for (int i = 0; i < m_LineN; ++i)
	{
		double t = pow(CalhiVal(u, v, i), CalhiVal(m, i));
		hival.push_back(t);

		if (CalhiVal(u, v, i) == 0 && CalhiVal(m, i) < 1)
			t = 0;
		else
			t = m_BoudaryLine[i][uv[0]] * CalhiVal(m, i)*pow(CalhiVal(u, v, i), CalhiVal(m, i) - 1);
		der1.push_back(t);

		if (CalhiVal(u, v, i) == 0 && CalhiVal(m, i) < 2)
			t = 0;
		else
			t = m_BoudaryLine[i][uv[0]] * m_BoudaryLine[i][uv[1]]
			* CalhiVal(m, i)* (CalhiVal(m, i) - 1)
			* pow(CalhiVal(u, v, i), CalhiVal(m, i) - 2);
		der2.push_back(t);
	}

	for (int i = 0; i < m_LineN; ++i)
	{
		double resi = der2[i];
		for (int j = 0; j < m_LineN; ++j)
		{
			if (i == j)
				continue;
			resi *= hival[j];
		}
		resT += resi;
	}

	for (int i = 0; i < m_LineN; ++i)
	{
		double resi = der1[i];
		for (int j = i + 1; j < m_LineN; ++j)
		{
			resi *= der1[j];
			for (int k = 0; k < m_LineN; ++k)
			{
				if (k == i || k == j)
					continue;
				resi *= hival[k];
			}
			resT += 2 * resi;
		}
	}
	return cm*resT;
}

//(u,v)处一阶导数
//dir：true表示对U求导，false表示对V求导
point3d toric::CalPtsDir1(const double u, const double v, const bool dir)
{
	point3d sumPts, torPts = GetToricPts(u, v);
	double sw = 0;
	for (int i = 0; i < m_LatticePts.size(); ++i)
	{
		for (int j = 0; j < m_LatticePts[i].size(); ++j)
		{
			sumPts += m_CtrlPts[i][j].w*CalhiDer1(u, v, m_LatticePts[i][j], m_cm[i][j], dir)*(m_CtrlPts[i][j] - torPts);
			sw += m_CtrlPts[i][j].w*ToricBasic(u, v, m_LatticePts[i][j], m_cm[i][j]);
		}
	}
	return sumPts / sw;
}

//(u,v)处二阶导数
//dir：【0，1，2，3】=【uu,vv,uv,vu】
point3d toric::CalPtsDir2(const double u, const double v, const int dir)
{
	int uv[2] = { 0,0 };
	switch (dir)
	{
	case 1:uv[0] = 1; uv[1] = 1; break;
	case 2:uv[1] = 1; break;
	case 3:uv[0] = 1; break;
	default: break;
	}

	point3d sumPts1, sumPts2, torPts = GetToricPts(u, v);
	double sw = 0, sw1 = 0;
	for (int i = 0; i < m_LatticePts.size(); ++i)
	{
		for (int j = 0; j < m_LatticePts[i].size(); ++j)
		{
			double t = m_CtrlPts[i][j].w*CalhiDer1(u, v, m_LatticePts[i][j], m_cm[i][j], uv[0]);
			sumPts1 += t*(m_CtrlPts[i][j] - torPts);
			sw += m_CtrlPts[i][j].w*ToricBasic(u, v, m_LatticePts[i][j], m_cm[i][j]);
			sumPts2 += m_CtrlPts[i][j].w*CalhiDer2(u, v, m_LatticePts[i][j], m_cm[i][j], dir)*(m_CtrlPts[i][j] - torPts);
			sw1 += t;
		}
	}
	point3d res = sumPts2 / sw - 2 * sw1*sumPts1 / (sw * sw);
	return res;
}

//判断是否为toric边界格点,是则返回边界号，否则返回-1
int toric::InEdge(const point2d &p)
{
	for (int i = 0; i < m_LineN; ++i)
	{
		double r = m_BoudaryLine[i][0] * p.x + m_BoudaryLine[i][1] * p.y + m_BoudaryLine[i][2];
		if (r == 0)
			return i;
	}
	return -1;
}

//参数域过点pts的U等值线的端点,返回端点在m_LatticePts的坐标
void toric::FindEndpointV(const point2d & pts, point2d & stIdx, point2d & edIdx)
{
	bool isSt = false;
	int u = pts.x, v = pts.y;
	for (int i = 0; i < m_LatticePts.size(); ++i)
	{
		for (int j = 0; j < m_LatticePts[i].size(); ++j)
		{
			if (u != m_LatticePts[i][j].x)
				continue;

			if (!isSt)
			{
				stIdx = point2d(i, j);
				isSt = true;
				break;
			}
			else
			{
				int isE = InEdge(m_LatticePts[i][j]);
				if (isE != -1)
				{
					edIdx = point2d(i, j);
					return;
				}
				break;
			}
		}
	}
}

point3d toric::CalM(const double u, const double v)
{
	point3d sumPts;
	for (int i = 0; i < m_LatticePts.size(); ++i)
	{
		for (int j = 0; j < m_LatticePts[i].size(); ++j)
		{
			sumPts += m_CtrlPts[i][j].w*ToricBasic(u, v, m_LatticePts[i][j], m_cm[i][j])*m_CtrlPts[i][j];
		}
	}
	return sumPts;
}

//toric分子式偏导
//dir：true表示对U求导，false表示对V求导
point3d toric::CalMDir1(const double u, const double v, const bool dir)
{
	point3d sumPts;
	for (int i = 0; i < m_LatticePts.size(); ++i)
	{
		for (int j = 0; j < m_LatticePts[i].size(); ++j)
		{
			sumPts += m_CtrlPts[i][j].w*CalhiDer1(u, v, m_LatticePts[i][j], m_cm[i][j], dir)*m_CtrlPts[i][j];
		}
	}
	return sumPts;
}
