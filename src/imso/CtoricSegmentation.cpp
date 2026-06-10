#include "stdafx.h"
#include "CtoricSegmentation.h"
#include "FittingBSpline.h"
#include "PSOLCT.h"
#include "globalFunc.h"
#include "RWGeometric.h"
#include <fstream>

toricSegment::toricSegment(varray<toric> & mtorics)
	:m_torics(mtorics)
{
}

toricSegment::~toricSegment()
{
}

void toricSegment::ToricSegmentation(const point3d CenPts, const varray<NurbsLine>& lines, const double r)
{
	m_CenPts = CenPts;
	m_lines = lines;
	m_r = r;

	for (int i = 0; i < m_lines.size(); ++i)
	{
		varray<point4d> der;
		m_lines[i].PtsDerivs(0, 1, der);
		m_linesDu.push_back(der[1]);
	}

	m_CenLine.clear();
	psoCenLine pcl;
	point3d cp;
	double dis;
	pcl.CalPsoCenLine(m_linesDu, cp, dis);
	cp = cp.Normalize();
	point3d p0 = m_CenPts - 1* m_r / 2 * cp,
		p1 = m_CenPts + 1 * m_r / 2 * cp;
	m_CenLine.push_back(p0);
	m_CenLine.push_back(p1);

	double la = (m_lines[0].m_CtrlPts[1]- m_lines[0].m_CtrlPts[0]).Magnitude();//沿轴线平移距离
	double lb = (p1 - p0).Magnitude() / 2;
	FirstVols(la,lb);
	RWGeometric rwg;
	//rwg.WriteNurbsVol("TestPoints.txt", m_Vols);
	rwg.ReadNurbsVol("TestPoints.txt", m_Vols);
	CalOutsideVec();
	FindOutsideVecTop();
	DelSurface();
	FittingBounSF();
	CreateBounVol();
	WriteData();
	//testData();
}

void toricSegment::testData()
{
	RWGeometric rwg;
	rwg.ReadNurbsSurface("m_OutSF.txt", m_OutsideSurface);
	rwg.ReadNurbsVol("vols.txt", m_Vols);

	varray<varray<point3d>> ovs;
	rwg.ReadPoint("m_OutsideVecs-pts.txt", ovs);
	for (int i = 0; i < ovs.size(); ++i)
	{
		outsideVec ov;
		ov.pts = ovs[i][0];
		ov.vec = ovs[i][1];
		ov.twinPts = ovs[i][2];
		m_OutsideVecs.push_back(ov);
	}

	varray<varray<point4d>> p4ds;
	rwg.ReadPoint("m_lineInf-p4ds.txt", p4ds);
	for (int i = 0; i < p4ds.size(); ++i)
	{
		lineinf linf;
		linf.idx0 = p4ds[i][0].x;
		linf.idx1 = p4ds[i][0].y;
		linf.ctrlPts.push_back(p4ds[i][1]);
		linf.ctrlPts.push_back(p4ds[i][2]);
		linf.ctrlPts.push_back(p4ds[i][3]);
		m_lineInf.push_back(linf);
	}
	
	rwg.ReadPoint("m_OutsideVecsTop-p4ds.txt", p4ds);
	for (int i = 0; i < p4ds.size(); ++i)
	{
		varray<point4d> p4d= p4ds[i];
		VecTop vt;
		vt.volIdx = p4d[0].x;
		for (int j = 0; j < 4; ++j)
			vt.coners.push_back(p4d[1][j]);
		vt.linePara.resize(4);
		for (int j = 2; j < 6; ++j)
			for (int k = 0; k < 3; ++k)
				vt.linePara[j - 2].push_back(p4d[j][k]);
		m_OutsideVecsTop.push_back(vt);
	}	

	//CreateBounVol();
}

//直线与toric曲面交点的参数域值,z值表示m_torics中的索引
//p0:直线点
//v0:直线方向向量
double toricSegment::LineCrossTorics(const point3d& p0, const point3d& v0, point3d& res)
{
	int torIdx = 0;
	point2d uv;
	double minang = LineCrossToric(p0, v0, 0, uv);
	for (int i = 1; i < m_torics.size(); ++i)
	{
		point2d uv1;
		double ang = LineCrossToric(p0, v0, i, uv1);
		//double ang = NewtonItera(p0, v0, i, uv1);
		if (ang < minang)
		{
			minang = ang;
			uv = uv1;
			torIdx = i;
		}
	}
	res = point3d(uv.x, uv.y, torIdx);
	return minang;
}

void toricSegment::FirstVols(const double la, const double lb)
{
	m_Vols.clear();

	varray<varray<point3d>> bis;
	CalBis(bis);
	NurbsVol vol;
	varray<double> knots;
	knots.push_back(0);
	knots.push_back(0);
	knots.push_back(1);
	knots.push_back(1);
	vol.SetVol(1, 1, 1, 2, 2, 2, knots, knots, knots);
	vol.m_CtrlPts.resize(8);
	vol.m_CtrlPts[0] = m_CenLine[0];
	vol.m_CtrlPts[1] = m_CenLine[1];

	for (int i = 0; i < m_lines.size(); ++i)
	{
		//骨架线上
		varray<point4d> line(2), linet, line3;
		line[0] = m_linesDu[i] * la + m_CenLine[0];
		line[1] = m_linesDu[i] * la + m_CenLine[1];
		point4d p0 = (point4d)m_linesDu[i].Normalize()*la + m_lines[0].m_CtrlPts[0];
		point4d dp = line[0] - p0;
		point4d vt = (point4d)m_linesDu[i].Cross(dp).Cross(m_linesDu[i]).Normalize();
		vol.m_CtrlPts[4] = vt * dp.Magnitude()*cos(vt.Angle(dp)*PI / 180) + p0;
		vol.m_CtrlPts[5] = -vt * dp.Magnitude()*cos(vt.Angle(dp)*PI / 180) + p0;
		line[0] = vol.m_CtrlPts[4];
		line[1] = vol.m_CtrlPts[5];

		point3d dis0, v1, v2, v;
		//前一
		int idxb = i - 1;
		if (idxb < 0)
			idxb = m_lines.size() - 1;
		dis0 = bis[idxb][0].Normalize();
		//骨架线与前角平分线之间
		vt = m_linesDu[i].Cross(dis0).Cross(m_linesDu[i]).Normalize();
		dp = line[1] - line[0];
		v1 = dp.Cross(m_linesDu[i]).Normalize();
		v2 = -v1;
		double ang1 = v1.Angle(vt), ang2 = v2.Angle(vt);
		vt = v1;
		if (ang2 < ang1)
			vt = v2;
		linet = line;
		linet[0] += vt*lb;
		linet[1] += vt*lb;
		vol.m_CtrlPts[6] = linet[0];
		vol.m_CtrlPts[7] = linet[1];
		//前角平分线上
		if (i == 0)
		{
			v = m_linesDu[i].Cross(dis0).Cross(dis0).Normalize();
			line3 = linet;
			Project2Plane(v, m_CenPts, line3, linet, m_linesDu[i].Normalize());
			vol.m_CtrlPts[2] = linet[0];
			vol.m_CtrlPts[3] = linet[1];
		}
		else
		{
			vol.m_CtrlPts[2] = m_Vols[m_Vols.size() - 1].m_CtrlPts[2];
			vol.m_CtrlPts[3] = m_Vols[m_Vols.size() - 1].m_CtrlPts[3];
		}
		m_Vols.push_back(vol);
		//后一
		dis0 = bis[i][0].Normalize();
		//骨架线与前角平分线之间
		vt *= -1;
		linet = line;
		linet[0] += vt*lb;
		linet[1] += vt*lb;
		vol.m_CtrlPts[6] = linet[0];
		vol.m_CtrlPts[7] = linet[1];
		if (i == m_lines.size() - 1)
		{
			vol.m_CtrlPts[2] = m_Vols[0].m_CtrlPts[2];
			vol.m_CtrlPts[3] = m_Vols[0].m_CtrlPts[3];
		}
		else
		{
			//后角平分线上
			v = m_linesDu[i].Cross(dis0).Cross(dis0).Normalize();
			line3 = linet;
			Project2Plane(v, m_CenPts, line3, linet, m_linesDu[i].Normalize());
			vol.m_CtrlPts[2] = linet[0];
			vol.m_CtrlPts[3] = linet[1];
		}
		m_Vols.push_back(vol);
	}
}

void toricSegment::CalBis(varray<varray<point3d>>& bis)
{
	bis.clear();
	point3d allVec;
	for (int i = 0; i < m_linesDu.size(); ++i)
		allVec += m_linesDu[i];
	allVec.Normalize();

	for (int i = 0; i < m_linesDu.size(); ++i)
	{
		varray<point3d> lbis;
		for (int j = i + 1; j < m_linesDu.size(); ++j)
		{
			point3d v = m_linesDu[i].Normalize() + m_linesDu[j].Normalize();
			if (v == point3d())
				v = -allVec;
			v = v.Normalize();
			lbis.push_back(v);
		}
		if (i == m_linesDu.size() - 1)
			lbis.push_back(bis[0][bis[0].size() - 1]);
		bis.push_back(lbis);
	}
}

void toricSegment::CalOutsideVec()
{
	m_OutsideVecs.clear();
	
	for (int i = 0; i < m_Vols.size(); ++i)
	{
		for (unsigned int j = 0; j < 8; ++j)
		{
			outsideVec pvec;
			bool u = (j & (1 << 2)), v = (j & (1 << 1)), w = (j & (1 << 0));
			pvec.pts = m_Vols[i].GetVolPoint(u, v, w);
			//矢量
			/*pvec.vec += (m_Vols[i].GetVolPoint(!u, v, w) - pvec.pts).Normalize();
			pvec.vec += (m_Vols[i].GetVolPoint(u, !v, w) - pvec.pts).Normalize();
			pvec.vec += (m_Vols[i].GetVolPoint(u, v, !w) - pvec.pts).Normalize();*/
			double u1 = u, v1 = v, w1 = w;
			if (u1==0)u1 += 0.01;
			else u1 -= 0.01;
			pvec.vec += (m_Vols[i].GetVolPoint(u1, v, w) - pvec.pts).Normalize();
			if (v1 == 0)v1 += 0.01;
			else v1 -= 0.01;
			pvec.vec += (m_Vols[i].GetVolPoint(u, v1, w) - pvec.pts).Normalize();
			if (w1 == 0)w1 += 0.01;
			else w1 -= 0.01;
			pvec.vec += (m_Vols[i].GetVolPoint(u, v, w1) - pvec.pts).Normalize();

			bool flag = false;
			for (int k = 0; k < m_OutsideVecs.size(); ++k)//若已存在，则求合矢
			{
				if (m_OutsideVecs[k].pts != pvec.pts)
					continue;
				m_OutsideVecs[k].vec += pvec.vec;
				flag = true;
			}
			if (!flag)//若不存在，直接存入
			{
				m_OutsideVecs.push_back(pvec);
			}
		}
	}
	int a[4] = { 4,5,6,7 };
	for (int i = 0; i < m_Vols.size(); ++i)
	{
		for (int j = 0; j < 4; ++j)
		{
			point3d pts = m_Vols[i].m_CtrlPts[a[j]];
			for (int k = 0; k < m_OutsideVecs.size(); ++k)
			{
				if (m_OutsideVecs[k].pts != pts)continue;
				m_OutsideVecs[k].isEdge = true;
				point3d vl = m_linesDu[i / 2];
				m_OutsideVecs[k].vec = vl.Cross(m_OutsideVecs[k].vec).Cross(vl).Normalize();
				
			}
		}
	}
	for (int i = 0; i < m_OutsideVecs.size(); ++i)
		m_OutsideVecs[i].vec = -m_OutsideVecs[i].vec.Normalize();
}

void toricSegment::FindOutsideVecTop()
{
	m_OutsideVecsTop.clear();
	int a[4]{ 0,1,3,2 };
	int b[5]{ 0,2,1,3,0 };
	for (int i = 0; i < m_Vols.size(); ++i)
	{
		VecTop aVT0, aVT1;
		aVT0.volIdx = i;
		aVT1.volIdx = i;
		varray<int> aface0, aface1;
		//u=0,1
		for (unsigned int j = 0; j < 4; ++j)
		{
			bool v = (a[j] & (1 << 1)), w = (a[j] & (1 << 0));
			point3d pts = m_Vols[i].GetVolPoint(0, v, w);
			int idx = FindIndex(pts);
			aface0.push_back(idx);
			pts = m_Vols[i].GetVolPoint(1, v, w);
			idx = FindIndex(pts);
			aface1.push_back(idx);

			varray<int> al0, al1;
			al0.push_back(0);
			al0.push_back(b[j]);
			al0.push_back(b[j + 1]);
			aVT0.linePara.push_back(al0);
			al1 = al0;
			al1[0] = 1;
			aVT1.linePara.push_back(al1);
		}
		aVT0.coners = aface0;
		aVT1.coners = aface1;
		m_OutsideVecsTop.push_back(aVT0);
		m_OutsideVecsTop.push_back(aVT1);

		aface0.clear();
		aface1.clear();
		aVT0.linePara.clear();
		aVT1.linePara.clear();
		//v=0,1
		for (unsigned int j = 0; j < 4; ++j)
		{
			bool u = (a[j] & (1 << 1)), w = (a[j] & (1 << 0));
			point3d pts = m_Vols[i].GetVolPoint(u, 0, w);
			int idx = FindIndex(pts);
			aface0.push_back(idx);
			pts = m_Vols[i].GetVolPoint(u, 1, w);
			idx = FindIndex(pts);
			aface1.push_back(idx);

			varray<int> al0, al1;
			al0.push_back(b[j]);
			al0.push_back(0);
			al0.push_back(b[j + 1]);
			aVT0.linePara.push_back(al0);
			al1 = al0;
			al1[1] = 1;
			aVT1.linePara.push_back(al1);
		}
		aVT0.coners = aface0;
		aVT1.coners = aface1;
		m_OutsideVecsTop.push_back(aVT0);
		m_OutsideVecsTop.push_back(aVT1);

		aface0.clear();
		aface1.clear();
		aVT0.linePara.clear();
		aVT1.linePara.clear();
		//w=0,1
		for (unsigned int j = 0; j < 4; ++j)
		{
			bool u = (a[j] & (1 << 1)), v = (a[j] & (1 << 0));
			point3d pts = m_Vols[i].GetVolPoint(u, v, 0);
			int idx = FindIndex(pts);
			aface0.push_back(idx);
			pts = m_Vols[i].GetVolPoint(u, v, 1);
			idx = FindIndex(pts);
			aface1.push_back(idx);

			varray<int> al0, al1;
			al0.push_back(b[j]);
			al0.push_back(b[j + 1]);
			al0.push_back(0);
			aVT0.linePara.push_back(al0);
			al1 = al0;
			al1[2] = 1;
			aVT1.linePara.push_back(al1);
		}
		aVT0.coners = aface0;
		aVT1.coners = aface1;
		m_OutsideVecsTop.push_back(aVT0);
		m_OutsideVecsTop.push_back(aVT1);
	}
}

//pts在m_OutsideVecs中的序号
int toricSegment::FindIndex(const point3d pts)
{
	for (int i = 0; i < m_OutsideVecs.size(); ++i)
	{
		if (m_OutsideVecs[i].pts == pts)
			return i;
	}
	return -1;
}

//去除重合面
void toricSegment::DelSurface()
{
	varray<VecTop> OutsideVecsTop = m_OutsideVecsTop;
	m_OutsideVecsTop.clear();
	for (int i = 0; i < OutsideVecsTop.size(); ++i)
	{
		bool a = true;
		for (int j = 0; j < OutsideVecsTop[i].coners.size(); ++j)
			a = a&&m_OutsideVecs[OutsideVecsTop[i].coners[j]].isEdge;
		if(a)
		{
			OutsideVecsTop.erase(OutsideVecsTop.begin() + i);
			i--;
		}
		else
		{
			for (int j = i + 1; j < OutsideVecsTop.size(); ++j)
			{
				bool res = IsSameEle(OutsideVecsTop[i].coners, OutsideVecsTop[j].coners);
				if (!res)continue;
				else
				{
					a = true;
					OutsideVecsTop.erase(OutsideVecsTop.begin() + i);
					OutsideVecsTop.erase(OutsideVecsTop.begin() + j - 1);
					i--;
					break;
				}
			}
		}
		if (!a)
			m_OutsideVecsTop.push_back(OutsideVecsTop[i]);
	}
	
}

//两数组元素是否相同(无序)
bool toricSegment::IsSameEle(const varray<int>& a, const varray<int>& b)
{
	for (int i = 0; i < a.size() ; ++i)
	{
		int j = 0;
		for (j = 0; j < b.size(); ++j)
		{
			if (a[i] == b[j])
				break;
		}
		if (j == b.size())
			return false;
	}
	return true;
}

//采样
//topIdx：m_OutsideVecsTop索引
//lineIdx：VecsTop中边的索引
//num：采样点数量
void toricSegment::Sampling(const int topIdx, const int lineIdx, const int num, varray<point3d>& sampts)
{
	sampts.clear();
	int idx0 = m_OutsideVecsTop[topIdx].coners[lineIdx];
	int idx1 = m_OutsideVecsTop[topIdx].coners[(lineIdx + 1) % 4];
	point3d p0 = m_OutsideVecs[idx0].pts;
	point3d p1 = m_OutsideVecs[idx1].pts;
	point3d v0 = m_OutsideVecs[idx0].vec.Normalize();
	point3d v1 = m_OutsideVecs[idx1].vec.Normalize();
	double ang = v0.Angle(v1)*PI / 180;
	point3d az = v0.Cross(v1).Normalize();
	for (int i = 0; i < num; ++i)///////////////////////////////////////////////////////////////
	{
		if (i == 0 && m_OutsideVecs[idx0].hasTwinPts)
		{
			sampts.push_back(m_OutsideVecs[idx0].twinPts);
			continue;
		}
		if (i == num - 1 && m_OutsideVecs[idx1].hasTwinPts)
		{
			sampts.push_back(m_OutsideVecs[idx1].twinPts);
			continue;
		}
		double u = 1.0*i / (num - 1);
		double dang = i*ang / (num - 1);
		point3d vt;
		if (az == point3d())
			vt = v0;
		else
			vt = v0.RotAxis(az, dang);
		if (i == num - 1)
		{
			u = 1;
			vt = v1;
		}
		point3d pt;
		pt = FindPrismPts(topIdx, lineIdx, u);
		point3d xyi;
		double ang = LineCrossTorics(pt, vt, xyi);
		if (i != 0 && i != num - 1 && ang > 1)continue;
		point3d	pts = m_torics[xyi.z].GetToricPts(xyi.x, xyi.y);
		if (i == 0)
		{
			m_OutsideVecs[idx0].twinPts = pts;
			m_OutsideVecs[idx0].hasTwinPts = true;
		}
		else if (i == num - 1)
		{
			m_OutsideVecs[idx1].twinPts = pts;
			m_OutsideVecs[idx1].hasTwinPts = true;
		}
		sampts.push_back(pts);
	}
}

//边界面拟合
void toricSegment::FittingBounSF()
{
	using namespace bsl;//B样条拟合命名空间

	m_OutsideSurface.clear();
	m_lineInf.clear();

	NurbsLine line;
	line.m_Degree = 2;
	line.m_Knots.clear();
	for (int i = 0; i < line.m_Degree + 1; ++i)
		line.m_Knots.push_back(0);
	for (int i = 0; i < line.m_Degree + 1; ++i)
		line.m_Knots.push_back(1);

	for (int i = 0; i < m_OutsideVecsTop.size(); ++i)
	{
		varray<varray<point4d>> ectrl4d0, ectrl4d;
		for (int j = 0; j < m_OutsideVecsTop[i].linePara.size(); ++j)
		{
			//采样
			varray<point3d> sampts;
			int idx0 = m_OutsideVecsTop[i].coners[j];
			int idx1 = m_OutsideVecsTop[i].coners[(j + 1) % m_OutsideVecsTop[i].coners.size()];
			bool isRev;//是否同向
			int lidx = FindLineInf(idx0, idx1, isRev);
			if (lidx == -1)
			{
				Sampling(i, j, 10, sampts);
				//拟合
				FitBSpline fbs;
				fbs.FittingBspl(sampts, line.m_Knots, line.m_Degree, line.m_Degree + 1);
				lineinf lineinf0;
				lineinf0.idx0 = idx0;
				lineinf0.idx1 = idx1;
				for (int k = 0; k < fbs.m_CtrlPts.size(); ++k)
					lineinf0.ctrlPts.push_back(fbs.m_CtrlPts[k]);
				m_lineInf.push_back(lineinf0);
				ectrl4d0.push_back(lineinf0.ctrlPts);
			}
			else
			{
				varray<point4d> cpt = m_lineInf[lidx].ctrlPts;
				if (isRev)cpt.reverse();
				ectrl4d0.push_back(cpt);
			}
		}
		ectrl4d.push_back(ectrl4d0[0]);
		ectrl4d0[3].reverse();
		ectrl4d.push_back(ectrl4d0[3]);
		ectrl4d0[2].reverse();
		ectrl4d.push_back(ectrl4d0[2]);
		ectrl4d.push_back(ectrl4d0[1]);
		NurbsSurface sf;
		sf.SetSurface(line.m_Degree, line.m_Degree,
			line.m_Degree + 1, line.m_Degree + 1, line.m_Knots, line.m_Knots);
		//coons
		sf.CoonsInterpolate(ectrl4d);
		m_OutsideSurface.push_back(sf);
	}
}

//边界判断
int toricSegment::InEdge(toric & tor, const point2d & p0)
{
	point2d cen;
	for (int i = 0; i < tor.m_ConerPts.size(); ++i)
		cen += tor.m_ConerPts[i];
	cen /= tor.m_ConerPts.size();
	point2d dp = p0 - cen;
	for (int i = 0; i < tor.m_ConerPts.size(); ++i)
	{
		point2d p1 = tor.m_ConerPts[i] - cen;
		point2d p2 = tor.m_ConerPts[(i + 1) % tor.m_ConerPts.size()] - cen;
		double ang = p1.Angle(p2);
		double ang1 = p1.Angle(dp);
		double ang2 = p2.Angle(dp);
		if (ang1 <= ang && ang2 <= ang)
			return i;
	}
}

//生成边界六面体
void toricSegment::CreateBounVol()
{
	//cube升阶
	for (int i = 0; i < m_Vols.size(); ++i)
		m_Vols[i].DegreeElevate(2, 2, 2);
	NurbsVol vol;
	varray<double> wknots;
	wknots.push_back(0);
	wknots.push_back(0);
	wknots.push_back(1);
	wknots.push_back(1);
	vol.SetVol(m_OutsideSurface[0].m_uDegree, m_OutsideSurface[0].m_vDegree, 1,
		m_OutsideSurface[0].m_uNum, m_OutsideSurface[0].m_vNum, 2,
		m_OutsideSurface[0].m_uKnots, m_OutsideSurface[0].m_vKnots, wknots);
	for (int i = 0; i < m_OutsideVecsTop.size(); ++i)
	{
		GetSFCtrlPtsWithVecTop(m_OutsideVecsTop[i], vol.m_CtrlPts);
		for (int j = 0; j < m_OutsideSurface[i].m_CtrlPts.size(); ++j)
			vol.m_CtrlPts.push_back(m_OutsideSurface[i].m_CtrlPts[j]);
		m_Vols.push_back(vol);
	}
}

//根据VecTop获得cube面上的控制点
void toricSegment::GetSFCtrlPtsWithVecTop(const VecTop & vectop, varray<point4d>& ctrlPts)
{
	ctrlPts.clear();
	NurbsVol vol = m_Vols[vectop.volIdx];
	//等值面的值
	bool uc = false, vc = false, wc = false;
	int u = vectop.linePara[0][0], v = vectop.linePara[0][1], w = vectop.linePara[0][2];
	for (int i = 1; i < vectop.linePara.size(); ++i)
	{
		if (!uc && u != vectop.linePara[i][0])uc = true;
		if (!vc && v != vectop.linePara[i][1])vc = true;
		if (!wc && w != vectop.linePara[i][2])wc = true;
	}
	if (!uc)
	{
		if (u == 1)u = vol.m_uNum - 1;
		for (int j = 0; j < vol.m_wNum; ++j)
			for (int i = 0; i < vol.m_vNum; ++i)
			{
				int idx = vol.CtrlPtsIdx(u, i, j);
				ctrlPts.push_back(vol.m_CtrlPts[idx]);
			}
	}
	else if (!vc)
	{
		if (v == 1)v = vol.m_vNum - 1;
		for (int j = 0; j < vol.m_wNum; ++j)
			for (int i = 0; i < vol.m_uNum; ++i)
			{
				int idx = vol.CtrlPtsIdx(i, v, j);
				ctrlPts.push_back(vol.m_CtrlPts[idx]);
			}
	}
	else if (!wc)
	{
		if (w == 1)w = vol.m_wNum - 1;
		for (int j = 0; j < vol.m_vNum; ++j)
			for (int i = 0; i < vol.m_uNum; ++i)
			{
				int idx = vol.CtrlPtsIdx(i, j, w);
				ctrlPts.push_back(vol.m_CtrlPts[idx]);
			}
	}
}

double toricSegment::NewtonItera(const point3d & p0, const point3d & v0, const int ToricIdx, point2d & res)
{
	//变换为与Z轴重合
	Matrix4d mat;
	CalTransMat(point3d(0, 0, 0), point3d(0, 0, 1), p0, v0, mat);
	toric tor(m_torics[ToricIdx]);
	for (int i = 0; i < tor.m_CtrlPts.size(); ++i)
		TransByMat(tor.m_CtrlPts[i], mat);

	//迭代
	const int itN = 100;
	double eps = 0.00001;

	//临近根
	double minang = 360;
	for (int i = 0; i < tor.m_LatticePts.size(); ++i)
	{
		for (int j = 0; j < tor.m_LatticePts[i].size(); ++j)
		{
			point3d pts = tor.GetToricPts(tor.m_LatticePts[i][j].x, tor.m_LatticePts[i][j].y);
			double ang = pts.Angle(point3d(0, 0, 1));
			if (ang <= minang)
			{
				res = tor.m_LatticePts[i][j];
				minang = ang;
			}
		}
	}
	if (minang < eps)
	{
		return minang;
	}
	int k = 0;
	Eigen::Matrix<double, 2, 2> Ak, Duv, Akinv;
	point3d dpu, dpv;
	dpu = tor.CalMDir1(res.x, res.y, true);
	dpv = tor.CalMDir1(res.x, res.y, false);
	Ak << dpu.x, dpv.x,
		dpu.y, dpv.y;

	Eigen::Matrix<double,2,1> Fxk, Fxk1, xk, yk, sk;
	xk << res.x, res.y;
	sk<< tor.m_xmax - tor.m_xmin,
		tor.m_ymax - tor.m_ymin;
	while (k < itN && (sk(0,0) > eps || sk(1,0) > eps))
	{
		Fxk << tor.CalM(xk(0, 0), xk(1, 0)).x,
			tor.CalM(xk(0, 0), xk(1, 0)).y;
		Akinv = Ak.inverse();
		sk = -Akinv*Fxk;
		xk -= sk;
		Fxk1<< tor.CalM(xk(0, 0), xk(1, 0)).x,
			tor.CalM(xk(0, 0), xk(1, 0)).y;
		yk = Fxk1 - Fxk;
		Eigen::Matrix<double, 1, 2> skt;
		skt << sk(0, 0), sk(1, 0);
		Ak += ((yk - Ak*sk)*skt) / (skt*sk);
		++k;
	}
	point3d pts = tor.GetToricPts(xk(0, 0), xk(1, 0));
	double ang = pts.Angle(point3d(0, 0, 1));
	res.x = xk(0, 0), res.y = xk(1, 0);
	return ang;
}

//直线与toric曲面交点的参数域值，牛顿迭代法
//p0:直线点
//v0:直线方向向量
double toricSegment::LineCrossToric(const point3d & p0, const point3d & v0, const int ToricIdx, point2d& res)
{
	//变换为与Z轴重合
	Matrix4d mat;
	CalTransMat(point3d(0, 0, 0), point3d(0, 0, 1), p0, v0, mat);
	toric tor(m_torics[ToricIdx]);
	for (int i = 0; i < tor.m_CtrlPts.size(); ++i)
		TransByMat(tor.m_CtrlPts[i], mat);

	//迭代
	const int itN = 100;
	double eps = 0.00001;
	//临近根
	
	double minang = 360;
	for (int i = 0; i < tor.m_LatticePts.size(); ++i)
	{
		for (int j = 0; j < tor.m_LatticePts[i].size(); ++j)
		{
			point3d pts = tor.GetToricPts(tor.m_LatticePts[i][j].x, tor.m_LatticePts[i][j].y);
			double ang = pts.Angle(point3d(0, 0, 1));
			if (ang <= minang)
			{
				res = tor.m_LatticePts[i][j];
				minang = ang;
			}
		}
	}
	if (minang < eps)
	{
		return minang;
	}
	point2d uv = res;
	point2d uv0 = uv, delt0; 
	point3d puv = tor.CalM(uv.x, uv.y);
	double lastAng = minang;
	int REV = 1;
	int cREV = 0;
	bool isE = false;
	point3d pts;
	for (int i = 0; i < itN; ++i)
	{
		double ang;
		//偏导
		point3d dpu, dpv;
		dpu = tor.CalMDir1(uv.x, uv.y, true);
		dpv = tor.CalMDir1(uv.x, uv.y, false);
		Eigen::Matrix2d Duv;
		Duv << dpu.x, dpv.x,
			dpu.y, dpv.y;
		Eigen::Vector2d deltauv, Buv;
		Buv << puv.x, puv.y;
		deltauv = Duv.inverse()*Buv;
		uv -= point2d(deltauv(0, 0), deltauv(1, 0))*REV;
		if (i == 0)
			delt0 = point2d(deltauv(0, 0), deltauv(1, 0));
L1:
		if (!tor.IsInSF(uv.x, uv.y) && !isE)
		{
			isE = true;
			int Eidx = InEdge(tor, uv);
			psoLCT psolct(tor, Eidx);
			psolct.CalPSOLCT(uv, ang);
			if (ang < minang)
			{
				minang = ang;
				res = uv;
			}
			if (REV == -1)
				break;
			else
				goto L2;
		}
		puv = tor.CalM(uv.x, uv.y);
		pts = tor.GetToricPts(uv.x, uv.y);
		ang = pts.Angle(point3d(0, 0, 1));
		if (ang < minang)
		{
			minang = ang;
			res = uv;
			cREV = 0;
		}
	L2:
		if (ang > lastAng)++cREV;
		if ((abs(ang - lastAng) < 0.0001 && ang < eps) || cREV > itN / 5)
		{
			break;
		}
		if ((abs(ang - lastAng)<0.0001 && ang > eps) || cREV > itN / 7)
		{
			if (REV == 1)
			{
				REV = -1;
				cREV = 0;
				isE = false;
				uv = uv0 + delt0;
				goto L1;
			}
		}
		lastAng = ang;
	}
	return minang;
}

//曲线查找，存在返回m_lineInf内索引，不存在则返回-1
//idx0，idx1：内部cube对应线段端点
//isRev：与已存在的线是否同向
int toricSegment::FindLineInf(const int idx0, const int idx1, bool& isRev)
{
	for (int i = 0; i < m_lineInf.size(); ++i)
	{
		if (idx0 == m_lineInf[i].idx0 && idx1 == m_lineInf[i].idx1)
		{
			isRev = false;
			return i;
		}
		else if (idx0 == m_lineInf[i].idx1 && idx1 == m_lineInf[i].idx0)
		{
			isRev = true;
			return i;
		}
	}
	return -1;
}

//根据top信息寻找棱线点
//topIdx：m_OutsideVecsTop索引
//lineIdx：VecsTop中边的索引
//x:采样点参数值
point3d toricSegment::FindPrismPts(const int topIdx, const int lineIdx, const double x)
{
	double u = m_OutsideVecsTop[topIdx].linePara[lineIdx][0];
	double v = m_OutsideVecsTop[topIdx].linePara[lineIdx][1];
	double w = m_OutsideVecsTop[topIdx].linePara[lineIdx][2];
	if (u == 2)u = x;
	else if (u == 3)u = 1 - x;
	if (v == 2)v = x;
	else if (v == 3)v = 1 - x;
	if (w == 2)w = x;
	else if (w == 3)w = 1 - x;
	point3d pts = m_Vols[m_OutsideVecsTop[topIdx].volIdx].GetVolPoint(u, v, w);
	return pts;
}


void toricSegment::WriteData()
{
	varray<varray<point3d>> ovs;
	for (int i = 0; i < m_OutsideVecs.size(); ++i)
	{
		varray<point3d> ov;
		ov.push_back(m_OutsideVecs[i].pts);
		ov.push_back(m_OutsideVecs[i].vec);
		ov.push_back(m_OutsideVecs[i].twinPts);
		ovs.push_back(ov);
	}
	RWGeometric rwg;
	rwg.WritePoint("m_OutsideVecs-pts.txt", ovs);

	varray<varray<point4d>> p4ds;
	for (int i = 0; i < m_lineInf.size(); ++i)
	{
		varray<point4d> p4d = m_lineInf[i].ctrlPts;
		p4d.insert(p4d.begin(), point4d(m_lineInf[i].idx0, m_lineInf[i].idx1, 0));
		p4ds.push_back(p4d);
	}
	rwg.WritePoint("m_lineInf-p4ds.txt", p4ds);

	p4ds.clear();
	for (int i = 0; i < m_OutsideVecsTop.size(); ++i)
	{
		VecTop vt = m_OutsideVecsTop[i];
		varray<point4d> p4d; 
		p4d.push_back(point4d(vt.volIdx, 0, 0, 1));
		p4d.push_back(point4d(vt.coners[0], vt.coners[1], vt.coners[2], vt.coners[3]));
		for (int j = 0; j < 4; ++j)
			p4d.push_back(point4d(vt.linePara[j][0], vt.linePara[j][1], vt.linePara[j][2], 1));
		p4ds.push_back(p4d);
	}
	rwg.WritePoint("m_OutsideVecsTop-p4ds.txt", p4ds);

	rwg.WriteNurbsSurface("m_OutSF.txt", m_OutsideSurface);
	rwg.WriteNurbsVol("allvols.txt", m_Vols);
}
