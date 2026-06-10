#include "CNurbs.h"
#include "globalFunc.h"
#include "../lib/eigen/Eigen/Dense"
#include "RWGeometric.h"
#include <fstream>

//计算u处所有n次Bernstein多项式的值
void BezierLine::AllBernstein(const int n, const double u, varray<double>& B)const
{
	B.resize(n + 1);
	B[0] = 1.0;
	double u1 = 1 - u;
	for (int j = 1; j <= n; ++j)
	{
		double saved = 0;
		for (int k = 0; k < j; ++k)
		{
			double temp = B[k];
			B[k] = saved + u1*temp;
			saved = u*temp;
		}
		B[j] = saved;
	}
}

point3d BezierLine::GetLinePoint(const double u)const
{
	point3d res;
	varray<double> B;
	double sumW = 0;
	AllBernstein(m_Degree, u, B);
	for (int k = 0; k <= m_Degree; ++k)
	{
		res += B[k] * m_CtrlPts[k] * m_CtrlPts[k].w;
		sumW += B[k] * m_CtrlPts[k].w;
	}
	return res / sumW;
}

//计算Bezier曲线
void BezierLine::CalLinePoint(const int Unum, varray<point3d>& linePts) const
{
	linePts.clear();
	double du = 1.0 / Unum;
	for (int i = 0; i <= Unum; i++)         //采用等分节点矢量插值B样条曲线
	{
		double u = du*i;
		point3d t = GetLinePoint(u);
		linePts.push_back(t);
	}
}

	/*计算节点下标
	x：节点值
	degree：次数
	CtrlPtsNum：控制点数量
	knots：节点矢量*/
	int NurbsBase::FindSpan(const double x, const int degree, const int CtrlPtsNum, const varray<double>& knots)const
	{
		int left = degree, right = CtrlPtsNum, mid = 0;
		if (x >= knots[right])
			return right - 1;
		mid = (left + right) / 2;
		while (x < knots[mid] || x >= knots[mid + 1])
		{
			if (x < knots[mid])
				right = mid;
			else
				left = mid;
			mid = (right + left) / 2;
		}
		return mid;
	}

	/*根据参数值，节点下标，计算基函数
	u：参数值
	k：节点下标
	degree：次数
	knots：节点矢量
	N：返回的基函数*/
	void NurbsBase::BasisFuns(const double u, const int k, const int degree, const varray<double>& knots, 
		varray<double>& N)const
	{
		int p = degree;
		varray<double>  left, right;
		right.resize(p + 1, 0);
		left.resize(p + 1, 0);
		N.resize(p + 1, 0);

		N[0] = 1.0;
		double saved, temp;
		for (int j = 1; j <= p; j++)
		{
			left[j] = u - knots[k + 1 - j];
			right[j] = knots[k + j] - u;
			saved = 0;
			for (int r = 0; r<j; r++)
			{
				temp = N[r] / (right[r + 1] + left[j - r]);
				N[r] = saved + right[r + 1] * temp;
				saved = left[j - r] * temp;
			}
			N[j] = saved;
		}
		left.clear();
		right.clear();
	}

	/*u处所有基函数
	u：参数值
	k：节点下标
	degree：次数
	knots：节点矢量
	ndu：返回的所有基函数*/
	void NurbsBase::AllBasisFuns(const double u, const int k, const int degree, 
		const varray<double>& knots, varray<varray<double>>& ndu)const
	{
		int order = degree + 1;
		double saved, temp;
		varray<double> left, right;

		left.resize(order);
		right.resize(order);
		ndu.resize(order, left);

		ndu[0][0] = 1;
		for (int j = 1; j <= degree; j++)
		{
			left[j] = u - knots[k + 1 - j];
			right[j] = knots[k + j] - u;
			saved = 0.0f;
			for (int r = 0; r<j; r++)
			{
				ndu[j][r] = right[r + 1] + left[j - r];
				temp = ndu[r][j - 1] / ndu[j][r];
				ndu[r][j] = saved + right[r + 1] * temp;
				saved = left[j - r] * temp;
			}
			ndu[j][j] = saved;
		}
	}

	/*基函数n阶导
	u：参数值
	k：节点下标
	degree：次数
	n：导矢阶数
	knots：节点矢量
	basisDu：基函数n阶导*/
	void NurbsBase::DerBasisFuns(const double u, const int k, const int degree, const int n, const varray<double>& knots,
		varray<varray<double>>& basisDu)const
	{
		int p = degree;
		int rn = n > p ? p : n;
		varray<double> left(p + 1, 0), right(p + 1, 0);
		varray<varray<double>>a(2), ndu(p + 1);
		basisDu.resize(n + 1);
		for (int i = 0; i < n+1; ++i)
			basisDu[i].resize(k + 1, 0);
		for (int i = 0; i < 2; ++i)
			a[i].resize(p + 1, 0);
		for (int i = 0; i < p + 1; ++i)
			ndu[i].resize(p + 1, 0);

		ndu[0][0] = 1.0;
		int r, s1, s2, rk, pk, j1, j2;
		double d;
		double  saved, temp;
		for (int j = 1; j <= p; j++)
		{
			left[j] = u - knots[k + 1 - j];
			right[j] = knots[k + j] - u;
			saved = 0.0;
			for (r = 0; r<j; r++)
			{
				ndu[j][r] = right[r + 1] + left[j - r];
				temp = ndu[r][j - 1] / ndu[j][r];
				ndu[r][j] = saved + right[r + 1] * temp;
				saved = left[j - r] * temp;
			}
			ndu[j][j] = saved;
		}
		for (int j = 0; j <= p; ++j)
			basisDu[0][j] = ndu[j][p];
		for (r = 0; r <= p; r++)
		{
			a[0][0] = 1.0;
			s1 = 0; s2 = 1;
			for (int i = 1; i <= n; ++i)
			{
				d = 0.0;
				rk = r - rn;
				pk = p - rn;
				if (r >= rn)
				{
					a[s2][0] = a[s1][0] / ndu[pk + 1][rk];
					d = a[s2][0] * ndu[rk][pk];
				}
				if (rk >= -1) j1 = 1;
				else j1 = (-rk);
				if (r - 1 <= pk)  j2 = rn - 1;
				else j2 = p - r;
				for (int j = j1; j <= j2; j++)
				{
					a[s2][j] = (a[s1][j] - a[s1][j - 1])
						/ ndu[pk + 1][rk + j];
					d += a[s2][j] * ndu[rk + j][pk];
				}
				if (r <= pk)
				{
					a[s2][rn] = -a[s1][rn - 1] / ndu[pk + 1][r];
					d += a[s2][rn] * ndu[r][pk];

				}
				basisDu[i][r] = d;
				s1 = s1 + s2; s2 = s1 - s2; s1 = s1 - s2;
			}
		}
		r = p;
		for (int i = 1; i <= n; i++)
		{
			for (int j = 0; j <= p; j++)
			basisDu[i][j] *= r;
			r *= (p - k);
		}	
	}

	void NurbsLine::CreatLineWithTwoPoints(const point3d & p1, const point3d & p2, int degree)
	{
		this->m_CtrlPts.clear();
		this->m_Knots.clear();

		this->m_Degree = 1;
		this->m_Knots.push_back(0);
		this->m_Knots.push_back(0);
		this->m_Knots.push_back(1);
		this->m_Knots.push_back(1);
		this->m_CtrlPts.push_back(point4d(p1));
		this->m_CtrlPts.push_back(point4d(p2));
		if (degree > 1) {
			this->DegreeElevate(degree);
		}
	}

	/*计算u节点对应的曲线坐标点
	u：节点参数*/
	point3d NurbsLine::GetLinePoint(const double u)const
	{
		int m_id;
		varray<double> Nu;
		Nu.resize(m_Degree + 1);
		point3d res(0, 0, 0);
		double SumW = 0;
		m_id = FindSpan(u, m_Degree, m_CtrlPts.size(), m_Knots);
		BasisFuns(u, m_id, m_Degree, m_Knots, Nu);
		for (int i = 0; i <= m_Degree; i++)
		{
			res += Nu[i] * m_CtrlPts[m_id - m_Degree + i] * m_CtrlPts[m_id - m_Degree + i].w;
			SumW += Nu[i] * m_CtrlPts[m_id - m_Degree + i].w;
		}
		res /= SumW;
		return res;
	}

	/*获取线上的点
	Unum：曲线点数量
	linePts：曲线点*/
	void NurbsLine::CalLinePoint(const int Unum, varray<point3d>& linePts)const
	{
		linePts.clear();
		double du = 1.0 / Unum;
		for (int i = 0; i <= Unum; i++)         //采用等分节点矢量插值B样条曲线
		{
			double u = du*i;
			point3d t = GetLinePoint(u);
			linePts.push_back(t);
		}
	}

	/*曲线矢值函数A(u)所有n阶导，矢值函数A(u)即NURBS曲线分子式
	u：参数
	n：导矢阶数
	Der：A(u)所有n阶导，当曲线为B样条时(权重为1)，即求得B样条导矢*/
	void NurbsLine::PtsDerivsAu(const double u, const int n, varray<point4d>& Der)const
	{
		Der.clear();

		int span = 0, p = m_Degree, num = m_CtrlPts.size(), tmp;
		int du = n < p ? n : p;
		//计算基函数
		varray<varray<double>> ndu;
		span = FindSpan(u, p, num, m_Knots);
		AllBasisFuns(u, span, p, m_Knots, ndu);

		varray<varray<point4d>> PKK;
		varray<point4d> PK;
		for (int i = 0; i<num; i++)
		{
			point4d pt;
			pt = m_CtrlPts[i] * m_CtrlPts[i].w;
			pt.w = m_CtrlPts[i].w;
			PK.push_back(pt);
		}
		PKK.push_back(PK);
		
		for (int k = 1; k <= du; k++)
		{
			PK.clear();
			tmp = p - k + 1;
			for (int i = 0; i<num - k; i++)
			{
				point4d pt;
				pt = tmp*(PKK[k - 1][i + 1] - PKK[k - 1][i]) / (m_Knots[i + p + 1] - m_Knots[i + k]);
				pt.w = tmp*(PKK[k - 1][i + 1].w - PKK[k - 1][i].w) / (m_Knots[i + p + 1] - m_Knots[i + k]);
				PK.push_back(pt);
			}
			PKK.push_back(PK);
		}

		//计算导数
		for (int k = 0; k <= du; k++)
		{
			point4d pt;
			double sw = 0;
			for (int j = 0; j <= p - k; j++)
			{
				pt += ndu[j][p - k] * PKK[k][span - p + j];
				sw += ndu[j][p - k] * PKK[k][span - p + j].w;
			}
			pt.w = sw;
			Der.push_back(pt);
		}
	}

	/*曲线上u处所有n阶导
	u：参数
	n：导矢阶数
	Der：所有n阶导 */
	void NurbsLine::PtsDerivs(const double u, const int n, varray<point4d>& Der) const
	{
		Der.clear();
		int du = n < m_Degree ? n : m_Degree;
		varray<point4d> DerAu;
		PtsDerivsAu(u, du, DerAu);
		for (int k = 0; k <= du; k++)
		{
			for (int i = 1; i <= k; i++)
				DerAu[k] = DerAu[k] - Bin(k,i) * DerAu[i].w*Der[k - i];

			point4d pt;
			pt = DerAu[k] / DerAu[0].w;
			if (k == 0)
			{
				pt.w = DerAu[0].w;
			}
			Der.push_back(pt);
		}
	}

	/*曲线n阶导
	step：参数域等分步进量
	Der：曲线点对应的n阶导*/
	void NurbsLine::CurveDerivs(const int n, const double step, varray<varray<point3d>>& Der)
	{
		Der.clear();
		for (double u = 0; u <= 1; u += step)
		{
			varray<point4d> CK;
			PtsDerivs(u, n, CK);
			varray<point3d> CK3;
			for (int i = 0; i < CK.size(); ++i)
				CK3.push_back(CK[i]);
			Der.push_back(CK3);
		}
	}

	/*节点插入
	u：需要插入的节点
	r：插入次数*/
	void NurbsLine::KnotInsert(const double u, const int r)
	{
		if (r <= 0)return;

		varray<point4d> newbpt;
		varray<double> newKonts;
		int n = m_CtrlPts.size() - 1;//原始控制点数-1
		newKonts.resize(m_Knots.size() + r);
		newbpt.resize(newKonts.size() - m_Degree - 1);
		int a, m;
		m = n + m_Degree + 1;
		a = FindSpan(u, m_Degree, m_CtrlPts.size(), m_Knots);
		for (int j = 0; j <= a - m_Degree; j++)
			newbpt[j] = m_CtrlPts[j];
		for (int j = a; j <= n; j++)
			newbpt[j + r] = m_CtrlPts[j];
		for (int j = 0; j <= a; j++)
			newKonts[j] = m_Knots[j];
		for (int j = a + m_Degree + 1; j <= m; j++)
			newKonts[j + r] = m_Knots[j];
		int i = a + m_Degree;
		int k = a + m_Degree + r;
		while (u <= m_Knots[i] && i>a)
		{
			newbpt[k - m_Degree - 1] = m_CtrlPts[i - m_Degree - 1];
			newKonts[k] = m_Knots[i];
			k--;
			i--;
		}
		newbpt[k - m_Degree - 1] = newbpt[k - m_Degree];
		//插入节点r次，逆序
		for (int j = r - 1; j >= 0; j--)
		{
			for (int L = 1; L <= m_Degree; L++)
			{
				int ind = k - m_Degree + L;
				double alfa = newKonts[k + L] - u, beta;
				if (alfa <= 0.0)
				{
					newbpt[ind - 1] = newbpt[ind];
				}
				else
				{
					alfa = alfa / (newKonts[k + L] - m_Knots[i - m_Degree + L]);
					double nw = alfa*newbpt[ind - 1].w + (1.0f - alfa)*newbpt[ind].w;
					beta = alfa*newbpt[ind - 1].w / nw;
					newbpt[ind - 1] = beta*newbpt[ind - 1] + (1.0f - beta)*newbpt[ind];
					newbpt[ind - 1].w = nw;
				}
			}
			newKonts[k] = u;
			k--;
		}
		m_Knots = newKonts;
		m_CtrlPts = newbpt;
	}

	/*节点去除,返回实际消去的次数
	u：需要删除的节点
	r：删除次数	*/
	int NurbsLine::KnotRemove(const double u, const int r)
	{
		int sp = FindSpan(u, m_Degree, m_CtrlPts.size(), m_Knots);
		size_t n = m_CtrlPts.size() - 1;
		size_t m = n + m_Degree + 1;
		size_t ord = m_Degree + 1;

		int s = 0;
		for (int i = 0; i < m_Knots.size(); ++i)
		{
			if (u == m_Knots[i])
				s++;
			if (u < m_Knots[i])
				break;
		}

		double wmin = 1, Pmax = 0;
		for (int i = 0; i < m_CtrlPts.size(); ++i)
		{
			if (m_CtrlPts[i].w < wmin)
				wmin = m_CtrlPts[i].w;
			double pd = m_CtrlPts[i].Magnitude();
			if (pd > Pmax)
				Pmax = pd;
		}
		double TOL = 999;//wmin / (1 + Pmax);//容差

		int fout = (2 * sp - s - m_Degree) / 2;
		int last = sp - s;
		int first = sp - m_Degree;
		int t = 0;
		for (t = 0; t < r; ++t)
		{
			varray<point4d> temp;
			temp.resize(2 * m_Degree + 1);
			int off = first - 1;
			temp[0] = m_CtrlPts[off];
			temp[last + 1 - off] = m_CtrlPts[last + 1];
			int i = first, j = last, ii = 1, jj = last - off, remflag = 0;
			while (j - i > t)
			{
				double alfi = (u - m_Knots[i]) / (m_Knots[i + ord + t] - m_Knots[i]);
				double alfj = (u - m_Knots[j - t]) / (m_Knots[j + ord] - m_Knots[j - t]);
				temp[ii] = (m_CtrlPts[i] - (1.0 - alfi)*temp[ii - 1]) / alfi;
				temp[ii].w = (m_CtrlPts[i].w - (1.0 - alfi)*temp[ii - 1].w) / alfi;
				temp[jj] = (m_CtrlPts[j] - alfj*temp[jj + 1]) / (1.0 - alfj);
				temp[jj].w = (m_CtrlPts[j].w - alfj*temp[jj + 1].w) / (1.0 - alfj);

				i++;
				ii++;
				j--;
				jj--;
			}

			if (j - i < t)
			{
				point4d dp = temp[ii - 1] - temp[jj + 1];
				dp.w = temp[ii - 1].w - temp[jj + 1].w;
				double ds = sqrt(dp.SquareMagnitude() + dp.w*dp.w);
				if (ds <= TOL)
					remflag = 1;
			}
			else
			{
				double alfi = (u - m_Knots[i]) / (m_Knots[i + ord + t] - m_Knots[i]);
				point4d dp = m_CtrlPts[i] - (alfi*temp[ii + t + 1] + (1.0 - alfi)*temp[ii - 1]);
				dp.w = m_CtrlPts[i].w - (alfi*temp[ii + t + 1].w + (1.0 - alfi)*temp[ii - 1].w);
				double ds = sqrt(dp.SquareMagnitude() + dp.w*dp.w);
				if (ds <= TOL)
					remflag = 1;
			}
			if (remflag == 0)
				break;
			else
			{
				i = first;
				j = last;
				while (j - i > t)
				{
					m_CtrlPts[i] = temp[i - off];
					m_CtrlPts[j] = temp[j - off];
					i++;
					j--;
				}
			}
			first--;
			last++;
		}

		if (t == 0)
			return 0;
		for (int k = sp + 1; k <= m; ++k)
			m_Knots[k - t] = m_Knots[k];
		int j = fout, i = j;
		for (int k = 1; k < t; ++k)
			if (k % 2 == 1)
				i++;
			else
				j--;
		for (int k = i + 1; k <= n; ++k)
		{
			m_CtrlPts[j] = m_CtrlPts[k];
			j++;
		}
		for (int k = 0; k < t; ++k)
		{
			m_Knots.pop_back();
			m_CtrlPts.pop_back();
		}
		return t;
	}

	/*节点细化
	u:需要插入的节点*/
	void NurbsLine::KnotsRefine(const varray<double>& u)
	{
		if (u.size() <= 0)return;

		varray<point4d> newbpt;
		varray<double> newKonts;
		int n = m_CtrlPts.size() - 1;//原始控制点数-1
		int r = u.size() - 1;//插入的节点数-1
		newKonts.resize(m_Knots.size()+ u.size());
		newbpt.resize(newKonts.size() - m_Degree - 1);
		int a, b, m;
		m = n + m_Degree + 1;
		a = FindSpan(u[0], m_Degree, m_CtrlPts.size(), m_Knots);
		b = FindSpan(u[r], m_Degree, m_CtrlPts.size(), m_Knots);
		b = b + 1;
		for (int j = 0; j <= a - m_Degree; j++)
			newbpt[j] = m_CtrlPts[j];
		for (int j = b - 1; j <= n; j++)
			newbpt[j + r + 1] = m_CtrlPts[j];
		for (int j = 0; j <= a; j++)
			newKonts[j] = m_Knots[j];
		for (int j = b + m_Degree; j <= m; j++)
			newKonts[j + r + 1] = m_Knots[j];

		int i = b + m_Degree - 1;
		int k = b + m_Degree + r;
		for (int j = r; j >= 0; j--)
		{
			while (u[j] <= m_Knots[i] && i>a)
			{
				newbpt[k - m_Degree - 1] = m_CtrlPts[i - m_Degree - 1];
				newKonts[k] = m_Knots[i];
				k--;
				i--;
			}
			newbpt[k - m_Degree - 1] = newbpt[k - m_Degree];
			for (int L = 1; L <= m_Degree; L++)
			{
				int ind = k - m_Degree + L;
				double alfa = newKonts[k + L] - u[j], beta;
				if (alfa <= 0.0)
				{
					newbpt[ind - 1] = newbpt[ind];
				}
				else
				{
					alfa = alfa / (newKonts[k + L] - m_Knots[i - m_Degree + L]);
					double nw = alfa*newbpt[ind - 1].w + (1.0f - alfa)*newbpt[ind].w;
					beta = alfa*newbpt[ind - 1].w / nw;
					newbpt[ind - 1] = beta*newbpt[ind - 1] + (1.0f - beta)*newbpt[ind];
					newbpt[ind - 1].w = nw;
				}
			}
			newKonts[k] = u[j];
			k--;
		}
		m_Knots = newKonts;
		m_CtrlPts = newbpt;
	}

	/* 曲线升阶
	degree：升阶后次数*/
	void NurbsLine::DegreeElevate(const int degree)
	{
		int t = degree - m_Degree;
		if (t < 1)
			return;

		varray<double> Uh;//升阶后的节点矢量
		varray<point4d> Qw;//升阶后的控制点

		{
			int sz = 1;
			for (int i = 0; i < m_Knots.size() - 1; ++i)
			{
				if (m_Knots[i] != m_Knots[i + 1])
					++sz;
			}
			Uh.resize(m_Knots.size() + 2*t+(sz - 2)*t);
			Qw.resize(Uh.size() - m_Degree - 1 - t);
		}

		int n = m_CtrlPts.size() - 1;
		int m = n + m_Degree + 1;
		int ph = m_Degree + t, ph2 = ph / 2;

		varray<varray<double>> bezalfs;
		bezalfs.resize(ph + 1);
		for (int i = 0; i < bezalfs.size(); ++i)
			bezalfs[i].resize(m_Degree + 1);

		varray<double> alphas(m_Degree - 1, 0);

		varray<point4d> bpts(m_Degree + 1, point4d()), ebpts(ph + 1, point4d()), Nextbpts(m_Degree - 1, point4d());

		//计算升阶系数
		bezalfs[0][0] = bezalfs[ph][m_Degree] = 1.0;
		for (int i = 1; i <= ph2; ++i)
		{
			double inv = 1.0 / Bin(ph, i);
			int mpi = m_Degree < i ? m_Degree : i;
			for (int j = 0 > i - t ? 0 : i - t; j <= mpi; ++j)
				bezalfs[i][j] = inv*Bin(m_Degree, j)*Bin(t, i - j);
		}
		for (int i = ph2 + 1; i < ph; ++i)
		{
			int mpi = m_Degree < i ? m_Degree : i;
			for (int j = 0 > i - t ? 0 : i - t; j <= mpi; ++j)
				bezalfs[i][j] = bezalfs[ph - i][m_Degree - j];
		}
		int mh = ph, kind = ph + 1;
		int r = -1;
		int a = m_Degree, b = m_Degree + 1, cind = 1;
		double ua = m_Knots[0];
		Qw[0] = m_CtrlPts[0];

		for (int i = 0; i <= ph; ++i)
			Uh[i] = ua;
		for (int i = 0; i <= m_Degree; ++i)
			bpts[i] = m_CtrlPts[i];

		while (b < m)
		{
			int i = b;
			while (b < m && m_Knots[b] == m_Knots[b + 1])
				++b;
			int mul = b - i + 1;
			mh += mul + t;
			double ub = m_Knots[b];
			int oldr = r;
			r = m_Degree - mul;
			int lbz = 0, rbz = 0;
			//插入节点r次
			if (oldr > 0)
				lbz = (oldr + 2) / 2;
			else
				lbz = 1;
			if (r > 0)
				rbz = ph - (r + 1) / 2;
			else
				rbz = ph;
			//插入节点获得贝塞尔曲线段
			if (r > 0)
			{
				double numer = ub - ua;
				varray<double> alfs;
				alfs.resize(m_Degree - mul);
				for (int k = m_Degree; k > mul; --k)
					alfs[k - mul - 1] = numer / (m_Knots[a + k] - ua);
				for (int j = 1; j <= r; ++j)
				{
					int save = r - j, s = mul + j;
					for (int k = m_Degree; k >= s; --k)
					{
						bpts[k] = alfs[k - s] * bpts[k] + (1 - alfs[k - s])*bpts[k - 1];
						bpts[k].w = alfs[k - s] * bpts[k].w + (1 - alfs[k - s])*bpts[k - 1].w;
					}
					Nextbpts[save] = bpts[m_Degree];
				}
			}

			//对贝塞尔曲线升阶
			for (i = lbz; i <= ph; ++i)
			{
				ebpts[i] = point4d(0,0,0,0);
				int mpi = m_Degree < i ? m_Degree : i;
				for (int j = 0 > i - t ? 0 : i - t; j <= mpi; ++j)
				{
					ebpts[i] += bezalfs[i][j] * bpts[j];
					ebpts[i].w += bezalfs[i][j] * bpts[j].w;
				}
			}

			//消去节点u=m_Knots[a]
			if (oldr > 1)
			{
				int first = kind - 2, last = kind;
				double den = ub - ua;
				double bet = (ub - Uh[kind - 1]) / den;
				for (int tr = 1; tr < oldr; ++tr)
				{
					i = first;
					int j = last;
					int kj = j - kind + 1;
					while (j - i > tr)//计算消去后的控制点
					{
						if (i < cind)
						{
							double alf = (ub - Uh[i]) / (ua - Uh[i]);
							Qw[i] = alf*Qw[i] + (1 - alf)*Qw[i - 1];
							Qw[i].w = alf*Qw[i].w + (1 - alf)*Qw[i - 1].w;
						}
						if (j >= lbz)
						{
							if (j - tr <= kind - ph + oldr)
							{
								double gam = (ub - Uh[j - tr]) / den;
								ebpts[kj] = gam*ebpts[kj] + (1 - bet)*ebpts[kj + 1];
								ebpts[kj].w = gam*ebpts[kj].w + (1 - bet)*ebpts[kj + 1].w;
							}
							else
							{
								ebpts[kj] = bet*ebpts[kj] + (1 - bet)*ebpts[kj + 1];
								ebpts[kj].w = bet*ebpts[kj].w + (1 - bet)*ebpts[kj + 1].w;
							}
						}
						++i; --j; --kj;
					}
					--first; ++last;
				}
			}//消去节点u=m_Knots[a]结束
			 //载入节点ua
			if (a != m_Degree)
				for (i = 0; i < ph - oldr; ++i)
				{
					Uh[kind] = ua;
					++kind;
				}
			//存入控制点
			for (int j = lbz; j <= rbz; ++j)
			{
				Qw[cind] = ebpts[j];
				++cind;
			}
			//为下次循环准备
			if (b < m)
			{
				for (int j = 0; j < r; ++j)
					bpts[j] = Nextbpts[j];
				for (int j = r; j <= m_Degree; ++j)
					bpts[j] = m_CtrlPts[b - m_Degree + j];
				a = b; ++b; ua = ub;
			}
			else//尾端节点
				for (i = 0; i <= ph; ++i)
					Uh[kind + i] = ub;
		}
		//删除容器多余元素及空间
		int nh = mh - ph;//控制点数量
		Qw.erase(Qw.begin() + nh, Qw.end());

		int nk = nh + m_Degree + t + 1;//节点矢量数量
		Uh.erase(Uh.begin() + nk, Uh.end());

		m_CtrlPts = Qw;
		m_Knots = Uh;
		m_Degree = m_Knots.size() - m_CtrlPts.size() - 1;
	}

	void NurbsLine::Segmentation(const double u, varray<NurbsLine>& lines)
	{
		lines.clear();
		if (u <= 0 || u >= 1)
			return;

		lines.resize(2);
		
		NurbsLine l0(*this);
		int insN = m_Degree;
		int span = 0;
		if (IsInSet(u, m_Knots))
		{
			span = FindSpan(u, m_Degree, m_CtrlPts.size(), m_Knots);
			for (int i = span; i > 0; --i)
			{
				if (u == m_Knots[i])
					--insN;
				else
					break;
			}
		}
		if (insN > 0)
			l0.KnotInsert(u, insN);
		lines[0].m_Degree = l0.m_Degree;
		lines[1].m_Degree = l0.m_Degree;
		//计算新节点矢量
		for (int i = 0; i < l0.m_Degree + 1; ++i)
		{
			lines[0].m_Knots.push_back(0);
			lines[1].m_Knots.push_back(0);
		}
		span = l0.FindSpan(u, l0.m_Degree, l0.m_CtrlPts.size(), l0.m_Knots);
		for (int i = l0.m_Degree + 1; i <= span - l0.m_Degree; ++i)
		{
			double newu = l0.m_Knots[i] / u;
			lines[0].m_Knots.push_back(newu);
		}
		for (int i = span + 1; i < l0.m_CtrlPts.size(); ++i)
		{
			double newu = 1 - (1 - l0.m_Knots[i]) / (1 - u);
			lines[1].m_Knots.push_back(newu);
		}
		for (int i = 0; i < l0.m_Degree + 1; ++i)
		{
			lines[0].m_Knots.push_back(1);
			lines[1].m_Knots.push_back(1);
		}
		//控制点
		int i = 0;
		for (i; i < lines[0].m_Knots.size() - lines[0].m_Degree - 1; ++i)
			lines[0].m_CtrlPts.push_back(l0.m_CtrlPts[i]);
		for (--i; i < l0.m_CtrlPts.size(); ++i)
			lines[1].m_CtrlPts.push_back(l0.m_CtrlPts[i]);
	}

	/*NURBS曲线分解为BEZIER
	Qw：BEZIER曲线段控制点*/
	void NurbsLine::Decompose(varray<varray<point4d>>& Qw)
	{
		int n = m_CtrlPts.size() - 1;
		int m = n + m_Degree + 1;
		int a = m_Degree, b = m_Degree + 1;
		int nb = 0;
		for (int i = 1; i < m_Knots.size(); ++i)
			if (m_Knots[i] != m_Knots[i - 1])
				nb++;
		Qw.resize(nb);
		for (int i = 0; i < nb; ++i)
			Qw[i].resize(m_Degree + 1);

		nb = 0;
		for (int i = 0; i <= m_Degree; ++i)
			Qw[nb][i] = m_CtrlPts[i];

		while (b < m)
		{
			int i = b;
			while (b < m && m_Knots[b + 1] == m_Knots[b])
				b++;
			int mult = b - i + 1;
			if (mult < m_Degree)
			{
				double numer = m_Knots[b] - m_Knots[a];
				varray<double> alphas(m_Degree - 1);
				for (int j = m_Degree; j > mult; --j)
					alphas[j - mult - 1] = numer / (m_Knots[a + j] - m_Knots[a]);
				int r = m_Degree - mult;
				for (int j = 1; j <= r; ++j)
				{
					int save = r - j;
					int s = mult + j;
					for (int k = m_Degree; k >= s; --k)
					{
						double alpha = alphas[k - a];
						Qw[nb][k] = alpha*Qw[nb][k] + (1 - alpha)*Qw[nb][k - 1];
						Qw[nb][k].w = alpha*Qw[nb][k].w + (1 - alpha)*Qw[nb][k - 1].w;
					}
					if (b < m)
						Qw[nb + 1][save] = Qw[nb][m_Degree];
				}
			}
			nb++;
			if (b < m)
			{
				for (int i = m_Degree - mult; i <= m_Degree; ++i)
					Qw[nb][i] = m_CtrlPts[b - m_Degree + i];
				a = b;
				b++;
			}
		}
	}

	void NurbsLine::CruveReverse()
	{
		int knotsLth = m_Knots.size();
		int cptLth = m_CtrlPts.size();
		varray<double> tempKnots;
		varray<point4d> tempPts;
		for (int i = 0; i < m_Degree + 1; i++)
		{
			tempKnots.push_back(m_Knots[i]);
		}
		int imax = knotsLth - 1 - 2 * m_Degree - 1;  //m-2p-1

		if (imax > 0)
		{
			for (int i = imax; i >= 1; i--)
			{
				tempKnots.push_back(m_Knots[m_Degree + i] * (-1.0) + m_Knots[0] + m_Knots[knotsLth - 1]);
			}
		}
		for (int i = 0; i < m_Degree + 1; i++)
		{
			tempKnots.push_back(m_Knots[knotsLth - 1]);
		}
		for (int i = 0; i < cptLth; i++)
		{
			tempPts.push_back(m_CtrlPts[cptLth - 1 - i]);
		}

		if (tempKnots.size() != knotsLth)
		{
			std::cout << "\n节点矢量数量不正确" << std::endl;
		}
		if (tempPts.size() != cptLth)
		{
			std::cout << "\n控制点数量不正确" << std::endl;
		}
		m_CtrlPts = tempPts;
		m_Knots = tempKnots;
		return;
	}

	void NurbsSurface::SetSurface(const int uDegree, const int vDegree, const int uNum, const int vNum, const varray<double>& uKnots, const varray<double>& vKnots)
	{
		m_uDegree = uDegree;
		m_vDegree = vDegree;
		m_uNum = uNum;
		m_vNum = vNum;
		m_uKnots = uKnots;
		m_vKnots = vKnots;
	}



	//计算（u，v）对应的曲面上的点
	point3d NurbsSurface::GetSurFacePoint(const double u, const double v)const
	{
		int m_uid = 0, m_vid = 0;
		varray<double> Nu, Nv;
		Nu.resize(m_uDegree + 1);
		Nv.resize(m_vDegree + 1);
		m_uid = FindSpan(u, m_uDegree, m_uNum, m_uKnots);
		BasisFuns(u, m_uid, m_uDegree, m_uKnots, Nu);
		m_vid = FindSpan(v, m_vDegree, m_vNum, m_vKnots);
		BasisFuns(v, m_vid, m_vDegree, m_vKnots, Nv);

		point3d res(0, 0, 0);
		double pw = 0.0;
		for (int j = 0; j <= m_vDegree; j++)
		{
			for (int i = 0; i <= m_uDegree; i++)
			{
				point4d bpti = m_CtrlPts[(m_uid - m_uDegree + i)*m_vNum + m_vid - m_vDegree + j];
				res += Nu[i] * Nv[j] * bpti*bpti.w;
				pw += Nu[i] * Nv[j] * bpti.w;
			}
		}
		res /= pw;
		return res;
	}
	
	//计算四边形面片显示数据
	threadParam NurbsSurface::CalQuads(const int Unum, const int Vnum)const
	{
		varray<varray<point3d>> quads;
		varray<varray<point3d>> lines;
		quads.clear();
		lines.clear();
		varray<varray<point3d>> L_u;
		double du = 1.0 / Unum;
		double dv = 1.0 / Vnum;
		for (int i = 0; i <= Unum; i++)
		{
			double u = du*i;
			varray<point3d> line;
			for (int j = 0; j <= Vnum; j++)
			{
				double v = dv*j;
				point3d t = GetSurFacePoint(u, v);
				line.push_back(t);
			}
			L_u.push_back(line);
		}
		
		for (int i = 0; i < L_u.size() - 1; ++i)
		{
			for (int j = 0; j < L_u[i].size() - 1; ++j)
			{
				varray<point3d> quad;
				quad.push_back(L_u[i][j]);
				quad.push_back(L_u[i + 1][j]);
				quad.push_back(L_u[i + 1][j + 1]);
				quad.push_back(L_u[i][j + 1]);
				quads.push_back(quad);
			}
		}

		varray<point3d> l0, l1;
		for (int i = 0; i < L_u.size(); ++i)
		{
			l0.push_back(L_u[i][0]);
			l1.push_back(L_u[i][L_u[i].size() - 1]);
		}
		lines.push_back(l0);//u0
		lines.push_back(l1);//u1
		lines.push_back(L_u[0]);//v0
		lines.push_back(L_u[L_u.size() - 1]);//v1
		threadParam param = { quads,lines };
		return param;
	}

	/*Coons插值
	EndgCtrlPts:边界控制点（v=0,u=0,v=1,u=1顺序）*/
	void NurbsSurface::CoonsInterpolate(const varray<varray<point4d>>& EdgeCtrlPts)
	{
		varray<varray<point4d>> coonsPatchCtrlpts;
		int upnum = EdgeCtrlPts[0].size();//u方向数量
		int vpnum = EdgeCtrlPts[1].size();//v方向数量
		point4d P00, P10, P01, P11;//角点
		P00 = EdgeCtrlPts[0][0];
		P10 = EdgeCtrlPts[0][upnum - 1];
		P01 = EdgeCtrlPts[2][0];
		P11 = EdgeCtrlPts[2][upnum - 1];
		for (int i = 0; i < upnum; ++i)
		{
			double tui = 1.0*i / (upnum - 1);
			varray<point4d> Ctrlpts_i;
			for (int j = 0; j < vpnum; ++j)
			{
				double tvj = 1.0*j / (vpnum - 1);
				point4d Pij = (1 - tui)*EdgeCtrlPts[1][j] + tui * EdgeCtrlPts[3][j]
					+ (1 - tvj)*EdgeCtrlPts[0][i] + tvj * EdgeCtrlPts[2][i]
					- (1 - tvj)*((1 - tui)*P00 + tui * P10)
					- tvj * ((1 - tui)*P01 + tui * P11);
				Pij.w = (1 - tui)*EdgeCtrlPts[1][j].w + tui * EdgeCtrlPts[3][j].w
					+ (1 - tvj)*EdgeCtrlPts[0][i].w + tvj * EdgeCtrlPts[2][i].w
					- (1 - tvj)*((1 - tui)*P00.w + tui * P10.w)
					- tvj * ((1 - tui)*P01.w + tui * P11.w);

				Ctrlpts_i.push_back(Pij);
			}
			coonsPatchCtrlpts.push_back(Ctrlpts_i);
		}
		ReduceVarrayDim(coonsPatchCtrlpts, m_CtrlPts, true);
	}

	/*Coons插值
	EdgeLines:边界曲线, F(u)v=0,F(v)u=0,F(u)v=1,F(v)u=1顺序）*/
	void NurbsSurface::CoonsInterpolate(const varray<NurbsLine>& EdgeLines)
	{
		if (EdgeLines.size() != 4)
			return;
		SetSurface(EdgeLines[0].m_Degree, EdgeLines[1].m_Degree, EdgeLines[0].m_CtrlPts.size(), EdgeLines[1].m_CtrlPts.size(),
			EdgeLines[0].m_Knots, EdgeLines[1].m_Knots);
		varray<varray<point4d>> cpt;
		for (int i = 0; i < EdgeLines.size(); ++i)
			cpt.push_back(EdgeLines[i].m_CtrlPts);
		CoonsInterpolate(cpt);
	}

	//曲面升阶
	//Udegree,Vdegree:升阶后次数
	void NurbsSurface::DegreeElevate(const int Udegree, const int Vdegree)
	{
		int tu = Udegree - m_uDegree;
		int tv = Vdegree - m_vDegree;
		//U方向
		if (tu > 0)
		{
			varray<varray<point4d>> NewConPoint;
			NurbsLine line;
			
			for (int i = 0; i < m_vNum; ++i)
			{
				line.m_Degree = m_uDegree;
				line.m_Knots = m_uKnots;
				line.m_CtrlPts.clear();
				for (int j = 0; j < m_uNum; ++j)
					line.m_CtrlPts.push_back(m_CtrlPts[m_vNum*j + i]);
				line.DegreeElevate(Udegree);
				NewConPoint.push_back(line.m_CtrlPts);
			}
			m_uDegree = Udegree;
			m_uKnots = line.m_Knots;
			m_uNum = line.m_CtrlPts.size();
			m_CtrlPts.resize(m_uNum*m_vNum);
			for (int i = 0; i < NewConPoint.size(); ++i)
				for (int j = 0; j < NewConPoint[i].size(); ++j)
					m_CtrlPts[m_vNum*j + i] = NewConPoint[i][j];
		}
		//V方向
		if (tv > 0)
		{
			varray<varray<point4d>> NewConPoint;
			NurbsLine line;
			
			for (int i = 0; i < m_uNum; ++i)
			{
				line.m_Degree = m_vDegree;
				line.m_Knots = m_vKnots;
				line.m_CtrlPts.clear();
				for (int j = 0; j < m_vNum; ++j)
					line.m_CtrlPts.push_back(m_CtrlPts[m_vNum*i + j]);
				line.DegreeElevate(Vdegree);
				NewConPoint.push_back(line.m_CtrlPts);
			}
			m_vDegree = Vdegree;
			m_vKnots = line.m_Knots;
			m_vNum = line.m_CtrlPts.size();
			m_CtrlPts.clear();
			for (int i = 0; i < NewConPoint.size(); ++i)
				for (int j = 0; j < NewConPoint[i].size(); ++j)
					m_CtrlPts.push_back(NewConPoint[i][j]);
		}
	}

	//曲面节点插入
	//Uknot,Vknot:需要插入的节点
	void NurbsSurface::KnotsRefine(const varray<double>& Uknot, const varray<double>& Vknot)
	{
		//U方向
		if (Uknot.size() > 0)
		{
			varray<varray<point4d>> NewConPoint;
			NurbsLine line;
			
			for (int i = 0; i < m_vNum; ++i)
			{
				line.m_Degree = m_uDegree;
				line.m_Knots = m_uKnots;
				line.m_CtrlPts.clear();
				for (int j = 0; j < m_uNum; ++j)
					line.m_CtrlPts.push_back(m_CtrlPts[m_vNum*j + i]);
				line.KnotsRefine(Uknot);
				NewConPoint.push_back(line.m_CtrlPts);
			}
			m_uKnots = line.m_Knots;
			m_uNum = line.m_CtrlPts.size();
			m_CtrlPts.resize(m_uNum*m_vNum);
			for (int i = 0; i < NewConPoint.size(); ++i)
				for (int j = 0; j < NewConPoint[i].size(); ++j)
					m_CtrlPts[m_vNum*j + i] = NewConPoint[i][j];
		}
		//V方向
		if (Vknot.size() > 0)
		{
			varray<varray<point4d>> NewConPoint;
			NurbsLine line;
			
			for (int i = 0; i < m_uNum; ++i)
			{
				line.m_Degree = m_vDegree;
				line.m_Knots = m_vKnots;
				line.m_CtrlPts.clear();
				for (int j = 0; j < m_vNum; ++j)
					line.m_CtrlPts.push_back(m_CtrlPts[m_vNum*i + j]);
				line.KnotsRefine(Vknot);
				NewConPoint.push_back(line.m_CtrlPts);
			}
			m_vKnots = line.m_Knots;
			m_vNum = line.m_CtrlPts.size();
			m_CtrlPts.clear();
			for (int i = 0; i < NewConPoint.size(); ++i)
				for (int j = 0; j < NewConPoint[i].size(); ++j)
					m_CtrlPts.push_back(NewConPoint[i][j]);
		}
	}

	//曲面分段为Bezier曲面
	//dir:0=U方向，1=V方向
	//QW：Bezier曲面控制点
	void NurbsSurface::Decompose(const bool dir, varray<varray<varray<point4d>>>& QW)
	{
		int a, b, i, mult, save, s;
		double numer, alpha;
		varray<double> alphas;
		varray<double> NUknot;   //新的U向量
		varray<double> NVknot;   //新的V向量
		int nb=0;
		if (dir == 0)
		{
			//Bezier片数及控制点数量初始化
			for (int i = 0; i<m_uKnots.size() - 1; i++)
			{
				if (m_uKnots[i + 1] != m_uKnots[i])
				{
					nb++;
				}
			}
			QW.resize(nb);
			for (int i = 0; i<nb; i++)
			{
				QW[i].resize(m_vNum);
				for (int j = 0; j<m_vNum; j++)
				{
					QW[i][j].resize(m_uDegree + 1);
				}
			}
			//划分为Bezier片
			a = m_uDegree; b = m_uDegree + 1; nb = 0; alphas.resize(m_uDegree);
			for (int i = 0; i <= m_uDegree; i++)
			{
				NUknot.push_back(m_uKnots[i]);
			}
			for (int row = 0; row<m_vNum; row++)
			{
				for (int i = 0; i <= m_uDegree; i++)
				{
					QW[nb][row][i] = m_CtrlPts[m_vNum*i+row];
				}
			}
			while (b<m_uKnots.size() - 1)
			{
				i = b;
				while (b<m_uKnots.size() - 1 && m_uKnots[b + 1] == m_uKnots[b])
				{
					NUknot.push_back(m_uKnots[b]);
					b++;
				}
				mult = b - i + 1;  //插入个数
				if (mult<m_uDegree)
				{
					NUknot.push_back(m_uKnots[b]);
					numer = m_uKnots[b] - m_uKnots[a];
					for (int j = m_uDegree; j>mult; j--)
					{
						alphas[j - mult - 1] = numer / (m_uKnots[a + j] - m_uKnots[a]);
					}
					for (int j = 1; j <= m_uDegree - mult; j++)
					{
						save = m_uDegree - mult - j;
						s = mult + j;  //s个新的控制点
						for (int row = 0; row<m_vNum; row++)
						{
							for (int k = m_uDegree; k >= s; k--)
							{
								alpha = alphas[k - s];
								QW[nb][row][k] = alpha*QW[nb][row][k] + (1.0 - alpha)*QW[nb][row][k - 1];
								QW[nb][row][k].w = alpha*QW[nb][row][k].w + (1.0 - alpha)*QW[nb][row][k - 1].w;
							}
						}
						if (b<m_uKnots.size() - 1)
						{
							for (int row = 0; row<m_vNum; row++)
							{
								QW[nb + 1][row][save] = QW[nb][row][m_uDegree];
							}
						}
					}
				}
				nb = nb + 1;
				if (b<m_uKnots.size() - 1)
				{
					for (int row = 0; row<m_vNum; row++)
					{
						for (int i = m_uDegree - mult; i <= m_uDegree; i++)
						{
							QW[nb][row][i] = m_CtrlPts[m_vNum*(b - m_uDegree + i) + row];
						}
					}
					a = b; b++;
				}
			}
		}
		if (dir == 1)
		{
			//Bezier片数及控制点数量初始化
			for (int i = 0; i<m_vKnots.size() - 1; i++)
			{
				if (m_vKnots[i + 1] != m_vKnots[i])
				{
					nb++;
				}
			}
			QW.resize(nb);
			for (int i = 0; i<nb; i++)
			{
				QW[i].resize(m_vDegree + 1);
				for (int j = 0; j <= m_vDegree; j++)
				{
					QW[i][j].resize(m_uNum);
				}
			}

			a = m_vDegree; b = m_vDegree + 1; nb = 0; alphas.resize(m_vDegree);
			for (int i = 0; i <= m_vDegree; i++)
			{
				NVknot.push_back(m_vKnots[i]);
				for (int row = 0; row<m_uNum; row++)
				{
					QW[nb][i][row] = m_CtrlPts[m_vNum*row + i];
				}
			}
			while (b<m_vKnots.size() - 1)
			{
				i = b;
				while (b<m_vKnots.size() - 1 && m_vKnots[b + 1] == m_vKnots[b])
				{
					NVknot.push_back(m_vKnots[b]);
					b++;
				}
				NVknot.push_back(m_vKnots[b]);
				mult = b - i + 1;  //插入个数
				if (mult<m_vDegree)
				{
					numer = m_vKnots[b] - m_vKnots[a];
					for (int j = m_vDegree; j>mult; j--)
					{
						alphas[j - mult - 1] = numer / (m_vKnots[a + j] - m_vKnots[a]);
					}
					for (int j = 1; j <= m_vDegree - mult; j++)
					{
						NVknot.push_back(m_vKnots[b]);
						save = m_vDegree - mult - j;
						s = mult + j;  //s个新的控制点
						for (int row = 0; row<m_uNum; row++)
						{
							for (int k = m_vDegree; k >= s; k--)
							{
								alpha = alphas[k - s];
								QW[nb][k][row] = alpha*QW[nb][k][row] + (1.0 - alpha)*QW[nb][k - 1][row];
								QW[nb][k][row].w = alpha*QW[nb][k][row].w + (1.0 - alpha)*QW[nb][k - 1][row].w;
							}
						}
						if (b<m_vKnots.size() - 1)
						{
							for (int row = 0; row<m_uNum; row++)
							{
								QW[nb + 1][save][row] = QW[nb][m_vDegree][row];
							}
						}
					}
				}
				nb = nb + 1;
				if (b<m_vKnots.size() - 1)
				{
					for (int row = 0; row<m_uNum; row++)
					{
						for (int i = m_vDegree - mult; i <= m_vDegree; i++)
						{
							QW[nb][i][row] = m_CtrlPts[m_vNum*row + (b - m_vDegree + i)];
						}
					}
					a = b; b++;
				}
			}
		}
	}

	//根据控制点二维序号计算一维序号
	inline int NurbsSurface::CtrlPtsIdx(const int uIdx, const int vIdx)
	{
		return m_vNum*uIdx + vIdx;
	}

	//控制点排序转换为U-V
	void NurbsSurface::OrderCtrlPts()
	{
		NurbsSurface sf = *this;
		OrderCtrlPts(sf);
		m_CtrlPts = sf.m_CtrlPts;
		SetSurface(sf.m_uDegree, sf.m_vDegree, sf.m_uNum, sf.m_vNum, sf.m_uKnots, sf.m_vKnots);
	}

	//控制点排序转换为U-V
	void NurbsSurface::OrderCtrlPts(NurbsSurface& sf)
	{
		sf.m_CtrlPts.clear();
		for (int i = 0; i < m_vNum; ++i)
		{
			for (int j = 0; j < m_uNum; ++j)
			{
				sf.m_CtrlPts.push_back(m_CtrlPts[CtrlPtsIdx(j, i)]);
			}
		}
		std::swap(sf.m_uNum, sf.m_vNum);
		std::swap(sf.m_uDegree, sf.m_vDegree);
		std::swap(sf.m_uKnots, sf.m_vKnots);
	}

	void NurbsSurface::GetEdgeLines(varray<NurbsLine>& EdgeLines)
	{
		EdgeLines.clear();
		EdgeLines.resize(4);
		NurbsLine nl;
		int num_pt = this->m_CtrlPts.size();
		//第一条,U,V=0
		nl.m_Degree = this->m_uDegree;
		nl.m_Knots = this->m_uKnots;
		for (int i = 0; i < this->m_uNum; ++i) {
			nl.m_CtrlPts.push_back(this->m_CtrlPts[i]);
		}
		EdgeLines[0] = nl;

		//第二条,U,V=1
		nl.m_CtrlPts.clear();
		for (int i = num_pt-this->m_uNum; i < num_pt; ++i) {
			nl.m_CtrlPts.push_back(this->m_CtrlPts[i]);
		}
		EdgeLines[1] = nl;

		//第三条,V,U=0
		nl.m_Degree = this->m_vDegree;
		nl.m_Knots = this->m_vKnots;
		nl.m_CtrlPts.clear();
		for (int i = 0; i < this->m_vNum; ++i) {
			nl.m_CtrlPts.push_back(this->m_CtrlPts[i*this->m_uNum]);
		}
		EdgeLines[2] = nl;

		//第四条,V,U=1
		nl.m_CtrlPts.clear();
		for (int i = 0; i < this->m_vNum; ++i) {
			nl.m_CtrlPts.push_back(this->m_CtrlPts[(i+1)*this->m_uNum-1]);
		}
		EdgeLines[3] = nl;
	}

	void NurbsVol::SetVol(const int uDegree, const int vDegree, const int wDegree, const int uNum, const int vNum, const int wNum, const varray<double>& uKnots, const varray<double>& vKnots, const varray<double>& wKnots)
	{
		m_uDegree = uDegree;
		m_vDegree = vDegree;
		m_wDegree = wDegree;
		m_uNum = uNum;
		m_vNum = vNum;
		m_wNum = wNum;
		m_uKnots = uKnots;
		m_vKnots = vKnots;
		m_wKnots = wKnots;
	}

	//(u,v,w)对应的体上的点
	point3d NurbsVol::GetVolPoint(const double u, const double v, const double w)const
	{
		int r = FindSpan(u, m_uDegree, m_uNum, m_uKnots);
		varray<double> uNknot;
		BasisFuns(u, r, m_uDegree, m_uKnots, uNknot);

		int s = FindSpan(v, m_vDegree, m_vNum, m_vKnots);
		varray<double> vNknot;
		BasisFuns(v, s, m_vDegree, m_vKnots, vNknot);

		int t = FindSpan(w, m_wDegree, m_wNum, m_wKnots);
		varray<double> wNknot;
		BasisFuns(w, t, m_wDegree, m_wKnots, wNknot);

		point3d val;
		double pw = 0.0;
		int ii, jj, kk;
		ii = jj = kk = 0;
		for (int k = 0; k <= m_wDegree; k++)
		{
			kk = t - m_wDegree + k;
			for (int j = 0; j <= m_uDegree; j++)
			{
				jj = r - m_uDegree + j;
				for (int i = 0; i <= m_vDegree; i++)
				{
					ii = s - m_vDegree + i;
					point4d bpti = m_CtrlPts[kk*m_uNum*m_vNum + jj*m_vNum + ii];
					val += uNknot[j] * vNknot[i] * wNknot[k] * bpti*bpti.w;
					pw += uNknot[j] * vNknot[i] * wNknot[k] * bpti.w;
				}
			}
		}
		val /= pw;
		return val;
	}

	//计算四边形面片显示数据
	threadParamVOL NurbsVol::CalQuads(const int Unum, const int Vnum, const int Wnum) const
	{
		varray<varray<varray<point3d>>> quads;
		varray<varray<varray<point3d>>> lines;
		quads.clear();
		lines.clear();
		int a = 0;
		int num[3]{ Unum ,Vnum ,Wnum };//曲面细分度三个方向的num
		for (int i = 0; i < 6; ++i)
		{
			varray<varray<point3d>> Q,L;
			CalIsoSurface(a, i % 2, num[(a + 1) % 3], num[(a + 2) % 3], Q, L);
			quads.push_back(Q);
			lines.push_back(L);
			if (i % 2)a += 1;
		}
		threadParamVOL vol = { quads,lines };
		
		return vol;
	}

	//根据控制点三维序号计算一维序号
	inline	int NurbsVol::CtrlPtsIdx(const int uIdx, const int vIdx, const int wIdx)
	{
		return m_uNum*m_vNum*wIdx + m_vNum*uIdx + vIdx;
	}

	//控制点排序转换为U-V-W
	void NurbsVol::OrderCtrlPts()
	{
		NurbsVol vol;
		OrderCtrlPts(vol);
		m_CtrlPts = vol.m_CtrlPts;
		SetVol(vol.m_uDegree, vol.m_vDegree, m_wDegree, 
			vol.m_uNum, vol.m_vNum, m_wNum, 
			vol.m_uKnots, vol.m_vKnots, m_wKnots);
	}

	//控制点排序转换为U-V-W
	void NurbsVol::OrderCtrlPts(NurbsVol& vol)
	{
		vol.m_CtrlPts.clear();
		for (int k = 0; k < m_wNum; ++k)
		{
			for (int i = 0; i < m_vNum; ++i)
			{
				for (int j = 0; j < m_uNum; ++j)
				{
					vol.m_CtrlPts.push_back(m_CtrlPts[CtrlPtsIdx(j, i, k)]);
				}
			}
		}
		std::swap(vol.m_uNum, vol.m_vNum);
		std::swap(vol.m_uDegree, vol.m_vDegree);
		std::swap(vol.m_uKnots, vol.m_vKnots);
	}

	void NurbsVol::KnotsRefine(varray<double>& Uknot, varray<double>& Vknot, varray<double>& Wknot)
	{

		int u = Uknot.size(), v = Vknot.size(), w = Wknot.size();

		if (u > 0 && v > 0)
		{
			varray<varray<point4d>> NewCpts;
			NurbsSurface Suv;
			for (int k = 0; k < m_wNum; k++)
			{
				Suv.SetSurface(m_uDegree, m_vDegree, m_uNum, m_vNum, m_uKnots, m_vKnots);
				for (int j = 0; j < m_uNum; j++)
				{
					for (int i = 0; i < m_vNum; i++)
					{
						Suv.m_CtrlPts.push_back(m_CtrlPts[i + j * m_vNum + k * m_uNum*m_vNum]);
					}
				}

				Suv.KnotsRefine(Uknot, Vknot);
				NewCpts.push_back(Suv.m_CtrlPts);
				Suv.m_CtrlPts.clear();
			}
			m_CtrlPts.clear();
			m_uKnots = Suv.m_uKnots;
			m_vKnots = Suv.m_vKnots;
			m_vNum = Suv.m_vNum;
			m_uNum = Suv.m_uNum;
			for (int i = 0; i < NewCpts.size(); i++)
				for (int j = 0; j < NewCpts[i].size(); j++)
					m_CtrlPts.push_back(NewCpts[i][j]);
		}

		if (w > 0 && v > 0)
		{

			varray<varray<point4d>> NewCpts;
			NurbsSurface Suv;
			for (int k = 0; k < m_uNum; k++)
			{
				Suv.SetSurface(m_wDegree, m_vDegree, m_wNum, m_vNum, m_wKnots, m_vKnots);
				for (int j = 0; j < m_wNum; j++)
				{
					for (int i = 0; i < m_vNum; i++)
					{
						Suv.m_CtrlPts.push_back(m_CtrlPts[i + j * m_vNum * m_uNum + k * m_vNum]);
					}
				}
				Vknot.clear();
				Suv.KnotsRefine(Wknot, Vknot);
				NewCpts.push_back(Suv.m_CtrlPts);
				Suv.m_CtrlPts.clear();
			}
			m_CtrlPts.clear();
			m_wKnots = Suv.m_uKnots;
			m_wNum = Suv.m_uNum;

			for (int k = 0; k < m_uNum; k++) {
				for (int j = 0; j < NewCpts.size(); j++) {
					for (int i = 0; i < m_vNum; i++)
					{
						m_CtrlPts.push_back(NewCpts[j][i + k * m_vNum]);
					}
				}
			}

		}
	}

	//升阶
	//Udegree,Vdegree,Wdegree:升阶后次数
	void NurbsVol::DegreeElevate(const int Udegree, const int Vdegree, const int Wdegree)
	{
		int tu = Udegree - m_uDegree;
		int tv = Vdegree - m_vDegree;
		int tw = Wdegree - m_wDegree;
		
		//U方向
		if (tu > 0)
		{
			varray<varray<varray<point4d>>> NewConPoint;
			NurbsLine line;
			for (int k = 0; k < m_wNum; ++k)
			{
				varray<varray<point4d>> NewConPointSF;
				for (int j = 0; j < m_vNum; ++j)
				{
					line.m_Degree = m_uDegree;
					line.m_Knots = m_uKnots;
					line.m_CtrlPts.clear();
					for (int i = 0; i < m_uNum; ++i)
						line.m_CtrlPts.push_back(m_CtrlPts[CtrlPtsIdx(i, j, k)]);
					line.DegreeElevate(Udegree);
					NewConPointSF.push_back(line.m_CtrlPts);
				}
				NewConPoint.push_back(NewConPointSF);//[w][v][u]
			}
			m_uDegree = Udegree;
			m_uKnots = line.m_Knots;
			m_uNum = line.m_CtrlPts.size();
			m_CtrlPts.resize(m_uNum*m_vNum*m_wNum);
			for (int i = 0; i < NewConPoint.size(); ++i)
			{
				for (int j = 0; j < NewConPoint[i].size(); ++j)
				{
					for (int k = 0; k < NewConPoint[i][j].size(); ++k)
					{
						int idx = CtrlPtsIdx(k, j, i);
						m_CtrlPts[idx] = NewConPoint[i][j][k];
					}
				}
			}
		}
		//V方向
		if (tv > 0)
		{
			varray<varray<varray<point4d>>> NewConPoint;
			NurbsLine line;
			for (int k = 0; k < m_wNum; ++k)
			{
				varray<varray<point4d>> NewConPointSF;
				for (int i = 0; i < m_uNum; ++i)
				{
					line.m_Degree = m_vDegree;
					line.m_Knots = m_vKnots;
					line.m_CtrlPts.clear();
					for (int j = 0; j < m_vNum; ++j)
						line.m_CtrlPts.push_back(m_CtrlPts[CtrlPtsIdx(i, j, k)]);
					line.DegreeElevate(Vdegree);
					NewConPointSF.push_back(line.m_CtrlPts);
				}
				NewConPoint.push_back(NewConPointSF);//[w][u][v]
			}
			m_vDegree = Vdegree;
			m_vKnots = line.m_Knots;
			m_vNum = line.m_CtrlPts.size();
			m_CtrlPts.resize(m_uNum*m_vNum*m_wNum);
			for (int i = 0; i < NewConPoint.size(); ++i)
			{
				for (int j = 0; j < NewConPoint[i].size(); ++j)
				{
					for (int k = 0; k < NewConPoint[i][j].size(); ++k)
					{
						int idx = CtrlPtsIdx(j, k, i);
						m_CtrlPts[idx] = NewConPoint[i][j][k];
					}
				}
			}
		}
		//W方向
		if (tw > 0)
		{
			varray<varray<varray<point4d>>> NewConPoint;
			NurbsLine line;
			for (int i = 0; i < m_uNum; ++i)
			{
				varray<varray<point4d>> NewConPointSF;
				for (int j = 0; j < m_vNum; ++j)
				{
					line.m_Degree = m_wDegree;
					line.m_Knots = m_wKnots;
					line.m_CtrlPts.clear();
					for (int k = 0; k < m_wNum; ++k)
						line.m_CtrlPts.push_back(m_CtrlPts[CtrlPtsIdx(i, j, k)]);
					line.DegreeElevate(Wdegree);
					NewConPointSF.push_back(line.m_CtrlPts);
				}
				NewConPoint.push_back(NewConPointSF);//[u][v][w]
			}
			m_wDegree = Wdegree;
			m_wKnots = line.m_Knots;
			m_wNum = line.m_CtrlPts.size();
			m_CtrlPts.resize(m_uNum*m_vNum*m_wNum);
			for (int i = 0; i < NewConPoint.size(); ++i)
			{
				for (int j = 0; j < NewConPoint[i].size(); ++j)
				{
					for (int k = 0; k < NewConPoint[i][j].size(); ++k)
					{
						int idx = CtrlPtsIdx(i, j, k);
						m_CtrlPts[idx] = NewConPoint[i][j][k];
					}
				}
			}
		}
	}

	/*扫描生成Nurbs体模型
	pathT：扫描路径
	nurbsSF：起始截面
	K：截面实例数量,一般取路径控制点数量减1
	*/
	void NurbsVol::CreateSweepNurbsVol(const NurbsLine& pathT, const NurbsSurface & nurbsSF, const int K)
	{
		//u方向节点矢量，控制点数量
		m_uDegree = nurbsSF.m_uDegree;
		m_uNum = nurbsSF.m_uNum;
		m_uKnots = nurbsSF.m_uKnots;

		//v方向节点矢量，控制点数量
		m_vDegree = nurbsSF.m_vDegree;
		m_vNum = nurbsSF.m_vNum;
		m_vKnots = nurbsSF.m_vKnots;

		m_wDegree = pathT.m_Degree;
		m_wKnots.clear();
		m_CtrlPts.clear();

		varray<double> pos;
		InsLocation(pathT, K, pos);//同时完成W方向节点矢量
		varray<varray<point4d>> TranMat;
		LocalCoordinates(pathT, pos, TranMat);
		varray<varray<point4d>> nurbsSFctrlPts;
		varray<varray<varray<double>>> SFw;
		varray<varray<double>> sws;
		for (int i = 0; i < nurbsSF.m_uNum; ++i)
		{
			varray<point4d> vctrl;
			varray<double> sw;
			for (int j = 0; j < nurbsSF.m_vNum; ++j)
			{
				vctrl.push_back(nurbsSF.m_CtrlPts[i*nurbsSF.m_vNum + j]);
				sw.push_back(nurbsSF.m_CtrlPts[i*nurbsSF.m_vNum + j].w);
			}
			nurbsSFctrlPts.push_back(vctrl);
			sws.push_back(sw);
		}
		varray<varray<varray<point4d>>> allNurbsSF;
		MatrixTran(nurbsSFctrlPts, TranMat, allNurbsSF);
		m_wNum = allNurbsSF.size();
		for (int i = 0; i < m_wNum; ++i)
		{
			SFw.push_back(sws);
		}
		SweepSurface(allNurbsSF, SFw, pos);
	}

	/*平移扫描生成Nurbs体模型
	pathT：扫描路径
	nurbsSF：起始截面*/
	void NurbsVol::CreateTransSweepNurbsVol(const NurbsLine& pathT, const NurbsSurface & nurbsSF)
	{
		//u方向节点矢量，控制点数量
		m_uDegree = nurbsSF.m_uDegree;
		m_uNum = nurbsSF.m_uNum;
		m_uKnots = nurbsSF.m_uKnots;

		//v方向节点矢量，控制点数量
		m_vDegree = nurbsSF.m_vDegree;
		m_vNum = nurbsSF.m_vNum;
		m_vKnots = nurbsSF.m_vKnots;

		m_wDegree = pathT.m_Degree;
		m_wNum = pathT.m_CtrlPts.size();
		m_wKnots = pathT.m_Knots;

		m_CtrlPts.clear();
		for (int i = 0; i<pathT.m_CtrlPts.size(); i++)
		{
			for (int j = 0; j < nurbsSF.m_CtrlPts.size(); j++)
			{
				point4d ControlPoint;
				ControlPoint = nurbsSF.m_CtrlPts[j] + pathT.m_CtrlPts[i] - pathT.m_CtrlPts[0];
				ControlPoint.w = nurbsSF.m_CtrlPts[j].w*pathT.m_CtrlPts[i].w;
				m_CtrlPts.push_back(ControlPoint);
			}
		}
	}

	//放样
	//path:路径
	//surfaces:放样截面
	void NurbsVol::LoftingNurbsVol(const NurbsLine& path, const varray<NurbsSurface>& surfaces)
	{
		m_CtrlPts.clear();
		m_uKnots.clear();
		m_vKnots.clear();
		m_wKnots.clear();

		m_wDegree = path.m_Degree;
		m_wNum = path.m_CtrlPts.size();

		//最高次
		int maxUDegree, maxVDegree;
		MaxDegree(surfaces, maxUDegree, maxVDegree);
		//升阶
		varray<NurbsSurface> UnitSurfaces = surfaces;
		for (int i = 0; i < UnitSurfaces.size(); ++i) 
			UnitSurfaces[i].DegreeElevate(maxUDegree, maxVDegree);
		//统一节点矢量
		varray<double> UKnots, VKnots;
		KnotsUnify(UnitSurfaces, UKnots, VKnots);
		for (int i = 0; i < UnitSurfaces.size(); ++i)
		{
			varray<double> diffUKnots, diffVKnots;
			KnotsDiff(UKnots, UnitSurfaces[i].m_uKnots, diffUKnots);
			KnotsDiff(VKnots, UnitSurfaces[i].m_vKnots, diffVKnots);
			//节点插入
			UnitSurfaces[i].KnotsRefine(diffUKnots, diffVKnots);
		}
		m_uKnots = UnitSurfaces[0].m_uKnots;
		m_vKnots = UnitSurfaces[0].m_vKnots;
		m_uNum = UnitSurfaces[0].m_uNum;
		m_vNum = UnitSurfaces[0].m_vNum;
		m_uDegree = UnitSurfaces[0].m_uDegree;
		m_vDegree = UnitSurfaces[0].m_vDegree;


		//原注释部分start

		//varray<double> DIS, UK, U_W; double Dis = 0;
		//for (int i = 0; i<path.m_CtrlPts.size() - 1; i++)
		//{
		//	point3d pt0, pt1; double d;
		//	pt0 = path.m_CtrlPts[i];
		//	pt1 = path.m_CtrlPts[i + 1];
		//	d = sqrt(pow((pt1.x - pt0.x), 2) + pow((pt1.y - pt0.y), 2) + pow((pt1.z - pt0.z), 2));
		//	DIS.push_back(d);
		//	Dis += d;
		//}
		//UK.push_back(0);
		//for (int i = 0; i<DIS.size() - 1; i++)
		//{
		//	UK.push_back(UK[UK.size() - 1] + DIS[i] / Dis);
		//}
		//UK.push_back(1);
		////节点向量
		//for (int i = 0; i < m_wDegree + 1; i++)
		//{
		//	U_W.push_back(0);
		//}
		//for (int j = 1; j<path.m_CtrlPts.size() - m_wDegree; j++)
		//{
		//	double uw = 0;
		//	for (int i = j; i <= j + m_wDegree - 1; i++)
		//	{
		//		uw += UK[i];
		//	}
		//	U_W.push_back(uw / (j + m_wDegree - 1));
		//}
		//for (int i = 0; i<m_wDegree + 1; i++)
		//{
		//	U_W.push_back(1);
		//}
		//for (int i = 0; i<U_W.size(); i++)
		//	m_wKnots.push_back(U_W[i]);
		//for (int i = 0; i<surfaces.size(); i++)
		//	for (int j = 0; j < surfaces[i].m_CtrlPts.size(); j++)
		//		m_CtrlPts.push_back(surfaces[i].m_CtrlPts[j]);

		//原注释部分end

		int K = (surfaces.size() - 1) > (path.m_CtrlPts.size() - 1) ? (surfaces.size() - 1) : (path.m_CtrlPts.size() - 1);
		varray<double> pos;
		InsLocation(path, K, pos);//同时完成W方向节点矢量
		varray<varray<point4d>> TranMat;
		LocalCoordinates(path, pos, TranMat);

		varray<varray<point4d>> nurbsSFctrlPts;
		for (int i = 0; i < UnitSurfaces[0].m_uNum; ++i)
		{
			varray<point4d> vctrl;
			for (int j = 0; j < UnitSurfaces[0].m_vNum; ++j)
				vctrl.push_back(UnitSurfaces[0].m_CtrlPts[i*UnitSurfaces[0].m_vNum + j]);
			nurbsSFctrlPts.push_back(vctrl);
		}
		varray<varray<varray<point4d>>> allNurbsSF;
		MatrixTran(nurbsSFctrlPts, TranMat, allNurbsSF);
		m_wNum = allNurbsSF.size();
	}

	void NurbsVol::LoftingNurbsVol2(const NurbsLine & path, const varray<NurbsSurface>& surfaces)
	{
		//m_CtrlPts.clear();
		//m_uKnots.clear();
		//m_vKnots.clear();
		//m_wKnots.clear();

		//m_wDegree = path.m_Degree;
		//m_wNum = path.m_CtrlPts.size();

		////最高次
		//int maxUDegree, maxVDegree;
		//MaxDegree(surfaces, maxUDegree, maxVDegree);
		////升阶
		//varray<NurbsSurface> UnitSurfaces = surfaces;
		//for (int i = 0; i < UnitSurfaces.size(); ++i)
		//	UnitSurfaces[i].DegreeElevate(maxUDegree, maxVDegree);
		////统一节点矢量
		//varray<double> UKnots, VKnots;
		//KnotsUnify(UnitSurfaces, UKnots, VKnots);
		//for (int i = 0; i < UnitSurfaces.size(); ++i)
		//{
		//	varray<double> diffUKnots, diffVKnots;
		//	KnotsDiff(UKnots, UnitSurfaces[i].m_uKnots, diffUKnots);
		//	KnotsDiff(VKnots, UnitSurfaces[i].m_vKnots, diffVKnots);
		//	//节点插入
		//	UnitSurfaces[i].KnotsRefine(diffUKnots, diffVKnots);
		//}
		//m_uKnots = UnitSurfaces[0].m_uKnots;
		//m_vKnots = UnitSurfaces[0].m_vKnots;
		//m_uNum = UnitSurfaces[0].m_uNum;
		//m_vNum = UnitSurfaces[0].m_vNum;
		//m_uDegree = UnitSurfaces[0].m_uDegree;
		//m_vDegree = UnitSurfaces[0].m_vDegree;

		////varray<double> DIS, UK, U_W; double Dis = 0;
		////for (int i = 0; i<path.m_CtrlPts.size() - 1; i++)
		////{
		////	Vec3 pt0, pt1; double d;
		////	pt0 = path.m_CtrlPts[i];
		////	pt1 = path.m_CtrlPts[i + 1];
		////	d = sqrt(pow((pt1.x - pt0.x), 2) + pow((pt1.y - pt0.y), 2) + pow((pt1.z - pt0.z), 2));
		////	DIS.push_back(d);
		////	Dis += d;
		////}
		////UK.push_back(0);
		////for (int i = 0; i<DIS.size() - 1; i++)
		////{
		////	UK.push_back(UK[UK.size() - 1] + DIS[i] / Dis);
		////}
		////UK.push_back(1);
		//////节点向量
		////for (int i = 0; i < m_wDegree + 1; i++)
		////{
		////	U_W.push_back(0);
		////}
		////for (int j = 1; j<path.m_CtrlPts.size() - m_wDegree; j++)
		////{
		////	double uw = 0;
		////	for (int i = j; i <= j + m_wDegree - 1; i++)
		////	{
		////		uw += UK[i];
		////	}
		////	U_W.push_back(uw / (j + m_wDegree - 1));
		////}
		////for (int i = 0; i<m_wDegree + 1; i++)
		////{
		////	U_W.push_back(1);
		////}
		////for (int i = 0; i<U_W.size(); i++)
		////	m_wKnots.push_back(U_W[i]);
		////for (int i = 0; i<surfaces.size(); i++)
		////	for (int j = 0; j < surfaces[i].m_CtrlPts.size(); j++)
		////		m_CtrlPts.push_back(surfaces[i].m_CtrlPts[j]);

		//int K = (surfaces.size() - 1) > (path.m_CtrlPts.size() - 1) ? (surfaces.size() - 1) : (path.m_CtrlPts.size() - 1);
		//varray<double> pos;
		//InsLocation(path, K, pos);//同时完成W方向节点矢量
		//varray<varray<point4d>> TranMat;
		//LocalCoordinates(path, pos, TranMat);

		//varray<varray<point4d>> nurbsSFctrlPts;
		//for (int i = 0; i < UnitSurfaces[0].m_uNum; ++i)
		//{
		//	varray<point4d> vctrl;
		//	for (int j = 0; j < UnitSurfaces[0].m_vNum; ++j)
		//		vctrl.push_back(UnitSurfaces[0].m_CtrlPts[i*UnitSurfaces[0].m_vNum + j]);
		//	nurbsSFctrlPts.push_back(vctrl);
		//}
		//varray<varray<varray<point4d>>> allNurbsSF;
		//MatrixTran(nurbsSFctrlPts, TranMat, allNurbsSF);
		//m_wNum = allNurbsSF.size();



		m_wDegree = path.m_Degree;
		m_wNum = path.m_CtrlPts.size();

		//最高次
		int maxUDegree, maxVDegree;
		MaxDegree(surfaces, maxUDegree, maxVDegree);
		//升阶
		varray<NurbsSurface> UnitSurfaces = surfaces;
		for (int i = 0; i < UnitSurfaces.size(); ++i)
			UnitSurfaces[i].DegreeElevate(maxUDegree, maxVDegree);
		//统一节点矢量
		varray<double> UKnots, VKnots;
		KnotsUnify(UnitSurfaces, UKnots, VKnots);
		for (int i = 0; i < UnitSurfaces.size(); ++i)
		{
			varray<double> diffUKnots, diffVKnots;
			KnotsDiff(UKnots, UnitSurfaces[i].m_uKnots, diffUKnots);
			KnotsDiff(VKnots, UnitSurfaces[i].m_vKnots, diffVKnots);
			//节点插入
			UnitSurfaces[i].KnotsRefine(diffUKnots, diffVKnots);
		}

		varray<double> DIS, UK, U_W; double Dis = 0;
		for (int i = 0; i < path.m_CtrlPts.size() - 1; i++)
		{
			point3d pt0, pt1; double d;
			pt0 = path.m_CtrlPts[i];
			pt1 = path.m_CtrlPts[i + 1];
			d = sqrt(pow((pt1.x - pt0.x), 2) + pow((pt1.y - pt0.y), 2) + pow((pt1.z - pt0.z), 2));
			DIS.push_back(d);
			Dis += d;
		}
		UK.push_back(0);
		for (int i = 0; i < DIS.size() - 1; i++)
		{
			UK.push_back(UK[UK.size() - 1] + DIS[i] / Dis);
		}
		UK.push_back(1);
		//节点向量
		for (int i = 0; i < m_wDegree + 1; i++)
		{
			U_W.push_back(0);
		}
		for (int j = 1; j < path.m_CtrlPts.size() - m_wDegree; j++)
		{
			double uw = 0;
			for (int i = j; i <= j + m_wDegree - 1; i++)
			{
				uw += UK[i];
			}
			U_W.push_back(uw / (j + m_wDegree - 1));
		}
		for (int i = 0; i < m_wDegree + 1; i++)
		{
			U_W.push_back(1);
		}

		//UV方向节点向量和控制点坐标
		m_uKnots.clear();
		m_vKnots.clear();
		m_wKnots.clear();
		for (int i = 0; i < U_W.size(); i++)
		{
			m_wKnots.push_back(U_W[i]);
		}
		for (int i = 0; i < surfaces[0].m_uKnots.size(); i++)
		{
			m_uKnots.push_back(surfaces[0].m_uKnots[i]);
		}
		for (int i = 0; i < surfaces[0].m_vKnots.size(); i++)
		{
			m_vKnots.push_back(surfaces[0].m_vKnots[i]);
		}
		m_uNum = surfaces[0].m_uNum;
		m_vNum = surfaces[0].m_vNum;
		m_uDegree = surfaces[0].m_uDegree;
		m_vDegree = surfaces[0].m_vDegree;

		m_CtrlPts.clear();
		for (int i = 0; i < surfaces.size(); i++)
		{
			for (int j = 0; j < surfaces[i].m_CtrlPts.size(); j++)
			{
				m_CtrlPts.push_back(surfaces[i].m_CtrlPts[j]);
			}
		}
	}

	void NurbsVol::LoftingNurbsVol(const NurbsLine & path, const NurbsSurface & surfaces0, const NurbsSurface & surfaces1)
	{
		m_CtrlPts.clear();
		m_uKnots.clear();
		m_vKnots.clear();
		m_wKnots.clear();

		m_wDegree = path.m_Degree;
		m_wNum = path.m_CtrlPts.size();

		varray<NurbsSurface> surfaces;
		surfaces.push_back(surfaces0);
		surfaces.push_back(surfaces1);
		//最高次
		MaxDegree(surfaces, m_uDegree, m_vDegree);
		//升阶
		for (int i = 0; i < surfaces.size(); ++i)
			surfaces[i].DegreeElevate(m_uDegree, m_vDegree);
		//统一节点矢量
		KnotsUnify(surfaces, m_uKnots, m_vKnots);
		for (int i = 0; i < surfaces.size(); ++i)
		{
			varray<double> diffUKnots, diffVKnots;
			KnotsDiff(m_uKnots, surfaces[i].m_uKnots, diffUKnots);
			KnotsDiff(m_vKnots, surfaces[i].m_vKnots, diffVKnots);
			//节点插入
			surfaces[i].KnotsRefine(diffUKnots, diffVKnots);
		}
		m_uNum = surfaces[0].m_uNum;
		m_vNum = surfaces[0].m_vNum;

		int K = path.m_CtrlPts.size() - 1;
		varray<double> pos;
		InsLocation(path, K, pos);//同时完成W方向节点矢量
		varray<varray<point4d>> TranMat01, TranMat10;
		LocalCoordinates(path, pos, TranMat01);

		//反向
		/*NurbsLine path10 = path;
		path10.m_CtrlPts.reverse();
		for (int i = pos01.size() - 1; i >= 0; --i)
			pos10.push_back(1 - pos01[i]);
		LocalCoordinates(path10, pos10, TranMat10);*/
		TranMat10 = TranMat01;
		TranMat10.reverse();
		for (int i = 0; i < TranMat10.size(); ++i)
		{
			TranMat10[i][2] *= -1;
		}
		//原始权重
		varray<varray<double>> SFw0, SFw1; 
		//控制点二维数组
		varray<varray<point4d>> nurbsSFctrlPts0, nurbsSFctrlPts1;
		
		for (int i = 0; i < surfaces[0].m_uNum; ++i)
		{
			varray<point4d> vctrl;
			varray<double> sw;
			for (int j = 0; j < surfaces[0].m_vNum; ++j)
			{
				vctrl.push_back(surfaces[0].m_CtrlPts[i*surfaces[0].m_vNum + j]);
				sw.push_back(surfaces[0].m_CtrlPts[i*surfaces[0].m_vNum + j].w);
			}
			nurbsSFctrlPts0.push_back(vctrl);
			SFw0.push_back(sw);
		}
		for (int i = 0; i < surfaces[1].m_uNum; ++i)
		{
			varray<point4d> vctrl;
			varray<double> sw;
			for (int j = 0; j < surfaces[1].m_vNum; ++j)
			{
				vctrl.push_back(surfaces[1].m_CtrlPts[i*surfaces[1].m_vNum + j]);
				sw.push_back(surfaces[0].m_CtrlPts[i*surfaces[0].m_vNum + j].w);
			}
			nurbsSFctrlPts1.push_back(vctrl);
			SFw1.push_back(sw);
		}
		//实例截面计算
		varray<varray<varray<point4d>>> allNurbsSF0, allNurbsSF1, allNurbsSF;
		varray<varray<varray<double>>>  SFw;
		MatrixTran(nurbsSFctrlPts0, TranMat01, allNurbsSF0);
		MatrixTran(nurbsSFctrlPts1, TranMat10, allNurbsSF1);
		
		//同一位置插值线性插值
		allNurbsSF.resize(pos.size());
		SFw.resize(pos.size());
		for (int i = 0; i < pos.size(); ++i)
		{
			allNurbsSF[i].resize(allNurbsSF0[i].size());
			SFw[i].resize(SFw0.size());
			for (int j = 0; j < allNurbsSF0[i].size(); ++j)
			{
				allNurbsSF[i][j].resize(allNurbsSF0[i][j].size());
				SFw[i][j].resize(SFw0[j].size());
				int sf0ij = allNurbsSF0[i][j].size();
				for (int k = 0; k < sf0ij; ++k)
				{
					point4d dp = allNurbsSF1[pos.size() - 1 - i][j][sf0ij - 1 - k] - allNurbsSF0[i][j][k];
					double dw = abs(allNurbsSF1[pos.size() - 1 - i][j][sf0ij - 1 - k].w - allNurbsSF0[i][j][k].w);
					point4d pts= allNurbsSF0[i][j][k]
						+ pos[i] * dp.Magnitude()*dp.Normalize();
					allNurbsSF[i][j][k] = pts;
					allNurbsSF[i][j][k].w = allNurbsSF0[i][j][k].w + pos[i] * dw;
					SFw[i][j][k] = SFw0[j][k] + pos[i] * abs(SFw1[j][k] - SFw0[j][k]);
				}
			}
		}
		m_wNum = allNurbsSF.size();
		SweepSurface(allNurbsSF, SFw, pos);
	}

	

	NurbsSurface NurbsVol::GetSingleSurfaces(int dir) const
	{
		NurbsSurface sf;
		assert(dir >= 1 && dir <= 6);
		assert(this->m_CtrlPts.size());
		//if (dir < 1 || dir>6)
		//	return {};
		if (dir == 1) {
			//UV面,W=0
			sf.m_uDegree = this->m_uDegree;
			sf.m_vDegree = this->m_vDegree;
			sf.m_uNum = this->m_uNum;
			sf.m_vNum = this->m_vNum;
			sf.m_uKnots = this->m_uKnots;
			sf.m_vKnots = this->m_vKnots;
			for (int i = 0; i < this->m_vNum; i++) {
				for (int j = 0; j < this->m_uNum; j++) {
					sf.m_CtrlPts.push_back(this->m_CtrlPts[i*this->m_uNum + j]);
				}
			}
		}

		else if (dir == 2) {
			//UW面，V=0
			sf.m_uDegree = this->m_uDegree;
			sf.m_vDegree = this->m_wDegree;
			sf.m_uNum = this->m_uNum;
			sf.m_vNum = this->m_wNum;
			sf.m_uKnots = this->m_uKnots;
			sf.m_vKnots = this->m_wKnots;
			for (int i = 0; i < this->m_wNum; i++) {
				for (int j = 0; j < this->m_uNum; j++) {
					sf.m_CtrlPts.push_back(this->m_CtrlPts[i*(this->m_uNum*this->m_vNum) + j]);
				}
			}
		}

		else if (dir == 5) {
			//VW面，U=0
			sf.m_uDegree = this->m_vDegree;
			sf.m_vDegree = this->m_wDegree;
			sf.m_uNum = this->m_vNum;
			sf.m_vNum = this->m_wNum;
			sf.m_uKnots = this->m_vKnots;
			sf.m_vKnots = this->m_wKnots;
			for (int i = 0; i < this->m_wNum; i++) {
				for (int j = 0; j < this->m_vNum; j++) {
					sf.m_CtrlPts.push_back(this->m_CtrlPts[i*(this->m_uNum*this->m_vNum) + j * m_uNum]);
				}
			}
		}

		else if (dir == 6) {
			//UV面,W=1
			sf.m_uDegree = this->m_uDegree;
			sf.m_vDegree = this->m_vDegree;
			sf.m_uNum = this->m_uNum;
			sf.m_vNum = this->m_vNum;
			sf.m_uKnots = this->m_uKnots;
			sf.m_vKnots = this->m_vKnots;
			int pos = this->m_uNum*this->m_vNum*(this->m_wNum - 1);		//第一个点的位置
			for (int i = 0; i < this->m_vNum; i++) {
				for (int j = 0; j < this->m_uNum; j++) {
					sf.m_CtrlPts.push_back(this->m_CtrlPts[pos + i * this->m_uNum + j]);
				}
			}
		}

		return sf;
	}

	void NurbsVol::VolCoonsInterpolate(const varray<NurbsLine>& EdgeLines)
	{
		//RWGeometric rw;
		varray<NurbsLine> sfLines;
		varray<NurbsSurface> sfs;
		NurbsSurface tmpsf;
		//0号面
		sfLines.clear();
		sfLines.push_back(EdgeLines[0]);
		sfLines.push_back(EdgeLines[1]);
		sfLines.push_back(EdgeLines[2]);
		sfLines.push_back(EdgeLines[3]);
		tmpsf.CoonsInterpolate(sfLines);
		sfs.push_back(tmpsf);
		//1号面
		sfLines.clear();
		sfLines.push_back(EdgeLines[0]);
		sfLines.push_back(EdgeLines[4]);
		sfLines.push_back(EdgeLines[8]);
		sfLines.push_back(EdgeLines[7]);
		tmpsf.CoonsInterpolate(sfLines);
		sfs.push_back(tmpsf);
		//2号面
		sfLines.clear();
		sfLines.push_back(EdgeLines[1]);
		sfLines.push_back(EdgeLines[4]);
		sfLines.push_back(EdgeLines[9]);
		sfLines.push_back(EdgeLines[5]);
		tmpsf.CoonsInterpolate(sfLines);
		sfs.push_back(tmpsf);
		//3号面
		sfLines.clear();
		sfLines.push_back(EdgeLines[2]);
		sfLines.push_back(EdgeLines[5]);
		sfLines.push_back(EdgeLines[10]);
		sfLines.push_back(EdgeLines[6]);
		tmpsf.CoonsInterpolate(sfLines);
		sfs.push_back(tmpsf);
		//4号面
		sfLines.clear();
		sfLines.push_back(EdgeLines[3]);
		sfLines.push_back(EdgeLines[7]);
		sfLines.push_back(EdgeLines[11]);
		sfLines.push_back(EdgeLines[6]);
		tmpsf.CoonsInterpolate(sfLines);
		sfs.push_back(tmpsf);
		//5号面
		sfLines.clear();
		sfLines.push_back(EdgeLines[8]);
		sfLines.push_back(EdgeLines[9]);
		sfLines.push_back(EdgeLines[10]);
		sfLines.push_back(EdgeLines[11]);
		tmpsf.CoonsInterpolate(sfLines);
		sfs.push_back(tmpsf);

		//UV换序
		for (auto& s : sfs) {
			s.OrderCtrlPts();
		}
		//rw.WriteNurbsSurface("D:\\冯文斌大论文\\素材\\示意图txt\\5.2COONS插值体\\5.2sf.txt", sfs);
		int num_u, num_v, num_w;
		num_u = sfs[0].m_uNum;
		num_v = sfs[0].m_vNum;
		num_w = sfs[1].m_vNum;
		this->m_uNum = num_u;
		this->m_vNum = num_v;
		this->m_wNum = num_w;
		this->m_uDegree = sfs[0].m_uDegree;
		this->m_vDegree = sfs[0].m_vDegree;
		this->m_wDegree = sfs[1].m_vDegree;
		this->m_uKnots = sfs[0].m_uKnots;
		this->m_vKnots = sfs[0].m_vKnots;
		this->m_wKnots = sfs[1].m_vKnots;
		for (auto p : sfs[0].m_CtrlPts) {
			this->m_CtrlPts.push_back(p);
		}
		for (int i = 1; i < num_w - 1; i++) {
			varray<NurbsLine> edgeL;
			NurbsLine tmpl;
			NurbsSurface tmps;
			//e1
			tmpl.m_Degree = sfs[1].m_uDegree;
			tmpl.m_Knots = sfs[1].m_uKnots;
			tmpl.m_CtrlPts.clear();
			for (int j = 0; j < num_u; j++) {
				tmpl.m_CtrlPts.push_back(sfs[1].m_CtrlPts[i*num_u + j]);
			}
			edgeL.push_back(tmpl);
			//e2
			tmpl.m_Degree = sfs[2].m_uDegree;
			tmpl.m_Knots = sfs[2].m_uKnots;
			tmpl.m_CtrlPts.clear();
			for (int j = 0; j < num_v; j++) {
				tmpl.m_CtrlPts.push_back(sfs[2].m_CtrlPts[i*num_v + j]);
			}
			edgeL.push_back(tmpl);
			//e3
			tmpl.m_Degree = sfs[3].m_uDegree;
			tmpl.m_Knots = sfs[3].m_uKnots;
			tmpl.m_CtrlPts.clear();
			for (int j = 0; j < num_u; j++) {
				tmpl.m_CtrlPts.push_back(sfs[3].m_CtrlPts[i*num_u + j]);
			}
			edgeL.push_back(tmpl);
			//e4
			tmpl.m_Degree = sfs[4].m_uDegree;
			tmpl.m_Knots = sfs[4].m_uKnots;
			tmpl.m_CtrlPts.clear();
			for (int j = 0; j < num_v; j++) {
				tmpl.m_CtrlPts.push_back(sfs[4].m_CtrlPts[i*num_v + j]);
			}
			edgeL.push_back(tmpl);
			tmps.CoonsInterpolate(edgeL);
			tmps.OrderCtrlPts();
			for (auto&p : tmps.m_CtrlPts) {
				this->m_CtrlPts.push_back(p);
			}
		}
		for (auto& p : sfs[5].m_CtrlPts) {
			this->m_CtrlPts.push_back(p);
		}
	}



	//计算等参面
	//uvw:0=u,1=v,2=w
	//t:参数
	//num:以u-v-w顺序
	//L:等参面四边形面片集
	void NurbsVol::CalIsoSurface(const int uvw, const double t, const int num1, const int num2,
		varray<varray<point3d>>& quads, varray<varray<point3d>>& lines)const
	{
		quads.clear();
		lines.clear();
		varray<varray<point3d>> L;
		double u[3]{ 0,0,0 };
		u[uvw] = t;
		int a = (uvw + 1) % 3;
		int b = (uvw + 2) % 3;

		double da = 1.0 / num1;
		double db = 1.0 / num2;
		for (int i = 0; i <= num1; i++)
		{
			u[a] = da*i;
			varray<point3d> line;
			for (int j = 0; j <= num2; j++)
			{
				u[b] = db*j;
				point3d t = GetVolPoint(u[0], u[1], u[2]);
				line.push_back(t);
			}
			L.push_back(line);
		}

		for (int i = 0; i < L.size() - 1; ++i)
		{
			for (int j = 0; j < L[i].size() - 1; ++j)
			{
				varray<point3d> quad;
				quad.push_back(L[i][j]);
				quad.push_back(L[i + 1][j]);
				quad.push_back(L[i + 1][j + 1]);
				quad.push_back(L[i][j + 1]);
				quads.push_back(quad);
			}
		}

		lines.push_back(L[0]);
		lines.push_back(L[L.size() - 1]);
		varray<point3d> l0, l1;
		for (int i = 0; i < L.size(); ++i)
		{
			l0.push_back(L[i][0]);
			l1.push_back(L[i][L[i].size() - 1]);
		}
		lines.push_back(l0);
		lines.push_back(l1);
	}

	/*扫描时的截面实例位置
	pathT：扫描路径
	K：截面实例数量
	pos：截面实例位置
	NewKnots：新节点矢量*/
	void NurbsVol::InsLocation(const NurbsLine & pathT, int K, varray<double>& pos)
	{
		pos.clear();
		int q = pathT.m_Degree;
		int ktv = pathT.m_Knots.size();
		int nsect = K + 1;
		double vsum;

		m_wKnots = pathT.m_Knots;

		if (ktv <= nsect + q)  //细化节点矢量
		{
			int m = nsect + q - ktv + 1;
			for (int i = 0; i < m; ++i)
			{
				//最长节点区间
				varray<double>::iterator idx = m_wKnots.begin();
				double maxlen = 0;
				for (auto it = m_wKnots.begin() + pathT.m_Degree;
				it != m_wKnots.begin() + (m_wKnots.size() - pathT.m_Degree - 2); ++it)
				{
					double dk = *(it + 1) - *it;
					if (dk >= maxlen)
					{
						maxlen = dk;
						idx = it;
					}
				}
				//插入中点
				double mid = (*idx + *(idx + 1)) / 2;
				m_wKnots.insert(idx + 1, mid);
			}
		}
		else if (ktv>nsect + q + 1) //增加实例
			nsect = ktv - q - 1;

		//实例位置
		pos.push_back(m_wKnots[0]);
		for (int k = 1; k<nsect - 1; k++)
		{
			vsum = 0;
			for (int l = k + 1; l<k + q + 1; l++)
			{
				vsum += m_wKnots[l];
			}
			pos.push_back(vsum / q);
		}
		pos.push_back(m_wKnots[m_wKnots.size() - 1]);
	}

	/*计算局部坐标系
	pathT：扫描路径
	pos：截面实例位置
	TranMat：pos处的局部坐标系*/
	void NurbsVol::LocalCoordinates(const NurbsLine & pathT, const varray<double> & pos,
		varray<varray<point4d>> & TranMat)
	{
		TranMat.clear();
		int q = pathT.m_Degree;
		//计算弗朗内特标
		//导矢
		varray<varray<point4d>> DersBasis;
		DersBasis.clear();
		for (int i = 0; i<pos.size(); i++)
		{
			varray<point4d> NuDerBasi;
			pathT.PtsDerivs(pos[i], 2, NuDerBasi);
			if (NuDerBasi.size() < 3 || NuDerBasi[2]== point4d())//不存在2阶导
			{
				point4d vDer = NuDerBasi[1].Normalize().Abs();
				int midx = 0;
				for (int j = 1; j < 3; ++j)
				{
					if (vDer[j] > vDer[midx])
					{
						vDer[midx] = 0;
						midx = j;
					}
					else
						vDer[j] = 0;
				}
				if (vDer.Angle(NuDerBasi[1]) == 0 || vDer.Angle(NuDerBasi[1]) == 180)
				{
					vDer[midx] = 0;
					vDer[(midx + 1) % 3] = 1;
				}
				point4d vt = NuDerBasi[1].Cross(vDer).Normalize();
				if (NuDerBasi.size() < 3)
					NuDerBasi.push_back(vt);
				else
					NuDerBasi[2] = vt;
			}
			DersBasis.push_back(NuDerBasi);
		}
		for (int i = 0; i<pos.size(); i++)
		{
			point4d BV = (DersBasis[i][1].Cross(DersBasis[i][2])).Normalize();
			point4d NV = (BV.Cross(DersBasis[i][1])).Normalize();
			DersBasis[i][1] = DersBasis[i][1].Normalize();
			varray<point4d> Stor;
			Stor.push_back(BV);
			Stor.push_back(NV);
			Stor.push_back(DersBasis[i][1]);
			Stor.push_back(DersBasis[i][0]);
			TranMat.push_back(Stor);
		}
	}

	/*沿扫描路径对截面进行变换
	nurbsSF：截面控制点
	TranMat：变换矩阵
	allNurbsSF：得到的所有截面实例控制点*/
	void NurbsVol::MatrixTran(const varray<varray<point4d>> & nurbsSF, const varray<varray<point4d>>& TranMat, 
		 varray<varray<varray<point4d>>>& allNurbsSF)
	{
		using Eigen::Matrix4d;
		using Eigen::Vector4d;

		allNurbsSF.clear();
		varray<point3d> OriCoordinates;
		for (int i = 0; i < TranMat[0].size();++i)
			OriCoordinates.push_back(TranMat[0][i]);

		for (int i = 0; i<TranMat.size(); i++)  //变换矩阵
		{
			Matrix4d mat;
			varray<point3d> newCoordinates;
			newCoordinates.push_back(TranMat[i][0]);
			newCoordinates.push_back(TranMat[i][1]);
			newCoordinates.push_back(TranMat[i][2]);
			newCoordinates.push_back(TranMat[i][3]);

			CalCoordinatesTransMat(OriCoordinates, newCoordinates, mat);

			varray<varray<point4d>> SectPoints;
			for (int j = 0; j<nurbsSF.size(); j++)
			{
				varray<point4d> SectPoint;
				for (int k = 0; k<nurbsSF[j].size(); k++)
				{
					Vector4d bas = P4dToV4d(nurbsSF[j][k]);
					bas[3] = 1;
					Vector4d torm = mat*bas;
					point4d pt = V4dToP4d(torm) * TranMat[i][3].w;
					pt.w = TranMat[i][3].w;
					SectPoint.push_back(pt);
				}
				SectPoints.push_back(SectPoint);
			}
			allNurbsSF.push_back(SectPoints);
		}
	}

	//扫描生成Nurbs体
	void NurbsVol::SweepSurface(const varray<varray<varray<point4d>>>& allNurbsSF, 
		const varray<varray<varray<double>>>& SFw, const varray<double> &pos)
	{
		using Eigen::MatrixXd;

		//解线性方程组，插值控制点坐标    
		int p = m_wDegree;
		MatrixXd Nbase(allNurbsSF.size(), allNurbsSF.size());
		MatrixXd NpointX(allNurbsSF.size(), m_uNum*m_vNum);
		MatrixXd NpointY(allNurbsSF.size(), m_uNum*m_vNum);
		MatrixXd NpointZ(allNurbsSF.size(), m_uNum*m_vNum);
		MatrixXd NpointW(allNurbsSF.size(), m_uNum*m_vNum);
		MatrixXd ConTropointX(allNurbsSF.size(), m_uNum*m_vNum);
		MatrixXd ConTropointY(allNurbsSF.size(), m_uNum*m_vNum);
		MatrixXd ConTropointZ(allNurbsSF.size(), m_uNum*m_vNum);
		MatrixXd ConTropointW(allNurbsSF.size(), m_uNum*m_vNum);
		for (int i = 0; i<pos.size(); i++)  //截面个数
		{
			//计算基函数
			varray<varray<double>> ndu;
			int span = FindSpan(pos[i], p, allNurbsSF.size(), m_wKnots);
			AllBasisFuns(pos[i], span, p, m_wKnots, ndu);
			for (int l = 0; l<span - p; l++)
			{
				Nbase(i, l) = 0;
			}
			for (int m = 0; m <= p; m++)
			{
				double npm = ndu[m][p];
				Nbase(i, span - p + m) = npm;
			}
			for (int n = span + 1; n<allNurbsSF.size(); n++)
			{
				Nbase(i, n) = 0;
			}
		}

		for (int i = 0; i<allNurbsSF.size(); i++) //第i个截面    
		{
			for (int j = 0; j<allNurbsSF[i].size(); j++)         //UV方向
			{
				for (int k = 0; k<allNurbsSF[i][j].size(); k++)
				{
					NpointX(i, j*allNurbsSF[i][j].size() + k) = allNurbsSF[i][j][k].x;          //体上的点
					NpointY(i, j*allNurbsSF[i][j].size() + k) = allNurbsSF[i][j][k].y;          //体上的点
					NpointZ(i, j*allNurbsSF[i][j].size() + k) = allNurbsSF[i][j][k].z;          //体上的点
					NpointW(i, j*allNurbsSF[i][j].size() + k) = allNurbsSF[i][j][k].w;          //体上的点
				}
			}
		}
		MatrixXd nbinv = Nbase.inverse();
		ConTropointX = Nbase.inverse()*NpointX;
		ConTropointY = Nbase.inverse()*NpointY;
		ConTropointZ = Nbase.inverse()*NpointZ;
		ConTropointW = Nbase.inverse()*NpointW;

		for (int i = 0; i<allNurbsSF.size(); i++) //第i个截面,w方向    
		{
			for (int j = 0; j<allNurbsSF[i].size(); j++) //u方向
			{
				for (int k = 0; k<allNurbsSF[i][j].size(); k++) //v方向
				{
					point4d control;
					control.w = ConTropointW(i, j*allNurbsSF[i][j].size() + k);
					control.x = ConTropointX(i, j*allNurbsSF[i][j].size() + k) / control.w;
					control.y = ConTropointY(i, j*allNurbsSF[i][j].size() + k) / control.w;
					control.z = ConTropointZ(i, j*allNurbsSF[i][j].size() + k) / control.w;
					control.w *= SFw[i][j][k];
					m_CtrlPts.push_back(control);
				}
			}
		}
	}

	//取最高次
	void NurbsVol::MaxDegree(const varray<NurbsSurface>& surfaces, int& uDegree, int& vDegree)
	{
		uDegree = 0;
		vDegree = 0;
		for (int i = 0; i < surfaces.size(); ++i)
		{
			uDegree = surfaces[i].m_uDegree > uDegree ? surfaces[i].m_uDegree : uDegree;
			vDegree = surfaces[i].m_vDegree > vDegree ? surfaces[i].m_vDegree : vDegree;
		}
	}

	//节点矢量并集
	void NurbsVol::KnotsUnify(const varray<NurbsSurface>& surfaces, varray<double>& NewUKnots, varray<double>& NewVKnots)
	{
		NewUKnots.clear();
		NewVKnots.clear();
		varray<double> temUKnots, temVKnots;
		for (int i = 0; i < surfaces.size(); ++i)
		{
			KnotUnify(surfaces[i].m_uKnots, temUKnots, NewUKnots);
			temUKnots = NewUKnots;
			KnotUnify(surfaces[i].m_vKnots, temVKnots, NewVKnots);
			temVKnots = NewVKnots;
		}
	}
