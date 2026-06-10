

#if !defined(_PRECOMPILED)
#endif
#include "spline.h"
#include "XFunction.h"
#include "assert.h"
#include "qmath.h"

namespace base {


	/*计算节点下标
	x：节点值
	degree：次数
	CtrlPtsNum：控制点数量
	knots：节点矢量*/
	int SplineBase::FindSpan(const double x, const int degree, const int CtrlPtsNum, const varray<double>& knots)const{
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
	void SplineBase::BasisFuns(const double u, const int k, const int degree, const varray<double>& knots,
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
			for (int r = 0; r < j; r++)
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
	void Spline::CreatLineWithTwoPoints(const Vec3 & p1, const Vec3 & p2, int degree)
	{
		this->m_CtrlPts.clear();
		this->m_Knots.clear();

		this->m_Degree = 1;
		this->m_Knots.push_back(0);
		this->m_Knots.push_back(0);
		this->m_Knots.push_back(1);
		this->m_Knots.push_back(1);
		this->m_CtrlPts.push_back(Vec4(p1));
		this->m_CtrlPts.push_back(Vec4(p2));
		if (degree > 1) {
			this->DegreeElevate(degree);
		}
	}


	//曲线反转
	void Spline::CruveReverse()
	{
		int knotsLth = m_Knots.size();
		int cptLth = m_CtrlPts.size();
		varray<double> tempKnots;
		varray<Vec4> tempPts;
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
		//更新
		m_CtrlPts = tempPts;
		m_Knots = tempKnots;
		return;
	}

	/*u处所有基函数
	u：参数值
	k：节点下标
	degree：次数
	knots：节点矢量
	ndu：返回的所有基函数*/
	void SplineBase::AllBasisFuns(const double u, const int k, const int degree,
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
			for (int r = 0; r < j; r++)
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
	void SplineBase::DerBasisFuns(const double u, const int k, const int degree, const int n, const varray<double>& knots,
		varray<varray<double>>& basisDu)const
	{
		int p = degree;
		int rn = n > p ? p : n;
		varray<double> left(p + 1, 0), right(p + 1, 0);
		varray<varray<double>>a(2), ndu(p + 1);
		basisDu.resize(n + 1);
		for (int i = 0; i < n + 1; ++i)
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
			for (r = 0; r < j; r++)
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

	/*计算u节点对应的曲线坐标点
	u：节点参数*/
	Vec3 Spline::GetLinePoint(const double u)const
	{
		int m_id;
		varray<double> Nu;
		Nu.resize(m_Degree + 1);
		Vec3 res(0, 0, 0);
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
	void Spline::CalLinePoint(const int Unum, varray<Vec3>& linePts)const
	{
		linePts.clear();
		double du = 1.0 / Unum;
		for (int i = 0; i <= Unum; i++)         //采用等分节点矢量插值B样条曲线
		{
			double u = du * i;
			Vec3 t = GetLinePoint(u);
			linePts.push_back(t);
		}
	}

	/*曲线矢值函数A(u)所有n阶导，矢值函数A(u)即NURBS曲线分子式
	u：参数
	n：导矢阶数
	Der：A(u)所有n阶导，当曲线为B样条时(权重为1)，即求得B样条导矢*/
	void Spline::PtsDerivsAu(const double u, const int n, varray<Vec4>& Der)const
	{
		Der.clear();

		int span = 0, p = m_Degree, num = m_CtrlPts.size(), tmp;
		int du = n < p ? n : p;
		//计算基函数
		varray<varray<double>> ndu;
		span = FindSpan(u, p, num, m_Knots);
		AllBasisFuns(u, span, p, m_Knots, ndu);

		varray<varray<Vec4>> PKK;
		varray<Vec4> PK;
		for (int i = 0; i < num; i++)
		{
			Vec4 pt;
			pt = m_CtrlPts[i] * m_CtrlPts[i].w;
			pt.w = m_CtrlPts[i].w;
			PK.push_back(pt);
		}
		PKK.push_back(PK);

		for (int k = 1; k <= du; k++)
		{
			PK.clear();
			tmp = p - k + 1;
			for (int i = 0; i < num - k; i++)
			{
				Vec4 pt;
				pt = tmp * (PKK[k - 1][i + 1] - PKK[k - 1][i]) / (m_Knots[i + p + 1] - m_Knots[i + k]);
				pt.w = tmp * (PKK[k - 1][i + 1].w - PKK[k - 1][i].w) / (m_Knots[i + p + 1] - m_Knots[i + k]);
				PK.push_back(pt);
			}
			PKK.push_back(PK);
		}

		//计算导数
		for (int k = 0; k <= du; k++)
		{
			Vec4 pt;
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
	void Spline::PtsDerivs(const double u, const int n, varray<Vec4>& Der) const
	{
		Der.clear();
		int du = n < m_Degree ? n : m_Degree;
		varray<Vec4> DerAu;
		PtsDerivsAu(u, du, DerAu);
		for (int k = 0; k <= du; k++)
		{
			for (int i = 1; i <= k; i++)
				DerAu[k] = DerAu[k] - Bin(k, i) * DerAu[i].w*Der[k - i];

			Vec4 pt;
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
	void Spline::CurveDerivs(const int n, const double step, varray<varray<Vec3>>& Der)
	{
		Der.clear();
		for (double u = 0; u <= 1; u += step)
		{
			varray<Vec4> CK;
			PtsDerivs(u, n, CK);
			varray<Vec3> CK3;
			for (int i = 0; i < CK.size(); ++i)
				CK3.push_back(CK[i]);
			Der.push_back(CK3);
		}
	}

	/*节点插入
	u：需要插入的节点
	r：插入次数*/
	void Spline::KnotInsert(const double u, const int r)
	{
		if (r <= 0)return;

		varray<Vec4> newbpt;
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
		while (u <= m_Knots[i] && i > a)
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
					double nw = alfa * newbpt[ind - 1].w + (1.0f - alfa)*newbpt[ind].w;
					beta = alfa * newbpt[ind - 1].w / nw;
					newbpt[ind - 1] = beta * newbpt[ind - 1] + (1.0f - beta)*newbpt[ind];
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
	int Spline::KnotRemove(const double u, const int r)
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
			varray<Vec4> temp;
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
				temp[jj] = (m_CtrlPts[j] - alfj * temp[jj + 1]) / (1.0 - alfj);
				temp[jj].w = (m_CtrlPts[j].w - alfj * temp[jj + 1].w) / (1.0 - alfj);

				i++;
				ii++;
				j--;
				jj--;
			}

			if (j - i < t)
			{
				Vec4 dp = temp[ii - 1] - temp[jj + 1];
				dp.w = temp[ii - 1].w - temp[jj + 1].w;
				double ds = sqrt(dp.SquareMagnitude() + dp.w*dp.w);
				if (ds <= TOL)
					remflag = 1;
			}
			else
			{
				double alfi = (u - m_Knots[i]) / (m_Knots[i + ord + t] - m_Knots[i]);
				Vec4 dp = m_CtrlPts[i] - (alfi*temp[ii + t + 1] + (1.0 - alfi)*temp[ii - 1]);
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
	void Spline::KnotsRefine(const varray<double>& u)
	{
		if (u.size() <= 0)return;

		varray<Vec4> newbpt;
		varray<double> newKonts;
		int n = m_CtrlPts.size() - 1;//原始控制点数-1
		int r = u.size() - 1;//插入的节点数-1
		newKonts.resize(m_Knots.size() + u.size());
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
			while (u[j] <= m_Knots[i] && i > a)
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
					double nw = alfa * newbpt[ind - 1].w + (1.0f - alfa)*newbpt[ind].w;
					beta = alfa * newbpt[ind - 1].w / nw;
					newbpt[ind - 1] = beta * newbpt[ind - 1] + (1.0f - beta)*newbpt[ind];
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
	void Spline::DegreeElevate(const int degree)
	{
		int t = degree - m_Degree;
		if (t < 1)
			return;

		varray<double> Uh;//升阶后的节点矢量
		varray<Vec4> Qw;//升阶后的控制点

		{    
			int sz = 1;
			for (int i = 0; i < m_Knots.size() - 1; ++i)
			{
				if (m_Knots[i] != m_Knots[i + 1])
					++sz;
			}
			Uh.resize(m_Knots.size() + 2 * t + (sz - 2)*t);
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

		varray<Vec4> bpts(m_Degree + 1, Vec4()), ebpts(ph + 1, Vec4()), Nextbpts(m_Degree - 1, Vec4());

		//计算升阶系数
		bezalfs[0][0] = bezalfs[ph][m_Degree] = 1.0;
		for (int i = 1; i <= ph2; ++i)
		{
			double inv = 1.0 / Bin(ph, i);
			int mpi = m_Degree < i ? m_Degree : i;
			for (int j = 0 > i - t ? 0 : i - t; j <= mpi; ++j)
				bezalfs[i][j] = inv * Bin(m_Degree, j)*Bin(t, i - j);
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
				ebpts[i] = Vec4(0, 0, 0, 0);
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
							Qw[i] = alf * Qw[i] + (1 - alf)*Qw[i - 1];
							Qw[i].w = alf * Qw[i].w + (1 - alf)*Qw[i - 1].w;
						}
						if (j >= lbz)
						{
							if (j - tr <= kind - ph + oldr)
							{
								double gam = (ub - Uh[j - tr]) / den;
								ebpts[kj] = gam * ebpts[kj] + (1 - bet)*ebpts[kj + 1];
								ebpts[kj].w = gam * ebpts[kj].w + (1 - bet)*ebpts[kj + 1].w;
							}
							else
							{
								ebpts[kj] = bet * ebpts[kj] + (1 - bet)*ebpts[kj + 1];
								ebpts[kj].w = bet * ebpts[kj].w + (1 - bet)*ebpts[kj + 1].w;
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

	void Spline::Segmentation(const double u, varray<Spline>& lines)
	{
		lines.clear();
		if (u <= 0 || u >= 1)
		{
			lines.push_back(*this);
			return;
		}
			

		lines.resize(2);

		Spline l0(*this);
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
	void Spline::Decompose(varray<varray<Vec4>>& Qw)
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
						Qw[nb][k] = alpha * Qw[nb][k] + (1 - alpha)*Qw[nb][k - 1];
						Qw[nb][k].w = alpha * Qw[nb][k].w + (1 - alpha)*Qw[nb][k - 1].w;
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


Spline::Spline()
{
	m_bSplineGenerateMode = 0; //XXX add, 
	m_yCrossSection       = -1.0f;
	m_Degree = 1;
	ClearStatus();
}


Spline::Spline(const Spline& spl)
{
	m_CtrlPts = spl.m_CtrlPts;
	m_Knots = spl.m_Knots;
	m_mode = spl.m_mode;
	m_bSplineGenerateMode = spl.m_bSplineGenerateMode;
	m_vTangent  		  = spl.m_vTangent;
	m_vTangentMagnitude0  = spl.m_vTangentMagnitude0;
	m_vTangentMagnitude1  = spl.m_vTangentMagnitude1;

	m_bSplineIsEdit			 = spl.m_bSplineIsEdit;
	m_nSelected				 = spl.m_nSelected;
	m_nMaped				 = spl.m_nMaped;
	m_Degree = spl.m_Degree;
}

Spline::~Spline()
{
	ClearCtrlPoint();
}
void Spline::FitSpline(varray<Vec4>& inputVecs,varray<float>& knots, int degreeNum,DWORD splineMode)
{
	ClearCtrlPoint();
	ChangeMode(splineMode,degreeNum);
}
void Spline::ChangeMode(DWORD mode,int degree)
{
	m_mode = mode;	
	SetSplineDegree(degree);
}
void Spline::SetSplineDegree(int n)
{
	if(m_mode == SPLMODE_LINER || m_mode == SPLMODE_SPLINE || m_mode == SPLMODE_CLOSED_SPLINE)
	{
        m_Degree = 1;
	}
	else if(m_mode == SPLMODE_2BSPLINE || m_mode == SPLMODE_CLOSED_2BSPLINE)
	{
        m_Degree = 2;
	}
	else if(m_mode == SPLMODE_3BSPLINE || m_mode == SPLMODE_CLOSED_3BSPLINE)
	{
        m_Degree = 3;
	}
	else if(m_mode == SPLMODE_NBSPLINE || m_mode == SPLMODE_CLOSED_NBSPLINE)
	{
        m_Degree = n;
	}
	else if(m_mode == SPLMODE_NONUNI_NBSPLINE)
	{
        m_Degree = n;
	}
	else if(m_mode == SPLMODE_BEZIER || m_mode == SPLMODE_CLOSED_BEZIER)
	{
    }
}
void Spline::ClearStatus()
{
	m_bSplineIsEdit = false;
	m_nSelected = -1;
	m_nMaped = -1;
	m_RenderPt.clear();
}

Vec4 Spline::GetPoint(int i, double t, float tension)
{
	Vec4			v;
	int				CtrlPtsize = 0;
	v.x = v.y = v.z = 0.f;

	CtrlPtsize = static_cast<int>(m_CtrlPts.size());

	if(m_mode == SPLMODE_CLOSED_SPLINE)
	{	
		m_vTangent.resize(CtrlPtsize + 1);
		m_vTangentMagnitude0.resize(CtrlPtsize + 1);
		m_vTangentMagnitude1.resize(CtrlPtsize + 1);
	}
	else
	{
		m_vTangent.resize(CtrlPtsize);
		m_vTangentMagnitude0.resize(CtrlPtsize);
		m_vTangentMagnitude1.resize(CtrlPtsize);
	}

	if ((m_mode != SPLMODE_SPLINE && m_mode != SPLMODE_CLOSED_SPLINE))
	{
		this->m_bSplineGenerateMode = 0; //normal mode 
	}
	//----------------------------------------------------------------
	if (this->m_bSplineGenerateMode == 0) //normal mode
	{
		if (m_mode == SPLMODE_LINER) //this code is right.
		{
			v = (1.0f - t) * m_CtrlPts[i] + t * m_CtrlPts[i+1];
		}
		else if (m_mode == SPLMODE_BEZIER)  //this code is right.
		{
			for(int j = 0; j < CtrlPtsize; j++)
			{
				v += BernsteinFun(CtrlPtsize - 1, j,t) * m_CtrlPts[j];
			}
		} 
		else if(m_mode == SPLMODE_CLOSED_BEZIER) //need to be verified.
		{		
			Vec4 pt = GetCtrlPoint(0);
			AddCtrlPoint(pt);

			int ptNum = GetCtrlPointCount();			
			for(int j = 0; j < ptNum; j++)
			{
				v += BernsteinFun(ptNum - 1, j,t)* m_CtrlPts[j];	
			}
			DelCtrlPoint(ptNum-1);
		}
		else if (m_mode == SPLMODE_SPLINE) //this code is right.
		{
			if ((m_CtrlPts[i+1] - m_CtrlPts[i]).Magnitude() < DBL_EPSILON)
			{
				v = m_CtrlPts[i];
			} 
			else
			{
				Vec4	Pd0, Pd1;
				if (m_CtrlPts.size() > 2)
				{
					if (i == 0) 							
						Pd0 = (1-tension)*((m_CtrlPts[1] + (m_CtrlPts[1] - (m_CtrlPts[0]+m_CtrlPts[2])*0.5f)*0.5f) - m_CtrlPts[0]);
					else									
						Pd0 = (1-tension)*((m_CtrlPts[i] - m_CtrlPts[i-1]) + (m_CtrlPts[i+1] - m_CtrlPts[i])) * 0.5f;

					if ((i + 2) >= CtrlPtsize)		
						Pd1 = (1-tension)*(m_CtrlPts[i+1]-(m_CtrlPts[i]+(m_CtrlPts[i] - (m_CtrlPts[i-1] + m_CtrlPts[i+1])*0.5f)*0.5f));
					else									
						Pd1 = (1-tension)*((m_CtrlPts[i+1] - m_CtrlPts[i]) + (m_CtrlPts[i+2] - m_CtrlPts[i+1])) * 0.5f;

					m_vTangent[i]				= Pd0.Normalize(); //XXX add, set tangent of control point i, 20050119
					m_vTangent[i+1]				= Pd1.Normalize(); //XXX add, set tangent of control point i+1
					m_vTangentMagnitude0[i]		= Pd0.Magnitude(); //XXX, add 20050125
					m_vTangentMagnitude1[i]		= Pd0.Magnitude();
					m_vTangentMagnitude0[i+1]	= Pd1.Magnitude(); //XXX, add 20050125
					m_vTangentMagnitude1[i+1]	= Pd1.Magnitude(); //XXX, add 20050125

					v = ((t-1) * (t-1) * (2*t + 1)) * m_CtrlPts[i] +
						(t * t * (3 - 2*t))         * m_CtrlPts[i+1] +
						((1-t) * (1-t) * t)         * Pd0 + 
						((t-1) * t * t)             * Pd1;
				} 
				else 
				{
					m_vTangent[i]				= (m_CtrlPts[i+1] - m_CtrlPts[i]).Normalize(); //XXX add, set tangent of control point i, 20050119
					m_vTangent[i+1]				= (m_CtrlPts[i+1] - m_CtrlPts[i]).Normalize(); //XXX add, set tangent of control point i+1
					m_vTangentMagnitude0[i]		= (m_CtrlPts[i+1] - m_CtrlPts[i]).Magnitude(); //XXX, add 20050125
					m_vTangentMagnitude1[i]		= (m_CtrlPts[i+1] - m_CtrlPts[i]).Magnitude();
					m_vTangentMagnitude0[i+1]	= (m_CtrlPts[i+1] - m_CtrlPts[i]).Magnitude(); //XXX, add 20050125
					m_vTangentMagnitude1[i+1]	= (m_CtrlPts[i+1] - m_CtrlPts[i]).Magnitude(); //XXX, add 20050125
					v = (1.0f - t) * m_CtrlPts[i] + t * m_CtrlPts[i+1];
				}
			}
		}
		else if (m_mode == SPLMODE_CLOSED_SPLINE) //this code is right.
		{
			int pim1 = i-1;
			int pi   = i;
			int pip1 = i+1;
			int pip2 = i+2;
			if (pim1 < 0) 
				pim1 = CtrlPtsize-1;
			if (pi   >= CtrlPtsize)				
				pi   = pi  - CtrlPtsize;
			if (pip1 >= CtrlPtsize)				
				pip1 = pip1-CtrlPtsize;
			if (pip2 >= CtrlPtsize)				
				pip2 = pip2-CtrlPtsize;
			if ((m_CtrlPts[pip1] - m_CtrlPts[pi]).Magnitude() < DBL_EPSILON)
			{
				v = m_CtrlPts[i];
			}
			else 
			{
				Vec4 Pd0, Pd1;
				Pd0 = (1-tension)*((m_CtrlPts[pi  ] - m_CtrlPts[pim1]) + (m_CtrlPts[pip1] - m_CtrlPts[pi  ])) * 0.5f;
				Pd1 = (1-tension)*((m_CtrlPts[pip1] - m_CtrlPts[pi  ]) + (m_CtrlPts[pip2] - m_CtrlPts[pip1])) * 0.5f;
				m_vTangent[i]				= Pd0.Normalize(); //XXX add, set tangent of control point i, 20050119
				m_vTangentMagnitude0[i]		= Pd0.Magnitude(); //XXX, add 20050125
				m_vTangentMagnitude1[i]		= Pd0.Magnitude(); //XXX, add 20050125
				if(i <  CtrlPtsize)
				{
					m_vTangent[i+1]				= Pd1.Normalize(); //XXX add, set tangent of control point i+1
					m_vTangentMagnitude0[i+1]	= Pd1.Magnitude();
					m_vTangentMagnitude1[i+1]	= Pd1.Magnitude();
				}
				v = ((t-1) * (t-1) * (2*t + 1)) * m_CtrlPts[pi] +
					(t * t * (3 - 2*t))         * m_CtrlPts[pip1] +
					((1-t) * (1-t) * t)         * Pd0 + 
					((t-1) * t * t)             * Pd1;
			}
		}		
		else if (m_mode == SPLMODE_2BSPLINE || m_mode == SPLMODE_CLOSED_2BSPLINE)  //following code is wrong because of not judging the validity for the indexes.
		{
			float F02 = 0.5f * t * t - t + 0.5f;
			float F12 = - t * t + t + 0.5f;
			float F22 = 0.5f * t * t;
			int index1 = i + 1;
			int index2 = i + 2;
			if (index1 >= CtrlPtsize) index1 -= CtrlPtsize;
			if (index2 >= CtrlPtsize) index2 -= CtrlPtsize;
			v = m_CtrlPts[i] * F02 + m_CtrlPts[index1] * F12 + m_CtrlPts[index2] * F22 ;
		}		
        else if (m_mode == SPLMODE_3BSPLINE || m_mode == SPLMODE_CLOSED_3BSPLINE)
		{
			int n=3;
			int j=0;
			float F03 = (-t*t*t + 3*t*t -3*t + 1)/6;
			float F13 = (3*t*t*t - 6*t*t + 4)/6;
			float F23 = (-3*t*t*t + 3*t*t + 3*t + 1)/6;
			float F33 = (t*t*t)/6;
			int index1 = i+1;
			int index2 = i+2;
			int index3 = i+3;
			if (index1 >= CtrlPtsize) index1 -= CtrlPtsize;
			if (index2 >= CtrlPtsize) index2 -= CtrlPtsize;
			if (index3 >= CtrlPtsize) index3 -= CtrlPtsize;
			v = m_CtrlPts[i] * F03 + m_CtrlPts[index1] * F13 + m_CtrlPts[index2] * F23 + m_CtrlPts[index3] * F33;        
		}		
		else if(m_mode == SPLMODE_NBSPLINE || m_mode == SPLMODE_CLOSED_NBSPLINE)
		{
			float scale = 0;
			for(int j=0; j<= m_Degree; j++)
			{
                scale = BSplineFun(m_Degree,j,t);
				int index = i + j;
				if(index >= CtrlPtsize) 
					index -= CtrlPtsize;
				v += m_CtrlPts[index] * scale;
			}
		}
		else if(m_mode == SPLMODE_NONUNI_NBSPLINE)
		{
			float scale = 0;
			for(int j=0; j< CtrlPtsize; j++) 
			{
				scale = OneBasisFun(m_Degree,m_Knots.size()-1,m_Knots,j,t);//BSplineFunRecursive(m_Degree,j,t,m_Knots);				
				v += m_CtrlPts[j] * scale;
			}
		}
	}
	else   //m_bSplineGenerateMode == 1,2?,this code is not used in common case unless tangent having been given.
	{
		if (m_CtrlPts.size() > 2)
		{
			int pip1 = i+1;
			if (pip1 >= CtrlPtsize)				
				pip1 = pip1 - CtrlPtsize;
			v = ((t-1) * (t-1) * (2*t + 1)) * m_CtrlPts[i] +
				(t * t * (3 - 2*t))         * m_CtrlPts[pip1] +
				((1-t) * (1-t) * t)         * m_vTangentMagnitude1[i]    * m_vTangent[i] + 
				((t-1) * t * t)             * m_vTangentMagnitude0[pip1] * m_vTangent[pip1];
		}
		else
		{
			if (i < 1)
				v = (1.0f - t) * m_CtrlPts[i] + t * m_CtrlPts[i+1];
		}
	}
	return v;
}

//----------------------------------------------------------------------
// name:		AddCtrlPoint
// function:	add ctrl point of spline
// argument:	v: the point will be added			
// return:		void
// author:		unknown
// date:		unknown
// update:	    
// author:		XXX
// date:		04/20/2006
//----------------------------------------------------------------------
void Spline::AddCtrlPoint(const Vec4& v)
{
	m_CtrlPts.push_back(v);
}
//样条的节点数目为：n+p+1, n为控制点数目，p为样条阶数，
void Spline::AddKnots(const double knot)
{
    m_Knots.push_back(knot);
	assert(m_Knots.size() <= m_Degree + m_CtrlPts.size() + 1);
	//ASSERT(m_Knots.size() <= m_Degree + m_CtrlPts.size() + 1);
}
void Spline::SetCtrlPoint(int index,const Vec4& v){
	GetCtrlPoint(index) = v;
}

//----------------------------------------------------------------------
// name:		DelCtrlPoint
// function:	delete control point
// argument:	index: the index of the point which will be deleted		
// return:		void
// author:		unknown
// date:		unknown
// update:	    
// author:		XXX
// date:		04/20/2006
//----------------------------------------------------------------------
void Spline::DelCtrlPoint(int index)
{
	int	i;
	int cnt = 0;
	varray<Vec4> tmp = m_CtrlPts;
	int size = GetCtrlPointCount();

	if(size - 1 > 0)
	{
		m_CtrlPts.resize(size-1);

		for(i=0;i<size;i++)
		{
			if(i != index)
			{
				m_CtrlPts[cnt] = tmp[i];
				cnt++;
			}
		}
	}
	else
	{
		ClearCtrlPoint();
	}

	//-------------------------------------------
	//XXX add 20050125
	varray<Vec4> tangent;
	varray<float> m0;
	varray<float> m1;

	tangent = m_vTangent;
	m0 = m_vTangentMagnitude0; 
	m1 = m_vTangentMagnitude1;
	cnt = 0;
	size = static_cast<int>(m_vTangent.size());
	
	if (size > 1 && m_bSplineGenerateMode)
	{
		m_vTangent.resize(size-1);
		m_vTangentMagnitude0.resize(size-1);
		m_vTangentMagnitude1.resize(size-1);

		for(i = 0; i< size;i++)
		{
			if(i != index)
			{
				m_vTangent[cnt]			  = tangent[i];
				m_vTangentMagnitude0[cnt] = m0[i];
				m_vTangentMagnitude1[cnt] = m1[i];
				cnt++;
			}
		}
	}
}

//---------------------------------------------------------------
// Name:	   
// Description:
// Argument:
//         :	
// Return:		
// Author:		XXX
// Date:		 
// Modified by:	XXX
// Updated date: 2005/10/24 24:10:2005   10:20	
//----------------------------------------------------------------
void Spline::InsertCtrlPoint(int idx,Vec4 vt)
{
	DWORD iMode = GetMode();
	if(iMode == Spline::SPLMODE_CLOSED_BEZIER 
	|| iMode == Spline::SPLMODE_CLOSED_2BSPLINE
	|| iMode == Spline::SPLMODE_CLOSED_SPLINE)
	{
		m_CtrlPts.push_back(m_CtrlPts.back());
	}
	if(idx < 0 || idx > size() -2)
	{
		return;
	}
	int i = 0;
	varray<Vec4>	tmp = m_CtrlPts;
	m_CtrlPts.clear();
	for(i = 0; i <= idx; i++)
	{
		m_CtrlPts.push_back(tmp[i]);
	}
	m_CtrlPts.push_back(vt);
	for(i = idx+1; i < static_cast<int>(tmp.size()); i++)
	{
		m_CtrlPts.push_back(tmp[i]);
	}

	//------ followes code should be checked if we want to use tangent line to control spline shape 
	varray<Vec4>		tangent;
	varray<float>		m0;
	varray<float>		m1;

	tangent = m_vTangent;
	m0 = m_vTangentMagnitude0; 
	m1 = m_vTangentMagnitude1;
	
	if (m_vTangent.size() > 1 && m_bSplineGenerateMode == 1)
	{
		m_vTangent.clear();
		m_vTangentMagnitude0.clear();
		m_vTangentMagnitude1.clear();

		for(i = 0; i< idx;i++)
		{
				m_vTangent.push_back(tangent[i]);
				m_vTangentMagnitude0.push_back(m0[i]);
				m_vTangentMagnitude1.push_back(m1[i]);
		}

		Vec4 vtTan = ((vt - tmp[idx]) + (tmp[idx+1] - vt)) / 2;
		vtTan = vtTan.Normalize();
	
		float fm0     = (tmp[idx+1] - tmp[idx]).Magnitude()/2;
		float fm1     = fm0;
		AddTangent(vtTan, fm0, fm1);


		for(i = idx+1; i < static_cast<int>(tangent.size()); i++)
		{
			m_vTangent.push_back(tangent[i]);
			m_vTangentMagnitude0.push_back(m0[i]);
			m_vTangentMagnitude1.push_back(m1[i]);
		}
	}
	if(iMode == Spline::SPLMODE_CLOSED_BEZIER 
	|| iMode == Spline::SPLMODE_CLOSED_2BSPLINE
	|| iMode == Spline::SPLMODE_CLOSED_SPLINE)
	{
		m_CtrlPts.pop_back();
	}
}
void Spline::ClearCtrlPoint()
{
	m_CtrlPts.clear();
	m_Knots.clear();
	//-----------------------------------
	//XXX add 20050125
	m_vTangent.clear();
	m_vTangentMagnitude0.clear();
	m_vTangentMagnitude1.clear();
	//----------------------------------
}

//Vec4 Spline::GetPointByLength(float len)
//{
//	float sumlen = 0;
//	Vec4 r;
//
//	if (len >= 0) 
//	{
//		Vec4 prevp = GetCtrlPoint(0);
//		r = prevp;
//
//		for (int i = 1; i < GetCtrlPointCount()*2; i++)
//		{
//			int p = i;
//			if (p >= GetCtrlPointCount()) 
//				p -= GetCtrlPointCount();
//			Vec4& v = GetCtrlPoint(p);
//			float l = (v-prevp).Magnitude();			
//			if (sumlen+l < len)
//			{
//				sumlen+=l;
//				r = v;
//			} 
//			else 
//			{
//				int pm1 = p-1;
//				if (pm1 < 0) pm1 = GetCtrlPointCount()-1;
//				float t = (len - sumlen)/l;
//				r = GetCtrlPoint(pm1)*(1-t)+GetCtrlPoint(p)*t;
//				break;
//			}
//			prevp = v;
//		}
//	} 
//	else 
//	{
//		int p = 0;
//		Vec4 prevp = GetCtrlPoint(0);
//		r = prevp;
//		len = -len;
//
//		for (int i = 0; i < GetCtrlPointCount()-1; i++ )
//		{
//			p--;
//			if (p < 0) p = GetCtrlPointCount()-1;
//
//			Vec4& v = GetCtrlPoint(p);
//			float l = (v-prevp).Magnitude();
//			
//			if (sumlen+l < len) {
//				sumlen+=l;
//				r = v;
//			} else {
//				int pp1 = p+1;
//				if (pp1 >= GetCtrlPointCount()) pp1 = 0;
//				float t = (len - sumlen)/l;
//				r = GetCtrlPoint(pp1)*(1-t)+GetCtrlPoint(p)*t;
//				break;
//			}
//			prevp = v;
//		}
//	}
//
//	return r;
//}
//XXX,2005.1.24
//bool Spline::GetPointByLength(float len,Vec4& vt)
//{	
//	if(len<0){
//		return false;
//	}
//	if(len==0){
//		vt = GetCtrlPoint(0);
//		return true;
//	}
//	//-------------------end modify------------------
//	int size = GetCtrlPointCount()-1;	
//	if(GetMode()==Spline::SPLMODE_CLOSED_SPLINE){
//		size++;
//	}
//	float sumlen = 0.0;
//	Vec4 prevp = p(0);
//	Vec4 pt;
//	bool bIsPtFind = false;
//	for(int i = 0; i < size; i++)
//	{
//		for(float t = 0; t < 1.01f ; t += 0.1f)
//		{
//			pt = GetPoint( i , t );
//			sumlen += (pt-prevp).Magnitude();	
//			if(sumlen > len)
//			{
//				float scl = (sumlen-len)/(pt-prevp).Magnitude();
//				vt = prevp*scl+pt*(1-scl);
//				bIsPtFind = true;
//				break;
//			}
//			prevp = pt;
//		}
//		if(bIsPtFind){
//			break;
//		}
//	}
//	return bIsPtFind;
//}
float Spline::GetLength(int n)
{
	float sumlen = 0;
	Vec4 prevp = GetCtrlPoint(0);
	for (int i = 1; i <= n; i++) 
	{
		int p = i;
		if (p >= GetCtrlPointCount()) 
			p -= GetCtrlPointCount();
		Vec4& v = GetCtrlPoint(p);
		sumlen += (v-prevp).Magnitude();
		prevp = v;
	}
	return sumlen;
}
// XXX,2005.1.13
//float Spline::GetSplLenBetweenTwoPts(int iFirPtIdx,int iSecPtIdx)
//{
//	int i;
//	float t,len=0.0;
//	Vec4 p;
//	Vec4 plist[1000];
//	int size = GetCtrlPointCount()-1;
//
//	int np=0;	
//	if(GetMode()==Spline::SPLMODE_CLOSED_SPLINE){
//		size++;
//	}
//	if(iFirPtIdx < 0){
//		iFirPtIdx = 0;
//	}
//	if(iSecPtIdx>size){
//		iSecPtIdx = size;
//	}
//	if(iFirPtIdx>iSecPtIdx){
//		int iTemp = iFirPtIdx;
//		iFirPtIdx = iSecPtIdx;
//		iSecPtIdx = iTemp;
//	}
//	for(i = iFirPtIdx; i < iSecPtIdx;i++){ 
//		for(t = 0; t < 1.01f ; t += 0.1f){
//			p = GetPoint( i , t );
//			plist[np] = p;
//
//			if(np>0){
//				if( (p-plist[np-1]).Magnitude()>0.001)
//					np++;
//			}
//			else
//				np++;
//		}
//	}
//	for(i=0; i < np-1; i++){
//		len += (plist[i+1] - plist[i]).Magnitude();
//	}
//	return len;
//}
bool Spline::Save(BaseOStream& bs) const
{
	bs.set_ascii(false);
	bs << (DWORD)GetMode();
	bs << (DWORD)GetCtrlPointCount();
	if( GetCtrlPointCount() > 0 ) bs.write(&m_CtrlPts[0], GetCtrlPointCount()*sizeof(Vec4));
	return true;
}

bool Spline::Load(BaseIStream& bs)
{
	DWORD t;
	bs >> t; ChangeMode(t);
	bs >> t; m_CtrlPts.resize(t);
	if (t > 0) bs.read(&m_CtrlPts[0], GetCtrlPointCount()*sizeof(Vec4));
	return true;
}
bool Spline::Save0010(BaseOStream& bs) const
{
	//bs.set_ascii(true);
	//DWORD	tt;
	//int		i; //XXX add 20050125
	//tt = (DWORD)GetMode();
	//bs << tt<<_T(" ");
	////-----------------------------------------
	////save spline generate mode, XXX add 20050121
	//tt = (DWORD)(this->m_bSplineGenerateMode);
	//bs << tt << _T(" "); 
	//bs << this->m_yCrossSection << _T(" "); 

	////save tangent vector
	//tt = (DWORD)this->m_vTangent.size(); 
	//bs << tt << endl;
	//for (i = 0; i < static_cast<int>(m_vTangent.size()); i++)
	//{
	//	bs << m_vTangent[i].x << _T(" ") << m_vTangent[i].y << _T(" ") <<  m_vTangent[i].z << endl;
	//}
	////save magnitude0
	//tt = (DWORD)this->m_vTangentMagnitude0.size();
	//bs << tt << endl;
	//for (i = 0; i < static_cast<int>(m_vTangentMagnitude0.size()); i++)
	//{
	//	bs << m_vTangentMagnitude0[i] << endl;
	//}
	////save magnitude1
	//tt = (DWORD)this->m_vTangentMagnitude1.size();
	//bs << tt << endl;
	//for (i = 0; i < static_cast<int>(m_vTangentMagnitude1.size()); i++)
	//{
	//	bs << m_vTangentMagnitude1[i] << endl;
	//}
	////------------------------------------------
	//tt = (DWORD)GetCtrlPointCount();
	//bs <<tt<<endl;
	//for(i = 0; i<GetCtrlPointCount(); i++)
	//{
	//	bs<<m_CtrlPts[i].x<<_T(" ")<<m_CtrlPts[i].y<<_T(" ")<<m_CtrlPts[i].z<<endl;
	//}
	//return true;
	return true;
}
bool Spline::Load0010(BaseIStream& bs,float fCurVersion)
{
	DWORD	t;
	unsigned int		i;//XXX add 20050125
	bool		bBinaryFormat = false;
	char	strTemp[1];
	if(fabs(fCurVersion-1.1) < 0.0005||(fCurVersion - 1.1 > 0.0005))
	{
		bBinaryFormat = true;
		bs >> t >> strTemp[0]; 
	}
	else
	{
		bBinaryFormat = false;
		bs >> t; 
	}

	ChangeMode(t);
	
	if (bBinaryFormat)
	{
		//when version >= 1.003, need to load tangent and spline gererate mode
		if(fabs(fCurVersion-1.003) < 0.0005||(fCurVersion - 1.003 > 0.0005))
		{
			//--------------------------------------------------------------
			bs >> t >> strTemp[0]; this->m_bSplineGenerateMode = t; //XXX add 20050121

			if(fabs(fCurVersion-1.019) < 0.0005||(fCurVersion - 1.019 > 0.0005))
			{ 
				bs >> m_yCrossSection >> strTemp[0];
			}

			//load tangent vector
			bs >> t >> strTemp[0]; this->m_vTangent.resize(t);
			for (i = 0; i < t; i++)
			{
				bs >> m_vTangent[i].x >> strTemp[0] >>  m_vTangent[i].y >> strTemp[0] >>  m_vTangent[i].z >> strTemp[0];
			}
			//load magnitude0
			bs >> t >> strTemp[0]; this->m_vTangentMagnitude0.resize(t);
			for (i = 0; i < t; i++)
			{
				bs >> m_vTangentMagnitude0[i]>> strTemp[0];
			}
			//load magnitude1
			bs >> t >> strTemp[0]; this->m_vTangentMagnitude1.resize(t);
			for (i = 0; i < t; i++)
			{
				bs >> m_vTangentMagnitude1[i] >> strTemp[0];
			}
		}
		//---------------------------------------------------------------

		bs >> t >> strTemp[0];
		m_CtrlPts.resize(t);
		for(unsigned int i = 0; i < t; i++)
		{
			bs>>m_CtrlPts[i].x >> strTemp[0] >>m_CtrlPts[i].y >> strTemp[0]>>m_CtrlPts[i].z >> strTemp[0];
		}
	}
	else
	{
		//when version >= 1.003, need to load tangent and spline gererate mode
		if(fabs(fCurVersion-1.003) < 0.0005||(fCurVersion - 1.003 > 0.0005))
		{
			//--------------------------------------------------------------
			bs >> t; this->m_bSplineGenerateMode = t; //XXX add 20050121
			if(fabs(fCurVersion-1.023) < 0.0005||(fCurVersion - 1.023 > 0.0005))
			{
				bs >> m_yCrossSection;
			}
			//load tangent vector
			bs >> t; this->m_vTangent.resize(t);
			for (i = 0; i < t; i++)
			{
				bs >> m_vTangent[i].x >>  m_vTangent[i].y >>  m_vTangent[i].z;
			}
			//load magnitude0
			bs >> t; this->m_vTangentMagnitude0.resize(t);
			for (i = 0; i < t; i++)
			{
				bs >> m_vTangentMagnitude0[i];
			}
			//load magnitude1
			bs >> t; this->m_vTangentMagnitude1.resize(t);
			for (i = 0; i < t; i++)
			{
				bs >> m_vTangentMagnitude1[i];
			}
		}
		//---------------------------------------------------------------

		bs >> t;
		m_CtrlPts.resize(t);
		for(unsigned int i = 0; i < t; i++)
		{
			bs>>m_CtrlPts[i].x >>m_CtrlPts[i].y>>m_CtrlPts[i].z;
		}
	}
	return true;
}
Spline* Spline::CreateMe() const
{
	Spline* pSpline = new Spline();
	*pSpline = (*this);
	return pSpline;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//----------------------------------------------------------------------
// name:		SetCtrlPntTangent
// function:	Get the tangent of the control point, Now only support spline
// argument:	int i: index of control point
// return:		void
// author:		XXX
// date:		2004/12/22
// update:	
// author:		
// date:		
//---------------------------------------------------------------------- 
void Spline::SetTangentOfCtrlPnt(Vec4& vTangent, int i)
{
	this->m_vTangent[i] = vTangent;
}

//----------------------------------------------------------------------
// name:		GetCtrlPntTangent
// function:	Get the tangent of the control point, Now only support spline
// argument:	int i: index of control point
// return:		Vec4: tangent
// author:		XXX
// date:		2004/12/22
// update:	
// author:		
// date:		
//---------------------------------------------------------------------- 
Vec4 Spline::GetTangentOfCtrlPnt(int i)
{
	return this->m_vTangent[i];
}

//----------------------------------------------------------------------
// name:		SetSplineGenerateMd
// function:	set spline generate mode, it is only for spline
//						0:normal mode; 	                                    
//						1:set the tangent of the control point directly for spline
// argument:	int iMode: spline generate mode
// return:		void 
// author:		XXX
// date:		2004/12/30
// update:	
// author:		
// date:		
//---------------------------------------------------------------------- 
void Spline::SetSplineGenerateMd(int iMode)
{
  this->m_bSplineGenerateMode = iMode;
}

//----------------------------------------------------------------------
// name:		GetSplineGenerateMd
// function:	Get spline generate mode, it is only for spline
//						0:normal mode; 	                                    
//						1:set the tangent of the control point directly for spline
// argument:	void
// return:		int
// author:		XXX
// date:		2004/12/30
// update:	
// author:		
// date:		
//---------------------------------------------------------------------- 
int Spline::GetSplineGenerateMd(void)const
{
  return this->m_bSplineGenerateMode;
}

//----------------------------------------------------------------------
// name:		SetMagnitudeOfTangent
// function:	set the magnitude of control point
// argument:	int i: control point index
//				int j: 0: set magnitude0, 1: set magnitude1
//				float magnitude: 
// return:		void
// author:		XXX
// date:		2004/12/30
// update:	
// author:		
// date:		
//---------------------------------------------------------------------- 
void Spline::SetMagnitudeOfTangent(int i, int j, float fMagnitude) 
{
	if (j == 0)
	{
        m_vTangentMagnitude0[i] = fMagnitude;
	}
	else
	{
		m_vTangentMagnitude1[i] = fMagnitude;
	}
}

//----------------------------------------------------------------------
// name:		GetMagnitudeOfTangent
// function:	Get the magnitude of control point
// argument:	int i: control point index
//				int j: 0: set magnitude0, 1: set magnitude1
// return:		float:  magnitude value
// author:		XXX
// date:		2004/12/30
// update:	
// author:		
// date:		
//---------------------------------------------------------------------- 
float Spline::GetMagnitudeOfTangent(int i, int j)
{
	if (j == 0)
	{
        return m_vTangentMagnitude0[i];
	}
	else
	{
		return m_vTangentMagnitude1[i];
	}
}

//----------------------------------------------------------------------
// name:		AddTangent
// function:	when insert a control point, we need to add its tangent
// argument:	vTangent: normalize vector of tangent
//				fMagnitude0: magnitude0
// return:		fMagnitude1: magnitude1
// author:		XXX
// date:		2004/12/30
// update:	
// author:		
// date:		
//---------------------------------------------------------------------- 
void Spline::AddTangent(Vec4 vTangent, float fMagnitude0, float fMagnitude1)
{
	int size = static_cast<int>(m_CtrlPts.size());
	if(size > 0)
	{
		m_vTangent.resize(size);
		m_vTangentMagnitude0.resize(size);
		m_vTangentMagnitude1.resize(size);
   
		m_vTangent[size -1] = vTangent;
		m_vTangentMagnitude0[size -1] = fMagnitude0;
		m_vTangentMagnitude1[size -1] = fMagnitude1;
	}
}
}