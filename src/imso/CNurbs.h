//Nurbs相关的标准函数
//算法来自《非均匀有理B样条(第2版)》 Les Piegl,Wayne Tiller著; 赵罡等译; 清华大学出版社
//@ USST 何文彬 2018

#pragma once

#include "stdafx.h"
#include "XVec.h"
class BezierLine
{
	//计算u处所有n次Bernstein多项式的值
	void AllBernstein(const int n, const double u, varray<double>& B)const;
public:
	int m_Degree;//曲线次数
	varray<point4d> m_CtrlPts;//控制点
	//计算Bezier曲线点
	point3d GetLinePoint(const double u)const;
	//计算Bezier曲线
	void CalLinePoint(const int Unum, varray<point3d>& linePts)const;
};

	class NurbsBase
	{
	public:
		/*计算节点下标
		x：节点值
		degree：次数
		CtrlPtsNum：控制点数量
		knots：节点矢量*/
		int FindSpan(const double x, const int degree, const int CtrlPtsNum, const varray<double>& knots)const;

		/*根据参数值，节点下标，计算基函数
		//u：参数值
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

	//NURBS曲线
	struct NurbsLine :public NurbsBase
	{
		int m_Degree;//曲线次数
		varray<double> m_Knots;//曲线节点矢量
		varray<point4d> m_CtrlPts;//控制点

		//创建直线形式曲线
		void CreatLineWithTwoPoints(const point3d &p1, const point3d &p2, int degree = 2);

		/*计算u节点对应的曲线坐标点
		u：节点参数*/
		point3d GetLinePoint(const double u)const;

		/*获取线上的点
		Unum：曲线点数量
		linePts：曲线点	*/
		void CalLinePoint(const int Unum, varray<point3d>& linePts)const;

		/*曲线矢值函数A(u)所有p阶导，矢值函数A(u)即NURBS曲线分子式
		u：参数
		n：导矢阶数
		Der：A(u)所有p阶导*/
		void PtsDerivsAu(const double u, const int n, varray<point4d>& Der)const;

		/*曲线上u处所有n阶导
		u：参数
		n：导矢阶数
		Der：所有n阶导 */
		void PtsDerivs(const double u, const int n, varray<point4d>& Der)const;

		/*曲线所有n阶导
		step：参数域等分步进量
		Der：曲线点对应的n阶导*/
		void CurveDerivs(const int n, const double step, varray<varray<point3d>>& Der);

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
		void Segmentation(const double u, varray<NurbsLine>& lines);

		/*NURBS曲线分解为BEZIER
		Qw：输出的BEZIER曲线段控制点*/
		void Decompose(varray<varray<point4d>>& Qw);

		//反转曲线方向
		void CruveReverse();
	};

	//NURBS曲面
	struct NurbsSurface :public NurbsBase
	{
		int m_uDegree;//v方向次数
		int m_vDegree;//v方向次数
		int m_uNum;//v方向控制点数量
		int m_vNum;//v方向控制点数量
		varray<double> m_uKnots;//v方向节点矢量
		varray<double> m_vKnots;//v方向节点矢量
		varray<point4d> m_CtrlPts;//控制点,按v-u方向存储

		void SetSurface(const int uDegree, const int vDegree, const int uNum, const int vNum, 
			const varray<double>& uKnots, const varray<double>& vKnots);



		//计算（u，v）对应的曲面上的点
		point3d GetSurFacePoint(const double u, const double v)const;

		//计算四边形面片显示数据
		//num:该方向采样点数量
		//quads:面片数据
		//lines:等参线数据
		threadParam CalQuads(const int Unum, const int Vnum)const;

		/*Coons插值
		EndgCtrlPts:边界控制点（e0,e1,e2,e3顺序）*/
		void CoonsInterpolate(const varray<varray<point4d>>& EdgeCtrlPts);

		/*Coons插值
		EdgeLines:边界曲线（e0,e1,e2,e3顺序）*/
		void CoonsInterpolate(const varray<NurbsLine>& EdgeLines);

        //曲面升阶
		//Udegree,Vdegree:升阶后次数
		void DegreeElevate(const int Udegree, const int Vdegree);

		//曲面节点插入
		//Uknot,Vknot:需要插入的节点
		void KnotsRefine(const varray<double>& Uknot, const varray<double>& Vknot);

		//曲面分段为Bezier曲面
		//dir:0=U方向，1=V方向
		//QW：Bezier曲面控制点
		void Decompose(const bool dir, varray<varray<varray<point4d>>>& QW);  

		//根据控制点二维序号计算一维序号
		int CtrlPtsIdx(const int uIdx, const int vIdx);

		//控制点排序转换为U-V
		void OrderCtrlPts();

		//控制点排序转换为U-V
		void OrderCtrlPts(NurbsSurface& sf);

		//提取边界线,排序已经是U-V情况下
		void GetEdgeLines(varray<NurbsLine>& EdgeLines);

	};

	//NURBS体
	struct NurbsVol :public NurbsBase
	{
		int m_uDegree;//u方向次数
		int m_vDegree;//v方向次数
		int m_wDegree;//w方向次数
		int m_uNum;//v方向控制点数量
		int m_vNum;//v方向控制点数量
		int m_wNum;//w方向控制点数量
		varray<double> m_uKnots;//v方向节点矢量
		varray<double> m_vKnots;//v方向节点矢量
		varray<double> m_wKnots;//w方向节点矢量
		varray<point4d> m_CtrlPts;//控制点，按v-u-w方向存储

		void SetVol(const int uDegree, const int vDegree, const int wDegree, const int uNum, const int vNum, const int wNum,
			const varray<double>& uKnots, const varray<double>& vKnots, const varray<double>& wKnots);

		//(u,v,w)对应的体上的点
		point3d GetVolPoint(const double u, const double v, const double w)const;
		

		//计算四边形面片显示数据
		//num:该方向采样点数量
		//quads:面片数据
		//lines:等参线数据
		threadParamVOL CalQuads(const int Unum, const int Vnum, const int Wnum)const;

		//根据控制点三维序号计算一维序号
		int CtrlPtsIdx(const int uIdx, const int vIdx, const int wIdx);

		//控制点排序转换为U-V-W
		void OrderCtrlPts();

		//控制点排序转换为U-V-W
		void OrderCtrlPts(NurbsVol& vol);

		//节点插入
		//Uknot,Vknot,Wknot:需要插入的节点
		void KnotsRefine(varray<double>& Uknot, varray<double>& Vknot, varray<double>& Wknot);
		
		//升阶
		//Udegree,Vdegree,Wdegree:升阶后阶数
		void DegreeElevate(const int Udegree, const int Vdegree, const int Wdegree);

		/*扫描生成Nurbs体模型，截面垂直于路径
		pathT：扫描路径
		nurbsSF：起始截面
		K：截面实例数量,一般取路径控制点数量减1
		*/
		void CreateSweepNurbsVol(const NurbsLine& pathT, const NurbsSurface& nurbsSF, const int K);

		/*平移扫描生成Nurbs体模型，截面不垂直于路径
		pathT：扫描路径
		nurbsSF：起始截面*/
		void CreateTransSweepNurbsVol(const NurbsLine& pathT, const NurbsSurface& nurbsSF);

		//放样
		//path:路径
		//surfaces:放样截面
		void LoftingNurbsVol(const NurbsLine& path, const varray<NurbsSurface>& surfaces);

		void LoftingNurbsVol2(const NurbsLine& path, const varray<NurbsSurface>& surfaces);

		//放样
		//path:路径
		//surfaces0:路径起点放样截面
		//surfaces1:路径终点放样截面
		void LoftingNurbsVol(const NurbsLine& path, const NurbsSurface& surfaces0,const NurbsSurface& surfaces1);

		//提取单个边界面
		//dir:1->6表示方向
		NurbsSurface GetSingleSurfaces(int dir) const;

		//体COONS插值
		void VolCoonsInterpolate(const varray<NurbsLine>& EdgeLines);

		//refinenum:细化次数
		void KnotsRefineNum(int refinenum)
		{
			//存放新产生的节点，作为细化函数的参数
			varray <double> new_uknots;
			varray <double> new_vknots;
			varray <double> new_wknots;
			int i = 1;
			while (i <= refinenum)
			{
				//找出中间节点 u方向
				for (int i = 1; i < m_uKnots.size(); i++)
				{
					if (m_uKnots[i - 1] != m_uKnots[i])
					{
						new_uknots.push_back((m_uKnots[i - 1] + m_uKnots[i]) / 2);
					}
				}

				//找出中间节点 v方向
				for (int i = 1; i < m_vKnots.size(); i++)
				{
					if (m_vKnots[i - 1] != m_vKnots[i])
					{
						new_vknots.push_back((m_vKnots[i - 1] + m_vKnots[i]) / 2);
					}
				}
				//找出中间节点 w方向
				for (int i = 1; i < m_wKnots.size(); i++)
				{
					if (m_wKnots[i - 1] != m_wKnots[i])
					{
						new_wknots.push_back((m_wKnots[i - 1] + m_wKnots[i]) / 2);
					}
				}
				//细化
				KnotsRefine(new_uknots, new_vknots, new_wknots);

				new_uknots.clear();
				new_vknots.clear();
				new_wknots.clear();

				i++;
			}
		}
	private:
		//计算等参面
		//uvw:0=u,1=v,2=w
		//t:参数
		//num:以u-v-w顺序
		//L:等参面四边形面片集
		void CalIsoSurface(const int uvw, const double t, const int num1, const int num2,
			varray<varray<point3d>>& quads, varray<varray<point3d>>& lines)const;

		/*扫描时的截面实例位置
		pathT：扫描路径
		K：截面实例数量
		pos：截面实例位置
		NewKnots：新节点矢量*/
		void InsLocation(const NurbsLine & pathT, int K, varray<double> & pos);

		/*计算扫描路径上的局部坐标系
		pathT：扫描路径
		pos：截面实例位置
		TranMat：pos处的局部坐标系*/
		void LocalCoordinates(const NurbsLine & pathT, const varray<double> & pos,
			varray<varray<point4d>> & TranMat);

		/*沿扫描路径对截面进行变换
		nurbsSF：截面控制点
		TranMat：变换矩阵
		OriCoordinates:截面的局部坐标系
		allNurbsSF：得到的所有截面实例控制点
		*/
		void MatrixTran(const varray<varray<point4d>> & nurbsSF, const varray<varray<point4d>>& TranMat,
			varray<varray<varray<point4d>>>& allNurbsSF);

		//扫描生成Nurbs体
		void SweepSurface(const varray<varray<varray<point4d>>>& allNurbsSF, 
			const varray<varray<varray<double>>>& SFw, const varray<double>& pos);

		//取最高次
		void MaxDegree(const varray<NurbsSurface>& surfaces, int& uDegree, int& vDegree);

		//节点矢量并集
		void KnotsUnify(const varray<NurbsSurface>& surfaces, varray<double>& NewUKnots, varray<double>& NewVKnots);
	};