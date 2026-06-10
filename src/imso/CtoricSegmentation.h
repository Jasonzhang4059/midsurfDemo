#pragma once
#include "varray.h"
#include "pointXd.h"
#include "CToric.h"
#include "CNurbs.h"

class toricSegment
{
private:
	//结点
	point3d m_CenPts;
	varray<point3d> m_linesDu;//骨架线首点切矢
	varray<NurbsLine> m_lines;//骨架线
	double m_r;

	//中心线
	varray<point3d> m_CenLine;
public:
	//外合矢信息
	struct outsideVec
	{
		point3d pts;//角点坐标
		point3d vec;//外合矢向量(单位化)
		point3d twinPts;//外合矢射线与曲面的交点
		bool isEdge = false;//是否为边界点
		bool hasTwinPts = false;
	};

	//曲线信息
	struct lineinf
	{
		int idx0;//曲线端点对应的cube
		int idx1;
		varray<point4d> ctrlPts;
	};
	varray<lineinf> m_lineInf;
	

	struct VecTop
	{
		int volIdx = -1;
		varray<int> coners;//[4]，角点编号
		varray<varray<int>> linePara;//[4][3],每条线的uvw情况，2表示为变量参数正序，3为逆序
	};
	varray<VecTop> m_OutsideVecsTop;//外合矢拓扑
	varray<outsideVec> m_OutsideVecs;//外合矢
	varray<NurbsSurface> m_OutsideSurface;
	NurbsLine sweepPath;
	//分割后的骨架曲线
	//varray<varray<NurbsLine>> m_SegLines;

	//varray<varray<point4d>> m_BondaryCtrlPts;//边界曲线控制点，按toric参数域方向顺序排列
	//varray<double> m_knots;
	varray<varray<point3d>> m_samPts; //内部分割线
	//varray<varray<varray<point4d>>> m_segSFBoundary; //分割后，四条线构成面片的控制点
	//int m_lineNum;

	varray<toric>& m_torics;
	varray<NurbsVol> m_Vols;

	//toricSegment() {};
	toricSegment(varray<toric>& mtoric);
	~toricSegment();

	/*
	*/
	void ToricSegmentation(const point3d CenPts, const varray<NurbsLine>& lines, const double r);

private:

	//直线与toric曲面交点的参数域值,z值表示m_toric中的编号
	//p0:直线点
	//v0:直线方向向量
	double LineCrossTorics(const point3d& p0, const point3d& v0, point3d& res);

	//第一组内部体
	void FirstVols(const double la, const double lb);

	//角平分线
	void CalBis(varray<varray<point3d>>& bis);

	//计算外合矢
	void CalOutsideVec();

	//外合矢拓扑
	void FindOutsideVecTop();

	int FindIndex(const point3d pts);

	//去除重合面
	void DelSurface();

	//两数组元素是否相同(无序)
	bool IsSameEle(const varray<int>& a, const varray<int>& b);

	//采样
	void Sampling(const int topIdx, const int lineIdx, const int num, varray<point3d>& sampts);

	//边界六面体
	void CreateBounVol();

	//射线与toric曲面交点的参数域值
	//p0:直线点
	//v0:直线方向向量
	//若不收敛，返回(-1,-1)
	double LineCrossToric(const point3d& p0, const point3d& v0, const int ToricIdx, point2d& res);

	//曲线查找
	int FindLineInf(const int idx0, const int idx1, bool& isRev);

	//根据top信息寻找棱线点
	//topIdx：m_OutsideVecsTop索引
	//lineIdx：VecsTop中边的索引
	//x:采样点参数值
	point3d FindPrismPts(const int topIdx, const int lineIdx, const double x);

	//边界面拟合
	void FittingBounSF();

	//边界判断
	int InEdge(toric & tor, const point2d & p0);

	//根据VecTop获得cube面上的控制点
	void GetSFCtrlPtsWithVecTop(const VecTop & vectop, varray<point4d>& ctrlPts);

	//拟牛顿迭代法
	double NewtonItera(const point3d& p0, const point3d& v0, const int ToricIdx, point2d& res);

	//test
	void WriteData();
	void testData();
};