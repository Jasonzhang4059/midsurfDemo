//三次B样条曲线曲面最小二乘法拟合

#pragma once
#ifndef CB_spline
#define CB_spline
#include "XVec.h"


//#include "varray.h"
#include "XVec.h"
//#include "D:/study/project/Visual Studio/Library/Math/Eigen/dense"
#include "MathUtil.h"
#include "../../3rdParty/eigen-3.3.8/Eigen/Dense"

#include <string>
#include <cmath>
#include <iostream>

namespace bsl
{
	using namespace base;
	using namespace Eigen;
	class FitBSpline
	{
	public:
		//默认构造函数
		FitBSpline();

		//默认析构函数
		~FitBSpline();
	public:
		double m_FittingErr;//拟合误差（标准差）
		varray<Vec4> m_CtrlPts;//控制点集
		varray<double> m_Knots;//节点矢量
		int m_Degree;//曲线次数
		
		varray<double> m_oriPtsPara;//离散点参数化后的量
	private:
		varray<Vec4> m_oriPtsVec4s;//离散点集
		int m_CtrlPtsNum;//控制点数量
		//积累弦长法参数化离散点
		void ParaPts();

		//创建节点矢量
		void CreateKnots();

		//最小二乘法求控制点
		void CalCtrlPts();

		//计算拟合精度
		void CalFittingErr();

	public:
		//B样条函数
		Vec4 Bspl(double t);

		/*拟合曲线
		inputVecs：离散点集
		sdegree：曲线次数
		ctrlNum:允许的最大控制点数量
		*/
		void FittingBspl(const varray<Vec4>& inputVecs, int sdegree = 3, int ctrlNum = 4);

		/*拟合曲线
		inputVecs：离散点集
		Knots:节点矢量
		sdegree：曲线次数
		ctrlNum:允许的最大控制点数量
		*/
		void FittingBspl(const varray<Vec4>& inputVecs, varray<double> Knots, int sdegree = 3, int ctrlNum = 4);

		//对PtsVec插值，使其点的数量等于ptsNum
		void Interpolate(varray<Vec4>& PtsVec, int ptsNum);
	};



//////////////////////////////FitBSplineSurface////////////B-样条曲面离散点拟合///////////////////////////////////

	class FitBSplineSurface
	{
	public:
		varray<varray<Vec4>> m_uvCtrlPts;//控制点集
		varray<double> m_uKnots;//u方向节点矢量
		varray<double> m_vKnots;//v方向节点矢量
		double m_FittingErr;

		FitBSplineSurface();
		~FitBSplineSurface();

		/*密集散乱点拟合曲面
		allInputPts:离散点集
		all4Edge:离散点边界
		degree:曲线次数
		uCtrlPnum:u方向控制点数
		vCtrlPnum:v方向控制点数
		iterations:迭代次数,若省略此参数则使用有序散乱点分组拟合
		*/
		void FittingBsurfaceNew(varray<Vec4>& allInputPts, varray<varray<Vec4>>& all4Edge,
			int udegree, int vdegree, int uCtrlPnum, int vCtrlPnum, int iterations);


		/*有序离散点分组拟合曲面
		allInputdata：离散点集
		all4edge：边界点集
		degree：曲线次数
		uCtrlNum:u方向控制点数量
		vCtrlNum:v方向控制点数量
		*/
		void FittingBsurface(const varray<Vec4>& allInputdata, const varray<varray<Vec4>>& all4edge,
			int udegree = 3, int vdegree = 3, int uCtrlNum = 4, int vCtrlNum = 4);

		//private:
		varray<varray<Vec4>> m_uPtsVec;//固定v后，u方向的点集

	private:
		struct Bsurface//曲面结构体
		{
			int udegree;//U方向曲线次数
			int vdegree;//V方向曲线次数
			varray<varray<Vec4>> uvCtrlPts;//控制点集
			varray<double> uKnots;//u方向节点矢量
			varray<double> vKnots;//v方向节点矢量

			void clear()
			{
				uvCtrlPts.clear();
				uKnots.clear();
				vKnots.clear();
			}
		};

		//输入的原始数据
		varray<Vec4> m_allInputPts;//离散点集
		varray<varray<Vec4>> m_all4Edge;//边界离散点集
		int m_udegree;//U方向曲线次数
		int m_vdegree;//V方向曲线次数
		int m_uCtrlPtsNum;//u方向控制点数量
		int m_vCtrlPtsNum;//v方向控制点数量
		
		//过程变量
        Bsurface m_baseSurface;//基面

		Vec4 m_uvOriginPts;//u,v原点（m_all4Edge[0]和m_all4Edge[1]交点）
		Vec4 m_uDir;//u方向向量(m_all4Edge[0]为u方向，m_all4Edge[1]为v方向）
		Vec4 m_vDir;//v方向向量(m_all4Edge[0]为u方向，m_all4Edge[1]为v方向）

		varray<int> m_uTx; //uvw坐标系对应的xyz坐标系, 0 = x, 1 = y, 2 = z

		varray<varray<Vec4>> m_EdgeCtrlPts;//边界控制点
		varray<varray<double>> m_EdgePara;//边界离散点参数
		
		varray<varray<Vec4>> m_uCtrlPts;//拟合u方向后的控制点集
		varray<varray<double>> m_uPara;
		varray<varray<double>> m_vPara;

		//B样条曲面函数
		Vec4 BSF(Bsurface surface, double u, double v);

		//转置二维PtsVec
		template<typename _T>
		void TransPose(varray<varray<_T>>& PtsVec);

		/*Coons插值
		EndgCtrlPts:边界控制点（e0,e1,e2,e3顺序）
		coonsPatchCtrlpts:返回的曲面控制点
		*/
		void CoonsInterpolate(const varray<varray<Vec4>>& EndgCtrlPts, varray<varray<Vec4>>& coonsPatchCtrlpts);

		//找原点
		void FindOriginPts();

		//反转点集内点的顺序
		void Reverse(varray<Vec4>& pnts);

		//对齐边界
		void AligningEdge();

		//计算u,v方向向量
		void CalUVDir();

		//确定uvw坐标系对应的xyz坐标系,0=x,1=y,2=z
		void UVWtoXYZ();

		//按向量dir的方向从小到大排序，dir=0,1,2
		void Sorted(varray<Vec4>& ptsArr, int dir);

		//拟合u方向
		void FittingUdir();

		//拟合v方向
		void FittingVdir();

//////////////////////////////////////////密集散乱点拟合/////////////////////////////////////

		//边界Coons插值基面
		void InterpolateBaseSurface();

		//获取点集ptsArr内以点pts为中心，在a*b矩形框内的点
		void GetNearPts(const varray<Vec4>& ptsArr, Vec4 pts, double a, double b, varray<Vec4>& nearPts);

		//计算基面空间对应点
		void Segmentation(const varray<double>& uPara, const varray<double>& vPara, varray<varray<Vec4>>& ptsArr);

		//求解hardy's系数矩阵C
		//ptsArr:邻域内数据点
		//b:hardy插值调节常数
		void CalHardysC(const varray<Vec4>& ptsArr, double b, MatrixXd& C);

		//Hardy's双二次插值
		Vec4 HardysInterpolate(const Vec4 pts, const varray<Vec4>& ptsArr, const MatrixXd& C, const double b);

		//计算范围
		void CalExtreme(double extrm[4]);

		//插值网格点
		void InterpolatGridPts(int uNum, int vNum);


/////////////////////////////有序散乱点分组拟合///////////////////////////////////////////////

		//v方向井字8领域边界点判断
		bool IsVEdgePts(Vec4 pts, const varray<Vec4>& ptsArr);

		//判断pnt是否在点集ptsArr内,返回索引,-1为不在点集内
		int InArr(Vec4 pnt, const varray<Vec4>& ptsArr);

		//计算固定v后，u方向的点集，构造m_uPtsVec
		void CalUPtsVec();

		//v方向参数化
		void VParaPts();

		//计算拟合误差
		void CalSurFaceFittingErr();

	};

}

#endif

