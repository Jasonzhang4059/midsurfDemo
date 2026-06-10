#if !defined(_SPLINEVOLUME_H_INCLUDED)
#define _SPLINEVOLUME_H_INCLUDED

#include "SplineSurface.h"
#include"varray.h"
#include"XVec.h"
using namespace base;
namespace base
{
	//每个四边面片。
	enum surfacePosition{
		U0sfPos = 0,
		U1sfPos = 1,
		V0sfPos = 2,
		V1sfPos = 3,
		W0sfPos = 4,
		W1sfPos = 5
	};
	enum volumeRenderMode{
		iRenderBoundary_ControlPoints = 0,
		iRenderBoundary_Patch = 1,
		iRenderBoundary_WithISOElements = 2,
		iRenderBoundary_All = 3,

		iRenderVolume_ControlPoints = 4,
		iRenderVolume_TinyPatch = 5,
		iRenderVolume_WithISOElements = 6,
		iRenderVolume_All = 7,
        iRenderVolume_ISOWireFrame = 8
	};
#define SEGMENTNUM  60;
class VolumeVertex//体模型中的一个控制点所有信息
{
public:
	Vec4  m_pt;      //坐标点
	Vec4  m_oript;   //原始控制点，用于备份形体变性前的控制点。
	Vec4  m_matval;  //材料参数
	Vec4  m_displacement;  //在施加载荷后形成的位移。  分别表示ux，uy,uz
	Vec4  m_stress;    //改点外力载荷。     分别表示sigma(x),sigma(y),sigma(z).
	float m_fvonMissStressMat, m_fDisplaceMat;    //表示根据最大最小值得到的应力颜色值和位移颜色值，范围在[0,1].需要显示位移或者应力时，只需将相应的值赋予颜色变量即可。
public:
	VolumeVertex();
	VolumeVertex(Vec4& vt);
	VolumeVertex(Vec4& vt,Vec4& matt);
	float getVonmissStress(){return sqrt((m_stress.x-m_stress.y)*(m_stress.x-m_stress.y)/2 + (m_stress.y-m_stress.z)*(m_stress.y-m_stress.z)/2 + (m_stress.z-m_stress.x)*(m_stress.z-m_stress.x)/2);};
	~VolumeVertex();    
};
class SplineVolume: public SplineBase
{
public:
	SplineVolume(void);
	~SplineVolume(void);
public:
	int   m_uNum, m_uDegree, m_uRenderNum;
	int   m_vNum, m_vDegree, m_vRenderNum;
	int   m_wNum, m_wDegree, m_wRenderNum;
	bool  m_bHeterogeneous;//？？？
	varray<Vec4>			 m_CtrlPts;
	varray<VolumeVertex> m_vAllCtrlPts;  //控制点
	varray<VolumeVertex> m_vVolumePts;   //用于显示的节点。
	varray<int>  m_CtrlPtsIDinKKKMatrix;   //每个控制点在总纲矩阵中的位置。 如果是单片，默认就是0~m_uNum*m_vNum*m_wNum-1;如果是多片就需要重新编号。
	bool  m_bHasInterpolated;//？？？
	varray<double> m_uKnots;
	varray<double> m_vKnots;
	varray<double> m_wKnots;
	varray<SplineSurface> m_6boundarySurface;  //排序前后都是对应m_6PatchIdx，对应于拟合的surface,存储的是旋转后的曲面
	bool  m_bHasIGASolution;
	float m_maxJocbian, m_minJocbian;
	int   m_minorJocbianNum;
	double m_isovalAdd;

public:
	void  Clear();
	//显示函数
	//void  Render(bool vertshow,bool boundarysurface=false,bool volumedisplay = false);
	/*void  RenderbyMode(int iVolumeMode,int iSurfaceMode);
	void  RenderPatch(bool bBoundary,int iSurfaceMode);
	void  RenderCtrlPts(bool bOnlyBoudary);
	void  RenderIsopoints(bool bOnlyBoudary);
	void  RenderIsoCurves(bool bOnlyBoudary);
	void  RenderIsoSurfaces(bool bOnlyBoudary,float alpha);
	void  RenderTinyCube();*/

	void  CreateAllVolumeRenderPts(int uN, int vN, int wN);//计算现实点
	void  ReCreateAllVolumeRenderPts();//重新计算现实点
	void  InterpolateInnerControlPtsByBoundarysueface();//通过边界面插值内部控制点
	//evaluate operation
	void  GetAllControlPointsVec4(varray<Vec4>& allVecs);
	Vec4  GetPoint(float u, float v, float w, Vec4& matPt);
	Vec4  GetControlPoint(int ui, int vi, int wi);
	Vec4  GetControlPointMatvalue(int ui, int vi, int wi);
	int   GetControlPointIndex(int ui, int vi, int wi);
	bool  GetIJKIndex(int i, int& ui, int&vi, int& wi, int unum, int vnum, int wnum);
	bool  IsBoundaryPoint(int ui, int vi, int wi, int unum, int vnum, int wnum);
	int   GetRenderPointIndex(int ui, int vi, int wi);
	void  SetControlPtNum(int un, int vn, int wn);//设置控制点个数
	void  SetDegree(int ud, int vd, int wd);//设置阶数
	void  SetControlPoints(varray<Vec4>& vts);//设置控制点
	void  SetControlPoints(varray<Vec4>& vts, varray<Vec4>& mts);
	void  SetControlPoints(varray<Vec4>& vts, int udegree, int vdegree, int wdegree, int un, int vn, int wn);//相当于构造函数
	bool  JudgeNeedInterpolate();

	//获取边界面上面的点
	void  Get6SurfaceCtrlPtsFromPts(varray<varray<Vec4> >& surfacePts, int uvwmode);//0,1 for u direction surface, 2,3 for v direction surface,4,5 for w direction surface.
	//获取
	void  GetBoundBox(Vec4& leftbot, Vec4& rightTop, int istatus = 0); //0 for render points, 1 for control points.

	//平均值法
	//void  InterpolateAllControlPtsByMeanValue(bool isinitial=true);
	//void  LoopInterpolateAllControlPtsByMeanValue(bool isinitial=true);  //mean value coordinate.	                              
	int   GetIndexForVolumeMatrix(int iu, int iv, int iw, int su, int sv, int sw, int localidx, bool& boundaryflag);
	void  GetLocalScaleForPoint(double scale[], int ciu, int civ, int ciw, bool laplacian = false, bool isinitial = true);
	void  ModifyLocalIndexForCube(int hindex, int& mu, int& mv, int& mw);

	//离散拉普拉斯方程
	/*void  InterpolateallControlPtsByLaplacian();*/

	//孔斯法
	//the following method is referred to 【Farin G.."Discrete coons patches."】
	void  InterpolateAllControlPtsByCoons();  //by Gang Xu's method named discrete coons method.

	//衡量每个体控制点的网格质量
	float  GetJacobianValue(float u, float v, float w);
	float  GetFirstPartialDirivatives(float u, float v, float w, Vec4& pu, Vec4& pv, Vec4& pw);
	void   TestMeshQuanity(int segmentNum = 60/*SEGMENTNUM*/);

	void   SaveVolume();

	//等几何分析相关
	void   SetDisplaceAndLoadvector(int ctrlPtid, Vec4& disVec, Vec4& loadVec);

	void SetVol(const int uDegree, const int vDegree, const int wDegree, const int uNum, const int vNum, const int wNum,
		const varray<double>& uKnots, const varray<double>& vKnots, const varray<double>& wKnots);

	//(u,v,w)对应的体上的点
	Vec3 GetVolPoint(const double u, const double v, const double w)const;


	//计算四边形面片显示数据
	//num:该方向采样点数量
	//quads:面片数据
	//lines:等参线数据
	threadParamSplineVOL CalQuads(const int Unum, const int Vnum, const int Wnum)const;

	//根据控制点三维序号计算一维序号
	int CtrlPtsIdx(const int uIdx, const int vIdx, const int wIdx);

	//控制点排序转换为U-V-W
	void OrderCtrlPts();

	//控制点排序转换为U-V-W
	void OrderCtrlPts(SplineVolume& vol);

	//控制点排序转换为U-V-W
	void OrderCtrlPts(SplineVolume& vol,int mode);

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
	void CreateSweepSplineVolume(const Spline& pathT, const SplineSurface& nurbsSF, const int K);

	/*平移扫描生成Nurbs体模型，截面不垂直于路径
	pathT：扫描路径
	nurbsSF：起始截面*/ 
	void CreateTransSweepSplineVolume(const Spline& pathT, const SplineSurface& nurbsSF);

	//放样
	//path:路径
	//surfaces:放样截面
	void LoftingSplineVolume(const Spline& path, const varray<SplineSurface>& surfaces);
	//提取单个边界面
	//dir:1->6表示方向
	SplineSurface GetSingleSurfaces(int dir) const;
	//放样
	//path:路径
	//surfaces0:路径起点放样截面
	//surfaces1:路径终点放样截面
	void LoftingSplineVolume(const Spline& path, const SplineSurface& surfaces0, const SplineSurface& surfaces1);
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
		varray<varray<Vec3>>& quads, varray<varray<Vec3>>& lines)const;

	/*扫描时的截面实例位置
	pathT：扫描路径
	K：截面实例数量
	pos：截面实例位置
	NewKnots：新节点矢量*/
	void InsLocation(const Spline & pathT, int K, varray<double> & pos);

	/*计算扫描路径上的局部坐标系
	pathT：扫描路径
	pos：截面实例位置
	TranMat：pos处的局部坐标系*/
	void LocalCoordinates(const Spline & pathT, const varray<double> & pos,
		varray<varray<Vec4>> & TranMat);



	//体COONS插值
	void VolCoonsInterpolate(const varray<Spline>& EdgeLines);

	/*沿扫描路径对截面进行变换
	nurbsSF：截面控制点
	TranMat：变换矩阵
	OriCoordinates:截面的局部坐标系
	allNurbsSF：得到的所有截面实例控制点
	*/
	void MatrixTran(const varray<varray<Vec4>> & nurbsSF, const varray<varray<Vec4>>& TranMat,
		varray<varray<varray<Vec4>>>& allNurbsSF);

	//扫描生成Nurbs体
	void SweepSurface(const varray<varray<varray<Vec4>>>& allNurbsSF,
		const varray<varray<varray<double>>>& SFw, const varray<double>& pos);

	//取最高次
	void MaxDegree(const varray<SplineSurface>& surfaces, int& uDegree, int& vDegree);

	//节点矢量并集
	void KnotsUnify(const varray<SplineSurface>& surfaces, varray<double>& NewUKnots, varray<double>& NewVKnots);
};
}
#endif