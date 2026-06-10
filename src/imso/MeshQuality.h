#pragma once
#pragma once
#include"varray.h"
#include"XVec.h"
#include "SplineSurface.h"
#include "SplineVolume.h"
using namespace base;
//质量评价方法
//来源 The Verdict Geometric Quality Library
//其中大部分方法可以在paraview中实现
//本程序希望利用这些指标进行综合评价
//因为单一指标评价容易产生片面的结果
//参考Huang jin： Evaluating Hex-mesh Quality Metrics via Correlation Analysis


class BaseInfo {
public:
	static BaseInfo* getInstancePtr() {
		if (instance == nullptr)
			instance = new BaseInfo();
		return instance;
	}
	BaseInfo(const BaseInfo&) = delete;
	BaseInfo& operator = (const BaseInfo&) = delete;
	varray<Vec3> pts;				//四个角点
	varray<Vec3> Li;				//单元边坐标
	varray<double> mLi;				//单元边长度
	double Lmin;					//最小边
	double Lmax;					//最大边
	varray<Vec3> Di;				//对角边
	varray<double> mDi;				//对角边长度
	double Dmax;					//最大对角边
	double Dmin;					//最小对角边
	varray<Vec3> Xi;				//坐标轴
	varray<Vec3> Ni;				//法向量
	varray<Vec3> ni;				//单位法向量
	Vec3 Nc;						//中心向量
	Vec3 nc;						//中心单位向量
	varray<double> ai;				//部分面积
	varray<varray<Vec3>> Ai;		//3x3向量
	varray<double> A2;				//Ai模的平方 
	varray<double> adjA2;			
	varray<varray<Vec3>> Ai_;
	varray<double> alphi;
	varray<double> alphi_;
	///////////////四边形指标计算
	double getArea();					//计算面积
	double getAspectRatio();			//计算宽高比
	double getCondition();
	double getDistortion();				//计算扭曲度
	double getEdgeRatio();				//计算边率
	double getJacobian();				//获得雅可比值
	void InitSufInfo(const varray<Vec4>& pts);
	void InitVolInfo(const varray<Vec4>& pts);
	void clearData();
	////////////四边形指标
	double Area;					//面积
	double AspectRatio;				//纵横比
	double Condition;
	double Distortion;				//扭曲度
	double EdgeRatio;				//边率
	double Jacobian;				//雅可比
	////////////六面体指标
	double Diagonal;			//对角线[0.65,1]
	double Volume;				//体积[0,+OO]
	double Stretch;				//伸展[0.25,1]
	double ScaledJacobian;		//边界雅可比[0.5,1]
	double Oddy;				//雅可比的另一种度量[0,0.5]
	////////////六面体指标计算
	double getDiagonal();
	double getVolume();
	double getStretch();
	double getScaledJacobian();

private:
	BaseInfo() {}

	static BaseInfo* instance;

};

//用于检测模型质量的类
class BaseQuality
{
public:
	BaseQuality(int n);
	BaseQuality();
	~BaseQuality();
	varray<varray<varray<Vec4>>> vecs;		//模型上的等参单元	
	void virtual calQuality() = 0;			//模型质量评价
	void virtual getSamplePts() = 0;		//获得模型等参单元
	int segmentNum;					//默认的取样次数为10
};

class SufQuality : public BaseQuality {
public:
	SufQuality(varray<SplineSurface>& sf, int n = 5);
	~SufQuality();
	/////////////////////////////////
	//用户创建类后只需调用这个函数即可//
	/////////////////////////////////
	void calQuality();
private:
	void getSamplePts();
	bool getBaseInfo(const varray<Vec4>& pt);
	bool compareData();
	void clearAll();
public:
	double minArea = INT_MAX;		//接受域大于0
	double minJacobian = INT_MAX; //接受域大于0
	double minEdgeRatio = INT_MAX;//>=1
	double maxEdgeRatio = INT_MIN;//<=1.3
	double minCondition = INT_MAX;//>=1
	double maxCondition = INT_MIN;//<=4
	double minAspectRatio = INT_MAX;//>=1
	double maxAspectRatio = INT_MIN;//<=1.3
	double minDistortion = INT_MAX;//>=0.5
	double maxDistortion = INT_MIN;//<=1


private:
	varray<SplineSurface> sf;		//输入的带计算曲面
};

class VolQuality : public BaseQuality {
public:
	VolQuality(varray<SplineVolume>& sv, int n=10);
	~VolQuality();
	void calQuality();
	void getSamplePts();
	bool getBaseInfo(const varray<Vec4>& pt);
	bool compareData();
	void clearAll();

	double minDiagonal = INT_MAX;			//对角线[0.65,1]
	double maxDiagonal = INT_MIN;
	double minVolume = INT_MAX;				//体积[0,+OO] 
	double maxVolume = INT_MIN;
	double minStretch = INT_MAX;				//伸展[0.25,1]
	double maxStretch = INT_MIN;
	double minScaledJacobian = INT_MAX;		//边界雅可比[0,1]
	double maxScaledJacobian = INT_MIN;		


private:
	varray<SplineVolume> sv;



};


