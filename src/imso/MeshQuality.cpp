#include "MeshQuality.h"
#include <iostream>

BaseInfo* BaseInfo::instance = nullptr;

void BaseInfo::InitSufInfo(const varray<Vec4>& pts) {
	if (pts.size() != 4) std::abort();
	Vec3 p0 = pts[0];
	Vec3 p1 = pts[1];
	Vec3 p2 = pts[2];
	Vec3 p3 = pts[3];

	Li.push_back(p1 - p0);
	Li.push_back(p2 - p1);
	Li.push_back(p3 - p2);
	Li.push_back(p0 - p3);

	mLi.push_back(Li[0].Magnitude());
	mLi.push_back(Li[1].Magnitude());
	mLi.push_back(Li[2].Magnitude());
	mLi.push_back(Li[3].Magnitude());
	Lmin = INT_MAX, Lmax = INT_MIN;
	for (int i = 0; i < mLi.size(); i++) {
		if (mLi[i] < Lmin)
			Lmin = mLi[i];
		if (mLi[i] > Lmax)
			Lmax = mLi[i];
	}
	Di.push_back(p2 - p0);
	Di.push_back(p3 - p1);
	mDi.push_back(Di[0].Magnitude());
	mDi.push_back(Di[1].Magnitude());
	Dmin = INT_MAX, Dmax = INT_MIN;
	for (int i = 0; i < mDi.size(); i++) {
		if (mDi[i] < Dmin)
			Dmin = mDi[i];
		if (mDi[i] > Dmax)
			Dmax = mDi[i];
	}
	Xi.push_back(p1 - p0 + p2 - p3);
	Xi.push_back(p2 - p1 + p3 - p0);
	Ni.push_back(Li[3].Cross(Li[0]));
	Ni.push_back(Li[0].Cross(Li[1]));
	Ni.push_back(Li[1].Cross(Li[2]));
	Ni.push_back(Li[2].Cross(Li[3]));

	Nc = Xi[0].Cross(Xi[1]);
	nc = Nc / Nc.Magnitude();

	for (int i = 0; i < 4; i++) {
		ai.push_back(nc.Dot(Ni[i]));
	}

}

void BaseInfo::clearData()
{
	pts.clear();			//四个角点
	Li.clear();				//单元边坐标
	mLi.clear();			//单元边长度
	Lmin = INT_MAX;			//最小边
	Lmax = INT_MIN;			//最大边
	Di.clear();				//对角边
	mDi.clear();			//对角边长度
	Dmax = INT_MIN;			//最大对角边
	Dmin = INT_MAX;			//最小对角边
	Xi.clear();				//坐标轴
	Ni.clear();				//法向量
	ni.clear();				//单位法向量			
	ai.clear();				//部分面积
	Ai.clear();				//3x3向量
	A2.clear();				//Ai模的平方 
	adjA2.clear();
	Ai_.clear();
	alphi.clear();
	alphi_.clear();
	
}

double BaseInfo::getDiagonal() {
	Diagonal = Dmin / Dmax;
	return Diagonal;
}

double BaseInfo::getVolume() {
	Volume = alphi[8] / 64;
	return Volume;
}

double BaseInfo::getStretch() {
	Stretch = sqrt(3)*Lmin / Dmax;
	return Stretch;
}

double BaseInfo::getScaledJacobian() {
	double res = INT_MAX;
	for (int i = 0; i < alphi_.size(); i++) {
		res = min(res, alphi_[i]);
	}
	ScaledJacobian = res;
	return res;
}

BaseQuality::BaseQuality(int n) : segmentNum(n)
{
	

}

BaseQuality::BaseQuality()
{
	

}

BaseQuality::~BaseQuality()
{
	
}



SufQuality::SufQuality(varray <SplineSurface>& sf, int n)
	: sf(sf), BaseQuality(n)
{
	//calQuality();
}

SufQuality::~SufQuality() {}

void SufQuality::calQuality()
{
	getSamplePts();
	int idx = 0;
	for (auto& pts : vecs) {
		for (auto& pt : pts) {
			if (getBaseInfo(pt) == false) {
				cout << "idx = " << idx << " error" << endl;
				break;
			}
		}
		clearAll();
		idx++;
	}
}
void SufQuality::clearAll() {
	minArea = INT_MAX;		//接受域大于0
	minJacobian = INT_MAX; //接受域大于0
	minEdgeRatio = INT_MAX;//>=1
	maxEdgeRatio = INT_MIN;//<=1.3
	minCondition = INT_MAX;//>=1
	maxCondition = INT_MIN;//<=4
	minAspectRatio = INT_MAX;//>=1
	maxAspectRatio = INT_MIN;//<=1.3
	minDistortion = INT_MAX;//>=0.5
	maxDistortion = INT_MIN;//<=1
}
void SufQuality::getSamplePts()
{
	varray<varray<Vec4>> pts;
	varray<Vec4> pt;
	for (auto& suf : sf) {
		for (int i = 0; i < segmentNum; i++) {
			for (int j = 0; j < segmentNum; j++) {
				auto p = suf.GetSurFacePoint(i*1.0 / segmentNum, j*1.0 / segmentNum);
				pt.push_back(p);
				p = suf.GetSurFacePoint((i + 1)*1.0 / segmentNum, j*1.0 / segmentNum);
				pt.push_back(p);
				p = suf.GetSurFacePoint((i + 1)*1.0 / segmentNum, (j + 1)*1.0 / segmentNum);
				pt.push_back(p);
				p = suf.GetSurFacePoint(i*1.0 / segmentNum, (j + 1)*1.0 / segmentNum);
				pt.push_back(p);
				pts.push_back(pt);
				pt.clear();
			}
		}
		vecs.push_back(pts);
		pts.clear();
	}

}

bool SufQuality::getBaseInfo(const varray<Vec4>& vec) {
	BaseInfo* b = BaseInfo::getInstancePtr();
	b->getInstancePtr();
	b->InitSufInfo(vec);
	b->getArea();
	b->getAspectRatio();
	b->getCondition();
	b->getDistortion();
	b->getEdgeRatio();
	b->getJacobian();
	bool r = compareData();
	b->clearData();
	return r;
}


bool SufQuality::compareData() {
	BaseInfo* b = BaseInfo::getInstancePtr();
	b->getInstancePtr();
	bool r = true;
	this->minArea = min(this->minArea, b->Area);
	if (minArea < 0) {
		cout << "minArea <= 0" << "," << minArea<<endl;
		r = false;
	}
	this->minJacobian = min(this->minJacobian, b->Jacobian);
	if (minJacobian < 0) {
		cout << "minJacobian <= 0" <<","<<minJacobian<< endl;
		r = false;
	}
	this->minEdgeRatio = min(this->minEdgeRatio, b->EdgeRatio);
	if (minEdgeRatio < 0.99) {
		cout << "minEdgeRatio < 1" <<","<<minEdgeRatio<< endl;
		r = false;
	}
	this->maxEdgeRatio = max(this->maxEdgeRatio, b->EdgeRatio);
	if (maxEdgeRatio > 1.5) {
		cout << "maxEdgeRatio > 1.3" <<","<<maxEdgeRatio<< endl;
		r = false;
	}
	this->minCondition = min(this->minCondition, b->Condition);
	if (minCondition < 0.99) {
		cout << "minCondition < 1" << "," <<minCondition<<endl;
		r = false;
	}
	this->maxCondition = min(this->maxCondition, b->Condition);
	if (maxCondition > 4) {
		cout << "maxCondition > 4" << ","<<maxCondition<< endl;
		r = false;
	}
	this->minAspectRatio = min(this->minAspectRatio, b->AspectRatio);
	if (minAspectRatio < 0.99) {
		cout << "minAspectRatio < 1" <<","<<minAspectRatio<< endl;
		r = false;
	}
	this->maxAspectRatio = max(this->maxAspectRatio, b->AspectRatio);
	if (maxAspectRatio > 1.5) {
		cout << "maxAspectRatio > 1.3" <<","<<maxAspectRatio<< endl;
		r = false;
	}
	this->minDistortion = min(this->minDistortion, b->Distortion);
	if (minDistortion < 0.5) {
		cout << "minDistorition < 0.5" <<","<<minDistortion<< endl;
		r = false;
	}
	this->maxDistortion = max(this->maxDistortion, b->Distortion);
	if (maxDistortion > 1.1) {
		cout << "maxDistorition > 1" << "," <<maxDistortion<<endl;
		r = false;
	}
	return r;
}

VolQuality::VolQuality(varray<SplineVolume>& sv, int n) :sv(sv) , BaseQuality(n)
{

}

VolQuality::~VolQuality() {

}

void VolQuality::calQuality() {
	getSamplePts();
	int idx = 0;
	for (auto& pts : vecs) {
		for (auto& pt : pts) {
			if (getBaseInfo(pt) == false) {

				cout << "idx = " << idx << " error" << endl;
				break;
			}
		}
		clearAll();
		idx++;
	}
}

void VolQuality::getSamplePts() {
	varray<varray<Vec4>> pts;
	varray<Vec4> pt;

	for (auto& vol : sv) {
		for (int i = 0; i < segmentNum; i++) {
			for (int j = 0; j < segmentNum; j++) {
				for (int k = 0; k < segmentNum; k++) {
					auto p = vol.GetVolPoint(i*1.0 / segmentNum, j*1.0 / segmentNum, k*1.0 / segmentNum);
					pt.push_back(p);
					p = vol.GetVolPoint((i+1)*1.0 / segmentNum, j*1.0 / segmentNum, k*1.0 / segmentNum);
					pt.push_back(p);
					p = vol.GetVolPoint((i+1)*1.0 / segmentNum, (j+1)*1.0 / segmentNum, k*1.0 / segmentNum);
					pt.push_back(p);
					p = vol.GetVolPoint(i*1.0 / segmentNum, (j+1)*1.0 / segmentNum, k*1.0 / segmentNum);
					pt.push_back(p);
					p = vol.GetVolPoint(i*1.0 / segmentNum, j*1.0 / segmentNum, (k+1)*1.0 / segmentNum);
					pt.push_back(p);
					p = vol.GetVolPoint((i + 1)*1.0 / segmentNum, j*1.0 / segmentNum, (k + 1)*1.0 / segmentNum);
					pt.push_back(p);
					p = vol.GetVolPoint((i + 1)*1.0 / segmentNum, (j + 1)*1.0 / segmentNum, (k + 1)*1.0 / segmentNum);
					pt.push_back(p);
					p = vol.GetVolPoint(i*1.0 / segmentNum, (j + 1)*1.0 / segmentNum, (k + 1)*1.0 / segmentNum);
					pt.push_back(p);
					pts.push_back(pt);
					pt.clear();
				}
			}
		}
		vecs.push_back(pts);
		pts.clear();
	}
	

	/*for (auto& suf : sf) {
		for (int i = 0; i < segmentNum; i++) {
			for (int j = 0; j < segmentNum; j++) {
				auto p = suf.GetSurFacePoint(i*1.0 / segmentNum, j*1.0 / segmentNum);
				pt.push_back(p);
				p = suf.GetSurFacePoint((i + 1)*1.0 / segmentNum, j*1.0 / segmentNum);
				pt.push_back(p);
				p = suf.GetSurFacePoint((i + 1)*1.0 / segmentNum, (j + 1)*1.0 / segmentNum);
				pt.push_back(p);
				p = suf.GetSurFacePoint(i*1.0 / segmentNum, (j + 1)*1.0 / segmentNum);
				pt.push_back(p);
				pts.push_back(pt);
				pt.clear();
			}
		}
		vecs.push_back(pts);
		pts.clear();
	}*/
}


//接受域[0,正无穷]
double BaseInfo::getArea()
{
	double ave = 0;
	for (int i = 0; i < ai.size(); i++) {
		ave += ai[i];
	}
	Area = ave / ai.size();
	return Area;
}

//接受域[1,1.3]
double BaseInfo::getAspectRatio()
{
	double A = getArea();
	double lsum = 0;
	for (int i = 0; i < Li.size(); i++) {
		lsum += mLi[i];
	}
	AspectRatio = Lmax * lsum / (4 * A);

	return AspectRatio;

}

//接收域[1,4]
double BaseInfo::getCondition()
{
	Condition = max((mLi[0] * mLi[0] + mLi[3] * mLi[3]) / ai[0], (mLi[0] * mLi[0] + mLi[1] * mLi[1]) / ai[1],
		(mLi[1] * mLi[1] + mLi[2] * mLi[2]) / ai[2], (mLi[2] * mLi[2] + mLi[3] * mLi[3]) / ai[3]);
	return Condition;
}

//接受域[0,正无穷]
double BaseInfo::getJacobian()
{
	Jacobian = min(ai[0], ai[1], ai[2], ai[3]);
	return Jacobian;
}

//接收域[0.5,1]
double BaseInfo::getDistortion()
{
	double J = getJacobian();
	double A = getArea();
	Distortion = J / A;
	return Distortion;
}

//接收域[1,1.3]
double BaseInfo::getEdgeRatio()
{
	EdgeRatio = Lmax / Lmin;
	return EdgeRatio;
}

void BaseInfo::InitVolInfo(const varray<Vec4>& pts) {
	clearData();
	if (pts.size() != 8) std::abort();
	Li.push_back(pts[1] - pts[0]);
	Li.push_back(pts[2] - pts[1]);
	Li.push_back(pts[3] - pts[2]);
	Li.push_back(pts[3] - pts[0]);
	Li.push_back(pts[4] - pts[0]);
	Li.push_back(pts[5] - pts[1]);
	Li.push_back(pts[6] - pts[2]);
	Li.push_back(pts[7] - pts[3]);
	Li.push_back(pts[5] - pts[4]);
	Li.push_back(pts[6] - pts[5]);
	Li.push_back(pts[7] - pts[6]);
	Li.push_back(pts[7] - pts[4]);
	Lmin = INT_MAX, Lmax = INT_MIN;
	for (int i = 0; i < Li.size(); i++) {
		double t = Li[i].Magnitude();
		Lmin = min(t, Lmin);
		Lmax = min(t, Lmax);
		mLi.push_back(t);
	}
	Di.push_back(pts[6] - pts[0]);
	Di.push_back(pts[7] - pts[1]);
	Di.push_back(pts[4] - pts[2]);
	Di.push_back(pts[5] - pts[3]);
	Dmin = min(Di[0].Magnitude(), Di[1].Magnitude(), Di[2].Magnitude(), Di[3].Magnitude());
	Dmax = max(Di[0].Magnitude(), Di[1].Magnitude(), Di[2].Magnitude(), Di[3].Magnitude());
	Xi.push_back(pts[1] - pts[0] + pts[2] - pts[3] + pts[5] - pts[4] + pts[6] - pts[7]);
	Xi.push_back(pts[3] - pts[0] + pts[2] - pts[1] + pts[7] - pts[4] + pts[6] - pts[5]);
	Xi.push_back(pts[4] - pts[0] + pts[5] - pts[1] + pts[6] - pts[2] + pts[7] - pts[3]);
	varray<Vec3> t;
	t.push_back(Li[0]);
	t.push_back(Li[3]);
	t.push_back(Li[4]);
	Ai.push_back(t);
	t.clear();
	t.push_back(Li[1]);
	t.push_back(-Li[0]);
	t.push_back(Li[5]);
	Ai.push_back(t);
	t.clear();
	t.push_back(Li[2]);
	t.push_back(-Li[1]);
	t.push_back(Li[6]);
	Ai.push_back(t);
	t.clear();
	t.push_back(-Li[3]);
	t.push_back(-Li[2]);
	t.push_back(Li[7]);
	Ai.push_back(t);
	t.clear();
	t.push_back(Li[11]);
	t.push_back(Li[8]);
	t.push_back(-Li[4]);
	Ai.push_back(t);
	t.clear();
	t.push_back(-Li[8]);
	t.push_back(Li[9]);
	t.push_back(-Li[5]);
	Ai.push_back(t);
	t.clear();
	t.push_back(-Li[9]);
	t.push_back(Li[10]);
	t.push_back(-Li[6]);
	Ai.push_back(t);
	t.clear();
	t.push_back(-Li[10]);
	t.push_back(-Li[11]);
	t.push_back(-Li[7]);
	Ai.push_back(t);
	t.clear();
	t.push_back(Xi[0]);
	t.push_back(Xi[1]);
	t.push_back(Xi[2]);
	Ai.push_back(t);
	t.clear();
	for (int i = 0; i < Ai.size(); i++) {
		auto a1 = Ai[i][0];
		auto a2 = Ai[i][1];
		auto a3 = Ai[i][2];
		A2.push_back(a1.Magnitude()*a1.Magnitude() + a2.Magnitude()*a2.Magnitude() + a3.Magnitude()*a3.Magnitude());
		double l1 = a1.Cross(a2).Magnitude();
		double l2 = a2.Cross(a3).Magnitude();
		double l3 = a3.Cross(a1).Magnitude();
		adjA2.push_back(l1*l1 + l2 * l2 + l3 * l3);
		varray<Vec3> t;
		auto a1_ = a1.Normalize();
		auto a2_ = a2.Normalize();
		auto a3_ = a3.Normalize();
		t.push_back(a1_);
		t.push_back(a2_);
		t.push_back(a3_);
		alphi.push_back(a2.Cross(a3).Dot(a1));
		alphi_.push_back(a2_.Cross(a3_).Dot(a1_));
	}
}

bool VolQuality::getBaseInfo(const varray<Vec4>& pt) {
	BaseInfo* b = BaseInfo::getInstancePtr();
	b->getInstancePtr();
	b->InitVolInfo(pt);
	b->getDiagonal();
	b->getVolume();
	b->getStretch();
	b->getScaledJacobian();
	bool r = compareData();
	b->clearData();
	return r;
}

bool VolQuality::compareData() {
	BaseInfo* b = BaseInfo::getInstancePtr();
	b->getInstancePtr();
	bool r = true;
	minDiagonal = min(b->Diagonal, minDiagonal);
	if (minDiagonal < 0.65) {
		cout << "minDiagonal < 0.65" << "," << minDiagonal << endl;
		r = false;
	}
	maxDiagonal = max(b->Diagonal, maxDiagonal);
	if (maxDiagonal > 1) {
		cout << "maxDiagonal > 1" << "," << maxDiagonal << endl;
		r = false;
	}
	minVolume = min(b->Volume, minVolume);
	maxVolume = max(b->Volume, maxVolume);
	if (minVolume*maxVolume < 0) {
		cout << "minVolume < 0" << "," << minVolume << endl;
		r = false;
	}

	minStretch = min(minStretch, b->Stretch);
	if (minStretch < 0) {
		cout << "minStretch < 0" << "," << minStretch << endl;
		r = false;
	}
	maxStretch = max(maxStretch, b->Stretch);
	if (maxStretch > 1) {
		cout << "maxStretch > 1" << "," << maxStretch << endl;
		r = false;
	}
	minScaledJacobian = min(minScaledJacobian, b->ScaledJacobian);
	maxScaledJacobian = max(maxScaledJacobian, b->ScaledJacobian);
	if (maxScaledJacobian*minScaledJacobian < 0) {
		cout << "minScaledJacobian < 0" << "," << minScaledJacobian << endl;
		r = false;
	}


	return r;
}

void VolQuality::clearAll() {
	minDiagonal = INT_MAX;			//对角线[0.65,1]
	maxDiagonal = INT_MIN;
	minVolume = INT_MAX;				//体积[0,+OO]
	minStretch = INT_MAX;				//伸展[0.25,1]
	maxStretch = INT_MIN;
	minScaledJacobian = INT_MAX;		//边界雅可比[0,1]
	maxScaledJacobian = INT_MIN;
}