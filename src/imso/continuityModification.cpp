#include"continuityModification.h"
#include"occIgesTrans.h"
#include <iostream>
#include <chrono>
#include <array>
#define DEBUG 0
using VertexType = continuityModification::NurbsVolVertexType;

continuityModification::continuityModification() :
	m_3d_conn(0), m_plannar_conn(0), m_composite_conn(0)
{
	m_filename = "model2-mid_G0";
	m_mode = 1;//G1=0，C1=1
	m_DegreeElevate = 8;//升阶次数
	m_rwg.ReadSplineVolume("./g1/" + m_filename + ".txt", m_SV);//读取想要改变的模型路径
	OccIgesTrans octrans;
	octrans.outputIges(m_SV, "./iges/init_" + m_filename + ".iges");
	for (auto &sv : m_SV)
	{	
		sv.DegreeElevate(m_DegreeElevate, m_DegreeElevate, m_DegreeElevate);//模型升阶  

		for (auto &j : sv.m_CtrlPts) {
			j.w = 1;//不将所有权值置为1会出现G1不连续问题，并且论文中不考虑权值
		}
		//sv.OrderCtrlPts(sv);
	}
	
}

continuityModification::continuityModification(string path)
	:m_3d_conn(0), m_plannar_conn(0), m_composite_conn(0)
{
	m_filename = "model";
	m_mode = 1;//G1=0，C1=1
	m_DegreeElevate = 8;//升阶次数
	m_rwg.ReadSplineVolume(path, m_SV);//读取想要改变的模型路径
	OccIgesTrans octrans;
	octrans.outputIges(m_SV, "./iges/init_" + m_filename + ".iges");
	for (auto& sv : m_SV)
	{
		sv.DegreeElevate(m_DegreeElevate, m_DegreeElevate, m_DegreeElevate);//模型升阶  

		for (auto& j : sv.m_CtrlPts) {
			j.w = 1;//不将所有权值置为1会出现G1不连续问题，并且论文中不考虑权值
		}
		//sv.OrderCtrlPts(sv);
	}
}

void continuityModification::adjustVolContinuity()
{
    if (m_SV.size()) 
	{
		auto start = std::chrono::high_resolution_clock::now();
        calComVerVol();//共角点的体的关系计算
        fixPoint();//共角点数量大于2则固定附近控制点
		//modifyCfltCptArrountVolver();//共角点周围的冲突点与边界点
		modifyBoundryCfltPts();
		createConflictArr();//创建冲突点链表并调整三个方向的冲突点
        ModifyNeighborContinuity();//调整剩余内部控制点
		auto end = std::chrono::high_resolution_clock::now();
		auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);

		std::cout << "耗时: " << duration.count() << " 毫秒" << std::endl;

        saveModel();//保存模型
		cout <<  " 立体" << m_3d_conn << " 对面" << m_composite_conn << "平面" << m_plannar_conn <<	endl;
    }
}

int continuityModification::GetNurbsVolVertexIndex(const SplineVolume& sv, const NurbsVolVertexType &verType)
{
	const int u = sv.m_uNum;
	const int v = sv.m_vNum;
	const int w = sv.m_wNum;

	switch (verType)
	{
		// Front-Bottom-Left (u=0, v=0, w=0)
	case NurbsVolVertexType::FBL:
		return 0;

		// Front-Bottom-Right (u=last, v=0, w=0)
	case NurbsVolVertexType::FBR:
		return u - 1;

		// Front-Top-Left (u=0, v=last, w=0)
	case NurbsVolVertexType::FTL:
		return u * v * (w - 1);

		// Front-Top-Right (u=last, v=last, w=0)
	case NurbsVolVertexType::FTR:
		return u * v * (w - 1) + u - 1;

		// Back-Bottom-Left (u=0, v=0, w=last)
	case NurbsVolVertexType::BBL:
		return u * (v - 1);

		// Back-Bottom-Right (u=last, v=0, w=last)
	case NurbsVolVertexType::BBR:
		return u * v - 1;

		// Back-Top-Left (u=0, v=last, w=last)
	case NurbsVolVertexType::BTL:
		return u * v * (w - 1) + u * (v - 1);

		// Back-Top-Right (u=last, v=last, w=last)
	case NurbsVolVertexType::BTR:
		return u * v * w - 1;

	default:
		// 无效的顶点类型，返回-1表示错误
		assert(false && "Invalid NurbsVolVertexType");
	}
	return -1;
}
Vec4& continuityModification::GetNurbsVolVertex(SplineVolume& sv, const NurbsVolVertexType &verType)
{
	const int u = sv.m_uNum;
	const int v = sv.m_vNum;
	const int w = sv.m_wNum;

	switch (verType)
	{
		// Front-Bottom-Left (u=0, v=0, w=0)
	case NurbsVolVertexType::FBL:
		return sv.m_CtrlPts[0];

		// Front-Bottom-Right (u=last, v=0, w=0)
	case NurbsVolVertexType::FBR:
		return sv.m_CtrlPts[u - 1];

		// Front-Top-Left (u=0, v=last, w=0)
	case NurbsVolVertexType::FTL:
		return sv.m_CtrlPts[u * v * (w - 1)];

		// Front-Top-Right (u=last, v=last, w=0)
	case NurbsVolVertexType::FTR:
		return sv.m_CtrlPts[u * v * (w - 1) + u - 1];

		// Back-Bottom-Left (u=0, v=0, w=last)
	case NurbsVolVertexType::BBL:
		return sv.m_CtrlPts[u * (v - 1)];

		// Back-Bottom-Right (u=last, v=0, w=last)
	case NurbsVolVertexType::BBR:
		return sv.m_CtrlPts[u * v - 1];

		// Back-Top-Left (u=0, v=last, w=last)
	case NurbsVolVertexType::BTL:
		return sv.m_CtrlPts[u * v * (w - 1) + u * (v - 1)];

		// Back-Top-Right (u=last, v=last, w=last)
	case NurbsVolVertexType::BTR:
		return sv.m_CtrlPts[u * v * w - 1];

	default:
		break;
	}
	return m_nullVertex;
}

const VertexType continuityModification::GetNurbsVolVertexType(const SplineVolume& sv, const Vec4 &vertex)
{
	const int u = sv.m_uNum;
	const int v = sv.m_vNum;
	const int w = sv.m_wNum;

	if (vertex == sv.m_CtrlPts[0]) return NurbsVolVertexType::FBL;
	if (vertex == sv.m_CtrlPts[u - 1]) return NurbsVolVertexType::FBR;
	if (vertex == sv.m_CtrlPts[u * (v - 1)]) return NurbsVolVertexType::BBL;
	if (vertex == sv.m_CtrlPts[u * v - 1]) return NurbsVolVertexType::BBR;
	if (vertex == sv.m_CtrlPts[u * v * (w - 1)]) return NurbsVolVertexType::FTL;
	if (vertex == sv.m_CtrlPts[u * v * (w - 1) + u - 1]) return NurbsVolVertexType::FTR;
	if (vertex == sv.m_CtrlPts[u * v * (w - 1) + u * (v - 1)]) return NurbsVolVertexType::BTL;
	if (vertex == sv.m_CtrlPts[u * v * w - 1]) return NurbsVolVertexType::BTR;

#if DEBUG == 1
	vector<Vec4> p;
	for (auto i : sv.m_CtrlPts)
	{
		p.push_back(i);
	}

	assert(false && "type : NurbsVolVertexType::UNKNOWN");
#endif

	return NurbsVolVertexType::UNKNOWN;
}

void continuityModification::GetVertexDeltas(const SplineVolume& sv,
	const NurbsVolVertexType &verType,
	int& deltaU,
	int& deltaV,
	int& deltaW)
{
	// 不需要使用sv参数，但保留以保持接口一致性
	(void)sv; // 避免未使用参数警告

	switch (verType)
	{
	case NurbsVolVertexType::FBL:
		deltaU = 1;
		deltaV = 1;
		deltaW = 1;
		break;

	case NurbsVolVertexType::FBR:
		deltaU = -1;
		deltaV = 1;
		deltaW = 1;
		break;

	case NurbsVolVertexType::FTL:
		deltaU = 1;
		deltaV = 1;
		deltaW = -1;
		break;

	case NurbsVolVertexType::FTR:
		deltaU = -1;
		deltaV = 1;
		deltaW = -1;
		break;

	case NurbsVolVertexType::BBL:
		deltaU = 1;
		deltaV = -1;
		deltaW = 1;
		break;

	case NurbsVolVertexType::BBR:
		deltaU = -1;
		deltaV = -1;
		deltaW = 1;
		break;

	case NurbsVolVertexType::BTL:
		deltaU = 1;
		deltaV = -1;
		deltaW = -1;
		break;

	case NurbsVolVertexType::BTR:
		deltaU = -1;
		deltaV = -1;
		deltaW = -1;
		break;

	default:
#if DEBUG ==1
		assert(false && "Invalid vertex type");
#endif

		deltaU = 0;
		deltaV = 0;
		deltaW = 0;
		break;
	}
}

bool continuityModification::CheckNurbsIndexValid(int u, int v, int w, int index)
{
	const int totalPoints = u * v * w;
	if (index == 168)
	{
		int x = 0;
	}
	if (index < 0 || index >= totalPoints) {
		assert(false && "Error index in CheckNurbsIndexValid");
		return false;
	}

	return true;
}

bool continuityModification::FindNurbsVolInnerCfltPnt(SplineVolume& sv, const Vec4& ver, Vec4* &volCfltPnt)
{
	// 1. 获取顶点类型
	NurbsVolVertexType verType = GetNurbsVolVertexType(sv, ver);

	// 2. 获取deltaU, deltaV, deltaW
	int deltaU, deltaV, deltaW;
	GetVertexDeltas(sv, verType, deltaU, deltaV, deltaW);

	// 3. 获取顶点索引
	int svIndex = GetNurbsVolVertexIndex(sv, verType);

	const int u = sv.m_uNum;
	const int v = sv.m_vNum;
	const int w = sv.m_wNum;

	// 4. 计算冲突点索引
	int conflictIndex = svIndex + deltaU + deltaV * u + deltaW * u * v;

	if (!CheckNurbsIndexValid(u, v, w, conflictIndex))
	{
		return false;
	}

	volCfltPnt = &sv.m_CtrlPts[conflictIndex];

	return true;
}

//寻找一个体sp内在角点pt周围三个点
const std::array<Vec4*, 3> continuityModification::FindThreePntArrAroundVer(SplineVolume &sv, const Vec4& pt)
{
	std::array<Vec4*, 3> ptsAroundVer;
	int u = sv.m_uNum, v = sv.m_vNum, w = sv.m_wNum;
	int cptNum = sv.m_CtrlPts.size();

	int deltaU = 0, deltaV = 0, deltaW = 0;

	NurbsVolVertexType verType = GetNurbsVolVertexType(sv, pt);

	GetVertexDeltas(sv, verType, deltaU, deltaV, deltaW);

	int svIndex = GetNurbsVolVertexIndex(sv, verType);
	int pntIdx = svIndex + deltaU * 1;
	CheckNurbsIndexValid(u, v, w, pntIdx);

#if DEBUG == 1
	vector<Vec4> p;
	for (auto i : sv.m_CtrlPts)
	{
		p.push_back(i);
	}
#endif

	ptsAroundVer[0] = &sv.m_CtrlPts[pntIdx];
	pntIdx = svIndex + deltaV * u;
	CheckNurbsIndexValid(u, v, w, pntIdx);

	ptsAroundVer[1] = &sv.m_CtrlPts[pntIdx];
	pntIdx = svIndex + deltaW * u*v;
	CheckNurbsIndexValid(u, v, w, pntIdx);

	ptsAroundVer[2] = &sv.m_CtrlPts[pntIdx];

	return ptsAroundVer;
}

//体内冲突点的两个方向上相交面的点
const std::array<Vec4, 3> continuityModification::FindThreePntInNeighbourFace(SplineVolume &sv, const Vec4& pt)
{
	std::array<Vec4, 3> ptsAroundVer;
	int u = sv.m_uNum, v = sv.m_vNum, w = sv.m_wNum;
	int cptNum = sv.m_CtrlPts.size();
	int du = 1, dv = u, dw = u * v;
	int deltaU = 0, deltaV = 0, deltaW = 0;

	NurbsVolVertexType verType = GetNurbsVolVertexType(sv, pt);

	GetVertexDeltas(sv, verType, deltaU, deltaV, deltaW);

	int svIndex = GetNurbsVolVertexIndex(sv, verType);
	int boundryIndex = svIndex + deltaU * du + deltaV * dv;
	CheckNurbsIndexValid(u, v, w, boundryIndex);
	
	ptsAroundVer[0] = sv.m_CtrlPts[boundryIndex];
	boundryIndex = svIndex + deltaV * dv + deltaW * dw;
	CheckNurbsIndexValid(u, v, w, boundryIndex);
	
	ptsAroundVer[1] = sv.m_CtrlPts[boundryIndex];
	boundryIndex = svIndex + deltaW * dw + deltaU * du;
	CheckNurbsIndexValid(u, v, w, boundryIndex);
	
	ptsAroundVer[2] = sv.m_CtrlPts[boundryIndex];

	return ptsAroundVer;
}


//输出vtk模型
//void continuityModification::putOutVTK(string objpath)
//{
//    cout << "putoutVTK" << endl;
//    varray<NurbsVol> NV;
//	varray<SplineVolume> tmp;
//	for (int i = 0; i < m_SV.size(); i++) {
//		if (i == 14 || i == 27)
//		{
//			m_SV[i].OrderCtrlPts(m_SV[i]);
//		}
//		tmp.push_back(m_SV[i]);
//	}
//    NV = NurbsTrans::SplinevolsToCvols(tmp);
//    objCp cp;       //输出vtk文件的类对象
//    cp = NV;
//    cp.OutputParaVolumeDataVTK(objpath);
//}

//两边点分别减去(两边点之和-边界点*2)/2
bool continuityModification::EvenModify(Vec4 &p1, Vec4 &p2, Vec4 com)
{
	bool suc = true;
	Vec4 tmp = (p1 + p2 - 2 * com) / 2;
	/*p1 -= tmp;
	p2 -= tmp;*/
	Vec4 tmp1 = p1 - tmp;
	Vec4 tmp2 = p2 - tmp;
	auto proPts = [](Vec4 pt1, Vec4 pt2, Vec4 initPt)->Vec4 {

		std::vector<double> AB(3), AP(3);
		std::vector<double> A = { pt1.x,pt1.y,pt1.z };
		std::vector<double> B = { pt2.x,pt2.y,pt2.z };
		std::vector<double> P = { initPt.x,initPt.y,initPt.z };

		for (int i = 0; i < 3; ++i) {
			AB[i] = B[i] - A[i];
			AP[i] = P[i] - A[i];
		}

		// ���� t
		double dot_AB_AB = 0.0, dot_AP_AB = 0.0;
		for (int i = 0; i < 3; ++i) {
			dot_AB_AB += AB[i] * AB[i];  // AB��AB
			dot_AP_AB += AP[i] * AB[i];  // AP��AB
		}

		// ͶӰϵ�� t
		double t = dot_AP_AB / dot_AB_AB;

		// ����ͶӰ������
		std::vector<double> proj(3);
		for (int i = 0; i < 3; ++i) {
			proj[i] = A[i] + t * AB[i];
		}

		return Vec4(proj[0], proj[1], proj[2]);
	};
	if (m_fixedPoint.find(&p1) == m_fixedPoint.end())
	{
		p1 = proPts(tmp1, tmp2, p1);
	}
	else
	{
		suc = false;
	}
	if (m_fixedPoint.find(&p2) == m_fixedPoint.end())
	{
		p2 = proPts(tmp1, tmp2, p2);
	}
	else{
		suc = false;
	}
	return suc;
}

double utility_boundryCpts(unsigned n, const double *x, double *grad, void *data)
{
	double* cx = (double*)data;
	for (int i = 0; i < 3; i++) {
		grad[i] = 0;
	}
	double ans = 0;
	for (int id = 0; id < 3; id++)
	{
		std::vector<double> AB(3), AP(3), A(3), B(3);

		for (int i = 0; i < 3; ++i) {
			A[i] = cx[id * 6 + i];
			B[i] = cx[id * 6 + i + 3];

			AB[i] = B[i] - A[i];
			AP[i] = x[i] - A[i];
		}

		
		double dot_AB_AB = 0.0, dot_AP_AB = 0.0;
		for (int i = 0; i < 3; ++i) {
			dot_AB_AB += AB[i] * AB[i];
			dot_AP_AB += AP[i] * AB[i];
		}

		
		double t = dot_AP_AB / dot_AB_AB;
		
		std::vector<double> closest_point(3);
		for (int i = 0; i < 3; ++i) {
			closest_point[i] = A[i] + t * AB[i];
		}

		
		double distance = 0.0;
		for (int i = 0; i < 3; ++i) {
			distance += (x[i] - closest_point[i]) * (x[i] - closest_point[i]);
		}
		distance = std::sqrt(distance);
		ans += distance;

		
		for (int i = 0; i < 3; ++i) {
			
			grad[i] += (x[i] - closest_point[i]) / distance;
		}

	}
	//cout << cx[0] <<" " << cx[1] << " " << cx[2] << " " << cx[3] << " "<<endl;
	return ans;
}
void continuityModification::modifyBoundryCfltPts() {
	for (auto mapIter : m_comVertexVolMap) {
		if (mapIter.second.size() > 4) {
			vector<Vec4*> verCpts;
			vector<pair<Vec4*, Vec4*>> tmp;
			for (int ii = 0; ii < mapIter.second.size(); ii++) {
				SplineVolume *vol = mapIter.second[ii];
				std::array<Vec4*, 3> pts = (FindThreePntArrAroundVer(*vol, mapIter.first));
				//pair<Vec4, Vec4> p;
				for (int jj = ii + 1; jj < mapIter.second.size(); jj++) {
					SplineVolume *vol2 = mapIter.second[jj];

					std::array<Vec4*, 3> pts2 = (FindThreePntArrAroundVer(*vol2, mapIter.first));
					if (*pts[0] == *pts2[0]) {
						if (*pts[1] == *pts2[2]) {
							tmp.push_back(make_pair(pts[2], pts2[1]));
						}
						else if (*pts[1] == *pts2[1]) {
							tmp.push_back(make_pair(pts[2], pts2[2]));
						}
						else if (*pts[2] == *pts2[1]) {
							tmp.push_back(make_pair(pts[1], pts2[2]));
						}
						else if (*pts[2] == *pts2[2]) {
							tmp.push_back(make_pair(pts[1], pts2[1]));
						}
					}
					if (*pts[0] == *pts2[1]) {
						if (*pts[1] == *pts2[2]) {
							tmp.push_back(make_pair(pts[2], pts2[0]));
						}
						else if (*pts[1] == *pts2[0]) {
							tmp.push_back(make_pair(pts[2], pts2[2]));
						}
						else if (*pts[2] == *pts2[2]) {
							tmp.push_back(make_pair(pts[1], pts2[0]));
						}
						else if (*pts[2] == *pts2[0]) {
							tmp.push_back(make_pair(pts[1], pts2[2]));
						}
					}
					if (*pts[0] == *pts2[2]) {
						if (*pts[1] == *pts2[0]) {
							tmp.push_back(make_pair(pts[2], pts2[1]));
						}
						else if (*pts[1] == *pts2[1]) {
							tmp.push_back(make_pair(pts[2], pts2[0]));
						}
						else if (*pts[2] == *pts2[0]) {
							tmp.push_back(make_pair(pts[1], pts2[1]));
						}
						else if (*pts[2] == *pts2[1]) {
							tmp.push_back(make_pair(pts[1], pts2[0]));
						}
					}
					if (*pts[1] == *pts2[1]) {
						if (*pts[2] == *pts2[2]) {
							tmp.push_back(make_pair(pts[0], pts2[0]));
						}
						else if (*pts[2] == *pts2[0]) {
							tmp.push_back(make_pair(pts[0], pts2[2]));
						}
					}
					if (*pts[1] == *pts2[2]) {
						if (*pts[2] == *pts2[1]) {
							tmp.push_back(make_pair(pts[0], pts2[0]));
						}
						else if (*pts[2] == *pts2[0]) {
							tmp.push_back(make_pair(pts[0], pts2[1]));
						}
					}
					
				}

			}

			//avoid duplicate
			vector<pair<Vec4*, Vec4*>> tmpvar;
			set<Vec4> tmpSet;
			for (auto i : tmp) {
				if (tmpSet.count(*i.first) && tmpSet.count(*i.second)) {
					continue;
				}
				tmpvar.push_back(i);
				tmpSet.insert(*i.first);
				tmpSet.insert(*i.second);

			}
			tmp = tmpvar;

			if (tmp.size() != 3) { 
				continue; 
			}
			//assert(tmp.size() == 3);
			Vec4 vec1(*tmp[0].first - *tmp[0].second);
			Vec4 vec2(*tmp[0].first - *tmp[0].second);
			Vec4 vec3(*tmp[0].first - *tmp[0].second);

			double angleOfCross = vec1.Cross(vec2).Angle(vec3);
			if (angleOfCross > 30 && angleOfCross < 150) continue;

			unordered_map<Vec4*,std::vector<Vec4*>> boundryToBoundryCptsArrMap;
			for (auto &tmpPair : tmp) {

				Vec4* pairVec1 = tmpPair.first, *pairVec2 = tmpPair.second;
				for (auto &vol : mapIter.second) {
					for (auto& tmpCpt : vol->m_CtrlPts)
					{
						if (tmpCpt == *pairVec1) {
							boundryToBoundryCptsArrMap[pairVec1].push_back(&tmpCpt);
						}
						if (tmpCpt == *pairVec2) {
							boundryToBoundryCptsArrMap[pairVec2].push_back(&tmpCpt);
						}
					}
				}
			}

			// ���� nlopt �Ż���
			nlopt_opt opter = nlopt_create(NLOPT_LD_LBFGS, 3);  // 3ά���⣬ʹ�� LBFGS �㷨

			double lb[3] = { -INF,-INF,-INF };//��Сֵ
			double ub[3] = { INF,INF,INF };//���ֵ

			// ���ñ߽�����
			nlopt_set_lower_bounds(opter, lb);
			nlopt_set_upper_bounds(opter, ub);

			double boundryCpts[3] = { mapIter.first.x,mapIter.first.y,mapIter.first.z };//ԭ���ĳ�ͻ��x

			double initCpts[18];
			int cnt = 0;
			for (auto i : tmp) {
				initCpts[cnt++] = i.first->x;
				initCpts[cnt++] = i.first->y;
				initCpts[cnt++] = i.first->z;
				initCpts[cnt++] = i.second->x;
				initCpts[cnt++] = i.second->y;
				initCpts[cnt++] = i.second->z;
			}

			nlopt_set_min_objective(opter, utility_boundryCpts, initCpts);

			double tol = 1e-8;//�ݲ�
			double f_min;  // ��Сֵ

			// stopping criterion
			nlopt_set_xtol_rel(opter, tol);
			nlopt_set_ftol_abs(opter, tol);
			nlopt_set_force_stop(opter, tol);

			// optimize
			nlopt_result result = nlopt_optimize(opter, boundryCpts, &f_min);

			// free
			nlopt_destroy(opter);

			Vec4 ver = Vec4{ boundryCpts[0],boundryCpts[1],boundryCpts[2] };
			auto proPts = [](Vec4 pt1, Vec4 pt2, Vec4 initPt)->Vec4 {

				std::vector<double> AB(3), AP(3);
				std::vector<double> A = { pt1.x,pt1.y,pt1.z };
				std::vector<double> B = { pt2.x,pt2.y,pt2.z };
				std::vector<double> P = { initPt.x,initPt.y,initPt.z };

				for (int i = 0; i < 3; ++i) {
					AB[i] = B[i] - A[i];
					AP[i] = P[i] - A[i];
				}

				// ���� t
				double dot_AB_AB = 0.0, dot_AP_AB = 0.0;
				for (int i = 0; i < 3; ++i) {
					dot_AB_AB += AB[i] * AB[i];  // AB��AB
					dot_AP_AB += AP[i] * AB[i];  // AP��AB
				}

				// ͶӰϵ�� t
				double t = dot_AP_AB / dot_AB_AB;

				// ����ͶӰ������
				std::vector<double> proj(3);
				for (int i = 0; i < 3; ++i) {
					proj[i] = A[i] + t * AB[i];
				}

				return Vec4(proj[0], proj[1], proj[2]);
			};

			for (auto& tmpPair : tmp) {
				Vec4 tmpVec1 = *tmpPair.first;
				Vec4 tmpVec2 = *tmpPair.second;
				EvenModify(tmpVec1, tmpVec2, ver);
				*tmpPair.first = proPts(tmpVec1, tmpVec2, *tmpPair.first);
				*tmpPair.second = proPts(tmpVec1, tmpVec2, *tmpPair.second);
				EvenModify(*tmpPair.first, *tmpPair.second, ver);
				m_fixedPoint.insert(tmpPair.first);
				m_fixedPoint.insert(tmpPair.second);
				for (auto& tmpBdyPts : boundryToBoundryCptsArrMap[tmpPair.first]) {
					if (tmpBdyPts == tmpPair.first) {
						continue;
					}
					*tmpBdyPts = *tmpPair.first;
				}
				for (auto& tmpBdyPts : boundryToBoundryCptsArrMap[tmpPair.second]) {
					if (tmpBdyPts == tmpPair.second) {
						continue;
					}
					*tmpBdyPts = *tmpPair.second;
				}
			}



			Vec4 *vertex = const_cast<Vec4*>(&mapIter.first);
			for (auto &vol : m_comVertexVolMap[*vertex]) {
				int u = vol->m_uNum, v = vol->m_vNum, w = vol->m_wNum;
				if (vol->m_CtrlPts[0] == *vertex) {
					vol->m_CtrlPts[0] = ver;
				}
				else if (vol->m_CtrlPts[u - 1] == *vertex) {
					vol->m_CtrlPts[u - 1] = ver;
				}
				else if (vol->m_CtrlPts[u*(v - 1)] == *vertex) {
					vol->m_CtrlPts[u*(v - 1)] = ver;
				}
				else if (vol->m_CtrlPts[u*v - 1] == *vertex) {
					vol->m_CtrlPts[u*v - 1] = ver;
				}
				else if (vol->m_CtrlPts[u*v*(w - 1)] == *vertex) {
					vol->m_CtrlPts[u*v*(w - 1)] = ver;
				}
				else if (vol->m_CtrlPts[u*v*(w - 1) + u - 1] == *vertex) {
					vol->m_CtrlPts[u*v*(w - 1) + u - 1] = ver;
				}
				else if (vol->m_CtrlPts[u*v*(w - 1) + u * (v - 1)] == *vertex) {
					vol->m_CtrlPts[u*v*(w - 1) + u * (v - 1)] = ver;
				}
				else if (vol->m_CtrlPts[u*v*(w - 1) + u * v - 1] == *vertex) {
					vol->m_CtrlPts[u*v*(w - 1) + u * v - 1] = ver;
				}

			}
			*vertex = ver;


		}
	}
}

//保存模型
void continuityModification::saveModel() 
{
    if (m_SV.size()) 
	{
        //m_rwg.WriteSplineVolume("./g1/post/post_" + m_filename + ".txt", m_SV);
        //putOutObj("./obj/" + m_filename + ".obj", 25);//保存obj模型
        //putOutVTK("./vtk/" + m_filename + ".vtk");//保存vtk
		OccIgesTrans octrans;
		octrans.outputIges(m_SV, "./iges/post_" + m_filename + ".iges");

		
    }
}


//罗德里格旋转公式
Vec4 continuityModification::glRotatef(const Vec4 &old_point, const Vec4 &axle, double angle) {
    Vec4 new_point;
    double rad = angle * PI / 180.0;
    double c = cos(rad);
    double s = sin(rad);
    new_point.x = (axle.x * axle.x*(1 - c) + c) * old_point.x + (axle.x*axle.y*(1 - c) - axle.z*s) * old_point.y + (axle.x*axle.z*(1 - c) + axle.y*s) * old_point.z;
    new_point.y = (axle.y*axle.x*(1 - c) + axle.z*s) * old_point.x + (axle.y*axle.y*(1 - c) + c) * old_point.y + (axle.y*axle.z*(1 - c) - axle.x*s) * old_point.z;
    new_point.z = (axle.x*axle.z*(1 - c) - axle.y*s) * old_point.x + (axle.y*axle.z*(1 - c) + axle.x*s) * old_point.y + (axle.z*axle.z*(1 - c) + c) * old_point.z;
    return new_point;
}

continuityModification::DirectionMapping continuityModification::GetDirectionMapping(
	CfltPntDirection outDir1,
	CfltPntDirection outDir2)
{
	// 将CfltPntDirection转换为基本方向（忽略正负）
	auto getBaseDirection = [](CfltPntDirection dir) {
		if (dir == UDirecetion || dir == UInvDirecetion) return 0; // U方向
		if (dir == VDirecetion || dir == VInvDirecetion) return 1; // V方向
		if (dir == WDirecetion || dir == WInvDirecetion) return 2; // W方向
		return -1; // 未知
	};

	int baseDir1 = getBaseDirection(outDir1);
	int baseDir2 = getBaseDirection(outDir2);

	// 根据基本方向组合确定映射关系
	switch (baseDir1) 
	{
	case 0: // U方向
		switch (baseDir2) 
		{
		case 0: return DirectionMapping::U1_to_U2;
		case 1: return DirectionMapping::U1_to_V2;
		case 2: return DirectionMapping::U1_to_W2;
		}
		break;
	case 1: // V方向
		switch (baseDir2) 
		{
		case 0: return DirectionMapping::V1_to_U2;
		case 1: return DirectionMapping::V1_to_V2;
		case 2: return DirectionMapping::V1_to_W2;
		}
		break;
	case 2: // W方向
		switch (baseDir2) 
		{
		case 0: return DirectionMapping::W1_to_U2;
		case 1: return DirectionMapping::W1_to_V2;
		case 2: return DirectionMapping::W1_to_W2;
		}
		break;
	default:
	{
		return DirectionMapping::UNKNOWN_DIRECTION;
		break;
	}
	}

}

bool continuityModification::FindMatchingDirections(
	SplineVolume* sv1,
	SplineVolume* sv2,
	const Vec4& ver,
	DirectionMapping& neiborMapping1,
	DirectionMapping& neiborMapping2,
	DirectionMapping& dirctionMapping)
{
	if (!sv1 || !sv2) {
		return false;
	}

	// 1. 获取顶点类型
	const VertexType& verType1 = GetNurbsVolVertexType(*sv1, ver);
	const VertexType& verType2 = GetNurbsVolVertexType(*sv2, ver);

	// 2. 获取周围的三个点
	const std::array<Vec4*, 3> aroundPnts1 = FindThreePntArrAroundVer(*sv1, ver);
	const std::array<Vec4*, 3> aroundPnts2 = FindThreePntArrAroundVer(*sv2, ver);

	// 3. 检查6个点中是否有4个点两两相等
	for (int idx1 = 0; idx1 < 3; ++idx1) 
	{
		for (int idx2 = 0; idx2 < 3; ++idx2) 
		{
			if (!aroundPnts1[idx1] || !aroundPnts2[idx2] ||
				*aroundPnts1[idx1] != *aroundPnts2[idx2]) 
			{
				continue;
			}

			// 找到第一对相等的点
			for (int otherIdx1 = idx1 + 1; otherIdx1 < 3; ++otherIdx1) 
			{
				for (int otherIdx2 = 0; otherIdx2 < 3; ++otherIdx2) 
				{
					if (otherIdx2 == idx2)
					{
						continue;
					}

					if (!aroundPnts1[otherIdx1] || !aroundPnts2[otherIdx2] ||
						*aroundPnts1[otherIdx1] != *aroundPnts2[otherIdx2]) 
					{
						continue;
					}

					// 找到第二对相等的点，处理剩下的2个点的方向
					int remainingIdx1 = 3 - idx1 - otherIdx1;
					int remainingIdx2 = 3 - idx2 - otherIdx2;

					if (!aroundPnts1[remainingIdx1] || !aroundPnts2[remainingIdx2]) 
					{
						continue;
					}
					auto getMapping = [&](int index1, int index2)->DirectionMapping
					{
						CfltPntDirection outDir1 = GetAxleDirection(*sv1, ver, aroundPnts1[index1]);
						CfltPntDirection outDir2 = GetAxleDirection(*sv2, ver, aroundPnts2[index2]);

						return GetDirectionMapping(outDir1, outDir2);
					};
					
					dirctionMapping = getMapping(remainingIdx1, remainingIdx2);
					neiborMapping1 = getMapping(otherIdx1, otherIdx2);
					neiborMapping2 = getMapping(idx1, idx2);
					return true;
				}
			}
		}
	}

	return false;
}

void continuityModification::ProcessDirections(SplineVolume * sv1,
	SplineVolume * sv2,
	DirectionMapping & neiborMapping1,
	DirectionMapping & neiborMapping2,
	DirectionMapping & dirctionMapping,
	const Vec4 & ver)
{
	// 1. 获取顶点类型
	const VertexType& verType1 = GetNurbsVolVertexType(*sv1, ver);
	const VertexType& verType2 = GetNurbsVolVertexType(*sv2, ver);

	// 2. 获取delta值
	int deltaU1, deltaV1, deltaW1;
	int deltaU2, deltaV2, deltaW2;

	GetVertexDeltas(*sv1, verType1, deltaU1, deltaV1, deltaW1);
	GetVertexDeltas(*sv2, verType2, deltaU2, deltaV2, deltaW2);

	int index1 = -1;
	index1 = GetNurbsVolVertexIndex(*sv1, verType1);

	int index2 = -1;
	index2 = GetNurbsVolVertexIndex(*sv2, verType2);

	auto GetDirectionIndex = [&](const SplineVolume & sv1, const SplineVolume & sv2, DirectionMapping &mapping,
		int &outDeltaIndex1, int &outDeltaIndex2, int &pntCount1,int &pntCount2)->void
	{
		switch (mapping) {
		case DirectionMapping::U1_to_U2:
			outDeltaIndex1 = (deltaU1 > 0) ? 1 : -1;
			outDeltaIndex2 = (deltaU2 > 0) ? 1 : -1;
			pntCount1 = sv1.m_uNum;
			pntCount2 = sv2.m_uNum;
			break;

		case DirectionMapping::U1_to_V2:
			outDeltaIndex1 = (deltaU1 > 0) ? 1 : -1;
			outDeltaIndex2 = (deltaV2 > 0) ? sv2.m_uNum : -sv2.m_uNum;
			pntCount1 = sv1.m_uNum;
			pntCount2 = sv2.m_vNum;
			break;

		case DirectionMapping::U1_to_W2:
			outDeltaIndex1 = (deltaU1 > 0) ? 1 : -1;
			outDeltaIndex2 = (deltaW2 > 0) ? sv2.m_uNum * sv2.m_vNum : -sv2.m_uNum * sv2.m_vNum;
			pntCount1 = sv1.m_uNum;
			pntCount2 = sv2.m_wNum;
			break;

		case DirectionMapping::V1_to_U2:
			outDeltaIndex1 = (deltaV1 > 0) ? sv1.m_uNum : -sv1.m_uNum;
			outDeltaIndex2 = (deltaU2 > 0) ? 1 : -1;
			pntCount1 = sv1.m_vNum;
			pntCount2 = sv2.m_uNum;
			break;

		case DirectionMapping::V1_to_V2:
			outDeltaIndex1 = (deltaV1 > 0) ? sv1.m_uNum : -sv1.m_uNum;
			outDeltaIndex2 = (deltaV2 > 0) ? sv2.m_uNum : -sv2.m_uNum;
			pntCount1 = sv1.m_vNum;
			pntCount2 = sv2.m_vNum;
			break;

		case DirectionMapping::V1_to_W2:
			outDeltaIndex1 = (deltaV1 > 0) ? sv1.m_uNum : -sv1.m_uNum;
			outDeltaIndex2 = (deltaW2 > 0) ? sv2.m_uNum * sv2.m_vNum : -sv2.m_uNum * sv2.m_vNum;
			pntCount1 = sv1.m_vNum;
			pntCount2 = sv2.m_wNum;
			break;

		case DirectionMapping::W1_to_U2:
			outDeltaIndex1 = (deltaW1 > 0) ? sv1.m_uNum * sv1.m_vNum : -sv1.m_uNum * sv1.m_vNum;
			outDeltaIndex2 = (deltaU2 > 0) ? 1 : -1;
			pntCount1 = sv1.m_wNum;
			pntCount2 = sv2.m_uNum;
			break;

		case DirectionMapping::W1_to_V2:
			outDeltaIndex1 = (deltaW1 > 0) ? sv1.m_uNum * sv1.m_vNum : -sv1.m_uNum * sv1.m_vNum;
			outDeltaIndex2 = (deltaV2 > 0) ? sv2.m_uNum : -sv2.m_uNum;
			pntCount1 = sv1.m_wNum;
			pntCount2 = sv2.m_vNum;
			break;

		case DirectionMapping::W1_to_W2:
			outDeltaIndex1 = (deltaW1 > 0) ? sv1.m_uNum * sv1.m_vNum : -sv1.m_uNum * sv1.m_vNum;
			outDeltaIndex2 = (deltaW2 > 0) ? sv2.m_uNum * sv2.m_vNum : -sv2.m_uNum * sv2.m_vNum;
			pntCount1 = sv1.m_wNum;
			pntCount2 = sv2.m_wNum;
			break;
		default:
			outDeltaIndex1 = 0;
			outDeltaIndex2 = 0;
			pntCount1 = 0;
			pntCount2 = 0;
#if DEBUG ==1
			assert(false && "Unknown direction mapping");
#endif
			break;
		}
	};

	int dirctionDelta1 ,dirctionDelta2 = 0;
	int noUse1 , noUse2 = 0;
	GetDirectionIndex(*sv1, *sv2, dirctionMapping, dirctionDelta1, dirctionDelta2, noUse1, noUse2);

	int pntCountForOutterLoop1 = 0, pntCountForOutterLoop2 = 0;
	int neighborOutterDelta1 =0 , neighborOutterDelta2 = 0;
	GetDirectionIndex(*sv1, *sv2, neiborMapping1, neighborOutterDelta1, neighborOutterDelta2, pntCountForOutterLoop1, pntCountForOutterLoop2);

	assert(pntCountForOutterLoop1 == pntCountForOutterLoop2);

	int pntCountForInnerLoop1 = 0, pntCountForInnerLoop2 = 0;
	int neighborInnerDelta1 = 0, neighborInnerDelta2 = 0;
	GetDirectionIndex(*sv1, *sv2, neiborMapping2, neighborInnerDelta1, neighborInnerDelta2, pntCountForInnerLoop1, pntCountForInnerLoop2);

	int outterCnt1 = 0;
	int outterCnt2 = 0;
	for (int i = 0; i < min(pntCountForOutterLoop1, pntCountForOutterLoop2); i++)
	{
		int innerCnt1 = 0;
		int innerCnt2 = 0;
		for (int j = 0; j < min(pntCountForInnerLoop1, pntCountForInnerLoop2); j++)
		{
			if (EvenModify(
				sv1->m_CtrlPts[index1 + innerCnt1 + outterCnt1 + dirctionDelta1],
				sv2->m_CtrlPts[index2 + innerCnt2 + outterCnt2 + dirctionDelta2],
				sv1->m_CtrlPts[index1 + innerCnt1 + outterCnt1]
			))
			{
				m_composite_conn++;
			}

			innerCnt1 += neighborInnerDelta1;
			innerCnt2 += neighborInnerDelta2;
		}
		outterCnt1 += neighborOutterDelta1;
		outterCnt2 += neighborOutterDelta2;
	}
}

void continuityModification::ProcessSplineVolumes(
	const Vec4& ver,
	const std::vector<SplineVolume*>& svArr)
{
	const int numVolumes = svArr.size();

	// 两两遍历所有体
	for (int i = 0; i < numVolumes; ++i)
	{
		SplineVolume* sv1 = svArr[i];
		if (!sv1)
		{
			continue;
		}

		for (int j = i + 1; j < numVolumes; ++j) 
		{
			SplineVolume* sv2 = svArr[j];
			if (!sv2)
			{
				continue;
			}

			DirectionMapping neiborMapping1, neiborMapping2, dirctionMapping;
			if (!FindMatchingDirections(sv1, sv2, ver, neiborMapping1, neiborMapping2, dirctionMapping))
			{
				continue;
			}

			ProcessDirections(sv1, sv2, neiborMapping1, neiborMapping2, dirctionMapping, ver);
		}
	}
}

continuityModification::CfltPntDirection continuityModification::GetAxleDirection(
	SplineVolume& sv,
	const Vec4& vertex,
	const Vec4* axlePnt)
{
	// 1. 获取axle索引并检查有效性
	const int u = sv.m_uNum;
	const int v = sv.m_vNum;
	const int w = sv.m_wNum;

	int axleIndex = FindPntIndexInNurbsVolume(sv, *axlePnt);

	// 2. 获取顶点类型和索引
	const VertexType verType = GetNurbsVolVertexType(sv, vertex);
	int vertexIndex = GetNurbsVolVertexIndex(sv, verType);
#if DEBUG == 1
	std::vector<Vec4> tmpArr;
	for (auto pt : sv.m_CtrlPts)
	{
		tmpArr.push_back(pt);
	}
#endif
	// 3. 计算索引差值
	int indexDiff = axleIndex - vertexIndex;

	// 4. 计算各方向的步长
	const int uStep = 1;          // u方向步长
	const int vStep = u;          // v方向步长
	const int wStep = u * v;      // w方向步长

	// 5. 判断6个方向
	if (indexDiff == uStep) {
		return CfltPntDirection::UDirecetion;      // +U方向
	}
	else if (indexDiff == -uStep) {
		return CfltPntDirection::UInvDirecetion;   // -U方向
	}
	else if (indexDiff == vStep) {
		return CfltPntDirection::VDirecetion;      // +V方向
	}
	else if (indexDiff == -vStep) {
		return CfltPntDirection::VInvDirecetion;   // -V方向
	}
	else if (indexDiff == wStep) {
		return CfltPntDirection::WDirecetion;      // +W方向
	}
	else if (indexDiff == -wStep) {
		return CfltPntDirection::WInvDirecetion;   // -W方向
	}
#if DEBUG ==1
	assert(false && "Invalid direction difference");
#endif
	return CfltPntDirection::UNKNOWN;
}

//查找冲突点，m_SV为所查找的体，p1为共角点、p2和p3分别是p1附近边界上最近的点，通过这三个点来确定冲突点的位置
std::vector<Vec4*>  continuityModification::GetConflictCptArr(SplineVolume &sv, const Vec4 &vertex, const Vec4 *p2, const Vec4 *p3, const Vec4 *axle)
{
	int axleIndex = FindPntIndexInNurbsVolume(sv, *axle);
	// 1. 获取顶点类型
	// 3. 检查索引有效性
	const int u = sv.m_uNum;
	const int v = sv.m_vNum;
	const int w = sv.m_wNum;
	const int totalPoints = u * v * w;
	CheckNurbsIndexValid(u, v, w, axleIndex);
	
	const VertexType& verType = GetNurbsVolVertexType(sv, vertex);

	// 2. 获取顶点索引
	int vertexIndex = GetNurbsVolVertexIndex(sv, verType);
	CheckNurbsIndexValid(u, v, w, vertexIndex);

	// 4. 计算索引差值
	int indexDiff = axleIndex - vertexIndex;

	// 5. 计算各方向的步长
	const int uStep = 1;          // u方向步长
	const int vStep = u;          // v方向步长
	const int wStep = u * v;      // w方向步长

	int deltaU, deltaV, deltaW;
	GetVertexDeltas(sv, verType, deltaU, deltaV, deltaW);

	// 6. 判断方向
	if (indexDiff != 0) {
		// U方向检查：差值必须是±1的倍数，且在u范围内
		if (indexDiff % uStep == 0 && std::abs(indexDiff) < vStep)
		{
			int cfltPntIndex = vertexIndex + vStep * deltaV + wStep * deltaW;
			CheckNurbsIndexValid(u, v, w, cfltPntIndex);
			return GetCfltCptArrAlongDirection(&sv, cfltPntIndex, (deltaU >0 ? CfltPntDirection::UDirecetion: CfltPntDirection::UInvDirecetion));
		}
		// V方向检查：差值必须是±u的倍数，且在v范围内
		else if (indexDiff % vStep == 0 && std::abs(indexDiff) < wStep) 
		{
			int cfltPntIndex = vertexIndex + uStep * deltaU + wStep * deltaW;
			CheckNurbsIndexValid(u, v, w, cfltPntIndex);
			return GetCfltCptArrAlongDirection(&sv, cfltPntIndex, (deltaV > 0 ? CfltPntDirection::VDirecetion : CfltPntDirection::VInvDirecetion));
		}
		// W方向检查：差值必须是±(u*v)的倍数，且在w范围内
		else if (indexDiff % wStep == 0 ) 
		{
			int cfltPntIndex = vertexIndex + uStep * deltaU + vStep * deltaV;
			CheckNurbsIndexValid(u, v, w, cfltPntIndex);
			return GetCfltCptArrAlongDirection(&sv, cfltPntIndex, (deltaW > 0 ? CfltPntDirection::WDirecetion : CfltPntDirection::WInvDirecetion));
		}
#if DEBUG == 1
		else
		{
			varray<SplineVolume> f;
			f.push_back(sv);
			m_rwg.WriteSplineVolume("d:/test.txt", f);
			assert(false);
		}
#endif
	}
}

//obj转varray<SplineSurface>
//varray<SplineSurface> continuityModification::getObjSurface(string path) {
//    std::ifstream file(path);
//    varray<SplineSurface> sf;
//    varray<Vec4> pts;
//    Model_Solution m;
//    if (file.is_open()) {
//        string line;
//        while (std::getline(file, line)) {
//            stringstream ss(line);
//            vector<string> s;
//            string p;
//            while (std::getline(ss, p, ' ')) {
//                if (p.size())
//                    s.push_back(p);
//            }
//            if (s[0] == "Vertex") {
//                Vec4 pt(stod(s[2]), stod(s[3]), stod(s[4]));
//                pts.push_back(pt);
//            }
//            else if (s[0] == "Face") {
//                if (s.size() > 5) {
//                    varray<Spline> sp;
//                    sp.resize(4);
//                    sp[0] = Feature_Line(pts[stoi(s[2]) - 1], pts[stoi(s[3]) - 1]);
//                    sp[1] = Feature_Line(pts[stoi(s[3]) - 1], pts[stoi(s[4]) - 1]);
//                    sp[2] = Feature_Line(pts[stoi(s[4]) - 1], pts[stoi(s[5]) - 1]);
//                    sp[3] = Feature_Line(pts[stoi(s[2]) - 1], pts[stoi(s[5]) - 1]);
//                    SplineSurface tmp;
//                    tmp.CoonsInterpolate(sp);
//                    sf.push_back(tmp);
//                }
//            }
//        }
//        file.close();
//    }
//    else {
//
//    }
//    m_rwg.WriteSplineSurface("./g1/postsurface.txt", sf);
//    return sf;
//}

void continuityModification::putOutObj(string path, int seg)
{
    //if (m_SV.size()) {
    //    cout << "putoutobj" << endl;
    //    varray<NurbsVol> NV;
    //    NV = NurbsTrans::SplinevolsToCvols(m_SV);
    //    objCp cp;       //输出vtk文件的类对象
    //    cp = NV;
    //    cp.OutputParaVolumeDataObj(path, seg);//保存文件路径
    //}
}


double xx[4] = { 0, };//外部参数

 /*目标函数，n代表待优化的因子个数，
    x是待优化的参数向量，
    grad 是梯度，对于基于梯度的算法，则在目标函数里面需要给grad赋值， grad[0]附值对应x[0]的偏导数，grad[1]附值对应x[1]的偏导数。对于无导数算法的，不需要计算导数，grad应该是NULL。
    data为外部参数 如 ax+by，a和b为外部参数
    */
double utility(unsigned n, const double *x, double *grad, void *data)
{
    double* cx = (double *)data;
    if (grad) {
        grad[0] = 2 * x[0] - 2 * xx[0];
        grad[1] = 2 * x[1] - 2 * xx[1];
        grad[2] = 2 * x[2] - 2 * xx[2];
        //grad[3] = 2 * x[3] - 2 * xx[3];
    }
    return (xx[0] - x[0])*(xx[0] - x[0]) + (xx[1] - x[1])*(xx[1] - x[1]) + (xx[2] - x[2])*(xx[2] - x[2])/*+ (xx[3] - x[3])*(xx[3] - x[3])*/;
}

//等式约束1，参数同目标函数
double constraint1(unsigned n, const double *x, double *grad, void *data)
{
    if (grad) {
        grad[0] = 1;
        grad[1] = 1;
        grad[2] = 0;
        //grad[3] = 0;
    }
    double *cx = (double *)data;
    return x[0] + x[1] - 2 * cx[0];
}

//等式约束2
double constraint2(unsigned n, const double *x, double *grad, void *data)
{
    if (grad) {
        grad[0] = 0;
        grad[1] = 1;
        grad[2] = 1;
        //grad[3] = 0;
    }
    double *cx = (double *)data;
    return x[1] + x[2] - 2 * cx[1];
}
double constraint3(unsigned n, const double *x, double *grad, void *data)
{
    if (grad) {
        grad[0] = 0;
        grad[1] = 0;
        grad[2] = 1;
        //grad[3] = 1;
    }
    double *cx = (double *)data;
    //cout << cx[0] << " " << cx[1] << endl;
    return x[2] + x[3] - 2 * cx[2];
}

//固定控制点
void continuityModification::fixPoint() 
{
	for (auto &i : m_comVertexVolMap) { //遍历每一个角点所在的体
		if (i.second.size() <= 2)
		{
			continue;
		}
		//如果共角点的体数量大于2则需要固定附近的点
		for (SplineVolume *v : i.second) {
			for (Vec4* pt : FindThreePntArrAroundVer(*v, i.first)) {  //固定角点周围uvw方向的一圈控制点
				m_fixedPoint.insert(pt);
			}
		}
	}
}

void continuityModification::ModifyNeighborContinuity() 
{
	for (auto mapIter : m_comVertexVolMap)
	{
		Vec4 ver = mapIter.first;
		std::vector<SplineVolume*> &svArr = mapIter.second;
		if (svArr.size() < 2)
		{
			continue;
		}
		ProcessSplineVolumes(ver, svArr);
	}
}

//共角点的体的关系计算
void continuityModification::calComVerVol()
{
    for (auto &sv : m_SV) 
	{
        m_comVertexVolMap[GetNurbsVolVertex(sv,NurbsVolVertexType::FBL)].push_back(&sv);
        m_comVertexVolMap[GetNurbsVolVertex(sv,NurbsVolVertexType::FBR)].push_back(&sv);
        m_comVertexVolMap[GetNurbsVolVertex(sv,NurbsVolVertexType::FTR)].push_back(&sv);
        m_comVertexVolMap[GetNurbsVolVertex(sv,NurbsVolVertexType::FTL)].push_back(&sv);
        m_comVertexVolMap[GetNurbsVolVertex(sv,NurbsVolVertexType::BBR)].push_back(&sv);
        m_comVertexVolMap[GetNurbsVolVertex(sv,NurbsVolVertexType::BBL)].push_back(&sv);
        m_comVertexVolMap[GetNurbsVolVertex(sv,NurbsVolVertexType::BTR)].push_back(&sv);
        m_comVertexVolMap[GetNurbsVolVertex(sv,NurbsVolVertexType::BTL)].push_back(&sv);
    }
}

void rowEchelonForm(MatrixXf& mat) {
    int rows = mat.rows();
    int cols = mat.cols();
    int lead = 0;

    for (int r = 0; r < rows; ++r) {
        if (lead >= cols)
            return;
        int i = r;
        while (mat(i, lead) == 0) {
            ++i;
            if (i == rows) {
                i = r;
                ++lead;
                if (lead == cols)
                    return;
            }
        }

        mat.row(i).swap(mat.row(r));
        mat.row(r) = mat.row(r) / mat(r, lead);

        for (int i = 0; i < rows; ++i) {
            if (i != r) {
                mat.row(i) -= mat.row(r) * mat(i, lead);
            }
        }
    }
}

//这个是模板不需要改动
template<typename _Scalar, int NX = Eigen::Dynamic, int NY = Eigen::Dynamic>
struct Functor
{
    typedef _Scalar Scalar;
    enum {
        InputsAtCompileTime = NX,
        ValuesAtCompileTime = NY
    };
    typedef Eigen::Matrix<Scalar, InputsAtCompileTime, 1> InputType;
    typedef Eigen::Matrix<Scalar, ValuesAtCompileTime, 1> ValueType;
    typedef Eigen::Matrix<Scalar, ValuesAtCompileTime, InputsAtCompileTime> JacobianType;

    const int m_inputs, m_values;

    Functor() : m_inputs(InputsAtCompileTime), m_values(ValuesAtCompileTime) {}
    Functor(int inputs, int values) : m_inputs(inputs), m_values(values) {}
    /*需要优化的参数个数*/
    int inputs() const { return m_inputs; }
    /*cost function 的项数*/
    int values() const { return m_values; }

    // you should define that in the subclass :
    //  void operator() (const InputType& x, ValueType* v, JacobianType* _j=0) const;
};

int tmp_cnt;//未知数（兰姆达）个数
map<Vec4, array<Vec4*, 2> > tmp_cfltArr;//共角体连续约束关系
vector<Vec4*> tmp_cfltCpts;//共角体内冲突点
struct my_functor : Functor<double>
{
    // 输出个数必须大于输入个数;
    my_functor(void) : Functor<double>(tmp_cfltArr.size() * 3, tmp_cfltArr.size() * 3) {}//前者表示输入变量的个数，后者为约束方程的项数
    int operator()(const Eigen::VectorXd &x, Eigen::VectorXd &fvec) const//fvec(i)为非线性方程组的求解公式，x为变量（此处为冲突点坐标值xyz）
    {
        int cnt = 0, n = tmp_cfltArr.size(), m = tmp_cfltCpts.size();//cnt代表李嘉诚论文中共角点连续约束方程组第几行,n为约束方程个数,m为冲突点个数
        int tmp_c = 0;//第i个未知数（兰姆达），后面循环会用
        for (auto i : tmp_cfltArr) {
            int vec1 = 0, vec2 = 0;
            //寻找对于约束方程tmp_cfltArr第cnt行的两个冲突点索引vec1,vec2，表示约束关系中两个冲突点位于tmp_cfltCpts的第几个冲突点
            for (int j = 0; j < tmp_cfltCpts.size(); j++) {
                if (i.second[0] == tmp_cfltCpts[j]) {
                    vec1 = j;
                }
                else if (i.second[1] == tmp_cfltCpts[j]) {
                    vec2 = j;
                }
            }
            //当行数小于约束方程个数-兰姆达未知数个数，兰姆达为1，
            if (cnt < tmp_cfltArr.size() - tmp_cnt) {
                //x[0,m-1]为第i个冲突点的x坐标，[m,m+tmp_cnt-1]为兰姆达的值,x[m+tmp_cnt,2*m+tmp_cnt-1]为y坐标，x[2*m+tmp_cnt，3*m+tmp_cnt-1]为z坐标
                //x
                fvec(cnt) = x(vec1) + x(vec2) - 2 * (i.first.x);
                //y
                fvec(cnt + n) = x(vec1 + m + tmp_cnt) + x(vec2 + m + tmp_cnt) - 2 * (i.first.y);
                //z
                fvec(cnt + 2 * n) = x(vec1 + 2 * m + tmp_cnt) + x(vec2 + 2 * m + tmp_cnt) - 2 * (i.first.z);


            }
            //当行数大于等于约束方程个数-兰姆达未知数个数，兰姆达为未知数
            else if (cnt >= tmp_cfltArr.size() - tmp_cnt) {
                //x(m + tmp_c)表示第tmp_c个兰姆达未知数
                //cout << " lamda["<<tmp_c<<"] = " << x(m + tmp_c)  << endl;
                //x
                fvec(cnt) = x(vec1) + x(m + tmp_c)*x(vec2) - (1 + x(m + tmp_c)) * (i.first.x);
                //y
                fvec(cnt + n) = x(vec1 + m + tmp_cnt) + x(m + tmp_c)*x(vec2 + m + tmp_cnt) - (1 + x(m + tmp_c)) * (i.first.y);
                //z
                fvec(cnt + 2 * n) = x(vec1 + 2 * m + tmp_cnt) + x(m + tmp_c)*x(vec2 + 2 * m + tmp_cnt) - (1 + x(m + tmp_c)) * (i.first.z);
                tmp_c++;
            }

            cnt++;
        }
        //cout << endl;
        return 0;

    }
};

void continuityModification::modifyCfltCptArrountVolver()
{
    for (auto &mapIter : m_comVertexVolMap) 
	{
		vector<SplineVolume*> &svArr = mapIter.second;
		Vec4 ver = mapIter.first;
		if (svArr.size() > 2) 
		{
            int cfltCptNum = svArr.size();
            std::map<Vec4, array<Vec4*, 2> > clftPntAndBoundryPntMap;//2个冲突点与1个相交点
			std::map<Vec4, Vec4*> boundryPntToCfltVerMap;
			std::vector<Vec4*> volInnerCfltPntArr; //需要修改的冲突点
			for (auto &sv : svArr)
			{
				//在冲突点的两个方向的相交面的点
				std::array<Vec4, 3> boundryPntArr = FindThreePntInNeighbourFace(*sv, ver);

				Vec4* cfltVolInnerPnt = NULL;
				//寻找体内部冲突点
				if (!FindNurbsVolInnerCfltPnt(*sv, ver, cfltVolInnerPnt))
				{
					continue;
				}

				for( Vec4 &boundryPnt: boundryPntArr)
				{
					if (boundryPntToCfltVerMap.find(boundryPnt) == boundryPntToCfltVerMap.end())
					{
						boundryPntToCfltVerMap.emplace(boundryPnt, cfltVolInnerPnt);
					}
					else
					{
						if (clftPntAndBoundryPntMap.find(boundryPnt) == clftPntAndBoundryPntMap.end())
						{
							clftPntAndBoundryPntMap[boundryPnt] = { cfltVolInnerPnt,boundryPntToCfltVerMap[boundryPnt] };
						}
						else
						{
							assert(false && "边缘点不止一对？");
						}
					}
				}
				volInnerCfltPntArr.push_back(cfltVolInnerPnt);
            }
            
			cout << "共找到" << clftPntAndBoundryPntMap.size() << "对冲突点" << endl;
			if (clftPntAndBoundryPntMap.size() > 2) {
				m_3d_conn++;
			}

            int cnt = clftPntAndBoundryPntMap.size();//未知数（兰姆达）个数，此处为约束关系的个数
            VectorXd x(volInnerCfltPntArr.size() * 3 + cnt);

            //赋初值
            for (int cn = 0; cn < volInnerCfltPntArr.size(); cn++) {
                x(cn) = volInnerCfltPntArr[cn]->x;
            }
            for (int cn = volInnerCfltPntArr.size(); cn < volInnerCfltPntArr.size() + cnt; cn++) {
                x(cn) = 0;
            }
            for (int cn = volInnerCfltPntArr.size() + cnt; cn < volInnerCfltPntArr.size() * 2 + cnt; cn++) {
                x(cn) = volInnerCfltPntArr[cn - volInnerCfltPntArr.size() - cnt]->y;
            }
            for (int cn = volInnerCfltPntArr.size() * 2 + cnt; cn < volInnerCfltPntArr.size() * 3 + cnt; cn++) {
                x(cn) = volInnerCfltPntArr[cn - cnt - volInnerCfltPntArr.size() * 2]->z;
            }


            //将优化所需用的值传递到全局
            tmp_cfltArr = clftPntAndBoundryPntMap;//连续的约束关系
            tmp_cfltCpts = volInnerCfltPntArr;//冲突点
            tmp_cnt = cnt;//未知数（兰姆达）个数


            my_functor functor;
            NumericalDiff<my_functor> numDiff(functor);
            LevenbergMarquardt<NumericalDiff<my_functor>, double> lm(numDiff);

            lm.parameters.maxfev = 1000; // 最大迭代次数
            lm.parameters.xtol = 1e-9;   // 收敛阈值

            //求优化，x的最小值变化
            int status = lm.minimize(x);

            cout << "共角点体修改：修改前的\n";
            for (int ii = 0; ii < volInnerCfltPntArr.size(); ii++) {
                //修改控制点的值
                cout << volInnerCfltPntArr[ii]->x << " " << volInnerCfltPntArr[ii]->y << " " << volInnerCfltPntArr[ii]->z << " " << endl;
                volInnerCfltPntArr[ii]->x = x(ii);
                volInnerCfltPntArr[ii]->y = x(ii + volInnerCfltPntArr.size() + cnt);
                volInnerCfltPntArr[ii]->z = x(ii + volInnerCfltPntArr.size() * 2 + cnt);
                m_fixedPoint.insert(volInnerCfltPntArr[ii]);//固定控制点


            }
            cout << "共" << cfltCptNum << "个冲突点，修改后的\n";
            for (auto i : volInnerCfltPntArr) {
                cout << i->x << " " << i->y << " " << i->z << " " << endl;
            }

            if (svArr.size() > 444) {
                vector<pair<Vec4*, Vec4*>> tmp;
                for (int ii = 0; ii < svArr.size(); ii++) {
                    SplineVolume *vol = svArr[ii];
					std::array<Vec4*, 3> pts = (FindThreePntArrAroundVer(*vol, mapIter.first));
                    for (int jj = ii + 1; jj < svArr.size(); jj++) {

                        SplineVolume *vol2 = svArr[jj];
                        //寻找vol和vol2的角点附近三个点，如果其中两个点相等，那么剩余两个点则为一对边界点
                       std::array<Vec4*, 3> pts2 = FindThreePntArrAroundVer(*vol2, mapIter.first);
                        if (*pts[0] == *pts2[0]) {
                            if (*pts[1] == *pts2[2]) {
                                tmp.push_back(make_pair(pts[2], pts2[1]));
                            }
                            else if (*pts[1] == *pts2[1]) {
                                tmp.push_back(make_pair(pts[2], pts2[2]));
                            }
                            else if (*pts[2] == *pts2[1]) {
                                tmp.push_back(make_pair(pts[1], pts2[2]));
                            }
                            else if (*pts[2] == *pts2[2]) {
                                tmp.push_back(make_pair(pts[1], pts2[1]));
                            }
                        }
                        if (*pts[0] == *pts2[1]) {
                            if (*pts[1] == *pts2[2]) {
                                tmp.push_back(make_pair(pts[2], pts2[0]));
                            }
                            else if (*pts[1] == *pts2[0]) {
                                tmp.push_back(make_pair(pts[2], pts2[2]));
                            }
                            else if (*pts[2] == *pts2[2]) {
                                tmp.push_back(make_pair(pts[1], pts2[0]));
                            }
                            else if (*pts[2] == *pts2[0]) {
                                tmp.push_back(make_pair(pts[1], pts2[2]));
                            }
                        }
                        if (*pts[0] == *pts2[2]) {
                            if (*pts[1] == *pts2[0]) {
                                tmp.push_back(make_pair(pts[2], pts2[1]));
                            }
                            else if (*pts[1] == *pts2[1]) {
                                tmp.push_back(make_pair(pts[2], pts2[0]));
                            }
                            else if (*pts[2] == *pts2[0]) {
                                tmp.push_back(make_pair(pts[1], pts2[1]));
                            }
                            else if (*pts[2] == *pts2[1]) {
                                tmp.push_back(make_pair(pts[1], pts2[0]));
                            }
                        }
                        if (*pts[1] == *pts2[1]) {
                            if (*pts[2] == *pts2[2]) {
                                tmp.push_back(make_pair(pts[0], pts2[0]));
                            }
                            else if (*pts[2] == *pts2[0]) {
                                tmp.push_back(make_pair(pts[0], pts2[2]));
                            }
                        }
                        if (*pts[1] == *pts2[0]) {
                            if (*pts[2] == *pts2[2]) {
                                tmp.push_back(make_pair(pts[0], pts2[1]));
                            }
                            else if (*pts[2] == *pts2[1]) {
                                tmp.push_back(make_pair(pts[0], pts2[2]));
                            }
                        }
                        if (*pts[1] == *pts2[2]) {
                            if (*pts[2] == *pts2[1]) {
                                tmp.push_back(make_pair(pts[0], pts2[0]));
                            }
                            else if (*pts[2] == *pts2[0]) {
                                tmp.push_back(make_pair(pts[0], pts2[1]));
                            }
                        }
                    }

                }
				return;
                //此时的tmp为所有可能存在的边界点与角点的数据结构，下述ver为新角点
                Vec4 ver = (*(tmp[0].first) + *(tmp[0].second)) / 2;
                for (int ii = 0; ii < tmp.size(); ii++) {

                    std::vector<Vec4*> tmp1, tmp2;//tmp1和tmp2为与tmp[ii]中的边界点相等的点
                    for (int jj = 0; jj < svArr.size(); jj++) {
                       std::array<Vec4*, 3> t = FindThreePntArrAroundVer(*svArr[jj], mapIter.first);
                        for (auto it : t) {
                            if (*it == *tmp[ii].first&&it != tmp[ii].first) {
                                tmp1.push_back(it);
                            }
                            if (*it == *tmp[ii].second&&it != tmp[ii].second) {
                                tmp2.push_back(it);
                            }
                        }

                    }

                    EvenModify(*tmp[ii].first, *tmp[ii].second, ver);

                    //修改与边界点相等的点
                    for (auto jj : tmp1) {
                        if (jj != tmp[ii].first) {
                            *jj = *tmp[ii].first;
                        }
                    }for (auto jj : tmp2) {
                        if (jj != tmp[ii].second) {
                            *jj = *tmp[ii].second;
                        }
                    }
                }

                //这里修改共角点体的关系的map
                Vec4 *vertex = const_cast<Vec4*>(&mapIter.first);
				for (auto &sv : m_comVertexVolMap[*vertex])
				{
					VertexType vtxType = GetNurbsVolVertexType(*sv, *vertex);
					int vtxIdx = GetNurbsVolVertexIndex(*sv, vtxType);
					sv->m_CtrlPts[vtxIdx] = ver;
				}
                *vertex = ver;

            }
        }
    }

}

//根据一排冲突点的距离dis算一排冲突点
std::vector<Vec4*> continuityModification::GetCfltCptArrAlongDirection
							(SplineVolume* sv, int conflictIndex,  CfltPntDirection dir)
{
	const int u = sv->m_uNum;
	const int v = sv->m_vNum;
	const int w = sv->m_wNum;
	int dist = 0, pntCount = 0;
	switch (dir)
	{
	case continuityModification::UDirecetion:
		dist = 1;
		pntCount = u;
		break;
	case continuityModification::VDirecetion:
		dist = u;
		pntCount = v;
		break;
	case continuityModification::WDirecetion:
		dist = u * v;
		pntCount = w;
		break;
	case continuityModification::UInvDirecetion:
		dist = -1;
		pntCount = u;
		break;
	case continuityModification::VInvDirecetion:
		dist = -u;
		pntCount = v;
		break;
	case continuityModification::WInvDirecetion:
		dist = -u * v;
		pntCount = w;
		break;
	default:
		break;
	}
	std::vector<Vec4*> cfltcpts;
    for (int i = 0; i < pntCount; i++) {
		//cout << "conflictIndex " << conflictIndex << " dis " << dis << endl;
        cfltcpts.push_back(&sv->m_CtrlPts[conflictIndex]);
        conflictIndex += dist;
    }
    return cfltcpts;
}

//同上述冲突点
vector<Vec4*> continuityModification::GetBoundaryCptArr(SplineVolume &sv, const Vec4 &ver, const Vec4*boundryPnt, const Vec4*axlePnt)
{
	int axleIndex = FindPntIndexInNurbsVolume(sv, *axlePnt);
	// 1. 获取顶点类型
	// 3. 检查索引有效性
	const int u = sv.m_uNum;
	const int v = sv.m_vNum;
	const int w = sv.m_wNum;
	const int totalPoints = u * v * w;
	CheckNurbsIndexValid(u, v, w, axleIndex);

	const VertexType& verType = GetNurbsVolVertexType(sv, ver);

	// 2. 获取顶点索引
	int vertexIndex = GetNurbsVolVertexIndex(sv, verType);
	CheckNurbsIndexValid(u, v, w, vertexIndex);

	// 4. 计算索引差值

	int indexDiff = axleIndex - vertexIndex;

	// 5. 计算各方向的步长
	const int uStep = 1;          // u方向步长
	const int vStep = u;          // v方向步长
	const int wStep = u * v;      // w方向步长

	// 6. 判断方向
	int pntSize = 0;
	if (indexDiff != 0) {
		// U方向检查：差值必须是±1的倍数，且在u范围内
		if (indexDiff % uStep == 0 && std::abs(indexDiff) < vStep) 
		{
			pntSize = u;
		}
		else if (indexDiff % vStep == 0 && std::abs(indexDiff) < wStep) 
		{
			pntSize = v;
		}
		else if(indexDiff % wStep == 0 ) 
		{
			// W方向检查：差值必须是±(u*v)的倍数，且在w范围内
			pntSize = w;
		}
#if DEBUG == 1
		else
		{
			assert(false);
		}
#endif
	}

	int boundryIdx = FindPntIndexInNurbsVolume(sv, *boundryPnt);
	std::vector<Vec4*> boundaryCpts;

    for (int idx = boundryIdx, i=0 ; i< pntSize ; idx += indexDiff,i++)
	{
		CheckNurbsIndexValid(u, v, w, idx);
        boundaryCpts.push_back(&sv.m_CtrlPts[idx]);
    }
    return boundaryCpts;
}

//找某个体内点的index
int continuityModification::FindPntIndexInNurbsVolume(SplineVolume &sv,const Vec4 p)
{
    int n = sv.m_CtrlPts.size(),idx= -1;
	double dist = DBL_MAX;
    for (int i = 0; i < n; i++) 
	{
		double d = (sv.m_CtrlPts[i]).Dist(p);
		if (d < dist)
		{
			dist = d;
			idx = i;
		}
    }

	//for debug
	vector<Vec4> v;
	for (auto i : sv.m_CtrlPts)
	{
		v.push_back(i);
	}
	assert(idx != -1 && "error in FindPntIndexInNurbsVolume");
    return idx;
}



void continuityModification::createConflictArr()
{

	for (auto &mapIter : m_comVertexVolMap)
	{ //遍历每一个角点所在的体
		std::vector<SplineVolume*> svArr = mapIter.second;
		if (svArr.size() > 2)
		{
			//如果共角点的体数量大于2则需要固定附近的点
			//固定冲突点
			Vec4 vertex = mapIter.first;
			const std::array<Vec4*, 3> cptarr = (FindThreePntArrAroundVer(*(svArr)[0], vertex));//寻找该共角点第一个体的附近3个边界上的点
			if (!cptarr.size()) continue;

			for (int c = 0; c < 3; c++)
			{  //每个共角点周围的体有三个方向uov、uow、vow
				conflictCptArr cfltCptarr; //创建冲突点的链表

				Vec4 *axlePnt = cptarr[c];
				cfltCptarr.m_axleCpt = axlePnt;//用一个点来确定方向

				Vec4 *boundryPnt1 = cptarr[(c + 1) % 3];
				Vec4 *boundryPnt2 = cptarr[(c + 2) % 3];

				std::vector<Vec4*> pt = GetConflictCptArr(*svArr[0], vertex, boundryPnt1, boundryPnt2, axlePnt);//其余两个点来寻找冲突点
				std::vector<Vec4*> lastBoundryPntArr = (GetBoundaryCptArr(*svArr[0], vertex, boundryPnt1, axlePnt));
				std::vector<Vec4*> nextBoundryPntArr = (GetBoundaryCptArr(*svArr[0], vertex, boundryPnt2, axlePnt));
				if (lastBoundryPntArr.empty() || nextBoundryPntArr.empty())
				{
					continue;
				}
				conflictCpt *cfltCpt = new conflictCpt(lastBoundryPntArr, nextBoundryPntArr, pt); //单个节点
				cfltCptarr.push_next(cfltCpt);

				std::set<int> processedSvSet;
				bool bFind = true;
				while (bFind)
				{
					bFind = false;
					for (int restSvArrCount = 1; restSvArrCount < svArr.size(); restSvArrCount++)
					{
						if (processedSvSet.find(restSvArrCount) != processedSvSet.end())
						{
							continue;
						}

						//遍历剩余的体，如果方向相同（tmp[i]==cfltCptarr.axleCpt）则判断是否有一点可以与链表的头或者尾结点的边界点衔接
						const std::array<Vec4*, 3> tmp = (FindThreePntArrAroundVer(*(svArr)[restSvArrCount], vertex));

						Vec4* otherSvBoundryPnt1 = tmp[1], *otherSvBoundryPnt2 = tmp[2], *otherSvAxlePnt = tmp[0];
						if (*otherSvBoundryPnt1 == *cfltCptarr.m_axleCpt)
						{
							Vec4* tmp = otherSvBoundryPnt1;
							otherSvBoundryPnt1 = otherSvAxlePnt;
							otherSvAxlePnt = tmp;
						}
						else if (*otherSvBoundryPnt2 == *cfltCptarr.m_axleCpt)
						{
							Vec4* tmp = otherSvBoundryPnt2;
							otherSvBoundryPnt2 = otherSvAxlePnt;
							otherSvAxlePnt = tmp;
						}
						else if (*otherSvAxlePnt != *cfltCptarr.m_axleCpt)
						{
							continue;
						}

						//如果方向相同，则进行判断是否可以衔接冲突点链表，
						//若可以则插入结点（在头部push_front，尾部push_next）
						bool bIsPushNext = false;
						if (*otherSvBoundryPnt1 == *cfltCptarr.m_head->lastCpt[0] && 
							*otherSvBoundryPnt2 != *cfltCptarr.m_head->nextCpt[0]
							|| *otherSvBoundryPnt2 == *cfltCptarr.m_head->lastCpt[0] 
							&& *otherSvBoundryPnt1 != *cfltCptarr.m_head->nextCpt[0]) 
						{
							if (*otherSvBoundryPnt1 == *cfltCptarr.m_head->lastCpt[0]) 
							{
								std::swap(otherSvBoundryPnt1, otherSvBoundryPnt2);
							}
						}
						else if (*otherSvBoundryPnt1 == *cfltCptarr.m_back->nextCpt[0] 
							&& *otherSvBoundryPnt2 != *cfltCptarr.m_back->lastCpt[0] 
							|| *otherSvBoundryPnt2 == *cfltCptarr.m_back->nextCpt[0] 
							&& *otherSvBoundryPnt1 != *cfltCptarr.m_back->lastCpt[0]) 
						{
							if (*otherSvBoundryPnt2 == *cfltCptarr.m_back->nextCpt[0]) 
							{
								std::swap(otherSvBoundryPnt1, otherSvBoundryPnt2);
							}
							bIsPushNext = true;
						}
						else
						{
							continue;
						}
						bFind = true;

						processedSvSet.insert(restSvArrCount);
						vector<Vec4*> cflt = (GetConflictCptArr(*svArr[restSvArrCount], vertex, otherSvBoundryPnt1, otherSvBoundryPnt2, otherSvAxlePnt));
						vector<Vec4*> la = GetBoundaryCptArr(*(svArr)[restSvArrCount], vertex, otherSvBoundryPnt1, otherSvAxlePnt);
						vector<Vec4*> nx = GetBoundaryCptArr(*(svArr)[restSvArrCount], vertex, otherSvBoundryPnt2, otherSvAxlePnt);
						if (la.empty() || nx.empty())
						{
							continue;
						}
						conflictCpt *otherCfltNode = new conflictCpt(la, nx, cflt);
						if (bIsPushNext)
						{
							cfltCptarr.push_next(otherCfltNode);
						}
						else
						{
							cfltCptarr.push_front(otherCfltNode);
						}
						break;
					}

					//如果一次循环没有新的节点加入或者链表首尾相接了，则break
					if (*cfltCptarr.m_back->nextCpt[0] == *cfltCptarr.m_head->lastCpt[0])
					{
						bFind = false;
					}
				}

				adjustConflictCpt(cfltCptarr, &vertex);//调整冲突点
			}
		}
    }
}
int tmp_numVar;
vector<double> tmp_boundrycpts;
//非线性方程
struct my_functor2 : Functor<double>
{
    // 输出个数必须大于输入个数;
    my_functor2(void) : Functor<double>(tmp_numVar * 3, tmp_numVar * 3) {}//前者表示输入变量的个数，后者为约束方程的项数
    int operator()(const Eigen::VectorXd &x, Eigen::VectorXd &fvec) const//fvec(i)为非线性方程组的求解公式，x为变量（此处为冲突点坐标值xyz）
    {
        int cnt = 0;
        for (int j = 0; j < tmp_numVar; j++) {
            for (int i = 0; i < 3; i++) {

                fvec(cnt++) = x(i*tmp_numVar + (j + 1) % tmp_numVar) + x(i*tmp_numVar + j) - 2 * tmp_boundrycpts[i*tmp_numVar + j];

            }
        }

        return 0;

    }
};

//四片的非线性优化目标函数，n为优化未知量个数，x为优化变量，grad为目标函数对x(i)的导数，data为外部参数
double utility_vol4(unsigned n, const double *x, double *grad, void *data)
{
    double* cx = (double *)data;
    if (grad) {
        grad[0] = 2 * x[0] - 2 * cx[0];
        grad[1] = 2 * x[1] - 2 * cx[1];
        grad[2] = 2 * x[2] - 2 * cx[2];
        grad[3] = 2 * x[3] - 2 * cx[3];
		grad[4] = 2 * x[4] - 2 * cx[4];
        grad[5] = 2 * x[5] - 2 * cx[5];
        grad[6] = 2 * x[6] - 2 * cx[6];
        grad[7] = 2 * x[7] - 2 * cx[7];
		grad[8] = 2 * x[8] - 2 * cx[8];
        grad[9] = 2 * x[9] - 2 * cx[9];
        grad[10] = 2 * x[10] - 2 * cx[10];
        grad[11] = 2 * x[11] - 2 * cx[11];
    }
    //cout << cx[0] <<" " << cx[1] << " " << cx[2] << " " << cx[3] << " "<<endl;
    return (cx[0] - x[0])*(cx[0] - x[0]) + (cx[1] - x[1])*(cx[1] - x[1]) + (cx[2] - x[2])*(cx[2] - x[2]) + (cx[3] - x[3])*(cx[3] - x[3])
		+ (cx[4] - x[4])*(cx[4] - x[4]) + (cx[5] - x[5])*(cx[5] - x[5]) + (cx[6] - x[6])*(cx[6] - x[6]) + (cx[7] - x[7])*(cx[7] - x[7])
		+ (cx[8] - x[8])*(cx[8] - x[8]) + (cx[9] - x[9])*(cx[9] - x[9]) + (cx[10] - x[10])*(cx[10] - x[10]) + (cx[11] - x[11])*(cx[11] - x[11]);
}

//等式约束1，参数同目标函数
double constraint1_vol4(unsigned n, const double *x, double *grad, void *data)
{
    double *cx = (double *)data;
	double ax = x[0] - cx[0], bx = cx[0] - x[3], ay = (x[1] - cx[1]), by = (cx[1] - x[4]), az = (x[2] - cx[2]), bz = (cx[2] - x[5]);
	double cosx = (ax*bx + ay*by + az*bz) / (std::sqrt(ax*ax + ay*ay + az*az)*std::sqrt(bx * bx + by * by + bz * bz));
    if (grad) {
        grad[0] =(-ax*(ax*bx+ay*by+az*bz))
			/(pow(ax*ax+ ay*ay+ az*az,1.5)
				*sqrt(bx*bx + by*by + bz*bz))
			+bx/(sqrt(ax*ax + ay*ay + az*az)
				*sqrt(bx*bx + by*by + bz*bz));
        grad[1] = /*-sqrt(1 - cosx * cosx)**/(-ay*(ax*bx + ay*by + az*bz))
			/ (pow(ax*ax + ay*ay + az*az, 1.5)
				*sqrt(bx*bx + by*by + bz*bz))
			+ by / (sqrt(ax*ax + ay*ay + az*az)
				*sqrt(bx*bx + by*by + bz*bz));
        grad[2] = /*-sqrt(1 - cosx * cosx)**/(-az*(ax*bx + ay*by + az*bz))
			/ (pow(ax*ax + ay*ay + az*az, 1.5)
				*sqrt(bx*bx + by*by + bz*bz))
			+ bz / (sqrt(ax*ax + ay*ay + az*az)
				*sqrt(bx*bx + by*by + bz*bz));
        grad[3] = /*-sqrt(1 - cosx * cosx)**/(-bx*(ax*bx + ay*by + az*bz))
			/ (pow(ax*ax + ay*ay + az*az, 0.5)
				*pow(bx*bx + by*by + bz*bz,1.5))
			+ (cx[0] - x[0]) / (sqrt(ax*ax + ay*ay + az*az)
				*sqrt(bx*bx + by*by + bz*bz));
		grad[4] = /*-sqrt(1 - cosx * cosx)**/(-by*(ax*bx + ay*by + az*bz))
			/ (pow(ax*ax + ay*ay + az*az, 0.5)
				*pow(bx*bx + by*by + bz*bz, 1.5))
			+ (cx[1] - x[1]) / (sqrt(ax*ax + ay*ay + az*az)
				*sqrt(bx*bx + by*by + bz*bz));
        grad[5] =/* -sqrt(1 - cosx * cosx)**/(-bz*(ax*bx + ay*by + az*bz))
			/ (pow(ax*ax + ay*ay + az*az, 0.5)
				*pow(bx*bx + by*by + bz*bz, 1.5))
			+ (cx[2] - x[2]) / (sqrt(ax*ax + ay*ay + az*az)
				*sqrt(bx*bx + by*by + bz*bz));
        grad[6] = 0;
        grad[7] = 0;
		grad[8] = 0;
        grad[9] = 0;
        grad[10] = 0;
        grad[11] = 0;
    }
    return cosx -1;
}

//等式约束2
double constraint2_vol4(unsigned n, const double *x, double *grad, void *data)
{
    
	double *cx = (double *)data;
	double cosx = ((x[3] - cx[3])*(cx[3] - x[6]) + (x[4] - cx[4])*(cx[4] - x[7]) + (x[5] - cx[5])*(cx[5] - x[8])) / (std::sqrt((x[3] - cx[3])*(x[3] - cx[3]) + (x[4] - cx[4])*(x[4] - cx[4]) + (x[5] - cx[5])*(x[5] - cx[5]))*std::sqrt((cx[3] - x[6]) * (cx[3] - x[6]) + (cx[4] - x[7]) * (cx[4] - x[7]) + (cx[5] - x[8]) * (cx[5] - x[8])));
	if (grad) {
		grad[0] = 0;
		grad[1] = 0;
		grad[2] = 0;
		grad[3] = (-(x[3] - cx[3])*((x[3] - cx[3])*(cx[3] - x[6]) + (x[4] - cx[4])*(cx[4] - x[7]) + (x[5] - cx[5])*(cx[5] - x[8])))
			/ (pow((x[3] - cx[3])*(x[3] - cx[3]) + (x[4] - cx[4])*(x[4] - cx[4]) + (x[5] - cx[5])*(x[5] - cx[5]), 1.5)
				*sqrt((cx[3] - x[6])*(cx[3] - x[6]) + (cx[4] - x[7])*(cx[4] - x[7]) + (cx[5] - x[8])*(cx[5] - x[8])))
			+ (cx[3] - x[6]) / (sqrt((x[3] - cx[3])*(x[3] - cx[3]) + (x[4] - cx[4])*(x[4] - cx[4]) + (x[5] - cx[5])*(x[5] - cx[5]))
				*sqrt((cx[3] - x[6])*(cx[3] - x[6]) + (cx[4] - x[7])*(cx[4] - x[7]) + (cx[5] - x[8])*(cx[5] - x[8])));
		grad[4] = (-(x[4] - cx[4])*((x[3] - cx[3])*(cx[3] - x[6]) + (x[4] - cx[4])*(cx[4] - x[7]) + (x[5] - cx[5])*(cx[5] - x[8])))
			/ (pow((x[3] - cx[3])*(x[3] - cx[3]) + (x[4] - cx[4])*(x[4] - cx[4]) + (x[5] - cx[5])*(x[5] - cx[5]), 1.5)
				*sqrt((cx[3] - x[6])*(cx[3] - x[6]) + (cx[4] - x[7])*(cx[4] - x[7]) + (cx[5] - x[8])*(cx[5] - x[8])))
			+ (cx[4] - x[7]) / (sqrt((x[3] - cx[3])*(x[3] - cx[3]) + (x[4] - cx[4])*(x[4] - cx[4]) + (x[5] - cx[5])*(x[5] - cx[5]))
				*sqrt((cx[3] - x[6])*(cx[3] - x[6]) + (cx[4] - x[7])*(cx[4] - x[7]) + (cx[5] - x[8])*(cx[5] - x[8])));
		grad[5] = (-(x[5] - cx[5])*((x[3] - cx[3])*(cx[3] - x[6]) + (x[4] - cx[4])*(cx[4] - x[7]) + (x[5] - cx[5])*(cx[5] - x[8])))
			/ (pow((x[3] - cx[3])*(x[3] - cx[3]) + (x[4] - cx[4])*(x[4] - cx[4]) + (x[5] - cx[5])*(x[5] - cx[5]), 1.5)
				*sqrt((cx[3] - x[6])*(cx[3] - x[6]) + (cx[4] - x[7])*(cx[4] - x[7]) + (cx[5] - x[8])*(cx[5] - x[8])))
			+ (cx[5] - x[8]) / (sqrt((x[3] - cx[3])*(x[3] - cx[3]) + (x[4] - cx[4])*(x[4] - cx[4]) + (x[5] - cx[5])*(x[5] - cx[5]))
				*sqrt((cx[3] - x[6])*(cx[3] - x[6]) + (cx[4] - x[7])*(cx[4] - x[7]) + (cx[5] - x[8])*(cx[5] - x[8])));
		grad[6] = (-(cx[3] - x[6])*((x[3] - cx[3])*(cx[3] - x[6]) + (x[4] - cx[4])*(cx[4] - x[7]) + (x[5] - cx[5])*(cx[5] - x[8])))
			/ (pow((x[3] - cx[3])*(x[3] - cx[3]) + (x[4] - cx[4])*(x[4] - cx[4]) + (x[5] - cx[5])*(x[5] - cx[5]), 0.5)
				*pow((cx[3] - x[6])*(cx[3] - x[6]) + (cx[4] - x[7])*(cx[4] - x[7]) + (cx[5] - x[8])*(cx[5] - x[8]), 1.5))
			+ (cx[3] - x[3]) / (sqrt((x[3] - cx[3])*(x[3] - cx[3]) + (x[4] - cx[4])*(x[4] - cx[4]) + (x[5] - cx[5])*(x[5] - cx[5]))
				*sqrt((cx[3] - x[6])*(cx[3] - x[6]) + (cx[4] - x[7])*(cx[4] - x[7]) + (cx[5] - x[8])*(cx[5] - x[8])));
		grad[7] = (-(cx[4] - x[7])*((x[3] - cx[3])*(cx[3] - x[6]) + (x[4] - cx[4])*(cx[4] - x[7]) + (x[5] - cx[5])*(cx[5] - x[8])))
			/ (pow((x[3] - cx[3])*(x[3] - cx[3]) + (x[4] - cx[4])*(x[4] - cx[4]) + (x[5] - cx[5])*(x[5] - cx[5]), 0.5)
				*pow((cx[3] - x[6])*(cx[3] - x[6]) + (cx[4] - x[7])*(cx[4] - x[7]) + (cx[5] - x[8])*(cx[5] - x[8]), 1.5))
			+ (cx[4] - x[4]) / (sqrt((x[3] - cx[3])*(x[3] - cx[3]) + (x[4] - cx[4])*(x[4] - cx[4]) + (x[5] - cx[5])*(x[5] - cx[5]))
				*sqrt((cx[3] - x[6])*(cx[3] - x[6]) + (cx[4] - x[7])*(cx[4] - x[7]) + (cx[5] - x[8])*(cx[5] - x[8])));
		grad[8] = (-(cx[5] - x[8])*((x[3] - cx[3])*(cx[3] - x[6]) + (x[4] - cx[4])*(cx[4] - x[7]) + (x[5] - cx[5])*(cx[5] - x[8])))
			/ (pow((x[3] - cx[3])*(x[3] - cx[3]) + (x[4] - cx[4])*(x[4] - cx[4]) + (x[5] - cx[5])*(x[5] - cx[5]), 0.5)
				*pow((cx[3] - x[6])*(cx[3] - x[6]) + (cx[4] - x[7])*(cx[4] - x[7]) + (cx[5] - x[8])*(cx[5] - x[8]), 1.5))
			+ (cx[5] - x[5]) / (sqrt((x[3] - cx[3])*(x[3] - cx[3]) + (x[4] - cx[4])*(x[4] - cx[4]) + (x[5] - cx[5])*(x[5] - cx[5]))
				*sqrt((cx[3] - x[6])*(cx[3] - x[6]) + (cx[4] - x[7])*(cx[4] - x[7]) + (cx[5] - x[8])*(cx[5] - x[8])));
		
		grad[9] = 0;
		grad[10] = 0;
		grad[11] = 0;
	}

	//Vec4 tmp(x[0], x[1], x[2]);
	//Vec4 tmp1(x[3], x[4], x[5]);
	//Vec4 tmp2(cx[0], cx[1], cx[2]);
	////Vec4 tmp(x[0], x[1], x[2]);
	//varray<Vec4> tmpt;
	//tmpt.push_back(tmp);
	//tmpt.push_back(tmp1);
	//tmpt.push_back(tmp2);
	//varray<varray<Vec4>> t; t.push_back(tmpt);
	//RWGeometric rwg;
	//rwg.WritePoint("./g1/tmppts.txt", t);
	return cosx - 1;
}
double constraint3_vol4(unsigned n, const double *x, double *grad, void *data)
{
    /*if (grad) {
        grad[0] = 0;
        grad[1] = 0;
        grad[2] = 1;
        grad[3] = 1;
    }
    double *cx = (double *)data;

    return x[2] + x[3] - 2 * cx[2];*/
	/*if (grad) {
		grad[0] = 1;
		grad[1] = 1;
		grad[2] = 0;
		grad[3] = 0;
		grad[4] = 1;
		grad[5] = 1;
		grad[6] = 0;
		grad[7] = 0;
		grad[8] = 1;
		grad[9] = 1;
		grad[10] = 0;
		grad[11] = 0;
	}*/
	double *cx = (double *)data;
	double cosx = ((x[6] - cx[6])*(cx[6] - x[9]) + (x[7] - cx[7])*(cx[7] - x[10]) + (x[8] - cx[8])*(cx[8] - x[11])) / (std::sqrt((x[6] - cx[6])*(x[6] - cx[6]) + (x[7] - cx[7])*(x[7] - cx[7]) + (x[8] - cx[8])*(x[8] - cx[8]))*std::sqrt((cx[6] - x[9]) * (cx[6] - x[9]) + (cx[7] - x[10]) * (cx[7] - x[10]) + (cx[8] - x[11]) * (cx[8] - x[11])));
	if (grad) {
		grad[0] = 0;
		grad[1] = 0;
		grad[2] = 0;
		grad[3] = 0;
		grad[4] = 0;
		grad[5] = 0;
		grad[6] = (-(x[6] - cx[6])*((x[6] - cx[6])*(cx[6] - x[9]) + (x[7] - cx[7])*(cx[7] - x[10]) + (x[8] - cx[8])*(cx[8] - x[11])))
			/ (pow((x[6] - cx[6])*(x[6] - cx[6]) + (x[7] - cx[7])*(x[7] - cx[7]) + (x[8] - cx[8])*(x[8] - cx[8]), 1.5)
				*sqrt((cx[6] - x[9])*(cx[6] - x[9]) + (cx[7] - x[10])*(cx[7] - x[10]) + (cx[8] - x[11])*(cx[8] - x[11])))
			+ (cx[6] - x[9]) / (sqrt((x[6] - cx[6])*(x[6] - cx[6]) + (x[7] - cx[7])*(x[7] - cx[7]) + (x[8] - cx[8])*(x[8] - cx[8]))
				*sqrt((cx[6] - x[9])*(cx[6] - x[9]) + (cx[7] - x[10])*(cx[7] - x[10]) + (cx[8] - x[11])*(cx[8] - x[11])));
		grad[7] = (-(x[7] - cx[7])*((x[6] - cx[6])*(cx[6] - x[9]) + (x[7] - cx[7])*(cx[7] - x[10]) + (x[8] - cx[8])*(cx[8] - x[11])))
			/ (pow((x[6] - cx[6])*(x[6] - cx[6]) + (x[7] - cx[7])*(x[7] - cx[7]) + (x[8] - cx[8])*(x[8] - cx[8]), 1.5)
				*sqrt((cx[6] - x[9])*(cx[6] - x[9]) + (cx[7] - x[10])*(cx[7] - x[10]) + (cx[8] - x[11])*(cx[8] - x[11])))
			+ (cx[7] - x[10]) / (sqrt((x[6] - cx[6])*(x[6] - cx[6]) + (x[7] - cx[7])*(x[7] - cx[7]) + (x[8] - cx[8])*(x[8] - cx[8]))
				*sqrt((cx[6] - x[9])*(cx[6] - x[9]) + (cx[7] - x[10])*(cx[7] - x[10]) + (cx[8] - x[11])*(cx[8] - x[11])));
		grad[8] = (-(x[8] - cx[8])*((x[6] - cx[6])*(cx[6] - x[9]) + (x[7] - cx[7])*(cx[7] - x[10]) + (x[8] - cx[8])*(cx[8] - x[11])))
			/ (pow((x[6] - cx[6])*(x[6] - cx[6]) + (x[7] - cx[7])*(x[7] - cx[7]) + (x[8] - cx[8])*(x[8] - cx[8]), 1.5)
				*sqrt((cx[6] - x[9])*(cx[6] - x[9]) + (cx[7] - x[10])*(cx[7] - x[10]) + (cx[8] - x[11])*(cx[8] - x[11])))
			+ (cx[8] - x[11]) / (sqrt((x[6] - cx[6])*(x[6] - cx[6]) + (x[7] - cx[7])*(x[7] - cx[7]) + (x[8] - cx[8])*(x[8] - cx[8]))
				*sqrt((cx[6] - x[9])*(cx[6] - x[9]) + (cx[7] - x[10])*(cx[7] - x[10]) + (cx[8] - x[11])*(cx[8] - x[11])));
		grad[9] = (-(cx[6] - x[9])*((x[6] - cx[6])*(cx[6] - x[9]) + (x[7] - cx[7])*(cx[7] - x[10]) + (x[8] - cx[8])*(cx[8] - x[11])))
			/ (pow((x[6] - cx[6])*(x[6] - cx[6]) + (x[7] - cx[7])*(x[7] - cx[7]) + (x[8] - cx[8])*(x[8] - cx[8]), 0.5)
				*pow((cx[6] - x[9])*(cx[6] - x[9]) + (cx[7] - x[10])*(cx[7] - x[10]) + (cx[8] - x[11])*(cx[8] - x[11]), 1.5))
			+ (cx[6] - x[6]) / (sqrt((x[6] - cx[6])*(x[6] - cx[6]) + (x[7] - cx[7])*(x[7] - cx[7]) + (x[8] - cx[8])*(x[8] - cx[8]))
				*sqrt((cx[6] - x[9])*(cx[6] - x[9]) + (cx[7] - x[10])*(cx[7] - x[10]) + (cx[8] - x[11])*(cx[8] - x[11])));
		grad[10] = (-(cx[7] - x[10])*((x[6] - cx[6])*(cx[6] - x[9]) + (x[7] - cx[7])*(cx[7] - x[10]) + (x[8] - cx[8])*(cx[8] - x[11])))
			/ (pow((x[6] - cx[6])*(x[6] - cx[6]) + (x[7] - cx[7])*(x[7] - cx[7]) + (x[8] - cx[8])*(x[8] - cx[8]), 0.5)
				*pow((cx[6] - x[9])*(cx[6] - x[9]) + (cx[7] - x[10])*(cx[7] - x[10]) + (cx[8] - x[11])*(cx[8] - x[11]), 1.5))
			+ (cx[7] - x[7]) / (sqrt((x[6] - cx[6])*(x[6] - cx[6]) + (x[7] - cx[7])*(x[7] - cx[7]) + (x[8] - cx[8])*(x[8] - cx[8]))
				*sqrt((cx[6] - x[9])*(cx[6] - x[9]) + (cx[7] - x[10])*(cx[7] - x[10]) + (cx[8] - x[11])*(cx[8] - x[11])));
		grad[11] = (-(cx[8] - x[11])*((x[6] - cx[6])*(cx[6] - x[9]) + (x[7] - cx[7])*(cx[7] - x[10]) + (x[8] - cx[8])*(cx[8] - x[11])))
			/ (pow((x[6] - cx[6])*(x[6] - cx[6]) + (x[7] - cx[7])*(x[7] - cx[7]) + (x[8] - cx[8])*(x[8] - cx[8]), 0.5)
				*pow((cx[6] - x[9])*(cx[6] - x[9]) + (cx[7] - x[10])*(cx[7] - x[10]) + (cx[8] - x[11])*(cx[8] - x[11]), 1.5))
			+ (cx[8] - x[8]) / (sqrt((x[6] - cx[6])*(x[6] - cx[6]) + (x[7] - cx[7])*(x[7] - cx[7]) + (x[8] - cx[8])*(x[8] - cx[8]))
				*sqrt((cx[6] - x[9])*(cx[6] - x[9]) + (cx[7] - x[10])*(cx[7] - x[10]) + (cx[8] - x[11])*(cx[8] - x[11])));

	
	}

	//Vec4 tmp(x[0], x[1], x[2]);
	//Vec4 tmp1(x[3], x[4], x[5]);
	//Vec4 tmp2(cx[0], cx[1], cx[2]);
	////Vec4 tmp(x[0], x[1], x[2]);
	//varray<Vec4> tmpt;
	//tmpt.push_back(tmp);
	//tmpt.push_back(tmp1);
	//tmpt.push_back(tmp2);
	//varray<varray<Vec4>> t; t.push_back(tmpt);
	//RWGeometric rwg;
	//rwg.WritePoint("./g1/tmppts.txt", t);
	return cosx - 1;
}

double constraint4_vol4(unsigned n, const double *x, double *grad, void *data)
{
    
	double *cx = (double *)data;
	double cosx = ((x[9] - cx[9])*(cx[9] - x[0]) + (x[10] - cx[10])*(cx[10] - x[1]) + (x[11] - cx[11])*(cx[11] - x[2])) / (std::sqrt((x[9] - cx[9])*(x[9] - cx[9]) + (x[10] - cx[10])*(x[10] - cx[10]) + (x[11] - cx[11])*(x[11] - cx[11]))*std::sqrt((cx[9] - x[0]) * (cx[9] - x[0]) + (cx[10] - x[1]) * (cx[10] - x[1]) + (cx[11] - x[2]) * (cx[11] - x[2])));
	if (grad) {
		grad[0] = (-(cx[9] - x[0])*((x[9] - cx[9])*(cx[9] - x[0]) + (x[10] - cx[10])*(cx[10] - x[1]) + (x[11] - cx[11])*(cx[11] - x[2])))
			/ (pow((x[9] - cx[9])*(x[9] - cx[9]) + (x[10] - cx[10])*(x[10] - cx[10]) + (x[11] - cx[11])*(x[11] - cx[11]), 0.5)
				*pow((cx[9] - x[0])*(cx[9] - x[0]) + (cx[10] - x[1])*(cx[10] - x[1]) + (cx[11] - x[2])*(cx[11] - x[2]), 1.5))
			+ (cx[9] - x[9]) / (sqrt((x[9] - cx[9])*(x[9] - cx[9]) + (x[10] - cx[10])*(x[10] - cx[10]) + (x[11] - cx[11])*(x[11] - cx[11]))
				*sqrt((cx[9] - x[0])*(cx[9] - x[0]) + (cx[10] - x[1])*(cx[10] - x[1]) + (cx[11] - x[2])*(cx[11] - x[2])));
		grad[1] = (-(cx[10] - x[1])*((x[9] - cx[9])*(cx[9] - x[0]) + (x[10] - cx[10])*(cx[10] - x[1]) + (x[11] - cx[11])*(cx[11] - x[2])))
			/ (pow((x[9] - cx[9])*(x[9] - cx[9]) + (x[10] - cx[10])*(x[10] - cx[10]) + (x[11] - cx[11])*(x[11] - cx[11]), 0.5)
				*pow((cx[9] - x[0])*(cx[9] - x[0]) + (cx[10] - x[1])*(cx[10] - x[1]) + (cx[11] - x[2])*(cx[11] - x[2]), 1.5))
			+ (cx[10] - x[10]) / (sqrt((x[9] - cx[9])*(x[9] - cx[9]) + (x[10] - cx[10])*(x[10] - cx[10]) + (x[11] - cx[11])*(x[11] - cx[11]))
				*sqrt((cx[9] - x[0])*(cx[9] - x[0]) + (cx[10] - x[1])*(cx[10] - x[1]) + (cx[11] - x[2])*(cx[11] - x[2])));
		grad[2] = (-(cx[11] - x[2])*((x[9] - cx[9])*(cx[9] - x[0]) + (x[10] - cx[10])*(cx[10] - x[1]) + (x[11] - cx[11])*(cx[11] - x[2])))
			/ (pow((x[9] - cx[9])*(x[9] - cx[9]) + (x[10] - cx[10])*(x[10] - cx[10]) + (x[11] - cx[11])*(x[11] - cx[11]), 0.5)
				*pow((cx[9] - x[0])*(cx[9] - x[0]) + (cx[10] - x[1])*(cx[10] - x[1]) + (cx[11] - x[2])*(cx[11] - x[2]), 1.5))
			+ (cx[11] - x[11]) / (sqrt((x[9] - cx[9])*(x[9] - cx[9]) + (x[10] - cx[10])*(x[10] - cx[10]) + (x[11] - cx[11])*(x[11] - cx[11]))
				*sqrt((cx[9] - x[0])*(cx[9] - x[0]) + (cx[10] - x[1])*(cx[10] - x[1]) + (cx[11] - x[2])*(cx[11] - x[2])));
		grad[3] = 0;
		grad[4] = 0;
		grad[5] = 0; 
		grad[6] = 0;
		grad[7] = 0;
		grad[8] = 0;
		grad[9] = (-(x[9] - cx[9])*((x[9] - cx[9])*(cx[9] - x[0]) + (x[10] - cx[10])*(cx[10] - x[1]) + (x[11] - cx[11])*(cx[11] - x[2])))
			/ (pow((x[9] - cx[9])*(x[9] - cx[9]) + (x[10] - cx[10])*(x[10] - cx[10]) + (x[11] - cx[11])*(x[11] - cx[11]), 1.5)
				*sqrt((cx[9] - x[0])*(cx[9] - x[0]) + (cx[10] - x[1])*(cx[10] - x[1]) + (cx[11] - x[2])*(cx[11] - x[2])))
			+ (cx[9] - x[0]) / (sqrt((x[9] - cx[9])*(x[9] - cx[9]) + (x[10] - cx[10])*(x[10] - cx[10]) + (x[11] - cx[11])*(x[11] - cx[11]))
				*sqrt((cx[9] - x[0])*(cx[9] - x[0]) + (cx[10] - x[1])*(cx[10] - x[1]) + (cx[11] - x[2])*(cx[11] - x[2])));
		grad[10] = (-(x[10] - cx[10])*((x[9] - cx[9])*(cx[9] - x[0]) + (x[10] - cx[10])*(cx[10] - x[1]) + (x[11] - cx[11])*(cx[11] - x[2])))
			/ (pow((x[9] - cx[9])*(x[9] - cx[9]) + (x[10] - cx[10])*(x[10] - cx[10]) + (x[11] - cx[11])*(x[11] - cx[11]), 1.5)
				*sqrt((cx[9] - x[0])*(cx[9] - x[0]) + (cx[10] - x[1])*(cx[10] - x[1]) + (cx[11] - x[2])*(cx[11] - x[2])))
			+ (cx[10] - x[1]) / (sqrt((x[9] - cx[9])*(x[9] - cx[9]) + (x[10] - cx[10])*(x[10] - cx[10]) + (x[11] - cx[11])*(x[11] - cx[11]))
				*sqrt((cx[9] - x[0])*(cx[9] - x[0]) + (cx[10] - x[1])*(cx[10] - x[1]) + (cx[11] - x[2])*(cx[11] - x[2])));
		grad[11] = (-(x[11] - cx[11])*((x[9] - cx[9])*(cx[9] - x[0]) + (x[10] - cx[10])*(cx[10] - x[1]) + (x[11] - cx[11])*(cx[11] - x[2])))
			/ (pow((x[9] - cx[9])*(x[9] - cx[9]) + (x[10] - cx[10])*(x[10] - cx[10]) + (x[11] - cx[11])*(x[11] - cx[11]), 1.5)
				*sqrt((cx[9] - x[0])*(cx[9] - x[0]) + (cx[10] - x[1])*(cx[10] - x[1]) + (cx[11] - x[2])*(cx[11] - x[2])))
			+ (cx[11] - x[2]) / (sqrt((x[9] - cx[9])*(x[9] - cx[9]) + (x[10] - cx[10])*(x[10] - cx[10]) + (x[11] - cx[11])*(x[11] - cx[11]))
				*sqrt((cx[9] - x[0])*(cx[9] - x[0]) + (cx[10] - x[1])*(cx[10] - x[1]) + (cx[11] - x[2])*(cx[11] - x[2])));
		

		
	}
	//Vec4 tmp(x[0], x[1], x[2]);
	//Vec4 tmp1(x[3], x[4], x[5]);
	//Vec4 tmp2(cx[0], cx[1], cx[2]);
	////Vec4 tmp(x[0], x[1], x[2]);
	//varray<Vec4> tmpt;
	//tmpt.push_back(tmp);
	//tmpt.push_back(tmp1);
	//tmpt.push_back(tmp2);
	//varray<varray<Vec4>> t; t.push_back(tmpt);
	//RWGeometric rwg;
	//rwg.WritePoint("./g1/tmppts.txt", t);
	return cosx - 1;
}

void continuityModification::ModifyFourChainEndToEnd(conflictCptArr &cfltCptarr,Vec4 *vertex)
{
	cout << "adjustConflictCpt首尾相接(4)" << endl;
	std::vector<Vec4*> boundryCpts, boundryCpts1;
	conflictCpt *tmp = cfltCptarr.m_head;
	int pntSize = tmp->lastCpt.size();
	for (int cptIndex = 0; cptIndex < pntSize; cptIndex++)
	{
		while (tmp) 
		{
			boundryCpts.push_back(tmp->nextCpt[cptIndex]);
			boundryCpts1.push_back(tmp->lastCpt[cptIndex]);
			tmp = tmp->next;
		}
		if (4 == boundryCpts.size())
		{
			EvenModify(*boundryCpts[0], *boundryCpts[2], (*boundryCpts[1] + *boundryCpts[3]) / 2);
			*boundryCpts1[1] = *boundryCpts[0];
			*boundryCpts1[3] = *boundryCpts[2];

			m_fixedPoint.insert(boundryCpts[0]);
			m_fixedPoint.insert(boundryCpts[1]);
			m_fixedPoint.insert(boundryCpts[2]);
			m_fixedPoint.insert(boundryCpts[3]);
		}
		else
		{
			assert(false);
		}

		//非线性优化
		double initCfltCpts[12];//原来的冲突点x,y,z
		const int numVar = cfltCptarr.m_count;

		double tol = 1e-3;//容差
		double lb[12] = { -INF,-INF,-INF,-INF,-INF,-INF,-INF,-INF,-INF,-INF,-INF,-INF };//最小值
		double ub[12] = { INF,INF,INF,INF, INF,INF,INF,INF, INF,INF,INF,INF };//最大值
		double postCfltCpts[12];//未知数求解

		conflictCpt *tmp = cfltCptarr.m_head;
		for (int i = 0; i < numVar; i++) {
			//赋初值
			postCfltCpts[i * 3] = initCfltCpts[i * 3] = (*tmp->cfltCpt[cptIndex]).x;
			postCfltCpts[i * 3 + 1] = initCfltCpts[i * 3 + 1] = (*tmp->cfltCpt[cptIndex]).y;
			postCfltCpts[i * 3 + 2] = initCfltCpts[i * 3 + 2] = (*tmp->cfltCpt[cptIndex]).z;
			tmp = tmp->next;
		}
#if DEBUG == 1
		cout << "initCfltCpts:" << " ";
		for (int i = 0; i < 12; i++) {
			cout << initCfltCpts[i] << " ";
		}
		cout << endl;
#endif
		double cboundryCpts[12];//邻边的控制点xyz
		tmp = cfltCptarr.m_head;
		for (int i = 0; i < numVar; i++) 
		{
			cboundryCpts[i * 3 + 0] = (*tmp->nextCpt[cptIndex]).x;
			cboundryCpts[i * 3 + 1] = (*tmp->nextCpt[cptIndex]).y;
			cboundryCpts[i * 3 + 2] = (*tmp->nextCpt[cptIndex]).z;
			tmp = tmp->next;
		}

		double f_min = INF;//目标函数的值

		// set up optimizer
		nlopt_opt opter = nlopt_create(NLOPT_LD_SLSQP, numVar * 3);

		// lower and upper bound
		nlopt_set_lower_bounds(opter, lb);
		nlopt_set_upper_bounds(opter, ub);

		// objective function
		nlopt_set_min_objective(opter, utility_vol4, initCfltCpts);

		// equality constraint
		nlopt_add_equality_constraint(opter, constraint1_vol4, cboundryCpts, tol);
		nlopt_add_equality_constraint(opter, constraint2_vol4, cboundryCpts, tol);
		nlopt_add_equality_constraint(opter, constraint3_vol4, cboundryCpts, tol);
		nlopt_add_equality_constraint(opter, constraint4_vol4, cboundryCpts, tol);


		// stopping criterion
		nlopt_set_xtol_rel(opter, tol);
		nlopt_set_ftol_abs(opter, tol);
		nlopt_set_force_stop(opter, tol);

		// optimize
		nlopt_result result = nlopt_optimize(opter, postCfltCpts, &f_min);
#if DEBUG == 1
		cout << "postCfltCpts:" << " ";
		for (int i = 0; i < 12; i++) {
			cout << postCfltCpts[i] << " ";
		}
		cout << endl;
#endif
		// free
		nlopt_destroy(opter);

		tmp = cfltCptarr.m_head;
		int cnt = 0;
		while (tmp) 
		{
			tmp->cfltCpt[cptIndex]->x = postCfltCpts[cnt * 3];
			tmp->cfltCpt[cptIndex]->y = postCfltCpts[cnt * 3 + 1];
			tmp->cfltCpt[cptIndex]->z = postCfltCpts[cnt * 3 + 2];
			cnt++;
			m_fixedPoint.insert(tmp->cfltCpt[cptIndex]);
			tmp = tmp->next;
		}
	}
}
void continuityModification::ModifyOddChainEndToEnd(conflictCptArr &cfltCptarr)
{
	const int numVar = cfltCptarr.m_count;
	conflictCpt *tmparr = cfltCptarr.m_head;
	int pntSize = tmparr->lastCpt.size();
	cout << "adjustConflictCpt首尾相接(3/5/7/9)" << endl;
	MatrixXf a(numVar, numVar);//根据首尾相接公式进行矩阵运算
	a.setZero();
	MatrixXf b(numVar, 3);
	b.setZero();
	for (int i = 0; i < numVar; i++) {
		a(i, i) = 1;

		a(i, (i + 1) % numVar) = 1;
	}

	for (int cptIndex = 0; cptIndex < pntSize; cptIndex++)
	{
		int cnt = 0;
		conflictCpt *tmp = cfltCptarr.m_head;
		while (tmp)
		{
			b(cnt, 0) = 2 * (*tmp->nextCpt[cptIndex]).x;
			b(cnt, 1) = 2 * (*tmp->nextCpt[cptIndex]).y;
			b(cnt, 2) = 2 * (*tmp->nextCpt[cptIndex]).z;
			cnt++;
			tmp = tmp->next;
		}
		MatrixXf cc(numVar, numVar + 3);
		cc << a, b;
		FullPivLU<MatrixXf> lu_decomp(cc);
#if DEBUG == 1
		cout << "c的秩" << lu_decomp.rank() << endl;
#endif
		MatrixXf d = cc;
		rowEchelonForm(cc);
#if DEBUG == 1
		cout << "行简化后:\n" << d << endl;
		cout << a << endl << b << endl;
#endif
		tmp = cfltCptarr.m_head;
		cnt = 0;
		MatrixXf c = a.inverse()*b;
		//cout << c << endl;

		while (tmp) 
		{
			if (m_fixedPoint.find(tmp->cfltCpt[cptIndex]) != m_fixedPoint.end())
			{
				tmp = tmp->next;
				continue;
			}
			//将计算后的点写入冲突点
#if DEBUG == 1
			cout << "首尾相接前点： " << (*tmp->cfltCpt[cptIndex]).x << " " << (*tmp->cfltCpt[cptIndex]).y << " " << (*tmp->cfltCpt[cptIndex]).z << endl;
#endif
			(*tmp->cfltCpt[cptIndex]).x = c(cnt, 0);
			(*tmp->cfltCpt[cptIndex]).y = c(cnt, 1);
			(*tmp->cfltCpt[cptIndex]).z = c(cnt, 2);
#if DEBUG == 1
			cout << "首尾相接后点： " << (*tmp->cfltCpt[cptIndex]).x << " " << (*tmp->cfltCpt[cptIndex]).y << " " << (*tmp->cfltCpt[cptIndex]).z << endl;
#endif
			m_fixedPoint.insert(tmp->cfltCpt[cptIndex]);//固定点
			tmp = tmp->next;
			cnt++;
		}
		cout << endl;
	}
}
void continuityModification::ModifyOpenConfilctChain(conflictCptArr &cfltCptarr, int mode, Vec4* vertex)
{
	cout << "adjustConflictCpt首尾不相接" << endl;
	int pntSize = cfltCptarr.m_head->lastCpt.size();
	//varray<varray<Vec4>> p;

	for (int cptIndex = 0; cptIndex < pntSize; cptIndex++)
	{
		switch (mode)
		{
		case 0: 
		{
			//延长调整完的等比例线
			//目前还有一点问题
			conflictCpt *tmp = cfltCptarr.m_head;
			while (tmp)
			{
				if (!tmp->pre) {
					Vec4 p = (*tmp->cfltCpt[cptIndex] + *(tmp->next->cfltCpt[cptIndex]) - 2 * *tmp->nextCpt[cptIndex]) / 2;
					*tmp->cfltCpt[cptIndex] = *tmp->cfltCpt[cptIndex] - p;
					*tmp->next->cfltCpt[cptIndex] -= p;
				}
				else {
					*tmp->cfltCpt[cptIndex] = 2 * *tmp->lastCpt[cptIndex] - *tmp->pre->cfltCpt[cptIndex];
					if (tmp->next) {
						Vec4 pp = *tmp->nextCpt[cptIndex] - *vertex;
						//*tmp->nextCpt[id].Dot(pp);
						Vec4 distance = (-*tmp->lastCpt[cptIndex] + *tmp->cfltCpt[cptIndex]);
						int cnt = 0;
#if DEBUG == 1
						cout << pp.x << " " << pp.y << " " << pp.z << endl;
#endif
						double preDot = 0;
						bool flagDot = 1;
						while (abs((*tmp->cfltCpt[cptIndex] - *tmp->nextCpt[cptIndex]).Dot(pp)) >= 1e-2) {

							preDot = (*tmp->cfltCpt[cptIndex] - *tmp->nextCpt[cptIndex]).Dot(pp);
							if (flagDot)
								*tmp->cfltCpt[cptIndex] = distance * (1 + 0.01*++cnt) + *tmp->lastCpt[cptIndex];
							else {
								*tmp->cfltCpt[cptIndex] = distance * (1 - 0.01*++cnt) + *tmp->lastCpt[cptIndex];
							}
#if DEBUG == 1
							cout << "dot= " << abs((*tmp->cfltCpt[cptIndex] - *tmp->nextCpt[cptIndex]).Dot(pp)) << " " << tmp->cfltCpt[cptIndex]->x << " " << tmp->cfltCpt[cptIndex]->y << " " << tmp->cfltCpt[cptIndex]->z << endl;
#endif
							if (preDot&&abs(preDot - (*tmp->cfltCpt[cptIndex] - *tmp->nextCpt[cptIndex]).Dot(pp)) < 0) {
								flagDot ^= 1;
							}


						}

					}
				}
				m_fixedPoint.insert(tmp->cfltCpt[cptIndex]);
				tmp = tmp->next;
			}
			break;
		}
		case 1: 
		{
			//只有一个点aa的优化
			conflictCpt *cfltNode = cfltCptarr.m_head;
			bool ist = 1;
			Vec4 newCfltCpt;
			int cnt = cfltCptarr.m_count - 1;
			while (cfltNode) {
				if (ist) {
					newCfltCpt += *cfltNode->cfltCpt[cptIndex];
				}
				else {
					newCfltCpt -= *cfltNode->cfltCpt[cptIndex];
				}
				ist ^= 1;
				cfltNode = cfltNode->next;
			}
			cfltNode = cfltCptarr.m_head->next;
			ist = 1;
			while (cnt) {
				if (ist) {
					newCfltCpt += 2 * cnt * *cfltNode->lastCpt[cptIndex];
				}
				else {
					newCfltCpt -= 2 * cnt * *cfltNode->lastCpt[cptIndex];
				}
				ist ^= 1;
				cnt--;
				cfltNode = cfltNode->next;
			}
			newCfltCpt /= cfltCptarr.m_count;
			//并根据求解的第一个点再逐个推导至下个点（2*边界点-冲突点）
			cfltNode = cfltCptarr.m_head;

			while (cfltNode) 
			{
				if (m_fixedPoint.find(cfltNode->cfltCpt[cptIndex]) != m_fixedPoint.end())
				{
					cfltNode = cfltNode->next;
					continue;
				}
#if DEBUG == 1
				cout << "求解前: ";
				cout << (*cfltNode->cfltCpt[cptIndex]).x << ", "
					<< (*cfltNode->cfltCpt[cptIndex]).y << ", "
					<< (*cfltNode->cfltCpt[cptIndex]).z << endl;
#endif
				//pts.push_back(*cfltNode->cfltCpt[cptIndex]);
				if (!cfltNode->pre) 
				{
					*cfltNode->cfltCpt[cptIndex] = newCfltCpt;
				}
				else 
				{
					*cfltNode->cfltCpt[cptIndex] = 2 * (*cfltNode->lastCpt[cptIndex]) - (*(cfltNode->pre->cfltCpt[cptIndex]));
				}
				(*cfltNode->cfltCpt[cptIndex]).w = 1;
				m_plannar_conn++;
#if DEBUG == 1
				cout << "求解后: ";
				cout << (*cfltNode->cfltCpt[cptIndex]).x << ", "
					<< (*cfltNode->cfltCpt[cptIndex]).y << ", "
					<< (*cfltNode->cfltCpt[cptIndex]).z << endl;
				cout << (*cfltNode->lastCpt[cptIndex]).x << ", "
					<< (*cfltNode->lastCpt[cptIndex]).y << ", "
					<< (*cfltNode->lastCpt[cptIndex]).z << endl;
#endif
				m_fixedPoint.insert(cfltNode->cfltCpt[cptIndex]);
				cfltNode = cfltNode->next;
			}	
			cout << "——————————————————————"<<endl;
			break;
		}
		case 2: 
		{
			//多变量的优化
			const int numVar = cfltCptarr.m_count;
			//cout << "numVar= " << numVar << endl;
			double tol = 1e-8;//容差
			double lb[3] = { -INF,-INF,-INF, };//最小值
			double ub[3] = { INF,INF,INF };//最大值
			double x[3], y[3], z[3];//未知数求解

			conflictCpt *tmp = cfltCptarr.m_head;
			for (int i = 0; i < numVar; i++) {
				x[i] = xx[i] = (*tmp->cfltCpt[cptIndex]).x;
				tmp = tmp->next;
			}

			tmp = cfltCptarr.m_head->next;
			double cx[3], cy[3], cz[3];

			for (int i = 0; i < numVar - 1; i++) {
				cx[i] = (*tmp->lastCpt[cptIndex]).x;
				cy[i] = (*tmp->lastCpt[cptIndex]).y;
				cz[i] = (*tmp->lastCpt[cptIndex]).z;
				tmp = tmp->next;
			}

			double f_min = INF;//目标函数的值

			// set up optimizer
			nlopt_opt opter = nlopt_create(NLOPT_LD_SLSQP, numVar);

			// lower and upper bound
			nlopt_set_lower_bounds(opter, lb);
			nlopt_set_upper_bounds(opter, ub);

			// objective function
			nlopt_set_min_objective(opter, utility, cx);

			// equality constraint
			nlopt_add_equality_constraint(opter, constraint1, cx, tol);
			nlopt_add_equality_constraint(opter, constraint2, cx, tol);
			nlopt_add_equality_constraint(opter, constraint3, cx, tol);


			// inequality constraint
			//nlopt_add_inequality_constraint(opter, inconstraint, NULL, tol);

			// stopping criterion
			nlopt_set_xtol_rel(opter, tol);
			nlopt_set_ftol_abs(opter, tol);
			nlopt_set_force_stop(opter, tol);

			// optimize
			nlopt_result result = nlopt_optimize(opter, x, &f_min);

			// free
			nlopt_destroy(opter);
			cout << "Minimun f=" << f_min << " x: ";
			for (int i = 0; i < numVar; i++) {
				cout << x[i] << " ";
			}
			cout << endl;
			//y
			tmp = cfltCptarr.m_head;
			for (int i = 0; i < numVar; i++) {
				y[i] = xx[i] = (*tmp->cfltCpt[cptIndex]).y;
				tmp = tmp->next;
			}

			opter = nlopt_create(NLOPT_LD_SLSQP, numVar);

			// lower and upper bound
			nlopt_set_lower_bounds(opter, lb);
			nlopt_set_upper_bounds(opter, ub);

			// objective function
			nlopt_set_min_objective(opter, utility, cy);

			// equality constraint
			nlopt_add_equality_constraint(opter, constraint1, cy, tol);
			nlopt_add_equality_constraint(opter, constraint2, cy, tol);;
			//nlopt_add_equality_constraint(opter, constraint3, cy, tol);;

			// stopping criterion

			nlopt_set_xtol_rel(opter, tol);
			nlopt_set_ftol_abs(opter, tol);
			nlopt_set_force_stop(opter, tol);

			// optimize
			result = nlopt_optimize(opter, y, &f_min);
			nlopt_destroy(opter);
			cout << "Minimun f=" << f_min << " y: ";
			for (int i = 0; i < numVar; i++) {
				cout << y[i] << " ";
			}
			cout << endl;
			//z
			tmp = cfltCptarr.m_head;
			for (int i = 0; i < numVar; i++) {
				z[i] = xx[i] = (*tmp->cfltCpt[cptIndex]).z;
				tmp = tmp->next;
			}
			opter = nlopt_create(NLOPT_LD_SLSQP, numVar);

			// lower and upper bound
			nlopt_set_lower_bounds(opter, lb);
			nlopt_set_upper_bounds(opter, ub);

			// objective function
			nlopt_set_min_objective(opter, utility, cz);

			// equality constraint
			nlopt_add_equality_constraint(opter, constraint1, cz, tol);
			nlopt_add_equality_constraint(opter, constraint2, cz, tol);
			//nlopt_add_equality_constraint(opter, constraint3, cz, tol);

			// stopping criterion
			nlopt_set_xtol_rel(opter, tol);
			nlopt_set_ftol_abs(opter, tol);
			nlopt_set_force_stop(opter, tol);

			// optimize
			result = nlopt_optimize(opter, z, &f_min);
			nlopt_destroy(opter);
			cout << "Minimun f=" << f_min << " z: ";
			for (int i = 0; i < numVar; i++) {
				cout << z[i] << " ";
			}
			cout << endl;
			//修改冲突点
			tmp = cfltCptarr.m_head;
			for (int i = 0; i < numVar; i++) {

				(*tmp->cfltCpt[cptIndex]).x = x[i];
				(*tmp->cfltCpt[cptIndex]).y = y[i];
				(*tmp->cfltCpt[cptIndex]).z = z[i];
				m_fixedPoint.insert(tmp->cfltCpt[cptIndex]);
				tmp = tmp->next;
			}
			break;
		}
		case 3:
		{
			if (cfltCptarr.m_count == 3) {
				conflictCpt *tmp = cfltCptarr.m_head->next;
				Vec4 vector_v2 = *tmp->cfltCpt[cptIndex] - *tmp->lastCpt[cptIndex], axle_v1v2 = *vertex - *tmp->lastCpt[cptIndex], vector_v1 = *tmp->pre->cfltCpt[cptIndex] - *tmp->lastCpt[cptIndex];
				Vec4 n1 = vector_v1.Cross(axle_v1v2), n2 = axle_v1v2.Cross(vector_v2);

				//cout << n1.Angle(n2) << endl;
				Vec4 new_p2 = glRotatef(*tmp->cfltCpt[cptIndex] - *tmp->lastCpt[cptIndex], axle_v1v2.Normalize(), (n1.Angle(n2)) / 2) + *tmp->lastCpt[cptIndex];
				Vec4 new_p1 = glRotatef(*tmp->pre->cfltCpt[cptIndex] - *tmp->lastCpt[cptIndex], axle_v1v2.Normalize(), (-n1.Angle(n2) / 2)) + *tmp->lastCpt[cptIndex];

				if ((new_p1 - *tmp->lastCpt[cptIndex]).Cross(axle_v1v2).Angle((axle_v1v2).Cross(new_p2 - *tmp->lastCpt[cptIndex])) > 1e-2) {
					new_p2 = glRotatef(*tmp->cfltCpt[cptIndex] - *tmp->lastCpt[cptIndex], axle_v1v2.Normalize(), -n1.Angle(n2) / 2) + *tmp->lastCpt[cptIndex];

				}


				Vec4 vector_v3 = new_p2 - *tmp->nextCpt[cptIndex], axle_v2v3 = *vertex - *tmp->nextCpt[cptIndex], vector_v4 = *tmp->next->cfltCpt[cptIndex] - *tmp->nextCpt[cptIndex];
				Vec4 n3 = vector_v3.Cross(axle_v2v3), n4 = axle_v2v3.Cross(vector_v4);
				*tmp->cfltCpt[cptIndex] = glRotatef(new_p2 - *tmp->nextCpt[cptIndex], axle_v2v3.Normalize(), (n3.Angle(n4)) / 2) + *tmp->nextCpt[cptIndex];
				*tmp->next->cfltCpt[cptIndex] = glRotatef(*tmp->next->cfltCpt[cptIndex] - *tmp->nextCpt[cptIndex], axle_v2v3.Normalize(), (-n3.Angle(n4) / 2)) + *tmp->nextCpt[cptIndex];
				m_fixedPoint.insert(tmp->cfltCpt[cptIndex]);
				m_fixedPoint.insert(tmp->next->cfltCpt[cptIndex]);

				if ((*tmp->cfltCpt[cptIndex] - *tmp->nextCpt[cptIndex]).Cross(axle_v2v3).Angle(axle_v2v3.Cross(*tmp->next->cfltCpt[cptIndex] - *tmp->nextCpt[cptIndex])) > 1e-2) {
					*tmp->cfltCpt[cptIndex] = glRotatef(new_p2 - *tmp->nextCpt[cptIndex], axle_v2v3.Normalize(), (-n3.Angle(n4)) / 2) + *tmp->nextCpt[cptIndex];
					*tmp->next->cfltCpt[cptIndex] = glRotatef(*tmp->next->cfltCpt[cptIndex] - *tmp->nextCpt[cptIndex], axle_v2v3.Normalize(), (n3.Angle(n4))) + *tmp->nextCpt[cptIndex];
				}
				double angle = ((*tmp->cfltCpt[cptIndex] - *tmp->lastCpt[cptIndex]).Cross(axle_v1v2)).Angle(axle_v1v2.Cross(*tmp->pre->cfltCpt[cptIndex] - *tmp->lastCpt[cptIndex]));
				*tmp->pre->cfltCpt[cptIndex] = glRotatef(*tmp->pre->cfltCpt[cptIndex] - *tmp->lastCpt[cptIndex], axle_v1v2.Normalize(), angle) + *tmp->lastCpt[cptIndex];
				if ((*tmp->cfltCpt[cptIndex] - *tmp->lastCpt[cptIndex]).Cross(axle_v1v2).Angle(axle_v1v2.Cross(*tmp->pre->cfltCpt[cptIndex] - *tmp->lastCpt[cptIndex])) > 1e-2) {
					*tmp->pre->cfltCpt[cptIndex] = glRotatef(*tmp->pre->cfltCpt[cptIndex] - *tmp->lastCpt[cptIndex], axle_v1v2.Normalize(), -2 * angle) + *tmp->lastCpt[cptIndex];
				}
				m_fixedPoint.insert(tmp->pre->cfltCpt[cptIndex]);

			}
			break;
		}
		default:
			break;
		}
	}
}

void continuityModification::adjustConflictCpt(conflictCptArr cfltCptarr, Vec4* vertex)
{
	if (cfltCptarr.m_count <= 2)
	{
		return;
	}
	//如果冲突点链表个数大于2，则使用公式进行求解
		//将一圈边界点固定

	conflictCpt *tmparr = cfltCptarr.m_head;

	int pntSize = tmparr->lastCpt.size();
	//对每一圈的共角点冲突点(cfltcpt[id])进行修改

	int mode = 1;
	// 0为延长点至垂直邻边，1为单点求导，2为nlopt求冲突点，3为G1的调整三个体的冲突点
	if (*cfltCptarr.m_head->lastCpt[0] == *cfltCptarr.m_back->nextCpt[0])
	{//首尾相接的情况，但好像会存在求出来的点为inf
		const int numVar = cfltCptarr.m_count;//冲突点个数
		//if(numVar == 3 || numVar == 5 || numVar == 7 || numVar == 9){
		if (numVar == 3 || numVar == 5 || numVar == 7 || numVar == 9)
		{
			ModifyOddChainEndToEnd(cfltCptarr);
		}
		if (numVar == 4)
		{
			//ModifyFourChainEndToEnd(cfltCptarr, vertex);
		}
	}
	else
	{
		ModifyOpenConfilctChain(cfltCptarr, mode, vertex);
	}
}

//TODO
continuityModification::~continuityModification()
{

}