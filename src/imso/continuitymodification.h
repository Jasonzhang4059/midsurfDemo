#pragma once
#include<SplineSurface.h>
#include<SplineVolume.h>
#include<RWGeometric.h>
#include<map>
#include<set>
#include<NurbsTrans.h>
#include<PolyIGA.h>
#include<fstream>
#include<string>

#include <nlopt.h>
#include <unsupported/Eigen/NonLinearOptimization>
#include <unsupported/Eigen/NumericalDiff>

#define INF (1000000)


/*
    需要将Vec4中的operator==改为下列：
    inline int
        operator == (const Vec4& v1, const Vec4& v2)
    {
        return abs(v1.x - v2.x)<1e-2 && abs(v1.y -v2.y) < 1e-2 && abs(v1.z - v2.z) < 1e-2 && abs(v1.w - v2.w) < 1e-2;
    }
*/

class continuityModification {

//enum
public:
	enum class NurbsVolVertexType {
		FBL = 0,  // Front-Bottom-Left
		FBR = 1,  // Front-Bottom-Right
		FTL = 2,  // Front-Top-Left
		FTR = 3,  // Front-Top-Right
		BBL = 4,  // Back-Bottom-Left
		BBR = 5,  // Back-Bottom-Right
		BTL = 6,  // Back-Top-Left
		BTR = 7,   // Back-Top-Right
		UNKNOWN = 8
	};

	enum CfltPntDirection
	{
		UDirecetion = 0,
		VDirecetion = 1,
		WDirecetion = 2,
		UInvDirecetion = 3,
		VInvDirecetion = 4,
		WInvDirecetion = 5,
		UNKNOWN = 6
	};
	enum DirectionMapping
	{
		U1_to_U2 = 0,    // sv1的U方向对应sv2的U方向
		U1_to_V2 = 1,    // sv1的U方向对应sv2的V方向  
		U1_to_W2 = 2,    // sv1的U方向对应sv2的W方向
		V1_to_U2 = 3,    // sv1的V方向对应sv2的U方向
		V1_to_V2 = 4,    // sv1的V方向对应sv2的V方向
		V1_to_W2 = 5,    // sv1的V方向对应sv2的W方向
		W1_to_U2 = 6,    // sv1的W方向对应sv2的U方向
		W1_to_V2 = 7,    // sv1的W方向对应sv2的V方向
		W1_to_W2 = 8,    // sv1的W方向对应sv2的W方向
		UNKNOWN_DIRECTION = 9
	};

public:
    void adjustVolContinuity();
    continuityModification();
    continuityModification(string path);
	~continuityModification();
	
    
private:
	int cnt=0;
    /*单个冲突点的节点，其中包括
左侧控制点（也就是衔接上一个面片的控制点）lastCpt、
右控制点（衔接下一个面片的控制点）cfltCpt、
冲突点（指针，为修改相应体内的冲突点的指针）、
下一个节点next、
上一个节点pre*/
    class conflictCpt {
    public:
        conflictCpt(vector<Vec4*> &la, vector<Vec4*> &nx, vector<Vec4*>&cfltCpt, conflictCpt * next = nullptr, conflictCpt * pre = nullptr)
            :lastCpt(la), nextCpt(nx), cfltCpt(cfltCpt), next(next), pre(pre) {
        }
        ~conflictCpt() {}
        vector<Vec4*> lastCpt;
        vector<Vec4*> cfltCpt;
        vector<Vec4*> nextCpt;
        conflictCpt *next;
        conflictCpt *pre;
    };

    //冲突点的链表，包括确定方向的控制点axleCpt、头节点head节点、尾插法的back节点、以及计数的count
    class conflictCptArr {
    public:
        Vec4 *m_axleCpt;
        conflictCpt* m_head;
        conflictCpt* m_back;
        //vector<Vec4*> m_vertex;
        int m_count;
        conflictCptArr() :m_head(nullptr), m_back(nullptr), m_count(0) {}
        //插入下个节点（尾插法）
        void push_next(conflictCpt * node) {
            if (!m_back) {
                m_head = node;
                m_back = node;
            }
            else {
                m_back->next = node;
                node->pre = m_back;
                m_back = node;

            }
            m_count++;
        }
        //插入头一个节点
        void push_front(conflictCpt * node) 
		{
            if (m_head) 
			{
                node->next = m_head;
                m_head->pre = node;
                m_head = node;
            }
            else {
                m_head = node;
                m_back = node;
            }
            m_count++;
        }

    };
    varray<SplineSurface> getObjSurface(string path);//obj转varray<SplineSurface>
    void putOutObj(string path,int seg=50);//创建obj文件
    Vec4 glRotatef(const Vec4 &old_point, const Vec4 &axle, double angle);
	int GetNurbsVolVertexIndex(const SplineVolume & sv, const NurbsVolVertexType & verType);
	DirectionMapping GetDirectionMapping(CfltPntDirection outDir1, CfltPntDirection outDir2);
	bool FindMatchingDirections(SplineVolume * sv1, SplineVolume * sv2, const Vec4 & ver,
		DirectionMapping& neiborMapping1, DirectionMapping& neiborMapping2, DirectionMapping& dirctionMapping);
	void ProcessDirections(SplineVolume * sv1, 
		SplineVolume * sv2, 
		DirectionMapping & neiborMapping1,
		DirectionMapping & neiborMapping2,
		DirectionMapping & dirctionMapping,
		const Vec4 & ver);
	void ProcessSplineVolumes(const Vec4 & ver, const std::vector<SplineVolume*>& svArr);
	CfltPntDirection GetAxleDirection(SplineVolume & sv, const Vec4 & vertex, const Vec4* axlePnt);
	//点绕指定方向旋转角度
    void fixPoint();
	void modifyBoundryCfltPts();
	//固定控制点
    void saveModel();//保存模型
	const std::array<Vec4*, 3> FindThreePntArrAroundVer( SplineVolume &sp, const Vec4& pt);//寻找一个体sp内在角点pt周围三个点
    std::vector<Vec4*> GetConflictCptArr(SplineVolume &sv, const Vec4 &p1, const Vec4 *p2, const Vec4 *p3, const Vec4 *axle);//查找冲突点，m_SV为所查找的体，p1为共角点、p2和p3分别是p1附近边界上最近的点，通过这三个点来确定冲突点的位置
    void ModifyNeighborContinuity();
	Vec4& GetNurbsVolVertex(SplineVolume & sv, const NurbsVolVertexType & v);
	const NurbsVolVertexType GetNurbsVolVertexType(const SplineVolume & sv, const Vec4 &testPoint);
	void GetVertexDeltas(const SplineVolume & sv, const NurbsVolVertexType & verType, int & deltaU, int & deltaV, int & deltaW);
	bool CheckNurbsIndexValid(int u, int v, int w, int index);
	//修改内部点
    void calComVerVol();//共角点的体的关系计算
    void createConflictArr();
	void ModifyFourChainEndToEnd(conflictCptArr & cfltCptarr, Vec4 * vertex);
	void ModifyOddChainEndToEnd(conflictCptArr & cfltCptarr);
	void ModifyOpenConfilctChain(conflictCptArr & cfltCptarr, int mode, Vec4 * vertex);
	//创建冲突点链表
    void adjustConflictCpt(conflictCptArr cfltCptarr,  Vec4* vertex);//调整冲突点
    void modifyCfltCptArrountVolver();//共角点周围的冲突点与边界点
	std::vector<Vec4*> GetCfltCptArrAlongDirection(SplineVolume* sv,int conflictIndex, CfltPntDirection dir);//cfltcpt单个结点寻找一排冲突点
	std::vector<Vec4*> GetBoundaryCptArr(SplineVolume &sv, const Vec4 &ver, const Vec4*boundryPnt, const Vec4*axlePnt);//cfltcpt单个结点寻找一排边界点
    int FindPntIndexInNurbsVolume(SplineVolume &sv, const Vec4 p);
	bool FindNurbsVolInnerCfltPnt(SplineVolume & sv, const Vec4& ver, Vec4 *& volCfltPnt);
	const std::array<Vec4, 3> FindThreePntInNeighbourFace(SplineVolume & sv, const Vec4 & pt);
	//寻找点在体内的index
    void putOutVTK(string objpath);//输出vtk模型
    bool EvenModify(Vec4 &p1, Vec4 &p2, Vec4 com);//两边点分别减去(两边点之和-边界点*2)/2
	
private:
	Vec4 m_nullVertex;
	int m_3d_conn;
	int m_composite_conn;
	int m_plannar_conn;
	int mtmp=0;
	set<Vec4> skippedPoints;
    varray<SplineVolume>  m_SV;//修改连续性的体
    RWGeometric m_rwg;//模型读写
    int m_DegreeElevate;//升阶次数
    string m_filename; //文件名
    int m_mode;//G1=0，C1=1
    map<Vec4, std::vector<SplineVolume*>> m_comVertexVolMap;//共角点关系的体
    set<Vec4*> m_fixedPoint;//共角点数量大于2则固定附近控制点
};

class objCp : public CPolyParaVolume {
public:
    objCp& operator=(const varray<NurbsVol> nurbsVols)
    {
        m_HexVolumes.resize(nurbsVols.size());
        for (int i = 0; i != nurbsVols.size(); i++)
        {
            //u节点向量
            for (int j = 0; j != nurbsVols.at(i).m_uKnots.size(); ++j)
            {
                //m_HexVolumes.at(i).m_splVol.m_uKnots.push_back(nurbsVols.at(i).m_uKnots[j]);
                m_HexVolumes.at(i).m_splVol.m_vKnots.push_back(nurbsVols.at(i).m_uKnots[j]);
            }
            //v节点向量
            for (int j = 0; j != nurbsVols.at(i).m_vKnots.size(); ++j)
            {
                //m_HexVolumes.at(i).m_splVol.m_vKnots.push_back(nurbsVols.at(i).m_vKnots[j]);
                m_HexVolumes.at(i).m_splVol.m_uKnots.push_back(nurbsVols.at(i).m_vKnots[j]);
            }
            //w节点向量
            for (int j = 0; j != nurbsVols.at(i).m_wKnots.size(); ++j)
            {
                m_HexVolumes.at(i).m_splVol.m_wKnots.push_back(nurbsVols.at(i).m_wKnots[j]);
            }
            //uvw阶数
            /*m_HexVolumes.at(i).m_splVol.m_uDegree = nurbsVols.at(i).m_uDegree;
            m_HexVolumes.at(i).m_splVol.m_vDegree = nurbsVols.at(i).m_vDegree;*/
            m_HexVolumes.at(i).m_splVol.m_vDegree = nurbsVols.at(i).m_uDegree;
            m_HexVolumes.at(i).m_splVol.m_uDegree = nurbsVols.at(i).m_vDegree;
            m_HexVolumes.at(i).m_splVol.m_wDegree = nurbsVols.at(i).m_wDegree;

            /*m_HexVolumes.at(i).m_splVol.m_uNum = nurbsVols.at(i).m_uNum;
            m_HexVolumes.at(i).m_splVol.m_vNum = nurbsVols.at(i).m_vNum;*/
            m_HexVolumes.at(i).m_splVol.m_vNum = nurbsVols.at(i).m_uNum;
            m_HexVolumes.at(i).m_splVol.m_uNum = nurbsVols.at(i).m_vNum;
            m_HexVolumes.at(i).m_splVol.m_wNum = nurbsVols.at(i).m_wNum;

            //控制点
            for (int j = 0; j != nurbsVols.at(i).m_CtrlPts.size(); ++j)
            {
                VolumeVertex v;
                v.m_pt.x = nurbsVols.at(i).m_CtrlPts[j].x;
                v.m_pt.y = nurbsVols.at(i).m_CtrlPts[j].y;
                v.m_pt.z = nurbsVols.at(i).m_CtrlPts[j].z;
                v.m_pt.w = nurbsVols.at(i).m_CtrlPts[j].w;
                m_HexVolumes.at(i).m_splVol.m_vAllCtrlPts.push_back(v);
            }
        }

        return *this;
    }
    void OutputParaVolumeDataObj(string path, int segmentNum = 40)
    {
        ofstream file(path); //以输出方式打开文件
        //file.open("C:\\r\\hexs.txt");
        //
        int uSegnum, vSegnum, wSegnum, patchPtNumCount, patchNumCount;
        uSegnum = vSegnum = wSegnum = segmentNum;

        patchPtNumCount = ((uSegnum + 1)*(vSegnum + 1) + (uSegnum + 1)*(wSegnum + 1) + (vSegnum + 1)*(wSegnum + 1)) * 2;  //点的数目
        int patchNum = m_HexVolumes.size();
        Vec4 pt, mpt;
        int started = 0;
        file << "g" << "\n";

        for (int patID = 0; patID < patchNum; patID++) {
            SplineVolume& sv = m_HexVolumes.at(patID).m_splVol;
            for (int j = 0; j <= vSegnum; j++)
            {
                for (int i = 0; i <= uSegnum; i++)
                {
                    pt = sv.GetPoint(i*1.f / uSegnum, j*1.f / vSegnum, 0, mpt);
                    file << "v " << pt.x << " " << pt.y << " " << pt.z << " " << "\n";
                }
            }

            for (int j = 0; j <= wSegnum; j++)
            {
                for (int i = 0; i <= uSegnum; i++)
                {
                    pt = sv.GetPoint(i*1.f / uSegnum, 0, j*1.f / wSegnum, mpt);
                    file << "v " << pt.x << " " << pt.y << " " << pt.z << " " << "\n";
                }
            }

            for (int j = 0; j <= wSegnum; j++)
            {
                for (int i = 0; i <= vSegnum; i++)
                {
                    pt = sv.GetPoint(0, i*1.f / vSegnum, j*1.f / wSegnum, mpt);
                    file << "v " << pt.x << " " << pt.y << " " << pt.z << " " << "\n";
                }
            }

            for (int j = 0; j <= vSegnum; j++)
            {
                for (int i = 0; i <= uSegnum; i++)
                {
                    pt = sv.GetPoint(i*1.f / uSegnum, j*1.f / vSegnum, 1, mpt);
                    file << "v " << pt.x << " " << pt.y << " " << pt.z << " " << "\n";
                }
            }

            for (int j = 0; j <= wSegnum; j++)
            {
                for (int i = 0; i <= uSegnum; i++)
                {
                    pt = sv.GetPoint(i*1.f / uSegnum, 1, j*1.f / wSegnum, mpt);
                    file << "v " << pt.x << " " << pt.y << " " << pt.z << " " << "\n";
                }
            }
            for (int j = 0; j <= wSegnum; j++)
            {
                for (int i = 0; i <= vSegnum; i++)
                {
                    pt = sv.GetPoint(1, i*1.f / vSegnum, j*1.f / wSegnum, mpt);
                    file << "v " << pt.x << " " << pt.y << " " << pt.z << " " << "\n";
                }
            }
        }
        for (int patID = 0; patID < patchNum; patID++)
        {
            started = patID * patchPtNumCount + 1;
            for (int k = 0; k < 2; k++)
            {

                for (int j = 0; j < vSegnum; j++)
                {
                    for (int i = 0; i < uSegnum; i++)
                    {
                        file << "f " << started + j * (uSegnum + 1) + i << " "
                            << started + j * (uSegnum + 1) + i + 1 << " "
                            << started + (j + 1) * (uSegnum + 1) + i + 1 << " "
                            << started + (j + 1) * (uSegnum + 1) + i << "\n";
                    }
                }
                started += (uSegnum + 1)*(vSegnum + 1);
                for (int j = 0; j < wSegnum; j++)
                {
                    for (int i = 0; i < uSegnum; i++)
                    {
                        file << "f " << started + j * (uSegnum + 1) + i << " "
                            << started + j * (uSegnum + 1) + i + 1 << " "
                            << started + (j + 1) * (uSegnum + 1) + i + 1 << " "
                            << started + (j + 1) * (uSegnum + 1) + i << "\n";
                    }
                }
                started += (uSegnum + 1)*(wSegnum + 1);
                for (int j = 0; j < wSegnum; j++)
                {
                    for (int i = 0; i < vSegnum; i++)
                    {
                        file << "f " << started + j * (vSegnum + 1) + i << " "
                            << started + j * (vSegnum + 1) + i + 1 << " "
                            << started + (j + 1) * (vSegnum + 1) + i + 1 << " "
                            << started + (j + 1) * (vSegnum + 1) + i << "\n";
                    }
                }
                started += (vSegnum + 1)*(wSegnum + 1);
            }
        }

        file.close();
    }
};