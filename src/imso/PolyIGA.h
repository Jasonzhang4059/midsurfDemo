#pragma once
#include <algorithm>

#include "SplineSurface.h"
#include "SplineVolume.h"
#include "FittingBSpline.h"
#include "varray.h"
#include "definition.h"
#include "XIOStream.h"
#include "CNurbs.h"

using namespace std;
using namespace base;
using namespace bsl;
namespace base{

	
	class CurveEdge
	{
	public:
		int startidx;
		int endidx;
		int icriticalStart;
		int icriticalEnd;
		varray<int>  PassbyidxinOrder;   //路径经过的点的序号。
		int iStatus;
		int persistence;
		bool bVisited;
		bool delflag;
	public:
		CurveEdge();
		~CurveEdge();
		void Clear() { startidx = -1; endidx = -1; PassbyidxinOrder.clear(); };
		bool HasTooPoints() {
			if (PassbyidxinOrder.size() > 100)
				return true;
			return false;
		};
		bool HasLoop()
		{
			for (int i = 0; i < PassbyidxinOrder.size() - 1; i++)
			{
				for (int j = i + 1; j < PassbyidxinOrder.size(); j++)
				{
					if (PassbyidxinOrder.at(i) == PassbyidxinOrder.at(j))
					{
						return true;
					}
				}
			}
			return false;
		};
	};

	

class Sided4Patch  
{
public:
	Sided4Patch();
	~Sided4Patch();
public:
	varray<CurveEdge> m_curveedges;
	varray<int>       m_includeFaceidx;
	XBaseMesh*        m_pnewMs;
	XBaseMesh*        m_pWholeoriMesh;
	SplineSurface     m_4sidedsplsuf;    //此处的spline曲面是旋转之前的。
	bool              m_bHasFitted;
	Vec4              m_geometricCenterPt;
	int               m_twinPatchId;
	bool              m_bEdgeC0ContinuityAdjusted;
	bool              m_bInnerC1ContinuityAdjusted;
private:
	Vec4              m_averageNorm;
	bool              m_bHasComputeAverageNorm;
public:
    void  ConstructBaseMeshFromPatch();
	void  funFitBSplineSurface();
    bool  IstwoPatchHasnorelation(Sided4Patch& sp);
    Vec4  GetAverageNormals();
	void  SetPatchGeometricCenterPt();
private:
	bool  CheckAndMakeValidEdge(varray<varray<Vec4> >& oriedgedata,int& uctrlnum,int& vctrlnum, int degree);
	void  AppendPoints(varray<Vec4>& arr, int ctrlNum);
	//void  OutSurfaceEdgeDataforTest(varray<varray<Vec4> >& edgedata);
};

//每个六面子片
class IGAHexCell    
{
public:
	IGAHexCell();
	~IGAHexCell();
public:
	XBaseMesh*        m_WholeoriMesh;
	varray<int>       m_6PatchIdx;  //三维片有6个片，片的idx在总链表中。排序好的6个片id，01 for u，23 for v, 45 for w;
	SplineVolume      m_splVol;     //转化为体参数化片。
    varray<int>       m_faceAdjacentHexIdx;  //六个面邻接的其他hex。如果没有就置为-1;
    varray<int>       m_controlPtGlobalIDs;  //存储控制点在全局中的编号。初值是0~m_uNum*m_vNum*m_wNum-1
	bool              m_bGenerateGlobalID;   //是否已经生成全局ID。
	bool			  m_isorder = false;			//是否被排序
public:
	void  Clear();
	bool  CreateParametricVolume6BoundarySurface(bool efalg=false); //会根据要求调整每个patch的控制点存放顺序。
	//为生成splinevolume做准备
	int   GetIdxIn6PatchofOneHexforNormAccordingtoAxis(int xyzmode, varray<Sided4Patch>& allPatchs);
	bool  OrderAndInput6Patchs(varray<Sided4Patch>& allPatchs,bool uReverse=false,bool vReverse=false,bool wReverse=false);
	void  Out6SurfaceControlPtData(string& filename);
    bool  GetLocalIDByGlobalID(int& uid,int& vid, int& wid,int globalID);
    int   GetGlobalIDByLocalID(int uid, int vid, int wid);
	Vec4  GetControlPtByGlobalID(int globalID);
};

class CPolyParaVolume
{
public:
	CPolyParaVolume(void);
	~CPolyParaVolume(void);
public:
	//void SetMesh(XBaseMesh* pms);
public:
	XBaseMesh*           m_oriWholeMesh;         //最原始的obj表面模型。
	varray<IGAHexCell>   m_HexVolumes;   //OBJ网格内包含的所有六面体。
	varray<Sided4Patch>  m_all4sidePatch;  //OBJ网格表面所有已划分的四边面片。
    bool                 m_showParaVolume;
	int                  m_testInputNum[2];  //界面传递参数。
	float                m_gmaxJoc, m_gminJoc;
	int                  m_gminorJocNum;
	double               m_gIsovalall;

public:
	
	void  ClearData();
	CPolyParaVolume& operator=(const varray<NurbsVol> nurbsVols);
	void  ReadallPachts(string& filename);  //读入数据，这个是唯一入口，但是文件不明 
	void  ReadVolumeTxt(const string filename);
	int   parse_patch_file(const char *file_name, varray<varray<varray<int> > > &patch_edges, varray<varray<int> > &patch_tris); 
	void  OutPatchsData();//输出的是四篇面片的数据
	void  OutputParaVolumeDataAXL(string filename);
	void  OutputParaVolumeDataTxt(string filename,string Cp);//输出volume体参数化模型的参数（老师给的文件就是这个）
	void  OutputParaVolumeDataVTK(string path);
	void  OutputConstraintsAndLoadByCtrlID(ofstream& idofs);
	void  OutFittedPatchsData(string filename);
	void  Find6ExtremEndsurfaceControlpointIDs(int surfaceID, varray<int>& CtrlptGlobalIDs);
	void  NearIGAHexCellSameUVW(IGAHexCell *StandarHexCell, IGAHexCell *WaitHexCell,int PatchIndx);//参数中的体模型必须是临近的一个模型
	void  Order();//内部排序，指定一个初始的体为基准
	
	//拟合体参数化数据.
	void  CreateParametericPatch();
	void  Create2DParametricPatch();
    void  SetTwinPatch();   //设置数组m_all4sidePatch中的值，为当前每块patch寻找twin片。
	void  SetAdjacentHexIdx();
    void  MakeHexC1Continuity();  //目前关于hex的8个角点处的一阶邻域没有考虑，另外，12条边的一阶邻域边也没有考虑。
	void  MakeHexC0Continuity();  //目前关于hex的8个角点处没有考虑。
	void  MakeHexContinuity();    //构建块连续性
    int   GetHexIdAccordingtoPatchID(int patchid);
public:
	//void  DisplayPolyVolume(int imode);      //显示网格内部所有六面体。
	//void  DisplayBoundary4sidedPatch(int imode);  //显示网格表面已划分四边面。
	bool  GetMirrorIJKOnShareSurface(int volume1, int volume2,int imode,int uid,int vid,int wid,int& muid,int& mvid,int& mwid);
    void  ResetInnerCtrlPoints(int volume1, int volume2,int uid,int vid,int wid,int muid,int mvid,int mwid,Vec4 v0,Vec4 v1,Vec4 v2);
public:
	void  ConstructControlptsIDsforAllBlocks(); //为每个volume中的m_CtrlPtsIDinKKKMatrix重新赋值，即将每个控制点组装到整体中去。
    int   GenerateGlobalIDforOneVolume(int volumeid, int startID);
	void  TestAllblockQuanity();
};
class PolyIGASolution
{
public:
    PolyIGASolution();
	~PolyIGASolution();
public:
	CPolyParaVolume* m_pMultiBlockDomain;
	bool             m_allVolHasSolution;
public:
	void SetMultiVolume(CPolyParaVolume* pBlocks);
    void ReadPatchSolutionData(const char* filename);
    void GenerateSolutionRenderData();
	void GenerateControlpointsDisplacementOrStressColor();  //归一化应力或位移值。
	void ChangeRenderDisplaceOrStress(bool disOrStress); //切换应力和位移的显示。
	void ClearSolutionData();

	/*void RenderDisplacementOrStress(int surfaceMode);*/
};
}