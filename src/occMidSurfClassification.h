#pragma once

#include"occTypeDefine.h"
#include"occMidSurfGrouper.h"

#include <BVH_Tree.hxx>
#include <TopTools_IndexedDataMapOfShapeListOfShape.hxx>

class TopoDS_Edge;
class TopoDS_Wire;

class MidSurfClassification
{
public:
	MidSurfClassification(TopoFaceArr &orgFacArr);

	bool Generate();

	OccMidSurfGrouper& GetMidSurfGrouper();

	void setThickness(double maxThickness, double minThickness)
	{
		m_minThickness = minThickness;
		m_maxThickness = maxThickness;
	}
	~MidSurfClassification();

	TopoFaceArr& GetOrgFaces()
	{
		return m_orgFacArr;
	}

	GpPntArr m_sdfMidPoints;

	//for test
	TopoFaceArr m_fac;
	std::vector<gp_Pnt> m_points;
	std::vector<TopoDS_Edge> m_edges;
	std::vector< std::vector<int> > testVec;
	std::vector< TopoFaceArr> testClsfyFac;
	std::vector<std::pair<gp_Pnt, gp_Vec>> testArrow;
	std::vector<std::pair<gp_Pnt, std::string>> testString;

private:
	void InitMidSurf();

	bool FindCommonAdjacentFace(const TopoDS_Face& aFac, const TopoDS_Face& bFac, TopoFaceArr& adjFacArr);
	
	bool SortByArea();

	bool IsDihedralAngleNear90(const TopoDS_Face& fac1, const TopoDS_Face& fac2, Standard_Real angMinDeg, Standard_Real angMaxDeg);

	bool IsConvexRelation(const TopoDS_Face& aFac, const TopoDS_Face& adjFac);

	int GetFaceLab(const TopoDS_Face& adjFac);

	bool FindMidFaceLab(const TopoDS_Face& adjFac, UINT& adjFacLab);

	bool CacheMidSurfAreaAndBoundingBox();

	void FindLateralFaces();
	void MergeGroupByThickness();
	void BuildConnRelationBetweenGroups();
	void GroupMidSurf();
	bool BuildOrderedWire(const std::vector<TopoDS_Edge>& edges, TopoDS_Wire& wire);
	bool ExtractGroupOuterWire(const OccMidSurfGroup& group, bool isSideA, std::vector<TopoDS_Edge> &outerEdges);
	bool SortAndOrientEdges(const std::vector<TopoDS_Edge>& inEdges, std::vector<TopoDS_Edge>& outEdges, std::vector<TopoDS_Edge>& noUsedEdges, double tol );
	bool BuildMidBoundaryWire(const OccMidSurfGroup& group, const TopoDS_Wire& aOuterWire, TopoDS_Wire& midWire);
	bool BuildMidBoundaryWire(const OccMidSurfGroup& group, const std::vector<TopoDS_Edge>&outerEdges, std::vector<TopoDS_Edge>& midWire);
	bool FindNearestBFaceForEdge(
		const TopoDS_Edge& edge,
		const std::vector<TopoDS_Face>& bFaces,
		TopoDS_Face& nearestBFace);

	void BuildGroupMidSurfOuterLoop();
	void GetFacDiscretePnt(TopoDS_Face& fac, GpPntArr& pntArr);
	void CollectGroupOuterEdges(const std::vector<TopoDS_Face>& faces, TopTools_ListOfShape& outerEdges);
	void GetFacCenPnt(TopoDS_Face &fac, gp_Pnt& centerPnt);
	void GetFacCenPntAndNrmVec(TopoDS_Face& fac, gp_Pnt& centerPnt, gp_Dir& nrmVec);

	TopoFaceArr m_orgFacArr;
	

	FaceLabSet m_lateralFacLabSet; //所有侧面

	std::vector<double> m_orgFacAreaArr;     // 缓存每个面的面积
	std::vector<Bnd_Box> m_orgFacBoxArr;     // 缓存每个面的面积
	
	OccMidSurfGrouper m_midSurfGrouper;

	double m_minThickness;
	double m_maxThickness;

	TopTools_IndexedDataMapOfShapeListOfShape m_edgeFaceMap;//缓存edge->Face的拓扑关系
	
	enum class ThicknessMatchType
	{
		Invalid = 0,
		Uniform = 1,
		Variable = 2
	};

	struct FaceMatchInfo
	{
		ThicknessMatchType type = ThicknessMatchType::Invalid;

		double avgThickness = 0.0;
		double minThickness = 0.0;
		double maxThickness = 0.0;
		double stdThickness = 0.0;
		double relRange = 0.0;
		double hitRatio = 0.0;

		int hitCount = 0;
		int sampleCount = 0;

		std::vector<gp_Pnt> midPts;
	};


	struct FaceBvhNode
	{
		Bnd_Box box;
		int left = -1;
		int right = -1;

		std::vector<int> faceIds;

		bool IsLeaf() const
		{
			return left < 0 && right < 0;
		}
	};

	//bvh
	std::vector<FaceBvhNode> m_faceBvhNodes;

	void BuildBvh();

	int BuildFaceBvhRecursive(
		std::vector<int>& faceIds,
		int begin,
		int end);

	void QueryFaceBvh(
		int nodeId,
		const Bnd_Box& queryBox,
		std::vector<int>& result) const;

	std::vector<int> QueryCandidateFacesByBvh(
		const Bnd_Box& queryBox) const;

	Bnd_Box MergeFaceBoxes(
		const std::vector<int>& faceIds,
		int begin,
		int end) const;

	double GetBoxCenterCoord(
		const Bnd_Box& box,
		int axis) const;

	int GetLongestAxis(
		const Bnd_Box& box) const;

	ThicknessMatchType JudgeThicknessTypeFromRayHits(
		const std::vector<double>& hitDists,
		int sampleCount,
		FaceMatchInfo& info);

	void BuildGroupsFromMatchMap(
		std::map<int, std::map<int, FaceMatchInfo>>& matchMap,
		ThicknessMatchType groupType);
};

