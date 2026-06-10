#pragma once

#include <vector>
#include <array>
#include <memory>
#include <map>
#include <set>
#include <string>
#include <iostream>
#include <stdexcept>
#include <algorithm>
#include <cmath>

#include <TopoDS_Face.hxx>
#include <GeomAbs_CurveType.hxx>
#include <TopoDS_Edge.hxx>
#include <TopoDS_Wire.hxx>
#include <TopExp_Explorer.hxx>
#include <TopAbs.hxx>
#include <TopLoc_Location.hxx>

#include <BRep_Tool.hxx>
#include <BRepTools.hxx>
#include <BRepTools_WireExplorer.hxx>

#include <Geom_Surface.hxx>
#include <Geom_BSplineSurface.hxx>
#include <Geom2d_Curve.hxx>
#include <Geom2d_BSplineCurve.hxx>

#include <gp_Pnt.hxx>
#include <gp_Pnt2d.hxx>
#include <gp_Vec.hxx>
#include <gp_Vec2d.hxx>

#include <TColgp_Array2OfPnt.hxx>
#include <TColStd_Array1OfReal.hxx>
#include <TColStd_Array1OfInteger.hxx>

#include "Polyline2dUtil.h"

enum class DebugStepSegmentType
{
    Bridge,
    Split
};

struct UVPnt2dLess
{
    bool operator()(const gp_Pnt2d& a, const gp_Pnt2d& b) const
    {
        if (a.X() < b.X() - 1.0e-10) return true;
        if (a.X() > b.X() + 1.0e-10) return false;
        return a.Y() < b.Y() - 1.0e-10;
    }
};

enum class UVPointCurveType
{
    Unknown = 0,
    Line,
    Circle,
    BSpline,
    Other,
    Mixed
};

struct DebugStepSegment2d
{
    DebugStepSegmentType type = DebugStepSegmentType::Bridge;
    gp_Pnt2d p0;
    gp_Pnt2d p1;
    std::vector<gp_Pnt2d> polyline;
};

struct DebugLoop2d
{
    std::vector<gp_Pnt2d> points;
    bool isOuter = true;
};

struct DebugFrame
{
    std::vector<DebugLoop2d> loopsBefore;
    std::vector<DebugLoop2d> loopsAfter;
    std::vector<DebugLoop2d> splitChildLoops;
    std::vector<DebugStepSegment2d> segments;
    std::string message;
};

struct BridgeCandidate
{
    gp_Pnt2d holePt;
    gp_Pnt2d outerPt;

    double length = 0.0;
    double outerDiff = 1.0;
    double holeDiff = 1.0;
    double lenScore = 1.0;
    double totalScore = 1.0e100;
};

struct BridgeEndContext
{
    gp_Vec2d prevDir2d;
    gp_Vec2d nextDir2d;
    gp_Vec   prevDir3d;
    gp_Vec   nextDir3d;

    bool has2d = false;
    bool has3d = false;
    bool valid = false;
};

struct UVPointInfo
{
    gp_Pnt2d uv;
    int edgeIndex = -1;
    double edgeParam = 0.0;

    bool isEdgeStart = false;
    bool isEdgeEnd = false;
    bool isTopoVertex = false;
};

class GeomAPI_PointsToBSplineSurface;

namespace OccUntrimUtil
{
    inline double Clamp(double x, double a, double b)
    {
        return std::max(a, std::min(b, x));
    }

    inline double Lerp(double a, double b, double t)
    {
        return a * (1.0 - t) + b * t;
    }

    inline gp_Pnt2d Lerp(const gp_Pnt2d& a, const gp_Pnt2d& b, double t)
    {
        return gp_Pnt2d(
            a.X() * (1.0 - t) + b.X() * t,
            a.Y() * (1.0 - t) + b.Y() * t
        );
    }

    inline gp_Pnt Lerp(const gp_Pnt& a, const gp_Pnt& b, double t)
    {
        return gp_Pnt(
            a.X() * (1.0 - t) + b.X() * t,
            a.Y() * (1.0 - t) + b.Y() * t,
            a.Z() * (1.0 - t) + b.Z() * t
        );
    }

    inline double Distance2d(const gp_Pnt2d& a, const gp_Pnt2d& b)
    {
        const double dx = a.X() - b.X();
        const double dy = a.Y() - b.Y();
        return std::sqrt(dx * dx + dy * dy);
    }

    inline double Distance3d(const gp_Pnt& a, const gp_Pnt& b)
    {
        const double dx = a.X() - b.X();
        const double dy = a.Y() - b.Y();
        const double dz = a.Z() - b.Z();
        return std::sqrt(dx * dx + dy * dy + dz * dz);
    }

    inline double PolylineLength2d(const std::vector<gp_Pnt2d>& pts)
    {
        if (pts.size() < 2) return 0.0;
        double sum = 0.0;
        for (size_t i = 1; i < pts.size(); ++i)
        {
            sum += Distance2d(pts[i - 1], pts[i]);
        }
        return sum;
    }

    gp_Pnt2d SamplePolylineByArcLength(const std::vector<gp_Pnt2d>& polyline, double t01);

    std::vector<gp_Pnt2d> ResamplePolylineUniform(
        const std::vector<gp_Pnt2d>& polyline,
        int sampleCount,
        bool keepEnds = true);

    bool IsClosed(const std::vector<gp_Pnt2d>& pts, double tol = 1e-8);
    double PolygonSignedArea(const std::vector<gp_Pnt2d>& pts);
    void EnsureClosed(std::vector<gp_Pnt2d>& pts, double tol = 1e-8);
    void EnsureCCW(std::vector<gp_Pnt2d>& pts);
    void EnsureCW(std::vector<gp_Pnt2d>& pts);
    gp_Pnt2d ComputeBBoxCenter(const std::vector<gp_Pnt2d>& pts);
    int FindLongestAxis(const std::vector<gp_Pnt2d>& pts);
}

struct UVCurveLoop
{
    std::vector<gp_Pnt2d> points;
    std::vector<UVPointInfo> pointInfos;
    bool isOuter = true;
};

struct MidSurfaceFaceData
{
    TopoDS_Face face;
    Handle(Geom_Surface) surface;

    double uMin = 0.0;
    double uMax = 0.0;
    double vMin = 0.0;
    double vMax = 0.0;

    std::vector<UVCurveLoop> loops;
    bool isBridgeMergedSingleLoop = false;
};

struct UVQuadPatch
{
    std::vector<gp_Pnt2d> south;
    std::vector<gp_Pnt2d> north;
    std::vector<gp_Pnt2d> west;
    std::vector<gp_Pnt2d> east;

    gp_Pnt2d p00;
    gp_Pnt2d p10;
    gp_Pnt2d p01;
    gp_Pnt2d p11;

    bool IsValid() const
    {
        return south.size() >= 2 && north.size() >= 2 &&
            west.size() >= 2 && east.size() >= 2;
    }
};

struct UVPolygonCell
{
    std::vector<gp_Pnt2d> boundary;
};

struct LogicalSide2d
{
    std::vector<gp_Pnt2d> points;
};

struct LogicalPolygon2d
{
    std::vector<int> cornerIndices;
    std::vector<LogicalSide2d> sides;
};

struct PolygonSplitCandidate
{
    int i = -1;
    int j = -1;
    double length = 0.0;
    double lenScore = 1.0;
    double angleScore = 1.0;
    double quadScore = 1.0;
    double totalScore = 1.0e100;
};

struct UntrimBuildOptions
{
    int boundarySampleCountPerEdge = 40;
    int coonsSampleU = 20;
    int coonsSampleV = 20;

    int fitDegreeU = 3;
    int fitDegreeV = 3;

    bool trySplitByHoles = true;
    bool trySplitByIsoLines = false;
    bool useSimpleDominantAxisQuad = true;
    bool verbose = true;
};

struct UntrimBuildResult
{
    std::vector<Handle(Geom_BSplineSurface)> patches;
    std::vector<UVQuadPatch> uvPatches;
    std::vector<DebugFrame> debugFrames;

    bool success = false;
    std::string message;
};

class UVCoonsMapper
{
public:
    UVCoonsMapper() = default;

    bool Build(const UVQuadPatch& patch);
    gp_Pnt2d Evaluate(double xi, double eta) const;
    bool IsReady() const { return m_ready; }

private:
    gp_Pnt2d EvalSouth(double xi) const;
    gp_Pnt2d EvalNorth(double xi) const;
    gp_Pnt2d EvalWest(double eta) const;
    gp_Pnt2d EvalEast(double eta) const;

private:
    UVQuadPatch m_patch;
    bool m_ready = false;
};

class BSplineSurfaceFitter
{
public:
    BSplineSurfaceFitter() = default;

    bool Fit(
        const std::vector<std::vector<gp_Pnt>>& gridPts,
        int degreeU,
        int degreeV,
        Handle(Geom_BSplineSurface)& outSurf,
        std::string* errMsg = nullptr) const;
};

class OccUntrimmingBuilder
{
public:
    explicit OccUntrimmingBuilder(const UntrimBuildOptions& options = UntrimBuildOptions());

    bool Build(const TopoDS_Face& inFace, UntrimBuildResult& outResult);
    bool Build(const std::vector<TopoDS_Face>& inFaces, UntrimBuildResult& outResult);
    const std::map<gp_Pnt2d, UVPointCurveType, UVPnt2dLess>& GetUvPointTypeMap() const;

private:
    bool BuildSingleFace(
        const TopoDS_Face& inFace,
        std::vector<Handle(Geom_BSplineSurface)>& outPatches,
        std::vector<UVQuadPatch>& outUvPatches);

    bool ExtractFaceData(const TopoDS_Face& face, MidSurfaceFaceData& outData);
    bool SplitToSimpleDomains(const MidSurfaceFaceData& inData, std::vector<MidSurfaceFaceData>& outDomains);
    bool BuildUVQuadLayout(const MidSurfaceFaceData& inData, std::vector<UVQuadPatch>& outPatches);
    bool FitSinglePatch(const Handle(Geom_Surface)& surf, const UVQuadPatch& uvPatch, Handle(Geom_BSplineSurface)& outSurf);

private:
    int FindPointOnLoopSegment(const std::vector<gp_Pnt2d>& loop, const gp_Pnt2d& p, double tol = 1e-8) const;
    bool InsertPointIntoLoop(const std::vector<gp_Pnt2d>& loop, const gp_Pnt2d& p, std::vector<gp_Pnt2d>& outLoop, double tol = 1e-8) const;
    std::vector<gp_Pnt2d> BuildBridgePolyline(const gp_Pnt2d& a, const gp_Pnt2d& b, int sampleCount = 6) const;

    std::vector<DebugLoop2d> ConvertToDebugLoops(
        const std::vector<gp_Pnt2d>& outerLoop,
        const std::vector<UVCurveLoop>& holes) const;
    std::vector<DebugLoop2d> ConvertCellsToDebugLoops(
        const std::vector<UVPolygonCell>& cells) const;
    DebugLoop2d ConvertQuadPatchToDebugLoop(const UVQuadPatch& q) const;
    std::vector<DebugLoop2d> ConvertQuadPatchesToDebugLoops(
        const std::vector<UVQuadPatch>& patches) const;

    void EnsureLoopClosedWithInfos(UVCurveLoop& loop, double tol = 1e-8) const;
    void EnsureLoopCCWWithInfos(UVCurveLoop& loop) const;
    void EnsureLoopCWWithInfos(UVCurveLoop& loop) const;

    bool ExtractFaceLoops2d(const TopoDS_Face& face, std::vector<UVCurveLoop>& loops);
    bool ExtractWireLoop2d(
        const TopoDS_Face& face,
        const TopoDS_Wire& wire,
        std::vector<gp_Pnt2d>& outLoop,
        std::vector<UVPointInfo>& outInfos);

    bool SampleEdgePCurve(
        const TopoDS_Face& face,
        const TopoDS_Edge& edge,
        int edgeIndex,
        std::vector<gp_Pnt2d>& outPts,
        std::vector<UVPointInfo>& outInfos,
        int sampleCount);

    bool BridgeAllHolesToOuterLoop(const MidSurfaceFaceData& inData, MidSurfaceFaceData& outData);
    std::vector<int> BuildSparseOuterCandidateIndices(const std::vector<gp_Pnt2d>& outerLoop, int targetCount) const;
    void CollectBridgeCandidates(const std::vector<gp_Pnt2d>& outerLoop, const std::vector<UVCurveLoop>& holes, int holeIndex, std::vector<BridgeCandidate>& outCands);
    bool IsBridgeGeometricallyValid(const gp_Pnt2d& holePt, const gp_Pnt2d& outerPt, const std::vector<gp_Pnt2d>& outerLoop, const std::vector<UVCurveLoop>& holes, int selfHoleIndex) const;
    bool BridgeProducesValidMergedLoop(const std::vector<gp_Pnt2d>& outerLoop, const std::vector<UVCurveLoop>& holes, int holeIndex, const gp_Pnt2d& outerPt, const gp_Pnt2d& holePt, std::vector<gp_Pnt2d>* outMergedLoop);
    double AnglePenalty(const gp_Vec2d& a, const gp_Vec2d& b) const;
    BridgeEndContext GetBridgeEndContext3d(const Handle(Geom_Surface)& surf, const std::vector<gp_Pnt2d>& loop, const gp_Pnt2d& pt, double tol) const;
    BridgeEndContext GetBridgeEndContext(const Handle(Geom_Surface)& surf, const std::vector<gp_Pnt2d>& loop, const gp_Pnt2d& pt, double tol) const;
    double AnglePenalty3d(const gp_Vec& a, const gp_Vec& b) const;
    bool MapUVDirToSurfaceTangent(const Handle(Geom_Surface)& surf, const gp_Pnt2d& uv, const gp_Vec2d& dir2d, gp_Vec& outTangent3d) const;
    bool EvaluateBridgeAngles2dAnd3d(const Handle(Geom_Surface)& surf, const std::vector<gp_Pnt2d>& outerLoop, const std::vector<gp_Pnt2d>& holeLoop, const gp_Pnt2d& outerPt, const gp_Pnt2d& holePt, double& outOuterDiff2d, double& outHoleDiff2d, double& outOuterDiff3d, double& outHoleDiff3d) const;
    bool EvaluateBridgeAngles(const std::vector<gp_Pnt2d>& outerLoop, const std::vector<gp_Pnt2d>& holeLoop, const gp_Pnt2d& outerPt, const gp_Pnt2d& holePt, double& outOuterDiff, double& outHoleDiff) const;
    bool FindValidBridgeToOuter(const Handle(Geom_Surface)& surface, const std::vector<gp_Pnt2d>& outerLoop, const std::vector<UVCurveLoop>& holes, int holeIndex, gp_Pnt2d& outHolePt, gp_Pnt2d& outOuterPt, double* outScore = nullptr);

    bool MergeHoleIntoOuterLoop(const std::vector<gp_Pnt2d>& outerLoop, const std::vector<gp_Pnt2d>& holeLoop, const gp_Pnt2d& holePt, const gp_Pnt2d& outerPt, std::vector<gp_Pnt2d>& mergedLoop);
    int FindClosestPointIndex(const std::vector<gp_Pnt2d>& pts, const gp_Pnt2d& target) const;
    std::vector<gp_Pnt2d> RotateClosedLoopToIndex(const std::vector<gp_Pnt2d>& loop, int startIdx) const;

    bool BuildDominantAxisQuads(const MidSurfaceFaceData& inData, std::vector<UVQuadPatch>& outPatches);
    bool BuildQuadPatchFromPolylineBand(const std::vector<gp_Pnt2d>& lowerCurve, const std::vector<gp_Pnt2d>& upperCurve, double splitParam0, double splitParam1, bool alongU, UVQuadPatch& outPatch);

    bool BuildPolygonQuadsAfterBridge(const MidSurfaceFaceData& inData, std::vector<UVQuadPatch>& outPatches);
    bool SplitBridgeMergedPolygonToCells(const std::vector<gp_Pnt2d>& mergedLoop, std::vector<UVPolygonCell>& outCells);
    bool PolygonCellToQuadPatches(const UVPolygonCell& cell, std::vector<UVQuadPatch>& outPatches);
    bool QuadifyTriangle(const std::vector<gp_Pnt2d>& tri, UVQuadPatch& outPatch);
    bool QuadifyQuad(const std::vector<gp_Pnt2d>& quad, UVQuadPatch& outPatch);
    bool QuadifyPentagon(const std::vector<gp_Pnt2d>& pentagon, std::vector<UVQuadPatch>& outPatches);
    bool SplitGeneralPolygonCell(const std::vector<gp_Pnt2d>& poly, std::vector<UVPolygonCell>& outChildren);
    bool BuildUVQuadFrom4Vertices(const std::array<gp_Pnt2d, 4>& corners, UVQuadPatch& outPatch);
    bool BuildFallbackPatchFromPolygon(const std::vector<gp_Pnt2d>& poly, UVQuadPatch& outPatch) const;

    std::vector<gp_Pnt2d> RemoveDuplicateClosingPoint(const std::vector<gp_Pnt2d>& loop) const;
    std::vector<gp_Pnt2d> RemoveDuplicateClosingPointLocal(const std::vector<gp_Pnt2d>& loop, double tol = 1e-8) const;
    std::vector<double> ComputeVertexTurningAngles(const std::vector<gp_Pnt2d>& loop, double lenTol = 1e-8) const;

    bool CanMergeCornerCandidates(
        int idxA,
        int idxB,
        const std::vector<UVPointInfo>& infos) const;

    std::vector<int> DetectSalientCorners(
        const UVCurveLoop& loopData,
        double angleThresholdDeg = 35.0,
        double lenTol = 1e-8) const;

    std::vector<int> DetectSalientCorners(
        const std::vector<gp_Pnt2d>& poly,
        double angleThresholdDeg = 35.0,
        double lenTol = 1e-8) const;

    std::vector<gp_Pnt2d> ExtractPathBetweenCorners(const std::vector<gp_Pnt2d>& poly, int startIdx, int endIdx) const;

    LogicalPolygon2d ExtractLogicalPolygon(
        const UVCurveLoop& loopData,
        double angleThresholdDeg = 35.0,
        double lenTol = 1e-8) const;

    LogicalPolygon2d ExtractLogicalPolygon(
        const std::vector<gp_Pnt2d>& poly,
        double angleThresholdDeg = 35.0,
        double lenTol = 1e-8) const;

    bool IsQuadDomain(const std::vector<gp_Pnt2d>& poly, double angleThresholdDeg = 25.0, double lenTol = 1e-8) const;

    bool IsTriangleDomain(const std::vector<gp_Pnt2d>& poly, double angleThresholdDeg = 35.0, double lenTol = 1e-8) const;

    std::vector<int> BuildPolygonCandidateIndices(const std::vector<gp_Pnt2d>& poly, int targetCount = 24) const;
    double EvaluatePolygonSplitAngleScore(const std::vector<gp_Pnt2d>& poly, int i, int j) const;
    double EvaluatePolygonSplitQuadScore(const std::vector<gp_Pnt2d>& poly, int i, int j) const;
    void CollectPolygonSplitCandidates(const std::vector<gp_Pnt2d>& poly, std::vector<PolygonSplitCandidate>& outCands) const;
    bool TryPeelQuadByBestSplit(const std::vector<gp_Pnt2d>& poly, std::vector<UVQuadPatch>& outQuadPatches, std::vector<UVPolygonCell>& outNextCells);

    bool SamplePatchOnSurface(const Handle(Geom_Surface)& surf, const UVQuadPatch& uvPatch, int sampleU, int sampleV, std::vector<std::vector<gp_Pnt>>& outGridPts);
    bool ValidateUVPatch(const UVQuadPatch& patch, std::string* errMsg = nullptr) const;
    bool CheckMapperJacobianBySampling(const UVCoonsMapper& mapper, int nu, int nv) const;

    std::vector<Polyline2dUtil::CornerInfo> DetectMergedLoopConcaveCorners(const MidSurfaceFaceData& inData) const;
    void PrintConcaveCorners(const std::vector<Polyline2dUtil::CornerInfo>& corners) const;
    std::vector<Polyline2dUtil::CornerInfo> DetectPolygonConcaveCorners(const std::vector<gp_Pnt2d>& poly) const;
    bool FindBestConcaveSplitDiagonal(const std::vector<gp_Pnt2d>& poly, int& outI, int& outJ) const;
    bool IsValidSplitDiagonal(const std::vector<gp_Pnt2d>& poly, int i, int j) const;
    bool SplitPolygonByDiagonal(const std::vector<gp_Pnt2d>& poly, int i, int j, std::vector<gp_Pnt2d>& outPolyA, std::vector<gp_Pnt2d>& outPolyB) const;
    double EstimateSplitScore(const std::vector<gp_Pnt2d>& poly, int i, int j) const;
    void AddUvPointEdgeRelation(
        const gp_Pnt2d& uv,
        const TopoDS_Edge& edge);
    GeomAbs_CurveType GetEdgeCurveType(const TopoDS_Edge& edge) const;
    UVPointCurveType ToUVPointCurveType(GeomAbs_CurveType type) const;
    void UpdateUvPointType(const gp_Pnt2d& uv, UVPointCurveType newType);
    UVPointCurveType GetUvPointType(const gp_Pnt2d& uv) const;
private:
    UntrimBuildOptions m_opt;
    BSplineSurfaceFitter m_fitter;
    std::vector<DebugFrame> m_debugFrames;
    std::map<gp_Pnt2d, UVPointCurveType, UVPnt2dLess> m_uvPntTypeMap;
};