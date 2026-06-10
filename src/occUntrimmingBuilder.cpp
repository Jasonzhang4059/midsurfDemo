#include "OccUntrimmingBuilder.h"

#include <GeomAPI_PointsToBSplineSurface.hxx>
#include <GeomAdaptor_Surface.hxx>
#include <BRepAdaptor_Curve.hxx>
#include <GCPnts_QuasiUniformDeflection.hxx>
#include <BRepAdaptor_Curve2d.hxx>
#include <BRepTools_WireExplorer.hxx>
#include <TopoDS.hxx>
#include <functional>
#include <Geom2dAdaptor_Curve.hxx>
#include <GeomAbs_CurveType.hxx>
#include <set>

// =====================================================
// OccUntrimUtil
// =====================================================
static void ReverseEdgeSamples(
    std::vector<gp_Pnt2d>& pts,
    std::vector<UVPointInfo>& infos)
{
    std::reverse(pts.begin(), pts.end());
    std::reverse(infos.begin(), infos.end());

    for (auto& info : infos)
    {
        std::swap(info.isEdgeStart, info.isEdgeEnd);
    }
}

gp_Pnt2d OccUntrimUtil::SamplePolylineByArcLength(const std::vector<gp_Pnt2d>& polyline, double t01)
{
    if (polyline.empty())
        return gp_Pnt2d(0.0, 0.0);

    if (polyline.size() == 1)
        return polyline.front();

    t01 = Clamp(t01, 0.0, 1.0);

    std::vector<double> acc(polyline.size(), 0.0);
    for (size_t i = 1; i < polyline.size(); ++i)
    {
        acc[i] = acc[i - 1] + Distance2d(polyline[i - 1], polyline[i]);
    }

    const double total = acc.back();
    if (total <= 1e-14)
        return polyline.front();

    const double target = total * t01;

    for (size_t i = 1; i < polyline.size(); ++i)
    {
        if (target <= acc[i])
        {
            const double segLen = acc[i] - acc[i - 1];
            if (segLen <= 1e-14)
                return polyline[i];

            const double localT = (target - acc[i - 1]) / segLen;
            return Lerp(polyline[i - 1], polyline[i], localT);
        }
    }

    return polyline.back();
}

std::vector<gp_Pnt2d> OccUntrimUtil::ResamplePolylineUniform(
    const std::vector<gp_Pnt2d>& polyline,
    int sampleCount,
    bool keepEnds)
{
    std::vector<gp_Pnt2d> out;
    if (polyline.empty() || sampleCount <= 0)
        return out;

    if (sampleCount == 1)
    {
        out.push_back(polyline.front());
        return out;
    }

    out.reserve(static_cast<size_t>(sampleCount));
    for (int i = 0; i < sampleCount; ++i)
    {
        double t = 0.0;
        if (keepEnds)
            t = static_cast<double>(i) / static_cast<double>(sampleCount - 1);
        else
            t = static_cast<double>(i + 1) / static_cast<double>(sampleCount + 1);

        out.push_back(SamplePolylineByArcLength(polyline, t));
    }
    return out;
}

bool OccUntrimUtil::IsClosed(const std::vector<gp_Pnt2d>& pts, double tol)
{
    if (pts.size() < 2) return false;
    return Distance2d(pts.front(), pts.back()) <= tol;
}

double OccUntrimUtil::PolygonSignedArea(const std::vector<gp_Pnt2d>& pts)
{
    if (pts.size() < 3) return 0.0;

    double area = 0.0;
    for (size_t i = 0; i + 1 < pts.size(); ++i)
    {
        area += pts[i].X() * pts[i + 1].Y() - pts[i + 1].X() * pts[i].Y();
    }
    return 0.5 * area;
}

void OccUntrimUtil::EnsureClosed(std::vector<gp_Pnt2d>& pts, double tol)
{
    if (pts.empty()) return;
    if (!IsClosed(pts, tol))
        pts.push_back(pts.front());
}

void OccUntrimUtil::EnsureCCW(std::vector<gp_Pnt2d>& pts)
{
    EnsureClosed(pts);
    if (PolygonSignedArea(pts) < 0.0)
        std::reverse(pts.begin(), pts.end());
}

void OccUntrimUtil::EnsureCW(std::vector<gp_Pnt2d>& pts)
{
    EnsureClosed(pts);
    if (PolygonSignedArea(pts) > 0.0)
        std::reverse(pts.begin(), pts.end());
}

gp_Pnt2d OccUntrimUtil::ComputeBBoxCenter(const std::vector<gp_Pnt2d>& pts)
{
    if (pts.empty()) return gp_Pnt2d(0.0, 0.0);

    double xmin = pts.front().X();
    double xmax = pts.front().X();
    double ymin = pts.front().Y();
    double ymax = pts.front().Y();

    for (const auto& p : pts)
    {
        xmin = std::min(xmin, p.X());
        xmax = std::max(xmax, p.X());
        ymin = std::min(ymin, p.Y());
        ymax = std::max(ymax, p.Y());
    }

    return gp_Pnt2d(0.5 * (xmin + xmax), 0.5 * (ymin + ymax));
}

int OccUntrimUtil::FindLongestAxis(const std::vector<gp_Pnt2d>& pts)
{
    if (pts.empty()) return 0;

    double xmin = pts.front().X();
    double xmax = pts.front().X();
    double ymin = pts.front().Y();
    double ymax = pts.front().Y();

    for (const auto& p : pts)
    {
        xmin = std::min(xmin, p.X());
        xmax = std::max(xmax, p.X());
        ymin = std::min(ymin, p.Y());
        ymax = std::max(ymax, p.Y());
    }

    const double du = xmax - xmin;
    const double dv = ymax - ymin;
    return (du >= dv) ? 0 : 1;
}

// =====================================================
// UVCoonsMapper
// =====================================================

bool UVCoonsMapper::Build(const UVQuadPatch& patch)
{
    if (!patch.IsValid())
        return false;

    m_patch = patch;
    m_ready = true;
    return true;
}

gp_Pnt2d UVCoonsMapper::EvalSouth(double xi) const
{
    return OccUntrimUtil::SamplePolylineByArcLength(m_patch.south, xi);
}

gp_Pnt2d UVCoonsMapper::EvalNorth(double xi) const
{
    return OccUntrimUtil::SamplePolylineByArcLength(m_patch.north, xi);
}

gp_Pnt2d UVCoonsMapper::EvalWest(double eta) const
{
    return OccUntrimUtil::SamplePolylineByArcLength(m_patch.west, eta);
}

gp_Pnt2d UVCoonsMapper::EvalEast(double eta) const
{
    return OccUntrimUtil::SamplePolylineByArcLength(m_patch.east, eta);
}

gp_Pnt2d UVCoonsMapper::Evaluate(double xi, double eta) const
{
    if (!m_ready)
        return gp_Pnt2d(0.0, 0.0);

    xi = OccUntrimUtil::Clamp(xi, 0.0, 1.0);
    eta = OccUntrimUtil::Clamp(eta, 0.0, 1.0);

    const gp_Pnt2d cSouth = EvalSouth(xi);
    const gp_Pnt2d cNorth = EvalNorth(xi);
    const gp_Pnt2d cWest = EvalWest(eta);
    const gp_Pnt2d cEast = EvalEast(eta);

    gp_Pnt2d term1(
        (1.0 - eta) * cSouth.X() + eta * cNorth.X(),
        (1.0 - eta) * cSouth.Y() + eta * cNorth.Y()
    );

    gp_Pnt2d term2(
        (1.0 - xi) * cWest.X() + xi * cEast.X(),
        (1.0 - xi) * cWest.Y() + xi * cEast.Y()
    );

    gp_Pnt2d bilinear(
        (1.0 - xi) * (1.0 - eta) * m_patch.p00.X() +
        xi * (1.0 - eta) * m_patch.p10.X() +
        (1.0 - xi) * eta * m_patch.p01.X() +
        xi * eta * m_patch.p11.X(),

        (1.0 - xi) * (1.0 - eta) * m_patch.p00.Y() +
        xi * (1.0 - eta) * m_patch.p10.Y() +
        (1.0 - xi) * eta * m_patch.p01.Y() +
        xi * eta * m_patch.p11.Y()
    );

    return gp_Pnt2d(
        term1.X() + term2.X() - bilinear.X(),
        term1.Y() + term2.Y() - bilinear.Y()
    );
}

// =====================================================
// BSplineSurfaceFitter
// =====================================================

bool BSplineSurfaceFitter::Fit(
    const std::vector<std::vector<gp_Pnt>>& gridPts,
    int degreeU,
    int degreeV,
    Handle(Geom_BSplineSurface)& outSurf,
    std::string* errMsg) const
{
    outSurf.Nullify();

    if (gridPts.empty() || gridPts.front().empty())
    {
        if (errMsg) *errMsg = "gridPts is empty.";
        return false;
    }

    const int nu = static_cast<int>(gridPts.size());
    const int nv = static_cast<int>(gridPts.front().size());

    for (int i = 1; i < nu; ++i)
    {
        if (static_cast<int>(gridPts[i].size()) != nv)
        {
            if (errMsg) *errMsg = "gridPts row size mismatch.";
            return false;
        }
    }

    if (nu < 2 || nv < 2)
    {
        if (errMsg) *errMsg = "gridPts dimension too small.";
        return false;
    }

    try
    {
        TColgp_Array2OfPnt arr(1, nu, 1, nv);
        for (int i = 0; i < nu; ++i)
        {
            for (int j = 0; j < nv; ++j)
            {
                arr.SetValue(i + 1, j + 1, gridPts[i][j]);
            }
        }

        GeomAPI_PointsToBSplineSurface builder(
            arr,
            3,
            std::max(3, std::max(degreeU, degreeV)),
            GeomAbs_C2,
            1.0e-3
        );

        outSurf = builder.Surface();
        if (outSurf.IsNull())
        {
            if (errMsg) *errMsg = "GeomAPI_PointsToBSplineSurface failed.";
            return false;
        }
    }
    catch (const Standard_Failure& e)
    {
        if (errMsg) *errMsg = std::string("OCC exception: ") + e.GetMessageString();
        return false;
    }
    catch (const std::exception& e)
    {
        if (errMsg) *errMsg = std::string("std exception: ") + e.what();
        return false;
    }

    return true;
}

// =====================================================
// OccUntrimmingBuilder
// =====================================================

OccUntrimmingBuilder::OccUntrimmingBuilder(const UntrimBuildOptions& options)
    : m_opt(options)
{
}

bool OccUntrimmingBuilder::Build(
    const TopoDS_Face& inFace,
    UntrimBuildResult& outResult)
{
    outResult = UntrimBuildResult();
    m_debugFrames.clear();

    try
    {
        std::vector<Handle(Geom_BSplineSurface)> patches;
        std::vector<UVQuadPatch> uvPatches;

        const bool ok = BuildSingleFace(inFace, patches, uvPatches);
        outResult.success = ok;
        outResult.patches = std::move(patches);
        outResult.uvPatches = std::move(uvPatches);
        outResult.message = ok ? "Build succeeded." : "BuildSingleFace failed.";
        outResult.debugFrames = m_debugFrames;
        return ok;
    }
    catch (const Standard_Failure& e)
    {
        outResult.success = false;
        outResult.message = std::string("OCC exception: ") + e.GetMessageString();
        return false;
    }
    catch (const std::exception& e)
    {
        outResult.success = false;
        outResult.message = std::string("std exception: ") + e.what();
        return false;
    }
}

bool OccUntrimmingBuilder::Build(
    const std::vector<TopoDS_Face>& inFaces,
    UntrimBuildResult& outResult)
{
    outResult = UntrimBuildResult();
    m_debugFrames.clear();

    bool allOk = true;

    for (const auto& face : inFaces)
    {
        std::vector<Handle(Geom_BSplineSurface)> onePatches;
        std::vector<UVQuadPatch> oneUvPatches;

        const bool ok = BuildSingleFace(face, onePatches, oneUvPatches);
        if (!ok)
            allOk = false;

        outResult.patches.insert(outResult.patches.end(), onePatches.begin(), onePatches.end());
        outResult.uvPatches.insert(outResult.uvPatches.end(), oneUvPatches.begin(), oneUvPatches.end());
    }

    outResult.success = allOk;
    outResult.message = allOk ? "All faces processed." : "Some faces failed.";
    outResult.debugFrames = m_debugFrames;
    return allOk;
}

bool OccUntrimmingBuilder::BuildSingleFace(
    const TopoDS_Face& inFace,
    std::vector<Handle(Geom_BSplineSurface)>& outPatches,
    std::vector<UVQuadPatch>& outUvPatches)
{
    outPatches.clear();
    outUvPatches.clear();

    MidSurfaceFaceData faceData;
    if (!ExtractFaceData(inFace, faceData))
        return false;

    std::vector<MidSurfaceFaceData> simpleDomains;
    if (!SplitToSimpleDomains(faceData, simpleDomains))
        return false;

    MidSurfaceFaceData domain;
    if (!simpleDomains.empty())
        domain = simpleDomains.front();
    else
        domain = faceData;

    //// bridge ϲĸӵɢǼ
    //if (domain.isBridgeMergedSingleLoop)
    //{
    //    const auto concaveCorners = DetectMergedLoopConcaveCorners(domain);

    //    if (m_opt.verbose)
    //        PrintConcaveCorners(concaveCorners);

    //    // ԰ concaveCorners Լı߷ֿ麯
    //    // ǰȲı BuildUVQuadLayout ǩֻӡ
    //}

    std::vector<UVQuadPatch> uvPatches;
    if (!BuildUVQuadLayout(domain, uvPatches))
        return false;

    for (const auto& uvPatch : uvPatches)
    {
        Handle(Geom_BSplineSurface) surfPatch;
        if (!FitSinglePatch(domain.surface, uvPatch, surfPatch))
            continue;

        outPatches.push_back(surfPatch);
        outUvPatches.push_back(uvPatch);
    }

    return !outPatches.empty();
}

std::vector<DebugLoop2d> OccUntrimmingBuilder::ConvertToDebugLoops(
    const std::vector<gp_Pnt2d>& outerLoop,
    const std::vector<UVCurveLoop>& holes) const
{
    std::vector<DebugLoop2d> out;

    if (!outerLoop.empty())
    {
        DebugLoop2d outer;
        outer.points = outerLoop;
        outer.isOuter = true;
        out.push_back(outer);
    }

    for (const auto& h : holes)
    {
        DebugLoop2d dh;
        dh.points = h.points;
        dh.isOuter = false;
        out.push_back(dh);
    }

    return out;
}

bool OccUntrimmingBuilder::ExtractFaceData(
    const TopoDS_Face& face,
    MidSurfaceFaceData& outData)
{
    outData = MidSurfaceFaceData();
    outData.face = face;
    outData.surface = BRep_Tool::Surface(face);

    if (outData.surface.IsNull())
        return false;

    BRepTools::UVBounds(face, outData.uMin, outData.uMax, outData.vMin, outData.vMax);

    if (!ExtractFaceLoops2d(face, outData.loops))
        return false;

    if (outData.loops.empty())
        return false;

    return true;
}

bool OccUntrimmingBuilder::SplitToSimpleDomains(
    const MidSurfaceFaceData& inData,
    std::vector<MidSurfaceFaceData>& outDomains)
{
    outDomains.clear();

    int holeCount = 0;
    for (const auto& loop : inData.loops)
    {
        if (!loop.isOuter)
            ++holeCount;
    }

    if (holeCount == 0)
    {
        outDomains.push_back(inData);
        return true;
    }

    MidSurfaceFaceData simpleData;
    if (!BridgeAllHolesToOuterLoop(inData, simpleData))
    {
        if (m_opt.verbose)
            std::cout << "[OccUntrimmingBuilder] BridgeAllHolesToOuterLoop failed." << std::endl;
        return false;
    }

    outDomains.push_back(simpleData);
    return true;
}

bool OccUntrimmingBuilder::BridgeAllHolesToOuterLoop(
    const MidSurfaceFaceData& inData,
    MidSurfaceFaceData& outData)
{
    outData = inData;
    outData.loops.clear();

    std::vector<gp_Pnt2d> outerLoop;
    std::vector<UVCurveLoop> holes;

    for (const auto& loop : inData.loops)
    {
        if (loop.isOuter)
            outerLoop = loop.points;
        else
            holes.push_back(loop);
    }

    if (outerLoop.empty())
        return false;

    OccUntrimUtil::EnsureClosed(outerLoop);

    while (!holes.empty())
    {
        int bestHoleIdx = -1;
        gp_Pnt2d bestHolePt, bestOuterPt;
        double bestScore = 1.0e100;

        for (int i = 0; i < static_cast<int>(holes.size()); ++i)
        {
            gp_Pnt2d holePt, outerPt;
            double score = 1.0e100;
            if (!FindValidBridgeToOuter(inData.surface, outerLoop, holes, i, holePt, outerPt, &score))
                continue;

            if (score < bestScore)
            {
                bestScore = score;
                bestHoleIdx = i;
                bestHolePt = holePt;
                bestOuterPt = outerPt;
            }
        }

        if (bestHoleIdx < 0)
        {
            if (m_opt.verbose)
                std::cout << "[OccUntrimmingBuilder] No valid bridge found for remaining holes." << std::endl;
            return false;
        }

        DebugFrame frame;
        frame.loopsBefore = ConvertToDebugLoops(outerLoop, holes);
        frame.loopsAfter.clear();
        frame.splitChildLoops.clear();
        frame.message = "bridge hole to outer";

        DebugStepSegment2d seg;
        seg.type = DebugStepSegmentType::Bridge;
        seg.p0 = bestOuterPt;
        seg.p1 = bestHolePt;
        seg.polyline = BuildBridgePolyline(bestOuterPt, bestHolePt, 6);
        frame.segments.push_back(seg);

        std::vector<gp_Pnt2d> mergedLoop;
        if (!MergeHoleIntoOuterLoop(
            outerLoop,
            holes[bestHoleIdx].points,
            bestHolePt,
            bestOuterPt,
            mergedLoop))
        {
            return false;
        }
        UpdateUvPointType(bestHolePt, UVPointCurveType::Line);
        UpdateUvPointType(bestOuterPt, UVPointCurveType::Line);
        std::vector<UVCurveLoop> holesAfter = holes;
        holesAfter.erase(holesAfter.begin() + bestHoleIdx);

        frame.loopsAfter = ConvertToDebugLoops(mergedLoop, holesAfter);
        m_debugFrames.push_back(frame);

        outerLoop = mergedLoop;
        holes = holesAfter;
    }

    OccUntrimUtil::EnsureClosed(outerLoop);
    OccUntrimUtil::EnsureCCW(outerLoop);

    UVCurveLoop finalLoop;
    finalLoop.points = outerLoop;
    finalLoop.isOuter = true;

    outData.loops.push_back(finalLoop);
    outData.isBridgeMergedSingleLoop = true;
    return true;
}


namespace
{
    static bool NormalizeVec2d(gp_Vec2d& v, double tol = 1.0e-12)
    {
        const double len = v.Magnitude();
        if (len < tol)
            return false;
        v /= len;
        return true;
    }

    static void AppendUniqueIndices(std::vector<int>& dst, const std::vector<int>& src)
    {
        std::set<int> seen(dst.begin(), dst.end());
        for (int v : src)
        {
            if (seen.insert(v).second)
                dst.push_back(v);
        }
    }
}

std::vector<int> OccUntrimmingBuilder::BuildSparseOuterCandidateIndices(
    const std::vector<gp_Pnt2d>& outerLoop,
    int targetCount) const
{
    std::vector<int> idxs;

    const int n = static_cast<int>(outerLoop.size());
    if (n < 2)
        return idxs;

    const int m =
        (OccUntrimUtil::Distance2d(outerLoop.front(), outerLoop.back()) < 1.0e-8)
        ? (n - 1) : n;

    if (m <= 0)
        return idxs;

    targetCount = std::max(4, targetCount);

    if (m <= targetCount)
    {
        for (int i = 0; i < m; ++i)
            idxs.push_back(i);
        return idxs;
    }

    const int step = std::max(1, m / targetCount);
    for (int i = 0; i < m; i += step)
        idxs.push_back(i);

    if (!idxs.empty() && idxs.back() != m - 1)
        idxs.push_back(m - 1);

    return idxs;
}

bool OccUntrimmingBuilder::IsBridgeGeometricallyValid(
    const gp_Pnt2d& holePt,
    const gp_Pnt2d& outerPt,
    const std::vector<gp_Pnt2d>& outerLoop,
    const std::vector<UVCurveLoop>& holes,
    int selfHoleIndex) const
{
    std::vector<std::vector<gp_Pnt2d>> holeLoops;
    holeLoops.reserve(holes.size());
    for (const auto& h : holes)
        holeLoops.push_back(h.points);

    if (Polyline2dUtil::SegmentHitsAnyLoopExceptSelf(
        holePt, outerPt, holeLoops, selfHoleIndex, 1.0e-8))
    {
        return false;
    }

    const gp_Pnt2d mid(
        0.5 * (holePt.X() + outerPt.X()),
        0.5 * (holePt.Y() + outerPt.Y()));

    if (!Polyline2dUtil::PointInPolygon(mid, outerLoop, 1.0e-8))
        return false;

    return true;
}

bool OccUntrimmingBuilder::BridgeProducesValidMergedLoop(
    const std::vector<gp_Pnt2d>& outerLoop,
    const std::vector<UVCurveLoop>& holes,
    int holeIndex,
    const gp_Pnt2d& outerPt,
    const gp_Pnt2d& holePt,
    std::vector<gp_Pnt2d>* outMergedLoop)
{
    if (holeIndex < 0 || holeIndex >= static_cast<int>(holes.size()))
        return false;

    const auto& holeLoop = holes[holeIndex].points;

    std::vector<gp_Pnt2d> mergedLoop;
    if (!MergeHoleIntoOuterLoop(outerLoop, holeLoop, holePt, outerPt, mergedLoop))
        return false;

    OccUntrimUtil::EnsureClosed(mergedLoop);

    if (mergedLoop.size() < 4)
        return false;

    const double area = std::abs(OccUntrimUtil::PolygonSignedArea(mergedLoop));
    if (area < 1.0e-8)
        return false;

    for (int i = 0; i < static_cast<int>(holes.size()); ++i)
    {
        if (i == holeIndex)
            continue;

        const gp_Pnt2d c = OccUntrimUtil::ComputeBBoxCenter(holes[i].points);
        if (!Polyline2dUtil::PointInPolygon(c, mergedLoop, 1.0e-8))
            return false;
    }

    if (outMergedLoop)
        *outMergedLoop = mergedLoop;

    return true;
}

double OccUntrimmingBuilder::AnglePenalty(
    const gp_Vec2d& a,
    const gp_Vec2d& b) const
{
    gp_Vec2d aa = a;
    gp_Vec2d bb = b;

    if (!NormalizeVec2d(aa) || !NormalizeVec2d(bb))
        return 1.0;

    double dot = aa.X() * bb.X() + aa.Y() * bb.Y();
    dot = std::max(-1.0, std::min(1.0, dot));

    const double ang = std::acos(dot);
    const double PI = 3.14159265358979323846;

    return ang / PI;
}
BridgeEndContext OccUntrimmingBuilder::GetBridgeEndContext3d(
    const Handle(Geom_Surface)& surf,
    const std::vector<gp_Pnt2d>& loop,
    const gp_Pnt2d& pt,
    double tol) const
{
    BridgeEndContext ctx;

    if (surf.IsNull())
        return ctx;

    std::vector<gp_Pnt2d> inserted;
    if (!InsertPointIntoLoop(loop, pt, inserted, tol))
        return ctx;

    OccUntrimUtil::EnsureClosed(inserted);

    const int idx = FindClosestPointIndex(inserted, pt);
    if (idx < 0)
        return ctx;

    const int n = static_cast<int>(inserted.size());
    if (n < 4)
        return ctx;

    int prev = idx - 1;
    int next = idx + 1;

    if (idx == 0)
        prev = n - 2;
    if (next >= n - 1)
        next = 1;

    const gp_Pnt2d& pPrev = inserted[prev];
    const gp_Pnt2d& pCurr = inserted[idx];
    const gp_Pnt2d& pNext = inserted[next];

    gp_Vec2d vPrev2d(pCurr, pPrev);
    gp_Vec2d vNext2d(pCurr, pNext);

    if (!NormalizeVec2d(vPrev2d) || !NormalizeVec2d(vNext2d))
        return ctx;

    gp_Vec prev3d, next3d;
    if (!MapUVDirToSurfaceTangent(surf, pCurr, vPrev2d, prev3d))
        return ctx;
    if (!MapUVDirToSurfaceTangent(surf, pCurr, vNext2d, next3d))
        return ctx;

    ctx.prevDir2d = vPrev2d;
    ctx.nextDir2d = vNext2d;
    ctx.prevDir3d = prev3d;
    ctx.nextDir3d = next3d;
    ctx.has2d = true;
    ctx.has3d = true;
    ctx.valid = true;
    return ctx;
}

static bool NormalizeVec3d(gp_Vec& v, double tol = 1.0e-12)
{
    const double len = v.Magnitude();
    if (len < tol)
        return false;
    v /= len;
    return true;
}

BridgeEndContext OccUntrimmingBuilder::GetBridgeEndContext(
    const Handle(Geom_Surface)& surf,
    const std::vector<gp_Pnt2d>& loop,
    const gp_Pnt2d& pt,
    double tol) const
{
    BridgeEndContext ctx;

    std::vector<gp_Pnt2d> inserted;
    if (!InsertPointIntoLoop(loop, pt, inserted, tol))
        return ctx;

    OccUntrimUtil::EnsureClosed(inserted);

    const int idx = FindClosestPointIndex(inserted, pt);
    if (idx < 0)
        return ctx;

    const int n = static_cast<int>(inserted.size());
    if (n < 4)
        return ctx;

    int prev = idx - 1;
    int next = idx + 1;

    if (idx == 0)
        prev = n - 2;
    if (next >= n - 1)
        next = 1;

    const gp_Pnt2d& pPrev = inserted[prev];
    const gp_Pnt2d& pCurr = inserted[idx];
    const gp_Pnt2d& pNext = inserted[next];

    // ===== 2D =====
    gp_Vec2d vPrev2d(pCurr, pPrev);
    gp_Vec2d vNext2d(pCurr, pNext);

    if (!NormalizeVec2d(vPrev2d) || !NormalizeVec2d(vNext2d))
        return ctx;

    ctx.prevDir2d = vPrev2d;
    ctx.nextDir2d = vNext2d;
    ctx.has2d = true;

    // ===== 3D =====
    if (!surf.IsNull())
    {
        gp_Pnt prev3d, curr3d, next3d;
        surf->D0(pPrev.X(), pPrev.Y(), prev3d);
        surf->D0(pCurr.X(), pCurr.Y(), curr3d);
        surf->D0(pNext.X(), pNext.Y(), next3d);

        gp_Vec vPrev3d(curr3d, prev3d);
        gp_Vec vNext3d(curr3d, next3d);

        if (NormalizeVec3d(vPrev3d) && NormalizeVec3d(vNext3d))
        {
            ctx.prevDir3d = vPrev3d;
            ctx.nextDir3d = vNext3d;
            ctx.has3d = true;
        }
    }

    ctx.valid = true;
    return ctx;
}

double OccUntrimmingBuilder::AnglePenalty3d(
    const gp_Vec& a,
    const gp_Vec& b) const
{
    gp_Vec aa = a;
    gp_Vec bb = b;

    if (!NormalizeVec3d(aa) || !NormalizeVec3d(bb))
        return 1.0;

    double dot = aa.Dot(bb);
    dot = std::max(-1.0, std::min(1.0, dot));

    const double ang = std::acos(dot);
    const double PI = 3.14159265358979323846;

    return ang / PI;
}
bool OccUntrimmingBuilder::MapUVDirToSurfaceTangent(
    const Handle(Geom_Surface)& surf,
    const gp_Pnt2d& uv,
    const gp_Vec2d& dir2d,
    gp_Vec& outTangent3d) const
{
    outTangent3d = gp_Vec(0.0, 0.0, 0.0);

    if (surf.IsNull())
        return false;

    gp_Pnt p;
    gp_Vec du, dv;
    surf->D1(uv.X(), uv.Y(), p, du, dv);

    gp_Vec result =
        du.Multiplied(dir2d.X()) +
        dv.Multiplied(dir2d.Y());

    if (!NormalizeVec3d(result))
        return false;

    outTangent3d = result;
    return true;
}
bool OccUntrimmingBuilder::EvaluateBridgeAngles2dAnd3d(
    const Handle(Geom_Surface)& surf,
    const std::vector<gp_Pnt2d>& outerLoop,
    const std::vector<gp_Pnt2d>& holeLoop,
    const gp_Pnt2d& outerPt,
    const gp_Pnt2d& holePt,
    double& outOuterDiff2d,
    double& outHoleDiff2d,
    double& outOuterDiff3d,
    double& outHoleDiff3d) const
{
    outOuterDiff2d = 1.0;
    outHoleDiff2d = 1.0;
    outOuterDiff3d = 1.0;
    outHoleDiff3d = 1.0;

    const double tol = 1.0e-8;

    BridgeEndContext outerCtx = GetBridgeEndContext(surf, outerLoop, outerPt, tol);
    BridgeEndContext holeCtx = GetBridgeEndContext(surf, holeLoop, holePt, tol);

    if (!outerCtx.valid || !holeCtx.valid || !outerCtx.has2d || !holeCtx.has2d)
        return false;

    gp_Vec2d dOuter2d(outerPt, holePt);
    gp_Vec2d dHole2d(holePt, outerPt);

    if (!NormalizeVec2d(dOuter2d) || !NormalizeVec2d(dHole2d))
        return false;

    // ===== 2D =====
    const double a1 = AnglePenalty(dOuter2d, outerCtx.prevDir2d);
    const double a2 = AnglePenalty(dOuter2d, outerCtx.nextDir2d);
    const double a3 = AnglePenalty(dHole2d, holeCtx.prevDir2d);
    const double a4 = AnglePenalty(dHole2d, holeCtx.nextDir2d);

    outOuterDiff2d = std::abs(a1 - a2);
    outHoleDiff2d = std::abs(a3 - a4);

    // ===== 3D =====
    if (!outerCtx.has3d || !holeCtx.has3d)
        return true;

    gp_Pnt outerP3d, holeP3d;
    surf->D0(outerPt.X(), outerPt.Y(), outerP3d);
    surf->D0(holePt.X(), holePt.Y(), holeP3d);

    gp_Vec dOuter3d(outerP3d, holeP3d);
    gp_Vec dHole3d(holeP3d, outerP3d);

    if (!NormalizeVec3d(dOuter3d) || !NormalizeVec3d(dHole3d))
        return true;

    // 这里直接使用 context 里已经算好的 3D 邻接方向
    const double b1 = AnglePenalty3d(dOuter3d, outerCtx.prevDir3d);
    const double b2 = AnglePenalty3d(dOuter3d, outerCtx.nextDir3d);
    const double b3 = AnglePenalty3d(dHole3d, holeCtx.prevDir3d);
    const double b4 = AnglePenalty3d(dHole3d, holeCtx.nextDir3d);

    outOuterDiff3d = std::abs(b1 - b2);
    outHoleDiff3d = std::abs(b3 - b4);

    return true;
}
bool OccUntrimmingBuilder::EvaluateBridgeAngles(
    const std::vector<gp_Pnt2d>& outerLoop,
    const std::vector<gp_Pnt2d>& holeLoop,
    const gp_Pnt2d& outerPt,
    const gp_Pnt2d& holePt,
    double& outOuterDiff,
    double& outHoleDiff) const
{
    outOuterDiff = 1.0;
    outHoleDiff = 1.0;

    const double tol = 1.0e-8;
    const Handle(Geom_Surface) surf;
    BridgeEndContext outerCtx = GetBridgeEndContext(surf,outerLoop, outerPt, tol);
    BridgeEndContext holeCtx = GetBridgeEndContext(surf,holeLoop, holePt, tol);

    if (!outerCtx.valid || !holeCtx.valid)
        return false;

    gp_Vec2d dOuter(outerPt, holePt);
    gp_Vec2d dHole(holePt, outerPt);

    if (!NormalizeVec2d(dOuter) || !NormalizeVec2d(dHole))
        return false;

    const double a1 = AnglePenalty(dOuter, outerCtx.prevDir2d);
    const double a2 = AnglePenalty(dOuter, outerCtx.nextDir2d);
    const double a3 = AnglePenalty(dHole, holeCtx.prevDir2d);
    const double a4 = AnglePenalty(dHole, holeCtx.nextDir2d);

    outOuterDiff = std::abs(a1 - a2);
    outHoleDiff = std::abs(a3 - a4);
    return true;
}

void OccUntrimmingBuilder::CollectBridgeCandidates(
    const std::vector<gp_Pnt2d>& outerLoop,
    const std::vector<UVCurveLoop>& holes,
    int holeIndex,
    std::vector<BridgeCandidate>& outCands)
{
    outCands.clear();

    if (holeIndex < 0 || holeIndex >= static_cast<int>(holes.size()))
        return;

    const auto& hole = holes[holeIndex].points;
    if (hole.size() < 2 || outerLoop.size() < 2)
        return;

    std::vector<int> holeIdx = Polyline2dUtil::BuildExtremeCandidateIndices(hole);

    {
        const int n = static_cast<int>(hole.size());
        const int m =
            (n >= 2 && OccUntrimUtil::Distance2d(hole.front(), hole.back()) < 1.0e-8)
            ? (n - 1) : n;

        if (m > 0)
        {
            const int step = std::max(1, m / 12);
            std::vector<int> extra;
            for (int i = 0; i < m; i += step)
                extra.push_back(i);
            AppendUniqueIndices(holeIdx, extra);
        }
    }

    std::vector<int> outerIdx = BuildSparseOuterCandidateIndices(outerLoop, 32);
    {
        std::vector<int> outerExtreme = Polyline2dUtil::BuildExtremeCandidateIndices(outerLoop);
        AppendUniqueIndices(outerIdx, outerExtreme);
    }

    for (int hi : holeIdx)
    {
        if (hi < 0 || hi >= static_cast<int>(hole.size()))
            continue;

        const gp_Pnt2d& hp = hole[hi];

        for (int oi : outerIdx)
        {
            if (oi < 0 || oi >= static_cast<int>(outerLoop.size()))
                continue;

            const gp_Pnt2d& op = outerLoop[oi];

            if (!IsBridgeGeometricallyValid(hp, op, outerLoop, holes, holeIndex))
                continue;

            std::vector<gp_Pnt2d> mergedLoop;
            if (!BridgeProducesValidMergedLoop(
                outerLoop, holes, holeIndex, op, hp, &mergedLoop))
            {
                continue;
            }

            BridgeCandidate c;
            c.holePt = hp;
            c.outerPt = op;
            c.length = OccUntrimUtil::Distance2d(hp, op);
            outCands.push_back(c);
        }
    }
}

bool OccUntrimmingBuilder::FindValidBridgeToOuter(
    const Handle(Geom_Surface)& surface,
    const std::vector<gp_Pnt2d>& outerLoop,
    const std::vector<UVCurveLoop>& holes,
    int holeIndex,
    gp_Pnt2d& outHolePt,
    gp_Pnt2d& outOuterPt,
    double* outScore)
{
    if (outScore)
        *outScore = 1.0e100;

    if (holeIndex < 0 || holeIndex >= static_cast<int>(holes.size()))
        return false;

    std::vector<BridgeCandidate> cands;
    CollectBridgeCandidates(outerLoop, holes, holeIndex, cands);

    if (cands.empty())
        return false;

    double minLen = 1.0e100;
    double maxLen = -1.0e100;
    for (const auto& c : cands)
    {
        minLen = std::min(minLen, c.length);
        maxLen = std::max(maxLen, c.length);
    }

    const auto& holeLoop = holes[holeIndex].points;

    int bestIdx = -1;
    double bestScore = 1.0e100;

    for (int i = 0; i < static_cast<int>(cands.size()); ++i)
    {
        auto& c = cands[i];

        if (!EvaluateBridgeAngles(
            outerLoop, holeLoop,
            c.outerPt, c.holePt,
            c.outerDiff, c.holeDiff))
        {
            continue;
        }
        /*double outerDiff3d = 1.0, holeDiff3d = 1.0;
        double outerDiff2d = 1.0, holeDiff2d = 1.0;
        if (!EvaluateBridgeAngles2dAnd3d(
            surface,
            outerLoop, holeLoop,
            c.outerPt, c.holePt,
            outerDiff2d, holeDiff2d,
            outerDiff3d, holeDiff3d))
        {
            continue;
        }*/

        if (maxLen - minLen < 1.0e-12)
            c.lenScore = 0.0;
        else
            c.lenScore = (c.length - minLen) / (maxLen - minLen);

        c.totalScore = 0.5 * c.lenScore
            + 0.25 * c.outerDiff
            + 0.25 * c.holeDiff;

        if (c.totalScore < bestScore)
        {
            bestScore = c.totalScore;
            bestIdx = i;
        }
    }

    if (bestIdx < 0)
        return false;

    outHolePt = cands[bestIdx].holePt;
    outOuterPt = cands[bestIdx].outerPt;

    if (outScore)
        *outScore = cands[bestIdx].totalScore;

    return true;
}

int OccUntrimmingBuilder::FindClosestPointIndex(
    const std::vector<gp_Pnt2d>& pts,
    const gp_Pnt2d& target) const
{
    if (pts.empty())
        return -1;

    int bestIdx = 0;
    double bestDist = OccUntrimUtil::Distance2d(pts[0], target);

    const int n = static_cast<int>(pts.size());
    const int m = (n >= 2 && OccUntrimUtil::Distance2d(pts.front(), pts.back()) < 1e-8) ? (n - 1) : n;

    for (int i = 1; i < m; ++i)
    {
        const double d = OccUntrimUtil::Distance2d(pts[i], target);
        if (d < bestDist)
        {
            bestDist = d;
            bestIdx = i;
        }
    }
    return bestIdx;
}

std::vector<gp_Pnt2d> OccUntrimmingBuilder::RotateClosedLoopToIndex(
    const std::vector<gp_Pnt2d>& loop,
    int startIdx) const
{
    std::vector<gp_Pnt2d> out;
    if (loop.size() < 2)
        return out;

    const int n = static_cast<int>(loop.size()) - 1;
    if (n <= 0)
        return out;

    startIdx = std::max(0, std::min(startIdx, n - 1));

    out.reserve(static_cast<size_t>(n + 1));
    for (int k = 0; k < n; ++k)
    {
        const int idx = (startIdx + k) % n;
        out.push_back(loop[idx]);
    }
    out.push_back(out.front());
    return out;
}

int OccUntrimmingBuilder::FindPointOnLoopSegment(
    const std::vector<gp_Pnt2d>& loop,
    const gp_Pnt2d& p,
    double tol) const
{
    if (loop.size() < 2)
        return -1;

    for (int i = 0; i + 1 < static_cast<int>(loop.size()); ++i)
    {
        if (Polyline2dUtil::IsPointOnSegment(p, loop[i], loop[i + 1], tol))
            return i;
    }

    return -1;
}

bool OccUntrimmingBuilder::InsertPointIntoLoop(
    const std::vector<gp_Pnt2d>& loop,
    const gp_Pnt2d& p,
    std::vector<gp_Pnt2d>& outLoop,
    double tol) const
{
    outLoop.clear();

    if (loop.size() < 2)
        return false;

    for (int i = 0; i < static_cast<int>(loop.size()); ++i)
    {
        if (OccUntrimUtil::Distance2d(loop[i], p) <= tol)
        {
            outLoop = loop;
            return true;
        }
    }

    const int segIdx = FindPointOnLoopSegment(loop, p, tol);
    if (segIdx < 0)
        return false;

    outLoop.reserve(loop.size() + 1);

    for (int i = 0; i <= segIdx; ++i)
        outLoop.push_back(loop[i]);

    if (OccUntrimUtil::Distance2d(outLoop.back(), p) > tol)
        outLoop.push_back(p);

    for (int i = segIdx + 1; i < static_cast<int>(loop.size()); ++i)
        outLoop.push_back(loop[i]);

    return true;
}

std::vector<gp_Pnt2d> OccUntrimmingBuilder::BuildBridgePolyline(
    const gp_Pnt2d& a,
    const gp_Pnt2d& b,
    int sampleCount) const
{
    std::vector<gp_Pnt2d> pts;

    sampleCount = std::max(2, sampleCount);
    pts.reserve(static_cast<size_t>(sampleCount));

    for (int i = 0; i < sampleCount; ++i)
    {
        const double t = static_cast<double>(i) / static_cast<double>(sampleCount - 1);
        pts.push_back(OccUntrimUtil::Lerp(a, b, t));
    }

    return pts;
}

bool OccUntrimmingBuilder::MergeHoleIntoOuterLoop(
    const std::vector<gp_Pnt2d>& outerLoop,
    const std::vector<gp_Pnt2d>& holeLoop,
    const gp_Pnt2d& holePt,
    const gp_Pnt2d& outerPt,
    std::vector<gp_Pnt2d>& mergedLoop)
{
    mergedLoop.clear();

    if (outerLoop.size() < 4 || holeLoop.size() < 4)
        return false;

    std::vector<gp_Pnt2d> outerInserted;
    std::vector<gp_Pnt2d> holeInserted;

    if (!InsertPointIntoLoop(outerLoop, outerPt, outerInserted))
        return false;

    if (!InsertPointIntoLoop(holeLoop, holePt, holeInserted))
        return false;

    OccUntrimUtil::EnsureClosed(outerInserted);
    OccUntrimUtil::EnsureClosed(holeInserted);

    const int outerIdx = FindClosestPointIndex(outerInserted, outerPt);
    const int holeIdx = FindClosestPointIndex(holeInserted, holePt);

    if (outerIdx < 0 || holeIdx < 0)
        return false;

    std::vector<gp_Pnt2d> outerRot = RotateClosedLoopToIndex(outerInserted, outerIdx);
    std::vector<gp_Pnt2d> holeRot = RotateClosedLoopToIndex(holeInserted, holeIdx);

    if (outerRot.empty() || holeRot.empty())
        return false;

    std::vector<gp_Pnt2d> holeRev = holeRot;
    //std::reverse(holeRev.begin(), holeRev.end());

    std::vector<gp_Pnt2d> bridgeOH = BuildBridgePolyline(outerPt, holePt, 2);
    std::vector<gp_Pnt2d> bridgeHO = BuildBridgePolyline(holePt, outerPt, 2);

    mergedLoop.reserve(
        outerRot.size() + holeRev.size() +
        bridgeOH.size() + bridgeHO.size() + 8);

    mergedLoop.push_back(outerPt);
    for (size_t i = 1; i < outerRot.size(); ++i)
    {
        if (OccUntrimUtil::Distance2d(mergedLoop.back(), outerRot[i]) > 1e-8)
            mergedLoop.push_back(outerRot[i]);
    }

    for (size_t i = 1; i < bridgeOH.size(); ++i)
    {
        if (OccUntrimUtil::Distance2d(mergedLoop.back(), bridgeOH[i]) > 1e-8)
            mergedLoop.push_back(bridgeOH[i]);
    }

    for (size_t i = 1; i < holeRev.size(); ++i)
    {
        if (OccUntrimUtil::Distance2d(mergedLoop.back(), holeRev[i]) > 1e-8)
            mergedLoop.push_back(holeRev[i]);
    }

    for (size_t i = 1; i < bridgeHO.size(); ++i)
    {
        if (OccUntrimUtil::Distance2d(mergedLoop.back(), bridgeHO[i]) > 1e-8)
            mergedLoop.push_back(bridgeHO[i]);
    }

    OccUntrimUtil::EnsureClosed(mergedLoop);

    std::vector<gp_Pnt2d> cleaned;
    cleaned.reserve(mergedLoop.size());
    for (size_t i = 0; i < mergedLoop.size(); ++i)
    {
        if (cleaned.empty() ||
            OccUntrimUtil::Distance2d(cleaned.back(), mergedLoop[i]) > 1e-8)
        {
            cleaned.push_back(mergedLoop[i]);
        }
    }

    OccUntrimUtil::EnsureClosed(cleaned);
    mergedLoop.swap(cleaned);

    return mergedLoop.size() >= 4;
}

bool OccUntrimmingBuilder::BuildUVQuadLayout(
    const MidSurfaceFaceData& inData,
    std::vector<UVQuadPatch>& outPatches)
{
    outPatches.clear();

    //if (inData.isBridgeMergedSingleLoop)
    {
       /* const auto concaveCorners = DetectMergedLoopConcaveCorners(inData);

        if (m_opt.verbose)
        {
            std::cout << "[BuildUVQuadLayout] bridge merged loop, concave count = "
                << concaveCorners.size() << std::endl;
        }*/

        // Ŀǰԭ bridge 
        // ɰ concaveCorners ȥϸ
        return BuildPolygonQuadsAfterBridge(inData, outPatches);
    }

    return BuildDominantAxisQuads(inData, outPatches);
}

bool OccUntrimmingBuilder::BuildPolygonQuadsAfterBridge(
    const MidSurfaceFaceData& inData,
    std::vector<UVQuadPatch>& outPatches)
{
    outPatches.clear();

    const UVCurveLoop* outer = nullptr;
    for (const auto& loop : inData.loops)
    {
        if (loop.isOuter)
        {
            outer = &loop;
            break;
        }
    }

    if (!outer)
        return false;

    std::vector<UVPolygonCell> pending;
    std::vector<UVPolygonCell> failed;

    UVPolygonCell root;
    root.boundary = outer->points;
    pending.push_back(root);

    while (!pending.empty())
    {
        UVPolygonCell cell = pending.back();
        pending.pop_back();

        const auto& poly = cell.boundary;

        if (IsQuadDomain(poly))
        {
            UVQuadPatch patch;
            if (QuadifyQuad(poly, patch))
            {
                outPatches.push_back(patch);
                continue;
            }
        }

        std::vector<UVQuadPatch> peeledQuads;
        std::vector<UVPolygonCell> nextCells;
        if (TryPeelQuadByBestSplit(poly, peeledQuads, nextCells))
        {
            outPatches.insert(outPatches.end(), peeledQuads.begin(), peeledQuads.end());
            pending.insert(pending.end(), nextCells.begin(), nextCells.end());
            continue;
        }

        failed.push_back(cell);
    }

    for (const auto& cell : failed)
    {
        UVQuadPatch patch;
        if (IsQuadDomain(cell.boundary) && QuadifyQuad(cell.boundary, patch))
        {
            outPatches.push_back(patch);
            continue;
        }

        std::vector<UVQuadPatch> localPatches;
        if (PolygonCellToQuadPatches(cell, localPatches))
        {
            outPatches.insert(outPatches.end(), localPatches.begin(), localPatches.end());
        }
    }

    return !outPatches.empty();
}

std::vector<gp_Pnt2d> OccUntrimmingBuilder::RemoveDuplicateClosingPoint(
    const std::vector<gp_Pnt2d>& loop) const
{
    std::vector<gp_Pnt2d> out = loop;
    if (out.size() >= 2 &&
        OccUntrimUtil::Distance2d(out.front(), out.back()) < 1e-8)
    {
        out.pop_back();
    }
    return out;
}

std::vector<double> OccUntrimmingBuilder::ComputeVertexTurningAngles(
    const std::vector<gp_Pnt2d>& loop,
    double lenTol) const
{
    std::vector<double> angles;
    const std::vector<gp_Pnt2d> poly = RemoveDuplicateClosingPoint(loop);
    const int n = static_cast<int>(poly.size());
    if (n < 3)
        return angles;

    angles.resize(n, 0.0);
    for (int i = 0; i < n; ++i)
    {
        const gp_Pnt2d& prev = poly[(i - 1 + n) % n];
        const gp_Pnt2d& curr = poly[i];
        const gp_Pnt2d& next = poly[(i + 1) % n];

        gp_Vec2d v1(prev, curr);
        gp_Vec2d v2(curr, next);
        const double len1 = v1.Magnitude();
        const double len2 = v2.Magnitude();
        if (len1 < lenTol || len2 < lenTol)
            continue;
        v1 /= len1;
        v2 /= len2;
        double dot = v1.X() * v2.X() + v1.Y() * v2.Y();
        dot = std::max(-1.0, std::min(1.0, dot));
        angles[i] = std::acos(dot);
    }
    return angles;
}

std::vector<gp_Pnt2d> OccUntrimmingBuilder::ExtractPathBetweenCorners(
    const std::vector<gp_Pnt2d>& poly,
    int startIdx,
    int endIdx) const
{
    std::vector<gp_Pnt2d> path;
    const std::vector<gp_Pnt2d> loop = RemoveDuplicateClosingPoint(poly);
    const int n = static_cast<int>(loop.size());
    if (n < 2)
        return path;

    auto normIdx = [n](int idx)
        {
            while (idx < 0) idx += n;
            while (idx >= n) idx -= n;
            return idx;
        };

    startIdx = normIdx(startIdx);
    endIdx = normIdx(endIdx);
    path.push_back(loop[startIdx]);
    int cur = startIdx;
    while (cur != endIdx)
    {
        cur = normIdx(cur + 1);
        path.push_back(loop[cur]);
    }
    return path;
}

bool OccUntrimmingBuilder::IsQuadDomain(
    const std::vector<gp_Pnt2d>& poly,
    double angleThresholdDeg,
    double lenTol) const
{
    LogicalPolygon2d logical = ExtractLogicalPolygon(poly, angleThresholdDeg, lenTol);
    return logical.sides.size() == 4;
}

bool OccUntrimmingBuilder::IsTriangleDomain(
    const std::vector<gp_Pnt2d>& poly,
    double angleThresholdDeg,
    double lenTol) const
{
    LogicalPolygon2d logical = ExtractLogicalPolygon(poly, angleThresholdDeg, lenTol);
    return logical.sides.size() == 3;
}

std::vector<int> OccUntrimmingBuilder::BuildPolygonCandidateIndices(
    const std::vector<gp_Pnt2d>& poly,
    int targetCount) const
{
    std::vector<int> idxs;
    const std::vector<gp_Pnt2d> loop = RemoveDuplicateClosingPoint(poly);
    const int n = static_cast<int>(loop.size());
    if (n < 3)
        return idxs;

    const auto corners = DetectPolygonConcaveCorners(loop);
    for (const auto& c : corners)
    {
        if (c.index >= 0 && c.index < n)
            idxs.push_back(c.index);
    }

    {
        std::vector<int> ex = Polyline2dUtil::BuildExtremeCandidateIndices(loop);
        idxs.insert(idxs.end(), ex.begin(), ex.end());
    }

    targetCount = std::max(8, targetCount);
    const int step = std::max(1, n / targetCount);
    for (int i = 0; i < n; i += step)
        idxs.push_back(i);

    std::sort(idxs.begin(), idxs.end());
    idxs.erase(std::unique(idxs.begin(), idxs.end()), idxs.end());
    return idxs;
}

double OccUntrimmingBuilder::EvaluatePolygonSplitAngleScore(
    const std::vector<gp_Pnt2d>& poly,
    int i,
    int j) const
{
    std::vector<gp_Pnt2d> loop = RemoveDuplicateClosingPoint(poly);
    const int n = static_cast<int>(loop.size());
    if (n < 3)
        return 1.0;

    auto normIdx = [n](int idx)
        {
            while (idx < 0) idx += n;
            while (idx >= n) idx -= n;
            return idx;
        };

    i = normIdx(i);
    j = normIdx(j);

    const gp_Pnt2d& pi = loop[i];
    const gp_Pnt2d& pj = loop[j];
    gp_Vec2d dij(pi, pj);
    gp_Vec2d dji(pj, pi);
    if (dij.Magnitude() < 1.0e-12 || dji.Magnitude() < 1.0e-12)
        return 1.0;

    const int ip = normIdx(i - 1), inx = normIdx(i + 1);
    const int jp = normIdx(j - 1), jn = normIdx(j + 1);

    gp_Vec2d viPrev(pi, loop[ip]), viNext(pi, loop[inx]);
    gp_Vec2d vjPrev(pj, loop[jp]), vjNext(pj, loop[jn]);

    const double a1 = AnglePenalty(dij, viPrev);
    const double a2 = AnglePenalty(dij, viNext);
    const double a3 = AnglePenalty(dji, vjPrev);
    const double a4 = AnglePenalty(dji, vjNext);

    return 0.5 * std::abs(a1 - a2) + 0.5 * std::abs(a3 - a4);
}

double OccUntrimmingBuilder::EvaluatePolygonSplitQuadScore(
    const std::vector<gp_Pnt2d>& poly,
    int i,
    int j) const
{
    std::vector<gp_Pnt2d> childA, childB;
    if (!SplitPolygonByDiagonal(RemoveDuplicateClosingPoint(poly), i, j, childA, childB))
        return 1.0;

    const bool quadA = IsQuadDomain(childA);
    const bool quadB = IsQuadDomain(childB);

    if (quadA && quadB)
        return 0.0;
    const bool triA = IsTriangleDomain(childA);
    const bool triB = IsTriangleDomain(childB);

    if (triA || triB)
        return 2.0;
    return 1.0;
}

void OccUntrimmingBuilder::CollectPolygonSplitCandidates(
    const std::vector<gp_Pnt2d>& poly,
    std::vector<PolygonSplitCandidate>& outCands) const
{
    outCands.clear();
    std::vector<gp_Pnt2d> loop = RemoveDuplicateClosingPoint(poly);
    const int n = static_cast<int>(loop.size());
    if (n < 4)
        return;

    const std::vector<int> candIdx = BuildPolygonCandidateIndices(loop, 24);
    if (candIdx.size() < 2)
        return;

    for (int a = 0; a < static_cast<int>(candIdx.size()); ++a)
    {
        for (int b = a + 1; b < static_cast<int>(candIdx.size()); ++b)
        {
            const int i = candIdx[a];
            const int j = candIdx[b];
            if (!IsValidSplitDiagonal(loop, i, j))
                continue;

            PolygonSplitCandidate c;
            c.i = i;
            c.j = j;
            c.length = OccUntrimUtil::Distance2d(loop[i], loop[j]);
            c.angleScore = EvaluatePolygonSplitAngleScore(loop, i, j);
            c.quadScore = EvaluatePolygonSplitQuadScore(loop, i, j);
            outCands.push_back(c);
        }
    }

    if (outCands.empty())
        return;

    double minLen = 1.0e100, maxLen = -1.0e100;
    for (const auto& c : outCands)
    {
        minLen = std::min(minLen, c.length);
        maxLen = std::max(maxLen, c.length);
    }

    for (auto& c : outCands)
    {
        c.lenScore = (maxLen - minLen < 1.0e-12) ? 0.0 : (c.length - minLen) / (maxLen - minLen);
        c.totalScore = 0.30 * c.lenScore + 0.7 * c.angleScore + 1 * c.quadScore;
    }

    std::sort(outCands.begin(), outCands.end(), [](const PolygonSplitCandidate& a, const PolygonSplitCandidate& b)
        {
            return a.totalScore < b.totalScore;
        });
}

bool OccUntrimmingBuilder::TryPeelQuadByBestSplit(
    const std::vector<gp_Pnt2d>& poly,
    std::vector<UVQuadPatch>& outQuadPatches,
    std::vector<UVPolygonCell>& outNextCells)
{
    outQuadPatches.clear();
    outNextCells.clear();

    std::vector<PolygonSplitCandidate> cands;
    CollectPolygonSplitCandidates(poly, cands);
    if (cands.empty())
        return false;

    const std::vector<gp_Pnt2d> srcPoly = RemoveDuplicateClosingPoint(poly);

    for (const auto& cand : cands)
    {
        std::vector<gp_Pnt2d> childA, childB;
        if (!SplitPolygonByDiagonal(srcPoly, cand.i, cand.j, childA, childB))
            continue;

        
        // 先记录这次 split 的直接结果：childA / childB
        DebugFrame frame;
        frame.message = "polygon split by diagonal";

        {
            DebugLoop2d beforeLoop;
            beforeLoop.points = poly;
            beforeLoop.isOuter = true;
            OccUntrimUtil::EnsureClosed(beforeLoop.points);
            frame.loopsBefore.push_back(beforeLoop);
        }

        {
            const gp_Pnt2d& a = srcPoly[cand.i];
            const gp_Pnt2d& b = srcPoly[cand.j];

            DebugStepSegment2d seg;
            seg.type = DebugStepSegmentType::Split;
            seg.p0 = a;
            seg.p1 = b;
            seg.polyline = BuildBridgePolyline(a, b, 2);
            frame.segments.push_back(seg);
        }

        {
            DebugLoop2d loopA;
            loopA.points = childA;
            loopA.isOuter = true;
            OccUntrimUtil::EnsureClosed(loopA.points);
            frame.splitChildLoops.push_back(loopA);

            DebugLoop2d loopB;
            loopB.points = childB;
            loopB.isOuter = true;
            OccUntrimUtil::EnsureClosed(loopB.points);
            frame.splitChildLoops.push_back(loopB);
        }
        UpdateUvPointType(childA.front(), UVPointCurveType::Line);
        //UpdateUvPointType(childB.front(), UVPointCurveType::Line);
        UpdateUvPointType(childA.back(), UVPointCurveType::Line);
        //UpdateUvPointType(childB.back(), UVPointCurveType::Line);
        const bool quadA = IsQuadDomain(childA);
        const bool quadB = IsQuadDomain(childB);
        bool accepted = false;

        std::vector<UVQuadPatch> localQuadPatches;
        std::vector<UVPolygonCell> localNextCells;

        if (quadA)
        {
            UVQuadPatch patchA;
            if (QuadifyQuad(childA, patchA))
            {
                localQuadPatches.push_back(patchA);
                accepted = true;
            }
        }
        else
        {
            UVPolygonCell cellA;
            cellA.boundary = childA;
            localNextCells.push_back(cellA);
            accepted = true;
        }

        if (quadB)
        {
            UVQuadPatch patchB;
            if (QuadifyQuad(childB, patchB))
            {
                localQuadPatches.push_back(patchB);
                accepted = true;
            }
        }
        else
        {
            UVPolygonCell cellB;
            cellB.boundary = childB;
            localNextCells.push_back(cellB);
            accepted = true;
        }

        if (!accepted)
            continue;

        
        outQuadPatches = std::move(localQuadPatches);
        outNextCells = std::move(localNextCells);

        // loopsAfter 不混用
        frame.loopsAfter.clear();

        m_debugFrames.push_back(frame);
        return true;
    }

    return false;
}

std::vector<Polyline2dUtil::CornerInfo>
OccUntrimmingBuilder::DetectPolygonConcaveCorners(
    const std::vector<gp_Pnt2d>& poly) const
{
    std::vector<gp_Pnt2d> closedPoly = poly;
    Polyline2dUtil::EnsureCCW(closedPoly);

    return Polyline2dUtil::DetectConcaveCorners(
        closedPoly,
        20.0,   // minTurnAngleDeg
        1e-6,   // minEdgeLen
        1e-8);
}

bool OccUntrimmingBuilder::IsValidSplitDiagonal(
    const std::vector<gp_Pnt2d>& poly,
    int i,
    int j) const
{
    const int n = static_cast<int>(poly.size());
    if (n < 4)
        return false;

    if (i < 0 || i >= n || j < 0 || j >= n || i == j)
        return false;

    // ڵ㲻Ϊ
    if ((i + 1) % n == j || (j + 1) % n == i)
        return false;

    const gp_Pnt2d& a = poly[i];
    const gp_Pnt2d& b = poly[j];

    // еڲ
    gp_Pnt2d mid(
        0.5 * (a.X() + b.X()),
        0.5 * (a.Y() + b.Y()));
    if (!Polyline2dUtil::PointInPolygon(mid, poly, 1e-8))
        return false;

    // ˵ڱཻ֮
    for (int k = 0; k < n; ++k)
    {
        int k2 = (k + 1) % n;

        //  i/j ڡ˵ı
        if (k == i || k2 == i || k == j || k2 == j)
            continue;

        if (Polyline2dUtil::SegmentIntersect(a, b, poly[k], poly[k2], 1e-8))
            return false;
    }

    std::vector<gp_Pnt2d> polyA, polyB;
    if (!SplitPolygonByDiagonal(poly, i, j, polyA, polyB))
        return false;

    if (polyA.size() < 3 || polyB.size() < 3)
        return false;

    std::vector<gp_Pnt2d> closedA = polyA;
    std::vector<gp_Pnt2d> closedB = polyB;
    Polyline2dUtil::EnsureClosed(closedA);
    Polyline2dUtil::EnsureClosed(closedB);

    if (Polyline2dUtil::PolylineSelfIntersect(closedA, 1e-8))
        return false;
    if (Polyline2dUtil::PolylineSelfIntersect(closedB, 1e-8))
        return false;

    return true;
}

bool OccUntrimmingBuilder::SplitPolygonByDiagonal(
    const std::vector<gp_Pnt2d>& poly,
    int i,
    int j,
    std::vector<gp_Pnt2d>& outPolyA,
    std::vector<gp_Pnt2d>& outPolyB) const
{
    outPolyA.clear();
    outPolyB.clear();

    const int n = static_cast<int>(poly.size());
    if (n < 4 || i == j)
        return false;

    if (i > j)
        std::swap(i, j);

    // A: i -> ... -> j
    for (int k = i; k <= j; ++k)
        outPolyA.push_back(poly[k]);

    // B: j -> ... -> end -> 0 -> ... -> i
    for (int k = j; k < n; ++k)
        outPolyB.push_back(poly[k]);
    for (int k = 0; k <= i; ++k)
        outPolyB.push_back(poly[k]);

    // ȥظ
    auto dedup = [](std::vector<gp_Pnt2d>& pts)
        {
            std::vector<gp_Pnt2d> cleaned;
            for (const auto& p : pts)
            {
                if (cleaned.empty() ||
                    Polyline2dUtil::Distance(cleaned.back(), p) > 1e-8)
                {
                    cleaned.push_back(p);
                }
            }
            pts.swap(cleaned);
        };

    dedup(outPolyA);
    dedup(outPolyB);

    return outPolyA.size() >= 3 && outPolyB.size() >= 3;
}

double OccUntrimmingBuilder::EstimateSplitScore(
    const std::vector<gp_Pnt2d>& poly,
    int i,
    int j) const
{
    std::vector<gp_Pnt2d> polyA, polyB;
    if (!SplitPolygonByDiagonal(poly, i, j, polyA, polyB))
        return 1e100;

    const double len = Polyline2dUtil::Distance(poly[i], poly[j]);
    const double balance = std::abs(static_cast<int>(polyA.size()) - static_cast<int>(polyB.size()));

    // ԽСԽ
    return len + 2.0 * balance;
}

bool OccUntrimmingBuilder::FindBestConcaveSplitDiagonal(
    const std::vector<gp_Pnt2d>& poly,
    int& outI,
    int& outJ) const
{
    outI = -1;
    outJ = -1;

    const auto corners = DetectPolygonConcaveCorners(poly);
    if (corners.size() < 2)
        return false;

    double bestScore = 1e100;

    for (size_t a = 0; a < corners.size(); ++a)
    {
        for (size_t b = a + 1; b < corners.size(); ++b)
        {
            const int i = corners[a].index;
            const int j = corners[b].index;

            if (!IsValidSplitDiagonal(poly, i, j))
                continue;

            const double score = EstimateSplitScore(poly, i, j);
            if (score < bestScore)
            {
                bestScore = score;
                outI = i;
                outJ = j;
            }
        }
    }

    return outI >= 0 && outJ >= 0;
}

bool OccUntrimmingBuilder::SplitBridgeMergedPolygonToCells(
    const std::vector<gp_Pnt2d>& mergedLoop,
    std::vector<UVPolygonCell>& outCells)
{
    outCells.clear();

    std::vector<gp_Pnt2d> poly = RemoveDuplicateClosingPoint(mergedLoop);
    if (poly.size() < 3)
        return false;

    // ͳһ򣬱ⰼжŻ
    std::vector<gp_Pnt2d> closedPoly = poly;
    Polyline2dUtil::EnsureCCW(closedPoly);
    poly = RemoveDuplicateClosingPoint(closedPoly);

    std::function<bool(const std::vector<gp_Pnt2d>&)> recurse;
    recurse = [&](const std::vector<gp_Pnt2d>& onePoly) -> bool
        {
            const int n = static_cast<int>(onePoly.size());
            if (n < 3)
                return false;

            // СֱΪ cell  PolygonCellToQuadPatches
            if (n <= 5)
            {
                UVPolygonCell cell;
                cell.boundary = onePoly;
                outCells.push_back(cell);
                return true;
            }

            int splitI = -1;
            int splitJ = -1;
            if (!FindBestConcaveSplitDiagonal(onePoly, splitI, splitJ))
            {
                // ҲϷǶԽߣͰѵǰ polygon ͨô
                UVPolygonCell cell;
                cell.boundary = onePoly;
                outCells.push_back(cell);
                return true;
            }

            std::vector<gp_Pnt2d> polyA, polyB;
            if (!SplitPolygonByDiagonal(onePoly, splitI, splitJ, polyA, polyB))
            {
                UVPolygonCell cell;
                cell.boundary = onePoly;
                outCells.push_back(cell);
                return true;
            }

            bool okA = recurse(polyA);
            bool okB = recurse(polyB);
            return okA && okB;
        };

    return recurse(poly);
}

bool OccUntrimmingBuilder::PolygonCellToQuadPatches(
    const UVPolygonCell& cell,
    std::vector<UVQuadPatch>& outPatches)
{
    outPatches.clear();

    std::vector<gp_Pnt2d> poly = RemoveDuplicateClosingPoint(cell.boundary);
    const int n = static_cast<int>(poly.size());

    if (n < 3)
        return false;

    if (n == 3)
    {
        UVQuadPatch patch;
        if (!QuadifyTriangle(poly, patch))
            return false;
        outPatches.push_back(patch);
        return true;
    }

    if (n == 4)
    {
        UVQuadPatch patch;
        if (!QuadifyQuad(poly, patch))
            return false;
        outPatches.push_back(patch);
        return true;
    }

    if (n == 5)
    {
        return QuadifyPentagon(poly, outPatches);
    }

    std::vector<UVPolygonCell> children;
    if (!SplitGeneralPolygonCell(poly, children))
        return false;

    for (const auto& ch : children)
    {
        std::vector<UVQuadPatch> sub;
        if (!PolygonCellToQuadPatches(ch, sub))
            continue;

        outPatches.insert(outPatches.end(), sub.begin(), sub.end());
    }

    return !outPatches.empty();
}

bool OccUntrimmingBuilder::QuadifyTriangle(
    const std::vector<gp_Pnt2d>& tri,
    UVQuadPatch& outPatch)
{
    if (tri.size() != 3)
        return false;

    gp_Pnt2d s = OccUntrimUtil::Lerp(tri[0], tri[1], 0.5);

    std::array<gp_Pnt2d, 4> corners =
    {
        tri[0], s, tri[1], tri[2]
    };

    return BuildUVQuadFrom4Vertices(corners, outPatch);
}

bool OccUntrimmingBuilder::QuadifyQuad(
    const std::vector<gp_Pnt2d>& quad,
    UVQuadPatch& outPatch)
{
    outPatch = UVQuadPatch();

    std::vector<gp_Pnt2d> raw = RemoveDuplicateClosingPoint(quad);
    if (raw.size() < 4)
        return false;

    LogicalPolygon2d logical = ExtractLogicalPolygon(raw);
    if (logical.cornerIndices.size() != 4)
        return false;

    const int c0 = logical.cornerIndices[0];
    const int c1 = logical.cornerIndices[1];
    const int c2 = logical.cornerIndices[2];
    const int c3 = logical.cornerIndices[3];

    std::vector<gp_Pnt2d> south = ExtractPathBetweenCorners(raw, c0, c1);
    std::vector<gp_Pnt2d> east = ExtractPathBetweenCorners(raw, c1, c2);
    std::vector<gp_Pnt2d> north = ExtractPathBetweenCorners(raw, c2, c3);
    std::vector<gp_Pnt2d> west = ExtractPathBetweenCorners(raw, c3, c0);

    if (south.size() < 2 || east.size() < 2 ||
        north.size() < 2 || west.size() < 2)
        return false;

    // 调整成标准四边patch方向：
    // south: p00 -> p10
    // east : p10 -> p11
    // north: p01 -> p11
    // west : p00 -> p01
    std::reverse(north.begin(), north.end()); // c3 -> c2
    std::reverse(west.begin(), west.end());   // c0 -> c3

    const int edgeSample = std::max(4, m_opt.boundarySampleCountPerEdge / 4);
    south = OccUntrimUtil::ResamplePolylineUniform(south, edgeSample, true);
    east = OccUntrimUtil::ResamplePolylineUniform(east, edgeSample, true);
    north = OccUntrimUtil::ResamplePolylineUniform(north, edgeSample, true);
    west = OccUntrimUtil::ResamplePolylineUniform(west, edgeSample, true);

    outPatch.south = south;
    outPatch.east = east;
    outPatch.north = north;
    outPatch.west = west;

    outPatch.p00 = south.front(); // c0
    outPatch.p10 = south.back();  // c1
    outPatch.p11 = east.back();   // c2
    outPatch.p01 = west.back();   // c3

    return ValidateUVPatch(outPatch, nullptr);
}

bool OccUntrimmingBuilder::QuadifyPentagon(
    const std::vector<gp_Pnt2d>& pentagon,
    std::vector<UVQuadPatch>& outPatches)
{
    outPatches.clear();

    if (pentagon.size() != 5)
        return false;

    gp_Pnt2d s = OccUntrimUtil::Lerp(pentagon[0], pentagon[1], 0.5);

    std::array<gp_Pnt2d, 4> quad1 =
    {
        s, pentagon[1], pentagon[2], pentagon[3]
    };

    std::array<gp_Pnt2d, 4> quad2 =
    {
        pentagon[0], s, pentagon[3], pentagon[4]
    };

    UVQuadPatch p1, p2;
    if (!BuildUVQuadFrom4Vertices(quad1, p1))
        return false;
    if (!BuildUVQuadFrom4Vertices(quad2, p2))
        return false;

    outPatches.push_back(p1);
    outPatches.push_back(p2);
    return true;
}

bool OccUntrimmingBuilder::SplitGeneralPolygonCell(
    const std::vector<gp_Pnt2d>& poly,
    std::vector<UVPolygonCell>& outChildren)
{
    outChildren.clear();

    const int n = static_cast<int>(poly.size());
    if (n < 6)
        return false;

    UVPolygonCell c1, c2;

    c1.boundary = { poly[0], poly[1], poly[2], poly[3] };

    c2.boundary.push_back(poly[0]);
    for (int i = 3; i < n; ++i)
        c2.boundary.push_back(poly[i]);

    outChildren.push_back(c1);
    outChildren.push_back(c2);
    return true;
}

bool OccUntrimmingBuilder::BuildUVQuadFrom4Vertices(
    const std::array<gp_Pnt2d, 4>& corners,
    UVQuadPatch& outPatch)
{
    outPatch = UVQuadPatch();

    const int edgeSample = std::max(2, m_opt.boundarySampleCountPerEdge / 4);

    const gp_Pnt2d& p00 = corners[0];
    const gp_Pnt2d& p10 = corners[1];
    const gp_Pnt2d& p11 = corners[2];
    const gp_Pnt2d& p01 = corners[3];

    outPatch.south = BuildBridgePolyline(p00, p10, edgeSample);
    outPatch.east = BuildBridgePolyline(p10, p11, edgeSample);
    outPatch.north = BuildBridgePolyline(p01, p11, edgeSample);
    //std::reverse(outPatch.north.begin(), outPatch.north.end());
    outPatch.west = BuildBridgePolyline(p00, p01, edgeSample);

    outPatch.p00 = p00;
    outPatch.p10 = p10;
    outPatch.p01 = p01;
    outPatch.p11 = p11;

    return outPatch.IsValid();
}

bool OccUntrimmingBuilder::BuildDominantAxisQuads(
    const MidSurfaceFaceData& inData,
    std::vector<UVQuadPatch>& outPatches)
{
    outPatches.clear();

    const UVCurveLoop* outer = nullptr;
    for (const auto& loop : inData.loops)
    {
        if (loop.isOuter)
        {
            outer = &loop;
            break;
        }
    }

    if (!outer || outer->points.size() < 5)
        return false;

    const bool alongU = (OccUntrimUtil::FindLongestAxis(outer->points) == 0);

    int idxMin = 0;
    int idxMax = 0;
    double minVal = alongU ? outer->points[0].X() : outer->points[0].Y();
    double maxVal = minVal;

    for (int i = 1; i < static_cast<int>(outer->points.size()) - 1; ++i)
    {
        const double val = alongU ? outer->points[i].X() : outer->points[i].Y();
        if (val < minVal) { minVal = val; idxMin = i; }
        if (val > maxVal) { maxVal = val; idxMax = i; }
    }

    if (idxMin == idxMax)
        return false;

    std::vector<gp_Pnt2d> chainA;
    std::vector<gp_Pnt2d> chainB;

    const auto& pts = outer->points;
    const int n = static_cast<int>(pts.size()) - 1;

    auto nextIndex = [n](int i) { return (i + 1) % n; };

    {
        int cur = idxMin;
        chainA.push_back(pts[cur]);
        while (cur != idxMax)
        {
            cur = nextIndex(cur);
            chainA.push_back(pts[cur]);
        }
    }

    {
        int cur = idxMax;
        chainB.push_back(pts[cur]);
        while (cur != idxMin)
        {
            cur = nextIndex(cur);
            chainB.push_back(pts[cur]);
        }
    }

    std::reverse(chainB.begin(), chainB.end());

    const double lenA = OccUntrimUtil::PolylineLength2d(chainA);
    const double lenB = OccUntrimUtil::PolylineLength2d(chainB);
    const double avgLen = 0.5 * (lenA + lenB);

    int patchCount = 4;
    if (avgLen < 1e-3) patchCount = 1;
    else
    {
        patchCount = std::max(1, std::min(8, static_cast<int>(avgLen / 10.0)));
    }

    if (patchCount == 1)
        patchCount = 2;

    for (int i = 0; i < patchCount; ++i)
    {
        const double t0 = static_cast<double>(i) / static_cast<double>(patchCount);
        const double t1 = static_cast<double>(i + 1) / static_cast<double>(patchCount);

        UVQuadPatch quad;
        if (!BuildQuadPatchFromPolylineBand(chainA, chainB, t0, t1, alongU, quad))
            continue;

        outPatches.push_back(quad);
    }

    return !outPatches.empty();
}

bool OccUntrimmingBuilder::BuildQuadPatchFromPolylineBand(
    const std::vector<gp_Pnt2d>& lowerCurve,
    const std::vector<gp_Pnt2d>& upperCurve,
    double splitParam0,
    double splitParam1,
    bool /*alongU*/,
    UVQuadPatch& outPatch)
{
    outPatch = UVQuadPatch();

    if (lowerCurve.size() < 2 || upperCurve.size() < 2)
        return false;

    const int edgeSample = std::max(2, m_opt.boundarySampleCountPerEdge / 4);

    std::vector<gp_Pnt2d> south;
    std::vector<gp_Pnt2d> north;
    south.reserve(static_cast<size_t>(edgeSample));
    north.reserve(static_cast<size_t>(edgeSample));

    for (int i = 0; i < edgeSample; ++i)
    {
        const double s = static_cast<double>(i) / static_cast<double>(edgeSample - 1);
        const double t = splitParam0 * (1.0 - s) + splitParam1 * s;
        south.push_back(OccUntrimUtil::SamplePolylineByArcLength(lowerCurve, t));
        north.push_back(OccUntrimUtil::SamplePolylineByArcLength(upperCurve, t));
    }

    const gp_Pnt2d p00 = south.front();
    const gp_Pnt2d p10 = south.back();
    const gp_Pnt2d p01 = north.front();
    const gp_Pnt2d p11 = north.back();

    std::vector<gp_Pnt2d> west;
    std::vector<gp_Pnt2d> east;
    west.reserve(static_cast<size_t>(edgeSample));
    east.reserve(static_cast<size_t>(edgeSample));

    for (int i = 0; i < edgeSample; ++i)
    {
        const double e = static_cast<double>(i) / static_cast<double>(edgeSample - 1);
        west.push_back(OccUntrimUtil::Lerp(p00, p01, e));
        east.push_back(OccUntrimUtil::Lerp(p10, p11, e));
    }

    outPatch.south = south;
    outPatch.north = north;
    outPatch.west = west;
    outPatch.east = east;
    outPatch.p00 = p00;
    outPatch.p10 = p10;
    outPatch.p01 = p01;
    outPatch.p11 = p11;

    return true;
}

bool OccUntrimmingBuilder::FitSinglePatch(
    const Handle(Geom_Surface)& surf,
    const UVQuadPatch& uvPatch,
    Handle(Geom_BSplineSurface)& outSurf)
{
    outSurf.Nullify();

    std::string err;
    if (!ValidateUVPatch(uvPatch, &err))
    {
        if (m_opt.verbose)
            std::cout << "[OccUntrimmingBuilder] Invalid UV patch: " << err << std::endl;
        return false;
    }

    std::vector<std::vector<gp_Pnt>> gridPts;
    if (!SamplePatchOnSurface(surf, uvPatch, m_opt.coonsSampleU, m_opt.coonsSampleV, gridPts))
        return false;

    if (!m_fitter.Fit(gridPts, m_opt.fitDegreeU, m_opt.fitDegreeV, outSurf, &err))
    {
        if (m_opt.verbose)
            std::cout << "[OccUntrimmingBuilder] Fitting failed: " << err << std::endl;
        return false;
    }

    return !outSurf.IsNull();
}

bool OccUntrimmingBuilder::SamplePatchOnSurface(
    const Handle(Geom_Surface)& surf,
    const UVQuadPatch& uvPatch,
    int sampleU,
    int sampleV,
    std::vector<std::vector<gp_Pnt>>& outGridPts)
{
    outGridPts.clear();

    UVCoonsMapper mapper;
    if (!mapper.Build(uvPatch))
        return false;

    if (!CheckMapperJacobianBySampling(mapper, 8, 8))
    {
        if (m_opt.verbose)
            std::cout << "[OccUntrimmingBuilder] Warning: mapper Jacobian may be invalid." << std::endl;
    }

    sampleU = std::max(2, sampleU);
    sampleV = std::max(2, sampleV);

    outGridPts.resize(static_cast<size_t>(sampleU));
    for (int i = 0; i < sampleU; ++i)
    {
        outGridPts[i].resize(static_cast<size_t>(sampleV));

        const double xi = static_cast<double>(i) / static_cast<double>(sampleU - 1);
        for (int j = 0; j < sampleV; ++j)
        {
            const double eta = static_cast<double>(j) / static_cast<double>(sampleV - 1);
            const gp_Pnt2d uv = mapper.Evaluate(xi, eta);
            outGridPts[i][j] = surf->Value(uv.X(), uv.Y());
        }
    }

    return true;
}

bool OccUntrimmingBuilder::ValidateUVPatch(const UVQuadPatch& patch, std::string* errMsg) const
{
    if (!patch.IsValid())
    {
        if (errMsg) *errMsg = "patch edges are not valid.";
        return false;
    }

    const double tol = 1e-6;
    if (OccUntrimUtil::Distance2d(patch.south.front(), patch.p00) > tol ||
        OccUntrimUtil::Distance2d(patch.south.back(), patch.p10) > tol ||
        OccUntrimUtil::Distance2d(patch.north.front(), patch.p01) > tol ||
        OccUntrimUtil::Distance2d(patch.north.back(), patch.p11) > tol)
    {
        if (errMsg) *errMsg = "south/north edge-corner mismatch.";
        return false;
    }

    if (OccUntrimUtil::Distance2d(patch.west.front(), patch.p00) > tol ||
        OccUntrimUtil::Distance2d(patch.west.back(), patch.p01) > tol ||
        OccUntrimUtil::Distance2d(patch.east.front(), patch.p10) > tol ||
        OccUntrimUtil::Distance2d(patch.east.back(), patch.p11) > tol)
    {
        if (errMsg) *errMsg = "west/east edge-corner mismatch.";
        return false;
    }

    return true;
}

bool OccUntrimmingBuilder::CheckMapperJacobianBySampling(const UVCoonsMapper& mapper, int nu, int nv) const
{
    if (!mapper.IsReady())
        return false;

    nu = std::max(3, nu);
    nv = std::max(3, nv);

    const double eps = 1e-4;
    int signRef = 0;

    for (int i = 1; i < nu - 1; ++i)
    {
        for (int j = 1; j < nv - 1; ++j)
        {
            const double xi = static_cast<double>(i) / static_cast<double>(nu - 1);
            const double eta = static_cast<double>(j) / static_cast<double>(nv - 1);

            const gp_Pnt2d p_x1 = mapper.Evaluate(xi - eps, eta);
            const gp_Pnt2d p_x2 = mapper.Evaluate(xi + eps, eta);
            const gp_Pnt2d p_y1 = mapper.Evaluate(xi, eta - eps);
            const gp_Pnt2d p_y2 = mapper.Evaluate(xi, eta + eps);

            const double dux = (p_x2.X() - p_x1.X()) / (2.0 * eps);
            const double duy = (p_x2.Y() - p_x1.Y()) / (2.0 * eps);
            const double dvx = (p_y2.X() - p_y1.X()) / (2.0 * eps);
            const double dvy = (p_y2.Y() - p_y1.Y()) / (2.0 * eps);

            const double det = dux * dvy - duy * dvx;

            const int sign = (det > 1e-10) ? 1 : ((det < -1e-10) ? -1 : 0);
            if (sign == 0)
                continue;

            if (signRef == 0)
                signRef = sign;
            else if (sign != signRef)
                return false;
        }
    }
    return true;
}

std::vector<Polyline2dUtil::CornerInfo>
OccUntrimmingBuilder::DetectMergedLoopConcaveCorners(
    const MidSurfaceFaceData& inData) const
{
    std::vector<Polyline2dUtil::CornerInfo> out;

    if (inData.loops.empty())
        return out;

    // bridge ֻ֮ʣһ outer loop
    const std::vector<gp_Pnt2d>& mergedLoop = inData.loops.front().points;
    if (mergedLoop.size() < 4)
        return out;

    // ֵ֮Ž UntrimBuildOptions
    const double minTurnAngleDeg = 20.0;
    const double minEdgeLen = 1e-6;

    out = Polyline2dUtil::DetectConcaveCorners(
        mergedLoop,
        minTurnAngleDeg,
        minEdgeLen,
        1e-8);

    return out;
}

void OccUntrimmingBuilder::PrintConcaveCorners(
    const std::vector<Polyline2dUtil::CornerInfo>& corners) const
{
    if (!m_opt.verbose)
        return;

    std::cout << "[OccUntrimmingBuilder] concave corner count = "
        << corners.size() << std::endl;

    for (size_t i = 0; i < corners.size(); ++i)
    {
        const auto& c = corners[i];
        std::cout << "  #" << i
            << " idx=" << c.index
            << " uv=(" << c.point.X() << ", " << c.point.Y() << ")"
            << " signedTurn=" << c.signedTurn
            << " turnAngleDeg=" << (c.turnAngle * 180.0 / M_PI)
            << std::endl;
    }
}
std::vector<DebugLoop2d> OccUntrimmingBuilder::ConvertCellsToDebugLoops(
    const std::vector<UVPolygonCell>& cells) const
{
    std::vector<DebugLoop2d> out;
    out.reserve(cells.size());

    for (const auto& cell : cells)
    {
        DebugLoop2d loop;
        loop.points = cell.boundary;
        loop.isOuter = true;
        OccUntrimUtil::EnsureClosed(loop.points);
        out.push_back(loop);
    }

    return out;
}

DebugLoop2d OccUntrimmingBuilder::ConvertQuadPatchToDebugLoop(
    const UVQuadPatch& q) const
{
    DebugLoop2d loop;
    loop.isOuter = true;
    loop.points.clear();

    loop.points.insert(loop.points.end(), q.south.begin(), q.south.end());

    if (!q.east.empty())
    {
        for (size_t i = 1; i < q.east.size(); ++i)
            loop.points.push_back(q.east[i]);
    }

    if (!q.north.empty())
    {
        for (int i = static_cast<int>(q.north.size()) - 2; i >= 0; --i)
            loop.points.push_back(q.north[i]);
    }

    if (!q.west.empty())
    {
        for (int i = static_cast<int>(q.west.size()) - 2; i > 0; --i)
            loop.points.push_back(q.west[i]);
    }

    OccUntrimUtil::EnsureClosed(loop.points);
    return loop;
}

std::vector<DebugLoop2d> OccUntrimmingBuilder::ConvertQuadPatchesToDebugLoops(
    const std::vector<UVQuadPatch>& patches) const
{
    std::vector<DebugLoop2d> out;
    out.reserve(patches.size());

    for (const auto& q : patches)
        out.push_back(ConvertQuadPatchToDebugLoop(q));

    return out;
}

void OccUntrimmingBuilder::EnsureLoopClosedWithInfos(UVCurveLoop& loop, double tol) const
{
    if (loop.points.empty())
        return;

    if (!OccUntrimUtil::IsClosed(loop.points, tol))
    {
        loop.points.push_back(loop.points.front());

        if (!loop.pointInfos.empty())
            loop.pointInfos.push_back(loop.pointInfos.front());
    }
}

void OccUntrimmingBuilder::EnsureLoopCCWWithInfos(UVCurveLoop& loop) const
{
    EnsureLoopClosedWithInfos(loop);

    if (OccUntrimUtil::PolygonSignedArea(loop.points) < 0.0)
    {
        std::reverse(loop.points.begin(), loop.points.end());
        if (!loop.pointInfos.empty())
            std::reverse(loop.pointInfos.begin(), loop.pointInfos.end());
    }
}

void OccUntrimmingBuilder::EnsureLoopCWWithInfos(UVCurveLoop& loop) const
{
    EnsureLoopClosedWithInfos(loop);

    if (OccUntrimUtil::PolygonSignedArea(loop.points) > 0.0)
    {
        std::reverse(loop.points.begin(), loop.points.end());
        if (!loop.pointInfos.empty())
            std::reverse(loop.pointInfos.begin(), loop.pointInfos.end());
    }
}

bool OccUntrimmingBuilder::ExtractFaceLoops2d(
    const TopoDS_Face& face,
    std::vector<UVCurveLoop>& loops)
{
    loops.clear();

    TopExp_Explorer wireExp(face, TopAbs_WIRE);
    for (; wireExp.More(); wireExp.Next())
    {
        const TopoDS_Wire& wire = TopoDS::Wire(wireExp.Current());

        std::vector<gp_Pnt2d> loopPts;
        std::vector<UVPointInfo> loopInfos;
        if (!ExtractWireLoop2d(face, wire, loopPts, loopInfos))
            continue;

        if (loopPts.size() < 4)
            continue;

        UVCurveLoop loop;
        loop.points = loopPts;
        loop.pointInfos = loopInfos;
        EnsureLoopClosedWithInfos(loop);

        if (loop.points.size() != loop.pointInfos.size())
            continue;

        loops.push_back(loop);
    }

    if (loops.empty())
        return false;

    int outerIdx = -1;
    double maxArea = -1.0;
    for (int i = 0; i < static_cast<int>(loops.size()); ++i)
    {
        const double area = std::abs(OccUntrimUtil::PolygonSignedArea(loops[i].points));
        if (area > maxArea)
        {
            maxArea = area;
            outerIdx = i;
        }
    }

    for (int i = 0; i < static_cast<int>(loops.size()); ++i)
    {
        loops[i].isOuter = (i == outerIdx);
        if (loops[i].isOuter)
            EnsureLoopCCWWithInfos(loops[i]);
        else
            EnsureLoopCWWithInfos(loops[i]);
    }

    return true;
}

bool OccUntrimmingBuilder::ExtractWireLoop2d(
    const TopoDS_Face& face,
    const TopoDS_Wire& wire,
    std::vector<gp_Pnt2d>& outLoop,
    std::vector<UVPointInfo>& outInfos)
{
    outLoop.clear();
    outInfos.clear();

    BRepTools_WireExplorer explorer(wire, face);
    bool firstEdge = true;
    int edgeIndex = 0;

    for (; explorer.More(); explorer.Next(), ++edgeIndex)
    {
        const TopoDS_Edge& edge = explorer.Current();

        std::vector<gp_Pnt2d> edgePts;
        std::vector<UVPointInfo> edgeInfos;
        if (!SampleEdgePCurve(face, edge, edgeIndex, edgePts, edgeInfos, m_opt.boundarySampleCountPerEdge))
            continue;

        if (edgePts.size() < 2 || edgePts.size() != edgeInfos.size())
            continue;

        if (!firstEdge && !outLoop.empty())
        {
            const double dFront = OccUntrimUtil::Distance2d(outLoop.back(), edgePts.front());
            const double dBack = OccUntrimUtil::Distance2d(outLoop.back(), edgePts.back());

            // 如果尾点更接近当前已有 loop 末端，说明这条 edge 的采样方向反了
            if (dBack < dFront)
                ReverseEdgeSamples(edgePts, edgeInfos);

            if (OccUntrimUtil::Distance2d(outLoop.back(), edgePts.front()) < 1e-8)
            {
                outLoop.insert(outLoop.end(), edgePts.begin() + 1, edgePts.end());
                outInfos.insert(outInfos.end(), edgeInfos.begin() + 1, edgeInfos.end());
            }
            else
            {
                outLoop.insert(outLoop.end(), edgePts.begin(), edgePts.end());
                outInfos.insert(outInfos.end(), edgeInfos.begin(), edgeInfos.end());
            }
        }
        else
        {
            outLoop.insert(outLoop.end(), edgePts.begin(), edgePts.end());
            outInfos.insert(outInfos.end(), edgeInfos.begin(), edgeInfos.end());
            firstEdge = false;
        }
    }

    if (outLoop.size() < 2 || outLoop.size() != outInfos.size())
        return false;

    if (!OccUntrimUtil::IsClosed(outLoop))
    {
        outLoop.push_back(outLoop.front());
        outInfos.push_back(outInfos.front());
    }

    return true;
}

bool OccUntrimmingBuilder::SampleEdgePCurve(
    const TopoDS_Face& face,
    const TopoDS_Edge& edge,
    int edgeIndex,
    std::vector<gp_Pnt2d>& outPts,
    std::vector<UVPointInfo>& outInfos,
    int sampleCount)
{
    outPts.clear();
    outInfos.clear();

    Standard_Real first = 0.0, last = 0.0;
    Handle(Geom2d_Curve) c2d = BRep_Tool::CurveOnSurface(edge, face, first, last);
    if (c2d.IsNull())
        return false;

    // 按 edge 方向修正 pcurve 采样方向
    if (edge.Orientation() == TopAbs_REVERSED)
    {
        std::swap(first, last);
    }

    if (sampleCount < 2)
        sampleCount = 2;

    Geom2dAdaptor_Curve adaptor(c2d, std::min(first, last), std::max(first, last));
    const GeomAbs_CurveType type = adaptor.GetType();
    const UVPointCurveType uvType = ToUVPointCurveType(type);

    auto PushUnique = [&](const gp_Pnt2d& p, double param, bool isStart, bool isEnd)
        {
            if (!outPts.empty() && OccUntrimUtil::Distance2d(outPts.back(), p) <= 1.0e-8)
            {
                //UpdateUvPointType(outPts.back(), uvType);
                return;
            }

            outPts.push_back(p);

            UVPointInfo info;
            info.uv = p;
            info.edgeIndex = edgeIndex;
            info.edgeParam = param;
            info.isEdgeStart = isStart;
            info.isEdgeEnd = isEnd;
            info.isTopoVertex = (isStart || isEnd);
            outInfos.push_back(info);

            UpdateUvPointType(p, uvType);
        };

    auto PushLine3Pts = [&](const gp_Pnt2d& p0, const gp_Pnt2d& p1)
        {
            PushUnique(p0, first, true, false);

            const double mid = 0.5 * (first + last);
            PushUnique(
                gp_Pnt2d(0.5 * (p0.X() + p1.X()), 0.5 * (p0.Y() + p1.Y())),
                mid, false, false);

            PushUnique(p1, last, false, true);
        };

    auto PushCurveSamples = [&](int n)
        {
            n = std::max(2, n);
            for (int i = 0; i < n; ++i)
            {
                const double t = static_cast<double>(i) / static_cast<double>(n - 1);
                const double param = OccUntrimUtil::Lerp(first, last, t);
                const gp_Pnt2d p = c2d->Value(param);

                PushUnique(p, param, i == 0, i == n - 1);
            }
        };

    if (type == GeomAbs_Line)
    {
        PushCurveSamples(10);
        return outPts.size() >= 2 && outPts.size() == outInfos.size();
    }

    if (type == GeomAbs_Circle)
    {
        const double kPi = 3.14159265358979323846;
        const double span = std::abs(last - first);

        if (std::abs(span - 2.0 * kPi) < 1.0e-6)
        {
            for (int i = 0; i < 16; ++i)
            {
                const double t = static_cast<double>(i) / 16.0;
                const double param = OccUntrimUtil::Lerp(first, last, t);
                PushUnique(c2d->Value(param), param, false, false);
            }
            return outPts.size() >= 4 && outPts.size() == outInfos.size();
        }

        const double smallArcTol = 15.0 * kPi / 180.0;
        if (span < smallArcTol)
        {
            const gp_Pnt2d p0 = c2d->Value(first);
            const gp_Pnt2d p1 = c2d->Value(last);
            PushLine3Pts(p0, p1);
            return outPts.size() >= 2 && outPts.size() == outInfos.size();
        }

        PushCurveSamples(10);
        return outPts.size() >= 3 && outPts.size() == outInfos.size();
    }

    PushCurveSamples(sampleCount);
    return outPts.size() >= 2 && outPts.size() == outInfos.size();
}
void OccUntrimmingBuilder::UpdateUvPointType(
    const gp_Pnt2d& uv,
    UVPointCurveType newType)
{
    auto it = m_uvPntTypeMap.find(uv);
    if (it == m_uvPntTypeMap.end())
    {
        m_uvPntTypeMap[uv] = newType;
        return;
    }

    UVPointCurveType& oldType = it->second;

    if (oldType == newType)
        return;

    /*if (oldType == UVPointCurveType::Unknown)
    {
        oldType = newType;
        return;
    }

    if (newType == UVPointCurveType::Unknown)
        return;*/

    // 只要出现不同类型，就升级成 Mixed
    oldType = UVPointCurveType::Mixed;
}

UVPointCurveType OccUntrimmingBuilder::GetUvPointType(const gp_Pnt2d& uv) const
{
    auto it = m_uvPntTypeMap.find(uv);
    if (it == m_uvPntTypeMap.end())
        return UVPointCurveType::Unknown;

    return it->second;
}
UVPointCurveType OccUntrimmingBuilder::ToUVPointCurveType(GeomAbs_CurveType type) const
{
    switch (type)
    {
    case GeomAbs_Line:
        return UVPointCurveType::Line;
    case GeomAbs_Circle:
        return UVPointCurveType::Circle;
    case GeomAbs_BSplineCurve:
        return UVPointCurveType::BSpline;
    default:
        return UVPointCurveType::Other;
    }
}

GeomAbs_CurveType OccUntrimmingBuilder::GetEdgeCurveType(
    const TopoDS_Edge& edge) const
{
    BRepAdaptor_Curve adaptor(edge);
    return adaptor.GetType();
}


std::vector<int> OccUntrimmingBuilder::DetectSalientCorners(
    const UVCurveLoop& loopData,
    double angleThresholdDeg,
    double lenTol) const
{
    std::vector<int> corners;

    const std::vector<gp_Pnt2d> loop = RemoveDuplicateClosingPoint(loopData.points);
    const int n = static_cast<int>(loop.size());
    if (n < 3)
        return corners;

    const double angleThresholdRad =
        angleThresholdDeg * 3.14159265358979323846 / 180.0;

    const std::vector<double> angles = ComputeVertexTurningAngles(loop, lenTol);

    std::vector<int> rawCorners;
    for (int i = 0; i < n; ++i)
    {
        bool isCorner = false;


        const UVPointCurveType t = GetUvPointType(loop[i]);

        if (angles[i] >= angleThresholdRad /*&& (t !=UVPointCurveType::Circle ) && (t != UVPointCurveType::BSpline)*/)
            isCorner = true;

        if (!isCorner && t == UVPointCurveType::Mixed)
            isCorner = true;

        if (isCorner)
            rawCorners.push_back(i);
    }

    if (rawCorners.empty())
    {
        std::vector<int> ex = Polyline2dUtil::BuildExtremeCandidateIndices(loop);
        std::sort(ex.begin(), ex.end());
        ex.erase(std::unique(ex.begin(), ex.end()), ex.end());
        return ex;
    }
    std::sort(rawCorners.begin(), rawCorners.end());
    rawCorners.erase(std::unique(rawCorners.begin(), rawCorners.end()), rawCorners.end());
    return rawCorners;
    // 先试：如果恰好检测到4个原始角点，直接返回
    if (rawCorners.size() == 4)
    {
        
    }

    // 小 polygon 不做或少做 merge
    int mergeWindow = 2;
    if (n <= 6)
        mergeWindow = 0;
    else if (n <= 10)
        mergeWindow = 1;

    const bool hasInfos =
        (!loopData.pointInfos.empty() &&
            loopData.pointInfos.size() >= static_cast<size_t>(n));

    for (int idx : rawCorners)
    {
        if (corners.empty())
        {
            corners.push_back(idx);
            continue;
        }

        const int last = corners.back();
        int diff = std::abs(idx - last);
        diff = std::min(diff, n - diff);

        bool allowMerge = (mergeWindow > 0 && diff <= mergeWindow);

        // 只有同一条边上的非 topo 顶点才允许 merge
        if (allowMerge && hasInfos)
            allowMerge = CanMergeCornerCandidates(last, idx, loopData.pointInfos);

        if (allowMerge)
        {
            if (angles[idx] > angles[last])
                corners.back() = idx;
        }
        else
        {
            corners.push_back(idx);
        }
    }

    // 处理首尾闭合相邻
    if (mergeWindow > 0 && corners.size() >= 2)
    {
        const int first = corners.front();
        const int last = corners.back();

        int diff = std::abs(first - last);
        diff = std::min(diff, n - diff);

        bool allowMerge = (diff <= mergeWindow);

        if (allowMerge && hasInfos)
            allowMerge = CanMergeCornerCandidates(first, last, loopData.pointInfos);

        if (allowMerge)
        {
            if (angles[last] > angles[first])
                corners.front() = last;
            corners.pop_back();
        }
    }

    std::sort(corners.begin(), corners.end());
    corners.erase(std::unique(corners.begin(), corners.end()), corners.end());

    // 再兜底
    if (corners.size() < 2)
    {
        std::vector<int> ex = Polyline2dUtil::BuildExtremeCandidateIndices(loop);
        std::sort(ex.begin(), ex.end());
        ex.erase(std::unique(ex.begin(), ex.end()), ex.end());
        return ex;
    }

    return corners;
}

std::vector<int> OccUntrimmingBuilder::DetectSalientCorners(
    const std::vector<gp_Pnt2d>& poly,
    double angleThresholdDeg,
    double lenTol) const
{
    UVCurveLoop loop;
    loop.points = poly;
    return DetectSalientCorners(loop, angleThresholdDeg, lenTol);
}

LogicalPolygon2d OccUntrimmingBuilder::ExtractLogicalPolygon(
    const UVCurveLoop& loopData,
    double angleThresholdDeg,
    double lenTol) const
{
    LogicalPolygon2d out;

    const std::vector<gp_Pnt2d> loop = RemoveDuplicateClosingPoint(loopData.points);
    if (loop.size() < 3)
        return out;

    out.cornerIndices = DetectSalientCorners(loopData, angleThresholdDeg, lenTol);
    if (out.cornerIndices.size() < 2)
        return out;

    const int m = static_cast<int>(out.cornerIndices.size());
    for (int k = 0; k < m; ++k)
    {
        const int i0 = out.cornerIndices[k];
        const int i1 = out.cornerIndices[(k + 1) % m];

        LogicalSide2d side;
        side.points = ExtractPathBetweenCorners(loop, i0, i1);
        if (side.points.size() >= 2)
            out.sides.push_back(side);
    }

    return out;
}

LogicalPolygon2d OccUntrimmingBuilder::ExtractLogicalPolygon(
    const std::vector<gp_Pnt2d>& poly,
    double angleThresholdDeg,
    double lenTol) const
{
    UVCurveLoop loopData;
    loopData.points = poly;
    return ExtractLogicalPolygon(loopData, angleThresholdDeg, lenTol);
}

bool OccUntrimmingBuilder::CanMergeCornerCandidates(
    int idxA,
    int idxB,
    const std::vector<UVPointInfo>& infos) const
{
    if (idxA < 0 || idxB < 0 ||
        idxA >= static_cast<int>(infos.size()) ||
        idxB >= static_cast<int>(infos.size()))
        return false;

    const UVPointInfo& a = infos[idxA];
    const UVPointInfo& b = infos[idxB];

    // 不同边，绝对不能合并
    if (a.edgeIndex != b.edgeIndex)
        return false;

    // 真实拓扑顶点，不能合并
    if (a.isTopoVertex || b.isTopoVertex)
        return false;

    return true;
}


const std::map<gp_Pnt2d, UVPointCurveType, UVPnt2dLess>&
OccUntrimmingBuilder::GetUvPointTypeMap() const
{
    return m_uvPntTypeMap;
}