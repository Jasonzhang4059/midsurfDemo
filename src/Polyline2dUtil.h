#pragma once

#include <vector>
#include <cmath>
#include <algorithm>

#include <gp_Pnt2d.hxx>

namespace Polyline2dUtil
{
    enum class SegmentIntersectType
    {
        None,
        Touch,
        Proper
    };

    struct Segment2d
    {
        gp_Pnt2d p0;
        gp_Pnt2d p1;
    };

    struct CornerInfo
    {
        int index = -1;              // 对应 polyline 顶点索引（去掉闭合重复点后的索引）
        gp_Pnt2d point;              // 顶点坐标
        double signedTurn = 0.0;     // 有符号转角，CCW loop 下 <0 常视为凹角
        double turnAngle = 0.0;      // 转角绝对值，单位 rad，范围 [0, pi]
        bool isConcave = false;
    };

    double Distance(const gp_Pnt2d& a, const gp_Pnt2d& b);

    gp_Pnt2d Lerp(const gp_Pnt2d& a, const gp_Pnt2d& b, double t);

    double Cross(const gp_Pnt2d& a, const gp_Pnt2d& b, const gp_Pnt2d& c);
    double Dot(const gp_Pnt2d& a, const gp_Pnt2d& b, const gp_Pnt2d& c);

    bool IsSamePoint(const gp_Pnt2d& a, const gp_Pnt2d& b, double tol = 1e-8);

    bool IsPointInBBox(const gp_Pnt2d& p,
        const gp_Pnt2d& a,
        const gp_Pnt2d& b,
        double tol = 1e-8);

    bool IsPointOnSegment(const gp_Pnt2d& p,
        const gp_Pnt2d& a,
        const gp_Pnt2d& b,
        double tol = 1e-8);

    SegmentIntersectType SegmentIntersectionType(
        const gp_Pnt2d& a0, const gp_Pnt2d& a1,
        const gp_Pnt2d& b0, const gp_Pnt2d& b1,
        double tol = 1e-8);

    bool SegmentIntersect(
        const gp_Pnt2d& a0, const gp_Pnt2d& a1,
        const gp_Pnt2d& b0, const gp_Pnt2d& b1,
        double tol = 1e-8);

    bool SegmentProperIntersect(
        const gp_Pnt2d& a0, const gp_Pnt2d& a1,
        const gp_Pnt2d& b0, const gp_Pnt2d& b1,
        double tol = 1e-8);

    bool IsClosed(const std::vector<gp_Pnt2d>& polyline, double tol = 1e-8);

    void EnsureClosed(std::vector<gp_Pnt2d>& polyline, double tol = 1e-8);

    double SignedArea(const std::vector<gp_Pnt2d>& polygon);

    bool PointInPolygon(const gp_Pnt2d& p,
        const std::vector<gp_Pnt2d>& polygon,
        double tol = 1e-8);

    bool SegmentIntersectsPolyline(
        const gp_Pnt2d& a,
        const gp_Pnt2d& b,
        const std::vector<gp_Pnt2d>& polyline,
        double tol = 1e-8);

    bool SegmentProperIntersectsPolyline(
        const gp_Pnt2d& a,
        const gp_Pnt2d& b,
        const std::vector<gp_Pnt2d>& polyline,
        double tol = 1e-8);

    bool PolylineSelfIntersect(
        const std::vector<gp_Pnt2d>& polyline,
        double tol = 1e-8);

    Segment2d ShrinkSegment(
        const gp_Pnt2d& a,
        const gp_Pnt2d& b,
        double ratio = 1e-4);

    bool SegmentHitsLoopExceptEndpoints(
        const gp_Pnt2d& a,
        const gp_Pnt2d& b,
        const std::vector<gp_Pnt2d>& loop,
        double tol = 1e-8);

    bool SegmentHitsAnyLoopExceptSelf(
        const gp_Pnt2d& a,
        const gp_Pnt2d& b,
        const std::vector<std::vector<gp_Pnt2d>>& loops,
        int selfLoopIndex,
        double tol = 1e-8);

    int FindLeftmostIndex(const std::vector<gp_Pnt2d>& pts);
    int FindRightmostIndex(const std::vector<gp_Pnt2d>& pts);
    int FindTopmostIndex(const std::vector<gp_Pnt2d>& pts);
    int FindBottommostIndex(const std::vector<gp_Pnt2d>& pts);

    std::vector<int> BuildExtremeCandidateIndices(const std::vector<gp_Pnt2d>& pts);

    // =========================
    // 凹角检测
    // =========================
    std::vector<gp_Pnt2d> RemoveDuplicateClosingPoint(
        const std::vector<gp_Pnt2d>& polyline,
        double tol = 1e-8);

    bool EnsureCCW(std::vector<gp_Pnt2d>& polygon, double tol = 1e-8);

    double VertexTurnAngle(
        const gp_Pnt2d& prev,
        const gp_Pnt2d& curr,
        const gp_Pnt2d& next);

    double VertexSignedTurn(
        const gp_Pnt2d& prev,
        const gp_Pnt2d& curr,
        const gp_Pnt2d& next);

    std::vector<CornerInfo> DetectConcaveCorners(
        const std::vector<gp_Pnt2d>& closedLoop,
        double minTurnAngleDeg = 15.0,
        double minEdgeLen = 1e-6,
        double tol = 1e-8);
}