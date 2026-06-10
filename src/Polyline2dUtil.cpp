#include "Polyline2dUtil.h"

#include <set>

namespace Polyline2dUtil
{
    double Distance(const gp_Pnt2d& a, const gp_Pnt2d& b)
    {
        const double dx = a.X() - b.X();
        const double dy = a.Y() - b.Y();
        return std::sqrt(dx * dx + dy * dy);
    }

    gp_Pnt2d Lerp(const gp_Pnt2d& a, const gp_Pnt2d& b, double t)
    {
        return gp_Pnt2d(
            a.X() * (1.0 - t) + b.X() * t,
            a.Y() * (1.0 - t) + b.Y() * t
        );
    }

    double Cross(const gp_Pnt2d& a, const gp_Pnt2d& b, const gp_Pnt2d& c)
    {
        const double x1 = b.X() - a.X();
        const double y1 = b.Y() - a.Y();
        const double x2 = c.X() - a.X();
        const double y2 = c.Y() - a.Y();
        return x1 * y2 - y1 * x2;
    }

    double Dot(const gp_Pnt2d& a, const gp_Pnt2d& b, const gp_Pnt2d& c)
    {
        const double x1 = b.X() - a.X();
        const double y1 = b.Y() - a.Y();
        const double x2 = c.X() - a.X();
        const double y2 = c.Y() - a.Y();
        return x1 * x2 + y1 * y2;
    }

    bool IsSamePoint(const gp_Pnt2d& a, const gp_Pnt2d& b, double tol)
    {
        return Distance(a, b) <= tol;
    }

    bool IsPointInBBox(const gp_Pnt2d& p,
        const gp_Pnt2d& a,
        const gp_Pnt2d& b,
        double tol)
    {
        const double xmin = std::min(a.X(), b.X()) - tol;
        const double xmax = std::max(a.X(), b.X()) + tol;
        const double ymin = std::min(a.Y(), b.Y()) - tol;
        const double ymax = std::max(a.Y(), b.Y()) + tol;

        return (p.X() >= xmin && p.X() <= xmax &&
            p.Y() >= ymin && p.Y() <= ymax);
    }

    bool IsPointOnSegment(const gp_Pnt2d& p,
        const gp_Pnt2d& a,
        const gp_Pnt2d& b,
        double tol)
    {
        if (std::abs(Cross(a, b, p)) > tol)
            return false;

        return IsPointInBBox(p, a, b, tol);
    }

    SegmentIntersectType SegmentIntersectionType(
        const gp_Pnt2d& a0, const gp_Pnt2d& a1,
        const gp_Pnt2d& b0, const gp_Pnt2d& b1,
        double tol)
    {
        const double c1 = Cross(a0, a1, b0);
        const double c2 = Cross(a0, a1, b1);
        const double c3 = Cross(b0, b1, a0);
        const double c4 = Cross(b0, b1, a1);

        const bool straddle1 = ((c1 > tol && c2 < -tol) || (c1 < -tol && c2 > tol));
        const bool straddle2 = ((c3 > tol && c4 < -tol) || (c3 < -tol && c4 > tol));

        if (straddle1 && straddle2)
            return SegmentIntersectType::Proper;

        // 端点接触 / 共线重叠
        if (std::abs(c1) <= tol && IsPointOnSegment(b0, a0, a1, tol))
            return SegmentIntersectType::Touch;
        if (std::abs(c2) <= tol && IsPointOnSegment(b1, a0, a1, tol))
            return SegmentIntersectType::Touch;
        if (std::abs(c3) <= tol && IsPointOnSegment(a0, b0, b1, tol))
            return SegmentIntersectType::Touch;
        if (std::abs(c4) <= tol && IsPointOnSegment(a1, b0, b1, tol))
            return SegmentIntersectType::Touch;

        return SegmentIntersectType::None;
    }

    bool SegmentIntersect(
        const gp_Pnt2d& a0, const gp_Pnt2d& a1,
        const gp_Pnt2d& b0, const gp_Pnt2d& b1,
        double tol)
    {
        return SegmentIntersectionType(a0, a1, b0, b1, tol) != SegmentIntersectType::None;
    }

    bool SegmentProperIntersect(
        const gp_Pnt2d& a0, const gp_Pnt2d& a1,
        const gp_Pnt2d& b0, const gp_Pnt2d& b1,
        double tol)
    {
        return SegmentIntersectionType(a0, a1, b0, b1, tol) == SegmentIntersectType::Proper;
    }

    bool IsClosed(const std::vector<gp_Pnt2d>& polyline, double tol)
    {
        if (polyline.size() < 2)
            return false;
        return IsSamePoint(polyline.front(), polyline.back(), tol);
    }

    void EnsureClosed(std::vector<gp_Pnt2d>& polyline, double tol)
    {
        if (polyline.empty())
            return;
        if (!IsClosed(polyline, tol))
            polyline.push_back(polyline.front());
    }

    double SignedArea(const std::vector<gp_Pnt2d>& polygon)
    {
        if (polygon.size() < 3)
            return 0.0;

        double area = 0.0;
        const int n = static_cast<int>(polygon.size());

        int m = n;
        if (n >= 2 && IsSamePoint(polygon.front(), polygon.back()))
            m = n - 1;

        for (int i = 0; i < m; ++i)
        {
            const gp_Pnt2d& p0 = polygon[i];
            const gp_Pnt2d& p1 = polygon[(i + 1) % m];
            area += p0.X() * p1.Y() - p1.X() * p0.Y();
        }
        return 0.5 * area;
    }

    bool PointInPolygon(const gp_Pnt2d& p,
        const std::vector<gp_Pnt2d>& polygon,
        double tol)
    {
        if (polygon.size() < 3)
            return false;

        int n = static_cast<int>(polygon.size());
        int m = n;
        if (n >= 2 && IsSamePoint(polygon.front(), polygon.back(), tol))
            m = n - 1;

        // 若点恰在边上，认为在内
        for (int i = 0; i < m; ++i)
        {
            const gp_Pnt2d& a = polygon[i];
            const gp_Pnt2d& b = polygon[(i + 1) % m];
            if (IsPointOnSegment(p, a, b, tol))
                return true;
        }

        bool inside = false;
        for (int i = 0, j = m - 1; i < m; j = i++)
        {
            const double xi = polygon[i].X();
            const double yi = polygon[i].Y();
            const double xj = polygon[j].X();
            const double yj = polygon[j].Y();

            const bool intersect =
                ((yi > p.Y()) != (yj > p.Y())) &&
                (p.X() < (xj - xi) * (p.Y() - yi) / ((yj - yi) + 1e-20) + xi);

            if (intersect)
                inside = !inside;
        }

        return inside;
    }

    bool SegmentIntersectsPolyline(
        const gp_Pnt2d& a,
        const gp_Pnt2d& b,
        const std::vector<gp_Pnt2d>& polyline,
        double tol)
    {
        if (polyline.size() < 2)
            return false;

        for (size_t i = 0; i + 1 < polyline.size(); ++i)
        {
            if (SegmentIntersect(a, b, polyline[i], polyline[i + 1], tol))
                return true;
        }
        return false;
    }

    bool SegmentProperIntersectsPolyline(
        const gp_Pnt2d& a,
        const gp_Pnt2d& b,
        const std::vector<gp_Pnt2d>& polyline,
        double tol)
    {
        if (polyline.size() < 2)
            return false;

        for (size_t i = 0; i + 1 < polyline.size(); ++i)
        {
            if (SegmentProperIntersect(a, b, polyline[i], polyline[i + 1], tol))
                return true;
        }
        return false;
    }

    bool PolylineSelfIntersect(
        const std::vector<gp_Pnt2d>& polyline,
        double tol)
    {
        if (polyline.size() < 4)
            return false;

        const int segCount = static_cast<int>(polyline.size()) - 1;
        const bool closed = IsClosed(polyline, tol);

        for (int i = 0; i < segCount; ++i)
        {
            const gp_Pnt2d& a0 = polyline[i];
            const gp_Pnt2d& a1 = polyline[i + 1];

            for (int j = i + 1; j < segCount; ++j)
            {
                // 相邻边共享顶点，跳过
                if (j == i + 1)
                    continue;

                // 首尾边闭合时也共享顶点，跳过
                if (closed && i == 0 && j == segCount - 1)
                    continue;

                const gp_Pnt2d& b0 = polyline[j];
                const gp_Pnt2d& b1 = polyline[j + 1];

                if (SegmentIntersect(a0, a1, b0, b1, tol))
                    return true;
            }
        }
        return false;
    }

    Segment2d ShrinkSegment(
        const gp_Pnt2d& a,
        const gp_Pnt2d& b,
        double ratio)
    {
        ratio = std::max(0.0, std::min(0.49, ratio));

        Segment2d seg;
        seg.p0 = Lerp(a, b, ratio);
        seg.p1 = Lerp(b, a, ratio);
        return seg;
    }

    bool SegmentHitsLoopExceptEndpoints(
        const gp_Pnt2d& a,
        const gp_Pnt2d& b,
        const std::vector<gp_Pnt2d>& loop,
        double tol)
    {
        if (loop.size() < 2)
            return false;

        // 缩短线段，避免端点恰落在边界上的误判
        const Segment2d shrunk = ShrinkSegment(a, b, 1e-4);

        for (size_t i = 0; i + 1 < loop.size(); ++i)
        {
            const gp_Pnt2d& p0 = loop[i];
            const gp_Pnt2d& p1 = loop[i + 1];

            if (SegmentIntersect(shrunk.p0, shrunk.p1, p0, p1, tol))
                return true;
        }
        return false;
    }

    bool SegmentHitsAnyLoopExceptSelf(
        const gp_Pnt2d& a,
        const gp_Pnt2d& b,
        const std::vector<std::vector<gp_Pnt2d>>& loops,
        int selfLoopIndex,
        double tol)
    {
        for (int i = 0; i < static_cast<int>(loops.size()); ++i)
        {
            if (i == selfLoopIndex)
                continue;

            if (SegmentHitsLoopExceptEndpoints(a, b, loops[i], tol))
                return true;
        }
        return false;
    }

    int FindLeftmostIndex(const std::vector<gp_Pnt2d>& pts)
    {
        if (pts.empty()) return -1;
        int idx = 0;
        for (int i = 1; i < static_cast<int>(pts.size()); ++i)
        {
            if (pts[i].X() < pts[idx].X())
                idx = i;
        }
        return idx;
    }

    int FindRightmostIndex(const std::vector<gp_Pnt2d>& pts)
    {
        if (pts.empty()) return -1;
        int idx = 0;
        for (int i = 1; i < static_cast<int>(pts.size()); ++i)
        {
            if (pts[i].X() > pts[idx].X())
                idx = i;
        }
        return idx;
    }

    int FindTopmostIndex(const std::vector<gp_Pnt2d>& pts)
    {
        if (pts.empty()) return -1;
        int idx = 0;
        for (int i = 1; i < static_cast<int>(pts.size()); ++i)
        {
            if (pts[i].Y() > pts[idx].Y())
                idx = i;
        }
        return idx;
    }

    int FindBottommostIndex(const std::vector<gp_Pnt2d>& pts)
    {
        if (pts.empty()) return -1;
        int idx = 0;
        for (int i = 1; i < static_cast<int>(pts.size()); ++i)
        {
            if (pts[i].Y() < pts[idx].Y())
                idx = i;
        }
        return idx;
    }

    std::vector<int> BuildExtremeCandidateIndices(const std::vector<gp_Pnt2d>& pts)
    {
        std::vector<int> out;
        if (pts.empty())
            return out;

        std::set<int> idxSet;
        idxSet.insert(FindLeftmostIndex(pts));
        idxSet.insert(FindRightmostIndex(pts));
        idxSet.insert(FindTopmostIndex(pts));
        idxSet.insert(FindBottommostIndex(pts));

        for (int idx : idxSet)
        {
            if (idx >= 0)
                out.push_back(idx);
        }
        return out;
    }
    
    std::vector<gp_Pnt2d> RemoveDuplicateClosingPoint(
        const std::vector<gp_Pnt2d>& polyline,
        double tol)
    {
        std::vector<gp_Pnt2d> out = polyline;
        if (out.size() >= 2 && IsSamePoint(out.front(), out.back(), tol))
            out.pop_back();
        return out;
    }

    bool EnsureCCW(std::vector<gp_Pnt2d>& polygon, double tol)
    {
        EnsureClosed(polygon, tol);
        if (polygon.size() < 4)
            return false;

        if (SignedArea(polygon) < 0.0)
            std::reverse(polygon.begin(), polygon.end());

        return true;
    }

    double VertexTurnAngle(
        const gp_Pnt2d& prev,
        const gp_Pnt2d& curr,
        const gp_Pnt2d& next)
    {
        const double v1x = curr.X() - prev.X();
        const double v1y = curr.Y() - prev.Y();
        const double v2x = next.X() - curr.X();
        const double v2y = next.Y() - curr.Y();

        const double len1 = std::sqrt(v1x * v1x + v1y * v1y);
        const double len2 = std::sqrt(v2x * v2x + v2y * v2y);

        if (len1 <= 1e-20 || len2 <= 1e-20)
            return 0.0;

        double c = (v1x * v2x + v1y * v2y) / (len1 * len2);
        c = std::max(-1.0, std::min(1.0, c));
        return std::acos(c);
    }

    double VertexSignedTurn(
        const gp_Pnt2d& prev,
        const gp_Pnt2d& curr,
        const gp_Pnt2d& next)
    {
        const double v1x = curr.X() - prev.X();
        const double v1y = curr.Y() - prev.Y();
        const double v2x = next.X() - curr.X();
        const double v2y = next.Y() - curr.Y();

        const double cross = v1x * v2y - v1y * v2x;
        const double dot = v1x * v2x + v1y * v2y;
        return std::atan2(cross, dot); // 范围 (-pi, pi)
    }

    std::vector<CornerInfo> DetectConcaveCorners(
        const std::vector<gp_Pnt2d>& closedLoop,
        double minTurnAngleDeg,
        double minEdgeLen,
        double tol)
    {
        std::vector<CornerInfo> out;

        if (closedLoop.size() < 4)
            return out;

        // 去闭合重复点
        std::vector<gp_Pnt2d> loop = RemoveDuplicateClosingPoint(closedLoop, tol);
        if (loop.size() < 3)
            return out;

        // 统一成 CCW，之后 signedTurn < 0 视为凹角
        std::vector<gp_Pnt2d> ccwLoop = loop;
        EnsureCCW(ccwLoop, tol);

        const double minTurnAngleRad = minTurnAngleDeg * M_PI / 180.0;
        const int n = static_cast<int>(ccwLoop.size());

        for (int i = 0; i < n; ++i)
        {
            const gp_Pnt2d& prev = ccwLoop[(i - 1 + n) % n];
            const gp_Pnt2d& curr = ccwLoop[i];
            const gp_Pnt2d& next = ccwLoop[(i + 1) % n];

            const double len1 = Distance(prev, curr);
            const double len2 = Distance(curr, next);

            if (len1 < minEdgeLen || len2 < minEdgeLen)
                continue;

            const double signedTurn = VertexSignedTurn(prev, curr, next);
            const double turnAngle = std::abs(signedTurn);

            CornerInfo info;
            info.index = i;
            info.point = curr;
            info.signedTurn = signedTurn;
            info.turnAngle = turnAngle;
            info.isConcave = (signedTurn < 0.0 && turnAngle >= minTurnAngleRad);

            if (info.isConcave)
                out.push_back(info);
        }

        return out;
    }
}