#pragma once

#include <TopoDS_Face.hxx>
#include <BRep_Tool.hxx>
#include <Geom_Surface.hxx>
#include <GeomAPI_ProjectPointOnSurf.hxx>
#include <gp_Pnt.hxx>
#include <gp_Vec.hxx>
#include <Standard_Real.hxx>
#include <Precision.hxx>
#include <Bnd_Box.hxx>
#include <AIS_TextLabel.hxx>

#include "occWin.h"
/**
 * @brief 静态工具类，提供点与面投影、距离和法向计算
 */
class OccMidSurfCommFunction
{
public:
    static bool GetMidpoint(const gp_Pnt& p1, const gp_Pnt& p2, gp_Pnt& midpoint);
    /**
     * @brief 将点投影到面上
     * @param p 输入点
     * @param face 面
     * @param projP 输出投影点
     * @return 投影成功返回 true，否则 false
     */
    static bool ProjectPointToFace(const gp_Pnt& p,
        const TopoDS_Face& face,
        gp_Pnt& projP);

    static TopoDS_Edge ReverseEdgeWithVertices(const TopoDS_Edge& edge);

    static double CalAngleOfTwoFace(const TopoDS_Face& face1, const TopoDS_Face& face2);


    static gp_Pnt GetEdgeMidPoint(const TopoDS_Edge& edge);

    static gp_Pnt GetEdgeMidPointAndDeriv(const TopoDS_Edge& edge, gp_Vec& t);

    /**
     * @brief 计算点到面最短距离
     * @param p 输入点
     * @param face 面
     * @return 点到面的最短距离，如果失败返回大值
     */
    static double PointToFaceDistance(const gp_Pnt& p,
        const TopoDS_Face& face,
        gp_Pnt &proj);

    static bool RayIntersectFace(const gp_Pnt& origin, const gp_Vec& dir, const TopoDS_Face& face, gp_Pnt& hitPnt, double& dist);

    static bool RayIntersectFace_OCC(const gp_Pnt& origin, const gp_Vec& dir, const TopoDS_Face& face, double maxDist, gp_Pnt& hitPnt, double& dist);

    static void GetFacDiscretePnt(TopoDS_Face& fac, GpPntArr& pntArr);

    static Handle(AIS_TextLabel) ShowTextAtPoint(
        const gp_Pnt& pos,
        const TCollection_ExtendedString& text,
        const Handle(AIS_InteractiveContext)& ctx,
        double height = 20.0,

        const Quantity_Color& color = Quantity_NOC_RED);
    /**
     * @brief 获取面上指定点处的法向向量
     * @param face 面
     * @param p 面上的点（或投影点）
     * @param normal 输出法向量
     * @return 成功返回 true，否则 false
     */
    static bool GetNormalAtPoint(const TopoDS_Face& face,
        const gp_Pnt& p,
        gp_Vec& normal);

    static double CalFacArea(const TopoDS_Face& fac);

    /// XYZ 方向等量扩展包围盒
    static void ExpandBndBox(Bnd_Box& box,
        double delta);
    static bool BuildFaceFromWire(const std::vector<TopoDS_Edge>& wire, GpPntArr& midPts, TopoDS_Face& outFace, bool isRev);
    static TopoDS_Shape MergeFacesToCompound(const std::vector<TopoDS_Face>& faces);
    static void SaveToObj(const TopoDS_Shape& shape, const std::string& filename);
    static TopoDS_Face FitPlateSurface(const std::vector<TopoDS_Edge>& edges, const std::vector<gp_Pnt>& pts);
};
