#include "OccMidSurfCommFunction.h"

#include <STEPControl_Writer.hxx>
#include <IFSelect_ReturnStatus.hxx>
#include <Interface_Static.hxx>

#include <BRepExtrema_DistShapeShape.hxx>
#include <BRepMesh_IncrementalMesh.hxx>
#include <ShapeFix_Wire.hxx>
#include <TopoDS_Compound.hxx>
#include <BRep_Builder.hxx>
#include <BRepTools.hxx>
#include <BRepCheck_Analyzer.hxx>
#include <IntCurvesFace_ShapeIntersector.hxx>
#include <Geom_Surface.hxx>
#include <Geom_BSplineSurface.hxx>
#include <GeomAdaptor_HCurve.hxx>
#include <GeomPlate_MakeApprox.hxx>
#include <GeomPlate_CurveConstraint.hxx>
#include <BRepBuilderAPI_MakeWire.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRepBuilderAPI_MakeVertex.hxx>
#include <Geom_BSplineCurve.hxx>
#include <GeomAPI_PointsToBSpline.hxx>
#include <TopoDS.hxx>
#include <GeomLProp_SLProps.hxx>
#include <GCPnts_UniformAbscissa.hxx>
#include <BRepAdaptor_Curve.hxx>
#include <BRepClass_FaceClassifier.hxx>
#include <BRepFill_Filling.hxx>
#include <BRepGProp.hxx>               // BRepGProp::SurfaceProperties
#include <GProp_GProps.hxx>            // GProp_GProps 用于存储面积/质量等

#include <BRep_Tool.hxx>
#include <BRepBuilderAPI_MakeEdge.hxx>
#include <TopExp.hxx>
#include <Geom_Curve.hxx>
#include <Geom_Line.hxx>
#include <GeomAPI_IntCS.hxx>
#include <Geom_TrimmedCurve.hxx>

bool OccMidSurfCommFunction::GetMidpoint(const gp_Pnt& p1, const gp_Pnt& p2, gp_Pnt& midpoint)
{
    midpoint.SetX((p1.X() + p2.X()) * 0.5);
    midpoint.SetY((p1.Y() + p2.Y()) * 0.5);
    midpoint.SetZ((p1.Z() + p2.Z()) * 0.5);
    return true;
}
bool OccMidSurfCommFunction::ProjectPointToFace(
    const gp_Pnt& p,
    const TopoDS_Face& face,
    gp_Pnt& projP)
{
    // 1️⃣ 把点转成 Vertex
    TopoDS_Vertex vtx = BRepBuilderAPI_MakeVertex(p);

    // 2️⃣ 计算 Shape-Shape 距离
    BRepExtrema_DistShapeShape distTool(vtx, face);
    distTool.Perform();

    if (!distTool.IsDone() || distTool.NbSolution() == 0)
        return false;

    // 3️⃣ 取最近解
    projP = distTool.PointOnShape2(1);
}

Handle(AIS_TextLabel)
OccMidSurfCommFunction::ShowTextAtPoint(
    const gp_Pnt& pos,
    const TCollection_ExtendedString& text,
    const Handle(AIS_InteractiveContext)& ctx,
    double height,
    const Quantity_Color& color)
{
    if (ctx.IsNull())
        return nullptr;

    Handle(AIS_TextLabel) aText = new AIS_TextLabel();

    // 文本内容（支持中文）
    aText->SetText(text);

    // 世界坐标位置
    aText->SetPosition(pos);

    // 模型空间字体高度
    aText->SetHeight(height);

    // 颜色
    aText->SetColor(color);

    // 居中对齐（更适合标注点）
    aText->SetHJustification(Graphic3d_HTA_CENTER);
    aText->SetVJustification(Graphic3d_VTA_CENTER);

    // 显示
    ctx->Display(aText, Standard_True);
    ctx->CurrentViewer()->Redraw();
    return aText;
}


TopoDS_Edge OccMidSurfCommFunction::ReverseEdgeWithVertices(const TopoDS_Edge& edge)
{
    // 1. 取原始曲线
    Standard_Real f, l;
    Handle(Geom_Curve) curve = BRep_Tool::Curve(edge, f, l);
    if (curve.IsNull())
        return TopoDS_Edge();

    // 2. 反转几何曲线（关键）
    Handle(Geom_Curve) reversedCurve = curve->Reversed();

    // 3. 取原始 vertex
    TopoDS_Vertex vStart, vEnd;
    TopExp::Vertices(edge, vStart, vEnd);

    // 4. 用“反向曲线 + 对调 vertex”重新构造 Edge
    BRepBuilderAPI_MakeEdge mkEdge(
        reversedCurve,
        vEnd,   // ⭐ 交换
        vStart  // ⭐ 交换
    );

    if (!mkEdge.IsDone())
        return TopoDS_Edge();

    return mkEdge.Edge();
}


double OccMidSurfCommFunction::CalAngleOfTwoFace(
    const TopoDS_Face& face1,
    const TopoDS_Face& face2)
{
    // 1. 计算两个面的最近点
    BRepExtrema_DistShapeShape dist(face1, face2);
    dist.Perform();

    if (!dist.IsDone() || dist.NbSolution() < 1)
    {
        return -1.0; // 失败标记
    }

    gp_Pnt p1 = dist.PointOnShape1(1);
    gp_Pnt p2 = dist.PointOnShape2(1);

    // 2. 获取曲面
    Handle(Geom_Surface) surf1 = BRep_Tool::Surface(face1);
    Handle(Geom_Surface) surf2 = BRep_Tool::Surface(face2);

    if (surf1.IsNull() || surf2.IsNull())
    {
        return -1.0;
    }

    // 3. 最近点投影到曲面，获取 (u, v)
    GeomAPI_ProjectPointOnSurf proj1(p1, surf1);
    GeomAPI_ProjectPointOnSurf proj2(p2, surf2);

    if (!proj1.IsDone() || !proj2.IsDone())
    {
        return -1.0;
    }

    Standard_Real u1, v1, u2, v2;
    proj1.LowerDistanceParameters(u1, v1);
    proj2.LowerDistanceParameters(u2, v2);

    // 4. 计算最近点处法向量
    GeomLProp_SLProps props1(surf1, u1, v1, 1, Precision::Confusion());
    GeomLProp_SLProps props2(surf2, u2, v2, 1, Precision::Confusion());

    if (!props1.IsNormalDefined() || !props2.IsNormalDefined())
    {
        return -1.0;
    }

    gp_Dir n1 = props1.Normal();
    gp_Dir n2 = props2.Normal();

    // 5. 法向量夹角（弧度）
    double angleRad = n1.Angle(n2);

    // 6. 转换为角度制
    double angleDeg = angleRad * 180.0 / M_PI;

    return angleDeg; // [0, 180]
}

void OccMidSurfCommFunction::ExpandBndBox(
    Bnd_Box& box,
    double delta)
{
    if (box.IsVoid())
        return;

    Standard_Real xmin, ymin, zmin;
    Standard_Real xmax, ymax, zmax;
    box.Get(xmin, ymin, zmin, xmax, ymax, zmax);

    box.SetVoid();
    box.Update(xmin - delta, ymin - delta, zmin - delta,
        xmax + delta, ymax + delta, zmax + delta);
}

void SampleEdge(
    const TopoDS_Edge& edge,
    std::vector<gp_Pnt>& pts,
    int n = 20)
{
    BRepAdaptor_Curve curve(edge);

    GCPnts_UniformAbscissa abscissa(curve, n);

    for (int i = 1; i <= abscissa.NbPoints(); ++i)
    {
        gp_Pnt p;
        curve.D0(abscissa.Parameter(i), p);
        pts.push_back(p);
    }
}

TopoDS_Edge FitClosedCurve(const std::vector<gp_Pnt>& pts)
{
    TColgp_Array1OfPnt arr(1, pts.size());

    for (int i = 0; i < pts.size(); ++i)
        arr.SetValue(i + 1, pts[i]);

    GeomAPI_PointsToBSpline builder(
        arr,
        3,
        8,
        GeomAbs_C2,
        1e-3);

    Handle(Geom_BSplineCurve) curve = builder.Curve();
    curve->SetPeriodic();  // 关键：闭合

    return  BRepBuilderAPI_MakeEdge(curve);
}

double GetEdgeLength(const TopoDS_Edge& edge)
{
    GProp_GProps props;
    BRepGProp::LinearProperties(edge, props);
    return props.Mass();  // 对线来说就是长度
}

bool OccMidSurfCommFunction::BuildFaceFromWire(const std::vector<TopoDS_Edge>& wire,
    GpPntArr& midPts, TopoDS_Face& outFace, bool isRev)
{
    BRepFill_Filling filler;
    GpPntArr fittingSamplePntArr;
    double totalLen = 0.0;

    BRepBuilderAPI_MakeWire mkWire;

    for (const TopoDS_Edge& e : wire)
    {
        mkWire.Add(e);
    }
    TopoDS_Wire awire = mkWire.Wire();
    //awire.Reverse();
    TopExp_Explorer exp;
    for (exp.Init(awire, TopAbs_EDGE); exp.More(); exp.Next())
    {
        TopoDS_Edge edge = TopoDS::Edge(exp.Current());
        filler.Add(edge, GeomAbs_C0);  // 使用 C1 连续性以增强平滑性
    }

    //for (auto e : wire)
    //{
    //    filler.Add(e, GeomAbs_C0);  // 使用 C1 连续性以增强平滑性
    //}

    // 将中面点拟合到曲面
    UINT midPtsSize = midPts.size();
    UINT fittingPntNum = 5;
    UINT step = midPtsSize / fittingPntNum;
    if (step == 0) step = 1;
    for (size_t i = 0; i < midPtsSize; i += step)
    {
        filler.Add(midPts[i]);
    }

    // 构建曲面
    filler.Build();
    if (!filler.IsDone())
        return false;

    // 获取曲面
    outFace = filler.Face();
    double dist = 0;
    for (auto& pt : midPts)
    {
        gp_Pnt p;
        dist += OccMidSurfCommFunction::PointToFaceDistance(pt, outFace, p);
    }

    if (dist > 10.0)
    {
        awire.Reverse();
        TopExp_Explorer exp;
        BRepFill_Filling filler2;
        for (exp.Init(awire, TopAbs_EDGE); exp.More(); exp.Next())
        {
            TopoDS_Edge edge = TopoDS::Edge(exp.Current());
            filler2.Add(edge, GeomAbs_C0);
        }
        UINT fittingPntNum = 3;
        UINT step = midPtsSize / fittingPntNum;
        for (size_t i = 0; i < midPtsSize; i += step)
        {
            filler2.Add(midPts[i]);
        }
        filler2.Build();
        if (!filler2.IsDone())
            return false;

        // 4️⃣ 获取曲面
        outFace = filler2.Face();
        double dist = 0;
        for (auto& pt : midPts)
        {
            gp_Pnt p;
            dist += OccMidSurfCommFunction::PointToFaceDistance(pt, outFace, p);
        }
        if (dist > 10.0)
        {
            TopExp_Explorer exp;
            BRepFill_Filling filler3;
            for (exp.Init(awire, TopAbs_EDGE); exp.More(); exp.Next())
            {
                TopoDS_Edge edge = TopoDS::Edge(exp.Current());
                filler3.Add(edge, GeomAbs_C0);
            }
            filler3.Build();
            if (!filler3.IsDone())
                return false;

            // 4️⃣ 获取曲面
            outFace = filler3.Face();
        }
    }

    return true;
}

//bool OccMidSurfCommFunction::BuildFaceFromWire(const std::vector<TopoDS_Edge>& wire, 
//    GpPntArr& midPts, TopoDS_Face& outFace, bool isRev)
//{
//    BRepFill_Filling filler;
//
//    GpPntArr fittingSamplePntArr;
//    double totalLen = 0.0;
//
//    BRepBuilderAPI_MakeWire mkWire;
//
//    for (const TopoDS_Edge& e : wire)
//    {
//        mkWire.Add(e);
//        /*GpPntArr tmpArr;
//        SampleEdge(e, tmpArr);
//        fittingSamplePntArr.insert(fittingSamplePntArr.end(), tmpArr.begin(), tmpArr.end());
//        totalLen += GetEdgeLength(e);*/
//    }
//    TopoDS_Wire awire = mkWire.Wire();
//    if (isRev)
//    {
//        
//    }
//    //ShapeFix_Wire fixWire;
//    //fixWire.Load(awire);
//    //fixWire.Perform();
//
//    //awire = fixWire.Wire();
//    ////
//    //BRepCheck_Analyzer ana(awire);
//    //if (!ana.IsValid())
//    //    std::cout << "wire invalid"<<std::endl;
//
//    //std::cout<<"BRep_Tool::IsClosed(awire)：<<"<<BRep_Tool::IsClosed(awire) << std::endl;
//
//    TopExp_Explorer exp;
//    for (exp.Init(awire, TopAbs_EDGE); exp.More(); exp.Next())
//    {
//        TopoDS_Edge edge = TopoDS::Edge(exp.Current());
//        filler.Add(edge, GeomAbs_C0);
//        // 在这里处理每条边
//    }
//
//    std::cout <<"totallen :"<< totalLen << std::endl;
//    /*e = FitClosedCurve(fittingSamplePntArr);
//    filler.Add(e, GeomAbs_C0);*/
//    //std::cout <<"fittinglen :"<< GetEdgeLength(e) << std::endl;
//    //filler.Add(e, GeomAbs_C0);
//
//    UINT midPtsSize = midPts.size();
//    UINT fittingPntNum = 10;
//    // 2️⃣ 将中面点拟合到曲面
//    UINT step = midPtsSize / fittingPntNum;
//    if (step == 0)
//    {
//        step = 1;
//    }
//    for (size_t i = 0; i < midPtsSize; i += step)
//    {
//        filler.Add(midPts[i]);
//    }
//    for (auto& pt : midPts)
//    {
//        //filler.Add(pt);
//    }
//
//    //return false;
//    // 3️⃣ 构建曲面
//    filler.Build();
//    if (!filler.IsDone())
//        return false;
//
//    // 4️⃣ 获取曲面
//    outFace = filler.Face();
//    double dist = 0;
//    for (auto& pt : midPts)
//    {
//        /*BRepClass_FaceClassifier classifier;
//        classifier.Perform(outFace, p, 1e-6);
//
//        if (classifier.State() == TopAbs_OUT)
//            continue;*/
//        gp_Pnt p;
//        dist += OccMidSurfCommFunction::PointToFaceDistance(pt, outFace,p);
//    }
//    //if (dist > 1.0)
//    //{
//    //    awire.Reverse();
//    //    TopExp_Explorer exp;
//    //    BRepFill_Filling filler2;
//    //    for (exp.Init(awire, TopAbs_EDGE); exp.More(); exp.Next())
//    //    {
//    //        TopoDS_Edge edge = TopoDS::Edge(exp.Current());
//    //        filler2.Add(edge, GeomAbs_C0);
//    //        // 在这里处理每条边
//    //    }
//    //    for (auto& pt : midPts)
//    //    {
//    //        filler2.Add(pt);
//    //    }
//    //    filler2.Build();
//    //    if (!filler2.IsDone())
//    //        return false;
//
//    //    // 4️⃣ 获取曲面
//    //    outFace = filler2.Face();
//    //}
//    return true;
//}

TopoDS_Shape OccMidSurfCommFunction::MergeFacesToCompound(const std::vector<TopoDS_Face>& faces) {
    // 创建BRep_Builder对象
    BRep_Builder aBuilder;

    // 创建空的复合体
    TopoDS_Compound aComp;
    aBuilder.MakeCompound(aComp);

    // 遍历所有形状，将每个形状添加到复合体中
    for (const auto& face : faces) {
        aBuilder.Add(aComp, face);
    }

    // 返回合并后的TopoDS_Shape（即TopoDS_Compound）
    return aComp;
}

void OccMidSurfCommFunction::SaveToObj(const TopoDS_Shape& shape, const std::string& filename) {
    // 创建网格
    BRepMesh_IncrementalMesh mesh(shape, 0.1);  // 第二个参数是网格化精度

    // 打开输出文件
    std::ofstream objFile(filename);
    if (!objFile.is_open()) {
        std::cerr << "Failed to open file " << filename << std::endl;
        return;
    }

    // 遍历所有面，提取三角形数据
    TopExp_Explorer faceExplorer(shape, TopAbs_FACE);
    int vertexCount = 0;  // 顶点索引计数器
    while (faceExplorer.More()) {
        TopoDS_Face face = TopoDS::Face(faceExplorer.Current());
        faceExplorer.Next();

        // 获取该面的三角化数据
        Handle(Poly_Triangulation) triangulation = BRep_Tool::Triangulation(face, TopLoc_Location());
        if (triangulation) {
            const Poly_Array1OfTriangle& triangles = triangulation->Triangles();
            const TColgp_Array1OfPnt& nodes = triangulation->Nodes();

            // 导出顶点到 obj 文件
            std::vector<int> vertexMapping(nodes.Length() + 1, 0);  // 用于存储顶点映射，避免重复
            for (int i = 1; i <= nodes.Length(); ++i) {
                const gp_Pnt& point = nodes(i);
                objFile << "v " << point.X() << " " << point.Y() << " " << point.Z() << std::endl;
                vertexMapping[i] = ++vertexCount;  // 更新顶点索引
            }

            // 导出三角形面到 obj 文件
            for (int i = 1; i <= triangles.Length(); ++i) {
                const Poly_Triangle& triangle = triangles(i);
                int v1, v2, v3;
                triangle.Get(v1, v2, v3);

                // 重新映射三角形的顶点索引
                objFile << "f " << vertexMapping[v1] << " " << vertexMapping[v2] << " " << vertexMapping[v3] << std::endl;
            }
        }
    }

    // 关闭文件
    objFile.close();
    std::cout << "OBJ file saved to " << filename << std::endl;

    STEPControl_Writer writer;

    // 可选：设置 STEP 输出模式
    // AP203 / AP214 / AP242 等
    Interface_Static::SetCVal("write.step.schema", "AP214");

    // 将 shape 传给 writer
    IFSelect_ReturnStatus status = writer.Transfer(shape, STEPControl_AsIs);
    status = writer.Write((filename + ".stp").c_str());
}

TopoDS_Face OccMidSurfCommFunction::FitPlateSurface(
    const std::vector<TopoDS_Edge>& edges,
    const std::vector<gp_Pnt>& pts)
{
    // =================================================
    // 1️⃣ 创建 Plate
    // =================================================
    GeomPlate_BuildPlateSurface plate(
        3,   // 阶次
        15,  // 迭代
        2    // 连续性
    );

    // =================================================
    // 2️⃣ 加入边界约束
    // =================================================
    BRepBuilderAPI_MakeWire mkWire;

    for (const TopoDS_Edge& edge : edges)
    {
        mkWire.Add(edge);

        Standard_Real f, l;

        Handle(Geom_Curve) curve =
            BRep_Tool::Curve(edge, f, l);

        if (curve.IsNull())
            continue;

        Handle(Adaptor3d_HCurve) adapt =
            new GeomAdaptor_HCurve(curve);

        Handle(GeomPlate_CurveConstraint) cc =
            new GeomPlate_CurveConstraint(
                adapt,
                0   // 0 = 位置约束
            );

        plate.Add(cc);
    }

    // =================================================
    // 3️⃣ 加入内部点约束
    // =================================================
    for (const gp_Pnt& p : pts)
    {
        Handle(GeomPlate_PointConstraint) pc =
            new GeomPlate_PointConstraint(p,
                0,
                1e-3
            );

        plate.Add(pc);
    }

    // =================================================
    // 4️⃣ 求解
    // =================================================
    plate.Perform();

    if (!plate.IsDone())
    {
        std::cout << "Plate failed\n";
        return TopoDS_Face();
    }

    Handle(GeomPlate_Surface) plateSurf =
        plate.Surface();

    // =================================================
    // 5️⃣ B-Spline 逼近
    // =================================================
    GeomPlate_MakeApprox approx(
        plateSurf,
        1e-3,  // 3D误差
        10,
        7,
        0.01,
        1
    );

    Handle(Geom_BSplineSurface) bsSurf =
        approx.Surface();
    TopoDS_Wire awire = mkWire.Wire();

    // =================================================
    // 6️⃣ 生成 Face
    // =================================================
    BRepBuilderAPI_MakeFace mkFace(
        bsSurf,
        awire,
        true);   // Inside = true
    TopoDS_Face face;
    if (!mkFace.IsDone())
    {
        // 裁剪失败
        return face;
    }

    face = mkFace.Face();
    /*
    TopoDS_Face face =
        BRepBuilderAPI_MakeFace(
            bsSurf,
            awire,
            1e-6
        );*/

    return face;
}

gp_Pnt OccMidSurfCommFunction::GetEdgeMidPoint(const TopoDS_Edge& edge)
{
    Standard_Real firstParam, lastParam;
    Handle(Geom_Curve) curve = BRep_Tool::Curve(edge, firstParam, lastParam);

    if (curve.IsNull())
    {
        // 如果边没有对应几何曲线，返回原点
        return gp_Pnt(0, 0, 0);
    }

    // 中点参数
    Standard_Real midParam = (firstParam + lastParam) / 2.0;

    // 计算中点
    gp_Pnt midPoint = curve->Value(midParam);
    return midPoint;
}
gp_Pnt OccMidSurfCommFunction::GetEdgeMidPointAndDeriv(const TopoDS_Edge& edge, gp_Vec &t)
{
    Standard_Real firstParam, lastParam;
    Handle(Geom_Curve) curve = BRep_Tool::Curve(edge, firstParam, lastParam);

    if (curve.IsNull())
    {
        // 如果边没有对应几何曲线，返回原点
        return gp_Pnt(0, 0, 0);
    }

    // 中点参数
    Standard_Real midParam = (firstParam + lastParam) / 2.0;
    gp_Pnt p;
    
    curve->D1(midParam, p, t);  // p 是边中点，t 是切向量
    t.Normalize();

    if (edge.Orientation() == TopAbs_REVERSED)
    {
        t.Reverse();
    }
    // 计算中点
    gp_Pnt midPoint = curve->Value(midParam);
    return midPoint;
}

double OccMidSurfCommFunction::PointToFaceDistance(
    const gp_Pnt& p,
    const TopoDS_Face& face,
    gp_Pnt &proj
    )
{
    if (!ProjectPointToFace(p, face, proj))
        return 1e12; // 投影失败返回大值

    return p.Distance(proj);
}

bool OccMidSurfCommFunction::RayIntersectFace(
    const gp_Pnt& origin,
    const gp_Vec& dir,
    const TopoDS_Face& face,
    gp_Pnt& hitPnt,
    double& dist)
{
    Handle(Geom_Surface) surface =
        BRep_Tool::Surface(face);

    if (surface.IsNull())
        return false;

    // 构造射线
    gp_Lin lin(origin, dir);
    Handle(Geom_Line) geomLine =
        new Geom_Line(lin);

    // 求交
    GeomAPI_IntCS intersector(
        geomLine,
        surface);

    if (!intersector.IsDone())
        return false;

    int nb = intersector.NbPoints();
    if (nb < 1)
        return false;

    double minDist = DBL_MAX;
    gp_Pnt bestPnt;

    for (int i = 1; i <= nb; ++i)
    {
        gp_Pnt p = intersector.Point(i);

        // 面域内判断
        BRepClass_FaceClassifier classifier;
        classifier.Perform(face, p, 1e-6);

        if (classifier.State() == TopAbs_OUT)
            continue;

        // 距离
        double d = origin.Distance(p);

        if (d < 1e-6)
            continue;

        // 射线正向性判断（关键）
        gp_Vec v(origin, p);
        if (v.Dot(dir) <= 0)
            continue;

        if (d < minDist)
        {
            minDist = d;
            bestPnt = p;
        }
    }

    if (minDist == DBL_MAX)
        return false;

    dist = minDist;
    hitPnt = bestPnt;

    return true;
}

bool OccMidSurfCommFunction::RayIntersectFace_OCC(
    const gp_Pnt& origin,
    const gp_Vec& dir,
    const TopoDS_Face& face,
    double maxDist,
    gp_Pnt& hitPnt,
    double& dist)
{
    if (dir.Magnitude() < Precision::Confusion())
        return false;

    gp_Dir rayDir(dir);
    gp_Lin ray(origin, rayDir);

    IntCurvesFace_ShapeIntersector intersector;
    intersector.Load(face, 1.0e-6);

    // 对射线而言，参数范围设为 [0, maxDist]
    intersector.Perform(ray, 0.0, maxDist);

    if (!intersector.IsDone() || intersector.NbPnt() <= 0)
        return false;

    double minW = DBL_MAX;
    gp_Pnt bestPnt;

    for (int k = 1; k <= intersector.NbPnt(); ++k)
    {
        const double w = intersector.WParameter(k);

        if (w <= 1.0e-6 || w > maxDist)
            continue;

        if (w < minW)
        {
            minW = w;
            bestPnt = intersector.Pnt(k);
        }
    }

    if (minW == DBL_MAX)
        return false;

    dist = minW;
    hitPnt = bestPnt;
    return true;
}

bool OccMidSurfCommFunction::GetNormalAtPoint(
    const TopoDS_Face& face,
    const gp_Pnt& p,
    gp_Vec& normal)
{
    Handle(Geom_Surface) surf = BRep_Tool::Surface(face);
    if (surf.IsNull())
        return false;

    GeomAPI_ProjectPointOnSurf projector(p, surf);
    if (projector.NbPoints() == 0)
        return false;

    Standard_Real u, v;
    projector.LowerDistanceParameters(u, v);

    gp_Pnt pSurf;
    gp_Vec du, dv;
    surf->D1(u, v, pSurf, du, dv);

    normal = du.Crossed(dv); // 如果方向反，改成 dv.Crossed(du)
    if (normal.Magnitude() < Precision::Confusion())
        return false;

    normal.Normalize();

    // 考虑面拓扑方向
    if (face.Orientation() == TopAbs_REVERSED)
        normal.Reverse();

    //// 可选：确保朝向投影点外部
    //gp_Vec toPoint(pSurf, p);
    //if (normal.Dot(toPoint) < 0)
    //    normal.Reverse();

    return true;
}



double OccMidSurfCommFunction::CalFacArea(
    const TopoDS_Face& fac)
{
    if (fac.IsNull())
        return -1;

    GProp_GProps props;

    try
    {
        BRepGProp::SurfaceProperties(fac, props);
    }
    catch (...)
    {
        // 如果计算失败，返回 false
        return -1;
    }

    return  props.Mass(); // 面积
}

void OccMidSurfCommFunction::GetFacDiscretePnt(
    TopoDS_Face& fac,
    GpPntArr& pntArr)
{
    pntArr.clear();

    // ========== ① 面内部采样 ==========
    Handle(Geom_Surface) surface =
        BRep_Tool::Surface(fac);

    Standard_Real uMin, uMax, vMin, vMax;
    BRepTools::UVBounds(fac, uMin, uMax, vMin, vMax);

    const int uSteps = 10;
    const int vSteps = 10;

    for (int i = 0; i <= uSteps; ++i)
    {
        for (int j = 0; j <= vSteps; ++j)
        {
            double u =
                uMin + (uMax - uMin) * i / uSteps;
            double v =
                vMin + (vMax - vMin) * j / vSteps;

            gp_Pnt point;
            surface->D0(u, v, point);

            // 裁剪判定
            BRepClass_FaceClassifier classifier;
            classifier.Perform(fac, point, 1e-6);

            if (classifier.State() == TopAbs_IN ||
                classifier.State() == TopAbs_ON)
            {
                pntArr.push_back(point);
            }
        }
    }

    // ========== ② 边界采样 ==========
    for (TopExp_Explorer exp(fac, TopAbs_EDGE);
        exp.More();
        exp.Next())
    {
        TopoDS_Edge edge =
            TopoDS::Edge(exp.Current());

        Standard_Real t0, t1;
        Handle(Geom_Curve) curve =
            BRep_Tool::Curve(edge, t0, t1);

        if (curve.IsNull())
            continue;

        // 起点
        gp_Pnt pStart;
        curve->D0(t0, pStart);
        pntArr.push_back(pStart);

        // 中点
        Standard_Real tMid = 0.5 * (t0 + t1);
        gp_Pnt pMid;
        curve->D0(tMid, pMid);
        pntArr.push_back(pMid);

        // 终点
        gp_Pnt pEnd;
        curve->D0(t1, pEnd);
        pntArr.push_back(pEnd);
    }
}