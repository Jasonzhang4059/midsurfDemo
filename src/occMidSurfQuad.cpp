//#include "occMidSurfQuad.h"
//
//#include <TopExp_Explorer.hxx>
//#include <BRepTools_WireExplorer.hxx>
////！三角网格模型处理头文件
//#include <iomanip>
//#include <cmath>
//#include <unordered_map>
//#include "Mesh.h"
//#include "Iterators.h"
//#include "FormTrait.h"
//#include "Point.h"
//#include "LSCM.h"
//#include <iostream>
//
//using namespace MeshLib;
//
//// ======================================================
//// SfCtainTreeNode
//// ======================================================
//
//SfCtainTreeNode::SfCtainTreeNode()
//    : isGenus(false),
//    isSort(false),
//    num(-1),
//    quaded(false)
//{
//}
//
//SfCtainTreeNode::SfCtainTreeNode(const std::vector<TopoDS_Edge>& outlines)
//    : outLines(outlines),
//    isGenus(false),
//    isSort(false),
//    num(-1),
//    quaded(false)
//{
//    UpdateAllLines();
//    UpdateVertexFromOutLines();
//}
//
//SfCtainTreeNode::SfCtainTreeNode(const std::vector<TopoDS_Edge>& outlines,
//    const std::vector<TopoDS_Edge>& conlines)
//    : outLines(outlines),
//    conLines(conlines),
//    isGenus(false),
//    isSort(false),
//    num(-1),
//    quaded(false)
//{
//    UpdateAllLines();
//    UpdateVertexFromOutLines();
//}
//
//SfCtainTreeNode::SfCtainTreeNode(const std::vector<TopoDS_Edge>& outlines,
//    const std::vector<TopoDS_Edge>& conlines,
//    const std::vector<int>& seg)
//    : outLines(outlines),
//    conLines(conlines),
//    isSeg(seg),
//    isGenus(false),
//    isSort(false),
//    num(-1),
//    quaded(false)
//{
//    UpdateAllLines();
//    UpdateVertexFromOutLines();
//}
//
//SfCtainTreeNode::SfCtainTreeNode(const std::vector<TopoDS_Edge>& outlines,
//    const std::vector<TopoDS_Edge>& conlines,
//    const std::vector<int>& seg,
//    bool genus)
//    : outLines(outlines),
//    conLines(conlines),
//    isSeg(seg),
//    isGenus(genus),
//    isSort(false),
//    num(-1),
//    quaded(false)
//{
//    UpdateAllLines();
//    UpdateVertexFromOutLines();
//}
//
//SfCtainTreeNode::~SfCtainTreeNode()
//{
//    for (SfCtainTreeNode* child : childs)
//    {
//        delete child;
//    }
//    childs.clear();
//}
//
//void SfCtainTreeNode::GetOutPoints(std::vector<gp_Pnt>& ps) const
//{
//    ps = vertex;
//}
//
//void SfCtainTreeNode::GetOutLines(std::vector<TopoDS_Edge>& outline) const
//{
//    outline = outLines;
//}
//
//void SfCtainTreeNode::GetSurfs(std::vector<TopoDS_Face>& outSurfs) const
//{
//    outSurfs = surfs;
//}
//
//void SfCtainTreeNode::AddChild(SfCtainTreeNode* child)
//{
//    if (child != nullptr)
//    {
//        childs.push_back(child);
//    }
//}
//
//void SfCtainTreeNode::AddSurf(const TopoDS_Face& face)
//{
//    surfs.push_back(face);
//}
//
//void SfCtainTreeNode::UpdateAllLines()
//{
//    allLines.clear();
//    for (const auto& e : outLines)
//    {
//        allLines.push_back(e);
//    }
//    for (const auto& e : conLines)
//    {
//        allLines.push_back(e);
//    }
//}
//
//void SfCtainTreeNode::UpdateVertexFromOutLines()
//{
//    vertex.clear();
//    for (const auto& edge : outLines)
//    {
//        gp_Pnt p;
//        if (GetEdgeStartPoint(edge, p))
//        {
//            vertex.push_back(p);
//        }
//    }
//}
//
//void SfCtainTreeNode::Clear()
//{
//    outLines.clear();
//    conLines.clear();
//    vertex.clear();
//    isSeg.clear();
//    allLines.clear();
//    outNumber.clear();
//    quadPolNumber.clear();
//    surfs.clear();
//
//    for (SfCtainTreeNode* child : childs)
//    {
//        delete child;
//    }
//    childs.clear();
//
//    isGenus = false;
//    isSort = false;
//    num = -1;
//    quaded = false;
//}
//
//bool SfCtainTreeNode::GetEdgeStartPoint(const TopoDS_Edge& edge, gp_Pnt& p) const
//{
//    TopoDS_Vertex v1, v2;
//    TopExp::Vertices(edge, v1, v2);
//    if (v1.IsNull())
//    {
//        return false;
//    }
//    p = BRep_Tool::Pnt(v1);
//    return true;
//}
//
//// ======================================================
//// OccMidSurfQuad
//// ======================================================
//
//OccMidSurfQuad::OccMidSurfQuad()
//{
//}
//
//OccMidSurfQuad::~OccMidSurfQuad()
//{
//}
//
//std::vector<TopoDS_Face> OccMidSurfQuad::QuadSurface()
//{
//    // 对应你给的 quadSurface() 主流程：
//    // genus -> unfoldMeshSurface -> getMeshModelBoundary -> quadPlane -> fittingSurface
//    std::vector<bool> genus;
//    genus.resize(3);
//    genus[0] = false;
//    genus[1] = true;
//    genus[2] = true;
//
//    std::vector<gp_Pnt> vecs;
//    unfoldMeshSurface("obj\\FourTubeCut.obj", vecs);
//
//    std::vector<std::vector<TopoDS_Edge>> boundRes;
//    getMeshModelBoundary("obj\\Plane.obj", boundRes);
//
//    std::vector<std::vector<gp_Pnt>> bV;
//    quadPlane(boundRes, genus, bV);
//
//    std::vector<TopoDS_Face> SS;
//    SS = fittingSurface(boundRes, 2, 3);
//    return SS;
//}
//
//bool OccMidSurfQuad::QuadFaces(const std::vector<TopoDS_Face>& m_fac,
//    std::vector<TopoDS_Face>& allSurf)
//{
//    allSurf.clear();
//    bool hasResult = false;
//
//    for (const auto& face : m_fac)
//    {
//        std::vector<TopoDS_Edge> outer;
//        std::vector<std::vector<TopoDS_Edge>> inner;
//        if (!ExtractFaceLoops(face, outer, inner))
//        {
//            continue;
//        }
//
//        std::vector<bool> genus;
//        std::vector<TopoDS_Face> faceSurf;
//        quad(outer, inner, genus, faceSurf);
//
//        for (const auto& one : faceSurf)
//        {
//            allSurf.push_back(one);
//        }
//
//        if (!faceSurf.empty())
//        {
//            hasResult = true;
//        }
//    }
//
//    return hasResult;
//}
//
//void OccMidSurfQuad::quad(std::vector<TopoDS_Edge>& outer,
//    std::vector<std::vector<TopoDS_Edge>>& inner,
//    std::vector<bool>& genus,
//    std::vector<TopoDS_Face>& allSurf)
//{
//    allSurf.clear();
//    std::vector<TopoDS_Edge> allLines;
//
//    // 对应原来的：
//    // varray<varray<Spline>> surf;
//    std::vector<std::vector<TopoDS_Edge>> surf;
//    surf.push_back(outer);
//    for (auto& suf : inner)
//    {
//        surf.push_back(suf);
//    }
//
//    // 提取所有内外轮廓线
//    for (auto& loop : surf)
//    {
//        for (auto& e : loop)
//        {
//            allLines.push_back(e);
//        }
//    }
//
//    // 默认都可分割
//    std::vector<std::vector<int>> seg;
//    seg.push_back(std::vector<int>(outer.size(), 1));
//    for (int i = 0; i < static_cast<int>(inner.size()); ++i)
//    {
//        seg.push_back(std::vector<int>(inner[i].size(), 1));
//    }
//
//    // 自动生成连接线
//    std::vector<TopoDS_Edge> addLines;
//    createAddline(surf, addLines);
//
//    // 建立包含树
//    SfCtainTreeNode* root = CreateSurfContainTree(surf, addLines, seg, genus);
//    if (root == nullptr)
//    {
//        return;
//    }
//
//    // 剖分
//    QuadWithContainTree(root);
//
//    // 取出截面
//    getAllSurface(root, allSurf);
//
//    // 剖分调整
//    quadAdjustUV(allSurf);
//
//    delete root;
//}
//
//bool OccMidSurfQuad::ExtractFaceLoops(const TopoDS_Face& face,
//    std::vector<TopoDS_Edge>& outer,
//    std::vector<std::vector<TopoDS_Edge>>& inner)
//{
//    outer.clear();
//    inner.clear();
//
//    bool first = true;
//    for (TopExp_Explorer exp(face, TopAbs_WIRE); exp.More(); exp.Next())
//    {
//        TopoDS_Wire wire = TopoDS::Wire(exp.Current());
//        std::vector<TopoDS_Edge> loop;
//        for (BRepTools_WireExplorer wexp(wire); wexp.More(); wexp.Next())
//        {
//            loop.push_back(wexp.Current());
//        }
//
//        if (loop.empty())
//        {
//            continue;
//        }
//
//        if (first)
//        {
//            outer = loop;
//            first = false;
//        }
//        else
//        {
//            inner.push_back(loop);
//        }
//    }
//
//    return !outer.empty();
//}
//
//// ======================================================
//// 以下函数：上传文件里出现了调用关系或函数名，
//// 但当前对话没有给出完整原始实现。
//// 按你的要求，这里不自己补新算法，只保留 OCC 版本接口。
//// ======================================================
//
//// 存储原始的 3D 和 2D 点映射
//std::unordered_map<int, MeshLib::Point> map3d;
//std::unordered_map<int, MeshLib::Point> map2d;
//
////！展开曲面
//void unfoldMeshSurface(const std::string& path, std::vector<gp_Pnt>& vecs) 
//{
//    std::cout << "--> Reading mesh..." << std::endl;
//    MeshLib::Mesh mesh;
//
//    // 读取原始网格曲面
//    mesh.read_obj(path.c_str());
//
//    // FormTrait 是 MeshLib 库中的一个类，用于实现网格数据和表单数据之间的转换
//    MeshLib::FormTrait traits(&mesh);
//
//    // 收集三角网格空间曲面上的点，用于后续曲面拟合
//    for (MeshVertexIterator viter(&mesh); !viter.end(); ++viter) {
//        Vertex* v = *viter;  // 迭代网格中的每个顶点
//        if (v->string().substr(0, 3) != "fix") { // 排除固定点
//            Point p = v->point();
//            gp_Pnt vec(p.x(), p.y(), p.z());  // 转换为 OpenCASCADE gp_Pnt 类型
//            vecs.push_back(vec);  // 存储到 vecs 中
//        }
//    }
//
//    // 计算共形映射
//    std::cout << "--> Computing conformal map..." << std::endl;
//    MeshLib::LSCM lscm(&mesh);
//    lscm.project();
//    map3d = lscm.map3d;
//    map2d = lscm.map2d;
//
//    std::cout << "--> Writing mesh..." << std::endl;
//    mesh.write_obj("Plane.obj");
//}
//
//std::vector<TopoDS_Face> OccMidSurfQuad::fittingSurface(
//    std::vector<std::vector<TopoDS_Edge>>& boundRes,
//    int uDegree,
//    int uNum)
//{
//    (void)boundRes;
//    (void)uDegree;
//    (void)uNum;
//    std::vector<TopoDS_Face> res;
//    return res;
//}
//
//bool OccMidSurfQuad::createAddline(std::vector<std::vector<TopoDS_Edge>>& surf,
//    std::vector<TopoDS_Edge>& addLines)
//{
//    (void)surf;
//    addLines.clear();
//    return true;
//}
//
//SfCtainTreeNode* OccMidSurfQuad::CreateSurfContainTree(
//    const std::vector<std::vector<TopoDS_Edge>>& surf,
//    const std::vector<TopoDS_Edge>& addLines,
//    const std::vector<std::vector<int>>& seg,
//    std::vector<bool>& genus)
//{
//    (void)genus;
//
//    if (surf.empty())
//    {
//        return nullptr;
//    }
//
//    if (!seg.empty())
//    {
//        return new SfCtainTreeNode(surf.front(), addLines, seg.front(), false);
//    }
//    return new SfCtainTreeNode(surf.front(), addLines);
//}
//
//void OccMidSurfQuad::QuadWithContainTree(SfCtainTreeNode* root)
//{
//    // 已知结构来自你上传的 quadPart.cpp：
//    // 先队列层次遍历，再压栈，后续由内向外剖分。
//    // 这里只保留 OCC 版流程壳，不补新算法。
//    if (root == nullptr)
//    {
//        return;
//    }
//
//    std::stack<SfCtainTreeNode*> nodes;
//    std::queue<SfCtainTreeNode*> mq;
//    mq.push(root);
//
//    while (!mq.empty())
//    {
//        SfCtainTreeNode* cur = mq.front();
//        mq.pop();
//
//        nodes.push(cur);
//
//        // 原实现里这里还有：
//        // if (cur->childs.empty() && !cur->isGenus && !cur->quaded && cur->outLines.size() == 3) { ... }
//        // 当前不补新算法。
//
//        if (!cur->childs.empty())
//        {
//            for (auto it = cur->childs.begin(); it != cur->childs.end(); ++it)
//            {
//                mq.push(*it);
//            }
//        }
//    }
//
//    // 原实现里后续会从栈顶开始，内到外执行真正剖分。
//    // 当前不补新算法。
//}
//
//void OccMidSurfQuad::getAllSurface(SfCtainTreeNode* root,
//    std::vector<TopoDS_Face>& allSurf) const
//{
//    allSurf.clear();
//    if (root == nullptr)
//    {
//        return;
//    }
//
//    std::queue<SfCtainTreeNode*> q;
//    q.push(root);
//
//    while (!q.empty())
//    {
//        SfCtainTreeNode* cur = q.front();
//        q.pop();
//
//        std::vector<TopoDS_Face> tmpsf;
//        cur->GetSurfs(tmpsf);
//        for (auto& s : tmpsf)
//        {
//            allSurf.push_back(s);
//        }
//
//        for (auto it = cur->childs.begin(); it != cur->childs.end(); ++it)
//        {
//            q.push(*it);
//        }
//    }
//}
//
//void OccMidSurfQuad::quadAdjustUV(std::vector<TopoDS_Face>& allSurf) const
//{
//    (void)allSurf;
//}