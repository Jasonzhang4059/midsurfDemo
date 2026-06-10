#include "occMidSurfClassification.h"

#include <BRep_Tool.hxx>
#include <BRepBndLib.hxx>
#include <Geom_Curve.hxx>
#include <BRepBuilderAPI_MakeWire.hxx>
#include <Geom_BSplineCurve.hxx>
#include <BRepBuilderAPI_MakeEdge.hxx>
#include <GeomAPI_PointsToBSpline.hxx>
#include <BRepTools.hxx>
#include <TopoDS.hxx>
#include <TopExp.hxx>

#include <BRep_Builder.hxx>
#include <TopoDS_Compound.hxx>
#include <GeomAPI_ProjectPointOnSurf.hxx>
#include <TopTools_DataMapOfShapeInteger.hxx>

#include <BRepMesh_IncrementalMesh.hxx>
#include <BRepExtrema_TriangleSet.hxx>
#include <BRepAdaptor_Surface.hxx>
#include <BRepClass_FaceClassifier.hxx>

#include"occMidSurfCommFunction.h"
//BVH
#include <BVH_LinearBuilder.hxx>
#include <BVH_BoxSet.hxx>
#include <unordered_map>
#include <cassert>
#include <stack>

//

MidSurfClassification::MidSurfClassification(TopoFaceArr &orgFacArr) :
                    m_orgFacArr(orgFacArr),
                    m_minThickness(0),
                    m_maxThickness(7)
{

}

bool MidSurfClassification::Generate()
{
    InitMidSurf();

    GroupMidSurf();

    BuildConnRelationBetweenGroups();

    MergeGroupByThickness();

    //for test
    for (auto grp : m_midSurfGrouper.GetMidSurfGroup())
    {
        std::vector<int> p;
        for (auto lab : grp.GetSideALabs())
        {
            p.push_back(lab);
        }
        //testVec.push_back(p);
        //p.clear();

        for (auto lab : grp.GetSideBLabs())
        {
            p.push_back(lab);
        }
        testVec.push_back(p);
    }

    BuildGroupMidSurfOuterLoop();

    //for test 
    for (auto cmpArrId : testVec)
    {
        TopoFaceArr cmpArr;
        for (auto facId : cmpArrId)
        {
            cmpArr.push_back(m_orgFacArr[facId]);
        }
        testClsfyFac.push_back(cmpArr);
    }

    return true;
}

MidSurfClassification::~MidSurfClassification()
{

}

Bnd_Box MidSurfClassification::MergeFaceBoxes(
    const std::vector<int>& faceIds,
    int begin,
    int end) const
{
    Bnd_Box mergedBox;

    for (int i = begin; i < end; ++i)
    {
        const int faceId = faceIds[i];

        if (faceId < 0 ||
            faceId >= static_cast<int>(m_orgFacBoxArr.size()))
        {
            continue;
        }

        mergedBox.Add(m_orgFacBoxArr[faceId]);
    }

    return mergedBox;
}

double MidSurfClassification::GetBoxCenterCoord(
    const Bnd_Box& box,
    int axis) const
{
    if (box.IsVoid())
        return 0.0;

    Standard_Real xmin, ymin, zmin;
    Standard_Real xmax, ymax, zmax;

    box.Get(xmin, ymin, zmin, xmax, ymax, zmax);

    if (axis == 0)
        return 0.5 * (xmin + xmax);
    if (axis == 1)
        return 0.5 * (ymin + ymax);

    return 0.5 * (zmin + zmax);
}

int MidSurfClassification::GetLongestAxis(
    const Bnd_Box& box) const
{
    if (box.IsVoid())
        return 0;

    Standard_Real xmin, ymin, zmin;
    Standard_Real xmax, ymax, zmax;

    box.Get(xmin, ymin, zmin, xmax, ymax, zmax);

    const double dx = xmax - xmin;
    const double dy = ymax - ymin;
    const double dz = zmax - zmin;

    if (dx >= dy && dx >= dz)
        return 0;
    if (dy >= dx && dy >= dz)
        return 1;

    return 2;
}

int MidSurfClassification::BuildFaceBvhRecursive(
    std::vector<int>& faceIds,
    int begin,
    int end)
{
    if (begin >= end)
        return -1;

    FaceBvhNode node;
    node.box = MergeFaceBoxes(faceIds, begin, end);

    const int nodeId = static_cast<int>(m_faceBvhNodes.size());
    m_faceBvhNodes.push_back(node);

    const int count = end - begin;
    const int leafSize = 6;

    if (count <= leafSize)
    {
        m_faceBvhNodes[nodeId].faceIds.reserve(count);

        for (int i = begin; i < end; ++i)
        {
            m_faceBvhNodes[nodeId].faceIds.push_back(faceIds[i]);
        }

        return nodeId;
    }

    const int axis = GetLongestAxis(node.box);
    const int mid = begin + count / 2;

    std::nth_element(
        faceIds.begin() + begin,
        faceIds.begin() + mid,
        faceIds.begin() + end,
        [&](int lhs, int rhs)
        {
            return GetBoxCenterCoord(m_orgFacBoxArr[lhs], axis) <
                GetBoxCenterCoord(m_orgFacBoxArr[rhs], axis);
        });

    const int leftId =
        BuildFaceBvhRecursive(faceIds, begin, mid);

    const int rightId =
        BuildFaceBvhRecursive(faceIds, mid, end);

    m_faceBvhNodes[nodeId].left = leftId;
    m_faceBvhNodes[nodeId].right = rightId;

    return nodeId;
}

void MidSurfClassification::BuildBvh()
{
    m_faceBvhNodes.clear();

    const int faceNum =
        static_cast<int>(m_orgFacArr.size());

    if (faceNum <= 0)
        return;

    std::vector<int> faceIds;
    faceIds.reserve(faceNum);

    for (int i = 0; i < faceNum; ++i)
    {
        faceIds.push_back(i);
    }

    BuildFaceBvhRecursive(
        faceIds,
        0,
        static_cast<int>(faceIds.size()));
}

void MidSurfClassification::QueryFaceBvh(
    int nodeId,
    const Bnd_Box& queryBox,
    std::vector<int>& result) const
{
    if (nodeId < 0 ||
        nodeId >= static_cast<int>(m_faceBvhNodes.size()))
    {
        return;
    }

    const FaceBvhNode& node =
        m_faceBvhNodes[nodeId];

    if (queryBox.IsOut(node.box))
        return;

    if (node.IsLeaf())
    {
        for (int faceId : node.faceIds)
        {
            if (faceId < 0 ||
                faceId >= static_cast<int>(m_orgFacBoxArr.size()))
            {
                continue;
            }

            if (!queryBox.IsOut(m_orgFacBoxArr[faceId]))
            {
                result.push_back(faceId);
            }
        }

        return;
    }

    QueryFaceBvh(node.left, queryBox, result);
    QueryFaceBvh(node.right, queryBox, result);
}

std::vector<int> MidSurfClassification::QueryCandidateFacesByBvh(
    const Bnd_Box& queryBox) const
{
    std::vector<int> result;

    if (m_faceBvhNodes.empty())
        return result;

    QueryFaceBvh(0, queryBox, result);

    return result;
}

TopoDS_Compound MakeCompound(const std::vector<TopoDS_Face>& faces)
{
    TopoDS_Compound compound;
    BRep_Builder builder;
    builder.MakeCompound(compound);

    for (const auto& f : faces)
    {
        builder.Add(compound, f);
    }

    return compound;
}

void BuildEdgeFaceAdjacency(
    const TopoDS_Shape& theShape,
    TopTools_IndexedDataMapOfShapeListOfShape& edgeFaceMap)
{
    edgeFaceMap.Clear();

    TopExp::MapShapesAndAncestors(
        theShape,
        TopAbs_EDGE,
        TopAbs_FACE,
        edgeFaceMap);

}

bool MidSurfClassification::SortByArea()
{
    // 面数量
    int n = static_cast<int>(m_orgFacAreaArr.size());

    // ---------- ① 建立索引 ----------
    std::vector<int> indices(n);
    for (int i = 0; i < n; ++i)
    {
        indices[i] = i;
    }

    // ---------- ② 按面积排序索引 ----------
    std::sort(indices.begin(), indices.end(),
        [&](int a, int b)
        {
            return m_orgFacAreaArr[a] < m_orgFacAreaArr[b]; // 升序
        });

    // ---------- ③ 生成排序后的新数组 ----------
    TopoFaceArr sortedFacArr;
    std::vector<double> sortedAreaArr;
    std::vector<Bnd_Box> sortedFacBoxArr;

    sortedFacArr.reserve(n);
    sortedAreaArr.reserve(n);

    for (int idx : indices)
    {
        sortedFacArr.push_back(m_orgFacArr[idx]);
        sortedAreaArr.push_back(m_orgFacAreaArr[idx]);
        sortedFacBoxArr.push_back(m_orgFacBoxArr[idx]);
    }

    // ---------- ④ 覆盖原数组（可选）----------
    m_orgFacArr = sortedFacArr;
    m_orgFacAreaArr = sortedAreaArr;
    m_orgFacBoxArr = sortedFacBoxArr;

    return true;
}

void MidSurfClassification::InitMidSurf()
{
    
    CacheMidSurfAreaAndBoundingBox();

    SortByArea();

    TopoDS_Compound shape = MakeCompound(m_orgFacArr);
    
    BuildEdgeFaceAdjacency(shape, m_edgeFaceMap);

    //FindLateralFaces();

    BuildBvh();
    
}

bool MidSurfClassification::FindCommonAdjacentFace(
    const TopoDS_Face& aFac,
    const TopoDS_Face& bFac,
    TopoFaceArr& adjFacArr)
{
    // 1️⃣ 找 aFac 的邻面集合
    TopTools_ListOfShape aAdjFaces;
    for (TopExp_Explorer exp(aFac, TopAbs_EDGE); exp.More(); exp.Next())
    {
        const TopoDS_Edge& e = TopoDS::Edge(exp.Current());
        if (!m_edgeFaceMap.Contains(e)) continue;

        const TopTools_ListOfShape& faces = m_edgeFaceMap.FindFromKey(e);
        for (TopTools_ListIteratorOfListOfShape it(faces); it.More(); it.Next())
        {
            const TopoDS_Face& f = TopoDS::Face(it.Value());
            if (f.IsSame(bFac))
            {
                return false;
            }
            if (!f.IsSame(aFac)) aAdjFaces.Append(f);
        }
    }

    // 2️⃣ 找 bFac 的邻面集合
    TopTools_ListOfShape bAdjFaces;
    for (TopExp_Explorer exp(bFac, TopAbs_EDGE); exp.More(); exp.Next())
    {
        const TopoDS_Edge& e = TopoDS::Edge(exp.Current());
        if (!m_edgeFaceMap.Contains(e)) continue;

        const TopTools_ListOfShape& faces = m_edgeFaceMap.FindFromKey(e);
        for (TopTools_ListIteratorOfListOfShape it(faces); it.More(); it.Next())
        {
            const TopoDS_Face& f = TopoDS::Face(it.Value());
            if (!f.IsSame(bFac)) bAdjFaces.Append(f);
        }
    }

    // 3️⃣ 求交集（找相同的邻面）
    for (TopTools_ListIteratorOfListOfShape itA(aAdjFaces); itA.More(); itA.Next())
    {
        const TopoDS_Face& fA = TopoDS::Face(itA.Value());
        for (TopTools_ListIteratorOfListOfShape itB(bAdjFaces); itB.More(); itB.Next())
        {
            const TopoDS_Face& fB = TopoDS::Face(itB.Value());
            if (fA.IsSame(fB) && std::find(adjFacArr.begin(), adjFacArr.end(), fA) == adjFacArr.end())
            {
                adjFacArr.push_back(fA);
            }
        }
    }

    return !adjFacArr.empty(); // 没有共同邻面

}

bool MidSurfClassification::IsDihedralAngleNear90(
    const TopoDS_Face& fac1,
    const TopoDS_Face& fac2,
    Standard_Real      angMinDeg,
    Standard_Real      angMaxDeg)
{
    gp_Pnt p1, p2;
    gp_Dir n1, n2;

    GetFacCenPntAndNrmVec(
        const_cast<TopoDS_Face&>(fac1), p1, n1);
    GetFacCenPntAndNrmVec(
        const_cast<TopoDS_Face&>(fac2), p2, n2);

    Standard_Real ang =
        n1.Angle(n2) * 180.0 / M_PI;

    return ang > angMinDeg && ang < angMaxDeg;
}

bool MidSurfClassification::IsConvexRelation(
    const TopoDS_Face& aFac,
    const TopoDS_Face& adjFac)
{
    // 1️⃣ 找 aFac 与 adjFac 的共边
    TopoDS_Edge commonEdge;
    bool found = false;

    for (TopExp_Explorer exp(aFac, TopAbs_EDGE); exp.More(); exp.Next())
    {
        const TopoDS_Edge& e = TopoDS::Edge(exp.Current());

        for (TopExp_Explorer expAdj(adjFac, TopAbs_EDGE); expAdj.More(); expAdj.Next())
        {
            const TopoDS_Edge& eAdj = TopoDS::Edge(expAdj.Current());

            if (e.IsSame(eAdj))
            {
                commonEdge = e;
                found = true;
                break;
            }
        }

        if (found) break;
    }

    if (!found)
        return false;

    // 2️⃣ 求共边中点 p 和切向 t
    Standard_Real f, l;
    Handle(Geom_Curve) c3d = BRep_Tool::Curve(commonEdge, f, l);
    if (c3d.IsNull())
        return false;

    Standard_Real mid = 0.5 * (f + l);
    gp_Pnt p;
    gp_Vec t;
    c3d->D1(mid, p, t);  // p 是边中点，t 是切向量
    t.Normalize();

    if (commonEdge.Orientation() == TopAbs_REVERSED)
    {
        t.Reverse();
    }
    // 3️⃣ 求 p 点处 aFac 的法向量
    gp_Vec nA;
    if (!OccMidSurfCommFunction::GetNormalAtPoint(aFac, p, nA))
        return false; // 如果求法向失败，则返回 false
    nA.Normalize();

    // 4️⃣ 求 p 点处 adjFac 的法向量
    gp_Vec nAdj;
    if (!OccMidSurfCommFunction::GetNormalAtPoint(adjFac, p, nAdj))
        return false;
    nAdj.Normalize();

    // 5️⃣ 叉乘向量（aFac切向与aFac法向）
    gp_Vec c = t.Crossed(nA);
    if (c.Magnitude() < Precision::Confusion())
        return false;

    c.Normalize();

    // 6️⃣ 判角：c 与 adjFac法向量
    Standard_Real ang = c.Angle(nAdj) * 180.0 / M_PI;

    return ang < 30.0; // 小于30°表示凸
}

int MidSurfClassification::GetFaceLab(const TopoDS_Face& adjFac)
{
    for (size_t i = 0; i < m_orgFacArr.size(); ++i)
    {
        if (adjFac.IsSame(m_orgFacArr[i]))
        {
            return i;
        }
    }
    return -1;
}
bool MidSurfClassification::FindMidFaceLab(
    const TopoDS_Face& adjFac,
    UINT& adjFacLab)
{
    // 遍历原始面数组
    for (size_t i = 0; i < m_orgFacArr.size(); ++i)
    {
        if (adjFac.IsSame(m_orgFacArr[i]))
        {
            adjFacLab = i;
            return true;
        }
    }

    return false; // 没找到
}

OccMidSurfGrouper& MidSurfClassification::GetMidSurfGrouper()
{
    return m_midSurfGrouper;
}

bool MidSurfClassification::CacheMidSurfAreaAndBoundingBox()
{
    m_orgFacAreaArr.clear();
    m_orgFacAreaArr.reserve(m_orgFacArr.size());

    bool allSuccess = true;

    for (size_t i = 0; i < m_orgFacArr.size(); ++i)
    {
        auto face = m_orgFacArr[i];
        double area = OccMidSurfCommFunction:: CalFacArea(face);
        if (area > 0)
        {
            m_orgFacAreaArr.push_back(area);
        }
        else
        {
            // 如果某个面面积计算失败，缓存为 0 并记录失败
            m_orgFacAreaArr.push_back(0.0);
            allSuccess = false;
            std::cerr << "Warning: Failed to calculate area for face index " << i << std::endl;
        }
        Bnd_Box bbox;
        BRepBndLib::Add(face, bbox);
        m_orgFacBoxArr.push_back(bbox);
    }

    return allSuccess;
}

void MidSurfClassification::FindLateralFaces()
{
    UINT midFacArrNum = m_orgFacArr.size();
    for (UINT i = 0; i < midFacArrNum; ++i)
    {
        const TopoDS_Face& aFac = m_orgFacArr[i];

        for (UINT j = i + 1; j < midFacArrNum; ++j)
        {
            const TopoDS_Face& bFac = m_orgFacArr[j];

            TopoFaceArr adjFacArr;
            if (!FindCommonAdjacentFace(aFac, bFac, adjFacArr))
                continue;
            if (adjFacArr.size() > 5)
            {
                continue;
            }
            for (auto& adjFac : adjFacArr)
            {
                double aFacAdjAngle = OccMidSurfCommFunction::CalAngleOfTwoFace(aFac, adjFac);
                UINT adjFacLab;
                if (!FindMidFaceLab(adjFac, adjFacLab))
                    continue;

                double aFacArea = m_orgFacAreaArr[i], bFacArea = m_orgFacAreaArr[j], adjFacArea = m_orgFacAreaArr[adjFacLab];
                if (aFacAdjAngle < 80 || aFacAdjAngle>100)
                {
                    continue;
                }
                double bFacAdjAngle = OccMidSurfCommFunction::CalAngleOfTwoFace(bFac, adjFac);

                if (bFacAdjAngle < 80 || bFacAdjAngle>100)
                {
                    continue;
                }

                if (!IsConvexRelation(aFac, adjFac) || !IsConvexRelation(bFac, adjFac))
                    continue;

                


                if (aFacArea + bFacArea < 3 * adjFacArea)
                {
                    continue;
                }
                // ✅ adjFac 是侧面
                m_lateralFacLabSet.insert(adjFacLab);
                m_fac.push_back(adjFac);
            }
        }
    }
}

void MidSurfClassification::MergeGroupByThickness()
{
    auto& groups = m_midSurfGrouper.GetMidSurfGroup();

    const double THICK_TOL = 0.1;

    // =====================================================
    // ① 初始化 root
    // =====================================================
    std::vector<int> grpRoot(groups.size());

    for (int i = 0; i < groups.size(); ++i)
        grpRoot[i] = i;

    auto FindRoot = [&](int g)
        {
            while (grpRoot[g] != g)
                g = grpRoot[g];
            return g;
        };

    // =====================================================
    // ② 两两判定
    // =====================================================
    for (int i = 0; i < groups.size(); ++i)
    {
        int rootI = FindRoot(i);

        auto& grpI = groups[rootI];

        double ti = grpI.GetMidFacThicnkess();

        // =================================================
        // ---------- AA + BB ----------
        // =================================================
        std::vector<int> interAA;

        std::set_intersection(
            grpI.GetAdjAA().begin(),
            grpI.GetAdjAA().end(),
            grpI.GetAdjBB().begin(),
            grpI.GetAdjBB().end(),
            std::back_inserter(interAA));

        for (int gLab :interAA)
        {
            int rootJ =FindRoot(gLab);

            if (rootI == rootJ)
                continue;

            auto& grpJ =groups[rootJ];

            double tj =grpJ.GetMidFacThicnkess();

            if (std::abs(ti - tj) > THICK_TOL * std::max(ti, tj))
                continue;

            // ---------- merge ----------
            for (int lab : grpJ.GetSideALabs())
            {
                grpI.AddSideALab(lab);

                m_midSurfGrouper.SetFacLabGrp(lab, rootI);
            }

            for (int lab :grpJ.GetSideBLabs())
            {
                grpI.AddSideBLab(lab);

                m_midSurfGrouper.SetFacLabGrp(lab, rootI);
            }

            grpRoot[rootJ] =rootI;
        }

        // =================================================
        // ---------- AB + BA ----------
        // =================================================
        std::vector<int> interAB;

        std::set_intersection(
            grpI.GetAdjAB().begin(),
            grpI.GetAdjAB().end(),
            grpI.GetAdjBA().begin(),
            grpI.GetAdjBA().end(),
            std::back_inserter(interAB));

        for (int gLab :interAB)
        {
            int rootJ = FindRoot(gLab);

            if (rootI == rootJ)
                continue;

            auto& grpJ = groups[rootJ];

            double tj = grpJ.GetMidFacThicnkess();

            if (std::abs(ti - tj) > THICK_TOL * std::max(ti, tj))
                continue;

            for (int lab : grpJ.GetSideBLabs())
            {
                grpI.AddSideALab(lab);

                m_midSurfGrouper.SetFacLabGrp(lab, rootI);

                m_midSurfGrouper.SetFacType(lab, OccMidSurfGrouper::ASideFac);
            }

            for (int lab : grpJ.GetSideALabs())
            {
                grpI.AddSideBLab(lab);

                m_midSurfGrouper.SetFacLabGrp(lab, rootI);

                m_midSurfGrouper.SetFacType(lab, OccMidSurfGrouper::BSideFac);
            }

            grpRoot[rootJ] = rootI;
        }
    }

    // =====================================================
    // ③ 重建组数组
    // =====================================================
    std::vector<OccMidSurfGroup> newGroups;

    std::map<int, int> rootToNew;

    for (int i = 0; i < groups.size(); ++i)
    {
        int r = FindRoot(i);

        if (rootToNew.find(r) == rootToNew.end())
        {
            rootToNew[r] = newGroups.size();

            newGroups.push_back(groups[r]);
        }
    }

    groups.swap(newGroups);
}



void MidSurfClassification::BuildConnRelationBetweenGroups()
{
    auto& groups = m_midSurfGrouper.GetMidSurfGroup();

    auto AddAdjRelation =
        [&](OccMidSurfGroup& grp,
            OccMidSurfGrouper::MidFacType curType,
            OccMidSurfGrouper::MidFacType adjType,
            int adjGrp)
        {
            if (curType == OccMidSurfGrouper::ASideFac &&
                adjType == OccMidSurfGrouper::ASideFac)
            {
                grp.AddAdjAA(adjGrp);
            }
            else if (curType == OccMidSurfGrouper::ASideFac &&
                adjType == OccMidSurfGrouper::BSideFac)
            {
                grp.AddAdjAB(adjGrp);
            }
            else if (curType == OccMidSurfGrouper::BSideFac &&
                adjType == OccMidSurfGrouper::ASideFac)
            {
                grp.AddAdjBA(adjGrp);
            }
            else if (curType == OccMidSurfGrouper::BSideFac &&
                adjType == OccMidSurfGrouper::BSideFac)
            {
                grp.AddAdjBB(adjGrp);
            }
        };

    auto ProcessFaceLab =
        [&](int gid, OccMidSurfGroup& grp, int lab)
        {
            if (lab < 0 || lab >= static_cast<int>(m_orgFacArr.size()))
                return;

            const TopoDS_Face& curFace = m_orgFacArr[lab];

            auto curType =
                m_midSurfGrouper.GetFacType(lab);

            if (curType == OccMidSurfGrouper::Unknown)
                return;

            for (TopExp_Explorer expE(curFace, TopAbs_EDGE);
                expE.More();
                expE.Next())
            {
                TopoDS_Edge edge =
                    TopoDS::Edge(expE.Current());

                if (!m_edgeFaceMap.Contains(edge))
                    continue;

                const TopTools_ListOfShape& adjFaces =
                    m_edgeFaceMap.FindFromKey(edge);

                for (TopTools_ListIteratorOfListOfShape it(adjFaces);
                    it.More();
                    it.Next())
                {
                    TopoDS_Face adjFace =
                        TopoDS::Face(it.Value());

                    if (adjFace.IsSame(curFace))
                        continue;

                    int adjLab = GetFaceLab(adjFace);

                    if (adjLab < 0)
                        continue;

                    int adjGrp =
                        m_midSurfGrouper.GetFacLabGrp(adjLab);

                    if (adjGrp < 0 || adjGrp == gid)
                        continue;

                    if (adjGrp >= static_cast<int>(groups.size()))
                        continue;

                    const OccMidSurfGroup& otherGrp =
                        groups[adjGrp];

                    // 关键：等厚组只和等厚组建立邻接，变厚组只和变厚组建立邻接
                    if (grp.GetThicknessType() !=
                        otherGrp.GetThicknessType())
                    {
                        continue;
                    }

                    auto adjType =
                        m_midSurfGrouper.GetFacType(adjLab);

                    if (adjType == OccMidSurfGrouper::Unknown)
                        continue;

                    AddAdjRelation(
                        grp,
                        curType,
                        adjType,
                        adjGrp);
                }
            }
        };

    for (int gid = 0; gid < static_cast<int>(groups.size()); ++gid)
    {
        OccMidSurfGroup& grp = groups[gid];

        grp.ClearAdjInfo();

        for (int lab : grp.GetSideALabs())
        {
            ProcessFaceLab(gid, grp, lab);
        }

        for (int lab : grp.GetSideBLabs())
        {
            ProcessFaceLab(gid, grp, lab);
        }
    }

    // Debug 输出
    for (int gid = 0; gid < static_cast<int>(groups.size()); ++gid)
    {
        const auto& grp = groups[gid];

        std::cout << "Group " << gid << std::endl;

        std::cout << " AA: ";
        for (int g : grp.GetAdjAA())
            std::cout << g << " ";

        std::cout << std::endl << " AB: ";
        for (int g : grp.GetAdjAB())
            std::cout << g << " ";

        std::cout << std::endl << " BA: ";
        for (int g : grp.GetAdjBA())
            std::cout << g << " ";

        std::cout << std::endl << " BB: ";
        for (int g : grp.GetAdjBB())
            std::cout << g << " ";

        std::cout << std::endl << std::endl;
    }
}

MidSurfClassification::ThicknessMatchType
MidSurfClassification::JudgeThicknessTypeFromRayHits(
    const std::vector<double>& hitDists,
    int sampleCount,
    FaceMatchInfo& info)
{
    info = FaceMatchInfo();

    if (sampleCount <= 0 || hitDists.empty())
    {
        info.type = ThicknessMatchType::Invalid;
        return info.type;
    }

    info.sampleCount = sampleCount;
    info.hitCount = static_cast<int>(hitDists.size());
    info.hitRatio =
        static_cast<double>(info.hitCount) /
        static_cast<double>(sampleCount);

    // 先判断有效射线比例
    if (info.hitRatio < 0.3)
    {
        info.type = ThicknessMatchType::Invalid;
        return info.type;
    }

    // 命中数量太少也不稳定
    if (info.hitCount < 3)
    {
        info.type = ThicknessMatchType::Invalid;
        return info.type;
    }

    double sumT = 0.0;
    double minT = hitDists.front();
    double maxT = hitDists.front();

    for (double d : hitDists)
    {
        sumT += d;
        minT = std::min(minT, d);
        maxT = std::max(maxT, d);
    }

    const double avgT =
        sumT / static_cast<double>(hitDists.size());

    double var = 0.0;
    for (double d : hitDists)
    {
        const double diff = d - avgT;
        var += diff * diff;
    }

    var /= static_cast<double>(hitDists.size());

    const double stdT = std::sqrt(var);
    const double relRange =
        (maxT - minT) / std::max(avgT, 1.0e-6);
    const double relStd =
        stdT / std::max(avgT, 1.0e-6);

    info.avgThickness = avgT;
    info.minThickness = minT;
    info.maxThickness = maxT;
    info.stdThickness = stdT;
    info.relRange = relRange;

    // 等厚度：厚度变化很小
    if (relRange < 0.10 && relStd < 0.05)
    {
        info.type = ThicknessMatchType::Uniform;
        return info.type;
    }

    // 变厚度：厚度有变化，但不能跳变太大
    if (stdT >= 0.10 && stdT < 0.40 || stdT/avgT>0.1&& stdT / avgT < 0.4)
    {
        info.type = ThicknessMatchType::Variable;
        return info.type;
    }

    info.type = ThicknessMatchType::Invalid;
    return info.type;
}

void MidSurfClassification::BuildGroupsFromMatchMap(
    std::map<int, std::map<int, FaceMatchInfo>>& matchMap,
    ThicknessMatchType groupType)
{
    while (!matchMap.empty())
    {
        std::map<int, bool> colorMap;

        auto it = matchMap.begin();
        int startId = it->first;

        std::stack<int> st;
        st.push(startId);

        colorMap[startId] = false;

        std::vector<int> componentFaces;
        std::vector<double> thicknessSamples;
        std::vector<gp_Pnt> groupMidPts;

        while (!st.empty())
        {
            int cur = st.top();
            st.pop();

            if (std::find(componentFaces.begin(), componentFaces.end(), cur)
                == componentFaces.end())
            {
                componentFaces.push_back(cur);
            }

            auto mapIt = matchMap.find(cur);
            if (mapIt == matchMap.end())
                continue;

            const auto& pairedMap = mapIt->second;

            for (const auto& pair : pairedMap)
            {
                int next = pair.first;
                const FaceMatchInfo& info = pair.second;

                if (info.type != groupType)
                    continue;

                thicknessSamples.push_back(info.avgThickness);

                groupMidPts.insert(
                    groupMidPts.end(),
                    info.midPts.begin(),
                    info.midPts.end());

                if (colorMap.find(next) == colorMap.end())
                {
                    colorMap[next] = !colorMap[cur];
                    st.push(next);
                }
                else
                {
                    if (colorMap[next] == colorMap[cur])
                    {
                        std::cout << "Warning: non-bipartite match graph: "
                            << cur << " - " << next << std::endl;
                    }
                }
            }
        }

        std::vector<UINT> aLabs;
        std::vector<UINT> bLabs;

        for (int facId : componentFaces)
        {
            if (colorMap[facId] == false)
                aLabs.push_back(static_cast<UINT>(facId));
            else
                bLabs.push_back(static_cast<UINT>(facId));
        }

        if (!aLabs.empty() && !bLabs.empty())
        {
            OccMidSurfGroup midSurfGrp;

            const int newGrpLab =
                static_cast<int>(m_midSurfGrouper.GetMidSurfGroup().size());

            for (UINT lab : aLabs)
            {
                midSurfGrp.AddSideALab(lab);
                m_midSurfGrouper.SetFacLabGrp(static_cast<int>(lab), newGrpLab);
                m_midSurfGrouper.SetFacType(
                    static_cast<int>(lab),
                    OccMidSurfGrouper::ASideFac);
            }

            for (UINT lab : bLabs)
            {
                midSurfGrp.AddSideBLab(lab);
                m_midSurfGrouper.SetFacLabGrp(static_cast<int>(lab), newGrpLab);
                m_midSurfGrouper.SetFacType(
                    static_cast<int>(lab),
                    OccMidSurfGrouper::BSideFac);
            }

            double avgThickness = 0.0;
            double minThickness = DBL_MAX;
            double maxThickness = -DBL_MAX;

            for (double t : thicknessSamples)
            {
                avgThickness += t;
                minThickness = std::min(minThickness, t);
                maxThickness = std::max(maxThickness, t);
            }

            if (!thicknessSamples.empty())
            {
                avgThickness /= static_cast<double>(thicknessSamples.size());
            }
            else
            {
                avgThickness = 0.0;
                minThickness = 0.0;
                maxThickness = 0.0;
            }

            midSurfGrp.SetMidFacThicnkess(avgThickness);

           
            if (groupType == ThicknessMatchType::Uniform)
            {
                midSurfGrp.SetThicknessType(
                    OccMidSurfGroup::ThicknessType::Uniform);
            }
            else if (groupType == ThicknessMatchType::Variable)
            {
                midSurfGrp.SetThicknessType(
                    OccMidSurfGroup::ThicknessType::Variable);
            }

            for (const gp_Pnt& p : groupMidPts)
            {
                midSurfGrp.AddMidPoint(p);
            }

            m_midSurfGrouper.AddMidSurfGroup(midSurfGrp);
        }

        // 删除已经处理的连通域
        for (int facId : componentFaces)
        {
            matchMap.erase(facId);
        }

        for (auto& kv : matchMap)
        {
            for (int facId : componentFaces)
            {
                kv.second.erase(facId);
            }
        }

        for (auto it2 = matchMap.begin(); it2 != matchMap.end(); )
        {
            if (it2->second.empty())
                it2 = matchMap.erase(it2);
            else
                ++it2;
        }
    }
}

void MidSurfClassification::GroupMidSurf()
{
    //const Standard_Real ANGLE_COS_TOL = -0.95; // 法向近似相反
    //const Standard_Real DIST_TOL = 1e-3;  // 距离容差（按模型单位）

    std::map<int, std::map<int, FaceMatchInfo>> uniformMatchMap;
    std::map<int, std::map<int, FaceMatchInfo>> variableMatchMap;

    const int midSurfNum =
        static_cast<int>(m_orgFacArr.size());

    for (int i = 0; i < midSurfNum; ++i)
    {
        TopoDS_Face aFac = m_orgFacArr[i];

        /*
        GetFacCenPnt(aFac, cen);*/
       /* gp_Dir s;
        gp_Pnt cen;
        GetFacCenPntAndNrmVec(aFac, cen, s);*/
        /*if(i==0)
        testArrow.emplace_back(cen, s);*/
        //testString.emplace_back(cen, std::to_string(i));
        //continue;
        //Bnd_Box aFacBox = m_orgFacBoxArr[i];
        double aFacArea = OccMidSurfCommFunction::CalFacArea(aFac);
        GpPntArr aFacDisPntArr;
        GetFacDiscretePnt(aFac, aFacDisPntArr);

        Bnd_Box aFacBox = m_orgFacBoxArr[i];
        OccMidSurfCommFunction::ExpandBndBox(aFacBox, m_maxThickness);

        std::vector<int> candidateFaces =
            QueryCandidateFacesByBvh(aFacBox);

        for (int j : candidateFaces)
        {
            if (j <= i)
                continue;

            TopoDS_Face bFac = m_orgFacArr[j];
            Bnd_Box bFacBox = m_orgFacBoxArr[j];

            //gp_Dir bs;
            //gp_Pnt bcen;
            //GetFacCenPntAndNrmVec(bFac, bcen, bs);

            //double angle =
            //    bs.Angle(s);

            //double angleDeg =
            //    angle * 180.0 / M_PI;

            //if (angleDeg < 150)
            //{
            //    continue;
            //}

            // 这里可以保留一次保险判断
            if (aFacBox.IsOut(bFacBox))
                continue;

            int disPntSize = 20;

            int pntSize =
                static_cast<int>(aFacDisPntArr.size());

            if (pntSize <= 0)
                continue;

            int pStep = pntSize / disPntSize;
            if (pStep <= 0)
                pStep = 1;

            std::vector<double> hitDists;
            std::vector<gp_Pnt> localMidPts;

            for (int pCnt = 0; pCnt < pntSize; pCnt += pStep)
            {
                gp_Pnt& p = aFacDisPntArr[pCnt];

                gp_Vec n;
                if (!OccMidSurfCommFunction::GetNormalAtPoint(aFac, p, n))
                    continue;

                gp_Vec rayDir = -n;
                rayDir.Normalize();

                gp_Pnt hitPnt;
                double dist = 0.0;
                testArrow.emplace_back(p, rayDir);
                if (!OccMidSurfCommFunction::RayIntersectFace_OCC(
                    p,
                    rayDir,
                    bFac,
                    m_maxThickness,
                    hitPnt,
                    dist))
                {
                    continue;
                }

                gp_Vec nb;
                if (!OccMidSurfCommFunction::GetNormalAtPoint(
                    bFac,
                    hitPnt,
                    nb))
                {
                    continue;
                }

                nb.Normalize();

                double hitNormalAngle =
                    rayDir.Angle(nb);

                double hitNormalAngleDeg =
                    hitNormalAngle * 180.0 / M_PI;

                if (hitNormalAngleDeg > 45.0 ||
                    dist > m_maxThickness ||
                    dist < m_minThickness)
                {
                    continue;
                }

                hitDists.push_back(dist);

                gp_Pnt mid(
                    0.5 * (hitPnt.X() + p.X()),
                    0.5 * (hitPnt.Y() + p.Y()),
                    0.5 * (hitPnt.Z() + p.Z())
                );

                localMidPts.push_back(mid);
            }

            const int sampleCount =
                (pntSize + pStep - 1) / pStep;

            FaceMatchInfo info;
            ThicknessMatchType type =
                JudgeThicknessTypeFromRayHits(
                    hitDists,
                    sampleCount,
                    info);

            if (type == ThicknessMatchType::Invalid)
            {
                continue;
            }

            info.midPts = localMidPts;

            if (type == ThicknessMatchType::Uniform)
            {
                uniformMatchMap[i][j] = info;
                uniformMatchMap[j][i] = info;
            }
            else if (type == ThicknessMatchType::Variable)
            {
                variableMatchMap[i][j] = info;
                variableMatchMap[j][i] = info;
            }

            m_points.insert(
                m_points.end(),
                localMidPts.begin(),
                localMidPts.end());
        }
    }

    BuildGroupsFromMatchMap(
        uniformMatchMap,
        ThicknessMatchType::Uniform);

    BuildGroupsFromMatchMap(
        variableMatchMap,
        ThicknessMatchType::Variable);
}

bool MidSurfClassification::BuildOrderedWire(
    const std::vector<TopoDS_Edge>& edges,
    TopoDS_Wire& wire)
{
    BRepBuilderAPI_MakeWire mkWire;
    for (const auto& e : edges)
        mkWire.Add(e);

    if (!mkWire.IsDone())
        return false;

    wire = mkWire.Wire();
    return true;
}

bool MidSurfClassification::ExtractGroupOuterWire(const OccMidSurfGroup& group, bool isSideA, std::vector<TopoDS_Edge>& outerEdges)
{
    std::vector<TopoDS_Edge> edgArr;

    const auto& facLabs = isSideA ? group.GetSideALabs() : group.GetSideBLabs();

    for (UINT facLab : facLabs)
    {
        const TopoDS_Face& face = m_orgFacArr[facLab];

        TopoDS_Wire w = BRepTools::OuterWire(face);
        for (TopExp_Explorer ex(w, TopAbs_EDGE); ex.More(); ex.Next())
        {
            const TopoDS_Edge& e = TopoDS::Edge(ex.Current());
            bool found = false;
            for (int i = 0; i < edgArr.size(); i++)
            {
                TopoDS_Edge existE = edgArr[i];
                if (e.IsSame(existE))
                {
                    found = true;
                    edgArr.erase(edgArr.begin() + i);
                    break;
                }
            }
            if (!found)
                edgArr.push_back(e);
        }
    }

    /*for (auto ed : edgArr)
    {
        Standard_Real f1, l1, f2, l2;
        gp_Pnt p1 = BRep_Tool::Curve(ed, f1, l1)->Value(f1);
        m_points.push_back(p1);
    }*/

    //for test
    //m_edges = edgArr;

    // 排序+定向
    std::vector<TopoDS_Edge> noUsedEdges;
    if (!SortAndOrientEdges(edgArr, outerEdges, noUsedEdges, Precision::Confusion()))
        return false;
    /*GpPntArr midPt;
    TopoDS_Face f;
    OccMidSurfCommFunction::BuildFaceFromWire(sortedEdges, midPt, f);
    m_fac.push_back(f);*/
    // 构造 Wire
    return true;
}

bool MidSurfClassification::SortAndOrientEdges(
    const std::vector<TopoDS_Edge>& inEdges,
    std::vector<TopoDS_Edge>& outEdges,
    std::vector<TopoDS_Edge>& noUsedEdges,
    double tol )
{
    if (inEdges.empty())
        return false;

    std::vector<TopoDS_Edge> unused = inEdges;
    outEdges.clear();

    // 1. 取第一条边，保持其原始方向
    TopoDS_Edge curr = unused.front();
    unused.erase(unused.begin());
    outEdges.push_back(curr);

    // 当前尾点：curr 的“终点”
    TopoDS_Vertex vStart, vEnd;
    TopExp::Vertices(curr, vStart, vEnd);
    gp_Pnt tail = BRep_Tool::Pnt(vEnd);
    gp_Pnt st = BRep_Tool::Pnt(vStart);

    // 2. 逐条寻找可连接的边
    while (!unused.empty())
    {
        bool found = false;

        for (auto it = unused.begin(); it != unused.end(); ++it)
        {
            const TopoDS_Edge& e = *it;

            TopoDS_Vertex veStart, veEnd;
            TopExp::Vertices(e, veStart, veEnd);

            gp_Pnt pStart = BRep_Tool::Pnt(veStart);
            gp_Pnt pEnd = BRep_Tool::Pnt(veEnd);

            // tail -> e.start
            if (tail.Distance(pStart) <= tol)
            {
                outEdges.push_back(e);
                tail = pEnd;
                unused.erase(it);
                found = true;
                break;
            }
            // tail -> e.end（需要反向）
            else if (tail.Distance(pEnd) <= tol)
            {
                TopoDS_Edge eRev = OccMidSurfCommFunction::ReverseEdgeWithVertices(e);

                TopoDS_Vertex veStart2, veEnd2;
                TopExp::Vertices(eRev, veStart2, veEnd2);

                gp_Pnt pStart2 = BRep_Tool::Pnt(veStart2);
                gp_Pnt pEnd2 = BRep_Tool::Pnt(veEnd2);

                outEdges.push_back(eRev);
                tail = pStart;
                unused.erase(it);
                found = true;
                break;
            }
        }

        // 找不到可连接的边，说明拓扑不连续
        if (!found)
        {
            noUsedEdges = unused;
            break;
        }
    }

    // 3. 闭合性检查（首边起点 vs 末边终点）
    {
        TopoDS_Vertex vFirstStart, vFirstEnd;
        TopoDS_Vertex vLastStart, vLastEnd;

        TopExp::Vertices(outEdges.front(), vFirstStart, vFirstEnd);
        TopExp::Vertices(outEdges.back(), vLastStart, vLastEnd);

        gp_Pnt pFirst = BRep_Tool::Pnt(vFirstStart);
        gp_Pnt pLast = BRep_Tool::Pnt(vLastEnd);

        if (pFirst.Distance(pLast) > tol)
            return false;
    }

    return true;
}

bool MidSurfClassification::FindNearestBFaceForEdge(
    const TopoDS_Edge& edge,
    const std::vector<TopoDS_Face>& bFaces,
    TopoDS_Face& nearestBFace)
{
    if (bFaces.empty())
        return false;

    // 1️⃣ 取 edge 的起点 / 中点 / 终点
    Standard_Real f, l;
    Handle(Geom_Curve) curve = BRep_Tool::Curve(edge, f, l);
    if (curve.IsNull())
        return false;

    gp_Pnt p0 = curve->Value(f);
    gp_Pnt p1 = curve->Value(0.5 * (f + l));
    gp_Pnt p2 = curve->Value(l);

    double minSumDist = Precision::Infinite();
    bool found = false;

    // 2️⃣ 遍历所有 B 面
    for (const auto& bFace : bFaces)
    {
        gp_Pnt noUsePnt;
        double d0 = OccMidSurfCommFunction::PointToFaceDistance(p0, bFace, noUsePnt);
        double d1 = OccMidSurfCommFunction::PointToFaceDistance(p1, bFace, noUsePnt);
        double d2 = OccMidSurfCommFunction::PointToFaceDistance(p2, bFace, noUsePnt);
        double sumDist = d0 + d1 + d2;

        if (sumDist < minSumDist)
        {
            minSumDist = sumDist;
            nearestBFace = bFace;
            found = true;
        }
    }

    return found;
}


bool MidSurfClassification::BuildMidBoundaryWire(
    const OccMidSurfGroup& group,
    const TopoDS_Wire& aOuterWire,
    TopoDS_Wire& midWire)
{
    std::vector<TopoDS_Edge> outEdges;
    std::vector<gp_Pnt> midPts;
    const int SAMPLE_N = 20;

    std::vector<TopoDS_Face> bFacArr;
    for (auto& bFacLab : group.GetSideBLabs())
    {
        bFacArr.push_back(m_orgFacArr[bFacLab]);
    }
   
    for (TopExp_Explorer ex(aOuterWire, TopAbs_EDGE); ex.More(); ex.Next())
    {
        TopoDS_Edge aEdge = TopoDS::Edge(ex.Current());

        TopoDS_Face bestBFace;
        if (!FindNearestBFaceForEdge(aEdge, bFacArr, bestBFace))
            continue;

        //std::cout << GetFaceLab(bestBFace) << std::endl;

        Standard_Real f, l;
        Handle(Geom_Curve) c = BRep_Tool::Curve(aEdge, f, l);
        if (c.IsNull())
            continue;

        TColgp_Array1OfPnt arr(1, SAMPLE_N + 1);
        for (int i = 0; i <= SAMPLE_N; ++i)
        {
            double t = f + (l - f) * i / SAMPLE_N;
            gp_Pnt pA = c->Value(t);

            gp_Pnt pB;
            if (!OccMidSurfCommFunction::ProjectPointToFace(pA, bestBFace, pB))
            {
                assert(false);
                continue;
            }

            gp_Pnt pMid(
                0.5 * (pA.X() + pB.X()),
                0.5 * (pA.Y() + pB.Y()),
                0.5 * (pA.Z() + pB.Z())
            );
            arr.SetValue(i + 1, pMid);
        }

        Handle(Geom_BSplineCurve) midCurve =
            GeomAPI_PointsToBSpline(arr).Curve();

        TopoDS_Edge midEdge = BRepBuilderAPI_MakeEdge(midCurve);
        outEdges.push_back(midEdge);
    }
    //m_edges = outEdges;
    return BuildOrderedWire(outEdges, midWire);
}
bool MidSurfClassification::BuildMidBoundaryWire(
    const OccMidSurfGroup& group,
    const std::vector<TopoDS_Edge> &outerEdges,
    std::vector<TopoDS_Edge>& midWire)
{
    std::vector<TopoDS_Edge> outEdges;
    std::vector<gp_Pnt> midPts;
    const int SAMPLE_N = 20;

    std::vector<TopoDS_Face> bFacArr;
    for (auto& bFacLab : group.GetSideBLabs())
    {
        bFacArr.push_back(m_orgFacArr[bFacLab]);
    }
   
    for (auto & aEdge: outerEdges)
    {

        //m_edges.push_back(aEdge);
        TopoDS_Face bestBFace;
        if (!FindNearestBFaceForEdge(aEdge, bFacArr, bestBFace))
            continue;

        std::cout<<"edgecnt" << m_edges.size()-1 << "facelab: " << GetFaceLab(bestBFace)<<std::endl;
        //std::cout << GetFaceLab(bestBFace) << std::endl;

        Standard_Real f, l;
        Handle(Geom_Curve) c = BRep_Tool::Curve(aEdge, f, l);
        if (c.IsNull())
            continue;

        TColgp_Array1OfPnt arr(1, SAMPLE_N + 1);
        for (int i = 0; i <= SAMPLE_N; ++i)
        {
            double t = f + (l - f) * i / SAMPLE_N;
            gp_Pnt pA = c->Value(t);

            gp_Pnt pB;
            if (!OccMidSurfCommFunction::ProjectPointToFace(pA, bestBFace, pB))
            {
                assert(false);
                continue;
            }
            /*gp_Vec bNrm;
            OccMidSurfCommFunction::GetNormalAtPoint(bestBFace, pB, bNrm);*/

            gp_Pnt pMid(
                0.5 * (pA.X() + pB.X()),
                0.5 * (pA.Y() + pB.Y()),
                0.5 * (pA.Z() + pB.Z())
            );
            arr.SetValue(i + 1, pMid);
        }

        Handle(Geom_BSplineCurve) midCurve =
            GeomAPI_PointsToBSpline(arr).Curve();

        TopoDS_Edge midEdge = BRepBuilderAPI_MakeEdge(midCurve);
        m_edges.push_back(midEdge);
        outEdges.push_back(midEdge);
    }
    midWire = outEdges;
    return true;
}


void MidSurfClassification::BuildGroupMidSurfOuterLoop()
{
    // 1️⃣ 获取所有中面分组（注意是引用）
    std::vector<OccMidSurfGroup>& grpArr =
        m_midSurfGrouper.GetMidSurfGroup();

    // 2️⃣ 遍历每一个中面分组
    for (OccMidSurfGroup& group : grpArr)
    {
        /* ===============================
         * Step 1: 提取 A 面整体外环
         * =============================== */
        std::vector<TopoDS_Edge>outerEdges;
        if (!ExtractGroupOuterWire(group, true /* Side A */, outerEdges))
        {
            // 该组无法形成有效外环，直接跳过
            continue;
        }

        /* ===============================
         * Step 2: 构造中线外环（Mid Outer Loop）
         *  - A 外环 → 投影到 B → 取中点
         * =============================== */
        //TopoDS_Wire midOuterWire;
        std::vector<TopoDS_Edge> midOuterWire;
        if (!BuildMidBoundaryWire(group, outerEdges, midOuterWire))
        {
            continue;
        }
        
        group.SetMidEdges(midOuterWire);

    }
}

using EdgeCountMap = TopTools_DataMapOfShapeInteger;

void MidSurfClassification::CollectGroupOuterEdges(
    const std::vector<TopoDS_Face>& faces,
    TopTools_ListOfShape& outerEdges)
{
    EdgeCountMap edgeCount;

    for (const auto& face : faces)
    {
        for (TopExp_Explorer exp(face, TopAbs_EDGE); exp.More(); exp.Next())
        {
            const TopoDS_Edge& e = TopoDS::Edge(exp.Current());
            if (!edgeCount.IsBound(e))
                edgeCount.Bind(e, 1);
            else
                edgeCount.ChangeFind(e)++;
        }
    }

    // 只出现一次的是外环
    for (EdgeCountMap::Iterator it(edgeCount); it.More(); it.Next())
    {
        if (it.Value() == 1)
            outerEdges.Append(it.Key());
    }
}
void MidSurfClassification::GetFacCenPnt(TopoDS_Face& fac, gp_Pnt& centerPnt)
{
    // 1. 获取修剪后的 UV 范围
    Standard_Real uMin, uMax, vMin, vMax;
    BRepTools::UVBounds(fac, uMin, uMax, vMin, vMax);

    // 2. 取 UV 中心
    Standard_Real uMid = 0.5 * (uMin + uMax);
    Standard_Real vMid = 0.5 * (vMin + vMax);

    // 3. 底层曲面
    Handle(Geom_Surface) surf = BRep_Tool::Surface(fac);

    // 4. UV → 3D
    surf->D0(uMid, vMid, centerPnt);
}

void MidSurfClassification::GetFacCenPntAndNrmVec(
    TopoDS_Face& fac,
    gp_Pnt& centerPnt,
    gp_Dir& nrmVec)
{
    // 1️⃣ 修剪后的 UV 范围
    Standard_Real uMin, uMax, vMin, vMax;
    BRepTools::UVBounds(fac, uMin, uMax, vMin, vMax);

    // 2️⃣ UV 中心
    const Standard_Real uMid = 0.5 * (uMin + uMax);
    const Standard_Real vMid = 0.5 * (vMin + vMax);

    // 3️⃣ 底层曲面
    Handle(Geom_Surface) surf = BRep_Tool::Surface(fac);

    // 4️⃣ 求点 + 一阶导
    gp_Vec du, dv;
    surf->D1(uMid, vMid, centerPnt, du, dv);

    // 5️⃣ 法向（右手系）
    gp_Vec n = du.Crossed(dv);

    // 6️⃣ 防止退化
    if (n.SquareMagnitude() < Precision::Confusion())
    {
        // 极少数情况下（奇异点）可轻微偏移 UV
        surf->D1(uMid + 1e-6, vMid + 1e-6, centerPnt, du, dv);
        n = du.Crossed(dv);
    }

    n.Normalize();

    // 7️⃣ 考虑 Face 方向（非常关键）
    if (fac.Orientation() == TopAbs_REVERSED)
    {
        n.Reverse();
    }

    nrmVec = gp_Dir(n);
}

void MidSurfClassification::GetFacDiscretePnt(
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