#pragma once

#include<vector>
#include<map>

#include"occTypeDefine.h"

/**
 * @brief 中面分组结构
 *        记录 A/B 两侧面标签、中面标签、厚度以及中面点集
 */
class OccMidSurfGroup
{
public:
    enum class ThicknessType
    {
        Unknown = 0,
        Uniform = 1,   // 等厚度
        Variable = 2   // 变厚度
    };


    OccMidSurfGroup() = default;
    ~OccMidSurfGroup() = default;

    void SetThicknessType(ThicknessType type);
    ThicknessType GetThicknessType() const;

    bool IsUniformThickness() const;
    bool IsVariableThickness() const;

    // =========================================================
    // 邻接信息结构
    // =========================================================
    struct AdjInfo
    {
        std::set<int> AA; // A - A
        std::set<int> AB; // A - B
        std::set<int> BA; // B - A
        std::set<int> BB; // B - B
    };

    /* ---------- Get ---------- */
    const FaceLabArr& GetSideALabs() const;
    const FaceLabArr& GetSideBLabs() const;

    GpPntArr& GetMidPntArr();

    UINT GetGrpLab() const;

    const AdjInfo& GetAdjInfo() const;

    const std::set<int>& GetAdjAA() const;
    const std::set<int>& GetAdjAB() const;
    const std::set<int>& GetAdjBA() const;
    const std::set<int>& GetAdjBB() const;

    const double& GetMidFacThicnkess() const;

    /* ---------- Set ---------- */
    void SetSideALabs(const FaceLabArr& labs);
    void SetSideBLabs(const FaceLabArr& labs);

    void SetGrpLab(UINT midFacLab);
    void SetMidFacThicnkess(double thickness);

    // 新增保存中线边
    void SetMidWire(const TopoDS_Wire& wire)
    {
        m_midWire = wire;

    }
    void SetMidEdges(const std::vector<TopoDS_Edge>& edgeArr)
    {
        m_midEdgeArr = edgeArr;
    }

    void SetAdjInfo(const AdjInfo& info);

    /* ---------- Add（推荐接口） ---------- */
    void AddSideALab(UINT facLab);
    void AddSideBLab(UINT facLab);

    void AddAdjAA(int gid);
    void AddAdjAB(int gid);
    void AddAdjBA(int gid);
    void AddAdjBB(int gid);

    void AddMidPoint(const gp_Pnt& pt);

    void ClearAdjInfo();

    
    const TopoDS_Wire& GetMidWire() const
    {
        return m_midWire;
    }
    const std::vector<TopoDS_Edge>& GetMidEdges() const
    {
        return m_midEdgeArr;
    }

    void SetMidFace(const TopoDS_Face& face) { m_midFace = face; }
    const TopoDS_Face& GetMidFace() const { return m_midFace; }

    std::vector<MidPointKey>& GetMidFacKeys() { return m_keys; }

private:

    FaceLabArr m_ASideLabs;
    FaceLabArr m_BSideLabs;

    ThicknessType m_thicknessType = ThicknessType::Unknown;

    UINT       m_midFacLab = 0;
    double     m_thickness = 0.0;

    GpPntArr   m_midPntArr;

    TopoDS_Wire m_midWire; // 保存每条 A-edge 对应的中线 edge

    std::vector<TopoDS_Edge> m_midEdgeArr;

    TopoDS_Face m_midFace; //生成的中面

    std::vector<MidPointKey> m_keys;

    AdjInfo m_adjInfo;
};

class OccMidSurfGrouper
{
public:
	OccMidSurfGrouper();
	~OccMidSurfGrouper();

    enum MidFacType
    {
        ASideFac = 0,
        BSideFac = 1,
        Unknown = 2
    };

    std::vector<OccMidSurfGroup>& GetMidSurfGroup();

    std::map<int, int>& GetFacLabToGrpLabMap()
    {
        return  m_facLabToGrpLabMap;
    }

    std::map<int, MidFacType>& GetFacLabToTypeMap() 
    {
        return m_facLabToTypeMap;
    }

    const int GetFacLabGrp(const int& facLab);

    const MidFacType GetFacType(const int& facLab);

    void SetFacLabGrp(const int& facLab ,const int &grpLab);

    void SetFacType(const int& facLab ,MidFacType type);
    
    void AddMidSurfGroup(OccMidSurfGroup& grp);
	
private:
	std::vector<OccMidSurfGroup> m_grpArr;
    UINT m_grpLab;
    std::map<int, int> m_facLabToGrpLabMap;
    std::map<int, MidFacType> m_facLabToTypeMap;
};

