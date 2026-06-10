#include"occMidSurfGrouper.h"

#include<cassert>
//Group
const FaceLabArr& OccMidSurfGroup::GetSideALabs() const
{
    return m_ASideLabs;
}

const FaceLabArr& OccMidSurfGroup::GetSideBLabs() const
{
    return m_BSideLabs;
}

void OccMidSurfGroup::SetThicknessType(ThicknessType type)
{
    m_thicknessType = type;
}

OccMidSurfGroup::ThicknessType OccMidSurfGroup::GetThicknessType() const
{
    return m_thicknessType;
}

bool OccMidSurfGroup::IsUniformThickness() const
{
    return m_thicknessType == ThicknessType::Uniform;
}

bool OccMidSurfGroup::IsVariableThickness() const
{
    return m_thicknessType == ThicknessType::Variable;
}

GpPntArr& OccMidSurfGroup::GetMidPntArr()
{
    return m_midPntArr;
}

UINT OccMidSurfGroup::GetGrpLab() const
{
    return m_midFacLab;
}

const double& OccMidSurfGroup::GetMidFacThicnkess() const
{
    return m_thickness;
}

const OccMidSurfGroup::AdjInfo&
OccMidSurfGroup::GetAdjInfo() const
{
    return m_adjInfo;
}

const std::set<int>&
OccMidSurfGroup::GetAdjAA() const
{
    return m_adjInfo.AA;
}

const std::set<int>&
OccMidSurfGroup::GetAdjAB() const
{
    return m_adjInfo.AB;
}

const std::set<int>&
OccMidSurfGroup::GetAdjBA() const
{
    return m_adjInfo.BA;
}

const std::set<int>&
OccMidSurfGroup::GetAdjBB() const
{
    return m_adjInfo.BB;
}

/* ---------- Set ---------- */

void OccMidSurfGroup::SetSideALabs(const FaceLabArr& labs)
{
    m_ASideLabs = labs;
}

void OccMidSurfGroup::SetSideBLabs(const FaceLabArr& labs)
{
    m_BSideLabs = labs;
}

void OccMidSurfGroup::SetGrpLab(UINT midFacLab)
{
    m_midFacLab = midFacLab;
}

void OccMidSurfGroup::SetAdjInfo(const AdjInfo& info)
{
    m_adjInfo = info;

}
void OccMidSurfGroup::SetMidFacThicnkess(double thickness)
{
    m_thickness = thickness;
}

/* ---------- Add ---------- */

void OccMidSurfGroup::AddSideALab(UINT facLab)
{
    m_ASideLabs.push_back(facLab);
}

void OccMidSurfGroup::AddSideBLab(UINT facLab)
{
    m_BSideLabs.push_back(facLab);
}

void OccMidSurfGroup::AddMidPoint(const gp_Pnt& pt)
{
    m_midPntArr.push_back(pt);
}

void OccMidSurfGroup::AddAdjAA(int gid)
{
    m_adjInfo.AA.insert(gid);
}

void OccMidSurfGroup::AddAdjAB(int gid)
{
    m_adjInfo.AB.insert(gid);
}

void OccMidSurfGroup::AddAdjBA(int gid)
{
    m_adjInfo.BA.insert(gid);
}

void OccMidSurfGroup::AddAdjBB(int gid)
{
    m_adjInfo.BB.insert(gid);
}

//clear
void OccMidSurfGroup::ClearAdjInfo()
{
    m_adjInfo.AA.clear();
    m_adjInfo.AB.clear();
    m_adjInfo.BA.clear();
    m_adjInfo.BB.clear();
}

//Grouper
OccMidSurfGrouper::OccMidSurfGrouper():m_grpLab(0)
{
}

OccMidSurfGrouper::~OccMidSurfGrouper()
{
}

std::vector<OccMidSurfGroup>& OccMidSurfGrouper::GetMidSurfGroup()
{
	return m_grpArr;
}

const int OccMidSurfGrouper::GetFacLabGrp(const int &facLab)
{
    if (m_facLabToGrpLabMap.find(facLab) != m_facLabToGrpLabMap.end())
    {
        return m_facLabToGrpLabMap[facLab];
    }
    return -1;
}

const OccMidSurfGrouper::MidFacType 
OccMidSurfGrouper::GetFacType(const int& facLab)
{
    if (m_facLabToTypeMap.find(facLab) != m_facLabToTypeMap.end())
    {
        return m_facLabToTypeMap[facLab];
    }
    return MidFacType::Unknown;
}

void OccMidSurfGrouper::SetFacLabGrp(const int& facLab, const int& grpLab)
{
    m_facLabToGrpLabMap[facLab] = grpLab;
}

void OccMidSurfGrouper::SetFacType(const int& facLab, MidFacType type)
{
    m_facLabToTypeMap[facLab] = type;
}

void OccMidSurfGrouper::AddMidSurfGroup(OccMidSurfGroup& grp)
{
	m_grpArr.push_back(grp);
	grp.SetGrpLab(m_grpLab++);

    /*for (auto& lab : grp.GetSideALabs())
    {
        if (m_facLabToGrpLabMap.find(lab) == m_facLabToGrpLabMap.end())
        {
            m_facLabToGrpLabMap.emplace(lab, grp.GetGrpLab());
        }
        else
        {
            assert(false);
        }
        if (m_facLabToTypeMap.find(lab) == m_facLabToTypeMap.end())
        {
            m_facLabToTypeMap.emplace(lab, MidFacType::ASideFac);
        }
        else
        {
            assert(false);
        }
    }
    for (auto& lab : grp.GetSideBLabs())
    {
        if (m_facLabToGrpLabMap.find(lab) == m_facLabToGrpLabMap.end())
        {
            m_facLabToGrpLabMap.emplace(lab, grp.GetGrpLab());
        }
        else
        {
            assert(false);
        }
        if (m_facLabToTypeMap.find(lab) == m_facLabToTypeMap.end())
        {
            m_facLabToTypeMap.emplace(lab, MidFacType::BSideFac);
        }
        else
        {
            assert(false);
        }
    }*/

}

