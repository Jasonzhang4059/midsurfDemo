/********************************************************************
* FILE NAME		:EXData.cpp
* AUTHOR		:XXX
* DATE			:May 17,2002
* MODIFIER		:
* MODIFY DATE	:
* DESCRIPTION	:implement functions for EX`s basic classes
* version		:1.0
********************************************************************/

#include "XData.h"
#include "XFunction.h"

namespace base {

//XVert
XVert::XVert()
: m_flag(0)
, m_IsOnBoundary(false)
, m_GaussCurvature(0)
, m_Deformation(0)
, m_Curvature(0)
 //, m_TotalAngle(0),
, m_vid(-1)
, m_symId(0)
, m_mark(0)
, m_fin(0)
, m_projFid(-1)
, m_st(0, 0) 

, m_uHarmonicValue(1.f)
,m_vHarmonicValue(1.f)
,m_bCurvatureSensitive(false)
,m_FieldRealVal(1.f)
,m_oriFieldRealVal(1.f)
{
	m_color = { 0, 0, 0 };
	m_CurtureColor = { 0, 0, 0 };
	uvw[0] = 0;
	uvw[1] = 0;
	uvw[2] = 0;
};

XVert::~XVert()
{
};


void XVert::DelAdjE(int ne) 
{
	bool flag=false;
	for(int i=0; i < static_cast<int>(m_adjEdge.size()); i++)
	{
		if(!flag && ne==m_adjEdge[i])
		{
			flag=true;
			continue;
		}

		if(flag)
			m_adjEdge[i-1] = m_adjEdge[i];
	}

	if(flag)
		m_adjEdge.resize(m_adjEdge.size()-1);
};

void XVert::DelAdjF(int nf)
{
	bool flag=false;
	for(int i=0; i<static_cast<int>(m_adjFace.size()); i++)
	{
		if(!flag && nf==m_adjFace[i])
		{
			flag=true;
			continue;
		}

		if(flag)
			m_adjFace[i-1] = m_adjFace[i];
	}

	if(flag)
		m_adjFace.resize(m_adjFace.size()-1);
};
void XVert::DelAdjP(int np)
{
	bool flag=false;
	for(int i=0; i<static_cast<int>(m_adjPoint.size()); i++)
	{
		if(!flag && np==m_adjPoint[i])
		{
			flag=true;
			continue;
		}

		if(flag)
			m_adjPoint[i-1] = m_adjPoint[i];
	}

	if(flag)
		m_adjPoint.resize(m_adjPoint.size()-1);
}
XVertEx::XVertEx()
: m_VertVisited(false)
, m_constrainedFlag(false)
, m_fixedFlag(false)
, m_MappingTID(0)
, m_newAdded(false)
, m_cornerFlag(false)
, m_featureFlag(false)
, m_TotalAngle(0)
{
};

XVertEx::~XVertEx()
{
};

//XEdge
XEdge::XEdge()  
: m_bVisible(true)
, m_orilen(0)
, m_curlen(0)
, m_IsBoundary(0)
, m_flag(0)
{ 
	m_idx[0] = -1;
	m_idx[1] = -1;
	m_fidx[0] = -1;
	m_fidx[1] = -1;
};


XEdge::~XEdge() 
{
	//m_newvertid.clear();
};
// Get the number of adjacent faces for an edge
int XEdge::GetAdjFCount()const
{    
	int facecount = 2;
	if(m_fidx[1] == -1)
		--facecount;
	if(m_fidx[0] == -1)
		--facecount;
	return facecount;
};

XEdgeEx::XEdgeEx()  
: m_visitedFlag(false)
, m_newAdded(false)
, m_iStripNO(0)
, m_crossLen(0)
, m_featureFlag(false)
{ 
	m_adjPtIdx[0] = -1;
	m_adjPtIdx[1] = -1;
};


XEdgeEx::~XEdgeEx() 
{
};
// XFace
XFace::XFace()
: m_bVisible(true)
, m_iSubMeshID(-1)

, m_bHalfVisible(false)
, m_IsBoundary(false)
, m_AreaIn2D(0)
, m_OriArea(0)
, m_matID(0)
, m_flag(0)
, m_projFid(-1)
, m_normIn2D(0)
{ 
	m_color = { 0, 0, 0 };
	m_idx[0] = -1;
	m_idx[1] = -1;
	m_idx[2] = -1;
	m_eref[0] = -1;
	m_eref[1] = -1;
	m_eref[2] = -1;
	m_adj[0] = -1;
	m_adj[1] = -1;
	m_adj[2] = -1;
	m_angle[0] = 0;
	m_angle[1] = 0;
	m_angle[2] = 0;
	m_oriAngle[0] = 0;
	m_oriAngle[1] = 0;
	m_oriAngle[2] = 0;
	m_pPos[0] = m_pPos[1] = m_pPos[2] = NULL;
};

XFace::~XFace()
{
};

float& XFace::Angle(int i)
{
	if(i>2||i<0)
	{
//		AfxMessageBox(ATT_0016);
		return m_angle[i];
	}
	else
	{
		return m_angle[i];
	}
};


XFaceEx::XFaceEx()
: m_MeshVisited(false)
, m_MappingTID(-1)
, m_deletedFlag(false)
, m_iStripNO(0)
{ 
	m_relatedPID[0] = -1;
	m_relatedPID[1] = -1;
	m_relatedPID[2] = -1;
	m_expectedAngle[0] = 0;
	m_expectedAngle[1] = 0;
	m_expectedAngle[2] = 0;
	m_expandingAngle[0] = 0;
	m_expandingAngle[1] = 0;
	m_expandingAngle[2] = 0;
	mapPara[0][0] = 0;
	mapPara[0][1] = 0;
	mapPara[0][2] = 0;
	mapPara[1][0] = 0;
	mapPara[1][1] = 0;
	mapPara[1][2] = 0;
	mapPara[2][0] = 0;
	mapPara[2][1] = 0;
	mapPara[2][2] = 0;
};

XFaceEx::~XFaceEx()
{
};
// Remove a vertex from the list
bool XFaceEx::RemoveFromVList(int vid)
{
	int i;
	bool flag=false;
	for(i=0;i<static_cast<int>(m_vlist.size());i++)
	{
		if(m_vlist.at(i)==vid)
		{
			m_vlist.erase(&m_vlist.at(i));
			flag=true;
			i--;
		}
	}
	return flag;
}



////---------------------------------------------------------------
//// Name:	    GetVertex()
//// Description: Get the index of  three vertices in the face
//// Argument:	v1,v2,v3:-- three vertices of the face
////         :	
//// Return:		
//// Author:	    XXX
//// Date:	    2006/04/30 30:4:2006   16:16
//// Update:	 
//// Author:	
//// Date: 
//// copyright: XXX. developed by XXX
////----------------------------------------------------------------
//void XFace::GetVertex(int &nv1, int &nv2, int &nv3) const
//{
//	nv1 = m_idx[0];
//	nv2 = m_idx[1];
//	nv3 = m_idx[2];
//}

////---------------------------------------------------------------
//// Name:	    CalNormal()
//// Description: Compute the face normal
//// Argument:
////         :	
//// Return:		
//// Author:	    XXX
//// Date:	    2006/04/30 30:4:2006   16:16
//// Update:	 
//// Author:	
//// Date: 
//// copyright: XXX. developed by XXX
////----------------------------------------------------------------
//void XFace::CalNormal()
//{
//	Vec4 v1,v2;
//	float len;
//
//	v1 = m_pt[1]-m_pt[0];
//	v2 = m_pt[2]-m_pt[1];
//
//	m_norm.x = v1.y * v2.z - v1.z * v2.y;
//	m_norm.y = v1.z * v2.x - v1.x * v2.z;
//	m_norm.z = v1.x * v2.y - v1.y * v2.x;
//
//	len = (float)sqrt(m_norm.x * m_norm.x + m_norm.y * m_norm.y + m_norm.z * m_norm.z);
//
//	if(len < 0.1e-6) 
//	{
//		m_norm=Vec4(1,0,0);
//	}
//	else	
//	{
//		m_norm=m_norm/len;
//	}
//}

////---------------------------------------------------------------
//// Name:	    Intersect_Line()
//// Description: Intersect the face with a line
//// Argument:	v1,v2:-- line endpoints
////         :	
//// Return:		Vec4:-- intersection point
//// Author:	    XXX
//// Date:	    2006/04/30 30:4:2006   16:20
//// Update:	 
//// Author:	
//// Date: 
//// copyright: XXX. developed by XXX
////----------------------------------------------------------------
//Vec4 XFace::Intersect_Line(Vec4 v1,Vec4 v2)
//{
//	int sd1=0,sd2=0;
//	float val1,val2;
//	Vec4 sp; 
//	float ratio;
//
//	val1=m_norm.x*(v1.x-m_pt[0].x)+m_norm.y*(v1.y-m_pt[0].y)+m_norm.z*(v1.z-m_pt[0].z);
//	if(val1>=0)	    sd1=1;
//	else sd1=-1;
//
//	val2=m_norm.x*(v2.x-m_pt[0].x)+m_norm.y*(v2.y-m_pt[0].y)+m_norm.z*(v2.z-m_pt[0].z);
//
//	if(val2>=0) 
//	{
//		sd2=1;
//	}
//	else
//	{
//		sd2=-1;
//	}
//
//	if((sd1==1&&sd2==-1)||(sd1==-1&&sd2==1))
//	{
//		val1=(float)fabs(val1);
//		val2=(float)fabs(val2);
//		ratio=val1/(val1+val2);
//
//		sp=v1+(v2-v1)*ratio;
//	}
//	else
//	{
//		sp=(v2*val1-v1*val2)/(val1-val2); 
//	}
//
//	//	float dval1=norm.x*(sp.x-m_pt[0].x)+norm.y*(sp.y-m_pt[0].y)+norm.z*(sp.z-m_pt[0].z);
//
//	return sp;
//}
//
////---------------------------------------------------------------
//// Name:	    IsPtInside()
//// Description: Judge whether a point is inside the face 
//// Argument:	pt:-- point to be tested
////         :	ne:-- which edge judge the point is out of the triangle, no very important meaning 
//// Return:		
//// Author:	    XXX
//// Date:	    2006/04/30 30:4:2006   16:25
//// Update:	 
//// Author:	
//// Date: 
//// copyright: XXX. developed by XXX
////----------------------------------------------------------------
//bool XFace::IsPtInside(Vec4 pt, int &ne)
//{
//	int i;
//	Vec4 plane_norm;
//	Vec4 edge_vect;
//	float dval;
//
//
//	for(i=0; i<=2; i++)
//	{
//		if(i==0)	    edge_vect = m_pt[1]-m_pt[0];
//		else if(i==1)	edge_vect = m_pt[2]-m_pt[1];
//		else if(i==2)	edge_vect = m_pt[0]-m_pt[2];
//
//		plane_norm = edge_vect.multiple(m_norm);
//
//		dval=plane_norm.x*(pt.x-m_pt[i].x)+plane_norm.y*(pt.y-m_pt[i].y)+plane_norm.z*(pt.z-m_pt[i].z);
//
//		if(dval>0)
//		{
//			ne = GetEdgeRef(i);
//			return false;
//		}
//
//	}
//	return true;
//}

////---------------------------------------------------------------
//// Name:	    ComputeUVW()
//// Description: Compute the u,v,w coordinate for a point taking this face as reference 
//// Argument:	pt:-- point
////         :	u,v,w:--u,v,w coordinate
//// Return:		
//// Author:	    XXX
//// Date:	    2006/04/30 30:4:2006   16:28
//// Update:	 
//// Author:	
//// Date: 
//// copyright: XXX. developed by XXX
////----------------------------------------------------------------
//void XFace::ComputeUVW(Vec4 pt, float &u, float &v, float &w)
//{
//	float dval, dist1, dist2;
//	Vec4 edge_vect, plane_norm, vt1, vt2, vt;
//	bool flag;
//	float uvw[3];
//
//	for(int i=0; i<3; i++)
//	{
//		vt = m_pt[i];
//
//		if(i==0)
//		{ 
//			vt1 = m_pt[1]; vt2 = m_pt[2];  
//		}
//		else if(i==1)
//		{
//			vt1 = m_pt[2]; vt2 = m_pt[0];  
//		}
//		else if(i==2)
//		{
//			vt1 = m_pt[0]; vt2 = m_pt[1];  
//		}
//
//		dist1 = GetDistOfPntToLn(pt, vt1, vt2);
//		dist2 = GetDistOfPntToLn(vt, vt1, vt2);
//
//		edge_vect = vt2 - vt1;
//		plane_norm = edge_vect.multiple(m_norm);
//
//		dval=plane_norm.x*(pt.x-vt1.x)+plane_norm.y*(pt.y-vt1.y)+plane_norm.z*(pt.z-vt1.z);
//		if(dval>0) 
//		{
//			flag = false;
//		}
//		else
//		{
//			flag = true;
//		}
//
//		if(flag==true)
//		{
//			uvw[i] = dist1/dist2;
//		}
//		else
//		{
//			uvw[i] = -dist1/dist2;
//		}
//	}
//
//	u = uvw[0];
//	v = uvw[1];
//	w = uvw[2];
//}
//
//
////---------------------------------------------------------------
//// Name:	    ComputePosFromUVW()
//// Description: Recover the point coordinate by the u,v,w and the face
//// Argument:	u,v,w:-- u,v,w coordinate
////         :	
//// Return:		Vec4: point recovered
//// Author:	    XXX
//// Date:	    2006/04/30 30:4:2006   16:29
//// Update:	 
//// Author:	
//// Date: 
//// copyright: XXX. developed by XXX
////----------------------------------------------------------------
//Vec4  XFace::ComputePosFromUVW(float u, float v, float w)
//{
//	Vec4 pt;
//
//	float uvw = u+v+w;
//	u= u/uvw;
//	v = v/uvw;
//	w = w/uvw;
//
//	pt = m_pt[0]*u + m_pt[1]*v + m_pt[2]*w;
//	return pt;
//}
//

void MaterialProperty::SetMaterialPara(float WeftConstant, float WarpConstant, 
		float minConstant, float MaxDeformRetioOfWeft, float MaxDeformRetioOfWrap,float MaxDeformRetioOfDiagonal)
{
	m_WeftConstant=WeftConstant;
	m_WarpConstant=WarpConstant;
	m_minConstant=minConstant;
	m_MaxDeformRetioOfWeft=MaxDeformRetioOfWeft;
	m_MaxDeformRetioOfWrap=MaxDeformRetioOfWrap;
	m_MaxDeformRetioOfDigonal= MaxDeformRetioOfDiagonal;
};



}