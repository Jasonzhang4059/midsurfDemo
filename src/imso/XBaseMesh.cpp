
#include "base.h"
#include "definition.h"
#include "XBaseMesh.h"
#include "XFunction.h"
#include <qmatrix4x4.h>

//#include "IntelMathLib/include/mkl.h"
namespace base{

CriPoints::CriPoints(void)
:vidx(-1)
,pt(Vec4(0.f,0.f,0.f))
,type(-1)
,candelRatio(0.f)
{

}
CriPoints::~CriPoints()
{

}
CriPoints& CriPoints::operator =(const CriPoints& cpt)
{
	pt = cpt.pt;
	vidx = cpt.vidx;
	type = cpt.type;
	candelRatio = cpt.candelRatio;
	return *this;
}
//---------------------------------------------------------------- 
XBaseMesh::XBaseMesh(void)
: m_bRectExisted(false)
, m_bCentreExisted(false)
, m_bAverageLenthInitFlag(false)
, m_bMeshBox3DComput(false)
, m_oriAverageLen(0)
, m_averageCrossingLen(0)
, m_scale(1)
, m_iMeshID(0)
{
	clear();
	m_smooth_angle = 180.0f;
	m_iModelMainOrientation = 0;
}


//---------------------------------------------------------------
// Name:		~XBaseMesh()
// Description:	Deconstruction of class XBasemesh
// Argument:	null
// Return:		void
// Author:		
// Date:		
// Modified by:	D. XXX	
// Updated date:05/15/2006
//---------------------------------------------------------------- 
XBaseMesh::~XBaseMesh(void)
{
	if(m_vert.size()>0)
	{
		m_vert.clear();
	}
	if(m_edge.size()>0)
	{
		m_edge.clear();
	}
	if(m_face.size()>0)
	{
	    m_face.clear();
	}
}




//---------------------------------------------------------------
// Name:		clear()
// Description:	Clear the data of class XBasemesh
// Argument:	null
// Return:		void
// Author:		
// Date:		
// Modified by:	D. XXX	
// Updated date:05/15/2006
//---------------------------------------------------------------- 
void XBaseMesh::clear()
{
	m_vert.resize(0); // here use resize to reduce memery operations
	m_edge.resize(0);
	m_face.resize(0);
	/*m_vert.clear();
	m_edge.clear();
	m_face.clear();*/
	m_AngleInited = false;
	m_oriEdgeLenInitedFlag=false;
	m_bAverageLenthInitFlag = false;
	m_BoundFlagSet = false;
}
//---------------------------------------------------------------
// Name:		GetDataFromXBaseMesh
// Function:	get the data from XBaseMesh object.
// Argument:	
// Return:		
// Author:		unknown
// Date:		unknown
// Modified by:		XXX(add comments)
// Updated date:	7/12/2006	
//---------------------------------------------------------------- 
void XBaseMesh::GetDataFromXBaseMesh(const XBaseMesh& baseMs)
{
	m_vert	= baseMs.m_vert;
	m_face	= baseMs.m_face;
	m_edge	= baseMs.m_edge;
	minp	= baseMs.minp;
	maxp	= baseMs.maxp;
	m_trans	= baseMs.m_trans;
	SetSmoothAngle(baseMs.GetSmoothAngle());
	m_scale = baseMs.m_scale;
	m_oriEdgeLenInitedFlag = baseMs.m_oriEdgeLenInitedFlag;
	m_oriAverageLen = baseMs.m_oriAverageLen;
	m_bAverageLenthInitFlag = baseMs.m_bAverageLenthInitFlag;
	m_BoundFlagSet = baseMs.m_BoundFlagSet;
	m_iModelMainOrientation = baseMs.m_iModelMainOrientation;
	//m_oldFacesNum = baseMs.m_oldFacesNum;
}
//---------------------------------------------------------------
// Name:		MakeXEdges()
// Description:	Establish the edge information of a mesh
// Argument:	null
// Return:		void
// Author:		
// Date:		
// Modified by:	D. XXX	
// Updated date:05/15/2006
// the implicit input is face and vertex and their relationship
//---------------------------------------------------------------- 
void  XBaseMesh::MakeXEdges() 
{
	int i, j;
	//static UINT fidxctable_p1[3]={1,2,0};

    //DWORD start = GetTickCount();

	int fSize = GetFSize();
	int vtSize = GetVSize();

	if (fSize < 1 || vtSize < 1)
		return;

	m_edge.resize(0);
	m_edge.reserve(fSize + vtSize - 2 + 100);// use Euler formula to evaluate the edge size approximately.

	int curMaxEdgeidx = 0;
	int vidx0 = 0,vidx1 = 0;
	int inarrayindex = 0;
	XEdge e;
	varray< varray<int> > vertexrelationship; // this is the edge flag with vertex index
	varray< varray<int> > vertexIdxarr;
	vertexrelationship.resize(vtSize);
	for(varray<int>* p = vertexrelationship.end() - 1; p >= vertexrelationship.begin(); --p)
	{
		p->reserve(8);
	}
	vertexIdxarr.resize(vtSize);
	for(varray<int>* p = vertexIdxarr.end() - 1; p >= vertexIdxarr.begin(); --p)
	{
		p->reserve(8);
	}

	XVert* endv = m_vert.end();
	for(XVert* iv = m_vert.begin(); iv < endv; ++iv)
	{
		iv->FastInit();
	}
	XFace* endf = m_face.end();
	for(XFace* ivf = m_face.begin(); ivf < endf; ++ivf)
	{
		ivf->FastInit();
	}

	i = 0;
	for(XFace* ivf = m_face.begin(); ivf < endf; ++ivf, ++i)
	{
		//XFace& f = GetF(i);
//		f.SetNormIndex(f.GetIndex(0), f.GetIndex(1), f.GetIndex(2));
		for(j = 0; j < 3; ++j)
		{
			vidx0 = ivf->p(j);
			vidx1 = ivf->p((j + 1)%3);
			assert(vidx0 >= 0 && vidx0 < static_cast<int>(m_vert.size()));
			assert(vidx1 >= 0 && vidx1 < static_cast<int>(m_vert.size()));
			XVert& v0 = GetV(vidx0);
			XVert& v1 = GetV(vidx1);

			//inarrayindex = GetInTheVarrayIndex(vidx1,vertexrelationship.at(vidx0));
			varray<int>& pVarray = vertexrelationship.at(vidx0);
			int* location = std::find(pVarray.begin(),pVarray.end(),vidx1);
			if(location != pVarray.end())
				inarrayindex =  static_cast<int>(location - pVarray.begin());
			else
				inarrayindex =  -1;

			if(inarrayindex > -1)// the two vertex already has an edge between them
			{
				assert(IsInTheVarray(vidx0,vertexrelationship.at(vidx1)));
				int edgeidx = vertexIdxarr.at(vidx0).at(inarrayindex);
				assert(edgeidx < static_cast<int>(m_edge.size()));
				XEdge& oldE = m_edge.at(edgeidx);
				oldE.GetFIdx(1) = i;

				// the following code set the newly added vertex information
				//if(!IsInTheVarray(i, v0.AdjF()))
				if(std::find(v0.AdjF().begin(),v0.AdjF().end(),i) == v0.AdjF().end()) 
				{
					v0.AddAdjF(i);
				}
				//if(!IsInTheVarray(i, v1.AdjF()))
				if(std::find(v1.AdjF().begin(),v1.AdjF().end(),i) == v1.AdjF().end()) 
				{
					v1.AddAdjF(i);
				}
				// over

				// the following code set the face information
				ivf->SetEdgeRef(j, edgeidx);
				ivf->SetAdjacent(j, oldE.GetFIdx(0));
				XFace& otherF = m_face.at(oldE.GetFIdx(0));
				for(int k = 0; k < 3; ++k)
				{
					if(static_cast<int>(otherF.GetEdgeRef(k)) == edgeidx) // ensure the adjacent face information sequence coincide with the edge information.
					{
						otherF.SetAdjacent(k,i);
						break;
					}
				}
				// over
			}
			else // the two vertex has no edge between them, need create an edge
			{
				assert(!IsInTheVarray(vidx0,vertexrelationship.at(vidx1)));
				e.SetIndex(vidx0,vidx1); // set the edge adjacent vertext
				e.GetFIdx(0) = i; // set the edge adjacent face index, here always set the first adjacent face.
				if(ivf->GetVisible()) //set the edge visible information
					e.GetVisible() = true;
				else 
					e.GetVisible() = false|e.GetVisible();

				// the following code set the newly added vertex information
				v0.AddAdjE(curMaxEdgeidx);
				v1.AddAdjE(curMaxEdgeidx);
				//if(!IsInTheVarray(i, v0.AdjF()))
				if(std::find(v0.AdjF().begin(),v0.AdjF().end(),i) == v0.AdjF().end()) 
				{
					v0.AddAdjF(i);
				}
				//if(!IsInTheVarray(i, v1.AdjF()))
				if(std::find(v1.AdjF().begin(),v1.AdjF().end(),i) == v1.AdjF().end()) 
				{
					v1.AddAdjF(i);
				}
				// over

				// the following code set the face information
				ivf->SetEdgeRef(j, curMaxEdgeidx);// only one face found,no information to create adjacent faces
				// over

				// set the temporary information
				m_edge.push_back(e);
				vertexrelationship.at(vidx0).push_back(vidx1);
				vertexrelationship.at(vidx1).push_back(vidx0);
				vertexIdxarr.at(vidx0).push_back(curMaxEdgeidx);
				vertexIdxarr.at(vidx1).push_back(curMaxEdgeidx);
				curMaxEdgeidx++;
				// over
			}
		}
	}
	SetAdjacentPt();

}




//---------------------------------------------------------------
// Name:		ComputeNormals()
// Description:	Compute the normal of each vertex
//              The normal of one vertex is the sum of all adjacent face normal.
// Argument:	null
// Return:		void
// Author:		
// Date:		
// Modified by:	D. XXX	
// Updated date: Apr. 25, 2006		
//---------------------------------------------------------------- 
void XBaseMesh::ComputeNormals(bool bIsHaveToCal/* = false*/)
{
	//int i;
	Vec4 norm;
	//int vSize = GetVSize();
	//int fSize = GetFSize();
	XVert* endv = m_vert.end();
	XFace* endf = m_face.end();

	for(XVert* v = m_vert.begin(); v < endv; ++v)
	{
		v->Norm().Zero();
	}

	for(XFace* f = m_face.begin(); f < endf; ++f)
	{
	    const Vec4& p1 =  GetV(f->GetIndex(0)).Pos();
	    const Vec4& p2 =  GetV(f->GetIndex(1)).Pos();
	    const Vec4& p3 =  GetV(f->GetIndex(2)).Pos();

		//get current face normal.
		norm = CrossVecX(p2-p1, p3-p2).Normalize();
		f->Norm() = norm;

		//add face normal to its vertexes.
		GetV(f->GetIndex(0)).Norm() += norm;
		GetV(f->GetIndex(1)).Norm() += norm;
		GetV(f->GetIndex(2)).Norm() += norm;
	}

	// get the unit normal
	for(XVert* v = m_vert.begin(); v < endv; ++v)
	{ 
		if(!bIsHaveToCal)
		{
			if(v->GetAdjFSize()==0) 
			{
				continue;
			}
		}
		 //GetV(i).Norm() = GetV(i).Norm().Normalize();
		Vec4& normV = v->Norm();
		normV = normV.Normalize();
	}
}

//---------------------------------------------------------------
// Name:		ComputeFaceNormal()
// Description:	compute the normal of a face
// Argument:	i: ID of the face
// Return:		normal of the face
// Author:		
// Date:		
// Modified by:	D. XXX	
// Updated date:05/15/2006
//---------------------------------------------------------------- 
Vec4 XBaseMesh::ComputeFaceNormal(int i)
{
	XFace& f = GetF(i);
	Vec4& v0 = GetV(f.GetIndex(0)).Pos();
	Vec4& v1 = GetV(f.GetIndex(1)).Pos();
	Vec4& v2 = GetV(f.GetIndex(2)).Pos();
	f.Norm() = CrossVecX((v1 - v0), (v2 - v0)).Normalize();
	return f.Norm();
}		


//---------------------------------------------------------------
// Name:		CopyMesh()
// Description:	Copy a mesh
// Argument:	xms_to: the target mesh to be copied 
// Return:		void
// Author:		
// Date:		
// Modified by:	D. XXX	
// Updated date:05/15/2006
//---------------------------------------------------------------- 
void XBaseMesh::CopyMesh(XBaseMesh& xms_to)
{
	xms_to = *this;
}
//void XBaseMesh::RenderMeshShade(const COLORREF& meshColor,float fLineWidth)
//{
//	glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
//	glEnable(GL_POLYGON_OFFSET_FILL);
//	glPolygonOffset(1.0, 1.0);
//
//	glLineWidth(fLineWidth);
//	float oldColor[4] = {0,0,0,0};	
//	glGetFloatv(GL_CURRENT_COLOR, oldColor); // get the old color
//	glColor3f(GetRValue(meshColor)/255.0f,GetGValue(meshColor)/255.0f,GetBValue(meshColor)/255.0f);
//	glDisable(GL_BLEND);
//	glEnable(GL_LIGHTING);
//	glShadeModel(GL_SMOOTH); //smooth render
//
//	RenderMeshData();
//
//	glColor4fv(oldColor);//recover the old color
//	glDepthMask(GL_TRUE);
//	glDisable(GL_POLYGON_OFFSET_FILL);
//}
//void XBaseMesh::RenderMeshTransparent(const COLORREF& meshColor,float fLineWidth)
//{
//	glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
//	glEnable(GL_POLYGON_OFFSET_FILL);
//	glPolygonOffset(1.0, 1.0);
//
//	glLineWidth(fLineWidth);
//	glEnable(GL_LIGHTING);
//
//	float factor, units;
//	glGetFloatv(GL_POLYGON_OFFSET_FACTOR, &factor);
//	glGetFloatv(GL_POLYGON_OFFSET_UNITS, &units);
//
//	float oldColor[4] = {0,0,0,0};	
//	glGetFloatv(GL_CURRENT_COLOR, oldColor);
//	glColor4f(1.0,1.0,1.0,1.0);
//
//	glShadeModel(GL_SMOOTH); //smooth render
//	glEnable(GL_BLEND); 
//	glColorMask(GL_FALSE,GL_FALSE,GL_FALSE,GL_FALSE);
//	glEnable(GL_DEPTH_TEST);
//
//	
//	RenderMeshData();
//	
//	glColorMask(GL_TRUE,GL_TRUE,GL_TRUE,GL_TRUE);
//	glColor4fv(oldColor);
//
//
//	glEnable(GL_BLEND);
//	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
//	//glColor4f(GetRValue(m_gColor)/255.0f,GetGValue(m_gColor)/255.0f,GetBValue(m_gColor)/255.0f,0.6f);
//	glColor4f(GetRValue(meshColor)/255.0f,GetGValue(meshColor)/255.0f,GetBValue(meshColor)/255.0f,0.6f);
//	RenderMeshData();
//	glDisable(GL_BLEND);	
//	glDepthMask(GL_TRUE);
//	glDisable(GL_POLYGON_OFFSET_FILL);
//	glColor4fv(oldColor);
//}
//void XBaseMesh::RenderMeshWire(const COLORREF& meshColor,float fLineWidth)
//{
//	glLineWidth(fLineWidth);
//
//	glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
//	glEnable(GL_POLYGON_OFFSET_FILL);
//	glPolygonOffset(1.0, 1.0);
//	float oldColor[4] = {0,0,0,0};	
//	glGetFloatv(GL_CURRENT_COLOR, oldColor);
//
//
//	//	glColor3f(GetRValue(m_msColor)/255.0f,GetGValue(m_msColor)/255.0f,GetBValue(m_msColor)/255.0f);
//	glColor3f(GetRValue(meshColor)/255.0f,GetGValue(meshColor)/255.0f,GetBValue(meshColor)/255.0f);
//	glColorMask(GL_FALSE,GL_FALSE,GL_FALSE,GL_FALSE);
//	glEnable(GL_DEPTH_TEST);
//
//	RenderMeshData();
//
//	glColorMask(GL_TRUE,GL_TRUE,GL_TRUE,GL_TRUE);
//
//	int i=0;
//	Vec4 v1, v2, v3, nor1, nor2, nor3;
//	glDisable(GL_LIGHTING);
//	int faceNum = GetFSize();
//	for(i = 0; i < faceNum; i++) 
//	{
//		const XFace& face = GetF(i);
//		if(!face.m_bVisible)
//		{
//			continue;
//		}
//		nor1	= GetV(face.GetIndex(0)).Norm();
//		v1		= GetV(face.GetIndex(0)).Pos();
//		nor2	= GetV(face.GetIndex(1)).Norm();
//		v2		= GetV(face.GetIndex(1)).Pos();
//		nor3	= GetV(face.GetIndex(2)).Norm();
//		v3		= GetV(face.GetIndex(2)).Pos();
//		glBegin(GL_LINE_LOOP);
//		glNormal3f(nor1.x, nor1.y, nor1.z);
//		glVertex3f(v1.x, v1.y, v1.z);
//		glNormal3f(nor2.x, nor2.y, nor2.z);
//		glVertex3f(v2.x, v2.y, v2.z);
//		glNormal3f(nor3.x, nor3.y, nor3.z);
//		glVertex3f(v3.x, v3.y, v3.z);
//		glEnd();
//	}
//	glEnable(GL_LIGHTING);
//	glColor4fv(oldColor);//recover the old color
//	glDepthMask(GL_TRUE);
//	glDisable(GL_POLYGON_OFFSET_FILL);
//}
//void XBaseMesh::RenderMeshVerts(const COLORREF& ptsColor,float fPointSize)
//{
//	float oldColor[4] = {0,0,0,0};	
//	glGetFloatv(GL_CURRENT_COLOR, oldColor);
//	glColor3f(GetRValue(ptsColor)/255.0f,GetGValue(ptsColor)/255.0f,GetBValue(ptsColor)/255.0f);
//
//	int i = 0, iVSize = GetVSize();
//
//	glPointSize(fPointSize);
//	glBegin(GL_POINTS);
//	for(i = 0; i < iVSize; i++)
//	{
//		Vec4& vt = GetV(i).Pos();
//		glVertex3f(vt.x,vt.y,vt.z);
//	}
//	glEnd();
//
//	glColor4fv(oldColor);//recover the old color
//
//}
//
//void XBaseMesh::RenderMeshData()
//{
//	glBegin(GL_TRIANGLES);
//	int i=0;
//	//Vec4 v1, v2, v3, nor1, nor2, nor3;
//	int fSize = GetFSize();
//
//	for(i = 0; i < fSize; i++) 
//	{
//		const XFace& face = GetF(i);
//		if(!face.m_bVisible)
//		{
//			continue;
//		}
//		const Vec4& nor1	= GetV(face.GetIndex(0)).Norm();
//		const Vec4& v1		= GetV(face.GetIndex(0)).Pos();
//		const Vec4& nor2	= GetV(face.GetIndex(1)).Norm();
//		const Vec4& v2		= GetV(face.GetIndex(1)).Pos();
//		const Vec4& nor3	= GetV(face.GetIndex(2)).Norm();
//		const Vec4& v3		= GetV(face.GetIndex(2)).Pos();
//
//		glNormal3f(nor1.x, nor1.y, nor1.z);
//		glVertex3f(v1.x, v1.y, v1.z);
//		glNormal3f(nor2.x, nor2.y, nor2.z);
//		glVertex3f(v2.x, v2.y, v2.z);
//		glNormal3f(nor3.x, nor3.y, nor3.z);
//		glVertex3f(v3.x, v3.y, v3.z);
//	}
//	glEnd();
//}



//---------------------------------------------------------------
// Name:		InitEdgeLen()
// Description: Compute the length of all edges in a mesh
// Argument:	null
// Return:		void
// Author:		unknown
// Date:		unknown
// Modified by:	D. XXX
// Updated date: Apr. 20, 2006
//---------------------------------------------------------------- 
void XBaseMesh::InitEdgeLen()
{
	//int i;
	Vec4 vect;

	m_oriAverageLen=0.0f;

	//int eSize = GetESize();
	XEdge* ende = m_edge.end();

	for(XEdge* e=m_edge.begin(); e<ende; ++e)
	{
		vect = GetV(e->p(0)).Pos() - GetV(e->p(1)).Pos();
		e->OriLen() =  vect.Magnitude();
		e->CurLen() =  e->OriLen();
		m_oriAverageLen += e->OriLen();
	}

	m_oriAverageLen=m_oriAverageLen/(float)GetESize();

	m_oriEdgeLenInitedFlag=true; // flag to show whether the length of edges has been computed
	m_bAverageLenthInitFlag = true;

	return;
}

//---------------------------------------------------------------
// Name:		AverageLenCalculation
// Function:	compute the average length of the edge.
// Argument:	
// Return:		
// Author:		unknown
// Date:		unknown
// Modified by:		XXX(add comments)
// Updated date:	4/28/2006	
//---------------------------------------------------------------- 

void XBaseMesh::AverageLenCalculation()
{
	int ESize=GetESize();
	float sumLen=0.0;
	for(int i=0; i<ESize;i++)
	{
		sumLen+=(GetV(GetE(i).p(1)).Pos()-GetV(GetE(i).p(0)).Pos()).Magnitude();
	}
	m_oriAverageLen=sumLen/(float)ESize;
	m_bAverageLenthInitFlag = true;
}

void XBaseMesh::ChangeModelNorm()
{
	double Min = 0;
	int iFace=-1;
	int fSize = GetFSize();

	// get first Min
	if(fSize>0)
	{
		Min=GetV(0).Pos().z;
	}

	// get one face that have the least z coordinate value
	for(int i=0;i<fSize;i++)
	{
		XFace &face = GetF(i);
		if(GetV(face.p(0)).Pos().z>Min)
		{
			Min=GetV(face.p(0)).Pos().z;
			iFace=i;
		}
	}

	if(iFace!=-1)
	{
		// opinion iFace's norm
		if(Dot(GetF(iFace).Norm() , Vec4(0,0,1))<1e-6) // yao 2002-1-15
		{	
			int i0, /*in0,*/ i1, /*in1,*/ i2/*, in2*/;

			// set mesh's norm
			for(int i=0;i<fSize;i++)
			{
				XFace &face = GetF(i);
				i0=face.GetIndex(0);
//				in0=face.GetNormIndex(0);
				i1=face.GetIndex(1);
//				in1=face.GetNormIndex(1);
				i2=face.GetIndex(2);
//				in2=face.GetNormIndex(2);

				GetF(i).Norm() = -GetF(i).Norm();
				face.SetIndex(i2,i1,i0);
//				face.SetNormIndex(in2,in1,in0);
			}

			int vSize = GetVSize();
			for(int i=0;i<vSize;i++)
			{
				GetV(i).Norm() = -GetV(i).Norm();
			}
		}
	}

	return;
}
Vec4 XBaseMesh::GetIntersectPlaneNorm(const Vec4 &stpt, int stfaceidx, const Vec4 &endpt, int endfaceidx,int iSpecialType)const
{
	Vec4 norm = ( GetF(stfaceidx).Norm() + GetF(endfaceidx).Norm() )/2;
	if(iSpecialType == 1)
	{	
		norm.y = 0;
		norm.Normalize();
	}
	else if (iSpecialType == 3)
	{	
		norm.x = 0;
		norm.Normalize();
	}
	return CrossVecX( norm , (endpt-stpt) ).Normalize(); 
}
//---------------------------------------------------------------
// Name:	    GetIntersectedFace()
// Description: search a face that intersects with a line from a starting face
// Argument:    ncurf:-- the start face id for beginning to search out the face that  really intersect with the line 
//         :	p1,p2:-- the two endpoints of the line to intersect with the mesh 
// Return:		int:-- the face id that really intersect with the line
// Author:		 
//----------------------------------------------------------------
int XBaseMesh::GetIntersectedFace(int ncurf, const Vec4& p1, const Vec4& p2)
{
	//XFace f;
	Vec4 pt;
	int ne, nf;
	int ni;

	ni=0;
	nf=ncurf;
    if(ncurf < 0)
	{
		return -1;      //XXX 2004-09-30
	}
	do{
		
		// compute the intersection point
		if(nf < 0)
		{
			return -1;		//XXX 2004-09-30
		}
		pt = Intersect_Line(p1, p2,nf);

		// check whether the point is inside the face
		if(IsPtInside(pt, ne ,nf)==true) return nf;
		else
		{
			if(GetE(ne).GetFIdx(0)==ncurf)
			    nf = GetE(ne).GetFIdx(1); 
			else
				nf = GetE(ne).GetFIdx(0);

			ncurf = nf;
		}
		ni++;
	}while(ni<20);

	return -1;
}

//---------------------------------------------------------------
// Name:	     Intersect_Line()
// Description:  Compute the intersect point with the line and face Id is given
//				Do not care whether the intersection is in the face or not.
// Argument:	 v1,v2:-- the two endpoints of the line to be intersect with the face
//         :	 faceID::-- the id of the face which is to be intersect with the line 
// Return:		Vec4:-- the intersect point of the line and the face
// Author:		
// Date:		 
// Modified by:	 D. XXX 
// Updated date: Apr. 25, 2006
//----------------------------------------------------------------
Vec4 XBaseMesh::Intersect_Line(const Vec4& v1,const Vec4& v2,int faceID)
{
	int sd1=0,sd2=0;
	float val1,val2;
	Vec4 sp; 
	float ratio;
    Vec4 vtTemp1 = GetV(GetF(faceID).p(0)).Pos();
	//XFace f;
    
    const XFace& f = GetF(faceID);
	val1=f.Norm().x*(v1.x-vtTemp1.x)+f.Norm().y*(v1.y-vtTemp1.y)+f.Norm().z*(v1.z-vtTemp1.z);

	if(val1>=0)	 
	{
		sd1=1;
	}
	else
	{
		sd1=-1;
	}

	val2=f.Norm().x*(v2.x-vtTemp1.x)+f.Norm().y*(v2.y-vtTemp1.y)+f.Norm().z*(v2.z-vtTemp1.z);
	if(val2>=0) 
	{
		sd2=1;
	}
	else 
	{
		sd2=-1;
	}

	if((sd1==1&&sd2==-1)||(sd1==-1&&sd2==1))
	{
		val1=(float)fabs(val1);
		val2=(float)fabs(val2);
		ratio=val1/(val1+val2);
		
		sp=v1+(v2-v1)*ratio;
	}
	else
	{
		sp=(v2*val1-v1*val2)/(val1-val2); 
	}

	return sp;
}

//---------------------------------------------------------------
// Name:	    IsPtInside()
// Description: Judge whether a point is on the given face 
// Argument:    pt:-- the point to be judged;     
//				faceID:-- the face id for test the point 
//				ne:-- which edge judge the point is out of the triangle, no very important meaning(just for speed up the searching process) 
// Return:		true: the point is inside a triangle; false: outside
// Author:		 
// Date:		 
// Modified by:	 D. Zhag
// Updated date: Apr. 25, 2006
//----------------------------------------------------------------
bool XBaseMesh::IsPtInside(const Vec4& pt, int &ne, int faceID)
{
	Vec4 plane_norm;
	Vec4 edge_vect;
	float dval;
    XFace f;
	Vec4* vtTemp[3];
    f = GetF(faceID);

	vtTemp[0] = &GetV(f.p(0)).Pos();
	vtTemp[1] = &GetV(f.p(1)).Pos();
	vtTemp[2] = &GetV(f.p(2)).Pos();

	for(int i=0; i<=2; i++)
	{
		if(i==0)	    edge_vect = *vtTemp[1] - *vtTemp[0];
		else if(i==1)	edge_vect = *vtTemp[2] - *vtTemp[1];
		else if(i==2)	edge_vect = *vtTemp[0] - *vtTemp[2];
        
		plane_norm = CrossVecX(edge_vect,f.Norm());

		dval=plane_norm.x*(pt.x - vtTemp[i]->x)+plane_norm.y*(pt.y - vtTemp[i]->y)+plane_norm.z*(pt.z - vtTemp[i]->z);
		if(dval>0)
		{
			ne = f.GetEdgeRef(i);
			return false;
		}

	}
	// added by qcy [1/10/2008] for bug 4212 grain line is wrong
	float len1 = (*vtTemp[0] - *vtTemp[1]).Magnitude();
	float len2 = (*vtTemp[0] - *vtTemp[2]).Magnitude();
	float len3 = (*vtTemp[1] - *vtTemp[2]).Magnitude();
	float maxLen = max(len1,max(len2,len3));
	len1 = (pt - *vtTemp[0]).Magnitude();
	len2 = (pt - *vtTemp[1]).Magnitude();
	len3 = (pt - *vtTemp[2]).Magnitude();
	float minLen = min(len1,min(len2,len3));
	if (minLen > maxLen)
	{
		return false;
	}
	// over

	return true;
}

//---------------------------------------------------------------
// Name:	     GetFaceIdxBy
// Description:  Search a face Id with its two boundary edges
// Argument:     edgeoneidx: index of the first edge
//				 edgetwoidx: index of the second edge
// Return:		 triangle ID
// Author:		 
// Date:		 
// Modified by:	 D. XXX
// Updated date: Apr. 25, 2006
//----------------------------------------------------------------
int XBaseMesh::GetFaceIdxBy(int edgeoneidx, int edgetwoidx)const
{
	if(edgeoneidx==-1 || edgeoneidx > GetESize() || edgetwoidx==-1 || edgetwoidx > GetESize())
		return -1;

	for(int j=0;j<2;j++)
	{
		if(GetE(edgeoneidx).GetFIdx(j) == GetE(edgetwoidx).GetFIdx(0) 
			|| GetE(edgeoneidx).GetFIdx(j) == GetE(edgetwoidx).GetFIdx(1))
		{
			return (GetE(edgeoneidx).GetFIdx(j) );
		}
	}

	return -1;
}

void XBaseMesh::GetBarycentCoorIn3D(const Vec4& P, int fid, float& u, float& v, float& w) const
{
	assert(fid >= 0 && fid < GetFSize());
	const XFace& f=GetF(fid);
	const Vec4& v0=GetV(f.p(0)).Pos();
	const Vec4&  v1=GetV(f.p(1)).Pos();
	const Vec4&  v2=GetV(f.p(2)).Pos();
	Vec4 v10 = v1 - v0;
	Vec4 v20 = v2 - v0;
	//Vec4 v21 = v2 - v1;
	Vec4 vp0 = P - v0;
	Vec4 vp1 = P - v1;
	Vec4 vp2 = P - v2;
	Vec4 refN = CrossVecX(v10,v20); // magnitude = area
	refN.unit();

	u = Dot(CrossVecX((vp1),(vp2)),refN);

	v = Dot(CrossVecX((vp2),(vp0)),refN); 

	w = Dot(CrossVecX((vp0),(vp1)),refN);

	float temp=u+v+w;
	if(temp < 1.0e-6)
	{
		u = v = w = (float)(1.0/3.0);
	}
	else
	{
		u=u/temp;
		v=v/temp;
		w=w/temp;
	}

}

//---------------------------------------------------------------
// Name:		GetBarycentCoorIn2D(...)
// Description:	Calculate the barycenter coordinate of a point in 2D 
// Argument:	P: the point to be calculate
//				fid: face index
// Return:		u, v, w: barycenter coordinate of the point
// Author:		ljt
// Date:		03/03/2003
// Modified by:	D. XXX	
// Updated date: Apr. 25,2006
//---------------------------------------------------------------- 
void XBaseMesh::GetBarycentCoorIn2D(const Vec4& P, int fid, float& u, float& v, float& w)const
{
	assert(fid >= 0 && fid < GetFSize());
	const XFace& f=GetF(fid);
	const Vec4& v0=GetV(f.p(0)).Pos2d();
	const Vec4&  v1=GetV(f.p(1)).Pos2d();
	const Vec4&  v2=GetV(f.p(2)).Pos2d();
	Vec4 v10 = v1 - v0;
	Vec4 v20 = v2 - v0;
	//Vec4 v21 = v2 - v1;
	Vec4 vp0 = P - v0;
	Vec4 vp1 = P - v1;
	Vec4 vp2 = P - v2;
	//Vec4 refN = CrossVecX(v10,v20); // magnitude = area
	float refN = v10.x * v20.y - v10.y * v20.x > 0 ? 1.0f : -1.0f;
	//refN.unit();

	//u = Dot(CrossVecX((vp1),(vp2)),refN);
	u = (vp1.x * vp2.y - vp1.y * vp2.x)*refN;

	//v = Dot(CrossVecX((vp2),(vp0)),refN); 
	v = (vp2.x * vp0.y - vp2.y * vp0.x)*refN;

	//w = Dot(CrossVecX((vp0),(vp1)),refN);
	w = (vp0.x * vp1.y - vp0.y * vp1.x)*refN;

	float temp=u+v+w;
	if(temp < 1.0e-6)
	{
		u = v = w = (float)(1.0/3.0);
	}
	else
	{
		u=u/temp;
		v=v/temp;
		w=w/temp;
	}

	return;
}



Vec4 XBaseMesh::GetVertFromBarycentCoorIn3D(int fid, float uvw[3])const
{
	Vec4 ret = GetV(GetF(fid).p(0)).Pos()*uvw[0];
	ret+=GetV(GetF(fid).p(1)).Pos()*uvw[1];
	ret+=GetV(GetF(fid).p(2)).Pos()*uvw[2];
	return ret;
}


Vec4 XBaseMesh::GetVertFromBarycentCoorIn3D(int fid, const Vec4& bary)const
{
	Vec4 ret = GetV(GetF(fid).p(0)).Pos()*bary.x;
	ret+=GetV(GetF(fid).p(1)).Pos()*bary.y;
	ret+=GetV(GetF(fid).p(2)).Pos()*bary.z;
	return ret;	
}


Vec4 XBaseMesh::GetVertFromBarycentCoorIn2D(int fid, float uvw[3])const
{
	Vec4 ret = GetV(GetF(fid).p(0)).Pos2d()*uvw[0];
	ret+=GetV(GetF(fid).p(1)).Pos2d()*uvw[1];
	ret+=GetV(GetF(fid).p(2)).Pos2d()*uvw[2];
	return ret;
}

Vec4 XBaseMesh::GetVertFromBarycentCoorIn2D(int fid, const Vec4& bary)const
{
	Vec4 ret = GetV(GetF(fid).p(0)).Pos2d()*bary.x;
	ret+=GetV(GetF(fid).p(1)).Pos2d()*bary.y;
	ret+=GetV(GetF(fid).p(2)).Pos2d()*bary.z;
	return ret;	
}

//---------------------------------------------------------------
// Name:		AngleInit()
// Description: Calculate the angle of faces in a mesh
// Argument:	null
// Return:		void
// Author:		unknown
// Date:		unknown
// Modified by:	D. XXX
// Updated date: Apr. 20, 2006
//---------------------------------------------------------------- 
void XBaseMesh::AngleInit()
{
	int i;
	Vec4 p0,p1,p2;
	int iFSize=GetFSize();

	for(i=0;i<iFSize;i++)
	{
		AngleInitInAFace(i);
	}

	m_AngleInited=true;  // a flag to label whether angles have been computed.
}

//---------------------------------------------------------------
// Name:		AngleInitInAFace()
// Description: Calculate internal angles of a face
// Argument:	int fid - index of a face
// Return:		void
// Author:		unknown
// Date:		unknown
// Modified by:	D. XXX
// Updated date: Apr. 20, 2006
void XBaseMesh::AngleInitInAFace(int fid)
{
	int j, m, n;
	//Vec4 p0,p1,p2;  // three points of a face
	
	XFace &f=GetF(fid);
	const Vec4& p0 =  GetV(f.GetIndex(0)).Pos();
 	const Vec4& p1 =  GetV(f.GetIndex(1)).Pos();
	const Vec4& p2 =  GetV(f.GetIndex(2)).Pos();

	f.Angle(0) = f.GetOriAngle(0)=GetAngleOf2Vector((p1-p0),(p2-p0));
	f.Angle(1) = f.GetOriAngle(1)=GetAngleOf2Vector((p0-p1),(p2-p1));
	f.Angle(2) = f.GetOriAngle(2)=GetAngleOf2Vector((p0-p2),(p1-p2));

	for(j=0;j<3;j++)
	{
		if(fabs(f.Angle(j))<0.0001f)
		{				
			m=j-1;
			n=j+1;

			if(m<0)
			{
				m=2;
			}
			if(n==3)
			{
				n=0;
			}				

			f.Angle(j)=0.0001f;

			if(f.Angle(m)>f.Angle(n))
			{
				f.Angle(m)=static_cast<float>(f.Angle(m)-(0.0001-f.Angle(j)));
			}
			else
			{
				f.Angle(n)=static_cast<float>(f.Angle(n)-(0.0001-f.Angle(j)));
			}
		}
	}

	return;
}

//----------------------------------------------------------------------
// Name:		GetOriAngle(...)
// Function:	Compute an internal angle of a triangle for a vertex
// Argument:	fid - index of the face
//              pid - index of the vertex
// Return:		angle cooresponding to the point: pid
// Author:		
// Date:		
// Modified by: D. XXX
// Updated date: Apr. 20, 2006
//----------------------------------------------------------------------
float XBaseMesh::GetOriAngle(int fid,int pid)
{
	int i;
	XFace& f=GetF(fid);

	for(i=0;i<3;i++)
	{
		if(static_cast<int>(f.p(i))==pid)
		{
			return(f.GetOriAngle(i));
		}
	}
	return -1.0;
}

float XBaseMesh::GetOrgLenInCrossingSpring(int eid, bool& flag)
{
	if(!m_AngleInited)
	{
		AngleInit();
	}
	int i;
	flag=true;
	const XEdge& sharedE=GetE(eid);
	int meshid1=(int)(sharedE.GetFIdx(0));
	int meshid2=(int)(sharedE.GetFIdx(1));
	if(meshid1<0 || meshid2<0)
	{
		flag=false;
		return -1.0;
	}

	int pid1=sharedE.p(0);
	int pid2=sharedE.p(1);
	const XFace& face1=GetF(meshid1);
	const XFace& face2=GetF(meshid2);
	int otherPid1,otherPid2,tmpPid1,tmpPid2;
	int eid1 = -1,eid2 = -1,tmpEid1,tmpEid2;
	float len1,len2;
	float angle1,angle2;
	//Vec4 v1,v2;
	
	for(i=0;i<3;i++)
	{
		tmpPid1=face1.p(i);
		tmpPid2=face2.p(i);
		tmpEid1=face1.GetEdgeRef(i);
		tmpEid2=face2.GetEdgeRef(i);

		if(tmpPid1!=pid1 && tmpPid1!=pid2)
		{
			otherPid1=tmpPid1;
		}
		if(tmpPid2!=pid1 && tmpPid2!=pid2)
		{
			otherPid2=tmpPid2;
		}
		if(tmpEid1!=eid)
		{
			if(GetE(tmpEid1).p(0)==pid1 || GetE(tmpEid1).p(1)==pid1)
			{
				eid1=tmpEid1;
			}
		}
		if(tmpEid2!=eid)
		{
			if(GetE(tmpEid2).p(0)==pid1 || GetE(tmpEid2).p(1)==pid1)
			{
				eid2=tmpEid2;
			}
		}
	}

	len1=GetE(eid1).OriLen();
	len2=GetE(eid2).OriLen();
	angle1=GetOriAngle(meshid1,pid1);
	angle2=GetOriAngle(meshid2,pid1);

	Vec4 v1(len1*sin(angle1),len1*cos(angle1),0.0);
    Vec4 v2(len2*sin(-angle2),len2*cos(-angle2),0.0);
    //v1, v2 is on the same plane ,that is to say,whenever the angle between the faces these share the same edge changes,the return value is the same and perserve the largest.  
	float crossLen=(v1-v2).Magnitude();
	return crossLen;
}


//----------------------------------------------------------------------
// Name:		BoundaryFlagSetting()
// function:	Set flag for boundary vertices, edges and faces
//			    true: boundary elements, false: not on boundary
// Argument:	void
// Return:		void
// Author:		XXX
// Date:		
// Modified by: D. XXX 	    
// Modified date: Apr. 20, 2006
// Modified by: XXX, for add multi obj to scene
// Modified date: 20060928
//----------------------------------------------------------------------
void XBaseMesh::BoundaryFlagSetting(int strPntIdx, int strFaceIdx, int strEdgeIdx) 
{
	XFace* endf = m_face.end();
	XEdge* ende = m_edge.end();
	XVert* endv = m_vert.end();

	for(XFace* f=m_face.begin()+strFaceIdx;f < endf; ++f)
	{
		//m_face[i].IsBoundary()=false;
		f->IsBoundary() = false;
	}

	for(XEdge* e = m_edge.begin()+strEdgeIdx; e < ende; ++e)
	{
		//m_edge[i].IsBoundary()=false;
		e->IsBoundary() = false;
	}

	for(XVert* v = m_vert.begin()+strPntIdx; v < endv; ++v)
	{
		//m_vert[i].IsOnBoundary()=false;
		v->IsOnBoundary() = false;
	}

	for(XEdge* e = m_edge.begin()+strEdgeIdx; e < ende; ++e)
	{
		if(e->GetFIdx(1)==-1) //if edge has only one adjancent face, it is on the boundary
		{
			e->IsBoundary()=true;
			m_vert[e->p(0)].IsOnBoundary()=true;
			m_vert[e->p(1)].IsOnBoundary()=true;
			GetF(e->GetFIdx(0)).IsBoundary()=true;
		}
	}

	m_BoundFlagSet=true;  // Flag that labels whether boundary flag has been set

	return;
}


//----------------------------------------------------------------------
// name:		ChangeNorm
// function:	For a 3D model, if whose normal direction points to inside,
//				change its normal, just change its vertexs's order
// argument:	void	
// return:		void
// author:		
// date:		
// author:		XXX
// date:		2006/04/18 
//----------------------------------------------------------------------
void XBaseMesh::ChangeNorm(float yTopRef /*= 1.0e6*/, float yBotRef/* = -1.0e6*/)
{
	double	Min = 0;;
	int		i = 0;
	int		iFace = -1;
	int		fSize = GetFSize();

	// get first Min for get least z
	if(GetFSize() > 0)
	{
//		XFace &face = GetF(0);
		Min = GetV(0).Pos().z;
	}
	Min = -1.0e6;
	//get one face that have least z
	for(i=0; i<fSize; i++)
	{
		XFace &face = GetF(i);
		Vec4& vt = GetV(face.p(0)).Pos();
		if(vt.y > yTopRef || vt.y < yBotRef)
		{
			continue;
		}
		if(vt.z > Min)
		{
			Min=GetV(face.p(0)).Pos().z;
			iFace=i;
		}
	}

	if(iFace != -1)
	{
		XFace &face = GetF(iFace);

		// check iFace's norm
		if(Dot(face.Norm(), Vec4(0,0,1)) < 1e-6)
		{	
			int i0, /*in0,*/ i1, /*in1,*/ i2/*, in2*/;

			// set mesh's norm
			for(int i=0;i<GetFSize();i++)
			{
				XFace &face = GetF(i);
				i0  = face.GetIndex(0);
				i1  = face.GetIndex(1);
				i2  = face.GetIndex(2);

//				in0 = face.GetNormIndex(0);
//				in1 = face.GetNormIndex(1);
//				in2 = face.GetNormIndex(2);

				face.Norm() = -face.Norm();
				face.SetIndex(i2,i1,i0);
//				face.SetNormIndex(in2,in1,in0);
			}
			//for(i=0;i<GetNormCount();i++)
			//{
			//	GetNorm(i)=-GetNorm(i);
			//}
		}
	}

}

//---------------------------------------------------------------
// Name:		InitNormal2D
// Function:	compute all the face normals in 2d
// Argument:	
// Return:		
// Author:		unknown
// Date:		unknown
// Modified by:		XXX(add comments)
// Updated date:	4/28/2006	
//---------------------------------------------------------------- 
void XBaseMesh::InitNormal2D()
{
	for(int i=0; i<GetFSize();i++)
	{
		ComputeFaceNormalIn2D(i);
	}
}

//---------------------------------------------------------------
// Name:		ComputeFaceNormalIn2D
// Function:	compute normal of the 2d face
// Argument:	i:face index
// Return:		
// Author:		unknown
// Date:		unknown
// Modified by:		XXX(add comments)
// Updated date:	4/28/2006	
//---------------------------------------------------------------- 
int XBaseMesh::ComputeFaceNormalIn2D(int i)
{
	XFace& f = GetF(i);
	float v0x = GetV(f.GetIndex(0)).Pos2d().x;
	float v0y = GetV(f.GetIndex(0)).Pos2d().y;
	//Vec4 v1(GetV(f.GetIndex(1)).Pos2d());
	//v1-=v0;
	//Vec4 v2(GetV(f.GetIndex(2)).Pos2d());
	//v2-=v0;
	float v1x = GetV(f.GetIndex(1)).Pos2d().x;
	float v1y = GetV(f.GetIndex(1)).Pos2d().y;
	float v2x = GetV(f.GetIndex(2)).Pos2d().x;
	float v2y = GetV(f.GetIndex(2)).Pos2d().y;
	//v1x -= v0x;
	//v1y -= v0y;
	//v2x -= v0x;
	//v2y -= v0y;
	//if(v1x*v2y - v1y*v2x > 0) // only calculate the z value 
	if((v1x - v0x)*(v2y - v0y) - (v1y - v0y)*(v2x - v0x) > 0)
		return f.NormIn2D() = 1;
	else
		return f.NormIn2D() = -1;
	//return f.NormIn2D() = CrossVecX((v1 - v0), (v2 - v0)).Normalize();	
}

//----------------------------------------------------------------------
// Name:		GetOriAngleOnVert()
// Function:	Compute an internal angle of a triangle for a vertex
// Argument:	fid - index of the face
//              pid - index of the vertex
// Return:		Angle cooresponding to the point: pid
// Author:		
// Date:		
// Modified by: D. XXX
// Updated date: Apr. 20, 2006
//----------------------------------------------------------------------
float XBaseMesh::GetOriAngleOnVert(int fid,int pid)
{
	int i;
	for(i=0;i<3;i++)
	{
		if(static_cast<int>(GetF(fid).p(i))==pid)
		{
			return GetF(fid).GetOriAngle(i);
		}
	}
	return -1.0;
}

//---------------------------------------------------------------
// Name:		GetSharedEdge
// Function:	get the shared edge index by two face indexes
// Argument:	currFid,adjFid: the two face indexes.
// Return:		if exist, return the edge index, else -1.
// Author:		unknown
// Date:		unknown
// Modified by:		XXX(add comments)
// Updated date:	4/27/2006	
//---------------------------------------------------------------- 
int XBaseMesh::GetSharedEdge(const int currFid, const int adjFid)const
{
	int i,j;
	const XFace& currF=GetF(currFid);
	const XFace& adjF=GetF(adjFid);
	for(i=0;i<3;i++)
	{
		for(j=0;j<3;j++)
		{
			if(currF.GetEdgeRef(i)==adjF.GetEdgeRef(j))
			{
				return (currF.GetEdgeRef(i));
			}
		}
	}
	return -1;
}

//---------------------------------------------------------------
// Name:		GetAdjFid
// Function:	get the adjacent face by edge index
// Argument:	fid: the face index
//				eid: the edge index
// Return:		the adjacent face index
// Author:		unknown
// Date:		unknown
// Modified by:		XXX(add comments)
// Updated date:	4/28/2006	
//---------------------------------------------------------------- 

int XBaseMesh::GetAdjFid(int fid, int eid)
{
	if(GetE(eid).GetAdjFCount()==1)
	{
		return -1;
	}

	else
	{
		if(GetE(eid).GetFIdx(0)!=fid)
		{
			return (GetE(eid).GetFIdx(0));
		}
		else
		{
			return (GetE(eid).GetFIdx(1));
		}
	}
}

//---------------------------------------------------------------
// Name:		GetLinkedEidInAFace
// Function:	get the two edge id linked to the given vertex
// Argument:	fid: the face index
//				pid: the vertex index
// Return:		eid1,eid2: the two edge indexes connected to the vertex
// Author:		unknown
// Date:		unknown
// Modified by:		XXX(add comments)
// Updated date:	4/28/2006	
//---------------------------------------------------------------- 
void XBaseMesh::GetLinkedEidInAFace(int fid, int pid, int& eid1, int& eid2)
{
	int i,j,k;
	eid1=eid2=-1;
	const XFace& currF=GetF(fid);
	for(i=0;i<3;i++)
	{
		if(static_cast<int>(currF.p(i))==pid)
		{
			const XVert& currV=GetV(currF.p(i));
			int adjESize=currV.GetAdjESize();
			for(j=0;j<adjESize;j++)
			{
				int tempEid=currV.AdjE(j);
				for(k=0;k<3;k++)
				{
					if(tempEid==static_cast<int>(currF.GetEdgeRef(k)))
					{
						if(eid1==-1)
						{
							eid1=currF.GetEdgeRef(k);
						}
						else
						{
							eid2=currF.GetEdgeRef(k);
							break;
						}
					}
				}
			}
		}
	}
}

//---------------------------------------------------------------
// Name:		GetTirVid
// Function:	get the vertex index other than the edge vertexes on the face
// Argument:	fid: the face index
//				eid: the edge index(the edge should belong to the face, or it may not get the correct result)
// Return:		the vertex index other than the edge vertexes on the face
// Author:		unknown
// Date:		unknown
// Modified by:		XXX(add comments)
// Updated date:	4/26/2006	
//---------------------------------------------------------------- 

int XBaseMesh::GetTirVid(int fid, int eid)
{
	for(int i=0;i<3;i++)
	{
		if(static_cast<int>(GetF(fid).p(i))!=GetE(eid).p(0) && static_cast<int>(GetF(fid).p(i))!=GetE(eid).p(1))
		{
			return GetF(fid).p(i);
		}
	}
	return -1;
}

//to a base triangle and its adjacent triangle and their converged vertex,
//
//---------------------------------------------------------------
// Name:		GetTirVid
// Function:	get the other vertex of their shared edge and the vertex opposite to the shared edge in adjacent triangle	
// Argument:	baseFid,adjFid:a base triangle and its adjacent triangle
//				convergedVid:converged vertex index
// Return:		the other vertex of their shared edge and the vertex opposite to the shared edge in adjacent triangle
// Author:		unknown
// Date:		unknown
// Modified by:		XXX(add comments)
// Updated date:	4/28/2006	
//---------------------------------------------------------------- 
void XBaseMesh::GetTirVid(int baseFid, int adjFid, int convergedVid,int& baseTirVNO,int& baseTirVid,int& adjTirVid)
{
	int i;
	const XFace& baseF=GetF(baseFid);
	const XFace& adjF=GetF(adjFid);
	int eid=GetSharedEdge(baseFid,adjFid);
	int tempVid1=GetE(eid).p(0);
	int tempVid2=GetE(eid).p(1);
	if(tempVid1!=convergedVid)
	{
		baseTirVid=tempVid1;
	}
	else if(tempVid2!=convergedVid)
	{
		baseTirVid=tempVid2;
	}
	for(i=0;i<3;i++)
	{
		if(static_cast<int>(baseF.p(i))==baseTirVid)
		{
			baseTirVNO=i;
		}
		if(static_cast<int>(adjF.p(i))!=baseTirVid && static_cast<int>(adjF.p(i))!=convergedVid)
		{
			adjTirVid=adjF.p(i);
		}
	}
}

//---------------------------------------------------------------
// Name:		IsTwoPtsAtSameArea
// Function:	test if the two point on same mesh
// Argument:	stfaceidx,endfaceidx: the first and second face to test
//				stpt,endpt: the start and end point on the face.
// Return:		
// Author:		unknown
// Date:		unknown
// Modified by:		XXX(add comments)
// Updated date:	4/28/2006	
//---------------------------------------------------------------- 
bool XBaseMesh::IsTwoPtsAtSameArea(int stfaceidx, const Vec4 &stpt, int endfaceidx, const Vec4 &endpt)//XXX 2003-0404
{
	if(stfaceidx < 0 || endfaceidx < 0 || stfaceidx >= GetFSize() || endfaceidx >= GetFSize())
		return false;

	if(endfaceidx==stfaceidx)
		return true;
#ifdef _DEBUG
	if((stpt - endpt).Magnitude() < 1.0e-20)// if the two point at the same position but different face, should be different area.
		return false;
#endif

	varray<int> curFList;
	varray<int> nextFList;
	varray<bool> vFlags;
	vFlags.resize(GetFSize(),false);
	curFList.push_back(stfaceidx);
	vFlags[stfaceidx]=true;

	int i,j,k;
	while(true)
	{
		if(curFList.size()==0)
			break;
		for(i=0;i<static_cast<int>(curFList.size());i++)
		{
			const XFace& f=GetF(curFList.at(i));
			for(j=0;j<3;j++)
			{
				const XVert& v=GetV(f.p(j));
				for(k=0;k<v.GetAdjFSize();k++)
				{
					if(vFlags[v.AdjF(k)])
						continue;
					if(v.AdjF(k)==endfaceidx)
						return true;

					nextFList.push_back(v.AdjF(k));
					vFlags[v.AdjF(k)]=true;
				}
			}
		}
		curFList.clear();
		curFList=nextFList;
		nextFList.clear();
	}
	return false;

	/*
	Vec4 plist[2000];
	int   pedges[2000];
	int   pnum = 0;

	int   pedges1[2000];
	int   pnum1 = 0;
	int i, j;

	Vec4 stonept, sttwopt,endonept,endtwopt;

	Vec4 norm;
	norm=GetIntersectPlaneNorm(stpt, stfaceidx, endpt, endfaceidx);
	if(IntersectPlane(stpt, stfaceidx,norm , plist, pedges, pnum, false ) )
	{
	if(pnum == 0 )
	{
	return true;//XXX 2003-04-14
	}
	if(GetE(pedges[0]).GetFIdx(0) != -1 && GetE(pedges[0]).GetFIdx(1) != -1)
	{
	return true;
	}

	stonept = plist[0];
	sttwopt = plist[pnum-1];
	if(IntersectPlane(endpt, endfaceidx, norm, plist, pedges1, pnum1, false ) )
	{

	for(i=0; i<pnum; i++)
	{
	for(j=0; j<pnum1; j++)
	{
	if(pedges[i] == pedges1[j])
	{
	return true;
	}
	}
	}
	}
	}

	return false;*/
}

//---------------------------------------------------------------
// Name:		GetFaceIdxBy(...)
// Description:	Get a face index by its three vertices
// Argument:	vid1, vid2, vid3: vertex index of a face
// Return:		face index
// Author:	    XXX
// Date:		Sept. 08, 2005
// Modified by:	D. XXX
// Updated date: Apr. 25, 2006
//---------------------------------------------------------------- 
int  XBaseMesh::GetFaceIdxBy(int vid1,int vid2,int vid3)const
{
	int eid1=GetEdgeIdBy(vid1, vid2);
	if(eid1==-1)
		return -1;

	int eid2=GetEdgeIdBy(vid1,vid3);
	if(eid2==-1)
		return -1;

	int fid=GetFaceIdxBy(eid1,eid2);

	return fid;
}

//---------------------------------------------------------------
// Name:		GetEdgeIdBy
// Function:	get the edge index by two vertex index
// Argument:	vid1,vid2: the two vertex indexes for find the edge
// Return:		if there exist an edge connect to the two vertex, return the edge index, else -1.
// Author:		unknown
// Date:		unknown
// Modified by:		XXX(add comments)
// Updated date:	4/26/2006			
//---------------------------------------------------------------- 

int XBaseMesh::GetEdgeIdBy(int vid1, int vid2)const
{
	int i, j;

	const XVert& v1=GetV(vid1);
	const XVert& v2=GetV(vid2);

	for(i=0;i<v1.GetAdjESize();i++)
	{
		for(j=0;j<v2.GetAdjESize();j++)
		{
			if(v1.AdjE(i)==v2.AdjE(j))
				return v1.AdjE(i);
		}
	}
	return -1;
}

//----------------------------------------------------------------------
// Name:		GetBoundEidOnVert(...)
// Function:    Get two boundary edges connecting to a vertex
// Argument:	int vid - the index of the vertex
//				int& resEid1 - the first boundary edge connnecting to the vertex
//				int& resEid2 - the second boundary edge connnecting to the vertex
// Return:		true - boundary edges exist; false - boundar edges do not exist
// Author:		
// Date:		
// Modified by: D. XXX
// Update date:	April. 20, 2006	
//----------------------------------------------------------------------
bool XBaseMesh::GetBoundEidOnVert(int vid, int& resEid1, int& resEid2)
{
	resEid1=resEid2=-1;

	// Check whether the flag of boundary elements have been set. If not, set the flag
	if(!HasBoundaryFlagSet())
	{
		BoundaryFlagSetting();
	}

	const XVert& v=GetV(vid);
	if(!v.IsOnBoundary())
	{
		return false;
	}

	for(int i=0;i<v.GetAdjESize();i++)
	{
		const XEdge& e=GetE(v.AdjE(i));
		if(e.GetFIdx(1)!=-1)
			continue;

		if(resEid1==-1)
		{
			resEid1=v.AdjE(i);
		}
		else
		{
			resEid2=v.AdjE(i);
			return true;
		}

	}

	return false;
}

int XBaseMesh::GetLoopNum(varray<int>& vids, int strPntIdx, int strEdgeIdx, int strFaceIdx)
{
	vids.clear();

	int i;
	int eSize=GetESize();
	int vSize=GetVSize();
	int loopCount=0;

	varray<bool> visitedEFlag;
	varray<bool> visitedVFlag;
	visitedEFlag.resize(eSize, true);
	visitedVFlag.resize(vSize, true);

	for(i=strEdgeIdx;i<eSize;i++)
	{
		const XEdge& e=GetE(i);
		if(e.GetFIdx(1)==-1)
		{
			visitedEFlag[i]=false;
			visitedVFlag[e.p(0)]=false;
			visitedVFlag[e.p(1)]=false;
		}
	}

	while(true)
	{
		int baseVid=-1;
		for(i=strPntIdx;i<vSize;i++) // get the seed vertex index
		{
			if(!visitedVFlag[i])
			{
				baseVid=i;
				break;
			}
		}
		if(baseVid==-1)
		{	
			break;		
		}
		//XVert baseV;
		//XEdge baseE;


		while(true)// find all the vertexes which at the same loop with the seed vertex.
		{
			if(baseVid==-1)
			{
				break;
			}
			visitedVFlag[baseVid]=true;
			const XVert& baseV=GetV(baseVid);			

			vids.push_back(baseVid);

			//			int bSize=0;
			int otherVid=-1;
			for(i=0;i<baseV.GetAdjESize();i++)
			{
				if(!visitedEFlag[baseV.AdjE(i)])
				{
					const XEdge& baseE=GetE(baseV.AdjE(i));
					if(!visitedVFlag[baseE.p(0)])
					{
						otherVid=baseE.p(0);						
					}
					else if(!visitedVFlag[baseE.p(1)])
					{
						otherVid=baseE.p(1);
					}					
				}
			}
			if(otherVid==-1)
			{
				vids.push_back(-1);
				loopCount++;
				break;
			}
			else
			{
				baseVid=otherVid;
			}
		}
	}
	return loopCount;
}

//---------------------------------------------------------------
// Name:		GetEidByVertexAndFace
// Function:	get the edge ids which on the face and linked the vertex
// Argument:	vid: the vertex id which link the edge
//				fid: the face id for searching
// Return:		eids: the edge ids linked to the vertex in the face
// Author:		XXX
// Date:		12/15/2006
// Modified by:		
// Updated date:	
//---------------------------------------------------------------- 
void XBaseMesh::GetEidByVertexAndFace(int vid, int fid, varray<int>& eids)const
{
	assert(vid < GetVSize()&& vid >= 0);
	assert(fid < GetFSize()&& fid >= 0);
	eids.resize(0);
	const varray<int>& adjeid = GetV(vid).AdjE();
	const XFace& f = GetF(fid);
	if(IsInTheVarray(f.GetEdgeRef(0),adjeid))
		eids.push_back(f.GetEdgeRef(0));
	if(IsInTheVarray(f.GetEdgeRef(1),adjeid))
		eids.push_back(f.GetEdgeRef(1));
	if(IsInTheVarray(f.GetEdgeRef(2),adjeid))
		eids.push_back(f.GetEdgeRef(2));
}
//---------------------------------------------------------------
// Name:		GetEidOnwise
// Function:	get the ordered intersect edges and faces when boundary line create.
// Argument:	vid: the vertex id on the boundary
//				startEid: the started edge id to judge the direction.
// Return:		eids: the intersected edges IDs in order at the given vertex.
//				fids: the intersected faces IDs in order at the given vertex.
// Author:		ljt
// Date:		7/14/2004
// Modified by:		XXX(add comments)
// Updated date:	4/26/2006		
//---------------------------------------------------------------- 
bool XBaseMesh::GetEidOnwise(int vid, int startEid, varray<int>& eids,varray<int>& fids) //2004-07-14-Li
{
	eids.clear();
	fids.clear();

	const XVert& v=GetV(vid);
	bool flag=false;
	varray<bool> visitFlag;
	visitFlag.resize(v.GetAdjESize(), false);

	int eid=-1;
	for(int i=0; i<v.GetAdjESize();i++)
	{
		eid=v.AdjE(i);
		if(eid!=startEid)
			continue;
		flag=true;
		visitFlag[i]=true;
		break;
	}

	if(!flag)
		return false;

	int curEid=eid;
	//	int nextEid=-1;
	eids.push_back(curEid);
	while(true)// get all the edges on the vertex
	{		
		flag=false;
		for(int i=0;i<v.GetAdjESize();i++)
		{
			if(visitFlag[i])
				continue;
			eid=v.AdjE(i);
			if(GetFaceIdxBy(curEid,eid)==-1)
				continue;			
			visitFlag[i]=true;
			eids.push_back(eid);
			curEid=eid;
			flag=true;
		}
		if(!flag)
			break;
	}
	if(eids.size()<=1)
		return false;
	for(int i = 0; i < static_cast<int>(eids.size()-1); i++)
	{
		fids.push_back(GetFaceIdxBy(eids.at(i),eids.at(i+1)));
	}
	int temp=fids.back();
	fids.push_back(temp);
	return true;
}

//---------------------------------------------------------------
// Name:		Get2DAngleInFace
// Function:	get 2d angle of the face
// Argument:	fid: face index
//				pid: point index
// Return:		
// Author:		unknown
// Date:		unknown
// Modified by:		XXX(add comments)
// Updated date:	4/28/2006	
//---------------------------------------------------------------- 
float XBaseMesh::Get2DAngleInFace(int fid,int pid)
{
	int i;
	XFace& f=GetF(fid);
	int selectedNO=-1;
	for( i=0;i<3;i++)
	{
		if(static_cast<int>(f.p(i))==pid)
		{
			selectedNO=i;
			break;
		}
	}

	if(selectedNO != -1)
		return(f.Angle(selectedNO));
	else 
		return 0;
}

//---------------------------------------------------------------
// Name:		Get3DAngleInFace
// Function:	get 3d angle of the face
// Argument:	fid: face index
//				pid: point index
// Return:		
// Author:		unknown
// Date:		unknown
// Modified by:		XXX(add comments)
// Updated date:	4/28/2006	
//---------------------------------------------------------------- 
float XBaseMesh::Get3DAngleInFace(int fid,int pid)
{
	int i;
	XFace& f=GetF(fid);
	int selectedNO=-1;
	for( i=0;i<3;i++)
	{
		if(static_cast<int>(f.p(i))==pid)
		{
			selectedNO=i;
			break;
		}
	}

	if(selectedNO != -1)
		return (f.GetOriAngle(selectedNO));
	else
		return 0;
}

//---------------------------------------------------------------
// Name:		Get3DTotalAngleAtPoint
// Function:	get total angle on the vertex in 3d
// Argument:	vid: the vertex index
// Return:		the total anngle on the vertex in 3d
// Author:		unknown
// Date:		unknown
// Modified by:		XXX(add comments)
// Updated date:	4/27/2006	
//---------------------------------------------------------------- 

float XBaseMesh::Get3DTotalAngleAtPoint(int vid)
{
	int i;
	if(!IsAngleInited())
	{
		AngleInit();
	}
	float totalAngle=0.0;
	const XVert& v=GetV(vid);
	for(i=0;i<v.GetAdjFSize();i++)
	{
		totalAngle+=Get3DAngleInFace(v.AdjF(i),vid);
	}
	return totalAngle;
}

//---------------------------------------------------------------
// Name:		Get2DArea
// Function:	get the mesh area in 3d
// Argument:	
// Return:		
// Author:		unknown
// Date:		unknown
// Modified by:		XXX(add comments)
// Updated date:	4/28/2006	
//---------------------------------------------------------------- 
float XBaseMesh::Get2DArea()  
{
	float result = 0.0f;
	for(int i=0;i<GetFSize();i++)
	{
		result += Get2DFaceArea(i);
	}

	return result;
}

//---------------------------------------------------------------
// Name:		Get3DArea
// Function:	get the mesh area in 3d
// Argument:	
// Return:		
// Author:		unknown
// Date:		unknown
// Modified by:		XXX(add comments)
// Updated date:	4/28/2006	
//---------------------------------------------------------------- 

float XBaseMesh::Get3DArea()  
{
	float result = 0.0f;
	for(int i=0;i<GetFSize();i++)
	{
		result += Get3DFaceArea(i);
	}

	return result;
}

//---------------------------------------------------------------
// Name:		Get2DFaceArea
// Function:	get the area of 3d face
// Argument:	fid: the face index
// Return:		the area of the face
// Author:		unknown
// Date:		unknown
// Modified by:		XXX(add comments)
// Updated date:	4/28/2006	
//---------------------------------------------------------------- 
float XBaseMesh::Get2DFaceArea(int fid) 
{
	const XFace& f = GetF(fid);

	// commented by XXX [8/14/2006] too slow when use computer.
	//float a = (GetV(f.p(0)).Pos2d() - GetV(f.p(1)).Pos2d() ).Magnitude();
	//float b = (GetV(f.p(1)).Pos2d() - GetV(f.p(2)).Pos2d() ).Magnitude();
	//float c = (GetV(f.p(2)).Pos2d() - GetV(f.p(0)).Pos2d() ).Magnitude();
	const Vec4& p0 = GetV(f.p(0)).Pos2d();
	const Vec4& p1 = GetV(f.p(1)).Pos2d();
	const Vec4& p2 = GetV(f.p(2)).Pos2d();

	return 0.25f * (CrossVecX(p1 - p0,p2 - p0).Magnitude() + CrossVecX(p1 - p2,p0 - p2).Magnitude());

	//float p = (a + b + c ) * 0.5f;

	//return sqrt(fabs(p * (p - a ) * (p - b ) * (p - c )));
}

//---------------------------------------------------------------
// Name:		Get3DFaceArea
// Function:	get the area of 3d face
// Argument:	fid: the face index
// Return:		the area of the face
// Author:		unknown
// Date:		unknown
// Modified by:		XXX(add comments)
// Updated date:	4/28/2006	
//---------------------------------------------------------------- 

float XBaseMesh::Get3DFaceArea(int fid) 
{
	const XFace& f = GetF(fid);

	const Vec4& p0 = GetV(f.p(0)).Pos();
	const Vec4& p1 = GetV(f.p(1)).Pos();
	const Vec4& p2 = GetV(f.p(2)).Pos();

	return 0.25f * (CrossVecX(p1 - p0,p2 - p0).Magnitude() + CrossVecX(p1 - p2,p0 - p2).Magnitude());
	// commented by XXX [8/14/2006] too slow when use computer.
	//float a = (GetV(f.p(0)).Pos() - GetV(f.p(1)).Pos() ).Magnitude();
	//float b = (GetV(f.p(1)).Pos() - GetV(f.p(2)).Pos() ).Magnitude();
	//float c = (GetV(f.p(2)).Pos() - GetV(f.p(0)).Pos() ).Magnitude();

	//float p = (a + b + c ) * 0.5f;
	//return sqrt(fabs(p * (p - a ) * (p - b ) * (p - c )));
}

//---------------------------------------------------------------
// Name:		IsEdgeOnFace
// Function:	to test if the edge belong to the face
// Argument:	eid: edge index
//				fid: the face index
// Return:		if the edge belong to the face, return true, else return false.
// Author:		unknown
// Date:		unknown
// Modified by:		XXX(add comments)
// Updated date:	4/27/2006	
//---------------------------------------------------------------- 
bool XBaseMesh::IsEdgeOnFace(int eid, int fid)
{
	if(fid==-1 || eid==-1)
		return false;

	const XFace& f=GetF(fid);
	for(int i=0; i<3;i++)
	{
		if(eid==static_cast<int>(f.GetEdgeRef(i)))
			return true;
	}
	return false;
}

//----------------------------------------------------------------------
// Name:		GetTotalAngleOnVert()
// Function:	Compute the total angles around a point p; it is the sum of angles of adjacent faces of p 
// Argument:	vid - index of the point
// Return:		Total angle around the point
// Author:		
// Date:		
// Modified by: D. XXX
// Updated date: Apr. 20, 2006
//----------------------------------------------------------------------
float XBaseMesh::GetTotalAngleOnVert(int vid)
{
	if(vid==-1)
		return 0.0f;

	if(!m_AngleInited)
	{
		AngleInit();
	}

	int adjFSize=GetV(vid).GetAdjFSize();
	float Angle=0.0f;

	for(int j=0;j<adjFSize;j++)
	{			
		Angle += GetOriAngleOnVert(GetV(vid).AdjF(j),vid);						
	}

	return Angle;
}

//---------------------------------------------------------------
// Name:		GetRect
// Function:	get the mesh 2d bounding box
// Argument:	
// Return:		Vec4(minX,maxY, maxX,minY);
// Author:		XXX
// Date:		3/25/2003
// Modified by:		XXX(add comments)
// Updated date:	4/28/2006	
//---------------------------------------------------------------- 

//XXX 2003-03-25
Vec4 XBaseMesh::GetRect()
{
	//if(m_bRectExisted)
	//{
	//	return m_meshRect;
	//}
	int size=GetVSize(),i;
	float minX,maxX, minY,maxY;

	minX=maxX=GetV(0).Pos2d().x;
	minY=maxY=GetV(0).Pos2d().y;

	for(i=0;i<size;i++)
	{
		// commented by XXX - 2003-11-06
		//		if(!GetV(i).IsOnBoundary())
		//			continue;
		if(minX>GetV(i).Pos2d().x)
		{
			minX=GetV(i).Pos2d().x;
		}
		if(maxX<GetV(i).Pos2d().x)
		{
			maxX=GetV(i).Pos2d().x;
		}

		if(minY>GetV(i).Pos2d().y)
		{
			minY=GetV(i).Pos2d().y;
		}
		if(maxY<GetV(i).Pos2d().y)
		{
			maxY=GetV(i).Pos2d().y;
		}
	}
	m_bRectExisted=true;
	m_meshRect=Vec4(minX,maxY, maxX,minY);
	return(m_meshRect);
}
void XBaseMesh::SetMainOrientationforModel()
{
    Vec4 minPt,maxPt;

	GetRect3D(maxPt,minPt);
	float xl = maxPt.x - minPt.x;
	float yl = maxPt.y - minPt.y;
	float zl = maxPt.z - minPt.z;
	if(xl >= yl && xl >= zl)
		m_iModelMainOrientation = 0;
	else if(yl >= xl && yl >= zl)
		m_iModelMainOrientation = 1;
	else if(zl >= xl && zl >= yl)
		m_iModelMainOrientation = 2;
}
//---------------------------------------------------------------
// Name:		GetRect3D
// Function:	get the mesh 3d bounding box
// Argument:	
// Return:		xyzMax: the max x, y and z;
//				xyzMin: the min x, y and z;
// Author:		XXX
// Date:		3/25/2003
// Modified by:		XXX(add comments)
// Updated date:	4/28/2006	
//---------------------------------------------------------------- 
void XBaseMesh::GetRect3D(Vec4& xyzMax, Vec4& xyzMin)
{
	// here need some flag and variables to store the status.
	if(m_vert.size() > 0)
	{
		xyzMax.x = m_vert.front().Pos().x;
		xyzMax.y = m_vert.front().Pos().y;
		xyzMax.z = m_vert.front().Pos().z;
		xyzMin.x = m_vert.front().Pos().x;
		xyzMin.y = m_vert.front().Pos().y;
		xyzMin.z = m_vert.front().Pos().z;
		for(int i = 1; i < GetVSize(); ++i)
		{
			const XVert& v = m_vert.at(i);
			if(v.Pos().x > xyzMax.x)
				xyzMax.x = v.Pos().x;
			if(v.Pos().y > xyzMax.y)
				xyzMax.y = v.Pos().y;
			if(v.Pos().z > xyzMax.z)
				xyzMax.z = v.Pos().z;

			if(v.Pos().x < xyzMin.x)
				xyzMin.x = v.Pos().x;
			if(v.Pos().y < xyzMin.y)
				xyzMin.y = v.Pos().y;
			if(v.Pos().z < xyzMin.z)
				xyzMin.z = v.Pos().z;
		}
	}
}
//---------------------------------------------------------------
// Name:		GetELength
// Function:	get the edge length
// Argument:	i: the edge index
// Return:		the length of the edge.
// Author:		unknown
// Date:		unknown
// Modified by:		XXX(add comments)
// Updated date:	4/28/2006	
//---------------------------------------------------------------- 
float XBaseMesh::GetELength(int i)
{
	const XEdge& edge = GetE(i);

	return (GetV(edge.p(0)).Pos2d() - GetV(edge.p(1)).Pos2d()).Magnitude();
}

//---------------------------------------------------------------
// Name:		GetMeshCenter
// Function:	get the mesh center position in 2d
// Argument:	
// Return:		the mesh center.
// Author:		unknown
// Date:		unknown
// Modified by:		XXX(add comments)
// Updated date:	4/28/2006	
//---------------------------------------------------------------- 

Vec4 XBaseMesh::GetMeshCenter()
{
	if(m_bCentreExisted)
	{
		return m_meshCentre;
	}

	if(GetVSize() == 1)
	{
		return GetV(0).Pos2d();
	}

	Vec4 allctrlpts = GetV(0).Pos2d();
	int ctrlptcount = 1;
	int i,n = GetVSize();

	for(i=1;i<n;i++)
	{
		allctrlpts += GetV(i).Pos2d();
		ctrlptcount++;
	}

	Vec4 ret = allctrlpts / static_cast<float>(ctrlptcount);
	m_bCentreExisted=true;
	m_meshCentre=ret;
	return ret;
}

//---------------------------------------------------------------
// Name:		GetMeshCenter3D
// Function:	get the mesh center position in 3d
// Argument:	
// Return:		the mesh center.
// Author:		unknown
// Date:		unknown
// Modified by:		XXX(add comments)
// Updated date:	4/28/2006	
//---------------------------------------------------------------- 
Vec4 XBaseMesh::GetMeshCenter3D()
{
	// here need some flag and variables to store the status.

	if(GetVSize() == 0)
	{
		return Vec4(0,0,0);
	}
	if(GetVSize() == 1)
	{
		return GetV(0).Pos();
	}

	Vec4 allctrlpts = GetV(0).Pos();
	int ctrlptcount = 1;
	int i,n = GetVSize();

	for(i=1;i<n;i++)
	{
		allctrlpts += GetV(i).Pos();
		ctrlptcount++;
	}

	allctrlpts = allctrlpts / static_cast<float>(ctrlptcount);
	return allctrlpts;
}

//---------------------------------------------------------------
// Name:		GetMeshBoxCenter3D
// Function:	get the mesh center position in 2d
// Argument:	
// Return:		the mesh center.
// Author:		unknown
// Date:		unknown
// Modified by:		XXX(add comments)
// Updated date:	4/28/2006	
//---------------------------------------------------------------- 
Vec4 XBaseMesh::GetMeshBoxCenter3D()
{
	// here need some flag and variables to store the status.
	//if(!m_bMeshBox3DComput)
	//{
		ComputeMeshBox3D();
	//}

	return (m_maxMeshBox3D + m_minMeshBox3D)*0.5f;;
}
//---------------------------------------------------------------
// Name:		IntersectionBetweenLineAndFaceIn3D
// Function:	get the intersect point of line and face in 3d
//				two face should have a shared edge.
// Argument:	sV,eV: the start and end point of the line
//				sFid,eFid: the related start and end face indexes in 3d
// Return:		intersection: the intersection point in 3d
//				intersectingEid: the intersected edge index
// Author:		unknown
// Date:		unknown
// Modified by:		XXX(add comments)
// Updated date:	4/28/2006	
//---------------------------------------------------------------- 
bool XBaseMesh::IntersectionBetweenLineAndFaceIn3D(const Vec4& sV, const Vec4& eV, int sFid, int eFid, Vec4& intersection, int& insectingEid)
{
	int eid=GetSharedEdge(sFid, eFid);
	if(eid<0)
	{
		return false;
	}
	insectingEid=eid;
	const XEdge& e=GetE(eid);
	Vec4 norm=(GetF(sFid).Norm()+GetF(eFid).Norm())/2;
	Vec4 dir=eV-sV;
	norm=CrossVecX(norm,dir);
	//intersection=GetLineFaceIntersectPnt(GetV(e.p(0)).Pos(),GetV(e.p(1)).Pos(),sV,norm);
	return true;
} 

//---------------------------------------------------------------
// Name:		IntersectionBetweenLineAndFaceIn2D
// Function:	get the intersect point of line and face in 2d
//				two face should have a shared edge.
// Argument:	sV,eV: the start and end point of the line
//				sFid,eFid: the related start and end face indexes in 2d
// Return:		intersection: the intersection point in 2d
//				intersectingEid: the intersected edge index
// Author:		unknown
// Date:		unknown
// Modified by:		XXX(add comments)
// Updated date:	4/28/2006	
//---------------------------------------------------------------- 
bool XBaseMesh::IntersectionBetweenLineAndFaceIn2D(const Vec4& sV, const Vec4& eV, int sFid, int eFid, Vec4& intersection, int& intersectingEid)
{

	int eid=GetSharedEdge(sFid, eFid);
	if(eid<0)
	{
		return false;
	}
	intersectingEid=eid;
	const XEdge& e=GetE(eid);

	float r,s;

	if(GetTwoLineIntersectRatio(sV,eV,GetV(e.p(0)).Pos2d(),GetV(e.p(1)).Pos2d(),r,s))
	{
		intersection= (sV + r * (eV - sV) );
	}
	else
	{
		false;
	}

	return true;
}

////---------------------------------------------------------------
//// Name:		MapPointOntoTriangleIn2D
//// Function:	get the mapping point on the triangle and areal coordinates
//// Argument:	P,TID: the point and related face index
//// Return:		u,v,w: the areal coordinates in the triangle.
//// Author:		unknown
//// Date:		unknown
//// Modified by:		XXX(add comments)
//// Updated date:	4/28/2006	
////---------------------------------------------------------------- 
//
//void XBaseMesh::MapPointOntoTriangleIn2D(const Vec4& P, int TID, float& u, float& v, float& w)
//{
//	const XFace& f=GetF(TID);
//	Vec4 v0=GetV(f.p(0)).Pos2d();
//	Vec4 v1=GetV(f.p(1)).Pos2d();
//	Vec4 v2=GetV(f.p(2)).Pos2d();
//
//	P = GetBarycentCoorInATriangle(P,v0,v1,v2);
//	u = P.x;
//	v = P.y;
//	w = P.z;
//	//u=CrossVecX((P-v1),(v2-v1)).Magnitude()/CrossVecX((v0-v1),(v2-v1)).Magnitude();
//	//if(IsOnDiffSide(P,v1,v2,v0))
//	//{
//	//	u=-u;
//	//}
//
//	//v=CrossVecX((P-v0),(v2-v0)).Magnitude()/CrossVecX((v1-v0),(v2-v0)).Magnitude();	
//	//if(IsOnDiffSide(P,v0,v2,v1))
//	//{
//	//	v=-v;
//	//}
//
//	//w=CrossVecX((P-v0),(v1-v0)).Magnitude()/CrossVecX((v2-v0),(v1-v0)).Magnitude();
//	//if(IsOnDiffSide(P,v0,v1,v2))
//	//{
//	//	w=-w;
//	//}
//}

////---------------------------------------------------------------
//// Name:		MapPointOntoTriangleIn3D
//// Function:	get the point on the triangle and areal coordinates in 3d
//// Argument:	P,TID: the point and the face id
////				u,v,w: the areal coordinates of the point in triangle.
//// Return:		
//// Author:		unknown
//// Date:		unknown
//// Modified by:		XXX(add comments)
//// Updated date:	4/28/2006	
////---------------------------------------------------------------- 
//void XBaseMesh::MapPointOntoTriangleIn3D(const Vec4& P, int TID, float& u, float& v, float& w)
//{
//	XFace f=GetF(TID);
//	Vec4 v0=GetV(f.p(0)).Pos();
//	Vec4 v1=GetV(f.p(1)).Pos();
//	Vec4 v2=GetV(f.p(2)).Pos();
//
//	P = GetBarycentCoorInATriangle(P,v0,v1,v2);
//	u = P.x;
//	v = P.y;
//	w = P.z;
//	//Vec4 nor=CrossVecX(v1-v0, v2-v0).Normalize();
//	//Vec4 v4=P-v0;
//	//float len=Dot(v4,nor); //V4.Magnitude();
//	//Vec4 v5=-len*nor;
//	//Vec4 projection=v0+(v4+v5);
//
//
//	//float d1=GetDistOfPntToLn(projection, v1, v2);
//	//float d2=GetDistOfPntToLn(v0, v1, v2);
//	//u=d1/d2;
//	//if(IsOnDiffSide(projection,v1,v2,v0))
//	//{
//	//	u=-u;
//	//}
//
//
//	//float d3=GetDistOfPntToLn(projection, v0, v2);
//	//float d4=GetDistOfPntToLn(v1, v0, v2);
//	//v=d3/d4;
//	//if(IsOnDiffSide(projection,v0,v2,v1))
//	//{
//	//	v=-v;
//	//}
//
//
//	//float d5=GetDistOfPntToLn(projection, v0, v1);
//	//float d6=GetDistOfPntToLn(v2, v0, v1);
//	//w=d5/d6;
//	//if(IsOnDiffSide(projection,v0,v1,v2))
//	//{
//	//	w=-w;
//	//}
//
//}

//---------------------------------------------------------------
// Name:		IsPointInTriangleIn2D()
// Description:	Check whether a point is inside a 2D triangle
// Argument:	testingP: point to be checked
//				fid: face ID
// Return:		true: inside; false: outside
// Author:		ljt
// Date:		9/8/2005
// Modified by:	D. XXX	
// Updated date: Apr. 25, 2006
//---------------------------------------------------------------
bool XBaseMesh::IsPointInTriangleIn2D(const Vec4& testingP, int fid) 
{
	const XFace& f=GetF(fid);

	return IsPointInTriangle(testingP,GetV(f.p(0)).Pos2d(), GetV(f.p(1)).Pos2d(), GetV(f.p(2)).Pos2d()) ;
}

//---------------------------------------------------------------
// Name:		IsAdjacent
// Function:	to test if the two face adjacent
// Argument:	iFace1,iFace2: the two face indexes.
// Return:		
// Author:		unknown
// Date:		unknown
// Modified by:		XXX(add comments)
// Updated date:	4/28/2006	
//---------------------------------------------------------------- 
bool XBaseMesh::IsAdjacent(int iFace1,int iFace2)
{
	XFace &face1=GetF(iFace1);
	for(int i=0;i<3;i++)
	{
		if(static_cast<int>(face1.GetAdjacent(i))==iFace2)
		{
			return true;
		}
	}
	return false;
}

//---------------------------------------------------------------
// Name:		Compute2DTotalAngleAtPoint
// Function:	compute the angle on a vertex in 2d
// Argument:	pid: the vertex index
// Return:		the total angle on the vertex
// Author:		unknown
// Date:		unknown
// Modified by:		XXX(add comments)
// Updated date:	4/27/2006	
//---------------------------------------------------------------- 
float XBaseMesh::Compute2DTotalAngleAtPoint(int pid)
{
	int i,j;
	const XVert& vert=GetV(pid);
	Vec4 v1, v2;
	float sumAngle=0.0;

	int adjFSize=GetV(pid).GetAdjFSize();
	for(i=0;i<adjFSize;i++)
	{
		const XFace& f=GetF(vert.AdjF(i));
		for(j=0;j<3;j++)
		{
			if(static_cast<int>(f.p(j))!=pid) continue;
			v1=GetV(f.p((j+1)%3)).Pos2d()-GetV(f.p((j)%3)).Pos2d();
			v2=GetV(f.p((j+2)%3)).Pos2d()-GetV(f.p((j)%3)).Pos2d();
			sumAngle+=GetAngleOf2Vector(v1,v2);
			break;
		}
	}
	return sumAngle;
}

//---------------------------------------------------------------
// Name:		PointInWhichTriangle
// Function:	get the triangle which contain the input point
// Argument:	P: the point postion
//				mesh: the mesh for find triangles.
// Return:		
// Author:		unknown
// Date:		unknown
// Modified by:		XXX(add comments)
// Updated date:	4/28/2006	
//---------------------------------------------------------------- 

int XBaseMesh::PointInWhichTriangle(const Vec4& P, XBaseMesh* mesh) //return the id of traingle in which the testing point lies
{
	int i,j;
	int fSize=mesh->GetFSize();
	for(i=0;i<fSize;i++)
	{
		if(IsPointInTriangle(P, mesh->GetV(mesh->GetF(i).p(0)).Pos2d(), mesh->GetV(mesh->GetF(i).p(1)).Pos2d(), mesh->GetV(mesh->GetF(i).p(2)).Pos2d()))
		{
			return i;
		}
	}
	varray<int> fidArr;


	for(i=0;i<fSize;i++)
	{
		if(IsPointInTheBoxOfTriangle(P, mesh->GetV(mesh->GetF(i).p(0)).Pos2d(), mesh->GetV(mesh->GetF(i).p(1)).Pos2d(), mesh->GetV(mesh->GetF(i).p(2)).Pos2d()))
		{
			fidArr.push_back(i);
		}
	}
	if(fidArr.size()==0)
	{
		for(i=0;i<mesh->GetFSize();i++)
		{
			fidArr.push_back(i);
		}
	}
	//XFace f;
	//XEdge e;
	int selectedFid=fidArr.at(0);
	float minLen=10000.0;

	for(i=0;i<static_cast<int>(fidArr.size());i++)
	{
		const XFace& f=mesh->GetF(fidArr.at(i));
		for(j=0;j<3;j++)
		{
			const XEdge& e=mesh->GetE(f.GetEdgeRef(j));
			float temp=CrossVecX((P-mesh->GetV(e.p(0)).Pos2d()),(mesh->GetV(e.p(1)).Pos2d()-mesh->GetV(e.p(0)).Pos2d()).Normalize()).Magnitude();
			if(temp<minLen)
			{
				selectedFid=fidArr.at(i);
				minLen=temp;
			}
		}
	}


	return selectedFid;
}

//---------------------------------------------------------------
// Name:		GetFaceAnotherVert
// Function:	get face third vertex by two vertexes
// Argument:	onevert,twovert: the two vertex indexes
// Return:		the third vertex index or -1
// Author:		unknown
// Date:		unknown
// Modified by:		XXX(add comments)
// Updated date:	4/28/2006	
//---------------------------------------------------------------- 

int XBaseMesh::GetFaceAnotherVert(int onevert, int twovert,bool boundaryflag)
{
	//XFace face;
	int i,j;

	for(i=0;i<GetFSize();i++)
	{
		const XFace& face = GetF(i);

		if(!boundaryflag)
		{
			if(!face.IsBoundary() )
			{
				continue;
			}
			for(j=0;j<3;j++)
			{
				if(static_cast<int>(face.p(j)) == onevert && static_cast<int>(face.p((j+1)%3)) == twovert)
				{
					return face.p((j+2)%3);
				}
				else if(static_cast<int>(face.p(j)) == twovert && static_cast<int>(face.p((j+1)%3)) == onevert)
				{
					return face.p((j+2)%3);
				}
			}
		}
		else
		{
			for(j=0;j<3;j++)
			{
				if(static_cast<int>(face.p(j)) == onevert && static_cast<int>(face.p((j+1)%3)) == twovert)
				{
					return face.p((j+2)%3);
				}				
			}
		}		
	}
	return -1;
}

//---------------------------------------------------------------
// Name:		GetOppEid
// Function:	Get the edge that does not connect with the specified vertex
// Argument:	fid: the face index
//				vid: the vertex index
// Return:		the edge index not connect the vertex
// Author:		unknown
// Date:		unknown
// Modified by:		XXX(add comments)
// Updated date:	4/28/2006	
//---------------------------------------------------------------- 
int XBaseMesh::GetOppEid(int fid, int vid)
{
	const XFace& f=GetF(fid);
	for(int i=0;i<3;i++)
	{
		if(GetE(f.GetEdgeRef(i)).p(0)!=vid && GetE(f.GetEdgeRef(i)).p(1)!=vid)
		{
			return f.GetEdgeRef(i);
		}
	}
	return -1;
}

//---------------------------------------------------------------
// Name:		GetAdjacentBoundaryEdge
// Function:	get the adjacent boundary edge
// Argument:	vertindex: the vertex for search
//				edgeindex: the edge index should NOT be returned.
// Return:		the boundary edge index 
// Author:		unknown
// Date:		unknown
// Modified by:		XXX(add comments)
// Updated date:	4/28/2006	
//---------------------------------------------------------------- 
int XBaseMesh::GetAdjacentBoundaryEdge(int vertindex,int edgeindex)
{
	const XVert& vert = GetV(vertindex);
	int i;

	for(i=0;i<vert.GetAdjESize();i++)
	{
		const XEdge& e= GetE(vert.AdjE(i));
		if( vert.AdjE(i) != edgeindex && e.IsBoundary() == true)
		{
			return vert.AdjE(i);
		}
	}

	return -1;
}

//---------------------------------------------------------------
// Name:		Save0010
// Function:	save the xmesh data for pattern module(ver1.0) need refinement later.........!!!
// Argument:	
// Return:		
// Author:		unknown
// Date:		unknown
// Modified by:		XXX(add comments)
// Updated date:	6/14/2006	
//---------------------------------------------------------------- 

bool  XBaseMesh::Save0010_for_oriMesh(BaseOStream& os) const
{
	//DWORD csize;

	int tempIntForVersions = 0;
	bool tempBoolForVersions = false;
	float smoothangle = GetSmoothAngle();
	os <<endl<<"PROP "<< smoothangle<<endl;
	os << "VERT "<< GetVSize()<<endl;
	smoothangle = 0.0;
	for(int i = 0; i < GetVSize(); i++)
	{
		os<<GetV(i).Pos().x<<" "<<GetV(i).Pos().y<<" "<<GetV(i).Pos().z<<endl;
	}
	os << "VCOL "<<GetVSize()<<endl;
	for(int i = 0; i < GetVSize(); i++)
	{
		os<<GetV(i).Color()[0] <<" " << GetV(i).Color()[1] << " "<<GetV(i).Color()[2]<<" "<<GetV(i).Color()[3]<<endl;
	}
	os << "NORM "<<GetVSize()<<endl;
	for(int i = 0; i < GetVSize(); i++)
	{
		os<<GetV(i).Norm().x<<" "<<GetV(i).Norm().y<<" "<<GetV(i).Norm().z<<endl;
	}

	os << "CORD "<<GetVSize()<<endl;
	for(int i = 0; i < GetVSize(); i++)
	{
		os<<GetV(i).ST().x<<" "<<GetV(i).ST().y<<" "<<smoothangle<<endl;
	}
	os << "FACE "<<GetFSize()<<endl;
	for(int i = 0; i < GetFSize(); i++)
	{
		const XFace& faceData = GetF(i); 	
		os<<faceData.GetVisible()<<endl;
		os<<faceData.m_color[0]<<" "<<faceData.m_color[1] <<" "<<faceData.m_color[2]<<" "<<faceData.m_color[3]<<" ";
//		os<<faceData.m_evis[0]<<" "<<faceData.m_evis[1]<<" "<<faceData.m_evis[2]<<endl;
		os<<tempBoolForVersions<<" "<<tempBoolForVersions<<" "<<tempBoolForVersions<<endl;
		os<<faceData.p(0)<<" "<<faceData.p(1)<<" "<<faceData.p(2)<<endl;
		os<<faceData.GetMaterialID()<<endl;
//		os<<faceData.m_nidx[0]<<" "<<faceData.m_nidx[1]<<" "<<faceData.m_nidx[2]<<endl;
		os<<tempIntForVersions<<" "<<tempIntForVersions<<" "<<tempIntForVersions<<endl;
		//for(int j = 0; j < MAX_TEXTURE_STAGE; j++)
		//{
		//	os<<faceData.m_tidx[j][0]<<" "<<faceData.m_tidx[j][1]<<" "<<faceData.m_tidx[j][2]<<endl;
		//}
		for(int j = 0; j < MAX_TEXTURE_STAGE; j++)
		{
			os<<tempIntForVersions<<" "<<tempIntForVersions<<" "<<tempIntForVersions<<endl;
		}
		os<<faceData.GetSubMeshID()<<endl;//XXX,2005.5.5
	}
	//END MESH CHUNK

	return true;
}

//---------------------------------------------------------------
// Name:		Load0010
// Function:	load mesh from file                need refinement later.........!!!
// Argument:	
// Return:		
// Author:		unknown
// Date:		unknown
// Modified by:		XXX(add comments)
// Updated date:	6/14/2006	
//---------------------------------------------------------------- 

//bool  XBaseMesh::Load0010_for_oriMesh(BaseIStream& is,float fCurVersion)
//{
//	if(fCurVersion < 0 || fCurVersion > 1.9)
//		return false;
//	_TCHAR cid[5];
//	//DWORD size;
//	int iNum;
//	bool tempBoolForVersions;
//	int tempIntForVersions;
//	clear();
//
//	//Prop
//	float smoothangle;
//	is.read(cid, 4);
//	is >> smoothangle;
//	SetSmoothAngle(smoothangle);
//
//	//Vert
//	is.read(cid, 4);
//	is >> iNum;
//	SetVSize(iNum);
//	for(int i = 0; i < iNum; i++)
//	{
//		is>>GetV(i).Pos().x>>GetV(i).Pos().y>>GetV(i).Pos().z;
//	}
//
//
//	//VCOL
//	is.read(cid, 4);
//	is >> iNum;
//	assert(iNum == GetVSize() || iNum == 0);
//	//SetVColCount(iNum);
//	for(int i = 0; i < iNum; i++)
//	{
//		is>>GetV(i).Color().a>>GetV(i).Color().b>>GetV(i).Color().g>>GetV(i).Color().r;
//	}
//
//	//NORM
//	is.read(cid, 4);
//	is >> iNum;
//	//SetNormCount(iNum);
//	assert(iNum == GetVSize() || iNum == 0);
//	for(int i = 0; i < iNum; i++)
//	{
//		is>>GetV(i).Norm().x>>GetV(i).Norm().y>>GetV(i).Norm().z;
//	}
//
//	//Texture Coord
//	is.read(cid, 4);
//	is >> iNum;
//	assert(iNum == GetVSize() || iNum == 0);
//	//SetTVertCount(iNum);
//	for(int i = 0; i < iNum; i++)
//	{
//		is>>GetV(i).ST().x>>GetV(i).ST().y>>smoothangle;
//	}
//	//Face
//	is.read(cid, 4);
//	is >> iNum;
//	SetFSize(iNum);
//	bool allinvisible = true;
//	for(int i = 0; i < iNum; i++)
//	{
//		XFace& faceData = GetF(i);
//		//	Face& faceData = GetFace(i); 
//		is>>faceData.GetVisible()>>faceData.m_color.a>>faceData.m_color.b>>faceData.m_color.g>>faceData.m_color.r;
//
//		//is>>faceData.m_evis[0]>>faceData.m_evis[1]>>faceData.m_evis[2];
//		is>>tempBoolForVersions>>tempBoolForVersions>>tempBoolForVersions;// changed by XXX
//
//		is>>faceData.p(0)>>faceData.p(1)>>faceData.p(2);
//		is>>faceData.GetMaterialID();
//
//		//is>>faceData.m_nidx[0]>>faceData.m_nidx[1]>>faceData.m_nidx[2];// changed by XXX
//		is>>tempIntForVersions>>tempIntForVersions>>tempIntForVersions;
//		for(int j = 0; j < MAX_TEXTURE_STAGE; j++)
//		{
//			//is>>faceData.m_tidx[j][0]>>faceData.m_tidx[j][1]>>faceData.m_tidx[j][2];// changed by XXX
//			is>>tempIntForVersions>>tempIntForVersions>>tempIntForVersions;
//		}
//		if(fabs(fCurVersion-1.009) < 0.0005 || (fCurVersion - 1.009) > 0.0005){
//			is>>faceData.GetSubMeshID();
//		}
//		if(allinvisible && faceData.GetVisible())
//		{
//			allinvisible = false;
//		}
//	}
//	// added by XXX [12/6/2005] for mesh visible (some old versions)
//
//	if(allinvisible || fCurVersion < 1.1005)
//	{
//		for(int i = 0; i < iNum; ++i)
//		{
//			m_face.at(i).GetVisible() = true;
//		}
//	}
//	// over
//	MakeXEdges();
//	ComputeNormals();
//
//	return true;
//}

//---------------------------------------------------------------
// Name:		Save
// Function:	save mesh data to file  need refinement later.........!!!
// Argument:	
// Return:		
// Author:		unknown
// Date:		unknown
// Modified by:		XXX(add comments)
// Updated date:	6/14/2006	
//---------------------------------------------------------------- 

//bool  XBaseMesh::Save_for_oriMesh(BaseOStream& os, Scene& scene) const
//{
//	DWORD csize;
//	Vec4 stV;
//	float smoothangle = GetSmoothAngle();
//	os << "PROP";
//	os << (DWORD)sizeof(float);
//	os << smoothangle;
//
//	csize = GetVSize() * sizeof(Vec4);
//	os << "VERT";
//	os.write((BYTE*)&csize,             sizeof(DWORD));
//	csize = sizeof(Vec4);
//	for(int i = 0; i < GetVSize(); ++i)
//	{		
//		os.write((BYTE*)&GetV(i).Pos(), csize);
//	}
//
//	csize = GetVSize() * sizeof(ColorF);
//	os << "VCOL";
//	os.write((BYTE*)&csize,             sizeof(DWORD));
//	csize = sizeof(ColorF);
//	for(int i = 0; i < GetVSize(); ++i)
//	{
//		os.write((BYTE*)&GetV(i).Color(), csize);
//	}
//
//	csize = GetVSize() * sizeof(Vec4);
//	os << "NORM";
//	os.write((BYTE*)&csize,             sizeof(DWORD));
//	csize = sizeof(Vec4);
//	for(int i = 0 ; i < GetVSize(); ++i)
//	{
//		os.write((BYTE*)&GetV(i).Norm(), csize);
//	}
//
//	csize = GetVSize() * sizeof(Vec4);
//	os << "CORD";
//	os.write((BYTE*)&csize,             sizeof(DWORD));
//	csize = sizeof(Vec4);
//	for(int i = 0; i < GetVSize(); ++i)
//	{
//		stV.x = GetV(i).ST().x;
//		stV.y = GetV(i).ST().y;
//		os.write((BYTE*)&stV, csize);
//	}
//
//	std::vector<FaceData>  fdat(GetFSize());
//	for (int j = 0; j < GetFSize(); j++) fdat[j] = *(FaceData*)&GetF(j);
//	csize = GetFSize() * sizeof(FaceData);
//	os << "FACE";
//	os.write((BYTE*)&csize,             sizeof(DWORD));
//	csize = sizeof(FaceData);
//	for(int i = 0; i < GetFSize(); ++i)
//	{
//		os.write((BYTE*)&fdat[0],           csize);
//	}
//	//END MESH CHUNK
//
//	return true;
//}

//---------------------------------------------------------------
// Name:		Load
// Function:	load mesh data from file   need refinement later.........!!!
// Argument:	
// Return:		
// Author:		unknown
// Date:		unknown
// Modified by:		XXX(add comments)
// Updated date:	6/14/2006	
//---------------------------------------------------------------- 

//bool  XBaseMesh::Load_for_oriMesh(BaseIStream& is, Scene& scene)
//{
//	_TCHAR cid[5];
//	DWORD size;
//	Vec4 stV;
//	assert(sizeof(_TCHAR) == 1);
//	clear();
//
//	//Prop
//	float smoothangle;
//	is.read(cid, 4);
//	is >> size;
//	is >> smoothangle;
//	SetSmoothAngle(smoothangle);
//
//	//Vert
//	is.read(cid, 4);
//	is >> size;
//	SetVSize(size/sizeof(Vec4));
//	size = sizeof(Vec4);
//	for(int i = 0; i < GetVSize(); ++i)
//	{
//		is.read(&GetV(i).Pos(), size);
//	}
//
//	//VCOL
//	is.read(cid, 4);
//	is >> size;
//	//SetVColCount(size/sizeof(ColorF));
//	assert(GetVSize() == static_cast<int>(size/sizeof(ColorF)));
//	if(size > 0)
//	{
//		size = sizeof(ColorF);
//		for(int i = 0; i < GetVSize(); ++i)
//		{
//			is.read(&GetV(i).Color(), size);
//		}
//	}
//
//	//NORM
//	is.read(cid, 4);
//	is >> size;
//	//SetNormCount(size/sizeof(Vec4));
//	assert(GetVSize() == static_cast<int>(size/sizeof(Vec4)));
//	if (size > 0)
//	{
//		size = sizeof(Vec4);
//		for(int i = 0; i < GetVSize(); ++i)
//		{
//			is.read(&GetV(i).Norm(), size);
//		}
//	}
//
//	//Texture Coord
//	is.read(cid, 4);
//	is >> size;
//	//SetTVertCount(size/sizeof(Vec4));
//	assert(GetVSize() == static_cast<int>(size/sizeof(Vec4)));
//	if (size > 0)
//	{
//		size = sizeof(Vec4);
//		for(int i = 0; i < GetVSize(); ++i)
//		{
//			is.read(&stV, size);
//			GetV(i).ST().x = stV.x;
//			GetV(i).ST().y = stV.y;
//		}
//	}
//
//	//Face
//	is.read(cid, 4);
//	is >> size;
//	DWORD fc = size/sizeof(FaceData);
//	std::vector<FaceData>  fdat(fc);
//	if (size > 0)
//	{
//		is.read(&fdat[0], size);
//	}
//
//	SetFSize(fc);
//	for (int i = 0; i < GetFSize(); i++) GetF(i) = *(XFace*)&fdat[i];
//	MakeXEdges();
//	//CalcFaceNormals();
//	ComputeNormals();
//
//	return true;
//}



//---------------------------------------------------------------
// Name:		AddMesh
// Function:	add the given mesh to the current mesh
// Argument:	m: the mesh data for merge
// Return:		
// Author:		unknown
// Date:		unknown
// Modified by:		XXX(add comments)
// Updated date:	6/14/2006	
//---------------------------------------------------------------- 
void XBaseMesh::AddMesh(const XBaseMesh& m) 
{
	int i;
	int vc = GetVSize();
	//int tc = GetTVertCount();
	//int cc = GetVColCount();
	//int nc = GetNormCount();
	int fc = GetFSize();

	//Resize 
	SetVSize(GetVSize() + m.GetVSize());
	//SetTVertCount(GetTVertCount() + m.GetTVertCount());
	//SetVColCount(GetVColCount() + m.GetVColCount());
	//SetNormCount(GetNormCount() + m.GetNormCount());
	SetFSize(GetFSize() + m.GetFSize());

	for (i = 0; i < m.GetVSize(); i++) 
		GetV(i+vc).Pos() = m.GetV(i).Pos();
	for (i = 0; i < m.GetVSize(); i++) // add for DS file export, 20080104
		GetV(i+vc).Pos2d() = m.GetV(i).Pos2d();
	for (i = 0; i < m.GetVSize(); i++) 
		GetV(i+vc).ST() = m.GetV(i).ST();
	for (i = 0; i < m.GetVSize(); i++) 
		GetV(i+vc).Color() = m.GetV(i).Color();
	for (i = 0; i < m.GetVSize(); i++) 
		GetV(i+vc).Norm() = m.GetV(i).Norm();

	for (i = 0; i < m.GetFSize(); i++) {
		GetF(fc + i).Norm() = m.GetF(i).Norm();

		const XFace& sf = m.GetF(i);
		XFace& nf = GetF(fc + i);
		nf = sf;
		nf.SetIndex(sf.p(0)+vc, sf.p(1)+vc, sf.p(2)+vc);
//		nf.SetTIndex(0, vc+sf.t(0, 0), vc+sf.t(0, 1), vc+sf.t(0, 2)); 
//		nf.SetNormIndex(vc+sf.n(0), vc+sf.n(1), vc+sf.n(2));
	}
	MakeXEdges();
}


//---------------------------------------------------------------
// Name:		 InitMesh(...)
// Description:  Initiate a mesh
// Argument:     IsTranslation: whether to translate the object. If it is, the center of the object will be moved to Vec4(0, 0, 0)
//				 vcent: the center of the object
//				 isFirstCall: whether the function is called first time. If it is, it is unnecessary to rebuild the topology info.
//				 bIsMayHasRect: whether it is a rectangular mesh. If it is, convert it into triangular mesh
// Return:		 void
// Author:		 Wang Mei
// Date:		 01/17/2005
// Modified by:	 D. XXX
// Updated date: 04/28/2006
//---------------------------------------------------------------- 
void XBaseMesh::InitMesh(bool IsTranslation,Vec4& vcent,bool isFirstCall,bool bIsMayHasRect)
{
	int i, j;
	int nv1, nv2, nv3;
//	int np1, np2;
//	int nf;
//	bool flag;

	m_scale = 1.0f;
	m_meshCentre.Zero();

	// to translate the object
	if(IsTranslation)
	{
		// Obtain the bounding box
		minp = Vec4(1.0e+6, 1.0e+6, 1.0e+6);
		maxp = Vec4(-1.0e+6, -1.0e+6, -1.0e+6);

		for(i=0; i<static_cast<int>(m_vert.size()); i++)
		{
			if(m_vert[i].Pos().x<minp.x) 
			{
				minp.x = m_vert[i].Pos().x;
			}
			if(m_vert[i].Pos().x>maxp.x) 
			{
				maxp.x = m_vert[i].Pos().x;
			}

			if(m_vert[i].Pos().y<minp.y)
			{
				minp.y = m_vert[i].Pos().y;
			}

			if(m_vert[i].Pos().y>maxp.y)
			{
				maxp.y = m_vert[i].Pos().y;
			}

			if(m_vert[i].Pos().z<minp.z)
			{
				minp.z = m_vert[i].Pos().z;
			}

			if(m_vert[i].Pos().z>maxp.z)
			{
				maxp.z = m_vert[i].Pos().z;
			}
		}

		Vec4 cent;

		// move the model to the center of coordinate axis
		cent = (minp+maxp)/2.0f;

		m_trans = cent;
		m_meshCentre = vcent = Vec4(cent.x,cent.y,cent.z); 

		for(i=0; i<static_cast<int>(m_vert.size()); i++)
		{
			m_vert[i].Pos() -= cent;
		}

		maxp -= cent;
		minp -= cent;

		// make the unit of the model as millimeter
		float scale = 1;
		if((maxp.y-minp.y)<0.0003f)
		{
			scale = 1000000.f;
		}
		if((maxp.y-minp.y)<0.003f)
		{
            scale = 100000.f;
		}
		else if((maxp.y-minp.y)<0.03f)
		{
			scale = 10000.f;
		}
		else if((maxp.y-minp.y)<0.3f)
		{
			scale = 1000.f;
		}
		else if((maxp.y-minp.y)<3.f)
		{
			scale = 100.f;
		}
		else if((maxp.y-minp.y)<30.f)
		{
			scale = 10.f;
		}
		else if((maxp.y-minp.y)<300.f)
		{
			scale = 1.f;
		}
		else if((maxp.y-minp.y)<3000.f)
		{
			scale = 0.1f;
		}
		else if((maxp.y-minp.y)<30000.f)
		{
			scale = 0.01f;
		}
		else if((maxp.y-minp.y)<300000.f)
		{
			scale = 0.001f;
		}
		else if((maxp.y-minp.y)<3000000.f)
		{
			scale = 0.0001f;
		}		
		scale *= 2.f;
		for(i=0; i<static_cast<int>(m_vert.size()); i++)
			m_vert[i].Pos() = m_vert[i].Pos()*scale;
		m_scale = scale;
		maxp = maxp * m_scale;
		minp = minp * m_scale;		

	}

	m_meshCentre = m_meshCentre * m_scale;
	vcent = vcent * m_scale;

	if(!isFirstCall)
	{
		return;
	}

	MakeXEdges();

	if(!bIsMayHasRect) // if there are no rectangle,return directly,XXX,2005.1.29
	{
		return;			// for following codes of converting  rectangular HMesh to triangular HMesh has a bug
		//  trangles like /_|, if not closed, the first and end trangle will be regarded as rectangular
	}

	// convert rectangular HMesh to triangular HMesh
	//XEdge ed1, ed2;
	//XVert	vtx1, vtx2;
	int ne1, ne2, ne3, k;

	int nfnew = 0;
	XFace *fadd = new XFace[m_edge.size()]; 

	for(i=0; i<static_cast<int>(m_edge.size()); i++)
	{
		ne1 = i;
		if(m_edge[i].GetFIdx(1) == -1)
		{
			nv1 = m_edge[i].p(0);
			nv2 = m_edge[i].p(1);
			for(j=0; j<m_vert[nv1].GetAdjESize(); j++)
			{
				ne2 = m_vert[nv1].AdjE(j);
				const XEdge& ed1 = m_edge[ne2];
				if(m_vert[nv1].AdjE(j)!=i && ed1.GetFIdx(1)==-1)
				{
					if(ed1.p(0)!=nv1)	nv3=ed1.p(0);
					else					nv3=ed1.p(1);

					for(k=0; k<m_vert[nv3].GetAdjESize(); k++)
					{
						ne3 = m_vert[nv3].AdjE(k);
						const XEdge& ed2 = m_edge[ne3];
						if(ed2.p(0)==nv2 || ed2.p(1)==nv2) // a new face
						{
							fadd[nfnew].p(0) = nv1;
							fadd[nfnew].p(1) = nv2;
							fadd[nfnew].p(2) = nv3;


							fadd[nfnew].SetEdgeRef(0, ne1);
							fadd[nfnew].SetEdgeRef(1, ne2);
							fadd[nfnew].SetEdgeRef(2, ne3);

							//fadd[nfnew].CalNormal();
							fadd[nfnew].Norm() = 
								CrossVecX((m_vert.at(nv2).Pos() - m_vert.at(nv1).Pos()), (m_vert.at(nv3).Pos() - m_vert.at(nv1).Pos())).Normalize();

							if(fadd[nfnew].Norm().dotmultiple(m_vert[nv1].Norm())<0)
							{
								fadd[nfnew].p(0) = nv1;
								fadd[nfnew].p(1) = nv3;
								fadd[nfnew].p(2) = nv2;


								fadd[nfnew].SetEdgeRef(0,ne1);
								fadd[nfnew].SetEdgeRef(1,ne3);
								fadd[nfnew].SetEdgeRef(2,ne2);
							}

							m_edge[ne1].GetFIdx(1) = static_cast<int>(m_face.size()) + nfnew;
							m_edge[ne2].GetFIdx(1) = static_cast<int>(m_face.size()) + nfnew;
							m_edge[ne3].GetFIdx(1) = static_cast<int>(m_face.size()) + nfnew;

							nfnew++;
						}
					}

				}
			}
		}
	}

	if(nfnew>0)
	{

		for(i = 0; i<nfnew; i++)
		{
			m_face.push_back(fadd[i]);
		}


		for(i=0; i<static_cast<int>(m_vert.size()); i++)
		{
			m_vert[i].Norm() = Vec4(0.0, 0.0, 0.0);
		}

		// calculate the normal of faces
		for(i=0; i<static_cast<int>(m_face.size()); i++)
		{

			m_face[i].Norm() = 
				CrossVecX((m_vert[m_face[i].p(1)].Pos() - m_vert[m_face[i].p(0)].Pos()),
					(m_vert[m_face[i].p(2)].Pos() - m_vert[m_face[i].p(0)].Pos())).Normalize();

			m_vert[m_face[i].p(0)].Norm() += m_face[i].Norm();
			m_vert[m_face[i].p(1)].Norm() += m_face[i].Norm();
			m_vert[m_face[i].p(2)].Norm() += m_face[i].Norm();
		}

		// calculate the normal of vertices
		for(i=0; i<static_cast<int>(m_vert.size()); i++)
		{
			m_vert[i].Norm() = m_vert[i].Norm()/(float)m_vert[i].GetAdjFSize();
		}
	}

	delete [] fadd;


	for(i=0;i<static_cast<int>(m_face.size());i++)
	{
		if (m_face[i].GetSubMeshID() < -1 || m_face[i].GetSubMeshID() > 100)
		{
			m_face[i].GetSubMeshID() = -1;
		}
	}

	return;
}



void XBaseMesh::AdjustMesh(Vec4 vcent,float fScl)
{
	int i = 0;
	for(i=0; i<static_cast<int>(m_vert.size()); i++)
	{
		m_vert[i].Pos() -= vcent;
	}
	for(i=0; i<static_cast<int>(m_vert.size()); i++)
		m_vert[i].Pos() = m_vert[i].Pos()*fScl;
}

//---------------------------------------------------------------
// Name:	    IntersectPlane()
// Description: Obtain the polygon by intersecting HMesh and a plane going through a point 
//			  : then, get convex of the polygon if flag is true 
//            : if there are several closed loops, only current loop is obtained.
//            : Method: using the connection information of HMesh to quick find the intersection polygon
// input :		numf--face id in which the start point is contained
//				pnorm--plane normal
//				IsConvex--flag if will get convex of the polygon.
// output:		plist--intersecting points
//				pnum--the number of intersecting points
//				nedge--id array of intersecting edges.
// Return:		if no closed loop is obtained, return false; else, return ture.
// Author:	    XXX 
// Date:	    Aug. 30, 01
// Update:	 
// Author:		XXX
// Date:		Sept. 07, 01
// Author:		XXX
// Date:		2006/04/30 30:4:2006   17:39
// copyright: XXX. developed by XXX
//----------------------------------------------------------------
bool XBaseMesh::IntersectPlane(Vec4 startpt, int numf, Vec4 pnorm, Vec4 *plist, int &pnum, int *nedge, float *eratio, bool IsConvex)
{
	int i, k;
	int num;
	Vec4 vt1, vt2, vt3;
	XEdge ed;
	int starte = -1, ne1 = -1, ne2 = -1;
	XFace f1;
	int nf=-1;
	float maxd, mind;
	float d1, d2, d3;

	num=0;
	f1=m_face[numf];

	int *flag; // for edges - 0: no intersection with the plane; 1: intersect with the plane; 3: has been searched

	flag = new int[m_edge.size()];

	for(i=0; i<static_cast<int>(m_edge.size()); i++)
	{
		flag[i] = 0;
	}
	for(i=0; i<static_cast<int>(m_edge.size()); i++)
	{
		vt1 = m_vert[m_edge[i].p(0)].Pos();
		vt2 = m_vert[m_edge[i].p(1)].Pos();

		d1 = pnorm.x*(startpt.x-vt1.x)+pnorm.y*(startpt.y-vt1.y)+pnorm.z*(startpt.z-vt1.z);
		d2 = pnorm.x*(startpt.x-vt2.x)+pnorm.y*(startpt.y-vt2.y)+pnorm.z*(startpt.z-vt2.z);

		if(d1==d2)	
		{
			continue;
		}

		if((d1>=0&&d2<=0)||(d1<=0&&d2>=0))
		{
			flag[i] = 1;
		}
	}

	// make the plane cross at least one edge; 
	// in some cases(such as picking), the plane maybe not intersection with the triangle
	vt1 = m_vert[m_face[numf].p(0)].Pos();
	vt2 = m_vert[m_face[numf].p(1)].Pos();
	vt3 = m_vert[m_face[numf].p(2)].Pos();

	d1 = pnorm.x*(vt1.x-startpt.x)+pnorm.y*(vt1.y-startpt.y)+pnorm.z*(vt1.z-startpt.z);
	d2 = pnorm.x*(vt2.x-startpt.x)+pnorm.y*(vt2.y-startpt.y)+pnorm.z*(vt2.z-startpt.z);
	d3 = pnorm.x*(vt3.x-startpt.x)+pnorm.y*(vt3.y-startpt.y)+pnorm.z*(vt3.z-startpt.z);

	maxd = d1;
	if(d2>maxd) 
	{
		maxd = d2;
	}
	if(d3>maxd) 
	{
		maxd = d3;
	}

	mind = d1;
	if(d2<mind)
	{
		mind = d2;
	}

	if(d3<mind)
	{
		mind = d3;
	}

	Vec4 tmp = startpt;

	// move the point a little up or down so that the plane crosses the triangle
	if(mind>0&&maxd>0) 
	{
		startpt += pnorm*(mind+0.00001f); 
	}
	else if(mind<0&&maxd<0) 
	{
		startpt += pnorm*(maxd-0.00001f);
	}

	// Get the first edge in the triangle as a start edge
	for(i=0; i<3; i++)
	{
		ne1 = f1.GetEdgeRef(i);

		vt1 = m_vert[m_edge[ne1].p(0)].Pos();
		vt2 = m_vert[m_edge[ne1].p(1)].Pos();

		d1 = pnorm.x*(startpt.x-vt1.x)+pnorm.y*(startpt.y-vt1.y)+pnorm.z*(startpt.z-vt1.z);
		d2 = pnorm.x*(startpt.x-vt2.x)+pnorm.y*(startpt.y-vt2.y)+pnorm.z*(startpt.z-vt2.z);

		if(d1==d2)
		{
			continue;
		}

		if((d1>=0&&d2<=0)||(d1<=0&&d2>=0))
		{
			starte = ne1;
			ne2 = ne1;
			if(m_edge[ne1].GetFIdx(0)==-1 || m_edge[ne1].GetFIdx(1)==-1)
			{
				continue;
			}

			// get the next face
			if(m_edge[ne1].GetFIdx(0)==numf)
			{
				nf = m_edge[ne1].GetFIdx(1);
			}
			else
			{
				nf = m_edge[ne1].GetFIdx(0);
			}

			// compute the intersecting point
			plist[num].x = (float)fabs(d1)/((float)fabs(d1)+(float)fabs(d2))*(vt2.x-vt1.x)+vt1.x;
			plist[num].y = (float)fabs(d1)/((float)fabs(d1)+(float)fabs(d2))*(vt2.y-vt1.y)+vt1.y;
			plist[num].z = (float)fabs(d1)/((float)fabs(d1)+(float)fabs(d2))*(vt2.z-vt1.z)+vt1.z;

			nedge[num] = starte;
			eratio[num] = (plist[num]-vt1).GetLength()/(vt2-vt1).GetLength();

			num++;
			break;
		}
	}


	// continue to find the next edge until the edge is the same as the start edge ( a loop is found)
	bool mark;
	float dist, dd, ang;

	do
	{
		if(nf<0)
		{
			delete [] flag;
			return false;
		}

		mark = false;
		f1=m_face[nf];
		for(i=0; i<3; i++)
		{
			ne1 = f1.GetEdgeRef(i);

			if(ne1==ne2)
			{
				continue;
			}
			if(flag[ne1] != 1)
			{
				continue;
			}
			vt1 = m_vert[m_edge[ne1].p(0)].Pos();
			vt2 = m_vert[m_edge[ne1].p(1)].Pos();

			d1 = pnorm.x*(startpt.x-vt1.x)+pnorm.y*(startpt.y-vt1.y)+pnorm.z*(startpt.z-vt1.z);
			d2 = pnorm.x*(startpt.x-vt2.x)+pnorm.y*(startpt.y-vt2.y)+pnorm.z*(startpt.z-vt2.z);

			if(d1==d2)
			{
				continue;
			}
			if((d1>=0&&d2<=0)||(d1<=0&&d2>=0))
			{
				// get the next face
				if(m_edge[ne1].GetFIdx(0)==nf)
				{
					nf = m_edge[ne1].GetFIdx(1);
				}
				else
				{
					nf = m_edge[ne1].GetFIdx(0);
				}

				ne2 = ne1;

				vt3.x = (float)fabs(d1)/((float)fabs(d1)+(float)fabs(d2))*(vt2.x-vt1.x)+vt1.x;
				vt3.y = (float)fabs(d1)/((float)fabs(d1)+(float)fabs(d2))*(vt2.y-vt1.y)+vt1.y;
				vt3.z = (float)fabs(d1)/((float)fabs(d1)+(float)fabs(d2))*(vt2.z-vt1.z)+vt1.z;

				if(num>1)
				{
					ang = GetAngleOf2Vector(vt3-plist[num-1], plist[num-1]-plist[num-2]);

					if(ang<Math::Pi/1.5)
					{
						plist[num] = vt3;

						nedge[num] = ne1;
						eratio[num] = (plist[num]-vt1).GetLength()/(vt2-vt1).GetLength();
						num++;
					}
					else
					{
						nf = -1;
					}
				}
				else
				{
					plist[num] = vt3;
					nedge[num] = ne1;
					eratio[num] = (plist[num]-vt1).GetLength()/(vt2-vt1).GetLength();
					num++;
				}

				flag[ne1] = 2;
				mark = true;

				break;
			}
		}

		// special case for the HMesh with incorrect topology
		if(!mark || nf==-1)
		{
			dist = 1.0e+6;
			for(k=0; k<static_cast<int>(m_edge.size()); k++)
			{
				if(flag[k] != 1 )
				{
					continue;
				}
				if(k==starte) 
				{
					continue;
				}

				vt1 = m_vert[m_edge[ne1].p(0)].Pos();
				vt2 = m_vert[m_edge[ne1].p(1)].Pos();

				d1 = pnorm.x*(startpt.x-vt1.x)+pnorm.y*(startpt.y-vt1.y)+pnorm.z*(startpt.z-vt1.z);
				d2 = pnorm.x*(startpt.x-vt2.x)+pnorm.y*(startpt.y-vt2.y)+pnorm.z*(startpt.z-vt2.z);

				vt3.x = (float)fabs(d1)/((float)fabs(d1)+(float)fabs(d2))*(vt2.x-vt1.x)+vt1.x;
				vt3.y = (float)fabs(d1)/((float)fabs(d1)+(float)fabs(d2))*(vt2.y-vt1.y)+vt1.y;
				vt3.z = (float)fabs(d1)/((float)fabs(d1)+(float)fabs(d2))*(vt2.z-vt1.z)+vt1.z;

				dd = (vt3-plist[num-1]).GetLength();
				mark = true;
				break;
			}

			nf = m_edge[k].GetFIdx(0);

			flag[ne2] = 2;
		}

		if(!mark || nf==-1)
			break;

	}while(ne2!=starte);

	pnum = num;

	delete [] flag;

	// -----------------  make convex curve -------------------------
	if(!IsConvex)
	{
		Vec4 *pp = new Vec4[num];
		bool *fg = new bool[num];

		for(i=0; i<num; i++)
		{
			pp[i] = plist[i]-plist[0];
		}

		// rotate the curve to X-Y plane
		pnorm.unit();
		Vec4 vct = pnorm.multiple(Vec4(0, 0, 1));
		float ang = (float)acos(pnorm.dotmultiple(Vec4(0, 0, 1)));

		for(i=0; i<pnum; i++)
		{
			pp[i] = pp[i].RotAxis(vct, ang);
		}

		float maxx, minx, maxy;
		int nmin, nmax, ntop;

		maxx = minx = pp[0].x;
		maxy = pp[0].y;
		nmin = nmax = ntop = 0;

		for(i=0; i<pnum; i++)
		{
			if(pp[i].x>maxx)
			{
				maxx = pp[i].x;
				nmax = i;
			}
			else if(pp[i].x<minx)
			{
				minx = pp[i].x;
				nmin = i;
			}

			if(pp[i].y>maxy)
			{
				maxy = pp[i].y;
				ntop = i;
			}
		}

		// seperate the loop into two parts: upper part and lower part
		Vec4 *UP = new Vec4[num];
		int *uf = new int[num];
		Vec4 *LP = new Vec4[num];
		int *lf = new int[num];

		int *elist = new int[num];
		float *eu = new float[num];

		int num1, num2;
		num1=num2=0;

		if(nmin<nmax)
		{
			if(ntop>nmin && ntop<nmax)
			{
				for(i=nmin; i<=nmax; i++)
				{ // upper part
					UP[num1] = pp[i];
					uf[num1] = i;
					num1++;
				}

				for(i=nmax; i<pnum-1; i++)
				{ // lower part
					LP[num2] = pp[i];
					lf[num2] = i;
					num2++;
				}
				for(i=0; i<=nmin; i++)
				{
					LP[num2] = pp[i];
					lf[num2] = i;
					num2++;
				}
			}
			else
			{
				for(i=nmax; i>=nmin; i--)
				{  // lower part
					LP[num2] = pp[i];
					lf[num2] = i;
					num2++;
				}

				for(i=nmin; i>0; i--)
				{ // upper part
					UP[num1] = pp[i];
					uf[num1] = i;
					num1++;
				}
				for(i=pnum-1; i>=nmax; i--)
				{ 
					UP[num1] = pp[i];
					uf[num1] = i;
					num1++;
				}
			}
		}
		else
		{ // nmin>nmax
			if(ntop>nmax && ntop<nmin)
			{
				for(i=nmin; i>=nmax; i--)
				{ // upper part
					UP[num1] = pp[i];
					uf[num1] = i;
					num1++;
				}

				for(i=nmax; i>0; i--)
				{ // lower part
					LP[num2] = pp[i];
					lf[num2] = i;
					num2++;
				}
				for(i=pnum-1; i>=nmin; i--)
				{ // lower part
					LP[num2] = pp[i];
					lf[num2] = i;
					num2++;
				}
			}
			else
			{
				for(i=nmax; i<=nmin; i++)
				{  // lower part
					LP[num2] = pp[i];
					lf[num2] = i;
					num2++;
				}

				for(i=nmin; i<pnum-1; i++)
				{ // upper part
					UP[num1] = pp[i];
					uf[num1] = i;
					num1++;
				}
				for(i=0; i<=nmax; i++)
				{ 
					UP[num1] = pp[i];
					uf[num1] = i;
					num1++;
				}
			}
		}

		// make upper part convex
		int ncur, npre;
		bool s1, s2, suc;

		for(i=0; i<num1; i++)
		{
			fg[i] = false;
		}

		npre = 0;
		ncur = 1;
		for(i=2; i<num1; i++)
		{
			suc = false;
			do
			{
				if(((UP[i].x - UP[npre].x)*(UP[ncur].y - UP[npre].y)-(UP[i].y - UP[npre].y)*(UP[ncur].x - UP[npre].x))>0)
				{
					npre = ncur;
					ncur = i;
					suc = true;
				}
				else
				{  // the point(ncur) is not on the convex hull
					s1 = false;
					s2 = false;
					fg[ncur] = true;
					for(int k=ncur-1; k>=0; k--)
					{
						if(fg[k]==false)
						{
							if(!s1)
							{
								ncur = k;
								s1 = true;
							}
							else if(!s2)
							{
								npre = k;
								s2 = true;
							}

							if(s1&&s2)
								break;
						}
					}

					if(ncur==0)
					{
						npre = 0;
						ncur = i;
						suc = true;
					}
				}
			}while(!suc);
		}

		int num=0;
		for(i=0; i<num1; i++)
		{
			if(!fg[i])
			{
				UP[num] = plist[uf[i]];
				elist[num] = nedge[uf[i]];
				eu[num] = eratio[uf[i]];
				num++;
			}
		}

		num1 = num;

		// lower part
		for(i=0; i<num2; i++)
		{
			fg[i] = false;
		}

		npre = 0;
		ncur = 1;
		for(i=2; i<num2; i++)
		{
			suc = false;
			do
			{
				if(((LP[i].x - LP[npre].x)*(LP[ncur].y - LP[npre].y)-(LP[i].y - LP[npre].y)*(LP[ncur].x - LP[npre].x))>0)
				{
					npre = ncur;
					ncur = i;
					suc = true;
				}
				else
				{  // the point(ncur) is not on the convex hull
					s1 = false;
					s2 = false;
					fg[ncur] = true;

					for(int k=ncur-1; k>=0; k--)
					{
						if(fg[k]==false)
						{
							if(!s1)
							{
								ncur = k;
								s1 = true;
							}
							else if(!s2)
							{
								npre = k;
								s2 = true;
							}

							if(s1&&s2)
								break;
						}
					}

					if(ncur==0)
					{
						npre = 0;
						ncur = i;
						suc = true;
					}
				}
			}while(!suc);
		}

		num=0;
		for(i=0; i<num2; i++)
		{
			if(!fg[i])
			{
				LP[num] = plist[lf[i]];
				elist[num1+num-1] = nedge[lf[i]];
				eu[num1+num-1] = eratio[lf[i]];
				num++;
			}
		}

		num2 = num;

		for(i=0; i<num1-1; i++)
		{
			plist[i] = UP[i]; 
		}
		for(i=0; i<num2; i++)
		{
			plist[i+num1-1] = LP[i];
		}

		pnum = num1+num2-1;
		for(i=0; i<pnum; i++)
		{
			nedge[i] = elist[i]; 
			eratio[i] = eu[i];
		}

		delete [] pp;
		delete [] fg;
		delete [] UP;
		delete [] uf;
		delete [] LP;
		delete [] lf;
		delete [] elist;
		delete [] eu;
	}

	return true;
}


//---------------------------------------------------------------
// Name:	    IntersectPlane(Vec4 pt, Vec4 pnorm, Vec4 *plist, int &pnum)
// Description: Compute the polygons by intersecting the HMesh and a plane going through a point
//				if there are several closed loops, separate them by point(-10000, -10000, -10000). 
//				Method: using the connection information of HMesh to quick find the intersection polygon
// Argument:	input : pt--a point on the cutting plane;	 pnorm--plane normal
//         :	output: plist--intersecting points;			 pnum--the number of intersecting points
// Return:		if no closed loop is obtained, return false; else, return ture.
//----------------------------------------------------------------
bool XBaseMesh::IntersectPlane(Vec4 pt, Vec4 pnorm, Vec4 *plist, int &pnum)
{
	int i;
	int starte, ne = -1, ne1 = -1, nf, num;
	Vec4 vt1, vt2;
	XFace f1;
	float d1, d2;
	bool *flag, suc;
	float dist, mind = 1.0e+8;

	pnum = 0;

	flag = new bool[m_edge.size()];

	suc = false;

	for(i=0; i<static_cast<int>(m_edge.size()); i++)
	{
		flag[i] = false;

		vt1 = m_vert[m_edge[i].p(0)].Pos();
		vt2 = m_vert[m_edge[i].p(1)].Pos();

		d1 = pnorm.x*(vt1.x-pt.x)+pnorm.y*(vt1.y-pt.y)+pnorm.z*(vt1.z-pt.z);
		d2 = pnorm.x*(vt2.x-pt.x)+pnorm.y*(vt2.y-pt.y)+pnorm.z*(vt2.z-pt.z);

		if(((d1>0&&d2<0)||(d1<0&&d2>0)||d1>0&&d2==0||d1==0&&d2>0))
		{
			// mark all edges acrossing the plane 
			flag[i] = true;  

			vt1 = vt1-pt;
			dist = vt1.x*vt1.x + vt1.y*vt1.y + vt1.z*vt1.z;
			if(dist<mind)
			{
				mind = dist;
				ne = i;
				suc = true;
			}
		}
	}

	if(suc==false)
	{
		delete []flag;
		return false;
	}

	starte = ne;
	num =0;

	suc = false;
	while(suc==false)
	{
		nf = m_edge[ne].GetFIdx(0);

		vt1 = m_vert[m_edge[ne].p(0)].Pos();
		vt2 = m_vert[m_edge[ne].p(1)].Pos();

		d1 = pnorm.x*(vt1.x-pt.x)+pnorm.y*(vt1.y-pt.y)+pnorm.z*(vt1.z-pt.z);
		d2 = pnorm.x*(vt2.x-pt.x)+pnorm.y*(vt2.y-pt.y)+pnorm.z*(vt2.z-pt.z);

		plist[num].x = (float)fabs(d1)/((float)fabs(d1)+(float)fabs(d2))*(vt2.x-vt1.x)+vt1.x;
		plist[num].y = (float)fabs(d1)/((float)fabs(d1)+(float)fabs(d2))*(vt2.y-vt1.y)+vt1.y;
		plist[num].z = (float)fabs(d1)/((float)fabs(d1)+(float)fabs(d2))*(vt2.z-vt1.z)+vt1.z;
		num++;

		// continue to find the next edge until the edge is the same as the start edge ( a loop is found)
		int nn=0;
		do
		{
			nn++;
			f1=m_face[nf];
			for(i=0; i<3; i++)
			{
				ne1 = f1.GetEdgeRef(i);

				if(ne1==ne || flag[ne1]==false) continue;

				vt1 = m_vert[m_edge[ne1].p(0)].Pos();
				vt2 = m_vert[m_edge[ne1].p(1)].Pos();

				d1 = pnorm.x*(pt.x-vt1.x)+pnorm.y*(pt.y-vt1.y)+pnorm.z*(pt.z-vt1.z);
				d2 = pnorm.x*(pt.x-vt2.x)+pnorm.y*(pt.y-vt2.y)+pnorm.z*(pt.z-vt2.z);

				if(((d1>=0&&d2<=0)||(d1<=0&&d2>=0))&&fabs(d1-d2)>0.000001)
				{

					// get the next face
					if(m_edge[ne1].GetFIdx(0)==nf)
						nf = m_edge[ne1].GetFIdx(1);
					else 
						nf = m_edge[ne1].GetFIdx(0);

					flag[ne1] = false;

					ne = ne1;

					// compute the intersecting point
					plist[num].x = (float)fabs(d1)/((float)fabs(d1)+(float)fabs(d2))*(vt2.x-vt1.x)+vt1.x;
					plist[num].y = (float)fabs(d1)/((float)fabs(d1)+(float)fabs(d2))*(vt2.y-vt1.y)+vt1.y;
					plist[num].z = (float)fabs(d1)/((float)fabs(d1)+(float)fabs(d2))*(vt2.z-vt1.z)+vt1.z;
					num++;

					break;
				}
			}
		}while(ne!=starte);

		suc = true;
		for(i=0; i<static_cast<int>(m_edge.size()); i++)
		{
			if(flag[i] == true)
			{
				ne = i;
				starte = ne;

				plist[num].x = -10000;
				plist[num].y = -10000;
				plist[num].z = -10000;
				num++;

				suc = false;
				break;
			}
		}
	};

	pnum = num;

	delete []flag;

	return true;
}

//---------------------------------------------------------------
// Name:	    IntersectPlane(Vec4 pt, Vec4 pnorm, Vec4 *plist, int &pnum, int nid,int iDir)
// Description: Compute the polygons by intersecting HMesh of current part and a plane, the polygon may be not closed.
//				method: get all intersecting points first, and then, arrange them by angle ascendingly.
// Argument:	input : pt--a point on the cutting plane;      pnorm--plane normal
//						nid--current part ID;    iDir--0: points on x-z plane,1:points on y-z plane, 2: points on x-y plane
//		   :	output: plist--intersecting points; pnum--the number of intersecting points
// Return:		if no closed loop is obtained, return false; else, return true.
// Author:	    XXX
// Date:	    Mar. 19, 2003
// Update:		XXX
// Author:		2006/05/07 7:5:2006   14:31
// Date: 
// copyright: XXX. developed by XXX
//----------------------------------------------------------------

bool XBaseMesh::IntersectPlane(Vec4 pt, Vec4 pnorm, Vec4 *plist, int &pnum, int nid,int iDir)
{
	int i, j;
	Vec4 vt1, vt2;
	float d1, d2;

	pnum = 0;

	for(i=0; i<static_cast<int>(m_edge.size()); i++)
	{		
		if(nid != -1 && m_vert[m_edge[i].p(0)].m_vid != nid && m_vert[m_edge[i].p(1)].m_vid != nid) continue;

		vt1 = m_vert[m_edge[i].p(0)].Pos();
		vt2 = m_vert[m_edge[i].p(1)].Pos();

		d1 = pnorm.x*(vt1.x-pt.x)+pnorm.y*(vt1.y-pt.y)+pnorm.z*(vt1.z-pt.z);
		d2 = pnorm.x*(vt2.x-pt.x)+pnorm.y*(vt2.y-pt.y)+pnorm.z*(vt2.z-pt.z);

		if(((d1>0&&d2<0)||(d1<0&&d2>0)||d1>0&&d2==0||d1==0&&d2>0))
		{  
			plist[pnum].x = (float)fabs(d1)/((float)fabs(d1)+(float)fabs(d2))*(vt2.x-vt1.x)+vt1.x;
			plist[pnum].y = (float)fabs(d1)/((float)fabs(d1)+(float)fabs(d2))*(vt2.y-vt1.y)+vt1.y;
			plist[pnum].z = (float)fabs(d1)/((float)fabs(d1)+(float)fabs(d2))*(vt2.z-vt1.z)+vt1.z;
			pnum++;
		}
	}

	if(pnum==0) return false;


	// get the center
	Vec4 center;

	center = Vec4(0, 0, 0);

	for(i=0; i<pnum; i++)
	{
		center += plist[i];
	}

	center = center/(float)pnum;

	// compute the angle of each point
	float *pang, tmp;

	pang = new float [pnum];

	if(pnorm.x == 0 || iDir == 0)
	{
		for(i=0; i<pnum; i++)
		{
			pang[i] = Angc(center.x, center.z, plist[i].x, plist[i].z);
		}
	}
	else if(pnorm.y==0 ||(fabs(pnorm.x)>fabs(2*pnorm.y) && fabs(pnorm.x)>fabs(2*pnorm.z)) || iDir == 1)
	{
		for(i=0; i<pnum; i++)
		{
			pang[i] = Angc(center.y, center.z, plist[i].y, plist[i].z);
		}
	}
	else
	{
		double xa, ya;
		Vec4 pp;
		pnorm.ComputeGlbAng(&xa, &ya);

		for(i=0; i<pnum; i++)
		{
			pp = plist[i] - center;

			pp = pp.RotateX(-xa);
			pp = pp.RotateY(-ya);

			pang[i] = Angc(0, 0, pp.x, pp.y);
		}

	}

	Vec4 vt;

	for(i=0; i<pnum-1; i++)
	{
		for(j=i+1; j<pnum; j++)
		{
			if(pang[i]>pang[j])
			{
				tmp = pang[i];
				pang[i] = pang[j];
				pang[j] = tmp;

				vt = plist[i];
				plist[i] = plist[j];
				plist[j] = vt;
			}
		}
	}

	plist[pnum] = plist[0];
	pnum++;

	delete [] pang;

	return true;
}


// Compute the mesh bounding box in 3d
void XBaseMesh::ComputeMeshBox3D(void)
{
	if(GetVSize() == 0)
	{
		m_maxMeshBox3D = Vec4(0,0,0);
		m_minMeshBox3D = m_maxMeshBox3D;
	}
	else if(GetVSize() == 1)
	{
		m_maxMeshBox3D = GetV(0).Pos();
		m_minMeshBox3D = GetV(0).Pos();
	}
	else
	{
		m_maxMeshBox3D = GetV(0).Pos();
		m_minMeshBox3D = GetV(0).Pos();

		for(int i = 1; i < GetVSize(); ++i)
		{
			m_maxMeshBox3D = Maximize(m_maxMeshBox3D, GetV(i).Pos());
			m_minMeshBox3D = Minimize(m_minMeshBox3D, GetV(i).Pos());
		}
	}
	m_bMeshBox3DComput = true;
}


//---------------------------------------------------------------
// Name:	    GetIntersectPlaneNorm()
// Description: Get the plane norm for intersect with the start and end points(face) are given to specify the plane
// Argument:	stpt:-- start point,       stfaceidx:--start face
//         :	endpt:--end point;         endfaceidx:--end face
// Return:		
// Author:	    XXX
// Date:	    2006/05/07 7:5:2006   15:34
// Update:	 
// Author:	
// Date: 
// copyright: XXX. developed by XXX
//----------------------------------------------------------------
Vec4 XBaseMesh::GetIntersectPlaneNorm( Vec4 &stpt, int stfaceidx,  Vec4 &endpt, int endfaceidx)
{    
	return ((m_face[stfaceidx].Norm()+m_face[endfaceidx].Norm())/2).CrossVecX(endpt-stpt).Normalize(); 
}

//---------------------------------------------------------------
// Name:	    IntersectCurLoop()
// Description: Compute the polygons by intersecting the mesh and a level. Only one loop will be obtained
//				This function only can not be used in pattern!!!!!!!!!!!!
// Argument:	pt - a point on the level;   ptlist: for intersecting points 
//         :	pnum: the number of intersecting points; norm:-- normal of the intersect plane
// Return:		
// Author:	    wangmei
// Date:	    2005.1.28
// Update:	 
// Author:		XXX
// Date:		2006/05/07 7:5:2006   16:23
// copyright: XXX. developed by XXX
//----------------------------------------------------------------

bool XBaseMesh::IntersectCurLoop(Vec4 pt, Vec4 norm, Vec4 *ptlist, int &ptnum)
{
	Vec4 vt1, vt2;
	float d1, d2;
	float dist, mindis = 1.0e+8;
	int starte = -1;
	varray<bool> flag;
	flag.resize(m_edge.size());

	for(int i = 0; i < static_cast<int>(m_edge.size()); i++)
	{
		flag[i] = false;
		vt1 = m_vert[m_edge[i].p(0)].Pos() - pt;
		vt2 = m_vert[m_edge[i].p(1)].Pos() - pt;

		d1 = norm.dotmultiple(vt1);
		d2 = norm.dotmultiple(vt2);

		if((d1 > 0 && d2 <= 0)||((d2 > 0) && (d1 <= 0)))
		{
			flag[i] = true;  

			dist = vt1.GetLength();
			if(dist < mindis)
			{
				mindis = dist;
				starte = i;
			}
		}
	}

	if (starte == -1)
		return false;

	int ne = starte;
	vt1 = m_vert[m_edge[ne].p(0)].Pos();
	vt2 = m_vert[m_edge[ne].p(1)].Pos();
	d1 = norm.dotmultiple(vt1 - pt);
	d2 = norm.dotmultiple(vt2 - pt);

	float ratio = fabs(d1) / (fabs(d1) + fabs(d2));
	ptlist[0] = (vt2 - vt1) * ratio + vt1;
	ptnum = 1;

	int nf = m_edge[ne].GetFIdx(0);
	int ne1;
	XFace face;

	while (nf != -1) 
	{
		face = m_face[nf];

		if (!flag[face.GetEdgeRef(0)] && !flag[face.GetEdgeRef(1)] && !flag[face.GetEdgeRef(2)])
		{
			return false;
		}

		for(int i = 0; i < 3; i++)
		{
			ne1 = face.GetEdgeRef(i);

			if (ne1 == ne || !flag[ne1])
			{
				continue;
			}

			if (m_edge[ne1].p(0) == -1 || m_edge[ne1].p(i) == -1)
			{
				return false;
			}
			vt1 = m_vert[m_edge[ne1].p(0)].Pos();
			vt2 = m_vert[m_edge[ne1].p(1)].Pos();
			d1 = norm.dotmultiple(vt1 - pt);
			d2 = norm.dotmultiple(vt2 - pt);

			if (d1 == 0)
			{
				if (ptlist[ptnum].x != vt1.x && ptlist[ptnum].y != vt1.y && ptlist[ptnum].z != vt1.z)
				{
					ptlist[ptnum++] = vt1;
				}
			}
			else if (d2 == 0)
			{
				if (ptlist[ptnum].x != vt2.x && ptlist[ptnum].y != vt2.y && ptlist[ptnum].z != vt2.z)
				{
					ptlist[ptnum++] = vt2;
				}
			}
			else
			{
				ratio = fabs(d1) / (fabs(d1) + fabs(d2));
				ptlist[ptnum++] = (vt2 - vt1) * ratio + vt1;
			}

			if (m_edge[ne1].GetFIdx(0) == nf)
				nf = m_edge[ne1].GetFIdx(1);
			else 
				nf = m_edge[ne1].GetFIdx(0);

			flag[ne1] = false;
			ne = ne1;

			break;
		}

		if (ne == starte)
		{
			break;
		}
	}

	return true;
}


//---------------------------------------------------------------
// Name:	    GetIntersectedFace(int ncurf, Vec4 p1, Vec4 p2)
// Description: search a face that intersects with a line from a starting face
// Argument:	ncurf: the start face id for searching out the face the intersect with the line
//				p1,p2: the endpoints of the line
// Return:		int (the face id that intersect with the give line p1p2)
// Author:		 
// Date:		 
// Modified by:	 XXX	
// Updated date: 2005/09/02 2:9:2005   17:27	
//----------------------------------------------------------------
/*
int XBaseMesh::GetIntersectedFace(int ncurf, Vec4 p1, Vec4 p2)
{
	XFace f;
	Vec4 pt;
	int ne, nf;
	int ni;

	ni=0;
	nf=ncurf;

	do{
		f=	m_face[nf];
		// compute the intersection point
		pt = f.Intersect_Line(p1, p2);

		// check whether the point is inside the face
		if(f.IsPtInside(pt, ne)==true)
		{
			return nf;
		}
		else // the the edge link information to check whether the neck face is intersect with p1p2
		{
			if(m_edge[ne].GetFIdx(0)==ncurf)
			{
				nf = m_edge[ne].GetFIdx(1);
			}
			else
			{
				nf = m_edge[ne].GetFIdx(0);
			}
			ncurf = nf;
		}
		ni++;
	}while(ni<20);

	return -1;
}
*/



//---------------------------------------------------------------
// Name:	    AngleInit(int iVid)
// Description: Calculate the 3 original inner angles for the faces of the inputed vertex linked
// Argument:	iVid:-- vertex index
//         :	
// Return:		
// Author:	    XXX
// Date:	    2006/05/07 7:5:2006   16:31
// Update:	 
// Author:	
// Date: 
// copyright: XXX. developed by XXX
//----------------------------------------------------------------
void XBaseMesh::AngleInit(int iVid)
{
	if(iVid <0 || iVid >= static_cast<int>(m_vert.size()))
	{
		return;
	}
	int i = 0;
	for(i = 0; i < m_vert[iVid].GetAdjFSize(); i++)
	{
//		m_face[m_vert[iVid].AdjF(i)].m_AngleInited = false;
		AngleInitInAFace(m_vert[iVid].AdjF(i));
	}
}

//---------------------------------------------------------------
// Name:	    GetAngleOnVert()
// Description: Calculate the current angle of the vertex in the specified face
// Argument:	fid:-- face index;  pid:-- vertex index of the face;(0,1,2)
//         :	
// Return:		
// Author:	    XXX
// Date:	    2006/05/07 7:5:2006   16:40
// Update:	 
// Author:	
// Date: 
// copyright: XXX. developed by XXX
//----------------------------------------------------------------
float XBaseMesh::GetAngleOnVert(int fid,int pid)
{
	int i;
	if(fid >= 0 && fid < static_cast<int>(m_face.size()))
	{
		for(i=0; i<3; i++)
		{
			if(m_face[fid].p(i) == pid)
			{
				return m_face[fid].Angle(i);
			}
		}
	}
	return -1.0;
}

//---------------------------------------------------------------
// Name:	    CurvatureOnVertexCalculation()
// Description: Calculate the gauss curvature of all the vertices in the mesh
// Argument:
//         :	
// Return:		
// Author:	    XXX
// Date:	    2006/05/07 7:5:2006   16:42
// Update:	 
// Author:	
// Date: 
// copyright: XXX. developed by XXX
//----------------------------------------------------------------
void XBaseMesh::CurvatureOnVertexCalculation()
{
	int i,j;
	XEdge* pEdge;
	int adjFSize;
	float expAngle;
	float Pi=3.1415926f;
	Vec4 vt;

	BoundaryFlagSetting();

	AngleInit();

	for(i = 0; i < static_cast<int>(m_vert.size()); i++)
	{
		m_vert[i].GetCurvature() = 0.0;
	}
	for(i = 0; i < static_cast<int>(m_vert.size()); i++)
	{
		expAngle=0.0;
		const Vec4& pt = m_vert[i].Pos();
		if(m_vert[i].IsOnBoundary())
		{
			continue;
		}
		adjFSize = m_vert[i].GetAdjFSize();
		if(adjFSize==1)
		{
			continue;
		}
		for(j = 0; j < adjFSize;j++)
		{
			expAngle+=GetOriAngleOnVert(m_vert[i].AdjF(j),i);
		}
		m_vert[i].GetCurvature() = static_cast<float>(fabs((2.*Pi-expAngle)/(2.*Pi)));

		Vec4 Vect = Vec4(0, 0, 0);
		// Check whether the vertex is convex or concave,and change the value of curvature correspondingly
		for(j = 0; j < m_vert[i].GetAdjESize(); j++)
		{
			pEdge = &(m_edge[m_vert[i].AdjE(j)]);
			if(pEdge->p(0) == i)
			{
				vt = m_vert[pEdge->p(1)].Pos();
			}
			else
			{
				vt = m_vert[pEdge->p(0)].Pos();
			}
			Vect += (vt - pt).Normalize();
		}
		if(Vect.dotmultiple(m_vert[i].Norm()) > 0)
		{
			m_vert[i].GetCurvature() = -m_vert[i].GetCurvature();
		}
	}



	int nume = 0, vid = 0;
	float slen;
	Vec4 p1,p2;
	for(i = 0; i < static_cast<int>(m_vert.size()); i++)
	{
		if(!m_vert[i].IsOnBoundary())
		{
			continue;
		}
		nume = m_vert[i].GetAdjESize();

		if(nume<=2)
			continue;

		pEdge = &(m_edge[m_vert[i].AdjE(0)]);
		p1 = m_vert[pEdge->p(0)].Pos();
		p2 = m_vert[pEdge->p(1)].Pos();
		slen = (p1 - p2).GetLength();

		for(j = 1; j < nume; j++)
		{
			pEdge = &(m_edge[m_vert[i].AdjE(j)]);
			p1 = m_vert[pEdge->p(0)].Pos();
			p2 = m_vert[pEdge->p(1)].Pos();
			if((p1 - p2).GetLength() < slen)
			{
				if(m_edge[m_vert[i].AdjE(j)].p(0) == i)
				{
					vid = m_edge[m_vert[i].AdjE(j)].p(1);
				}
				else
				{
					vid = m_edge[m_vert[i].AdjE(j)].p(0);
				}
				if(m_vert[vid].IsOnBoundary())			
				{
					continue;
				}
				slen = (p1 - p2).GetLength();
				m_vert[i].GetCurvature() = m_vert[vid].GetCurvature();
			}
		}
	}

	for(i = 0; i < static_cast<int>(m_vert.size()); i++)
	{
		if(!m_vert[i].IsOnBoundary())
		{
			continue;
		}		

		nume = m_vert[i].GetAdjESize();

		if(nume != 2)
			continue;

		pEdge = &(m_edge[m_vert[i].AdjE(0)]);
		p1 = m_vert[pEdge->p(0)].Pos();
		p2 = m_vert[pEdge->p(1)].Pos();
		slen = (p1 - p2).GetLength();

		for(j = 1; j < nume; j++)
		{
			pEdge = &(m_edge[m_vert[i].AdjE(j)]);
			p1 = m_vert[pEdge->p(0)].Pos();
			p2 = m_vert[pEdge->p(1)].Pos();
			if((p1 - p2).GetLength() < slen)
			{
				if(m_edge[m_vert[i].AdjE(j)].p(0) == i)
				{
					vid = m_edge[m_vert[i].AdjE(j)].p(1);
				}
				else
				{
					vid = m_edge[m_vert[i].AdjE(j)].p(0);
				}
				if(!m_vert[vid].IsOnBoundary())		
				{
					continue;
				}
				slen = (p1 - p2).GetLength();
				m_vert[i].GetCurvature() = m_vert[vid].GetCurvature();
			}
		}
	}
}
//---------------------------------------------------------------
// Name:		CurvatureOnVertexCalculation()
// Description: Calculate the gauss curvature of the specified vertex in the mesh
// Argument:	iVid:-- vertex index in the mesh vertices array
//         :	
// Return:		
// Author:		XXX
// Date:		 
// Modified by:	XXX
// Updated date: 2005/11/23 23:11:2005   10:49	
//----------------------------------------------------------------
float  XBaseMesh:: CurvatureOnVertexCalculation(int iVid)
{
	int j;
	XEdge* pEdge;
	int adjFSize;
	float expAngle;
	float Pi=3.1415926f;
	Vec4 vt, pt;
	float fCurvature = 0.0;

	if(iVid < 0 || iVid >= static_cast<int>(m_vert.size()))
	{
		return fCurvature;
	}
	BoundaryFlagSetting();
	if(!m_AngleInited)
	{
		AngleInit();
	}
	else
	{
		AngleInit(iVid);
	}

	expAngle=0.0;
	pt = m_vert[iVid].Pos();
	if(m_vert[iVid].IsOnBoundary())
	{
		fCurvature = 0;
	}
	else
	{
		adjFSize = m_vert[iVid].GetAdjFSize();
		if(adjFSize==1)
		{
			fCurvature = 0;
			return fCurvature;
		}
		for(j = 0; j < adjFSize;j++)
		{
			expAngle+=GetOriAngleOnVert(m_vert[iVid].AdjF(j),iVid);
		}
		fCurvature = static_cast<float>(fabs((2.*Pi-expAngle)/(2.*Pi)));
		Vec4 Vect = Vec4(0, 0, 0);
		// Check whether the vertex is convex or concave,and change the value of curvature correspondingly
		for(j = 0; j < m_vert[iVid].GetAdjESize(); j++)
		{
			pEdge = &(m_edge[m_vert[iVid].AdjE(j)]);
			if(pEdge->p(0) == iVid)
			{
				vt = m_vert[pEdge->p(1)].Pos();
			}
			else
			{
				vt = m_vert[pEdge->p(0)].Pos();
			}
			Vect += (vt - pt).Normalize();
		}
		if(Vect.dotmultiple(m_vert[iVid].Norm()) > 0)
		{
			fCurvature = -fCurvature;
		}
	}
	return fCurvature;
}


//---------------------------------------------------------------
// Name:	    CalCurvTFunc()
// Description: Calculate the value as a Eq:  T = g(iVid)*g(iVid) + Sum( g(qj) * g(qj)) )
//              Where g(iVid) is the iVid vertex gauss curvature, qj is the adjacent vertex of iVid
//              Sum() is a function to get the sum of all the variable 
// Argument:   
//         :	
// Return:		
// Author:		XXX
// Date:		 
// Modified by:	XXX
// Updated date: 2005/11/23 23:11:2005   10:49	
//----------------------------------------------------------------
double  XBaseMesh::CalCurvTFunc(int iVid)
{
	double fT = 0;
	if(iVid < 0 || iVid >= static_cast<int>(m_vert.size()))
	{
		return fT;
	}
	AngleInit(iVid);
	m_AngleInited = false;
	float fTmp = CurvatureOnVertexCalculation(iVid);
	fT = fTmp * 1.0e2;
	fT = fT * fT;

	int i = 0;
	int iAdjV = -1;
	XEdge* pEdge = NULL;
	double fG = 0;
	for(i = 0; i < m_vert[iVid].GetAdjESize(); i++)
	{
		pEdge = &(m_edge[m_vert[iVid].AdjE(i)]);
		iAdjV = (pEdge->p(0) == iVid) ? pEdge->p(1) : pEdge->p(0);
		fG = 0;
		if(iAdjV >= 0 && iAdjV < static_cast<int>(m_vert.size()))
		{
			fG = CurvatureOnVertexCalculation(iAdjV) * 1.0e2;
			fG = fG * fG;
		}
		fT += fG;
	}
	return fT;
}
//---------------------------------------------------------------
// Name:	     CalDerivativeOfT()
// Description:  Get the derivative of T, T = g(iVid)*g(iVid) + Sum( g(qj) * g(qj)) )
// Argument:
//         :	
// Return:		
// Author:		XXX
// Date:		 
// Modified by:	XXX
// Updated date: 2005/11/23 23:11:2005   11:03	
//----------------------------------------------------------------
double XBaseMesh::CalDerivativeOfT(int iVid)
{
	float fDT = 0;
	if(iVid < 0 || iVid >= static_cast<int>(m_vert.size()))
	{
		return fDT;
	}
	double fT1,fT2;
	Vec4 ptBak = m_vert[iVid].Pos();
	m_vert[iVid].Pos() = ptBak - m_vert[iVid].Norm();
	fT1 = CalCurvTFunc(iVid);
	m_vert[iVid].Pos() = ptBak + m_vert[iVid].Norm();
	fT2 = CalCurvTFunc(iVid);

	fDT = static_cast<float>((fT2 - fT1)/2);
	m_vert[iVid].Pos() = ptBak;
	return fDT;
}

//---------------------------------------------------------------
// Name:		GetAdjFids(...)
// Description:	Find adjacent faces on the boundary
// Argument:	int fid: index of a boundary face
//				adjFids: a list of adjacent to be searched
//				loopNum: number of loop to be searched
// Return:		void
// Author:		ljt
// Date:		9/9/2005
// Modified by:	D. XXX	
// Updated date: Apr. 25,2006
//---------------------------------------------------------------- 
void XBaseMesh::GetAdjFids(int fid, varray<int>& adjFids, int loopNum)
{
	int i,j,k, adjFid, adjEid, adjvid;
	varray<int> frontVids;
	varray<int> curVids;
	varray<int> preVids;
	const XFace& f=GetF(fid);

	if(loopNum<1)
		loopNum=1;

	for(i=0;i<3;i++)
	{
		if(GetV(f.p(i)).GetAdjFSize() <= 1)
			continue;
		curVids.push_back(f.p(i));
	}

	for(i=0;i<loopNum;i++)	
	{
		for(j=0;j<static_cast<int>(curVids.size());j++)
		{
			const XVert& v=GetV(curVids.at(j));
			for(k=0;k<v.GetAdjFSize();k++)
			{
				adjFid=v.AdjF(k);

				if(IsInTheVarray(adjFid,adjFids))
					continue;
				adjFids.push_back(adjFid);
			}

			for(i=0; i<v.GetAdjESize(); i++)
			{
				adjEid=v.AdjE(i);
				const XEdge& e=GetE(adjEid);
				adjvid=e.p(0)!=curVids.at(j)? e.p(0):e.p(1);

				if(IsInTheVarray(adjvid,frontVids) || IsInTheVarray(adjvid,preVids))
					continue;

				frontVids.push_back(adjvid);
			}
		}

		for(j=0;j<static_cast<int>(curVids.size());j++)
		{
			preVids.push_back(curVids.at(j));
		}

		curVids.clear();
		curVids=frontVids;
		frontVids.clear();
	}

	return;
}

//---------------------------------------------------------------
// Name:		GetAdjFids(...)
// Description:	Get all adjacent faces connecting to specified vertices
// Argument:	inputVids: a list of vertex indices
//				adjFids: a list of face indices
// Return:		void
// Author:		ljt
// Date:		9/9/2005
// Modified by:	D. XXX	
// Updated date: Apr. 25,2006
//---------------------------------------------------------------- 
void XBaseMesh::GetAdjFids(const varray<int>& inputVids, varray<int>& adjFids)
{
	int i,j, fid;
	adjFids.clear();

	for(i=0;i<static_cast<int>(inputVids.size());i++)
	{
		const XVert& v=GetV(inputVids.at(i));

		for(j=0;j<v.GetAdjFSize();j++)
		{
			fid=v.AdjF(j);
			if(IsInTheVarray(fid, adjFids))
			{
				continue;
			}

			adjFids.push_back(fid);
		}
	}

	return;
}

////---------------------------------------------------------------
//// Name:		GetAdjFidsOnBound(...)
//// Description:	Find adjacent faces on the boundary
//// Argument:	int fid: index of a boundary face
////				adjFids: a list of adjacent to be searched
////				loopNum: number of loop to be searched
//// Return:		void
//// Author:		ljt
//// Date:		9/9/2005
//// Modified by:	D. XXX	
//// Updated date: Apr. 25,2006
////---------------------------------------------------------------- 
//void XBaseMesh::GetAdjFidsOnBound(int fid, varray<int>& adjFids, int loopNum)
//{
//	int i,j,k, adjFid, adjEid, adjvid;
//	varray<int> frontVids;
//	varray<int> curVids;
//	varray<int> preVids;
//	const XFace& f=GetF(fid);
//
//	if(loopNum<1)
//		loopNum=1;
//
//
//	for(i=0;i<3;i++)
//	{
//		if(m_vertex.at(f.p(i)).IsCorner())
//			continue;
//
//		curVids.push_back(f.p(i));
//	}
//
//	if(curVids.size()==0)
//	{
//		float maxAng=-1.0e6;
//		float tempAng;
//		int idx = 0;
//		for(i=0;i<3;i++)
//		{
//			tempAng=Get3DTotalAngleAtPoint(f.p(i));
//			if(tempAng>maxAng)
//			{
//				idx=i;
//				maxAng=tempAng;
//			}
//		}
//
//		curVids.push_back(f.p((idx+1)%3));
//		curVids.push_back(f.p((idx+2)%3));
//	}
//
//	for(i=0;i<loopNum;i++)	
//	{
//		for(j=0;j<static_cast<int>(curVids.size());j++)
//		{
//			const XVert& v=GetV(curVids.at(j));
//			for(k=0;k<v.GetAdjFSize();k++)
//			{
//				adjFid=v.AdjF(k);
//
//				if(IsInTheVarray(adjFid,adjFids))
//					continue;
//
//				adjFids.push_back(adjFid);
//			}
//
//			for(k=0; k<v.GetAdjESize(); k++)
//			{
//				adjEid=v.AdjE(k);
//				const XEdge& e=GetE(adjEid);
//				adjvid=e.p(0)!=curVids.at(j)? e.p(0):e.p(1);
//
//				if(IsInTheVarray(adjvid,frontVids) || IsInTheVarray(adjvid,preVids))
//					continue;
//
//				frontVids.push_back(adjvid);
//			}
//		}
//
//		for(j=0;j<static_cast<int>(curVids.size());j++)
//		{
//			preVids.push_back(curVids.at(j));
//		}
//
//		curVids.clear();
//		curVids=frontVids;
//		frontVids.clear();
//	}
//
//	return;
//}

//---------------------------------------------------------------
// Name:		GetInitialAngleInFace
// Function:	get 3d angle of the face (same as get OriAngle)
// Argument:	fid: face index
//				pid: point index
// Return:		
// Author:		unknown
// Date:		unknown
// Modified by:		XXX(add comments)
// Updated date:	4/28/2006	
//---------------------------------------------------------------- 
float XBaseMesh::GetInitialAngleInFace(int fid,int pid)
{
	int i;
	XFace& f=GetF(fid);
	int selectedNO=-1;
	for( i=0;i<3;i++)
	{
		if(static_cast<int>(f.p(i))==pid)
		{
			selectedNO=i;
			break;
		}
	}

	return(f.GetOriAngle(selectedNO));
}


//---------------------------------------------------------------
// Name:		Comp3DAngOnVert()
// Description:	Calculate one angle of a face. The angle is corresponding to the vertex
// Argument:	fid: face ID
//				pid: vertex ID
// Return:		normal value
// Author:		ljt
// Date:		9/8/2005
// Modified by:	D. XXX	
// Updated date: Apr. 25, 2006
//---------------------------------------------------------------
float XBaseMesh::Comp3DAngOnVert(int fid, int pid)
{
	int i;
	XFace& f=GetF(fid);

	for(i=0;i<3;i++)
	{
		if(f.p(i)==pid)
		{
			float ang=GetAngleOf2Vector(GetV(f.p((i+1)%3)).Pos()-GetV(f.p(i)).Pos(),GetV(f.p((i+2)%3)).Pos()-GetV(f.p(i)).Pos());
			return ang;
		}
	}

	return -1.0;
}

//---------------------------------------------------------------
// Name:		ComputeVertNormal()
// Description:	Calculate the normal of a vertex
// Argument:	vid: vertex index
// Return:		normal value
// Author:		ljt
// Date:		9/8/2005
// Modified by:	D. XXX	
// Updated date: Apr. 25, 2006
//---------------------------------------------------------------
Vec4 XBaseMesh::ComputeVertNormal(int vid)
{
	int i, j,k;
	XVert& v=GetV(vid);
	v.Norm()=Vec4 (0.,0.,0.);
	Vec4 selFNorm;

	for(i=0;i<v.GetAdjFSize();i++)
	{
		XFace& f = GetF(v.AdjF(i));		
		Vec4& v0 = GetV(f.GetIndex(0)).Pos();
		Vec4& v1 = GetV(f.GetIndex(1)).Pos();
		Vec4& v2 = GetV(f.GetIndex(2)).Pos();
		f.Norm() = CrossVecX((v1 - v0), (v2 - v0));

		if(f.Norm().Magnitude()<1.0e-4)
		{
			bool flag=false;
			for(j=0;j<3;j++)
			{
				const XVert& v=GetV(f.p(j));
				for(k=0;k<v.GetAdjFSize();k++)
				{
					XFace& f1 = GetF(v.AdjF(k));		
					Vec4& v0 = GetV(f1.GetIndex(0)).Pos();
					Vec4& v1 = GetV(f1.GetIndex(1)).Pos();
					Vec4& v2 = GetV(f1.GetIndex(2)).Pos();
					Vec4 temp = CrossVecX((v1 - v0), (v2 - v0));
					if(temp.Magnitude()<1.0e-4)
					{
						continue;
					}
					f.Norm()=temp.Normalize();
					flag=true;
					break;
				}

				if(flag)
				{
					break;
				}
			}

			if(!flag)
			{
				f.Norm()=Vec4(1.0,0.,0.);
			}
		}
		else
		{
			f.Norm()=f.Norm().Normalize();
			selFNorm=f.Norm();
		}
		v.Norm()=v.Norm()+f.Norm();
	}

	v.Norm()=v.Norm().Normalize();
	if(v.Norm().Magnitude()<1.0e-3)
	{
		v.Norm()=selFNorm;
	}

	return v.Norm();
}

////---------------------------------------------------------------
//// Name:		Comput3DFaceArea()
//// Description:	Compute the area of a triangle
//// Argument:	fid: face ID
//// Return:		area value
//// Author:		ljt
//// Date:		9/8/2005
//// Modified by:	D. XXX	
//// Updated date: Apr. 25, 2006
////---------------------------------------------------------------
//float XBaseMesh::Comput3DFaceArea(int fid)
//{
//	const XFace&f=GetF(fid);
//	Vec4 v1=GetV(f.p(1)).Pos()-GetV(f.p(0)).Pos();
//	Vec4 v2=GetV(f.p(2)).Pos()-GetV(f.p(0)).Pos();
//	float area=static_cast<float>(0.5*CrossVecX(v1,v2).Magnitude());
//
//	return area;
//}

//---------------------------------------------------------------
// Name:		GetDihedral(int eid)
// Description:	Calculate the angle of two triangles sharing an edge: eid
// Argument:	int eid - edge ID
// Return:		angle 
// Author:	    
// Date:		
// Modified by:	D. XXX
// Updated date: Apr. 25, 2006
//---------------------------------------------------------------- 
float XBaseMesh::GetDihedral(int eid)
{
	const XEdge& e=GetE(eid);
	float ang=0.;

	if(e.GetFIdx(0)==-1 || e.GetFIdx(1)==-1)
		return ang;

	const XFace& f1=GetF(e.GetFIdx(0));
	const XFace& f2=GetF(e.GetFIdx(1));
	float area1, area2;
	Vec4 norm1, norm2;

	norm1=CrossVecX(GetV(f1.p(1)).Pos()-GetV(f1.p(0)).Pos(),GetV(f1.p(2)).Pos()-GetV(f1.p(0)).Pos());	
	norm2=CrossVecX(GetV(f2.p(1)).Pos()-GetV(f2.p(0)).Pos(),GetV(f2.p(2)).Pos()-GetV(f2.p(0)).Pos());

	area1=norm1.Magnitude();
	area2=norm2.Magnitude();

	if(area1<1.0e-4 || area2<1.0e-4)
		return 0.;

	norm1=norm1.Normalize();
	norm2=norm2.Normalize();

	ang=GetAngleOf2Vector(norm1,norm2);

	return ang;
}

//---------------------------------------------------------------
// Name:		Comput3DAngOnVert(int fid, int vid)
// Description:	Calculate the angle of the triangle sharing a vertex
// Argument:	int fid - face ID
//				int vid - vertex ID 
// Return:		angle 
// Author:	    
// Date:		
// Modified by:	D. XXX - add comment
// Updated date: Mar. 04, 2006
//---------------------------------------------------------------- 
float XBaseMesh::Comput3DAngOnVert(int fid, int vid)
{
	const XFace& f=GetF(fid);
	int i;
	float ang=-1.0;
	//Vec4 v1, v2;
	for(i=0;i<3;i++)
	{
		if(f.p(i)==vid)
		{
			Vec4 v1=GetV(f.p((i+1)%3)).Pos()-GetV(f.p(i)).Pos();
			Vec4 v2=GetV(f.p((i+2)%3)).Pos()-GetV(f.p(i)).Pos();
			ang=GetAngleOf2Vector(v1,v2);
			break;
		}
	}
	return ang;
}

//---------------------------------------------------------------
// Name:		Comput3DCosOnVert(int fid, int vid)
// Description:	 Calculate the dot value of two vectors consisted of two edge of a face
// Argument:	int fid - face ID
//				int vid - vertex ID 
// Return:		dot value of two vectors 
// Author:	    
// Date:		
// Modified by:	D. XXX - add comment
// Updated date: Mar. 04, 2006
//---------------------------------------------------------------- 
float XBaseMesh::Comput3DCosOnVert(int fid, int vid)
{
	const XFace& f=GetF(fid);
	int i;
	float dot=1.0;
	//Vec4 v1, v2;
	float len1, len2;

	for(i=0;i<3;i++)
	{
		if(f.p(i)==vid)
		{
			Vec4 v1=GetV(f.p((i+1)%3)).Pos()-GetV(f.p(i)).Pos();
			Vec4 v2=GetV(f.p((i+2)%3)).Pos()-GetV(f.p(i)).Pos();
			len1=v1.Magnitude();
			len2=v2.Magnitude();
			if(len1<1.0e-6 || len2<1.0e-6)
			{
				return dot;
			}
			v1=v1/len1;
			v2=v2/len2;
			dot=Dot(v1,v2);
			return dot;
		}
	}

	return dot;
}

//---------------------------------------------------------------
// Name:		Comput3DAngOnVert(int vid)
// Description:	Calculate the total angle of triangles linking to a vertex
// Argument:	int vid: vertex index
// Return:		angle
// Author:	    
// Date:		
// Modified by:	D. XXX - add comment
// Updated date: Mar. 04, 2006
//---------------------------------------------------------------- 
float XBaseMesh::Comput3DAngOnVert(int vid)
{
	int i;
	const XVert& v=GetV(vid);
	float ang=0.;

	for(i=0;i<v.GetAdjFSize();i++)
	{
		ang+=Comput3DAngOnVert(v.AdjF(i),vid);
	}

	return ang;
}

//---------------------------------------------------------------
// Name:		GetAdjVOnWise(...)
// Description:	get another ID of edge on a vertex in the same face
// Argument:	inputFid: face index
//				inputVid: vertex index
//				inputEid: edge index
// Return:		edge index connecting to the vertex
// Author:		ljt
// Date:		9/9/2005
// Modified by:	D. XXX	
// Updated date: Apr. 25,2006
//---------------------------------------------------------------- 
int XBaseMesh::GetAnotherEdgeOnVert(int inputFid, int inputVid, int inputEid)
{
	int i;

	if(inputFid==-1)
		return -1;

	const XFace& f=GetF(inputFid);
	int eid;

	for(i=0;i<3;i++)
	{
		eid=f.GetEdgeRef(i);
		if(eid==inputEid)
			continue;

		const XEdge& e=GetE(eid);
		if(e.p(0)==inputVid || e.p(1)==inputVid	)
			return eid;
	}

	return -1;
}

//---------------------------------------------------------------
// Name:		GetAdjVOnWise(...)
// Description:	Order the sequence of all adjacent vertices of a vertex
// Argument:	vid: vertex index
// Return:		outputAdjVids: sequence of vertices
// Author:		ljt
// Date:		9/9/2005
// Modified by:	D. XXX	
// Updated date: Apr. 25,2006
//---------------------------------------------------------------- 
bool XBaseMesh::GetAdjVOnWise(int vid, varray<int>& outputAdjVids)
{
	const XVert&v=GetV(vid);
	int i, j;
	//int adjESize=v.GetAdjESize();
	int preFid=-1;
	int eid=-1;
	int fid;

	int iterNum=0;
	varray<int> eids;
	eid=v.AdjE(0);

	while(true)
	{
		eids.push_back(eid);	

		const XEdge& e=GetE(eid);

		for(j=0;j<2;j++)
		{
			fid=e.GetFIdx(j);

			if(fid!=preFid)
			{
				eid=GetAnotherEdgeOnVert(fid, vid, eid);
				preFid=fid;
				break;				
			}
		}

		if(eid==-1)
			break;

		if(eid==eids.front())
			break;

		iterNum++;
	}

	if(static_cast<int>(eids.size()) != v.GetAdjESize())
		return false;


	int adjVid;
	outputAdjVids.resize(eids.size(),-1);

	for(i=0;i<static_cast<int>(eids.size());i++)
	{
		const XEdge& e=GetE(eids.at(i));
		adjVid=e.p(0)!=vid? e.p(0):e.p(1);
		outputAdjVids.at(i)=adjVid;
	}

	return true;
}

//---------------------------------------------------------------
// Name:		 V_Edge_InTriangle
// Description:	 Compute the shortest distance from a point to an edge of a triangle
// Argument:	 null
// Return:		 void
// Author:		 XXX
// Date:		 9/8/2005
// Modified by:	 D. XXX	
// Updated date: Apr. 25, 2006		
//---------------------------------------------------------------- 
float XBaseMesh::V_Edge_InTriangle(Vec4 v, int inputFid, int& output_refEid)
{
	Vec4 map;
	float dis;
	float min=1.0e8;
	const XFace& f=GetF(inputFid);
	output_refEid=-1;
	//int sel;

	for(int i=0;i<3;i++)
	{
		const XEdge& e=GetE(f.GetEdgeRef(i));
		map = MapAVertexToASegment(v, GetV(e.p(0)).Pos(),GetV(e.p(1)).Pos());
		dis = (map-v).Magnitude();

		if(dis<min)
		{
			min=dis;
			output_refEid=f.GetEdgeRef(i);
			//sel=i;
		}
	}	

	return min;
}

//---------------------------------------------------------------
// Name:		VertReset
// Description:	Rotate a 2D pattern around z-axis with rotating center being Vec(0., 0., 0.,)
// Argument:	angle: value of rotating angle
// Return:		
// Author:		ljt
// Date:		9/8/2005
// Modified by:		
// Updated date:		
//---------------------------------------------------------------- 

void XBaseMesh::VertReset(float angle)
{
	int i;
	int vertSize=GetVSize();
	QMatrix4x4 GemMatrix;
	//Matrix4 GemMatrix;
	//GemMatrix.SetRotateZ(angle);	
	GemMatrix.rotate(angle, 0, 0, 1);
	for(i=0;i<vertSize;i++)
	{
		//GetV(i).Pos2d()=GetV(i).Pos2d()*GemMatrix;
		QVector3D a(GetV(i).Pos2d().x, GetV(i).Pos2d().y, GetV(i).Pos2d().z);
		a = a * GemMatrix;
		GetV(i).Pos2d().x = a.x();
		GetV(i).Pos2d().y = a.y();
		GetV(i).Pos2d().z = a.z();
	}
}

//---------------------------------------------------------------
// Name:		GetAdjFaceID
// Description:	Get Id of adjacent triangle of face(meshid) on edge(eid)
// Argument:	varray<bool>* faceVisited: visiting flag of triangles 
// Return:		if the adjacet triangle does not exist or has been visited, return -1
// Author:		ljt
// Date:		9/8/2005
// Modified by:		XXX(add comments)
// Updated date:	4/26/2006		
//---------------------------------------------------------------- 
int XBaseMesh::GetAdjFaceID(varray<bool>* faceVisited,int fid,int eid) //modified
{
	int size=GetE(eid).GetAdjFCount();
	for(int i=0;i<size;i++)
	{
		if((GetE(eid).GetFIdx(i)!=fid)&&(!faceVisited->at(GetE(eid).GetFIdx(i))))
		{
			return GetE(eid).GetFIdx(i);
		}
	}
	return -1;
}

//---------------------------------------------------------------
// Name:		Comp3DAngOnVertByLength()
// Description:	Calculate one angle of a face. The angle is corresponding to the vertex
// Argument:	fid: face ID
//				pid: vertex ID
// Return:		normal value
// Author:		ljt
// Date:		08/20/2006
// Modified by:	D. XXX	
// Updated date: Aug. 19, 2006
//---------------------------------------------------------------
float XBaseMesh::Comp3DAngOnVertByLength(int fid, int pid)
{
	int j;
	float la, lb, lc, val;
	float ang = 0;

	XFace& f=GetF(fid);

	//	float ang=GetAngleOf2Vector(GetV(f.p((i+1)%3)).Pos()-GetV(f.p(i)).Pos(),GetV(f.p((i+2)%3)).Pos()-GetV(f.p(i)).Pos());

	for(j=0; j<3; j++)
	{
		if(GetE(f.GetEdgeRef(j)).p(0)!=pid && GetE(f.GetEdgeRef(j)).p(1)!=pid)
		{
			lc = GetE(f.GetEdgeRef(j)).CurLen();
			la = GetE(f.GetEdgeRef((j+1)%3)).CurLen();
			lb = GetE(f.GetEdgeRef((j+2)%3)).CurLen();

			//^^^
			val = (la*la + lb*lb - lc*lc)/(2.*la*lb);

			if( (lc-la-lb)>0 || (la-lb-lc)>0 || (lb-la-lc)>0 || val>1)
			{
				return -1;
			}

			ang = (float)acos(val);
		}
	}

	return ang;
}


//---------------------------------------------------------------
// Name:		MoveSplineCptLeaveEdge
// Function:	move spline control pt leave a face'edge, mainly for neckline or armhole creation
// Argument:	varray<Vec4> cpts: spline control pt
//			varray<int> fids: spline control face
// Return:		
// Author:		XXX
// Date:		11-24-2006
// Modified by:		
// Updated date:	
//---------------------------------------------------------------- 
//bool XBaseMesh::MoveSplineCptLeaveEdge(varray<Vec4> &cpts, varray<int> &fids)
//{
//	if (cpts.size() != fids.size())
//	{
//		return false;
//	}
//
//	int i, j;
//	int cptCount = static_cast<int>(cpts.size());
//	////confirm the cpt in correspond triangle
//	for(i =0; i<cptCount; i++)
//	{
//		// judge if a cpt is on cf's plane
//		const XFace& face = GetF(fids.at(i));
//		Vec4 P[3];
//		for (j=0; j<3; j++)
//		{
//			P[j] = GetV(face.p(j)).Pos();
//		}
//
//		Vec4 fnorm = CrossVecX(P[2]-P[1], P[0]-P[1]).Normalize();// get the face normal
//		float dot = Dot((cpts.at(i) - P[0]).Normalize(), fnorm);
//		if (fabs(dot) > 1.0e-3) // not in same plane 
//		{
//			//int errorIdx = i;	
//		}
//		//assert(fabs(dot) < 5.0e-3); // not in same plane
//			
//		//test cpt in cf triangle or not, if a cpt is too close to cf's edge, move it leave edge 
//		Vec4 uvwTemp = GetBarycentCoorInATriangle(cpts.at(i), P[0], P[1], P[2]);
//
//		bool modifyFlag=false;
//		if (uvwTemp.x<1.0e-3f)
//		{
//			assert(uvwTemp.x > -1.0e-2f);
//			uvwTemp.x=1.0e-3f;
//			modifyFlag=true;
//		}
//		if (uvwTemp.y<1.0e-3f)
//		{
//			assert(uvwTemp.y > -1.0e-2f);
//			uvwTemp.y=1.0e-3f;
//			modifyFlag=true;
//		}
//		if (uvwTemp.z<1.0e-3f)
//		{
//			assert(uvwTemp.z > -1.0e-2f);
//			uvwTemp.z=1.0e-3f;
//			modifyFlag=true;
//		}
//
//		if (modifyFlag)
//		{
//			Vec4 uvw;
//			float sumCordTemp = uvwTemp.x+uvwTemp.y+uvwTemp.z;
//			uvw.x = uvwTemp.x/sumCordTemp;
//			uvw.y = uvwTemp.y/sumCordTemp;
//			uvw.z = uvwTemp.z/sumCordTemp;
//
//			Vec4 newCpt = uvw.x*P[0] + uvw.y*P[1] + uvw.z*P[2];
//			cpts.at(i) = newCpt;
//		}
//
//	}
//	return true;
//	
//}
//
//bool XBaseMesh::MoveSplineCptLeaveEdge(FitSpline& fitspline)
//{
//	if (fitspline.GetCtrlPointCount() != fitspline.GetCtrlFaceCount())
//	{
//		return false;
//	}
//
//	int i, j;
//	int cptCount = fitspline.GetCtrlPointCount();
//	////confirm the cpt in correspond triangle
//	for(i =0; i<cptCount; i++)
//	{
//		// judge if a cpt is on cf's plane
//		const XFace& face = GetF(fitspline.GetCtrlFaceIdx(i));
//		Vec4 P[3];
//		for (j=0; j<3; j++)
//		{
//			P[j] = GetV(face.p(j)).Pos();
//		}
//
//		Vec4 fnorm = CrossVecX(P[2]-P[1], P[0]-P[1]).Normalize();// get the face normal
//		float dot = Dot((fitspline.GetCtrlPoint(i) - P[0]).Normalize(), fnorm);
//		if (fabs(dot) > 1.0e-3) // not in same plane 
//		{
//			fitspline.GetCtrlPoint(i) = GetMappingPointOnAPlane(fitspline.GetCtrlPoint(i), P[0], P[1], P[2]);
//		}
//
//		//test cpt in cf triangle or not, if a cpt is too close to cf's edge, move it leave edge 
//		Vec4 uvwTemp = GetBarycentCoorInATriangle(fitspline.GetCtrlPoint(i), P[0], P[1], P[2]);
//
//		bool modifyFlag=false;
//		if (uvwTemp.x<1.0e-3f)
//		{
//			assert(uvwTemp.x > -1.0e-2f);
//			uvwTemp.x=1.0e-3f;
//			modifyFlag=true;
//		}
//		if (uvwTemp.y<1.0e-3f)
//		{
//			assert(uvwTemp.y > -1.0e-2f);
//			uvwTemp.y=1.0e-3f;
//			modifyFlag=true;
//		}
//		if (uvwTemp.z<1.0e-3f)
//		{
//			assert(uvwTemp.z > -1.0e-2f);
//			uvwTemp.z=1.0e-3f;
//			modifyFlag=true;
//		}
//
//		if (modifyFlag)
//		{
//			Vec4 uvw;
//			float sumCordTemp = uvwTemp.x+uvwTemp.y+uvwTemp.z;
//			uvw.x = uvwTemp.x/sumCordTemp;
//			uvw.y = uvwTemp.y/sumCordTemp;
//			uvw.z = uvwTemp.z/sumCordTemp;
//
//			Vec4 newCpt = uvw.x*P[0] + uvw.y*P[1] + uvw.z*P[2];
//			fitspline.SetCtrlPoint(i, newCpt);
//		}
//	}
//	return true;
//}

//---------------------------------------------------------------
// Name:		FindBLoop
// Function:	Find boundary loops of mesh, return loop vids and eids
// Argument:	BLoopVids, BLoopEids
// Return:		
// Author:		XXX
// Date:		4-27-2007
// Modified by:		
// Updated date:	
//---------------------------------------------------------------- 
bool XBaseMesh::FindBLoop(varray<varray<int > >& BLoopVids, varray<varray<int > >& BLoopEids)
{
	if(!m_BoundFlagSet) 
	{
		BoundaryFlagSetting();
	}

	int eSize = GetESize();
	int i, j, k, sEid, sVid, lastVid;

	varray<bool> eVisit;
	eVisit.resize(eSize, false);

	for(i=0; i<eSize; i++)
	{
		if(!GetE(i).IsBoundary())
			eVisit[i] = true;
	}

	while(true)
	{
		sEid = -1;
		for(i=0; i<eSize; i++)
		{
			if(!GetE(i).IsBoundary() || eVisit[i])
				continue;

			sEid = i;
			eVisit[sEid] = true;
			break;
		}

		if(sEid == -1)
			break;

		varray<int> BLoopV;
		varray<int> BLoopE;
		sVid = -1;

		for(j=0; j<2; j++)
		{
			for(k=0; k<GetV(GetE(sEid).p(j)).GetAdjESize(); k++)
			{
				if(GetE(GetV(GetE(sEid).p(j)).AdjE(k)).IsBoundary() && 
					(!eVisit[GetV(GetE(sEid).p(j)).AdjE(k)]))
				{
					sVid = GetE(sEid).p(j);
					break;
				}
			}

			if(sVid != -1)
				break;
		}	

		if(sVid == -1)
		{
			break;
		}

		BLoopV.push_back(sVid);
		while(true)
		{
			lastVid = sVid;
			for(j=0; j<GetV(sVid).GetAdjESize(); j++)
			{
				sEid = GetV(sVid).AdjE(j);
				if(eVisit[sEid] || !GetE(sEid).IsBoundary())
					continue;

				eVisit[sEid] = true;
				BLoopE.push_back(sEid);

				if(GetE(sEid).p(0) != sVid)
					sVid = GetE(sEid).p(0);
				else
					sVid = GetE(sEid).p(1);

				break;
			}

			if(lastVid == sVid)
			{
				break;
			}
			else
			{
				BLoopV.push_back(sVid);
			}
		}

		BLoopE.push_back(GetEdgeIdBy(BLoopV.front(), BLoopV.back()));
		assert( BLoopV.size() == BLoopE.size() );
		BLoopVids.push_back(BLoopV);
		BLoopEids.push_back(BLoopE);
	}

	return true;

}

//---------------------------------------------------------------
// Name:	     SetAdjPtIdxOfEdge( )
// Description:  Set two points on two triangles sharing the edge but not on the edge for each edge
// Argument:	null
// Return:		void
// Author:		 
// Date:		 
// Modified by:	 D. XXX
// Updated date: Aug. 30, 2005
//----------------------------------------------------------------
void XBaseMesh::SetAdjPtIdxOfEdge(void)
{
	int i, k, m;

	for(i=0; i<GetESize(); i++)
	{
		XEdge& e = GetE(i);

		e.GetAdjPtIdx(0) = -1;
		e.GetAdjPtIdx(1) = -1;

		for(k=0; k<2; k++)
		{
			if(e.GetFIdx(k)==-1)  // the edge has only one adjacent face
				continue;

			XFace& f0 = GetF(e.GetFIdx(k));

			for(m=0; m<3; m++)
			{
				if(f0.p(m) != e.p(0) && f0.p(m) != e.p(1) )
				{
					e.GetAdjPtIdx(k) = f0.p(m);
				}
			}
		}

		if(e.GetAdjPtIdx(0) == -1 || e.GetAdjPtIdx(1) == -1)
			e.IsBoundary() = true;
		else
			e.IsBoundary() = false;
	}
}
void XBaseMesh::OrgLenInCrossingSpringInitializing()
{
	int i;
	int eSize=GetESize();
	for(i=0;i<eSize;i++)
	{	
		GetE(i).GetCrossLen()=-1.0;	
	}
	int count=0;
	m_averageCrossingLen=0.0;
	bool flag;
	for(i=0;i<eSize;i++)
	{
		if(GetE(i).IsBoundary()){continue;}
		GetE(i).GetCrossLen()=GetOrgLenInCrossingSpring(i,flag);
		if(!flag)
		{
			continue;
		}
		m_averageCrossingLen+=GetE(i).GetCrossLen();
		count++;
	}
	m_averageCrossingLen=m_averageCrossingLen/count;
	m_crossLenCompuFlag=true;
}

//---------------------------------------------------------------
// Name:	     DivideSubMeshPatches
// Description:  
// Argument:	int& meshnum - output the submesh number
// Return:		
// Author:		XXX 
// Date:		Jul. 4th, 2007 
// Modified by:	 
// Updated date: 
//----------------------------------------------------------------
XBaseMesh** XBaseMesh::DivideSubMeshPatches(int& meshnum)
{
	int *vstack;
	int ns, nv, np;
	XEdge ed;
	vstack = new int[m_vert.size()];
	XBaseMesh* pmesh;
	varray<XBaseMesh*> vMeshPointer;
	varray<XVert> vVert;
	varray<XEdge> vEdge;
	varray<XFace> vFace;
	varray<int> vVertIndex;
	varray<int> vFaceIndex;
	varray<bool> vFaceFlag;
	vFaceFlag.resize(m_face.size());
	//vVert.resize(m_vert.size());
	//vFace.resize(m_face.size());
	int iVertSize = 0, iFaceSize = 0;
	meshnum = 0;
	XBaseMesh** submesh = NULL;
	
	for(int i=0; i< m_vert.size(); i++)
	{
		m_vert[i].m_mark = false;
	}

	for(int i=0; i< m_face.size(); i++)
	{
		vFaceFlag.at(i) = false;
	}

	for(int i=0; i< m_vert.size(); i++)
	{	
		// Already collected by some submesh
		if(m_vert[i].m_mark)
			continue;

		// Is the seed point for one submesh
		vVert.clear();
		vEdge.clear();
		vFace.clear();
		vVertIndex.clear();
		vFaceIndex.clear();

		nv = i;
		ns = 1;	
		vstack[ns-1] = nv;
		m_vert[nv].m_mark = true;
		vVert.push_back(m_vert[nv]);
		vVertIndex.push_back(nv);
		
		do
		{
			ns--;
			nv = vstack[ns];
			for(int j=0; j< static_cast<int>(m_vert[nv].GetAdjESize()); j++)
			{
				ed =  m_edge[ m_vert[nv].AdjE(j)];
				if(ed.p(0) == nv) 
					np = ed.p(1);
				else
					np = ed.p(0);

				if(!m_vert[np].m_mark)
				{
					vVert.push_back(m_vert[np]);
					vVertIndex.push_back(np);
					m_vert[np].m_mark = true;
					vstack[ns] = np;
					ns++;
				}
			}
		}while(ns > 0);

		iVertSize = static_cast<int>(vVert.size());
		if(iVertSize < 3)
			continue;

		// Collect faces
		for(int j=0; j<iVertSize; j++)
		{
			const XVert& vert = vVert.at(j);
			for(int k=0; k<static_cast<int>(vert.GetAdjFSize()); k++)
			{
				if(!vFaceFlag.at(vert.AdjF(k)))
				{
					vFaceFlag.at(vert.AdjF(k)) = true;
					vFace.push_back(GetF(vert.AdjF(k)));
					//vFaceIndex.push_back(vert.AdjF(k));
				}
			}
		}

		iFaceSize = static_cast<int>(vFace.size());
		if(iFaceSize < 1)
			continue;
		
		// Add verts and faces to mesh
		pmesh = new XBaseMesh;
		pmesh->m_vert.resize(iVertSize);
		pmesh->m_face.resize(iFaceSize);

		for(int i=0; i<iVertSize; i++)
		{
			pmesh->m_vert.at(i).Pos() = vVert.at(i).Pos();
			if(vVert.at(i).IsOnBoundary())
				pmesh->m_vert.at(i).m_flag = true;
			else
				pmesh->m_vert.at(i).m_flag = false;
		}

		for(int i=0; i<iFaceSize; i++)
		{
			pmesh->m_face.at(i).p(0) = GetInTheVarrayIndex(vFace.at(i).p(0), vVertIndex);
			pmesh->m_face.at(i).p(1) = GetInTheVarrayIndex(vFace.at(i).p(1), vVertIndex);
			pmesh->m_face.at(i).p(2) = GetInTheVarrayIndex(vFace.at(i).p(2), vVertIndex);
		}

		vMeshPointer.push_back(pmesh);
		meshnum++;

	}

	submesh = new XBaseMesh* [meshnum];

	for(int i=0; i<meshnum; i++)
	{
		submesh[i] = vMeshPointer.at(i);
	}

	delete [] vstack;

	return submesh;
}


float XBaseMesh::GetAngleEdge(int eid)
{
	if(eid < 0 || eid >= GetESize())
		return 0;
	XEdge& edge = GetE(eid);
	if(edge.GetAdjFCount() != 2)
		return 0;
	XFace& face1 = GetF(edge.GetFIdx(0));
	XFace& face2 = GetF(edge.GetFIdx(1));
	float angle2Face = GetAngleOf2Vector(face1.Norm(),face2.Norm());
	return angle2Face;
}

void XBaseMesh::SetAdjacentPt()
{
	int centreIdx, neighbourIdx;
	int vnum = GetVSize();
	int esize;
	for(int i=0; i<vnum; i++)
	{
		XVert& vert = GetV(i);
		vert.AdjPClear();
		esize = vert.GetAdjESize();
		centreIdx = i;
		for(int j=0; j<esize; j++)
		{
			const XEdge& edge = GetE(vert.AdjE(j));
			neighbourIdx = edge.p(0);
			if(neighbourIdx == centreIdx)
				neighbourIdx = edge.p(1);
			vert.AddAdjP(neighbourIdx);
		}
	}	
};
//目前只处理模型本身就是封闭的情况，对于模型本身不封闭的情况，目前暂不处理。
//以下函数有问题。
void XBaseMesh::FixMeshConversionFromCADModel(bool IsModelClosed)
{
	if(!IsModelClosed)
		return;
	InitEdgeLen();
	//首先寻找变界边。
	BoundaryFlagSetting();
	int pcount = 0;
	for(int i=0; i<GetVSize(); i++)
	{
		if(GetV(i).IsOnBoundary())
		{
			pcount++;
		}
	}
	if(pcount == 0)
	{
		return;
	}
	//寻找边界点
	int vcount = 0;
	int i;
	float avelen = GetAverageELen();
	varray<int> boundaryVIdx;
	int hasnoNearptVIdx = -1;
	int itertime = 0;
	do 
	{
		itertime++;
		boundaryVIdx.clear();
		for(i=0; i<GetVSize(); i++)
		{
			if(GetV(i).IsOnBoundary())
			{
				boundaryVIdx.push_back(i);
			}
		}
		vcount =  boundaryVIdx.size();
		if(vcount <= 1)
			break;
		int startidx = -1;
		int max_link_edge_num = 12;
		for(int i=0; i<boundaryVIdx.size(); i++)
		{
			if(GetV(boundaryVIdx.at(i)).GetAdjPSize() > max_link_edge_num ||  boundaryVIdx.at(i) == hasnoNearptVIdx)
				continue;
			else
			{
				startidx = i;
				break;
			}
		}
		if(startidx == -1)
			break;
		XVert& vt1 = GetV(boundaryVIdx.at(startidx));
        int nearIdx = -1;
		float minlen = 100000;
		for(int i=0; i<vcount; i++)
		{
			bool bCanConnect = true;
			if(i == startidx)
				continue; 
			if(GetV(boundaryVIdx.at(i)).GetAdjPSize() > max_link_edge_num)
				continue;
			for(int j=0;j<vt1.GetAdjPSize();j++)
			{
				if(boundaryVIdx.at(i) == vt1.AdjP(j))
				{
					bCanConnect = false;
					break;
				}
			}
			if(!bCanConnect)
				continue;
			float templen = (vt1.Pos() - GetV(boundaryVIdx.at(i)).Pos()).GetLength();
			if(templen < minlen && templen < 2*avelen)
			{
				minlen = templen;
				nearIdx = i;
			}
		}
		if(nearIdx == -1)
		{
			hasnoNearptVIdx = boundaryVIdx.at(startidx);
			continue;
		}
		else
		{
			hasnoNearptVIdx = -1;
		}
		XVert vt2 = GetV(boundaryVIdx.at(nearIdx));
		bool canDelcurrentVert = true;
        if(vt1.GetAdjPSize() == vt2.GetAdjPSize())
		{
			if(rand() % 2 == 1)
			{
				canDelcurrentVert = false;
			}
		}
		else if(vt1.GetAdjPSize() < vt2.GetAdjPSize())
		{
			canDelcurrentVert = false;
		}
        if(!canDelcurrentVert)
		{
            MergeVertByIdx(boundaryVIdx.at(startidx),boundaryVIdx.at(nearIdx));
		}
		else
		{
			MergeVertByIdx(boundaryVIdx.at(nearIdx),boundaryVIdx.at(startidx));
		}
	} while(vcount > 1 && itertime < pcount);
	//重建边。
	MakeXEdges();	
	BoundaryFlagSetting();
	boundaryVIdx.clear();
	for(i=0; i<GetVSize(); i++)
	{
		if(GetV(i).IsOnBoundary())
		{
			boundaryVIdx.push_back(i);
		}
	}
	/*String str;
	str.Format(_T("还剩%d个边界点"),boundaryVIdx.size());
	AfxMessageBox(str);*/
};
void   XBaseMesh::MergeVertByIdx(int Mergeidx,int removeIdx)
{
	if(removeIdx < 0 || removeIdx >= GetVSize() || Mergeidx < 0 || Mergeidx >= GetVSize())
		return;

	//首先删除点
	m_vert.erase(m_vert.begin()+removeIdx);

	//然后重置面中点的编号。
	int fsz = GetFSize();
	for(int i=0; i<fsz; i++)
	{
		for(int j=0; j<3; j++)
		{
			if(GetF(i).GetIndex(j) == removeIdx)
			{
				GetF(i).p(j) = Mergeidx;
				break;
			}
		}
	}
	//由于点的编号已更改，因此面中点的id也要更改。
	for(int i=0; i<fsz; i++)
	{
		for(int j=0; j<3; j++)
		{
			if( GetF(i).GetIndex(j) > removeIdx)
			{
				GetF(i).p(j) -= 1;
			}
		}
	}
	//另外边中的id也要更改。
	/*for(int i=0; i<GetESize(); i++)
	{
		for(int j=0; j<2; j++)
		{
			if(GetE(i).GetIndex(j) > removeIdx)
			{
				GetE(i).p(j) -= 1;
			}
		}
	}*/
	MakeXEdges();
	BoundaryFlagSetting();
};
}
