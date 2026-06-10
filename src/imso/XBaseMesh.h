/********************************************************************
* Copyright (c) 2006，XXX, 
* All rights reserved.

* Filename：XBaseMesh.h
* File description：Define the class XBaseMesh: basic class of mesh surface

* Version：		1.0
* Author：		D. XXX
* Date：		Jan. 26, 2004
* Modified by： 
* Updated date：
********************************************************************/


#if !defined(__XBASEMESH_H_INCLUDED)
#define __XBASEMESH_H_INCLUDED


//#include "base.h"
#include "XData.h"
//#include "XMatrixMath.h"//目前缺失

//#include "XBaseMeshSubData.h"//目前缺失
//#include "FitSpline.h"//缺失
#include "FittingBSpline.h"
//#include "CutLine.h"//缺失
//#include "Scene.h"//缺失


enum CURVATURE{
     Maxmum_pt = 0,
	 Minmum_pt = 1,
	 Saddle_pt = 2,
	 Upsaddle_pt = 3,
	 Downsaddle_pt = 4,
	 Monkey_pt = 5,
	 Source_pt = 6
};

//................end add...................../*/

#pragma once

namespace base {


	const int  XBASEMESH_CLASS_ID = 0x5fe269e5; //???
    class XVert;
	class XEdge;
	class XFace;

	class CriPoints
	{
	public:
		CriPoints();
		~CriPoints();
		CriPoints& operator =(const CriPoints& cpt);
	public:
		Vec4 pt;
		int  vidx;
		int  type;   //0 for max, 1 for min, 2 for max-saddle, 3 for min-saddle.
		float candelRatio; //true for the point can be deleted, false for the point can not be deleted.
		//bool  bVisited;
		int   saddlestatus;  //0 for saddle to saddle, 1 for linking to minimum point,-1 for linking to maximum point.
		bool  bDelflag;
	public:
		bool  IsMinMaxPt(){ return type == Minmum_pt || type == Maxmum_pt;};
		bool  IsSaddlePt(){return type == Saddle_pt || type == Upsaddle_pt || type == Downsaddle_pt || type == Monkey_pt;};
	} ;

	class  XBaseMesh
	{
	public:
		XBaseMesh(void);
		~XBaseMesh(void);

	public:
		varray<XVert>	m_vert;   // vertex
		varray<XEdge>	m_edge;   // edge
		varray<XFace>	m_face;   // triangle
		int				m_iMeshID;  //mesh id, XXX add for adding multi obj file, 20060928

		bool			m_oriEdgeLenInitedFlag;// whether the mesh 3d edge length init.
		float			m_oriAverageLen; // the 3d edge average length
		bool			m_bAverageLenthInitFlag; // whether the average 3D length init.
		bool			m_BoundFlagSet;// whether the edge, vertex, and face boundary flag set or not

		bool			m_AngleInited;// whether 3d face angle initialized, use AngleInit() to initialize.
		float			m_averageCrossingLen; // the average length of crossing spring
		bool			m_crossLenCompuFlag; //the flag whether the length of the crossing spring has been computed or not
		bool			m_bGaussCurInited;

		// Variables from HMesh
		Vec4			minp,maxp;
		Vec4			m_trans;        
		float			m_scale;
		Vec4			m_maxMeshBox3D;
		Vec4			m_minMeshBox3D;
		int             m_iModelMainOrientation;

	private:
		float			m_smooth_angle;  	//Normal Management
		Vec4			m_meshRect; // the bounding box of the XBaseMesh in 2d
		bool			m_bRectExisted; // whether bounding box of mesh in 2d exist.
		//bool			m_bRectExisted3D;
		Vec4			m_meshCentre; // the centre position of mesh in 2d
		bool			m_bCentreExisted; // whether mesh centre in 2d exist

		bool			m_bMeshBox3DComput;


	protected:
		bool			m_bEdgeLenCompFlag;

	public:
		// get the class ID of XBaseMesh
		//DWORD			ClassID() const {return  XBASEMESH_CLASS_ID;};

		// clear vertexes, faces, and edges.
		void			clear();

		// set vertex size
		void			SetVSize(int i)   { m_vert.resize(i);  }; // set the number of vertex

		// get vertex size
		int				GetVSize() const  { return static_cast<int>(m_vert.size()); }; // get the number of vertex

		// set edge size
		void			SetESize(int i)   { m_edge.resize(i);     }; // set the number of edge

		// get edge size
		int				GetESize()  { return static_cast<int>(m_edge.size()); };   // get the number of edge
		int				GetESize() const { return static_cast<int>(m_edge.size()); };   // get the number of edge

		// set face size
		void			SetFSize(int i)   { m_face.resize(i);     }; // set the number of face

		// get face size
		int				GetFSize()  { return static_cast<int>(m_face.size()); };   // get the number of face
		int				GetFSize() const { return static_cast<int>(m_face.size()); };   // get the number of face


		XVert&			GetV(int i) { return m_vert[i];} //get a vertex
		XEdge&			GetE(int i) { return m_edge[i];} //get a edge
		XFace&			GetF(int i) { return m_face[i];} //get a face

		const XVert&	GetV(int i) const { return m_vert[i];} //get a vertex
		const XEdge&	GetE(int i) const { return m_edge[i];} //get a edge
		const XFace&	GetF(int i) const { return m_face[i];} //get a face

		

		// set the smooth angle of the mesh
		void			SetSmoothAngle(float angle) {	m_smooth_angle = angle; }
		// get the smooth angle of the mesh
		float			GetSmoothAngle() const {	return m_smooth_angle; }

		bool			HasBoundaryFlagSet()    {return m_BoundFlagSet;};

		bool&			IsAngleInited()         {return m_AngleInited;};

		bool			IsCrossLenComp()        {return m_crossLenCompuFlag;};

		bool			IsEdgeLenComp()         {return m_bEdgeLenCompFlag;};

		void			ResetRect()				{m_bRectExisted = false;};

		void			ResetCentre()			{m_bCentreExisted = false;};

		float&			GetScale()		{ return m_scale; }
	public:
		// show the mesh using openGL.
		/*void			RenderMeshShade(const COLORREF& meshColor,float fLineWidth);
		void			RenderMeshTransparent(const COLORREF& meshColor,float fLineWidth);
		void			RenderMeshWire(const COLORREF& meshColor,float fLineWidth);
		void			RenderMeshData();
		void			RenderMeshVerts(const COLORREF& ptsColor,float fPointSize);*/
	public:
		void			ConvertToMesh(class Mesh &m)   {assert(&m);/*Convert2Mesh(m);*/};// will delete some time later.

		// get the data from XBaseMesh object.
		void			GetDataFromXBaseMesh(const XBaseMesh& baseMs);

		void		    BoundaryFlagSetting(int strPntIdx = 0, int strFaceIdx = 0, int strEdgeIdx = 0); // set the boundary flag of each vertex , edge and traingle.

		// Find boundary loops of mesh, return loop vids and eids, added by XXX 2007-04-27 
		bool			FindBLoop(varray<varray<int > >&  BLoopVids, varray<varray<int > >&  BLoopEids);

		//bool			Save(BaseOStream& os, Scene& scene) const;
		//bool			Load(BaseIStream& is, Scene& scene);

		//bool			Save_for_oriMesh(BaseOStream& os, Scene& scene) const;
		//bool			Load_for_oriMesh(BaseIStream& is, Scene& scene);

		bool			Save0010_for_oriMesh(BaseOStream& os) const;
		//bool			Load0010_for_oriMesh(BaseIStream& is,float fCurVersion);

		//according to the face, vertex and their relationship, create the edges and related relationship between face vertex and edge.
		void			MakeXEdges();					// generate edge information 

		// For a 3D model, whose normal direction points to inside, change its normal.
		void			ChangeNorm(float yTopRef = 1.0e6, float yBotRef = -1.0e6);

		//	void			Convert2Mesh(Mesh& ms_to) const;  // convert XBaseMesh to Mesh

		void			ComputeNormals(bool bIsHaveToCal = false);    // compute the normal of vertices and faces

		Vec4			ComputeFaceNormal(int i); // compute the normal of a face

		void			CopyMesh(XBaseMesh& xms_to);    // copy a mesh

		void			InitEdgeLen();      // compute the lengths of all edges
		void			AverageLenCalculation(); // compute 3d average length.
		float&			GetAverageELen()        {return m_oriAverageLen;}; // return or set the average length of the mesh.

		//  The following five functions are for mesh simplifcation
		//void			Swapping();            // mesh refinement through edge swapping
		//bool			Simplification(XBaseMesh& newMs, int numNewVert, bool IsMergeBoundary/*, bool SimplifyCallBack(double)*/); // simplification
		//void			ReorganizeMesh();  // remove unnecessary vertices, edges and faces after the mesh is simplified

		// get the intersecting lines between a plane and a mesh
		//bool			IntersectPlane(Vec4 startpt, int numf, Vec4 pnorm, Vec4* plist,int* pedges, int& pnum, bool IsConvex) const;
		//bool			IntersectPlane(Vec4 pt, Vec4 pnorm, Vec4 *plist, int &pnum, int nid);
		//bool			IntersectPlane(Vec4 pt, Vec4 pnorm, Vec4 *plist, int &pnum, int nid,int *edge_n);
		//bool			IntersectPlane(Vec4 pt, Vec4 norm, Vec4 *ptlist, int &ptnum, int *edgeid)const; //add by wangmei 2005.2.1
		//bool			IntersectPlane(Vec4 pt, Vec4 norm,varray<Vec4>& ptsArr, varray<int>& edgeidArr)const;//XXX 

		//bool			IntersectPlane_2D(Vec4 startpt, int numf, Vec4 pnorm, varray<Vec4>& plist,varray<int>& pedges)const;
		// following three functions are from HMesh
		bool			IntersectPlane(Vec4 startpt, int numf, Vec4 pnorm, Vec4 *plist, int &pnum, int *nedge, float *eratio, bool IsConvex);
		// Compute the polygons by intersecting the mesh and a plane going through a point- results maybe several closed polygons	
		bool			IntersectPlane(Vec4 pt, Vec4 pnorm, Vec4 *plist, int &pnum);
		//Compute the polygons by intersecting HMesh of current part and a plane, the polygon may be not closed.	
		bool			IntersectPlane(Vec4 pt, Vec4 pnorm, Vec4 *plist, int &pnum, int nid, int iDir);

		//bool			IntersectPlaneMultiLoop(Vec4 pt, Vec4 pnorm, varray<Vec4>& plist) const;
		// intersect a plane with the given length
		//bool			InterPlaneWithLen(const Vec4 &mousept, int numf, const Vec4 &pnorm, const Vec4 arrowDir, float inputLineLen, varray<Vec4>& plist, varray<int>& pEdges,varray<int>& pFaces, bool bAlways = false) const;
		//bool			InterPlaneWithLen_2D(const Vec4 &mousept, int numf, const Vec4 &pnorm, const Vec4 arrowDir, float inputLineLen, varray<Vec4>& plist, varray<int>& pEdges,varray<int>& pFaces, bool bAlways = false) const;
		// intersect a plane with the given two control point(a little like CLine)
		//bool			InterPlaneWith2P(const Vec4 &sP, int sfid, const Vec4& eP, int eFid, varray<Vec4>& plist, varray<int>& pEdges,varray<int>& pFaces, bool planeNormExist = false, Vec4 p_Norm = Vec4(0.,0.,0.)) const;
		//bool			InterPlaneWith2P_2D(const Vec4 &sP, int sfid, const Vec4& eP, int eFid, varray<Vec4>& plist, varray<int>& pEdges,varray<int>& pFaces, bool planeNormExist = false, Vec4 p_Norm = Vec4(0.,0.,0.)) const;

		//int				GetFaceIdxByPt(int edgeoneidx, int edgetwoidx,Vec4 PtInput)const;
		int				GetIntersectedFace(int ncurf, const Vec4& p1, const Vec4& p2); // add by wj 2004-6-4
		Vec4			Intersect_Line(const Vec4& v1,const Vec4& v2,int faceID);    // add by wj 2004-6-4
		bool			IsPtInside(const Vec4& pt, int &ne, int faceID);

		// temp commented by XXX [8/14/2006] the fucntion need to improve the number of intersections.
		//bool			GetCurve2P( Vec4 &startpt, int numf1, Vec4 &endpt, int numf2, Vec4 *plist, int &pnum, bool IsConvex);

		//float			Angc(float xr,float yr,float x,float y) const;
		Vec4			GetIntersectPlaneNorm(const Vec4 &stpt, int stfaceidx, const Vec4 &endpt, int endfaceidx,int iSpecialType = 0)const;
		int				GetFaceIdxBy(int edgeoneidx, int edgetwoidx)const;
		int				GetFaceIdxBy(int vid1,int vid2,int vid3)const ;
		void			ChangeModelNorm();

		// get the areal coordinates of the point in a triangle.
		void			GetBarycentCoorIn3D(const Vec4& P, int fid, float& u, float& v, float& w) const;
		void			GetBarycentCoorIn2D(const Vec4& P, int fid, float& u, float& v, float& w) const;

		// according to the areal coordiantes, calculate the position in the triangle.
		Vec4			GetVertFromBarycentCoorIn3D(int fid, float uvw[3]) const;
		Vec4			GetVertFromBarycentCoorIn2D(int fid, float uvw[3]) const;

		Vec4			GetVertFromBarycentCoorIn3D(int fid, const Vec4& uvw) const;
		Vec4			GetVertFromBarycentCoorIn2D(int fid, const Vec4& uvw) const;
		//int				GetFaceIdxBy3P(int pidx1, int pidx2,int pidx3)const;


		// get angle in 3d in a triangle.
		float			GetOriAngle(int fid,int pid); // return a original current internal angle

		//compute original angle(3D) on the vertex based on the length of edges - specailly for creating multilevel mesh
		float			Comp3DAngOnVertByLength(int fid, int pid);

		//comupte the corss-spring between two triangles	
		float			GetOrgLenInCrossingSpring(int eid, bool& flag);

		// get ordered points on the mesh by intersect a plane.( threshold = 1.0e-4: the point at least has a distance of 1.0e-4 from it self to the mesh vertex.) 
		//bool			GetOrderedIntersectPts(Vec4 pointOnPlane, Vec4 planeNorm, varray<Vec4>& intersections, varray<int>& interEids,bool& bCircled);

		//calculate normal of mesh in 2d
		void			InitNormal2D();

		// compute the normal of a face in 2D plane
		int				ComputeFaceNormalIn2D(int i); 

		// init the angle of a face.
		void			AngleInitInAFace(int fid);// initializing the internal angles of a triangle

		// init the angle of all the faces.
		void			AngleInit(); // initializing the internal angles of each triangle

		// get 3d angle on the vertex.
		float			GetOriAngleOnVert(int fid,int pid); //return a origenal internal angle

		// get 2 faces' shared edge
		int				GetSharedEdge(const int currFid, const int adjFid)const; // get the id of an edge shared by two triangles

		// add a XBase mesh to the current mesh
		void			AddMesh(const XBaseMesh& m);
		// get a adjacent triangle which shares a edge with the current triangle
		//if there doesn't exist a adjacent face, just return -1
		int				GetAdjFid(int fid, int eid); 

		// get two edges which share the vertex in the triangle
		void			GetLinkedEidInAFace(int fid, int pid, int& eid1, int& eid2);

		//get the vertex index other than the edge vertexes on the face
		int				GetTirVid(int fid, int eid);

		//get the vertex index by two vertex index
		int				GetFaceAnotherVert(int onevert, int twovert,bool boundaryflag = false);//XXX 2002-10-22

		//get the other vertex of their shared edge and the vertex opposite to the shared edge in adjacent triangle
		void			GetTirVid(int baseFid, int adjFid, int convergedVid,int& baseTirVNO,int& baseTirVid,int& adjTirVid);

		//test if the two point on same mesh
		bool			IsTwoPtsAtSameArea(int stfaceidx, const Vec4 &stpt, int endfaceidx, const Vec4 &endpt);

		//get edge index by two vertex indexes
		int				GetEdgeIdBy(int vid1, int vid2)const;

		//get boundary edge connect to the given vertex
		bool			GetBoundEidOnVert(int vid, int& resEid1, int& resEid2);

		// search the boundary loop of a surface 
		//int				GetLoopNum(void);   //XXX 2002-9
		int				GetLoopNum(varray<int>& vids, int strPntIdx = 0 ,int strEdgeIdx = 0, int strFaceIdx = 0);

		// get the ordered intersect edges and faces(for boundary lines)
		bool			GetEidOnwise(int vid, int startEid, varray<int>& eids,varray<int>& fids); //2004-07-14-Li

		// get the edge ids by the face and a vertex in the face
		void			GetEidByVertexAndFace(int vid, int fid, varray<int>& eids)const;

		// get 2d angle of the face 
		float			Get2DAngleInFace(int fid,int pid);

		//get 3d angle of the face
		float			Get3DAngleInFace(int fid,int pid);

		//return the sum of angle at a point in 3D
		float			Get3DTotalAngleAtPoint(int vid); 

		// get 2d area of the mesh
		float			Get2DArea()  ;//XXX 2003-01-21

		// get 3d area of the mesh
		float			Get3DArea()  ;//XXX 2003-01-21

		// get 3d area of a face.
		float			Get3DFaceArea(int fid) ;

		// get 2d area of a face.
		float			Get2DFaceArea(int fid) ;

		//test if the edge belong to the face
		bool			IsEdgeOnFace(int eid, int fid);

		//compute the total angles around a vertex
		float			GetTotalAngleOnVert(int vid);//2003-09-05
		float			Compute2DTotalAngleAtPoint(int pid); 

		//Get the bounding box of the mesh in 2d
		Vec4			GetRect(); //XXX 2003-03-25 

		//Get the bounding box of the mesh in 3d
		void			GetRect3D(Vec4& xyzMax, Vec4& xyzMin); // added by XXX [6/12/2006]

		//Get the centre position of the mesh in 2d
		Vec4			GetMeshCenter(); //XXX 2003-04-07 

		//Get the centre position of the mesh in 3d
		Vec4			GetMeshCenter3D();

		//Get the centre position of the mesh box;
		Vec4			GetMeshBoxCenter3D();

		//get the edge length
		float			GetELength(int i);	//XXX

		// get the intersection on the edge of two point on two adjacent face.
		bool			IntersectionBetweenLineAndFaceIn3D(const Vec4& sV, const Vec4& eV, int sFid, int eFid, Vec4& intersection,int& insectingEid);
		bool			IntersectionBetweenLineAndFaceIn2D(const Vec4& sV, const Vec4& eV, int sFid, int eFid, Vec4& intersection, int& intersectingEid);


		// if a point is in the triangle in 2d
		bool			IsPointInTriangleIn2D(const Vec4& testingP, int fid); 

		// if the two face adjacent
		bool			IsAdjacent(int iFace1,int iFace2);

		//find the triangle which the point placed.
		int				PointInWhichTriangle(const Vec4& P, XBaseMesh* mesh);

		// get the edge that does not connect with the specified vertex
		int				GetOppEid(int fid, int vid);

		//get the adjacent boundary edge
		int				GetAdjacentBoundaryEdge(int vertindex,int edgeindex);

		// Functions from HMesh
		// this function can deal with abnormal meshes, and make all the face to triangles.
		void		InitMesh(bool IsTranslation,Vec4& vcent,bool isFirstCall,bool bIsMayHasRect); //XXX 2004-09-24

		void		AdjustMesh(Vec4 vcent,float fScl);
		//Get the plane norm for intersect with the start and end points(face) are given to specify the plane	
		Vec4		GetIntersectPlaneNorm( Vec4 &stpt, int stfaceidx,  Vec4 &endpt, int endfaceidx);

		// Compute the polygons by intersecting the mesh and a level. Only one loop will be obtained	
		bool		IntersectCurLoop(Vec4 pt, Vec4 norm, Vec4 *ptlist, int &ptnum);//wangmei 2005.3.16


		//cut mesh by close fitspline   add by XXX
		//bool		SurfaceTrim(varray<FitSpline>& vtrimfitspline, XBaseMesh* newmesh, int ifsplineType=0);  
		//bool		SetCutFaceInfoByCloseFitSpline(varray<CLine> &fitsplineclines, _CuttedFace *cutFace, int arrayPtOldSize);
		//bool		CheckXVertBelongInByCloseFitSpline(varray<CLine> &fitsplineclines, _CuttedFace *cutFace, varray<Vec4> &arrayPt,
		//	            int arrayPtOldSize, varray<int> &oriXMeshVFlag,varray<bool> &oriXMeshEFlag);
		//bool		GetPolygonByCloseFitSpline(_CuttedFace *cutFace, varray<Vec4> &arrayPt, int arrayPtOldSize, varray<_Polygon> &vPolygon,varray<int> &oriXMeshVFlag);
		//bool        GetPolygonIncludeVertex(_CuttedFace *cutFace, int curCutFaceIdx, varray<Vec4> &arrayPt, int arrayPtOldSize, varray<_Polygon> &vPolygon, varray<int> &oriXMeshVFlag);
		//bool        GetPolygonExcludeVertex(_CuttedFace *cutFace, int curCutFaceIdx, varray<Vec4> &arrayPt, int arrayPtOldSize, varray<_Polygon> &vPolygon);
		//bool		TrigulationByCloseFitSpline(varray<Vec4> &arrayPt, varray<_Polygon> &vPolygon, varray<_NewFace> &vNewFace);
		//bool		MoveSplineCptLeaveEdge(varray<Vec4>& cpts, varray<int>& fids);
		//bool		MoveSplineCptLeaveEdge(FitSpline& fitspline);
		//bool		PointExistInPolygon(int pointID, const _Polygon &polygon); 


		//Calculate the 3 original inner angles for the faces of the inputed vertex linked	
		void    	AngleInit(int iVid);

		//Calculate the current angle of the vertex in the specified face	
		float   	GetAngleOnVert(int fid,int pid);

		// Calculate the gauss curvature of all the vertices in the mesh	
		void    	CurvatureOnVertexCalculation();

		// Calculate the gauss curvature of the specified vertex in the mesh	
		float   	CurvatureOnVertexCalculation(int iVid);

		//Calculate the value as a Eq:  T = g(iVid)*g(iVid) + Sum( g(qj) * g(qj)) )	
		//Where g(iVid) is the iVid vertex gauss curvature, qj is the adjacent vertex of iVid	
		//Sum() is a function to get the sum of all the variable 	
		double   	CalCurvTFunc(int iVid);

		//Get the derivative of T, T = g(iVid)*g(iVid) + Sum( g(qj) * g(qj)) )	
		double   	CalDerivativeOfT(int iVid);

		// get the adjacent face indexes around the triangle.
		void		GetAdjFids(int fid, varray<int>& adjFids,int loopNum=1);

		// get the adjacent face indexes around the triangle.
		void		GetAdjFids(const varray<int>& inputVids, varray<int>& adjFids);

		//	Find adjacent faces on the boundary
		//void		GetAdjFidsOnBound(int fid, varray<int>& adjFids, int loopNum=1);

		// get the original angle of a vertex in the specified face.
		float		GetInitialAngleInFace(int fid,int pid);

		// compute gausscurvatrue in mesh smooth.
		//void		GaussCurvatureCompForVertexInMeshSmooth();

		//compute original angle(3D) on the vertex
		float		Comp3DAngOnVert(int fid, int pid);

		// compute one vertex normal
		Vec4		ComputeVertNormal(int vid);

		// compute one face area in 3d
		//float		Comput3DFaceArea(int fid);

		// compute the dihedral angle on the edge
		float		GetDihedral(int eid);

		// compute the 3d angle on the vertex.
		float		Comput3DAngOnVert(int fid, int vid);

		// compute the 3d cos on the vertex
		float		Comput3DCosOnVert(int fid, int vid);

		// compute the total angle on the vertex same as GetTotalAngleOnVert()
		float		Comput3DAngOnVert(int vid);

		// get another ID of edge on a vertex in the same face
		int			GetAnotherEdgeOnVert(int inputFid, int inputVid, int inputEid);

		// Order the sequence of all adjacent vertices of a vertex
		bool		GetAdjVOnWise(int vid, varray<int>& outputAdjVids);

		// Compute the shortest distance from a point to an edge of a triangle
		float		V_Edge_InTriangle(Vec4 v, int inputFid, int& output_refEid);

		// Rotate a 2D position around z-axis with rotating center being Vec(0., 0., 0.,)
		void		VertReset(float angle);

		// Get Id of adjacent triangle of face(meshid) on edge(eid)
		int			GetAdjFaceID(varray<bool>* meshVisited,int meshid,int eid);  //modified

		//bool	/*0*/	_IntersectPlane(Vec4 startpt, int numf, Vec4 pnorm, varray<Vec4>& plist,varray<int>& pedges, bool IsConvex) const;
		//bool	/*1*/	_IntersectPlane(Vec4 startpt, int numf, Vec4 pnorm, varray<Vec4>& plist,varray<int>& nedge, varray<float>& eratio, bool IsConvex) const;
		//bool	/*2*/	_IntersectPlane(Vec4 pt, Vec4 pnorm, varray<Vec4>& plist) const;//-----4

		//bool	/*3*/	_IntersectPlane(Vec4 pt, Vec4 norm,  varray<Vec4>& ptsArr, varray<int>& edgeidArr)const;//------4


		//bool  IntersectPlane(Vec4 pt, Vec4 norm, Vec4 *ptlist, int &ptnum, float *boundarybox, bool needsort);
		//int	  MapPointToFaceOnMesh(Vec4& InputPoint, float* BoundaryBox); // Added by XXX, Jul. 9th, 2007
		//int	  MapPointToFaceOnMesh(Vec4& InputPoint, float box_dis); // Added by XXX, Jul. 27th, 2007

		// Set two points on two triangles sharing the edge but not on the edge for each edge
		void 			SetAdjPtIdxOfEdge();
		void			OrgLenInCrossingSpringInitializing();

		XBaseMesh**		DivideSubMeshPatches(int& meshnum);

	public:
		// Compute the mesh bounding box in 3d 计算网格边界
		void ComputeMeshBox3D(void);

		//曲面操作
		//曲面光顺
		//void SmoothMesh1(int iterNum); //目前使用，cl写
		//void SmoothMesh2(int niter); //效果太差
		//void SmoothMesh3(const varray<int>& vVtIdx,int niter); //没什么效果

		//Smooth the surface iteratively	
		//void		SurfaceSmoothing(int niter);
		//Smooth the surface with constraint that some given point can't be changed	
		//void		SurfaceSmoothingWithConstraint(int niter,const varray<int>& consIdxArr);//Smooth the surface with constraint that some given point can't be changed,XXX,2005.3.16 
		//Smooth the surface by adjust the specified vertices array in the mesh	
		//void    	SurfaceSmoothing(int niter,const varray<int>& idxArr);

		//get angle between two face which share the same edge.
		float  GetAngleEdge(int eid);

		//set main orientation for model.
		void   SetMainOrientationforModel();
		
		void   SetAdjacentPt();   //according to the adjacent edge.

		//fix the crack of mesh because the transition from CAD model to obj model.
		void   FixMeshConversionFromCADModel(bool IsModelClosed = true);
		void   MergeVertByIdx(int Mergeidx,int removeIdx);
};

} // base

#endif