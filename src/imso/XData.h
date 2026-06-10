/********************************************************************
* FILE NAME		:XData.h
* AUTHOR		:D. XXX
* DATE			:May 16, 2002
* MODIFIER		:
* MODIFY DATE	:
* DESCRIPTION	:basic data classes
* version		:1.0
********************************************************************/

#if !defined(__XDATA_H_INCLUDED)
#define __XDATA_H_INCLUDED
#pragma once

#include "XVec.h"
#include "varray.h"

using std::vector;

namespace base {

struct _tagSideInfo{
	int ClineID;
	int nSide;

	_tagSideInfo() {
		ClineID = -1;
		nSide = -1;
	}
	_tagSideInfo(int _id, int _side) {
		ClineID = _id;
		nSide = _side;
	}
};

// Basic class of vertex for a mesh
class  XVert
{
private:
	Vec4					m_pos;    // position of the vertex
	Vec4					m_pos2d;    // 2d position of the vertex - after expanding added by Ljt(Some times it can be used in another way---original vertex position)
	Vec4					m_pos_pre;
	Vec4					m_pos_org;

	Vec4					m_norm;   // normal of the vertex
	Vec2					m_st;    // texture mapping coordinate
	vector<int>					m_color; //Vertex Color
	

	varray<int>				m_adjEdge;     // Adjacent edge list
	varray<int>				m_adjFace;     // Adjacent face list
	varray<int>             m_adjPoint; //neigbhboor point according to the adjacent edge.  //the index of points is not sorted since the index of edges is not sorted.

	float					m_GaussCurvature; // the gauss curvature on each vertex
	float					m_Deformation; //deformation after flattening
	float					m_Curvature;
	vector<int>					m_CurtureColor;
//	float					m_TotalAngle; // the sum of angles on the vertex

public:
	// the two following members should be moved to private some time later.
	int						m_projFid; //the projection triangle id in original model when project a vertex in flattened mesh onto it. XXX 2003-03-28
	Vec4					m_projuvw; //the relative coordinate in orgianl model XXX 2003-03-28

	int						m_flag;  // the two are for temporarily use (may need remove some time later)
	int						m_mark;
	int						m_vid;
	int						m_symId;
	float					uvw[3]; // commented by XXX used in temp information for mesh optimization(need delete some time later)[7/10/2006] mapping information - bycentric coordinate relative to its attached triangle

	//the following two variables for mapping before and after refining the remeshed triangles
	int						m_fin;    // the number of triangle where the vertex is embeded - for progressive mesh
	Vec4					m_uvw;   // mapping information - bycentric coordinate relative to its attached triangle
   
	float                   m_uHarmonicValue,m_vHarmonicValue; //give a value inside [0,1] for every point by using harmonic method when parameterization.
    varray<float>           m_fOmiga;   //mean value coordinate method.
	bool                    m_bCurvatureSensitive;

	float                   m_FieldRealVal;
	float                   m_oriFieldRealVal;
	varray<int>             m_SortidxByRealValue;
private:
	bool					m_IsOnBoundary;  // true: the vertex is on the boundary, otherwise, false

public:
	XVert();
	~XVert();

	Vec4&			Pos()				{return m_pos;};  
	Vec4&			Pos2d()				{return m_pos2d;  };  //added by Ljt
	void			SetPos(Vec4 pos)    {m_pos = pos;};		// added by qcy 2006.10.18
	void			SetNorm(Vec4 norm)  {m_norm = norm;};	// added by qcy 2006.10.25
	void			SetST(Vec2 st)		{m_st = st;};		// added by qcy 2006.10.25		
	Vec4&			Pos_Old()            {return m_pos2d;};   //modified by XXX
	Vec4&			Pos_Pre()            {return m_pos_pre;};  
	Vec4&			Pos_Org()            {return m_pos_org;};  
	Vec4&			Norm()				{return m_norm; };
	Vec2&			ST()				{return m_st;  };
	vector<int>&			Color()				{return m_color; }; 
	vector<int>&			CurvatureColor()	{return m_CurtureColor;};
	float&			GetGaussCurvature()   {return m_GaussCurvature;};
	float&			GetDeformation()      {return m_Deformation;};
	float&			GetCurvature()        {return m_Curvature;};	

	const Vec4&		Pos()     const {return m_pos;  };  
	const Vec4&		Pos2d()   const {return m_pos2d;  };  //added by Ljt
	const Vec4&		Norm()    const {return m_norm; };
	const Vec2&		ST()      const {return m_st;  };
	const vector<int>&	Color()   const {return m_color; }; 
	const vector<int>&	CurvatureColor() const{ return m_CurtureColor;};
	const float&	GetGaussCurvature()const{return m_GaussCurvature;};
	const float&	GetCurvature()    const {return m_Curvature;};

	bool&			IsOnBoundary()          {return m_IsOnBoundary; }
	const bool&		IsOnBoundary()    const {return m_IsOnBoundary; }

	void			SetAdjESize(int i)      {m_adjEdge.resize(i);     }; // set the number of adjacent edges
	int				GetAdjESize()     const {return static_cast<int>(m_adjEdge.size()); }; // get the number of adjacent edges

	void			AdjEClear()             { m_adjEdge.clear(); };
	void			AdjFClear()             { m_adjFace.clear(); };
	void            AdjPClear()              {m_adjPoint.clear();};

	void			SetAdjFSize(int i)      { m_adjFace.resize(i);     }; // set the number of adjacent faces
	int				GetAdjFSize()     const {return static_cast<int>(m_adjFace.size()); }; // get the number of adjacent faces

    void			SetAdjPSize(int i)      { m_adjPoint.resize(i);     };
    int				GetAdjPSize()     const {return static_cast<int>(m_adjPoint.size()); }; 

	void			AddAdjE(int ne)         { m_adjEdge.push_back(ne); }; // added one adjacent edge
	void			AddAdjF(int nf)         { m_adjFace.push_back(nf); }; // added one adjacent face
	void            AddAdjP(int np)         { m_adjPoint.push_back(np);};
	void			FastInit()				{m_adjEdge.resize(0);m_adjFace.resize(0);m_adjPoint.resize(0);m_adjEdge.reserve(8);m_adjFace.reserve(8);m_adjPoint.reserve(8);};
	void			DelAdjE(int ne); // delete one adjacent edge - ne: edge index
	void			DelAdjF(int nf); // delete one adjacent face - nf: face index
	void            DelAdjP(int np);

	int&			AdjE(int i)           {return m_adjEdge[i];} ;//get an adjacent edge
	varray<int>&	AdjE()				  {return m_adjEdge;}
	int&			AdjF(int i)           {return m_adjFace[i];}; //get an adjacent face
	varray<int>&	AdjF()				  {return m_adjFace;}
	varray<int>&    AdjP()                {return m_adjPoint;};
	int&            AdjP(int i)           {return m_adjPoint[i];};

	const int&		AdjF(int i)     const {return m_adjFace[i];}; //get an adjacent face
	const varray<int>&	AdjF()		    const {return m_adjFace;};
	const int&		AdjE(int i)     const {return m_adjEdge[i];} ;//get an adjacent edge
	const varray<int>&	AdjE()			const {return m_adjEdge;} ;
	const varray<int>&    AdjP()          const      {return m_adjPoint;};
	const int&            AdjP(int i)      const     {return m_adjPoint[i];};
	// ---------------------------------------------------------------
public:
	Vec4	m_prevpos;	
	Vec4	m_newpos;	

	Vec4	m_prevnorm;	
	Vec4	m_newnorm;	

	Vec4	m_vel;	

	float	m_mass;			

	bool	m_bfixed; // whether it is fixed or not during cloth simulation

	Vec4	m_Force; // the force on the vertex in flattening iteration 
	Vec4&	Force(){return m_Force;};
};


// Extended class of vertex for flattening
class  XVertEx
{
private:
	Vec4					m_Dis;   //the displacement of the vertex in flattening iteration
	Vec4					m_Force; // the force on the vertex in flattening iteration 
	

	float					m_TotalAngle; // the sum of angles on the vertex

	//the following mumber for constrained flattening
	Vec4					m_crosspondingV;

	int						m_MappingTID;

	//the following mumber for material reflecting
	Vec4					m_temppos2d;

	bool					m_constrainedFlag;
	bool					m_fixedFlag;
	bool					m_VertVisited;    //the flag whether the vertex has been visited 
	bool					m_newAdded; //whether the vertex is a new added vertex or not in gerneral strip expanding

	//int						m_edgeid_on; // which edge the vertex is on
	//int						m_oriedgeid; //  the original edge index 
	
	bool					m_cornerFlag;

	bool					m_featureFlag; // to indicate whether the vertex is a feature edge, used in mesh smooth


public:
	XVertEx();
	~XVertEx();

	bool&			IsFeature()      {return m_featureFlag;};
	const bool&		IsFeature()const {return m_featureFlag;};

	Vec4&			GetTempPos2d()   {return m_temppos2d;}; // for material reflecting
	Vec4&			Dis()            {return m_Dis;};
	Vec4&			Force()          {return m_Force;};
	bool&			Visited()        {return m_VertVisited;};

	float&			GetTotalAngle()  {return m_TotalAngle;};

	Vec4 &			CrosspondingV()       {return m_crosspondingV; };
	bool &			ConstrainedFlag()     {return m_constrainedFlag;};
	bool&			FixedFlag()           {return m_fixedFlag;};
	
	const Vec4&		Dis()     const {return m_Dis;};
	const Vec4&		Force()   const {return m_Force;};
	const bool&		Visited() const {return m_VertVisited;};

	int&			GetMappingTID()       {return m_MappingTID;};
	const int&		GetMappingTID() const {return m_MappingTID;};

	bool&			IsNewAdded()          {return m_newAdded;};

	bool&			IsCorner()            {return m_cornerFlag;}
	bool&			IsFixed()             {return m_fixedFlag;}
};


class  XEdge
{
public: 
	int				m_flag;  // used for simplification and progressive mesh

private:
	int				m_idx[2];
	int				m_fidx[2];   // adjacent face

	float			m_orilen;  // original length(3D length)
	float			m_curlen;  // current length (2D length)
	bool			m_bVisible;
	bool			m_IsBoundary; // true: the edge is at the boundary, otherwise, false

public:
	XEdge();  
	~XEdge();

	int&			p(int i) {return m_idx[i];}
	int				p(int i) const {return m_idx[i];}
	int				GetIndex(int i) const {return m_idx[i];}
	void			SetIndex(int i0, int i1) {m_idx[0] = i0; m_idx[1] = i1;}
	bool&			GetVisible() {return m_bVisible;}
	const bool&		GetVisible() const {return m_bVisible;}

	float&			OriLen()         {return m_orilen; }  // get edge`s original length
	const float&	OriLen()   const {return m_orilen;}
	float&			CurLen()         {return m_curlen; }  // get edge`s current length
	const float&	CurLen()   const {return m_curlen;}

	int&			GetFIdx(int i)   {return m_fidx[i]; } // get the edge`s adjacent face
	const int&		GetFIdx(int i)const { return m_fidx[i]; } // get the edge`s adjacent face
	bool&			IsBoundary()     {return m_IsBoundary; }
	const bool&		IsBoundary()const{return m_IsBoundary; }
	int				GetAdjFCount() const ;    // number of adjacent faces for an edge
public:
	int		        m_vnum;	        // number of voxels associated with the edge
	int				m_adjPtIdx[2];	// Two points on two triangles sharing the edge, but not on the edge
	float			m_crossLen; // the len between its opposite points in its two adjacent triangles 

	// Get two points on two triangles sharing the edge, but not on the edge
	int&			GetAdjPtIdx(int i) { return m_adjPtIdx[i]; }
	float&			GetCrossLen(){return m_crossLen;};
};


// Extended class for flattening
class XEdgeEx
{
private:

	int				m_iStripNO; //to show the edge belong to which strip
	float			m_crossLen; // the len between its opposite points in its two adjacent triangles 

	int				m_adjPtIdx[2];	// Two points on two triangles sharing the edge, but not on the edge
	bool			m_visitedFlag;
	bool			m_newAdded; //whether the edge is a new added edge or not in general strip expandeing
	bool			m_featureFlag; // to indicate whether the edge is a feature edge, used in mesh smooth

public:
	XEdgeEx();  
	~XEdgeEx();

	bool&			IsFeature()      {return m_featureFlag;};
	const bool&		IsFeature()const {return m_featureFlag;};
	bool&			IsNewAdded()     {return m_newAdded;}
	const bool&		IsNewAdded()const{return m_newAdded;}
	bool&			IsVisitedFlag()  {return m_visitedFlag;};
	const bool&		IsVisitedFlag()  const {return m_visitedFlag;};
	int&			GetAdjPtIdx(int i)    { return m_adjPtIdx[i]; }; 
	int&			GetStripNO()     {return m_iStripNO;};
	float&			GetCrossLen()    {return m_crossLen;};
};

// basic class of face
class  XFace
{
private:
	int				m_idx[3];
	int				m_normIn2D; // modified by XXX [8/18/2006] the norm of the face after being expanded (0: not calculated(initial value),1: normal z > 0, -1: normal z < 0)
	// the 2d position only use x and y coordinates :)
	Vec4			m_norm;
	float			m_AreaIn2D; //the area of the triangle after being flattened

	int				m_eref[3];
	float			m_OriArea; //the area of the triangle before being flattened
	int				m_adj[3];
//	bool			m_AngleInited;
	int				m_matID;
	float			m_angle[3];// three current internal angles added by XXX
	float			m_oriAngle[3]; // three original internal angles

public: 
	Vec4*			m_pPos[3];
	int				m_OriFid; // when a mesh add sleeve or collar, face id will change, note the old id added by XXX
	int				m_iSubMeshID;
	vector<int>			m_color;
	int				m_flag;         // used for simplification and progressive mesh
	int				m_projFid; //the projection triangle id in the original modle when project a flattened face onto the original model
	bool			m_bVisible;
	bool			m_bHalfVisible;

private:
	bool			m_IsBoundary; // true: the triangle is at the boundary, otherwise, false
	

public:
	XFace();
	~XFace();

	void			SetIndex(int i0, int i1, int i2) {m_idx[0]=i0;m_idx[1]=i1;m_idx[2]=i2;}

	int				p(int i) const {return m_idx[i];}

	int&			p(int i) {return m_idx[i];}

	int				GetIndex(int i) const {return m_idx[i];}

	vector<int>&			GetColor() {return m_color;}

	const vector<int>&	GetColor() const {return m_color;}

	bool&			GetVisible() {return m_bVisible;}

	const bool&		GetVisible() const {return m_bVisible;}

	void			SetEdgeRef(int r0, int r1, int r2) {/*assert(r0>=1&&r1>=0&&r2>=0); */m_eref[0] = r0;m_eref[1] = r1;m_eref[2] = r2;}

	void			SetEdgeRef(int idx, int r) {/*assert(idx >= 0 && idx < 3); */m_eref[idx] = r;}

	int				GetEdgeRef(int i) const {return m_eref[i];}

	void			SetAdjacent(int r0, int r1, int r2)
	{
		m_adj[0] = r0; m_adj[1] = r1; m_adj[2] = r2;
	}

	void			SetAdjacent(int idx, int r) {/*assert(idx >= 0 && idx < 3);*/ m_adj[idx] = r;}

	int				GetAdjacent(int i) const {/*assert(i >= 0 && i < 3);*/ return m_adj[i];}

	void			FastInit(){m_eref[2] = m_eref[1] = m_eref[0] = -1; m_adj[2] = m_adj[1] = m_adj[0] = -1;};

	bool&			IsBoundary()      { return m_IsBoundary; }
	const bool&		IsBoundary()const { return m_IsBoundary; }

	float&			GetAreaIn2D()     {return m_AreaIn2D;};
	const float&	GetAreaIn2D()      const{return m_AreaIn2D;};

	float &			GetOriArea()      {return m_OriArea;};
	const float&	GetOriArea() const{return m_OriArea;};
	const Vec4&		Norm()       const{return m_norm; };
	Vec4&			Norm()            {return m_norm; };
	int&			NormIn2D()        {return m_normIn2D; };
	float&			Angle(int i);
	float&			GetOriAngle(int i)        {return m_oriAngle[i];};
	void			SetVertex(int nv1, int nv2, int nv3)
	{
		m_idx[0] = nv1;
		m_idx[1] = nv2;
		m_idx[2] = nv3;
	};
	bool            IsCurFace(int idx1,int idx2,int idx3)
	{
		return      idx1 == m_idx[0] && idx2 == m_idx[1] && idx3 == m_idx[2]
		        ||  idx1 == m_idx[0] && idx2 == m_idx[2] && idx3 == m_idx[1]
				||  idx1 == m_idx[1] && idx2 == m_idx[0] && idx3 == m_idx[2]
				||  idx1 == m_idx[1] && idx2 == m_idx[2] && idx3 == m_idx[0]
				||  idx1 == m_idx[2] && idx2 == m_idx[0] && idx3 == m_idx[1]
				||  idx1 == m_idx[2] && idx2 == m_idx[1] && idx3 == m_idx[0];
	};
	bool			IsInVisibleRegion(bool bInPoly) const {  
		return m_bVisible || (m_bHalfVisible && bInPoly);
		}

	int&			GetMaterialID() {return m_matID;}

	const int&		GetMaterialID() const {return m_matID;}

	int&			GetSubMeshID() {return m_iSubMeshID;}

	const int&		GetSubMeshID() const {return m_iSubMeshID;}
public:			
	bool    m_mark; 
};

// Extended class of face for flattening
class  XFaceEx// : public Face
{
public: 
	varray<int>		m_vlist; // list of vertices included - for progressive mesh
	//	int				m_edgeDirVert[3][2]; //used in material reflecting
	//	bool			m_bHalfVisible; // face is cutted by symmetric line, half is visible, the rest is invisible, by XXX[5/3/2005]

private:
	float			m_expandingAngle[3];// used in expanding a triangle strip
	float			m_expectedAngle[3]; // the expected angle after flatten

	//the following mumbers for mapping before and after refining the remeshed triangles
	int				m_MappingTID; //the refined triangle on which the vertex should be mapped
	float			mapPara[3][3]; //three parameters to identify the projected point on the projected triangle.
	int				m_relatedPID[3];
	int				m_iStripNO; //to show the face belong to which strip
	bool			m_MeshVisited;    // the flag whether a traingle has been visited
	bool			m_deletedFlag; //the flag to indicate that this vertex has been canceled during the refinment.



public:
	XFaceEx();
	~XFaceEx();

	bool&			Visited()         {return m_MeshVisited;};
	const bool&		Visited()    const{return m_MeshVisited;};

	float&			GetExpectedAngle(int i)   {return m_expectedAngle[i];};

	//three parameters to identify the projected point on the projected triangle.
	int&			GetMappingTID()           {return m_MappingTID;};
	float&			GetMapPara(int i, int j)  {return mapPara[i][j];};
	int&			GetRelatedPID(int i)      {return m_relatedPID[i];};
	bool&			IsDeleted()               {return m_deletedFlag;}; //the flag to indicate that this vertex has been canceled during the refinment.
	varray<int>*	GetVList()                {return &m_vlist;};
	int&			GetStripNO()              {return m_iStripNO;}
	
	float&			GetExpandingAngle(int i)  {return m_expandingAngle[i];};
	bool			RemoveFromVList(int vid);
};


class  MaterialProperty
{
protected:
	float		m_WeftConstant; // the unit spring constant on the dirction of weft 
	float		m_WarpConstant; // the unit spring constant on the dirction of warp  
	float		m_minConstant; //th minimal spring constant of material
	float		m_MaxDeformRetioOfWeft; //the maxal deformation ratio in the direction of weft
	float		m_MaxDeformRetioOfWrap;//the maxal deformation ratio in the direction of warp
	float		m_MaxDeformRetioOfDigonal;
public:
	void		SetMaterialPara(float WeftConstant, float WarpConstant, float minConstant,
						float MaxDeformRetioOfWeft, float MaxDeformRetioOfWrap, float MaxDeformRetioOfDiagonal);
	float		GetWeftCons(){return m_WeftConstant;};
	float		GetWarpCons(){return m_WarpConstant;};
	float		GetMinCons(){return m_minConstant;};
	float		GetMaxDeformRatioWeft(){return m_MaxDeformRetioOfWeft;};
	float		GetMaxDeformRatioWarp(){return m_MaxDeformRetioOfWrap;};
	float		GetMaxDeformRatioDig(){return m_MaxDeformRetioOfDigonal;};
};

class  MatProp
{
public:
	float			m_A; // area of the woven element
	float			m_Ku; // strain constant in weft direction
	float			m_Kv; //strain constant in warp direction
	float			m_I; //yarn cross-sectional area moment of inertia
	float			m_k; //spring constant
	float			m_L; // total specimen fabric length
	float			m_P1; // pitch spacing in weft direction
	float			m_P2; // pitch spacing in warp direction
	float			m_E;  // Young modulus
	float			m_u;  // Possion coefficient
};


template <class Item>class WIterator
{
public:
	virtual void			First() = 0;
	virtual void			Next() = 0;
	virtual bool			IsDone() const= 0;
	virtual const Item &	CurrentItem() const = 0;
	virtual Item &			CurrentItem() = 0;
	virtual int				Index()const = 0;
	//int						Iterator();	
};//XXX 2003-02-13


template <class Item>class IteratorPtr
{
public:
	IteratorPtr(WIterator<Item> * i): _i(i)   { }

	~IteratorPtr() 
	{
		delete _i;
	}

	WIterator<Item>  * operator -> ()
	{
		return _i;
	}

	WIterator<Item>  & operator * ()
	{
		return *_i;
	}

private:
	IteratorPtr(const IteratorPtr&);
	//IteratorPtr & operator = (const IteratorPtr&);

	WIterator<Item> * _i;
	
};

}

#endif

