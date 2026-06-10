#if !defined(__XFUNCTIONS_H_INCLUDED)
#define __XFUNCTIONS_H_INCLUDED

#include <fstream>
#include <string>

#include "spline.h"
#include "SplineSurface.h"
#include "XVec.h"
#include "definition.h"
#include "XBaseMesh.h"

using namespace base;
using std::string;
const int NMAX_CHAR_NUM_READ=500;
const int NMAX_INT_NUM_READ=30;

typedef struct _MyLengthCompute
{
	dFloat length;
	Vec4 interpoint;
	int  interidx;
}MyLengthCompute;

//////////////////////////////////////////////////////////////////////////
//Math function
  bool      GetSolTwohypoLinearEquation(double *aa,double *bb,double *cc);
  void      NormalizeArrayData(varray<float>& dataval);
// raster operation
  void		DDALineRaster(int p1x, int p1y, int p2x, int p2y, varray<int>& res);
  void		DDALineRaster(float p1x, float p1y, float p2x, float p2y, varray<Vec4>& res, bool duplicatepoints = false, bool onlyduplicatepoints = false);
  int		InterPointLengthCompare( const void *arg1, const void *arg2 );

//////////////////////////////////////////////////////////////////////////
//data structure operation
  bool		IsInTheVarray(int ID,const varray<int> &pVarray);
  bool		IsInTheVarray(Vec4 element, varray<Vec4> &L);
  int		GetInTheVarrayIndex(int ID,const varray<int> &pVarray);
  int		GetFaceNumber(char *s, varray<IntVec4> &fidx);
  varray<int>	GetArrayIntersection(varray<int>&selMeshIdArr, varray<int>&refMsidArr); //get the intersection of the two int array XXX 2003-04-04
 // String	GetFileNameWithoutPostfix(String filepath);//file and string operator
  bool      IstwoIntarraySame(const varray<int>& arr1, const varray<int>& arr2, bool compareByorder = false);
//////////////////////////////////////////////////////////////////////////
//calculate for point,line and plane.
//2D
  float		Angc(float xr,float yr,float x,float y);//Get the angle between two line segments
  int		side_line(float x,float y,float x1,float y1,float x2,float y2);//Judge a point(x,y) is in which side of a line(x1,y1,x2,y2);
  float 	GetAngleOf2VectorIn2D(Vec2 vecStr, Vec2 vecEnd, int dir);
  bool		IsSegmentsIntersect(Vec2 seg1a, Vec2 seg1b, Vec2 seg2a, Vec2 seg2b); //XXX
  bool		IsBoxOfTwoSegIntersect(Vec2 seg1P1, Vec2 seg1P2, Vec2 seg2P1, Vec2 seg2P2); //XXX
  Vec2		GetTwoLineInterPt2D(Vec2 ln1StPt,Vec2 ln1EndPt,Vec2 ln2StPt,Vec2 ln2EndPt);
  Vec2		MapAVertexToA2DSegment(const Vec2& vertex,  const Vec2& segstartpt, const Vec2& segendpt); //XXX
//3D
  float		GetLineLength(const Vec4 &p1, const Vec4 &p2);
  bool		TLineIntersect(Vec4 v11, Vec4 v12, Vec4 v21, Vec4 v22, Vec4 &sp);
  bool		Intersection(const Vec4 &v1, const Vec4 &v2, const Vec4 &v3, const Vec4 &v4, float &r, float &s);
  bool      Get2LineIntersectPtInPlane(Vec4& v11,Vec4& v12,Vec4& v21,Vec4& v22,Vec2& rv,int imode);
  bool		IsTwoLineIntersectIn2d(Vec4 ln1StPt,Vec4 ln1EndPt,Vec4 ln2StPt,Vec4 ln2EndPt);
  bool		GetTwoLineIntersectRatio(const Vec4 &v1, const Vec4 &v2, const Vec4 &v3, const Vec4 &v4, float &r, float &s);
  Vec4		GetTwoLineInterPt(Vec4 ln1StPt,Vec4 ln1EndPt,Vec4 ln2StPt,Vec4 ln2EndPt);
  bool		IsTwoEdgeIntersect(Vec4 v1, Vec4 v2, Vec4 v3, Vec4 v4,int viewid);
  bool		IsTwoPointAtLineSameSide(const Vec4 &pt1,const Vec4 &pt2,
												 const Vec4 &linestpt,const Vec4 &lineendpt);
  bool		IsTwoLineIntersectedOnFace(const Vec4 &lineOneSt,const Vec4 &lineOneEnd,
												   const Vec4 &lineTwoSt,const Vec4 &LineTwoEnd,const Vec4 &faceNorm);
  Vec4		GetMappingVertOnLine(const Vec4& Vec4Point,const Vec4& linestartpt,const Vec4& lineendpt);
  bool		IsSharpAngleOfTwoVert(const Vec4 & vectOne,const Vec4 &vectTwo);
  float		GetDistanceFromPtToLineSeg(const Vec4& Ve3Point, const Vec4& lineStartPt, const Vec4& lineEndPt, Vec4& interPt);
  bool		IsTwoPtVeryNear(const Vec4 &pa,const Vec4 &pb);
  bool		IsVertBetweenTwoVerts(const Vec4& Vert, const Vec4& Vec4First, const Vec4& Vec4Last) ; 
  double    GetDistanceFromPtToLine(const Vec4& Ve3Point, const Vec4& lineStartPt, const Vec4& lineEndPt) ;
  Vec4		GetPlumbPointAtLine(const Vec4 &lineV0, const Vec4 & lineV1, const Vec4 & sourceV);
  Vec4		GetLineFaceIntersectPnt(Vec4& lsp,Vec4& lep,Vec4& fp,Vec4& fnorm);
  Vec4		MapAVertexToASegment(const Vec4& vertex,  const Vec4& segstartpt, const Vec4& segendpt); //2003-11-06 Li
  bool		MapAVertexToASegment(const Vec4& vertex,  const Vec4& segstartpt, const Vec4& segendpt, Vec4& mappedPos);
  bool		IsTwoPointsAtLineDiffSideOnFace(const Vec4 &pointOne,const Vec4 &pointTwo,
														const Vec4 &lineSt,const Vec4 &LineEnd,const Vec4 &faceNorm);
  float		GetAngleOf2Vector(const Vec4 &v1, const Vec4 &v2);
  float		GetDistOfPntToLn(Vec4 pt, Vec4 p1, Vec4 p2);
  double	GetCosofAngleAOB(const Vec4& PtA,const Vec4& PtO, const Vec4& PtB ) ;
  double	GetSinofAngleAOB(const Vec4& PtA, const Vec4& PtO,const Vec4& PtB ) ;
  Vec4		GetMappingPointOnAPlane(Vec4 inputP,Vec4 planeP1,Vec4 planeP2, Vec4 planeP3);//XXX
  Vec4		GetMappingPointOnAPlane(Vec4 inputP, Vec4 planePnt, Vec4 planeNormal); //XXX add
  bool		IsOnDiffSide(const Vec4 &p1, const Vec4 &p2, const Vec4 &p3, const Vec4 &p4);
  void		GetPerpendicularDir(Vec4 inputV, Vec4& outputV1, Vec4& outputV2); 
  float		GetAngleBetweenVectorAndPlane(Vec4 vect, Vec4 pNorm); //vector: vect,  plane normal: pNorm
 	bool		IsPointOnPlane(Vec4 inputPoint, Vec4 PointOnPlane, Vec4 PlaneNorm); // XXX
  bool		IsTwoLinesAtSamePos(const Vec4 &lineOneSt,const Vec4 &lineOneEnd,
											const Vec4 &lineTwoSt,const Vec4 &lineTwoEnd);
  bool		IsLinePlaneIntersect(Vec4 lineStart, Vec4 lineEnd, Vec4 planePoint, Vec4 planeNorm);
  Vec4		MapAVectorToAPlane(Vec4 vect, Vec4 pNorm) ; //vector: vect,  plane normal: pNorm

//////////////////////////////////////////////////////////////////////////
//calculate for polygon,including triangle.
//get the intersect point for line and triangle.
  float		GetDistanceFromPtToTriangle(const Vec4& Ve3Point, const Vec4& P0, const Vec4& P1, const Vec4& P2, Vec4& interPt);
  bool        TLineTriangleIntersect(Vec4& v1,Vec4& v2,Vec4& a,Vec4& b,Vec4& c,Vec4& fnorm,Vec4& sp);
  bool		IsPointIn3DTriangle(const Vec4 &testP, const  Vec4 &vertex1, const Vec4 &vertex2, const Vec4 &vertex3,bool bIsFast = false);
  bool		IsPointInTriangle(const Vec4 &testP, const  Vec4 &vertex1, const Vec4 &vertex2, const Vec4 &vertex3,const float ThreshHold = 1.0e-6);
  bool		IsPointInTheBoxOfTriangle(Vec4 testingP, Vec4 v1, Vec4 v2, Vec4 v3);
  Vec4		GetProjectPntOnTriangle(Vec4 v1, Vec4 triV[3]);// get the projection of v1 on the triangle with 3 vertex of triV[3]
  Vec4		GetBarycentCoorInATriangle(Vec4 P, Vec4 v0, Vec4 v1, Vec4 v2); 
  void		GetThirdPointByAngle( const Vec4 &p1,const Vec4 &p2,float p1p3, float angle, Vec4 &p31, Vec4 &p32);
  void		GetThirdPoint(const Vec4 &p1,const Vec4 &p2, float b, float a, Vec4 &p31, Vec4 &p32);
  bool		IsPtAndTriangleAtFaceDiffSide(const Vec4 &pt,const Vec4 &pa,const Vec4 &pb,
													  const Vec4 &pc,const Vec4 &norm,const Vec4 &ptinface);
  Vec4		GetCircleCenterPtInATriangle(Vec4 v0, Vec4 v1, Vec4 v2);
  void        GetRatioByCircleCenterPtInATriangle(Vec4 v0, Vec4 v1, Vec4 v2,float& r0,float& r1,float& r2);
//polygon operator
  bool		IsPointInPolygon(Vec4 testP, varray<Vec4> &polygonP);
  void		GetPolygonFromDiscretePoints(varray<Vec4>& DiscreteP,varray<Vec4>& leftV, varray<Vec4>& rightV, varray<Vec4> &polygonP);
  void		GetConvexHullFormPolygon(varray<Vec4>& leftV,varray<Vec4>& rightV,varray<Vec4> &convexHullP);
  bool		IsPointInConexPolygon(Vec2 testP, const varray<Vec2>& polygon);//, Vec2 polygon_cent);//XXX
  bool        IsPolygonVertexConvex(varray<Vec2>&vtsPoly,int idx);   //XXX,2006.2.8
  bool		UnifyPolygonEdge(const varray<Vec2>&closedCurv1, const Vec2 cent, varray <Vec2>&adjustCurv1); //XXX

//compute the line and the polygon intersection point after the polygon is offseted(or scaled) from its center
  bool		GetInterSectPntOfLnPolygon(varray<Vec2>& vtsArr,Vec4& vt,int iMode,Vec4 vtCen3D = Vec4(-10e6,-10e6,-10e6));
  bool		GetInterSectPntOfLnPolygon(varray<Vec2>& vtsArr,Vec4& vt,float offset,int iMode,Vec4 vtCen3D = Vec4(-10e6,-10e6,-10e6));
//Offset the polygon with the given offset distance
  void		OffSetPolygon(varray<Vec4>& vtsArr,float fOffset, Vec4 vtCen3D = Vec4(-10e6,-10e6,-10e6));
  void		MovePointToTrangle(Vec4& ptTemp, Vec4 ptTemp0,Vec4 ptTemp1,Vec4 ptTemp2);

//////////////////////////////////////////////////////////////////////////
//calculate for basic elements relative to coordinate and axis.
  float		GetAngleVecWithXCo(Vec2 v);//Calculate the angle from x axis to the vector,range from PI*3/2~-PI/2 ,if(0,0) return 0;
  float		GetAngleOf2VectorIn2DWithSign(Vec2 vecStr, Vec2 vecEnd, int dir);
  int			ComputeGlbAngNew(Vec4 vect,float *xrot, float *yrot,float *zrot);//Compute the angle between the vector and the x,y,z axis 
//Rotate the points on a plane to horizontal
  bool		RotatePlaneToHorizontal(varray<Vec4>& ptsArr, Vec4 pNorm, Vec4 ptCen = Vec4(-10e6,-10e6,-10e6));
  Vec4		RotAxis(Vec4 pt, Vec4 vect,double ang); // rotate around a vector: vect
  Vec4		RotateX(Vec4 pt, float ang);  // rotate around X-axis
  Vec4		RotateY(Vec4 pt, float ang);  // rotate around X-axis
  Vec4		RotateZ(Vec4 pt, float ang);  // rotate around X-axis

//////////////////////////////////////////////////////////////////////////
//calculate for spline and its poly-line
//for spline basic function.
  //Spline	CreateSpline(const varray<Vec4> &contrpt); 
  void		SplineToPolyline(Spline& spl, varray<Vec4>& vtsOut); //XXX changed, 
  void		GetSplineCtrlPtsFromItsLine(Spline& spl,const varray<Vec4>& ptsArr);
//  bool		IsSelfIntersectForSplnNew(Spline& spl, int iMode);//the spline is self intersect or not,can support x-z, x-y,z-y plane
  bool		ConvertSevel2DSplineIntoOne(varray<Spline>&oriSpl, int splMode,Spline& resSpl);
  void		AdjustSplineControlPoints(Spline& spline); //2003-09-02
//Judge whether two spline intersect with each other except their endpoints
  //bool		IsTwoSplIntersect(Spline& spl1,Spline& spl2,int iMode);
//Find the point index in the given spline which is very near to the given y coordinate
  int       FindSplIdxAtGivenYValue(const Spline& spl,float yRef, int idxNotToFind = -1);
//Judge whether the inputed spline will self-intersect
  bool		IsSelfIntersectForSpln(Spline& spl, int iMode);
//some operation for spline's discrete points.
//Get the index that a point in spline-polyline points array, also get the scale,
  void		GetIdxAndScl(const varray<Vec4>& ptsArr,const Vec4& pt,int iMode,int& iIdx,float& fScl);
  float     GetPartialLength(const varray<Vec4>& vloop,int segidx1,float ratio1,int segidx2,float ratio2,float& otherLength,bool bIsClosed,varray<Vec4>& vMeasurePts,bool bIsCurPart);
  void		EliminateRuffleOfSpline(const varray<Vec4>& oriPArr,varray<Vec4>& linePArr,float threshholdAng);
  void		MakeConvex(varray<Vec4> &vlist, Vec4 pnorm);//Make the polyline points convex

//////////////////////////////////////////////////////////////////////////
//calculate for Point list, Poly-line, Loop and Slice.
// Remake a poly-line with equal length
//Poly-line
  void		MakePolyline(const varray<Vec4>& lines,varray<Vec4>&pline, int nseg);
  void		MakePolyline(Vec4 *lines, int npt, varray<Vec4>& pline, int nseg);
  void		MakePolyline(Vec4 *lines, int npt, Vec4* pline, int nseg);
  void		MakePolyline(const varray<Vec4>& lines, int npt, varray<Vec4>& pline, int nseg);
  void		MakePolylineByAngle(Vec4 * lines, int npt,int iPlane,Vec4 vtCen, Vec4* pline, int nseg);// Remake a poly-line with equal length
  void		MakePolylineByAngle(const varray<Vec4>& lines,int iPlane,Vec4 vtCen,varray<Vec4>& pline, int nseg);
  void		MakePolylineByOneDirection(const varray<Vec4> vtsIn, varray<Vec4>& vtsOut,int iSegNum,int iDir);

  bool      DivideMultiLineIntoSegMent(const varray<Vec4>& oriarr,const int segnum,varray<Vec4>& outarr,bool bIsClosed = false);
  void      DivideMultiLineIntoSegMent(varray<Vec4>& oriarr,const int segnum,bool bIsClosed = false);
  void		SimplifyALine(varray<bool>& canDelFlag, const varray<Vec4>& oriPArr, varray<Vec4>& resPArr, float thresholdAng, float thresholdLen,float expandMeshProcision); //Li 2003-09-02
  void		SimplifyLines(const varray<Vec4>& oriPArr, varray<Vec4>& resPArr, float tol); //Zheng 2007-02-07
  void		SimplifyDP(const varray<Vec4>& PArr, int stIdx, int endIdx, varray<bool>& PArrMark, float thresholdLen);	 
  float		GetDistanceFromPtToPolyLine(const Vec4& Ve3Point, const varray<Vec4>& verts, int &iSeg, float d = -1.0);
  void		AddNewCtrlPtToSimplefyLine(const varray<Vec4>& oriPArr,varray<Vec4>& resPArr,float expandMeshProcision);
//order operator of point array
  bool      GetTwoPolyLineInterPtIn2D(int iViewId,varray<Vec4>& vtsArr1,varray<Vec4>& vtsArr2,Vec4& vtInter);
  void		StoreInSameClockwise(const varray<Vec2>&prePoly,varray<Vec2>&adjustPoly, bool& wiseChanged);//XXX
  bool		IsPointStoreInClockWise(const varray<Vec2>&inputPoly);
//Sort the vertices increase or decrease in x(y,z) direction
  void		SortVts(varray<Vec4>& vtsArr,int iDir,int iMode);//iDir: vertexes sort by which(x,y,z)direction;iMode:0--increase,1--decrease 
//polyline a line with the given direction coordinate equal
  void		StorePointOnPolygonInClockwise(varray<Vec2>&inputPoly);//XXX
//intersect for two polygon 
  bool		IsTwo2DCurveInterSect(varray<Vec2>&curv1, varray<Vec2>&curv2); //XXX
//Judge whether two curve intersect with each other
  bool      IsTwoCurveIntersect(const varray<Vec4>&vtsArr1,const varray<Vec4>&vtsArr2,int iMode);
//collision operator
  bool		CollisionAdjust(const varray<Vec2>&curv1, const varray<Vec2>&curv2,const varray <Vec2>&preCurv1, Vec2 cent,bool closedCurvFlag1,bool closedCurvFlag2,float offset, varray<Vec2>&adjustCurv1);//XXX
  bool		GetIntersectPntOf2DLnCurve(const Vec2 segNod1,const Vec2 segNod2, const varray<Vec2>& curv, bool closedFlag,varray<Vec2>& intersections);//XXX
  //bool		CollisionAdjust(const varray<Vec2>&closedCurv1, const varray<Vec2>&closedCurv2, const varray<Vec2>&preClosedCurv1,Vec2 cent, float offset,varray<Vec2>&adjustCurv1);//XXX
//Adjust the inputed polygon shape by collision detection with another inputed polygon
  bool		CollisionAdjustBetween2Polygons(const varray<Vec2>&closedCurv1, const varray<Vec2>&closedCurv2, const varray<Vec2>&preClosedCurv1,Vec2 cent, float offset,varray<Vec2>&adjustCurv1);//XXX
  void		CollisionAdjustBetween2Polygons(const varray<Vec4>& vtsRef,varray<Vec4>& vtsArr,float fOffset,int iMode,Vec4 vtCen3D = Vec4(-10e6,-10e6,-10e6));
  void		CollisionAdjustBetween2Polygons(const varray<Vec4>& vtsRef,varray<Vec4>& vtsArr,varray<Vec4>& vtsPreArr, float fOffset,int iMode,Vec4 vtCen3D = Vec4(-10e6,-10e6,-10e6));
 // bool		ShapeAdjustInCollision(const varray<Vec2>&closedCurv1,const varray<Vec2>&closedCurv2,const varray <Vec2>&preClosedCurv1,Vec2 cent,varray<Vec2>&adjustShape);//XXX
  bool	    CollisionAdjustBetweenPolygonAndVec(const varray<Vec4>& vtsRef,Vec4& vt,float fOffset,Vec4 vtCenter);
  bool      CollisionDetectonBetweenTwoPolygonWithMultiCenterPts(varray<Vec4>& vtsRef,varray<Vec4>& vtsTar,float fMargin,int imode);

//Point list
//Get a point in the given points array by the specified length scale 
  bool 		GetPtByLenSclInPtsArr(Vec4& ptOut,int& idx, const varray <Vec4>& ptsArr,float fScl, bool bIsClosed);
  void  		MakeConvex(Vec4 *plist, int &pnum, Vec4 pnorm);//Make the point list convex
//Sort the inputed points array in anticlockwise direction, 
  void 		SortPtsInAntiClockWise(varray<Vec4>& vtsArr,Vec4 pnorm,Vec4 center = Vec4(1.0e6,1.0e6,1.0e6));
// Calculate the curve length between two points in retrorse direction
  float 		CalCurveLengthBetween2Points(Vec4 *fslice, int fsnum, Vec4 mfpt1,Vec4 mfpt2,bool bIsFSliceLoop,Vec4 pnorm);
  Vec4  		InterPolatePtBySacl(const varray<Vec4>& ptarr, float scal);
  void        GetArcLengthScaleForArrayPoints(const varray<Vec4>& ptarr,varray<float>& scalarr);
 	Vec4		GetCenter(Vec4* plist,int iNum);
 	Vec4		GetCenter(const varray<Vec4>& plist,int iNum);
  Vec4		GetCenter(const varray<Vec4>& ctrlPtsArr);
 	Vec4		GetPointFromLinePtsArrByLength(float fLen,float fLenSum,const varray<Vec4>& vtsList);
  bool		DivideSegmentsToTwoByLenScl(const varray<Vec4>&vtsList,varray<Vec4>& vtsList1,varray<Vec4>& vtsList2,float fScl);
//Devide 1 segments into 2 segments by given its mid x value
  bool        DivideSegmentsToTwoByMidX(const varray<Vec4>&vtsList,varray<Vec4>& vtsList1,varray<Vec4>& vtsList2,float fXMid);
  bool        DivideSegmentsToTwoByMidX(const varray<Vec4>&vtsList,varray<Vec4>& vtsList1,varray<Vec4>& vtsList2,Vec4 vtMid);
// Check whether a line is monotonic in y direction
  bool        IsLineMonotonicInYDir(const varray<Vec4> & vtsArr,bool bIsIncrease); 
//Check whether the inputed line has only one point can be intersected at given y-coordinate 
  bool        IsLineMonotonicInYDir(const varray<Vec4> & vtsArr,float yPos); 
//Offset the input line a little
  void        OffsetLine(varray<Vec4>& ptsArr,float fDis);
//Judge whether two points array have same coordinate at given y-direction range
  bool        IsTwoPtsArrSameAtYRange(const varray<Vec4>& vtsArr1,const varray<Vec4>& vtsArr2, float ySt,float yEnd,int iMode);
//Get the left,right,front,back extreme values for inputed points array
  bool		GetExtremValue(const varray<Vec4>& vtsArr,varray<Vec4>& vtsExt);
//Get a interpolated point from the given points array in x(y,z)direction
  Vec4		GetInterPolatePt(const varray<Vec4>&ptsArr,const Vec4& vtIn,int iMode,bool& bIsInterPolated);//iMode:0,x;1,y;2,z,XXX,2005.3.28;
  Vec4		GetInterPolatePt(varray<Vec4>&ptsArr,const Vec4& vtIn,int iMode,int iNearDir,bool& bIsInterPolated);//the direction that we use to judge a nearest point with the vtIn
  bool		GetInterPolatePt(Vec4 pt, varray<Vec4>& ptsArr,Vec4 pnorm, int iNearDir, Vec4& ptRet);
//Trim a points array with y(x,z) direction boundary is given the vtsArr should in ascend(descend) order at given direction
  void		TrimVts(float fSt,float fEd,int iMode,const varray<Vec4>& vtsArr,varray<Vec4>& vtsTrim);//iMode 0: x direction,1:y direction 2: z direction

//Loop
// Divide a loop with equal angle to create a new loop
  void 		DivideLoopByAngle(Vec4 *plist, int &npt, Vec4 norm, int nump, float sang);
  void 		DivideLoopByAngle(varray<Vec4>& plist, int &npt, Vec4 norm, int nump, float sang);
//Divide the inputed loop by angle equivalent
  void 		DivideLoopByAngle(const varray<Vec4>& ptsIn,varray<Vec4>& ptsOut, Vec4 norm, int nump, float sang,Vec4 vtCen = Vec4(-10e6,-10e6,-10e6));
//Divide the inputed loop by distance equivalent
  void		DivideLoopByDis(const varray<Vec4>& ptsIn, varray<Vec4>& plist,Vec4 vtCen,int nump);
  void		DivideLoopByDis(const varray<Vec4>& ptsIn, varray<Vec4>& plist,Vec4 vtCen,float fXAidedMid,int nump);
  void		DivideLoopByDis(const varray<Vec4>& ptsIn, varray<Vec4>& plist,Vec4 vtCen,const varray<int>& numpArr,bool bIsAnticlockwise);
//Arrange the loop in anticlockwise,the loop should be in well topology
  void 		AntiClockWiseLoop(varray<Vec4>&ptsArr);
//interpolate operator
  bool		InterpolateANewLoop(varray<Vec4>& intputLoop1, varray<Vec4>& intputLoop2, float ratio, varray<Vec4>& newLoop);//XXX

//Slice
//Refine a slice to avoid wrinkle
  void		RefineSlice1(varray<Vec4>& plist, int &nump);
  void		RefineSlice1(Vec4* plist, int &nump);
//Get the enclosing box points and index of the inputed slice points
  void        GetBoundPointsOfSlice(Vec4 *slice, int num, Vec4 *fpt, int *idx);
  void        GetBoundPointsOfSlice(varray<Vec4>& slice, int num, Vec4 *fpt, int *idx);
  bool        GetBBoxExtremPtIdx(const varray<Vec4>& parr,int& idx1,int& idx2,int imode);

//////////////////////////////////////////////////////////////////////////
//for mesh
//collision detection and avoidance for two xmesh
  //void        XMeshColliXMesh(XBaseMesh* resMesh,XBaseMesh* basMesh);
 // bool        LoadObj(string filename, XBaseMesh* mesh);
 // bool        SaveObj(string filename, XBaseMesh* mesh);
  bool        SaveXBaseMeshData(FileStream& out, const XBaseMesh* mesh);
//////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////
//for read and save function

  void        GetVariantLengthIntLine(/*int strnum, */const char str[],/* int valnum, */int *hidx/*,bool bZeroStart=false*/);
 
  //void        ConstructMeshFromFaceidx(XBaseMesh* oripms, varray<int>& includeFaceidx, XBaseMesh* newpms);   

#endif