#include "XFunction.h"
#include <qmatrix4x4.h>

using namespace std;

Vec4 RotAxis(Vec4 pt, Vec4 vect,double ang)
{
	Vec4 rvt;
	double xa,ya;

	vect.ComputeGlbAng(&xa,&ya);

	rvt = pt;
	rvt=RotateX(rvt, xa);
	rvt=RotateY(rvt, ya);
	rvt=RotateZ(rvt, ang);
	rvt=RotateY(rvt, -ya);
	rvt=RotateX(rvt, -xa);

	return rvt;
}


Vec4 RotateX(Vec4 pt, float ang)
{
	Vec4 v;
	float sa, ca;

	sa=(float)sin(ang);
	ca=(float)cos(ang);

	v.x = pt.x;
	v.y = pt.y*ca-pt.z*sa;
	v.z = pt.y*sa+pt.z*ca;

	return v;
}


Vec4 RotateY(Vec4 pt, float ang)
{
	Vec4 v;
	float sa, ca;

	sa=(float)sin(ang);
	ca=(float)cos(ang);

	v.x = pt.x*ca+pt.z*sa;
	v.y = pt.y;
	v.z = -pt.x*sa+pt.z*ca;

	return v;
}

Vec4 RotateZ(Vec4 pt, float ang)
{
	Vec4 v;
	float sa, ca;

	sa=(float)sin(ang);
	ca=(float)cos(ang);

	v.x = pt.x*ca-pt.y*sa;
	v.y = pt.x*sa+pt.y*ca;
	v.z = pt.z;

	return v;
}

bool IsPointIn3DTriangle(const Vec4 &testP, const Vec4 &vertex1, const Vec4 &vertex2, const Vec4 &vertex3, bool bIsFast/* = false*/)
{
	Vec4 vert[3] = { vertex1, vertex2, vertex3 };
	const float dis = (float)1.0e-4;
	//use box test to accelerate the speed,XXX XXX,2006.3.30 
	if(bIsFast)
	{
		float xMax,xMin,yMax,yMin,zMax,zMin;
		xMax = max(vert[0].x,max(vert[1].x,vert[2].x))+dis;
		xMin = min(vert[0].x,min(vert[1].x,vert[2].x))-dis;
		yMax = max(vert[0].y,max(vert[1].y,vert[2].y))+dis;
		yMin = min(vert[0].y,min(vert[1].y,vert[2].y))-dis;
		zMax = max(vert[0].z,max(vert[1].z,vert[2].z))+dis;
		zMin = min(vert[0].z,min(vert[1].z,vert[2].z))-dis;
		if(testP.x > xMax || testP.x < xMin || testP.y > yMax || testP.y < yMin || testP.z > zMax || testP.z < zMin)
		{
			return false;
		}
	}

	Vec4 norm = CrossVecX(vert[2]-vert[1], vert[0]-vert[1]).Normalize();// get the face normal

	for (int i=0; i<3; i++)// judge if same plane
	{
		if ((testP-vert[i]).Magnitude() < dis) // too close to vertex
			continue;

		float dot = Dot((testP-vert[i]).Normalize(), norm);
		// change from 1.0e-2 to 1.0e-4, more precision, by XXX[12/14/2005]
		if (fabs(dot) > 1.0e-4) // not in same plane 
			return false;	
	}
	for (int i=0; i<3; i++)// judge if in triangle
	{
		if ((testP-vert[(i+1)%3]).Magnitude() < dis || (testP-vert[i]).Magnitude() < dis) // too close
			continue;

		Vec4 tempNorm = CrossVecX(testP-vert[i], testP-vert[(i+1)%3]).Normalize();
		if (Dot(tempNorm, norm) < 0) // point out of triangle
			return false;
	}

	return true;
}
bool IsInTheVarray(int ID,const varray<int> &pVarray)
{
	int *p = std::find(pVarray.begin(), pVarray.end(), ID);
	if(p == pVarray.end())
		return false;
	return true;
}
bool IsInTheVarray(Vec4 element, varray<Vec4> &L)  
{
	if (L.size() <= 0) 
		return false;

	for(int i=0;i<static_cast<int>(L.size());i++)
	{
		Vec4 tmp = L[i];
		if(tmp.Equals(element, (float)1.0e-6)) 
		{
			return true;
		}
	}
	return false;	
}
int GetInTheVarrayIndex(int ID, const varray<int> &pVarray)
{
	int* location = std::find(pVarray.begin(),pVarray.end(),ID);
	if(location != pVarray.end())
		return static_cast<int>(location - pVarray.begin());
	else
		return -1;;
}

float GetLineLength(const Vec4 &p1, const Vec4 &p2)
{
	return (p1- p2).Magnitude();
}
Vec4 GetMappingVertOnLine(const Vec4& Vec4Point, const Vec4& linestartpt, const Vec4& lineendpt) 
{
	Vec4 result;
	//double R;

	double mag1 = (Vec4Point-linestartpt).SquareMagnitude();
	double mag2 = (Vec4Point-lineendpt).SquareMagnitude();
	if(mag1<1.0e-8)
	{
		return linestartpt;
	}
	if(mag2<1.0e-8)
	{
		return lineendpt;
	}
	Vec4 dir = (lineendpt-linestartpt).Normalize();
	double dot = Dot((Vec4Point-linestartpt),dir);
	Vec4 res = linestartpt+(float)dot*dir;
	return res;

}

bool IsSharpAngleOfTwoVert(const Vec4 & vectOne,const Vec4 &vectTwo) 
{
	if(Dot(vectOne,vectTwo ) > 1e-6 ) // yao 2002-03-21
	{	
		return true;
	}
	return false;
}
//void DrawLine(const Vec4& pt1, const Vec4& pt2) 
//{	
//	glBegin(GL_LINES);
//	glVertex3f(pt1.x,pt1.y,pt1.z);
//	glVertex3f(pt2.x,pt2.y,pt2.z);
//	glEnd();
//}
//void DrawLine(const Vec2& pt1, const Vec2& pt2)
//{	
//	glBegin(GL_LINES);
//	glVertex3f(pt1.x, pt1.y, 0.0f);
//	glVertex3f(pt2.x, pt2.y, 0.0f);
//	glEnd();
//}
//void DrawPoint(const Vec4 &Vec4Point)
//{
//	glBegin(GL_POINTS);
//	glVertex3f(Vec4Point.x,Vec4Point.y,Vec4Point.z);
//	glEnd();
//}
//void DrawOnePt(const Vec4& pt,double dPointSize)
//{
//	glPushMatrix();
//	glTranslated(pt.x,pt.y,pt.z);
//	GLUquadricObj*	q = gluNewQuadric();
//	gluQuadricDrawStyle(q, GLU_FILL);
//	gluQuadricNormals(q, GLU_SMOOTH);
//	gluSphere(q, GLdouble(dPointSize), 20, 20);
//	gluDeleteQuadric(q);
//	glPopMatrix();
//}
bool IsTwoPtVeryNear(const Vec4 &pa,const Vec4 &pb)
{	
	const double RangeLong = 0.0001;
	if( (pa - pb).Magnitude() < RangeLong )
	{
		return true;
	}
	else
	{
		return false;
	}
}
bool IsVertBetweenTwoVerts(const Vec4& Vert, const Vec4& Vec4First, const Vec4& Vec4Last) 
{
	if(((Vert.x<=Vec4First.x && Vert.x>=Vec4Last.x)||(Vert.x>=Vec4First.x&&Vert.x<=Vec4Last.x))
		&&((Vert.y<=Vec4First.y && Vert.y>=Vec4Last.y)||(Vert.y>=Vec4First.y&&Vert.y<=Vec4Last.y))
		&&((Vert.z<=Vec4First.z && Vert.z>=Vec4Last.z)||(Vert.z>=Vec4First.z&&Vert.z<=Vec4Last.z)))
	{
		return true;
	}
	else
	{
		return false;
	}
}
bool IsPointInTriangle(const Vec4 &testP, const Vec4 &vertex1, const Vec4 &vertex2, const Vec4 &vertex3,const float ThreshHold)
{
	Vec4 cross1=CrossVecX(testP-vertex1, testP-vertex2).Normalize();
	Vec4 cross2=CrossVecX(testP-vertex2, testP-vertex3).Normalize();
	Vec4 cross3=CrossVecX(testP-vertex3, testP-vertex1).Normalize();

	Vec4 ptTemp1,ptTemp2;
	for(int i=0;i<3;i++)// to test if the point is on the edge of triangle
	{
		switch(i)
		{
		case 0:
			{
				ptTemp1 = vertex1;
				ptTemp2 = vertex2;
				break;
			}
		case 1:
			{
				ptTemp1 = vertex2;
				ptTemp2 = vertex3;
				break;
			}
		case 2:
			{
				ptTemp1 = vertex3;
				ptTemp2 = vertex1;
				break;
			}
		default:
			break;
		}
		float fLen1 = (testP-ptTemp1).Magnitude();
		float fLen2 = (ptTemp2-testP).Magnitude();
		float fLen3 = (ptTemp2-ptTemp1).Magnitude();
		float fLen  = fLen1 + fLen2 - fLen3;
		if( fabs(fLen)<ThreshHold)	
		{ 
			return true;
		}
	}
	if((!IsSharpAngleOfTwoVert(cross1,cross2) ) || (!IsSharpAngleOfTwoVert(cross1,cross3) ) 
		||(!IsSharpAngleOfTwoVert(cross3,cross2) ) )
	{ 
		return false;
	}
	return true;
}
double GetDistanceFromPtToLine(const Vec4& Vec4Point, const Vec4& lineStartPt, const Vec4& lineEndPt)
{
	double len=(lineEndPt - lineStartPt).Magnitude();
	if(len < 1.0e-4) //XXX modified on 2005-03-25
	{
		len = (Vec4Point - lineStartPt).Magnitude();
		return len;
	}
	double temp = GetSinofAngleAOB(Vec4Point, lineStartPt, lineEndPt);
	return temp * (lineStartPt - Vec4Point).Magnitude();
}
float GetDistanceFromPtToTriangle(const Vec4& Ve3Point, const Vec4& P0, const Vec4& P1, const Vec4& P2, Vec4& interPt)
{
	Vec4 e1 = P1 - P0;
	Vec4 e2 = P2 - P0;

	Vec4 e0 = P0 - Ve3Point;
	float a,b,c,d,e,f;
	a = Dot(e1,e1);
	b = Dot(e1,e2);
	c = Dot(e2,e2);
	d = Dot(e1,e0);
	e = Dot(e2,e0);
	f = Dot(e0,e0);

	float det,s,t;
	det = a*c - b*b;
	s = b*e - c*d;
	t = b*d - a*e;

	if (s+t <= det)
	{
		if (s < 0.0f)
		{
			if (t < 0.0f)
			{
				//Region 4
				if (d > e)
				{
					s = d >= 0.0f ? 0.0f:(-d >= a ? 1.0f:-d/a);
					t = 0.0f;
				}
				else
				{
					s = 0.0f;
					t = e >= 0.0f ? 0.0f:(-e >= c ? 1.0f:-e/c);
				}
			}
			else
			{
				//Region 3
				s = 0.0f;
				t = e >= 0.0f ? 0.0f:(-e >= c ? 1.0f:-e/c);
			}
		}
		else
		{
			if (t < 0.0f)
			{
				//Region 5
				s = d >= 0.0f ? 0.0f:(-d >= a ? 1.0f:-d/a);
				t = 0.0f;
			}
			else
			{
				//Region 0
				float invDet = 1.0f/det;
				s *= invDet;
				t *= invDet;
			}
		}
	}
	else
	{
		if (s < 0.0f)
		{
			//Region 2
			float tmp0 = b + d;
			float tmp1 = c + e;
			if (tmp1 > tmp0)
			{
				float numer = tmp1 - tmp0;
				float denom = a - 2.0f*b + c;
				s = (numer >= denom)?1.0f:(numer/denom);
				t = 1.0f - s;
			}
			else
			{
				s = 0.0f;
				t = (e >= 0.0f)?0.0f:( -e >= c ? 1.0f:-e/c);
			}
		}
		else if (t < 0.0f)
		{
			//Region 6
			float tmp0 = b + e;
			float tmp1 = a + d;
			if (tmp1 > tmp0)
			{
				float numer = tmp1 - tmp0;
				float denom = a - 2.0f*b + c;
				t = (numer >= denom)?1.0f:(numer/denom);
				s = 1.0f - t;
			}
			else
			{
				t = 0.0f;
				s = d >= 0.0f ? 0.0f:(-d >= a ? 1.0f:-d/a);
			}
		}
		else
		{
			//Region 1
			float numer = c + e - b - d;
			if (numer <= 0.0f)
			{
				s =0.0f;
			}
			else
			{
				float denom = a - 2.0f*b + c;
				s = (numer >= denom ? 1.0f:numer/denom);
			}
			t = 1.0f - s;
		}
	}

	interPt = P0 +s*e1+t*e2;
	return (Ve3Point - interPt).Magnitude();

}
Vec4 GetPlumbPointAtLine(const Vec4 &lineV0, const Vec4 & lineV1, const Vec4 & sourceV)
{
	float t = ((lineV1.x-lineV0.x)*(sourceV.x-lineV0.x) + (lineV1.y-lineV0.y)*(sourceV.y-lineV0.y) + (lineV1.z-lineV0.z)*(sourceV.z-lineV0.z))
		/ ((lineV1.x-lineV0.x)*(lineV1.x-lineV0.x) + (lineV1.y-lineV0.y)*(lineV1.y-lineV0.y)  + (lineV1.z-lineV0.z)*(lineV1.z-lineV0.z) );
	Vec4 plumbPoint;
	plumbPoint.x = lineV0.x + (lineV1.x-lineV0.x)*t;
	plumbPoint.y = lineV0.y + (lineV1.y-lineV0.y)*t;
	plumbPoint.z = lineV0.z + (lineV1.z-lineV0.z)*t;
	return plumbPoint;
}
//---------------------------------------------------------------
// Name:		Intersection
// Description: get the length ratio of the two intersecting lines
// Argument:	v1: line one first point
//				v2  line one second point
//				v3: line two first point
//				v4: line two second point
//				r:  first line ratio: length start point to intersection with length of start point to end point  
//				s:  second line ratio: length start point to intersection with length of start point to end point		
//---------------------------------------------------------------- 
bool GetTwoLineIntersectRatio(const Vec4 &v1, const Vec4 &v2, const Vec4 &v3, const Vec4 &v4, float &r, float &s)
{
	float temp1,temp2,temp3;		
	temp1=(v1.y-v3.y)*(v4.x-v3.x)-(v1.x-v3.x)*(v4.y-v3.y);
	temp2=(v2.x-v1.x)*(v4.y-v3.y)-(v2.y-v1.y)*(v4.x-v3.x);
	temp3=(v1.y-v3.y)*(v2.x-v1.x)-(v1.x-v3.x)*(v2.y-v1.y);

	if(temp2 == 0)
	{
		return false;
	}	

	r=temp1/temp2;
	s=temp3/temp2;
	return true;
}

//---------------------------------------------------------------
// Name:		GetTwoLineInterPt2D
// Description: Get intersection point of two 2D lines
// Argument:			ln1StPt, ln1EndPt: the start and end point of line1
//						ln2StPt, ln2EndPt: the start and end point of line2
// Return:		2D intersect point	
//---------------------------------------------------------------- 
Vec2 GetTwoLineInterPt2D(Vec2 ln1StPt,Vec2 ln1EndPt,Vec2 ln2StPt,Vec2 ln2EndPt)
{
	Vec4 v1(ln1StPt.x,  ln1StPt.y, 0.0);
	Vec4 v2(ln1EndPt.x, ln1EndPt.y, 0.0);
	Vec4 v3(ln2StPt.x, ln2StPt.y, 0.0);
	Vec4 v4(ln2EndPt.x, ln2EndPt.y, 0.0);
	Vec4 res = GetTwoLineInterPt(v1, v2, v3, v4);
	return(Vec2(res.x,res.y));
}

//---------------------------------------------------------------
// Name:		GetTwoLineInterPt
// Description: Get intersection point of two lines
// Argument:			ln1StPt, ln1EndPt: the start and end point of line1
//						ln2StPt, ln2EndPt: the start and end point of line2
// Return:		intersect point	
//---------------------------------------------------------------- 
Vec4 GetTwoLineInterPt(Vec4 ln1StPt,Vec4 ln1EndPt,Vec4 ln2StPt,Vec4 ln2EndPt)
{
	float r, s;

	if(GetTwoLineIntersectRatio(ln1StPt,ln1EndPt,ln2StPt,ln2EndPt,r,s))
	{
		return (ln1StPt + r * (ln1EndPt - ln1StPt) );
	}
	else
	{
		return ln1EndPt;
	}
}
Vec4 GetLineFaceIntersectPnt(Vec4& lsp, Vec4& lep, Vec4& fp, Vec4& fnorm)
{	
	Vec4 vinterpoint;	//point of intersection
	float testf = Dot( (lep-lsp).Normalize() , fnorm.Normalize() );
	if(fabs(testf) < 1.0e-6)
	{
		vinterpoint = lsp;
		return vinterpoint;
	}
	// the following code may not suitable for future. need optimization.
	vinterpoint = lsp + (lep-lsp) * (( Dot( (fp-lsp) , fnorm ) ) / Dot( (lep-lsp) , fnorm ));
	return vinterpoint;
}

//----------------------------------------------------------------------
// name:		MapAVertexToASegment
// function:	get the projection of point on a segment(3D)
//				If the perpendicular foot of a vertex to a segment is not on the segment
//				then return the endpoint nearest to the vertex as the mapping point
//				else return the perpendicular foot  
// argument:	segstartpt,segendpt: end and start point of segment
//				vertex: vertex
// return:		project point of v1
//----------------------------------------------------------------------
Vec4 MapAVertexToASegment(const Vec4& vertex,  const Vec4& segstartpt, const Vec4& segendpt)
{
	Vec4 res = GetMappingVertOnLine(vertex, segstartpt, segendpt);
	float segLen = (segstartpt - segendpt).Magnitude();
	float tempLen = (segstartpt - res).Magnitude() + (res - segendpt).Magnitude();
	if(fabs(tempLen-segLen) > 1.0e-4)
	{  //not on segment
		res = (segstartpt - res).Magnitude() < (res - segendpt).Magnitude() ? segstartpt:segendpt;
	}	
	return res;
}
//----------------------------------------------------------------------
// name:		MapAVertexToASegment
// function:	get the projection of point on a segment(3D) 
// argument:	segstartpt,segendpt: end and start point of segment
//				vertex: vertex
//				mappedPos: project pooint
// return:		if the mapping point is on the segment, return true; otherwise return false
//----------------------------------------------------------------------
bool MapAVertexToASegment(const Vec4& vertex,  const Vec4& segstartpt, const Vec4& segendpt, Vec4& mappedPos)
{
	Vec4 res =  GetMappingVertOnLine(vertex, segstartpt, segendpt);
	float segLen = (segstartpt - segendpt).Magnitude();
	float tempLen = (segstartpt - res).Magnitude()+(res - segendpt).Magnitude();
	if(fabs(tempLen-segLen) > 1.0e-4)
	{
		return false;
	}	
	mappedPos=res;
	return true;
}
//---------------------------------------------------------------
// Name:		IsTwoPointsAtLineDiffSideOnFace
// Description: the two points is at the face's same side or not, 
//				the face is vertical to the face that the line located
// Argument:	pointOne, pointTwo: point1, point2		
//				lineSt, lineSt: the start and end point of line
//				faceNorm: the face normal the line located
// Return:		false for at same side
//---------------------------------------------------------------- 
bool IsTwoPointsAtLineDiffSideOnFace(const Vec4 &pointOne,const Vec4 &pointTwo,
									 const Vec4 &lineSt,const Vec4 &LineEnd,const Vec4 &faceNorm)
{
	const double RangeLong=0.0001;

	if(lineSt.Equals(LineEnd, (float)RangeLong) )//XXX 2002-12-09
	{
		return false;
	}

	Vec4 interfnorm;
	interfnorm = CrossVecX(faceNorm,(LineEnd - lineSt)).Normalize();

	float dist1, dist2;
	dist1 = Dot(pointOne-lineSt, interfnorm);
	dist2 = Dot(pointTwo-lineSt, interfnorm);

	if(dist1*dist2 <= 0)
	{
		return true;
	}
	else if(fabs(dist1) <= RangeLong || fabs(dist2) <= RangeLong)
	{
		return true;
	}
	else 
	{
		return false;
	}
}
//----------------------------------------------------------------------
// Name:		GetAngleOf2Vector(...)
// Function:    Compute the angle of two vectors
//				angle = acos(dot(v1,v2)/|v1||v2|)
// Argument:	Vec4 &v1 - the first vector
//				Vec4 &v2 - the second vector
// Return:		the angle of two vectors: [0, pi]
//----------------------------------------------------------------------
float GetAngleOf2Vector(const Vec4 &v1, const Vec4 &v2)
{
	float len1, len2, r, val, cos, angle;

	len1=sqrt(v1.x*v1.x+v1.y*v1.y+v1.z*v1.z);
	len2=sqrt(v2.x*v2.x+v2.y*v2.y+v2.z*v2.z);
	r=v1.x*v2.x+v1.y*v2.y+v1.z*v2.z;

	val = len1*len2;

	if(fabs(val)<1.0e-8)  
	{
		angle = 0;
		return 0.0f;
	}
	else
	{
		cos=r/val;
		if(cos >= 1.0) 
		{
			return 0.0f;
		}
		else if(cos <= -1.0) 
		{
			return static_cast<float>(Math::Pi);
		}

		angle=acos(cos);
	}

	return angle;
}
/********************************************************************
* FUNCTION NAME:int InterPointLengthCompare( const void *arg1, const void *arg2 )
* MODIFIER     :
* MODIFY DATE  :
* DESCRIPTION  :compare point length when use quick sort, use this function to compare from "short" to "long"
* PARAMETER    :input:const void *arg1, const void *arg2
********************************************************************/
int InterPointLengthCompare( const void *arg1, const void *arg2)
{
	if((((_MyLengthCompute*)arg1)->length>((_MyLengthCompute*)arg2)->length))
	{
		return 1;
	}
	else if(((((_MyLengthCompute*)arg1)->length<((_MyLengthCompute*)arg2)->length)))
	{
		return -1;
	}
	else
	{
		return 0;
	}
}
float GetDistOfPntToLn(Vec4 pt, Vec4 p1, Vec4 p2)
{
	float dist;
	float vl,vm,vn,a,b,c,d;

	vl = p2.x - p1.x;
	vm = p2.y - p1.y;
	vn = p2.z - p1.z; 

	a = ((pt.x-p1.x)*vm - (pt.y-p1.y)*vl) * ((pt.x-p1.x)*vm - (pt.y-p1.y)*vl);
	b = ((pt.y-p1.y)*vn - (pt.z-p1.z)*vm) * ((pt.y-p1.y)*vn - (pt.z-p1.z)*vm);
	c = ((pt.z-p1.z)*vl - (pt.x-p1.x)*vn) * ((pt.z-p1.z)*vl - (pt.x-p1.x)*vn);
	d = vl*vl + vm*vm + vn*vn;

	if(d > 1.0e-6)
	{
		dist=(float)sqrt((a+b+c)/d);
	}
	else // error 
	{
		dist = (pt - p1).Magnitude();
	}

	return dist;
}
//---------------------------------------------------------------
// Name:		IsTwoLineIntersectedOnFace
// Description: the two line is intersect or not on a face
// Argument:			lineOneSt, lineOneEnd: the start and end point of line1
//						lineTwoSt, lineTwoEnd: the start and end point of line2
//						faceNorm:			   face normal
//---------------------------------------------------------------- 
bool IsTwoLineIntersectedOnFace(const Vec4 &lineOneSt,const Vec4 &lineOneEnd,
								const Vec4 &lineTwoSt,const Vec4 &LineTwoEnd,const Vec4 &faceNorm)
{
	if(IsTwoPointsAtLineDiffSideOnFace( lineOneSt,lineOneEnd,lineTwoSt,LineTwoEnd,faceNorm) 
		&& IsTwoPointsAtLineDiffSideOnFace( lineTwoSt,LineTwoEnd,lineOneSt,lineOneEnd,faceNorm) )
	{
		return true;
	}

	return false;
}
//----------------------------------------------------------------------
// Name:		GetCosofAngleAOB(...)
// Function:    calculate cosecant value of the angle formed by the line PaPt and the line PaPb
// Argument:	PtA, Pt0, PtB: three points
// Return:		coseant value of the angle
//----------------------------------------------------------------------
double GetCosofAngleAOB(const Vec4& PtA, const Vec4& PtO,const Vec4& PtB ) 
{
	Vec4 p0(PtA);
	Vec4 p1(PtB);
	p0 -= PtO;
	p1 -= PtO;
	double mag1 = p0.Magnitude();
	double mag2 = p1.Magnitude();
	if(mag1 < 1.0e-4)
	{
		return 1.0;
	}
	if(mag2 < 1.0e-4)
	{
		return 1.0;
	}
	mag1 = Dot(p0,p1)/(mag1*mag2);
	if(mag1 > 1)
		mag1 = 1;
	else if(mag1 < -1)
		mag1 = -1;
	return mag1;
}

//----------------------------------------------------------------------
// FUNCTION NAME: GetSinofAngleAOB
// DESCRIPTION  : calculate sine value of the angle formed by the line PaPt and the line PaPb
// PARAMETER    : input: Vec4 Pa, Vec4 Pt, Vec4 Pb
// output:        the sine value
// version      : 1.0
//----------------------------------------------------------------------
double GetSinofAngleAOB(const Vec4& PtA, const Vec4& PtO,const Vec4& PtB ) 
{
	double cos = GetCosofAngleAOB(PtA, PtO,PtB );
	return sqrt(1 - cos * cos);
}
//----------------------------------------------------------------------
// name:		GetMappingPointOnAPlane
// function:	get the projection of point on a plane
// argument:	inputP: input point
//				planePnt: plane pnt
//				planeNormal: plane normal
//----------------------------------------------------------------------
Vec4 GetMappingPointOnAPlane(Vec4 inputP, Vec4 planePnt, Vec4 planeNormal)
{
	Vec4 mapPt = inputP - Dot(planeNormal, inputP - planePnt)*planeNormal/planeNormal.SquareMagnitude();
	return mapPt;
}
//----------------------------------------------------------------------
// name:		GetMappingPointOnAPlane
// function:	get the projection of point on a line
// argument:	inputP: input point
//				planeP1, planeP2,planeP3: three points to define the plane
// return:		project point
//----------------------------------------------------------------------
Vec4 GetMappingPointOnAPlane(Vec4 inputP, Vec4 planeP1, Vec4 planeP2, Vec4 planeP3)
{
	Vec4 norm = CrossVecX(planeP2-planeP1, planeP3-planeP1)/*.Normalize()*/;
	Vec4 mapPt= inputP - Dot(norm, inputP - planeP1)*norm/norm.SquareMagnitude();
	return mapPt;
}
//---------------------------------------------------------------
// Name:		IsPointInTheBoxOfTriangle(...)
// Description:	judge whether a testing point is in a box of a triangle
// Argument:	testingP: the point to be checked
//				v1, v2, v3: three points of a triangle
// Return:		true: a point is inside the box of a triangle; false: the point is outside of the box of the triangle
//---------------------------------------------------------------
bool IsPointInTheBoxOfTriangle(Vec4 testingP, Vec4 v1, Vec4 v2, Vec4 v3)
{
	float maxX=v1.x;
	float maxY=v1.y;
	float minX=v1.x;
	float minY=v1.y;

	if(v1.x < v2.x)
	{
		maxX=v2.x;
	}
	else
	{
		minX=v2.x;
	}

	if(maxX<v3.x)
	{
		maxX=v3.x;
	}

	if(minX>v3.x)
	{
		minX=v3.x;
	}

	if(v1.y < v2.y)
	{
		maxY=v2.y;
	}
	else
	{
		minY=v2.y;
	}

	if(maxY<v3.y)
	{
		maxY=v3.y;
	}

	if(minY>v3.y)
	{
		minY=v3.y;
	}

	bool rtn;
	if((testingP.x>=minX && testingP.x<=maxX) && (testingP.y>=minY && testingP.y<=maxY))
	{
		rtn = true;
	}
	else
	{
		rtn = false;
	}

	return rtn;
}
//----------------------------------------------------------------------
// name:		GetDistanceFromPtToLineSeg
// function:	compute the distance from a point to a lineseg
// argument:	Vec4 Vec4Point: point
//				Vec4 lineStartPt, Vec4 lineEndPt: line point
// return:		distance of point to a lineseg
//----------------------------------------------------------------------
float GetDistanceFromPtToLineSeg(const Vec4& Vec4Point, const Vec4& lineStartPt, const Vec4& lineEndPt, Vec4& interPt)
{
	Vec4 vec1 = lineEndPt - lineStartPt;
	Vec4 vec2 = Vec4Point - lineStartPt;

	float temp = vec1.SquareMagnitude();
	if(temp < 1.0e-6) 
	{
		interPt = lineStartPt;
		return vec2.Magnitude();
	}

	float t = Dot(vec1,vec2);
	if (t<=0)
	{
		interPt = lineStartPt;
		return vec2.Magnitude();
	}

	if (t>temp)
	{
		interPt = lineEndPt;
		return (Vec4Point - lineEndPt).Magnitude();
	}

	interPt = lineStartPt+vec1*t/temp;
	return sqrt(vec2.SquareMagnitude() - t*t/temp);

}
//---------------------------------------------------------------
// Name:	    GetCenter(Vec4* pList,int nv)
// Description: compute the center of a loop
// Argument:    pList:-- points in the loop; 
//         :	iNum:-- points number in the loop
// Return:		Vec4:-- loop center
//----------------------------------------------------------------
Vec4 GetCenter(const varray<Vec4>& plist,int iNum)
{
	int i = 0;
	Vec4 vtCenter = Vec4(0,0,0);
	if(iNum == 0)
	{
		return vtCenter;
	}
	int iActNum = 0;
	for( i = 0; i < iNum; i++)
	{
		if(i < iNum-1)
		{
			vtCenter += plist[i];
			iActNum++;
		}
		else if((plist[0]-plist[iNum-1]).Magnitude() > 1.0e-3)
		{
			vtCenter += plist[i];
			iActNum++;
		}
	}
	return vtCenter/(float)iActNum;
}

//---------------------------------------------------------------
// Name:	    GetCenter(Vec4* pList,int nv)
// Description: compute the center of a loop
// Argument:    pList:-- points in the loop; 
//         :	iNum:-- points number in the loop
// Return:		Vec4:-- loop center
//----------------------------------------------------------------
Vec4 GetCenter(Vec4* plist,int iNum)
{
	int i = 0;
	Vec4 vtCenter = Vec4(0,0,0);
	if(iNum == 0)
	{
		return vtCenter;
	}
	int iActNum = 0;
	for( i = 0; i < iNum; i++)
	{
		if(i < iNum-1)
		{
			vtCenter += plist[i];
			iActNum++;
		}
		else if((plist[0]-plist[iNum-1]).Magnitude() > 1.0e-3)
		{
			vtCenter += plist[i];
			iActNum++;
		}
	}
	return vtCenter/(float)iActNum;
}
//---------------------------------------------------------------
// Name:	    GetCenter(const varray<Vec4>& ctrlPtsArr) 
// Description: compute the center of a loop
// Argument:    ctrlPtsArr:-- points array in the loop; 
//         :	
// Return:		Vec4:-- loop center
//----------------------------------------------------------------
Vec4 GetCenter(const varray<Vec4>& ctrlPtsArr)
{
	Vec4 vtCenter = Vec4(0.0,0.0,0.0);
	if(ctrlPtsArr.size() == 0)
	{
		return vtCenter;
	}
	//if first and last point is too near, do not use last point
	int t = 0; 
	if((ctrlPtsArr.at(0) - ctrlPtsArr.back()).Magnitude() < 1.0e-1)
	{
		t = 1;
	}
	for(int i = 0; i < static_cast<int>(ctrlPtsArr.size()- t); i++)
	{
		vtCenter += ctrlPtsArr.at(i);
	}
	return vtCenter/(float)(ctrlPtsArr.size()-t);
}
bool SaveObj(string filename, XBaseMesh* mesh)
{
	try
	{
		//SetFileAttributes(filename.c_str(), FILE_ATTRIBUTE_NORMAL);
		FileStream out(filename.c_str(), FileStream::WRITE);
		SaveXBaseMeshData(out,mesh);
		out.close();
	}
	catch(...)
	{
		/*AfxMessageBox(_T("数据存储错误"));*/
		return false;
	}
	return true;
}
bool LoadObj(string filename, XBaseMesh *ms)
{
	int		i;
	char	str[200];
	char	s[3];
	Vec4	vt, norm;
	ULONG	numv = 0L;  //vertex number
	ULONG	numn = 0L;	//normal vector number
	ULONG	numt = 0L;  //textture number
	ULONG	numf = 0L;  //face number
	
	float	x, y, z;
	int		nf;
	varray<IntVec4> fidx;

	FILE *fp = NULL;
	const char* filenm = filename.c_str();
	fp = fopen(filenm, "r");
	if(fp == NULL)
	{
		return false;
	}

	while (fgets(str, sizeof(str), fp) != NULL)
	{
		switch( str[0] )
		{
		case 'v':
			switch( str[1] )
			{
			case ' ':	// vertex
				numv++;
				break;
			case '\t':
				numv++;
				break;
			case 'n':	//	normal
				numn++;
				break;
			case 't':	//	texture coord
				numt++;
				break;

			default:
				break;
			}
			break;
		case 'f':
			nf = GetFaceNumber(str, fidx);
			numf += nf;
			fidx.resize(0);
			break;
		default:
			break;
		}
	}

	// check if the file is valid, add by wangmei 2005.6.15
	if (numv == 0 || numf == 0)
	{
		if(fp) // Added by XXX [3/28/2007] if opened the file , close it
			fclose(fp);
		return false;
	}
	//////////////
	ms->SetVSize(numv);  // number of points
	//ms->SetTVertCount(numt);  
	//ms->SetNormCount(numn);  
	ms->SetFSize(numf);   // number of triangles

	// go to the head of the file
	rewind( fp );
	numv = 0L;
	numn = 0L;
	numt = 0L;
	numf = 0L;

	while (fgets( str, sizeof(str), fp ) != NULL )	
	{
		switch( str[0] )	
		{
		case 'v':
			memset( s, 0x00, sizeof(s) );
			switch( str[1] )	
			{
			case ' ':	//	set vertex
				sscanf( str, "%s %f %f %f", s, &x, &y, &z );
				ms->GetV(numv).Pos() = Vec4(x, y, z );
				ms->GetV(numv).m_vid = numv;
				numv++;
				break;
			case '\t':	//	set vertex
				sscanf( str, "%s %f %f %f", s, &x, &y, &z );
				ms->GetV(numv).Pos() = Vec4(x, y, z );
				ms->GetV(numv).m_vid = numv;
				numv++;
				break;
			case 'n':	//	set normal
				sscanf( str, "%s %f %f %f", s, &x, &y, &z );

				//ms->GetNorm(numn) = Vec4(x, y, z );					
				numn++;
				break;
			case 't':	//	set normal
				sscanf( str, "%s %f %f %f", s, &x, &y, &z );
				//if(numt<ms->GetVertCount())
				//{
				//	ms->GetTVert(numt) = Vec4(x, y, z); 
				//	numt++;
				//}
				break;
			default:
				break;
			}
			break;
		case 'f':      // 
			if ( str[1] == ' ' || str[1] == '\t')	
			{
				for(i=0; i<GetFaceNumber(str, fidx); i++)
				{
					ms->GetF(numf).SetIndex(fidx[i].x-1, fidx[i].y-1, fidx[i].z-1);
					numf++;
				}
				fidx.resize(0);
			}
			break;
		default:
			break;
		}
	}
	fclose(fp);

	// check the validity of the obj file
	for(i=0; i<static_cast<int>(numf); i++)
	{
		if(ms->GetF(i).GetIndex(0)>=static_cast<int>(numv) 
			|| ms->GetF(i).GetIndex(1)>=static_cast<int>(numv) 
			|| ms->GetF(i).GetIndex(2)>=static_cast<int>(numv))
			return false;
	}

	return true;
}
bool SaveXBaseMeshData(FileStream& out, const XBaseMesh* mesh)
{
	int i;
	out.set_ascii(true);
	int vsize = (int)mesh->GetVSize();
	for (i = 0; i < vsize; i++) {
		const Vec4& v = mesh->GetV(i).Pos();
		out << "v " << v.x << " " << v.y << " " << v.z << "\n";
	}

	for (i = 0; i < vsize; i++) {
		const Vec4& v = mesh->GetV(i).Norm();
		out << "vn " << v.x << " " << v.y << " " << v.z << "\n";
	}

	out << "g default\n";

	for (i = 0; i < mesh->GetFSize(); i++) {
		const XFace& f = mesh->GetF(i);
		out << "f " << f.p(0)+1 << "/" << f.p(0)+1 <<  "/" << f.p(0)+1 << " " <<
			f.p(1)+1 << "/" << f.p(1)+1 <<  "/" << f.p(1)+1 << " " <<
			f.p(2)+1 << "/" << f.p(2)+1 <<  "/" << f.p(2)+1 << "\n";
	}

	return true;
}

int GetFaceNumber(char *s, varray<IntVec4> &fidx)
{
	int						nf = 0, m = 0;
	bool					flag = false;
	char					str[10];
	static varray<int>		vid;

	char*					ends = s + static_cast<int>(strlen(s));
	char*					curs = s + 1; 
	vid.resize(0);

	for(; curs < ends; ++curs)
	{
		if(*curs == '\0' || *curs == '\n') break;

		if(flag==false && (*curs == ' ' || *curs == '\t'))
		{
			flag = true;
			m = 0;
		}
		else if(flag && *curs != ' ' && *curs != '/' && *curs != '\t')
		{
			str[m] = *curs;
			m++;
		}
		else if(flag && (*curs == ' ' || *curs == '\0' || *curs == '\n' || *curs == '/' || *curs == '\t'))
		{
			if(m>0)
			{
				str[m] = '\0';

				m = atoi(str);
				if(std::find(vid.begin(),vid.end(),m) == vid.end())
				{
					vid.push_back(m);
					nf++;
				}
				flag = false;
				--curs;
			}
		}

	}

	if(flag && nf>0 && m > 0)
	{
		str[m] = '\0';
		m = atoi(str);
		if(std::find(vid.begin(),vid.end(),m) == vid.end())
		{
			vid.push_back(atoi(str));
			nf++;
		}
	}
	if(nf < 3)
	{
		return 0; 
	}
	fidx.resize(nf-2);
	for(int i=0; i<nf-2; i++)
	{
		fidx[i].x = vid[0];
		fidx[i].y = vid[i+1];
		fidx[i].z = vid[i+2];
	}

	vid.resize(0);
	return nf-2;
}
//spline operator
//----------------------------------------------------------------------
// Name:		CreateSpline
// Function:    create a spline based on discrete points
// Argument:	contrpt: discrete points array
// Return:		return a spline
// Author:		
// Date:		
// Modified by: XXX
// Update date:	2006/05/05	
//----------------------------------------------------------------------
//Spline CreateSpline(const varray<Vec4> &contrpt)
//{
//	KLine	kl;
//	Spline	spline;
//	Vec4	pt;
//	for(int i=0;i<static_cast<int>(contrpt.size());i++)
//	{
//		pt = contrpt.at(i);
//		kl.Add(pt );
//	}
//	create spline
//	kl.ChangeToSpline(spline);
//
//	spline.ChangeMode(Spline::SPLMODE_SPLINE);
//
//	//if first ctrl point and last point is very close, then set the spline to close
//	if(spline.GetCtrlPointCount() > 1 
//		&& (spline.GetCtrlPoint(0) - spline.GetCtrlPoint(spline.GetCtrlPointCount() - 1 ) ).Magnitude() < 0.1 )//XXX 2003-01-23
//	{
//		spline.DelCtrlPoint(spline.GetCtrlPointCount() - 1 );
//		spline.ChangeMode(Spline::SPLMODE_CLOSED_SPLINE);
//	}
//
//	return spline;
//}
//---------------------------------------------------------------
// Name:		IsTwoLineIntersectIn2d
// Description: two 2D line is intersect or not
// Argument:			ln1StPt, ln1EndPt: the start and end point of line1
//						ln2StPt, ln2EndPt: the start and end point of line2
// Return:		true if intersect
// Author:		
// Date:		
// Modified by:		XXX
// Updated date:	2006/05/05
//---------------------------------------------------------------- 
bool IsTwoLineIntersectIn2d(Vec4 ln1StPt,Vec4 ln1EndPt,Vec4 ln2StPt,Vec4 ln2EndPt)
{
	Vec4 m1 = ln1StPt  - ln2StPt;
	Vec4 m2 = ln1EndPt - ln2StPt;
	Vec4 m3 = ln2EndPt - ln2StPt;
	float z1 = CrossVecX(m1,m3).z;
	float z2 = CrossVecX(m2,m3).z;

	if(z1 * z2 > 0)
	{
		return false;
	}

	m1 = ln2StPt  - ln1StPt;
	m2 = ln2EndPt - ln1StPt;
	m3 = ln1EndPt - ln1StPt;
	z1 = CrossVecX(m1,m3).z;
	z2 = CrossVecX(m2,m3).z;
	if(z1 * z2 > 0)
	{
		return false;
	}

	return true;
}
//---------------------------------------------------------------
// Name:		SimplifyALine
// Description: to delete some points which is nearly on a straight line 
//				or which is very close to another one
// Argument:	canDelFlag : whether the control point can delete (by default the end point of the line can not be deleted)
//				oriPArr  : original control point of the curve
//				resPArr  : the control point to return 
//				thresholdAng : the angle to judge whether to segment line to be a straight line.
//				thresholdLen : if the line segment length lager than the thresholdLen, then push to the resPArr(do not need to judge the angle)
//				expandMeshProcision : just see the document.     :)
// Return:		
// Author:		ljt
// Date:		2:9:2003
// Modified by:		
// Updated date:		
//---------------------------------------------------------------- 
void SimplifyALine(varray<bool>& canDelFlag, const varray<Vec4>& oriPArr, varray<Vec4>& resPArr, float thresholdAng, float thresholdLen, float expandMeshProcision=2.0)
{
	resPArr.clear();
	if(oriPArr.size()<=2)
	{
		resPArr=oriPArr;
		return;
	}	

	int size=static_cast<int>(oriPArr.size()), i,j;
	Vec4 v1, v2, v3, v4;
	float ang1, ang2, tempLen;

	float tolLen=0.;
	for(i=0;i<size-1;i++)
	{
		tolLen+=(oriPArr.at(i)-oriPArr.at(i+1)).Magnitude();
	}

	float segLen=tolLen/2;

	varray<Vec4> midVs;
	bool simpFlag  = false;
	bool startSimp = false;
	Vec4 StartSimpV;

	for(i=0;i<size;i++)
	{
		if(i==0|| i==size-1 || !canDelFlag.at(i))
		{//for the first point and last point and the point that cannot be delete
			if(!canDelFlag.at(i) && resPArr.size()>0)
			{
				if( (resPArr.back()-oriPArr.at(i)).Magnitude() < thresholdLen/5)
				{
					bool flag=false;
					if(i>0)
					{
						for(j=i;j>=0;j--)
						{
							if(canDelFlag.at(j))
							{
								if((oriPArr.at(j)-resPArr.back()).Magnitude()<1.0e-3)
								{									
									if(canDelFlag.at(j))
									{
										flag=true;	
									}									
									break;
								}
							}
						}
					}

					//if(!flag)
					if(flag)
					{
						resPArr.pop_back();
					}
					resPArr.push_back(oriPArr.at(i));
					continue;
				}
			}
			resPArr.push_back(oriPArr.at(i));
			continue;
		}
		v1=oriPArr.at(i);
		v2=resPArr.back();
		v3=oriPArr.at(i+1);

		if((v1-v3).Magnitude()<0.001f)
		{
			continue;
		}

		simpFlag=false;

		ang1=GetAngleOf2Vector(v2-v1,v3-v1);

		if(ang1<thresholdAng)
		{
			tempLen=(v1-v2).Magnitude();
			if(tempLen<thresholdLen  )  //take the point before in resPArr into account when length less than thresholdLen
			{
				if(resPArr.size()>1)
				{
					v4=resPArr.at(resPArr.size()-2);
					ang2=GetAngleOf2Vector(v4-v2,v1-v2);			
					if(ang1<ang2/*&&ang2>thresholdAng*/)
					{						
						if(!startSimp)
						{
							StartSimpV=resPArr.back();
							startSimp=true;
						}	
						resPArr.pop_back();

						if((StartSimpV-v1).Magnitude()>1.5*thresholdLen)
						{
							resPArr.push_back(StartSimpV);
							startSimp=false;
						}

						resPArr.push_back(v1);
						simpFlag=true;				
						midVs.clear();
					}					
				}	
				else
				{
					resPArr.push_back(v1);
					midVs.clear();
					simpFlag=true;
					startSimp=false;
				}
			} 
			else
			{
				resPArr.push_back(v1);
				midVs.clear();
				simpFlag=true;
				startSimp=false;
			}			

			if(simpFlag)
			{
				if(midVs.size()>2 && resPArr.size()>1)
				{
					float maxDis=0;
					int selIdx=-1;
					for(j=0;j<static_cast<int>(midVs.size());j++)
					{
						tempLen=GetDistOfPntToLn(midVs.at(j), resPArr.back(),resPArr.at(resPArr.size()-2));
						if(tempLen>maxDis)
						{
							maxDis=tempLen;
							selIdx=j;
						}
					}
					if(maxDis>thresholdLen/20)
					{
						Vec4 tempV=resPArr.back();
						resPArr.pop_back();
						resPArr.push_back(midVs.at(selIdx));
						resPArr.push_back(tempV);
						startSimp=false;
					}
				}
				midVs.clear();
			}
		}
		else
		{
			midVs.push_back(v1);

			float tempLen=(midVs.back()-resPArr.back()).Magnitude();
			if(tempLen>5*thresholdLen || tempLen>segLen)
			{
				resPArr.push_back(midVs.back());
				midVs.clear();
				startSimp=false;
			}
		}

	}
}

//---------------------------------------------------------------
// Name:		SimplifyLines
// Description: to delete some points which is nearly on a straight line or which is very close to another one
// Argument:	oriPArr  : original control point of the curve
//				resPArr  : the control point to return 
//				tol : approximation tolerance
// Return:		
// Author:		XXX
// Date:		2007-02-07
// Modified by:		
// Updated date:		
//---------------------------------------------------------------- 
void SimplifyLines(const varray<Vec4>& oriPArr, varray<Vec4>& resPArr, float tol)
{
	int i;
	int oriSize = static_cast<int>(oriPArr.size());
	if (oriSize == 2)    //no need simplify
	{
		resPArr.push_back(oriPArr.front());
		resPArr.push_back(oriPArr.back());
		return;
	}

	float  tol2 = tol * tol;       // tolerance squared
	varray<Vec4> PArr;
	varray<bool>  PArrMark;

	// STAGE 1.  Vertex Reduction within tolerance of prior vertex cluster
	PArr.push_back(oriPArr.front());             // start at the beginning
	for (i=1; i<oriSize-1; i++)
	{
		if ((oriPArr.at(i) - PArr.back()).SquareMagnitude() < tol2)
			continue;

		PArr.push_back(oriPArr.at(i));
	}

	if ((oriPArr.back() - PArr.back()).SquareMagnitude() < tol2 && static_cast<int>(PArr.size()) > 1)
	{
		PArr.pop_back();
	}

	PArr.push_back(oriPArr.back());    //finish at the end, end pt should reserve

	// STAGE 2.  Douglas-Peucker polyline simplification
	int curSize = static_cast<int>(PArr.size());
	PArrMark.resize(curSize, false);
	PArrMark.front() = true;
	PArrMark.back() = true;       // mark the first and last vertices
	SimplifyDP(PArr, 0, curSize-1, PArrMark, tol);

	for (i=0; i<static_cast<int>(PArr.size()); ++i)
	{
		if (PArrMark.at(i))
			resPArr.push_back(PArr.at(i));
	}

}
//---------------------------------------------------------------
// Name:		SimplifyDP
// Description: This is the Douglas-Peucker recursive simplification routine
//				It just marks vertices that are part of the simplified polyline
//				for approximating the polyline subchain PArr[stIdx] to PArr[endIdx].
// Argument:	PArr  : polyline array of vertex points 
//				stIdx  : start indice for the subchain  
//				endIdx : end indice for the subchain 
//				PArrMark : array of markers matching vertex array PArr
//				tol : approximation tolerance
// Return:		
// Author:		XXX
// Date:		2007-02-07
// Modified by:		
// Updated date:		
//---------------------------------------------------------------- 
void SimplifyDP(const varray<Vec4>& PArr, int stIdx, int endIdx, varray<bool>& PArrMark, float tol)
{
	if (endIdx <= stIdx+1) // there is nothing to simplify
		return;

	// check for adequate approximation by segment S from PArr[stIdx] to PArr[endIdx]
	int     maxi = stIdx;          // index of vertex farthest from S
	float   maxd2 = 0;         // distance squared of farthest vertex
	float   tol2 = tol * tol;  // tolerance squared
	Vec4	u = PArr.at(endIdx) - PArr.at(stIdx);   // segment direction vector
	float   cu = Dot(u, u);     // segment length squared

	// test each vertex PArr[i] for max distance from S
	Vec4  w;
	Vec4   Pb;                // base of perpendicular from PArr[i] to S
	float  b, cw, dv2;        // dv2 = distance PArr[i] to S squared

	for (int i=stIdx+1; i<endIdx; ++i)
	{
		// compute distance squared
		w = PArr[i] - PArr[stIdx];
		cw = Dot(w, u);
		if ( cw <= 0 )
			dv2 = (PArr[i] - PArr[stIdx]).SquareMagnitude();
		else if ( cu <= cw )
			dv2 = (PArr[i] - PArr[endIdx]).SquareMagnitude();
		else
		{
			b = cw / cu;
			Pb = PArr[stIdx] + b * u;
			dv2 = (PArr[i] - Pb).SquareMagnitude();
		}
		// test with current max distance squared
		if (dv2 >= maxd2) 
		{		
			// PArr[i] is a new max vertex
			maxi = i;
			maxd2 = dv2;
		}
	} 

	if (maxd2 > tol2)        // error is worse than the tolerance
	{
		// split the polyline at the farthest vertex from S
		PArrMark[maxi] = 1;      // mark pArr[maxi] for the simplified polyline
		// recursively simplify the two subpolylines at pArr[maxi]
		SimplifyDP( PArr, stIdx, maxi, PArrMark, tol);  // polyline PArr[stIdx] to PArr[maxi]
		SimplifyDP( PArr, maxi, endIdx, PArrMark, tol);  // polyline PArr[maxi] to PArr[endIdx]
	}

	// else the approximation is OK, so ignore intermediate vertices
	return;
}
//---------------------------------------------------------------
// Name:		EliminateRuffleOfSpline
// Description: eliminate the ruffle of the line; when the length radio from two 
//				adjacent line segments is too high or too low, the spline will has ruffle.
//				we can add more control point from original array to avoid this case,
//				that is to say that the spline will has  more control points 
// Argument:	oriPArr:		the spline's original control points before simplyfying
//				lineParr:		input the control points array before added points 
//								output the control points array after added points
//				threshholeAng:	the largest angle  that two line segment form.
// Return:		void
// Author:		
// Date:		
// Modified by:	XXX	
// Updated date:20060507		
//---------------------------------------------------------------- 
void EliminateRuffleOfSpline(const varray<Vec4>& oriPArr,varray<Vec4>& linePArr,float threshholdAng)
{
	if(oriPArr.size() < 3)
		return;
	if(linePArr.size() < 3)
		return;

	int		i,j;
	int		flag1,flag2;
	float	length1,length2;
	Vec4	v1,v2,v3;
	int		ncount = static_cast<int>(linePArr.size());
	float	localheight;
	varray<float> heightarr;
	int		maxHeightVid;

	flag1 = -1;
	flag2 = -1;
	for(i=0;i<ncount-1;i++)
	{
		v1 = linePArr.at(i);
		v2 = linePArr.at(i+1);
		if(i < ncount-2)
		{
			v3 = linePArr.at(i+2);
		}
		else if(i == ncount-2)
		{
			if ((v2 - linePArr[0]).Magnitude() < 1e-6)
			{ //closed spline
				v3 = linePArr.at(1);
			}
			else
			{
				return;
			}
		} 

		length1 = Vec4(v1-v2).Magnitude();
		length2 = Vec4(v3-v2).Magnitude();

		if(GetAngleOf2Vector(v1-v2,v3-v2) < threshholdAng)
		{
			//return;
			continue;//XXX add 20060507
		}
		if(length1/length2 > 1/5 && length1/length2 < 5)
		{
			//return;
			continue;//XXX add 20060507
		}

		if(length1/length2 <= 1/5)
		{
			for(j=0; j < static_cast<int>(oriPArr.size()); j++)
			{
				if(oriPArr.at(j) == v2)
				{
					flag1=j;
					continue;
				}
				if(oriPArr.at(j) == v3)
				{
					if(j>flag1)
						flag2=j;
					continue;
				}
			}			   
		}
		else if(length1/length2 >= 5)
		{
			for(j=0;j<static_cast<int>(oriPArr.size());j++)
			{
				if(oriPArr.at(j) == v1)
				{
					flag1 = j;
					continue;
				}
				if(oriPArr.at(j) == v2)
				{
					if(j > flag1)
						flag2 = j;
					continue;
				}
			}
		}
		if(flag1 == -1|| flag2 == -1 || flag1>static_cast<int>(oriPArr.size()-1) || flag2>static_cast<int>(oriPArr.size()-1))
		{
			//return;
			continue;
		}
		for(j=flag1+1;j<flag2;j++)
		{
			if(length1/length2 < 1/5)
				heightarr.push_back(GetDistOfPntToLn(oriPArr.at(j),v2,v3));
			else if(length1/length2 > 5)
				heightarr.push_back(GetDistOfPntToLn(oriPArr.at(j),v1,v2));
		}
		if(heightarr.size()<1)
		{
			//return;
			continue;
		}

		localheight=heightarr.at(0);
		maxHeightVid=0;
		for(j=0;j<static_cast<int>(heightarr.size());j++)
		{
			if(heightarr.at(j) > localheight)
			{
				localheight=heightarr.at(j);
				maxHeightVid=j;
			}
		}
		if((maxHeightVid+flag1+1) < static_cast<int>(oriPArr.size()) && (maxHeightVid+flag1+1)>-1)
			v2 = oriPArr.at(maxHeightVid+flag1+1);
		linePArr.at(i+1) = v2;
	}
}
//----------------------------------------------------------------------
// Name:		ConvertSevel2DSplineIntoOne
// Function:    merge some 2D splines into one
// Argument:	oriSpl: origin splines
//				splMode: spline  mode. Not Used
//				resSpl:  merged spline
// Return:		return a spline
// Author:		
// Date:		
// Modified by: XXX
// Update date:	2006/05/05	
//----------------------------------------------------------------------
bool ConvertSevel2DSplineIntoOne(varray<Spline>&oriSpl, int splMode,Spline& resSpl)
{
	assert(&splMode);
	int size = static_cast<int>(oriSpl.size());
	if(size == 0)
		return false;

	int i, j;
	//can not merge closed spline
	for(i=0; i<static_cast<int>(oriSpl.size()); ++i) //Added by Zhou Chuan(2004.12.7)
	{
		if(oriSpl.at(i).GetMode()==Spline::SPLMODE_CLOSED_SPLINE)
			return false;
	}

	float threshold = (float)0.01;
	varray<bool> flagArr;
	varray<Vec4> ctrlPArr;
	flagArr.resize(oriSpl.size(), false);

	Spline& curSpl = oriSpl.at(0);
	flagArr.at(0)  = true;

	Vec4 currP1 = curSpl.GetCtrlPoint(0);
	Vec4 currP2 = curSpl.GetCtrlPoint(curSpl.GetCtrlPointCount()-1);
	Vec4 currP;
	Vec4 nextP1,nextP2;
	bool success  = false;
	bool findFlag = false;

	bool onceFlag=true;
	while(true)
	{
		success=true;
		for(i=0;i<size;i++)
		{
			if(!flagArr.at(i))
			{
				success=false;
				break;
			}
		}
		if(success)
			break;

		if(onceFlag) 
		{// the first time to connect the spline check the two lines head 
			// and tail if they can connect together
			for(i=0;i<size;i++)
			{
				if(flagArr.at(i))
					continue;
				findFlag = false;

				Spline& nextSpl = oriSpl.at(i);
				nextP1 = nextSpl.GetCtrlPoint(0);
				nextP2 = nextSpl.GetCtrlPoint(nextSpl.GetCtrlPointCount()-1);
				if( (currP1-nextP1).Magnitude()<threshold)
				{
					for(j=curSpl.GetCtrlPointCount()-1;j>=0;j--)
					{
						ctrlPArr.push_back(curSpl.GetCtrlPoint(j));
					}
					for(j=1;j<nextSpl.GetCtrlPointCount();j++)
					{
						ctrlPArr.push_back(nextSpl.GetCtrlPoint(j));
					}

					flagArr[i]=true;
					currP=nextP2;
					findFlag=true;
					break;
				} 
				else if((currP1-nextP2).Magnitude()<threshold)
				{
					for(j=curSpl.GetCtrlPointCount()-1;j>=0;j--)
					{
						ctrlPArr.push_back(curSpl.GetCtrlPoint(j));
					}
					for(j=nextSpl.GetCtrlPointCount()-2;j>=0;j--)
					{
						ctrlPArr.push_back(nextSpl.GetCtrlPoint(j));
					}
					flagArr.at(i)=true;
					currP=nextP1;
					findFlag=true;
					break;
				}
				else if((currP2-nextP1).Magnitude()<threshold)
				{
					for(j=0;j<curSpl.GetCtrlPointCount();j++)
					{
						ctrlPArr.push_back(curSpl.GetCtrlPoint(j));
					}
					for(j=1; j<nextSpl.GetCtrlPointCount();j++)
					{
						ctrlPArr.push_back(nextSpl.GetCtrlPoint(j));
					}

					flagArr[i]=true;
					currP=nextP2;
					findFlag=true;
					break;
				}
				else if((currP2-nextP2).Magnitude()<threshold)
				{
					for(j=0;j<curSpl.GetCtrlPointCount();j++)
					{
						ctrlPArr.push_back(curSpl.GetCtrlPoint(j));
					}
					for(j=nextSpl.GetCtrlPointCount()-2;j>=0;j--)
					{
						ctrlPArr.push_back(nextSpl.GetCtrlPoint(j));
					}

					flagArr[i]=true;
					currP=nextP1;
					findFlag=true;
					break;
				}
			}	 
		}
		else           
		{ // to connect the spline with the current temp spline.
			for(i=0;i<size;i++)
			{
				if(flagArr.at(i))
					continue;
				findFlag=false;

				Spline& nextSpl=oriSpl.at(i);
				nextP1=nextSpl.GetCtrlPoint(0);
				nextP2=nextSpl.GetCtrlPoint(nextSpl.GetCtrlPointCount()-1);
				if( (currP-nextP1).Magnitude()<threshold)
				{			
					for(j=1; j<nextSpl.GetCtrlPointCount();j++)
					{
						ctrlPArr.push_back(nextSpl.GetCtrlPoint(j));
					}
					flagArr[i]=true;
					currP=nextP2;
					findFlag=true;
					break;
				}
				else if((currP-nextP2).Magnitude()<threshold)
				{
					for(j=nextSpl.GetCtrlPointCount()-2;j>=0;j--)
					{
						ctrlPArr.push_back(nextSpl.GetCtrlPoint(j));
					}
					flagArr[i]=true;
					currP=nextP1;
					findFlag=true;
					break;
				}
			}

		}
		if(!findFlag)
		{
			return false;
		}
		onceFlag=false;
	}
	if(!success)
		return false;

	resSpl.ChangeMode(1);  // change to mode SPLMODE_SPLINE
	for(i=0;i<static_cast<int>(ctrlPArr.size());i++)
	{
		resSpl.AddCtrlPoint(ctrlPArr.at(i));
	}
	return true;	
}
//---------------------------------------------------------------
// Name:		AdjustSplineControlPoints
// Description: adjust the spline to make the control point more well-proportioned
//				just change the position of control point1 and point n-2;
// Argument:	spline: the spline 
// Return:		
// Author:		
// Date:		
// Modified by:	XXX	
// Updated date:20060507		
//---------------------------------------------------------------- 
void AdjustSplineControlPoints(Spline& spline)
{
	float len1, len2;
	if(spline.GetCtrlPointCount() >= 3)
	{
		len1 = spline.GetLength(1);
		len2 = spline.GetLength(2);
		if(len2/len1 > 3 ||
			(spline.GetCtrlPoint(1)-spline.GetCtrlPoint(0)).Magnitude()*5.0f
			<(spline.GetCtrlPoint(2)-spline.GetCtrlPoint(1)).Magnitude())
		{   //change position of control point 1
			Vec4& pos = spline.GetCtrlPoint(1);
			pos = spline.GetPoint(1, 0.25);
		}

		len1=spline.GetLength(spline.GetCtrlPointCount()-2);
		len2=spline.GetLength(spline.GetCtrlPointCount()-3);
		if(len2/len1 > 3 ||
			(spline.GetCtrlPoint(spline.GetCtrlPointCount()-2)-spline.GetCtrlPoint(spline.GetCtrlPointCount()-1)).Magnitude()*5
			<(spline.GetCtrlPoint(spline.GetCtrlPointCount()-2)-spline.GetCtrlPoint(spline.GetCtrlPointCount()-3)).Magnitude() )
		{   //change position of control point n-2
			Vec4& pos=spline.GetCtrlPoint(spline.GetCtrlPointCount()-2);
			pos=spline.GetPoint(spline.GetCtrlPointCount()-3,0.75);
		}
	}
}
//projection operator 
//----------------------------------------------------------------------
// name:		GetProjectPntOnTriangle
// function:	get the projection of v1 on the triangle with 3 vertex of triV[3]
// argument:	Vec4 v1: point
//				triV[3]: triangle points
// return:		project point of v1
// author:		
// date:		
// update:	    
// author:		XXX
// date:		20060504
//----------------------------------------------------------------------
Vec4 GetProjectPntOnTriangle(Vec4 v1, Vec4 triV[3])
{
	Vec4 norm = CrossVecX(triV[1]-triV[0], triV[2]-triV[0]).Normalize();
	Vec4 v2 = v1-triV[0];
	float len = Dot(v2,norm);
	Vec4 projection = v1-len*norm;
	return projection;
}

//----------------------------------------------------------------------
// name:		GetBarycentCoorInATriangle
// function:	get the bary center coordinate in a triangle 
// argument:	Vec4 v1: point
//				triV[3]: triangle points
// return:		project point of v1
// author:		
// date:		
// update:	    
// author:		XXX
// date:		20060504
//----------------------------------------------------------------------
Vec4 GetBarycentCoorInATriangle(Vec4 P, Vec4 v0, Vec4 v1, Vec4 v2)
{
	float u,v,w;
	Vec4 v10 = v1 - v0;
	Vec4 v20 = v2 - v0;
	//Vec4 v21 = v2 - v1;
	Vec4 vp0 = P - v0;
	Vec4 vp1 = P - v1;
	Vec4 vp2 = P - v2;
	Vec4 refN = CrossVecX(v10,v20); // magnitude = area
	refN.unit();

	u = Dot(CrossVecX((vp1),(vp2)),refN);
	v=Dot(CrossVecX((vp2),(vp0)),refN); 


	w=Dot(CrossVecX((vp0),(vp1)),refN);
	float temp=u+v+w;
	u=u/temp;
	v=v/temp;
	w=w/temp;
	//over

	Vec4 uvw(u,v,w);
	return(uvw);
}
//----------------------------------------------------------------------
// name:		GetThirdPointByAngle
// function:	get the third point when expanding
// argument:	p1:
//				p2:		  deriection	
//				p1p3:	  length
//				angle:    angle
//				p31,p32:  thrid point
// return:		void
// author:		Li
// date:		2003/11/06
// update:	    
// author:		XXX
// date:		20060504
//----------------------------------------------------------------------
void GetThirdPointByAngle( const Vec4 &p1, const Vec4 &p2, float p1p3, float angle, Vec4 &p31, Vec4 &p32)
{
	Vec4 dir= (p2-p1).Normalize();
	dir = p1p3*dir;

	QMatrix4x4 matrix1, matrix2;
	//matrix1.SetRotateZ(angle);
	matrix1.rotate(angle, 0, 0, 1);
	//matrix2.SetRotateZ(-angle);
	matrix2.rotate(-angle, 0, 0, 1);
	QVector3D temp;
	temp.setX(dir.x);
	temp.setY(dir.y);
	temp.setZ(dir.z);

	p31.x = p1.x + (temp * matrix1).x();
	p31.x = p1.y + (temp * matrix1).y();
	p31.x = p1.z + (temp * matrix1).z();

	p32.x = p1.x + (temp * matrix2).x();
	p32.x = p1.y + (temp * matrix2).y();
	p32.x = p1.z + (temp * matrix2).z();
}
//---------------------------------------------------------------
// Name:		IsOnDiffSide
// Description: the two points p1,p4 is at the same side of p2p3 or not
// Argument:	p1,p2,p3 p4: point1, point2,point3, point4 				
// Return:		ture if p1 and p4 are on the different side of p2p3
// Author:		unknown
// Date:		
// Modified by:	XXX
// Updated date:20060504
//---------------------------------------------------------------- 
bool IsOnDiffSide(const Vec4 &p1, const Vec4 &p2, const Vec4 &p3, const Vec4 &p4)
{
	Vec4 m1 = p1-p2;
	Vec4 m2 = p4-p2;
	Vec4 m3 = p3-p2;
	Vec4 cross1 = CrossVecX(m1,m3);
	Vec4 cross2 = CrossVecX(m2,m3);
	if(IsSharpAngleOfTwoVert(cross1,cross2) )
	{
		return false;
	}
	else
	{
		return true;
	}
}
//----------------------------------------------------------------------
// name:		GetPerpendicularDir
// function:	get the perpendicular vector on the x-y plane.
//				i.e. rotate the vector pi/2 and -pi/2 around z axis 
// argument:	Vec4 outputV1, outputV2: out put vector
//				inputV:  input point
// return:		project point of v1
// author:		
// date:		
// update:	    
// author:		XXX
// date:		20060504
//----------------------------------------------------------------------
void GetPerpendicularDir(Vec4 inputV, Vec4& outputV1, Vec4& outputV2) //2003-05-14
{
	if(inputV.Magnitude() < 1.0e-6)
	{
		inputV = Vec4(1.0,0.0,0.0);
	}
	inputV = inputV.Normalize();
	QMatrix4x4 matrix1, matrix2;
	//matrix1.SetRotateZ((float)PI/2);
	matrix1.rotate((float)PI / 2, 0, 0, 1);
	//matrix2.SetRotateZ((float)-PI/2);
	matrix2.rotate((float)-PI / 2, 0, 0, 1);
	QVector3D temp;
	temp.setX(inputV.x);
	temp.setY(inputV.y);
	temp.setZ(inputV.z);
	outputV1.x = (temp*matrix1).x();
	outputV1.y = (temp*matrix1).y();
	outputV1.x = (temp*matrix1).x();

	outputV2.x = (temp*matrix2).x();
	outputV2.y = (temp*matrix2).y();
	outputV2.z = (temp*matrix2).z();
}
/********************************************************************
* FUNCTION NAME:GetThirdPoint(Vec4 p1,Vec4 p2, float b, float a, Vec4 &p31, Vec4 &p32)
* AUTHOR	   :XXX
* DATE		   :2001/1
* MODIFIER     :
* MODIFY DATE  :
* DESCRIPTION  :draw two circles using p1 and p2 as their central point and get the two intersections say p31 p32
if they dont have intersection, the result can not be used.
* PARAMETER    :input: p1:one point 
p2:the other point
a: the distance to p1
b: the distance to p2

:output:none            p31:  one point
p32:  the other point.
* version      :1.0
********************************************************************/
void GetThirdPoint(const Vec4 &p1,const Vec4 &p2, float b, float a, Vec4 &p31, Vec4 &p32)
{
	Vec4	temp = p2-p1;
	float	m = p2.x-p1.x;
	float	n = p2.y-p1.y;
	float	c = b*b + m*m + n*n - a*a;
	float temp1 = 4*m*m*b*b + 4*n*n*b*b - c*c;
	if(temp1 < 0)
	{
		temp1 = 0.0;
	}

	float temp2 = m*sqrt(temp1);
	float temp3 = n*c;
	float temp4 = 2*(n*n+m*m);

	float y1 = (temp3+temp2)/temp4;
	float y2 = (temp3-temp2)/temp4;
	float x1 = (c-2*n*y1)/(2*m);
	float x2 = (c-2*n*y2)/(2*m);

	if(fabs(m)<1.0e-6)
	{
		temp2=n*sqrt(temp1);
		temp3=m*c;
		x1=(temp3+temp2)/temp4;
		x2=(temp3-temp2)/temp4;
		y1=(c-2*m*x1)/(2*n);
		y2=(c-2*m*x2)/(2*n);
	}

	Vec4 v1 = Vec4(x1,y1,0);
	Vec4 v2 = Vec4(x2,y2,0);
	p31 = v1+p1;
	p32 = v2+p1;
}
//----------------------------------------------------------------------
// Name:		GetAngleBetweenVectorAndPlane()
// Function:    get the angle of vectors to a plane
//				angle = acos(dot(v1,v2)/|v1||v2|)
// Argument:	vect: vector
//				pNorm: plane normal vector
// Return:		the angle of a vectors to a plane: [0, pi]
// Author:		
// Date:		
// Modified by: XXX
// Update date:	2006/05/04	
//----------------------------------------------------------------------
float GetAngleBetweenVectorAndPlane(Vec4 vect, Vec4 pNorm) 
{
	vect  = vect.Normalize();
	pNorm = pNorm.Normalize();
	float dot = Dot(vect, pNorm);
	float ang;
	if(fabs(dot-1.0) < 1.0e-4)
	{
		if(dot >= 0.0)
		{
			ang = 0.0;
		}
		else
		{
			ang = (float)PI;
		}
	}
	else if(fabs(dot) < 1.0e-4)
	{
		ang = (float)(PI*0.5);
	}
	else
		ang = acos(dot);

	ang = (float)fabs(ang - PI/2);
	return ang;
}
//---------------------------------------------------------------
// Name:	    Angc()
// Description: Get the angle between two line segments
// Argument:
//         :	
// Return:		
// Author:	    XXX XXX
// Date:	    2006/04/29 29:4:2006   16:32
// Update:	 
// Author:	
// Date: 
// copyright: XXX. developed by XXX
//----------------------------------------------------------------
float Angc(float xr,float yr,float x,float y)
{
	float dx,dy,dxy;
	dx	=	x-xr;
	dy	=	y-yr;
	dxy	=	(float)sqrt(dx*dx+dy*dy);

	if(dxy<1.0e-5)
	{
		return 0.0;
	}

	float fRet = 0;
	if(dy>=0&&dx>0)     
	{
		fRet = ((float)asin(fabs(dy/dxy)));	
	}
	else if(dy>0&&dx<=0)
	{
		fRet = ((float)(PI-asin(fabs(dy/dxy))));
	}
	else if(dy<=0&&dx<0)
	{
		fRet = ((float)(PI+asin(fabs(dy/dxy))));
	}
	else if(dy<0&&dx>=0)
	{
		fRet = ((float)(2*PI-asin(fabs(dy/dxy))));
	}
	return fRet;
}
//-----------------------------------------------------------------
// Name:	    ComputeGlbAngNew()
// Description: Compute the angle between the vector and the x,y,z axis 
// Argument:    vect:-- vector; 
//         :	xrot,yrot,zrot:--angle between the vector and the x,y,z axis 
// Return:		
// Author:       		
// Date:	 	 
// Modified by:	 
// Updated date: 
// copyright: XXX. developed by XXX
//------------------------------------------------------------------
int ComputeGlbAngNew(Vec4 vect,float *xrot, float *yrot,float *zrot)
{
	float x,y,z;
	//	float tmp;

	x=vect.x;
	y=vect.y;
	z=vect.z;

	// rotate x-axis
	if(fabs(z) < ERR && fabs(y)<ERR) 
	{
		*xrot = 0;
	}
	else if(fabs(z) < ERR && y > 0) 
	{
		*xrot = (float)(PI/2.0);
	}
	else if(fabs(z) < ERR && y < 0) 
	{
		*xrot = (float)(-PI/2.0);
	}
	else
	{
		if(fabs(z)<ERR)
		{
			return 0;
		}

		*xrot=(float)atan(y/z);
	}

	if(z<0 && fabs(z) >= ERR)
	{
		if(y>0) 
		{
			*xrot=(float)(*xrot+PI);
		}
		else   
		{
			*xrot=(float)(*xrot-PI);
		}
	}

	//rotate y-axis
	if(fabs(z) < ERR && fabs(x)<ERR) 
	{
		*yrot = 0;
	}
	else if(fabs(z) < ERR && x > 0) 
	{
		*yrot = (float)(PI/2.0);
	}
	else if(fabs(z) < ERR && x < 0) 
	{
		*yrot = (float)(-PI/2.0);
	}
	else
	{
		if(fabs(z)<ERR)
		{
			return 0;
		}

		*yrot=(float)atan(x/z);
	}

	if(z < 0 && fabs(z) >= ERR)
	{
		if(x > 0) 
		{
			*yrot=(float)(*yrot+PI);
		}
		else   
		{
			*yrot=(float)(*yrot-PI);
		}
	}

	// rotate z-axis
	if(fabs(x) < ERR && fabs(y)<ERR) 
	{
		*zrot = 0;
	}
	else if(fabs(x) < ERR && y > 0) 
	{
		*zrot = (float)(PI/2.0);
	}
	else if(fabs(x) < ERR && y < 0) 
	{
		*zrot = (float)(-PI/2.0);
	}
	else
	{
		if(fabs(x)<ERR)
		{
			return 0;
		}

		*zrot=(float)atan(y/x);
	}

	if(x<0 && fabs(x) >= ERR)
	{
		if(y>0) 
		{
			*zrot=(float)(*zrot+PI);
		}
		else   
		{
			*zrot=(float)(*zrot-PI);
		}
	}
	return 1;
}
//-----------------------------------------------------------------
// Name:	    side_line()
// Description: Judge a point(x,y) is in which side of a line(x1,y1,x2,y2),
//              if on_right return(ON_RIGHT),if on_left return(ON_LEFT)
// Argument:    x,y -- point coordinate; 
//         :	x1,y1,x2,y2:-- line endpoints
// Return:		
// Author:       		
// Date:	 	 
// Modified by:	 
// Updated date: 
// copyright: XXX. developed by XXX
//------------------------------------------------------------------
int side_line(float x,float y,float x1,float y1,float x2,float y2)
{
	float f;
	int side=0;  // 1-left, 0-on, -1-right

	f=(x2-x1)*(y-y1)-(y2-y1)*(x-x1);

	if(fabs(f)<=ERR) 
	{
		side=0;
	}
	else if(f<0)   
	{
		side=-1;
	}
	else if(f>0)  
	{
		side=1;
	}
	return side;
}


//---------------------------------------------------------------
// Name:	     MakeConvex()
// Description:  Make the point list convex
// Argument:     plist:--points list inputed; pnum:-- number of points list inputed
//         :	 pnorm:--the plane that points list inputed on
// Return:		
// Author:	    XXX XXX
// Date:	    2006/04/29 29:4:2006   16:42
// Update:	 
// Author:	
// Date: 
// copyright: XXX. developed by XXX
//----------------------------------------------------------------
void  MakeConvex(Vec4 *plist, int &pnum, Vec4 pnorm)
{
	int i; 
	Vec4 *pp = new Vec4[pnum];
	bool *fg = new bool[pnum];
	Vec4 vtRef = Vec4(0, 0, 1);

	for(i=0; i<pnum; i++)
	{
		pp[i] = plist[i]-plist[0];
	}

	// rotate the curve to X-Y plane
	pnorm.Normalize();
	Vec4 vct = pnorm.multiple(vtRef);

	float ang = (float)acos(pnorm.dotmultiple(vtRef));

	for(i=0; i<pnum; i++)
	{
		pp[i] = pp[i].RotAxis(vct, ang);
	}

	float maxx, minx, maxy, miny, nmin, nmax, ntop;

	float x1, x2, y1, y2;

	maxx = minx = pp[0].x;
	maxy = miny = pp[0].y;
	nmin = nmax = ntop = 0;

	x1 = y1 = 1.0e+6;
	x2 = y2 = -1.0e+6;

	for(i=0; i<pnum; i++)
	{
		if(pp[i].x<x1)
		{
			x1 = pp[i].x;
			y1 = pp[i].y;
		}
		else if(pp[i].x>x2)
		{
			x2 = pp[i].x;
			y2 = pp[i].y;
		}
	}

	// seperate the loop into two parts: upper part and lower part
	Vec4 *UP = new Vec4[pnum];
	int *uf = new int[pnum];
	Vec4 *LP = new Vec4[pnum];
	int *lf = new int[pnum];


	int num1, num2;
	num1=num2=0;
	int ncur, npre;

	for(i=0;i<pnum;i++)
	{
		if(((x2 - x1)*(pp[i].y - y1)-(y2 - y1)*(pp[i].x - x1))>0)
		{
			UP[num1]=pp[i];
			uf[num1] = i;
			num1++;
		}
		else
		{
			LP[num2]=pp[i];
			lf[num2] = i;
			num2++;
		}

	}

	// sort upper part
	Vec4 tmp;
	int j, tmpf;
	for (i=0; i<num1; i++)
	{
		for (j=i+1; j<num1; j++)
		{
			if (UP[i].x > UP[j].x)
			{
				tmp = UP[i];
				UP[i] = UP[j];
				UP[j] = tmp;

				tmpf = uf[i];
				uf[i] = uf[j];
				uf[j] = tmpf;
			}
		}
	}

	// make upper part convex
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
						{
							break;
						}
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

	int tmpnum=0;
	for(i=0; i<num1; i++)
	{
		if(!fg[i])
		{
			UP[tmpnum] = plist[uf[i]];
			tmpnum++;
		}
	}

	num1 = tmpnum;

	//sort lower part

	for (i=0; i<num2; i++)
	{
		for (j=i+1; j<num2; j++)
		{
			if (LP[i].x < LP[j].x)
			{
				tmp = LP[i];
				LP[i] = LP[j];
				LP[j] = tmp;

				tmpf = lf[i];
				lf[i] = lf[j];
				lf[j] = tmpf;

			}
		}
	}

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
						{
							break;
						}
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

	tmpnum=0;
	for(i=0; i<num2; i++)
	{
		if(!fg[i])
		{
			LP[tmpnum] = plist[lf[i]];
			tmpnum++;
		}
	}

	num2 = tmpnum;

	for(i=0; i<num1-1; i++)
	{
		plist[i] = UP[i]; 
	}
	for(i=0; i<num2; i++)
	{
		plist[i+num1-1] = LP[i];
	}

	pnum = num1+num2-1;


	delete [] pp;
	delete [] fg;
	delete [] UP;
	delete [] uf;
	delete [] LP;
	delete [] lf;

}
//---------------------------------------------------------------
// Name:	    MakePolylineByAngle()
// Description: remake a polyline to the one which consists of several line segments with equal angle
//			  : the inputed line should be arranged in order
// Argument:	lines: original polyline;			pline: new polyline
//         :	npt:-- original polyline number		nseg:  number of segment
//         :	iPlane:--  0,XY plane; 1,XZ plane, 2,YZ plane;   
// Return:		
// Author:	    XXX XXX
// Date:	  	26/10/2006   15:03
// Update:	 
// Author:	
// Date: 
// copyright: XXX. developed by XXX
//----------------------------------------------------------------
void  MakePolylineByAngle(Vec4* lines, int npt ,int iPlane,Vec4 vtCen, Vec4* pline, int nseg)
{
	int i = 0, j = 0;
	varray<Vec2> lines2D;
	Vec2 vtCen2D;
	lines2D.resize(npt);
	if(iPlane == 0)
	{
		for(i = 0; i < npt; i++)
		{
			lines2D[i] = Vec2(lines[i].x,lines[i].y);
		}
		vtCen2D = Vec2(vtCen.x,vtCen.y);
	}
	else if(iPlane == 1)
	{
		for(i = 0; i < npt; i++)
		{
			lines2D[i] = Vec2(lines[i].x,lines[i].z);
		}
		vtCen2D = Vec2(vtCen.x,vtCen.z);
	}
	else if(iPlane == 2)
	{
		for(i = 0; i < npt; i++)
		{
			lines2D[i] = Vec2(lines[i].y,lines[i].z);
		}
		vtCen2D = Vec2(vtCen.y,vtCen.z);
	}
	else
	{
		assert(iPlane >= 0 && iPlane <= 2); 
		return;
	}
	Vec4 vt0, vt1,vt2,vtDir,vtCen3D;
	Vec4 p1,p2,p3,p4;
	vtCen3D = Vec4(vtCen2D.x,vtCen2D.y,0);
	vt0		= Vec4(lines2D[0].x,lines2D[0].y,0) - vtCen3D;
	vt1		= Vec4(lines2D[0].x,lines2D[0].y,0) -  vtCen3D;
	vt2		= Vec4(lines2D[npt-1].x,lines2D[npt-1].y,0) - vtCen3D;

	float fAngSum = GetAngleOf2Vector(vt1,vt2);
	float fAngSeg = fAngSum/nseg;
	float fScl,fAng,fAng1,fAng2;
	varray<bool> vFlag;
	for(i = 0; i <= nseg; i++)
	{
		vFlag.push_back(false);
	}

	for(i = 1; i < nseg; i++)
	{
		fAng =  i * fAngSeg;
		QMatrix4x4 m1;
		//m1.SetRotateZ(fAng);
		m1.rotate(fAng, 0, 0, 1);
		vtDir = vt0;
		QVector3D temp;
		temp.setX(vtDir.x);
		temp.setY(vtDir.y);
		temp.setZ(vtDir.z);

		vtDir.x = (temp * m1).x();
		vtDir.y = (temp * m1).y();
		vtDir.z = (temp * m1).z();

		p1 = vtCen3D;
		p2 = vtCen3D + vtDir.Normalize() * 500;
		for(j = 0; j < npt-1; j++)
		{
			vt1 = Vec4(lines2D[j].x,lines2D[j].y,0) - vtCen3D;
			vt2 = Vec4(lines2D[j+1].x,lines2D[j+1].y,0) - vtCen3D;
			fAng1 = (j == 0) ? 0 : GetAngleOf2Vector(vt1,vt0);
			fAng2 =  GetAngleOf2Vector(vt2,vt0);

			if(fAng1 <= fAng && fAng2 >= fAng)
			{
				p3 = Vec4(lines2D[j].x,lines2D[j].y,0);
				p4 = Vec4(lines2D[j+1].x,lines2D[j+1].y,0);

				fScl = fabs((fAng - fAng1)/(fAng2 - fAng1));

				if(IsTwoLineIntersectIn2d(p1,p2, p3, p4))	
				{
					Vec4 res = GetTwoLineInterPt(p1, p2, p3, p4);
					Vec2 res2D = Vec2(res.x,res.y);
					fScl = (res2D - lines2D[j]).Magnitude()/(lines2D[j+1] - lines2D[j]).Magnitude();
				}
				pline[i] = lines[j] * (1 - fScl) + lines[j+1] * fScl;
				vFlag[i] = true;
			}
		}
	}
	pline[0]	= lines[0];
	pline[nseg] = lines[npt-1];

	bool bFlag = true;
	for(i = 1; i < nseg; i++)
	{
		bFlag = bFlag & vFlag[i];
	}
	if(!bFlag)
	{
		assert(bFlag);
		MakePolyline(lines,npt,pline,nseg);
	}
}

void  MakePolylineByAngle(const varray<Vec4>& lines,int iPlane,Vec4 vtCen,varray<Vec4>& pline, int nseg)
{
	int npt = lines.size();
	Vec4 linesTemp[1000],plineTemp[1000];
	if(npt > 1000 || nseg > 999)
	{
		assert(npt <= 1000 && nseg <= 999);
		return;
	}
	int i = 0;
	for(i = 0; i < lines.size(); i++)
	{
		linesTemp[i] = lines[i];
	}
	MakePolylineByAngle(linesTemp,npt,iPlane,vtCen,plineTemp,nseg);
	pline.clear();
	for(i = 0; i <= nseg; i++)
	{
		pline.push_back(plineTemp[i]);
	}
}

//-----------------------------------------------------------------
// Name:	    MakePolyline
// Description: remake a polyline to the one which consists of several line segments with equal length
// Argument:    lines: original polyline
//         :	pline: new polyline
//              nseg:  number of segment
// Return:		
// Author:       		
// Date:	 	 
// Modified by:	 XXX XXX
// Updated date: 2006/04/29 29:4:2006
// copyright: XXX. developed by XXX
//------------------------------------------------------------------
void MakePolyline(Vec4 *lines, int npt, varray<Vec4>& pline, int nseg)
{
	int i, k;
	int np;
	float length, slen;
	float *len;

	for(i=0; i<nseg+1; i++)
	{
		pline[i] = Vec4(0, 0, 0);
	}
	len = new float[npt-1];

	length = 0;
	for(i=1; i<npt; i++)
	{
		len[i-1] = (lines[i]-lines[i-1]).GetLength();
		length += len[i-1];
	}

	slen = length/nseg;

	pline[0] = lines[0];
	pline[nseg] = lines[npt-1];

	np = 0;
	length = 0;
	k = 1;
	for(i=0; i<npt-1; i++)
	{
		length += len[i];
		if(len[i]<ERR) 
		{
			continue;
		}
		if(length>=slen)
		{
			do
			{
				length = length - slen;
				pline[k] = lines[i+1] + (lines[i]-lines[i+1])*length/len[i];
				k++;
			}while(length>=slen);
		}
	}

	if(k==nseg)
	{
		pline[k] = lines[npt-1];
	}
	delete [] len;
}


//-----------------------------------------------------------------
// Name:	    MakePolyline
// Description: remake a polyline to the one which consists of several line segments with equal length
// Argument:    lines: original polyline
//         :	pline: new polyline
//              nseg:  number of segment
// Return:		
// Author:       		
// Date:	 	 
// Modified by:	 XXX XXX
// Updated date: 2006/04/29 29:4:2006
// copyright: XXX. developed by XXX
//------------------------------------------------------------------
void MakePolyline(const varray<Vec4>&lines, int npt, varray<Vec4>& pline, int nseg)
{
	int i, k;
	int np;
	float length, slen;
	float *len;
	pline.resize(nseg + 1);

	for(i=0; i<nseg+1; i++)
	{
		pline[i] = Vec4(0, 0, 0);
	}
	len = new float[npt-1];

	length = 0;
	for(i=1; i<npt; i++)
	{
		len[i-1] = (lines[i]-lines[i-1]).GetLength();
		length += len[i-1];
	}

	slen = length/nseg;

	pline[0] = lines[0];
	pline[nseg] = lines[npt-1];

	np = 0;
	length = 0;
	k = 1;
	for(i=0; i<npt-1; i++)
	{
		//		float x=lines[i].x;
		//		float y=lines[i].y;
		//		float z=lines[i].z;

		length += len[i];
		if(len[i]<ERR) 
		{
			continue;
		}
		if(length>=slen)
		{
			do
			{
				length = length - slen;
				pline[k] = lines[i+1] + (lines[i]-lines[i+1])*length/len[i];
				k++;
			}while(length>=slen);
		}
	}

	if(k==nseg)
	{
		pline[k] = lines[npt-1];
	}
	delete [] len;
}

//-----------------------------------------------------------------
// Name:	    MakePolyline
// Description: remake a polyline to the one which consists of several line segments with equal length
// Argument:    lines: original polyline
//         :	pline: new polyline
//              nseg:  number of segment
// Return:		
// Author:       		
// Date:	 	 
// Modified by:	 XXX XXX
// Updated date: 2006/04/29 29:4:2006
// copyright: XXX. developed by XXX
//------------------------------------------------------------------
void MakePolyline(Vec4 *lines, int npt, Vec4 *pline, int nseg)
{
	int i, k;
	int np;
	float length, slen;
	float *len;

	for(i=0; i<nseg+1; i++)
	{
		pline[i] = Vec4(0, 0, 0);
	}
	len = new float[npt-1];

	length = 0;
	for(i=1; i<npt; i++)
	{
		len[i-1] = (lines[i]-lines[i-1]).GetLength();
		length += len[i-1];
	}

	slen = length/nseg;

	pline[0] = lines[0];
	pline[nseg] = lines[npt-1];

	np = 0;
	length = 0;
	k = 1;
	for(i=0; i<npt-1; i++)
	{
		//		float x=lines[i].x;
		//		float y=lines[i].y;
		//		float z=lines[i].z;

		length += len[i];
		if(len[i]<ERR) 
		{
			continue;
		}
		if(length>=slen)
		{
			do
			{
				length = length - slen;
				pline[k] = lines[i+1] + (lines[i]-lines[i+1])*length/len[i];
				k++;
			}while(length>=slen);
		}
	}

	if(k==nseg)
	{
		pline[k] = lines[npt-1];
	}
	delete [] len;
}
//-----------------------------------------------------------------
// Name:	    DivideLoopByAngle(Vec4 *plist, int npt, Vec4 norm, Vec4 *newp, int nump, float sang)
// Description: Divide a loop with equal angle to create a new loop
// Argument:    plist: point list in the original loop
//         :	npt: number of points in the original loop
//              norm: plane normal of the loop
//              newp: point list in the new loop
//              nump: number of points in the new loop
//              sang: starting angle for the first point in the new loop
// Return:		
// Author:       		
// Date:	 Mar. 28, 2003	 
// Modified by:	 XXX XXX
// Updated date: 2006/04/29 29:4:2006
// copyright: XXX. developed by XXX
//------------------------------------------------------------------
void DivideLoopByAngle(Vec4 *plist, int &npt, Vec4 norm, int nump, float sang)
{
	int i, j, k;
	Vec4 cen;
	float *ang;
	float angtmp, dang;
	Vec4 p1, p2;
	float d1, d2;

	cen = plist[0];
	for(i=1; i<npt; i++)
	{
		cen += plist[i];
	}

	cen = cen/(float)npt;

	ang = new float[npt];

	if( (norm.y-1.0) <= ERR )
	{
		for(i=0; i<npt; i++)
		{
			ang[i] = Angc(0, 0, plist[i].x - cen.x, plist[i].z-cen.z);
		}

		// order the point
		Vec4 ptmp;
		for(i=0; i<npt-1; i++)
		{
			for(j=i+1; j<npt; j++)
			{
				if(ang[i]>ang[j])
				{
					angtmp = ang[i];
					ang[i] = ang[j];
					ang[j] = angtmp;

					ptmp = plist[i];
					plist[i] = plist[j];
					plist[j] = ptmp;
				}
			}
		}
	}

	Vec4 *newp;
	newp = new Vec4 [nump];

	dang = 2.0f*(float)PI/(float)nump;
	bool sign;

	for(i=0; i<nump; i++)
	{
		angtmp = sang + i*dang;
		if(angtmp > PI*2.0f)
		{
			angtmp -= (float)PI*2.0f;
		}

		sign =false;
		for(k=1; k<npt; k++)
		{
			if(fabs(ang[k-1]-ang[k])>ERR)
			{
				if((angtmp >= ang[k-1] && angtmp <= ang[k]) || (angtmp <= ang[k-1] && angtmp >= ang[k]))
				{
					p1 = plist[k-1];
					p2 = plist[k];

					if( !(fabs(p1.x-p2.x)<=ERR && fabs(p1.y-p2.y)<=ERR && fabs(p1.z-p2.z)<=ERR))
					{
						sign = true;
						break;
					}
				}
			}
		}

		if(!sign)
		{
			p1 = plist[0];

			for(k=npt-1; k>0; k--)
			{
				if(ang[k]>PI)
				{
					p2 = plist[k];
					break;
				}
			}
		}

		// calculate the intersecting point
		Vec4 p1tmp = (p1 - cen).Normalize();
		Vec4 p2tmp = (p2 - cen).Normalize();
		Vec4 centmp = Vec4(0,0,0);
		d1 = GetDistOfPntToLn( p1tmp, centmp, Vec4((float)cos(angtmp), 0, (float)sin(angtmp)) );
		d2 = GetDistOfPntToLn( p2tmp, centmp, Vec4((float)cos(angtmp), 0, (float)sin(angtmp)) );

		newp[i] = p1 + (p2-p1)*d1/(d1+d2);
	}

	npt = nump+1;
	for(i=0; i<nump; i++)
	{
		plist[i] = newp[i];
	}
	plist[npt-1] = plist[0];

	delete [] ang;
	delete [] newp;	

	return;
}


//-----------------------------------------------------------------
// Name:	    DivideLoopByAngle(Vec4 *plist, int npt, Vec4 norm, Vec4 *newp, int nump, float sang)
// Description: Divide a loop with equal angle to create a new loop
// Argument:    plist: point list in the original loop
//         :	npt: number of points in the original loop
//              norm: plane normal of the loop
//              newp: point list in the new loop
//              nump: number of points in the new loop
//              sang: starting angle for the first point in the new loop
// Return:		
// Author:       		
// Date:	 Mar. 28, 2003	 
// Modified by:	 XXX XXX
// Updated date: 2006/04/29 29:4:2006
// copyright: XXX. developed by XXX
//------------------------------------------------------------------
void DivideLoopByAngle(varray<Vec4>& plist, int &npt, Vec4 norm, int nump, float sang)
{
	int i, j, k;
	Vec4 cen;
	float *ang;
	float angtmp, dang;
	Vec4 p1, p2;
	float d1, d2;

	cen = plist[0];
	for(i=1; i<npt; i++)
	{
		cen += plist[i];
	}

	cen = cen/(float)npt;

	ang = new float[npt];

	if( (norm.y-1.0) <= ERR )
	{
		for(i=0; i<npt; i++)
		{
			ang[i] = Angc(0, 0, plist[i].x - cen.x, plist[i].z-cen.z);
		}

		// order the point
		Vec4 ptmp;
		for(i=0; i<npt-1; i++)
		{
			for(j=i+1; j<npt; j++)
			{
				if(ang[i]>ang[j])
				{
					angtmp = ang[i];
					ang[i] = ang[j];
					ang[j] = angtmp;

					ptmp = plist[i];
					plist[i] = plist[j];
					plist[j] = ptmp;
				}
			}
		}
	}

	Vec4 *newp;
	newp = new Vec4 [nump];

	dang = 2.0f*(float)PI/(float)nump;
	bool sign;

	for(i=0; i<nump; i++)
	{
		angtmp = sang + i*dang;
		if(angtmp > PI*2.0f)
		{
			angtmp -= (float)PI*2.0f;
		}

		sign =false;
		for(k=1; k<npt; k++)
		{
			if(fabs(ang[k-1]-ang[k])>ERR)
			{
				if((angtmp >= ang[k-1] && angtmp <= ang[k]) || (angtmp <= ang[k-1] && angtmp >= ang[k]))
				{
					p1 = plist[k-1];
					p2 = plist[k];

					if( !(fabs(p1.x-p2.x)<=ERR && fabs(p1.y-p2.y)<=ERR && fabs(p1.z-p2.z)<=ERR))
					{
						sign = true;
						break;
					}
				}
			}
		}

		if(!sign)
		{
			p1 = plist[0];

			for(k=npt-1; k>0; k--)
			{
				if(ang[k]>PI)
				{
					p2 = plist[k];
					break;
				}
			}
		}

		// calculate the intersecting point
		Vec4 p1tmp = (p1 - cen).Normalize();
		Vec4 p2tmp = (p2 - cen).Normalize();
		Vec4 centmp = Vec4(0,0,0);
		d1 = GetDistOfPntToLn( p1tmp, centmp, Vec4((float)cos(angtmp), 0, (float)sin(angtmp)) );
		d2 = GetDistOfPntToLn( p2tmp, centmp, Vec4((float)cos(angtmp), 0, (float)sin(angtmp)) );

		newp[i] = p1 + (p2-p1)*d1/(d1+d2);
	}

	npt = nump+1;
	for(i=0; i<nump; i++)
	{
		plist[i] = newp[i];
	}
	plist[npt-1] = plist[0];

	delete [] ang;
	delete [] newp;	

	return;
}

//---------------------------------------------------------------
// Name:	    GetPtByLenSclInPtsArr()
// Description: Get a point in the given points array by the specified length scale 
// Argument:    ptOut:-- point to be gotten; ptsArr:-- points array inputed
//		   :	idx:-- the index in the points array that the point interpolated by the length
//         :	fScl:-- len1/lensum; here, len1 is the length between the start point and a arbitrary point in the points array
//		   :							   lensum is the whole length
//         :    bIsClosed:-- whether the points array is closed
// Return:		bool:-- if the point is calculated successfully
// Author:		XXX XXX
// Date:		 
// Modified by:	XXX XXX
// Updated date: 2005/09/20 20:9:2005   11:36	
//----------------------------------------------------------------
bool GetPtByLenSclInPtsArr(Vec4& ptOut,int& idx,const varray <Vec4>& ptsArr,float fScl, bool bIsClosed)
{
	idx = -1;
	if(ptsArr.size() == 0)
	{
		return false;
	}
	else if(ptsArr.size() == 1)
	{
		idx = 0;
		ptOut = ptsArr[0];
		return true;
	}

	int i = 0;
	float fLenSum = 0, fLen = 0,fLenTmp = 0; 
	bool bIsRealClosed = false;

	if( (ptsArr[0] - ptsArr.back()).GetLength() < 0.005)
	{
		bIsRealClosed = true;
	}

	for(i = 0; i < static_cast<int>(ptsArr.size()-1); i++)
	{
		fLenSum += (ptsArr[i+1] - ptsArr[i]).GetLength();
	}
	if(bIsClosed && !bIsRealClosed)
	{
		fLenSum += (ptsArr[0] - ptsArr.back()).GetLength();
	}
	fLen = fLenSum * fScl;

	bool bIsPtFind = false;
	for(i = 0; i < static_cast<int>(ptsArr.size()-1); i++)
	{
		fLenTmp +=  (ptsArr[i+1] - ptsArr[i]).GetLength();
		if(fLenTmp > fLen)
		{
			float scl = (fLenTmp-fLen)/(ptsArr[i+1] - ptsArr[i]).GetLength();
			ptOut = ptsArr[i] * scl + ptsArr[i+1] * (1-scl);
			bIsPtFind = true;
			idx = i;
			break;
		}
	}
	if(!bIsPtFind)
	{
		if(bIsClosed && !bIsRealClosed)
		{
			fLenTmp += (ptsArr[0] - ptsArr.back()).GetLength();
			if(fLenTmp > fLen)
			{
				float scl = (fLenTmp-fLen)/(ptsArr[i+1] - ptsArr[i]).GetLength();
				ptOut = ptsArr[i] * scl + ptsArr[i+1] * (1-scl);
				bIsPtFind = true;
				idx = ptsArr.size()-1;
			}
		}
	}
	return bIsPtFind;
}
//-----------------------------------------------------------------
// Name:		RefineSlice1()   
// Description: refine a slice to avoid wrinkle
// Argument:    plist: input points.
//         :	nump: segment number to be refine.
// Return:		
// Author:       		
// Date:	 	 
// Modified by:	 XXX XXX
// Updated date: 2006/04/29 29:4:2006   17:18
// copyright: XXX. developed by XXX
//------------------------------------------------------------------
void RefineSlice1(varray<Vec4>& plist, int &nump)
{
	int i, k, nv;
	Vec4 p1, pt, p2, v1, v2;

	bool *flag = new bool [nump];

	float val = (float)cos(PI/2.0);

	// delete the points causing wrinkle
	for(k=0; k<2; k++)
	{
		for(i=0; i<nump; i++)
		{
			flag[i] = false;
		}

		p1 = plist[0];
		for(i=1; i<nump-1; i++)
		{
			pt = plist[i];
			p2 = plist[i+1];

			v1 = p1-pt;
			v2 = p2-pt;
			if(v1.GetLength()<=ERR || v2.GetLength()<=ERR)
			{
				flag[i] = true;
				continue;
			}

			v1.unit();
			v2.unit();

			if(v1.dotmultiple(v2)>val)
			{
				if(!flag[i-1])
				{
					flag[i] = true;
				}
			}

			if(!flag[i])
			{
				p1 = pt;
			}
		}

		nv=0;
		for(i=0; i<nump; i++)
		{
			if(!flag[i])
			{
				plist[nv++] = plist[i];
			}
		}

		nump = nv;
	}

	// smoothing
	val = (float)cos(PI/2.0f);

	for(k=0; k<2; k++)
	{
		for(i=1; i<nump-1; i++)
		{
			p1 = plist[i-1];
			pt = plist[i];
			p2 = plist[i+1];

			v1 = p1-pt;
			v2 = p2-pt;

			v1.unit();
			v2.unit();

			if(v1.dotmultiple(v2)>val)
			{
				plist[i] = (pt + (p1 + p2)*0.5f)/2.0f;
			}
		}
	}

	delete [] flag;
}



//-----------------------------------------------------------------
// Name:		RefineSlice1()   
// Description: refine a slice to avoid wrinkle
// Argument:    plist: input points.
//         :	nump: segment number to be refine.
// Return:		
// Author:       		
// Date:	 	 
// Modified by:	 XXX XXX
// Updated date: 2006/04/29 29:4:2006   17:18
// copyright: XXX. developed by XXX
//------------------------------------------------------------------
void RefineSlice1(Vec4* plist, int &nump)
{
	int i, k, nv;
	Vec4 p1, pt, p2, v1, v2;

	bool *flag = new bool [nump];

	float val = (float)cos(PI/2.0);

	// delete the points causing wrinkle
	for(k=0; k<2; k++)
	{
		for(i=0; i<nump; i++)
		{
			flag[i] = false;
		}

		p1 = plist[0];
		for(i=1; i<nump-1; i++)
		{
			pt = plist[i];
			p2 = plist[i+1];

			v1 = p1-pt;
			v2 = p2-pt;
			if(v1.GetLength()<=ERR || v2.GetLength()<=ERR)
			{
				flag[i] = true;
				continue;
			}

			v1.unit();
			v2.unit();

			if(v1.dotmultiple(v2)>val)
			{
				if(!flag[i-1])
				{
					flag[i] = true;
				}
			}

			if(!flag[i])
			{
				p1 = pt;
			}
		}

		nv=0;
		for(i=0; i<nump; i++)
		{
			if(!flag[i])
			{
				plist[nv++] = plist[i];
			}
		}

		nump = nv;
	}

	// smoothing
	val = (float)cos(PI/2.0f);

	for(k=0; k<2; k++)
	{
		for(i=1; i<nump-1; i++)
		{
			p1 = plist[i-1];
			pt = plist[i];
			p2 = plist[i+1];

			v1 = p1-pt;
			v2 = p2-pt;

			v1.unit();
			v2.unit();

			if(v1.dotmultiple(v2)>val)
			{
				plist[i] = (pt + (p1 + p2)*0.5f)/2.0f;
			}
		}
	}

	delete [] flag;
}

/********************************************************************
* FUNCTION NAME :MakePolyline
* AUTHOR		:XXX XXX	
* DATE			:2005-03-26
* MODIFIER		:
* MODIFY DATE	:
* DESCRIPTION	:Make the polyline 
* PARAMETER     :
*		Input 	    :lines:-- points inputed to disperse; nseg:-- line segment number to disperse
*		Return		:
*		Output:	    :pline:-- points dispersed with the segment number is give 
* Version		:1.0
********************************************************************/
void MakePolyline(const varray<Vec4>& lines,varray<Vec4>&pline, int nseg)
{
	int i, k;
	int np;
	int npt = static_cast<int>(lines.size());
	float length, slen;
	float *len;
	if(npt == 0)
	{
		return;
	}
	pline.resize(nseg+1);
	for(i=0; i<nseg+1; i++)
		pline[i] = Vec4(0, 0, 0);

	len = new float[npt-1];

	length = 0;
	for(i=1; i<npt; i++)
	{
		len[i-1] = (lines[i]-lines[i-1]).Magnitude();
		length += len[i-1];
	}

	slen = length/nseg;


	pline[0] = lines[0];
	pline[nseg] = lines[npt-1];

	np = 0;
	length = 0;
	k = 1;
	for(i=0; i<npt-1; i++)
	{
		length += len[i];
		if(len[i]<ERR) 
		{
			continue;
		}
		if(length>=slen)
		{
			do
			{
				length = length - slen;
				pline[k] = lines[i+1] + (lines[i]-lines[i+1])*length/len[i];
				k++;
			}while(length>=slen);
		}
	}

	if(k==nseg)
	{
		pline[k] = lines[npt-1];
	}
	delete [] len;
}

//-----------------------------------------------------------------
// Name:	    GetSplineCtrlPtsFromItsLine()
// Description: Get the spline control points from its corresponding polyline
// Argument:    spl:-- spline; ptsArr:--corresponding polyline points array 
//         :	
// Return:		
// Author:       		
// Date:	 	 
// Modified by:	 
// Updated date: 
// copyright: XXX. developed by XXX
//------------------------------------------------------------------
void GetSplineCtrlPtsFromItsLine(Spline& spl,const varray<Vec4>& ptsArr)
{
	if(ptsArr.size() < spl.size())
	{
		return;
	}

	int iNum = 10;

	if(spl.GetMode() == Spline::SPLMODE_CLOSED_BEZIER || spl.GetMode() == Spline::SPLMODE_CLOSED_2BSPLINE
		||spl.GetMode() == Spline::SPLMODE_CLOSED_SPLINE)
	{
		iNum = int(  ( int((ptsArr.size() - 1)) )/spl.size() + 0.5);
	}
	else
	{
		iNum = int( ( int((ptsArr.size() - 1)) )/(spl.size() -1) + 0.5);
	}

	int i = 0;
	for(i = 0; i < spl.size(); i++)
	{
		spl.p(i) = ptsArr[i * iNum];
	}
	if(spl.GetMode() == Spline::SPLMODE_BEZIER || spl.GetMode() == Spline::SPLMODE_2BSPLINE
		|| spl.GetMode() == Spline::SPLMODE_SPLINE)
	{
		spl.pBack() = ptsArr.back();
	}
}

//-----------------------------------------------------------------
// Name:	    GetIdxAndScl()
// Description: Get the index that a point in spline-polyline points array, also get the scale,
// Argument:    ptsArr:-- polyline points array of a spline; pt: inputed point
//         :	iMode:--0,1,2 means interpolate in x,y,z direction
//         :    iIdx:-- index of the points in the polyline array between which the inputed point locate 
// Return:		scl: -- the scale shows how the inputed points locate between two adjacent points in the polylin array 
// Author:   XXX XXX    		
// Date:	2005.3.18 	 
// Modified by:	 
// Updated date: 
// copyright: XXX. developed by XXX
//------------------------------------------------------------------
void GetIdxAndScl(const varray<Vec4>& ptsArr,const Vec4& pt, int iMode,int& iIdx,float& fScl)
{
	int i=0;
	if(iMode==0)
	{	//interpolate by x-direction
		if(pt.x<=min(ptsArr[0].x,ptsArr.back().x))
		{
			iIdx = 0;
			fScl = 0;
		}
		else if(pt.x>=max(ptsArr[0].x,ptsArr.back().x))
		{
			iIdx = ptsArr.size()-2;
			fScl = 1;
		}
		else
		{
			for(i=0; i<ptsArr.size()-1; i++)
			{
				if((ptsArr[i].x <= pt.x && ptsArr[i+1].x > pt.x)
					||(ptsArr[i].x > pt.x && ptsArr[i+1].x <= pt.x) )
				{
					iIdx = i;
					fScl = fabs((pt.x-ptsArr[i].x)/(ptsArr[i+1].x-ptsArr[i].x));
					break;
				}
			}
		}
	}
	else if(iMode==1)
	{	//interpolate by y-direction
		if(pt.y <= min(ptsArr[0].y,ptsArr.back().y))
		{
			iIdx = 0;
			fScl = 0;
		}
		else if(pt.y >= max(ptsArr[0].y,ptsArr.back().y) )
		{
			iIdx = ptsArr.size()-2;
			fScl = 1;
		}
		else
		{
			for(i = 0; i < ptsArr.size()-1; i++)
			{
				if((ptsArr[i].y <= pt.y && ptsArr[i+1].y > pt.y)
					||(ptsArr[i].y > pt.y && ptsArr[i+1].y <= pt.y) )
				{
					iIdx = i;
					fScl = fabs((pt.y-ptsArr[i].y)/(ptsArr[i+1].y-ptsArr[i].y));
					break;
				}
			}
		}
	}
	else if(iMode ==2)
	{
		if(pt.z <= min(ptsArr[0].z,ptsArr.back().z))
		{
			iIdx = 0;
			fScl = 0;
		}
		else if(pt.z >= max(ptsArr[0].z,ptsArr.back().z) )
		{
			iIdx = ptsArr.size()-2;
			fScl = 1;
		}
		else
		{
			for(i = 0; i < ptsArr.size()-1; i++)
			{
				if((ptsArr[i].z <= pt.z && ptsArr[i+1].z > pt.z)
					||(ptsArr[i].z > pt.z && ptsArr[i+1].z <= pt.z) )
				{
					iIdx = i;
					fScl = fabs((pt.z-ptsArr[i].z)/(ptsArr[i+1].z-ptsArr[i].z));
					break;
				}
			}
		}
	}
}
//---------------------------------------------------------------
// Name:	    RotatePlaneToHorizontal()
// Description: Rotate the points on a plane to horizontal 
// Argument:    ptsArr:-- points on a plane;  pNorm:-- plane norm
//         :	ptCen:-- center of the points array
// Return:		
// Author:	    XXX XXX
// Date:	    2006/04/30 30:4:2006   9:02
// Update:	 
// Author:	
// Date: 
// copyright: XXX. developed by XXX
//----------------------------------------------------------------
bool RotatePlaneToHorizontal(varray<Vec4>& ptsArr, Vec4 pNorm , Vec4 ptCen)
{
	float xa = 0, ya = 0,za = 0;
	if(ComputeGlbAngNew(pNorm,&xa,&ya,&za) == 0)
	{
		return false;
	}
	if(ptsArr.size() == 0)
	{
		return false;
	}

	int i = 0;
	Vec4 ptVect;
	//if no center point is given, calculate the center
	if( (ptCen.x + 10e6) < 1 && (ptCen.y + 10e6) < 1 && (ptCen.z + 10e6) < 1 )
	{
		ptCen = Vec4(0,0,0);
		for(i = 0; i < ptsArr.size(); i++)
		{
			ptCen += ptsArr[i];
		}
		if((ptsArr.back() - ptsArr[i]).GetLength() > 0.01)
		{
			ptCen += ptsArr[0];
			ptCen = ptCen / (ptsArr.size() + 1);
		}
		else
		{
			ptCen = ptCen / ptsArr.size();
		}
	}

	for(i = 0; i < ptsArr.size(); i++)
	{
		ptVect = ptsArr[i] - ptCen;
		//	ptVect = ptVect.RotateY(-ya);
		ptVect = ptVect.RotateZ(PI/2 - za);	
		ptVect = ptVect.RotateX(xa - PI/2);

		//	ptVect.y = 0;
		ptsArr[i] = ptCen + ptVect;
	}
	return true;
}
//-----------------------------------------------------------------
// Name:	   TLineIntersect
// Description: get the intersect point for two line,on x-y plane, the z direction is ignored
// Argument:    p11,p121 for the first line two point.
//         :	p21,p22 for the second line two point.
// Return:		
// Author:       		
// Date:	 	 
// Modified by:	 
// Updated date: 04/26/2006
// copyright: XXX. developed by XXX
//------------------------------------------------------------------
bool TLineIntersect(Vec4 p11, Vec4 p12, Vec4 p21, Vec4 p22, Vec4 &sp)
{
	float a, b, u;
	float pro=1.0e-2;

	a = (p12.x-p11.x)*(p22.y-p21.y)-(p12.y-p11.y)*(p22.x-p21.x);
	b = (p22.x-p21.x)*(p11.y-p21.y)-(p22.y-p21.y)*(p11.x-p21.x);

	if(fabs(a)<pro) //two line is parallel
	{
		return false;
	}
	else  
	{
		u = b/a;
	}
	sp = p11 + (p12-p11)*u;

	if(u<-pro||u-1.0>pro) 
	{
		return false;
	}

	if(p21.x-p22.x>pro)
	{
		if(sp.x-p21.x>pro || sp.x-p22.x<-pro)
		{
			return false;
		}
	}
	else if(p21.x-p22.x<-pro)
	{
		if(sp.x-p22.x>pro || sp.x-p21.x<-pro)
		{
			return false;
		}
	}

	if(p21.y-p22.y>pro)
	{
		if(sp.y-p21.y>pro || sp.y-p22.y<-pro)
		{
			return false;
		}
	}
	else if(p21.y-p22.y<-pro)
	{
		if(sp.y-p22.y>pro || sp.y-p21.y<-pro)
		{
			return false;
		}
	}
	return true;
}	
//-----------------------------------------------------------------
// Name:	    TLineTriangleIntersect 
// Description: get the intersect point for line and triangle.
// Argument:    v1,v2 for the first line two point.
//         :	a,b,c for the triangle.fnorm for the plane normal.
//              sp for the intersect point.
// Return:		
// Author:       		
// Date:	 	 
// Modified by:	 
// Updated date: 
// copyright: XXX. developed by XXX
//------------------------------------------------------------------
bool TLineTriangleIntersect(Vec4& v1,Vec4& v2,Vec4& a,Vec4& b,Vec4& c,Vec4&fnorm,Vec4& sp)
{
	float pro1 = 5.0e-2;
	float pro2 = 1.0e-1;
	float val1,val2,val;	
	Vec4 norm_ep,vect_e;
	val1=fnorm.x*(v1.x-a.x)+fnorm.y*(v1.y-a.y)+fnorm.z*(v1.z-a.z);
	val2=fnorm.x*(v2.x-a.x)+fnorm.y*(v2.y-a.y)+fnorm.z*(v2.z-a.z);	

	if(val1*val2>pro1 /*|| fabs(val1-val2)<pro1*/)
	{
		return false;
	}

	float u = val1/(val1-val2);
	sp = v1+(v2-v1) * u; // intersection point
	if(u - 1.0 > pro1 || u < -pro1) 
	{
		return false;
	}

	//whether the point is in the triangle
	vect_e = b - a;
	norm_ep = vect_e.CrossVecX(fnorm).Normalize();
	val=norm_ep.x*(sp.x-a.x)+norm_ep.y*(sp.y-a.y)+norm_ep.z*(sp.z-a.z);

	if(val > pro2)
	{
		return false;					
	}

	vect_e = c - b;
	norm_ep = vect_e.CrossVecX(fnorm).Normalize();
	val=norm_ep.x*(sp.x-b.x)+norm_ep.y*(sp.y-b.y)+norm_ep.z*(sp.z-b.z);

	if(val > pro2)
	{
		return false;
	}

	vect_e = a - c;
	norm_ep = vect_e.CrossVecX(fnorm).Normalize();
	val=norm_ep.x*(sp.x-c.x)+norm_ep.y*(sp.y-c.y)+norm_ep.z*(sp.z-c.z);

	if(val > pro2)
	{
		return false;			
	}
	return true;
}
//---------------------------------------------------------------
// Name:	    AntiClockWiseLoop() 
// Description: Arrange the loop in anticlockwise,the loop should be in well topology
// Argument:    ptsArr:-- points array
//         :	
// Return:		
// Author:		XXX XXX
// Date:		 
// Modified by:	XXX XXX
// Updated date: 2005/12/31 31:12:2005   13:08	
//----------------------------------------------------------------
void AntiClockWiseLoop(varray<Vec4>&ptsArr)
{
	if(ptsArr.size() < 4)
	{
		return;
	}
	Vec4 pt1,pt2,pt3;
	Vec4 vtDir;
	pt1 = ptsArr[0];
	pt2 = ptsArr[ptsArr.size()/4];
	pt3 = ptsArr[ptsArr.size()/2];

	vtDir = pt3 - pt1;

	if((pt2 - pt1).multiple(vtDir).y > 0)
	{
		return;
	}
	varray<Vec4> ptsArrTmp;
	ptsArrTmp.push_back(ptsArr[0]);
	for(int i = ptsArr.size()-1; i > 0; i--)
	{
		ptsArrTmp.push_back(ptsArr[i]);
	}
	ptsArr = ptsArrTmp;
}

//---------------------------------------------------------------
// ptsArr:	    UnitTwoCross() 
// Description: Unit tow cross into one, the two cross should be anticlockwise and horizontal
// Argument:               ---            ---            -----------
//         :	        --     --  +   --    --   =   --            --
// Return:		           ---            ---            -----------
// Author:		XXX XXX
// Date:		 
// Modified by:	XXX XXX
// Updated date: 2005/12/16 16:12:2005   16:55	
//----------------------------------------------------------------
void  UnitTwoCross( varray<Vec4>& ptsArr1, varray<Vec4>& ptsArr2,varray<Vec4>& ptsOut,float xCen)
{
	ptsOut.clear();
	if(ptsArr1.size() == 0 || ptsArr2.size() == 0)
	{
		return;
	}
	int i = 0;
	int iIdxF1 = -1,iIdxB1 = -1,iIdxL1 = -1,iIdxR1 = -1;
	int iIdxF2 = -1,iIdxB2 = -1,iIdxL2 = -1,iIdxR2 = -1;
	float zMax = -1.0e6, zMin = 1.0e6, xMax = -1.0e6,xMin = 1.0e6;
	float xCen1 = 0,xCen2 = 0;

	for(i = 0; i < ptsArr1.size(); i++)
	{
		xCen1 = xCen1 + ptsArr1[i].x; 
	}
	xCen1 /= ptsArr1.size();
	for(i = 0; i < ptsArr2.size(); i++)
	{
		xCen2 += ptsArr2[i].x;
	}
	xCen2 /= ptsArr2.size();

	AntiClockWiseLoop(ptsArr1);
	AntiClockWiseLoop(ptsArr2);

	const varray<Vec4>* pPtsArr1,*pPtsArr2;
	pPtsArr1 = pPtsArr2 = NULL;
	if(xCen1 < xCen2)
	{
		pPtsArr1 = &(ptsArr1);
		pPtsArr2 = &(ptsArr2);
	}
	else
	{
		pPtsArr1 = &(ptsArr2);
		pPtsArr2 = &(ptsArr1);
	}

	//find out the z-min and z-max index
	{
		for(i = 0; i < pPtsArr1->size(); i++)
		{
			const Vec4& pt = pPtsArr1->at(i);
			if(pt.z > zMax && pt.x < xCen)
			{
				zMax = pt.z;
				iIdxF1 = i;
			}
			if(pt.z < zMin && pt.x < xCen)
			{
				zMin = pt.z;
				iIdxB1 = i;
			}
			if(pt.x > xMax)
			{
				xMax = pt.x;
				iIdxR1 = i;
			}
			if(pt.x < xMin)
			{
				xMin = pt.x;
				iIdxL1 = i;
			}
		}

		xMax = zMax = -1.0e6, xMin = zMin = 1.0e6;
		for(i = 0; i < pPtsArr2->size(); i++)
		{
			const Vec4& pt = pPtsArr2->at(i);
			if(pt.z > zMax && pt.x > xCen)
			{
				zMax = pt.z;
				iIdxF2 = i;
			}
			if(pt.z < zMin && pt.x > xCen)
			{
				zMin = pt.z;
				iIdxB2 = i;
			}
			if(pt.x > xMax)
			{
				xMax = pt.x;
				iIdxR2 = i;
			}
			if(pt.x < xMin)
			{
				xMin = pt.x;
				iIdxL2 = i;
			}
		}
	}
	if(iIdxL1 == -1 || iIdxR1 == -1 || iIdxF1 == -1 || iIdxB1 == -1
		||iIdxL2 == -1 || iIdxR2 == -1 || iIdxF2 == -1 || iIdxB2 == -1 )
	{
		assert(false);
		return;
	}
	//Unit two cross section 
	{
		ptsOut.clear();
		if(iIdxF2 > iIdxB2)
		{
			for(i = iIdxF2; i < pPtsArr2->size(); i++)
			{
				if(ptsOut.size() > 1)
				{
					if( (pPtsArr2->at(i) - ptsOut.back()).GetLength() > 0.1)
					{
						ptsOut.push_back(pPtsArr2->at(i));
					}
				}
				else
				{
					ptsOut.push_back(pPtsArr2->at(i));
				}
			}
			for(i = 0; i <= iIdxB2; i++)
			{
				if(ptsOut.size() > 1)
				{
					if( (pPtsArr2->at(i) - ptsOut.back()).GetLength() > 0.1)
					{
						ptsOut.push_back(pPtsArr2->at(i));
					}
				}
				else
				{
					ptsOut.push_back(pPtsArr2->at(i));
				}
			}
		}
		else
		{
			for(i = iIdxF2; i <= iIdxB2; i++)
			{
				if(ptsOut.size() > 1)
				{
					if( (pPtsArr2->at(i) - ptsOut.back()).GetLength() > 0.1)
					{
						ptsOut.push_back(pPtsArr2->at(i));
					}
				}
				else
				{
					ptsOut.push_back(pPtsArr2->at(i));
				}
			}
		}

		if(iIdxB1 > iIdxF1)
		{
			for(i = iIdxB1; i < pPtsArr1->size(); i++)
			{
				if( (pPtsArr1->at(i) - ptsOut.back()).GetLength() > 0.1)
				{
					ptsOut.push_back(pPtsArr1->at(i));
				}	
			}
			for(i = 0; i <= iIdxF1; i++)
			{
				if( (pPtsArr1->at(i) - ptsOut.back()).GetLength() > 0.1)
				{
					ptsOut.push_back(pPtsArr1->at(i));
				}	
			}
		}
		else
		{
			for(i = iIdxB1; i <= iIdxF1; i++)
			{
				if( (pPtsArr1->at(i) - ptsOut.back()).GetLength() > 0.1)
				{
					ptsOut.push_back(pPtsArr1->at(i));
				}	
			}
		}
		Vec4 ptTmp = ptsOut[0];
		ptsOut.push_back(ptTmp);
	}
}


//-----------------------------------------------------------------
// Name:	    GetBoundPointsOfSlice
// Description: Get the enclosing box points and index of the inputed slice points
// Argument:    slice:-- slice points;    num:-- number of slice points
//         :	fpt:-- enclosing box points; idx:-- index of the enclosing box points in the slice points array
// Return:		
// Author:       		
// Date:	 	 
// Modified by:	 
// Updated date: 
// copyright: XXX. developed by XXX
//------------------------------------------------------------------
void GetBoundPointsOfSlice(Vec4 *slice, int num, Vec4 *fpt, int *idx)
{
	float xmax, ymax, zmax, xmin, ymin, zmin;
	xmax = xmin = slice[0].x;
	ymax = ymin = slice[0].y;
	zmax = zmin = slice[0].z;

	for (int i = 0; i < 6; i++)
		idx[i] = 0;

	for (int i = 0; i < num; i++)
	{
		if (slice[i].x > xmax)
		{
			xmax = slice[i].x;
			idx[0] = i;//left point
		}
		else if (slice[i].x < xmin)
		{
			xmin = slice[i].x;
			idx[1] = i;//right point
		}

		if (slice[i].y > ymax)
		{
			ymax = slice[i].y;
			idx[2] = i;//upper point
		}
		else if (slice[i].y < ymin)
		{
			ymin = slice[i].y;
			idx[3] = i;//lower point
		}

		if (slice[i].z > zmax)
		{
			zmax = slice[i].z;
			idx[4] = i;//front point
		}
		else if (slice[i].z < zmin)
		{
			zmin = slice[i].z;
			idx[5] = i;//back point
		}
	}

	fpt[0] = slice[idx[0]];
	fpt[1] = slice[idx[1]];
	fpt[2] = slice[idx[2]];
	fpt[3] = slice[idx[3]];
	fpt[4] = slice[idx[4]];
	fpt[5] = slice[idx[5]];
}

void GetBoundPointsOfSlice(varray<Vec4>& slice, int num, Vec4 *fpt, int *idx)
{
	float xmax, ymax, zmax, xmin, ymin, zmin;
	xmax = xmin = slice[0].x;
	ymax = ymin = slice[0].y;
	zmax = zmin = slice[0].z;

	for (int i = 0; i < 6; i++)
		idx[i] = 0;

	for (int i = 0; i < num; i++)
	{
		if (slice[i].x > xmax)
		{
			xmax = slice[i].x;
			idx[0] = i;//left point
		}
		else if (slice[i].x < xmin)
		{
			xmin = slice[i].x;
			idx[1] = i;//right point
		}

		if (slice[i].y > ymax)
		{
			ymax = slice[i].y;
			idx[2] = i;//upper point
		}
		else if (slice[i].y < ymin)
		{
			ymin = slice[i].y;
			idx[3] = i;//lower point
		}

		if (slice[i].z > zmax)
		{
			zmax = slice[i].z;
			idx[4] = i;//front point
		}
		else if (slice[i].z < zmin)
		{
			zmin = slice[i].z;
			idx[5] = i;//back point
		}
	}

	fpt[0] = slice[idx[0]];
	fpt[1] = slice[idx[1]];
	fpt[2] = slice[idx[2]];
	fpt[3] = slice[idx[3]];
	fpt[4] = slice[idx[4]];
	fpt[5] = slice[idx[5]];
}
//bool IsSelfIntersectForSplnNew(Spline& spl, int iMode)
//{
//	try
//	{
//		varray<Vec4> varV3SplVts;
//		varray<Vec4> varV2SplVts;
//		int nump = 0;
//		int i = 0;
//
//		SplineToPolyline(spl,varV3SplVts,nump);
//		nump = varV3SplVts.size();
//		varV2SplVts.resize(nump);
//		if(iMode == 0)				//x-y
//		{
//			for(i = 0; i < nump; i++)
//			{
//				varV2SplVts[i] = Vec4(varV3SplVts[i].x,varV3SplVts[i].y,0);
//			}
//		}
//		else if(iMode == 1)		//x-z
//		{
//			for(i = 0; i < nump; i++)
//			{
//				varV2SplVts[i] = Vec4(varV3SplVts[i].x,varV3SplVts[i].z,0);
//			}
//		}
//		else if(iMode == 2)		//z-y
//		{
//			for(i = 0; i < nump; i++)
//			{
//				varV2SplVts[i] = Vec4(varV3SplVts[i].z,varV3SplVts[i].y,0);
//			}
//		}
//		else
//		{
//			assert((iMode >= 0) && (iMode <= 2) );
//			return true;
//		}
//
//		float r,s;
//		int tmpNum = nump;
//		for(i = 0; i < nump - 1; i++)
//		{
//			tmpNum = nump;
//			if (i == 0 && spl.GetMode() == Spline::SPLMODE_CLOSED_SPLINE)
//			{
//				tmpNum = nump - 1;
//			}
//
//			for(int k = i+2; k < tmpNum-1; k++)
//			{
//				//---------------------------------------------------
//				//enclosing box check
//				float minX,minY, maxX, maxY;
//				minX = min(varV2SplVts[i].x, varV2SplVts[i+1].x);	minY = min(varV2SplVts[i].y, varV2SplVts[i+1].y);		
//				maxX = max(varV2SplVts[i].x, varV2SplVts[i+1].x); maxY = max(varV2SplVts[i].y, varV2SplVts[i+1].y);
//				bool flag1 = (varV2SplVts[k].x > maxX && varV2SplVts[k+1].x > maxX);
//				bool flag2 = (varV2SplVts[k].x < minX && varV2SplVts[k+1].x < minX);
//				bool flag3 = (varV2SplVts[k].y > maxY && varV2SplVts[k+1].y > maxY);
//				bool flag4 = (varV2SplVts[k].y < minY && varV2SplVts[k+1].y < minY);
//				if( flag1 || flag2 || flag3 || flag4)
//				{
//					continue;
//				}					
//				if(GetTwoLineIntersectRatio(varV2SplVts[i], varV2SplVts[i+1], varV2SplVts[k], varV2SplVts[k+1],r,s))
//				{				
//					return true;	 
//				}
//			}
//		}
//		return false;
//	}
//	catch (...)
//	{
//		return true;
//	}
//}


//-----------------------------------------------------------------
// Name:	   XMeshColliXMesh
// Description: collision detection for two xmesh.
// Argument:    resMesh : the xmesh to be checked.
//         :	basMesh : the base xmesh 
// Return:		
// Author:       XXX		
// Date:	 	 
// Modified by:	 
// Updated date: 04/26/2006
// copyright: XXX. developed by XXX
//------------------------------------------------------------------
void XMeshColliXMesh(XBaseMesh* resMesh,XBaseMesh* basMesh)
{
	float margin = 1;
	if(resMesh->GetVSize()<1||basMesh->GetVSize()<1)
		return;
	int i,j, m;
	Vec4 pt, p1, p2, pf1, pf2, pf3, vt, sp, vect_e;
	Vec4 norm_ep, fnorm;
	float val1, val2, val;
	bool flag;
	//get the extremum value of y coordinate
	float ymin,ymax;
	ymin=10000;
	ymax=-10000;
	for(i=0;i<basMesh->GetVSize();i++)
	{
		if(basMesh->GetV(i).Pos().y>ymax)
		{
			ymax=basMesh->GetV(i).Pos().y;
		}
		if(basMesh->GetV(i).Pos().y<ymin)
		{
			ymin=basMesh->GetV(i).Pos().y;
		}
	}

	for(i=0; i<resMesh->GetVSize(); i++)
	{
		// get the line segment for checking intersection
		if(resMesh->GetV(i).Pos().y>ymax+margin||resMesh->GetV(i).Pos().y<ymin-margin)
		{
			continue;
		}
		pt = resMesh->GetV(i).Pos();
		p1 = pt - resMesh->GetV(i).Norm()*margin;
		p2 = pt + resMesh->GetV(i).Norm()*100.0f;

		for(j=0; j<basMesh->GetFSize(); j++)
		{
			// normal and points of a face
			fnorm = basMesh->GetF(j).Norm();
			pf1 = basMesh->GetV(basMesh->GetF(j).p(0)).Pos();
			pf2 = basMesh->GetV(basMesh->GetF(j).p(1)).Pos();
			pf3 = basMesh->GetV(basMesh->GetF(j).p(2)).Pos();

			val1=fnorm.x*(p1.x-pf1.x)+fnorm.y*(p1.y-pf1.y)+fnorm.z*(p1.z-pf1.z);
			val2=fnorm.x*(p2.x-pf1.x)+fnorm.y*(p2.y-pf1.y)+fnorm.z*(p2.z-pf1.z);	

			if(val1*val2>0 || fabs(val1-val2)<1.0e-6)
			{
				continue;
			}

			sp = p1+(p2-p1) * val1/(val1-val2); // intersection point
			flag = true;

			for(m=0; m<3; m++)
			{
				vt = basMesh->GetV(basMesh->GetF(j).p(m)).Pos();

				if(m==0)     
				{
					vect_e = pf2 - pf1;
				}
				else if(m==1)
				{
					vect_e = pf3 - pf2;
				}
				else	
				{
					vect_e = pf1 - pf3;
				}

				norm_ep = CrossVecX(vect_e,fnorm).Normalize();
				val=norm_ep.x*(sp.x-vt.x)+norm_ep.y*(sp.y-vt.y)+norm_ep.z*(sp.z-vt.z);

				if(val > 0)
				{
					flag = false;					
					break;
				}
			}

			//collision avoidance
			if(flag)
			{
				resMesh->GetV(i).Pos() = sp + resMesh->GetV(i).Norm()*margin;
			}
		}

	}  
	resMesh->ComputeNormals();
}
//---------------------------------------------------------------
// Name:	    SortPtsInAntiClockWise()
// Description: Sort the inputed points array in anticlockwise direction, 
// Argument:    vtsArr:--inputed points array,should be closed 
//         :	pnorm:--norm of the plane that the inputed points on
// Return:		
// Author:	XXX XXX
// Date:	2006/04/29 29:4:2006   14:48
// Update:	 
// Author:	
// Date: 
// copyright: XXX. developed by XXX
//----------------------------------------------------------------
void SortPtsInAntiClockWise(varray<Vec4>& vtsArr,Vec4 pnorm,Vec4 center/* = Vec4(1.0e6,1.0e6,1.0e6)*/)
{
	int i = 0, j = 0, pnum = vtsArr.size();
	//	Vec4 center = Vec4(0, 0, 0); // get the center

	if(fabs(center.x - 1.0e6) < 0.1)
	{
		center = Vec4(0, 0, 0);
		for(i = 0; i < vtsArr.size(); i++)
		{
			center += vtsArr[i];
		}
		center = center/(float)pnum;
	}
	// compute the angle of each point
	float tmp;
	varray<float> pang;
	pang.resize(pnum);

	if(pnorm.x == 0)
	{
		for(i = 0; i < pnum; i++)
		{
			pang[i] = Angc(center.x, center.z, vtsArr[i].x, vtsArr[i].z);
		}
	}
	else if(pnorm.y==0 || (fabs(pnorm.x)>fabs(2*pnorm.y) && fabs(pnorm.x)>fabs(2*pnorm.z)))
	{
		for(i = 0; i < pnum; i++)
		{
			pang[i] = Angc(center.y, center.z, vtsArr[i].y, vtsArr[i].z);
		}
	}
	else
	{
		double xa, ya;
		Vec4 pp;
		pnorm.ComputeGlbAng(&xa, &ya);

		for(i=0; i<pnum; i++)
		{
			pp = vtsArr[i] - center;

			QMatrix4x4 m;
			//m.SetRotateX(-xa);
			m.rotate(-xa, 1, 0, 0);
			QVector3D temp;
			temp.setX(pp.x);
			temp.setY(pp.y);
			temp.setZ(pp.z);
			pp.x = (temp*m).x();
			pp.y = (temp*m).y();
			pp.z = (temp*m).z();

			QMatrix4x4 m2;
			//m2.SetRotateY(-ya);
			m2.rotate(-ya, 0, 1, 0);
			pp.x = (temp*m2).x();
			pp.y = (temp*m2).y();
			pp.z = (temp*m2).z();

			pang[i] = Angc(0, 0, pp.x, pp.y);
		}

	}

	Vec4 vt;
	for(i = 0; i < pnum - 1; i++)
	{
		for(j = i+1; j < pnum; j++)
		{

			if(pang[i] > pang[j])
			{
				tmp = pang[i];
				pang[i] = pang[j];
				pang[j] = tmp;

				vt = vtsArr[i];
				vtsArr[i] = vtsArr[j];
				vtsArr[j] = vt;
			}
		}
	}
}

//---------------------------------------------------------------
// Name:		CalCurveLengthBetween2Points
// Description: Calculate the curve length between two points in retrorse direction
// Argument:	fslice : the point array of the curve
//              fsnum  : the number of curve point;
//              mfpt1, mfpt2:   the two point on the curve.
//              bIsFSliceLoop : if the curve loop
//              pnorm  :  the normal vector to judge the retrorse direction.
// Return:		curve segment length.
// Author:		XXX XXX
// Date:		1:9:2005
// Modified by:		
// Updated date:		
//---------------------------------------------------------------- 
float  CalCurveLengthBetween2Points(Vec4 *fslice, int fsnum, Vec4 mfpt1,Vec4 mfpt2,bool bIsFSliceLoop,Vec4 pnorm)
{
	int i, j, k, fsumTemp;

	int stId = -1, endId= -1;
	Vec4 *fsliceTemp = new Vec4[fsnum+2];
	Vec4 fsliceTempNew[3];
	Vec4 ptmp;

	fsumTemp = 0;
	fsliceTemp[fsumTemp] = mfpt1; 	
	fsumTemp++;
	for(i=0;i<fsnum;i++)
	{  
		if(bIsFSliceLoop)
		{
			if(CrossVecX((mfpt2-mfpt1),(fslice[i]-mfpt1)).y>0)
			{
				fsliceTemp[fsumTemp]=fslice[i];
				fsumTemp++;
			}
		}
		else
		{
			fsliceTemp[fsumTemp]=fslice[i];
			fsumTemp++;
		}
	}
	fsliceTemp[fsumTemp] = mfpt2;
	fsumTemp++;

	// if the fslice is a loop ,sort the points in the order of retrorse direction
	// else sort the points in  x descending 
	float *pang = new float[fsumTemp];
	float tmp;
	// get the center
	Vec4 center;

	center = Vec4(0, 0, 0);
	for(i=0; i<fsnum; i++)
	{
		center += fslice[i];
	}
	center = center/(float)fsnum;

	if(pnorm.x ==0)
	{
		for(i=0; i<fsumTemp; i++)
		{
			pang[i] = Angc(center.x, center.z, fsliceTemp[i].x, fsliceTemp[i].z);
		}
	}
	else if(pnorm.y==0 ||(fabs(pnorm.x)>fabs(2*pnorm.y) && fabs(pnorm.x)>fabs(2*pnorm.z)))
	{
		for(i=0; i<fsumTemp; i++)
		{
			pang[i] = Angc(center.y, center.z, fsliceTemp[i].y, fsliceTemp[i].z);
		}
	}
	else
	{
		double xa, ya;
		Vec4 pp;
		pnorm.ComputeGlbAng(&xa, &ya);

		for(i=0; i<fsumTemp; i++)
		{
			pp = fsliceTemp[i] - center;
			QMatrix4x4 m;
			//m.SetRotateX(-xa);
			m.rotate(-xa, 1, 0, 0);
			QVector3D temp;
			temp.setX(pp.x);
			temp.setY(pp.y);
			temp.setZ(pp.z);
			pp.x = (temp*m).x();
			pp.y = (temp*m).y();
			pp.z = (temp*m).z();

			QMatrix4x4 m2;
			//m2.SetRotateY(-ya);
			m2.rotate(-ya, 0, 1, 0);
			pp.x = (temp*m2).x();
			pp.y = (temp*m2).y();
			pp.z = (temp*m2).z();
			pang[i] = Angc(0, 0, pp.x, pp.y);
		}
	}
	if(bIsFSliceLoop)
	{
		float stAng,endAng;
		stAng = pang[0];
		endAng = pang[fsumTemp-1];
		for(i=0; i<fsumTemp-1; i++)
		{
			for(j=i+1; j<fsumTemp; j++)
			{
				if(pang[i]>pang[j])
				{
					tmp = pang[i];
					pang[i] = pang[j];
					pang[j] = tmp;

					ptmp = fsliceTemp[i];		
					fsliceTemp[i] = fsliceTemp[j];
					fsliceTemp[j] = ptmp;		
				}
			}
		}
		for(i=0;i<fsumTemp;i++)
		{
			if(fabs(pang[i]-stAng)<1.0e-6)
			{
				stId = i;
			}
			if(fabs(pang[i]-endAng)<1.0e-6)
			{
				endId = i;
			}
		}
		if(stId!=0)
		{
			varray<Vec4> vTemp;
			int t = 0;
			for(i=0;i<fsumTemp;i++)
			{
				vTemp.push_back(fsliceTemp[i]);
			}
			for(i = endId+1;i<fsumTemp;i++)
			{
				fsliceTemp[t++] = vTemp.at(i); 
			}
			for(i = 0; i<= endId; i++)
			{
				fsliceTemp[t++] = vTemp.at(i);
			}
		}
	}
	else
	{
		for(j=0; j<fsumTemp-1; j++)
		{
			for(k=j+1; k<fsumTemp; k++)
			{

				if(fsliceTemp[j].x < fsliceTemp[k].x)
				{
					ptmp = fsliceTemp[j];
					fsliceTemp[j] = fsliceTemp[k];
					fsliceTemp[k] = ptmp;
				}
			}
		}
	}

	float len = 0.0;
	for(i=0; i<fsumTemp-1; i++)
	{
		len += (fslice[i+1] - fslice[i]).Magnitude();
	}

	delete []fsliceTemp;
	delete []pang;

	return len;
}
//end of distance calculate 
/////////////////////////////////////////////////////////////////////////

//---------------------------------------------------------------
// Name:	    DivideLoopByAngle() 
// Description: Divide the inputed loop by angle equivalent
// Argument:    plist: point list in the new loop,should be closed;
//         :	norm: plane normal of the loop;    ptsIn: point list in original loop 
//         :    sang: starting angle for the first point in the new loop
//         :    nump: the division number; vtCen-- the center of the loop 
// Return:		
// Author:	XXX XXX
// Date:	2006/04/29 29:4:2006   14:51
// Update:	 
// Author:	
// Date: 
// copyright: XXX. developed by XXX
//----------------------------------------------------------------
void DivideLoopByAngle(const varray<Vec4>& ptsIn, varray<Vec4>& plist, Vec4 norm, int nump, float sang,Vec4 vtCen)
{
	int i, j, k;
	Vec4 cen;
	float *ang;
	float angtmp, dang;
	Vec4 p1, p2;
	float d1, d2;
	int npt = ptsIn.size();
	plist = ptsIn;

	if( (vtCen.x + 10e6) > 1 && (vtCen.y + 10e6) > 1 && (vtCen.y + 10e6) > 1)
	{
		cen = vtCen;
	}
	else
	{
		cen = plist[0];
		for(i=1; i<npt; i++)
		{
			cen += plist[i];
		}

		cen = cen/(float)npt;
	}


	ang = new float[npt];

	if( (norm.y-1.0) <= ERR )
	{
		for(i=0; i<npt; i++)
		{
			ang[i] = Angc(0, 0, plist[i].x - cen.x, plist[i].z-cen.z);
		}

		// order the point
		Vec4 ptmp;
		for(i=0; i<npt-1; i++)
		{
			for(j=i+1; j<npt; j++)
			{
				if(ang[i]>ang[j])
				{
					angtmp = ang[i];
					ang[i] = ang[j];
					ang[j] = angtmp;

					ptmp = plist[i];
					plist[i] = plist[j];
					plist[j] = ptmp;
				}
			}
		}
	}

	Vec4 *newp;
	newp = new Vec4 [nump];

	dang = 2.0f*(float)PI/(float)nump;
	bool sign;

	for(i=0; i<nump; i++)
	{
		angtmp = sang + i*dang;
		if(angtmp > PI*2.0f)
			angtmp -= (float)PI*2.0f;

		sign =false;
		for(k=1; k<npt; k++)
		{
			if(fabs(ang[k-1]-ang[k])>ERR)
			{
				if((angtmp >= ang[k-1] && angtmp <= ang[k]) || (angtmp <= ang[k-1] && angtmp >= ang[k]))
				{
					p1 = plist[k-1];
					p2 = plist[k];

					if( !(fabs(p1.x-p2.x)<=ERR && fabs(p1.y-p2.y)<=ERR && fabs(p1.z-p2.z)<=ERR))
					{
						sign = true;
						break;
					}
				}
			}
		}

		if(!sign)
		{
			p1 = plist[0];

			for(k=npt-1; k>0; k--)
			{
				if(ang[k]>PI)
				{
					p2 = plist[k];
					break;
				}
			}
		}

		// calculate the intersecting point
		Vec4 p1tmp = (p1 - cen).Normalize();
		Vec4 p2tmp = (p2 - cen).Normalize();
		Vec4 centmp = Vec4(0,0,0);
		d1 = GetDistOfPntToLn( p1tmp, centmp, Vec4((float)cos(angtmp), 0, (float)sin(angtmp)) );
		d2 = GetDistOfPntToLn( p2tmp, centmp, Vec4((float)cos(angtmp), 0, (float)sin(angtmp)) );
		//	 d1 = GetDistOfPntToLn( p1, cen, Vec4((float)cos(angtmp), 0, (float)sin(angtmp)) );
		//	 d2 = GetDistOfPntToLn( p2, cen, Vec4((float)cos(angtmp), 0, (float)sin(angtmp)) );
		newp[i] = p1 + (p2-p1)*d1/(d1+d2);
	}

	plist.resize(nump);
	for(i=0; i<nump; i++)
	{
		plist[i] = newp[i];
	}
	delete [] ang;
	delete [] newp;	

	return;
}


bool Get2LineIntersectPtInPlane(Vec4& v11,Vec4& v12,Vec4& v21,Vec4& v22,Vec2& rv,int imode)
{
	if(imode == -1) return false;
	rv.x = rv.y = 0.f;
	Vec2 v1,v2,v3,v4;
	if(imode == 0){
		v1 = Vec2(v11.y,v11.z);
		v2 = Vec2(v12.y,v12.z);
		v3 = Vec2(v21.y,v21.z);
		v4 = Vec2(v22.y,v22.z);
	}
	else if(imode == 1){
		v1 = Vec2(v11.x,v11.z);
		v2 = Vec2(v12.x,v12.z);
		v3 = Vec2(v21.x,v21.z);
		v4 = Vec2(v22.x,v22.z);
	}
	else if(imode == 2){
		v1 = Vec2(v11.x,v11.y);
		v2 = Vec2(v12.x,v12.y);
		v3 = Vec2(v21.x,v21.y);
		v4 = Vec2(v22.x,v22.y);
	}
	float r,s,temp1,temp2,temp3;
	temp1=(v1.y-v3.y)*(v4.x-v3.x)-(v1.x-v3.x)*(v4.y-v3.y);
	temp2=(v2.x-v1.x)*(v4.y-v3.y)-(v2.y-v1.y)*(v4.x-v3.x);
	temp3=(v1.y-v3.y)*(v2.x-v1.x)-(v1.x-v3.x)*(v2.y-v1.y);
	if(abs(temp2) < 1.0e-8){
		return false;
	}	
	r=temp1/temp2;
	s=temp3/temp2;
	if(r >= 0.f && r <= 1.f && s >= 0.f && s <= 1.f)
	{
		rv = v1 + r*(v2 - v1);
		return true;
	}
	return false;
}
//oriarr: the point array to be divided.
//segment: the segment number to get.
//outarr: the output point array.
//if the curve is closed, the first point must be push in the array to make the array closed.
//but the output array just for the unsame points.
bool DivideMultiLineIntoSegMent(const varray<Vec4>& oriarr,const int segnum,varray<Vec4>& outarr,bool bIsClosed)
{
	outarr.clear();
	if(oriarr.size() < 2 || segnum < 1)
		return false;
	if(!bIsClosed)
	{
		MakePolyline(oriarr,outarr,segnum);
		return true;
	}
	else
	{
		varray<Vec4> temparr;
		for(int j=0; j<oriarr.size(); j++)
			temparr.push_back(oriarr.at(j));
		temparr.push_back(oriarr.at(0));
		MakePolyline(temparr,outarr,segnum);
		outarr.pop_back();
		return true;
	}
	float lec = 0.0;
	float les, leseg;
	float ratiol;
	Vec4 vt;
	int i,j;

	for(i=0;i<oriarr.size()-1;i++)
	{
		lec += (oriarr.at(i+1) - oriarr.at(i)).Magnitude();
	}
	les = lec/segnum;
	outarr.push_back(oriarr.at(0));
	lec = 0.0;
	i = j = 0;
	Vec4 vsta,vend;
	vsta = oriarr.at(0);
	vend = oriarr.at(1);
	do 
	{
		leseg = (vend - vsta).Magnitude();
		ratiol = 0.0;
		ratiol = (outarr.size()*les - lec)/leseg;
		if(lec <= outarr.size()*les && lec+leseg >= outarr.size()*les)  {		
			outarr.push_back(vsta*(1.0-ratiol) + vend*ratiol);
			if(leseg*(1-ratiol) >= les){
				lec = (outarr.size()-1)*les;
				vsta = outarr.at(outarr.size()-1);
				vend = oriarr.at(i+1);
				continue;
			}
		}
		lec += leseg;
		i++;
		if(i == oriarr.size() -1){
			if((outarr.at(outarr.size()-1) - vend).Magnitude() > 0.05*les)
				outarr.push_back(vend);
			break;
		}
		vsta = oriarr.at(i);
		vend = oriarr.at(i+1);
	} while(i<oriarr.size()-1);


	if((outarr.at(outarr.size()-1) - outarr.at(0)).Magnitude() < 0.02*les)
		outarr.pop_back();
	if(bIsClosed){
		assert(outarr.size() == segnum);
		return outarr.size() == segnum;
	}
	else{
		assert(outarr.size() == segnum+1);
		return outarr.size() == segnum+1;
	}
}
void DivideMultiLineIntoSegMent(varray<Vec4>& oriarr,const int segnum,bool bIsClosed)
{
	varray<Vec4> outarr;

	if(oriarr.size() < 2 || segnum < 1)
		return;
	if(!bIsClosed)
	{
		MakePolyline(oriarr,outarr,segnum);
	}
	else
	{
		varray<Vec4> temparr;
		for(int j=0; j<oriarr.size(); j++)
			temparr.push_back(oriarr.at(j));
		temparr.push_back(oriarr.at(0));
		MakePolyline(temparr,outarr,segnum);
		outarr.pop_back();
	}
	oriarr.clear();
	for(int i=0; i<outarr.size(); i++)
		oriarr.push_back(outarr.at(i));
}
//void DrawSpherePoint(Vec4 vt,float radius)
//{
//	glPushMatrix();
//	glTranslated(vt.x,vt.y,vt.z);
//	GLUquadricObj*	q = gluNewQuadric();
//	gluQuadricDrawStyle(q, GLU_FILL);
//	gluQuadricNormals(q, GLU_SMOOTH);
//	gluSphere(q, GLdouble(radius), 20, 20);
//	gluDeleteQuadric(q);
//	glPopMatrix();
//}
//void DrawSpherePoint(Vec4 vt,float radius,COLORREF clr)
//{
//	float oldColor[4] = {0,0,0,0};	
//	glGetFloatv(GL_CURRENT_COLOR, oldColor);
//	glColor3f(GetRValue(clr)/255.0f,GetGValue(clr)/255.0f,GetBValue(clr)/255.0f);
//
//	glPushMatrix();
//	glTranslated(vt.x,vt.y,vt.z);
//	GLUquadricObj*	q = gluNewQuadric();
//	gluQuadricDrawStyle(q, GLU_FILL);
//	gluQuadricNormals(q, GLU_SMOOTH);
//	gluSphere(q, GLdouble(radius), 20, 20);
//	gluDeleteQuadric(q);
//	glPopMatrix();
//
//	glColor4fv(oldColor);
//}
void GetArcLengthScaleForArrayPoints(const varray<Vec4>& ptarr,varray<float>& scalarr)
{
	int npt = ptarr.size();
	if( npt < 2)
		return;
	scalarr.clear();
	int i;
	float le,suble;
	le = 0.f;
	for(i=0;i<npt-1;i++)
		le += (ptarr.at(i+1) - ptarr.at(i)).Magnitude();
	suble = 0.f;
	scalarr.push_back(0.f);
	for(i=0;i<npt-1;i++){
		suble += (ptarr.at(i+1) - ptarr.at(i)).Magnitude();
		scalarr.push_back(suble/le);
	}
}
Vec4 InterPolatePtBySacl(const varray<Vec4>& ptarr, float scal)
{
	varray<float> scalarr;
	Vec4 vRet;
	vRet.x = vRet.y = vRet.z = 0.f;
	GetArcLengthScaleForArrayPoints(ptarr,scalarr);
	for(int i=0;i<scalarr.size()-1;i++)
	{
		if(scal >= scalarr.at(i) && scal <= scalarr.at(i))
		{
			vRet = (scal - scalarr.at(i))/(scalarr.at(i+1) - scalarr.at(i)) * (ptarr.at(i+1) - ptarr.at(i)) + ptarr.at(i);
			break;
		}
	}
	return vRet;
}
bool  GetBBoxExtremPtIdx(const varray<Vec4>& parr,int& idx1,int& idx2,int imode)
{
	if(parr.size() == 0)
		return false;
	idx1 = idx2 = 0;
	int i;
	if(imode == 0)
	{
		for(i=1; i<parr.size(); i++)
		{
			if(parr.at(i).x < parr.at(idx1).x)
				idx1 = i;
			if(parr.at(i).x > parr.at(idx2).x)
				idx2 = i;
		}
	}
	else if(imode == 1)
	{
		for(i=1; i<parr.size(); i++)
		{
			if(parr.at(i).y < parr.at(idx1).y)
				idx1 = i;
			if(parr.at(i).y > parr.at(idx2).y)
				idx2 = i;
		}
	}
	else if(imode == 2)
	{
		for(i=1; i<parr.size(); i++)
		{
			if(parr.at(i).z < parr.at(idx1).z)
				idx1 = i;
			if(parr.at(i).z > parr.at(idx2).z)
				idx2 = i;
		}
	}
	return true;
}
//currently just for nhypo = 2;
bool GetSolTwohypoLinearEquation(double *aa,double *bb,double *cc)
{
	double A,B,C;
	A = aa[0] * bb[1] - aa[1] * bb[0];
	B = aa[2] * bb[1] * (-1) - aa[1] * bb[2] * (-1); 
	C = aa[0] * bb[2] * (-1) - aa[2] * bb[0] * (-1);

	if(abs(A) < 1.0e-5)
		return false;
	cc[0] = B / A;
	cc[1] = C / A;
	return true;
}
float GetPartialLength(const varray<Vec4>& vloop,int segidx1,float ratio1,int segidx2,float ratio2,float& otherLength,bool bIsClosed,varray<Vec4>& vMeasurePts,bool bIsCurPart)
{
	if(segidx1 < 0 || segidx1 >= vloop.size() || segidx2 < 0 || segidx2 >= vloop.size())
		return 0.f;
	vMeasurePts.clear();

	otherLength = 0.0;
	varray<Vec4> temploop;
	int i,npt;
	npt = vloop.size();
	for(i=0; i<npt; i++)
		temploop.push_back(vloop.at(i));
	if(bIsClosed && (temploop.at(0) - temploop.at(npt-1)).Magnitude() > 0.1)
		temploop.push_back(vloop.at(0));
	if(!bIsClosed && (temploop.at(0) - temploop.at(npt-1)).Magnitude() < 0.1)
		temploop.pop_back();


	float sum,segsum;
	sum = segsum = 0.f;

	npt = temploop.size();
	for(i=0; i < npt-1; i++)
	{
		sum += (temploop.at(i+1) - temploop.at(i)).Magnitude();
	}

	if(segidx1 == segidx2)
	{
		if(segidx1 < 0 || segidx1 >= npt-1)
			return 0.0;
		segsum = abs(ratio2 - ratio1) * (temploop.at(segidx1 + 1) - temploop.at(segidx1)).Magnitude();
	}
	else
	{
		if(segidx2 < 0 || segidx2 >= npt-1)
			return 0.0;
		if(segidx1 > segidx2)
		{
			std::swap(segidx1,segidx2);
			std::swap(ratio1,ratio2);
		}
		for(i=segidx1; i<segidx2; i++)
			segsum += (temploop.at(i+1) - temploop.at(i)).Magnitude();
		segsum -= (temploop.at(segidx1 + 1) - temploop.at(segidx1)).Magnitude() * ratio1;
		segsum += (temploop.at(segidx2 + 1) - temploop.at(segidx2)).Magnitude() * ratio2;
		vMeasurePts.push_back((temploop.at(segidx1 + 1) - temploop.at(segidx1)) * ratio1 + temploop.at(segidx1));
		for(i=segidx1+1; i<=segidx2; i++)
			vMeasurePts.push_back(temploop.at(i));
		vMeasurePts.push_back((temploop.at(segidx2 + 1) - temploop.at(segidx2)) * ratio2 + temploop.at(segidx2));
	}
	otherLength = sum - segsum;
	if(otherLength < 0)
		otherLength = 0.f;
	if(!bIsCurPart && vMeasurePts.size() > 2)
	{
		if(segidx1 > segidx2)
		{
			std::swap(segidx1,segidx2);
			std::swap(ratio1,ratio2);
		}
		Vec4 vta,vtb;
		vta = vMeasurePts.at(0);
		vtb = vMeasurePts.at(vMeasurePts.size()-1);
		vMeasurePts.clear();

		vMeasurePts.push_back(vtb);
		npt = temploop.size();
		for(i=segidx2+1; i<npt; i++)
			vMeasurePts.push_back(temploop.at(i));
		for(i=0; i<=segidx1; i++)
			vMeasurePts.push_back(temploop.at(i));
		vMeasurePts.push_back(vta);
	}
	return segsum;
}
void DisplayPartialSpl(const varray<Vec4>& vloop, int segidx1,float ratio1,int segidx2,float ratio2,bool bIsClosed,bool whichPart,COLORREF& clr)
{
	if(vloop.size() < 2)
		return;
	varray<Vec4> temploop,disloop;
	Vec4 vt1,vt2;
	int i,npt;
	npt = vloop.size();
	for(i=0; i<npt; i++)
		temploop.push_back(vloop.at(i));
	if(bIsClosed && (temploop.at(0) - temploop.at(npt-1)).Magnitude() > 0.1)
		temploop.push_back(vloop.at(0));
	if(!bIsClosed && (temploop.at(0) - temploop.at(npt-1)).Magnitude() < 0.1)
		temploop.pop_back();

	if(segidx1 < 0 || segidx1 >= npt-1 || segidx2 < 0 || segidx2 >= npt-1)
		return;
	vt1 = (temploop.at(segidx1 + 1) - temploop.at(segidx1)) * ratio1 + temploop.at(segidx1);
	vt2 = (temploop.at(segidx2 + 1) - temploop.at(segidx2)) * ratio2 + temploop.at(segidx2);
	disloop.clear();
	if(whichPart)
	{
		disloop.push_back(vt1);
		disloop.push_back(vt2);
		if(segidx1 != segidx2)
		{
			if(segidx1 > segidx2)
			{
				std::swap(segidx1,segidx2);
				std::swap(ratio1,ratio2);
			}
			for(i=segidx1; i<=segidx2; i++)
				disloop.push_back(temploop.at(i));
		}
	}
	else
	{
		disloop.push_back(vt2);
		for(i=segidx2+1;i<npt;i++)
			disloop.push_back(temploop.at(i));
		if(bIsClosed)
		{
			for(i=0; i<=segidx1;i++)
				disloop.push_back(temploop.at(i));
		}
		disloop.push_back(vt1);
	}
	//DisplayWholeSpl(disloop,false,clr);
}
//void DisplayWholeSpl(const varray<Vec4>& vloop,bool bIsClosed,COLORREF clr)
//{
//	if(vloop.size() < 2)
//		return;
//
//	float oldColor[4] = {0,0,0,0};	
//	glGetFloatv(GL_CURRENT_COLOR, oldColor);
//	glColor3f(GetRValue(clr)/255.0f,GetGValue(clr)/255.0f,GetBValue(clr)/255.0f);
//
//	glDisable(GL_LIGHTING);
//	glBegin(GL_LINES);		
//
//	Vec4 pt1,pt2;
//	int i;
//	int npt = vloop.size();
//	if(!bIsClosed)
//	{
//		for(i =0; i < npt-1; i++)
//		{ 
//			pt1 = vloop.at(i);
//			pt2 = vloop.at(i+1);
//			glVertex3f(pt1.x, pt1.y, pt1.z);
//			glVertex3f(pt2.x, pt2.y, pt2.z);	
//		}
//	}
//	else
//	{
//		for(i =0; i < npt; i++)
//		{ 
//			pt1 = vloop.at(i);
//			if(i < npt - 1)
//				pt2 = vloop.at(i+1);
//			else
//				pt2 = vloop.at(0);
//			glVertex3f(pt1.x, pt1.y, pt1.z);
//			glVertex3f(pt2.x, pt2.y, pt2.z);	
//		}
//	}
//
//	glEnd();
//	glEnable(GL_LIGHTING);
//	glColor4fv(oldColor);
//}

//---------------------------------------------------------------
// Name:		
// Description: get the length ratio of the two intersecting lines
// Argument:	v1: line one first point
//				v2  line one second point
//				v3: line two first point
//				v4: line two second point
//				r:  first line ratio: length start point to intersection with length of start point to end point  
//				s:  second line ratio: length start point to intersection with length of start point to end point
// Return:		
// Author:		unknown
// Date:		1:9:2005
// Modified by:		
// Updated date:		
//---------------------------------------------------------------- 
bool Intersection(const Vec4 &v1, const Vec4 &v2, const Vec4 &v3, const Vec4 &v4, float &r, float &s)
{
	float temp1,temp2,temp3;		
	temp1=(v1.y-v3.y)*(v4.x-v3.x)-(v1.x-v3.x)*(v4.y-v3.y);
	temp2=(v2.x-v1.x)*(v4.y-v3.y)-(v2.y-v1.y)*(v4.x-v3.x);
	temp3=(v1.y-v3.y)*(v2.x-v1.x)-(v1.x-v3.x)*(v2.y-v1.y);

	if(temp2 == 0)
	{
		return false;
	}	

	r=temp1/temp2;
	s=temp3/temp2;
	return true;
}

//---------------------------------------------------------------
// Name:		GetAngleVecWithXCo
// Description: calculate the angle from x axis to the vector
//				range from PI*3/2~-PI/2 ,if(0,0) return 0;
// Argument:	
// Return:		
// Author:		unknown
// Date:		2:9:2005
// Modified by:	 XXX XXX	
// Updated date: 2006/04/29 29:4:2006   13:50		
//---------------------------------------------------------------- 
float GetAngleVecWithXCo(Vec2 v)
{
	float ang=0;
	if(fabs(v.x)<0.001||fabs(v.y)<0.001)
	{
		if(fabs(v.x)<0.001&&fabs(v.y)<0.001)
		{
			ang=0;
		}
		else if(fabs(v.x)<0.001)
		{
			if(v.y>=0)
			{
				ang= (float)(PI*0.5);
			}
			else if(v.y<0)
			{
				ang= (float)(PI*3/2);
			}
		}
		else if(fabs(v.y)<0.001)
		{
			if(v.x>=0)
			{
				ang= 0;
			}
			else if(v.x<0)
			{
				ang= (float)PI;
			}
		}
	}
	else
	{
		if(v.x>0&&v.y>0)
		{
			ang=atan(v.y/v.x);
		}
		else if(v.x<0&&v.y>0)
		{
			ang=(float)(PI+atan(v.y/v.x));
		}
		else if(v.x<0&&v.y<0)
		{
			ang=(float)(PI+atan(v.y/v.x));
		}
		else if(v.x>0&&v.y<0)
		{
			ang=atan(v.y/v.x);
		}
	}
	return ang;
}
//----------------------------------------------------------------------
// name:		GetAngleOf2Vector
// function:	get the angle that vec1 rotate to vec2 in a setting direction
// argument:	vecStr, vecEnd: vector
//				dir: direction. 1: anticlockwise ; -1: clock wise
// return:		angle of two 2d vector
// author:		XXX
// date:		20050927
// update:	    
// author:		
// date:		
//----------------------------------------------------------------------
float GetAngleOf2VectorIn2D(Vec2 vecStr, Vec2 vecEnd, int dir)
{
	float	angle = 0.0;
	vecStr = vecStr.Normalize();
	vecEnd = vecEnd.Normalize();

	float angleToX1 = GetAngleVecWithXCo(vecStr);
	float angleToX2 = GetAngleVecWithXCo(vecEnd);
	if (dir == -1)
	{
		if(angleToX2 < angleToX1 )
		{
			angle = angleToX1 - angleToX2;
		}
		else
		{
			angle = Math::Pi2 - (angleToX2 - angleToX1);
		}
	}
	else if (dir == 1)
	{
		if(angleToX2 < angleToX1 )
		{
			angle = Math::Pi2 - (angleToX1 - angleToX2);
		}
		else
		{
			angle = angleToX2 - angleToX1;
		}
	}
	return angle;
}
// added by qcy 2006.10.23
//---------------------------------------------------------------
// Name:		GetAngleOf2VectorIn2DWithSign()
// Description: get the angle that vec1 rotate to vec2 in a setting direction
// Argument:	vecStr, vecEnd: vector
//         :	dir: direction. 1: anticlockwise ; -1: clock wise
// Return:		angle of two 2d vector, with + or - sign
// Author:	    XXX XXX
// Date:	    2006/06/07 7:6:2006   15:01
// Update:	 
// Author:	
// Date: 
// copyright: XXX. developed by XXX
//----------------------------------------------------------------
float		GetAngleOf2VectorIn2DWithSign(Vec2 vecStr, Vec2 vecEnd, int dir)
{
	float	angle = 0.0;
	vecStr = vecStr.Normalize();
	vecEnd = vecEnd.Normalize();

	float angleToX1 = GetAngleVecWithXCo(vecStr);
	float angleToX2 = GetAngleVecWithXCo(vecEnd);

	angle = (dir == -1) ? (angleToX1 - angleToX2) :  (angleToX2 - angleToX1);
	return angle;
}


//----------------------------------------------------------------------
// name:		GetDistanceFromPtToPolyLine
// function:	compute the distance from a point to polyline
// argument:	Vec4 Vec4Point: point
//				varray<Vec4>& verts: polyline point
// return:		distance of point to a polyline
// author:		XXX
// date:		2006-11-24
// update:	    
// author:		
// date:		
//----------------------------------------------------------------------
float GetDistanceFromPtToPolyLine(const Vec4& Ve3Point, const varray<Vec4>& verts, int &iSeg, float d)
{
	float dSq = 1.0e6;
	float dSqTemp = 1.0e6;
	Vec4 interPt;
	float xMinA,yMinB,zMinC;
	float xNextMinA,yNextMinB,zNextMinC;
	float xMinASq,yMinBSq,zMinCSq;
	float xNextMinASq,yNextMinBSq,zNextMinCSq;

	xMinA = verts.at(0).x - Ve3Point.x;
	yMinB = verts.at(0).y - Ve3Point.y;
	zMinC = verts.at(0).z - Ve3Point.z;

	xMinASq = xMinA*xMinA;
	yMinBSq = yMinB*yMinB;
	zMinCSq = zMinC*zMinC;

	xNextMinA = verts.at(1).x - Ve3Point.x;
	yNextMinB = verts.at(1).y - Ve3Point.y;
	zNextMinC = verts.at(1).z - Ve3Point.z;

	xNextMinASq = xNextMinA*xNextMinA;
	yNextMinBSq = yNextMinB*yNextMinB;
	zNextMinCSq = zNextMinC*zNextMinC;

	if (d < 0.0)
	{
		d = GetDistanceFromPtToLineSeg(Ve3Point,verts[0], verts[1], interPt);
		dSq = min(d*d,xMinASq+yMinBSq+zMinCSq);
		dSq = min(dSq,xNextMinASq+yNextMinBSq+zNextMinCSq);
	}
	else
	{
		dSq = d*d;
	}

	for (int i=0; i<static_cast<int>(verts.size()-1); i++)
	{
		//rejection test
		if (((xMinASq>dSq)&&(xNextMinASq>dSq)&&(xMinA*xNextMinA>0.0)) 
			|| ((yMinBSq>dSq)&&(yNextMinBSq>dSq)&&(yMinB*yNextMinB>0.0)) 
			|| ((zMinCSq>dSq)&&(zNextMinCSq>dSq)&&(zMinC*zNextMinC>0.0)))
		{
			if (i != static_cast<int>(verts.size()-2))
			{
				xMinA = xNextMinA;
				yMinB = yNextMinB;
				zMinC = zNextMinC;

				xNextMinA = verts[i+2].x - Ve3Point.x;
				yNextMinB = verts[i+2].y - Ve3Point.y;
				zNextMinC = verts[i+2].z - Ve3Point.z;

				xMinASq = xMinA*xMinA;
				yMinBSq = yMinB*yMinB;
				zMinCSq = zMinC*zMinC;

				xNextMinASq = xNextMinA*xNextMinA;
				yNextMinBSq = yNextMinB*yNextMinB;
				zNextMinCSq = zNextMinC*zNextMinC;
			}
			continue;
		}

		//check distance to line
		d = GetDistanceFromPtToLineSeg(Ve3Point,verts[i], verts[i+1], interPt);
		dSqTemp = min(d*d,xMinASq+yMinBSq+zMinCSq);
		dSqTemp = min(dSqTemp,xNextMinASq+yNextMinBSq+zNextMinCSq);

		if (dSqTemp < dSq)
		{
			dSq =dSqTemp;
			iSeg = i;
		}
		if (i != static_cast<int>(verts.size()-2))
		{
			xMinA = xNextMinA;
			yMinB = yNextMinB;
			zMinC = zNextMinC;

			xNextMinA = verts[i+2].x - Ve3Point.x;
			yNextMinB = verts[i+2].y - Ve3Point.y;
			zNextMinC = verts[i+2].z - Ve3Point.z;

			xMinASq = xMinA*xMinA;
			yMinBSq = yMinB*yMinB;
			zMinCSq = zMinC*zMinC;

			xNextMinASq = xNextMinA*xNextMinA;
			yNextMinBSq = yNextMinB*yNextMinB;
			zNextMinCSq = zNextMinC*zNextMinC;
		}
	}
	return sqrt(dSq);
}


bool IsPointOnPlane(Vec4 inputPoint, Vec4 PointOnPlane, Vec4 PlaneNorm)
{
	Vec4 dir = (inputPoint - PointOnPlane).Normalize();
	if (abs(Dot(dir, PlaneNorm)) < 1.0e-3)
		return true;
	else
		return false;
}
//---------------------------------------------------------------
// Name:		IsPointInPolygon
// Description: the point is in polygon(2D) or not, only support convex polygon
//				Not used. 
// Argument:			polygonP: polygon point array
//						testP	: point
// Return:		true for in polygon
// Author:		
// Date:		
// Modified by:	XXX
// Updated date:20060504
//----------------------------------------------------------------
bool IsPointInPolygon(Vec4 testP, varray<Vec4> &polygonP)
{
	int i,j;
	int nCross = 0;
	int nTempCross = 1;

	for(i=0; i < static_cast<int>(polygonP.size());i++)
	{
		j = i+1;
		if(j == static_cast<int>(polygonP.size()) )
		{
			j = 0;
		}
		Vec4 V1 = polygonP.at(j);
		Vec4 V2 = polygonP.at(i);
		if((testP.y<=V2.y && testP.y>=V1.y)||(testP.y>=V2.y && testP.y<=V1.y))
		{
			float z = CrossVecX(testP-V1, testP-V2).z;
			if(z == 0.0)
			{
				return false;
			}
			else if(z>0)
			{
				nCross += nTempCross;
			}
			else
			{
				nCross -= nTempCross;
			}
		}
	}
	if(abs(nCross) == static_cast<int>(polygonP.size()))
	{
		return true;
	}
	return false;
}

//2004-09-10 XXX
//by using ray method
// check if the point is in the convex polygon in 2d.
// check the number of point of intersection, if the num is even(out of polygon) odd (in polygon); 
//---------------------------------------------------------------
// Name:		IsPointInConexPolygon
// Description: the point is in polygon(2D) or not, only support convex polygon
// Argument:			polygonP: polygon point array
//						testP	: point
// Return:		true for in polygon
// Author:		
// Date:		
// Modified by:	XXX
// Updated date:20060504
//----------------------------------------------------------------
bool IsPointInConexPolygon(Vec2 testP, const varray<Vec2>& polygon)
{
	int size=static_cast<int>(polygon.size());
	int intersize=0;
	bool flag1, flag2;
	float threshold=(float)1.0e-6; 
	Vec2 v1, v2;
	//first with bounded box 
	float xmin,ymin;
	float xmax,ymax;
	xmin = ymin = 1.0e6;
	xmax = ymax = -1.0e6;
	for(int j = 0;j<size;j++)
	{
		if(polygon.at(j).x>xmax)
		{
			xmax = polygon.at(j).x;
		}
		if(polygon.at(j).x<xmin)
		{
			xmin = polygon.at(j).x;
		}
		if(polygon.at(j).y>ymax)
		{
			ymax = polygon.at(j).y;
		}
		if(polygon.at(j).y<ymin)
		{
			ymin = polygon.at(j).y;
		}
	}
	//	check if is in the bonding box
	if(testP.x <= xmin||testP.x >= xmax|| testP.y >= ymax||testP.y <= ymin) 
		return false; 

	//--------------filter the same point,XXX XXX 2006.2.8-----------------
	varray<Vec2> polygonTmp;
	polygonTmp.push_back(polygon[0]);
	for(int i = 1; i < static_cast<int>(polygon.size()); i++)
	{
		if((polygonTmp.back() - polygon[i]).Magnitude() > threshold)
		{
			polygonTmp.push_back(polygon[i]);
		}
	}
	size = static_cast<int>(polygonTmp.size());
	int tt = 0 , i = 0;
	varray<bool> vIsVisited;
	for(i = 0; i < size; i++)
	{
		vIsVisited.push_back(false);
	}
	for(i=0;i<size;i++)
	{
		tt = (i+1)%size;
		v1=polygonTmp.at(i);
		v2=polygonTmp.at(tt);

		if(v1.x<testP.x && v2.x<testP.x)
			continue;

		flag1=fabs(v2.y-testP.y)<threshold;

		if(flag1 && !vIsVisited[tt])  //conditions as pic 6,7 in wangjin's paper "line clipping against polygonal window algorithm based on the mutiple virtual boxes rejecting
		{
			if(fabs(v1.y-testP.y)<threshold) //XXX XXX add,2006.2.8,see pic7(a)(b)(c)
			{
				if(IsPolygonVertexConvex(polygonTmp,i)) 
				{
					intersize++;
				}
				if(IsPolygonVertexConvex(polygonTmp,tt))
				{
					intersize++;
				}
			}
			else
			{
				flag2=(v1.y-testP.y)*(polygonTmp.at((i+2)%size).y-testP.y)>=0;//pic6(b)(c)
				if(!flag2)//pic 6(a)
				{
					intersize++;
				}
			}
			vIsVisited[tt] = true;
			continue;
		}
		else if(fabs(v1.y-testP.y)<threshold && !vIsVisited[i])
		{
			Vec2 v0 = polygonTmp.at((i+size-1)%size);
			if(fabs(v0.y-testP.y)<threshold) //XXX XXX add,2006.2.8,see pic7(a)(b)(c)
			{
				if(IsPolygonVertexConvex(polygonTmp,i)) 
				{
					intersize++;
				}
				if(IsPolygonVertexConvex(polygonTmp,tt))
				{
					intersize++;
				}
			}
			else
			{
				flag2=(v0.y-testP.y)*(v2.y-testP.y)>=0;//pic6(b)(c)
				if(!flag2)//pic 6(a)
				{
					intersize++;
				}
			}
			vIsVisited[i] = true;
			continue;
		}

		flag1=v1.y>testP.y;
		flag2=v2.y>testP.y;
		if(flag1&&!flag2 || !flag1&&flag2)
		{
			if(IsSegmentsIntersect(testP,Vec2(1.0e8,testP.y),v1,v2))
			{
				intersize++;
			}
		}

	}
	return (intersize%2 == 1);
}
//---------------------------------------------------------------
// Name:	    IsPolygonVertexConvex()
// Description: Judge whether the given polygon vertex is a convex
// Argument:    vtsPoly:polygon vertices array;
//         :	idx:-- vertex index to be tested
// Return:		bool: true:-- if the vertex is convex
// Author:		XXX XXX
// Date:		 
// Modified by:	XXX XXX
// Updated date: 2006/02/08 8:2:2006   14:47	
//----------------------------------------------------------------
bool IsPolygonVertexConvex(varray<Vec2>&vtsPoly, int idx)
{
	if(idx < 0 || idx >= static_cast<int>(vtsPoly.size()))
	{
		return false;
	}
	int i = 0;
	int iL = -1,iR = -1,iT = -1,iB = -1;
	Vec2 vtLeft,vtRight,vtTop,vtBot;
	vtLeft.x = vtBot.y = 1.0e6;
	vtRight.x = vtTop.y = -1.0e6;
	for(i = 0; i < static_cast<int>(vtsPoly.size()); i++)
	{
		if(vtLeft.x > vtsPoly[i].x)
		{
			vtLeft = vtsPoly[i];
			iL = i;
		}
		if(vtRight.x < vtsPoly[i].x)
		{
			vtRight = vtsPoly[i];
			iR = i;
		}
		if(vtBot.y > vtsPoly[i].y)
		{
			vtBot = vtsPoly[i];
			iB = i;
		}
		if(vtTop.y < vtsPoly[i].y)
		{
			vtTop = vtsPoly[i];
			iT = i;
		}
	}
	float Ri,Rk = -1;
	Vec2 dir1,dir2,vt1,vt2,vt3;

	if(iT >= iR && iL >= iT)
	{
		Rk = 1;
	}
	int iSize = static_cast<int>(vtsPoly.size());
	vt2 = vtsPoly[idx];
	vt1 = vtsPoly[(idx-1+iSize)%iSize];
	vt3 = vtsPoly[(idx+1)%iSize];
	dir1 = (vt2 - vt1).Normalize();
	dir2 = (vt3 - vt2).Normalize();

	Vec4 vect1,vect2;
	vect1.Set(dir1.x,0,dir1.y);
	vect2.Set(dir2.x,0,dir2.y);
	Ri = CrossVecX(vect1,vect2).y;
	bool bRet = bool (Ri * Rk > 0); 
	return bRet;
}
//---------------------------------------------------------------
// Name:		IsPtAndTriangleAtFaceDiffSide
// Description: the point and triangle on face's different side
// Argument:			pt: point
//						pa,pb,pc: triangle points
//						norm:     face normal vector
//						ptinface: point in face
// Return:		true for on different side
// Author:		
// Date:		
// Modified by:	XXX
// Updated date:20060504
//----------------------------------------------------------------
bool IsPtAndTriangleAtFaceDiffSide(const Vec4 &pt,const Vec4 &pa,const Vec4 &pb,
								   const Vec4 &pc,const Vec4 &norm,const Vec4 &ptinface)
{
	const double RangeLong = 0.0001;
	const double NRangeLong= -0.0001;

	double pt_ptdot = Dot(pt - ptinface, norm);
	double pa_ptdot = Dot(pa - ptinface, norm);
	double pb_ptdot = Dot(pb - ptinface, norm);
	double pc_ptdot = Dot(pc - ptinface, norm);

	if( pt_ptdot >= NRangeLong && pa_ptdot <= RangeLong
		&& pb_ptdot <= RangeLong && pc_ptdot <= RangeLong
		|| pt_ptdot <= RangeLong && pa_ptdot >= NRangeLong
		&& pb_ptdot >= NRangeLong && pc_ptdot >= NRangeLong )
	{
		return true;
	}
	else
	{
		return false;
	}
}

//---------------------------------------------------------------
// Name:		IsTwoPointAtLineSameSide
// Description: the two points is at the line's same side or not
// Argument:	pt1, pt2: point1, point2		
//				lineStPt, lineEndPt: the start and end point of line
//				
// Return:		true for at same size
// Author:		unknown
// Date:		
// Modified by:		XXX
// Updated date:	20060504
//---------------------------------------------------------------- 
bool IsTwoPointAtLineSameSide(const Vec4 &pt1,const Vec4 &pt2,
							  const Vec4 &lineStPt,const Vec4 &lineEndPt)
{
	if(lineStPt.Equals(lineEndPt, 0.00001f) )//XXX 2002-12-09
	{
		return true;
	}

	Vec4 pt1andclinenorm = CrossVecX((lineStPt - lineEndPt), (pt1 - lineStPt) );
	Vec4 pt2andclinenorm = CrossVecX((lineStPt - lineEndPt), (pt2 - lineStPt) );

	if(IsSharpAngleOfTwoVert(pt1andclinenorm, pt2andclinenorm) )
	{
		return true;
	}
	else
	{
		return false;
	}
}




/////////////////////////////////////////////////////////////////////////////////
//line operator
//---------------------------------------------------------------
// Name:		IsTwoLinesAtSamePos
// Description: the two line has same position or not, i.e superposition
// Argument:			lineOneSt, lineOneEnd: the start and end point of line1
//						lineTwoSt, lineTwoEnd: the start and end point of line2
// Return:		true for has same position
// Author:		unknown
// Date:		
// Modified by:		XXX
// Updated date:	20060504
//---------------------------------------------------------------- 
bool IsTwoLinesAtSamePos(const Vec4 &lineOneSt,const Vec4 &lineOneEnd,
						 const Vec4 &lineTwoSt,const Vec4 &lineTwoEnd)
{
	const double RangeLong = 0.001;//0.0001;
	if((lineOneSt.Equals(lineTwoSt, (float)RangeLong) && lineOneEnd.Equals(lineTwoEnd, (float)RangeLong))
		||(lineOneSt.Equals(lineTwoEnd, (float)RangeLong) && lineOneEnd.Equals(lineTwoSt, (float)RangeLong)))
	{
		return true;
	}
	return false;
}

//---------------------------------------------------------------
// Name:		IsLinePlaneIntersect
// Description: judge the line is intersect with a plane or not,
//				just judge the point of line is in one side or two side of the plane
// Argument:			lineStart   :the start point of line
//						lineEnd     :the end point of line
//						planePoint  :one point in the face
//						planeNorm   :norm of the face
// Return:		ture if intersect
// Author:		ljt
// Date:		10:12:2004
// Modified by:	XXX	
// Updated date:2006/05/04		
//---------------------------------------------------------------- 
bool IsLinePlaneIntersect(Vec4 lineStart, Vec4 lineEnd, Vec4 planePoint, Vec4 planeNorm )
{
	float dot1 = Dot((lineStart - planePoint).Normalize(), planeNorm.Normalize());
	float dot2 = Dot((lineEnd - planePoint).Normalize(), planeNorm.Normalize());
	if(fabs(dot1) < 1.0e-4 || fabs(dot2) < 1.0e-4)
	{ //one point of line on the plane
		return  true;
	}
	return (dot1*dot2 < 0);
}


//---------------------------------------------------------------
// Name:		IsTwoEdgeIntersect
// Description: two 2D line segment is intersect or not in the appointed view
// Argument:			v1, v2: the start and end point of line1
//						v3, v4: the start and end point of line2
//						viewid: view Id, i.e.front view or side view or top view
// Return:		true if intersect
// Author:		
// Date:		
// Modified by:		XXX
// Updated date:	2006/05/05
//---------------------------------------------------------------- 
bool IsTwoEdgeIntersect(Vec4 v1, Vec4 v2, Vec4 v3, Vec4 v4,int viewid)
{
	Vec4 m1 = v1-v3;
	Vec4 m2 = v2-v3;
	Vec4 m3 = v1-v4;
	Vec4 m4 = v2-v4;

	if(viewid == 0)
	{   //x-y
		if(CrossVecX(m1,m2).z*CrossVecX(m3,m4).z>0)
			return false;
	}
	else if(viewid == 1)
	{	//y-z
		if(CrossVecX(m1,m2).x*CrossVecX(m3,m4).x>0)
			return false;
	}
	else if(viewid == 2) //top view
	{   //x-z
		if(CrossVecX(m1,m2).y*CrossVecX(m3,m4).y>0)
			return false;
	}
	m1 = v3-v1;
	m2 = v4-v1;
	m3 = v3-v2;
	m4 = v4-v2;
	if(viewid == 0)
	{
		if(CrossVecX(m1,m2).z*CrossVecX(m3,m4).z>0)
			return false;
	}
	else if(viewid==1)
	{
		if(CrossVecX(m1,m2).x*CrossVecX(m3,m4).x>0)
			return false;
	}
	else if(viewid == 2)
	{
		if(CrossVecX(m1,m2).y*CrossVecX(m3,m4).y>0)
			return false;
	}
	return true;
}





//---------------------------------------------------------------
// Name:	    GetTwoPolyLineInterPt()
// Description: Get the intersection point of two polyline in a appointed view, only one intersection points case is accepted 
// Argument:    vtsArr1,vtsArr2: two polyline;     vtInter: intersection point
//         :	iViewId: 0:--check in x-y direction, 1: x-z direction 2: z-y direction
// Return:		true: if intersection is computed
// Author:		XXX XXX
// Date:		2006/02/24 24:2:2006   11:44
// Update:	 
// Author:	
// Date: 
// copyright: XXX. developed by XXX
//----------------------------------------------------------------
bool GetTwoPolyLineInterPtIn2D(int iViewId, varray<Vec4>& vtsArr1, varray<Vec4>& vtsArr2, Vec4& vtInter)
{
	//precondition
	if(vtsArr1.size() < 2 || vtsArr2.size() < 2 || iViewId < 0 && iViewId >= 3)
	{
		return false;
	}
	//compute....
	int				i = 0, j = 0;
	Vec2			vt1,vt2,pt1,pt2;
	varray<Vec2>	vec2Arr1,vec2Arr2;

	if(iViewId == 0)
	{   //x-y
		for(i = 0; i < static_cast<int>(vtsArr1.size()); i++)
		{
			vec2Arr1.push_back(Vec2(vtsArr1[i].x,vtsArr1[i].y));
		}
		for(i = 0; i < static_cast<int>(vtsArr2.size()); i++)
		{
			vec2Arr2.push_back(Vec2(vtsArr2[i].x,vtsArr2[i].y));
		}
	}
	else if(iViewId == 1)
	{   //x-z
		for(i = 0; i < static_cast<int>(vtsArr1.size()); i++)
		{
			vec2Arr1.push_back(Vec2(vtsArr1[i].x,vtsArr1[i].z));
		}
		for(i = 0; i < static_cast<int>(vtsArr2.size()); i++)
		{
			vec2Arr2.push_back(Vec2(vtsArr2[i].x,vtsArr2[i].z));
		}
	}
	else if(iViewId == 2)
	{   //y-z
		for(i = 0; i < static_cast<int>(vtsArr1.size()); i++)
		{
			vec2Arr1.push_back(Vec2(vtsArr1[i].z,vtsArr1[i].y));
		}
		for(i = 0; i < static_cast<int>(vtsArr2.size()); i++)
		{
			vec2Arr2.push_back(Vec2(vtsArr2[i].z,vtsArr2[i].y));
		}
	}

	float minX = 0,minY = 0, maxX = 0, maxY = 0;
	bool flag1 = false,flag2 = false,flag3 = false,flag4 = false;
	for(i = 0; i < static_cast<int>(vec2Arr1.size()-1); i++)
	{
		vt1 = vec2Arr1[i];
		vt2 = vec2Arr1[i+1];
		for(j = 0; j < static_cast<int>(vec2Arr2.size()-1); j++)
		{
			pt1 = vec2Arr2[j];
			pt2 = vec2Arr2[j+1];
			//box test
			minX = min(vt1.x, vt2.x);	minY = min(vt1.y, vt2.y);		
			maxX = max(vt1.x, vt2.x);   maxY = max(vt1.y, vt2.y);
			flag1 = (pt1.x > maxX && pt2.x > maxX);
			flag2 = (pt1.x < minX && pt2.x < minX);
			flag3 = (pt1.y > maxY && pt2.y > maxY);
			flag4 = (pt1.y < minY && pt2.y < minY);
			if( flag1 || flag2 || flag3 || flag4)
			{
				continue;
			}

			Vec4 vt11 = Vec4(vt1.x,vt1.y,0);
			Vec4 vt22 = Vec4(vt2.x,vt2.y,0);
			Vec4 pt11 = Vec4(pt1.x,pt1.y,0);
			Vec4 pt22 = Vec4(pt2.x,pt2.y,0);
			if(IsTwoLineIntersectIn2d(vt11, vt22, pt11, pt22))		
			{				
				if(i == static_cast<int>( vec2Arr1.size()-1) && j == static_cast<int>(vec2Arr2.size() - 1))
				{
					return false;
				}
				else
				{

					Vec2 vec2Inter = GetTwoLineInterPt2D(vt1,vt2,pt1,pt2);
					if(iViewId == 0)
					{
						vtInter = Vec4(vec2Inter.x,vec2Inter.y,vtsArr1[0].z);
					}
					else if(iViewId == 1)
					{
						vtInter = Vec4(vec2Inter.x,vtsArr1[0].y,vec2Inter.y);
					}
					else if(iViewId == 2)
					{
						vtInter = Vec4(vtsArr1[0].x,vec2Inter.y,vec2Inter.x);
					}
					return true;
				} 
			}
		}
	}
	return false;
}

//---------------------------------------------------------------
// Name:		GetArrayIntersection
// Description: compare the two array and return the elements which they both have
// Argument:	
// Return:		
// Author:		XXX
// Date:		1:9:2005
// Modified by:	XXX	
// Updated date:20060507		
//---------------------------------------------------------------- 
varray<int> GetArrayIntersection(varray<int>&selMeshIdArr, varray<int>&refMsidArr)
{
	varray<int> intersection;
	int size1=static_cast<int>(selMeshIdArr.size());
	int size2=static_cast<int>(refMsidArr.size());
	int i,j;
	for(i=0;i<size1;i++)
	{
		for(j=0;j<size2;j++)
		{
			if(selMeshIdArr.at(i)==refMsidArr.at(j))
			{
				intersection.push_back(selMeshIdArr.at(i));
			}
		}
	}
	return intersection;
}

//---------------------------------------------------------------
// Name:		AddNewCtrlPtToSimplefyLine
// Description: add some control point when the curve is smooth as a straight line.
//				if the simplified line is larger then error allowance compare with origen line
//				add new control point
// Argument:	expandMeshProcision: just see the document.     :)
//				oriPArr  : original control point of the curve
//				resPArr  : the control point to return 
// Return:		
// Author:		cl
// Date:		21:12:2004
// Modified by:	XXX 	
// Updated date:20060507		
//---------------------------------------------------------------- 
void AddNewCtrlPtToSimplefyLine(const varray<Vec4>& oriPArr,varray<Vec4>& resPArr,float expandMeshProcision)
{
	varray< float > localheight;
	varray< float > localmaxheight;
	float			maxheight = 0;
	float			heih;
	int				maxitemId = -1;
	int				kk = 0;
	int				jj = 0;
	int				flag1 = 0, flag2 = 0; //the respond position in origen point array for position in resPArr
	varray < int >	neededInsertPtId;
	int				added = 0;
	bool			modflag1 = true;   //flag for flag1 has been set
	bool			modflag2 = true;   //flag for flag2 has been set
	do
	{
		localmaxheight.clear();
		neededInsertPtId.clear();
		for(kk=0;kk<static_cast<int>(resPArr.size()-2);kk++)
		{
			localheight.clear();
			modflag1=true;
			modflag2=true;
			//get position in oriPArr for k,kk+1
			for(jj=0; jj<static_cast<int>(oriPArr.size()); jj++)
			{
				if(modflag1)
				{
					if(oriPArr.at(jj)==resPArr.at(kk))
					{
						flag1 = jj;
						modflag1=false;
					}
				}
				if(modflag2)
				{
					if(oriPArr.at(jj)==resPArr.at(kk+1))
					{
						if(jj<flag1)
							continue;
						flag2=jj;
						modflag2=false;
					}
				}
			}
			if(!flag1 && !flag2)
			{
				jj = flag2-flag1;
				if(jj < 2)
					continue;
				//calculate the distance for the point between flag1 and flag2 to line kk,kk+1
				for(jj=flag1+1; jj<flag2; jj++)
				{		   
					localheight.push_back(GetDistOfPntToLn(oriPArr.at(jj),resPArr.at(kk),resPArr.at(kk+1)));
				} 

				if(localheight.size() == 0)
					continue;

				//get the max distance to line kk,kk+1
				heih = 0;
				for(jj=0;jj<static_cast<int>(localheight.size());jj++)
				{
					if(localheight.at(jj) > heih)
					{
						heih = localheight.at(jj);
						maxitemId=jj;
					}
				}
				//add by XXX XXX to avoid error
				if(maxitemId == -1)
				{
					continue;
				}

				if(localheight.at(maxitemId) > expandMeshProcision)
				{
					neededInsertPtId.push_back(kk);
					neededInsertPtId.push_back(flag1+maxitemId+1);
				}
				localmaxheight.push_back(localheight.at(maxitemId));
			}
			else 
				continue;
		}

		//insert new point
		added = 1;
		for(jj=0; jj<static_cast<int>(neededInsertPtId.size()/2); jj++)
		{	   
			resPArr.insert(&resPArr.at(neededInsertPtId.at(2*jj)+added),oriPArr.at(neededInsertPtId.at(2*jj+1)));
			added++;
		}
		//get max distance to whole simplify line
		heih = 0;
		if(localmaxheight.size() == 0)
			break;
		for(jj=0; jj<static_cast<int>(localmaxheight.size()); jj++)
		{
			if(localmaxheight.at(jj)>heih)
			{
				heih=localmaxheight.at(jj);
				maxitemId=jj;
			}
		}
		maxheight=localmaxheight.at(maxitemId);
	} while( maxheight > expandMeshProcision);
}

//以下函数注意度的计算
void SplineToPolyline(Spline& spl, varray<Vec4>& vtsOut)
{
	int i;
	double t;
	Vec4 p;
	int np=0;	
	varray<Vec4> plist;
    
	bool basserted = false;
	int pointN;
	//曲线整体处于【0,1】，因此不用循环,所有控制点给出一条曲线，因此曲线不用分段。
	if(spl.GetMode() == Spline::SPLMODE_BEZIER || spl.GetMode() == Spline::SPLMODE_CLOSED_BEZIER 
		|| spl.GetMode() == Spline::SPLMODE_NONUNI_NBSPLINE)
	{
	    pointN = 1.f/DELTA2 + 1;
		for(int kk = 0; kk < pointN; kk++)
		{
		    t = kk * DELTA2;
			p = spl.GetPoint( 0 , t );
			if(np>0)
			{
				if( (p-plist[np-1]).Magnitude()>ERR)
				{
					plist.push_back(p);
					np++;
				}
				else
				{
					if(!basserted)
					{
						assert(FALSE);
						basserted = true;
					}
				}
			}
			else
			{
				plist.push_back(p);
				np++;
			}
		}
	}
	//曲线每两个控制点之间都有区间【0,1】，因此循环。即整条曲线是由多条曲线拼接而成。由于曲线阶次都为二次，因此若有n个控制点，则有n-1条曲线。
	else 
	{
		int size = 0;	
		if(spl.GetMode() == Spline::SPLMODE_CLOSED_SPLINE || spl.GetMode() == Spline::SPLMODE_CLOSED_2BSPLINE
			|| spl.GetMode() == Spline::SPLMODE_CLOSED_3BSPLINE  || spl.GetMode() == Spline::SPLMODE_CLOSED_NBSPLINE)
		{
			size = spl.GetCtrlPointCount();
		}
		else 
		{
            size = spl.GetCtrlPointCount() - spl.GetSplineDegree();
		}				
	    for(i=0;i<size;i++)
		{ 
			pointN = 1.f/DELTA1 + 1;
			for(int kk = 0; kk < pointN; kk++)
			{
			    t = kk * DELTA1;
				if(i > 0 && t == 0)
				{
					continue;
				}
				p = spl.GetPoint( i , t );
				if(np>0)
				{
					if( (p-plist[np-1]).Magnitude()>ERR)
					{
						plist.push_back(p);
						np++;
					}
					else
					{
						if(!basserted)
						{
							assert(FALSE);
							basserted = true;
						}
					}
				}
				else
				{
					plist.push_back(p);
					np++;
				}
			}
		}
	}
	
	vtsOut.resize(np);
	for(i=0; i<np; i++)
	{
		vtsOut[i] = plist[i];
	}
}
void GetPolygonFromDiscretePoints(varray<Vec4>& DiscreteP, varray<Vec4>& leftV, varray<Vec4>& rightV, varray<Vec4> &polygonP)
{
	int i,j,k;
	int dPSize=static_cast<int>(DiscreteP.size());
	int *sortPid;
	varray<Vec4> sortP;
	sortPid=new int[dPSize];

	for(i=0;i<dPSize;i++)
	{
		sortPid[i]=-1;
	}

	sortPid[0]=0;
	for(i=1;i<dPSize;i++) //this wheel to sort the discrete points on P.x
	{
		bool breakFlag=false;
		for(j=0;j<dPSize;j++)
		{
			if(sortPid[j]==-1)
			{
				breakFlag=true;
				break;
			}
			if(DiscreteP.at(i).x<=DiscreteP.at(sortPid[j]).x)
			{
				if(DiscreteP.at(i).x==DiscreteP.at(sortPid[j]).x)
				{
					if(DiscreteP.at(i).y<DiscreteP.at(sortPid[j]).y)
					{
						for(k=dPSize;k>j;k--)
						{
							sortPid[k]=sortPid[k-1];
						}
						sortPid[j]=i;
						break;
					}
					else
					{
						for(k=dPSize;k>j+1;k--)
						{
							sortPid[k]=sortPid[k-1];
						}
						sortPid[j+1]=i;
						break;
					}
				}
				else
				{
					for(k=dPSize;k>j;k--)
					{
						sortPid[k]=sortPid[k-1];
					}
					sortPid[j]=i;
					break;
				}
			}
		}
		if(breakFlag)
		{
			sortPid[j]=i;
		}		
	}

	varray<Vec4> temp;
	temp=DiscreteP;
	DiscreteP.clear();
	for(i=0;i<dPSize;i++)
	{
		DiscreteP.push_back(temp.at(sortPid[i]));
	}

	//	delete sortPid;

	//the following codes to get a simple polygon
	//	Vec4 dividingLine=DiscreteP.at(dPSize-1)-DiscreteP.at(0);
	Vec4 headV=DiscreteP.at(0);
	Vec4 tailV=DiscreteP.at(dPSize-1);
	varray<int> leftPid;
	varray<int> rightPid;
	leftPid.push_back(0);
	for(i=1;i<dPSize;i++)
	{
		float z=CrossVecX(DiscreteP.at(i)-headV, DiscreteP.at(i)-tailV).z;
		if(z>0)
		{
			leftPid.push_back(i);
		}
		else if(z<0)
		{
			rightPid.push_back(i);
		}
	}
	leftPid.push_back(dPSize-1);

	for(i=0;i<static_cast<int>(leftPid.size());i++)
	{
		polygonP.push_back(DiscreteP.at(leftPid.at(i)));
		leftV.push_back(DiscreteP.at(leftPid.at(i)));
	}

	for(i=static_cast<int>(rightPid.size()-1);i>=0;i--)
	{
		polygonP.push_back(DiscreteP.at(rightPid.at(i)));		
		rightV.push_back(DiscreteP.at(rightPid.at(i)));
	}
}

//---------------------------------------------------------------
// Name:		GetConvexHullFormPolygon
// Description: get the min convex hull from the given polygon.
//              NOTE: the function do not used ...............................................
// Argument:	just the same meaning of function GetPolygonFromDiscretePoints();
// Return:		
// Author:		unknown
// Date:		1:9:2005
// Modified by:		
// Updated date:		
//---------------------------------------------------------------- 
void GetConvexHullFormPolygon(varray<Vec4>& leftV,varray<Vec4>& rightV,varray<Vec4> &convexHullP)
{
	int i,j;
	varray<bool> leftFlag;
	varray<bool> rightFlag;
	for(i=0;i<static_cast<int>(leftV.size());i++)
	{
		leftFlag.push_back(false);
	}
	for(i=0;i<static_cast<int>(rightV.size());i++)
	{
		rightFlag.push_back(false);
	}

	if(leftV.size()>2)
	{
		for(i=1;i<static_cast<int>(leftV.size()-1);i++)
		{
			float z=CrossVecX(leftV.at(i)-leftV.at(i-1), leftV.at(i)-leftV.at(i+1)).z;
			if(z<0)
			{
				leftFlag.at(i)=true;
				int startPid=i+1;
				for(j=startPid-2;j>0;j--)
				{
					if(leftFlag.at(j))    {continue;}
					float z1=CrossVecX(leftV.at(j)-leftV.at(j-1), leftV.at(j)-leftV.at(startPid)).z;
					if(z1<=0.0)
					{
						leftFlag.at(j)=true;
					}
					else
					{
						break;
					}
				}
			}
		}
	}
	if(rightV.size()>2)
	{
		for(i=1;i<static_cast<int>(rightV.size()-1);i++)
		{
			float z=CrossVecX(rightV.at(i)-rightV.at(i-1),rightV.at(i)-rightV.at(i+1)).z;
			if(z<0)
			{
				rightFlag.at(i)=true;
				int startPid=i+1;
				for(j=startPid-2;j>0;j--)
				{
					if(rightFlag.at(j))    {continue;}
					float z1=CrossVecX(rightV.at(j)-rightV.at(j-1),rightV.at(j)-rightV.at(startPid)).z;
					if(z1<=0.0)
					{
						rightFlag.at(j)=true;
					}
					else
					{
						break;
					}
				}
			}
		}
	}
	for(i=0;i<static_cast<int>(leftV.size());i++)
	{
		if(leftFlag.at(i))
		{
			continue;
		}
		convexHullP.push_back(leftV.at(i));
	}
	for(i=0;i<static_cast<int>(rightV.size());i++)
	{
		if(rightFlag.at(i))    {continue;}
		convexHullP.push_back(rightV.at(i));
	}

}
//String GetFileNameWithoutPostfix(String filepath)
//{
//	String ret;
//	char dist = '\\';
//	char dot = '.';
//	char step;
//	int st=-1,end=-1;
//
//	for(int i=filepath.len()-1;i>=0;i--)
//	{
//		step = filepath.at(i);
//		if(dist == step )
//		{
//			st = i;
//			break;
//		}
//		if(dot == step )
//		{
//			end = i;
//		}
//	}
//	//------add by XXX XXX to avoid error------------ 
//	if(st == -1 || end == -1)
//	{
//		return ret;
//	}
//	//--------
//	ret = filepath.right(filepath.len()-1-st);
//	ret = ret.left(end-st-1);
//	return ret;
//}

//----------------------------------------------------------------------
// name:		MapAVectorToAPlane
// function:	get the projection of point on a plane
//				i.e. rotate the vector pi/2 and -pi/2 around z axis 
// argument:	plane normal: pNorm
//				vector: vect
// return:		project point of v1
// author:		
// date:		
// update:	    
// author:		XXX
// date:		20060504
//----------------------------------------------------------------------
Vec4 MapAVectorToAPlane(Vec4 vect, Vec4 pNorm) 
{
	Vec4 vectDir = vect.Normalize();

	pNorm = pNorm.Normalize();
	float dot = Dot(vectDir,pNorm);
	float ang;
	if(fabs(dot-1.0) < 1.0e-4)
	{
		if(dot>=0.0)
		{
			ang=0.0;
			return vect;
		}
		else
		{
			ang=(float)PI;
			return -vect;
		}
	}
	else if(fabs(dot)<1.0e-4)
	{
		ang = (float)(PI*0.5);
	}
	else
		ang = acos(dot);


	Vec4 axis = CrossVecX(vectDir, pNorm).Normalize();

	ang = (float)(ang-PI/2.0);

	QMatrix4x4 matrix;
	//matrix.SetRotate(axis,ang);
	matrix.rotate(ang, axis.x, axis.y, axis.z);
	QVector3D temp;
	temp.setX(vect.x);
	temp.setY(vect.y);
	temp.setZ(vect.z);
	Vec4 res;
	res.x = (temp*matrix).x();
	res.y = (temp*matrix).y();
	res.z = (temp*matrix).z();
	return res;
}

//----------------------------------------------------------------------
// name:		MapAVertexToA2DSegment
// function:	get the projection of point on a 2D segment
// argument:	segstartpt,segendpt: end and start point of segment
//				vertex: vertex
// return:		project point of v1
// author:		Li
// date:		2003/11/06
// update:	    
// author:		XXX
// date:		20060504
//----------------------------------------------------------------------
Vec2 MapAVertexToA2DSegment(const Vec2& vertex,  const Vec2& segstartpt, const Vec2& segendpt)
{
	Vec4 v1(vertex.x,vertex.y,0.0);
	Vec4 v2(segstartpt.x, segstartpt.y,0.);
	Vec4 v3(segendpt.x, segendpt.y,0.);
	Vec4 res= MapAVertexToASegment(v1,v2,v3);
	return Vec2(res.x, res.y);
}

void StoreInSameClockwise(const varray<Vec2>&prePoly,varray<Vec2>&adjustPoly, bool& wiseChanged)
{
	wiseChanged = false;

	float	maxX = -1.0e8;
	int		size = static_cast<int>(prePoly.size());
	int		i, res = 0;
	// find the max x point
	for(i=0; i<size; i++)			
	{
		const Vec2& v = prePoly.at(i);
		if(maxX < v.x)
		{
			maxX = v.x;
			res = i;
		}		
	}
	Vec2 v0 = prePoly.at((res-1+size)%size);
	Vec2 v1 = prePoly.at(res);
	Vec2 v2 = prePoly.at((res+1)%size);
	Vec4 cross = CrossVecX(Vec4((v2-v1).x, (v2-v1).y, 0.), Vec4((v0-v1).x, (v0-v1).y, 0.));
	bool flag1 = cross.z > 0;

	maxX = -1.0e8;
	size = static_cast<int>(adjustPoly.size());
	res  = 0;
	// find the max x point for adjustPoly
	for(i=0;i<size;i++)			
	{
		const Vec2& v=adjustPoly.at(i);
		if(maxX < v.x)
		{
			maxX = v.x;
			res  = i;
		}		
	}
	v0=adjustPoly.at((res-1+size)%size);
	v1=adjustPoly.at(res);
	v2=adjustPoly.at((res+1)%size);
	cross=CrossVecX(Vec4((v2-v1).x,(v2-v1).y,0.),Vec4((v0-v1).x,(v0-v1).y,0.));
	bool flag2=cross.z>0;

	if(flag1*flag2 > 0)
	{  //already has same order
		return;
	}

	wiseChanged = true;
	Vec2 temp;
	for(i=0;i<static_cast<int>(adjustPoly.size()/2);i++)
	{
		temp=adjustPoly.at(i);
		adjustPoly.at(i)=adjustPoly.at(adjustPoly.size()-1-i);
		adjustPoly.at(adjustPoly.size()-1-i)=temp;
	}
}

//---------------------------------------------------------------
// Name:		IsPointStoreInClockWise
// Description: the point's order of 2d ploygon is clock wise or not
// Argument:	
// Return:		inputPoly:    input polygon        
// Author:		false if in clock wise
// Date:		1:9:2005
// Modified by:	XXX 	
// Updated date:20060507		
//---------------------------------------------------------------- 
bool IsPointStoreInClockWise(const varray<Vec2>&inputPoly)
{
	float maxX=-1.0e8;

	int size=static_cast<int>(inputPoly.size());
	int i,res=0;
	for(i=0;i<size;i++) // find the max x coordinates point.
	{
		const Vec2& v=inputPoly.at(i);
		if(maxX<v.x)
		{
			maxX=v.x;		
			res=i;
		}
	}

	Vec2 v1, v2,v3;
	v1 = inputPoly.at(res);
	for(i=res;;i++)  
	{// find the point near v1 (increase order)
		if(res == static_cast<int>((i+1)%inputPoly.size()) )
		{			
			break;
		}
		v2 = inputPoly.at((i+1)%inputPoly.size());
		if((v2-v1).Magnitude()>1.0e-3)
		{
			break;
		}
	}

	for(i=res;;i--)  
	{// find the point near v1  (decrease order)
		if(res==static_cast<int>( (i-1+ inputPoly.size())%inputPoly.size()) )
		{			
			break;
		}
		v3=inputPoly.at((i-1+inputPoly.size())%inputPoly.size());
		if((v3-v1).Magnitude()>1.0e-3)
		{		
			break;
		}
	}

	Vec4 dir1((v2-v1).x, (v2-v1).y, 0.);
	Vec4 dir2((v3-v1).x, (v3-v1).y, 0.);

	float cross = CrossVecX(dir1,dir2).z;
	bool flag = cross > 0;
	return !flag;
}
//---------------------------------------------------------------
// Name:		StorePointOnPolygonInClockwise
// Description: set point's order to anti clockwise
// Argument:	
// Return:		inputPoly:    input polygon        
// Author:		true if in clock wise
// Date:		1:9:2005
// Modified by:	XXX 	
// Updated date:20060507		
//----------------------------------------------------------------
void StorePointOnPolygonInClockwise(varray<Vec2>&inputPoly)
{
	float maxX=-1.0e8;

	int size=static_cast<int>(inputPoly.size());
	int i,res=0;
	for(i=0;i<size;i++)
	{
		const Vec2& v=inputPoly.at(i);
		if(maxX<v.x)
		{
			maxX=v.x;		
			res=i;
		}
	}

	Vec2 v1, v2,v3;
	v1=inputPoly.at(res);
	for(i=res;;i++)
	{
		if(res==static_cast<int>( (i+1)%inputPoly.size()) )
		{			
			break;
		}
		v2=inputPoly.at((i+1)%inputPoly.size());
		if((v2-v1).Magnitude()>1.0e-3)
		{
			break;
		}
	}
	for(i=res;;i--)
	{
		if(res==static_cast<int>( (i-1+inputPoly.size())%inputPoly.size()) )
		{			
			break;
		}
		v3=inputPoly.at((i-1+inputPoly.size())%inputPoly.size());
		if((v3-v1).Magnitude()>1.0e-3)
		{		
			break;
		}
	}

	Vec4 dir1((v2-v1).x, (v2-v1).y, 0.);
	Vec4 dir2((v3-v1).x, (v3-v1).y, 0.);

	float cross = CrossVecX(dir1,dir2).z;
	bool flag   = cross > 0;

	Vec2 temp;
	if(flag)
	{   //clock wise
		for(i=0;i<size/2;i++)
		{
			temp=inputPoly.at(i);
			inputPoly.at(i)=inputPoly.at(inputPoly.size()-1-i);
			inputPoly.at(inputPoly.size()-1-i)=temp;
		}
	}

}

//---------------------------------------------------------------
// Name:	    SortVts()
// Description: Sort the vertices increase or decrease in x(y,z) direction
// Argument:	iDir: vertexes sort by which(x,y,z)direction;
//         :	iMode:0--increase, 1--decrease 
// Return:		
// Author:	    XXX XXX
// Date:	    2006/04/29 29:4:2006   15:13
// Update:	 
// Author:	
// Date: 
// copyright: XXX. developed by XXX
//----------------------------------------------------------------
void SortVts(varray<Vec4>& vtsArr,int iDir,int iMode)
{
	int i = 0, j= 0;
	Vec4 vtmp;
	if(iDir == 0)//x-direction
	{
		if(iMode==0)
		{
			for(i=0; i<static_cast<int>(vtsArr.size()-1); i++)
			{
				for(j=i+1; j<static_cast<int>(vtsArr.size()); j++)
				{
					if(vtsArr[i].x>vtsArr[j].x)
					{
						vtmp = vtsArr[i];
						vtsArr[i] = vtsArr[j];
						vtsArr[j] = vtmp;
					}
				}
			}
		}
		else
		{
			for(i=0; i<static_cast<int>(vtsArr.size()-1); i++)
			{
				for(j=i+1; j<static_cast<int>(vtsArr.size()); j++)
				{
					if(vtsArr[i].x<vtsArr[j].x)
					{
						vtmp = vtsArr[i];
						vtsArr[i] = vtsArr[j];
						vtsArr[j] = vtmp;
					}
				}
			}
		}
	}
	else if(iDir == 1)//y-direction
	{
		if(iMode==0)
		{
			for(i=0; i<static_cast<int>(vtsArr.size()-1); i++)
			{
				for(j=i+1; j<static_cast<int>(vtsArr.size()); j++)
				{
					if(vtsArr[i].y>vtsArr[j].y)
					{
						vtmp = vtsArr[i];
						vtsArr[i] = vtsArr[j];
						vtsArr[j] = vtmp;
					}
				}
			}
		}
		else
		{
			for(i=0; i<static_cast<int>(vtsArr.size()-1); i++)
			{
				for(j=i+1; j<static_cast<int>(vtsArr.size()); j++)
				{
					if(vtsArr[i].y<vtsArr[j].y)
					{
						vtmp = vtsArr[i];
						vtsArr[i] = vtsArr[j];
						vtsArr[j] = vtmp;
					}
				}
			}
		}
	}
	else if(iDir == 2)//z-direction
	{
		if(iMode==0)
		{
			for(i=0; i<static_cast<int>(vtsArr.size()-1); i++)
			{
				for(j=i+1; j<static_cast<int>(vtsArr.size()); j++)
				{
					if(vtsArr[i].z>vtsArr[j].z)
					{
						vtmp = vtsArr[i];
						vtsArr[i] = vtsArr[j];
						vtsArr[j] = vtmp;
					}
				}
			}
		}
		else
		{
			for(i=0; i<static_cast<int>(vtsArr.size()-1); i++)
			{
				for(j=i+1; j<static_cast<int>(vtsArr.size()); j++)
				{
					if(vtsArr[i].z<vtsArr[j].z)
					{
						vtmp = vtsArr[i];
						vtsArr[i] = vtsArr[j];
						vtsArr[j] = vtmp;
					}
				}
			}
		}
	}
}
//end of point order operator
/////////////////////////////////////////////////////////////////////////////////


//////////////////////////////////////////////////////////////////////////
//collision operator
//---------------------------------------------------------------
// Name:		CollisionDetectonBetweenTwoPolygonWithMultiCenterPts
// Description:	Collision Detection Between Two Polygon 
//				current this function just for garment and body cross section collision.
// Argument:	vtsRef, vtsTar: two Polygons
//				fMargin:		margin
//				imode:			view id
// Return:		bool
// Author:		chl
// Date:		9/7/2005
// Modified by:	XXX	
// Updated date:20060507		
//---------------------------------------------------------------- 
bool  CollisionDetectonBetweenTwoPolygonWithMultiCenterPts(varray<Vec4>& vtsRef, varray<Vec4>& vtsTar, float fMargin, int imode)
{
	if(vtsRef.size() < 3 || vtsTar.size() < 3 )
		return false;

	int i;
	varray<Vec2> refArr,tarArr;
	//input the 2d point data.
	if(imode == 0)
	{	//y-z 
		for(i=0;i < static_cast<int>(vtsRef.size()); i++)
			refArr.push_back(Vec2(vtsRef.at(i).y,vtsRef.at(i).z));
		for(i=0;i < static_cast<int>(vtsTar.size()); i++)
			tarArr.push_back(Vec2(vtsTar.at(i).y,vtsTar.at(i).z));
	}
	else if(imode == 1)
	{   //x-z
		for(i=0;i < static_cast<int>(vtsRef.size()); i++)
			refArr.push_back(Vec2(vtsRef.at(i).x,vtsRef.at(i).z));
		for(i=0;i < static_cast<int>(vtsTar.size()); i++)
			tarArr.push_back(Vec2(vtsTar.at(i).x,vtsTar.at(i).z));
	}
	else if(imode == 2)
	{  //x-y
		for(i=0;i < static_cast<int>(vtsRef.size()); i++)
			refArr.push_back(Vec2(vtsRef.at(i).x,vtsRef.at(i).y));
		for(i=0;i < static_cast<int>(vtsTar.size()); i++)
			tarArr.push_back(Vec2(vtsTar.at(i).x,vtsTar.at(i).y));
	}
	//find center points for the reference polygon
	//currently just for x_co.
	float xmax,ymax,xmin,ymin;
	xmax = ymax = -1.0e6;
	xmin = ymin = 1.0e6;
	for(i=0;i<static_cast<int>(refArr.size());i++)
	{
		if(refArr.at(i).x > xmax)
			xmax = refArr.at(i).x;
		if(refArr.at(i).x < xmin)
			xmin = refArr.at(i).x;
		if(refArr.at(i).y > ymax)
			ymax = refArr.at(i).y;
		if(refArr.at(i).y < ymin)
			ymin = refArr.at(i).y;
	}

	bool	isMultiCenter;
	Vec2	ptcenL, ptcenR, ptcen;
	int		ptnumL, ptnumR;
	ptnumR = ptnumL = 0;
	ptcenL.x = ptcenL.y = ptcenR.x = ptcenR.y = ptcen.x = ptcen.y = 0.0;
	ptcen.x  = (xmax+xmin)/2;
	ptcen.y  = (ymax+ymin)/2;

	for(i=0; i < static_cast<int>(refArr.size()); i++)
	{
		if(refArr.at(i).x <= ptcen.x)
		{
			ptcenL += refArr.at(i);
			ptnumL++;
		}
		else 
		{
			ptcenR += refArr.at(i);
			ptnumR++;
		}
	}
	ptcenL = ptcenL/(float)ptnumL;
	ptcenR = ptcenR/(float)ptnumR;

	if(ptcenR.x - ptcenL.x > (xmax - xmin)*0.5) 
		isMultiCenter = true;
	else
		isMultiCenter = false;

	//collision for the polygon
	Vec2 ptBase;
	Vec4 rep;
	for(i=0; i < static_cast<int>(tarArr.size()); i++)
	{
		Vec2&  vt = tarArr.at(i);
		ptBase = ptcen;			//default center
		if(vt.x > ptcenR.x && isMultiCenter)
		{
			ptBase = ptcenR;
		}
		else if(vt.x < ptcenL.x && isMultiCenter)
		{
			ptBase = ptcenL;
		}
		rep = Vec4(vt.x,vt.y,0.f);
		GetInterSectPntOfLnPolygon(refArr, rep, fMargin, 0, Vec4(ptBase.x,ptBase.y,0.f));
		vt.x = rep.x;	vt.y = rep.y;
	}
	if(imode==0)
	{
		for(i=0;i<static_cast<int>(tarArr.size());i++)
		{
			vtsTar.at(i).x = tarArr.at(i).x;
			vtsTar.at(i).y = tarArr.at(i).y;	 
		}
	}
	else if(imode==1)
	{
		for(i=0;i<static_cast<int>(tarArr.size());i++)
		{
			vtsTar.at(i).x = tarArr.at(i).x;
			vtsTar.at(i).z = tarArr.at(i).y;	 
		}
	}
	else if(imode == 2)
	{
		for(i=0;i<static_cast<int>(tarArr.size());i++)
		{
			vtsTar.at(i).y = tarArr.at(i).x;
			vtsTar.at(i).z = tarArr.at(i).y;	 
		}
	}
	return true;
}


//---------------------------------------------------------------
// Name:		ShapeAdjustInCollision
// Description:	Merge closedCurv1 and closedCurv2 to get their out-boundary
//				if poly1 include poly2, return poly2; else return intersection of two polygon
//				This function need to improve, please use it carefully. Or Do not use
//				Note: NOT USEED.!!!!!!
// Argument:	closedCurv1, closedCurv2 are two closed curves,
//				preClosedCurv1: ???
//				adjustedShape: vertices on the out-boundary.
// Return:		Success return true, otherwise return false
// Author:		ljt
// Date:		9/7/2005
// Modified by:		
// Updated date:		
//---------------------------------------------------------------- 
bool ShapeAdjustInCollision(const varray<Vec2>&closedCurv1, const varray<Vec2>&closedCurv2,const varray<Vec2>&preClosedCurv1,varray<Vec2>&adjustedShape)
{
	adjustedShape.clear();

	int i, j;//, size;

	varray<Vec2> polygon1;
	varray<Vec2> polygon2;

	polygon1.resize(closedCurv1.size());
	polygon2.resize(closedCurv2.size());

	for(i=0;i<static_cast<int>(closedCurv1.size());i++)
	{
		polygon1[i]=closedCurv1.at(i);
	}
	for(i=0;i<static_cast<int>(closedCurv2.size());i++)
	{
		polygon2[i]=closedCurv2.at(i);
	}

	//change point order to clock wise
	Vec2 temp;
	if(!IsPointStoreInClockWise(preClosedCurv1)) //preClosedCurv1 ????
	{		
		for(i=0;i<static_cast<int>(polygon1.size()/2);i++)
		{
			temp=polygon1.at(i);
			polygon1.at(i)=polygon1.at(polygon1.size()-1-i);
			polygon1.at(polygon1.size()-1-i)=temp;
		}
	}
	if(!IsPointStoreInClockWise(polygon2))
	{
		for(i=0;i<static_cast<int>(polygon2.size()/2);i++)
		{
			temp=polygon2.at(i);
			polygon2.at(i)=polygon2.at(polygon2.size()-1-i);
			polygon2.at(polygon2.size()-1-i)=temp;
		}
	}

	varray<bool> insideFlag;	
	insideFlag.resize(polygon1.size(),false);

	int insideSize=0;
	for(i=0;i<static_cast<int>(polygon1.size());i++)	
	{
		if(IsPointInConexPolygon(polygon1.at(i),polygon2))//,cent))
		{
			insideFlag.at(i)=true;			
			insideSize++;
		}
	}

	if(insideSize== static_cast<int>(polygon1.size()) )		
	{//if the whole poygon1 is inside polygon2
		adjustedShape.resize(closedCurv2.size());
		for(i=0;i<static_cast<int>(closedCurv2.size());i++)
		{
			adjustedShape.at(i)=closedCurv2.at(i);
		}		
		return true;
	}
	if(insideSize==0)				
	{//if the whole poygon2 is inside polygon1
		adjustedShape.clear();
		adjustedShape=closedCurv1;
		return true;
	}

	//compute intersection between polygon1 and polygon2
	varray<int> collidedFlagInCurve1;
	varray<int> collidedFlagInCurve2;
	varray<Vec2> intersections;
	bool flag1=false, flag2=false,flag=false;
	Vec2 v1, v2, v3, v4;
	Vec2 inter;
	float tempLen;

	for(i=0;i<static_cast<int>(polygon1.size());i++)	
	{
		flag1 = insideFlag.at(i);
		flag2 = insideFlag.at((i+1)%polygon1.size());

		flag = (flag1 && !flag2) || (!flag1 && flag2);

		if(!flag)
			continue;

		v1 = polygon1.at(i);
		v2 = polygon1.at((i+1)%polygon1.size());

		bool findFlag = false;
		for(j=0;j<static_cast<int>(polygon2.size());j++)	
		{   //compute intersection between v1v2 and polygon2
			v3=polygon2.at(j);
			v4=polygon2.at((j+1)%polygon2.size());

			//if(IsSegmentsIntersect(v1,v2,v3,v4))
			if(IsTwoLineIntersectIn2d(Vec4(v1.x,v1.y,0.0), Vec4(v2.x,v2.y,0.0), Vec4(v3.x,v3.y,0.),Vec4(v4.x,v4.y,0.)))
			{
				Vec2 inter = GetTwoLineInterPt2D(v1,v2,v3,v4);

				//intersect point is out of line?
				if(Dot(inter-v1,v2-v1)<0)
					continue;
				if(Dot(inter-v3,v4-v3)<0)
					continue;

				tempLen = fabs((inter-v1).Magnitude()+(inter-v2).Magnitude()-(v1-v2).Magnitude());
				if(tempLen>1.0e-4)
					continue;

				tempLen = fabs((inter-v3).Magnitude()+(inter-v4).Magnitude()-(v3-v4).Magnitude());
				if( tempLen>1.0e-4)
					continue;

				collidedFlagInCurve1.push_back(i);
				collidedFlagInCurve2.push_back(j);
				intersections.push_back(inter);
				findFlag=true;				
				break;
			}
		}
		if(!findFlag)
		{
			Vec2  inter;
			//Vec2  refV = (v1+v2)/2.0;
			//float len;
			//int resId;

			for(j=0;j<static_cast<int>(polygon2.size());j++)
			{
				v3=polygon2.at(j);
				v4=polygon2.at((j+1)%polygon2.size());				
				inter = GetTwoLineInterPt2D(v1,v2,v3,v4);

				if(Dot(inter-v1,v2-v1)<0)
					continue;
				if(Dot(inter-v3,v4-v3)<0)
					continue;

				tempLen= fabs((inter-v1).Magnitude()+(inter-v2).Magnitude()-(v1-v2).Magnitude());
				if( tempLen>1.0e-4)
					continue;

				tempLen=fabs((inter-v3).Magnitude()+(inter-v4).Magnitude()-(v3-v4).Magnitude());
				if( tempLen>1.0e-4)
					continue;

				collidedFlagInCurve1.push_back(i);
				collidedFlagInCurve2.push_back(j);
				intersections.push_back(inter);
			}
		}

		if(!findFlag)
		{
			return false;
		}
	}


	if(collidedFlagInCurve1.size()<2)
		return false;

	if(collidedFlagInCurve1.size()%2 == 1)//???
	{
		return false;
	}

	//following merge polygon1 and polygon2 to get the out-boudary vertices
	int id1,id2;
	int ord3,ord4;

	if(!insideFlag.front())
	{
		for(i=0;i<collidedFlagInCurve1.front();i++)
		{
			adjustedShape.push_back(polygon1.at(i));
		}
	}

	for(i=0;i<static_cast<int>(collidedFlagInCurve1.size()-1);i++)
	{
		id1=collidedFlagInCurve1.at(i);
		if(insideFlag.at(id1)) 
		{	
			adjustedShape.push_back(intersections.at(i));
			if(id1+1<static_cast<int>(polygon1.size()))
			{
				if(i== static_cast<int>( collidedFlagInCurve1.size()-1) )
				{
					for(j=id1+1;j<static_cast<int>(polygon1.size());j++)
					{
						adjustedShape.push_back(polygon1.at(j));
					}
				}
				else
				{
					for(j=id1+1;j<collidedFlagInCurve1.at(i+1);j++)
					{
						adjustedShape.push_back(polygon1.at(j));
					}
				}

			}
			continue;
		}			
		//come into the section on torso
		adjustedShape.push_back(polygon1.at(collidedFlagInCurve1.at(i)));
		adjustedShape.push_back(intersections.at(i));

		id2=collidedFlagInCurve1.at(i+1);
		ord3=collidedFlagInCurve2.at(i);
		ord4=collidedFlagInCurve2.at(i+1);
		if(ord4>=ord3+1)
		{
			if(ord4>ord3+1)
			{
				for(j=ord3+1;j<ord4;j++)
				{
					adjustedShape.push_back(polygon2.at(j));
				}
			}
		}
		else
		{
			if(ord4==ord3)
			{
				;
			}
			else if(static_cast<int>(polygon2.size())>ord3+1)
			{
				for(j=ord3+1;j<static_cast<int>(polygon2.size());j++)
				{
					adjustedShape.push_back(polygon2.at(j));
				}
				for(j=0;j<ord4;j++)
				{
					adjustedShape.push_back(polygon2.at(j));
				}
			}			
		}		

		adjustedShape.push_back(intersections.at(i+1));
		if(id2+1<static_cast<int>(polygon1.size()))
		{
			if(i+1>=static_cast<int>(collidedFlagInCurve1.size()-1))
			{		
				for(j=id2+1;j<static_cast<int>(polygon1.size());j++)
				{
					adjustedShape.push_back(polygon1.at(j));
				}			
			}
		}
	}

	id1=collidedFlagInCurve1.back();
	if(!insideFlag.at(id1))
	{
		adjustedShape.push_back(polygon1.at(id1));
		adjustedShape.push_back(intersections.back());

		ord3=collidedFlagInCurve2.back();		
		ord4=collidedFlagInCurve2.front();
		if(ord4>=ord3+1)
		{
			if(ord4>ord3)
			{
				for(j=ord3+1;j<ord4;j++)
				{
					adjustedShape.push_back(polygon2.at(j));
				}
			}
		}
		else
		{
			if(static_cast<int>(polygon2.size())>ord3+1)
			{				
				for(j=ord3+1;j<static_cast<int>(polygon2.size());j++)
				{
					adjustedShape.push_back(polygon2.at(j));
				}
				for(j=0;j<ord4;j++)
				{
					adjustedShape.push_back(polygon2.at(j));
				}
				adjustedShape.push_back(intersections.at(0));
			}
		}
	}

	bool wiseChanged=false;
	StoreInSameClockwise(closedCurv1,adjustedShape, wiseChanged);

	Vec2 tempV1, tempV2;
	float threshold=(float)1.0e-4;
	for(i=0;i<static_cast<int>(adjustedShape.size());i++)
	{
		tempV1=adjustedShape.at(i);
		tempV2=adjustedShape.at((i+1)%adjustedShape.size());
		if((tempV1-tempV2).Magnitude()<threshold)
		{
			adjustedShape.erase(&adjustedShape.at(i));
			i--;
		}
	}

	return true;
}


//---------------------------------------------------------------
// Name:		CollisionAdjustBetween2Polygons
// Description:	intersect between 2 ploygons
// Argument:	closedCurv1, closedCurv2 are two closed curves,
//				preClosedCurv1: the previous shape of curv1
//				cent: center point
//				offset: offset
//				adjustedShape: vertices on the out-boundary.
// Return:		Success return true, otherwise return false
// Author:		ljt
// Date:		2004/09/14
// Modified by:		
// Updated date:		
//---------------------------------------------------------------- 
bool CollisionAdjustBetween2Polygons(const varray<Vec2>&closedCurv1, const varray<Vec2>&closedCurv2, const varray<Vec2>&preClosedCurv1,Vec2 cent, float offset,varray<Vec2>&adjustCurv1)
{	
	varray<Vec2> adjustedShape;
	int i;

	bool successFlag=true;
	if(!ShapeAdjustInCollision(closedCurv1, closedCurv2, preClosedCurv1,adjustedShape))
	{	
		successFlag=false;
	}
	else
	{
		if(adjustedShape.size()/closedCurv1.size()>1.5)
			successFlag=false;
	}
	if(!successFlag)
	{
		//to make edges on each section with a nearly uniform length
		adjustedShape.clear();
		adjustedShape.resize(closedCurv1.size());
		for(i=0;i<static_cast<int>(closedCurv1.size());i++)
		{
			adjustedShape[i]=closedCurv1.at(i);
		}
	}

	int		sel1=0, sel2=0;
	int		preSel1=0, preSel2=0;
	float	dot;
	Vec2	axisy(0.0,-1.0);
	float	max1=-1.0e6, max2=-1.0e6;
	//get point located at Y and -Y direction
	for(i=0;i<static_cast<int>(preClosedCurv1.size());i++)
	{
		Vec2 dir(preClosedCurv1.at(i) - cent);
		dir = dir.Normalize();
		dot = Dot(dir,axisy);
		if(dot > max1)
		{
			max1 = dot;
			preSel1 = i;
		}
		if(-dot>max2)
		{
			max2=-dot;
			preSel2=i;
		}
	}

	max1=-1.0e6, max2=-1.0e6;
	for(i=0;i<static_cast<int>(adjustedShape.size());i++)
	{
		Vec2 dir(adjustedShape.at(i)-cent);
		dir=dir.Normalize();
		dot=Dot(dir,axisy);
		if(dot>max1)
		{
			max1=dot;
			sel1=i;
		}
		if(-dot>max2)
		{
			max2=-dot;
			sel2=i;
		}
	}

	int divid=0;
	varray<Vec2> adjustedVs;
	if(sel2 < sel1)
	{
		for(i=sel2;i<=sel1;i++)
		{
			adjustedVs.push_back(adjustedShape.at(i));
		}
		divid = static_cast<int>(adjustedVs.size()-1);
		if(sel1+1 < static_cast<int>(adjustedShape.size()))
		{
			for(i=sel1+1;i<static_cast<int>(adjustedShape.size());i++)
			{
				adjustedVs.push_back(adjustedShape.at(i));
			}
		}
		for(i=0;i<sel2;i++)
		{
			adjustedVs.push_back(adjustedShape.at(i));
		}		
	}
	else
	{
		for(i=sel2;i<static_cast<int>(adjustedShape.size());i++)
		{
			adjustedVs.push_back(adjustedShape.at(i));
		}
		for(i=0;i<=sel1;i++)
		{
			adjustedVs.push_back(adjustedShape.at(i));
		}
		divid=static_cast<int>(adjustedVs.size()-1);
		if(sel1+1<sel2)
		{
			for(i=sel1+1;i<sel2;i++)
			{
				adjustedVs.push_back(adjustedShape.at(i));
			}
		}		
	}

	float partSumLen1=0.0, partSumLen2=0.;
	float averLen1, averLen2;
	for(i=1;i<static_cast<int>(adjustedVs.size());i++)
	{
		if(i <= divid)
		{
			partSumLen1+=(adjustedVs.at(i)-adjustedVs.at(i-1)).Magnitude();
		}
		else
		{
			partSumLen2+=(adjustedVs.at(i)-adjustedVs.at(i-1)).Magnitude();
		}
	}
	partSumLen2+=(adjustedVs.back()-adjustedVs.at(adjustedVs.size()-2)).Magnitude();
	averLen1 = partSumLen1/20;
	averLen2 = partSumLen2/20;

	varray<Vec2>adjustedVs2;
	adjustedVs2.resize(40);
	adjustedVs2[0]=adjustedVs.front();
	adjustedVs2[20]=adjustedVs.at(divid);

	int k=1;
	float tempSumLen1=0., tempLen;
	for(i=1;i<=divid;i++)
	{
		tempLen=(adjustedVs.at(i)-adjustedVs.at(i-1)).Magnitude();
		tempSumLen1+=tempLen;
		if(tempSumLen1>k*averLen1)
		{
			adjustedVs2[k]=adjustedVs.at(i)-(tempSumLen1-k*averLen1)*(adjustedVs.at(i)-adjustedVs.at(i-1)).Normalize();
			k++;
			i--;
			tempSumLen1-=tempLen;
			if(k==20)
			{
				break;
			}
		}
	}

	if(k!=20)
	{
		return false;		
	}

	float tempSumLen2=0.;
	k=1;

	for(i=divid+1;i<=static_cast<int>(adjustedVs.size());i++)
	{
		tempLen=(adjustedVs.at(i%adjustedVs.size())-adjustedVs.at(i-1)).Magnitude();
		tempSumLen2+=tempLen;
		if(tempSumLen2>k*averLen2)
		{
			adjustedVs2[k+20]=adjustedVs.at(i%adjustedVs.size())-(tempSumLen2-k*averLen2)*(adjustedVs.at(i%adjustedVs.size())-adjustedVs.at(i-1)).Normalize();
			k++;
			i--;
			tempSumLen2-=tempLen;
			if(k==20)
			{
				break;
			}
		}
	}
	if(k!=20)
	{
		return false;	
	}
	adjustedVs2[0]=adjustedVs.front();
	adjustedVs2[20]=adjustedVs.at(divid);


	adjustCurv1.clear();
	for(i=30;i<40;i++)
	{
		adjustCurv1.push_back(adjustedVs2[i]);
	}
	for(i=0;i<30;i++)
	{
		adjustCurv1.push_back(adjustedVs2[i]);
	}

	///////////////////////////////////
	Vec2 tempV1, tempV2, tempV3;
	Vec2 dir;
	bool insideFlag=false;
	float largeLen=1.0e6;

	int j;
	for(i=0;i<static_cast<int>(adjustCurv1.size());i++)
	{
		Vec2& v=adjustCurv1.at(i);
		if(!IsPointInConexPolygon(v,closedCurv2))		
			continue;

		insideFlag=true;

		tempV1=adjustCurv1.at((i+1)%adjustCurv1.size());
		tempV2=adjustCurv1.at((i-1+adjustCurv1.size())%adjustCurv1.size());

		if((tempV2-v).Magnitude()<1.0e-6 || (tempV1-v).Magnitude()<1.0e-6)
		{
			dir=v-cent;
		}
		else
		{
			dir=((tempV2-v).Normalize()+(tempV1-v).Normalize())/2.;
			if(dir.Magnitude()<1.0e-6)
			{
				dir=v-cent;
			}
		}

		if(Dot(v-cent,dir)<0)
		{
			dir=-dir;
		}
		if(dir.Magnitude()<1.0e-8)
		{
			continue;
		}
		dir=dir.Normalize();

		tempV3=v+largeLen*dir;
		for(j=0;j<static_cast<int>(closedCurv2.size());j++)
		{
			tempV1=closedCurv2.at(j);
			tempV2=closedCurv2.at((j+1)%closedCurv2.size());
			if(IsSegmentsIntersect(v,tempV3,tempV1,tempV2))
			{
				Vec4 v1(v.x,v.y,0.);
				Vec4 v2(tempV3.x,tempV3.y,0.);
				Vec4 v3(tempV1.x,tempV1.y,0.);
				Vec4 v4(tempV2.x,tempV2.y,0.);
				Vec4 inter=GetTwoLineInterPt(v1,v2,v3,v4);
				v=Vec2(inter.x,inter.y);
				break;
			}
		}
	}

	if(insideFlag)
	{
		for(i=0;i<static_cast<int>(adjustCurv1.size());i++) //make a margin
		{
			Vec2& v=adjustCurv1.at(i);
			tempV1=adjustCurv1.at((i+1)%adjustCurv1.size());
			tempV2=adjustCurv1.at((i-1+adjustCurv1.size())%adjustCurv1.size());
			dir=((tempV2-v).Normalize()+(tempV1-v).Normalize())/2.;
			if(Dot(v,dir)<0)
			{
				dir=-dir;
			}
			dir=dir.Normalize();
			v=v+dir*offset/2.;
		}
	}	
	return true;
}

//---------------------------------------------------------------
// Name:		CollisionAdjust
// Description:	adjust curve1 and make it not intersect with curve2
// Argument:	curv1,curv2:		two curves used to detected whether they intersect with each other,
//									if intersecting happens adjust curv1 to get adjustCurv1, 
//									and make them no intersection between adjustCurv1 and curv2 
//				preClosedCurv1:		the previous shape of curv1
//				cent:				center of curv2
//				offset:				easing space between two curves
//				adjustedShape:		vertices on the out-boundary.
//				closedCurvFlag1, closedCurvFlag2: flag to indicate whether curv1 and curv2 are closed
// Return:		is collision happens return true else false;
// Author:		ljt
// Date:		2004/09/14
// Modified by:		
// Updated date:		
//---------------------------------------------------------------- 
bool CollisionAdjust(const varray<Vec2>&curv1, const varray<Vec2>&curv2,const varray<Vec2>&preCurv1, Vec2 cent,bool closedCurvFlag1,bool closedCurvFlag2, float offset,varray<Vec2>&adjustCurv1)
{
	int i, j,k;
	int size1=static_cast<int>(curv1.size());
	int size2=static_cast<int>(curv2.size());

	adjustCurv1.resize(size1);
	for(i=0;i<size1;i++)
	{
		adjustCurv1.at(i)=curv1.at(i);
	}

	if(size1<2 || size2<2)
		return false;

	Vec4 bary;

	bool resFlag=false;
	varray<bool> collidedFlag;
	collidedFlag.resize(curv1.size(),false);

	int sum1=0;
	for(i=0;i<static_cast<int>(curv1.size());i++)
	{
		if(IsPointInConexPolygon(curv1.at(i),curv2))//,cent))
		{
			collidedFlag.at(i)=true;
			resFlag=true;
			sum1++;
		}
	}

	Vec2 currP, preP, sP, eP;
	varray<Vec2> intersections;
	float len, len1,len2;

	Vec2 dir1, dir2;
	float dot;

	float threshold = Math::Cos(15);

	for(i=0;i<static_cast<int>(collidedFlag.size());i++)
	{
		if(!collidedFlag.at(i))
			continue;

		currP = curv1.at(i);
		preP  = preCurv1.at(i);
		len = (currP-preP).Magnitude();

		sP = currP;
		eP = preP;

		dir1= (eP-sP).Normalize();
		dir2= (0.5*(eP+sP) - cent).Normalize();

		dot=Dot(dir1,dir2);
		if(fabs(dot) < threshold) //30degree
		{
			sP=eP;
			eP=cent;
		}

		if(GetIntersectPntOf2DLnCurve(currP,preP,curv2, true,intersections))
		{
			float minLen=1.0e8;
			int sel=-1;
			for(k=0;k<static_cast<int>(intersections.size());k++)
			{
				len1=(currP-intersections.at(k)).Magnitude();
				len2=(preP-intersections.at(k)).Magnitude();
				if(fabs((len1+len2)-len)<1.0e-4) 
				{//the section in on the segment prePcurP
					sel=k;
					break;
				}
				else
				{
					if(minLen>len2)
					{
						len2=minLen;
						sel=k;
					}
				}
			}
			adjustCurv1.at(i)=intersections.at(sel)+offset*(intersections.at(sel)-currP).Normalize();
		}
	}

	if(!closedCurvFlag1)
	{
		size1 = size1-1;
	}
	if(!closedCurvFlag2)
	{
		size2 = size2-1;
	}

	Vec2 outP, dis;
	Vec4 foot;
	for(i=0;i<size1; i++)
	{
		for(j=0;j<size2;j++)
		{
			if(!IsSegmentsIntersect(adjustCurv1.at(i),adjustCurv1.at((i+1)%size1),curv2.at(j),curv2.at((j+1)%size2)))
				continue;
			Vec2 p1 = adjustCurv1.at(i);
			Vec2 p2 = adjustCurv1.at((i+1)%size1);
			Vec2 testP = curv2.at(i);
			bary=GetBarycentCoorInATriangle(Vec4(testP.x,testP.y,0.0),Vec4(cent.x,cent.y,0.0), Vec4(p1.x,p1.y,0.0), Vec4(p2.x,p2.y,0.0));
			if(bary.x<=0)
			{
				outP=testP;
			}
			else
			{
				outP=curv2.at((j+1)%size2);
			}
			foot=MapAVertexToASegment(Vec4(outP.x,outP.y,0.0),Vec4(p1.x,p1.y,0.),Vec4(p2.x,p2.y,0.));

			Vec2 foot1(foot.x,foot.y);
			dis=(1+offset)*(outP-foot1);
			adjustCurv1.at(i)=adjustCurv1.at(i)+dis;
			adjustCurv1.at((i+1)%size1)=adjustCurv1.at((i+1)%size1)+dis;
		}
	}

	int sum3=0;
	for(i=0;i<size1; i++)
	{
		for(j=0;j<size2;j++)
		{
			if(IsSegmentsIntersect(adjustCurv1.at(i),adjustCurv1.at((i+1)%size1),curv2.at(j),curv2.at((j+1)%size2)))
				sum3++;
		}
	}

	return resFlag;
}


//---------------------------------------------------------------
// Name:		IsTwo2DCurveInterSect
// Description:	2 2d curves is intersect or not 
// Argument:	curv1,curv2:		two curves
// Return:		true if intersected;
// Author:		ljt
// Date:		2004/09/14
// Modified by:		
// Updated date:		
//---------------------------------------------------------------- 
bool IsTwo2DCurveInterSect(varray<Vec2>&curv1, varray<Vec2>&curv2)
{
	int i, j;
	float size1=(float)curv1.size();
	float size2=(float)curv2.size();

	if(size1<2 || size2<2)
		return false;


	for(i=0;i<size1-1; i++)
	{
		for(j=0;j<size2-1;j++)
		{
			if(IsSegmentsIntersect(curv1.at(i),curv1.at(i+1),curv2.at(j),curv2.at(j+1)))
				return true;
		}
	}
	return false;
}

//---------------------------------------------------------------
// Name:		IsSegmentsIntersect
// Description:	2 2d segments is intersect or not 
// Argument:	Vec2 seg1a, Vec2 seg1b: start and end point's of segment1
//				Vec2 seg2a, Vec2 seg2b: start and end point's of segment2
// Return:		true if intersected;
// Author:		ljt
// Date:		2004/09/14
// Modified by:		
// Updated date:		
//---------------------------------------------------------------- 
bool IsSegmentsIntersect(Vec2 seg1a, Vec2 seg1b, Vec2 seg2a, Vec2 seg2b)
{
	float threshold = (float)1.0e-6;
	//float tx1,tx2,ty1,ty2;
	Vec4 bary;
	if(fabs(seg1a.x-seg1b.x) < threshold)
	{
		if(fabs(seg1a.y-seg1b.y)<  threshold)
			return false;

		if(seg1a.x<=seg2a.x && seg1a.x>=seg2b.x || seg1a.x>=seg2a.x && seg1a.x<=seg2b.x) 
		{   //two segments may intersect
			bary=GetBarycentCoorInATriangle(Vec4(seg1a.x,seg1a.y,0.0),Vec4(seg1b.x,seg1b.y,0.0), Vec4(seg2a.x,seg2a.y,0.0), Vec4(seg2b.x,seg2b.y,0.0));
			if(bary.x<=0 && bary.y>=0 && bary.z>=0)
				return true;
			return false;
		}
	}
	else
	{
		if(IsBoxOfTwoSegIntersect(seg1a,seg1b,seg2a,seg2b)) //box text
		{
			float dot=Dot((seg1a-seg1b).Normalize(), (seg2a-seg2b).Normalize());
			if(fabs(dot-1)<1.0e-4) //the two segments are parallel
			{
				float t1=(seg2a.x-seg1a.x)/(seg1b.x-seg1a.x);
				float t2=(seg2a.y-seg1a.y)/(seg1b.y-seg1a.y);

				if(fabs(t1-t2)<1.0e-4) //the two segments are on the same line
				{
					return true;
				}
			}

			bary=GetBarycentCoorInATriangle(Vec4(seg1a.x,seg1a.y,0.0),Vec4(seg1b.x,seg1b.y,0.0), Vec4(seg2a.x,seg2a.y,0.0), Vec4(seg2b.x,seg2b.y,0.0));
			if(bary.x<=0 && bary.y>=0 && bary.z>=0)
				return true;
			return false;
		}
		else
			return false;
	}
	return false;
}
//---------------------------------------------------------------
// Name:		IsBoxOfTwoSegIntersect
// Description:	the box of segment is intersect or not, box test
// Argument:	Vec2 seg1P1, Vec2 seg1P2: start and end point's of segment1
//				Vec2 seg2P1, Vec2 seg2P1: start and end point's of segment2
// Return:		true if intersected;
// Author:		ljt
// Date:		2004/09/14
// Modified by:	XXX	
// Updated date:20060508		
//---------------------------------------------------------------- 
bool IsBoxOfTwoSegIntersect(Vec2 seg1P1, Vec2 seg1P2, Vec2 seg2P1, Vec2 seg2P2)
{
	float minXSeg1=min(seg1P1.x,seg1P2.x);	

	float maxXSeg1=max(seg1P1.x,seg1P2.x);	

	float minXSeg2=min(seg2P1.x,seg2P2.x);

	float maxXSeg2=max(seg2P1.x,seg2P2.x);


	bool flagX1= maxXSeg1>=minXSeg2 && maxXSeg1<=maxXSeg2;
	bool flagX2= maxXSeg2>=minXSeg1 && maxXSeg2<=maxXSeg1;

	bool flagX=flagX1 || flagX2;

	if(!flagX)
		return false;

	float minYSeg1=min(seg1P1.y,seg1P2.y);
	float maxYSeg1=max(seg1P1.y,seg1P2.y);

	float minYSeg2=min(seg2P1.y,seg2P2.y);
	float maxYSeg2=max(seg2P1.y,seg2P2.y);


	bool flagY1=maxYSeg1>=minYSeg2 && maxYSeg1<=maxYSeg2;
	bool flagY2= maxYSeg2>=minYSeg1 && maxYSeg2<=maxYSeg1;

	bool flagY = flagY1 || flagY2;

	if(!flagY)
		return false;

	return true;	
}

//---------------------------------------------------------------
// Name:		GetIntersectPntOf2DLnCurve
// Description:	get intersect of line to curve
// Argument:	segNod1, segNod2:	Two  points on a line
//				curv:				nodes on a discrete curve
//				closedFlag:			indicate whether the curve is closed
//				intersections:		the result 
// Return:		true if intersected;
// Author:		ljt
// Date:		2004/09/14
// Modified by:	XXX	
// Updated date:20060508		
//---------------------------------------------------------------- 
bool GetIntersectPntOf2DLnCurve(const Vec2 segNod1,const Vec2 segNod2, const varray<Vec2>& curv, bool closedFlag,varray<Vec2>& intersections)
{
	intersections.clear();
	if(curv.size()<2)
		return false;

	//get max value of curve
	float maxX=curv.front().x;
	float minX=curv.front().x;
	float minY=curv.front().y;
	float maxY=curv.front().y;
	int i;
	for(i=1;i<static_cast<int>(curv.size());i++)
	{
		maxX=max(maxX,curv.at(i).x);
		minX=min(minX,curv.at(i).x);
		minY=min(minY,curv.at(i).y);
		maxY=max(maxY,curv.at(i).y);
	}

	float len = (Vec2(minX,maxY),Vec2(maxX,minY)).Magnitude();

	Vec2 sP = segNod1;
	Vec2 eP = sP+len*(segNod2-segNod1).Normalize();
	sP = 2*sP - eP; //i.e. sp = sp - len*(segNod2-segNod1).Normalize()

	int size = static_cast<int>(curv.size());
	if(!closedFlag)
	{
		size=size-1;
	}

	Vec4 inter;
	for(i=0;i<static_cast<int>(curv.size());i++)
	{
		if(IsSegmentsIntersect(sP, eP,curv.at(i),curv.at((i+1)%size)))
		{
			Vec4 v1(sP.x,sP.y,0.);
			Vec4 v2(eP.x,eP.y,0.);
			Vec4 v3(curv.at(i).x,curv.at(i).y,0.);
			Vec4 v4(curv.at((i+1)%size).x,curv.at((i+1)%size).y,0.);
			inter=GetTwoLineInterPt(v1,v2,v3,v4);
			intersections.push_back(Vec2(inter.x,inter.y));
		}
	}
	return intersections.size()>0;
}

//---------------------------------------------------------------
// Name:		UnifyPolygonEdge
// Description:	unify the length of polygon edge
// Argument:	closedCurv1:		curve 1
//				cent:				center
//				adjustCurv1:		adjusted curve
// Return:		true
// Author:		ljt
// Date:		2004/09/14
// Modified by:	XXX	
// Updated date:20060508		
//---------------------------------------------------------------- 
bool UnifyPolygonEdge(const varray<Vec2>&closedCurv1, const Vec2 cent, varray <Vec2>&adjustCurv1)
{	
	varray<Vec2> adjustedShape;
	int i;
	adjustedShape.resize(closedCurv1.size());
	for(i=0;i<static_cast<int>(closedCurv1.size());i++)
	{
		adjustedShape[i]=closedCurv1.at(i);
	}

	int		sel1 = 0, sel2 = 0;
	float	dot;

	Vec2	axisy(0.0,-1.0);
	float	max1=-1.0e6, max2=-1.0e6;

	//get point located at -y and +y
	max1 = -1.0e6, max2 = -1.0e6;
	for(i=0;i<static_cast<int>(adjustedShape.size());i++)
	{
		Vec2 dir(adjustedShape.at(i)-cent);
		dir=dir.Normalize();
		dot=Dot(dir,axisy);
		if(dot>max1)
		{
			max1=dot;
			sel1=i;
		}
		if(-dot>max2)
		{
			max2=-dot;
			sel2=i;
		}
	}

	//reorder the point: from +y to -y
	int divid = 0;
	varray<Vec2> adjustedVs;
	if(sel2 < sel1)
	{
		for(i=sel2;i<=sel1;i++)
		{
			adjustedVs.push_back(adjustedShape.at(i));
		}
		divid = static_cast<int>(adjustedVs.size()-1);
		if(sel1+1 < static_cast<int>(adjustedShape.size()))
		{
			for(i=sel1+1; i<static_cast<int>(adjustedShape.size()); i++)
			{
				adjustedVs.push_back(adjustedShape.at(i));
			}
		}
		for(i=0;i<sel2;i++)
		{
			adjustedVs.push_back(adjustedShape.at(i));
		}		
	}
	else
	{
		for(i=sel2;i<static_cast<int>(adjustedShape.size());i++)
		{
			adjustedVs.push_back(adjustedShape.at(i));
		}
		for(i=0;i<=sel1;i++)
		{
			adjustedVs.push_back(adjustedShape.at(i));
		}
		divid=static_cast<int>(adjustedVs.size()-1);
		if(sel1+1<sel2)
		{
			for(i=sel1+1;i<sel2;i++)
			{
				adjustedVs.push_back(adjustedShape.at(i));
			}
		}		
	}
	///////////////////////////////////////////////
	float partSumLen1=0.0, partSumLen2=0.;
	float averLen1, averLen2;
	for(i=1;i<static_cast<int>(adjustedVs.size());i++)
	{
		if(i<=divid)
		{
			partSumLen1+=(adjustedVs.at(i)-adjustedVs.at(i-1)).Magnitude();
		}
		else
		{
			partSumLen2+=(adjustedVs.at(i)-adjustedVs.at(i-1)).Magnitude();
		}
	}
	partSumLen2 += (adjustedVs.back()-adjustedVs.at(adjustedVs.size()-2)).Magnitude();
	averLen1 = partSumLen1/20;
	averLen2 = partSumLen2/20;

	varray<Vec2>	adjustedVs2;
	adjustedVs2.resize(40);
	adjustedVs2[0]  = adjustedVs.front();
	adjustedVs2[20] = adjustedVs.at(divid);

	int k=1;
	float tempSumLen1=0., tempLen;
	for(i=1;i<=divid;i++)
	{
		tempLen=(adjustedVs.at(i) - adjustedVs.at(i-1)).Magnitude();
		tempSumLen1+=tempLen;
		if(tempSumLen1> k*averLen1)
		{
			adjustedVs2[k]=adjustedVs.at(i)-(tempSumLen1-k*averLen1)*(adjustedVs.at(i)-adjustedVs.at(i-1)).Normalize();
			k++;
			i--;
			tempSumLen1-=tempLen;
			if(k==20)
			{
				break;
			}
		}
	}

	float tempSumLen2=0.;
	k=1;
	for(i=divid+1;i<=static_cast<int>(adjustedVs.size());i++)
	{
		tempLen=(adjustedVs.at(i%adjustedVs.size())-adjustedVs.at(i-1)).Magnitude();
		tempSumLen2+=tempLen;
		if(tempSumLen2 > k*averLen2)
		{
			adjustedVs2[k+20] = adjustedVs.at(i%adjustedVs.size())-(tempSumLen2-k*averLen2)*(adjustedVs.at(i%adjustedVs.size())-adjustedVs.at(i-1)).Normalize();
			k++;
			i--;
			tempSumLen2-=tempLen;
			if(k==20)
			{
				break;
			}
		}
	}
	adjustedVs2[0]=adjustedVs.front();
	adjustedVs2[20]=adjustedVs.at(divid);

	adjustCurv1.clear();
	for(i=30;i<40;i++)
	{
		adjustCurv1.push_back(adjustedVs2[i]);
	}
	for(i=0;i<30;i++)
	{
		adjustCurv1.push_back(adjustedVs2[i]);
	}
	return true;

}
//---------------------------------------------------------------
// Name:	    GetExtremValue()
// Description: Get the left,right,front,back extreme values in x-z plane for inputed points array  
// Argument:    vtsExt[0]: leftmost,vtsExt[1]: front most,vtsExt[2]:rightmost, vtsExt[3]:backmost	
//         :	vtsArr:inputed points array
// Return:		
// Author:	XXX XXX
// Date:	2006/04/29 29:4:2006   13:50
// Update:	 
// Author:	
// Date: 
// copyright: XXX. developed by XXX
//----------------------------------------------------------------
bool GetExtremValue(const varray<Vec4>& vtsArr, varray<Vec4>& vtsExt)
{
	if(vtsArr.size() == 0)
	{
		return false;
	}

	Vec4 vtCen = GetCenter(vtsArr);
	float xcenter = vtCen.x;
	float zcenter = vtCen.z;
	int i = 0;
	float fxmin = 1.0e+8, bxmin = 1.0e+8;
	float lzmin = 1.0e+8, rzmin = 1.0e+8;
	float xdif,zdif;
	vtsExt.resize(4);

	for(i=0; i < static_cast<int>(vtsArr.size()); i++)
	{
		xdif = fabs(vtsArr[i].x - xcenter);
		zdif = fabs(vtsArr[i].z- zcenter);

		if(xdif <= bxmin && vtsArr[i].z<zcenter) 
		{ // back center
			bxmin = xdif;
			vtsExt[3] = vtsArr[i];
		}

		if(xdif <= fxmin && vtsArr[i].z>zcenter) 
		{ //front center
			fxmin = xdif;
			vtsExt[1] = vtsArr[i];
		}

		if(zdif <= lzmin && vtsArr[i].x>xcenter) 
		{// right
			lzmin = zdif;
			vtsExt[2] = vtsArr[i];
		}

		if(zdif <= rzmin && vtsArr[i].x<xcenter)
		{//left
			rzmin = zdif;
			vtsExt[0] = vtsArr[i];
		}
	}
	return true;
}


/********************************************************************
* FUNCTION NAME :MakeConvex
* AUTHOR		:XXX XXX	
* DATE			:2005-03-21
* MODIFIER		:
* MODIFY DATE	:
* DESCRIPTION	:Make the polyline points convex
* PARAMETER     :
*		Input 	    :
*		Return		:
*		Output:	    :
* Version		:1.0
********************************************************************/
void MakeConvex(varray<Vec4> &vlist, Vec4 pnorm)
{
	int i; 
	int pnum = static_cast<int>(vlist.size());
	Vec4 *pp = new Vec4[pnum];
	bool *fg = new bool[pnum];

	for(i=0; i<pnum; i++)
	{
		pp[i] = vlist[i]-vlist[0];
	}

	// rotate the curve to X-Y plane
	pnorm.Normalize();
	Vec4 vct = CrossVecX(pnorm, Vec4(0, 0, 1));
	float ang = (float)acos(Dot(pnorm, Vec4(0, 0, 1)));

	for(i=0; i<pnum; i++)
	{
		pp[i] = RotAxis(pp[i], vct, ang);
	}

	float maxx, minx, maxy, miny, nmin, nmax, ntop;

	float x1, x2, y1, y2;

	maxx = minx = pp[0].x;
	maxy = miny = pp[0].y;
	nmin = nmax = ntop = 0;

	x1 = y1 = 1.0e+6;
	x2 = y2 = -1.0e+6;


	for(i=0; i<pnum; i++)
	{
		if(pp[i].x<x1)
		{
			x1 = pp[i].x;
			y1 = pp[i].y;
		}
		else if(pp[i].x>x2)
		{
			x2 = pp[i].x;
			y2 = pp[i].y;
		}
	}

	// seperate the loop into two parts: upper part and lower part
	Vec4 *UP = new Vec4[pnum];
	int *uf = new int[pnum];
	Vec4 *LP = new Vec4[pnum];
	int *lf = new int[pnum];


	int num1, num2;
	num1=num2=0;
	int ncur, npre;


	for(i=0;i<pnum;i++)
	{

		if(((x2 - x1)*(pp[i].y - y1)-(y2 - y1)*(pp[i].x - x1))>0)
		{
			UP[num1]=pp[i];
			uf[num1] = i;
			num1++;
		}
		else 
		{
			LP[num2]=pp[i];
			lf[num2] = i;
			num2++;
		}

	}

	// sort upper part
	Vec4 tmp;
	int j, tmpf;

	for (i=0; i<num1; i++)
	{
		for (j=i+1; j<num1; j++)
		{
			if (UP[i].x > UP[j].x)
			{
				tmp = UP[i];
				UP[i] = UP[j];
				UP[j] = tmp;

				tmpf = uf[i];
				uf[i] = uf[j];
				uf[j] = tmpf;
			}
		}
	}

	// make upper part convex
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
						{
							break;
						}
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

	int tmpnum=0;
	for(i=0; i<num1; i++)
	{
		if(!fg[i])
		{
			UP[tmpnum] = vlist[uf[i]];
			tmpnum++;
		}
	}

	num1 = tmpnum;

	//sort lower part
	for (i=0; i<num2; i++)
	{
		for (j=i+1; j<num2; j++)
		{
			if (LP[i].x < LP[j].x)
			{
				tmp = LP[i];
				LP[i] = LP[j];
				LP[j] = tmp;

				tmpf = lf[i];
				lf[i] = lf[j];
				lf[j] = tmpf;
			}
		}
	}

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
						{
							break;
						}
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

	tmpnum=0;
	for(i=0; i<num2; i++)
	{
		if(!fg[i])
		{
			LP[tmpnum] = vlist[lf[i]];
			tmpnum++;
		}
	}

	num2 = tmpnum;
	vlist.resize(num1+num2-1);
	for(i=0; i<num1-1; i++)
	{
		vlist[i] = UP[i]; 
	}
	for(i=0; i<num2; i++)
	{
		vlist[i+num1-1] = LP[i];
	}

	pnum = num1+num2-1;

	delete [] pp;
	delete [] fg;
	delete [] UP;
	delete [] uf;
	delete [] LP;
	delete [] lf;
}
/////////////////////////////////////////////////////////////////////////////////
//interploate operation
//2004-09-05 XXX 
bool InterpolateANewLoop(varray<Vec4>& intputLoop1, varray<Vec4>& intputLoop2, float ratio, varray<Vec4>& newLoop)
{
	int size = static_cast<int>(intputLoop1.size());

	if(size != static_cast<int>(intputLoop2.size()) )
	{
		return false;
	}

	int i;
	newLoop.resize(size);
	for(i=0; i<size; i++)
	{
		newLoop.at(i) = intputLoop1.at(i) + ratio*(intputLoop2.at(i)-intputLoop1.at(i));
	}
	return true;
}
/********************************************************************
* FUNCTION NAME :GetInterPolatePt
* AUTHOR		:XXX XXX	
* DATE			:2005-03-28
* MODIFIER		:
* MODIFY DATE	:
* DESCRIPTION	:Get a interpolated point from the given points array in x(y,z)direction 
* PARAMETER     :
*		Input 	    : ptsArr:-- points array for interpolation; vtIn:-- a reference for interpolation
*                   : iMode:-- 0,1,2 means interpolate pt by x,y,z; bIsInterPolated-- whether interpolate operation is succsessful 
*		Return		:
*		Output:	    :
* Version		:1.0
********************************************************************/
Vec4 GetInterPolatePt(const varray<Vec4>&ptsArr,const Vec4& vtIn,int iMode,bool& bIsInterPolated)
{
	if(ptsArr.size()==0)
	{
		return Vec4(0,0,0);
	}
	int i=0;
	bIsInterPolated=false;
	float scl=1;
	Vec4 vtTemp;
	int t=0;
	if(iMode==0)//interpolate pt by x
	{
		if(vtIn.x <= min(ptsArr[0].x,ptsArr.back().x) )
		{
			vtTemp = (ptsArr[0].x > ptsArr.back().x) ? ptsArr.back():ptsArr[0];
			bIsInterPolated = true;
			return vtTemp;
		}
		else if(vtIn.x >= max(ptsArr[0].x,ptsArr.back().x))
		{
			vtTemp = (ptsArr[0].x > ptsArr.back().x) ? ptsArr[0]:ptsArr.back();
			bIsInterPolated = true;
			return vtTemp;
		}
		else
		{
			for(i=0;i<static_cast<int>(ptsArr.size()-1);i++)
			{
				if((ptsArr[i].x<vtIn.x&&ptsArr[i+1].x>=vtIn.x)
					|| (ptsArr[i].x>=vtIn.x&&ptsArr[i+1].x<vtIn.x))
				{
					scl = fabs((vtIn.x-ptsArr[i].x)/(ptsArr[i+1].x-ptsArr[i].x));
					bIsInterPolated=true;
					t=i;
					break;
				}
			}
		}	
	}
	else if(iMode==1)//interpolate pt by y
	{   
		if(vtIn.y <= min(ptsArr[0].y,ptsArr.back().y) )
		{
			vtTemp = (ptsArr[0].y > ptsArr.back().y) ? ptsArr.back():ptsArr[0];
			bIsInterPolated = true;
			return vtTemp;
		}
		else if(vtIn.y >= max(ptsArr[0].y,ptsArr.back().y))
		{
			vtTemp = (ptsArr[0].y > ptsArr.back().y) ? ptsArr[0]:ptsArr.back();
			bIsInterPolated = true;
			return vtTemp;
		}
		else
		{
			for(i=0;i<static_cast<int>(ptsArr.size()-1);i++)
			{
				if((ptsArr[i].y<vtIn.y&&ptsArr[i+1].y>=vtIn.y)
					|| (ptsArr[i].y>=vtIn.y&&ptsArr[i+1].y<vtIn.y))
				{
					scl = fabs((vtIn.y-ptsArr[i].y)/(ptsArr[i+1].y-ptsArr[i].y));		
					bIsInterPolated=true;
					t=i;
					break;
				}
			}
		}		
	}
	else if(iMode==2)//interplate pt by z
	{
		if(vtIn.z <= min(ptsArr[0].z,ptsArr.back().z) )
		{
			vtTemp = (ptsArr[0].z > ptsArr.back().z) ? ptsArr.back():ptsArr[0];
			bIsInterPolated = true;
			return vtTemp;
		}
		else if(vtIn.z >= max(ptsArr[0].z,ptsArr.back().z))
		{
			vtTemp = (ptsArr[0].z > ptsArr.back().z) ? ptsArr[0]:ptsArr.back();
			bIsInterPolated = true;
			return vtTemp;
		}
		else
		{
			for(i=0;i<static_cast<int>(ptsArr.size()-1);i++)
			{
				if((ptsArr[i].z<vtIn.z&&ptsArr[i+1].z>=vtIn.z)
					|| (ptsArr[i].z>=vtIn.z&&ptsArr[i+1].z<vtIn.z))
				{
					scl = fabs((vtIn.z-ptsArr[i].z)/(ptsArr[i+1].z-ptsArr[i].z));
					bIsInterPolated=true;
					t=i;
					break;
				}
			}
		}
	}

	if(bIsInterPolated)
	{
		if(scl<1.0e-3)
		{
			vtTemp=ptsArr[t];	
		}
		else if(fabs(scl-1)<1.0e-3)
		{
			vtTemp=ptsArr[t+1];
		}
		else
		{
			vtTemp=ptsArr[t]*(1-scl)+ptsArr[t+1]*scl;
		}
		return vtTemp;
	}
	else
	{
		return Vec4(0,0,0);
	}
}
//---------------------------------------------------------------
// Name:	    GetInterPolatePt()
// Description: if we can interpolate more than one point,we return the nearest one with the vtIn in given direction	
//            : This function handle points array in 2D
// Argument:	ptsArr:-- points array for interpolation; vtIn:-- a reference for interpolation
//         :	iMode:-- 0,1,2 means interpolate pt by x,y,z;
//         :    iNearDir:--the direction that we use to judge a nearest point with the vtIn 
//         :           
// Return:		
// Author:	    XXX XXX
// Date:	    2006/04/29 29:4:2006   18:17
// Update:	 
// Author:	
// Date: 
// copyright: XXX. developed by XXX
//----------------------------------------------------------------
Vec4 GetInterPolatePt(varray<Vec4>&ptsArr,const Vec4& vtIn,int iMode,int iNearDir,bool& bIsInterPolated)
{
	if(ptsArr.size()==0)
	{
		return Vec4(0,0,0);
	}
	int i=0;
	bIsInterPolated=false;
	float scl=1;
	Vec4 vtTemp,vtRet;
	int t=0;
	varray<int> idxArr;
	varray<float> fSclArr;
	int idx1 = -1,idx2 = -1;
	if(iMode==0)
	{//interpolate pt by x
		float xmin = 1.0e6, xmax = -1.0e6;
		for(i = 0; i < static_cast<int>(ptsArr.size()); i++)
		{
			if(ptsArr[i].x > xmax)
			{
				xmax = ptsArr[i].x;
				idx1 = i;
			}
			if(ptsArr[i].x < xmin)
			{
				xmin = ptsArr[i].x;
				idx2 = i;  
			}
		}
		if(vtIn.x <= xmin )
		{
			vtTemp = ptsArr[idx2];
			bIsInterPolated = true;
			return vtTemp;
		}
		else if(vtIn.x >= xmax)
		{
			vtTemp = ptsArr[idx1];
			bIsInterPolated = true;
			return vtTemp;
		}
		else
		{
			for(i=0;i<static_cast<int>(ptsArr.size()-1);i++)
			{
				if((ptsArr[i].x<vtIn.x&&ptsArr[i+1].x>=vtIn.x) || (ptsArr[i].x>=vtIn.x&&ptsArr[i+1].x<vtIn.x))
				{
					scl = fabs((vtIn.x-ptsArr[i].x)/(ptsArr[i+1].x-ptsArr[i].x));
					bIsInterPolated=true;
					t=i;
					idxArr.push_back(i);
					fSclArr.push_back(scl);
					//	break;
				}
			}
		}	
	}
	else if(iMode==1)
	{//interpolate pt by y
		float ymax = -1.0e6,ymin = 1.0e6;
		for(i = 0; i < static_cast<int>(ptsArr.size()); i++)
		{
			if(ptsArr[i].y > ymax)
			{
				ymax = ptsArr[i].y;
				idx1 = i;
			}
			if(ptsArr[i].y < ymin)
			{
				ymin = ptsArr[i].y;
				idx2 = i;
			}
		}
		if(vtIn.y <= ymin)
		{
			vtTemp = ptsArr[idx2];
			bIsInterPolated = true;
			return vtTemp;
		}
		else if(vtIn.y >= ymax)
		{
			vtTemp = ptsArr[idx1];
			bIsInterPolated = true;
			return vtTemp;
		}
		else
		{
			for(i=0;i<static_cast<int>(ptsArr.size()-1);i++)
			{
				if((ptsArr[i].y<vtIn.y&&ptsArr[i+1].y>=vtIn.y) || (ptsArr[i].y>=vtIn.y&&ptsArr[i+1].y<vtIn.y))
				{
					scl = fabs((vtIn.y-ptsArr[i].y)/(ptsArr[i+1].y-ptsArr[i].y));		
					bIsInterPolated=true;
					t=i;
					idxArr.push_back(i);
					fSclArr.push_back(scl);
					//	break;
				}
			}
		}		
	}
	else if(iMode==2)
	{//interplate pt by z
		float zmax = -1.0e6,zmin = 1.0e6;
		for(i = 0; i < static_cast<int>(ptsArr.size()); i++)
		{
			if(ptsArr[i].z > zmax)
			{
				zmax = ptsArr[i].z;
				idx1 = i;
			}
			if(ptsArr[i].z < zmin)
			{
				zmin = ptsArr[i].z;
				idx2 = (int)zmin;
			}
		}
		if(vtIn.z <= zmin )
		{
			vtTemp = ptsArr[idx2];
			bIsInterPolated = true;
			return vtTemp;
		}
		else if(vtIn.z >= zmax)
		{
			vtTemp = ptsArr[idx1];
			bIsInterPolated = true;
			return vtTemp;
		}
		else
		{
			for(i=0;i<static_cast<int>(ptsArr.size()-1);i++)
			{
				if((ptsArr[i].z<vtIn.z&&ptsArr[i+1].z>=vtIn.z) || (ptsArr[i].z>=vtIn.z&&ptsArr[i+1].z<vtIn.z))
				{
					scl = fabs((vtIn.z-ptsArr[i].z)/(ptsArr[i+1].z-ptsArr[i].z));
					bIsInterPolated=true;
					t=i;
					idxArr.push_back(i);
					fSclArr.push_back(scl);
					break;
				}
			}
		}
	}
	if(bIsInterPolated)
	{
		vtRet = Vec4(1.0e6,1.0e6,1.0e6);
		for(i = 0; i < static_cast<int>(idxArr.size()); i++)
		{
			scl = fSclArr[i];
			t = idxArr[i];
			if(scl<1.0e-3)
			{
				vtTemp=ptsArr[t];	
			}
			else if(fabs(scl-1)<1.0e-3)
			{
				vtTemp=ptsArr[t+1];
			}
			else
			{
				vtTemp=ptsArr[t]*(1-scl)+ptsArr[t+1]*scl;
			}
			if(iNearDir == 0)
			{
				if(fabs(vtTemp.x - vtIn.x) < fabs(vtRet.x - vtIn.x) )
				{
					vtRet = vtTemp;
				}
			}
			else if(iNearDir == 1)
			{
				if(fabs(vtTemp.y - vtIn.y) < fabs(vtRet.y - vtIn.y) )
				{
					vtRet = vtTemp;
				}
			}
			else if(iNearDir == 2)
			{
				if(fabs(vtTemp.z - vtIn.z) < fabs(vtRet.z - vtIn.z))
				{
					vtRet = vtTemp;
				}
			}	
		}
	}
	else
	{
		if(vtIn.y > max(ptsArr[0].y,ptsArr.back().y))
		{
			vtRet = (ptsArr[0].y > ptsArr.back().y) ? ptsArr[0] : ptsArr.back();
		}
		else if(vtIn.y < min(ptsArr[0].y,ptsArr.back().y))
		{
			vtRet = (ptsArr[0].y > ptsArr.back().y) ? ptsArr.back() : ptsArr[0];
		}
	}
	return vtRet;
}

//---------------------------------------------------------------
// Name:	    GetInterPolatePt()
// Description: if we can interpolate more than one point,we return the nearest one with the vtIn in given direction
//            : this function can interpolate the line in not horizontal way
// Argument:    iNearDir, the direction that we use to judge a nearest point with the vtIn
//         :    here, also add constraint for making the pt and ptsArr has the same coordinate  in one direction
//         :	iNearDir:0 -- pt.z =  ptsArr[i].z, and judge a nearest point with the vtIn in x-direction
//         :    iNearDir:2 -- pt.x = ptsArr[i].x,  and judge a nearest point with the vtIn in z-direction
//         :    iNearDir:1 -- without definition
// Return:		
// Author:	    XXX XXX
// Date:	    2006/04/30 30:4:2006   8:27
// Update:	 
// Author:	
// Date: 
// copyright: XXX. developed by XXX
//----------------------------------------------------------------
bool GetInterPolatePt(Vec4 pt, varray<Vec4>& ptsArr,Vec4 pnorm,int iNearDir,Vec4& ptRet)
{   
	int j = 0;
	float d1 = 0, d2 = 0, ra = 0 , dt=(float)1.0e-5;
	Vec4 vt1,vt2;
	varray<Vec4> ptsRetArr;
	for(j = 0; j < static_cast<int>(ptsArr.size() -1); j++)
	{
		vt1 = ptsArr[j];
		vt2 = ptsArr[j+1];
		if(iNearDir == 0)
		{
			pt.z = vt2.z = vt1.z;
		}
		else if(iNearDir == 2)
		{
			pt.x = vt2.x = vt1.x;
		}
		d1 = pnorm.x*(pt.x-vt1.x)+pnorm.y*(pt.y-vt1.y)+pnorm.z*(pt.z-vt1.z);
		d2 = pnorm.x*(pt.x-vt2.x)+pnorm.y*(pt.y-vt2.y)+pnorm.z*(pt.z-vt2.z);

		if(d1==d2)	
		{
			continue;
		}
		if ((d1 >= 0 && d2 <= 0) || (d1 <= 0 && d2 >= 0))
		{
			ra = fabs(d1)/(fabs(d1)+fabs(d2));

			if(ra==0) 
				ra = dt;
			else if(ra==1.0)
				ra = 1-dt;
			// compute the intersecting point
			ptRet.x = ra * (vt2.x - vt1.x) + vt1.x;
			ptRet.y = ra * (vt2.y - vt1.y) + vt1.y;
			ptRet.z = ra * (vt2.z - vt1.z) + vt1.z;
			ptsRetArr.push_back(ptRet);
		}
	}
	if(ptsRetArr.size() >= 1)
	{
		ptRet = ptsRetArr[0];
		for(j = 1; j < static_cast<int>(ptsRetArr.size()); j++)
		{
			if(iNearDir == 0) // x-direction nearest
			{
				if( fabs(ptRet.x - pt.x) > fabs(ptsRetArr[j].x - pt.x))
				{
					ptRet = ptsRetArr[j];
				}
			}
			else if(iNearDir == 1)// y-direction nearest
			{
				if( fabs(ptRet.y - pt.y) > fabs(ptsRetArr[j].y - pt.y))
				{
					ptRet = ptsRetArr[j];
				}
			}
			else if(iNearDir == 2)// z-direction nearest
			{
				if( fabs(ptRet.z - pt.z) > fabs(ptsRetArr[j].z - pt.z))
				{
					ptRet = ptsRetArr[j];
				}
			}
		}
	}
	else if(ptsRetArr.size() == 0)
	{
		if(pt.y > max(ptsArr[0].y,ptsArr.back().y))
		{
			ptRet = (ptsArr[0].y > ptsArr.back().y) ? ptsArr[0] : ptsArr.back();
		}
		else if(pt.y < min(ptsArr[0].y,ptsArr.back().y))
		{
			ptRet = (ptsArr[0].y > ptsArr.back().y) ? ptsArr.back() : ptsArr[0];
		}
		return false;
	}
	return true;
}
/********************************************************************
* FUNCTION NAME :TrimVts
* AUTHOR		:XXX XXX	
* DATE			:2005-4-1
* MODIFIER		:XXX XXX
* MODIFY DATE	:2006.4.29
* DESCRIPTION	: Trim a points array with y(x,z) direction boundary is given      
*               : the vtsArr should in ascend(descend) order at given direction
* PARAMETER     :
*		Input 	    :fSt,fEd:-- y,or(x,z) direction boundary value for trimming
*                   :vtsArr:-- points array to trim
*		Return		:void
*		Output:	    :vtsTrim:-- points trimed by the y(x,z)direction boundary value is given
* Version		:1.0
********************************************************************/
void TrimVts(float fSt,float fEd,int iMode,const varray<Vec4>& vtsArr,varray<Vec4>& vtsTrim)
{
	int iSt = 0, iEd = 0, i = 0;
	float fMin = min(fSt,fEd);
	float fMax = max(fSt,fEd);
	Vec4 vSt,vEd;
	float scl = 1;

	vtsTrim.clear();

	if(iMode == 0)    //x direction trim
	{

	}
	else if(iMode == 1)//y direction trim
	{
		if(fMin > max(vtsArr[0].y,vtsArr.back().y) || fMax < min(vtsArr[0].y,vtsArr.back().y) )
		{
			return;
		}
		if(fMin < min(vtsArr[0].y,vtsArr.back().y) )
		{
			if(vtsArr[0].y < vtsArr.back().y)
			{
				iSt = 0;
				vSt = vtsArr[0];
			}
			else
			{
				iEd = static_cast<int>(vtsArr.size()-1);
				vEd = vtsArr.back();
			}
		}
		if(fMax > max(vtsArr[0].y,vtsArr.back().y) )
		{
			if(vtsArr[0].y < vtsArr.back().y)
			{
				iEd = static_cast<int>(vtsArr.size() - 1); 
				vEd = vtsArr.back();
			}
			else
			{
				iSt = 0;
				vSt = vtsArr[0];
			}
		}
		for(i = 0; i < static_cast<int>(vtsArr.size()-1); i++)
		{
			const Vec4& v1 = vtsArr[i];
			const Vec4& v2 = vtsArr[i+1];
			if( (v1.y <= fSt && v2.y >= fSt) || (v1.y >= fSt && v2.y <= fSt) )
			{
				scl = (fSt - v1.y) / (v2.y - v1.y);
				vSt = v1 * (1 - scl) + v2 * scl;
				iSt = i+1;
				break;
			}
		}

		for(i = 0; i < static_cast<int>(vtsArr.size()-1); i++)
		{
			const Vec4& v1 = vtsArr[i];
			const Vec4& v2 = vtsArr[i+1];
			if( (v1.y <= fEd && v2.y >= fEd) || (v1.y >= fEd && v2.y <= fEd) )
			{
				scl = (fEd - v1.y) / (v2.y - v1.y);
				vEd = v1 * (1 - scl) + v2 * scl;
				iEd = i;
				break;
			}
		}
		if(iSt > 0)
		{
			vtsTrim.push_back(vSt);
		}	
		for(i = iSt; i < iEd; i++)
		{
			vtsTrim.push_back(vtsArr[i]);
		}

		vtsTrim.push_back(vEd);

	}
	else if(iMode == 2)//z direction trim
	{

	}
}
//---------------------------------------------------------------
// Name:	    GetInterSectPntOfLnPolygon()
// Description: compute the line and the polygon intersection point after the polygon is offseted(or scaled) from its center
// Argument:    vtsArr:-- polygon points; vt,vtCen3D:-- two point used to intersect with the polygon,
//         :	vtCen3D:-- is the center of the polygon
//         :    iMode:-- 0-- polygon on XY plane; 1-- polygon on YZ plane; 2--polygon on XZ plane
//         :    offset:-- the distance of the polygon to offset from is center
// Return:		bool:-- whether the line  and the polygon has intersection point 
// Author:		XXX XXX
// Date:		2006/04/29 29:4:2006   14:15
// Update:	 
// Author:	
// Date: 
// copyright:	XXX. developed by XXX
//----------------------------------------------------------------
bool GetInterSectPntOfLnPolygon(varray<Vec2>& vtsArr,Vec4& vt,float offset,int iMode,Vec4 vtCen3D)
{
	int i = 0;
	Vec2 vtCen = Vec2(0,0);
	if( (vtCen3D.x + 10e6)>1 || (vtCen3D.y + 10e6)>1 || (vtCen3D.z + 10e6)>1)
	{
		if(iMode == 0)
		{
			vtCen = Vec2(vtCen3D.x,vtCen3D.y);
		}
		else if(iMode==1)
		{
			vtCen = Vec2(vtCen3D.z,vtCen3D.y);
		}
		else if(iMode==2)
		{
			vtCen = Vec2(vtCen3D.x,vtCen3D.z);
		}
	}
	else
	{
		for(i = 0; i < static_cast<int>(vtsArr.size()); i++)
		{
			vtCen += vtsArr[i];
		}
		vtCen /= (float)vtsArr.size();
	}

	varray<Vec2> vtsArrTmp = vtsArr;
	for(i = 0; i < static_cast<int>(vtsArrTmp.size()); i++)
	{
		vtsArrTmp[i] += (vtsArrTmp[i] - vtCen).Normalize() * offset;
	}
	bool ret = GetInterSectPntOfLnPolygon(vtsArrTmp,vt,iMode,vtCen3D);
	return ret;
}
/********************************************************************
* FUNCTION NAME :GetInterSectPntOfLnPolygon
* AUTHOR		:XXX XXX	
* DATE			:2005-4-1
* MODIFIER		:XXX XXX
* MODIFY DATE	:2006-4-29
* DESCRIPTION	:compute the line and the polygon intersection point,the polygon should be closed
:iMode: 0-- polygon on XY plane; 1-- polygon on YZ plane; 2--polygon on XZ plane
:now the YZ plane are finished
* PARAMETER     :
*		Input 	:vtsArr:-- polygon points; vt,vtCen3D:-- two point used to intersect with the polygon
*               :iMode:--0-- polygon on XY plane; 1-- polygon on YZ plane; 2--polygon on XZ plane
*		Return		:bool:-- whether the line  and the polygon has intersection point 
*		Output:	    :
* Version		:1.0
********************************************************************/
bool GetInterSectPntOfLnPolygon(varray<Vec2>& vtsArr,Vec4& vt,int iMode,Vec4 vtCen3D)
{
	bool bRet = false;
	int i = 0; 
	Vec2 vt_cen = Vec2(0,0);
	if( (vtCen3D.x + 10e6)>1 || (vtCen3D.y + 10e6)>1 || (vtCen3D.z + 10e6)>1)
	{
		if(iMode == 0)
		{
			vt_cen = Vec2(vtCen3D.x,vtCen3D.y);
		}
		else if(iMode==1)
		{
			vt_cen = Vec2(vtCen3D.z,vtCen3D.y);
		}
		else if(iMode==2)
		{
			vt_cen = Vec2(vtCen3D.x,vtCen3D.z);
		}
	}
	else
	{
		for(i = 0; i < static_cast<int>(vtsArr.size()); i++)
		{
			vt_cen += vtsArr[i];
		}
		vt_cen /= (float)vtsArr.size();
	}

	if(iMode == 0)
	{		//x-y
		if(IsPointInConexPolygon(Vec2(vt.x,vt.y),vtsArr) )
		{
			Vec2 dir = Vec2(vt.x,vt.y);
			dir = vt_cen + (dir - vt_cen).Normalize() * 1000;
			Vec2 v3,v4;
			for(i = 0; i < static_cast<int>(vtsArr.size()-1); i++)
			{
				v3 = vtsArr[i];
				v4 = vtsArr[i+1];

				if(IsTwoLineIntersectIn2d(Vec4(vt_cen.x,vt_cen.y,0.0),Vec4(dir.x,dir.y,0.0),Vec4(v3.x,v3.y,0.0),Vec4(v4.x,v4.y,0.0)) )
				{
					Vec2 inter =  GetTwoLineInterPt2D(vt_cen,dir,v3,v4);
					vt = Vec4(inter.x,inter.y,vt.z);
					bRet = true;
					break;
				}
			}
		}
		else
		{
			return false;
		}
	}
	else if(iMode == 1)
	{	//y-z
		if(IsPointInConexPolygon(Vec2(vt.z,vt.y),vtsArr) )
		{
			Vec2 dir = Vec2(vt.z,vt.y);
			dir = vt_cen + (dir - vt_cen).Normalize() * 1000;
			Vec2 v3,v4;
			for(i = 0; i < static_cast<int>(vtsArr.size()-1); i++)
			{
				v3 = vtsArr[i];
				v4 = vtsArr[i+1];

				if(IsTwoLineIntersectIn2d(Vec4(vt_cen.x,vt_cen.y,0.0),Vec4(dir.x,dir.y,0.0),Vec4(v3.x,v3.y,0.0),Vec4(v4.x,v4.y,0.0)) )
				{
					Vec2 inter =  GetTwoLineInterPt2D(vt_cen,dir,v3,v4);
					vt = Vec4(vt.x,inter.y,inter.x);
					bRet = true;
					break;
				}
			}
		}
		else
		{
			return false;
		}
	}
	else if(iMode == 2)
	{	 //x-z
		if(IsPointInConexPolygon(Vec2(vt.x,vt.z),vtsArr) )
		{
			Vec2 dir = Vec2(vt.x,vt.z);
			dir = vt_cen + (dir - vt_cen).Normalize() * 1000;
			Vec2 v3,v4;
			for(i = 0; i < static_cast<int>(vtsArr.size()-1); i++)
			{
				v3 = vtsArr[i];
				v4 = vtsArr[i+1];

				if(IsTwoLineIntersectIn2d(Vec4(vt_cen.x,vt_cen.y,0.0),Vec4(dir.x,dir.y,0.0),Vec4(v3.x,v3.y,0.0),Vec4(v4.x,v4.y,0.0)) )
				{
					Vec2 inter =  GetTwoLineInterPt2D(vt_cen,dir,v3,v4);
					vt = Vec4(inter.x,vt.y,inter.y);
					bRet = true;
					break;
				}
			}
		}
		else
		{
			return false;
		}
	}
	return bRet;
}

//---------------------------------------------------------------
// Name:	    OffSetPolygon()
// Description: Offset the polygon with the given offset distance
// Argument:    vtsArr:--polygon points inputed;  fOffset:-- offset distance
//         :	vtCen3D:-- polygon center
// Return:		
// Author:	XXX XXX
// Date:	2006/04/29 29:4:2006   14:19
// Update:	 
// Author:	
// Date: 
// copyright: XXX. developed by XXX
//----------------------------------------------------------------
void OffSetPolygon(varray<Vec4>& vtsArr,float fOffset, Vec4 vtCen3D)
{
	int i = 0;
	Vec4 vtCen = Vec4(0,0,0);
	if( (vtCen3D.x + 10e6)>1 || (vtCen3D.y + 10e6)>1 || (vtCen3D.z + 10e6)>1)
	{
		vtCen = vtCen3D;
	}
	else
	{
		for(i = 0; i < static_cast<int>(vtsArr.size()); i++)
		{
			vtCen += vtsArr[i];
		}
		vtCen /= (float)vtsArr.size();
	}
	Vec4 vtDir;
	for(i = 0; i < static_cast<int>(vtsArr.size()); i++)
	{
		vtDir = (vtsArr[i]-vtCen);
		vtsArr[i] += vtDir.Normalize() * fOffset;
	}
}
//---------------------------------------------------------------
// Name:	    CollisionAdjustBetween2Polygons
// Description: Adjust the inputed polygon shape by collision detection with another inputed polygon
// Argument:    vtsRef:-- the polygon for reference;  vtsArr:-- the polygon to adjust
//         :	fOffset:-- offset distances for the references polygon
//         :	iMode: 0-- points on XY plane; 1-- polygon on YZ plane; 2--polygon on XZ plane
// Return:		
// Author:		XXX XXX
// Date:		2006/04/29 29:4:2006   14:24
// Update:	 
// Author:	
// Date: 
// copyright: XXX. developed by XXX
//----------------------------------------------------------------
void CollisionAdjustBetween2Polygons(const varray<Vec4>& vtsRef,varray<Vec4>& vtsArr,float fOffset,int iMode,Vec4 vtCen3D)
{
	if(iMode < 0 || iMode > 2)
	{
		return;
	}

	int i = 0;
	varray<Vec2> plgRef;
	if(iMode == 0)
	{
		for(i = 0; i < static_cast<int>(vtsRef.size()); i++)
		{
			plgRef.push_back(Vec2(vtsRef[i].x,vtsRef[i].y));
		}

	}
	else if(iMode == 1)
	{
		for(i = 0; i < static_cast<int>(vtsRef.size()); i++)
		{
			plgRef.push_back(Vec2(vtsRef[i].z,vtsRef[i].y));
		}

	}
	else if(iMode == 2)
	{
		for(i = 0; i < static_cast<int>(vtsRef.size()); i++)
		{
			plgRef.push_back(Vec2(vtsRef[i].x,vtsRef[i].z));
		}
	}

	// offset the reference polygon
	Vec2 vtCen = Vec2(0,0);
	if( (vtCen3D.x + 10e6)>1 || (vtCen3D.y + 10e6)>1 || (vtCen3D.z + 10e6)>1)
	{
		if(iMode == 0)
		{
			vtCen = Vec2(vtCen3D.x,vtCen3D.y);
		}
		else if(iMode==1)
		{
			vtCen = Vec2(vtCen3D.z,vtCen3D.y);
		}
		else if(iMode==2)
		{
			vtCen = Vec2(vtCen3D.x,vtCen3D.z);
		}
	}
	else
	{
		for(i = 0; i < static_cast<int>(plgRef.size()); i++)
		{
			vtCen += plgRef[i];
		}
		vtCen /= (float)plgRef.size();
	}

	Vec2 vtDir;
	for(i = 0; i < static_cast<int>(plgRef.size()); i++)
	{
		vtDir = (plgRef[i]-vtCen);
		plgRef[i] += vtDir.Normalize() * fOffset;
	}

	for(i = 0; i < static_cast<int>(vtsArr.size()); i++)
	{
		GetInterSectPntOfLnPolygon(plgRef,vtsArr[i],iMode,vtCen3D);
	}
}

//---------------------------------------------------------------
// Name:	    CollisionAdjustBetween2Polygons
// Description: Adjust the inputed polygon shape by collision detection with another inputed polygon
// Argument:    vtsRef:-- the polygon for reference; 
//         :    vtsArr:-- the polygon to adjust (after edited); vtsPreArr:-- the polygon to adjust (before edited)
//         :	fOffset:-- offset distances for the references polygon
//         :	iMode: 0-- points on XY plane; 1-- polygon on YZ plane; 2--polygon on XZ plane
//         :    vtCen3D:-- polygon center
// Return:		
// Author:		XXX XXX
// Date:		2006/04/29 29:4:2006   14:31
// Update:	 
// Author:	
// Date: 
// copyright: XXX. developed by XXX
//----------------------------------------------------------------
void CollisionAdjustBetween2Polygons(const varray<Vec4>& vtsRef,varray<Vec4>& vtsArr,varray<Vec4>& vtsPreArr, float fOffset,int iMode,Vec4 vtCen3D)
{
	if(iMode < 0 || iMode > 2)
	{
		return;
	}

	int i = 0;
	varray<Vec2> plgRef,vec2Arr,vec2PreArr;

	if(iMode == 0)
	{
		for(i = 0; i < static_cast<int>(vtsRef.size()); i++)
		{
			plgRef.push_back(Vec2(vtsRef[i].x,vtsRef[i].y));
		}
		for(i = 0; i < static_cast<int>(vtsArr.size()); i++)
		{
			vec2Arr.push_back(Vec2(vtsArr[i].x,vtsArr[i].y));
		}
		for(i = 0; i < static_cast<int>(vtsPreArr.size()); i++)
		{
			vec2PreArr.push_back(Vec2(vtsPreArr[i].x,vtsPreArr[i].y));
		}
	}
	else if(iMode == 1)
	{
		for(i = 0; i < static_cast<int>(vtsRef.size()); i++)
		{
			plgRef.push_back(Vec2(vtsRef[i].z,vtsRef[i].y));
		}
		for(i = 0; i < static_cast<int>(vtsArr.size()); i++)
		{
			vec2Arr.push_back(Vec2(vtsArr[i].z,vtsArr[i].y));
		}
		for(i = 0; i < static_cast<int>(vtsPreArr.size()); i++)
		{
			vec2PreArr.push_back(Vec2(vtsPreArr[i].z,vtsPreArr[i].y));
		}
	}
	else if(iMode == 2)
	{
		for(i = 0; i < static_cast<int>(vtsRef.size()); i++)
		{
			plgRef.push_back(Vec2(vtsRef[i].x,vtsRef[i].z));
		}
		for(i = 0; i < static_cast<int>(vtsArr.size()); i++)
		{
			vec2Arr.push_back(Vec2(vtsArr[i].x,vtsArr[i].z));
		}
		for(i = 0; i < static_cast<int>(vtsPreArr.size()); i++)
		{
			vec2PreArr.push_back(Vec2(vtsPreArr[i].x,vtsPreArr[i].z));
		}
	}
	// offset the reference polygon
	Vec2 vtCen = Vec2(0,0);
	if( (vtCen3D.x + 10e6)>1 || (vtCen3D.y + 10e6)>1 || (vtCen3D.z + 10e6)>1)
	{
		if(iMode == 0)
		{
			vtCen = Vec2(vtCen3D.x,vtCen3D.y);
		}
		else if(iMode==1)
		{
			vtCen = Vec2(vtCen3D.z,vtCen3D.y);
		}
		else if(iMode==2)
		{
			vtCen = Vec2(vtCen3D.x,vtCen3D.z);
		}
	}
	else
	{
		for(i = 0; i < static_cast<int>(plgRef.size()); i++)
		{
			vtCen += plgRef[i];
		}
		vtCen /= (float)plgRef.size();
	}

	Vec2 vtDir;
	for(i = 0; i < static_cast<int>(plgRef.size()); i++)
	{
		vtDir = (plgRef[i]-vtCen);
		plgRef[i] += vtDir.Normalize() * fOffset;
	}

	if(vec2Arr.size() != vec2PreArr.size())
	{
		for(i = 0; i < static_cast<int>(vtsArr.size()); i++)
		{
			GetInterSectPntOfLnPolygon(plgRef,vtsArr[i],iMode,vtCen3D);
		}
	}
	else
	{
		for(i = 0; i < static_cast<int>(vtsArr.size()); i++)
		{
			if((vec2Arr[i] - vec2PreArr[i]).Magnitude() < 0.001)
			{
				continue;
			}
			GetInterSectPntOfLnPolygon(plgRef,vtsArr[i],iMode,vtCen3D);
		}
	}
}

//---------------------------------------------------------------
// Name:	    DevideLoopByDis()
// Description: Divide the inputed loop by distance equivalent in left-front-right-back 4 parts 
// Argument:    ptsIn:-- point list in original loop, should be closed, and is on x-z plane
//         :	plist:-- point list in the new loop;    vtCen:-- loop center
//         :    nump: the division number
// Return:		
// Author:	XXX XXX
// Date:	2006/04/29 29:4:2006   14:55
// Update:	 
// Author:	
// Date: 
// copyright: XXX. developed by XXX
//----------------------------------------------------------------
void DivideLoopByDis(const varray<Vec4>& ptsIn, varray<Vec4>& plist,Vec4 vtCen,int nump)
{
	//precondition
	if(ptsIn.size() < 4)
	{
		return;
	}

	int i;
	int npt = static_cast<int>(ptsIn.size());

	//check the loop direction
	Vec4 vt0 = ptsIn[0];
	Vec4 vt1 = ptsIn[npt/4];
	Vec4 vt2 = ptsIn[npt/2];

	varray<Vec4> ptsInTmp = ptsIn;
	varray<Vec4> ptsInBak = ptsIn;

	if((ptsInTmp[0] - ptsInTmp.back()).Magnitude() < 0.001)
	{
		ptsInTmp.pop_back();
		ptsInBak.pop_back();
	}

	if(CrossVecX((vt2 - vt0),(vt1-vt0)).y < 0)
	{
		int iSize = static_cast<int>(ptsInBak.size());
		for(i = 1; i < static_cast<int>(ptsInBak.size()); i++)
		{
			ptsInTmp[i] = ptsInBak.at(iSize-i);
		}
	}

	varray<Vec4> ptsInTmp1;
	ptsInTmp1.push_back(ptsInTmp[0]);
	for(i = 1; i < static_cast<int>(ptsInTmp.size()); i++)
	{
		if( (ptsInTmp[i] - ptsInTmp1.back()).Magnitude() > 1.0e-4)
		{
			ptsInTmp1.push_back(ptsInTmp[i]);
		}
	}

	ptsInTmp = ptsInTmp1;
	Vec4 vt = ptsInTmp[0];
	ptsInTmp.push_back(vt);

	npt = static_cast<int>(ptsInTmp.size());
	// divide the loop into four segment based on the center of the loop
	Vec4 p1, p_f, p_b, p_l, p_r;
	int n_f = -1, n_b = -1, n_l = -1, n_r = -1;

	for(i=1; i < static_cast<int>(ptsInTmp.size()); i++)
	{
		if((ptsInTmp[i]-ptsInTmp[i-1]).Magnitude() <= ERR) 
		{
			continue;
		}
		if( (ptsInTmp[i-1].x >= vtCen.x && ptsInTmp[i].x <= vtCen.x) || (ptsInTmp[i-1].x <= vtCen.x && ptsInTmp[i].x >= vtCen.x))
		{
			if(ptsInTmp[i].z > vtCen.z)
			{
				p_f = ptsInTmp[i-1] +(ptsInTmp[i] - ptsInTmp[i-1])*(vtCen.x - ptsInTmp[i-1].x)/(ptsInTmp[i].x - ptsInTmp[i-1].x);
				n_f = i;
			}
			else
			{
				p_b = ptsInTmp[i-1] +(ptsInTmp[i] - ptsInTmp[i-1])*(vtCen.x - ptsInTmp[i-1].x)/(ptsInTmp[i].x - ptsInTmp[i-1].x);
				n_b = i;
			}
		}

		if( (ptsInTmp[i-1].z >= vtCen.z && ptsInTmp[i].z <= vtCen.z) || (ptsInTmp[i-1].z <= vtCen.z && ptsInTmp[i].z >= vtCen.z))
		{
			if(ptsInTmp[i].x > vtCen.x)
			{
				p_r = ptsInTmp[i-1] +(ptsInTmp[i] - ptsInTmp[i-1])*(vtCen.z - ptsInTmp[i-1].z)/(ptsInTmp[i].z - ptsInTmp[i-1].z);
				n_r = i;
			}
			else
			{
				p_l = ptsInTmp[i-1] +(ptsInTmp[i] - ptsInTmp[i-1])*(vtCen.z - ptsInTmp[i-1].z)/(ptsInTmp[i].z - ptsInTmp[i-1].z);
				n_l = i;
			}
		}
	}

	if(n_l < 0 || n_r < 0 || n_f < 0 || n_b <0)
	{
		return;
	}

	int npt_f_r=0, npt_f_l=0, npt_b_r=0, npt_b_l=0;
	int nn=0;

	varray<Vec4> plist_f_r, plist_f_l, plist_b_r, plist_b_l;

	// for the front-right segment
	nn=1;
	if(n_f > n_r)
	{
		npt_f_r = n_f - n_r + 2;
		plist_f_r.resize(npt_f_r);

		plist_f_r[0] = p_r;
		plist_f_r[npt_f_r-1] = p_f;
		for(i=n_r; i<n_f; i++)
		{
			plist_f_r[nn++] = ptsInTmp[i];
		}
	}
	else
	{
		npt_f_r = n_f + (npt-n_r)+2;

		plist_f_r.resize(npt_f_r);

		plist_f_r[0] = p_r;
		plist_f_r[npt_f_r-1] = p_f;

		for(i=n_r; i<npt; i++)
		{
			plist_f_r[nn++] = ptsInTmp[i];
		}
		for(i=0; i<n_f; i++)
		{
			plist_f_r[nn++] = ptsInTmp[i];
		}
	}

	////
	// for the front-left segment
	nn=1;
	if(n_l>n_f)
	{
		npt_f_l = n_l - n_f + 2;

		plist_f_l.resize(npt_f_l);

		plist_f_l[0] = p_f;
		plist_f_l[npt_f_l-1] = p_l;

		for(i=n_f; i<n_l; i++)
		{
			plist_f_l[nn++] = ptsInTmp[i];
		}
	}
	else
	{
		npt_f_l = n_l + (npt-n_f)+2;

		plist_f_l.resize(npt_f_l);

		plist_f_l[0] = p_f;
		plist_f_l[npt_f_l-1] = p_l;

		for(i=n_f; i<npt; i++)
		{
			plist_f_l[nn++] = ptsInTmp[i];
		}

		for(i=0; i<n_l; i++)
		{
			plist_f_l[nn++] = ptsInTmp[i];
		}
	}

	// for the back-left segment
	nn=1;
	if(n_b>n_l)
	{
		npt_b_l = n_b - n_l + 2;

		plist_b_l.resize(npt_b_l);

		plist_b_l[0] = p_l;
		plist_b_l[npt_b_l-1] = p_b;

		for(i=n_l; i<n_b; i++)
		{
			plist_b_l[nn++] = ptsInTmp[i];
		}
	}
	else
	{
		npt_b_l = n_b + (npt-n_l)+2;

		plist_b_l.resize(npt_b_l);

		plist_b_l[0] = p_l;
		plist_b_l[npt_b_l-1] = p_b;

		for(i=n_l; i<npt; i++)
		{
			plist_b_l[nn++] = ptsInTmp[i];
		}

		for(i=0; i<n_b; i++)
		{
			plist_b_l[nn++] = ptsInTmp[i];
		}
	}

	// for the back-right segment
	nn=1;
	if(n_r>n_b)
	{
		npt_b_r = n_r - n_b + 2;

		plist_b_r.resize(npt_b_r);

		plist_b_r[0] = p_b;
		plist_b_r[npt_b_r-1] = p_r;

		for(i=n_b; i<n_r; i++)
		{
			plist_b_r[nn++] = ptsInTmp[i];
		}
	}
	else
	{
		npt_b_r = n_r + (npt-n_b)+2;

		plist_b_r.resize(npt_b_r);

		plist_b_r[0] = p_b;
		plist_b_r[npt_b_r-1] = p_r;

		for(i=n_b; i<npt; i++)
		{
			plist_b_r[nn++] = ptsInTmp[i];
		}

		for(i=0; i<n_r; i++)
		{
			plist_b_r[nn++] = ptsInTmp[i];
		}
	}

	// divide each segment to a polyline whose points have equal length
	varray<Vec4> pline;
	plist.resize(nump);
	nn=0;
	MakePolyline(plist_f_r,pline,nump/4);
	for(i = 0; i < static_cast<int>(pline.size()); i++)
	{
		plist[nn++] = pline[i];
	}
	MakePolyline(plist_f_l,pline,nump/4);
	for(i = 1; i < static_cast<int>(pline.size()); i++)
	{
		plist[nn++] = pline[i];
	}
	MakePolyline(plist_b_l,pline,nump/4);
	for(i = 1; i < static_cast<int>(pline.size()); i++)
	{
		plist[nn++] = pline[i];
	}
	MakePolyline(plist_b_r,pline,nump/4);
	for(i = 1; i < static_cast<int>(pline.size()-1); i++)
	{
		plist[nn++] = pline[i];
	}
}
//---------------------------------------------------------------
// Name:	     DevideLoopByDis()
// Description: Divide the inputed loop by distance equivalent in 8 parts with 2 x-center as reference
// Argument:    ptsIn:-- point list in original loop, should be closed, and is on x-z plane
//         :	 plist:-- point list in the new loop;    vtCen:-- loop center, 
//         :    nump: the division number               fXAidedMid:-- aided loop x-center
// Return:		
// Author:		 XXX XXX
// Date:		 2006/04/29 29:4:2006   14:55
// Update:	 
// Author:	
// Date: 
// copyright: XXX. developed by XXX
//----------------------------------------------------------------
void DivideLoopByDis(const varray<Vec4>& ptsIn, varray<Vec4>& plist,Vec4 vtCen,float fXAidedMid,int nump)
{
	//precondition
	if(ptsIn.size() < 4)
	{
		return;
	}
	int i;
	int npt = static_cast<int>(ptsIn.size());
	//check the loop direction
	Vec4 vt0 = ptsIn[0];
	Vec4 vt1 = ptsIn[npt/4];
	Vec4 vt2 = ptsIn[npt/2];

	varray<Vec4> ptsInTmp = ptsIn;
	varray<Vec4> ptsInBak = ptsIn;
	if((ptsInTmp[0] - ptsInTmp.back()).Magnitude() < 0.001)
	{
		ptsInTmp.pop_back();
		ptsInBak.pop_back();
	}
	if(CrossVecX((vt2 - vt0),(vt1-vt0)).y < 0)
	{
		int iSize = static_cast<int>(ptsInBak.size());
		for(i = 1; i < static_cast<int>(ptsInBak.size()); i++)
		{
			ptsInTmp[i] = ptsInBak.at(iSize-i);
		}
	}
	varray<Vec4> ptsInTmp1;
	ptsInTmp1.push_back(ptsInTmp[0]);
	for(i = 1; i < static_cast<int>(ptsInTmp.size()); i++)
	{
		if( (ptsInTmp[i] - ptsInTmp1.back()).Magnitude() > 1.0e-4)
		{
			ptsInTmp1.push_back(ptsInTmp[i]);
		}
	}
	ptsInTmp = ptsInTmp1;
	Vec4 vt = ptsInTmp[0];
	ptsInTmp.push_back(vt);

	npt = static_cast<int>(ptsInTmp.size());
	// divide the loop into four segment based on the center of the loop
	Vec4 p1, p_f, p_b, p_l, p_r;
	int n_f = -1, n_b = -1, n_l = -1, n_r = -1;

	for(i=1; i < static_cast<int>(ptsInTmp.size()); i++)
	{
		if((ptsInTmp[i]-ptsInTmp[i-1]).Magnitude() <= ERR) 
		{
			continue;
		}
		if( (ptsInTmp[i-1].x >= vtCen.x && ptsInTmp[i].x <= vtCen.x) || (ptsInTmp[i-1].x <= vtCen.x && ptsInTmp[i].x >= vtCen.x))
		{
			if(ptsInTmp[i].z > vtCen.z)
			{
				p_f = ptsInTmp[i-1] +(ptsInTmp[i] - ptsInTmp[i-1])*(vtCen.x - ptsInTmp[i-1].x)/(ptsInTmp[i].x - ptsInTmp[i-1].x);
				n_f = i;
			}
			else
			{
				p_b = ptsInTmp[i-1] +(ptsInTmp[i] - ptsInTmp[i-1])*(vtCen.x - ptsInTmp[i-1].x)/(ptsInTmp[i].x - ptsInTmp[i-1].x);
				n_b = i;
			}
		}

		if( (ptsInTmp[i-1].z >= vtCen.z && ptsInTmp[i].z <= vtCen.z) || (ptsInTmp[i-1].z <= vtCen.z && ptsInTmp[i].z >= vtCen.z))
		{
			if(ptsInTmp[i].x > vtCen.x)
			{
				p_r = ptsInTmp[i-1] +(ptsInTmp[i] - ptsInTmp[i-1])*(vtCen.z - ptsInTmp[i-1].z)/(ptsInTmp[i].z - ptsInTmp[i-1].z);
				n_r = i;
			}
			else
			{
				p_l = ptsInTmp[i-1] +(ptsInTmp[i] - ptsInTmp[i-1])*(vtCen.z - ptsInTmp[i-1].z)/(ptsInTmp[i].z - ptsInTmp[i-1].z);
				n_l = i;
			}
		}
	}

	if(n_l < 0 || n_r < 0 || n_f < 0 || n_b <0)
	{
		return;
	}
	int npt_f_r=0, npt_f_l=0, npt_b_r=0, npt_b_l=0;
	int nn=0;

	varray<Vec4> plist_f_r, plist_f_l, plist_b_r, plist_b_l;

	// for the front-right segment
	nn=1;
	if(n_f > n_r)
	{
		npt_f_r = n_f - n_r + 2;
		plist_f_r.resize(npt_f_r);

		plist_f_r[0] = p_r;
		plist_f_r[npt_f_r-1] = p_f;
		for(i=n_r; i<n_f; i++)
		{
			plist_f_r[nn++] = ptsInTmp[i];
		}
	}
	else
	{
		npt_f_r = n_f + (npt-n_r)+2;

		plist_f_r.resize(npt_f_r);

		plist_f_r[0] = p_r;
		plist_f_r[npt_f_r-1] = p_f;

		for(i=n_r; i<npt; i++)
		{
			plist_f_r[nn++] = ptsInTmp[i];
		}
		for(i=0; i<n_f; i++)
		{
			plist_f_r[nn++] = ptsInTmp[i];
		}
	}

	////
	// for the front-left segment
	nn=1;
	if(n_l>n_f)
	{
		npt_f_l = n_l - n_f + 2;

		plist_f_l.resize(npt_f_l);

		plist_f_l[0] = p_f;
		plist_f_l[npt_f_l-1] = p_l;

		for(i=n_f; i<n_l; i++)
		{
			plist_f_l[nn++] = ptsInTmp[i];
		}
	}
	else
	{
		npt_f_l = n_l + (npt-n_f)+2;

		plist_f_l.resize(npt_f_l);

		plist_f_l[0] = p_f;
		plist_f_l[npt_f_l-1] = p_l;

		for(i=n_f; i<npt; i++)
		{
			plist_f_l[nn++] = ptsInTmp[i];
		}

		for(i=0; i<n_l; i++)
		{
			plist_f_l[nn++] = ptsInTmp[i];
		}
	}

	// for the back-left segment
	nn=1;
	if(n_b>n_l)
	{
		npt_b_l = n_b - n_l + 2;

		plist_b_l.resize(npt_b_l);

		plist_b_l[0] = p_l;
		plist_b_l[npt_b_l-1] = p_b;

		for(i=n_l; i<n_b; i++)
		{
			plist_b_l[nn++] = ptsInTmp[i];
		}
	}
	else
	{
		npt_b_l = n_b + (npt-n_l)+2;

		plist_b_l.resize(npt_b_l);

		plist_b_l[0] = p_l;
		plist_b_l[npt_b_l-1] = p_b;

		for(i=n_l; i<npt; i++)
		{
			plist_b_l[nn++] = ptsInTmp[i];
		}

		for(i=0; i<n_b; i++)
		{
			plist_b_l[nn++] = ptsInTmp[i];
		}
	}

	// for the back-right segment
	nn=1;
	if(n_r>n_b)
	{
		npt_b_r = n_r - n_b + 2;

		plist_b_r.resize(npt_b_r);

		plist_b_r[0] = p_b;
		plist_b_r[npt_b_r-1] = p_r;

		for(i=n_b; i<n_r; i++)
		{
			plist_b_r[nn++] = ptsInTmp[i];
		}
	}
	else
	{
		npt_b_r = n_r + (npt-n_b)+2;

		plist_b_r.resize(npt_b_r);

		plist_b_r[0] = p_b;
		plist_b_r[npt_b_r-1] = p_r;

		for(i=n_b; i<npt; i++)
		{
			plist_b_r[nn++] = ptsInTmp[i];
		}

		for(i=0; i<n_r; i++)
		{
			plist_b_r[nn++] = ptsInTmp[i];
		}
	}

	// divide each segment to a polyline whose points have equal length
	//Use the aided x-middle value to separate the 4 segments into 8 segments
	float fXAidedL = fXAidedMid, fXAidedR = -fXAidedMid;
	varray<Vec4> plist_f_r_0,plist_f_r_1,plist_f_l_0,plist_f_l_1;
	varray<Vec4> plist_b_l_0,plist_b_l_1,plist_b_r_0,plist_b_r_1;
	bool bFlag1 = DivideSegmentsToTwoByMidX(plist_f_r,plist_f_r_0,plist_f_r_1,fXAidedR);
	bool bFlag2 = DivideSegmentsToTwoByMidX(plist_f_l,plist_f_l_0,plist_f_l_1,fXAidedL);

	if(bFlag1 && bFlag2 )
	{
		varray<Vec4> pline;
		plist.resize(nump);
		nn=0;
		MakePolyline(plist_f_r_0,pline,nump/8);
		for(i = 0; i < static_cast<int>(pline.size()); i++)
		{
			plist[nn++] = pline[i];
		}
		MakePolyline(plist_f_r_1,pline,nump/8);
		for(i = 1; i < static_cast<int>(pline.size()); i++)
		{
			plist[nn++] = pline[i];
		}
		MakePolyline(plist_f_l_0,pline,nump/8);
		for(i = 1; i < static_cast<int>(pline.size()); i++)
		{
			plist[nn++] = pline[i];
		}
		MakePolyline(plist_f_l_1,pline,nump/8);
		for(i = 1; i < static_cast<int>(pline.size()); i++)
		{
			plist[nn++] = pline[i];
		}

		MakePolyline(plist_b_l,pline,nump/4);
		for(i = 1; i < static_cast<int>(pline.size()); i++)
		{
			plist[nn++] = pline[i];
		}
		MakePolyline(plist_b_r,pline,nump/4);
		for(i = 1; i < static_cast<int>(pline.size()-1); i++)
		{
			plist[nn++] = pline[i];
		}
	}
	else
	{
		varray<Vec4> pline;
		plist.resize(nump);
		nn=0;
		MakePolyline(plist_f_r,pline,nump/4);
		for(i = 0; i < static_cast<int>(pline.size()); i++)
		{
			plist[nn++] = pline[i];
		}
		MakePolyline(plist_f_l,pline,nump/4);
		for(i = 1; i < static_cast<int>(pline.size()); i++)
		{
			plist[nn++] = pline[i];
		}
		MakePolyline(plist_b_l,pline,nump/4);
		for(i = 1; i < static_cast<int>(pline.size()); i++)
		{
			plist[nn++] = pline[i];
		}
		MakePolyline(plist_b_r,pline,nump/4);
		for(i = 1; i < static_cast<int>(pline.size()-1); i++)
		{
			plist[nn++] = pline[i];
		}
	}

}

//---------------------------------------------------------------
// Name:	    DevideLoopByDis()
// Description: Divide the inputed loop by distance equivalent in left-front-right-back 4 parts 
// Argument:    ptsIn:-- point list in original loop, should be closed, and is on x-z plane
//         :	 plist:-- point list in the new loop;    vtCen:-- loop center
//         :    numpArr[0]:--right-front points number; [1] front-left points number;
//         :           [2]:--left-back points number;   [3] back--right points number
//         :    bIsAnticlockwise:--whether the inputed polygon points is in anticlockwise direction
// Return:		
// Author:	XXX XXX
// Date:	2006/04/29 29:4:2006   14:55
// Update:	 
// Author:	
// Date: 
// copyright: XXX. developed by XXX
//----------------------------------------------------------------
void DivideLoopByDis(const varray<Vec4>& ptsIn, varray<Vec4>& plist,Vec4 vtCen,const varray<int>& numpArr,bool bIsAnticlockwise)
{
	//precondition
	if(ptsIn.size() < 4 || numpArr.size() < 4)
	{
		return;
	}
	int i;
	int npt = static_cast<int>(ptsIn.size());
	//check the loop direction
	Vec4 vt0 = ptsIn[0];
	Vec4 vt1 = ptsIn[npt/4];
	Vec4 vt2 = ptsIn[npt/2];

	varray<Vec4> ptsInTmp = ptsIn;
	varray<Vec4> ptsInBak = ptsIn;
	if((ptsInTmp[0] - ptsInTmp.back()).Magnitude() < 0.001)
	{
		ptsInTmp.pop_back();
		ptsInBak.pop_back();
	}
	//	 if(CrossVecX((vt2 - vt0),(vt1-vt0)).y < 0)
	if(bIsAnticlockwise)
	{
		int iSize = static_cast<int>(ptsInBak.size());
		for(i = 1; i < static_cast<int>(ptsInBak.size()); i++)
		{
			ptsInTmp[i] = ptsInBak.at(iSize-i);
		}
	}
	Vec4 vt = ptsInTmp[0];
	ptsInTmp.push_back(vt);
	// divide the loop into four segment based on the center of the loop
	Vec4 p1, p_f, p_b, p_l, p_r,tempv;
	int n_f = -1, n_b = -1, n_l = -1, n_r = -1;

	for(i=1; i < static_cast<int>(ptsInTmp.size()); i++)
	{
		if((ptsInTmp[i]-ptsInTmp[i-1]).Magnitude() <= ERR) 
		{
			continue;
		}
		if( (ptsInTmp[i-1].x >= vtCen.x && ptsInTmp[i].x <= vtCen.x) || (ptsInTmp[i-1].x <= vtCen.x && ptsInTmp[i].x >= vtCen.x))
		{
			if(ptsInTmp[i].z > vtCen.z)
			{
				tempv = ptsInTmp[i-1] +(ptsInTmp[i] - ptsInTmp[i-1])*(vtCen.x - ptsInTmp[i-1].x)/(ptsInTmp[i].x - ptsInTmp[i-1].x);
				if(n_f!=-1)
				{
					if(p_f.z < tempv.z)
					{
						p_f = tempv;
						n_f = i;
					}
				}
				else
				{
					p_f = tempv;
					n_f = i;
				}
			}
			else
			{
				tempv = ptsInTmp[i-1] +(ptsInTmp[i] - ptsInTmp[i-1])*(vtCen.x - ptsInTmp[i-1].x)/(ptsInTmp[i].x - ptsInTmp[i-1].x);
				if(n_b!=-1)
				{
					if(p_f.z > tempv.z)
					{
						p_b = tempv; 
						n_b = i;
					}
				}
				else
				{ 
					p_b = tempv; 
					n_b = i;
				}
			}
		}

		if( (ptsInTmp[i-1].z >= vtCen.z && ptsInTmp[i].z <= vtCen.z) || (ptsInTmp[i-1].z <= vtCen.z && ptsInTmp[i].z >= vtCen.z))
		{
			if(ptsInTmp[i].x > vtCen.x)
			{
				tempv = ptsInTmp[i-1] +(ptsInTmp[i] - ptsInTmp[i-1])*(vtCen.z - ptsInTmp[i-1].z)/(ptsInTmp[i].z - ptsInTmp[i-1].z);
				if(n_r!=-1)
				{
					if(p_r.x < tempv.x)
					{
						p_r = tempv; 
						n_r = i;
					}
				}
				else
				{ 
					p_r = tempv; 
					n_r = i;
				}
			}
			else
			{
				tempv = ptsInTmp[i-1] +(ptsInTmp[i] - ptsInTmp[i-1])*(vtCen.z - ptsInTmp[i-1].z)/(ptsInTmp[i].z - ptsInTmp[i-1].z);
				if(n_l!=-1)
				{
					if(p_l.x > tempv.x)
					{
						p_l = tempv; 
						n_l = i;
					}
				}
				else
				{ 
					p_l = tempv; 
					n_l = i;
				}
			}
		}
	}

	if(n_l < 0 || n_r < 0 || n_f < 0 || n_b <0)
	{
		return;
	}
	int npt_f_r=0, npt_f_l=0, npt_b_r=0, npt_b_l=0;
	int nn=0;

	varray<Vec4> plist_f_r, plist_f_l, plist_b_r, plist_b_l;

	// for the front-right segment
	nn=1;
	if(n_f > n_r)
	{
		npt_f_r = n_f - n_r + 2;
		plist_f_r.resize(npt_f_r);

		plist_f_r[0] = p_r;
		plist_f_r[npt_f_r-1] = p_f;
		for(i=n_r; i<n_f; i++)
		{
			plist_f_r[nn++] = ptsInTmp[i];
		}
	}
	else
	{
		npt_f_r = n_f + (npt-n_r)+2;

		plist_f_r.resize(npt_f_r);

		plist_f_r[0] = p_r;
		plist_f_r[npt_f_r-1] = p_f;

		for(i=n_r; i<npt; i++)
		{
			plist_f_r[nn++] = ptsInTmp[i];
		}
		for(i=0; i<n_f; i++)
		{
			plist_f_r[nn++] = ptsInTmp[i];
		}
	}

	////
	// for the front-left segment
	nn=1;
	if(n_l>n_f)
	{
		npt_f_l = n_l - n_f + 2;

		plist_f_l.resize(npt_f_l);

		plist_f_l[0] = p_f;
		plist_f_l[npt_f_l-1] = p_l;

		for(i=n_f; i<n_l; i++)
		{
			plist_f_l[nn++] = ptsInTmp[i];
		}
	}
	else
	{
		npt_f_l = n_l + (npt-n_f)+2;

		plist_f_l.resize(npt_f_l);

		plist_f_l[0] = p_f;
		plist_f_l[npt_f_l-1] = p_l;

		for(i=n_f; i<npt; i++)
		{
			plist_f_l[nn++] = ptsInTmp[i];
		}

		for(i=0; i<n_l; i++)
		{
			plist_f_l[nn++] = ptsInTmp[i];
		}
	}

	// for the back-left segment
	nn=1;
	if(n_b>n_l)
	{
		npt_b_l = n_b - n_l + 2;

		plist_b_l.resize(npt_b_l);

		plist_b_l[0] = p_l;
		plist_b_l[npt_b_l-1] = p_b;

		for(i=n_l; i<n_b; i++)
		{
			plist_b_l[nn++] = ptsInTmp[i];
		}
	}
	else
	{
		npt_b_l = n_b + (npt-n_l)+2;

		plist_b_l.resize(npt_b_l);

		plist_b_l[0] = p_l;
		plist_b_l[npt_b_l-1] = p_b;

		for(i=n_l; i<npt; i++)
		{
			plist_b_l[nn++] = ptsInTmp[i];
		}

		for(i=0; i<n_b; i++)
		{
			plist_b_l[nn++] = ptsInTmp[i];
		}
	}

	// for the back-right segment
	nn=1;
	if(n_r>n_b)
	{
		npt_b_r = n_r - n_b + 2;

		plist_b_r.resize(npt_b_r);

		plist_b_r[0] = p_b;
		plist_b_r[npt_b_r-1] = p_r;

		for(i=n_b; i<n_r; i++)
		{
			plist_b_r[nn++] = ptsInTmp[i];
		}
	}
	else
	{
		npt_b_r = n_r + (npt-n_b)+2;

		plist_b_r.resize(npt_b_r);

		plist_b_r[0] = p_b;
		plist_b_r[npt_b_r-1] = p_r;

		for(i=n_b; i<npt; i++)
		{
			plist_b_r[nn++] = ptsInTmp[i];
		}

		for(i=0; i<n_r; i++)
		{
			plist_b_r[nn++] = ptsInTmp[i];
		}
	}

	// divide each segment to a polyline whose points have equal length
	varray<Vec4> pline;
	plist.clear();
	nn=0;
	MakePolyline(plist_f_r,pline,numpArr[0]);
	for(i = 0; i < static_cast<int>(pline.size()); i++)
	{
		plist.push_back(pline[i]);
	}
	MakePolyline(plist_f_l,pline,numpArr[1]);
	for(i = 1; i < static_cast<int>(pline.size()); i++)
	{
		plist.push_back(pline[i]);
	}
	MakePolyline(plist_b_l,pline,numpArr[2]);
	for(i = 1; i < static_cast<int>(pline.size()); i++)
	{
		plist.push_back(pline[i]);
	}
	MakePolyline(plist_b_r,pline,numpArr[3]);
	for(i = 1; i < static_cast<int>(pline.size()-1); i++)
	{
		plist.push_back(pline[i]);
	}
}


//---------------------------------------------------------------
// Name:	    MakePolylineByOneDirection()
// Description: polyline a line with the given direction coordinate equal
// Argument:    vtsIn:--inputed point
//         :	iSegNum: the segment number to polyline; iDir: 0:x-direction,1:--y,2:--z	
// Return:		
// Author:	    XXX XXX
// Date:	    2006/04/29 29:4:2006   15:19
// Update:	 
// Author:	
// Date: 
// copyright: XXX. developed by XXX
//----------------------------------------------------------------
void MakePolylineByOneDirection(const varray<Vec4> vtsIn, varray<Vec4>& vtsOut,int iSegNum,int iDir)
{
	if(vtsIn.size() == 0)
	{
		return;
	}

	int i = 0;
	Vec4 vt = Vec4(0,0,0);
	bool bIsInterpolated = false;

	vtsOut.resize(iSegNum+1);

	if(iDir == 0)
	{
		vtsOut[0] = vtsIn[0];
		for(i = 1; i < iSegNum; i++)
		{
			vt.x = vtsIn[0].x + (vtsIn.back().x - vtsIn[0].x) * i/iSegNum;
			vtsOut[i] = GetInterPolatePt(vtsIn,vt,0,bIsInterpolated);
		}
		vtsOut[iSegNum] = vtsIn.back();
	}
	else if(iDir == 1)
	{
		vtsOut[0] = vtsIn[0];
		for(i = 1; i < iSegNum; i++)
		{
			vt.y = vtsIn[0].y + (vtsIn.back().y - vtsIn[0].y) * i/iSegNum;
			vtsOut[i] = GetInterPolatePt(vtsIn,vt,1,bIsInterpolated);
		}
		vtsOut[iSegNum] = vtsIn.back();
	}
	else if( iDir == 2)
	{
		vtsOut[0] = vtsIn[0];
		for(i = 1; i < iSegNum; i++)
		{
			vt.z = vtsIn[0].z + (vtsIn.back().z - vtsIn[0].z) * i/iSegNum;
			vtsOut[i] = GetInterPolatePt(vtsIn,vt,2,bIsInterpolated);
		}
		vtsOut[iSegNum] = vtsIn.back();
	}
}
//---------------------------------------------------------------
// Name:	    IsSelfIntersectForSpln()
// Description: Judge whether the inputed spline will self-intersect
//           : the function only check for 2d situation, for every segment of spline check if intersect with other segment.
// Argument:    iMode: 0:--check in x-y direction, 1: x-z direction 2: z-y direction
//         :	
// Return:		
// Author:	    XXX XXX
// Date:	    2006/04/29 29:4:2006   15:23
// Update:	 
// Author:	
// Date: 
// copyright: XXX. developed by XXX
//----------------------------------------------------------------
bool IsSelfIntersectForSpln(Spline& spl, int iMode)
{
	try
	{
		varray<Vec4> varV3SplVts;
		varray<Vec4> varV2SplVts;
		int nump = 0;
		int i = 0;

		SplineToPolyline(spl,varV3SplVts);
		nump = static_cast<int>(varV3SplVts.size());
		varV2SplVts.resize(nump);
		if(iMode == 0)				//x-y
		{
			for(i = 0; i < nump; i++)
			{
				varV2SplVts[i] = Vec4(varV3SplVts[i].x,varV3SplVts[i].y,0);
			}
		}
		else if(iMode == 1)		//x-z
		{
			for(i = 0; i < nump; i++)
			{
				varV2SplVts[i] = Vec4(varV3SplVts[i].x,varV3SplVts[i].z,0);
			}
		}
		else if(iMode == 2)		//z-y
		{
			for(i = 0; i < nump; i++)
			{
				varV2SplVts[i] = Vec4(varV3SplVts[i].z,varV3SplVts[i].y,0);
			}
		}
		else
		{
			assert((iMode >= 0) && (iMode <= 2) );
			return true;
		}


		int tmpNum = nump;
		for(i = 0; i < nump - 1; i++)
		{
			tmpNum = nump;
			if (i == 0 && spl.GetMode() == Spline::SPLMODE_CLOSED_SPLINE)
			{
				tmpNum = nump - 1;
			}

			for(int k = i+2; k < tmpNum-1; k++)
			{
				//---------------------------------------------------
				//enclosing box check
				float minX,minY, maxX, maxY;
				minX = min(varV2SplVts[i].x, varV2SplVts[i+1].x);	minY = min(varV2SplVts[i].y, varV2SplVts[i+1].y);		
				maxX = max(varV2SplVts[i].x, varV2SplVts[i+1].x); maxY = max(varV2SplVts[i].y, varV2SplVts[i+1].y);
				bool flag1 = (varV2SplVts[k].x > maxX && varV2SplVts[k+1].x > maxX);
				bool flag2 = (varV2SplVts[k].x < minX && varV2SplVts[k+1].x < minX);
				bool flag3 = (varV2SplVts[k].y > maxY && varV2SplVts[k+1].y > maxY);
				bool flag4 = (varV2SplVts[k].y < minY && varV2SplVts[k+1].y < minY);
				if( flag1 || flag2 || flag3 || flag4)
				{
					continue;
				}
				//---------------------------------------------------
				//		 if (IsSegmentsIntersect(varV2SplVts[i], varV2SplVts[i+1], varV2SplVts[k], varV2SplVts[k+1])) //	line segment intersection check.
				if(IsTwoLineIntersectIn2d(varV2SplVts[i], varV2SplVts[i+1], varV2SplVts[k], varV2SplVts[k+1]))		
				{				
					return true;	 
				}
			}
		}
		return false;
	}
	catch (...)
	{
		return true;
	}
}
//---------------------------------------------------------------
// Name:	    IsTwoCurveIntersect() 
// Description: Judge whether two curve intersect with each other
// Argument:    iMode: 0:--check in x-y direction, 1: x-z direction 2: z-y direction
//         :	vtsArr1,vtsArr2:-- two curve points array
// Return:		
// Author:		XXX XXX
// Date:		 
// Modified by:	XXX XXX
// Updated date: 2005/12/05 5:12:2005   11:14	
//----------------------------------------------------------------
bool IsTwoCurveIntersect(const varray<Vec4>&vtsArr1,const varray<Vec4>&vtsArr2,int iMode)
{
	try
	{
		if(iMode < 0 || iMode > 2)
		{
			return true;
		}

		int i = 0, j = 0;
		varray<Vec4> vts2DArr1,vts2DArr2;
		vts2DArr1.resize(vtsArr1.size());
		vts2DArr2.resize(vtsArr2.size());
		if(iMode == 0)
		{
			for(i = 0; i < static_cast<int>(vtsArr1.size()); i++)
			{
				vts2DArr1[i] = Vec4(vtsArr1[i].x,vtsArr1[i].y,0);
			}
			for(i = 0; i < static_cast<int>(vtsArr2.size()); i++)
			{
				vts2DArr2[i] = Vec4(vtsArr2[i].x,vtsArr2[i].y,0);
			}
		}
		else if(iMode == 1)
		{
			for(i = 0; i < static_cast<int>(vtsArr1.size()); i++)
			{
				vts2DArr1[i] = Vec4(vtsArr1[i].x,vtsArr1[i].z,0);
			}
			for(i = 0; i < static_cast<int>(vtsArr2.size()); i++)
			{
				vts2DArr2[i] = Vec4(vtsArr2[i].x,vtsArr2[i].z,0);
			}
		}
		else if(iMode == 2)
		{
			for(i = 0; i < static_cast<int>(vtsArr1.size()); i++)
			{
				vts2DArr1[i] = Vec4(vtsArr1[i].z,vtsArr1[i].y,0);
			}
			for(i = 0; i < static_cast<int>(vtsArr2.size()); i++)
			{
				vts2DArr2[i] = Vec4(vtsArr2[i].z,vtsArr2[i].y,0);
			}
		}
		else
		{
			return true;
		}

		int iSize1 = static_cast<int>(vtsArr1.size()-1);
		int iSize2 = static_cast<int>(vtsArr2.size()-1);
		float minX = 0,minY = 0, maxX = 0, maxY = 0;

		for(i = 0; i < iSize1; i++)
		{
			const Vec4& vt1 = vts2DArr1[i];
			const Vec4& vt2 = vts2DArr1[i+1];
			for(j = 0; j < iSize2; j++)
			{
				const Vec4& pt1 = vts2DArr2[j];
				const Vec4& pt2 = vts2DArr2[j+1];
				//---------------------------------------------------
				//enclosing box check
				minX = min(vt1.x, vt2.x);	minY = min(vt1.y, vt2.y);		
				maxX = max(vt1.x, vt2.x);   maxY = max(vt1.y, vt2.y);
				bool flag1 = (pt1.x > maxX && pt2.x > maxX);
				bool flag2 = (pt1.x < minX && pt2.x < minX);
				bool flag3 = (vt1.y > maxY && vt2.y > maxY);
				bool flag4 = (vt1.y < minY && vt2.y < minY);
				if( flag1 || flag2 || flag3 || flag4)
				{
					continue;
				}
				if(IsTwoLineIntersectIn2d(vt1,vt2,pt1,pt2))		
				{				
					if(i == iSize1 - 1 && j == iSize2 - 1)
					{
						return false;
					}
					else
					{
						return true;
					} 
				}
			}
		}

		return false;
	}
	catch (...)
	{
		return true;
	}
}
//---------------------------------------------------------------
// Name:	    MovePointToTrangle()
// Description: if a point is on the edge of a triangle,move it into the triangle	
// Argument:    ptTemp:--point on the edge of a triangle;
//         :	ptTemp0,ptTemp1,ptTemp2:--three vertices of the specified triangle
// Return:		
// Author:	    XXX XXX
// Date:	    2006/04/29 29:4:2006   15:32
// Update:	 
// Author:	
// Date: 
// copyright: XXX. developed by XXX
//----------------------------------------------------------------
void MovePointToTrangle(Vec4& ptTemp, Vec4 ptTemp0,Vec4 ptTemp1,Vec4 ptTemp2)
{
	Vec4 ptEverage;
	if (fabs((ptTemp-ptTemp0).Magnitude() + (ptTemp1-ptTemp).Magnitude() - (ptTemp1-ptTemp0).Magnitude())<1.0e-3||
		fabs((ptTemp-ptTemp1).Magnitude() + (ptTemp2-ptTemp).Magnitude() - (ptTemp2-ptTemp1).Magnitude())<1.0e-3||
		fabs((ptTemp-ptTemp2).Magnitude() + (ptTemp0-ptTemp).Magnitude() - (ptTemp0-ptTemp2).Magnitude())<1.0e-3
		)
	{
		//	ptEverage = (ptTemp0 + ptTemp1 + ptTemp2) / 3;
		ptEverage = GetTwoLineInterPt(ptTemp0,(ptTemp1+ptTemp2)/2,ptTemp1,(ptTemp2+ptTemp0)/2);
		ptEverage -= ptTemp;
		// modified by XXX [3/8/2006] 
		if(ptEverage.Magnitude() >= 1.0)
		{
			ptTemp += ptEverage.Normalize() * (float)0.01;
		}
		else
		{
			ptTemp += ptEverage*(float)0.01;
		}
	}

}
//---------------------------------------------------------------
// Name:	    IsTwoPtsArrSameAtYRange()
// Description: Judge whether two points array have same coordinate at given y-direction range
// Argument:    vtsArr1,vtsArr2:--points array  ;  ySt,yEnd:-- y direction range;
//         :	iMode:--0: compare x,y coordinate, 1: compare z,y coordinate; 
// Return:		bool:-- true,if two points array are same
// Author:		XXX XXX
// Date:		 
// Modified by:	XXX XXX
// Updated date: 2005/10/12 12:10:2005   10:40	
//----------------------------------------------------------------
bool IsTwoPtsArrSameAtYRange(const varray<Vec4>& vtsArr1,const varray<Vec4>& vtsArr2, float ySt,float yEnd,int iMode)
{
	try
	{
		if(vtsArr1.size() != vtsArr2.size())
		{
			return false;
		}
		int i = 0;
		bool bRet = true,bIsInterpolated = false;
		Vec4 vtSt,vtEnd;
		Vec4 vtSt1,vtEnd1,vtSt2,vtEnd2;
		if(ySt < yEnd)
		{
			std::swap(ySt,yEnd);
		}
		vtSt.Set(0,ySt,0);
		vtEnd.Set(0,yEnd,0);

		if(iMode == 0)
		{
			for(i = 0; i < static_cast<int>(vtsArr1.size()); i++)
			{
				if(vtsArr1[i].y > ySt || vtsArr1[i].y < yEnd)
				{
					continue;
				}
				if(fabs(vtsArr1[i].x - vtsArr2[i].x) > 0.01 || fabs(vtsArr1[i].y - vtsArr2[i].y) > 0.01)
				{
					return false;
				}
			}
			vtSt1 = GetInterPolatePt(vtsArr1,vtSt,1,bIsInterpolated);
			vtSt2 = GetInterPolatePt(vtsArr2,vtSt,1,bIsInterpolated);
			if(fabs(vtSt1.x - vtSt2.x) > 0.01) 
			{
				return false;
			}
			vtEnd1 = GetInterPolatePt(vtsArr1,vtEnd,1,bIsInterpolated);
			vtEnd2 = GetInterPolatePt(vtsArr2,vtEnd,1,bIsInterpolated);
			if(fabs(vtEnd1.x - vtEnd2.x) > 0.01)
			{
				return false;
			}
		}
		else if(iMode == 1)
		{
			for(i = 0; i < static_cast<int>(vtsArr1.size()); i++)
			{
				if(vtsArr1[i].y > ySt || vtsArr1[i].y < yEnd)
				{
					continue;
				}
				if(fabs(vtsArr1[i].z - vtsArr2[i].z) > 0.01 || fabs(vtsArr1[i].y - vtsArr2[i].y) > 0.01)
				{
					return false;
				}
			}
			vtSt1 = GetInterPolatePt(vtsArr1,vtSt,1,bIsInterpolated);
			vtSt2 = GetInterPolatePt(vtsArr2,vtSt,1,bIsInterpolated);
			if(fabs(vtSt1.z - vtSt2.z) > 0.01) 
			{
				return false;
			}
			vtEnd1 = GetInterPolatePt(vtsArr1,vtEnd,1,bIsInterpolated);
			vtEnd2 = GetInterPolatePt(vtsArr2,vtEnd,1,bIsInterpolated);
			if(fabs(vtEnd1.z - vtEnd2.z) > 0.01)
			{
				return false;
			}
		}
		return bRet;
	}
	catch (...) 
	{
		return false;
	}
}
//---------------------------------------------------------------
// Name:	    FindSplIdxAtGivenYValue()
// Description: Find the point index in the given spline which is very near to the given y coordinate
// Argument:    spl:-- given spline; yRef:-- given y coordinate
//         :	 idxNotToFind:-- points index not need to be searched
// Return:		int :-- point index find;
// Author:		XXX XXX
// Date:		 
// Modified by:	XXX XXX
// Updated date: 2005/10/26 26:10:2005   10:30	
//----------------------------------------------------------------
int FindSplIdxAtGivenYValue(const Spline& spl,float yRef, int idxNotToFind /*= -1*/)
{
	int i = 0, iRet = -1;

	for(i = 0; i < spl.size(); i++)
	{
		if(fabs(spl.p(i).y - yRef) < 0.01)
		{
			if(i != idxNotToFind)
			{
				iRet = i;
				break;
			}
		}
	}
	if(iRet == -1)
	{
		float fDis = 1.0e6;
		for(i = 0; i < spl.size(); i++)
		{
			if(fabs(spl.p(i).y - yRef) < fDis)
			{
				if(i != idxNotToFind)
				{
					iRet = i;
					fDis = fabs(spl.p(i).y - yRef);
				}
			}
		}
	}
	return iRet;
}
//---------------------------------------------------------------
// Name:	    SymmetricSpline()
// Description: Make a spline (in x-z plane) symmetric,the spline 
//              points shoulder be sorted by right-front--left-back-right direction  
// Argument:   
//         :	bIsLeftAsRef:--true,take the left as symmetric base, that is the left points  are kept unchanged
// Return:		
// Author:		XXX XXX
// Date:		 
// Modified by:	XXX XXX
// Updated date: 2005/10/31 30:9:2005   14:24	
//----------------------------------------------------------------
void SymmetricTrouSpline(Spline& spl,Vec4 vtCen,int iLeftIdx, int iBackIdx,int iFrontIdx, bool bIsLeftAsRef)
{
	if(iBackIdx < 0  || iFrontIdx < 0 || spl.size() < max(iBackIdx,iFrontIdx) )
	{
		return;
	}
	Vec4 vtL = spl.p(iLeftIdx);
	Vec4 vtR = spl.p(0);
	int i = 0;
	varray<Vec4> ptsArr;
	Vec4 ptTmp;
	float scl = 0;
	if(!bIsLeftAsRef)
	{
		for(i = 0; i <= iFrontIdx; i++)
		{
			ptsArr.push_back(spl.p(i));
		}
		for(i = iFrontIdx-1; i >= 0; i--)
		{
			scl = (spl.p(i).x - vtCen.x)/(vtR.x - vtCen.x);
			ptTmp = spl.p(i);
			ptTmp.x = vtCen.x * (1 - scl) + vtL.x * scl;
			//ptTmp = Vec4(2 * vtCen.x - ptTmp.x,ptTmp.y,ptTmp.z);
			ptsArr.push_back(ptTmp);
		}
		for(i = spl.size()-1 ; i > iBackIdx; i--)
		{
			scl = (spl.p(i).x - vtCen.x)/(vtR.x - vtCen.x);
			ptTmp = spl.p(i);
			ptTmp.x = vtCen.x * (1 - scl) + vtL.x * scl;
			//ptTmp = Vec4(2 * vtCen.x - ptTmp.x,ptTmp.y,ptTmp.z);
			ptsArr.push_back(ptTmp);
		}
		for(i = iBackIdx; i < spl.size(); i++)
		{
			ptsArr.push_back(spl.p(i));
		}
	}
	else
	{
		for(i = iLeftIdx; i > iFrontIdx; i--)
		{
			scl = (spl.p(i).x - vtCen.x)/(vtL.x - vtCen.x);
			ptTmp = spl.p(i);
			ptTmp.x = vtCen.x * (1 - scl) + vtR.x * scl;
			//	ptTmp = Vec4(2 * vtCen.x -ptTmp.x,ptTmp.y,ptTmp.z);
			ptsArr.push_back(ptTmp);
		}
		for(i = iFrontIdx; i <= iBackIdx; i++)
		{
			ptsArr.push_back(spl.p(i));
		}
		for(i = iBackIdx-1 ; i > iLeftIdx; i--)
		{
			scl = (spl.p(i).x - vtCen.x)/(vtL.x - vtCen.x);
			ptTmp = spl.p(i);
			ptTmp.x = vtCen.x * (1 - scl) + vtR.x * scl;
			//	ptTmp = Vec4(2 * vtCen.x -ptTmp.x,ptTmp.y,ptTmp.z);
			ptsArr.push_back(ptTmp);	
		}
	}
	spl.ClearCtrlPoint();
	for(i = 0; i < static_cast<int>(ptsArr.size()); i++)
	{
		spl.AddCtrlPoint(ptsArr[i]);
	}
}
//---------------------------------------------------------------
// Name:	    OffsetLine()
// Description: Offset the input line a little
// Argument:    ptsArr:--Points array to be offseted, should be a horizontal array 	
//         :	fDis:-- offset distance
// Return:		
// Author:	    XXX XXX
// Date:	    2006/04/29 29:4:2006   15:39
// Update:	 
// Author:	
// Date: 
// copyright: XXX. developed by XXX
//----------------------------------------------------------------
void OffsetLine(varray<Vec4>& ptsArr,float fDis)
{
	if(ptsArr.size() == 0)
	{
		return;
	}

	Vec4 ptCen,ptAdd1,ptAdd2,ptOff,pt1,pt2,pt3;

	if((ptsArr.back() - ptsArr.front()).Magnitude()<0.0005)
	{
		ptsArr.pop_back();
	}
	int i = 0;
	int iSize = static_cast<int>(ptsArr.size());

	for(i = 0; i < static_cast<int>(ptsArr.size()); i++)
	{
		ptCen += ptsArr[i];
	}

	ptCen += ptsArr[0];

	ptCen = ptCen / (float)(iSize + 1);

	varray<Vec4> ptsArrTmp = ptsArr;
	for(i = 0; i < static_cast<int>(ptsArr.size()); i++)
	{
		pt1 = ptsArr[i];
		pt2 = ptsArr[(i+1) % iSize];
		pt3 = ptsArr[(i-1 + iSize) % iSize];		
		pt1.y = pt2.y = pt3.y =  ptsArr[0].y;

		ptAdd1 = pt2 - ptCen;
		ptAdd2 = pt3 - ptCen;

		ptOff = ptAdd1.Normalize() + ptAdd2.Normalize();
		ptOff.y = 0;

		ptOff = ptOff.Normalize() * fDis;

		if(Dot(ptOff,pt1) < 0) 
		{
			ptOff = Vec4(0,0,0) - ptOff;
		}
		ptsArrTmp[i] += Vec4(ptOff.x,0,ptOff.z); 
	}
	ptsArr = ptsArrTmp;
	ptsArr.push_back(ptsArr[0]);
}
//---------------------------------------------------------------
// Name:	    DivideSegmentsToTwoByLenScl()
// Description: Divide one segments into two by the given length scl;
// Argument:    vtsList:--inputed segments points array; 
//         :	vtsList1,vtsList2:--divided segments points array
//         :    fScl:-- length scale
// Return:		
// Author:	XXX XXX
// Date:	2006/03/10 10:3:2006   16:43
// Update:	 
// Author:	
// Date: 
// copyright: XXX. developed by XXX
//----------------------------------------------------------------
bool DivideSegmentsToTwoByLenScl(const varray<Vec4>&vtsList,varray<Vec4>& vtsList1,varray<Vec4>& vtsList2,float fScl)
{
	vtsList1.clear();
	vtsList2.clear();
	const float FMINERR = (float)1.0e-3;
	if(fabs(fScl - 0) < FMINERR || fScl < 0)
	{
		vtsList2 = vtsList;
	}
	else if(fabs(fScl-1) < FMINERR || fScl > 1)
	{
		vtsList1 = vtsList;
	}
	else 
	{
		if(vtsList.size() <= 2)
		{
			vtsList1 = vtsList2 = vtsList;
			return false;
		}
		int i = 0;
		float fLenSum = 0, fLen = 0;
		for(i = 0; i < static_cast<int>(vtsList.size()-1); i++)
		{
			fLenSum += (vtsList[i] - vtsList[i+1]).Magnitude();
		}
		fLen = fLenSum * fScl;
		//insert a point by given the scale,and record the index of segment that  the point inserted 
		Vec4 vtInsert;
		int idx = -1;
		float fLenSum1 = 0, fScl1 = 0;
		for(i = 0; i < static_cast<int>(vtsList.size()-1); i++)
		{
			fLenSum1 += (vtsList[i+1] - vtsList[i]).Magnitude();
			if(fLenSum1 > fLen)
			{
				idx = i;
				fScl1 = 1 - (fLenSum1 - fLen)/(vtsList[i+1] - vtsList[i]).Magnitude();
				vtInsert = vtsList[i] * (1 - fScl1) + vtsList[i+1] * fScl1;
				break;
			}
		}
		if(idx == -1)
		{
			assert(idx == -1);
			return false;
		}
		for(i = 0; i <= idx; i++)
		{
			vtsList1.push_back(vtsList[i]);
		}
		vtsList1.push_back(vtInsert);
		vtsList2.push_back(vtInsert);
		for(i = idx+1; i < static_cast<int>(vtsList.size()); i++)
		{
			vtsList2.push_back(vtsList[i]);
		}
	}
	return true;
}
//---------------------------------------------------------------
// Name:	    DivideSegmentsToTwoByMidX()
// Description: Devide 1 segments into 2 segments by given its mid x value
// Argument:    vtsList:-- segments input, should be increase or decrease in x-direction
//         :	
// Return:		
// Author:	XXX XXX
// Date:	2006/03/13 13:3:2006   11:30
// Update:	 
// Author:	
// Date: 
// copyright: XXX. developed by XXX
//----------------------------------------------------------------
bool DivideSegmentsToTwoByMidX(const varray<Vec4>&vtsList,varray<Vec4>& vtsList1,varray<Vec4>& vtsList2,float fXMid)
{
	//status clear
	vtsList1.clear();
	vtsList2.clear();
	//precondition
	if(vtsList.size() < 2 || fXMid > max(vtsList[0].x,vtsList.back().x) || fXMid < min(vtsList[0].x,vtsList.back().x))
	{
		return false;
	}

	//insert a point at the given x middle value
	int i = 0 , idx = -1;
	Vec4 vtInsert;
	float fScl = 0;
	const float fERR = (float)1.0e-4;
	for(i = 0; i < static_cast<int>(vtsList.size() - 1); i++)
	{
		const Vec4& vt1 = vtsList[i];
		const Vec4& vt2 = vtsList[i+1];
		if((vt1.x <= fXMid && vt2.x > fXMid)|| (vt1.x > fXMid && vt2.x <= fXMid))
		{
			idx = i;
			if(fabs(vt1.x - vt2.x) < fERR)
			{
				vtInsert = vt1;
			}
			else
			{
				fScl = (fXMid - vt1.x)/(vt2.x - vt1.x);
				vtInsert = vt1 * (1 - fScl) + vt2 * fScl;
			}
			break;
		}
	}
	assert(idx >= 0);
	if(idx == -1)
	{
		return false;
	}
	for(i = 0; i <= idx; i++)
	{
		vtsList1.push_back(vtsList[i]);
	}
	vtsList1.push_back(vtInsert);
	vtsList2.push_back(vtInsert);
	for(i = idx+1; i < static_cast<int>(vtsList.size()); i++)
	{
		vtsList2.push_back(vtsList[i]);
	}
	return true;
}
//---------------------------------------------------------------
// Name:	    DivideSegmentsToTwoByMidX()
// Description: Devide 1 segments into 2 segments by given a point to specify its mid x value
// Argument:    vtsList:-- segments input, should be increase or decrease in x-direction
//         :	
// Return:		
// Author:	XXX XXX
// Date:	2006/03/13 13:3:2006   11:30
// Update:	 
// Author:	
// Date: 
// copyright: XXX. developed by XXX
//---------------------------------------------------------------
bool DivideSegmentsToTwoByMidX(const varray<Vec4>&vtsList,varray<Vec4>& vtsList1,varray<Vec4>& vtsList2,Vec4 vtMid)
{
	//status clear
	vtsList1.clear();
	vtsList2.clear();
	float fXMid = vtMid.x;
	//precondition
	if(vtsList.size() < 2 || fXMid > max(vtsList[0].x,vtsList.back().x) || fXMid < min(vtsList[0].x,vtsList.back().x))
	{
		return false;
	}
	//insert a point at the given x middle value
	int i = 0 , idx = -1;
	varray<int> idxArr;
	Vec4 vtInsert;
	varray<Vec4> vtInsertArr;
	float fScl = 0;
	const float fERR = (float)1.0e-4;
	for(i = 0; i < static_cast<int>(vtsList.size() - 1); i++)
	{
		const Vec4& vt1 = vtsList[i];
		const Vec4& vt2 = vtsList[i+1];
		if((vt1.x <= fXMid && vt2.x > fXMid)|| (vt1.x > fXMid && vt2.x <= fXMid))
		{
			idx = i;
			if(fabs(vt1.x - vt2.x) < fERR)
			{
				vtInsert = vt1;
			}
			else
			{
				fScl = (fXMid - vt1.x)/(vt2.x - vt1.x);
				vtInsert = vt1 * (1 - fScl) + vt2 * fScl;
			}
			idxArr.push_back(idx);
			vtInsertArr.push_back(vtInsert);
		}
	}
	assert(idxArr.size() > 0);
	if(idxArr.size() == 0)
	{
		return false;
	}
	//if there are more than one point at the mid x position,select a point nearest to the given middle point

	if(idxArr.size() > 1)
	{
		float fDis = 1.0e6;
		for(i = 0; i < static_cast<int>(vtInsertArr.size()); i++)
		{
			if(fabs(vtInsertArr[i].z - vtMid.z) < fDis)
			{
				fDis = fabs(vtInsertArr[i].z - vtMid.z);
				idx = idxArr[i];
				vtInsert = vtInsertArr[i];
			}
		}
	}

	for(i = 0; i <= idx; i++)
	{
		vtsList1.push_back(vtsList[i]);
	}
	vtsList1.push_back(vtInsert);
	vtsList2.push_back(vtInsert);
	for(i = idx+1; i < static_cast<int>(vtsList.size()); i++)
	{
		vtsList2.push_back(vtsList[i]);
	}
	return true;
}
//---------------------------------------------------------------
// Name:	    IsLineMonotonicInYDir()
// Description: Check whether a line is monotonic in y direction
// Argument:    vtsArr:--line points array; 
//         :	bIsIncrease:--whether is increase
// Return:		bool:-- true, if a line is monotonic in y direction
// Author:		XXX XXX
// Date:		 
// Modified by:	XXX XXX
// Updated date: 2005/12/13 13:12:2005   11:49	
//----------------------------------------------------------------
bool IsLineMonotonicInYDir(const varray<Vec4> & vtsArr,bool bIsIncrease)
{
	if(vtsArr.size() == 0)
	{
		return false;
	}
	bool bRet = true;
	int i = 0;
	if(bIsIncrease)
	{
		for(i = 1; i < static_cast<int>(vtsArr.size()); i++)
		{
			if(vtsArr[i].y < vtsArr[i -1].y)
			{
				return false;
			}
		}
	}
	else
	{
		for(i = 1; i < static_cast<int>(vtsArr.size()); i++)
		{
			if(vtsArr[i].y > vtsArr[i -1].y)
			{
				return false;
			}
		}
	}
	return bRet;
}

//---------------------------------------------------------------
// Name:	    IsLineMonotonicInYDir()
// Description: Check whether the inputed line has only one point can be intersected at given y-coordinate 
// Argument:    vtsArr:--line points array;
//         :	yPos:--y corrdinate of check point 
// Return:		
// Author:	    XXX XXX
// Date:	    2006/04/29 29:4:2006   15:55
// Update:	 
// Author:	
// Date: 
// copyright: XXX. developed by XXX
//----------------------------------------------------------------
bool IsLineMonotonicInYDir(const varray<Vec4> & vtsArr,float yPos)
{
	if(vtsArr.size() == 0)
	{
		return false;
	}
	bool bRet = true;
	int i = 0;
	int iSum = 0; //intersect points array at y position

	for(i = 0; i < static_cast<int>(vtsArr.size() - 1); i++)
	{
		if(i == static_cast<int>( vtsArr.size() - 2) )
		{
			if( (vtsArr[i].y >= yPos && vtsArr[i+1].y <= yPos)
				||  (vtsArr[i].y <= yPos && vtsArr[i+1].y >= yPos)
				||  (vtsArr[i].y == yPos && vtsArr[i+1].y == yPos) 
				)	
			{
				iSum++;
			}
		}
		else
		{
			if( (vtsArr[i].y >= yPos && vtsArr[i+1].y < yPos)
				||  (vtsArr[i].y <= yPos && vtsArr[i+1].y > yPos)
				||  (vtsArr[i].y == yPos && vtsArr[i+1].y == yPos) 
				)	
			{
				iSum++;
			}	
		}
	}
	if(iSum >= 2)
	{
		bRet = false;
	}
	return bRet;
}

//---------------------------------------------------------------
// Name:	     ConvStrToCStr()
// Description: convert String to CString
// Argument:    str: String data
// Return:		
// Author:	    
// Date:	    
// Update:	 
// Author:		XXX
// Date:       20060507
// copyright: XXX. developed by XXX
//----------------------------------------------------------------
//string ConvStrToCStr(const String& str)
//{
//	char* buf= (char*)str.getbuf();
//	string cstr(buf);
//	return cstr;
//}
Vec4 GetPointFromLinePtsArrByLength(float fLen,float fLenSum,const varray<Vec4>& vtsList)
{
	Vec4 vtRet = Vec4(0,0,0);
	if(vtsList.size() == 0)
	{
		return vtRet;
	}
	if(fLen <= 0)
	{
		return vtsList[0];
	}
	else if(fLen >= fLenSum)
	{
		return vtsList.back();
	}

	int i = 0/*, j = 0*/;
	int iSize = static_cast<int> (vtsList.size()-1);
	bool bIsPtFind = false;
	float sumlen = 0, scl = 0;
	for(i = 0; i < iSize; i++)
	{	
		sumlen += (vtsList[i+1] - vtsList[i]).Magnitude();	
		if(sumlen > fLen)
		{
			scl = (sumlen-fLen)/(vtsList[i+1] - vtsList[i]).Magnitude();
			vtRet = vtsList[i]*scl+vtsList[i+1]*(1-scl);
			bIsPtFind = true;
			break;
		}	
	}
	if(!bIsPtFind)
	{
		vtRet = vtsList.back();
	}
	return vtRet;
}

//---------------------------------------------------------------
// Name:		DDALineRaster
// Function:	get the 2D line coordinates of the raster shape(DDA method)
// Argument:	p1, p2: two control points of the line segment
// Return:		res: the integer coordinates of the lines, x coordinate and y coordinate are placed one after another.
//				::::::NOTE::::::
//				this function may not works well on the mechine which does not support SSE2 instruction sets!!!!!!!
// Author:		XXX
// Date:		9/21/2006
// Modified by:		XXX(add comments)
// Updated date:	9/21/2006	
//---------------------------------------------------------------- 

void DDALineRaster(int p1x, int p1y, int p2x, int p2y, varray<int>& res)
{
	res.resize(0);
	int length = abs(p1x - p2x);
	if(abs(p2y - p1y) > length)
		length = abs(p2y - p1y);
	if(length == 0)
	{
		res.push_back(p1x);
		res.push_back(p1y);
		return;
	}
	else if(length == 1)
	{
		res.push_back(p1x);
		res.push_back(p1y);
		res.push_back(p2x);
		res.push_back(p2y);
		return;
	}
	//length = floor(length);
	float dx = (p2x - p1x)/(float)length;
	float dy = (p2y - p1y)/(float)length;
	float x = (float)p1x,y = (float)p1y;
	if(dx > 0)
		x += 0.5f*dx;
	else if(dx < 0)
		x -= 0.5f*dx;
	if(dy > 0)
		y += 0.5f*dy;
	else if(dy < 0)
		y -= 0.5f*dy;
	// deal with the head point
	if((int)floor(x) - p1x != 0 || (int)floor(y) - p1y != 0)
	{
		res.push_back(p1x);
		res.push_back(p1y);
	}
	for(int i = 1; i <= length; ++i)
	{
		res.push_back((int)floor(x)); // be careful the instruction floor in different processors.
		res.push_back((int)floor(y));
		x += dx;
		y += dy;
	}
	// deal with the end point
	int* p = res.end();
	--p;
	if(*p - p2y != 0 || *(--p) - p2x != 0)
	{
		res.push_back(p2x);
		res.push_back(p2y);
	}

}

//---------------------------------------------------------------
// Name:		DDALineRaster
// Function:	get the 2D line coordinates of the raster shape(DDA method)
// Argument:	p1, p2: two control points of the line segment
//				duplicatepoints: whether need more points on the lines 
// Return:		res: the integer coordinates of the lines, x coordinate and y coordinate are placed one after another.
//				::::::NOTE::::::
//				this function may not works well on the mechine which does not support SSE2 instruction sets!!!!!!!
// Author:		XXX
// Date:		9/21/2006
// Modified by:		XXX(add comments)
// Updated date:	9/21/2006	
//---------------------------------------------------------------- 
// the input two vertex should have the decimal part to ensure the line right!!!
void DDALineRaster(float p1x, float p1y, float p2x, float p2y, varray<Vec4>& res, bool duplicatepoints/* = false*/, bool onlyduplicatepoints/* = false*/)
{
	res.resize(0);
	int length = abs((int)p1x - (int)p2x);
	if(abs((int)p2y - (int)p1y) > length)
		length = abs((int)p2y - (int)p1y);
	Vec4 p1(p1x,p1y,0);
	Vec4 p2(p2x,p2y,0);
	if(length == 0)
	{
		if(!onlyduplicatepoints)
			res.push_back(p1);
		return;
	}
	else if(length == 1)
	{
		if(!onlyduplicatepoints)
		{
			res.push_back(p1);
			res.push_back(p2);
			return;
		}
	}
	//length = floor(length);
	double dx = (p2x - p1x)/(double)length;
	double dy = (p2y - p1y)/(double)length;
	double x = p1x,y = p1y;

	Vec4 lastpoint((float)x,(float)y,0);
	for(int i = 1; i <= length; ++i)
	{
		p1.x = (float)x;
		p1.y = (float)y;
		res.push_back(p1);
		if(duplicatepoints && res.size() > 1)
		{
			if(floor(lastpoint.x) - floor(p1.x) != 0 && floor(lastpoint.y) - floor(p1.y) != 0
				&& dx != 0 && dy != 0)
			{
				double deltax = floor(p1.x) - lastpoint.x;
				double deltay = floor(p1.y) - lastpoint.y;
				deltax /= dx;
				deltay /= dy;
				if(deltax < deltay)
				{
					if(dx > 0)
					{
						p1.x = floor(lastpoint.x + 1);
					}
					else
					{
						p1.x = floor(lastpoint.x - 1);
					}
					p1.y = lastpoint.y;
					lastpoint = res.back();
					if(onlyduplicatepoints)
						res.pop_back();
					res.push_back(p1);
				}
				else if(deltax > deltay)
				{
					if(dy > 0)
					{
						p1.y = floor(lastpoint.y + 1);
					}
					else
					{
						p1.y = floor(lastpoint.y - 1);
					}
					p1.x = lastpoint.x;
					lastpoint = res.back();
					if(onlyduplicatepoints)
						res.pop_back();
					res.push_back(p1);
				}
				else
				{
					// no duplicate points
					lastpoint = p1;
					if(onlyduplicatepoints)
						res.pop_back();
				}

			}
			else
			{
				lastpoint = p1;
				if(onlyduplicatepoints)
					res.pop_back();
			}
		}

		x += dx;
		y += dy;
	}
	// deal with the end point
	if((int)lastpoint.y - (int)p2y != 0 || (int)lastpoint.x - (int)p2x != 0)
	{
		if(!onlyduplicatepoints)
			res.push_back(p2);
		if(duplicatepoints && res.size() > 0)
		{
			if(floor(lastpoint.x) - floor(p2.x) != 0 && floor(lastpoint.y) - floor(p2.y) != 0
				&& dx != 0 && dy != 0)
			{
				double deltax = floor(p2.x) - lastpoint.x;
				double deltay = floor(p2.y) - lastpoint.y;
				deltax /= dx;
				deltay /= dy;
				if(deltax < deltay)
				{
					if(dx > 0)
					{
						p2.x = floor(lastpoint.x + 1);
					}
					else
					{
						p2.x = floor(lastpoint.x - 1);
					}
					p2.y = lastpoint.y;
					res.push_back(p2);
				}
				else if(deltax > deltay)
				{
					if(dy > 0)
					{
						p2.y = floor(lastpoint.y + 1);
					}
					else
					{
						p2.y = floor(lastpoint.y - 1);
					}
					p2.x = lastpoint.x;
					res.push_back(p2);
				}
				else
				{
					// no duplicate points
				}

			}
		}
	}

}

// added by qcy 2006.10.18
//---------------------------------------------------------------
// Name:	    CollisionAdjustBetweenPolygonAndVec
// Description: Adjust the inputed polygon shape by collision detection with another inputed polygon
// Argument:    vtsRef:-- the polygon for reference;  vt:-- the vector to adjust
//         :	fOffset:-- offset distances for the references polygon
//         :
// Return:		
// Author:		Qiu Chunyu
// Date:		2006/08/19 
// Update:	 
// Author:	
// Date: 
// copyright: XXX. developed by XXX
//----------------------------------------------------------------
bool CollisionAdjustBetweenPolygonAndVec(const varray<Vec4>& vtsRef,Vec4& vt,float fOffset,Vec4 vtCenter)
{
	int i = 0;
	Vec4 vtCen3D = Vec4(0,0,0);
	varray<Vec2> plgRef;
	bool bRet;

	for(i = 0; i < static_cast<int>(vtsRef.size()); i++)
	{
		plgRef.push_back(Vec2(vtsRef[i].x,vtsRef[i].z));
		vtCen3D += vtsRef[i];
	}
	vtCen3D /= static_cast<float>(vtsRef.size());

	// offset the reference polygon
	Vec2 vtCen = Vec2(0,0);
	if( (vtCen3D.x + 10e6)>1 || (vtCen3D.y + 10e6)>1 || (vtCen3D.z + 10e6)>1)
	{
		vtCen = Vec2(vtCen3D.x,vtCen3D.z);
	}
	else
	{
		for(i = 0; i < static_cast<int>(plgRef.size()); i++)
		{
			vtCen += plgRef[i];
		}
		vtCen /= static_cast<float>(plgRef.size());
	}

	Vec2 vtDir;
	for(i = 0; i < static_cast<int>(plgRef.size()); i++)
	{
		vtDir = (plgRef[i]-vtCen);
		plgRef[i] += vtDir.Normalize() * fOffset;
	}

	bRet = GetInterSectPntOfLnPolygon(plgRef,vt,2,vtCenter);
	return bRet;
}
//三角形外心将一个锐角分为三个部分，三个部分的面积之比通过三个参数化返回。
void GetRatioByCircleCenterPtInATriangle(Vec4 v0, Vec4 v1, Vec4 v2,float& r0,float& r1,float& r2)
{
    float cos0 = GetCosofAngleAOB(v1,v0,v2);
	float cos1 = GetCosofAngleAOB(v0,v1,v2);
	float cos2 = GetCosofAngleAOB(v1,v2,v0);
	float sin02 = cos0 * sqrt(1 - cos0*cos0);
    float sin12 = cos1 * sqrt(1 - cos1*cos1);
    float sin22 = cos2 * sqrt(1 - cos2*cos2);
	float sum = sin02 + sin12 + sin22;
	r0 = sin02 / sum;
	r1 = sin12 / sum;
	r2 = sin22 / sum;
}
//this function is not complected.
Vec4 GetCircleCenterPtInATriangle(Vec4 v0, Vec4 v1, Vec4 v2)
{
	double Cos102 = GetCosofAngleAOB(v1,v0,v2);
	double radius = (v1-v2).Magnitude() / sqrt(1 - Cos102*Cos102);
	
	return v0;     

}
void NormalizeArrayData(varray<float>& dataval)
{
	float resetmax,resetmin;
	resetmax = resetmin = 0;
	for(int i=0; i<dataval.size(); i++)
	{
		if(dataval.at(i) < resetmin)
			resetmin = dataval.at(i);
		if(dataval.at(i) > resetmax)
			resetmax = dataval.at(i);
	}
	for(int i=0; i<dataval.size(); i++)
	{
		dataval.at(i) = (dataval.at(i) - resetmin)/(resetmax - resetmin);
	}
}
void GetVariantLengthIntLine(const char str[], int *hidx)  //NMAX_INT_NUM_READ = 30
{
	for(int i=0; i<NMAX_INT_NUM_READ; i++)
		hidx[i] = 0;
	sscanf( str, "%d%d%d%d%d%d%d%d%d%d%d%d%d%d%d%d%d%d%d%d%d%d%d%d%d%d%d%d%d%d", &hidx[0], &hidx[1],&hidx[2], &hidx[3],\
		&hidx[4], &hidx[5],&hidx[6], &hidx[7],&hidx[8], &hidx[9],&hidx[10], &hidx[11],&hidx[12], &hidx[13],&hidx[14],\
		&hidx[15], &hidx[16],&hidx[17], &hidx[18],&hidx[19], &hidx[20],&hidx[21], &hidx[22],&hidx[23], &hidx[24],&hidx[25],\
		&hidx[26], &hidx[27],&hidx[28], &hidx[29]);
}
void  ConstructMeshFromFaceidx(XBaseMesh* oripms, varray<int>& includeFaceidx, XBaseMesh* newpms)
{
	//initialize the output mesh
	if(includeFaceidx.size() < 2)
		return;
	if(oripms->GetVSize() < 4)
		return;
	newpms->clear();


	int facenum = includeFaceidx.size();
	varray<int> vidxs;
	for(int i=0;i <facenum; i++)
	{
		for(int j=0; j<3; j++)
		{
			int vid;
			vid = oripms->GetF(includeFaceidx.at(i)).GetIndex(j);
			if(!IsInTheVarray(vid,vidxs))
			{
				vidxs.push_back(vid);
			}
		}
	}

	//构建新的mesh。
	newpms->SetVSize(vidxs.size());
	newpms->SetFSize(includeFaceidx.size());
	for(int i=0; i<vidxs.size(); i++)
	{
		newpms->GetV(i).Pos() = oripms->GetV(vidxs.at(i)).Pos();
		newpms->GetV(i).m_vid = i;
	}
	for(int i=0;i <facenum; i++)
	{
		int vid1,vid2,vid3;
		vid1 = oripms->GetF(includeFaceidx.at(i)).GetIndex(0);
		vid2 = oripms->GetF(includeFaceidx.at(i)).GetIndex(1);
		vid3 = oripms->GetF(includeFaceidx.at(i)).GetIndex(2);
		int newid1 = GetInTheVarrayIndex(vid1,vidxs);
		int newid2 = GetInTheVarrayIndex(vid2,vidxs);
		int newid3 = GetInTheVarrayIndex(vid3,vidxs);
		assert(newid1 != -1);
		assert(newid2 != -1);
		assert(newid3 != -1);
		newpms->GetF(i).SetIndex(newid1,newid2,newid3); 
	}
}
bool IstwoIntarraySame(const varray<int>& arr1, const varray<int>& arr2, bool compareByorder/* = false*/)
{
	int sz1 = arr1.size();
	int sz2 = arr2.size();
	if( sz1 != sz2)
		return false;
	if(compareByorder)
	{
		for(int i=0; i<sz1; i++)
		{
			if(arr1.at(i) != arr2.at(i))
				return false;
		}
	}
	else
	{
		int pcount = 0;
		for(int i=0; i<sz1; i++)
		{
			if(IsInTheVarray(arr2.at(i),arr1))
				pcount++;
		}
		if(pcount != sz1)
			return false;
	}
	return true;
}
//void  Display3DText(const CString& str,int x,int y,CDC* pdc)
//{
//	CGLFont* Font;
//	HFONT hFont;
//	Font = new CGLFont();
//	//第一个参数是字号大小，最后一个是字体名称，其它的不想多说了，可以自己去查
//	hFont  =CreateFont(16,0,0,0,400,0,0,0,GB2312_CHARSET,0,0,0,FF_MODERN,TEXT("宋体"));
//	//hFont0 =CreateFont(-48,0,0,0,800,0,0,0,GB2312_CHARSET,0,0,0,FF_MODERN,TEXT("黑体"));
//
//	Font->settext(x,y,LPCTSTR(str),hFont,1.0f,1.0f,0.0f);
//	//Font->settext(x,y,LPCTSTR(str),hFont,1.0f,1.0f,0.0f);
//	//Font->Printfc3d(LPCTSTR(str),hFont,0.05f);
//	Font->Printftext(x, y, LPCTSTR(str),hFont);
//	delete Font;
//}