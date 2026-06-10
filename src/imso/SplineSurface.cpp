

#include "SplineSurface.h"
#include "spline.h"
#include "assert.h"


namespace base {
SplineSurface::SplineSurface(void)
{
	m_UVCtrlPts.clear();
	m_uDegree = 3;
    m_vDegree = 3;
	m_iSurfaceMode = SURFACEMODE_BSPLINE;
	m_uNum = 0;
	m_vNum = 0;
	m_vShowMeshes.clear();
	m_uKnots.clear();
	m_vKnots.clear();
	m_4sideSpline.clear();
}
SplineSurface::~SplineSurface(void)
{
	if(m_vShowMeshes.size() > 0)
	{
		m_vShowMeshes.clear();
	}
	m_UVCtrlPts.clear();
	m_uKnots.clear();
	m_vKnots.clear();
	m_4sideSpline.clear();
}
void SplineSurface::Clear()
{
	if(m_vShowMeshes.size() > 0)
	{
		m_vShowMeshes.clear();
	}
	m_UVCtrlPts.clear();
	m_uKnots.clear();
	m_vKnots.clear();
	m_4sideSpline.clear();
}



SplineSurface* SplineSurface::CreateMe() const
{
	SplineSurface* pSplineSurface = new SplineSurface();
	*pSplineSurface = (*this);
	return pSplineSurface;
}
SplineSurface::SplineSurface(const SplineSurface& surface)
{
	m_CtrlPts = surface.m_CtrlPts;
	m_UVCtrlPts = surface.m_UVCtrlPts;
	m_uDegree = surface.m_uDegree;
	m_vDegree = surface.m_vDegree;
	m_iSurfaceMode = surface.m_iSurfaceMode;
	m_uNum = surface.m_uNum;
	m_vNum = surface.m_vNum;
	m_uKnots = surface.m_uKnots;
	m_vKnots = surface.m_vKnots;
	m_4sideSpline = surface.m_4sideSpline;
}
SplineSurface& SplineSurface:: operator = (const SplineSurface& src)
{
	m_UVCtrlPts = src.m_UVCtrlPts;
	m_CtrlPts = src.m_CtrlPts;
	m_uDegree = src.m_uDegree;
	m_vDegree = src.m_vDegree;
	m_iSurfaceMode = src.m_iSurfaceMode;
	m_uNum = src.m_uNum;
	m_vNum = src.m_vNum;
	m_uKnots = src.m_uKnots;
	m_vKnots = src.m_vKnots;
	m_4sideSpline = src.m_4sideSpline;
	return *this;
}

void SplineSurface::GetEdgeLines(varray<Spline>& EdgeLines) {
	EdgeLines.clear();
	EdgeLines.resize(4);
	Spline nl;
	int num_pt = this->m_CtrlPts.size();
	//第一条,U,V=0
	nl.m_Degree = this->m_uDegree;
	nl.m_Knots = this->m_uKnots;
	for (int i = 0; i < this->m_uNum; ++i) {
		nl.m_CtrlPts.push_back(this->m_CtrlPts[i]);
	}
	EdgeLines[0] = nl;

	//第二条,U,V=1
	nl.m_CtrlPts.clear();
	for (int i = num_pt - this->m_uNum; i < num_pt; ++i) {
		nl.m_CtrlPts.push_back(this->m_CtrlPts[i]);
	}
	EdgeLines[1] = nl;

	//第三条,V,U=0
	nl.m_Degree = this->m_vDegree;
	nl.m_Knots = this->m_vKnots;
	nl.m_CtrlPts.clear();
	for (int i = 0; i < this->m_vNum; ++i) {
		nl.m_CtrlPts.push_back(this->m_CtrlPts[i*this->m_uNum]);
	}
	EdgeLines[2] = nl;

	//第四条,V,U=1
	nl.m_CtrlPts.clear();
	for (int i = 0; i < this->m_vNum; ++i) {
		nl.m_CtrlPts.push_back(this->m_CtrlPts[(i + 1)*this->m_uNum - 1]);
	}
	EdgeLines[3] = nl;
}

void SplineSurface::AddCtrlPts(varray<varray<Vec4> >& CtrlPts)
{
    m_UVCtrlPts = CtrlPts;
    m_uNum = CtrlPts.size();
	m_vNum = CtrlPts.at(0).size();
}
void SplineSurface::AddKnotsVector(varray<double>& knots,bool uvflag)
{
    if(uvflag)
	{
        m_uKnots = knots;
        assert(m_uKnots.size() <= m_uNum + m_uDegree + 1);
	}		
	else
	{
        m_vKnots = knots;
		assert(m_vKnots.size() <= m_vNum + m_vDegree + 1);
	}		
}
void SplineSurface::AddKnotsVector(varray<float>& knots,bool uvflag)
{
	if(uvflag)
	{
		m_uKnots.clear();
		for(int i=0; i<knots.size(); i++)
		{
			m_uKnots.push_back((double)knots.at(i));
		}
		assert(m_uKnots.size() <= m_uNum + m_uDegree + 1);
	}		
	else
	{
		m_vKnots.clear();
		for(int i=0; i<knots.size();i++)
		{
			m_vKnots.push_back((double)knots.at(i));
		}
		assert(m_vKnots.size() <= m_vNum + m_vDegree + 1);
	}		
}
void SplineSurface::ChangeSurfaceMode(DWORD iMode)
{
    m_iSurfaceMode = iMode;	
}
void SplineSurface::SetSurfaceDegree(int uDegree,int vDegree)
{
    m_uDegree = uDegree;
    m_vDegree = vDegree;
}
void SplineSurface::Generate4SidedBoudaryCurve()
{
    if(m_4sideSpline.size() != 4)
	{
		m_4sideSpline.clear();
		m_4sideSpline.resize(4);
	}

	if(m_iSurfaceMode == SURFACEMODE_BSPLINE)
	{
		for(int i=0; i<4; i++)
		{
			m_4sideSpline.at(i).ChangeMode(Spline::SPLMODE_NONUNI_NBSPLINE);
		}
	}
	else if(m_iSurfaceMode == SURFACEMODE_BZIER)
	{
		for(int i=0; i<4; i++)
		{
			m_4sideSpline.at(i).ChangeMode(Spline::SPLMODE_BEZIER);
		}
	}
	else if(m_iSurfaceMode == SURFACEMODE_NONUNI_BSPLINE)
	{
		for(int i=0; i<4; i++)
		{
			m_4sideSpline.at(i).ChangeMode(Spline::SPLMODE_NONUNI_NBSPLINE);
		}
	}

	//4条线按逆时针顺序排列。
	for(int i=0; i<m_uNum; i++)
	{
		m_4sideSpline.at(0).AddCtrlPoint(m_UVCtrlPts.at(0).at(i));
	}
	for(int i=0; i<m_vNum; i++)
	{
		m_4sideSpline.at(1).AddCtrlPoint(m_UVCtrlPts.at(i).at(m_uNum-1));
	}
	for(int i=m_uNum-1; i>=0; i--)
	{
		m_4sideSpline.at(2).AddCtrlPoint(m_UVCtrlPts.at(m_vNum-1).at(i));
	}
	for(int i=m_vNum-1; i>=0; i--)
	{
		m_4sideSpline.at(3).AddCtrlPoint(m_UVCtrlPts.at(i).at(0));
	}

	m_4sideSpline.at(0).SetSplineDegree(m_uDegree);
	m_4sideSpline.at(1).SetSplineDegree(m_vDegree);
	m_4sideSpline.at(2).SetSplineDegree(m_uDegree);
	m_4sideSpline.at(3).SetSplineDegree(m_vDegree);

	int nsize = m_uKnots.size();
	for(int i=0; i<nsize; i++)
	{
		m_4sideSpline.at(0).AddKnots(m_uKnots.at(i));
		m_4sideSpline.at(2).AddKnots(m_uKnots.at(i));
	}
	nsize = m_vKnots.size();
	for(int i=0; i<nsize; i++)
	{
		m_4sideSpline.at(1).AddKnots(m_vKnots.at(i));
		m_4sideSpline.at(3).AddKnots(m_vKnots.at(i));
	}
}
Vec4 SplineSurface::GetUVPoint(float u,float v)
{
    Vec4 surfacePt;	
	int i,j;
	float FU,FV;
	varray<float> Uvals,Vvals;

	surfacePt.x = surfacePt.y = surfacePt.z = 0.f;
	FU = FV = 0;
	if(m_iSurfaceMode == SURFACEMODE_BZIER)//not right
	{
		for(i=0; i<m_vNum;i++)
		{
			FV = BernsteinFun(m_vNum-1,i,v);
			for(j=0; j<m_uNum; j++)
			{            
				FU = BernsteinFun(m_uNum-1,j,u);
				surfacePt += m_UVCtrlPts.at(i).at(j) * FU * FV;
			}
		}
    }
	else if(m_iSurfaceMode == SURFACEMODE_BSPLINE)
	{
		BSplineFunArray(m_uDegree,u,Uvals);
		BSplineFunArray(m_vDegree,v,Vvals);
		for(i=0; i<m_vNum;i++)
		{
			FV = Vvals.at(i);
			for(j=0; j<m_uNum; j++)
			{            
				FU = Uvals.at(j);
				surfacePt += m_UVCtrlPts.at(i).at(j) * FU * FV;
			}
		}
	}
	else if(m_iSurfaceMode == SURFACEMODE_NONUNI_BSPLINE)
	{
        for(i=0;i<m_vNum;i++)
		{
		    FV = OneBasisFun(m_vDegree,m_vKnots.size()-1,m_vKnots,i,v);//BSplineFunRecursive(m_uDegree,i,u,m_uKnots);
            for(j=0; j<m_uNum; j++)
			{
                 FU = OneBasisFun(m_vDegree,m_uKnots.size()-1,m_uKnots,j,u);//BSplineFunRecursive(m_vDegree,j,v,m_vKnots);
                 surfacePt += m_UVCtrlPts.at(i).at(j) * FU * FV;
			}
		}
	}
	return surfacePt;
}
void SplineSurface::CreateShowMesh(bool bClear)
{
	if(bClear)
		m_vShowMeshes.clear();
	if(m_vShowMeshes.size() > 0)
		return;
    XBaseMesh xbsms;
	Vec4 vt;
	double t1,t2;
	double delt1,delt2;
	int np,nf;
	int UKnotNum,VKnotNum;
	np = nf = 0;
	delt1 = delt2 = 0.1;
    UKnotNum = (int)(1.00001/delt1);
    VKnotNum = (int)(1.00001/delt2);
	xbsms.SetVSize((UKnotNum+1) * (VKnotNum+1));
	xbsms.SetFSize(UKnotNum*VKnotNum*2);
	for(int i=0; i<VKnotNum+1; i++)
	{
		t1 = 0.0 + delt1*i;
		for(int j=0; j<UKnotNum+1; j++)
		{
			t2 = 0.0 + delt2*j;
			vt.x = vt.y = vt.z = 0.0;
            vt = GetUVPoint(t1,t2);
			xbsms.GetV(np++).Pos() = vt;
		}
	}
	int index;
    for(int i=0; i<UKnotNum; i++)
	{
		for(int j=0; j<VKnotNum; j++)
		{
			index = i*(UKnotNum + 1) + j;
            xbsms.GetF(nf++).SetIndex(index,index + UKnotNum + 1,index + UKnotNum + 2);
			xbsms.GetF(nf++).SetIndex(index,index + UKnotNum + 2,index + 1);
		}
	}
    xbsms.MakeXEdges();
	m_vShowMeshes.push_back(xbsms); 
	for(int i=0; i<m_vShowMeshes.size();i++)
	{
		m_vShowMeshes.at(i).ComputeNormals();
	}
}
bool SplineSurface::IstwoSurfaceSame(SplineSurface& sf, bool notConsiderUVDirection/* = true*/)
{
	float minLentorence = 0.0001;//???? magicNumber???
	if(notConsiderUVDirection)
	{
		 float len = GetDistanceBetweenTwoSurface(sf);
		 if(len > minLentorence)
			 return false;
	}
	else
	{
		 float len = GetDistanceBetweenTwoOrderSurface(sf);
		 //容许误差是多少呢？
		 if(len > minLentorence)
		 {
			 return false;
		 }
	}
	return true;
};
//由于当前曲面和给定的sf曲面控制点是对应的，因此，只需求对应的距离，然后平均一下即可。
float SplineSurface::GetDistanceBetweenTwoOrderSurface(SplineSurface& sf)
{
	int unum,vnum;
	unum = sf.GetUCtrlPtNum();
	vnum = sf.GetVCtrlPtNum();
	if(unum != GetUCtrlPtNum() || vnum != GetVCtrlPtNum())
	{
		return 1000000;
	}
	float lenadd = 0;
	for(int i=0; i<vnum; i++)
	{
		for(int j=0; j<unum; j++)
		{
			lenadd += (sf.GetCtrlPt(i,j) - GetCtrlPt(i,j)).Magnitude();

		}
	}
	return lenadd/(unum*vnum);
}
//由于当前曲面和指定曲面没有办法对应，因此针对当前曲面的每个控制点，然后求取给定的曲面与当前控制点的最近距离，
//最后再求取一个平均值。
float SplineSurface::GetDistanceBetweenTwoSurface(SplineSurface& sf)
{
	float meanMinlen = 0;
	for(int i=0; i<GetVCtrlPtNum();i++)
	{
		for(int j=0; j<GetUCtrlPtNum();j++)
		{
			Vec4 vt = GetCtrlPt(i,j);
			float minlen = 1000000;
			int uminid = -1;
			int vminid = -1;
			for(int k=0; k<sf.GetVCtrlPtNum();k++)
			{
				for(int l=0; l<sf.GetUCtrlPtNum(); l++)
				{
                     if((vt - sf.GetCtrlPt(k,l)).Magnitude()<minlen)
					 {
						 vminid = k;
						 uminid = l;
						 minlen = (vt - sf.GetCtrlPt(k,l)).Magnitude();
					 }
				}
			}
			meanMinlen += minlen;
		}
	}
	meanMinlen /= (GetVCtrlPtNum()*GetUCtrlPtNum());
	return meanMinlen;
}
bool SplineSurface::IsTwoVec4arraySameinOrder(const varray<Vec4>& arr1, const varray<Vec4>& arr2)
{
	if(arr1.size() != arr2.size())
		return false;
	int nsize = arr1.size();
	//只要首尾两点重合，就认为该边重合。
	float end_fitting_err = 0.01;
	float fitting_err = 2.0;
	int pcount = 0;

	float starterr = (arr1.at(0)-arr2.at(0)).Magnitude();
	float enderr = (arr1.at(nsize-1)-arr2.at(nsize-1)).Magnitude();
	
	if(starterr > end_fitting_err)
		pcount++;
	if(enderr > end_fitting_err)
		pcount++;

	for(int i=1; i<nsize-1; i++)
	{
		float length = (arr1.at(i) - arr2.at(i)).Magnitude();
		if(length > fitting_err)   //此处经常因为拟合误差的原因，造成判断失误，从而找不到重合边。
		{
			pcount++;
		}
	}
	if(pcount > nsize/2)
		return false;
	return true;
}
int  SplineSurface::GetTwoSurfaceEdgeshareState(const SplineSurface& sf) //以当前面作为基准面，作为最下面，且以左前方构建三维坐标系。
{
	varray<Vec4> uvedge[4];
	varray<Vec4> sfuvedge[4];
	for(int i=0; i<m_uNum; i++)
	{
		uvedge[0].push_back(GetCtrlPt(0,i));  //u0
		uvedge[1].push_back(GetCtrlPt(m_vNum-1,i));  //u1
	}	
	for(int i=0;i<m_vNum; i++)              
	{
		uvedge[2].push_back(GetCtrlPt(i,0));    //v0
		uvedge[3].push_back(GetCtrlPt(i,m_uNum-1));    //v1
	}

	//另外一个面。
	for(int i=0; i<sf.m_uNum; i++)
	{
		sfuvedge[0].push_back(sf.GetCtrlPt(0,i));
		sfuvedge[1].push_back(sf.GetCtrlPt(sf.m_vNum-1,i));
	}	
	for(int i=0;i<sf.m_vNum; i++)
	{
		sfuvedge[2].push_back(sf.GetCtrlPt(i,0));
		sfuvedge[3].push_back(sf.GetCtrlPt(i,sf.m_uNum-1));
	}

	int istate = 0;  //表示两条边不共享
	if(IsTwoVec4arraySameinOrder(uvedge[0],sfuvedge[0]))  //u0u0
		istate = 1;
	else if(IsTwoVec4arraySameinOrder(uvedge[0],sfuvedge[1])) //u0u1
		istate = 2;
	else if(IsTwoVec4arraySameinOrder(uvedge[0],sfuvedge[2])) //u0v0
		istate = 3;
	else if(IsTwoVec4arraySameinOrder(uvedge[0],sfuvedge[3])) //u0v1
		istate = 4;
	else if(IsTwoVec4arraySameinOrder(uvedge[1],sfuvedge[0]))  //u1u0
		istate = 11;
	else if(IsTwoVec4arraySameinOrder(uvedge[1],sfuvedge[1]))  //u1u1
		istate = 12;
	else if(IsTwoVec4arraySameinOrder(uvedge[1],sfuvedge[2]))  //u1v0
		istate = 13;
	else if(IsTwoVec4arraySameinOrder(uvedge[1],sfuvedge[3])) //u1v1
		istate = 14;
	else if(IsTwoVec4arraySameinOrder(uvedge[2],sfuvedge[0]))  //v0u0
		istate = 21;
	else if(IsTwoVec4arraySameinOrder(uvedge[2],sfuvedge[1]))  //v0u1
		istate = 22;  
	else if(IsTwoVec4arraySameinOrder(uvedge[2],sfuvedge[2]))  //v0v0
		istate = 23;
	else if(IsTwoVec4arraySameinOrder(uvedge[2],sfuvedge[3]))  //v0v1
		istate = 24;
	else if(IsTwoVec4arraySameinOrder(uvedge[3],sfuvedge[0]))  //v1u0
		istate = 31;
	else if(IsTwoVec4arraySameinOrder(uvedge[3],sfuvedge[1]))  //v1u1
		istate = 32;
	else if(IsTwoVec4arraySameinOrder(uvedge[3],sfuvedge[2]))  //v1v0
		istate = 33;
	else if(IsTwoVec4arraySameinOrder(uvedge[3],sfuvedge[3]))  //v1v1
		istate = 34;

	if(istate == 0)
	{
		//将第二条边全部逆向。
		for(int i=0; i<4; i++)
		{
			std::reverse(sfuvedge[i].begin(),sfuvedge[i].end());
		}
		if(IsTwoVec4arraySameinOrder(uvedge[0],sfuvedge[0]))
			istate = -1;
		else if(IsTwoVec4arraySameinOrder(uvedge[0],sfuvedge[1]))
			istate = -2;
		else if(IsTwoVec4arraySameinOrder(uvedge[0],sfuvedge[2]))
			istate = -3;
		else if(IsTwoVec4arraySameinOrder(uvedge[0],sfuvedge[3]))
			istate = -4;
		else if(IsTwoVec4arraySameinOrder(uvedge[1],sfuvedge[0]))
			istate = -11;
		else if(IsTwoVec4arraySameinOrder(uvedge[1],sfuvedge[1]))
			istate = -12;
		else if(IsTwoVec4arraySameinOrder(uvedge[1],sfuvedge[2]))
			istate = -13;
		else if(IsTwoVec4arraySameinOrder(uvedge[1],sfuvedge[3]))
			istate = -14;
		else if(IsTwoVec4arraySameinOrder(uvedge[2],sfuvedge[0]))
			istate = -21;
		else if(IsTwoVec4arraySameinOrder(uvedge[2],sfuvedge[1]))
			istate = -22;
		else if(IsTwoVec4arraySameinOrder(uvedge[2],sfuvedge[2]))
			istate = -23;
		else if(IsTwoVec4arraySameinOrder(uvedge[2],sfuvedge[3]))
			istate = -24;
		else if(IsTwoVec4arraySameinOrder(uvedge[3],sfuvedge[0]))
			istate = -31;
		else if(IsTwoVec4arraySameinOrder(uvedge[3],sfuvedge[1]))
			istate = -32;
		else if(IsTwoVec4arraySameinOrder(uvedge[3],sfuvedge[2]))
			istate = -33;
		else if(IsTwoVec4arraySameinOrder(uvedge[3],sfuvedge[3]))
			istate = -34;
	}
	return istate;
}
int  SplineSurface::GetTwoSurfaceEdgeshareStateNew(const SplineSurface& sf)
{
	varray<Vec4> uvedge[4];
	varray<Vec4> sfuvedge[4];
	for(int i=0; i<m_uNum; i++)
	{
		uvedge[0].push_back(GetCtrlPt(0,i));  //u0
		uvedge[1].push_back(GetCtrlPt(m_vNum-1,i));  //u1
	}	
	for(int i=0;i<m_vNum; i++)              
	{
		uvedge[2].push_back(GetCtrlPt(i,0));    //v0
		uvedge[3].push_back(GetCtrlPt(i,m_uNum-1));    //v1
	}

	//另外一个面。
	for(int i=0; i<sf.m_uNum; i++)
	{
		sfuvedge[0].push_back(sf.GetCtrlPt(0,i));
		sfuvedge[1].push_back(sf.GetCtrlPt(sf.m_vNum-1,i));
	}	
	for(int i=0;i<sf.m_vNum; i++)
	{
		sfuvedge[2].push_back(sf.GetCtrlPt(i,0));
		sfuvedge[3].push_back(sf.GetCtrlPt(i,sf.m_uNum-1));
	}

	//共16种正向边的情况，16种逆向边的情况。
	//因此设定一个4*4的数组。
	varray<varray<float> > lenarr;
	lenarr.resize(8);
	for(int i=0; i<8; i++)
	{
		lenarr.at(i).resize(4);
	}
	float templen;
	for(int i=0; i<4; i++)
	{
		for(int j=0; j<4; j++)
		{
			templen = 0;
			if(uvedge[i].size() != sfuvedge[j].size())
			{
				templen = 1000000;
			}
			else
			{
				int arrsz = uvedge[i].size();
				templen = 0;
				for(int k=0; k<arrsz; k++)
				{
					templen += (uvedge[i].at(k) - sfuvedge[j].at(k)).Magnitude();
				}
			}
			lenarr.at(i).at(j) = templen;
		}
	}
	//第二个面全部逆向，再重新判断一次；
	for(int i=0; i<4; i++)
	{
		std::reverse(sfuvedge[i].begin(),sfuvedge[i].end());
	}
	for(int i=0; i<4; i++)
	{
		for(int j=0; j<4; j++)
		{
			templen = 0;
			if(uvedge[i].size() != sfuvedge[j].size())
			{
				templen = 1000000;
			}
			else
			{
				int arrsz = uvedge[i].size();
				templen = 0;
				for(int k=0; k<arrsz; k++)
				{
					templen += (uvedge[i].at(k) - sfuvedge[j].at(k)).Magnitude();
				}
			}
			lenarr.at(i+4).at(j) = templen;
		}
	}
	//寻找32个数中最小值。
	int minid1,minid2;
	minid1 = 0;minid2 = 0;
	templen = lenarr.at(0).at(0);
	for(int i=0; i<8; i++)
	{
		for(int j=0; j<4; j++)
		{
			if(lenarr.at(i).at(j) < templen)
			{
				templen = lenarr.at(i).at(j);
				minid1 = i;
				minid2 = j;
			}
		}
	}
	
	//进行情况判断；
	int istate = 0;
	if(minid1 < 4)
	{
        istate = minid1 * 10  + minid2 + 1;
	} 
	else if(minid1 >= 4)
	{
		istate = (minid1 - 4)* 10 + minid2 + 1;
		istate *= -1;
	}
	return istate;
}
void SplineSurface::RotateControlPoints(int imode)
{
	if(imode == 0) //只反u向
	{
		for(int i=0; i<m_vNum; i++)
		{
			for(int j=0; j<m_uNum/2; j++)
			{
				std::swap(m_UVCtrlPts.at(i).at(j),m_UVCtrlPts.at(i).at(m_uNum-1-j));
			}
		}
		std::reverse(m_uKnots.begin(),m_uKnots.end());
		for(int i=0;i<m_uKnots.size(); i++)
		{
			m_uKnots.at(i) = 1.0-m_uKnots.at(i);
		}
	}
	else if(imode == 1) //只反v向
	{
		for(int i=0; i<m_uNum; i++)
		{
			for(int j=0; j<m_vNum/2; j++)
			{
				std::swap(m_UVCtrlPts.at(j).at(i),m_UVCtrlPts.at(m_vNum-1-j).at(i));
			}
		}
		std::reverse(m_vKnots.begin(),m_vKnots.end());
		for(int i=0;i<m_vKnots.size(); i++)
		{
			m_vKnots.at(i) = 1.0-m_vKnots.at(i);
		}
	}
	else if(imode == 2) //uv向都要反
	{
		for(int i=0; i<m_vNum; i++)
		{
			for(int j=0; j<m_uNum/2; j++)
			{
				std::swap(m_UVCtrlPts.at(i).at(j),m_UVCtrlPts.at(i).at(m_uNum-1-j));
			}
		}
		std::reverse(m_uKnots.begin(),m_uKnots.end());
		for(int i=0;i<m_uKnots.size(); i++)
		{
			m_uKnots.at(i) = 1.0-m_uKnots.at(i);
		}
		for(int i=0; i<m_uNum; i++)
		{
			for(int j=0; j<m_vNum/2; j++)
			{
				std::swap(m_UVCtrlPts.at(j).at(i),m_UVCtrlPts.at(m_vNum-1-j).at(i));
			}
		}
		std::reverse(m_vKnots.begin(),m_vKnots.end());
		for(int i=0;i<m_vKnots.size(); i++)
		{
			m_vKnots.at(i) = 1.0-m_vKnots.at(i);
		}
	}
	else if(imode == 3) //uv矩阵转置
	{
		varray<varray<Vec4> > arr = m_UVCtrlPts;
		m_UVCtrlPts.clear();
		//重置数组。
		m_UVCtrlPts.resize(m_uNum);
		for(int i=0; i<m_uNum; i++)
		{
			m_UVCtrlPts.at(i).resize(m_vNum);
			for(int j=0; j<m_vNum; j++)
			{
				m_UVCtrlPts.at(i).at(j) = arr.at(j).at(i);
			}
		}
		std::swap(m_uNum,m_vNum);
		std::swap(m_uDegree,m_vDegree);
		varray<double> uk = m_uKnots;
		m_uKnots.clear();
		for(int i=0; i<m_vKnots.size();i++)
		{
			m_uKnots.push_back(m_vKnots.at(i));
		}
		m_vKnots.clear();
		for(int i=0; i<uk.size();i++)
		{
			m_vKnots.push_back(uk.at(i));
		}
	}
}
int SplineSurface::RotateTheWholeSurfaceAccordingtoBasesurface(int istate)
{
	if(istate == 0)  
		return -1;//表示两个面不共享

	if(istate == 1 || istate == 11 || istate == 21 || istate == 31) //uv矩阵不动。
	{
		int testid = 0;
	}
	else if(istate == 2 || istate == 12 || istate == 22 || istate == 32) //只反v向。
	{
		RotateControlPoints(1);
	}
	else if(istate == 3 || istate == 13 || istate == 23 || istate == 33) //uv控制点矩阵要转置。
	{
		RotateControlPoints(3);
	}
	else if(istate == 4 || istate == 14 || istate == 24 || istate == 34) //uv控制点矩阵转置后，再v向反向。
	{
		RotateControlPoints(3);
		RotateControlPoints(1);
	}
	else if(istate == -1 || istate == -11 || istate == -21 || istate == -31) //只反u向
	{
		RotateControlPoints(0);
	}
	else if(istate == -2 || istate == -12 || istate == -22 || istate == -32) //uv向都要反。
	{
		RotateControlPoints(2);
	}
	else if(istate == -3 || istate == -13 || istate == -23 || istate == -33)  //uv控制点转置，然后u向反。
	{
		RotateControlPoints(3);
		RotateControlPoints(0);
	}
	else if(istate == -4 || istate == -14 || istate == -24 || istate == -34)  //uv控制点转置后，再uv反向。
	{
		RotateControlPoints(3);
		RotateControlPoints(2);
	}

	int iPosition = -1;
	if(istate == 1 || istate == 2 || istate == 3 || istate == 4 || istate == -1 || istate == -2 || istate == -3 || istate == -4)
		iPosition = 0;     //u向第一面。
	else if(istate == 11 || istate == 12 || istate == 13 || istate == 14 || istate == -11 || istate == -12 || istate == -13 || istate == -14)
		iPosition = 2;     //u向第二面
	else if(istate == 21 || istate == 22 || istate == 23 || istate == 24 || istate == -21 || istate == -22 || istate == -23 || istate == -24)
		iPosition =  1;    //v向第一面
	else if(istate == 31 || istate == 32 || istate == 33 || istate == 34 || istate == -31 || istate == -32 || istate == -33 || istate == -34)
		iPosition = 3;     //v向第二面
	return iPosition;
};
//以下函数转到v正向和y轴正向平行，u正向和x轴正向平行。
//按照组合，共有8种情况。目前只能用坐标轴简单的判断一下。
//目前将所有的基准面全转到第一象限。即L模型不需要旋转就一定能对。而球模型需要旋转。
void SplineSurface::RotateBaseSurface()  
{
	//判断是否需要转置。
	Vec4 u0v0 = GetCtrlPt(0,0);
	Vec4 u1v0 = GetCtrlPt(0,m_uNum-1);
	Vec4 u0v1 = GetCtrlPt(m_vNum-1,0);
	Vec4 u1v1 = GetCtrlPt(m_vNum-1,m_uNum-1);
	Vec4 uDir = u1v0 - u0v0;
	Vec4 vDir = u0v1 - u0v0;
	uDir = uDir.Normalize();
	vDir = vDir.Normalize();

	/*float len1 = abs(uDir.dotmultiple(Vec4(0,-1,0)));
	float len2 = abs(uDir.dotmultiple(Vec4(0,1,0)));
	float len3 = abs(vDir.dotmultiple(Vec4(0,-1,0)));
	float len4 = abs(vDir.dotmultiple(Vec4(0,1,0)));
	if((len1 > len3 && len1 > len4) || (len2 > len3 && len2 > len4))
	{
	RotateControlPoints(3);
	}*/
	float lenuu = abs(uDir.dotmultiple(Vec4(1,0,0)));
	float lenuv = abs(uDir.dotmultiple(Vec4(0,1,0)));
	float lenvu = abs(vDir.dotmultiple(Vec4(1,0,0)));
	float lenvv = abs(vDir.dotmultiple(Vec4(0,1,0)));
	if(lenvu > lenuu && lenuv > lenvv)
	{
		RotateControlPoints(3);
	}

	//再判断是否需要反向。
	bool uDirCorrect,vDirCorrect;
	uDirCorrect = vDirCorrect = false;
	u0v0 = GetCtrlPt(0,0);
	u1v0 = GetCtrlPt(0,m_uNum-1);
	u0v1 = GetCtrlPt(m_vNum-1,0);
	u1v1 = GetCtrlPt(m_vNum-1,m_uNum-1);
	Vec4 udir1 = u1v0 - u0v0;
	Vec4 udir2 = u1v1 - u0v1;
	Vec4 vdir1 = u0v1 - u0v0;
	Vec4 vdir2 = u1v1 - u1v0;
	if(udir1.x > 0 || udir2.x > 0)
	{
		uDirCorrect = true;
	}
	if(vdir1.y > 0 || vdir2.y > 0)
	{
		vDirCorrect = true;
	}
	if(!uDirCorrect)
		RotateControlPoints(0);
	if(!vDirCorrect)
		RotateControlPoints(1);
}



//以下函数使用对基准面的朝向有关系，要求v正向和y轴正向平行，u正向和x轴正向平行。因此使用之前需要旋转基准面。
//否则执行结果不对。以shphere和L-shape为例，其基准面的朝向不同，结果就不对。
int  SplineSurface::RotateLastSuface(int istate)
{
	if(istate == 0)  
		return -1;//表示两个面不共享
	//由于是最后一个曲面，因此只有可能某边和U1边重合。因此istate的值只有可能是11,12,13,14，-11，-12，-13，-14.其他值不可能。
	assert(istate == -11 || istate == -12 || istate == -13 || istate == -14 || istate == 11 || istate == 12 || istate == 13 || istate == 14);
	if(istate == 11) //uv转置。
	{
		RotateControlPoints(3);
	}
	else if(istate == 12) //uv转置在u反向。
	{
		RotateControlPoints(3);
		RotateControlPoints(0);
	}
	else if(istate == 13) //uv不动。
	{
		int testid = 0;
	}
	else if(istate == 14) //u向反向。
	{
		RotateControlPoints(0);
	}
	else if(istate == -11) //uv转置再v反向
	{
		RotateControlPoints(3);
		RotateControlPoints(1);
	}
	else if(istate == -12) //uv转置再uv向反向。
	{
		RotateControlPoints(3);
		RotateControlPoints(2);
	}
	else if(istate == -13)  //v反向。
	{
		RotateControlPoints(1);
	}
	else if( istate == -14)  //再uv反向。
	{
		RotateControlPoints(2);
	}

	int iPosition = -1;
	iPosition = 4; //最后的封闭曲面。
	return iPosition;
}
//void SplineSurface::DisplaySplineSurface(int renderMode,bool bShowBoundary)
//{
//	int meshSize = m_vShowMeshes.size();
//	if(meshSize == 0 && m_UVCtrlPts.size() > 0)
//	{
//		CreateShowMesh();
//	}
//	meshSize = m_vShowMeshes.size();
//	if(meshSize != 0)
//	{
//		for(int i=0; i<meshSize; i++)
//		{
//			//COLORREF clr = RGB(0,125,125);
//			int randred,randgreen,randblue;
//			randred = rand()/255;
//			if(randred < 125)
//				randred += 80;
//			randgreen = rand() /255;
//			if(randgreen < 125)
//				randgreen += 50;
//			randblue = rand()/255;
//			if(randblue < 125)
//				randblue += 30;
//			COLORREF clr = RGB(randred,randgreen,randblue);
//			if(renderMode == SHADE_MODE)
//				m_vShowMeshes.at(i).RenderMeshShade(clr/*COLORREF(RGB(0,125,125))*/,1);
//			else if(renderMode == WIREFRAME_MODE)
//				m_vShowMeshes.at(i).RenderMeshWire(clr/*COLORREF(RGB(0,125,125))*/,1);
//			else if(renderMode == TRANSPARENT_MODE)
//				m_vShowMeshes.at(i).RenderMeshTransparent(clr/*COLORREF(RGB(0,125,125))*/,1);
//			else if(renderMode == POINT_MODE)
//				m_vShowMeshes.at(i).RenderMeshVerts(clr/*COLORREF(RGB(0,125,125))*/,1);
//		}		
//	}
//	if(bShowBoundary)
//	{
//		for(int i=0; i<4; i++)
//		{
//			m_4sideSpline.at(i).DrawOneSpline();
//		}
//	}
//};
//void SplineSurface::DrawSurfaceCtrlPts(COLORREF clr)
//{
//	int iUNum,iVNum;
//	iUNum = GetUCtrlPtNum();
//	iVNum = GetVCtrlPtNum();
//	for(int i=0; i<iVNum; i++)
//	{
//		for(int j=0; j<iUNum; j++)
//		{
//			DrawOnePt(GetCtrlPt(i,j),2);
//		}
//	}
//}
void SplineSurface::CopySurfaceWithoutMesh(SplineSurface& sp)
{
	m_UVCtrlPts = sp.m_UVCtrlPts;
	m_uDegree = sp.m_uDegree;
	m_vDegree = sp.m_vDegree;
	m_iSurfaceMode = sp.m_iSurfaceMode;
	m_uNum = sp.m_uNum;
	m_vNum = sp.m_vNum;
	m_uKnots = sp.m_uKnots;
	m_vKnots = sp.m_vKnots;
	m_4sideSpline = sp.m_4sideSpline;
}
//Coons插值
//EdgeCtrlPts就是4条边的控制点，顺序为P(u,0),P(0,v),P(u,1),P(1,v)
void SplineSurface::CoonsInterpolate(const varray<varray<Vec4>>& EdgeCtrlPts, varray<varray<Vec4>>& coonsPatchCtrlpts)
{
	coonsPatchCtrlpts.clear();
	int upnum = EdgeCtrlPts[0].size();//u方向数量
	int vpnum = EdgeCtrlPts[1].size();//v方向数量
	Vec4 P00, P10, P01, P11;//角点
	P00 = EdgeCtrlPts[0][0];
	P10 = EdgeCtrlPts[0][upnum - 1];
	P01 = EdgeCtrlPts[2][0];
	P11 = EdgeCtrlPts[2][upnum - 1];
	for (int i = 0; i < upnum; ++i)
	{
		double tui = 1.0*i / upnum;
		varray<Vec4> Ctrlpts_i;
		for (int j = 0; j < vpnum; ++j)
		{
			double tvj = 1.0*j / vpnum;
			Vec4 Pij = (1 - tui)*EdgeCtrlPts[1][j] + tui * EdgeCtrlPts[3][j]
			+ (1 - tvj)*EdgeCtrlPts[0][i] + tvj * EdgeCtrlPts[2][i]
			- (1 - tvj)*((1 - tui)*P00 + tui * P10)
				- tvj * ((1 - tui)*P01 + tui * P11);
			Ctrlpts_i.push_back(Pij);
		}
		coonsPatchCtrlpts.push_back(Ctrlpts_i);
	}
}
/*
	功能：初始化曲面参数：uv方向阶数、uv控制点个数、uv节点向量
	*uDegree：u方向阶数
	*vDegree：v方向阶数
	*uNum：u控制点个数
	*vNum：v控制点个数
	*uKnots：u方向节点向量
	*vKnots：v方向节点向量
*/
void SplineSurface::SetSurface(const int uDegree, const int vDegree, const int uNum, const int vNum, const varray<double>& uKnots, const varray<double>& vKnots)
{
	m_uDegree = uDegree;
	m_vDegree = vDegree;
	m_uNum = uNum;
	m_vNum = vNum;
	m_uKnots = uKnots;
	m_vKnots = vKnots;
}

//计算（u，v）对应的曲面上的点
Vec3 SplineSurface::GetSurFacePoint(const double u, const double v)const
{
	int m_uid = 0, m_vid = 0;
	varray<double> Nu, Nv;
	Nu.resize(m_uDegree + 1);
	Nv.resize(m_vDegree + 1);
	m_uid = FindSpan(u, m_uDegree, m_uNum, m_uKnots);
	BasisFuns(u, m_uid, m_uDegree, m_uKnots, Nu);
	m_vid = FindSpan(v, m_vDegree, m_vNum, m_vKnots);
	BasisFuns(v, m_vid, m_vDegree, m_vKnots, Nv);

	Vec3 res(0, 0, 0);
	double pw = 0.0;
	for (int j = 0; j <= m_vDegree; j++)
	{
		for (int i = 0; i <= m_uDegree; i++)
		{
			Vec4 bpti = m_CtrlPts[(m_uid - m_uDegree + i)*m_vNum + m_vid - m_vDegree + j];
			res += Nu[i] * Nv[j] * bpti*bpti.w;
			pw += Nu[i] * Nv[j] * bpti.w;
		}
	}
	res /= pw;
	return res;
}


//计算四边形面片显示数据
threadParamSpline SplineSurface::CalQuads(const int Unum, const int Vnum)const
{
	varray<varray<Vec3>> quads;
	varray<varray<Vec3>> lines;
	quads.clear();
	lines.clear();
	varray<varray<Vec3>> L_u;
	double du = 1.0 / Unum;
	double dv = 1.0 / Vnum;
	for (int i = 0; i <= Unum; i++)
	{
		double u = du * i;
		varray<Vec3> line;
		for (int j = 0; j <= Vnum; j++)
		{
			double v = dv * j;
			Vec3 t = GetSurFacePoint(u, v);
			line.push_back(t);
		}
		L_u.push_back(line);
	}

	for (int i = 0; i < L_u.size() - 1; ++i)
	{
		for (int j = 0; j < L_u[i].size() - 1; ++j)
		{
			varray<Vec3> quad;
			quad.push_back(L_u[i][j]);
			quad.push_back(L_u[i + 1][j]);
			quad.push_back(L_u[i + 1][j + 1]);
			quad.push_back(L_u[i][j + 1]);
			quads.push_back(quad);
		}
	}

	varray<Vec3> l0, l1;
	for (int i = 0; i < L_u.size(); ++i)
	{
		l0.push_back(L_u[i][0]);
		l1.push_back(L_u[i][L_u[i].size() - 1]);
	}
	lines.push_back(l0);//u0
	lines.push_back(l1);//u1
	lines.push_back(L_u[0]);//v0
	lines.push_back(L_u[L_u.size() - 1]);//v1
	varray<varray<Vec3>> p1;
	varray<varray<Vec3>> p2;
	p1 = quads;
	p2 = lines;
	threadParamSpline param = { p1,p2 };
	return param;
}

/*
	Coons插值：计算内部控制点
	EndgCtrlPts:边界控制点（v=0,u=0,v=1,u=1顺序）
*/
void SplineSurface::CoonsInterpolate(const varray<varray<Vec4>>& EdgeCtrlPts)
{
	//Coons面片控制点数组
	varray<varray<Vec4>> coonsPatchCtrlpts;

	//u方向控制点数量
	int upnum = EdgeCtrlPts[0].size();

	//v方向控制点数量
	int vpnum = EdgeCtrlPts[1].size();

	Vec4 P00, P10, P01, P11;//角点,四边形四个角点
	P00 = EdgeCtrlPts[0][0];
	P10 = EdgeCtrlPts[0][upnum - 1];
	P01 = EdgeCtrlPts[2][0];
	P11 = EdgeCtrlPts[2][upnum - 1];

	//四边形插值算法：计算出中间点的坐标和权值
	for (int i = 0; i < upnum; ++i)
	{
		double tui = 1.0*i / (upnum - 1);
		varray<Vec4> Ctrlpts_i;
		for (int j = 0; j < vpnum; ++j)
		{
			double tvj = 1.0*j / (vpnum - 1);

			//获得插值后的控制点Pij的位置信息
			Vec4 Pij = (1 - tui)*EdgeCtrlPts[1][j] + tui * EdgeCtrlPts[3][j]
				+ (1 - tvj)*EdgeCtrlPts[0][i] + tvj * EdgeCtrlPts[2][i]
				- (1 - tvj)*((1 - tui)*P00 + tui * P10)
				- tvj * ((1 - tui)*P01 + tui * P11);

			//获得查之后的控制点Pij的权值信息
			Pij.w = (1 - tui)*EdgeCtrlPts[1][j].w + tui * EdgeCtrlPts[3][j].w
				+ (1 - tvj)*EdgeCtrlPts[0][i].w + tvj * EdgeCtrlPts[2][i].w
				- (1 - tvj)*((1 - tui)*P00.w + tui * P10.w)
				- tvj * ((1 - tui)*P01.w + tui * P11.w);

			Ctrlpts_i.push_back(Pij);
		}
		coonsPatchCtrlpts.push_back(Ctrlpts_i);
	}

	//二维容器转化为一维容器，存入SplineSurface对象的m_CtrlPts成员变量中
	ReduceVarrayDim(coonsPatchCtrlpts, m_CtrlPts, true);
}

/*
	*Coons插值
	*EdgeLines:边界曲线, F(u)v=0,F(v)u=0,F(u)v=1,F(v)u=1顺序）
	*调用该函数之前可以调用Model_Solution::OrderCoonsLines对边界线进行调整
*/
void SplineSurface::CoonsInterpolate(const varray<Spline>& EdgeLines)
{
	if (EdgeLines.size() != 4)//保证为四边形
		return;

	//初始化曲面参数
	SetSurface(EdgeLines[0].m_Degree, EdgeLines[1].m_Degree, EdgeLines[0].m_CtrlPts.size(), EdgeLines[1].m_CtrlPts.size(),
		EdgeLines[0].m_Knots, EdgeLines[1].m_Knots);

	//边界曲线的控制点
	varray<varray<Vec4>> cpt;
	for (int i = 0; i < EdgeLines.size(); ++i)//将边界曲线存入cpt中
		cpt.push_back(EdgeLines[i].m_CtrlPts);
	CoonsInterpolate(cpt);
}


//曲面升阶
//Udegree,Vdegree:升阶后次数
void SplineSurface::DegreeElevate(const int Udegree, const int Vdegree)
{
	int tu = Udegree - m_uDegree;
	int tv = Vdegree - m_vDegree;
	//U方向
	if (tu > 0)
	{
		varray<varray<Vec4>> NewConPoint;
		Spline line;

		for (int i = 0; i < m_vNum; ++i)
		{
			line.m_Degree = m_uDegree;
			line.m_Knots = m_uKnots;
			line.m_CtrlPts.clear();
			for (int j = 0; j < m_uNum; ++j)
				line.m_CtrlPts.push_back(m_CtrlPts[m_vNum*j + i]);
			line.DegreeElevate(Udegree);
			NewConPoint.push_back(line.m_CtrlPts);
		}
		m_uDegree = Udegree;
		m_uKnots = line.m_Knots;
		m_uNum = line.m_CtrlPts.size();
		m_CtrlPts.resize(m_uNum*m_vNum);
		for (int i = 0; i < NewConPoint.size(); ++i)
			for (int j = 0; j < NewConPoint[i].size(); ++j)
				m_CtrlPts[m_vNum*j + i] = NewConPoint[i][j];
	}
	//V方向
	if (tv > 0)
	{
		varray<varray<Vec4>> NewConPoint;
		Spline line;

		for (int i = 0; i < m_uNum; ++i)
		{
			line.m_Degree = m_vDegree;
			line.m_Knots = m_vKnots;
			line.m_CtrlPts.clear();
			for (int j = 0; j < m_vNum; ++j)
				line.m_CtrlPts.push_back(m_CtrlPts[m_vNum*i + j]);
			line.DegreeElevate(Vdegree);
			NewConPoint.push_back(line.m_CtrlPts);
		}
		m_vDegree = Vdegree;
		m_vKnots = line.m_Knots;
		m_vNum = line.m_CtrlPts.size();
		m_CtrlPts.clear();
		for (int i = 0; i < NewConPoint.size(); ++i)
			for (int j = 0; j < NewConPoint[i].size(); ++j)
				m_CtrlPts.push_back(NewConPoint[i][j]);
	}
}

//曲面节点插入
//Uknot,Vknot:需要插入的节点
void SplineSurface::KnotsRefine(const varray<double>& Uknot, const varray<double>& Vknot)
{
	//U方向
	if (Uknot.size() > 0)
	{
		varray<varray<Vec4>> NewConPoint;
		Spline line;

		for (int i = 0; i < m_vNum; ++i)
		{
			line.m_Degree = m_uDegree;
			line.m_Knots = m_uKnots;
			line.m_CtrlPts.clear();
			for (int j = 0; j < m_uNum; ++j)
				line.m_CtrlPts.push_back(m_CtrlPts[m_vNum*j + i]);
			line.KnotsRefine(Uknot);
			NewConPoint.push_back(line.m_CtrlPts);
		}
		m_uKnots = line.m_Knots;
		m_uNum = line.m_CtrlPts.size();
		m_CtrlPts.resize(m_uNum*m_vNum);
		for (int i = 0; i < NewConPoint.size(); ++i)
			for (int j = 0; j < NewConPoint[i].size(); ++j)
				m_CtrlPts[m_vNum*j + i] = NewConPoint[i][j];
	}
	//V方向
	if (Vknot.size() > 0)
	{
		varray<varray<Vec4>> NewConPoint;
		Spline line;

		for (int i = 0; i < m_uNum; ++i)
		{
			line.m_Degree = m_vDegree;
			line.m_Knots = m_vKnots;
			line.m_CtrlPts.clear();
			for (int j = 0; j < m_vNum; ++j)
				line.m_CtrlPts.push_back(m_CtrlPts[m_vNum*i + j]);
			line.KnotsRefine(Vknot);
			NewConPoint.push_back(line.m_CtrlPts);
		}
		m_vKnots = line.m_Knots;
		m_vNum = line.m_CtrlPts.size();
		m_CtrlPts.clear();
		for (int i = 0; i < NewConPoint.size(); ++i)
			for (int j = 0; j < NewConPoint[i].size(); ++j)
				m_CtrlPts.push_back(NewConPoint[i][j]);
	}
}

void SplineSurface::KnotsRefineNum(int refinenum)
{
	//存放新产生的节点，作为细化函数的参数
	varray <double> new_uknots;
	varray <double> new_vknots;
	int i = 1;
	while (i <= refinenum)
	{
		//找出中间节点 u方向
		for (int i = 1; i < m_uKnots.size(); i++)
		{
			if (m_uKnots[i - 1] != m_uKnots[i])
			{
				new_uknots.push_back((m_uKnots[i - 1] + m_uKnots[i]) / 2);
			}
		}

		//找出中间节点 v方向
		for (int i = 1; i < m_vKnots.size(); i++)
		{
			if (m_vKnots[i - 1] != m_vKnots[i])
			{
				new_vknots.push_back((m_vKnots[i - 1] + m_vKnots[i]) / 2);
			}
		}
		
		//细化
		KnotsRefine(new_uknots, new_vknots);

		new_uknots.clear();
		new_vknots.clear();

		i++;
	}
}

//曲面分段为Bezier曲面
//dir:0=U方向，1=V方向
//QW：Bezier曲面控制点
void SplineSurface::Decompose(const bool dir, varray<varray<varray<Vec4>>>& QW)
{
	int a, b, i, mult, save, s;
	double numer, alpha;
	varray<double> alphas;
	varray<double> NUknot;   //新的U向量
	varray<double> NVknot;   //新的V向量
	int nb = 0;
	if (dir == 0)
	{
		//Bezier片数及控制点数量初始化
		for (int i = 0; i < m_uKnots.size() - 1; i++)
		{
			if (m_uKnots[i + 1] != m_uKnots[i])
			{
				nb++;
			}
		}
		QW.resize(nb);
		for (int i = 0; i < nb; i++)
		{
			QW[i].resize(m_vNum);
			for (int j = 0; j < m_vNum; j++)
			{
				QW[i][j].resize(m_uDegree + 1);
			}
		}
		//划分为Bezier片
		a = m_uDegree; b = m_uDegree + 1; nb = 0; alphas.resize(m_uDegree);
		for (int i = 0; i <= m_uDegree; i++)
		{
			NUknot.push_back(m_uKnots[i]);
		}
		for (int row = 0; row < m_vNum; row++)
		{
			for (int i = 0; i <= m_uDegree; i++)
			{
				QW[nb][row][i] = m_CtrlPts[m_vNum*i + row];
			}
		}
		while (b < m_uKnots.size() - 1)
		{
			i = b;
			while (b < m_uKnots.size() - 1 && m_uKnots[b + 1] == m_uKnots[b])
			{
				NUknot.push_back(m_uKnots[b]);
				b++;
			}
			mult = b - i + 1;  //插入个数
			if (mult < m_uDegree)
			{
				NUknot.push_back(m_uKnots[b]);
				numer = m_uKnots[b] - m_uKnots[a];
				for (int j = m_uDegree; j > mult; j--)
				{
					alphas[j - mult - 1] = numer / (m_uKnots[a + j] - m_uKnots[a]);
				}
				for (int j = 1; j <= m_uDegree - mult; j++)
				{
					save = m_uDegree - mult - j;
					s = mult + j;  //s个新的控制点
					for (int row = 0; row < m_vNum; row++)
					{
						for (int k = m_uDegree; k >= s; k--)
						{
							alpha = alphas[k - s];
							QW[nb][row][k] = alpha * QW[nb][row][k] + (1.0 - alpha)*QW[nb][row][k - 1];
							QW[nb][row][k].w = alpha * QW[nb][row][k].w + (1.0 - alpha)*QW[nb][row][k - 1].w;
						}
					}
					if (b < m_uKnots.size() - 1)
					{
						for (int row = 0; row < m_vNum; row++)
						{
							QW[nb + 1][row][save] = QW[nb][row][m_uDegree];
						}
					}
				}
			}
			nb = nb + 1;
			if (b < m_uKnots.size() - 1)
			{
				for (int row = 0; row < m_vNum; row++)
				{
					for (int i = m_uDegree - mult; i <= m_uDegree; i++)
					{
						QW[nb][row][i] = m_CtrlPts[m_vNum*(b - m_uDegree + i) + row];
					}
				}
				a = b; b++;
			}
		}
	}
	if (dir == 1)
	{
		//Bezier片数及控制点数量初始化
		for (int i = 0; i < m_vKnots.size() - 1; i++)
		{
			if (m_vKnots[i + 1] != m_vKnots[i])
			{
				nb++;
			}
		}
		QW.resize(nb);
		for (int i = 0; i < nb; i++)
		{
			QW[i].resize(m_vDegree + 1);
			for (int j = 0; j <= m_vDegree; j++)
			{
				QW[i][j].resize(m_uNum);
			}
		}

		a = m_vDegree; b = m_vDegree + 1; nb = 0; alphas.resize(m_vDegree);
		for (int i = 0; i <= m_vDegree; i++)
		{
			NVknot.push_back(m_vKnots[i]);
			for (int row = 0; row < m_uNum; row++)
			{
				QW[nb][i][row] = m_CtrlPts[m_vNum*row + i];
			}
		}
		while (b < m_vKnots.size() - 1)
		{
			i = b;
			while (b < m_vKnots.size() - 1 && m_vKnots[b + 1] == m_vKnots[b])
			{
				NVknot.push_back(m_vKnots[b]);
				b++;
			}
			NVknot.push_back(m_vKnots[b]);
			mult = b - i + 1;  //插入个数
			if (mult < m_vDegree)
			{
				numer = m_vKnots[b] - m_vKnots[a];
				for (int j = m_vDegree; j > mult; j--)
				{
					alphas[j - mult - 1] = numer / (m_vKnots[a + j] - m_vKnots[a]);
				}
				for (int j = 1; j <= m_vDegree - mult; j++)
				{
					NVknot.push_back(m_vKnots[b]);
					save = m_vDegree - mult - j;
					s = mult + j;  //s个新的控制点
					for (int row = 0; row < m_uNum; row++)
					{
						for (int k = m_vDegree; k >= s; k--)
						{
							alpha = alphas[k - s];
							QW[nb][k][row] = alpha * QW[nb][k][row] + (1.0 - alpha)*QW[nb][k - 1][row];
							QW[nb][k][row].w = alpha * QW[nb][k][row].w + (1.0 - alpha)*QW[nb][k - 1][row].w;
						}
					}
					if (b < m_vKnots.size() - 1)
					{
						for (int row = 0; row < m_uNum; row++)
						{
							QW[nb + 1][save][row] = QW[nb][m_vDegree][row];
						}
					}
				}
			}
			nb = nb + 1;
			if (b < m_vKnots.size() - 1)
			{
				for (int row = 0; row < m_uNum; row++)
				{
					for (int i = m_vDegree - mult; i <= m_vDegree; i++)
					{
						QW[nb][i][row] = m_CtrlPts[m_vNum*row + (b - m_vDegree + i)];
					}
				}
				a = b; b++;
			}
		}
	}
}

//根据控制点二维序号计算一维序号
inline int SplineSurface::CtrlPtsIdx(const int uIdx, const int vIdx)
{
	return m_vNum * uIdx + vIdx;
}

//控制点排序转换为U-V
void SplineSurface::OrderCtrlPts()
{
	SplineSurface sf;
	OrderCtrlPts(sf);
	m_CtrlPts = sf.m_CtrlPts;
	SetSurface(sf.m_uDegree, sf.m_vDegree, sf.m_uNum, sf.m_vNum, sf.m_uKnots, sf.m_vKnots);
}

//控制点排序转换为U-V
void SplineSurface::OrderCtrlPts(SplineSurface& sf)
{
	varray<Vec4> temp;
	for (int i = 0; i < m_vNum; ++i)
	{
		for (int j = 0; j < m_uNum; ++j)
		{
			temp.push_back(m_CtrlPts[CtrlPtsIdx(j, i)]);
		}
	}
	std::swap(sf.m_uNum, sf.m_vNum);
	std::swap(sf.m_uDegree, sf.m_vDegree);
	std::swap(sf.m_uKnots, sf.m_vKnots);
	sf.m_CtrlPts = temp;
}


}