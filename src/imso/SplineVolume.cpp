
#include "SplineVolume.h"
//#include "XFunction.h"

using namespace base;
namespace base{
VolumeVertex::VolumeVertex()
{
};
VolumeVertex::VolumeVertex(Vec4& vt)
{
	m_pt=vt;
	m_matval.x = m_matval.y = m_matval.z = 0;
};
VolumeVertex::VolumeVertex(Vec4& vt,Vec4& matt)
{
	m_pt=vt;
	m_matval=matt;
};
VolumeVertex::~VolumeVertex()
{

}
SplineVolume::SplineVolume(void)
:m_uNum(0),
m_vNum(0),
m_wNum(0),
m_uRenderNum(0),
m_vRenderNum(0),
m_wRenderNum(0),
m_uDegree(3),
m_vDegree(3),
m_wDegree(3),
m_bHeterogeneous(false),
m_bHasInterpolated(false),
m_bHasIGASolution(false)
{
	m_CtrlPtsIDinKKKMatrix.clear();
}

SplineVolume::~SplineVolume(void)
{
	m_6boundarySurface.clear();
	m_CtrlPtsIDinKKKMatrix.clear();
}
void  SplineVolume::CreateAllVolumeRenderPts(int uN,int vN,int wN)
{
 	m_vVolumePts.clear();
	m_uRenderNum = uN;
	m_vRenderNum = vN;
	m_wRenderNum = wN;
	Vec4 pt,matPt;

	for(int i=0; i<m_wRenderNum;i++)
	{
		for(int j=0; j<m_vRenderNum; j++)
		{
			for(int k=0; k<m_uRenderNum; k++)
			{
				pt = GetPoint(k*1.f/(m_uRenderNum-1),j*1.f/(m_vRenderNum-1),i*1.f/(m_wRenderNum-1),matPt);
				m_vVolumePts.push_back(VolumeVertex(pt,matPt));
			}
		}
	}
}
void  SplineVolume::ReCreateAllVolumeRenderPts()
{
	m_vVolumePts.clear();

	Vec4 pt,matPt;

	for(int i=0; i<m_wRenderNum;i++)
	{
		for(int j=0; j<m_vRenderNum; j++)
		{
			for(int k=0; k<m_uRenderNum; k++)
			{
				pt = GetPoint(k*1.f/(m_uRenderNum-1),j*1.f/(m_vRenderNum-1),i*1.f/(m_wRenderNum-1),matPt);
				m_vVolumePts.push_back(VolumeVertex(pt,matPt));
			}
		}
	}
}
bool SplineVolume::JudgeNeedInterpolate()
{
    int num = m_vAllCtrlPts.size();
	Vec4 vt;
	int pcount = 0;
	for(int i=0;i <num; i++)
	{
		vt = m_vAllCtrlPts.at(i).m_pt;
		if(vt.x * vt.x + vt.y*vt.y + vt.z * vt.z < 1.0e-5)
			pcount++;
		if(pcount > 1)
			m_bHasInterpolated = true;
	}
	if(pcount < 2)
		m_bHasInterpolated = false;
	return m_bHasInterpolated;
}
// void SplineVolume::RenderbyMode(int iVolumeMode,int iSurfaceMode)
//{
//	float oldColor[4] = {0,0,0,0};	
//	glGetFloatv(GL_CURRENT_COLOR, oldColor);
//	glEnable(GL_LIGHTING);	
//
//	if(iVolumeMode == iRenderBoundary_ControlPoints)
//	{
//		RenderCtrlPts(false);
//	}
//	else if(iVolumeMode == iRenderBoundary_Patch)
//	{
//		RenderPatch(true,iSurfaceMode);
//	}
//	else if(iVolumeMode == iRenderBoundary_WithISOElements)
//	{
//		RenderIsoSurfaces(true,1);
//		RenderIsoCurves(true);
//		RenderIsopoints(true);
//	}
//	else if(iVolumeMode == iRenderBoundary_All)
//	{
//		RenderCtrlPts(true);
//		RenderPatch(true,iSurfaceMode);
//		RenderIsopoints(true);
//		RenderIsoCurves(true);
//		RenderIsoSurfaces(true,0.5);
//	}
//	else if(iVolumeMode == iRenderVolume_ControlPoints)
//	{
//		RenderCtrlPts(false);
//	}
//	else if(iVolumeMode == iRenderVolume_TinyPatch)
//	{
//		RenderPatch(false,iSurfaceMode);
//	}
//	else if(iVolumeMode == iRenderVolume_WithISOElements)
//	{
//		RenderIsopoints(false);
//		RenderIsoCurves(false);
//		//RenderIsoSurfaces(false,0.5);
//	}
//	else if(iVolumeMode == iRenderVolume_All)
//	{
//		RenderCtrlPts(false);
//		RenderPatch(false,iSurfaceMode);
//		RenderIsopoints(false);
//		RenderIsoCurves(false);
//		RenderIsoSurfaces(false,0.5);
//	}
//	else if(iVolumeMode == iRenderVolume_ISOWireFrame)
//	{
//		RenderIsopoints(false);
//		RenderIsoCurves(false);
//	}
//
//	glDisable(GL_LIGHTING);
//	glColor4fv(oldColor);//recover the old color
//}
// void  SplineVolume::RenderPatch(bool bBoundary,int iSurfaceMode)
// {
//	 if(bBoundary)
//	 {
//		 for(int i=0; i<m_6boundarySurface.size(); i++)
//		 {
//			 m_6boundarySurface.at(i).DisplaySplineSurface(iSurfaceMode,true);
//		 }
//	 }
//	 else
//	 {
//		 RenderTinyCube();
//	 }
// }
//void  SplineVolume::RenderCtrlPts(bool bOnlyBoudary)
//{
//    int i = 0, iVSize = m_vAllCtrlPts.size();	
//	int ui,vi,wi;
//	float oldColor[4] = {0,0,0,0};	
//	glGetFloatv(GL_CURRENT_COLOR, oldColor);
//
//	glColor4f(1.0,1.0,1.0,1.0);
//	glPointSize(2);
//	glBegin(GL_POINTS);
//	for(i = 0; i < iVSize; i++)
//	{
//		if(m_bHeterogeneous)
//	        glColor3f(m_vAllCtrlPts.at(i).m_matval.x,0,1-m_vAllCtrlPts.at(i).m_matval.x);
//	    else
//		    glColor3f(1.f,0.f,0.f);
//		if(bOnlyBoudary)
//		{
//            if(GetIJKIndex(i,ui,vi,wi,m_uNum,m_vNum,m_wNum))
//			{
//				if(IsBoundaryPoint(ui,vi,wi,m_uNum,m_vNum,m_wNum))
//				{
//					Vec4& vt = m_vAllCtrlPts.at(i).m_pt;
//					glVertex3f(vt.x,vt.y,vt.z);
//					//DrawSpherePoint(vt,2,COLORREF(RGB(255,0,0)));
//					continue;
//				}
//			}
//		}	
//		else
//		{
//			Vec4& vt = m_vAllCtrlPts.at(i).m_pt;
//			glVertex3f(vt.x,vt.y,vt.z);
//			//DrawSpherePoint(vt,2,COLORREF(RGB(255,0,0)));
//		}		
//	}
//	glEnd();
//    glColor4fv(oldColor);
//}
//void  SplineVolume::RenderIsopoints(bool bOnlyBoudary)
//{
//	int i = 0;
//	int ui,vi,wi;
//	int iVSize = m_vVolumePts.size();
//	float oldColor[4] = {0,0,0,0};	
//	glGetFloatv(GL_CURRENT_COLOR, oldColor);
//
//	Vec4 colorVt = Vec4(0.f,0.f,0.f);
//	glColor4f(1.0,1.0,1.0,1.0);		
//	glPointSize(2);
//	glBegin(GL_POINTS);
//	for(i = 0; i < iVSize; i++)
//	{
//		if(m_bHeterogeneous)
//			glColor3f(m_vVolumePts.at(i).m_matval.x,0,1 - m_vVolumePts.at(i).m_matval.x);
//		else
//			glColor3f(colorVt.x,colorVt.y,colorVt.z);
//		if(bOnlyBoudary)
//		{
//			if(GetIJKIndex(i,ui,vi,wi,m_uRenderNum,m_vRenderNum,m_wRenderNum))
//			{
//				if(!IsBoundaryPoint(ui,vi,wi,m_uRenderNum,m_vRenderNum,m_wRenderNum))
//				{
//					continue;
//				}
//			}
//		}	
//		else
//		{
//			Vec4& vt = m_vVolumePts.at(i).m_pt;
//			glVertex3f(vt.x,vt.y,vt.z);
//		}
//		
//	}		
//	glEnd();
//    glColor4fv(oldColor);
//}
//void  SplineVolume::RenderIsoCurves(bool bOnlyBoudary)
//{
//	Vec4 vert;	
//	float oldColor[4] = {0,0,0,0};	
//	glGetFloatv(GL_CURRENT_COLOR, oldColor);
//
//    Vec4 colorVt = Vec4(0.f,1.f,0.f);
//	glColor4f(0.f,1.f,0.f,0.f);
//	glLineWidth(1);
//	for(int i=0; i<m_wRenderNum; i++)
//	{
//		for(int j=0; j<m_vRenderNum; j++)
//		{
//			glBegin(GL_LINES);
//			for(int k=0; k<m_uRenderNum-1; k++)
//			{
//				if(bOnlyBoudary)
//				{
//                    if(!IsBoundaryPoint(k,j,i,m_uRenderNum,m_vRenderNum,m_wRenderNum) 
//						|| !IsBoundaryPoint(k+1,j,i,m_uRenderNum,m_vRenderNum,m_wRenderNum))
//					{
//						continue;
//					}
//				}
//				vert = m_vVolumePts.at(GetRenderPointIndex(k,j,i)).m_pt;
//				if(m_bHeterogeneous)
//				{
//					float clr = m_vVolumePts.at(GetRenderPointIndex(k,j,i)).m_matval.x;
//					glColor3f(clr,0,1-clr);
//				}
//				else
//					glColor3f(colorVt.x,colorVt.y,colorVt.z);
//				glVertex3f(vert.x,vert.y,vert.z);
//
//				vert = m_vVolumePts.at(GetRenderPointIndex(k+1,j,i)).m_pt;
//				if(m_bHeterogeneous)
//				{
//					float clr = m_vVolumePts.at(GetRenderPointIndex(k+1,j,i)).m_matval.x;
//					glColor3f(clr,0,1-clr);
//				}
//				else
//					glColor3f(colorVt.x,colorVt.y,colorVt.z);
//				glVertex3f(vert.x,vert.y,vert.z);	
//			}
//			glEnd();
//		}
//	}
//	for(int i=0; i<m_uRenderNum; i++)
//	{
//		for(int j=0; j<m_vRenderNum; j++)
//		{
//			glBegin(GL_LINES);
//			for(int k=0; k<m_wRenderNum-1; k++)
//			{
//				if(bOnlyBoudary)
//				{
//					if(!IsBoundaryPoint(i,j,k,m_uRenderNum,m_vRenderNum,m_wRenderNum)
//						|| !IsBoundaryPoint(i,j,k+1,m_uRenderNum,m_vRenderNum,m_wRenderNum))
//					{
//						continue;
//					}
//				}
//				vert = m_vVolumePts.at(GetRenderPointIndex(i,j,k)).m_pt;
//				if(m_bHeterogeneous)
//				{
//					float clr = m_vVolumePts.at(GetRenderPointIndex(i,j,k)).m_matval.x;
//					glColor3f(clr,0,1-clr);
//				}
//				else
//					glColor3f(colorVt.x,colorVt.y,colorVt.z);
//				glVertex3f(vert.x,vert.y,vert.z);
//				vert = m_vVolumePts.at(GetRenderPointIndex(i,j,k+1)).m_pt;
//				if(m_bHeterogeneous)
//				{
//					float clr = m_vVolumePts.at(GetRenderPointIndex(i,j,k+1)).m_matval.x;
//					glColor3f(clr,0,1-clr);
//				}
//				else
//					glColor3f(colorVt.x,colorVt.y,colorVt.z);
//				glVertex3f(vert.x,vert.y,vert.z);
//			}
//			glEnd();
//		}
//	}
//	for(int i=0; i<m_wRenderNum; i++)
//	{		
//		for(int j=0; j<m_uRenderNum; j++)
//		{			
//			glBegin(GL_LINES);
//			for(int k=0; k<m_vRenderNum-1; k++)
//			{
//				if(bOnlyBoudary)
//				{
//					if(!IsBoundaryPoint(j,k,i,m_uRenderNum,m_vRenderNum,m_wRenderNum)
//					|| !IsBoundaryPoint(j,k+1,i,m_uRenderNum,m_vRenderNum,m_wRenderNum))
//					{
//						continue;
//					}
//				}
//				vert = m_vVolumePts.at(GetRenderPointIndex(j,k,i)).m_pt;
//				glVertex3f(vert.x,vert.y,vert.z);if(m_bHeterogeneous)
//				{
//					float clr = m_vVolumePts.at(GetRenderPointIndex(j,k,i)).m_matval.x;
//					glColor3f(clr,0,1-clr);
//				}
//				else
//					glColor3f(colorVt.x,colorVt.y,colorVt.z);
//				vert = m_vVolumePts.at(GetRenderPointIndex(j,k+1,i)).m_pt;
//				if(m_bHeterogeneous)
//				{
//					float clr = m_vVolumePts.at(GetRenderPointIndex(j,k+1,i)).m_matval.x;
//					glColor3f(clr,0,1-clr);
//				}
//				else
//					glColor3f(colorVt.x,colorVt.y,colorVt.z);
//				glVertex3f(vert.x,vert.y,vert.z);
//			}
//			glEnd();
//		}
//	}
//
//	glColor4fv(oldColor);
//}
//void  SplineVolume::RenderTinyCube()
//{
//	int i,j,k;
//	Vec4 vt[8];
//	for(k=0; k<m_wRenderNum-1; k++)
//	{
//		for(j=0; j<m_vRenderNum-1; j++)
//		{
//			for(i=0; i<m_uRenderNum-1; i++)
//			{
//				vt[0] = m_vVolumePts.at(GetRenderPointIndex(i,j,k)).m_pt;
//				vt[1] = m_vVolumePts.at(GetRenderPointIndex(i+1,j,k)).m_pt;
//				vt[2] = m_vVolumePts.at(GetRenderPointIndex(i+1,j+1,k)).m_pt;
//				vt[3] = m_vVolumePts.at(GetRenderPointIndex(i,j+1,k)).m_pt;
//				vt[4] = m_vVolumePts.at(GetRenderPointIndex(i,j,k+1)).m_pt;
//				vt[5] = m_vVolumePts.at(GetRenderPointIndex(i+1,j,k+1)).m_pt;
//				vt[6] = m_vVolumePts.at(GetRenderPointIndex(i+1,j+1,k+1)).m_pt;
//				vt[7] = m_vVolumePts.at(GetRenderPointIndex(i,j+1,k+1)).m_pt;
//				glBegin(GL_POLYGON); //下底面
//				glVertex3f(vt[0].x,vt[0].y,vt[0].z);
//				glVertex3f(vt[1].x,vt[1].y,vt[1].z);
//				glVertex3f(vt[2].x,vt[2].y,vt[2].z);
//				glVertex3f(vt[3].x,vt[3].y,vt[3].z);
//				glEnd();
//				glBegin(GL_POLYGON); //上底面。
//				glVertex3f(vt[4].x,vt[4].y,vt[4].z);
//				glVertex3f(vt[5].x,vt[5].y,vt[5].z);
//				glVertex3f(vt[6].x,vt[6].y,vt[6].z);
//				glVertex3f(vt[7].x,vt[7].y,vt[7].z);
//				glEnd();
//				glBegin(GL_POLYGON); //左面。
//				glVertex3f(vt[0].x,vt[0].y,vt[0].z);
//				glVertex3f(vt[4].x,vt[4].y,vt[4].z);
//				glVertex3f(vt[7].x,vt[7].y,vt[7].z);
//				glVertex3f(vt[3].x,vt[3].y,vt[3].z);
//				glEnd();
//				glBegin(GL_POLYGON); //右面。
//				glVertex3f(vt[1].x,vt[1].y,vt[1].z);
//				glVertex3f(vt[2].x,vt[2].y,vt[2].z);
//				glVertex3f(vt[6].x,vt[6].y,vt[6].z);
//				glVertex3f(vt[5].x,vt[5].y,vt[5].z);
//				glEnd();
//				glBegin(GL_POLYGON); //前面。
//				glVertex3f(vt[0].x,vt[0].y,vt[0].z);
//				glVertex3f(vt[1].x,vt[1].y,vt[1].z);
//				glVertex3f(vt[5].x,vt[5].y,vt[5].z);
//				glVertex3f(vt[4].x,vt[4].y,vt[4].z);
//				glEnd();
//				glBegin(GL_POLYGON); //后面。
//				glVertex3f(vt[2].x,vt[2].y,vt[2].z);
//				glVertex3f(vt[3].x,vt[3].y,vt[3].z);
//				glVertex3f(vt[7].x,vt[7].y,vt[7].z);
//				glVertex3f(vt[6].x,vt[6].y,vt[6].z);
//				glEnd();
//			}
//		}
//	}	
//}
////以下注释的地方，需要验证。效果不一定行。
//void  SplineVolume::RenderIsoSurfaces(bool bOnlyBoudary,float alpha)
//{
//	//glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
//	//glEnable(GL_POLYGON_OFFSET_FILL);
//	//glShadeModel(GL_SMOOTH); //smooth render		
//	//glLineWidth(1);
//    //glPolygonOffset(1.0, 1.0);	
//	//glEnable(GL_BLEND);//启用融合效果
//	//glBlendFunc(GL_SRC_ALPHA,GL_ONE_MINUS_SRC_ALPHA);//设置融合方式   
//	////glBlendFunc(GL_SRC_ALPHA,GL_DST_COLOR);
//	//glEnable(GL_DEPTH_TEST);	
//	//glDepthMask(GL_TRUE);		
//	//RenderIsoSurfaces(true,0.0);
//	//glDisable(GL_DEPTH_TEST);
//	//glDisable(GL_BLEND);			
//	//glDisable(GL_POLYGON_OFFSET_FILL);*/
//
//	Vec4 vert[4];  //因为是四边片，所以只有四个点。
//	float clr[4];  //对于非均质有每个点都有一个不同的颜色。
//	int i,j,k;
//	float oldColor[4] = {0,0,0,0};	
//	glGetFloatv(GL_CURRENT_COLOR, oldColor);
//	Vec4 colorVt = Vec4(0.f,1.f,0.f);
//	
//	//glColor4f(1.0,1.0,1.0,1.0);
//	//首先显示u向等参面。
//	for(i=0; i<m_uRenderNum; i++)
//	{
//		for(k=0; k<m_wRenderNum-1; k++)
//		{
//			for(j=0; j<m_vRenderNum-1; j++)
//			{		
//				vert[0] = m_vVolumePts.at(GetRenderPointIndex(i,j,k)).m_pt;
//				clr[0] = m_vVolumePts.at(GetRenderPointIndex(i,j,k)).m_matval.x;
//				vert[1] = m_vVolumePts.at(GetRenderPointIndex(i,j,k+1)).m_pt;
//				clr[1] = m_vVolumePts.at(GetRenderPointIndex(i,j,k+1)).m_matval.x;
//				vert[2] = m_vVolumePts.at(GetRenderPointIndex(i,j+1,k+1)).m_pt;
//				clr[2] = m_vVolumePts.at(GetRenderPointIndex(i,j+1,k+1)).m_matval.x;
//				vert[3] = m_vVolumePts.at(GetRenderPointIndex(i,j+1,k)).m_pt;
//				clr[3] = m_vVolumePts.at(GetRenderPointIndex(i,j+1,k)).m_matval.x;
//				if(!m_bHeterogeneous)
//				{
//					glColor4f(colorVt.x,colorVt.y,colorVt.z,alpha);
//					glBegin(GL_POLYGON);
//					if(i==0)
//					{
//						for(int idx=0; idx<4; idx++)
//						{
//							glVertex3f(vert[idx].x,vert[idx].y,vert[idx].z);
//						}
//					}
//					else
//					{
//						for(int idx=0; idx<4; idx++)
//						{
//							glVertex3f(vert[3-idx].x,vert[3-idx].y,vert[3-idx].z);
//						}
//					}
//					glEnd();
//				}
//				else
//				{
//					glBegin(GL_POLYGON);
//					if(i == 0)
//					{
//						for(int idx=0; idx<4; idx++)
//						{
//							glColor4f(clr[idx],0,1-clr[idx],alpha);
//							glVertex3f(vert[idx].x,vert[idx].y,vert[idx].z);
//						}
//					}
//					else
//					{
//						for(int idx=0; idx<4; idx++)
//						{
//							glColor4f(clr[3-idx],0,1-clr[3-idx],alpha);
//							glVertex3f(vert[3-idx].x,vert[3-idx].y,vert[3-idx].z);
//						}
//					}
//					glEnd();
//				}
//			}				
//		}
//		if(bOnlyBoudary && i==0)
//			i = m_uRenderNum-2;
//	}
//	//其次显示v向曲面
//	for(j=0; j<m_vRenderNum; j++)
//	{
//		for(k=0; k<m_wRenderNum-1; k++)
//		{
//			for(i=0; i<m_uRenderNum-1; i++)
//			{		
//				vert[0] = m_vVolumePts.at(GetRenderPointIndex(i,j,k)).m_pt;
//				clr[0] = m_vVolumePts.at(GetRenderPointIndex(i,j,k)).m_matval.x;
//				vert[1] = m_vVolumePts.at(GetRenderPointIndex(i+1,j,k)).m_pt;
//				clr[1] = m_vVolumePts.at(GetRenderPointIndex(i+1,j,k)).m_matval.x;
//				vert[2] = m_vVolumePts.at(GetRenderPointIndex(i+1,j,k+1)).m_pt;
//				clr[2] = m_vVolumePts.at(GetRenderPointIndex(i+1,j,k+1)).m_matval.x;
//				vert[3] = m_vVolumePts.at(GetRenderPointIndex(i,j,k+1)).m_pt;
//				clr[3] = m_vVolumePts.at(GetRenderPointIndex(i,j,k+1)).m_matval.x;
//				if(!m_bHeterogeneous)
//				{
//					glColor4f(colorVt.x,colorVt.y,colorVt.z,alpha);
//					glBegin(GL_POLYGON);
//					if(j==0)
//					{
//						for(int idx=0; idx<4; idx++)
//						{
//							glVertex3f(vert[idx].x,vert[idx].y,vert[idx].z);
//						}
//					}
//					else
//					{
//						glVertex3f(vert[0].x,vert[0].y,vert[0].z);
//						glVertex3f(vert[3].x,vert[3].y,vert[3].z);
//						glVertex3f(vert[2].x,vert[2].y,vert[2].z);
//						glVertex3f(vert[1].x,vert[1].y,vert[1].z);
//					}
//					glEnd();
//				}
//				else
//				{
//					glBegin(GL_POLYGON);
//					if(j == 0)
//					{
//						for(int idx=0; idx<4; idx++)
//						{
//							glColor4f(clr[idx],0,1-clr[idx],alpha);
//							glVertex3f(vert[idx].x,vert[idx].y,vert[idx].z);
//						}
//					}
//					else
//					{
//						glColor4f(clr[0],0,1-clr[0],alpha);
//						glVertex3f(vert[0].x,vert[0].y,vert[0].z);
//						glColor4f(clr[3],0,1-clr[3],alpha);
//						glVertex3f(vert[3].x,vert[3].y,vert[3].z);
//						glColor4f(clr[2],0,1-clr[2],alpha);
//						glVertex3f(vert[2].x,vert[2].y,vert[2].z);
//						glColor4f(clr[1],0,1-clr[1],alpha);
//						glVertex3f(vert[1].x,vert[1].y,vert[1].z);
//					}
//					glEnd();
//				}
//			}				
//		}
//		if(bOnlyBoudary && j==0)
//			j = m_vRenderNum-2;
//	}
//	//最后显示w向等参面。
//	for(k=0; k<m_wRenderNum; k++)
//	{
//		for(i=0; i<m_uRenderNum-1; i++)
//		{
//			for(j=0; j<m_vRenderNum-1; j++)
//			{		
//				vert[0] = m_vVolumePts.at(GetRenderPointIndex(i,j,k)).m_pt;
//				clr[0] = m_vVolumePts.at(GetRenderPointIndex(i,j,k)).m_matval.x;
//				vert[1] = m_vVolumePts.at(GetRenderPointIndex(i,j+1,k)).m_pt;
//				clr[1] = m_vVolumePts.at(GetRenderPointIndex(i,j+1,k)).m_matval.x;
//				vert[2] = m_vVolumePts.at(GetRenderPointIndex(i+1,j+1,k)).m_pt;
//				clr[2] = m_vVolumePts.at(GetRenderPointIndex(i+1,j+1,k)).m_matval.x;
//				vert[3] = m_vVolumePts.at(GetRenderPointIndex(i+1,j,k)).m_pt;
//				clr[3] = m_vVolumePts.at(GetRenderPointIndex(i+1,j,k)).m_matval.x;
//				if(!m_bHeterogeneous)
//				{
//					glColor4f(colorVt.x,colorVt.y,colorVt.z,alpha);
//					glBegin(GL_POLYGON);
//					if(k==0)
//					{
//						for(int idx=0; idx<4; idx++)
//						{
//							glVertex3f(vert[idx].x,vert[idx].y,vert[idx].z);
//						}
//					}
//					else
//					{
//						glVertex3f(vert[0].x,vert[0].y,vert[0].z);
//						glVertex3f(vert[3].x,vert[3].y,vert[3].z);
//						glVertex3f(vert[2].x,vert[2].y,vert[2].z);
//						glVertex3f(vert[1].x,vert[1].y,vert[1].z);
//					}
//					glEnd();
//				}
//				else
//				{
//					glBegin(GL_POLYGON);
//					if(k == 0)
//					{
//						for(int idx=0; idx<4; idx++)
//						{
//							glColor4f(clr[idx],0,1-clr[idx],alpha);
//							glVertex3f(vert[idx].x,vert[idx].y,vert[idx].z);
//						}
//					}
//					else
//					{
//						glColor4f(clr[0],0,1-clr[0],alpha);
//						glVertex3f(vert[0].x,vert[0].y,vert[0].z);
//						glColor4f(clr[3],0,1-clr[3],alpha);
//						glVertex3f(vert[3].x,vert[3].y,vert[3].z);
//						glColor4f(clr[2],0,1-clr[2],alpha);
//						glVertex3f(vert[2].x,vert[2].y,vert[2].z);
//						glColor4f(clr[1],0,1-clr[1],alpha);
//						glVertex3f(vert[1].x,vert[1].y,vert[1].z);
//					}
//					glEnd();
//				}
//			}				
//		}
//		if(bOnlyBoudary && k==0)
//			k = m_wRenderNum-2;
//	}
//	glColor4fv(oldColor);
//}
Vec4  SplineVolume::GetPoint(float u,float v,float w,Vec4& matPt)
{
    Vec4 uvwPt;
	float fourVal;
	int i,j,k;
    uvwPt.x = uvwPt.y = uvwPt.z = 0.f;
	matPt.x = matPt.y = matPt.z = 0.f;
	fourVal = 0.f;

    double Nip,Njq,Nkr;
    varray<double> Nips,Njqs,Nkrs;
	for(i=0; i<m_uNum; i++)
	{
        Nip = OneBasisFun(m_uDegree,m_uKnots.size()-1,m_uKnots,i,u);
		Nips.push_back(Nip);
	}
	for(j=0; j<m_vNum; j++)
	{
		Njq = OneBasisFun(m_vDegree,m_vKnots.size()-1,m_vKnots,j,v);
		Njqs.push_back(Njq);
	}
	for(k=0; k<m_wNum; k++)
	{
		Nkr = OneBasisFun(m_wDegree,m_wKnots.size()-1,m_wKnots,k,w);
		Nkrs.push_back(Nkr);
	}
    for(i=0; i<m_uNum; i++)
	{	
        Nip = Nips.at(i);
		if(Nip == 0)
			continue;
		for(j=0; j<m_vNum; j++)
		{
			Njq = Njqs.at(j);
			if(Njq == 0)
				continue;
			for(k=0; k<m_wNum; k++)
			{
				Nkr = Nkrs.at(k);
				if(Nkr == 0)
					continue;
				uvwPt += Nip * Njq * Nkr * GetControlPoint(i,j,k);
				matPt += Nip * Njq * Nkr * GetControlPointMatvalue(i,j,k);
			}
		}
	}
	return uvwPt;
}
Vec4  SplineVolume::GetControlPoint(int ui,int vi,int wi)
{
 /*   if(ui < 0 || ui >= m_uNum
		|| vi < 0 || vi >= m_vNum
		|| wi < 0 || wi >= m_wNum)
		AfxMessageBox(_T("控制点索引错误")); */
    int id = GetControlPointIndex(ui,vi,wi);
	Vec4 vt = m_vAllCtrlPts.at(id).m_pt;
    return vt;
}
Vec4  SplineVolume::GetControlPointMatvalue(int ui,int vi,int wi)
{
	/*if(ui < 0 || ui >= m_uNum
		|| vi < 0 || vi >= m_vNum
		|| wi < 0 || wi >= m_wNum)
		AfxMessageBox(_T("控制点索引错误"));*/ 
	int id = GetControlPointIndex(ui,vi,wi);
	Vec4 vt = m_vAllCtrlPts.at(id).m_matval;
	return vt;
}
int SplineVolume::GetControlPointIndex(int ui,int vi,int wi)
{
	int index = ui + vi * m_uNum + wi * (m_uNum * m_vNum);
    return index;
}
bool  SplineVolume::IsBoundaryPoint(int ui,int vi,int wi,int unum,int vnum,int wnum)
{
    if(ui == 0 || vi == 0 || wi == 0 || ui == unum-1 || vi == vnum-1 || wi == wnum-1)
		return true;
	return false;
}
bool  SplineVolume::GetIJKIndex(int i, int& ui,int&vi,int& wi,int unum,int vnum,int wnum)
{
	if(i < 0 || i >= m_vAllCtrlPts.size())
		return false;
    wi = i / (unum * vnum);
	i -= wi * (unum * vnum);
	vi = i / unum;
	i -= vi*unum;
	ui = i;
	if((ui >= 0 || ui < unum) && (vi >= 0 || vi < vnum) && (wi >= 0 || wi < wnum))
	    return true;
    return false;
}
int SplineVolume::GetRenderPointIndex(int ui,int vi,int wi)
{
	int index = ui + vi * m_uRenderNum + wi * (m_uRenderNum * m_vRenderNum);
    return index;
}
//节点数目计算：m = n + p + 1 
void  SplineVolume::SetControlPtNum(int un,int vn,int wn)
{
    m_uNum = un; 
	m_vNum = vn;
	m_wNum = wn;

	//set knot;
	m_uKnots.clear();
	m_vKnots.clear();
	m_wKnots.clear();

	int i=0;
    for(i=0; i<=m_uDegree; i++)
		m_uKnots.push_back(0.f);
	for(i=0; i<=m_vDegree; i++)
		m_vKnots.push_back(0.f);
	for(i=0; i<=m_wDegree; i++)
		m_wKnots.push_back(0.f);

	int seg = m_uNum + m_uDegree + 1;
	seg -= 2 * (m_uDegree + 1);
	seg += 1;
    for(i=1; i<seg; i++)
		m_uKnots.push_back(1.f/seg * i);

	seg = m_vNum + m_vDegree + 1;
	seg -= 2 * (m_vDegree + 1);
	seg += 1;
	for(i=1; i<seg; i++)
		m_vKnots.push_back(1.f/seg * i);

	seg = m_wNum + m_wDegree + 1;
	seg -= 2 * (m_wDegree + 1);
	seg += 1;
	for(i=1; i<seg; i++)
		m_wKnots.push_back(1.f/seg * i);

	for(i=0; i<=m_uDegree; i++)
		m_uKnots.push_back(1.f);
	for(i=0; i<=m_vDegree; i++)
		m_vKnots.push_back(1.f);
	for(i=0; i<=m_wDegree; i++)
		m_wKnots.push_back(1.f);
}
void  SplineVolume::SetDegree(int ud,int vd,int wd)
{
	m_uDegree = ud;
	m_vDegree = vd;
	m_wDegree = wd;
}
void  SplineVolume::SetControlPoints(varray<Vec4>& vts)
{
    Clear();
	int nsize = vts.size();
	for(int i=0; i<nsize; i++)
	{
         m_vAllCtrlPts.push_back(VolumeVertex(vts.at(i)));
	}
}
void  SplineVolume::Clear()
{
	m_uKnots.clear();
	m_vKnots.clear();
	m_wKnots.clear();
	m_vAllCtrlPts.clear();
	m_vVolumePts.clear();
	m_bHasIGASolution = false;
}
void  SplineVolume::SetControlPoints(varray<Vec4>& vts,int udegree,int vdegree,int wdegree, int un,int vn,int wn)
{
	Clear();
	SetControlPoints(vts);
	SetDegree(udegree,vdegree,wdegree);
	SetControlPtNum(un,vn,wn); //控制点，取决于表面控制点
	//首先判断要不要进行控制点插值
	if(JudgeNeedInterpolate())
	{			
		//InterpolateallControlPtsByLaplacian();
		//LoopInterpolateAllControlPtsByMeanValue();	
		InterpolateAllControlPtsByCoons();
	}	
	//TestMeshQuanity(20);
	CreateAllVolumeRenderPts(6,6,6); //渲染点，可以随便取
	//SaveVolume();
}
void  SplineVolume::SetControlPoints(varray<Vec4>& vts,varray<Vec4>& mts)
{
	Clear();
	int nsize = vts.size();
	for(int i=0; i<nsize; i++)
	{
		m_vAllCtrlPts.push_back(VolumeVertex(vts.at(i),mts.at(i)));
	}
	m_bHeterogeneous = true;
}
void  SplineVolume::Get6SurfaceCtrlPtsFromPts(varray<varray<Vec4> >& surfacePts,int uvwmode)
{
	surfacePts.clear();
	if(uvwmode == 0) //for u向两个面
	{
		surfacePts.resize(m_wNum);
		for(int k=0; k<m_wNum; k++)
		{
			for(int j=0; j<m_vNum; j++)
			{
				surfacePts.at(k).push_back(GetControlPoint(0,j,k));
			}
		}
	}
	else if(uvwmode == 1) //for u向两个面
	{
		surfacePts.resize(m_wNum);
		for(int k=0; k<m_wNum; k++)
		{
			for(int j=0; j<m_vNum; j++)
			{
				surfacePts.at(k).push_back(GetControlPoint(m_uNum-1,j,k));
			}
		}
	}
	else if(uvwmode == 2) //for v向两个面
	{
		surfacePts.resize(m_wNum);
		for(int k=0; k<m_wNum; k++)
		{
			for(int i=0; i<m_uNum; i++)
			{
				surfacePts.at(k).push_back(GetControlPoint(i,0,k));
			}
		}
	}
	else if(uvwmode == 3) //for v向两个面
	{
		surfacePts.resize(m_wNum);
		for(int k=0; k<m_wNum; k++)
		{
			for(int i=0; i<m_uNum; i++)
			{
				surfacePts.at(k).push_back(GetControlPoint(i,m_vNum-1,k));
			}
		}
	}
	else if(uvwmode == 4) //for w向两个面
	{
		surfacePts.resize(m_vNum);
		for(int j=0; j<m_vNum; j++)
		{
			for(int i=0; i<m_uNum; i++)
			{
				surfacePts.at(j).push_back(GetControlPoint(i,j,0));
			}
		}
	}
	else if(uvwmode == 5) //for w向两个面
	{
		surfacePts.resize(m_vNum);
		for(int j=0; j<m_vNum; j++)
		{
			for(int i=0; i<m_uNum; i++)
			{
				surfacePts.at(j).push_back(GetControlPoint(i,j,m_wNum-1));
			}
		}
	}
}
void  SplineVolume::GetLocalScaleForPoint(double scale[],int ciu,int civ,int ciw,bool laplacian,bool isinitial)
{
	//laplacian 方法的参数设置法
	if(laplacian)  
	{
         for(int i=0; i<27; i++)
			 scale[i] = 0;
		 int l2 = (m_uNum-1)*(m_uNum-1);
		 int m2 = (m_vNum-1)*(m_vNum-1);
		 int n2 = (m_wNum-1)*(m_wNum-1);
		 int lmn = 2*(l2+m2+n2);
         scale[10] = scale[16] = (float)l2/lmn;
		 scale[12] = scale[14] = (float)m2/lmn;
		 scale[4] = scale[22] = (float)n2/lmn;
		 scale[13] = -1;
	}
	else  //以下比例值似乎不太起作用。
	{
		if(isinitial)
		{
			double alpha = 1/(6 + 6*sqrt(2.f) + 8*sqrt(3.f));
			double surfacescale,edgescale,pointscale;
			surfacescale = alpha;
			edgescale = alpha*sqrt(2.f);
			pointscale = alpha*sqrt(3.f);
			//surfacescale = edgescale = pointscale = 1.f/26;  //运行通过
			scale[0] = scale[2] = scale[6] = scale[8] = pointscale;
			scale[0+18] = scale[2+18] = scale[6+18] = scale[8+18] = pointscale;
			scale[1] = scale[3] = scale[5] = scale[7] = edgescale;
			scale[9] = scale[11] = scale[15] = scale[17] = edgescale;
			scale[1+18] = scale[3+18] = scale[5+18] = scale[7+18] = edgescale;
			scale[4] = scale[10] = scale[12] = scale[14] = scale[16] = scale[22] = pointscale;
			scale[13] = -1;
		}
		else
		{
			varray<Vec4> pts;
			int muu,mvv,mww;
			Vec4 pt;
			for(int h=0;h<27; h++)
			{
				ModifyLocalIndexForCube(h,muu,mvv,mww);
				pt = m_vAllCtrlPts.at(GetControlPointIndex(ciu+muu,civ+mvv,ciw+mww)).m_pt;
				pts.push_back(pt);
			}
			double sum = 0;
			double sc;
			double scaleedge,scalepoint;
			varray<double> vscale;
			scaleedge = sqrt(2.0);
			scalepoint = sqrt(3.0);
			for(int i=0; i<27; i++)
			{
				if(i==13)
				{
					vscale.push_back(0.0);
					continue;
				}
				sc = /*1.0/*/(pts[i]-pts[13]).Magnitude();
				if(i==1||i==3||i==5||i==7||i==9||i==11||i==15||i==17||i==19||i==21||i==23||i==25)
					sc *= scaleedge;
				if(i==0||i==2||i==6||i==8||i==18||i==20||i==24||i==26)
					sc *= scalepoint;
				sum += sc;
				vscale.push_back(sc);
			}
			for(int i=0; i<27; i++)
			{
				scale[i] = vscale.at(i)/(float)sum;
			}
			scale[13] = -1;
		}
	}
}
//void  SplineVolume::LoopInterpolateAllControlPtsByMeanValue(bool isinitial)
//{
//    for(int i=0; i<20; i++)  //循环还是有用的。
//	{
//		InterpolateAllControlPtsByMeanValue(i==0);
//	}
//}
//void  SplineVolume::InterpolateAllControlPtsByMeanValue(bool isinitial)
//{
//    //初始化比例
//	double allscale[27];
//
//    //设置矩阵
//	//初始化矩阵
//	Matrix1Dim scaleMatrix;  //先u向、再v向、最后在w向。
//	int nsz = (m_uNum-2)*(m_vNum-2)*(m_wNum-2);
//	scaleMatrix.dim = nsz;
//	int currowid,hidx;
//	bool isdoundary;
//	//初始化右列值
//	varray<double> bx,by,bz;
//	bx.resize(nsz);
//	by.resize(nsz);
//	bz.resize(nsz);
//	for(int i=0; i<nsz; i++)
//	{
//		bx.at(i) = 0; by.at(i) = 0; bz.at(i) = 0;
//	} 
//  
//	int muu,mvv,mww;
//	Vec4 virtualPt;
//	for(int k=1; k<m_wNum-1; k++)
//	{
//	    for(int j=1; j<m_vNum-1; j++)
//		{
//		     for(int i=1; i<m_uNum-1; i++)
//			 {
//				  GetLocalScaleForPoint(allscale,i,j,k,false,isinitial);
//				  currowid = GetIndexForVolumeMatrix(i-1,j-1,k-1,m_uNum-2,m_vNum-2,m_wNum-2,13,isdoundary);
//				  for(int h=0;h<27; h++)
//				  {
//					  hidx = GetIndexForVolumeMatrix(i-1,j-1,k-1,m_uNum-2,m_vNum-2,m_wNum-2,h,isdoundary);
//					  if(!isdoundary)
//                          scaleMatrix.push_back(allscale[h],currowid,hidx);
//					  else
//					  {
//                          ModifyLocalIndexForCube(h,muu,mvv,mww);
//						  virtualPt = m_vAllCtrlPts.at(GetControlPointIndex(i+muu,j+mvv,k+mww)).m_pt;
//                          bx.at(currowid) -= virtualPt.x*allscale[h];
//                          by.at(currowid) -= virtualPt.y*allscale[h];
//						  bz.at(currowid) -= virtualPt.z*allscale[h];
//					  }
//				  }                  
//			 }
//		}
//	}
//
//	//求解：
//    XMatrixMath solveMatrix;
//    varray<double> vx,vy,vz;
//	int oriidx;
//	solveMatrix.solveSparseLinearEquation(scaleMatrix,vx,bx);
//	solveMatrix.solveSparseLinearEquation(scaleMatrix,vy,by);
//	solveMatrix.solveSparseLinearEquation(scaleMatrix,vz,bz);
//	nsz = bx.size();
//	for(int i=0; i<nsz; i++)
//	{
//		 mww = i/((m_uNum-2)*(m_vNum-2));
//		 mvv = (i - mww * (m_uNum-2)*(m_vNum-2))/(m_uNum-2);
//		 muu = i - mww * (m_uNum-2)*(m_vNum-2) - mvv * (m_uNum-2);
//         oriidx = GetControlPointIndex(muu+1,mvv+1,mww+1);
//         m_vAllCtrlPts.at(oriidx).m_pt.x = vx.at(i);
//		 m_vAllCtrlPts.at(oriidx).m_pt.y = vy.at(i);
//		 m_vAllCtrlPts.at(oriidx).m_pt.z = vz.at(i);
//	}
//}
//void  SplineVolume::InterpolateallControlPtsByLaplacian()
//{
//	//初始化比例
//	double allscale[27];
//
//	//设置矩阵
//	//初始化矩阵
//	Matrix1Dim scaleMatrix;  //先u向、再v向、最后在w向。
//	int nsz = (m_uNum-2)*(m_vNum-2)*(m_wNum-2);
//	scaleMatrix.dim = nsz;
//	int currowid,hidx;
//	bool isdoundary;
//	//初始化右列值
//	varray<double> bx,by,bz;
//	bx.resize(nsz);
//	by.resize(nsz);
//	bz.resize(nsz);
//	for(int i=0; i<nsz; i++)
//	{
//		bx.at(i) = 0; by.at(i) = 0; bz.at(i) = 0;
//	} 
//
//	int muu,mvv,mww;
//	Vec4 virtualPt;
//	for(int k=1; k<m_wNum-1; k++)
//	{
//		for(int j=1; j<m_vNum-1; j++)
//		{
//			for(int i=1; i<m_uNum-1; i++)
//			{
//				GetLocalScaleForPoint(allscale,i,j,k,true,true);
//				currowid = GetIndexForVolumeMatrix(i-1,j-1,k-1,m_uNum-2,m_vNum-2,m_wNum-2,13,isdoundary);
//				for(int h=0;h<27; h++)
//				{
//					if(abs(allscale[h])<1.0e-6)
//						continue;
//					hidx = GetIndexForVolumeMatrix(i-1,j-1,k-1,m_uNum-2,m_vNum-2,m_wNum-2,h,isdoundary);
//					if(!isdoundary)
//						scaleMatrix.push_back(allscale[h],currowid,hidx);
//					else
//					{
//						ModifyLocalIndexForCube(h,muu,mvv,mww);
//						virtualPt = m_vAllCtrlPts.at(GetControlPointIndex(i+muu,j+mvv,k+mww)).m_pt;
//						bx.at(currowid) -= virtualPt.x*allscale[h];
//						by.at(currowid) -= virtualPt.y*allscale[h];
//						bz.at(currowid) -= virtualPt.z*allscale[h];
//					}
//				}                  
//			}
//		}
//	}
//
//	//求解：
//	XMatrixMath solveMatrix;
//	varray<double> vx,vy,vz;
//	int oriidx;
//	solveMatrix.solveSparseLinearEquation(scaleMatrix,vx,bx);
//	solveMatrix.solveSparseLinearEquation(scaleMatrix,vy,by);
//	solveMatrix.solveSparseLinearEquation(scaleMatrix,vz,bz);
//	nsz = bx.size();
//	for(int i=0; i<nsz; i++)
//	{
//		mww = i/((m_uNum-2)*(m_vNum-2));
//		mvv = (i - mww * (m_uNum-2)*(m_vNum-2))/(m_uNum-2);
//		muu = i - mww * (m_uNum-2)*(m_vNum-2) - mvv * (m_uNum-2);
//		oriidx = GetControlPointIndex(muu+1,mvv+1,mww+1);
//		m_vAllCtrlPts.at(oriidx).m_pt.x = vx.at(i);
//		m_vAllCtrlPts.at(oriidx).m_pt.y = vy.at(i);
//		m_vAllCtrlPts.at(oriidx).m_pt.z = vz.at(i);
//	}
//}

SplineSurface SplineVolume::GetSingleSurfaces(int dir) const
{
	SplineSurface sf;
	assert(dir >= 1 && dir <= 6);
	assert(this->m_CtrlPts.size());
	//if (dir < 1 || dir>6)
	//	return {};
	if (dir == 1) {
		//UV面,W=0
		sf.m_uDegree = this->m_uDegree;
		sf.m_vDegree = this->m_vDegree;
		sf.m_uNum = this->m_uNum;
		sf.m_vNum = this->m_vNum;
		sf.m_uKnots = this->m_uKnots;
		sf.m_vKnots = this->m_vKnots;
		for (int i = 0; i < this->m_vNum; i++) {
			for (int j = 0; j < this->m_uNum; j++) {
				sf.m_CtrlPts.push_back(this->m_CtrlPts[i*this->m_uNum + j]);
			}
		}
	}

	else if (dir == 2) {
		//UW面，V=0
		sf.m_uDegree = this->m_uDegree;
		sf.m_vDegree = this->m_wDegree;
		sf.m_uNum = this->m_uNum;
		sf.m_vNum = this->m_wNum;
		sf.m_uKnots = this->m_uKnots;
		sf.m_vKnots = this->m_wKnots;
		for (int i = 0; i < this->m_wNum; i++) {
			for (int j = 0; j < this->m_uNum; j++) {
				sf.m_CtrlPts.push_back(this->m_CtrlPts[i*(this->m_uNum*this->m_vNum) + j]);
			}
		}
	}

	else if (dir == 5) {
		//VW面，U=0
		sf.m_uDegree = this->m_vDegree;
		sf.m_vDegree = this->m_wDegree;
		sf.m_uNum = this->m_vNum;
		sf.m_vNum = this->m_wNum;
		sf.m_uKnots = this->m_vKnots;
		sf.m_vKnots = this->m_wKnots;
		for (int i = 0; i < this->m_wNum; i++) {
			for (int j = 0; j < this->m_vNum; j++) {
				sf.m_CtrlPts.push_back(this->m_CtrlPts[i*(this->m_uNum*this->m_vNum) + j * m_uNum]);
			}
		}
	}

	else if (dir == 6) {
		//UV面,W=1
		sf.m_uDegree = this->m_uDegree;
		sf.m_vDegree = this->m_vDegree;
		sf.m_uNum = this->m_uNum;
		sf.m_vNum = this->m_vNum;
		sf.m_uKnots = this->m_uKnots;
		sf.m_vKnots = this->m_vKnots;
		int pos = this->m_uNum*this->m_vNum*(this->m_wNum - 1);		//第一个点的位置
		for (int i = 0; i < this->m_vNum; i++) {
			for (int j = 0; j < this->m_uNum; j++) {
				sf.m_CtrlPts.push_back(this->m_CtrlPts[pos + i * this->m_uNum + j]);
			}
		}
	}

	return sf;
}

void SplineVolume::VolCoonsInterpolate(const varray<Spline>& EdgeLines)
{
	//RWGeometric rw;
	varray<Spline> sfLines;
	varray<SplineSurface> sfs;
	SplineSurface tmpsf;
	//0号面
	sfLines.clear();
	sfLines.push_back(EdgeLines[0]);
	sfLines.push_back(EdgeLines[1]);
	sfLines.push_back(EdgeLines[2]);
	sfLines.push_back(EdgeLines[3]);
	tmpsf.CoonsInterpolate(sfLines);
	sfs.push_back(tmpsf);
	//1号面
	sfLines.clear();
	sfLines.push_back(EdgeLines[0]);
	sfLines.push_back(EdgeLines[4]);
	sfLines.push_back(EdgeLines[8]);
	sfLines.push_back(EdgeLines[7]);
	tmpsf.CoonsInterpolate(sfLines);
	sfs.push_back(tmpsf);
	//2号面
	sfLines.clear();
	sfLines.push_back(EdgeLines[1]);
	sfLines.push_back(EdgeLines[4]);
	sfLines.push_back(EdgeLines[9]);
	sfLines.push_back(EdgeLines[5]);
	tmpsf.CoonsInterpolate(sfLines);
	sfs.push_back(tmpsf);
	//3号面
	sfLines.clear();
	sfLines.push_back(EdgeLines[2]);
	sfLines.push_back(EdgeLines[5]);
	sfLines.push_back(EdgeLines[10]);
	sfLines.push_back(EdgeLines[6]);
	tmpsf.CoonsInterpolate(sfLines);
	sfs.push_back(tmpsf);
	//4号面
	sfLines.clear();
	sfLines.push_back(EdgeLines[3]);
	sfLines.push_back(EdgeLines[7]);
	sfLines.push_back(EdgeLines[11]);
	sfLines.push_back(EdgeLines[6]);
	tmpsf.CoonsInterpolate(sfLines);
	sfs.push_back(tmpsf);
	//5号面
	sfLines.clear();
	sfLines.push_back(EdgeLines[8]);
	sfLines.push_back(EdgeLines[9]);
	sfLines.push_back(EdgeLines[10]);
	sfLines.push_back(EdgeLines[11]);
	tmpsf.CoonsInterpolate(sfLines);
	sfs.push_back(tmpsf);

	//UV换序
	for (auto& s : sfs) {
		s.OrderCtrlPts();
	}
	//rw.WriteNurbsSurface("D:\\冯文斌大论文\\素材\\示意图txt\\5.2COONS插值体\\5.2sf.txt", sfs);
	int num_u, num_v, num_w;
	num_u = sfs[0].m_uNum;
	num_v = sfs[0].m_vNum;
	num_w = sfs[1].m_vNum;
	this->m_uNum = num_u;
	this->m_vNum = num_v;
	this->m_wNum = num_w;
	this->m_uDegree = sfs[0].m_uDegree;
	this->m_vDegree = sfs[0].m_vDegree;
	this->m_wDegree = sfs[1].m_vDegree;
	this->m_uKnots = sfs[0].m_uKnots;
	this->m_vKnots = sfs[0].m_vKnots;
	this->m_wKnots = sfs[1].m_vKnots;
	for (auto p : sfs[0].m_CtrlPts) {
		this->m_CtrlPts.push_back(p);
	}
	for (int i = 1; i < num_w - 1; i++) {
		varray<Spline> edgeL;
		Spline tmpl;
		SplineSurface tmps;
		//e1
		tmpl.m_Degree = sfs[1].m_uDegree;
		tmpl.m_Knots = sfs[1].m_uKnots;
		tmpl.m_CtrlPts.clear();
		for (int j = 0; j < num_u; j++) {
			tmpl.m_CtrlPts.push_back(sfs[1].m_CtrlPts[i*num_u + j]);
		}
		edgeL.push_back(tmpl);
		//e2
		tmpl.m_Degree = sfs[2].m_uDegree;
		tmpl.m_Knots = sfs[2].m_uKnots;
		tmpl.m_CtrlPts.clear();
		for (int j = 0; j < num_v; j++) {
			tmpl.m_CtrlPts.push_back(sfs[2].m_CtrlPts[i*num_v + j]);
		}
		edgeL.push_back(tmpl);
		//e3
		tmpl.m_Degree = sfs[3].m_uDegree;
		tmpl.m_Knots = sfs[3].m_uKnots;
		tmpl.m_CtrlPts.clear();
		for (int j = 0; j < num_u; j++) {
			tmpl.m_CtrlPts.push_back(sfs[3].m_CtrlPts[i*num_u + j]);
		}
		edgeL.push_back(tmpl);
		//e4
		tmpl.m_Degree = sfs[4].m_uDegree;
		tmpl.m_Knots = sfs[4].m_uKnots;
		tmpl.m_CtrlPts.clear();
		for (int j = 0; j < num_v; j++) {
			tmpl.m_CtrlPts.push_back(sfs[4].m_CtrlPts[i*num_v + j]);
		}
		edgeL.push_back(tmpl);
		tmps.CoonsInterpolate(edgeL);
		tmps.OrderCtrlPts();
		for (auto&p : tmps.m_CtrlPts) {
			this->m_CtrlPts.push_back(p);
		}
	}
	for (auto& p : sfs[5].m_CtrlPts) {
		this->m_CtrlPts.push_back(p);
	}
}

//iu,iv,iw是中心点的index,localidx 是在正方体中的index。
//localidx 是每个点在正方体内的编号，从0~26
int SplineVolume::GetIndexForVolumeMatrix(int iu,int iv,int iw,int su,int sv,int sw,int localidx,bool& boundaryflag)
{
	int ulocal,vlocal,wlocal;
	ModifyLocalIndexForCube(localidx,ulocal,vlocal,wlocal);
	iu += ulocal;
	iv += vlocal;
	iw += wlocal;
	boundaryflag = false;
	if(iu == -1 || iv == -1 || iw == -1 || iu == su || iv == sv || iw == sw)
		boundaryflag = true;
    return iu + iv * su + iw * su * sv;
}
//一个正方体的控制点数目为su,sv,sw
//给定某个点在正方体内的编号，得到其对应的点相对于中心的编号。
void SplineVolume::ModifyLocalIndexForCube(int hindex,int& mu,int& mv,int& mw)
{
    mu = mv = mw = 0;
	int wlocal = hindex/9;
	int vlocal = (hindex - wlocal*9)/3;
	int ulocal = hindex - wlocal*9 - vlocal*3;
	wlocal -= 1;
	vlocal -= 1;
	ulocal -= 1;
	mu = ulocal;
	mv = vlocal;
	mw = wlocal;
};
void SplineVolume::InterpolateAllControlPtsByCoons()
{
	Vec4 volumePt;
	int l = m_uNum-1;
	int m = m_vNum-1;
	int n = m_wNum-1;
	float u,v,w;
    for(int k=1; k<n; k++)
	{
        w = (float)k/(float)n;
		for(int j=1;j<m; j++)
		{
			v = (float)j/(float)m;
			for(int i=1; i<l; i++)
			{
				u =(float) i/(float)l;
                  volumePt.x = volumePt.y = volumePt.z = 0.f;

                  volumePt += (1-u)*GetControlPoint(0,j,k) + u*GetControlPoint(l,j,k);
				  volumePt += (1-v)*GetControlPoint(i,0,k) + v*GetControlPoint(i,m,k);
                  volumePt += (1-w)*GetControlPoint(i,j,0) + w*GetControlPoint(i,j,n);

                  volumePt -= (1-u)*(1-v)*GetControlPoint(0,0,k) + (1-u)*v*GetControlPoint(0,m,k);
				  volumePt -= u*(1-v)*GetControlPoint(l,0,k) + u*v*GetControlPoint(l,m,k);
				  volumePt -= (1-v)*(1-w)*GetControlPoint(i,0,0) + (1-v)*w*GetControlPoint(i,0,n);
				  volumePt -= v*(1-w)*GetControlPoint(i,m,0) + v*w*GetControlPoint(i,m,n);
				  volumePt -= (1-w)*(1-u)*GetControlPoint(0,j,0) + (1-w)*u*GetControlPoint(l,j,0);
				  volumePt -= w*(1-u)*GetControlPoint(0,j,n) + w*u*GetControlPoint(l,j,n);

				  volumePt += (1-w)*((1-u)*(1-v)*GetControlPoint(0,0,0) + (1-u)*v*GetControlPoint(0,m,0));
				  volumePt += (1-w)*(u*(1-v)*GetControlPoint(l,0,0) + u*v*GetControlPoint(l,m,0));
				  volumePt += w*((1-u)*(1-v)*GetControlPoint(0,0,n) + (1-u)*v*GetControlPoint(0,m,n));
				  volumePt += w*(u*(1-v)*GetControlPoint(l,0,n) + u*v*GetControlPoint(l,m,n));

				  m_vAllCtrlPts.at(GetControlPointIndex(i,j,k)) = VolumeVertex(volumePt);
			}
		}
	}
};
float SplineVolume::GetJacobianValue(float u,float v,float w)
{
	float nip,njq,nkr;
	varray<double> Nips,Njqs,Nkrs,Nip1s,Njq1s,Nkr1s;
	int i,j,k,i0,j0,k0,i1,j1,k1,i2,j2,k2;
	varray<double> u1Knots = m_uKnots; 
	u1Knots.pop_front(); u1Knots.pop_back();
	varray<double> v1Knots = m_vKnots; 
	v1Knots.pop_front(); v1Knots.pop_back();
	varray<double> w1Knots = m_wKnots; 
	w1Knots.pop_front(); w1Knots.pop_back();
	for(i=0; i<m_uNum; i++)
	{
		nip = OneBasisFun(m_uDegree,m_uKnots.size()-1,m_uKnots,i,u);
		Nips.push_back(nip);
	}
	for(i=0; i<m_uNum-1; i++)
	{
		nip = OneBasisFun(m_uDegree-1,u1Knots.size()-1,u1Knots,i,u);
		Nip1s.push_back(nip);
	}
	for(j=0; j<m_vNum; j++)
	{
		njq = OneBasisFun(m_vDegree,m_vKnots.size()-1,m_vKnots,j,v);
		Njqs.push_back(njq);
	}
	for(j=0; j<m_vNum-1; j++)
	{
		njq = OneBasisFun(m_vDegree-1,v1Knots.size()-1,v1Knots,j,v);
		Njq1s.push_back(njq);
	}
	for(k=0; k<m_wNum; k++)
	{
		nkr = OneBasisFun(m_wDegree,m_wKnots.size()-1,m_wKnots,k,w);
		Nkrs.push_back(nkr);
	}
	for(k=0; k<m_wNum-1; k++)
	{
		nkr = OneBasisFun(m_wDegree-1,w1Knots.size()-1,w1Knots,k,w);
		Nkr1s.push_back(nkr);
	}

	float nip2,nip1,nip0,njq2,njq1,njq0,nkr2,nkr1,nkr0;
	Vec4 vi,vj,vk;
	float Jac = 0;
	float PM;
	float degreeProduct = m_uDegree*m_vDegree*m_wDegree;
	float pm0,pm1,pm2;
	Vec4 crossVjvkproduct;
	for(k2=0; k2<m_wNum-1;k2++)
	{
		nkr2 = Nkr1s.at(k2);
		if(abs(nkr2)<1.0e-6)
			break;
		pm2 = nkr2;
		for(j2=0;j2<m_vNum;j2++)
		{
			njq2 = Njqs.at(j2);
			if(abs(njq2)<1.0e-6)
				break;
			pm2 *= njq2;
            for(i2=0; i2<m_uNum;i2++)
			{
				nip2 = Nips.at(i2);
				if(abs(nip2)<1.0e-6)
					break;
				vk = GetControlPoint(i2,j2,k2+1) - GetControlPoint(i2,j2,k2);
				pm2 *= nip2;
				for(k1=0; k1<m_wNum;k1++)
				{
					nkr1 = Nkrs.at(k1);
					if(abs(nkr1)<1.0e-6)
						break;
					pm1 = nkr1;
					for(j1=0;j1<m_vNum-1;j1++)
					{
						njq1 = Njq1s.at(j1);
						if(abs(njq1)<1.0e-6)
							break;
						pm1 *= njq1;
						for(i1=0; i1<m_uNum;i1++)
						{
							nip1 = Nips.at(i1);
							if(abs(nip1)<1.0e-6)
								break;
							pm1 *= nip1;
							vj = GetControlPoint(i1,j1+1,k1) - GetControlPoint(i1,j1,k1);
							crossVjvkproduct = vj.CrossVecX(vk);
							pm1 = nip1*njq1*nkr1;
							for(k0=0; k0<m_wNum;k0++)
							{
								nkr0 = Nkrs.at(k0);
								if(abs(nkr0)<1.0e-6)
									break;
								pm0 = nkr0;
								for(j0=0;j0<m_vNum;j0++)
								{
									njq0 = Njqs.at(j0);
									if(abs(njq0)<1.0e-6)
										break;
									pm0 *= njq0;
									for(i0=0; i0<m_uNum-1;i0++)
									{
										 nip0 = Nip1s.at(i0);  
										 if(abs(nip0)<1.0e-6)
											 break;
										 pm0 *= nip0;
                                         vi = GetControlPoint(i0+1,j0,k0) - GetControlPoint(i0,j0,k0);
										 PM = pm0*pm1*pm2;
										 PM *= degreeProduct;
										 PM *= vi.dotmultiple(crossVjvkproduct);
                                         Jac += PM;
									}
								}
							}
						}
					}
				}
			}
		}
	}
	return Jac;
}
float  SplineVolume::GetFirstPartialDirivatives(float u,float v,float w,Vec4& pu,Vec4& pv,Vec4& pw)
{
	 int i,j,k;
	 Vec4 tup,tvp,twp,deltu,deltv,deltw;
	 float nip,njq,nkr;
	 float uvwscale;
	 varray<double> Nips,Njqs,Nkrs,Nip1s,Njq1s,Nkr1s;
	 varray<double> u1Knots = m_uKnots; u1Knots.pop_front(); u1Knots.pop_back();
	 varray<double> v1Knots = m_vKnots; v1Knots.pop_front(); v1Knots.pop_back();
	 varray<double> w1Knots = m_wKnots; w1Knots.pop_front(); w1Knots.pop_back();
	 for(i=0; i<m_uNum; i++)
	 {
		 nip = OneBasisFun(m_uDegree,m_uKnots.size()-1,m_uKnots,i,u);
		 Nips.push_back(nip);
	 }
	 for(i=0; i<m_uNum-1; i++)
	 {
		 nip = OneBasisFun(m_uDegree-1,u1Knots.size()-1,u1Knots,i,u);
		 Nip1s.push_back(nip);
	 }
	 for(j=0; j<m_vNum; j++)
	 {
		 njq = OneBasisFun(m_vDegree,m_vKnots.size()-1,m_vKnots,j,v);
		 Njqs.push_back(njq);
	 }
	 for(j=0; j<m_vNum-1; j++)
	 {
		 njq = OneBasisFun(m_vDegree-1,v1Knots.size()-1,v1Knots,j,v);
		 Njq1s.push_back(njq);
	 }
	 for(k=0; k<m_wNum; k++)
	 {
		 nkr = OneBasisFun(m_wDegree,m_wKnots.size()-1,m_wKnots,k,w);
		 Nkrs.push_back(nkr);
	 }
	 for(k=0; k<m_wNum-1; k++)
	 {
		 nkr = OneBasisFun(m_wDegree-1,w1Knots.size()-1,w1Knots,k,w);
		 Nkr1s.push_back(nkr);
	 }

	 tup = Vec4(0.f,0.f,0.f);
	 for(k=0; k<m_wNum; k++)
	 {
		 nkr = Nkrs.at(k);
		 if(abs(nkr)<1.0e-6)
             break;
		 for(j=0; j<m_vNum; j++)
		 {
			 njq = Njqs.at(j);
			 if(abs(njq)<1.0e-6)
				 break;
			 for(i=0; i<m_uNum-1; i++)
			 {
				 nip = Nip1s.at(i);
				 if(abs(nip)<1.0e-6)
					 break;
				 deltu = (GetControlPoint(i+1,j,k) - GetControlPoint(i,j,k));
				 uvwscale = (float)m_uDegree/(m_uKnots.at(i+m_uDegree+1)-m_uKnots.at(i+1));
				 deltu = deltu*uvwscale;
				 tup += nip*njq*nkr*deltu;
			 }
		 }
	 }
	
	 tvp = Vec4(0.f,0.f,0.f);
	 for(k=0; k<m_wNum; k++)
	 {
		 nkr = Nkrs.at(k);
		 if(abs(nkr)<1.0e-6)
			 continue;
		 for(j=0; j<m_vNum-1; j++)
		 {
			 njq = Njq1s.at(j);
			 if(abs(njq)<1.0e-6)
				 continue;
			 uvwscale = (float)m_vDegree/(m_vKnots.at(j+m_vDegree+1)-m_vKnots.at(j+1));
			 for(i=0; i<m_uNum; i++)
			 {
				 nip = Nips.at(i);
				 if(abs(nip)<1.0e-6)
					 continue;
				 deltv = (GetControlPoint(i,j+1,k) - GetControlPoint(i,j,k));
				 deltv = deltv * uvwscale;
				 tvp += nip*njq*nkr*deltv;
			 }
		 }
	 }

	 twp = Vec4(0.f,0.f,0.f);
	 for(k=0; k<m_wNum-1; k++)
	 {
		 nkr = Nkr1s.at(k);
		 if(abs(nkr)<1.0e-6)
			 continue;
		 uvwscale = (float)m_wDegree/(m_wKnots.at(k+m_wDegree+1)-m_wKnots.at(k+1));
		 for(j=0; j<m_vNum; j++)
		 {
			 njq = Njqs.at(j);
			 if(abs(njq)<1.0e-6)
				 continue;
			 for(i=0; i<m_uNum; i++)
			 {
				 nip = Nips.at(i);
				 if(abs(nip)<1.0e-6)
					 continue;
				 deltw = (GetControlPoint(i,j,k+1) - GetControlPoint(i,j,k));
				 deltw = deltw * uvwscale;
				 twp += nip*njq*nkr*deltw;
			 }
		 }
	 }
	 pu = tup;
	 pv = tvp;
	 pw = twp;
	 float KK = tup.x*tup.x+tup.y*tup.y+tup.z*tup.z;
	 KK += tvp.x*tvp.x+tvp.y*tvp.y+tvp.z*tvp.z;
	 KK += twp.x*twp.x+twp.y*twp.y+twp.z*twp.z;
	 return KK;
}
void   SplineVolume::TestMeshQuanity(int segmentNum)
{
	m_minorJocbianNum = 0;
	m_isovalAdd = 0;
	m_minJocbian = 1.0e6;
	m_maxJocbian = -1.0e6;

	float u,v,w;
	 u =0.f;v=0.f;w=0.f;
	 int i,j,k;
	 varray<float> Jacs;
	 varray<float> Isoparaval;
	 Vec4 pu,pv,pw;
	 float steplength = 1.f/segmentNum;
     for(k=0; k<=segmentNum;k++)
	 {
		 w = steplength*k;
          for(j=0; j<=segmentNum; j++)
		  {
			  v = steplength*j;
			  for(i=0; i<=segmentNum; i++)
			  {
				  u = steplength*i;
				  Jacs.push_back(GetJacobianValue(u,v,w));
                  Isoparaval.push_back(GetFirstPartialDirivatives(u,v,w,pu,pv,pw));
			  }
		  }
	 }

	 for(i=0; i<Jacs.size(); i++)
	 {
          if(Jacs.at(i) < 0)
			  m_minorJocbianNum++;
		  if(Jacs.at(i) > m_maxJocbian)
			  m_maxJocbian = Jacs.at(i);
		  else if(Jacs.at(i) < m_minJocbian)
			  m_minJocbian = Jacs.at(i);
	 }	 

	 for(i=0; i<Isoparaval.size(); i++)
	 {
		 m_isovalAdd += Isoparaval.at(i);
	 }	

}
void  SplineVolume::GetBoundBox(Vec4& leftbot, Vec4& rightTop,int istatus)
{
	leftbot.x = leftbot.y = leftbot.z = 10e6;
	rightTop.x = rightTop.y = rightTop.z = -10e6;
	 if(istatus == 0)
	 {
		 int ptnum = m_vVolumePts.size();
		 for(int i=0; i<ptnum; i++)
		 {
			 if(m_vVolumePts.at(i).m_pt.x < leftbot.x)
				 leftbot.x = m_vVolumePts.at(i).m_pt.x;
			 if(m_vVolumePts.at(i).m_pt.y < leftbot.y)
				 leftbot.y = m_vVolumePts.at(i).m_pt.y;
			 if(m_vVolumePts.at(i).m_pt.z < leftbot.z)
				 leftbot.z = m_vVolumePts.at(i).m_pt.z;
			 if(m_vVolumePts.at(i).m_pt.x > rightTop.x)
                 rightTop.x = m_vVolumePts.at(i).m_pt.x;
			 if(m_vVolumePts.at(i).m_pt.y > rightTop.y)
				 rightTop.y = m_vVolumePts.at(i).m_pt.y;
			 if(m_vVolumePts.at(i).m_pt.z > rightTop.z)
				 rightTop.z = m_vVolumePts.at(i).m_pt.z;
		 }
	 }
};
void SplineVolume::SaveVolume()
{
	FILE *fp;
	fp = fopen("F:\\meshdata.txt","w");
	fprintf(fp,"三个方向的控制点数目：%d  %d %d",m_uNum,m_vNum, m_wNum);
	if(fp != NULL)
	{
        for(int i=0; i<m_vAllCtrlPts.size();i++)
		{
			fprintf(fp,"%f %f %f %f \n",m_vAllCtrlPts.at(i).m_pt.x,m_vAllCtrlPts.at(i).m_pt.y,m_vAllCtrlPts.at(i).m_pt.z,m_vAllCtrlPts.at(i).m_matval.x);
		}
	}
	/*int ui,vi,wi;
	if(fp != NULL)
	{
		for(int i=0; i<m_vAllCtrlPts.size();i++)
		{
            GetIJKIndex(i,ui,vi,wi,m_uNum,m_vNum,m_wNum);
			if(ui < 8)
			fprintf(fp,"%f %f %f\n",m_vAllCtrlPts.at(i).x,m_vAllCtrlPts.at(i).y,m_vAllCtrlPts.at(i).z);
		}
	}*/
	fclose(fp);
};
void  SplineVolume::GetAllControlPointsVec4(varray<Vec4>& allVecs)
{
	allVecs.clear();
	for(int i=0; i<m_vAllCtrlPts.size(); i++)
	{
		allVecs.push_back(m_vAllCtrlPts.at(i).m_pt);
	}
};
//以下插值函数的应用和基准面的朝向密切相关。
void  SplineVolume::InterpolateInnerControlPtsByBoundarysueface()
{
	//填充体插值的控制点。以左下角为原点开始填充。
	int ucount = m_6boundarySurface.at(W0sfPos).GetUCtrlPtNum();
	int vcount = m_6boundarySurface.at(W0sfPos).GetVCtrlPtNum();
	int wcount = m_6boundarySurface.at(U0sfPos).GetVCtrlPtNum();
	int uDegree = m_6boundarySurface.at(W0sfPos).GetUDegree();
	int vDegree = m_6boundarySurface.at(W0sfPos).GetVDegree();
	int wDegree = m_6boundarySurface.at(U0sfPos).GetVDegree();

	varray<Vec4> allControlPts;
	int i,j,k;
	for(k=0; k<wcount; k++)
	{
		if(k == 0)
		{
			for(i=0; i<vcount;i++)
			{
				for(j=0; j<ucount; j++)
				{
					allControlPts.push_back(m_6boundarySurface.at(W0sfPos).GetCtrlPt(i,j));
				}
			}
		}
		else if(k == wcount -1)
		{
			for(i=0; i<vcount;i++)
			{
				for(j=0; j<ucount; j++)
				{
					allControlPts.push_back(m_6boundarySurface.at(W1sfPos).GetCtrlPt(i,j));
				}
			}
		}
		else
		{
			for(i=0; i<vcount; i++)
			{
				if(i==0)
				{
					for(j=0; j<ucount; j++)
					{
						allControlPts.push_back(m_6boundarySurface.at(V0sfPos).GetCtrlPt(k,j));
					}
				}
				else if(i == vcount-1)
				{
					for(j=0; j<ucount; j++)
					{
						allControlPts.push_back(m_6boundarySurface.at(V1sfPos).GetCtrlPt(k,j));
					}
				}
				else
				{
					for(j=0; j<ucount; j++)
					{
						if(j == 0)
						{
							allControlPts.push_back(m_6boundarySurface.at(U0sfPos).GetCtrlPt(k,i));
						}
						else if(j == ucount -1)
						{
							allControlPts.push_back(m_6boundarySurface.at(U1sfPos).GetCtrlPt(k,i));
						}
						else
						{
							allControlPts.push_back(Vec4(0.f,0.f,0.f));
						}
					}
				}
			}
		}
	}
	SetControlPoints(allControlPts,uDegree,vDegree,wDegree,ucount,vcount,wcount);
}
void  SplineVolume::SetDisplaceAndLoadvector(int ctrlPtid, Vec4& disVec,Vec4& loadVec)
{
	if(ctrlPtid < 0 || ctrlPtid >= m_vAllCtrlPts.size())
		return;
	m_vAllCtrlPts.at(ctrlPtid).m_displacement = disVec;
	m_vAllCtrlPts.at(ctrlPtid).m_stress = loadVec;
};


void SplineVolume::SetVol(const int uDegree, const int vDegree, const int wDegree, const int uNum, const int vNum, const int wNum, const varray<double>& uKnots, const varray<double>& vKnots, const varray<double>& wKnots)
{
	m_uDegree = uDegree;
	m_vDegree = vDegree;
	m_wDegree = wDegree;
	m_uNum = uNum;
	m_vNum = vNum;
	m_wNum = wNum;
	m_uKnots = uKnots;
	m_vKnots = vKnots;
	m_wKnots = wKnots;
}

//(u,v,w)对应的体上的点
Vec3 SplineVolume::GetVolPoint(const double u, const double v, const double w)const
{
	int r = FindSpan(u, m_uDegree, m_uNum, m_uKnots);
	varray<double> uNknot;
	BasisFuns(u, r, m_uDegree, m_uKnots, uNknot);

	int s = FindSpan(v, m_vDegree, m_vNum, m_vKnots);
	varray<double> vNknot;
	BasisFuns(v, s, m_vDegree, m_vKnots, vNknot);

	int t = FindSpan(w, m_wDegree, m_wNum, m_wKnots);
	varray<double> wNknot;
	BasisFuns(w, t, m_wDegree, m_wKnots, wNknot);

	Vec3 val;
	double pw = 0.0;
	int ii, jj, kk;
	ii = jj = kk = 0;
	for (int k = 0; k <= m_wDegree; k++)
	{
		kk = t - m_wDegree + k;
		for (int j = 0; j <= m_uDegree; j++)
		{
			jj = r - m_uDegree + j;
			for (int i = 0; i <= m_vDegree; i++)
			{
				ii = s - m_vDegree + i;
				Vec4 bpti = m_CtrlPts[kk*m_uNum*m_vNum + jj * m_vNum + ii];
				val += uNknot[j] * vNknot[i] * wNknot[k] * bpti*bpti.w;
				pw += uNknot[j] * vNknot[i] * wNknot[k] * bpti.w;
			}
		}
	}
	val /= pw;
	return val;
}

//计算四边形面片显示数据
threadParamSplineVOL SplineVolume::CalQuads(const int Unum, const int Vnum, const int Wnum) const
{
	varray<varray<varray<Vec3>>> quads;
	varray<varray<varray<Vec3>>> lines;
	quads.clear();
	lines.clear();
	int a = 0;
	int num[3]{ Unum ,Vnum ,Wnum };//曲面细分度三个方向的num
	for (int i = 0; i < 6; ++i)
	{
		varray<varray<Vec3>> Q, L;
		CalIsoSurface(a, i % 2, num[(a + 1) % 3], num[(a + 2) % 3], Q, L);
		quads.push_back(Q);
		lines.push_back(L);
		if (i % 2)a += 1;
	}
	threadParamSplineVOL vol = { quads,lines };

	return vol;
}

//根据控制点三维序号计算一维序号
inline	int SplineVolume::CtrlPtsIdx(const int uIdx, const int vIdx, const int wIdx)
{
	return m_uNum * m_vNum*wIdx + m_vNum * uIdx + vIdx;
}


//控制点排序转换为U-V-W
void SplineVolume::OrderCtrlPts()
{
	SplineVolume vol;
	OrderCtrlPts(vol);
	m_CtrlPts = vol.m_CtrlPts;
	SetVol(vol.m_uDegree, vol.m_vDegree, m_wDegree,
		vol.m_uNum, vol.m_vNum, m_wNum,
		vol.m_uKnots, vol.m_vKnots, m_wKnots);
}

//控制点排序V-U-W转换为U-V-W
void SplineVolume::OrderCtrlPts(SplineVolume& vol)
{
	varray<Vec4> temp;
	for (int k = 0; k < m_wNum; ++k)
	{
		for (int i = 0; i < m_vNum; ++i)
		{
			for (int j = 0; j < m_uNum; ++j)
			{
				temp.push_back(m_CtrlPts[CtrlPtsIdx(j, i, k)]);
			}
		}
	}
	std::swap(vol.m_uNum, vol.m_vNum);
	std::swap(vol.m_uDegree, vol.m_vDegree);
	std::swap(vol.m_uKnots, vol.m_vKnots);
	vol.m_CtrlPts = temp;
}
////控制点排序转换为U-V-W
//void SplineVolume::OrderCtrlPts1(SplineVolume& vol,int mode) {
//	//视角：从-y轴往正y轴看
//	// -w-v-u转化为U-V-W 
//	if (mode == 1) {
//		varray<Vec4> temp;
//		for (int k = m_wNum - 1; k >= 0; --k)
//		{
//			for (int i = 0; i < m_vNum; ++i)
//			{
//				for (int j = 0; j < m_uNum; ++j)
//				{
//					temp.push_back(m_CtrlPts[CtrlPtsIdx(i, j, k)]);
//				}
//			}
//		}
//		SplineVolume sv;
//		sv.m_CtrlPts = temp;
//		vol.m_CtrlPts.clear();
//		for (int k = 0; k < m_wNum; ++k)
//		{
//			for (int i = 0; i < m_vNum; ++i)
//			{
//				for (int j = 0; j < m_uNum; ++j)
//				{
//					vol.m_CtrlPts.push_back(sv.m_CtrlPts[CtrlPtsIdx(i, k, j)]);
//				}
//			}
//		}
//	}
//	//w-v--u转换为u-v-w
//	if (mode == 2) {
//		varray<Vec4> temp;
//		for (int k = 0; k < m_wNum; ++k)
//		{
//			for (int i = 0; i < m_vNum; ++i)
//			{
//				for (int j = 0; j < m_uNum; ++j)
//				{
//					temp.push_back(m_CtrlPts[CtrlPtsIdx(k, i, j)]);
//				}
//			}
//		}
//		vol.m_CtrlPts = temp;
//	}
//	//u--v-w转换为u-v-w
//	if (mode == 3) {
//		varray<Vec4> temp;
//		for (int k = 0; k <m_wNum; ++k)
//		{
//			for (int i = m_vNum - 1; i >= 0; --i)
//			{
//				for (int j = 0; j < m_uNum; ++j)
//				{
//					temp.push_back(m_CtrlPts[CtrlPtsIdx(i, j ,k )]);
//				}
//			}
//		}
//		vol.m_CtrlPts = temp;
//	}
//	//w--v-u转换为u-v-w
//	if (mode == 4) {
//		varray<Vec4> temp;
//		for (int k = 0; k < m_wNum; ++k)
//		{
//			for (int i = m_vNum - 1; i >= 0; --i)
//			{
//				for (int j = 0; j < m_uNum; ++j)
//				{
//					temp.push_back(m_CtrlPts[CtrlPtsIdx(i,j ,k )]);
//				}
//			}
//		}
//		vol.m_CtrlPts.clear();
//		for (int k = 0; k < m_wNum; ++k)
//		{
//			for (int i= 0; i < m_vNum; ++i)
//			{
//				for (int j = 0; j < m_uNum; ++j)
//				{
//					vol.m_CtrlPts.push_back(temp[CtrlPtsIdx(i, k, j)]);
//				}
//			}
//		}
//		
//	}
//	// -w--v-u转化为U-V-W 
//	if (mode == 5) {
//		varray<Vec4> temp,temp1;
//		for (int k = m_wNum - 1; k >= 0; --k)
//		{
//			for (int i = 0; i < m_vNum; ++i)
//			{
//				for (int j = 0; j < m_uNum; ++j)
//				{
//					temp.push_back(m_CtrlPts[CtrlPtsIdx(i, j, k)]);
//				}
//			}
//		}
//		
//		for (int k = 0; k < m_wNum; ++k)
//		{
//			for (int i = m_vNum - 1; i >= 0; --i)
//			{
//				for (int j = 0; j < m_uNum; ++j)
//				{
//					temp1.push_back(temp[CtrlPtsIdx(i, j, k)]);
//				}
//			}
//		}
//		vol.m_CtrlPts.clear();
//		for (int k = 0; k < m_wNum; ++k)
//		{
//			for (int i = 0; i < m_vNum; ++i)
//			{
//				for (int j = 0; j < m_uNum; ++j)
//				{
//					vol.m_CtrlPts.push_back(temp1[CtrlPtsIdx(i, k, j)]);
//				}
//			}
//		}
//	}
//}

void SplineVolume::KnotsRefine(varray<double>& Uknot, varray<double>& Vknot, varray<double>& Wknot)
{

	int u = Uknot.size(), v = Vknot.size(), w = Wknot.size();
	double x = Uknot[0];
	if (u > 0 && v > 0)
	{
		varray<varray<Vec4>> NewCpts;
		SplineSurface Suv;
		for (int k = 0; k < m_wNum; k++)
		{
			Suv.SetSurface(m_uDegree, m_vDegree, m_uNum, m_vNum, m_uKnots, m_vKnots);
			for (int j = 0; j < m_uNum; j++)
			{
				for (int i = 0; i < m_vNum; i++)
				{
					Suv.m_CtrlPts.push_back(m_CtrlPts[i + j * m_vNum + k * m_uNum*m_vNum]);
				}
			}

			Suv.KnotsRefine(Uknot, Vknot);
			NewCpts.push_back(Suv.m_CtrlPts);
			Suv.m_CtrlPts.clear();
		}
		m_CtrlPts.clear();
		m_uKnots = Suv.m_uKnots;
		m_vKnots = Suv.m_vKnots;
		m_vNum = Suv.m_vNum;
		m_uNum = Suv.m_uNum;
		for (int i = 0; i < NewCpts.size(); i++)
			for (int j = 0; j < NewCpts[i].size(); j++)
				m_CtrlPts.push_back(NewCpts[i][j]);
	}

	if (w > 0 && v > 0)
	{

		varray<varray<Vec4>> NewCpts;
		SplineSurface Suv;
		for (int k = 0; k < m_uNum; k++)
		{
			Suv.SetSurface(m_wDegree, m_vDegree, m_wNum, m_vNum, m_wKnots, m_vKnots);
			for (int j = 0; j < m_wNum; j++)
			{
				for (int i = 0; i < m_vNum; i++)
				{
					Suv.m_CtrlPts.push_back(m_CtrlPts[i + j * m_vNum * m_uNum + k * m_vNum]);
				}
			}
			Vknot.clear();
			Suv.KnotsRefine(Wknot, Vknot);
			NewCpts.push_back(Suv.m_CtrlPts);
			Suv.m_CtrlPts.clear();
		}
		m_CtrlPts.clear();
		m_wKnots = Suv.m_uKnots;
		m_wNum = Suv.m_uNum;

		for (int k = 0; k < m_uNum; k++) {
			for (int j = 0; j < NewCpts.size(); j++) {
				for (int i = 0; i < m_vNum; i++)
				{
					m_CtrlPts.push_back(NewCpts[j][i + k * m_vNum]);
				}
			}
		}

	}
}

//升阶
//Udegree,Vdegree,Wdegree:升阶后次数
void SplineVolume::DegreeElevate(const int Udegree, const int Vdegree, const int Wdegree)
{
	int tu = Udegree - m_uDegree;
	int tv = Vdegree - m_vDegree;
	int tw = Wdegree - m_wDegree;

	//U方向
	if (tu > 0)
	{
		varray<varray<varray<Vec4>>> NewConPoint;
		Spline line;
		for (int k = 0; k < m_wNum; ++k)
		{
			varray<varray<Vec4>> NewConPointSF;
			for (int j = 0; j < m_vNum; ++j)
			{
				line.m_Degree = m_uDegree;
				line.m_Knots = m_uKnots;
				line.m_CtrlPts.clear();
				for (int i = 0; i < m_uNum; ++i)
					line.m_CtrlPts.push_back(m_CtrlPts[CtrlPtsIdx(i, j, k)]);
				line.DegreeElevate(Udegree);
				NewConPointSF.push_back(line.m_CtrlPts);
			}
			NewConPoint.push_back(NewConPointSF);//[w][v][u]
		}
		m_uDegree = Udegree;
		m_uKnots = line.m_Knots;
		m_uNum = line.m_CtrlPts.size();
		m_CtrlPts.resize(m_uNum*m_vNum*m_wNum);
		for (int i = 0; i < NewConPoint.size(); ++i)
		{
			for (int j = 0; j < NewConPoint[i].size(); ++j)
			{
				for (int k = 0; k < NewConPoint[i][j].size(); ++k)
				{
					int idx = CtrlPtsIdx(k, j, i);
					m_CtrlPts[idx] = NewConPoint[i][j][k];
				}
			}
		}
	}
	//V方向
	if (tv > 0)
	{
		varray<varray<varray<Vec4>>> NewConPoint;
		Spline line;
		for (int k = 0; k < m_wNum; ++k)
		{
			varray<varray<Vec4>> NewConPointSF;
			for (int i = 0; i < m_uNum; ++i)
			{
				line.m_Degree = m_vDegree;
				line.m_Knots = m_vKnots;
				line.m_CtrlPts.clear();
				for (int j = 0; j < m_vNum; ++j)
					line.m_CtrlPts.push_back(m_CtrlPts[CtrlPtsIdx(i, j, k)]);
				line.DegreeElevate(Vdegree);
				NewConPointSF.push_back(line.m_CtrlPts);
			}
			NewConPoint.push_back(NewConPointSF);//[w][u][v]
		}
		m_vDegree = Vdegree;
		m_vKnots = line.m_Knots;
		m_vNum = line.m_CtrlPts.size();
		m_CtrlPts.resize(m_uNum*m_vNum*m_wNum);
		for (int i = 0; i < NewConPoint.size(); ++i)
		{
			for (int j = 0; j < NewConPoint[i].size(); ++j)
			{
				for (int k = 0; k < NewConPoint[i][j].size(); ++k)
				{
					int idx = CtrlPtsIdx(j, k, i);
					m_CtrlPts[idx] = NewConPoint[i][j][k];
				}
			}
		}
	}
	//W方向
	if (tw > 0)
	{
		varray<varray<varray<Vec4>>> NewConPoint;
		Spline line;
		for (int i = 0; i < m_uNum; ++i)
		{
			varray<varray<Vec4>> NewConPointSF;
			for (int j = 0; j < m_vNum; ++j)
			{
				line.m_Degree = m_wDegree;
				line.m_Knots = m_wKnots;
				line.m_CtrlPts.clear();
				for (int k = 0; k < m_wNum; ++k)
					line.m_CtrlPts.push_back(m_CtrlPts[CtrlPtsIdx(i, j, k)]);
				line.DegreeElevate(Wdegree);
				NewConPointSF.push_back(line.m_CtrlPts);
			}
			NewConPoint.push_back(NewConPointSF);//[u][v][w]
		}
		m_wDegree = Wdegree;
		m_wKnots = line.m_Knots;
		m_wNum = line.m_CtrlPts.size();
		m_CtrlPts.resize(m_uNum*m_vNum*m_wNum);
		for (int i = 0; i < NewConPoint.size(); ++i)
		{
			for (int j = 0; j < NewConPoint[i].size(); ++j)
			{
				for (int k = 0; k < NewConPoint[i][j].size(); ++k)
				{
					int idx = CtrlPtsIdx(i, j, k);
					m_CtrlPts[idx] = NewConPoint[i][j][k];
				}
			}
		}
	}
}

/*扫描生成Nurbs体模型
pathT：扫描路径
nurbsSF：起始截面
K：截面实例数量,一般取路径控制点数量减1
*/
void SplineVolume::CreateSweepSplineVolume(const Spline& pathT, const SplineSurface & nurbsSF, const int K)
{
	//u方向节点矢量，控制点数量
	m_uDegree = nurbsSF.m_uDegree;
	m_uNum = nurbsSF.m_uNum;
	m_uKnots = nurbsSF.m_uKnots;

	//v方向节点矢量，控制点数量
	m_vDegree = nurbsSF.m_vDegree;
	m_vNum = nurbsSF.m_vNum;
	m_vKnots = nurbsSF.m_vKnots;

	m_wDegree = pathT.m_Degree;
	m_wKnots.clear();
	m_CtrlPts.clear();

	varray<double> pos;
	InsLocation(pathT, K, pos);//同时完成W方向节点矢量
	varray<varray<Vec4>> TranMat;
	LocalCoordinates(pathT, pos, TranMat);
	varray<varray<Vec4>> nurbsSFctrlPts;
	varray<varray<varray<double>>> SFw;
	varray<varray<double>> sws;
	for (int i = 0; i < nurbsSF.m_uNum; ++i)
	{
		varray<Vec4> vctrl;
		varray<double> sw;
		for (int j = 0; j < nurbsSF.m_vNum; ++j)
		{
			vctrl.push_back(nurbsSF.m_CtrlPts[i*nurbsSF.m_vNum + j]);
			sw.push_back(nurbsSF.m_CtrlPts[i*nurbsSF.m_vNum + j].w);
		}
		nurbsSFctrlPts.push_back(vctrl);
		sws.push_back(sw);
	}
	varray<varray<varray<Vec4>>> allNurbsSF;
	MatrixTran(nurbsSFctrlPts, TranMat, allNurbsSF);
	m_wNum = allNurbsSF.size();
	for (int i = 0; i < m_wNum; ++i)
	{
		SFw.push_back(sws);
	}
	SweepSurface(allNurbsSF, SFw, pos);
}

/*平移扫描生成Nurbs体模型
pathT：扫描路径
nurbsSF：起始截面*/
void SplineVolume::CreateTransSweepSplineVolume(const Spline& pathT, const SplineSurface & nurbsSF)
{
	//u方向节点矢量，控制点数量
	m_uDegree = nurbsSF.m_uDegree;
	m_uNum = nurbsSF.m_uNum;
	m_uKnots = nurbsSF.m_uKnots;

	//v方向节点矢量，控制点数量
	m_vDegree = nurbsSF.m_vDegree;
	m_vNum = nurbsSF.m_vNum;
	m_vKnots = nurbsSF.m_vKnots;

	m_wDegree = pathT.m_Degree;
	m_wNum = pathT.m_CtrlPts.size();
	m_wKnots = pathT.m_Knots;

	m_CtrlPts.clear();
	for (int i = 0; i < pathT.m_CtrlPts.size(); i++)
	{
		for (int j = 0; j < nurbsSF.m_CtrlPts.size(); j++)
		{
			Vec4 ControlPoint;
			ControlPoint = nurbsSF.m_CtrlPts[j] + pathT.m_CtrlPts[i] - pathT.m_CtrlPts[0];
			ControlPoint.w = nurbsSF.m_CtrlPts[j].w*pathT.m_CtrlPts[i].w;
			m_CtrlPts.push_back(ControlPoint);
		}
	}
}

//放样
//path:路径
//surfaces:放样截面
void SplineVolume::LoftingSplineVolume(const Spline& path, const varray<SplineSurface>& surfaces)
{
	m_CtrlPts.clear();
	m_uKnots.clear();
	m_vKnots.clear();
	m_wKnots.clear();

	m_wDegree = path.m_Degree;
	m_wNum = path.m_CtrlPts.size();

	//最高次
	int maxUDegree, maxVDegree;
	MaxDegree(surfaces, maxUDegree, maxVDegree);
	//升阶
	varray<SplineSurface> UnitSurfaces = surfaces;
	for (int i = 0; i < UnitSurfaces.size(); ++i)
		UnitSurfaces[i].DegreeElevate(maxUDegree, maxVDegree);
	//统一节点矢量
	varray<double> UKnots, VKnots;
	KnotsUnify(UnitSurfaces, UKnots, VKnots);
	for (int i = 0; i < UnitSurfaces.size(); ++i)
	{
		varray<double> diffUKnots, diffVKnots;
		KnotsDiff(UKnots, UnitSurfaces[i].m_uKnots, diffUKnots);
		KnotsDiff(VKnots, UnitSurfaces[i].m_vKnots, diffVKnots);
		//节点插入
		UnitSurfaces[i].KnotsRefine(diffUKnots, diffVKnots);
	}
	m_uKnots = UnitSurfaces[0].m_uKnots;
	m_vKnots = UnitSurfaces[0].m_vKnots;
	m_uNum = UnitSurfaces[0].m_uNum;
	m_vNum = UnitSurfaces[0].m_vNum;
	m_uDegree = UnitSurfaces[0].m_uDegree;
	m_vDegree = UnitSurfaces[0].m_vDegree;

	//varray<double> DIS, UK, U_W; double Dis = 0;
	//for (int i = 0; i<path.m_CtrlPts.size() - 1; i++)
	//{
	//	Vec3 pt0, pt1; double d;
	//	pt0 = path.m_CtrlPts[i];
	//	pt1 = path.m_CtrlPts[i + 1];
	//	d = sqrt(pow((pt1.x - pt0.x), 2) + pow((pt1.y - pt0.y), 2) + pow((pt1.z - pt0.z), 2));
	//	DIS.push_back(d);
	//	Dis += d;
	//}
	//UK.push_back(0);
	//for (int i = 0; i<DIS.size() - 1; i++)
	//{
	//	UK.push_back(UK[UK.size() - 1] + DIS[i] / Dis);
	//}
	//UK.push_back(1);
	////节点向量
	//for (int i = 0; i < m_wDegree + 1; i++)
	//{
	//	U_W.push_back(0);
	//}
	//for (int j = 1; j<path.m_CtrlPts.size() - m_wDegree; j++)
	//{
	//	double uw = 0;
	//	for (int i = j; i <= j + m_wDegree - 1; i++)
	//	{
	//		uw += UK[i];
	//	}
	//	U_W.push_back(uw / (j + m_wDegree - 1));
	//}
	//for (int i = 0; i<m_wDegree + 1; i++)
	//{
	//	U_W.push_back(1);
	//}
	//for (int i = 0; i<U_W.size(); i++)
	//	m_wKnots.push_back(U_W[i]);
	//for (int i = 0; i<surfaces.size(); i++)
	//	for (int j = 0; j < surfaces[i].m_CtrlPts.size(); j++)
	//		m_CtrlPts.push_back(surfaces[i].m_CtrlPts[j]);

	int K = (surfaces.size() - 1) > (path.m_CtrlPts.size() - 1) ? (surfaces.size() - 1) : (path.m_CtrlPts.size() - 1);
	varray<double> pos;
	InsLocation(path, K, pos);//同时完成W方向节点矢量
	varray<varray<Vec4>> TranMat;
	LocalCoordinates(path, pos, TranMat);

	varray<varray<Vec4>> nurbsSFctrlPts;
	for (int i = 0; i < UnitSurfaces[0].m_uNum; ++i)
	{
		varray<Vec4> vctrl;
		for (int j = 0; j < UnitSurfaces[0].m_vNum; ++j)
			vctrl.push_back(UnitSurfaces[0].m_CtrlPts[i*UnitSurfaces[0].m_vNum + j]);
		nurbsSFctrlPts.push_back(vctrl);
	}
	varray<varray<varray<Vec4>>> allNurbsSF;
	MatrixTran(nurbsSFctrlPts, TranMat, allNurbsSF);
	m_wNum = allNurbsSF.size();
}

void SplineVolume::LoftingSplineVolume(const Spline & path, const SplineSurface & surfaces0, const SplineSurface & surfaces1)
{
	m_CtrlPts.clear();
	m_uKnots.clear();
	m_vKnots.clear();
	m_wKnots.clear();

	m_wDegree = path.m_Degree;
	m_wNum = path.m_CtrlPts.size();

	varray<SplineSurface> surfaces;
	surfaces.push_back(surfaces0);
	surfaces.push_back(surfaces1);
	//最高次
	MaxDegree(surfaces, m_uDegree, m_vDegree);
	//升阶
	for (int i = 0; i < surfaces.size(); ++i)
		surfaces[i].DegreeElevate(m_uDegree, m_vDegree);
	//统一节点矢量
	KnotsUnify(surfaces, m_uKnots, m_vKnots);
	for (int i = 0; i < surfaces.size(); ++i)
	{
		varray<double> diffUKnots, diffVKnots;
		KnotsDiff(m_uKnots, surfaces[i].m_uKnots, diffUKnots);
		KnotsDiff(m_vKnots, surfaces[i].m_vKnots, diffVKnots);
		//节点插入
		surfaces[i].KnotsRefine(diffUKnots, diffVKnots);
	}
	m_uNum = surfaces[0].m_uNum;
	m_vNum = surfaces[0].m_vNum;

	int K = path.m_CtrlPts.size() - 1;
	varray<double> pos;
	InsLocation(path, K, pos);//同时完成W方向节点矢量
	varray<varray<Vec4>> TranMat01, TranMat10;
	LocalCoordinates(path, pos, TranMat01);

	//反向
	/*Spline path10 = path;
	path10.m_CtrlPts.reverse();
	for (int i = pos01.size() - 1; i >= 0; --i)
		pos10.push_back(1 - pos01[i]);
	LocalCoordinates(path10, pos10, TranMat10);*/
	TranMat10 = TranMat01;
	TranMat10.reverse();
	for (int i = 0; i < TranMat10.size(); ++i)
	{
		TranMat10[i][2] *= -1;
	}
	//原始权重
	varray<varray<double>> SFw0, SFw1;
	//控制点二维数组
	varray<varray<Vec4>> nurbsSFctrlPts0, nurbsSFctrlPts1;

	for (int i = 0; i < surfaces[0].m_uNum; ++i)
	{
		varray<Vec4> vctrl;
		varray<double> sw;
		for (int j = 0; j < surfaces[0].m_vNum; ++j)
		{
			vctrl.push_back(surfaces[0].m_CtrlPts[i*surfaces[0].m_vNum + j]);
			sw.push_back(surfaces[0].m_CtrlPts[i*surfaces[0].m_vNum + j].w);
		}
		nurbsSFctrlPts0.push_back(vctrl);
		SFw0.push_back(sw);
	}

	
	for (int i = 0; i < surfaces[1].m_uNum; ++i)
	{
		varray<Vec4> vctrl;
		varray<double> sw;
		for (int j = 0; j < surfaces[1].m_vNum; ++j)
		{
			vctrl.push_back(surfaces[1].m_CtrlPts[i*surfaces[1].m_vNum + j]);
			sw.push_back(surfaces[0].m_CtrlPts[i*surfaces[0].m_vNum + j].w);
		}
		nurbsSFctrlPts1.push_back(vctrl);
		SFw1.push_back(sw);
	}
	//实例截面计算
	varray<varray<varray<Vec4>>> allNurbsSF0, allNurbsSF1, allNurbsSF;
	varray<varray<varray<double>>>  SFw;
	MatrixTran(nurbsSFctrlPts0, TranMat01, allNurbsSF0);
	MatrixTran(nurbsSFctrlPts1, TranMat10, allNurbsSF1);
	
	//同一位置插值线性插值
	allNurbsSF.resize(pos.size());
	SFw.resize(pos.size());
	for (int i = 0; i < pos.size(); ++i)
	{
		allNurbsSF[i].resize(allNurbsSF0[i].size());
		SFw[i].resize(SFw0.size());
		for (int j = 0; j < allNurbsSF0[i].size(); ++j)
		{
			allNurbsSF[i][j].resize(allNurbsSF0[i][j].size());
			SFw[i][j].resize(SFw0[j].size());
			int sf0ij = allNurbsSF0[i][j].size();
			for (int k = 0; k < sf0ij; ++k)
			{
				Vec4 dp = allNurbsSF1[pos.size() - 1 - i][j][sf0ij - 1 - k] - allNurbsSF0[i][j][k];
				double dw = abs(allNurbsSF1[pos.size() - 1 - i][j][sf0ij - 1 - k].w - allNurbsSF0[i][j][k].w);
				Vec4 pts = allNurbsSF0[i][j][k]
					+ pos[i] * dp.Magnitude()*dp.Normalize();
				allNurbsSF[i][j][k] = pts;
				allNurbsSF[i][j][k].w = allNurbsSF0[i][j][k].w + pos[i] * dw;
				SFw[i][j][k] = SFw0[j][k] + pos[i] * abs(SFw1[j][k] - SFw0[j][k]);
			}
		}
	}
	m_wNum = allNurbsSF.size();
	SweepSurface(allNurbsSF, SFw, pos);
	
}





//计算等参面
//uvw:0=u,1=v,2=w
//t:参数
//num:以u-v-w顺序
//L:等参面四边形面片集
void SplineVolume::CalIsoSurface(const int uvw, const double t, const int num1, const int num2,
	varray<varray<Vec3>>& quads, varray<varray<Vec3>>& lines)const
{
	quads.clear();
	lines.clear();
	varray<varray<Vec3>> L;
	double u[3]{ 0,0,0 };
	u[uvw] = t;
	int a = (uvw + 1) % 3;
	int b = (uvw + 2) % 3;

	double da = 1.0 / num1;
	double db = 1.0 / num2;
	for (int i = 0; i <= num1; i++)
	{
		u[a] = da * i;
		varray<Vec3> line;
		for (int j = 0; j <= num2; j++)
		{
			u[b] = db * j;
			Vec3 t = GetVolPoint(u[0], u[1], u[2]);
			line.push_back(t);
		}
		L.push_back(line);
	}

	for (int i = 0; i < L.size() - 1; ++i)
	{
		for (int j = 0; j < L[i].size() - 1; ++j)
		{
			varray<Vec3> quad;
			quad.push_back(L[i][j]);
			quad.push_back(L[i + 1][j]);
			quad.push_back(L[i + 1][j + 1]);
			quad.push_back(L[i][j + 1]);
			quads.push_back(quad);
		}
	}

	lines.push_back(L[0]);
	lines.push_back(L[L.size() - 1]);
	varray<Vec3> l0, l1;
	for (int i = 0; i < L.size(); ++i)
	{
		l0.push_back(L[i][0]);
		l1.push_back(L[i][L[i].size() - 1]);
	}
	lines.push_back(l0);
	lines.push_back(l1);
}

/*扫描时的截面实例位置
pathT：扫描路径
K：截面实例数量
pos：截面实例位置
NewKnots：新节点矢量*/
void SplineVolume::InsLocation(const Spline & pathT, int K, varray<double>& pos)
{
	pos.clear();
	int q = pathT.m_Degree;
	int ktv = pathT.m_Knots.size();
	int nsect = K + 1;
	double vsum;

	m_wKnots = pathT.m_Knots;

	if (ktv <= nsect + q)  //细化节点矢量
	{
		int m = nsect + q - ktv + 1;
		for (int i = 0; i < m; ++i)
		{
			//最长节点区间
			varray<double>::iterator idx = m_wKnots.begin();
			double maxlen = 0;
			for (auto it = m_wKnots.begin() + pathT.m_Degree;
				it != m_wKnots.begin() + (m_wKnots.size() - pathT.m_Degree - 2); ++it)
			{
				double dk = *(it + 1) - *it;
				if (dk >= maxlen)
				{
					maxlen = dk;
					idx = it;
				}
			}
			//插入中点
			double mid = (*idx + *(idx + 1)) / 2;
			m_wKnots.insert(idx + 1, mid);
		}
	}
	else if (ktv > nsect + q + 1) //增加实例
		nsect = ktv - q - 1;

	//实例位置
	pos.push_back(m_wKnots[0]);
	for (int k = 1; k < nsect - 1; k++)
	{
		vsum = 0;
		for (int l = k + 1; l < k + q + 1; l++)
		{
			vsum += m_wKnots[l];
		}
		pos.push_back(vsum / q);
	}
	pos.push_back(m_wKnots[m_wKnots.size() - 1]);
}

/*计算局部坐标系
pathT：扫描路径
pos：截面实例位置
TranMat：pos处的局部坐标系*/
void SplineVolume::LocalCoordinates(const Spline & pathT, const varray<double> & pos,
	varray<varray<Vec4>> & TranMat)
{
	TranMat.clear();
	int q = pathT.m_Degree;
	//计算弗朗内特标
	//导矢
	varray<varray<Vec4>> DersBasis;
	DersBasis.clear();
	for (int i = 0; i < pos.size(); i++)
	{
		varray<Vec4> NuDerBasi;
		pathT.PtsDerivs(pos[i], 2, NuDerBasi);
		if (NuDerBasi.size() < 3 || NuDerBasi[2] == Vec4())//不存在2阶导
		{
			Vec4 vDer = NuDerBasi[1].Normalize().Abs();
			int midx = 0;
			for (int j = 1; j < 3; ++j)
			{
				if (vDer[j] > vDer[midx])
				{
					vDer[midx] = 0;
					midx = j;
				}
				else
					vDer[j] = 0;
			}
			if (vDer.Angle(NuDerBasi[1]) == 0 || vDer.Angle(NuDerBasi[1]) == 180)
			{
				vDer[midx] = 0;
				vDer[(midx + 1) % 3] = 1;
			}
			Vec4 vt = NuDerBasi[1].CrossVecX(vDer).Normalize();
			if (NuDerBasi.size() < 3)
				NuDerBasi.push_back(vt);
			else
				NuDerBasi[2] = vt;
		}
		DersBasis.push_back(NuDerBasi);
	}
	for (int i = 0; i < pos.size(); i++)
	{
		Vec4 BV = (DersBasis[i][1].CrossVecX(DersBasis[i][2])).Normalize();
		Vec4 NV = (BV.CrossVecX(DersBasis[i][1])).Normalize();
		DersBasis[i][1] = DersBasis[i][1].Normalize();
		varray<Vec4> Stor;
		Stor.push_back(BV);
		Stor.push_back(NV);
		Stor.push_back(DersBasis[i][1]);
		Stor.push_back(DersBasis[i][0]);
		TranMat.push_back(Stor);
	}
}

/*沿扫描路径对截面进行变换
nurbsSF：截面控制点
TranMat：变换矩阵
allNurbsSF：得到的所有截面实例控制点*/
void SplineVolume::MatrixTran(const varray<varray<Vec4>> & nurbsSF, const varray<varray<Vec4>>& TranMat,
	varray<varray<varray<Vec4>>>& allNurbsSF)
{
	using Eigen::Matrix4d;
	using Eigen::Vector4d;

	allNurbsSF.clear();
	varray<Vec3> OriCoordinates;
	for (int i = 0; i < TranMat[0].size(); ++i)
		OriCoordinates.push_back(TranMat[0][i]);

	for (int i = 0; i < TranMat.size(); i++)  //变换矩阵
	{
		Matrix4d mat;
		varray<Vec3> newCoordinates;
		newCoordinates.push_back(TranMat[i][0]);
		newCoordinates.push_back(TranMat[i][1]);
		newCoordinates.push_back(TranMat[i][2]);
		newCoordinates.push_back(TranMat[i][3]);

		//CalCoordinatesTransMat(OriCoordinates, newCoordinates, mat);

		varray<varray<Vec4>> SectPoints;
		for (int j = 0; j < nurbsSF.size(); j++)
		{
			varray<Vec4> SectPoint;
			for (int k = 0; k < nurbsSF[j].size(); k++)
			{

				//Vector4d bas = P4dToV4d(nurbsSF[j][k]);
				//bas[3] = 1;
				//Vector4d torm = mat * bas;
				//Vec4 pt = V4dToP4d(torm) * TranMat[i][3].w;
			//	pt.w = TranMat[i][3].w;
				//SectPoint.push_back(pt);
			}
			SectPoints.push_back(SectPoint);
		}

		allNurbsSF.push_back(SectPoints);
	}
}

//扫描生成Nurbs体
void SplineVolume::SweepSurface(const varray<varray<varray<Vec4>>>& allNurbsSF,
	const varray<varray<varray<double>>>& SFw, const varray<double> &pos)
{
	using Eigen::MatrixXd;

	//解线性方程组，插值控制点坐标    
	int p = m_wDegree;
	MatrixXd Nbase(allNurbsSF.size(), allNurbsSF.size());
	MatrixXd NpointX(allNurbsSF.size(), m_uNum*m_vNum);
	MatrixXd NpointY(allNurbsSF.size(), m_uNum*m_vNum);
	MatrixXd NpointZ(allNurbsSF.size(), m_uNum*m_vNum);
	MatrixXd NpointW(allNurbsSF.size(), m_uNum*m_vNum);
	MatrixXd ConTropointX(allNurbsSF.size(), m_uNum*m_vNum);
	MatrixXd ConTropointY(allNurbsSF.size(), m_uNum*m_vNum);
	MatrixXd ConTropointZ(allNurbsSF.size(), m_uNum*m_vNum);
	MatrixXd ConTropointW(allNurbsSF.size(), m_uNum*m_vNum);
	for (int i = 0; i < pos.size(); i++)  //截面个数
	{
		//计算基函数
		varray<varray<double>> ndu;
		int span = FindSpan(pos[i], p, allNurbsSF.size(), m_wKnots);
		AllBasisFuns(pos[i], span, p, m_wKnots, ndu);
		for (int l = 0; l < span - p; l++)
		{
			Nbase(i, l) = 0;
		}
		for (int m = 0; m <= p; m++)
		{
			double npm = ndu[m][p];
			Nbase(i, span - p + m) = npm;
		}
		for (int n = span + 1; n < allNurbsSF.size(); n++)
		{
			Nbase(i, n) = 0;
		}
	}

	for (int i = 0; i < allNurbsSF.size(); i++) //第i个截面    
	{
		for (int j = 0; j < allNurbsSF[i].size(); j++)         //UV方向
		{
			for (int k = 0; k < allNurbsSF[i][j].size(); k++)
			{
				NpointX(i, j*allNurbsSF[i][j].size() + k) = allNurbsSF[i][j][k].x;          //体上的点
				NpointY(i, j*allNurbsSF[i][j].size() + k) = allNurbsSF[i][j][k].y;          //体上的点
				NpointZ(i, j*allNurbsSF[i][j].size() + k) = allNurbsSF[i][j][k].z;          //体上的点
				NpointW(i, j*allNurbsSF[i][j].size() + k) = allNurbsSF[i][j][k].w;          //体上的点
			}
		}
	}
	MatrixXd nbinv = Nbase.inverse();
	ConTropointX = Nbase.inverse()*NpointX;
	ConTropointY = Nbase.inverse()*NpointY;
	ConTropointZ = Nbase.inverse()*NpointZ;
	ConTropointW = Nbase.inverse()*NpointW;

	for (int i = 0; i < allNurbsSF.size(); i++) //第i个截面,w方向    
	{
		for (int j = 0; j < allNurbsSF[i].size(); j++) //u方向
		{
			for (int k = 0; k < allNurbsSF[i][j].size(); k++) //v方向
			{
				Vec4 control;
				control.w = ConTropointW(i, j*allNurbsSF[i][j].size() + k);
				control.x = ConTropointX(i, j*allNurbsSF[i][j].size() + k) / control.w;
				control.y = ConTropointY(i, j*allNurbsSF[i][j].size() + k) / control.w;
				control.z = ConTropointZ(i, j*allNurbsSF[i][j].size() + k) / control.w;
				control.w *= SFw[i][j][k];
				m_CtrlPts.push_back(control);
			}
		}
	}
}

//取最高次
void SplineVolume::MaxDegree(const varray<SplineSurface>& surfaces, int& uDegree, int& vDegree)
{
	uDegree = 0;
	vDegree = 0;
	for (int i = 0; i < surfaces.size(); ++i)
	{
		uDegree = surfaces[i].m_uDegree > uDegree ? surfaces[i].m_uDegree : uDegree;
		vDegree = surfaces[i].m_vDegree > vDegree ? surfaces[i].m_vDegree : vDegree;
	}
}

//节点矢量并集
void SplineVolume::KnotsUnify(const varray<SplineSurface>& surfaces, varray<double>& NewUKnots, varray<double>& NewVKnots)
{
	NewUKnots.clear();
	NewVKnots.clear();
	varray<double> temUKnots, temVKnots;
	for (int i = 0; i < surfaces.size(); ++i)
	{
		KnotUnify(surfaces[i].m_uKnots, temUKnots, NewUKnots);
		temUKnots = NewUKnots;
		KnotUnify(surfaces[i].m_vKnots, temVKnots, NewVKnots);
		temVKnots = NewVKnots;
	}
}  


}