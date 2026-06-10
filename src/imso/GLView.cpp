/********************************************************************
* FILE NAME		:GLView.cpp
* AUTHOR		:
* DATE			:
* MODIFIER		:
* MODIFY DATE	:
* DESCRIPTION	:GLView.cpp : インプリメンテーション ファイル
* version		:1.0
********************************************************************/

#include "stdafx.h"
//#include "resource.h"

#include "../includes/view/GLView.h"
#include "..\includes\base.h"
#include "../includes/base/IniFile.h"
#include "../includes/base/XFunction.h"


#ifdef _DEBUG
#define new DEBUG_NEW
#undef THIS_FILE
static char THIS_FILE[] = __FILE__;
#endif

//#define CAMERADIST 500

/////////////////////////////////////////////////////////////////////////////
// CGLView
using namespace Math;
IMPLEMENT_DYNCREATE(CGLView, CView)

CGLView::CGLView()
:	m_bscenceOperSet(false)
{
	m_bActive			  = FALSE;
	m_scenceOper		  = -1;

	m_eye			      = Vec3(0.,0.,0.);

	m_pWndStatusBar       = NULL;
	m_nMouseOp            = -1;

	m_bLButtonDown = false;
    m_bMButtonDown = false;
	m_bRButtonDown = false;	

	m_render			= 0;
	m_renderBefBmpDrag  = 0;

	SelectDxfFlag		  = false;
	m_bIsShowDrawingLines = true;//XXX 2003-01-23
	m_bIsFileOpen		  = false;

	m_rot_Axis			  = 4;
	//m_rgbModelColor	  = RGB(cfg.GetNum(CFGID_MODELCLRR),cfg.GetNum(CFGID_MODELCLRG),cfg.GetNum(CFGID_MODELCLRB));
	m_rgbModelColor       = RGB(216,216,127);//(217,217,178);
	//m_rgbGridColor		  = RGB(255,0,0);
	m_rgbGridColor		  = RGB(216,216,127);
	m_TranslatedVector	  = Vec3(0,0,0);


}

CGLView::~CGLView()
{
}


BEGIN_MESSAGE_MAP(CGLView, CView)
	//{{AFX_MSG_MAP(CGLView)
	ON_WM_LBUTTONDOWN()
	ON_WM_LBUTTONUP()
	ON_WM_RBUTTONDOWN()
	ON_WM_MOUSEMOVE()
	ON_WM_MBUTTONDBLCLK()
	ON_WM_MOUSEWHEEL()
	ON_WM_RBUTTONUP()
	ON_WM_ERASEBKGND()
	ON_WM_DESTROY()
	ON_WM_SIZE()
	ON_WM_SETFOCUS()
	ON_WM_KILLFOCUS()
	ON_WM_KEYUP()
	ON_WM_KEYDOWN()
	ON_WM_NCLBUTTONUP()
	ON_WM_NCMBUTTONUP()
	ON_WM_NCRBUTTONUP()
	//}}AFX_MSG_MAP
END_MESSAGE_MAP()

/////////////////////////////////////////////////////////////////////////////
// CGLView 描画

void CGLView::OnDraw(CDC* pDC)
{
	ASSERT(pDC);
	Render();
}

/////////////////////////////////////////////////////////////////////////////
// CGLView 診断

#ifdef _DEBUG
void CGLView::AssertValid() const
{
	CView::AssertValid();
}

void CGLView::Dump(CDumpContext& dc) const
{
	CView::Dump(dc);
}
#endif //_DEBUG

/////////////////////////////////////////////////////////////////////////////
// CGLView メッセージ ハンドラ

void CGLView::OnLButtonDown(UINT nFlags, CPoint point) 
{
	m_bLButtonDown = true;
	m_prevpos = point;
	//	if (!GetCapture()) SetCapture();//XXX 2003-02-25

	m_ezgl.BeginScene();
	SetProjection();
	//	SetModelView();
	//	m_prev3dpos = m_ezgl.Conv2Dto3D(point.x, point.y, Vec3(0,0,0));
	MatrixF	mt1 = m_axis.GetInvGlobalMatrix();	
	Vec4 vd1 = Vec4(0, 0,m_camerapos-50, 0)*mt1;
	Vec3 V2 = Vec3(vd1.x, vd1.y, vd1.z);
	m_prev3dpos = m_ezgl.Conv2Dto3D(point.x,point.y,V2);//XXX 2003-04-08

	m_ezgl.EndScene();

	if(GetAsyncKeyState(VK_MENU)&0x8000 && GetCapture() != this)
		SetCapture();	// added by XXX 2004.6.10

	CView::OnLButtonDown(nFlags, point);
}

void CGLView::OnLButtonUp(UINT nFlags, CPoint point) 
{
	m_bLButtonDown = false;
	if (!(m_bLButtonDown || m_bRButtonDown || m_bMButtonDown)) {
		ReleaseCapture();
	}

	CView::OnLButtonUp(nFlags, point);
}

void CGLView::OnMButtonDown(UINT nFlags, CPoint point) 
{	
	m_bMButtonDown = true;
	m_prevpos = point;
	// added by XXX 2004.6.10
	if (GetCapture() != this)
		SetCapture();
	ASSERT(nFlags - nFlags == 0 && point.x - point.x ==0);
	//CView::OnMButtonDown(nFlags,point);
}

void CGLView::OnMButtonUp(UINT nFlags, CPoint point) 
{
	m_bMButtonDown = false;
	if (!(m_bLButtonDown || m_bRButtonDown || m_bMButtonDown)) {
		ReleaseCapture();
	}
	ASSERT(nFlags - nFlags == 0 && point.x - point.x ==0);
	//CView::OnMButtonUp(nFlags,point);
}

void CGLView::OnRButtonDown(UINT nFlags, CPoint point) 
{
	m_bRButtonDown = true;
	m_prevpos = point;

	if(GetAsyncKeyState(VK_MENU)&0x8000 && GetCapture() != this)
		SetCapture();	// added by XXX 2004.6.10

	CView::OnRButtonDown(nFlags, point);
}

void CGLView::OnRButtonUp(UINT nFlags, CPoint point) 
{
	m_bRButtonDown = false;
	if (!(m_bLButtonDown || m_bRButtonDown || m_bMButtonDown)) {
		ReleaseCapture();
	}

	CView::OnRButtonUp(nFlags, point);
}

void CGLView::OnMouseMove(UINT nFlags, CPoint point) 
{
	bool bIsScreenOp = false;

	// modified by XXX 2003-12-18
	float dx = (float)(point.x - m_prevpos.x);
	float dy = (float)(point.y - m_prevpos.y);

	m_ezgl.BeginScene();
	SetProjection();
	//SetModelView();
	MatrixF	mt1 = m_axis.GetInvGlobalMatrix();	
	Vec4 vd1 = Vec4(0, 0,m_camerapos-50, 0)*mt1;
	Vec3 V2 = Vec3(vd1.x, vd1.y, vd1.z);
	Vec3 curpos = m_ezgl.Conv2Dto3D(point.x,point.y,V2);//XXX 2003-04-08
	//	Vec3 curpos = m_ezgl.Conv2Dto3D(point.x, point.y, Vec3(0,0,0));
	m_ezgl.EndScene();

	// added by XXX 2004.11.18
	int nOperation = -1;
	nOperation = (m_nMouseOp!=-1)?m_nMouseOp:(m_bLButtonDown?m_scenceOper:-1);

	//AfxTrace("m_sceneop=%d, nOperation=%d\n", m_scenceOper, nOperation);

	// modified by XXX 2004..11.17
	if(nOperation != -1 && GetCapture() != this)
	{
		SetCapture();
	}

	//if ( bAltKeyDown && (m_bMButtonDown) 
	//	|| ((m_bLButtonDown)&&(m_bRButtonDown))
	//	|| (SCALE == m_scenceOper && m_bLButtonDown) )	// modified by XXX 2003-12-18
	if (nOperation == SCALE)
	{
		//			m_camerapos -= dx;
		if(fabs(dx) -fabs(dy) >= 0.)
			Zoom(-dx);
		else
			Zoom(dy);

		SetCursor(AfxGetApp()->LoadCursor(IDC_ARROW));	// added by XXX 2004.8.9
		bIsScreenOp = true;

		Render();
	}
	//else if( ((bAltKeyDown) && m_bLButtonDown)
	//	|| (ROTATE==m_scenceOper && m_bLButtonDown))
	else if( nOperation == ROTATE)
	{
		//if (m_bLButtonDown) )
		{
			//the following codes added by XXX 2003-04-01
//			float bk = m_roty;
			dy *= -0.01f;
			m_roty += dy; 

			if(m_rot_Axis==1)
			{
				m_axis.Rotate(NULL, Vec3(1,0,0), dy);
			}
			else if(m_rot_Axis==2 || GetAsyncKeyState(VK_CONTROL)&0x8000)//rotate around the y-axis when the ctrl key is pressed
			{
				m_axis.Rotate(&m_axis, Vec3(0,1,0), -dx*0.01f);
			}
			else if(m_rot_Axis==3)
			{
				m_axis.Rotate(&m_axis, Vec3(0,0,1), -dx*0.01f);
			}
			else
			{
				m_axis.Rotate(&m_axis, Vec3(0,1,0), -dx*0.01f);
				m_axis.Rotate(NULL, Vec3(1,0,0), dy);               
				//m_axis.Rotate(NULL, Vec3(0,1,0), -dx*0.01f);          //added by XXX for bug1144
			}

			// modified by XXX 2004.7.17
			SetViewDir();
			SetCursor(AfxGetApp()->LoadCursor(IDC_ARROW));	// added by XXX 2004.8.9
			bIsScreenOp = true;

			Render();		
		}

	}
	else if( nOperation == TRANSLATE)
	{
		//bool flag=bAltKeyDown ) &&(m_bRButtonDown)
		//	||(TRANSLATE==m_scenceOper&& m_bLButtonDown) );
		//if (flag) 
		{
			m_ezgl.BeginScene();
			SetProjection();
			SetModelView();
			Vec3 prepos = m_ezgl.Conv2Dto3D(m_prevpos.x, m_prevpos.y, Vec3(0,0,0));
			Vec3 curpos = m_ezgl.Conv2Dto3D(point.x, point.y, Vec3(0,0,0));
			m_ezgl.EndScene();
			m_axis.Translate(NULL, (curpos - prepos) * 1.0f );//XXX 2003-04-08
			//m_TranslatedVector += curpos - prepos;

			// modified by XXX 2004.7.17
			SetViewDir();

			SetCursor(AfxGetApp()->LoadStandardCursor(IDC_ARROW));	// added by XXX 2004.8.9
			bIsScreenOp = true;

			Render();
		}
	}
	m_prevpos = point;
	m_prev3dpos = curpos;


	if(!bIsScreenOp && m_scenceOper != TRANSLATE && m_scenceOper != ROTATE && m_scenceOper != SCALE)// added by XXX [10/31/2005]
	{
		SetCurCursor();
	}
		

	// added by XXX 2004.9.28
	if(m_pWndStatusBar)
	{
		ShowStatusCoordinateInfo(point);
	}

	CView::OnMouseMove(nFlags, point);

}

BOOL CGLView::OnMouseWheel(UINT nFlags, short zDelta, CPoint pt) 
{
	Zoom(float(zDelta)*0.1f);//XXX changed for new request,  float(-zDelta)*0.1f;    
	Render();

	return CView::OnMouseWheel(nFlags, zDelta, pt);
}


void CGLView::OnInitialUpdate() 
{
	CView::OnInitialUpdate();

	if (!m_ezgl.Initialized()) {
		m_axis.SetIdentity();
		m_axis.Rotate(&m_axis, Vec3(0,1,0), Math::Deg2Rad(-50.0f));
		m_axis.Rotate(NULL, Vec3(1,0,0), -Math::Deg2Rad(10.0f));
		m_roty = Math::Deg2Rad(-10.0f);
		m_camerapos = START_CAMERADIST;

		m_ezgl.Create(GetSafeHwnd());

		m_ezgl.BeginScene();		
		SetProjection();
		glLightModeli(GL_LIGHT_MODEL_TWO_SIDE, GL_TRUE);
		glShadeModel(GL_SMOOTH);
		glDisable(GL_CULL_FACE);
		glBlendFunc(GL_SRC_ALPHA,GL_ONE_MINUS_SRC_ALPHA);

		//ライトＯＮ
		GLfloat lspec[4] 	= {0.4f, 0.4f, 0.4f, 0.0f};
		GLfloat dspec[4]	= {0.7f, 0.7f, 0.7f, 1.0f};
		//GLfloat lpos[4] 	= {0.0f, 100.0f, 100.0f, 0.0f};
		GLfloat lpos[4] 	= {0.0f, -500.f, 500.0f, 0.0f};

		glLightfv(GL_LIGHT0, GL_DIFFUSE, dspec);
		glLightfv(GL_LIGHT0, GL_SPECULAR, lspec);
		glLightfv(GL_LIGHT0, GL_POSITION, lpos);
		glLightf(GL_LIGHT0, GL_SPOT_CUTOFF,  180.0f);
		glEnable(GL_LIGHT0);
		GLfloat mat_Ambient[] = {0.9f, 0.9f, 0.9f, 1.0f};
		glLightModelfv(GL_LIGHT_MODEL_AMBIENT, mat_Ambient);
		glEnable(GL_LIGHTING);

		//デブスバッファの初期化
		glDepthRange(0.0f, 1.0f);
		glDepthFunc(GL_LEQUAL);
		glEnable(GL_DEPTH_TEST); 

		//カラーモードON
		glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE);
		glEnable(GL_COLOR_MATERIAL);
		glEnable(GL_NORMALIZE);

		SetProjection();
		SetModelView();
		m_ezgl.EndScene();

		BuildScene();
	}
}

BOOL CGLView::OnEraseBkgnd(CDC* pDC) 
{
	// TODO: この位置にメッセージ ハンドラ用のコードを追加するかまたはデフォルトの処理を呼び出してください

	//return CView::OnEraseBkgnd(pDC);
	ASSERT(pDC);
	return TRUE;
}

void CGLView::OnDestroy() 
{
	m_ezgl.Release();
	CView::OnDestroy();	
}

void CGLView::RotAxis()
{
	MatrixF& m = m_axis.GetGlobalMatrix();
	glMultMatrixf((GLfloat*)&m);
}


//---------------------------------------------------------------
// Name:	     SetModelView()
// Description:  set the ModelView matrix, use gluLookAt funtion
// Argument:     none
// Return:       void
// Author:		  
// Date:		 
// Modified by:	 XXX
// Updated date: 2006/04/18
//----------------------------------------------------------------
void CGLView::SetModelView()
{
	glMatrixMode( GL_MODELVIEW );
	glLoadIdentity();
	gluLookAt(
	m_eye.x, m_eye.y, m_camerapos+300., 
	m_eye.x, m_eye.y, 0.0f, 
	0.0f, 1.0f, 0.0f);
	/*gluLookAt(
	m_eye.x, -(m_camerapos+300.), m_eye.z, 
	m_eye.x, 0.0f, m_eye.z,
	0.0f, 0.0f, -1.0f);*/
}

//---------------------------------------------------------------
// Name:	     SetProjection(int x, int y,double size)
// Description:  set the projection matrix,(for also setting picking matrix).
// Argument:     x   --the x coordinate for set pick projection matrix
//               y   --the y coordinate for set pick projection matrix
//               size--for set the size of pick projection matrix
// Return:       void
// Author:		  
// Date:		 
// Modified by:	 XXX				XXX
// Updated date: 2006/04/18        06/06/2006
//----------------------------------------------------------------
void CGLView::SetProjection(int x, int y,double size)	// yao 2002-4-25 modify
{
	glMatrixMode( GL_PROJECTION ); 
	glLoadIdentity();

	//if (x != -1) {
		GLint viewport[4];
		glGetIntegerv(GL_VIEWPORT, viewport);
		//pyy modified for select an dxf pattern 2002/04/08
		if(!SelectDxfFlag)
		{
			gluPickMatrix((GLdouble)x, (GLdouble)(viewport[3]-y), size,size,viewport); // yao 2002-3-27 modfiy 0.00001
		}
		else
		{
			gluPickMatrix((GLdouble)x, (GLdouble)(viewport[3]-y), 5.0,5.0,viewport); 
		}
	float aspect =  float(m_ezgl.Width()) / float(m_ezgl.Height());

	// the last near and far parameter are changed from 5000 to 20000 to fix bug 1623, modified by XXX[4/28/2005]
	glOrtho(-m_camerapos, m_camerapos, -m_camerapos/aspect, m_camerapos/aspect, -20000.0f, 20000.0f); // new setting
	glMatrixMode( GL_MODELVIEW );
}	

//---------------------------------------------------------------
// Name:	     SetProjection
// Description:  set the projection matrix.
// Argument:     void
// Return:       void
// Author:		XXX  
// Date:		 06/07/2006
// Modified by:	 
// Updated date: 
//----------------------------------------------------------------
void CGLView::SetProjection()
{
	glMatrixMode( GL_PROJECTION ); 
	glLoadIdentity();

	float aspect =  float(m_ezgl.Width()) / float(m_ezgl.Height());

	glOrtho(-m_camerapos, m_camerapos, -m_camerapos/aspect, m_camerapos/aspect, -20000.0f, 20000.0f); // new setting
	glMatrixMode( GL_MODELVIEW );
}	

void CGLView::SetPerView(dFloat ang)
{
	glMatrixMode( GL_PROJECTION ); 
	glLoadIdentity();

	assert(ang > 0 && ang < 180);

	dFloat aspect =  dFloat(m_ezgl.Width()) / dFloat(m_ezgl.Height());
	gluPerspective(ang, aspect, 100.0f, 20000.0f);    // modified
	glMatrixMode( GL_MODELVIEW );
}

void CGLView::SetIdentityView(void)
{
	m_axis.SetIdentity();
}

LRESULT CGLView::DefWindowProc(UINT message, WPARAM wParam, LPARAM lParam) 
{
	switch (message) {
	case WM_MBUTTONDOWN:
		OnMButtonDown((UINT)wParam, CPoint(LOWORD(lParam), HIWORD(lParam)));
		break;
	case WM_MBUTTONUP:
		OnMButtonUp((UINT)wParam, CPoint(LOWORD(lParam), HIWORD(lParam)));
		break;
	}

	return CView::DefWindowProc(message, wParam, lParam);
}

void CGLView::OnSize(UINT nType, int cx, int cy) 
{
	CView::OnSize(nType, cx, cy);

	if (m_ezgl.Initialized()) {
		m_ezgl.OnSize();
		m_ezgl.BeginScene();
		SetProjection();
		m_ezgl.EndScene();
	}
}


void CGLView::Render()
{
	m_ezgl.BeginScene();
	SetProjection();
	glClearColor(0,0,0,0);
	glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT);
	m_ezgl.Flip();
	m_ezgl.EndScene();
}

void CGLView::BuildScene()
{
}


//---------------------------------------------------------------
// Name:	     BeginPicking(const CPoint& p,double size)	
// Description:  initialize for pick process
// Argument:     p--the current mouse point for pick
//               size--for set the size of pick projection matrix
// Return:       void
// Author:		  
// Date:		 
// Modified by:	 XXX
// Updated date: 2006/04/18
//----------------------------------------------------------------
void CGLView::BeginPicking(const CPoint& p,double size)	// yao 2002-4-25 modify
{
	m_ezgl.BeginScene();
	SetProjection();

	//セレクションモード移行手続き
	glSelectBuffer(/*1024, */20000, m_pickbuf); // modified by XXX [09/13/2006]
	glRenderMode(GL_SELECT);
	glInitNames();
	glPushName(30000);

	glMatrixMode( GL_PROJECTION ); 
	glPushMatrix();
	SetProjection(p.x, p.y,size);	

	//glMatrixMode( GL_PROJECTION ); 
	SetModelView();
	RotAxis();
}

//---------------------------------------------------------------
// Name:	     EndPicking()
// Description:  at the end of picking process,calculate the selected object
// Argument:     none
// Return:       int--return the nearest object selected from the viewer 
// Author:		  
// Date:		 
// Modified by:	 XXX
// Updated date: 2006/04/18
//----------------------------------------------------------------
int CGLView::EndPicking()
{
	
	int i, j;

	glFlush();
	glPopName();


	int hits = glRenderMode(GL_RENDER);//GL_RENDER	

	int currentmode = -1;
	glGetIntegerv(GL_MATRIX_MODE,&currentmode);
	glMatrixMode( GL_PROJECTION ); 
	glPopMatrix();
	glMatrixMode(currentmode);

	m_ezgl.EndScene(); 

	//	m_ezgl.EndScene();        //we concel the codes here
	//Pick解析
	int selected = -1, names;
	GLuint *ptr = m_pickbuf;
	GLuint min = 0xffffffff;
	for (i = 0; i < hits; i++) {
		names = *ptr++;
		GLuint z = *ptr;  //int
		ptr++;
		ptr++;
		for (j = 0; j < names; j++) {

			if (z < min) {
				min = z;
				selected = (int)*ptr;
			}
			ptr++;
			//goto end_pickanalize;
		}
	}

	return selected;
}


//---------------------------------------------------------------
// Name:	     EndPicking(int& minPicked, int& maxPicked)
// Description:  at the end of picking process,calculate the selected object
// Argument:     minPicked---the nearest object selected from the viewer
//               maxPicked---the furthest object selected from the viewer
// Return:       void 
// Author:		  
// Date:		 
// Modified by:	 XXX
// Updated date: 2006/04/18
//----------------------------------------------------------------
void CGLView::EndPicking(int& minPicked, int& maxPicked)
{
	minPicked=maxPicked=-1;

	int i, j;
	glFlush();
	glPopName();

	int hits = glRenderMode(GL_RENDER);

	int currentmode = -1;
	glGetIntegerv(GL_MATRIX_MODE,&currentmode);
	glMatrixMode( GL_PROJECTION ); 
	glPopMatrix();
	glMatrixMode(currentmode);

	m_ezgl.EndScene(); 

	int selected = -1, selected2=-1, names;
	GLuint *ptr = m_pickbuf;
	GLuint min = 0xffffffff;
	GLuint max = 0; ;
	bool flag=false;
	for (i = 0; i < hits; i++) {
		names = *ptr++;
		GLuint z = *ptr;  //int
		ptr++;
		ptr++;
		for (j = 0; j < names; j++) {

			if (z < min) {
				min = z;
				selected = (int)*ptr;
			}
			if(!flag)
			{
				flag=true;
				max= z;
				selected2= (int)*ptr;
			}
			else
			{
				if(z>max)
				{
					max=z;
					selected2= (int)*ptr;
				}
			}
			ptr++;
		}
	}
	minPicked=selected;
	maxPicked=selected2;	
}


//---------------------------------------------------------------
// Name:	     EndPicking(varray<int>& idArr)
// Description:  at the end of picking process,calculate the selected object
// Argument:     idArr--all object selected is in the array
// Return:       if idArr.size() >0 return true, else return false 
// Author:		  
// Date:		 
// Modified by:	 XXX
// Updated date: 2006/04/18
//----------------------------------------------------------------
bool CGLView::EndPicking(varray<int>& idArr)
{
	int i, j;

	idArr.clear();

	glFlush();
	glPopName();

	int hits = glRenderMode(GL_RENDER);//GL_RENDER	

	int currentmode = -1;
	glGetIntegerv(GL_MATRIX_MODE,&currentmode);
	glMatrixMode( GL_PROJECTION ); 
	glPopMatrix();
	glMatrixMode(currentmode);

	m_ezgl.EndScene(); 

	//Pick解析
	int /*selected = -1, */names;
	GLuint *ptr = m_pickbuf;
	for (i = 0; i < hits; i++) {
		names = *ptr++;
		ptr++;
		ptr++;
		for (j = 0; j < names; j++) {

			idArr.push_back((int)*ptr);
			ptr++;
		}
	}
	if(idArr.size()>0)
		return true;
	return false;
}


//---------------------------------------------------------------
// Name:	     EndPickingOnView(varray<int>& idArr)
// Description:  at the end of picking process, get the selected object
// Argument:     idArr--all object selected is in the array
// Return:       if idArr.size() >0 return true, else return false 
// Author:		 Li 
// Date:		 2003/12/19
// Modified by:	 XXX
// Updated date: 2006/04/18
//----------------------------------------------------------------
bool CGLView::EndPickingOnView(varray<int>& idArr)
{
	int i, j;
	glFlush();
	glPopName();
	int hits = glRenderMode(GL_RENDER);

	int currentmode = -1;
	glGetIntegerv(GL_MATRIX_MODE,&currentmode);
	glMatrixMode( GL_PROJECTION ); 
	glPopMatrix();
	glMatrixMode(currentmode);

	m_ezgl.EndScene(); 

	int /*selected = -1, */names;
	GLuint *ptr = m_pickbuf;
//	GLuint min = 0xffffffff;

	idArr.clear();
	varray<GLuint> zdt;
	for (i = 0; i < hits; i++) 
	{
		names = *ptr++;
		GLuint z = *ptr;  
		ptr++;
		ptr++;
		for (j = 0; j < names; j++) 
		{
			zdt.push_back(z);
			idArr.push_back((int)*ptr);
			ptr++;
		}
	}

	if(zdt.size() == 0)
	{
		return false;
	}

	int tempZ;
	GLuint tempPtr;
	for(i=0; i<static_cast<int>(zdt.size()-1); i++)
		for(j=i+1; j<static_cast<int>(zdt.size()); j++)
		{
			if(zdt[i]>zdt[j])
			{
				tempPtr = idArr.at(i);
				tempZ = zdt.at(i);			
				idArr.at(i) =idArr.at(j);
				zdt.at(i)=zdt.at(j);
				idArr.at(j)=tempPtr;
				zdt.at(j)=tempZ;
			}
		}

		return idArr.size()>0;
}

void CGLView::MouseClick(UINT menuid,CPoint point)
{
	CMenu menu;	
	if(menu.LoadMenu(menuid))
	{
		menu.GetSubMenu(0)->TrackPopupMenu(TPM_LEFTALIGN|TPM_RIGHTBUTTON,point.x,point.y,this);
		menu.GetSubMenu(0)->DestroyMenu();
	}
}

void CGLView::Zoom(float dx)
{
	const float NZ3DZoomMax = 20000.0;	// 3D拡大操作カメラ位置最大値
#ifdef _DEBUG
	const float NZ3DZoomMin = 0.05;
#endif
#ifdef NDEBUG
	const float NZ3DZoomMin = 50.0;      // 3D拡大操作カメラ位置最小値			if( p3DView->m_camerapos > NZ3DZoomMax ) p3DView->m_camerapos = NZ3DZoomMax;
#endif
	

#ifdef _DEBUG
	if(m_camerapos < 5)
		m_camerapos +=dx*0.1f;
	else if(m_camerapos < 1)
		m_camerapos +=dx*0.02f;
	else
		m_camerapos += dx;

#endif
#ifdef NDEBUG
	m_camerapos += dx;
#endif
	if( m_camerapos < NZ3DZoomMin ) 
	{
		m_camerapos = NZ3DZoomMin;
	}
	else if( m_camerapos > NZ3DZoomMax ) 
	{
		m_camerapos = NZ3DZoomMax;
	}
}

void CGLView::Translate(const Vec3 &transpt)
{
	m_axis.Translate(NULL, transpt);
}

void CGLView::Rotate(float dx, float dy, float radian)
{
	ASSERT(dx - dx - dy + dy - radian + radian + 1 > 0);
}

bool CGLView::IsFileOpen()const
{
	return m_bIsFileOpen;
}

void CGLView::FileNew()
{

}

void CGLView::FileOpen()
{

}

bool CGLView::FileSave()
{
	return false;
}

void CGLView::FileSaveAs()
{

}

void CGLView::UnDo()
{

}
void CGLView::ReDo()
{


}

bool CGLView::IsCanUnDo()const
{
	return false;
}


bool CGLView::IsCanReDo()const
{
	return false;
}

void CGLView::SetShowDrawingLines(void)//XXX 2003-01-22
{
	m_bIsShowDrawingLines = true;
}

void CGLView::SetHideDrawingLines(void)//XXX 2003-01-22
{
	m_bIsShowDrawingLines = false;
}

//----------------------------------------------------------------------
// name:		IsShowDrawingLines
// function:	show drawing lines or not
// argument:	void		
// return:		void
// author:		unknown
// date:		unknown
// update:	    
// author:		XXX
// date:		04/20/2006
//----------------------------------------------------------------------
bool  CGLView::IsShowDrawingLines(void)
{
	return m_bIsShowDrawingLines;
}

bool  CGLView::IsCanCopy(void)const
{
	return false;
}

bool  CGLView::IsCanCut(void)const
{
	return false;
}
bool  CGLView::IsCanPaste(void)const
{
	return false;
}

void CGLView::OnMButtonDblClk(UINT nFlags, CPoint point)
{
	ZoomAll();

	ASSERT(nFlags - nFlags == 0 && point.x - point.x ==0);
	//CView::OnMButtonDblClk(nFlags,point);
}

void CGLView::ZoomAll()
{

}


// modified by XXX - 2003-8-27
// Display rectangle for region picking
void CGLView::DrawRectAroundPoint(CPoint point, Vec3 pt3d, float size, COLORREF color)
{//Modifide by YJY 2006.05.25
	Vec3 p1, p2, p3, p4, base0, base1;
//	int x, y;
//	float z;

	float offset = 2000.0f;
	size = size/2;

	base0 = m_ezgl.Conv2Dto3D(point.x, point.y, Vec3(0,0,0));
	point.x += (LONG)size;
	base1 = m_ezgl.Conv2Dto3D(point.x, point.y, Vec3(0,0,0));

	p1 = ::RotAxis(m_viewy, m_viewdir, (float)Pi/4.f);
	p2 = ::RotAxis(p1, m_viewdir, (float)Pi/2.f);
	p3 = -p1;
	p4 = -p2;

	float rat = (base1 - base0).Magnitude();
	p1 *= 1.3f * rat;
	p2 *= 1.3f * rat;
	p3 *= 1.3f * rat;
	p4 *= 1.3f * rat;


	p1 += pt3d;
	p2 += pt3d;
	p3 += pt3d;
	p4 += pt3d;

	p1 += m_viewdir * offset;
	p2 += m_viewdir * offset;
	p3 += m_viewdir * offset;
	p4 += m_viewdir * offset;

	glColor3f(GetRValue(color)/255.0f, GetGValue(color)/255.0f, GetBValue(color)/255.0f); 
	glLineWidth(1.0);

	glBegin(GL_LINE_LOOP);	
	glVertex3f(p1.x, p1.y, p1.z);
	glVertex3f(p2.x, p2.y, p2.z);
	glVertex3f(p3.x, p3.y, p3.z);
	glVertex3f(p4.x, p4.y, p4.z);
	glEnd();	
}


void CGLView::OnActivateView(BOOL bActivate, CView* pActivateView, CView* pDeactiveView)
{
	// is the status of this view has been changed
	bool bTemp = bActivate==TRUE;
	bool bStateChanged = ( m_bActive != bTemp );
	m_bActive = bTemp;

	// if it has been changed, redraw the window
	if( bStateChanged )
	{
		RedrawWindow();
	}

	CView::OnActivateView(bActivate, pActivateView, pDeactiveView);
}

//----------------------------------------------------------------------
// name:		IsActive
// function:	get the state of view 
// argument:	void
// return:		bool
// author:		unknown
// date:		unknown
// update:	    
// author:		XXX
// date:		04/19/2006
//----------------------------------------------------------------------
bool CGLView::IsActive(void)
{
	return m_bActive;
}

void CGLView::SetPerspectiveView(void)
{
	m_axis.SetIdentity();
	m_axis.Rotate(&m_axis, Vec3(0,1,0), Math::Deg2Rad(45.0f));
	m_axis.Rotate(NULL, Vec3(1,0,0), -Math::Deg2Rad(10.0f));
	m_axis.Translate(NULL, Vec3(0,-75.0,0));
	m_roty = -Math::Deg2Rad(10.0f);
	m_camerapos = 300.0f;
}

void CGLView::OnSetFocus(CWnd* pOldWnd)
{
	CView::OnSetFocus(pOldWnd);

	// TODO: Add your message handler code here
}

void CGLView::OnKillFocus(CWnd* pNewWnd)
{
	CView::OnKillFocus(pNewWnd);

	// TODO: Add your message handler code here
}

void CGLView::OnKeyDown(UINT nChar, UINT nRepCnt, UINT nFlags)
{
	CView::OnKeyDown(nChar, nRepCnt,nFlags);
}

void CGLView::OnKeyUp(UINT nChar, UINT nRepCnt, UINT nFlags)
{
	CView::OnKeyUp(nChar, nRepCnt,nFlags);
}

// added by XXX 2004.7.17
void CGLView::SetViewDir(void)
{
	//============== for the wireframe hidden ================
	MatrixF mt = m_axis.GetInvGlobalMatrix();
	Vec4 vd = Vec4(0, 0, 1, 0)*mt;
	m_viewdir = Vec3(vd.x, vd.y, vd.z);
	//============== for the wireframe hidden ================

	//Added by YJY 2006.05.25
	Vec4 vy = Vec4(0, 1, 0, 0)*mt;
	m_viewy = Vec3(vy.x, vy.y, vy.z);
	//The end
}

// added by XXX 2004.8.7
BOOL CGLView::OnSetCursor(CWnd* pWnd, UINT nHitTest, UINT message)
{
	//if bAltKeyDown) 
	//{	
	//	ifbAltKeyDown &&( m_bMButtonDown) || (m_bLButtonDown) 
	//		&& (m_bRButtonDown)) ))
	//	//if(m_bMButtonDown || (m_bLButtonDown && m_bRButtonDown))
	//	{
	//		SetCursor(AfxGetApp()->LoadCursor (IDC_MAGNIFY));
	//	}
	////}
	//else
	//{
	//	SetCursor(AfxGetApp()->LoadStandardCursor (IDC_ARROW)); 
	//}
	//following code to set cursur for new request
	//if(m_scenceOper == TRANSLATE /*&& m_bLButtonDown*/)// commented by XXX [10/28/2005]
	//{
	//	SetCursor(AfxGetApp()->LoadStandardCursor(IDC_SIZEALL));		
	//}
	//else if(m_scenceOper == ROTATE/*&&m_bLButtonDown*/)// commented by XXX [10/28/2005]
	//{
	//	SetCursor(AfxGetApp()->LoadCursor(IDC_ROTATE3D));
	//}
	//else if(m_scenceOper == SCALE/*&&m_bLButtonDown*/)// commented by XXX [10/28/2005]
	//{
	//	SetCursor(AfxGetApp()->LoadCursor(IDC_MAGNIFY));
	//}
	//// over
	ASSERT(pWnd);
	ASSERT(nHitTest - nHitTest + message -message + 1 > 0);
	return TRUE;
}

void CGLView::OnNcLButtonUp(UINT nHitTest, CPoint point)
{
	m_bLButtonDown = false;

	CView::OnNcLButtonUp(nHitTest, point);
}

void CGLView::OnNcMButtonUp(UINT nHitTest, CPoint point)
{
	m_bMButtonDown = false;

	CView::OnNcMButtonUp(nHitTest, point);
}

void CGLView::OnNcRButtonUp(UINT nHitTest, CPoint point)
{
	m_bRButtonDown = false;

	CView::OnNcRButtonUp(nHitTest, point);
}

void CGLView::SetCurCursor(void)
{
	
	SetCursor(AfxGetApp()->LoadStandardCursor(IDC_ARROW));
}

//---------------------------------------------------------------
// Name:	    billboardCheatSphericalBegin()
// Description: Undo all rotation operations, so the measure text can always be on the front 
// Argument:   
//         :	
// Return:		
// Author:		 
// Date:		 
// Modified by:	 XXX
// Updated date: 2005/09/14 14:9:2005   9:58	
//----------------------------------------------------------------
void CGLView::billboardCheatSphericalBegin()
{

	float modelview[16];
	int i,j;

	int matrixMode;
	glGetIntegerv(GL_MATRIX_MODE, &matrixMode);

	// save the current modelview matrix
	glMatrixMode(GL_MODELVIEW_MATRIX);	// added by XXX 2004.9.3
	glPushMatrix();

	// get the current modelview matrix
	glGetFloatv(GL_MODELVIEW_MATRIX , modelview);

	// undo all rotations
	// beware all scaling is lost as well 
	for( i=0; i<3; i++ ) 
		for( j=0; j<3; j++ ) {
			if ( i==j )
				modelview[i*4+j] = 1.0;
			else
				modelview[i*4+j] = 0.0;
		}

		// set the modelview with no rotations
		glLoadMatrixf(modelview);

		glMatrixMode(matrixMode);
}

// added by XXX 2004.9.3
void CGLView::billboardEnd()
{

	// restore the previously 
	// stored modelview matrix
	glPopMatrix();
}

//// added by XXX 2004.9.15
//CString CGLView::GetPrecision(void)
//{
//	CString temp;
//	switch(m_precisionType) {
//	case UNIT_PRECISION1:
//		temp = "%.1f";
//		break;
//	case UNIT_PRECISION2:
//		temp = "%.2f";
//		break;
//	case UNIT_PRECISION3:
//		temp = "%.3f";
//		break;
//	case UNIT_PRECISION4:
//		temp = "%.4f";
//		break;
//	default:
//		ASSERT(FALSE);
//		break;
//	}
//	return temp;
//}

//---------------------------------------------------------------
// Name:	    DrawArrow()
// Description: Draw an arrow shape,for measurement display
// Argument:   x,y,z:--the arrow head point coordinate;
//         :   iDirection:-- arrow direction; fScale:-- arrow scale which decides the arrow sizes
// Return:	   void
// Author:		 
// Date:		 
// Modified by:	 XXX
// Updated date: 2005/09/14 14:9:2005   10:01	
//----------------------------------------------------------------
void CGLView::DrawArrow(float x, float y, float z, int iDirection, float fScale)
{
	float fAngle=0;
	switch(iDirection)
	{
	case ARROW_LEFT:
		fAngle=0;
		break;
	case ARROW_RIGHT:
		fAngle=180;
		break;
	case ARROW_UPER:
		fAngle=-90;
		break;
	case ARROW_LOWER:
		fAngle=90;
		break;
	default:
		break;
	}
	glPushMatrix();
	glTranslatef(x, y, z);
	glRotatef(fAngle, 0,0,1);
	glBegin(GL_LINES);
	glVertex3f(0,0,0);
	glVertex3f(1.732f*fScale,1*fScale,0);
	glVertex3f(0,0,0);
	glVertex3f(1.732f*fScale,-1*fScale,0);
	glEnd();
	glPopMatrix();
}
//---------------------------------------------------------------
// Name:	    DrawArc()
// Description: Draw an arc, for measurement display
// Argument:    fRadius:-- the arc radius;  fAngle:--the arc central angle
//         :	
// Return:		void
// Author:		 
// Date:		 
// Modified by:	 XXX
// Updated date: 2005/09/14 14:9:2005   10:25	
//----------------------------------------------------------------
void CGLView::DrawArc(float fRadius, float fAngle)
{
	fAngle = fAngle/2;

	glBegin(GL_LINES);
	{
		for(int i=(int)fAngle; i>0; --i)
		{
			float x,y;
			x = (float)(fRadius*cos((float)i/180*Pi));
			y = (float)(fRadius*sin((float)i/180*Pi));
			glVertex3f(x, y, 0);
			x = (float)(fRadius*cos((float)(i-1)/180*Pi));
			y = (float)(fRadius*sin((float)(i-1)/180*Pi));
			glVertex3f(x, y, 0);
			x = (float)(fRadius*cos(-(float)i/180*Pi));
			y = (float)(fRadius*sin(-(float)i/180*Pi));
			glVertex3f(x, y, 0);
			x = (float)(fRadius*cos(-(float)(i-1)/180*Pi));
			y = (float)(fRadius*sin(-(float)(i-1)/180*Pi));
			glVertex3f(x, y, 0);
		}
	}
	glEnd();
}

void CGLView::ShowStatusCoordinateInfo(const CPoint &point)
{
	if(!m_pWndStatusBar)
		return;

	m_ezgl.BeginScene();
	SetProjection();
	Vec3 pt = m_ezgl.Conv2Dto3D(point.x,point.y,Vec3(0,0,0)); 
	m_ezgl.EndScene();

	pt = pt / 1;

	CString precision = _T(".3f%");
	CString str;

	str.Format(precision, pt.x);
	str = _T("x:") + str;
	m_pWndStatusBar->SetPaneText(1, str, TRUE);

	str.Format(precision, pt.y);
	str = _T("y:" )+ str;
	m_pWndStatusBar->SetPaneText(2, str, TRUE);

	str.Format(precision, pt.z);
	str = _T("z:") + str;
	m_pWndStatusBar->SetPaneText(3, str, TRUE);

}

void CGLView::ShowStatusCoordinateInfo(const Vec3 &vecPt)
{
	if(!m_pWndStatusBar)
		return;

	CString strx,stry,strz;
	Vec3 pt = vecPt;
	float scale = 1;
	CString precision = _T(".3f%");

	strx.Format(precision, pt.x/scale);
	strx = _T("x:") + strx;
	stry.Format(precision, pt.y/scale);
	stry = _T("y:") + stry;
	strz.Format(precision, pt.z/scale);
	strz = _T("z:" ) + strz;

	m_pWndStatusBar->SetPaneText(1,strx,TRUE);
	m_pWndStatusBar->SetPaneText(2,stry,TRUE);
	m_pWndStatusBar->SetPaneText(3,strz,TRUE);
}
void CGLView::DetectMouseOp(UINT nFlags)
{

	m_nMouseOp = -1;

	m_bLButtonDown = (nFlags&MK_LBUTTON) != 0;
	m_bRButtonDown = (nFlags&MK_RBUTTON) != 0;
	m_bMButtonDown = (nFlags&MK_MBUTTON) != 0;

	bool bAltKeyDown = (bool)( (GetAsyncKeyState(VK_MENU)&0x8000) == 0x8000);

	if(bAltKeyDown && ((m_bLButtonDown && m_bRButtonDown) || m_bMButtonDown))
	{
		m_nMouseOp = SCALE;
		return;
	}

	if(bAltKeyDown && m_bLButtonDown)
	{
		m_nMouseOp = ROTATE;
		return;
	}

	if(bAltKeyDown && m_bRButtonDown)
	{
		m_nMouseOp = TRANSLATE;
		return;
	}
}

void CGLView::ConvertVecToPoint(varray<Vec3>&vecArr, varray<CPoint>& resPArr)
{
	resPArr.clear();
	if(vecArr.size()==0)
		return;

	int x,y; 
	float z;

	resPArr.resize(vecArr.size());
	m_ezgl.BeginScene();
	SetProjection();
	int size = (int)vecArr.size();
	for(int i=0; i<size; i++)
	{
		m_ezgl.Conv3Dto2D(&x,&y,&z, &vecArr.at(i));
		resPArr[i] = CPoint(x,y);
	}
	m_ezgl.EndScene();
}