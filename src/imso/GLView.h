/********************************************************************
* FILE NAME		:CGLView.h
* AUTHOR		:
* DATE			:
* MODIFIER		:
* MODIFY DATE	:
* DESCRIPTION	:header file
* version		:1.0
********************************************************************/

#if !defined(AFX_GLVIEW_H__4840CC5C_E341_40AA_9503_8655D9D28002__INCLUDED_)
#define AFX_GLVIEW_H__4840CC5C_E341_40AA_9503_8655D9D28002__INCLUDED_

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000
// GLView.h
//

#include "../includes/view/EasyGLView.h"
#include "../includes/base.h"
#include "../includes/view/Node.h"


using namespace base;

#define START_CAMERADIST		500 

/////////////////////////////////////////////////////////////////////////////
// CGLView 

enum VIEW_DIRECTION
{
	FRONT_VIEW=0,
	BACK_VIEW,
	LEFT_VIEW,
	RIGHT_VIEW,
	TOP_VIEW,
	BOTTOM_VIEW,
	PERSPECTIVE_VIEW
};


class  CGLView : public CView
{
	DECLARE_DYNCREATE(CGLView)

public:
	CEasyGlView		m_ezgl;
	HGLRC			m_hGLContext;

	CStatusBar *	m_pWndStatusBar;
	COLORREF		m_rgbModelColor;	// XXX 2003.5.1
	COLORREF		m_rgbGridColor;

	float			m_camerapos;
	Vec3			m_eye;
	Vec3			m_viewdir; 
	Vec3			m_viewy;            //Added by YJY 2006.5.25


	bool			m_bActive;
	int				m_rot_Axis;
	Node			m_axis;
	float			m_roty;
	//text display

	//pick buf
	GLuint			m_pickbuf[/*1024*/20000]; // modified by XXX [09/13/2006]

	bool			m_bIsShowDrawingLines;//XXX 2003-01-23

	bool			m_bIsFileOpen;
	bool			SelectDxfFlag;

	//mouse position
	CPoint			m_downpos;
	CPoint			m_prevpos;
	Vec3			m_prev3dpos;

	//mouse state
	bool			m_bLButtonDown;
	bool			m_bMButtonDown;
	bool			m_bRButtonDown;

	//render mode
	int				m_render;		
	int				m_renderBefBmpDrag;

	//sreen transformation operator
	enum OPERATION
	{
		ROTATE=0,
		TRANSLATE,
		SCALE,
	};
	int			m_scenceOper; 
	bool		m_bscenceOperSet;
	int			m_nMouseOp;   
	Vec3		m_TranslatedVector;	// added by XXX 2003.11.17

	enum _arrow_direction
	{
		ARROW_LEFT, 
		ARROW_RIGHT, 
		ARROW_UPER, 
		ARROW_LOWER
	};

protected:
	CGLView();           
	virtual ~CGLView();

public:
	bool			IsActive(void);
	
	void			RotAxis();
	void			Zoom(float dx);
	int&			GetRotateAxis(){return m_rot_Axis;}

	//pick
	void			BeginPicking(const CPoint& p,double size=0.001);//initialize for pick process
	int				EndPicking();//at the end of picking process,calculate the selected object
	bool			EndPickingOnView(varray<int>& idArr);
	bool			EndPicking(varray<int>& idArr);//at the end of picking process,calculate the selected object
	void			EndPicking(int& minPicked, int& maxPicked);//at the end of picking process,calculate the selected object

	void			MouseClick(UINT menuid,CPoint point); // popMenu

	void			billboardCheatSphericalBegin();
	void			billboardEnd();

	void			SetShowDrawingLines(void);
	void			SetHideDrawingLines(void);
	bool			IsShowDrawingLines(void);

	// Display rectangle for region picking
	void			DrawRectAroundPoint(CPoint point, Vec3 pt3d, float size, COLORREF color = RGB(0,0,0)); //2003-08-27
	//aid line
	bool			DrawAdidedLine(Vec3 preV, CPoint preP, CPoint currP, float thresholdDis, CPoint& resP);//2003-08-27 Li 

	void			DrawArrow(float x, float y, float z, int iDirection, float fScale = 3.0);
	void			DrawArc(float fRadius, float fAngle);

	// added by XXX 2004.9.28
	virtual void	ShowStatusCoordinateInfo(const CPoint &point);
	virtual void	ShowStatusCoordinateInfo(const Vec3 &vecPt);// by XXX[1/13/2005]
	
	// added by XXX 2004.11.16
	void			DetectMouseOp(UINT nFlags);
	int				GetMouseOp(){return m_nMouseOp;}

	void			SetPerspectiveView(void);
	void			SetIdentityView(void);
	void			ConvertVecToPoint(varray<Vec3>&vecArr, varray<CPoint>& resPArr);	

	// ClassWizard
	//{{AFX_VIRTUAL(CGLView)
	public:
	virtual void OnInitialUpdate();
	protected:
	virtual void OnDraw(CDC* pDC);      
	virtual LRESULT DefWindowProc(UINT message, WPARAM wParam, LPARAM lParam);
	//}}AFX_VIRTUAL
public:
	virtual void	BuildScene();

	virtual void	SetModelView();//set the ModelView matrix
	virtual void	SetViewDir(void);
	//set the projection matrix,if x!=-1,set the pick projection matrix
	virtual void	SetProjection(int x, int y,double size=0.001);
	virtual void	SetProjection();
	void			SetPerView(dFloat ang);

	virtual void	ZoomAll();
	virtual void	Translate(const Vec3 &transpt);
	virtual void	Rotate(float dx, float dy, float radian);

	virtual void	FileNew();
	virtual void	FileOpen();
	virtual bool	FileSave();
	virtual void	FileSaveAs();
	virtual bool	IsFileOpen()const;

	virtual void	UnDo();
	virtual void	ReDo();
	virtual bool	IsCanUnDo()const;
	virtual bool	IsCanReDo()const;

	virtual bool	IsCanCopy(void)const;
	virtual bool	IsCanCut(void)const;
	virtual bool	IsCanPaste(void)const;

	virtual void	SetCurCursor(void);	// added by XXX 2004.8.9

	virtual void	Render();

public:
	//{{AFX_MSG(CGLView)
	afx_msg void OnLButtonDown(UINT nFlags, CPoint point);
	afx_msg void OnLButtonUp(UINT nFlags, CPoint point);
	afx_msg void OnLButtonDblClk(UINT nFlags, CPoint point){ASSERT(nFlags - nFlags == 0 && point.x - point.x ==0);}
	afx_msg void OnRButtonDown(UINT nFlags, CPoint point);
	afx_msg void OnRButtonUp(UINT nFlags, CPoint point);
	afx_msg void OnRButtonDblClk(UINT nFlags, CPoint point){ASSERT(nFlags - nFlags == 0 && point.x - point.x ==0);}
	afx_msg void OnMouseMove(UINT nFlags, CPoint point);
	afx_msg void OnMButtonDblClk(UINT nFlags, CPoint point);
	afx_msg BOOL OnMouseWheel(UINT nFlags, short zDelta, CPoint pt);
	afx_msg BOOL OnSetCursor(CWnd* pWnd, UINT nHitTest, UINT message);
	afx_msg BOOL OnEraseBkgnd(CDC* pDC);
	afx_msg void OnSetFocus(CWnd* pOldWnd);
	afx_msg void OnKillFocus(CWnd* pNewWnd);
	afx_msg void OnDestroy();
	afx_msg void OnSize(UINT nType, int cx, int cy);
	afx_msg void OnKeyUp(UINT nChar, UINT nRepCnt, UINT nFlags);
	afx_msg void OnKeyDown(UINT nChar, UINT nRepCnt, UINT nFlags);
	afx_msg void OnNcLButtonUp(UINT nHitTest, CPoint point);
	afx_msg void OnNcMButtonUp(UINT nHitTest, CPoint point);
	afx_msg void OnNcRButtonUp(UINT nHitTest, CPoint point);
	//}}AFX_MSG
	DECLARE_MESSAGE_MAP()
	virtual void OnActivateView(BOOL bActivate, CView* pActivateView, CView* pDeactiveView);

protected:
	void OnMButtonDown(UINT nFlags, CPoint point);
	void OnMButtonUp(UINT nFlags, CPoint point);
#ifdef _DEBUG
	virtual void	AssertValid() const;
	virtual void	Dump(CDumpContext& dc) const;
#endif
};

/////////////////////////////////////////////////////////////////////////////

//{{AFX_INSERT_LOCATION}}
// Microsoft Visual C++ 

//----------------------
// Dll Import
//----------------------


#endif // !defined(AFX_GLVIEW_H__4840CC5C_E341_40AA_9503_8655D9D28002__INCLUDED_)
