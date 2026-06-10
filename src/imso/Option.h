#pragma once

#include <qstring.h>
#include <fstream>
#include <QOpenGLFunctions_4_5_Core>
#include "stdafx.h"
#include "RWGeometric.h"
#include "PolyIGA.h"

class Option:protected QOpenGLFunctions_4_5_Core
{
public:
	//第一次画图还是第二次画图，0是第一次，非零第二次
	int m_again = 0;
	Option();
	~Option();
	//开锁，关锁操作
	void setLockOn();//开锁
	void setLockClose();//关锁
	bool isLcokOn();//判断锁状态

	//文件读取之后的操作
	void OnBnClickedCreateModel(QOpenGLContext * p, QWidget * father);//正常显示NurbsVol
	void OnBnClickedCreateModel(QOpenGLContext * p, QWidget * father, CPolyParaVolume polyv);//读取一个CPolyParaVolume并显示
	void OnBnClickedViewre();
	void OnBnClicked3MF();//制作3mf文件

	//模型的渲染函数
	void createData(QOpenGLContext * p, varray<varray<varray<point3d>>> varrayPoint, varray<varray<varray<point3d>>> varrayLine);//渲染均质模型
	void createData(QOpenGLContext * p, varray<varray<varray<Vec3>>> varrayPoint, varray<varray<varray<Vec3>>> varrayLine);//渲染均质模型
	void createAllData(QOpenGLContext * p, QWidget * father, CPolyParaVolume polyv);//非均质模型的渲染
	void createData(QOpenGLContext * p, vector<float> data);//渲染一个非均质模型片
	void createMode();
	void createCP(QOpenGLContext * p);//控制点的输入

	//转换函数
	void TransCnurbsvolToPolyIGA(varray<NurbsVol> &nurbsVols, const CPolyParaVolume poly);//Curbs转换为CPolyParaVolume

	//调试之中单独使用的渲染函数
	void drawNurbsVol(QOpenGLContext * p, QWidget * father, varray<NurbsVol> nvs);//显示出来NurbsVol
	void drawNurbsSurface(QOpenGLContext * p, QWidget * father, varray<NurbsSurface> nsfs);//显示出来NurbsSurface
	void drawNurbsLine(QOpenGLContext * p, QWidget * father, varray<NurbsLine> nls);//显示出来NurbsLine
	void drawBezierLine(QOpenGLContext * p, QWidget * father, varray<BezierLine> bzls);//显示BezierLine
	void drawpoint4d(QOpenGLContext * p, QWidget * father, varray<varray<point4d>> p4d2);//显示point4d
	void drawpoint3d(QOpenGLContext * p, QWidget * father, varray<varray<point3d>> p3d2);//显示point3d
	void reDraw(QOpenGLContext * p, QWidget * father);//重新绘制

	void drawSplineVolume(QOpenGLContext * p, QWidget * father, varray<SplineVolume> nvs);//显示出来SplineVolume
	void drawSplineSurface(QOpenGLContext * p, QWidget * father, varray<SplineSurface> nsfs);//显示出来SplineSurface
	void drawSpline(QOpenGLContext * p, QWidget * father, varray<Spline> nls);//显示出来Spline
	void drawVec4(QOpenGLContext * p, QWidget * father, varray<varray<Vec4>> p4d2);//显示Vec4
	void drawVec3(QOpenGLContext * p, QWidget * father, varray<varray<Vec3>> p3d2);//显示Vec3
private:
	//当前绘制图形窗口的句柄
	QOpenGLContext * _openglContext;
	//控件数值变量
	QString m_modelPath;
	//根据个人的电脑可以设置不同的最大线程个数，
	//注意，本程序是计算密集型程序，并不是越多的线程越好
	//线程增多反而会造成程序的计算能力下降
	int m_threadNum = 6;
	//设置修改锁，在图形显示之后打开锁
	bool _modifyLock = false;
protected:
	////保存参数至Setting
	//void SaveSetting();
	////刷新参数
	//void ReflashPara();
	//计算法向量
	point3d LegalVector(point3d a, point3d b, point3d c);
	//string分隔
	void Split(const std::string& s, varray<std::string>& v, const std::string& c);
};

