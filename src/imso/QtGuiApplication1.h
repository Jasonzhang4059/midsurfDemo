#pragma once

#include <QtWidgets/QMainWindow>
#include "ui_QtGuiApplication1.h"


class QtGuiApplication1 : public QMainWindow
{
	Q_OBJECT

public:
	QtGuiApplication1(QWidget *parent = Q_NULLPTR);

public:
	//测试文件函数
	void test();

protected:
	//设置左侧的显示数据
	void setDockWidget_data(varray<NurbsVol> vol);
	void setDockWidget_data(varray<NurbsSurface> vol);
	void setDockWidget_data(varray<NurbsLine> vol);

	//打开授权按钮
	void authorityOpen();
public slots:
	//双击左侧数据反应函数
	void slotTest(QTreeWidgetItem * in1, int in2);
	void slotChanged(QTreeWidgetItem * in1, int in2);
private:
	varray<SplineSurface> Spf;
	varray<SplineVolume> Spv;
	varray<Spline> Spl;
	Ui::QtGuiApplication1Class ui;
	Option option;
};
