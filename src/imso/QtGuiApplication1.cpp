#include "QtGuiApplication1.h"
#include <qfiledialog.h>
#include <qcolordialog.h>
#include <qinputdialog.h>
#include <qprogressdialog.h>
#include <qdialog.h>

#include "MyDoc.h"
#include "sensitiveSettingDialog.h"
#include "modleSettingDialog.h"
#include "fragmentDialog.h"
#include "singlePic.h"
#include "readFileDialog.h"
#include "ChangeFineScaleDialog.h"
#include "camerLocatonDialog.h"
#include "PolyIGA.h"
#include "ReducerDialog.h"
#include "Geardialog.h"

#define nurbsline 3;
#define nurbssurface 4;
#define nurbsvol 5;

QtGuiApplication1::QtGuiApplication1(QWidget *parent)
	: QMainWindow(parent)
{
	ui.setupUi(this);
	setWindowState(Qt::WindowMaximized);//窗口最大化显示
	ui.centralWidget->grabKeyboard();//让widget具有键盘捕捉的能力
	MyDoc::Ptr pdoc = MyDoc::getInstance();
	//合并两个dockwidget
	tabifyDockWidget(ui.dockWidget_tools, ui.dockWidget_data);

	//设置dockWidget_data的初始显示
	ui.treeWidget->setHeaderLabels(QStringList() << QString::fromLocal8Bit("参数名称") << QString::fromLocal8Bit("数值/数量"));

	//测试文件
	//----------
	connect(ui.actiongear, &QAction::triggered, [=]() {
		//测试新文件
		ui.centralWidget->releaseKeyboard();
		Geardialog dialog(this);
		dialog.exec();
		ui.centralWidget->grabKeyboard();
		QString path = dialog.getPath();
		varray<SplineVolume> gear = dialog.getGear();
		if (!path.isEmpty()) {
			RWGeometric rwg;
			rwg.WriteSplineVolume(path.toStdString(), gear);
		}
		QOpenGLContext * openglContext = ui.centralWidget->context();//获得渲染环境
		option.drawSplineVolume(openglContext, this, gear);
		option.createCP(openglContext);
		pdoc->cleanUp();

	});

	connect(ui.actionReducer, &QAction::triggered, [=]() {
		//测试新文件
		ui.centralWidget->releaseKeyboard();
		ReducerDialog dialog(this);
		dialog.exec();
		ui.centralWidget->grabKeyboard();
		QString path = dialog.getPath();
		varray<SplineVolume> reducer = dialog.getReducer();
		if (!path.isEmpty()) {
			RWGeometric rwg;
			rwg.WriteSplineVolume(path.toStdString(), reducer);
		}
		QOpenGLContext * openglContext = ui.centralWidget->context();//获得渲染环境
		option.drawSplineVolume(openglContext, this, reducer);
		option.createCP(openglContext);
		pdoc->cleanUp();

	});




	connect(ui.actionclear_all, &QAction::triggered, [=]() {
		pdoc->clearScrn();
	});
	
	//读取文件
	//----------
	connect(ui.actionreadFile, &QAction::triggered, [=]() {
		readFileDialog dialog(this);
		dialog.exec();
		QOpenGLContext * openglContext = ui.centralWidget->context();//获得渲染环境

		option.OnBnClickedCreateModel(openglContext, this);
		option.createCP(openglContext);//进行显示计算

		//临时的处理方法，进行数组的清空
		//但是仍然会出现问题，在创造程序的一瞬间内存占有率仍然会过大
		pdoc->cleanUp();

		authorityOpen();
	});

	//生成3mf文件
	//----------
	connect(ui.actionsaveAs3MF, &QAction::triggered, [=]() {
		//option.OnBnClickedCreateModel();
		option.OnBnClicked3MF();

		//临时的处理方法，进行数组的清空
		//但是仍然会出现问题，在创造程序的一瞬间内存占有率仍然会过大
		MyDoc::Ptr pdoc = MyDoc::getInstance();
		pdoc->cleanUp();
	});

	//灵敏度设置
	//----------
	connect(ui.actionsenstive, &QAction::triggered, [=]() {
		ui.centralWidget->releaseKeyboard();//释放键盘捕捉的能力
		sensitiveSettingDialog dialog(this);
		dialog.exec();
		ui.centralWidget->grabKeyboard();
	});

	//颜色设置
	//----------
	connect(ui.actionbordLineColor, &QAction::triggered, [=]() {
		ui.centralWidget->releaseKeyboard();//释放键盘捕捉的能力
		QColor color = QColorDialog::getColor(Qt::white);
		if (color.isValid())
		{
			MyDoc::Ptr pdoc = MyDoc::getInstance();
			//颜色数据类型转化
			pdoc->_bordLineColor = { (float)color.red() / (float)255,(float)color.green() / (float)255,
				(float)color.blue() / (float)255 };
		}
		ui.centralWidget->grabKeyboard();
	});

	connect(ui.actionsurfaceColor, &QAction::triggered, [=]() {
		ui.centralWidget->releaseKeyboard();//释放键盘捕捉的能力
		QColor color = QColorDialog::getColor(Qt::white);
		if (color.isValid())
		{
			MyDoc::Ptr pdoc = MyDoc::getInstance();
			pdoc->_surfaceColor = { (float)color.red() / 255.0f,(float)color.green() / 255.0f,
				(float)color.blue() / 255.0f };
		}
		ui.centralWidget->grabKeyboard();
	});

	connect(ui.actionControlpointColor, &QAction::triggered, [=]() {
		ui.centralWidget->releaseKeyboard();//释放键盘捕捉的能力
		QColor color = QColorDialog::getColor(Qt::white);
		if (color.isValid())
		{
			MyDoc::Ptr pdoc = MyDoc::getInstance();
			pdoc->_controlPoinrColor = { (float)color.red() / 255.0f,(float)color.green() / 255.0f,
				(float)color.blue() / 255.0f  };
		}
		ui.centralWidget->grabKeyboard();
	});

	connect(ui.actionsliceSurfaceColor, &QAction::triggered, [=]() {
		ui.centralWidget->releaseKeyboard();//释放键盘捕捉的能力
		QColor color = QColorDialog::getColor(Qt::white);
		if (color.isValid())
		{
			MyDoc::Ptr pdoc = MyDoc::getInstance();
			pdoc->_sliceSurfaceColor = { (float)color.red() / 255.0f,(float)color.green() / 255.0f,
				(float)color.blue() / 255.0f  };
		}
		ui.centralWidget->grabKeyboard();
	});

	connect(ui.actionsliceSupportColor, &QAction::triggered, [=]() {
		ui.centralWidget->releaseKeyboard();//释放键盘捕捉的能力
		QColor color = QColorDialog::getColor(Qt::white);
		if (color.isValid())
		{
			MyDoc::Ptr pdoc = MyDoc::getInstance();
			pdoc->_sliceSuppotrColor = { (float)color.red() / 255.0f,(float)color.green() / 255.0f,
				(float)color.blue() / 255.0f  };
		}
		ui.centralWidget->grabKeyboard();
	});

	//是否显示坐标系
	//--------
	connect(ui.actioncoordinate, &QAction::triggered, [=]() {
		pdoc->m_showCoordinates = !pdoc->m_showCoordinates;
	});

	//设置显示模式，点，线，面
	//----------
	connect(ui.actionPoints, &QAction::triggered, [=]() {
		pdoc->m_GLMode = 0;
		ui.actionLines->setChecked(false);
		ui.actionsurface->setChecked(false);
	});

	connect(ui.actionLines, &QAction::triggered, [=]() {
		pdoc->m_GLMode = 3;
		ui.actionPoints->setChecked(false);
		ui.actionsurface->setChecked(false);
	});

	connect(ui.actionsurface, &QAction::triggered, [=]() {
		pdoc->m_GLMode = 7;
		ui.actionPoints->setChecked(false);
		ui.actionLines->setChecked(false);
	});

	//是否显示控制点
	//----------
	connect(ui.actioncontrolPoints, &QAction::triggered, [=]() {
		pdoc->m_showCtrlPts = !pdoc->m_showCtrlPts;
	});

	//模型参数设置
	//----------
	connect(ui.actionmodelSetting, &QAction::triggered, []() {
		modleSettingDialog *dialog = new modleSettingDialog;
		dialog->show();
		dialog->setAttribute(Qt::WA_DeleteOnClose);
	});

	//单片显示
	//----------
	connect(ui.actionsinglePic, &QAction::triggered, [=]() {
		ui.centralWidget->releaseKeyboard();//释放键盘捕捉的能力
		singlePic *dialog = new singlePic;
		dialog->show();
		dialog->setAttribute(Qt::WA_DeleteOnClose);
		ui.centralWidget->grabKeyboard();
	});

	//切片
	//----------
	connect(ui.actionFragment, &QAction::triggered, [=]() {
		ui.centralWidget->releaseKeyboard();//释放键盘捕捉的能力
		fragmentDialog dialog(this);
		dialog.exec();
		ui.centralWidget->grabKeyboard();
	});

	//重新设置num
	//----------
	connect(ui.actionchangeFineScale, &QAction::triggered, [=]() {
		ui.centralWidget->releaseKeyboard();//释放键盘捕捉的能力
		ChangeFineScaleDialog dialog(this);
		dialog.exec();
		QOpenGLContext * openglContext = ui.centralWidget->context();//获得渲染环境
		option.OnBnClickedCreateModel(openglContext,this);//进行数据计算
		option.createCP(openglContext);//进行显示计算

		//临时的处理方法，进行数组的清空
		//但是仍然会出现问题，在创造程序的一瞬间内存占有率仍然会过大
		pdoc->cleanUp();
		ui.centralWidget->grabKeyboard();
	});

	//设置摄像机所在的位置
	//----------
	connect(ui.actioncameraLocation, &QAction::triggered, [=]() {
		ui.centralWidget->releaseKeyboard();//释放键盘捕捉的能力
		camerLocatonDialog dialog(this);
		dialog.exec();
		ui.centralWidget->grabKeyboard();
	});

	//工具栏的显示与关闭
	//----------
	connect(ui.actiontools, &QAction::triggered, [=]() {
		if (ui.actiontools->isChecked())
		{
			ui.dockWidget_tools->show();
			ui.dockWidget_data->show();
		}
		else
		{
			ui.dockWidget_tools->close();
			ui.dockWidget_data->close();
		}
	});

	//重置视角按钮
	//connect(ui.initCameraLocationButton, &QPushButton::clicked, [=]() {
	//	pdoc->m_restView = true;
	//});


	//右侧数据点击响应
	connect(ui.treeWidget, &QTreeWidget::itemDoubleClicked,this,&QtGuiApplication1::slotTest);

	//右侧点击修改数据之后响应结束
	connect(ui.treeWidget, &QTreeWidget::itemChanged, this, &QtGuiApplication1::slotChanged);
}

//void QtGuiApplication1::authorityOpen()
//{
//	//原来是灰色的按钮，不可以使用，调用这个函数之后就可以使用
//	//---------
//	ui.actionsaveAs3MF->setEnabled(true);
//	ui.actionFragment->setEnabled(true);
//	ui.actionsliceSupportColor->setEnabled(true);
//	ui.actionsliceSurfaceColor->setEnabled(true);
//	ui.actionsinglePic->setEnabled(true);
//}

//void QtGuiApplication1::setDockWidget_data(varray<NurbsVol> vol)
//{
//
//	//用来设置显示在左侧的数据
//	QList<QTreeWidgetItem* > items;
//	for (int i = 0; i != vol.size(); ++i)
//	{
//		//创建一个根目录
//		QTreeWidgetItem * item = new QTreeWidgetItem(ui.treeWidget);
//		item->setText(0, QString::fromLocal8Bit("第")+QString::number(i)+ QString::fromLocal8Bit("片"));
//
//		//创建根目录下面的子目录
//		QList<QTreeWidgetItem *> childlist;
//		//放入各方向控制点数量
//		QTreeWidgetItem * m_uNum = new QTreeWidgetItem(QStringList() << "m_uNum" << QString::number(vol[i].m_uNum));
//		QTreeWidgetItem * m_vNum = new QTreeWidgetItem(QStringList() << "m_vNum" << QString::number(vol[i].m_vNum));
//		QTreeWidgetItem * m_wNum = new QTreeWidgetItem(QStringList() << "m_wNum" << QString::number(vol[i].m_wNum));
//		//放入各方向阶数
//		QTreeWidgetItem * m_uDegree = new QTreeWidgetItem(QStringList() << "m_uDegree" << QString::number(vol[i].m_uDegree));
//		QTreeWidgetItem * m_vDegree = new QTreeWidgetItem(QStringList() << "m_vDegree" << QString::number(vol[i].m_vDegree));
//		QTreeWidgetItem * m_wDegree = new QTreeWidgetItem(QStringList() << "m_wDegree" << QString::number(vol[i].m_wDegree));
//
//		//放入节点矢量
//		QTreeWidgetItem * m_uKnots = new QTreeWidgetItem(item);
//		QTreeWidgetItem * m_vKnots = new QTreeWidgetItem(item);
//		QTreeWidgetItem * m_wKnots = new QTreeWidgetItem(item);
//
//		m_uKnots->setText(0, "m_uKnots");
//		m_uKnots->setText(1, QString::number(vol[i].m_uKnots.size()));
//		m_vKnots->setText(0, "m_vKnots");
//		m_vKnots->setText(1, QString::number(vol[i].m_vKnots.size()));
//		m_wKnots->setText(0, "m_wKnots");
//		m_wKnots->setText(1, QString::number(vol[i].m_wKnots.size()));
//
//		//加入u节点矢量
//		for (int j = 0; j != vol[i].m_uKnots.size(); ++j)
//		{
//			QTreeWidgetItem * mm = new QTreeWidgetItem(m_uKnots);
//			mm->setText(0, QString::number(j));
//			mm->setText(1, QString::number(vol[i].m_uKnots[j]));
//		}
//		//加入v节点矢量
//		for (int j = 0; j != vol[i].m_vKnots.size(); ++j)
//		{
//			QTreeWidgetItem * mm = new QTreeWidgetItem(m_vKnots);
//			mm->setText(0, QString::number(j));
//			mm->setText(1, QString::number(vol[i].m_vKnots[j]));
//		}
//		//加入w节点矢量
//		for (int j = 0; j != vol[i].m_wKnots.size(); ++j)
//		{
//			QTreeWidgetItem * mm = new QTreeWidgetItem(m_wKnots);
//			mm->setText(0, QString::number(j));
//			mm->setText(1, QString::number(vol[i].m_wKnots[j]));
//		}
//
//		//加入控制点
//		QTreeWidgetItem * contralPoints = new QTreeWidgetItem(item);
//		contralPoints->setText(0, "m_CtrlPts");
//		contralPoints->setText(1, QString::number(vol[i].m_CtrlPts.size()));
//		for (int j=0; j != vol[i].m_CtrlPts.size(); ++j)
//		{
//			QTreeWidgetItem * aa = new QTreeWidgetItem(contralPoints);
//			aa->setText(0, QString::number(j));
//			//加入各个点的xyz坐标值
//			QTreeWidgetItem *x = new QTreeWidgetItem(aa);
//			x->setText(0, "x");
//			x->setText(1, QString::number(vol[i].m_CtrlPts[j].x));
//
//			QTreeWidgetItem *y = new QTreeWidgetItem(aa);
//			y->setText(0, "y");
//			y->setText(1, QString::number(vol[i].m_CtrlPts[j].y));
//
//			QTreeWidgetItem *z = new QTreeWidgetItem(aa);
//			z->setText(0, "z");
//			z->setText(1, QString::number(vol[i].m_CtrlPts[j].z));
//		}
//
//		childlist.push_back(m_uNum);
//		childlist.push_back(m_vNum);
//		childlist.push_back(m_wNum);
//		childlist.push_back(m_uDegree);
//		childlist.push_back(m_vDegree);
//		childlist.push_back(m_wDegree);
//		childlist.push_back(m_uKnots);
//		childlist.push_back(m_vKnots);
//		childlist.push_back(m_wKnots);
//		childlist.push_back(contralPoints);
//
//		item->addChildren(childlist);
//
//		//把创建好的一个根目录放进队列中
//		items.push_back(item);
//	}
//	ui.treeWidget->addTopLevelItems(items);
//}

//void QtGuiApplication1::setDockWidget_data(varray<NurbsSurface> vol)
//{
//	//用来设置显示在左侧的数据
//	QList<QTreeWidgetItem* > items;
//	for (int i = 0; i != vol.size(); ++i)
//	{
//		//创建一个根目录
//		QTreeWidgetItem * item = new QTreeWidgetItem(ui.treeWidget);
//		item->setText(0, QString::fromLocal8Bit("第") + QString::number(i) + QString::fromLocal8Bit("片"));
//
//		//创建根目录下面的子目录
//		QList<QTreeWidgetItem *> childlist;
//		//放入各方向控制点数量
//		QTreeWidgetItem * m_uNum = new QTreeWidgetItem(QStringList() << "m_uNum" << QString::number(vol[i].m_uNum));
//		QTreeWidgetItem * m_vNum = new QTreeWidgetItem(QStringList() << "m_vNum" << QString::number(vol[i].m_vNum));
//		//放入各方向阶数
//		QTreeWidgetItem * m_uDegree = new QTreeWidgetItem(QStringList() << "m_uDegree" << QString::number(vol[i].m_uDegree));
//		QTreeWidgetItem * m_vDegree = new QTreeWidgetItem(QStringList() << "m_vDegree" << QString::number(vol[i].m_vDegree));
//
//		//放入节点矢量
//		QTreeWidgetItem * m_uKnots = new QTreeWidgetItem(item);
//		QTreeWidgetItem * m_vKnots = new QTreeWidgetItem(item);
//
//		m_uKnots->setText(0, "m_uKnots");
//		m_uKnots->setText(1, QString::number(vol[i].m_uKnots.size()));
//		m_vKnots->setText(0, "m_vKnots");
//		m_vKnots->setText(1, QString::number(vol[i].m_vKnots.size()));
//
//		//加入u节点矢量
//		for (int j = 0; j != vol[i].m_uKnots.size(); ++j)
//		{
//			QTreeWidgetItem * mm = new QTreeWidgetItem(m_uKnots);
//			mm->setText(0, QString::number(j));
//			mm->setText(1, QString::number(vol[i].m_uKnots[j]));
//		}
//		//加入v节点矢量
//		for (int j = 0; j != vol[i].m_vKnots.size(); ++j)
//		{
//			QTreeWidgetItem * mm = new QTreeWidgetItem(m_vKnots);
//			mm->setText(0, QString::number(j));
//			mm->setText(1, QString::number(vol[i].m_vKnots[j]));
//		}
//
//		//加入控制点
//		QTreeWidgetItem * contralPoints = new QTreeWidgetItem(item);
//		contralPoints->setText(0, "m_CtrlPts");
//		contralPoints->setText(1, QString::number(vol[i].m_CtrlPts.size()));
//		for (int j = 0; j != vol[i].m_CtrlPts.size(); ++j)
//		{
//			QTreeWidgetItem * aa = new QTreeWidgetItem(contralPoints);
//			aa->setText(0, QString::number(j));
//			//加入各个点的xyz坐标值
//			QTreeWidgetItem *x = new QTreeWidgetItem(aa);
//			x->setText(0, "x");
//			x->setText(1, QString::number(vol[i].m_CtrlPts[j].x));
//
//			QTreeWidgetItem *y = new QTreeWidgetItem(aa);
//			y->setText(0, "y");
//			y->setText(1, QString::number(vol[i].m_CtrlPts[j].y));
//
//			QTreeWidgetItem *z = new QTreeWidgetItem(aa);
//			z->setText(0, "z");
//			z->setText(1, QString::number(vol[i].m_CtrlPts[j].z));
//		}
//
//		childlist.push_back(m_uNum);
//		childlist.push_back(m_vNum);
//		childlist.push_back(m_uDegree);
//		childlist.push_back(m_vDegree);
//		childlist.push_back(m_uKnots);
//		childlist.push_back(m_vKnots);
//		childlist.push_back(contralPoints);
//
//		item->addChildren(childlist);
//
//		//把创建好的一个根目录放进队列中
//		items.push_back(item);
//	}
//	ui.treeWidget->addTopLevelItems(items);
//}

//void QtGuiApplication1::setDockWidget_data(varray<NurbsLine> vol)
//{
//	//用来设置显示在左侧的数据
//	QList<QTreeWidgetItem* > items;
//	for (int i = 0; i != vol.size(); ++i)
//	{
//		//创建一个根目录
//		QTreeWidgetItem * item = new QTreeWidgetItem(ui.treeWidget);
//		item->setText(0, QString::fromLocal8Bit("第") + QString::number(i) + QString::fromLocal8Bit("片"));
//
//		//创建根目录下面的子目录
//		QList<QTreeWidgetItem *> childlist;
//		//放入各方向阶数
//		QTreeWidgetItem * m_uDegree = new QTreeWidgetItem(QStringList() << "m_uDegree" << QString::number(vol[i].m_Degree));
//
//		//放入节点矢量
//		QTreeWidgetItem * m_uKnots = new QTreeWidgetItem(item);
//
//		m_uKnots->setText(0, "m_uKnots");
//		m_uKnots->setText(1, QString::number(vol[i].m_Knots.size()));
//
//		//加入u节点矢量
//		for (int j = 0; j != vol[i].m_Knots.size(); ++j)
//		{
//			QTreeWidgetItem * mm = new QTreeWidgetItem(m_uKnots);
//			mm->setText(0, QString::number(j));
//			mm->setText(1, QString::number(vol[i].m_Knots[j]));
//		}
//
//		//加入控制点
//		QTreeWidgetItem * contralPoints = new QTreeWidgetItem(item);
//		contralPoints->setText(0, "m_CtrlPts");
//		contralPoints->setText(1, QString::number(vol[i].m_CtrlPts.size()));
//		for (int j = 0; j != vol[i].m_CtrlPts.size(); ++j)
//		{
//			QTreeWidgetItem * aa = new QTreeWidgetItem(contralPoints);
//			aa->setText(0, QString::number(j));
//			//加入各个点的xyz坐标值
//			QTreeWidgetItem *x = new QTreeWidgetItem(aa);
//			x->setText(0, "x");
//			x->setText(1, QString::number(vol[i].m_CtrlPts[j].x));
//
//			QTreeWidgetItem *y = new QTreeWidgetItem(aa);
//			y->setText(0, "y");
//			y->setText(1, QString::number(vol[i].m_CtrlPts[j].y));
//
//			QTreeWidgetItem *z = new QTreeWidgetItem(aa);
//			z->setText(0, "z");
//			z->setText(1, QString::number(vol[i].m_CtrlPts[j].z));
//		}
//
//		childlist.push_back(m_uDegree);
//		childlist.push_back(m_uKnots);
//		childlist.push_back(contralPoints);
//
//		item->addChildren(childlist);
//
//		//把创建好的一个根目录放进队列中
//		items.push_back(item);
//	}
//	ui.treeWidget->addTopLevelItems(items);
//}

//void QtGuiApplication1::slotTest(QTreeWidgetItem * in1, int in2)
//{
//	MyDoc::Ptr pdoc = MyDoc::getInstance();
//	QString text = in1->text(in2);//获得所选的内容
//	QRegExp temp(QString::fromLocal8Bit("第"));
//	if (text.contains(temp))//查看是否匹配单个显示体模型
//	{
//		QRegularExpression exp("\\d{1,2}");
//		QRegularExpressionMatch match = exp.match(text);
//		if (match.hasMatch())
//		{
//			QString matched = match.captured();
//			pdoc->m_CellIdx = matched.toInt();
//		}
//	}
//	if (in1->parent())
//	{
//		//如果点击的是控制点中的xyz任意一个坐标轴的值
//		if (in1->parent()->parent())
//		{
//			//修改xyz坐标轴的值
//			ui.treeWidget->openPersistentEditor(in1, 1);
//		}
//	}
//}

//void QtGuiApplication1::slotChanged(QTreeWidgetItem * in1, int in2)
//{
//	if (option.isLcokOn())
//	{
//		//获取改变的值
//		QString a = in1->text(in2);
//		ui.treeWidget->closePersistentEditor(in1, in2);
//		MyDoc::Ptr pdoc = MyDoc::getInstance();
//		pdoc->clearScrn();
//		option.m_again = 1;
//		//控制点改变为刚才修改的值
//		//------------
//		//找到爷爷节点，也就是第几个体
//		QString matched;
//		QRegularExpression exp("\\d{1,2}");
//		QRegularExpressionMatch match = exp.match(in1->parent()->parent()->parent()->text(0));
//		if (match.hasMatch())
//		{
//			matched = match.captured();
//		}
//		//找到是第几个节点
//		QString temp = in1->parent()->text(0);
//		//找到具体位置
//		if (in1->text(0) == "x")
//		{
//			pdoc->m_ShowData4D[matched.toInt()][temp.toInt()].x = a.toFloat();
//		}else		if (in1->text(0) == "y")
//		{
//			pdoc->m_ShowData4D[matched.toInt()][temp.toInt()].y = a.toFloat();
//		}else		if (in1->text(0) == "z")
//		{
//			pdoc->m_ShowData4D[matched.toInt()][temp.toInt()].z = a.toFloat();
//		}
//		//重新画出图形
//		QOpenGLContext * openglContext = ui.centralWidget->context();//获得渲染环境
//		option.OnBnClickedCreateModel(openglContext, this);
//		option.createCP(openglContext);//进行显示计算
//	}
//}

void QtGuiApplication1::test()
{
	//控制点细化6*6
	auto ControlPSpecify = [](varray<NurbsVol> &nvs)
	{
		vector<double> knot = { 0.25,0.5,0.75 };
		varray<double> uknots, vknots, wknots;
		for (auto i : knot)
		{
			uknots.push_back(i);
			vknots.push_back(i);
			wknots.push_back(i);
		}
		for (int i = 0; i != nvs.size(); ++i)
		{
			nvs[i].KnotsRefine(uknots, vknots, wknots);
		}
	};

	auto GloblControlP = [](CPolyParaVolume &cppv)
	{
		//生成全局控制点
		int idx = 0;
		for (idx; idx < cppv.m_HexVolumes[0].m_splVol.m_vAllCtrlPts.size(); idx++) {
			cppv.m_HexVolumes[0].m_controlPtGlobalIDs.push_back(idx);
		}
		cppv.m_HexVolumes[0].m_bGenerateGlobalID = true;
		for (int i = 1; i < cppv.m_HexVolumes.size(); i++) {
			for (int j = 0; j < cppv.m_HexVolumes[i].m_splVol.m_vAllCtrlPts.size(); j++) {
				bool flag = true;
				for (int k = 0; k < i; k++) {
					for (int t = 0; t < cppv.m_HexVolumes[k].m_splVol.m_vAllCtrlPts.size(); t++) {
						if (abs(cppv.m_HexVolumes[i].m_splVol.m_vAllCtrlPts[j].m_pt.x - cppv.m_HexVolumes[k].m_splVol.m_vAllCtrlPts[t].m_pt.x) < ERRF
							&& abs(cppv.m_HexVolumes[i].m_splVol.m_vAllCtrlPts[j].m_pt.y - cppv.m_HexVolumes[k].m_splVol.m_vAllCtrlPts[t].m_pt.y) < ERRF
							&& abs(cppv.m_HexVolumes[i].m_splVol.m_vAllCtrlPts[j].m_pt.z - cppv.m_HexVolumes[k].m_splVol.m_vAllCtrlPts[t].m_pt.z) < ERRF)
						{
							cppv.m_HexVolumes[i].m_controlPtGlobalIDs.push_back(cppv.m_HexVolumes[k].m_controlPtGlobalIDs[t]);
							flag = false;
							break;
						}
					}
					if (flag == false)
						break;
				}
				if (flag == true) {
					cppv.m_HexVolumes[i].m_controlPtGlobalIDs.push_back(idx);
					idx++;
				}
			}
			cppv.m_HexVolumes[i].m_bGenerateGlobalID = true;
		}
	};

	//auto ChangeOrderedVol = [](const varray<NurbsVol> &vol, varray<NurbsVol> &orderVol) {
	//	int patchNum = vol.size();
	//	for (int i = 0; i < patchNum; i++) {
	//		
	//	}
	//};

	QOpenGLContext * openglContext = ui.centralWidget->context();//获得渲染环境
	MyDoc::Ptr pdoc = MyDoc::getInstance();


	RWGeometric rwg;
	varray<NurbsVol> nvs;
	rwg.ReadNurbsVol("D:\\quadTest\\多孔板2\\duokongban-vol.txt", nvs);
	for (int i = 0; i != nvs.size(); ++i)
	{
		nvs[i].DegreeElevate(2, 2, 2);
	}
	rwg.WriteNurbsVol("D:\\quadTest\\多孔板2\\duokongban-vol-2degree.txt", nvs);
	//ControlPSpecify(nvs);
	CPolyParaVolume cppv;
	cppv = nvs;
	cppv.SetAdjacentHexIdx();
	cppv.Order();
	GloblControlP(cppv);
	//cppv.OutputParaVolumeDataTxt("test/1.txt", "test/2.txt");
	cppv.OutputParaVolumeDataVTK("D:\\quadTest\\多孔板2\\duokongban-vol.vtk");
	nvs.clear();
	option.TransCnurbsvolToPolyIGA(nvs, cppv);
	rwg.WriteNurbsVol("D:\\quadTest\\多孔板2\\duokongban-vol-ordered.txt", nvs);
	//setDockWidget_data(nvs);

	//由于文件读取之后没有选取显示的模型，特比选取一下
	//pdoc->modelType = 5;

	option.drawNurbsVol(openglContext, this, nvs);
	option.OnBnClickedCreateModel(openglContext, this);
	option.createCP(openglContext);//进行显示计算
	//下面的是显示的函数，必须保留
	option.setLockOn();

	////测试新文件
	//QOpenGLContext * openglContext = ui.centralWidget->context();//获得渲染环境
	////新的测试函数可以写在下面
	//RWGeometric rwg;
	//varray<NurbsVol> nvs;
	//rwg.ReadNurbsVol("test/new_allvols20_3根正交管02 - 副本.txt", nvs);

	////控制点细化6*6
	///*vector<double> knot={0.25,0.5,0.75};
	//varray<double> uknots,vknots,wknots;
	//for (auto i : knot)
	//{
	//	uknots.push_back(i);
	//	vknots.push_back(i);
	//	wknots.push_back(i);
	//}
	//for (int i = 0; i != nvs.size(); ++i)
	//{
	//	nvs[i].KnotsRefine(uknots, vknots, wknots);
	//}*/
	//CPolyParaVolume cppv;
	//cppv = nvs;
	//cppv.SetAdjacentHexIdx();
	//cppv.Order();
	//
	////生成全局控制点
	//int idx = 0;
	//for (idx; idx < cppv.m_HexVolumes[0].m_splVol.m_vAllCtrlPts.size(); idx++) {
	//	cppv.m_HexVolumes[0].m_controlPtGlobalIDs.push_back(idx);
	//}

	//for (int i = 1; i < cppv.m_HexVolumes.size(); i++) {
	//	for (int j = 0; j < cppv.m_HexVolumes[i].m_splVol.m_vAllCtrlPts.size(); j++) {
	//		bool flag = true;
	//		for (int k = 0; k < i; k++) {
	//			for (int t = 0; t < cppv.m_HexVolumes[k].m_splVol.m_vAllCtrlPts.size(); t++) {
	//				if (abs(cppv.m_HexVolumes[i].m_splVol.m_vAllCtrlPts[j].m_pt.x - cppv.m_HexVolumes[k].m_splVol.m_vAllCtrlPts[t].m_pt.x) < ERRF
	//					&& abs(cppv.m_HexVolumes[i].m_splVol.m_vAllCtrlPts[j].m_pt.y - cppv.m_HexVolumes[k].m_splVol.m_vAllCtrlPts[t].m_pt.y) < ERRF
	//					&& abs(cppv.m_HexVolumes[i].m_splVol.m_vAllCtrlPts[j].m_pt.z - cppv.m_HexVolumes[k].m_splVol.m_vAllCtrlPts[t].m_pt.z) < ERRF)
	//				{
	//					cppv.m_HexVolumes[i].m_controlPtGlobalIDs.push_back(cppv.m_HexVolumes[k].m_controlPtGlobalIDs[t]);
	//					flag = false;
	//					break;
	//				}
	//			}
	//			if (flag == false)
	//				break;
	//		}
	//		if (flag == true) {
	//			cppv.m_HexVolumes[i].m_controlPtGlobalIDs.push_back(idx);
	//			idx++;
	//		}
	//	}
	//	cppv.m_HexVolumes[i].m_bGenerateGlobalID = true;
	//}
	//cppv.OutputParaVolumeDataTxt("test/1.txt", "test/2.txt");

	//MyDoc::Ptr pdoc = MyDoc::getInstance();
	////setDockWidget_data(nvs);

	////由于文件读取之后没有选取显示的模型，特比选取一下
	//pdoc->modelType = 5;

	//option.drawNurbsVol(openglContext, this, nvs);
	//option.OnBnClickedCreateModel(openglContext, this);
	//option.createCP(openglContext);//进行显示计算
	////下面的是显示的函数，必须保留
	//option.setLockOn();
}

void QtGuiApplication1::authorityOpen()
{
	//原来是灰色的按钮，不可以使用，调用这个函数之后就可以使用
	//---------
	ui.actionsaveAs3MF->setEnabled(true);
	ui.actionFragment->setEnabled(true);
	ui.actionsliceSupportColor->setEnabled(true);
	ui.actionsliceSurfaceColor->setEnabled(true);
	ui.actionsinglePic->setEnabled(true);
}

void QtGuiApplication1::setDockWidget_data(varray<NurbsVol> vol)
{

	//用来设置显示在左侧的数据
	QList<QTreeWidgetItem* > items;
	for (int i = 0; i != vol.size(); ++i)
	{
		//创建一个根目录
		QTreeWidgetItem * item = new QTreeWidgetItem(ui.treeWidget);
		item->setText(0, QString::fromLocal8Bit("第") + QString::number(i) + QString::fromLocal8Bit("片"));

		//创建根目录下面的子目录
		QList<QTreeWidgetItem *> childlist;
		//放入各方向控制点数量
		QTreeWidgetItem * m_uNum = new QTreeWidgetItem(QStringList() << "m_uNum" << QString::number(vol[i].m_uNum));
		QTreeWidgetItem * m_vNum = new QTreeWidgetItem(QStringList() << "m_vNum" << QString::number(vol[i].m_vNum));
		QTreeWidgetItem * m_wNum = new QTreeWidgetItem(QStringList() << "m_wNum" << QString::number(vol[i].m_wNum));
		//放入各方向阶数
		QTreeWidgetItem * m_uDegree = new QTreeWidgetItem(QStringList() << "m_uDegree" << QString::number(vol[i].m_uDegree));
		QTreeWidgetItem * m_vDegree = new QTreeWidgetItem(QStringList() << "m_vDegree" << QString::number(vol[i].m_vDegree));
		QTreeWidgetItem * m_wDegree = new QTreeWidgetItem(QStringList() << "m_wDegree" << QString::number(vol[i].m_wDegree));

		//放入节点矢量
		QTreeWidgetItem * m_uKnots = new QTreeWidgetItem(item);
		QTreeWidgetItem * m_vKnots = new QTreeWidgetItem(item);
		QTreeWidgetItem * m_wKnots = new QTreeWidgetItem(item);

		m_uKnots->setText(0, "m_uKnots");
		m_uKnots->setText(1, QString::number(vol[i].m_uKnots.size()));
		m_vKnots->setText(0, "m_vKnots");
		m_vKnots->setText(1, QString::number(vol[i].m_vKnots.size()));
		m_wKnots->setText(0, "m_wKnots");
		m_wKnots->setText(1, QString::number(vol[i].m_wKnots.size()));

		//加入u节点矢量
		for (int j = 0; j != vol[i].m_uKnots.size(); ++j)
		{
			QTreeWidgetItem * mm = new QTreeWidgetItem(m_uKnots);
			mm->setText(0, QString::number(j));
			mm->setText(1, QString::number(vol[i].m_uKnots[j]));
		}
		//加入v节点矢量
		for (int j = 0; j != vol[i].m_vKnots.size(); ++j)
		{
			QTreeWidgetItem * mm = new QTreeWidgetItem(m_vKnots);
			mm->setText(0, QString::number(j));
			mm->setText(1, QString::number(vol[i].m_vKnots[j]));
		}
		//加入w节点矢量
		for (int j = 0; j != vol[i].m_wKnots.size(); ++j)
		{
			QTreeWidgetItem * mm = new QTreeWidgetItem(m_wKnots);
			mm->setText(0, QString::number(j));
			mm->setText(1, QString::number(vol[i].m_wKnots[j]));
		}

		//加入控制点
		QTreeWidgetItem * contralPoints = new QTreeWidgetItem(item);
		contralPoints->setText(0, "m_CtrlPts");
		contralPoints->setText(1, QString::number(vol[i].m_CtrlPts.size()));
		for (int j = 0; j != vol[i].m_CtrlPts.size(); ++j)
		{
			QTreeWidgetItem * aa = new QTreeWidgetItem(contralPoints);
			aa->setText(0, QString::number(j));
			//加入各个点的xyz坐标值
			QTreeWidgetItem *x = new QTreeWidgetItem(aa);
			x->setText(0, "x");
			x->setText(1, QString::number(vol[i].m_CtrlPts[j].x));

			QTreeWidgetItem *y = new QTreeWidgetItem(aa);
			y->setText(0, "y");
			y->setText(1, QString::number(vol[i].m_CtrlPts[j].y));

			QTreeWidgetItem *z = new QTreeWidgetItem(aa);
			z->setText(0, "z");
			z->setText(1, QString::number(vol[i].m_CtrlPts[j].z));
		}

		childlist.push_back(m_uNum);
		childlist.push_back(m_vNum);
		childlist.push_back(m_wNum);
		childlist.push_back(m_uDegree);
		childlist.push_back(m_vDegree);
		childlist.push_back(m_wDegree);
		childlist.push_back(m_uKnots);
		childlist.push_back(m_vKnots);
		childlist.push_back(m_wKnots);
		childlist.push_back(contralPoints);

		item->addChildren(childlist);

		//把创建好的一个根目录放进队列中
		items.push_back(item);
	}
	ui.treeWidget->addTopLevelItems(items);
}

void QtGuiApplication1::setDockWidget_data(varray<NurbsSurface> vol)
{
	//用来设置显示在左侧的数据
	QList<QTreeWidgetItem* > items;
	for (int i = 0; i != vol.size(); ++i)
	{
		//创建一个根目录
		QTreeWidgetItem * item = new QTreeWidgetItem(ui.treeWidget);
		item->setText(0, QString::fromLocal8Bit("第") + QString::number(i) + QString::fromLocal8Bit("片"));

		//创建根目录下面的子目录
		QList<QTreeWidgetItem *> childlist;
		//放入各方向控制点数量
		QTreeWidgetItem * m_uNum = new QTreeWidgetItem(QStringList() << "m_uNum" << QString::number(vol[i].m_uNum));
		QTreeWidgetItem * m_vNum = new QTreeWidgetItem(QStringList() << "m_vNum" << QString::number(vol[i].m_vNum));
		//放入各方向阶数
		QTreeWidgetItem * m_uDegree = new QTreeWidgetItem(QStringList() << "m_uDegree" << QString::number(vol[i].m_uDegree));
		QTreeWidgetItem * m_vDegree = new QTreeWidgetItem(QStringList() << "m_vDegree" << QString::number(vol[i].m_vDegree));

		//放入节点矢量
		QTreeWidgetItem * m_uKnots = new QTreeWidgetItem(item);
		QTreeWidgetItem * m_vKnots = new QTreeWidgetItem(item);

		m_uKnots->setText(0, "m_uKnots");
		m_uKnots->setText(1, QString::number(vol[i].m_uKnots.size()));
		m_vKnots->setText(0, "m_vKnots");
		m_vKnots->setText(1, QString::number(vol[i].m_vKnots.size()));

		//加入u节点矢量
		for (int j = 0; j != vol[i].m_uKnots.size(); ++j)
		{
			QTreeWidgetItem * mm = new QTreeWidgetItem(m_uKnots);
			mm->setText(0, QString::number(j));
			mm->setText(1, QString::number(vol[i].m_uKnots[j]));
		}
		//加入v节点矢量
		for (int j = 0; j != vol[i].m_vKnots.size(); ++j)
		{
			QTreeWidgetItem * mm = new QTreeWidgetItem(m_vKnots);
			mm->setText(0, QString::number(j));
			mm->setText(1, QString::number(vol[i].m_vKnots[j]));
		}

		//加入控制点
		QTreeWidgetItem * contralPoints = new QTreeWidgetItem(item);
		contralPoints->setText(0, "m_CtrlPts");
		contralPoints->setText(1, QString::number(vol[i].m_CtrlPts.size()));
		for (int j = 0; j != vol[i].m_CtrlPts.size(); ++j)
		{
			QTreeWidgetItem * aa = new QTreeWidgetItem(contralPoints);
			aa->setText(0, QString::number(j));
			//加入各个点的xyz坐标值
			QTreeWidgetItem *x = new QTreeWidgetItem(aa);
			x->setText(0, "x");
			x->setText(1, QString::number(vol[i].m_CtrlPts[j].x));

			QTreeWidgetItem *y = new QTreeWidgetItem(aa);
			y->setText(0, "y");
			y->setText(1, QString::number(vol[i].m_CtrlPts[j].y));

			QTreeWidgetItem *z = new QTreeWidgetItem(aa);
			z->setText(0, "z");
			z->setText(1, QString::number(vol[i].m_CtrlPts[j].z));
		}

		childlist.push_back(m_uNum);
		childlist.push_back(m_vNum);
		childlist.push_back(m_uDegree);
		childlist.push_back(m_vDegree);
		childlist.push_back(m_uKnots);
		childlist.push_back(m_vKnots);
		childlist.push_back(contralPoints);

		item->addChildren(childlist);

		//把创建好的一个根目录放进队列中
		items.push_back(item);
	}
	ui.treeWidget->addTopLevelItems(items);
}

void QtGuiApplication1::setDockWidget_data(varray<NurbsLine> vol)
{
	//用来设置显示在左侧的数据
	QList<QTreeWidgetItem* > items;
	for (int i = 0; i != vol.size(); ++i)
	{
		//创建一个根目录
		QTreeWidgetItem * item = new QTreeWidgetItem(ui.treeWidget);
		item->setText(0, QString::fromLocal8Bit("第") + QString::number(i) + QString::fromLocal8Bit("片"));

		//创建根目录下面的子目录
		QList<QTreeWidgetItem *> childlist;
		//放入各方向阶数
		QTreeWidgetItem * m_uDegree = new QTreeWidgetItem(QStringList() << "m_uDegree" << QString::number(vol[i].m_Degree));

		//放入节点矢量
		QTreeWidgetItem * m_uKnots = new QTreeWidgetItem(item);

		m_uKnots->setText(0, "m_uKnots");
		m_uKnots->setText(1, QString::number(vol[i].m_Knots.size()));

		//加入u节点矢量
		for (int j = 0; j != vol[i].m_Knots.size(); ++j)
		{
			QTreeWidgetItem * mm = new QTreeWidgetItem(m_uKnots);
			mm->setText(0, QString::number(j));
			mm->setText(1, QString::number(vol[i].m_Knots[j]));
		}

		//加入控制点
		QTreeWidgetItem * contralPoints = new QTreeWidgetItem(item);
		contralPoints->setText(0, "m_CtrlPts");
		contralPoints->setText(1, QString::number(vol[i].m_CtrlPts.size()));
		for (int j = 0; j != vol[i].m_CtrlPts.size(); ++j)
		{
			QTreeWidgetItem * aa = new QTreeWidgetItem(contralPoints);
			aa->setText(0, QString::number(j));
			//加入各个点的xyz坐标值
			QTreeWidgetItem *x = new QTreeWidgetItem(aa);
			x->setText(0, "x");
			x->setText(1, QString::number(vol[i].m_CtrlPts[j].x));

			QTreeWidgetItem *y = new QTreeWidgetItem(aa);
			y->setText(0, "y");
			y->setText(1, QString::number(vol[i].m_CtrlPts[j].y));

			QTreeWidgetItem *z = new QTreeWidgetItem(aa);
			z->setText(0, "z");
			z->setText(1, QString::number(vol[i].m_CtrlPts[j].z));
		}

		childlist.push_back(m_uDegree);
		childlist.push_back(m_uKnots);
		childlist.push_back(contralPoints);

		item->addChildren(childlist);

		//把创建好的一个根目录放进队列中
		items.push_back(item);
	}
	ui.treeWidget->addTopLevelItems(items);
}

void QtGuiApplication1::slotTest(QTreeWidgetItem * in1, int in2)
{
	MyDoc::Ptr pdoc = MyDoc::getInstance();
	QString text = in1->text(in2);//获得所选的内容
	QRegExp temp(QString::fromLocal8Bit("第"));
	if (text.contains(temp))//查看是否匹配单个显示体模型
	{
		QRegularExpression exp("\\d{1,2}");
		QRegularExpressionMatch match = exp.match(text);
		if (match.hasMatch())
		{
			QString matched = match.captured();
			pdoc->m_CellIdx = matched.toInt();
		}
	}
	if (in1->parent())
	{
		//如果点击的是控制点中的xyz任意一个坐标轴的值
		if (in1->parent()->parent())
		{
			//修改xyz坐标轴的值
			ui.treeWidget->openPersistentEditor(in1, 1);
		}
	}
}

void QtGuiApplication1::slotChanged(QTreeWidgetItem * in1, int in2)
{
	if (option.isLcokOn())
	{
		//获取改变的值
		QString a = in1->text(in2);
		ui.treeWidget->closePersistentEditor(in1, in2);
		MyDoc::Ptr pdoc = MyDoc::getInstance();
		pdoc->clearScrn();
		option.m_again = 1;
		//控制点改变为刚才修改的值
		//------------
		//找到爷爷节点，也就是第几个体
		QString matched;
		QRegularExpression exp("\\d{1,2}");
		QRegularExpressionMatch match = exp.match(in1->parent()->parent()->parent()->text(0));
		if (match.hasMatch())
		{
			matched = match.captured();
		}
		//找到是第几个节点
		QString temp = in1->parent()->text(0);
		//找到具体位置
		if (in1->text(0) == "x")
		{
			pdoc->m_ShowData4D[matched.toInt()][temp.toInt()].x = a.toFloat();
		}
		else		if (in1->text(0) == "y")
		{
			pdoc->m_ShowData4D[matched.toInt()][temp.toInt()].y = a.toFloat();
		}
		else		if (in1->text(0) == "z")
		{
			pdoc->m_ShowData4D[matched.toInt()][temp.toInt()].z = a.toFloat();
		}
		//重新画出图形
		QOpenGLContext * openglContext = ui.centralWidget->context();//获得渲染环境
		option.OnBnClickedCreateModel(openglContext, this);
		option.createCP(openglContext);//进行显示计算
	}
}

