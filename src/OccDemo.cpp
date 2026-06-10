#include "OccDemo.h"
#include <QFileDialog>
#include <TopoDS_Shape.hxx>
#include"occIoCad.h"
OccDemo::OccDemo(QWidget *parent)
    : QMainWindow(parent)
{
    ui.setupUi(this);
	this->installEventFilter(this);
	setExtraUiSetting();
	//m_midsurfWidget = new OccMidsurfWidget(this);
	//m_midsurfWidget->hide();
}

void OccDemo::setExtraUiSetting()
{
	connect(ui.midsurfPushButton, &QPushButton::clicked,
		this, &OccDemo::onMidsurfButtonClicked);
	connect(ui.quaddivisionPushButton, &QPushButton::clicked,
		ui.occWidget, &OccWin::GenerateQuadDivision);
	//connect(ui.midsurfPushButton, &QPushButton::clicked, ui.occWidget, &OccWin::GenerateMidSurface);
}
void OccDemo::onMidsurfButtonClicked()
{
	m_midsurfWidget = new OccMidsurfWidget(); // 独立窗口
	m_midsurfWidget->show();

	connect(m_midsurfWidget, &OccMidsurfWidget::generateMidSurface,
		ui.occWidget, &OccWin::GenerateMidSurface);
}
bool OccDemo::eventFilter(QObject *obj, QEvent *event)
{
	if (obj == this)  // 监听 QMainWindow 自己
	{
		ui.occWidget->getView()->Redraw();
	}
	return QMainWindow::eventFilter(obj, event);
}
OccDemo::~OccDemo()
{}

void OccDemo::on_actionReadFile_triggered() {
	// 1. 设置文件过滤器
	QString filter = tr("STEP Files (*.stp *.step);;All Files (*.*)");

	// 2. 打开文件对话框
	QString stepFile = QFileDialog::getOpenFileName(
		this,                                   // 父窗口
		tr("Open STEP File"),                   // 对话框标题
		QDir::homePath(),                       // 默认路径（用户目录）
		filter                                  // 文件过滤器
	);

	// 3. 检查是否选择了文件
	if (stepFile.isEmpty()) {
		return; // 用户取消选择
	}
	TopoDS_Shape shape;
	bool suc = ReadStepFile(stepFile.toUtf8().constData(), shape);
	ExportShapesToSTL({ shape }, "output.stl");
	if (suc)
	{
		ui.occWidget->DisplayShape(shape);
	}
	else
	{
		ui.statusBar->showMessage(tr("Failed to read STEP file!"), 5000);
		return;
	}

	ui.statusBar->showMessage(tr("Loaded: ") + stepFile, 3000);
}

