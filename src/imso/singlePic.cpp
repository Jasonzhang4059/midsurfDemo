#include "singlePic.h"
#include "MyDoc.h"
#include "qvalidator.h"

singlePic::singlePic(QWidget *parent)
	: QWidget(parent)
{
	ui.setupUi(this);

	MyDoc::Ptr pdoc = MyDoc::getInstance();

	ui.totalPicEdit->setText(QString::number(pdoc->m_tolNum));//读取当前的信息
	ui.currentPicEdit->setText(QString::number(pdoc->m_CellIdx));
	ui.lineEdit->setText(QString::number(pdoc->m_NUMPic));

	ui.lineEdit->setValidator(new QIntValidator(1, 1000, this));//设置只能输入数字
	ui.currentPicEdit->setValidator(new QIntValidator(1, 1000, this));

	connect(ui.closeButton, &QPushButton::clicked, this, &QWidget::close);

	connect(ui.nextPicButton, &QPushButton::clicked, [=]() {
		MyDoc::Ptr pdoc = MyDoc::getInstance();
		++pdoc->m_CellIdx;
		if (pdoc->m_CellIdx >= pdoc->m_tolNum)
			pdoc->m_CellIdx = -1;
		ui.totalPicEdit->setText(QString::number(pdoc->m_tolNum));
		ui.currentPicEdit->setText(QString::number(pdoc->m_CellIdx));
	});

	connect(ui.lastPicButton, &QPushButton::clicked, [=]() {
		MyDoc::Ptr pdoc = MyDoc::getInstance();
		--pdoc->m_CellIdx;
		if (pdoc->m_CellIdx < -1)
			pdoc->m_CellIdx = pdoc->m_tolNum - 1;
		ui.totalPicEdit->setText(QString::number(pdoc->m_tolNum));
		ui.currentPicEdit->setText(QString::number(pdoc->m_CellIdx));
	});
}

singlePic::~singlePic()
{
}
