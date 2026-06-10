#include "ChangeFineScaleDialog.h"
#include "MyDoc.h"
#include <qmessagebox.h>
#include <qvalidator.h>

ChangeFineScaleDialog::ChangeFineScaleDialog(QWidget *parent)
	: QDialog(parent)
{
	ui.setupUi(this);

	MyDoc::Ptr pdoc = MyDoc::getInstance();
	ui.U_NUM->setText(QString::number(pdoc->m_Unum));//读取当前的信息
	ui.V_NUM->setText(QString::number(pdoc->m_Vnum));
	ui.W_NUM->setText(QString::number(pdoc->m_Wnum));

	ui.U_NUM->setValidator(new QIntValidator(0, 1000, this));//设置只能输入数字
	ui.V_NUM->setValidator(new QIntValidator(0, 1000, this));
	ui.W_NUM->setValidator(new QIntValidator(0, 1000, this));
	
	connect(ui.conform, &QPushButton::clicked, [=]() {
		
		if (ui.U_NUM->text() == NULL || ui.V_NUM->text() == NULL || ui.W_NUM->text() == NULL)
		{
			QMessageBox box(QMessageBox::Warning, QString::fromLocal8Bit("警告"), QString::fromLocal8Bit("内容不能为空"));
			box.setStandardButtons(QMessageBox::Ok);
			box.setButtonText(QMessageBox::Ok, QString::fromLocal8Bit("确 定"));
			box.exec();
		}
		else
		{
			pdoc->m_Unum = ui.U_NUM->text().toInt();
			pdoc->m_Vnum = ui.V_NUM->text().toInt();
			pdoc->m_Wnum = ui.W_NUM->text().toInt();
			this->close();
			pdoc->clearScrn();
		}
	});

	connect(ui.cancle, &QPushButton::clicked, this, &QWidget::close);
}

ChangeFineScaleDialog::~ChangeFineScaleDialog()
{
}
