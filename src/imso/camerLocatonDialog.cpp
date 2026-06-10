#include "camerLocatonDialog.h"
#include "MyDoc.h"
#include <qmessagebox.h>
#include <QKeyEvent>
#include <qvalidator.h>

camerLocatonDialog::camerLocatonDialog(QWidget *parent)
	: QDialog(parent)
{
	ui.setupUi(this);
	MyDoc::Ptr pdoc = MyDoc::getInstance();

	ui.X->setText(QString::number(pdoc->cameraX));//读取当前的信息
	ui.Y->setText(QString::number(pdoc->cameraY));
	ui.Z->setText(QString::number(pdoc->cameraZ));

	ui.X->setValidator(new QIntValidator(0, 10000, this));//限制只能输入数字，且范围是0-10000
	ui.Y->setValidator(new QIntValidator(0, 10000, this));
	ui.Z->setValidator(new QIntValidator(0, 10000, this));

	connect(ui.conform, &QPushButton::clicked, [=]() {
		if (ui.X->text() == NULL || ui.X->text() == NULL || ui.X->text() == NULL)
		{
			QMessageBox box(QMessageBox::Warning, QString::fromLocal8Bit("警告"), QString::fromLocal8Bit("内容不能为空"));
			box.setStandardButtons(QMessageBox::Ok);
			box.setButtonText(QMessageBox::Ok, QString::fromLocal8Bit("确 定"));
			box.exec();
		}
		else
		{
			pdoc->cameraX = ui.X->text().toFloat();
			pdoc->cameraY = ui.Y->text().toFloat();
			pdoc->cameraZ = ui.Z->text().toFloat();
			pdoc->m_restView = true;
			this->close();
		}
	});

	connect(ui.cancle, &QPushButton::clicked, [=]() {
		this->close();
	});
}

camerLocatonDialog::~camerLocatonDialog()
{
}
