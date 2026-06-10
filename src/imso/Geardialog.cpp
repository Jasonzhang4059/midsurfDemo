#include "Geardialog.h"
#include "FeatureNetwork.h"
#include "qmessagebox.h"

Geardialog::Geardialog(QWidget *parent)
	: QDialog(parent)
{
	ui.setupUi(this);
	ui.lineEdit_path->setEnabled(false);
	connect(ui.zeroparams, &QCheckBox::clicked, [=]() {

		paramsOthrity = ui.zeroparams->isChecked();
		ui.lineEdit_alph->setEnabled(!paramsOthrity);
		ui.lineEdit_B->setEnabled(!paramsOthrity);
		ui.lineEdit_cx->setEnabled(!paramsOthrity);
		ui.lineEdit_Dk->setEnabled(!paramsOthrity);
		ui.lineEdit_hax->setEnabled(!paramsOthrity);
		ui.lineEdit_m->setEnabled(!paramsOthrity);
		ui.lineEdit_x->setEnabled(!paramsOthrity);
		ui.lineEdit_z->setEnabled(!paramsOthrity);
	});

	connect(ui.outputmodel, &QCheckBox::clicked, [=]() {
		output = ui.outputmodel->isChecked();
		ui.lineEdit_path->setEnabled(output);

	});

	connect(ui.Button_ok, &QPushButton::clicked, [=]() {
		if (output) {
			Path = ui.lineEdit_path->text();
		}
		if (paramsOthrity) {
			Gear_Straight* g = new Gear_Straight();
			Gear = g->getGear();
			delete g;
			g = nullptr;
		}
		else {
			double m = ui.lineEdit_m->text().toDouble();
			int z = ui.lineEdit_z->text().toInt(); 
			double alph = ui.lineEdit_alph->text().toDouble(); 
			double hax = ui.lineEdit_hax->text().toDouble();
			double cx = ui.lineEdit_cx->text().toDouble(); 
			double B = ui.lineEdit_B->text().toDouble();  
			double x = ui.lineEdit_x->text().toDouble();
			double Dk = ui.lineEdit_Dk->text().toDouble();
			if (m <= 0 || z <= 0 || alph <= 0 || hax < 0 || cx < 0 || B <= 0 || x < 0 || Dk <= 0) {
				QMessageBox box(QMessageBox::Warning, QString::fromLocal8Bit("ERROR"), QString::fromLocal8Bit("parameters error"));
				box.setStandardButtons(QMessageBox::Ok);
				box.setButtonText(QMessageBox::Ok, QString::fromLocal8Bit("È· ¶¨"));
				box.exec();
			}
			else
			{
				Gear_Straight* g = new Gear_Straight(m,z,alph,hax,cx,B,x,Dk);
				Gear = g->getGear();
				delete g;
				g = nullptr;
			}
	

		}
		this->close();
	});

}

Geardialog::~Geardialog()
{

}
