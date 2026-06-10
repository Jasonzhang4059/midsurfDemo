#include "ReducerDialog.h"
#include "FeatureNetwork.h"
#include <qmessagebox.h>

ReducerDialog::ReducerDialog(QWidget *parent)
	: QDialog(parent)
{
	ui.setupUi(this);
	ui.lineEdit_path->setEnabled(false);
	
	connect(ui.zeroparams, &QCheckBox::clicked, [=]() {
		paramsOthrity = ui.zeroparams->isChecked();
		ui.lineEdit_box_l->setEnabled(!paramsOthrity);
		ui.lineEdit_box_t->setEnabled(!paramsOthrity);
		ui.lineEdit_gear_r1->setEnabled(!paramsOthrity);
		ui.lineEdit_gear_r2->setEnabled(!paramsOthrity);
		ui.lineEdit_path->setEnabled(!paramsOthrity);
		ui.lineEdit_shaft_h->setEnabled(!paramsOthrity);
		ui.lineEdit_shaft_r1->setEnabled(!paramsOthrity);
		ui.lineEdit_shaft_r2->setEnabled(!paramsOthrity);
		ui.lineEdit_shaft_r3->setEnabled(!paramsOthrity);
		ui.lineEdit_shaft_l1->setEnabled(!paramsOthrity);
		ui.lineEdit_shaft_l2->setEnabled(!paramsOthrity);
		ui.lineEdit_shaft_r3->setEnabled(!paramsOthrity);
		ui.lineEdit_shaft_t->setEnabled(!paramsOthrity);

	});

	connect(ui.outputmodel, &QCheckBox::clicked, [=]() {
		output = ui.outputmodel->isChecked();
		ui.lineEdit_path->setEnabled(output);
	});
	
	connect(ui.button_ok, &QPushButton::clicked, [=]() {
		if (output) {
			path = ui.lineEdit_path->text();
		}

		if (paramsOthrity) {
			Reducer* r = new Reducer();
			reducer = r->getVols();
			delete r;
			r = nullptr;
		}
		else {
			double gear_r1 = ui.lineEdit_gear_r1->text().toDouble();
			double gear_r2 = ui.lineEdit_gear_r2->text().toDouble();
			double shaft_r1 = ui.lineEdit_shaft_r1->text().toDouble();
			double shaft_r2 = ui.lineEdit_shaft_l2->text().toDouble();
			double shaft_r3 = ui.lineEdit_shaft_r3->text().toDouble();
			double shaft_l1 = ui.lineEdit_shaft_l1->text().toDouble();
			double shaft_l2 = ui.lineEdit_shaft_l2->text().toDouble();
			double shaft_h = ui.lineEdit_shaft_h->text().toDouble();
			double shaft_t = ui.lineEdit_shaft_t->text().toDouble();
			double box_t = ui.lineEdit_box_t->text().toDouble();
			double box_l = ui.lineEdit_box_l->text().toDouble();
			if (gear_r1 <= 0 || gear_r2 <= 0 || shaft_r1 <= 0 || shaft_r2 <= 0 || shaft_r3 <= 0 ||
				shaft_l1 <= 0 || shaft_l2 <= 0 || shaft_h <= 0 || shaft_t <= 0 || box_t <= 0 || box_l <= 0) {
				QMessageBox box(QMessageBox::Warning, QString::fromLocal8Bit("ERROR"), QString::fromLocal8Bit("parameters error"));
				box.setStandardButtons(QMessageBox::Ok);
				box.setButtonText(QMessageBox::Ok, QString::fromLocal8Bit("È· ¶¨"));
				box.exec();
			}

			Reducer* r = new Reducer(gear_r1, gear_r2, shaft_r1, shaft_r2, shaft_r3, shaft_l1, shaft_l2,
				shaft_h, box_l, box_t, shaft_t);
			reducer = r->getVols();
			delete r;
			r = nullptr;
		}
		this->close();
	});
	


}

ReducerDialog::~ReducerDialog()
{
}
