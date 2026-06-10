#pragma once

#include <QDialog>
#include "ui_Geardialog.h"
#include "SplineVolume.h"

class Geardialog : public QDialog
{
	Q_OBJECT

public:
	Geardialog(QWidget *parent = Q_NULLPTR);
	~Geardialog();
	varray<SplineVolume> getGear() { return Gear; }
	QString getPath() { return Path; }
	bool isOutput() { return output;}

private:
	bool output = 0;
	QString Path;
	varray<SplineVolume> Gear;
	bool paramsOthrity = 0;
	Ui::Geardialog ui;
};
