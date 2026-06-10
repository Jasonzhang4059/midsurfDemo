#pragma once

#include <QDialog>
#include "ui_GearBoxdialog.h"
#include "SplineVolume.h"

class GearBoxdialog : public QDialog
{
	Q_OBJECT

public:
	GearBoxdialog(QWidget *parent = Q_NULLPTR);
	~GearBoxdialog();
	varray<SplineVolume> getGearBox() { return GearBox; }
	QString getPath() { return Path; }
	bool isOutput() { return output; }

private:
	bool output = 0;
	QString Path;
	varray<SplineVolume> GearBox;
	bool paramsOthrity = 0;
	Ui::GearBoxdialog ui;
};
