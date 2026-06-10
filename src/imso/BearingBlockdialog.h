#pragma once

#include <QDialog>
#include "ui_BearingBlockdialog.h"
#include "SplineVolume.h"

class BearingBlockdialog : public QDialog
{
	Q_OBJECT

public:
	BearingBlockdialog(QWidget *parent = Q_NULLPTR);
	~BearingBlockdialog();
	varray<SplineVolume> getBearingBlock() { return BearingBlock; }
	QString getPath() { return Path; }
	bool isOutput() { return output; }

private:
	bool output = 0;
	QString Path;
	varray<SplineVolume> BearingBlock;
	bool paramsOthrity = 0;
	Ui::BearingBlockdialog ui;
};
