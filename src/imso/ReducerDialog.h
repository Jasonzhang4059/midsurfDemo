#pragma once

#include <QDialog>
#include "ui_ReducerDialog.h"
#include"SplineVolume.h"

class ReducerDialog : public QDialog
{
	Q_OBJECT

public:
	ReducerDialog(QWidget *parent = Q_NULLPTR);
	~ReducerDialog();
	varray<SplineVolume> getReducer() { return reducer; }
	QString getPath() { return path; }
	bool isOutput() { return output; }
private:
	bool output = 0;
	QString path;
	varray<SplineVolume> reducer;
	bool paramsOthrity = 0;
	Ui::ReducerDialog ui;
};
