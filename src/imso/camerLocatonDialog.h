#pragma once

#include <QDialog>
#include "ui_camerLocatonDialog.h"

class camerLocatonDialog : public QDialog
{
	Q_OBJECT

public:
	camerLocatonDialog(QWidget *parent = Q_NULLPTR);
	~camerLocatonDialog();

private:
	Ui::camerLocatonDialog ui;
};
