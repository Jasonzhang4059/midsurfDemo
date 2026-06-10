#pragma once

#include <QDialog>
#include "ui_ChangeFineScaleDialog.h"

class ChangeFineScaleDialog : public QDialog
{
	Q_OBJECT

public:
	ChangeFineScaleDialog(QWidget *parent = Q_NULLPTR);
	~ChangeFineScaleDialog();

private:
	Ui::ChangeFineScaleDialog ui;
};
