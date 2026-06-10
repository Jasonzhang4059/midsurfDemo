#pragma once

#include <QDialog>
#include "ui_fragmentDialog.h"

class fragmentDialog : public QDialog
{
	Q_OBJECT

public:
	fragmentDialog(QWidget *parent = Q_NULLPTR);
	~fragmentDialog();

private:
	Ui::fragmentDialog ui;
};
