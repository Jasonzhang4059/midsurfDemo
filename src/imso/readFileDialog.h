#pragma once

#include <QDialog>
#include "ui_readFileDialog.h"

class readFileDialog : public QDialog
{
	Q_OBJECT

public:
	readFileDialog(QWidget *parent = Q_NULLPTR);
	~readFileDialog();

private:
	Ui::readFileDialog ui;
};
