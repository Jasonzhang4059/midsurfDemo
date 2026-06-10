#pragma once

#include <QDialog>
#include "ui_sensitiveSettingDialog.h"

class sensitiveSettingDialog : public QDialog
{
	Q_OBJECT

public:
	sensitiveSettingDialog(QWidget *parent = Q_NULLPTR);
	~sensitiveSettingDialog();

private:
	Ui::sensitiveSettingDialog ui;
};
