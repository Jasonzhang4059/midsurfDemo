#pragma once

#include <QWidget>
#include "ui_modleSettingDialog.h"

class modleSettingDialog : public QWidget
{
	Q_OBJECT

public:
	modleSettingDialog(QWidget *parent = Q_NULLPTR);
	~modleSettingDialog();

private:
	Ui::modleSettingDialog ui;
};
