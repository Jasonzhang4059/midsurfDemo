#pragma once

#include <QWidget>
#include "ui_singlePic.h"

class singlePic : public QWidget
{
	Q_OBJECT

public:
	singlePic(QWidget *parent = Q_NULLPTR);
	~singlePic();

private:
	Ui::singlePic ui;
};
