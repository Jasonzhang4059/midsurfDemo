#pragma once

#include <QtWidgets/QMainWindow>
#include "ui_OccDemo.h"
#include "OccMidsurfWidget.h"

class OccDemo : public QMainWindow
{
    Q_OBJECT

public:
    explicit OccDemo(QWidget *parent = nullptr);
	~OccDemo();


private slots:
	void on_actionReadFile_triggered();
	void onMidsurfButtonClicked();

private:
	void setExtraUiSetting();

protected:
	bool eventFilter(QObject * obj, QEvent * event);
private:
    Ui::OccDemoClass ui;
	OccMidsurfWidget* m_midsurfWidget;
};
