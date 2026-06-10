#ifndef OCCMIDSURFWIDGET_H
#define OCCMIDSURFWIDGET_H

#include <QWidget>

class QPushButton;
class QLineEdit;

class OccMidsurfWidget : public QWidget
{
    Q_OBJECT

public:
    explicit OccMidsurfWidget(QWidget* parent = nullptr);

signals:
    // ◊¢“‚≤Œ ˝À≥–Ú∆•≈‰ OccWin
    void generateMidSurface(double maxThickness, double minThickness);

private slots:
    void onGenerateClicked();

private:
    QPushButton* groupButton;
    QLineEdit* minThicknessEdit;
    QLineEdit* maxThicknessEdit;
    QPushButton* generateButton;
};

#endif // OCCMIDSURFWIDGET_H