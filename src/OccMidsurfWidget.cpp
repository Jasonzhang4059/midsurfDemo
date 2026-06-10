#include "OccMidsurfWidget.h"

#include <QPushButton>
#include <QLineEdit>
#include <QLabel>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QGuiApplication>
#include <QScreen>
#include <QDoubleValidator>

OccMidsurfWidget::OccMidsurfWidget(QWidget* parent)
    : QWidget(parent)
{
    // ===== 设置窗口为独立窗口，有右上角关闭按钮 =====
    setWindowFlags(Qt::Window);
    setFixedSize(600, 400);
    setWindowTitle(u8"中面生成");

    // ===== 第二行：最小厚度 =====
    QLabel* minLabel = new QLabel(u8"最小厚度：");
    minThicknessEdit = new QLineEdit();
    minThicknessEdit->setValidator(new QDoubleValidator(0, 10000, 6, this));

    QHBoxLayout* row1 = new QHBoxLayout();
    row1->addWidget(minLabel);
    row1->addWidget(minThicknessEdit);

    // ===== 第三行：最大厚度 =====
    QLabel* maxLabel = new QLabel(u8"最大厚度：");
    maxThicknessEdit = new QLineEdit();
    maxThicknessEdit->setValidator(new QDoubleValidator(0, 10000, 6, this));

    QHBoxLayout* row2 = new QHBoxLayout();
    row2->addWidget(maxLabel);
    row2->addWidget(maxThicknessEdit);

    // ===== 第四行：生成 + 分组显示按钮 =====
    generateButton = new QPushButton(u8"生成");
    groupButton = new QPushButton(u8"分组显示");

    QHBoxLayout* row3 = new QHBoxLayout();
    row3->addWidget(generateButton);
    row3->addStretch();
    row3->addWidget(groupButton);

    // ===== 总布局 =====
    QVBoxLayout* mainLayout = new QVBoxLayout(this);
    mainLayout->addStretch(1);
    mainLayout->addLayout(row1);
    mainLayout->addStretch(1);
    mainLayout->addLayout(row2);
    mainLayout->addStretch(1);
    mainLayout->addLayout(row3);
    mainLayout->addStretch(1);

    // ===== 居中显示 =====
    QScreen* screen = QGuiApplication::primaryScreen();
    QRect screenGeometry = screen->availableGeometry();
    int x = (screenGeometry.width() - width()) / 2;
    int y = (screenGeometry.height() - height()) / 2;
    move(x, y);

    // ===== 生成按钮信号连接 =====
    connect(generateButton, &QPushButton::clicked,
        this, &OccMidsurfWidget::onGenerateClicked);
}

void OccMidsurfWidget::onGenerateClicked()
{
    double minT = minThicknessEdit->text().toDouble();
    double maxT = maxThicknessEdit->text().toDouble();
    close(); // 自动关闭窗口

    emit generateMidSurface(maxT, minT); // 参数顺序匹配 OccWin
}