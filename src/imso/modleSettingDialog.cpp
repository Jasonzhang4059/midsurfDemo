#include "modleSettingDialog.h"
#include "MyDoc.h"
#include <qslider.h>

modleSettingDialog::modleSettingDialog(QWidget *parent)
	: QWidget(parent)
{
	ui.setupUi(this);

	MyDoc::Ptr pdoc = MyDoc::getInstance();

	ui.linWidthScrollBar->setValue((int)pdoc->_lineWidth);
	ui.controlPtsScrollBar->setValue((int)pdoc->m_CPTSdotSZ);
	ui.PtsScrollBar->setValue((int)pdoc->m_dotSZ);

	connect(ui.linWidthScrollBar, &QSlider::valueChanged, [=]() {
		pdoc->_lineWidth = ui.linWidthScrollBar->value();
	});

	connect(ui.controlPtsScrollBar, &QSlider::valueChanged, [=]() {
		pdoc->m_CPTSdotSZ = ui.controlPtsScrollBar->value();
	});

	connect(ui.PtsScrollBar, &QSlider::valueChanged, [=]() {
		pdoc->m_dotSZ = ui.PtsScrollBar->value();
	});
}

modleSettingDialog::~modleSettingDialog()
{
}
