#include "sensitiveSettingDialog.h"
#include "MyDoc.h"
#include "qslider.h"
sensitiveSettingDialog::sensitiveSettingDialog(QWidget *parent)
	: QDialog(parent)
{
	ui.setupUi(this);
	MyDoc::Ptr pdoc = MyDoc::getInstance();
	ui.keyBordSpinBox->setValue((int)(pdoc->_keyBordSensitive ));
	ui.keyBordScrollBar->setValue((int)(pdoc->_keyBordSensitive ));

	ui.mouseSpinBox->setValue((int)(pdoc->_mouseSenstive));
	ui.mouseScrollBar->setValue((int)(pdoc->_mouseSenstive));

	void (QSpinBox:: *keyBordsignal)(int) = &QSpinBox::valueChanged;
	connect(ui.keyBordSpinBox, keyBordsignal, ui.keyBordScrollBar, &QSlider::setValue);

	connect(ui.keyBordScrollBar, &QSlider::valueChanged, ui.keyBordSpinBox, &QSpinBox::setValue);

	void (QSpinBox:: *mousesignal)(int) = &QSpinBox::valueChanged;
	connect(ui.mouseSpinBox, mousesignal, ui.mouseScrollBar, &QSlider::setValue);

	connect(ui.mouseScrollBar, &QSlider::valueChanged, ui.mouseSpinBox, &QSpinBox::setValue);

	connect(ui.confirmButton,&QPushButton::clicked,[=](){
		int x=ui.keyBordSpinBox->value();
		pdoc->_keyBordSensitive = (float)x ;
		x = ui.mouseSpinBox->value();
		pdoc->_mouseSenstive = (float)x / 1000;
		this->close();
	});

	connect(ui.cancleButton, &QPushButton::clicked, this, &QWidget::close);
}

sensitiveSettingDialog::~sensitiveSettingDialog()
{
}
