#include "FeatureNtkDialog.h"

FeatureNtkDialog::FeatureNtkDialog(QWidget *parent)
	: QDialog(parent)
{
	ui.setupUi(this);
	connect(ui.button_select, &QPushButton::clicked, [=]() {
		QString text = ui.select_shapes->currentText();

		QTextCursor textCursor = ui.text_input->textCursor();//得到当前的光标

		textCursor.movePosition(QTextCursor::End);

		if (textCursor.hasSelection())//如果有选中，则取消，以免受受影响
			textCursor.clearSelection();
		textCursor.deletePreviousChar();//删除前一个字符
		int cur_mode = -1;
		ui.text_input->setTextCursor(textCursor);
		if (text == QString::fromLocal8Bit("正方型")) {
			cur_mode = 1;
			QString context;
			context = "RecTangle (double L0,double H0,double x,double y,double z,double alph)";
			ui.text_output->setText(context);
		}
		
		connect(ui.button_generate, &QPushButton::clicked, [=]() {
			mode = cur_mode;
			this->close();
		});



	});


}

FeatureNtkDialog::~FeatureNtkDialog()
{
}
