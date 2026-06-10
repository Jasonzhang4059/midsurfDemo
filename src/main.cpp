#include "OccDemo.h"
#include <QtWidgets/QApplication>
//#include "PublicModels.h"
#include <QDir>

int main(int argc, char *argv[])
{
   /* PublicSolution p;
    p.quadSurface();*/
    qputenv("QT_QPA_PLATFORM_PLUGIN_PATH", "D:/QT/5.15/5.15.2/msvc2019_64/plugins/platforms");
    QApplication a(argc, argv);
    OccDemo w;
    w.show();
    return a.exec();
}
