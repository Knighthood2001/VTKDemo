#include "VTK930.h"
#include <QtWidgets/QApplication>

int main(int argc, char *argv[])
{
    QApplication app(argc, argv);
    VTK930 window;
    window.show();
    return app.exec();
}
