#include "mainwindow.h"
#include <qapplication.h>

/*int main(int argc, char **argv) {
  QApplication application(argc, argv);

  Viewer viewer;

  viewer.setWindowTitle("Iterative Closest Point");

  viewer.show();

  return application.exec();
}*/

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    MainWindow w;
    w.show();

    return a.exec();
}
