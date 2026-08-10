#include <QApplication>
#include <QTimer>
#include <QWidget>

#include "application.hpp"

int main(int argc, char* argv[]) {
    QApplication qt(argc, argv);

    Application app;
    app.show();

    return qt.exec();
}