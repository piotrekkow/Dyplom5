#include <QApplication>

#include "root/application.hpp"

int main(int argc, char* argv[]) {
    QApplication qt(argc, argv);

    Application app;
    app.show();

    return qt.exec();
}