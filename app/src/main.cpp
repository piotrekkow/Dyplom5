#include <QApplication>
#include "root/application.hpp"

#if defined(_WIN32)
#include <cstdio>
#include <windows.h>
#include <iostream>

inline void enableUtf8AndAnsi() {
    // set output CP to UTF-8
    SetConsoleOutputCP(CP_UTF8);

    // enable virtual terminal processing for ANSI escapes
    HANDLE hOut = GetStdHandle(STD_OUTPUT_HANDLE);
    if (hOut == INVALID_HANDLE_VALUE) return;

    DWORD mode = 0;
    if (!GetConsoleMode(hOut, &mode)) return;
    mode |= ENABLE_VIRTUAL_TERMINAL_PROCESSING;
    SetConsoleMode(hOut, mode);
}
#endif

int main(int argc, char* argv[]) {
    #if defined(_WIN32)
    enableUtf8AndAnsi();
    #endif
    QApplication qt(argc, argv);
    Application app;
    
    app.show();

    return qt.exec();
}