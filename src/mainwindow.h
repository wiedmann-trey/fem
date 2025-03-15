#pragma once

#include <QMainWindow>
#include "glwidget.h"

class MainWindow : public QWidget
{
    Q_OBJECT

public:
    MainWindow(QString config);
    ~MainWindow();

private:

    GLWidget *glWidget;
};
