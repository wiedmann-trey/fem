#include "mainwindow.h"
#include <QHBoxLayout>

MainWindow::MainWindow(QString config)
{
    glWidget = new GLWidget(config);

    QHBoxLayout *container = new QHBoxLayout;
    container->addWidget(glWidget);
    this->setLayout(container);
}

MainWindow::~MainWindow()
{
    delete glWidget;
}
