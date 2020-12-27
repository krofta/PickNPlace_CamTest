#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QImage>

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    this->image.loadFromData(&buffer, sizeof(buffer), QImage::Format_RGB16);
    this->pxmap = new QPixmap();
    this->pxmap->fromImage(this->image);
    this->scene = new QGraphicsScene(this);
    this->scene->addPixmap(this->pxmap);
    this->ui->graphicsView->setScene(this->scene);
    ui->setupUi(this);
}

MainWindow::~MainWindow()
{
    delete ui;
}

