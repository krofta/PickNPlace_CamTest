#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QGraphicsScene>

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

#define QVGA_RGB565 (320*240*2)

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

private:
    Ui::MainWindow *ui;
    QGraphicsScene *scene;
    QPixmap *pxmap;
    QImage image = QImage(320,240, QImage::Format::Format_RGB16);
    unsigned char buffer[QVGA_RGB565];
};
#endif // MAINWINDOW_H
