#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include "viewer.h"

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = 0);
    ~MainWindow();

private:
    // Main viewers
    Viewer *view;

    // File menu
    QActionGroup *fileActionGroup;
    void initFileMenu();
    void initToolBars();
    void initFileActions();

private Q_SLOTS:
    void openMesh();
};

#endif // MAINWINDOW_H
