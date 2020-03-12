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
    void initFileMenu();
    void initToolBars();
    void initFileActions();
    void initDisplayDockWidgets();
    void writeJSON(QJsonObject &json) const;

private Q_SLOTS:
    void openMesh();
    void saveMesh();
    void saveJSON();

private:
    void openBaseMesh();
    bool isBase;
    Viewer *view;
    QActionGroup *fileActionGroup;
};

#endif // MAINWINDOW_H
