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
    void writeJSON(QJsonObject &json);
    void openFile();

private Q_SLOTS:
    void openMesh();
    void openFibMesh();
    void saveJSON();

private:
    void openBaseMesh();
    void openFibulaBase();
    bool isBase;
    Viewer *view;
    QActionGroup *fileActionGroup;
    QString meshFileName;
};

#endif // MAINWINDOW_H
