#include "mainwindow.h"
#include <QLayout>
#include <QGroupBox>
#include <QDockWidget>
#include <QSlider>
#include <QFormLayout>
#include <QPushButton>

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
{
    if (this->objectName().isEmpty())
        this->setObjectName("window");
    this->resize(929, 891);

    // The qglviewer
    StandardCamera *sc = new StandardCamera();
    view = new Viewer(this, sc);

    // Main widget
    QWidget *mainWidget = new QWidget(this);

    // Horizontal layout
    QHBoxLayout *windowLayout = new QHBoxLayout();

    // Add the viewer to the layout
    windowLayout->addWidget(view);

    // Add the layout to the main widget
    mainWidget->setLayout(windowLayout);

    QGroupBox * viewerGroupBox = new QGroupBox();

    QGridLayout * gridLayout = new QGridLayout(viewerGroupBox);
    gridLayout->setObjectName("gridLayout");

    gridLayout->addWidget(mainWidget, 0, 1, 1, 1);

    viewerGroupBox->setLayout(gridLayout);

    this->setCentralWidget(viewerGroupBox);

    initDisplayDockWidgets();
    initFileMenu();
    initToolBars();

    this->setWindowTitle("Iterative Closest Point");

   // openBaseMesh();
}

MainWindow::~MainWindow()
{

}

void MainWindow::initFileActions(){
    fileActionGroup = new QActionGroup(this);

    QAction *openFileAction = new QAction("Open mand mesh", this);
    connect(openFileAction, &QAction::triggered, this, &MainWindow::openMesh);

    QAction *openFibFileAction = new QAction("Open fib mesh", this);
    connect(openFibFileAction, &QAction::triggered, this, &MainWindow::openFibMesh);

    QAction *saveJsonAction = new QAction("Save json", this);
    connect(saveJsonAction, &QAction::triggered, this, &MainWindow::saveJSON);

    QAction *icpStepAction = new QAction("Registration iteration", this);
    connect(icpStepAction, &QAction::triggered, view, &Viewer::registrationSingleStep);

    QAction *icpAction = new QAction("Registration", this);
    connect(icpAction, &QAction::triggered, view, &Viewer::registration);

    fileActionGroup->addAction(openFileAction);
    fileActionGroup->addAction(openFibFileAction);
    fileActionGroup->addAction(saveJsonAction);
    fileActionGroup->addAction(icpAction);
    fileActionGroup->addAction(icpStepAction);
}

void MainWindow::initFileMenu(){
    initFileActions();

    QMenu *fileMenu = menuBar()->addMenu(tr("File"));
    fileMenu->addActions(fileActionGroup->actions());
}

void MainWindow::initToolBars () {
    QToolBar *fileToolBar = new QToolBar(this);
    fileToolBar->addActions(fileActionGroup->actions());
    addToolBar(fileToolBar);
}

void MainWindow::initDisplayDockWidgets(){
    QDockWidget* dockW = new QDockWidget("Controls");

    QHBoxLayout* layout = new QHBoxLayout();

    // The contents of the dockWidget
    QWidget *contents = new QWidget();
    QFormLayout *contentLayout = new QFormLayout();

    QSlider *baseAlphaSlider = new QSlider(Qt::Horizontal);
    baseAlphaSlider->setMaximum(100);
    baseAlphaSlider->setSliderPosition(50);
    contentLayout->addRow("Base transparency", baseAlphaSlider);

    QSlider *meshAlphaSlider = new QSlider(Qt::Horizontal);
    meshAlphaSlider->setMaximum(100);
    meshAlphaSlider->setSliderPosition(50);
    contentLayout->addRow("Mesh transparency", meshAlphaSlider);

    QSlider *rotateXSlider = new QSlider(Qt::Horizontal);
    rotateXSlider->setMaximum(360);
    contentLayout->addRow("Rotate X", rotateXSlider);

    QSlider *rotateYSlider = new QSlider(Qt::Horizontal);
    rotateYSlider->setMaximum(360);
    contentLayout->addRow("Rotate Y", rotateYSlider);

    QSlider *rotateZSlider = new QSlider(Qt::Horizontal);
    rotateZSlider->setMaximum(360);
    contentLayout->addRow("Rotate Z", rotateZSlider);


    // Connect the skull sliders
    connect(baseAlphaSlider, static_cast<void (QSlider::*)(int)>(&QSlider::sliderMoved), view, &Viewer::setBaseAlpha);
    connect(meshAlphaSlider, static_cast<void (QSlider::*)(int)>(&QSlider::sliderMoved), view, &Viewer::setMeshAlpha);
    connect(rotateXSlider, static_cast<void (QSlider::*)(int)>(&QSlider::sliderMoved), view, &Viewer::rotateX);
    connect(rotateYSlider, static_cast<void (QSlider::*)(int)>(&QSlider::sliderMoved), view, &Viewer::rotateY);
    connect(rotateZSlider, static_cast<void (QSlider::*)(int)>(&QSlider::sliderMoved), view, &Viewer::rotateZ);

    contents->setLayout(contentLayout);

    layout->addWidget(contents);

    QWidget* controlWidget = new QWidget();
    controlWidget->setLayout(layout);

    dockW->setWidget(controlWidget);

    this->addDockWidget(Qt::RightDockWidgetArea, dockW);
}

void MainWindow::openMesh(){
    openFile();
    openBaseMesh();
    view->initCurve(true);
}

void MainWindow::openFibMesh(){
    openFile();
    openFibulaBase();
    view->initCurve(false);
}

void MainWindow::openFile(){
    QString openFileNameLabel, selectedFilter;
    QString fileFilter = "OFF (*.off)";
    meshFileName = QFileDialog::getOpenFileName(this, tr("Select a mesh"), openFileNameLabel, fileFilter, &selectedFilter);
    if(meshFileName.isEmpty()) return;
    view->openOFF(meshFileName, view->getMesh(true), true);
}

void MainWindow::openBaseMesh(){
    QString filename = "C:\\Users\\Medmax\\Documents\\Bianca\\Meshes\\Mandible.off";
    view->openOFF(filename, view->getMesh(false), false);
}

void MainWindow::openFibulaBase(){
    QString filename = "C:\\Users\\Medmax\\Documents\\Bianca\\Meshes\\fibula_ocho.off";
    view->openOFF(filename, view->getMesh(false), false);
}

void MainWindow::writeJSON(QJsonObject &json){
    QJsonObject meshJSON;
    QJsonArray cntrlArray;

    view->writeJSON(cntrlArray);
    json["control points"] = cntrlArray;
    json["mesh file"] = QJsonValue(meshFileName);
}

void MainWindow::saveJSON(){
    QString fileName = QFileDialog::getSaveFileName(this, "Save mesh file as ", "./data/", "JSON (*.json)");
    QFile saveFile(fileName);

    if (!saveFile.open(QIODevice::WriteOnly)) {
        qWarning("Couldn't open save file.");
        return;
    }

    QJsonObject jsonObject;
    writeJSON(jsonObject);
    QJsonDocument saveDoc(jsonObject);
    saveFile.write(saveDoc.toJson());
}
