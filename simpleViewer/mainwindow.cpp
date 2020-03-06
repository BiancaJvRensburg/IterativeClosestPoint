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
    //QString skullFilename = "C:\\Users\\Medmax\\Documents\\Project\\Mand_B.off";
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

    initFileMenu();
    initToolBars();

    this->setWindowTitle("Iterative Closest Point");
}

MainWindow::~MainWindow()
{

}

void MainWindow::initFileActions(){
    fileActionGroup = new QActionGroup(this);

    QAction *openFileAction = new QAction("Open mesh", this);
    connect(openFileAction, &QAction::triggered, this, &MainWindow::openMesh);

    QAction *icpStepAction = new QAction("Registration iteration", this);
    connect(icpStepAction, &QAction::triggered, view, &Viewer::registrationSingleStep);

    QAction *icpAction = new QAction("Registration", this);
    connect(icpAction, &QAction::triggered, view, &Viewer::registration);

   /* QAction *autoRAction = new QAction("Auto align", this);
    connect(autoRAction, &QAction::triggered, view, &Viewer::autoRotate);*/

    QAction *rxAction = new QAction("Rotate X 90°", this);
    connect(rxAction, &QAction::triggered, view, &Viewer::rotateX);

    QAction *ryAction = new QAction("Rotate Y 90°", this);
    connect(ryAction, &QAction::triggered, view, &Viewer::rotateY);

    QAction *rzAction = new QAction("Rotate Z 90°", this);
    connect(rzAction, &QAction::triggered, view, &Viewer::rotateZ);

    fileActionGroup->addAction(openFileAction);
    fileActionGroup->addAction(icpAction);
    fileActionGroup->addAction(icpStepAction);
    //fileActionGroup->addAction(autoRAction);
    fileActionGroup->addAction(rxAction);
    fileActionGroup->addAction(ryAction);
    fileActionGroup->addAction(rzAction);
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

void MainWindow::openMesh(){
    QString openFileNameLabel, selectedFilter;

    QString fileFilter = "OFF (*.off)";

    QString fileName = QFileDialog::getOpenFileName(this, tr("Select a mesh"), openFileNameLabel, fileFilter, &selectedFilter);

    if(fileName.isEmpty()) return;

    view->openOFF(fileName);
}

