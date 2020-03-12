TEMPLATE = app
TARGET   = simpleViewer

HEADERS  = \
    Triangle.h \
    Vec3D.h \
    controlpoint.h \
    curve.h \
    mainwindow.h \
    mesh.h \
    meshreader.h \
    standardcamera.h \
    viewer.h
SOURCES  = main.cpp \
    controlpoint.cpp \
    curve.cpp \
    mainwindow.cpp \
    mesh.cpp \
    standardcamera.cpp \
    viewer.cpp

include( ../baseInclude.pri )
INCLUDEPATH += "..\eigen-3.3.7\Eigen"
INCLUDEPATH += "..\nanoflann\include"
