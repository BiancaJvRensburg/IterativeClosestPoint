TEMPLATE = app
TARGET   = simpleViewer

HEADERS  = \
    Triangle.h \
    Vec3D.h \
    mainwindow.h \
    mesh.h \
    meshreader.h \
    standardcamera.h \
    viewer.h
SOURCES  = main.cpp \
    mainwindow.cpp \
    mesh.cpp \
    standardcamera.cpp \
    viewer.cpp

include( ../baseInclude.pri )
