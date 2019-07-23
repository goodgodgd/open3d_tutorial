#-------------------------------------------------
#
# Project created by QtCreator 2019-07-23T13:47:23
#
#-------------------------------------------------

QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = examples
TEMPLATE = app

# The following define makes your compiler emit warnings if you use
# any feature of Qt which has been marked as deprecated (the exact warnings
# depend on your compiler). Please consult the documentation of the
# deprecated API in order to know how to port your code away from it.
DEFINES += QT_DEPRECATED_WARNINGS

# You can also make your code fail to compile if you use deprecated APIs.
# In order to do so, uncomment the following line.
# You can also select to disable deprecated APIs only up to a certain version of Qt.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0


INCLUDEPATH += /home/ian/mylib/deploy/open3d/include \
                /home/ian/mylib/deploy/open3d/include/Open3D/3rdparty/fmt/include \
                /home/ian/mylib/deploy/open3d/include/Open3D/3rdparty/Eigen

LIBS += -L/home/ian/mylib/deploy/open3d/lib \
        -lOpen3D


SOURCES += \
        main.cpp \
        mainwindow.cpp \
    open3dexamples.cpp

HEADERS += \
        mainwindow.h \
    open3dexamples.h

FORMS += \
        mainwindow.ui
