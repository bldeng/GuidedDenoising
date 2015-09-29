#-------------------------------------------------
#
# Project created by QtCreator 2015-03-31T15:50:41
#
#-------------------------------------------------

QT       += core gui opengl

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = MeshDenoising
TEMPLATE = app

DEFINES += _USE_MATH_DEFINES

SOURCES += main.cpp\
        mainwindow.cpp \
    glviewer.cpp \
    glexaminer.cpp \
    meshexaminer.cpp \
    datamanager.cpp \
    parameterset.cpp \
    parametersetwidget.cpp \
    calculationthread.cpp \
    Algorithms/Noise.cpp \
    Algorithms/BilateralMeshDenoising.cpp \
    Algorithms/NonIterativeFeaturePreservingMeshFiltering.cpp \
    Algorithms/FastAndEffectiveFeaturePreservingMeshDenoising.cpp \
    Algorithms/BilateralNormalFilteringForMeshDenoising.cpp \
    Algorithms/MeshDenoisingViaL0Minimization.cpp \
    Algorithms/MeshDenoisingBase.cpp \
    iothread.cpp \
    Algorithms/GuidedMeshNormalFiltering.cpp

HEADERS  += mainwindow.h \
    glviewer.h \
    glwrapper.h \
    glexaminer.h \
    meshexaminer.h \
    mesh.h \
    datamanager.h \
    parameterset.h \
    parametersetwidget.h \
    calculationthread.h \
    Algorithms/Noise.h \
    Algorithms/BilateralMeshDenoising.h \
    Algorithms/NonIterativeFeaturePreservingMeshFiltering.h \
    Algorithms/FastAndEffectiveFeaturePreservingMeshDenoising.h \
    Algorithms/BilateralNormalFilteringForMeshDenoising.h \
    Algorithms/MeshDenoisingViaL0Minimization.h \
    Algorithms/MeshDenoisingBase.h \
    iothread.h \
    Algorithms/GuidedMeshNormalFiltering.h

FORMS    += mainwindow.ui

include(../LibsInclude.pri)

RESOURCES += \
    mainwindow.qrc
