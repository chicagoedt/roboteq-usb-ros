TEMPLATE = app
CONFIG += console
CONFIG -= app_bundle
CONFIG -= qt

SOURCES += main.cpp \
    serialNetPort.cpp \
    serialPort.cpp

include(deployment.pri)
qtcAddDeployment()

INCLUDEPATH += $$PWD/../../include/

HEADERS += \
    serialException.h \
    ../../include/serialLogger.h \
    ../../include/serialNetPort.h \
    ../../include/serialPort.h
