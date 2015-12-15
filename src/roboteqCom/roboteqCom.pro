TEMPLATE = app
CONFIG += console
CONFIG -= app_bundle
CONFIG -= qt

SOURCES += main.cpp \
    roboteqCom.cpp \
    roboteqThread.cpp \
    ../serialConnector/serialPort.cpp

include(deployment.pri)
qtcAddDeployment()

INCLUDEPATH += $$PWD/../../include/

HEADERS += \
    serialException.h \
    ../../include/serialLogger.h \
    ../../include/serialNetPort.h \
    ../../include/serialPort.h \
    ../../include/roboteqCom.h \
    ../../include/roboteqComEvent.h \
    ../../include/roboteqMutex.h \
    ../../include/roboteqThread.h \
    ../../include/serialException.h

QMAKE_CXXFLAGS += -m64 -std=c++11
QMAKE_CFLAGS += -m64 -std=c++11
