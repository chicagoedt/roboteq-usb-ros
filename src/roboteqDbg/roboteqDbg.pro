TEMPLATE = app
CONFIG += console
CONFIG -= app_bundle
CONFIG -= qt

LIBS += -lncurses

SOURCES += main.cpp \
    mainWindow.cpp\
    roboteqLogger.cpp\
    ../roboteqCom/roboteqCom.cpp\
    ../roboteqCom/roboteqThread.cpp\
    ../serialConnector/serialPort.cpp


include(deployment.pri)
qtcAddDeployment()

INCLUDEPATH += $$PWD/../../include/

HEADERS += \
    ../../include/serialException.h \
    ../../include/serialLogger.h \
    ../../include/serialNetPort.h \
    ../../include/serialPort.h \
    ../../include/roboteqCom.h \
    ../../include/roboteqComEvent.h \
    mainWindow.h \
    roboteqLogger.h

