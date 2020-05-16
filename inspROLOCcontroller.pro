TEMPLATE = app
CONFIG += c++11
CONFIG += console
QT -= gui
QT += dbus widgets
#QT += core

CONFIG -= app_bundle

TARGET = inspROLOCcontroller
    target.files = inspROLOCcontroller
    target.path = /home/pi
    #policy.path = $${DESTDIR}/etc/dbus-1/system.d
    #policy.files = com.inspectron.inspROLOCcontroller.conf

INSTALLS += target
#policy

HEADERS += \
    inspRolocControllerDbus.hpp \
    rolocController.hpp \
    i2c.hpp


SOURCES += main.cpp \
    inspRolocControllerDbus.cpp \
    rolocController.cpp \
    i2c.cpp

LIBS += -ludev

DBUS_ADAPTORS += com.inspectron.inspROLOCcontroller.xml


# The following define makes your compiler emit warnings if you use
# any feature of Qt which as been marked deprecated (the exact warnings
# depend on your compiler). Please consult the documentation of the
# deprecated API in order to know how to port your code away from it.
DEFINES += QT_DEPRECATED_WARNINGS

# You can also make your code fail to compile if you use deprecated APIs.
# In order to do so, uncomment the following line.
# You can also select to disable deprecated APIs only up to a certain version of Qt.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0


DISTFILES += \
    com.inspectron.inspROLOCcontroller.conf
