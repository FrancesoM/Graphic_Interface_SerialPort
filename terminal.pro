QT += widgets serialport 3dcore 3drender 3dinput 3dextras

greaterThan(QT_MAJOR_VERSION, 4): QT += printsupport

TARGET = terminal
TEMPLATE = app
DESTDIR = $$(PWD)

SOURCES += \
    main.cpp \
    mainwindow.cpp \
    settingsdialog.cpp \
    console.cpp \
    qcustomplot.cpp \
    classifier.cpp

HEADERS += \
    mainwindow.h \
    settingsdialog.h \
    console.h \
    qcustomplot.h \
    classifier.h

FORMS += \
    mainwindow.ui \
    settingsdialog.ui \
    classifier.ui

RESOURCES += \
    terminal.qrc

target.path = DESTDIR
INSTALLS += target

DISTFILES +=
