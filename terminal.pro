QT += widgets serialport

greaterThan(QT_MAJOR_VERSION, 4): QT += printsupport

TARGET = terminal
TEMPLATE = app
DESTDIR = $$(PWD)

SOURCES += \
    main.cpp \
    mainwindow.cpp \
    settingsdialog.cpp \
    console.cpp \
    qcustomplot.cpp

HEADERS += \
    mainwindow.h \
    settingsdialog.h \
    console.h \
    qcustomplot.h

FORMS += \
    mainwindow.ui \
    settingsdialog.ui

RESOURCES += \
    terminal.qrc

target.path = DESTDIR
INSTALLS += target
