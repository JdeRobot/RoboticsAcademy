SOURCES += \
    main.cpp \
    robot/sensors.cpp \
    robot/robot.cpp \
    robot/actuators.cpp \
    robot/threadupdaterobot.cpp \
    gui/gui.cpp \
    gui/threadupdategui.cpp \
    gui/stategui.cpp \
    gui/widget/controlvw.cpp \
    MyAlgorithm.cpp \
    gui/widget/cameraswidget.cpp \
    gui/widget/glwidget.cpp \
    pioneer/pioneeropengl.c \
    pioneer/pioneer.c \
    gui/widget/laserwidget.cpp \
    depuratewindow.cpp

HEADERS += \
    robot/sensors.h \
    robot/robot.h \
    robot/actuators.h \
    robot/threadupdaterobot.h \
    gui/gui.h \
    gui/threadupdategui.h \
    gui/stategui.h \
    gui/widget/controlvw.h \
    gui/widget/cameraswidget.h \
    gui/widget/glwidget.h \
    pioneer/pioneeropengl.h \
    pioneer/pioneer.h \
    gui/widget/laserwidget.h \
    depuratewindow.h



QT           += opengl

#jderobot
INCLUDEPATH +=  /usr/local/include/jderobot  /usr/local/include/gearbox
LIBS += -L/usr/local/lib/jderobot \
        -lbgfgsegmentation  -ljderobotice         -lprogeo \
        -lcolorspacesmm     -lJderobotInterfaces  -lvisionlib \
        -lcolorspaces       -ljderobotutil

#opencv
INCLUDEPATH += /usr/include/opencv
LIBS += -L/usr/lib \
    -lopencv_core \
    -lopencv_highgui \
    -lopencv_imgproc

#ICE
LIBS += -L/usr/lib \
    -lIce -lIceUtil

OTHER_FILES += \
    introrob.cfg

RESOURCES += \
    resources.qrc
