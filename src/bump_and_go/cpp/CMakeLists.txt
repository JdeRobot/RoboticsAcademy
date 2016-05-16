    cmake_minimum_required(VERSION 2.8)

if (${QT_COMPILE})

    SET(qt_SOURCES main.cpp depuratewindow.cpp MyAlgorithm.cpp
				   robot/actuators.cpp robot/robot.cpp robot/sensors.cpp robot/threadupdaterobot.cpp 
			       gui/gui.cpp gui/threadupdategui.cpp
			       gui/widget/cameraswidget.cpp gui/widget/controlvw.cpp gui/widget/glwidget.cpp gui/widget/laserwidget.cpp
    )

    SET(qt_HEADERS
			    depuratewindow.h
			    robot/actuators.h robot/robot.h robot/sensors.h robot/threadupdaterobot.h 
         		pioneer/pioneer.c pioneer/pioneeropengl.c
			    gui/gui.h gui/stategui.h gui/threadupdategui.h
			    gui/widget/cameraswidget.h gui/widget/controlvw.h gui/widget/glwidget.h gui/widget/laserwidget.h

    )

    SET(qt_RESOURCES resources.qrc)
    QT4_ADD_RESOURCES(qt_RESOURCES_RCC ${qt_RESOURCES})

    QT4_WRAP_CPP(qt_HEADERS_MOC ${qt_HEADERS})

    ADD_DEFINITIONS(${QT_DEFINITIONS})

    include_directories(
        ${INTERFACES_CPP_DIR}
        ${LIBS_DIR}/
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${easyiceconfig_INCLUDE_DIRS}
    )

    link_directories(
        ${easyiceconfig_LIBRARY_DIRS}
    )

    add_executable( bump_and_go main.cpp depuratewindow.cpp MyAlgorithm.cpp
				    robot/actuators.cpp robot/robot.cpp robot/sensors.cpp robot/threadupdaterobot.cpp 
			        pioneer/pioneer.c pioneer/pioneeropengl.c
			        gui/gui.cpp gui/stategui.cpp gui/threadupdategui.cpp
			        gui/widget/cameraswidget.cpp gui/widget/controlvw.cpp gui/widget/glwidget.cpp gui/widget/laserwidget.cpp
			        ${qt_HEADERS_MOC}
			        ${qt_RESOURCES_RCC}
    )
			       
    target_link_libraries(bump_and_go
        ${CMAKE_THREAD_LIBS_INIT}
        ${OpenCV_LIBRARIES}
	    ${QT_LIBRARIES_JDE}
        JderobotInterfaces
        jderobotutil
        ${easyiceconfig_LIBRARIES}
        ${ZeroCIce_LIBRARIES}
${OPENGL_LIBRARIES} ${GLUT_LIBRARY}
    )
ENDIF()

