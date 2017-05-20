cmake_minimum_required(VERSION 2.8.3)
project(led_demo)
add_definitions ( -Wall )
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
)

add_message_files(
    DIRECTORY msg
    FILES
    led.msg
)

generate_messages(
    DEPENDENCIES
    std_msgs
)


find_package ( Qt4 REQUIRED )
include ( ${QT_USE_FILE} )
include_directories (
	${CMAKE_SOURCE_DIR} ${CMAKE_CURRENT_BINARY_DIR}
	${QT_QTCORE_INCLUDE_DIR} ${QT_QTGUI_INCLUDE_DIR}
	)

set ( untitled_HDRS

        )

set ( untitled_SRCS
        src/main.cpp
        #window.cpp
        #sliderexample.cpp
        src/mainwindow.cpp
        )

set ( untitled_UIS
        ui/mainwindow.ui
	)

QT4_WRAP_UI(UIS ${untitled_UIS})

set ( untitled_MOCS
        #window.h
        #sliderexample.h
        include/led_demo/mainwindow.h
	)

QT4_WRAP_CPP(MOCS ${untitled_MOCS})

catkin_package(

    CATKIN_DEPENDS message_runtime

)


include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(led_show
    src/main.cpp
    src/mainwindow.cpp

    include/led_demo/mainwindow.h
)

add_library( led_demo SHARED ${untitled_SRCS} ${UIS} ${RSCS} ${TRS} ${MOCS} )

target_link_libraries( led_demo  ${QT_QTCORE_LIBRARY} ${QT_QTGUI_LIBRARY} ${catkin_LIBRARIES})

target_link_libraries(led_show led_demo)

add_dependencies(led_show ${catkin_EXPORTED_TARGETS})

add_definitions(-std=c++11)
