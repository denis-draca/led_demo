cmake_minimum_required(VERSION 2.8.3)
project(led_demo)
add_definitions ( -Wall )
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
)

find_package(Threads)
SET(CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS} "-std=c++11 -pthread -lpthread -Wl,--no-as-needed")

add_message_files(
    DIRECTORY msg
    FILES
    led.msg
)

generate_messages(
    DEPENDENCIES
    std_msgs
)

set (untitled_HDRS
)

set (untitled_SRCS
    src/main.cpp
    src/mainwindow.cpp
)

set (untitled_UIS
    ui/mainwindow.ui
)

set (RESOURCES
    resources/caslogo.qrc
)

set (untitled_MOCS
    include/led_demo/mainwindow.h
)


find_package ( Qt4 REQUIRED )
include ( ${QT_USE_FILE} )
add_definitions( ${QT_DEFINITIONS} )
include_directories (
    ${CMAKE_SOURCE_DIR} ${CMAKE_CURRENT_BINARY_DIR}
    ${QT_QTCORE_INCLUDE_DIR} ${QT_QTGUI_INCLUDE_DIR}
    ${CMAKE_BINARY_DIR}
)


QT4_WRAP_UI(UIS ${untitled_UIS})
QT4_WRAP_CPP(MOCS ${untitled_MOCS})
QT4_ADD_RESOURCES( RES_SOURCES ${RESOURCES} )

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

    ${SOURCES} ${MOC_SRCS} ${RES_SOURCES} ${UI_HEADERS}
)

add_library( led_demo SHARED ${untitled_SRCS} ${UIS} ${RSCS} ${TRS} ${MOCS} ${RES_SOURCES} )

target_link_libraries( led_demo  ${QT_QTCORE_LIBRARY} ${QT_QTGUI_LIBRARY} ${catkin_LIBRARIES} ${CMAKE_THREAD_LIBS_INIT} ${QT_LIBRARIES})

target_link_libraries(led_show led_demo)

add_dependencies(led_show ${catkin_EXPORTED_TARGETS})

add_definitions(-std=c++11)
