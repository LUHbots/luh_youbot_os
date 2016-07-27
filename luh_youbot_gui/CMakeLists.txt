##############################################################################
# CMake
##############################################################################

cmake_minimum_required(VERSION 2.8.11)
project(luh_youbot_gui)

##############################################################################
# Catkin
##############################################################################

# qt_build provides the qt cmake glue, roscpp the comms for a default talker
find_package(catkin REQUIRED COMPONENTS roscpp roslib luh_youbot_kinematics sensor_msgs actionlib)


##############################################################################
# Qt Environment
##############################################################################

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)
find_package(Qt5Gui REQUIRED)
find_package(Qt5Widgets REQUIRED)
include_directories(${Qt5Gui_INCLUDE_DIRS})
include_directories(${Qt5Widgets_INCLUDE_DIRS})
add_definitions(${Qt5Widgets_DEFINITIONS})
set(CMAKE_CXX_FLAGS "${Qt5Gui_EXECUTABLE_COMPILE_FLAGS}")

##############################################################################
# Catkin 2
##############################################################################
include_directories(${catkin_INCLUDE_DIRS})
include_directories(include)
catkin_package()

##############################################################################
# Sections
##############################################################################

#file(GLOB QT_FORMS RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} ui/*.ui)
#file(GLOB QT_RESOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} resources/*.qrc)
#file(GLOB_RECURSE QT_MOC RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS include/luh_youbot_gui/*.hpp)

#QT4_ADD_RESOURCES(QT_RESOURCES_CPP ${QT_RESOURCES})
#QT4_WRAP_UI(QT_FORMS_HPP ${QT_FORMS})
#QT4_WRAP_CPP(QT_MOC_HPP ${QT_MOC})

##############################################################################
# Sources
##############################################################################

file(GLOB_RECURSE QT_SOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS src/*.cpp)
file(GLOB_RECURSE QT_HEADERS RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS include/luh_youbot_gui/*.hpp)

##############################################################################
# Binaries
##############################################################################

add_executable(luh_youbot_gui ${QT_SOURCES} ${QT_HEADERS} ${QT_RESOURCES_CPP} ${QT_FORMS_HPP} ${QT_MOC_HPP} ${QT_MOC})
#target_link_libraries(luh_youbot_gui ${QT_LIBRARIES} ${catkin_LIBRARIES} yaml-cpp luh_youbot_kinematics)
target_link_libraries(luh_youbot_gui ${Qt5Gui_LIBRARIES} ${Qt5Widgets_LIBRARIES} ${catkin_LIBRARIES} yaml-cpp luh_youbot_kinematics)
install(TARGETS luh_youbot_gui RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

