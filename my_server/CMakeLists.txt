##############################################################################
# CMake
##############################################################################

cmake_minimum_required(VERSION 2.8.0)
project(my_server)


##############################################################################
# Catkin
##############################################################################

# qt_build provides the qt cmake glue, roscpp the comms for a default talker

find_package(catkin REQUIRED COMPONENTS qt_build roscpp image_transport cv_bridge std_msgs message_generation)

# add the my messages
add_message_files(DIRECTORY msg
   FILES
   PID_constant.msg
   led_control.msg
   servo_control.msg
   robot_control.msg
)
generate_messages(DEPENDENCIES sensor_msgs std_msgs)


find_package(OpenCV REQUIRED)
find_package(CUDA)

#qt libraries
#set(CMAKE_AUTOMOC ON)

#find_package(Qt5Widgets)
#find_package(Qt5Core)
#set(CMAKE_CXX_FLAGS_COVERAGE "${CMAKE_CXX_FLAGS_RELEASE} -fprofile-arcs -ftest-coverage")
#set(CMAKE_CXX_STANDARD 14)

set(CUDA_NVCC_FLAGS ${CUDA_NVCC_FLAGS} -arch compute_20 -code sm_20)
list(APPEND CUDA_NVCC_FLAGS "-arch=sm_20;-std=c++11;-O2;-DVERBOSE")
SET(CUDA_PROPAGATE_HOST_FLAGS OFF)

include_directories(${catkin_INCLUDE_DIRS})
include_directories(include ${OpenCV_INCLUDE_DIRS})
include_directories(include ${CUDA_INCLUDE_DIRS})

link_directories(${CUDA_LIBRARY_DIRS})
# Use this to define what the package will export (e.g. libs, headers).
# Since the default here is to produce only a binary, we don't worry about
# exporting anything. 
catkin_package()

##############################################################################
# Qt Environment
##############################################################################

# this comes from qt_build's qt-ros.cmake which is automatically 
# included via the dependency call in package.xml
rosbuild_prepare_qt4(QtCore QtGui) # Add the appropriate components to the component list here

##############################################################################
# Sections
##############################################################################

file(GLOB QT_FORMS RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} ui/*.ui)
file(GLOB QT_RESOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} resources/*.qrc)
file(GLOB_RECURSE QT_MOC RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS include/my_server/*.hpp)

QT4_ADD_RESOURCES(QT_RESOURCES_CPP ${QT_RESOURCES})
QT4_WRAP_UI(QT_FORMS_HPP ${QT_FORMS})
QT4_WRAP_CPP(QT_MOC_HPP ${QT_MOC})
##############################################################################
# Sources
##############################################################################

file(GLOB_RECURSE QT_SOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS src/*.cpp)

##############################################################################
# Binaries
##############################################################################

add_executable(my_server ${QT_SOURCES} ${QT_RESOURCES_CPP} ${QT_FORMS_HPP} ${QT_MOC_HPP} ${UI_HEADERS})
target_link_libraries(my_server ${QT_LIBRARIES} ${QT5Core_LIBRARIES} ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${Qt5Widgets_LIBRARIES})
add_library(main_window src/main_window.cpp)
target_link_libraries(main_window ${QT_LIBRARIES} ${QT5Core_LIBRARIES} ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${Qt5Widgets_LIBRARIES})
install(TARGETS my_server RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

