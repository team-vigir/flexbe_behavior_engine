cmake_minimum_required(VERSION 3.0.2)
project(flexbe_widget)

find_package(catkin REQUIRED)

catkin_python_setup()

catkin_package(
    INCLUDE_DIRS src
    LIBRARIES ${PROJECT_NAME})

FILE(GLOB BIN_FILES "bin/*")

install(PROGRAMS ${BIN_FILES} DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
