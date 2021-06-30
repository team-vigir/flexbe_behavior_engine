cmake_minimum_required(VERSION 3.0.2)
project(flexbe_input)

find_package(catkin REQUIRED)

catkin_python_setup()

catkin_package()

install(PROGRAMS bin/behavior_input
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
