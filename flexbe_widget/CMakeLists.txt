cmake_minimum_required(VERSION 3.0.2)
project(flexbe_widget)

find_package(catkin REQUIRED)

catkin_python_setup()

catkin_package()


install(DIRECTORY launch
  	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(PROGRAMS
    bin/be_action_server
    bin/be_compress
    bin/be_launcher
    bin/breakpoint
    bin/create_repo
    bin/evaluate_logs
DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
