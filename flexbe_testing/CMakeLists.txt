cmake_minimum_required(VERSION 3.0.2)
project(flexbe_testing)

find_package(catkin REQUIRED)

catkin_python_setup()

catkin_package()

FILE(GLOB BIN_FILES "bin/*")

install(PROGRAMS ${BIN_FILES} DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY test DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

if(CATKIN_ENABLE_TESTING)
	find_package(rostest REQUIRED)
	add_rostest(test/selftest.test)
endif()
