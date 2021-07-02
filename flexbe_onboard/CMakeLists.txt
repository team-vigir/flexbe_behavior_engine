cmake_minimum_required(VERSION 3.0.2)
project(flexbe_onboard)

find_package(catkin REQUIRED)

catkin_python_setup()

catkin_package()

install(PROGRAMS bin/start_behavior
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY launch
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY test
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
	PATTERN "*.py" EXCLUDE) # do not install scripts twice

catkin_install_python(
	PROGRAMS test/test_onboard.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
	find_package(rostest REQUIRED)
	add_rostest(test/${PROJECT_NAME}.test)
endif()
