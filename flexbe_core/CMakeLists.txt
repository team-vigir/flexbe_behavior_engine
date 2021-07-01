cmake_minimum_required(VERSION 3.0.2)
project(flexbe_core)

find_package(catkin REQUIRED)

catkin_python_setup()

catkin_package()

install(DIRECTORY test
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
	PATTERN "*.py" EXCLUDE # do not install scripts twice
)
catkin_install_python(
	PROGRAMS test/test_proxies.py test/test_exceptions.py test/test_core.py
	DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
	find_package(rostest REQUIRED)
	add_rostest(test/${PROJECT_NAME}.test)
	add_rostest(test/flexbe_exceptions.test)
	add_rostest(test/flexbe_proxies.test)
endif()
