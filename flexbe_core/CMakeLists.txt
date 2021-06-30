cmake_minimum_required(VERSION 3.0.2)
project(flexbe_core)

find_package(catkin REQUIRED)

catkin_python_setup()

catkin_package()

if(CATKIN_ENABLE_TESTING)
	find_package(rostest REQUIRED)
	add_rostest(test/${PROJECT_NAME}.test)
	add_rostest(test/flexbe_exceptions.test)
	add_rostest(test/flexbe_proxies.test)
endif()
