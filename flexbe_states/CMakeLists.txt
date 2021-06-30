cmake_minimum_required(VERSION 3.0.2)
project(flexbe_states)

find_package(catkin REQUIRED)

catkin_python_setup()

catkin_package()

if(CATKIN_ENABLE_TESTING)
	find_package(rostest REQUIRED)
	add_rostest(tests/run_rostest.test)
endif()

