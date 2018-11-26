cmake_minimum_required(VERSION 2.8.3)
project(flexbe_states)

find_package(catkin REQUIRED)

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
catkin_python_setup()

# specify catkin-specific information
# INCLUDE_DIRS - The exported include paths (i.e. cflags) for the package
# LIBRARIES - The exported libraries from the project
# CATKIN_DEPENDS - Other catkin projects that this project depends on
# DEPENDS - Non-catkin CMake projects that this project depends on
# CFG_EXTRAS - Additional configuration options 
catkin_package(
    INCLUDE_DIRS src
    LIBRARIES ${PROJECT_NAME})

# use add_library() or add_executable() as required
#add_library(${PROJECT_NAME} ${${PROJECT_NAME}_SRCS})

# install executables
#install(PROGRAMS bin/hello
#        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(CODE "
  file(MAKE_DIRECTORY \"$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}/src\")
  execute_process(COMMAND
    \"${CMAKE_COMMAND}\" -E create_symlink \"../../../${CATKIN_PACKAGE_PYTHON_DESTINATION}\"
                                           \"$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}/src/${PROJECT_NAME}\"
  )
")

# run tests
if(CATKIN_ENABLE_TESTING)
	find_package(rostest REQUIRED)
	add_rostest(tests/run_rostest.test)
endif()

