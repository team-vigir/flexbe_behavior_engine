cmake_minimum_required(VERSION 2.8.3)
project(flexbe_msgs)

find_package(catkin REQUIRED COMPONENTS message_generation actionlib_msgs actionlib)

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
#catkin_python_setup()

# generate msg and srv
add_message_files(FILES 
  BEStatus.msg
  BehaviorLog.msg
  BehaviorModification.msg
  BehaviorRequest.msg
  BehaviorSelection.msg
  BehaviorSync.msg
  CommandFeedback.msg
  Container.msg
  ContainerStructure.msg
  OutcomeCondition.msg
  OutcomeRequest.msg
  StateInstantiation.msg
  SynthesisErrorCodes.msg
  SynthesisRequest.msg
  UICommand.msg
)
add_action_files(FILES 
  BehaviorInput.action
  BehaviorExecution.action
  BehaviorSynthesis.action
)
#add_action_files(FILES ...)
# opt arg DEPENDENCIES ...
generate_messages(DEPENDENCIES actionlib_msgs)

# specify catkin-specific information
# INCLUDE_DIRS - The exported include paths (i.e. cflags) for the package
# LIBRARIES - The exported libraries from the project
# CATKIN_DEPENDS - Other catkin projects that this project depends on
# DEPENDS - Non-catkin CMake projects that this project depends on
# CFG_EXTRAS - Additional configuration options 
catkin_package(
#    INCLUDE_DIRS src
#    LIBRARIES ${PROJECT_NAME}
    CATKIN_DEPENDS message_runtime actionlib_msgs)

# use add_library() or add_executable() as required
#add_library(${PROJECT_NAME} ${${PROJECT_NAME}_SRCS})

# install executables
#install(PROGRAMS bin/hello
#        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

