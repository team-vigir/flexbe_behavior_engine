cmake_minimum_required(VERSION 3.5)

project(flexbe_msgs)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

set(msg_files
  "msg/BEStatus.msg"
  "msg/BehaviorLog.msg"
  "msg/BehaviorModification.msg"
  "msg/BehaviorRequest.msg"
  "msg/BehaviorSelection.msg"
  "msg/BehaviorSync.msg"
  "msg/CommandFeedback.msg"
  "msg/Container.msg"
  "msg/ContainerStructure.msg"
  "msg/OutcomeCondition.msg"
  "msg/OutcomeRequest.msg"
  "msg/StateInstantiation.msg"
  "msg/SynthesisErrorCodes.msg"
  "msg/SynthesisRequest.msg"
  "msg/UICommand.msg"
)
set(action_files
  "action/BehaviorInput.action"
  "action/BehaviorExecution.action"
  "action/BehaviorSynthesis.action"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${action_files}
  ADD_LINTER_TESTS
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
