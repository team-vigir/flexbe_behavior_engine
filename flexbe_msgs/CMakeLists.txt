cmake_minimum_required(VERSION 3.0.2)
project(flexbe_msgs)

find_package(catkin REQUIRED COMPONENTS actionlib actionlib_msgs message_generation)

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
  UserdataInfo.msg
)
add_action_files(FILES
  BehaviorExecution.action
  BehaviorInput.action
  BehaviorSynthesis.action
)

add_service_files(FILES
  GetUserdata.srv
)

generate_messages(DEPENDENCIES actionlib_msgs)

catkin_package(CATKIN_DEPENDS actionlib actionlib_msgs message_runtime)
