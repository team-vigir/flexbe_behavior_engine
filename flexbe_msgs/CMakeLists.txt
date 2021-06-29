cmake_minimum_required(VERSION 2.8.3)
project(flexbe_msgs)

find_package(catkin REQUIRED COMPONENTS message_generation actionlib_msgs actionlib)

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
generate_messages(DEPENDENCIES actionlib_msgs)

catkin_package(CATKIN_DEPENDS message_runtime actionlib_msgs)
