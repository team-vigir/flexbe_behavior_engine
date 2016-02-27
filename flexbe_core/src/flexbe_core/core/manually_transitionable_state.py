#!/usr/bin/env python
import rospy

from flexbe_core.proxy import ProxyPublisher, ProxySubscriberCached
from flexbe_msgs.msg import CommandFeedback, OutcomeRequest

from flexbe_core.core.monitoring_state import MonitoringState


class ManuallyTransitionableState(MonitoringState):
    """
    A state for that a desired outcome can be declared.
    If any outcome is declared, this outcome is forced.
    """
    
    def __init__(self, *args, **kwargs):
        super(ManuallyTransitionableState, self).__init__(*args, **kwargs)
        
        self._force_transition = False
        
        self.__execute = self.execute
        self.execute = self._manually_transitionable_execute

        self._feedback_topic = 'flexbe/command_feedback'
        self._transition_topic = 'flexbe/command/transition'

        self._pub = ProxyPublisher()
        self._sub = ProxySubscriberCached()


    def _manually_transitionable_execute(self, *args, **kwargs):
        if self._is_controlled and self._sub.has_buffered(self._transition_topic):
            command_msg = self._sub.get_from_buffer(self._transition_topic)
            self._pub.publish(self._feedback_topic, CommandFeedback(command="transition", args=[command_msg.target, self.name]))
            if command_msg.target != self.name:
                rospy.logwarn("--> Requested outcome for state " + command_msg.target + " but active state is " + self.name)
            else:
                self._force_transition = True
                outcome = self._outcome_list[command_msg.outcome]
                rospy.loginfo("--> Manually triggered outcome " + outcome + " of state " + self.name)
                return outcome
        
        # return the normal outcome
        self._force_transition = False
        return self.__execute(*args, **kwargs)


    def _enable_ros_control(self):
        super(ManuallyTransitionableState, self)._enable_ros_control()
        self._pub.createPublisher(self._feedback_topic, CommandFeedback)
        self._sub.subscribe(self._transition_topic, OutcomeRequest)
        self._sub.enable_buffer(self._transition_topic)

    def _disable_ros_control(self):
        super(ManuallyTransitionableState, self)._disable_ros_control()
        self._sub.unsubscribe_topic(self._transition_topic)
