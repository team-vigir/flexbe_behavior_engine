#!/usr/bin/env python
from flexbe_core.logger import Logger

from flexbe_msgs.msg import CommandFeedback, OutcomeRequest

from flexbe_core.core.ros_state import RosState


class ManuallyTransitionableState(RosState):
    """
    A state for that a desired outcome can be declared.
    If any outcome is declared, this outcome is forced.
    """

    def __init__(self, *args, **kwargs):
        super(ManuallyTransitionableState, self).__init__(*args, **kwargs)
        self.__execute = self.execute
        self.execute = self._manually_transitionable_execute

        self._force_transition = False

        self._feedback_topic = 'flexbe/command_feedback'
        self._transition_topic = 'flexbe/command/transition'

    def _manually_transitionable_execute(self, *args, **kwargs):
        if self._is_controlled and self._sub.has_buffered(self._transition_topic):
            command_msg = self._sub.get_from_buffer(self._transition_topic)
            self._pub.publish(self._feedback_topic, CommandFeedback(command="transition",
                                                                    args=[command_msg.target, self.name]))
            if command_msg.target != self.name:
                Logger.logwarn("Requested outcome for state %s but active state is %s" %
                               (command_msg.target, self.name))
            else:
                self._force_transition = True
                outcome = self.outcomes[command_msg.outcome]
                Logger.localinfo("--> Manually triggered outcome %s of state %s" % (outcome, self.name))
                return outcome
        # otherwise, return the normal outcome
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
