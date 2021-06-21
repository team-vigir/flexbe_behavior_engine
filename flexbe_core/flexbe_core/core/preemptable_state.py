#!/usr/bin/env python
from flexbe_core.logger import Logger

from flexbe_msgs.msg import CommandFeedback
from std_msgs.msg import Empty

from flexbe_core.core.lockable_state import LockableState


class PreemptableState(LockableState):
    """
    A state that can be preempted.
    If preempted, the state will not be executed anymore and return the outcome preempted.
    """

    _preempted_name = 'preempted'
    preempt = False

    def __init__(self, *args, **kwargs):
        super(PreemptableState, self).__init__(*args, **kwargs)
        self.__execute = self.execute
        self.execute = self._preemptable_execute

        PreemptableState.preempt = False

        self._feedback_topic = 'flexbe/command_feedback'
        self._preempt_topic = 'flexbe/command/preempt'

    def _preemptable_execute(self, *args, **kwargs):
        if self._is_controlled and self._sub.has_msg(self._preempt_topic):
            self._sub.remove_last_msg(self._preempt_topic)
            self._pub.publish(self._feedback_topic, CommandFeedback(command="preempt"))
            PreemptableState.preempt = True
            Logger.localinfo("--> Behavior will be preempted")

        if PreemptableState.preempt:
            if not self._is_controlled:
                Logger.localinfo("Behavior will be preempted")
            self._force_transition = True
            return self._preempted_name

        return self.__execute(*args, **kwargs)

    def _notify_skipped(self):
        # make sure we dont miss a preempt even if not being executed
        if self._is_controlled and self._sub.has_msg(self._preempt_topic):
            self._sub.remove_last_msg(self._preempt_topic)
            self._pub.publish(self._feedback_topic, CommandFeedback(command="preempt"))
            PreemptableState.preempt = True

    def _enable_ros_control(self):
        super(PreemptableState, self)._enable_ros_control()
        self._pub.createPublisher(self._feedback_topic, CommandFeedback)
        self._sub.subscribe(self._preempt_topic, Empty)
        PreemptableState.preempt = False

    def _disable_ros_control(self):
        super(PreemptableState, self)._disable_ros_control()
        self._sub.unsubscribe_topic(self._preempt_topic)
