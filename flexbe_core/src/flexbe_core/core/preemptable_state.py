#!/usr/bin/env python
import rospy

from flexbe_core.core.loopback_state import LoopbackState

from flexbe_core.proxy import ProxyPublisher, ProxySubscriberCached
from flexbe_msgs.msg import CommandFeedback
from std_msgs.msg import Empty


class PreemptableState(LoopbackState):
    """
    A state that can be preempted.
    If preempted, the state will not be executed anymore and return the outcome preempted.
    """
    
    _preempted_name = 'preempted'
    preempt = False
    switching = False

    def __init__(self, *args, **kwargs):
        # add preempted outcome
        if len(args) > 0 and type(args[0]) is list:
            # need this ugly check for list type, because first argument in CBState is the callback
            args[0].append(self._preempted_name)
        else:
            outcomes = kwargs.get('outcomes', [])
            outcomes.append(self._preempted_name)
            kwargs['outcomes'] = outcomes
            
        super(PreemptableState, self).__init__(*args, **kwargs)
        self.__execute = self.execute
        self.execute = self._preemptable_execute

        self._feedback_topic = 'flexbe/command_feedback'
        self._preempt_topic = 'flexbe/command/preempt'

        self._pub = ProxyPublisher()
        self._sub = ProxySubscriberCached()

        PreemptableState.preempt = False


    def _preemptable_execute(self, *args, **kwargs):
        preempting = False
        if self._is_controlled and self._sub.has_msg(self._preempt_topic):
            self._sub.remove_last_msg(self._preempt_topic)
            self._pub.publish(self._feedback_topic, CommandFeedback(command="preempt"))
            preempting = True
            PreemptableState.preempt = True
            rospy.loginfo("--> Behavior will be preempted")

        if PreemptableState.preempt:
            preempting = True
            rospy.loginfo("Behavior will be preempted")
            
        if preempting:
            self.service_preempt()
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