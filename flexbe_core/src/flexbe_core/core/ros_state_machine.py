#!/usr/bin/env python
import rospy
from flexbe_core.proxy import ProxyPublisher, ProxySubscriberCached

from flexbe_core.core.state_machine import StateMachine


class RosStateMachine(StateMachine):
    """
    A state machine to interface with ROS.
    """

    def __init__(self, *args, **kwargs):
        super(RosStateMachine, self).__init__(*args, **kwargs)
        self._is_controlled = False

        self._pub = ProxyPublisher()
        self._sub = ProxySubscriberCached()

    def wait(self, seconds=None, condition=None):
        if seconds is not None:
            rospy.sleep(seconds)
        if condition is not None:
            rate = rospy.Rate(100)
            while not rospy.is_shutdown():
                if condition():
                    break
                rate.sleep()

    def _enable_ros_control(self):
        self._is_controlled = True
        for state in self._states:
            state._enable_ros_control()

    def _disable_ros_control(self):
        self._is_controlled = False
        for state in self._states:
            state._disable_ros_control()
