#!/usr/bin/env python
from flexbe_core.proxy import ProxyPublisher, ProxySubscriberCached

from flexbe_core.core.state_machine import StateMachine


class RosStateMachine(StateMachine):
    """
    A state machine to interface with ROS.
    """
    _node = None

    @staticmethod
    def initialize_ros(node):
        RosStateMachine._node = node

    def __init__(self, *args, **kwargs):
        super(RosStateMachine, self).__init__(*args, **kwargs)
        self._is_controlled = False

        self._pub = ProxyPublisher()
        self._sub = ProxySubscriberCached()

    def wait(self, seconds=None, condition=None):
        if seconds is not None:
            RosStateMachine._node.create_rate(1 / seconds).sleep()
        if condition is not None:
            rate = RosStateMachine._node.create_rate(100)
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
