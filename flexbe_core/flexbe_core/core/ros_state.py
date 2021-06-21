#!/usr/bin/env python
from flexbe_core.proxy import ProxyPublisher, ProxySubscriberCached

from flexbe_core.core.state import State


class RosState(State):
    """
    A state to interface with ROS.
    """
    _node = None
    _breakpoints = []

    @staticmethod
    def initialize_ros(node):
        RosState._node = node
        RosState._breakpoints = node.declare_parameter('breakpoints', [])

    def __init__(self, *args, **kwargs):
        super(RosState, self).__init__(*args, **kwargs)
        self._rate = RosState._node.create_rate(10)
        self._is_controlled = False

        self._pub = ProxyPublisher()
        self._sub = ProxySubscriberCached()

    def sleep(self):
        self._rate.sleep()

    @property
    def sleep_duration(self):
        return self._rate._time.time_until_next_call()

    def set_rate(self, rate):
        """
        Set the execution rate of this state,
        i.e., the rate with which the execute method is being called.

        Note: The rate is best-effort, real-time support is not yet available.

        @type label: float
        @param label: The desired rate in Hz.
        """
        self._rate = RosState._node.create_rate(rate)

    def _enable_ros_control(self):
        self._is_controlled = True

    def _disable_ros_control(self):
        self._is_controlled = False

    @property
    def is_breakpoint(self):
        return self.path in RosState._breakpoints.get_parameter_value().string_array_value
