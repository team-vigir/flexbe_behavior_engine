#!/usr/bin/env python
import rospy
from flexbe_core.proxy import ProxyPublisher, ProxySubscriberCached

from .state import State


class RosState(State):
    """
    A state to interface with ROS.
    """

    def __init__(self, *args, **kwargs):
        super(RosState, self).__init__(*args, **kwargs)
        self._rate = rospy.Rate(10)
        self._is_controlled = False

        self._pub = ProxyPublisher()
        self._sub = ProxySubscriberCached()

    def sleep(self):
        self._rate.sleep()

    @property
    def sleep_duration(self):
        return self._rate.remaining().to_sec()

    def set_rate(self, rate):
        """
        Set the execution rate of this state,
        i.e., the rate with which the execute method is being called.

        Note: The rate is best-effort,
              a rospy.Rate does not guarantee real-time properties.

        @type label: float
        @param label: The desired rate in Hz.
        """
        self._rate = rospy.Rate(rate)

    def _enable_ros_control(self):
        self._is_controlled = True

    def _disable_ros_control(self):
        self._is_controlled = False

    @property
    def is_breakpoint(self):
        return self.path in rospy.get_param('/flexbe/breakpoints', [])
