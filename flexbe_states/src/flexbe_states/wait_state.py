#!/usr/bin/env python
import rospy
from flexbe_core import EventState


class WaitState(EventState):
    '''
    Implements a state that can be used to wait on timed process.

    -- wait_time 	float	Amount of time to wait in seconds.

    <= done					Indicates that the wait time has elapsed.
    '''

    def __init__(self, wait_time):
        super(WaitState, self).__init__(outcomes=['done'])
        self._wait = wait_time

    def execute(self, userdata):
        elapsed = rospy.get_rostime() - self._start_time
        if elapsed.to_sec() > self._wait:
            return 'done'

    def on_enter(self, userdata):
        '''Upon entering the state, save the current time and start waiting.'''
        self._start_time = rospy.get_rostime()
