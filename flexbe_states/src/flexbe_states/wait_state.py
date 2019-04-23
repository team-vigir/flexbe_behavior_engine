#!/usr/bin/env python

import rospy
from flexbe_core import EventState
from rospy.exceptions import ROSInterruptException

'''
Created on 15.06.2013

@author: David Conner
'''

class WaitState(EventState):
	'''
	Implements a state that can be used to wait on timed process.

	-- wait_time 	float	Amount of time to wait in seconds.

	<= done					Indicates that the wait time has elapsed.

	'''
	def __init__(self, wait_time):
		'''Constructor'''
		super(WaitState, self).__init__(outcomes=['done'])
		self._wait = wait_time

		
	def execute(self, userdata):
		'''Execute this state'''

		elapsed = rospy.get_rostime() - self._start_time;
		if (elapsed.to_sec() > self._wait):
			return 'done'

	
	def on_enter(self, userdata):
		'''Upon entering the state, save the current time and start waiting.'''

		self._start_time = rospy.get_rostime()
		
		try:
			self._rate.sleep()
		except ROSInterruptException:
			rospy.logwarn('Skipped sleep.')
