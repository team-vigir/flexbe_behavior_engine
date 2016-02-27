#!/usr/bin/env python
import rospy
from flexbe_core import EventState

class TestAddState(EventState):

	def __init__(self, first_arg):
		'''Constructor'''
		super(TestAddState, self).__init__(outcomes=['done'],
										input_keys=['second_arg'],
										output_keys=['result'])
		self._first_arg = first_arg

		
	def execute(self, userdata):
		userdata.result = userdata.second_arg.data + self._first_arg
		return 'done'

