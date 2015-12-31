#!/usr/bin/env python
import rospy
from flexbe_core import EventState


class ImportOnlyState(EventState):
	
	def __init__(self):
		'''Constructor'''
		super(ImportOnlyState, self).__init__(outcomes=['done'])
		
		raise Exception('Test should be import only!')
