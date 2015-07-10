#!/usr/bin/env python

from flexbe_core import EventState
import subprocess
import signal
import os
import rospy

"""Created on Oct. 17, 2014

@author: Spyros Maniatopoulos
"""

class StopRecordLogsState(EventState):
	"""
	A state that records the contents of the specified ROS topics in a bag file.

	># rosbag_process 	subprocess	A system process, whose ID is used to kill it.

	<= stopped						Indicates that a command to kill the process has been issued.

	"""
	
	def __init__(self):
		"""Constructor"""
		super(StopRecordLogsState, self).__init__(outcomes=['stopped'],
												  input_keys=['rosbag_process'])

	def execute(self, userdata):
		
		return 'stopped'
	
	def on_enter(self, userdata):
		"""Upon entering the state, kill the process"""

		os.killpg(userdata.rosbag_process.pid, signal.SIGINT)
