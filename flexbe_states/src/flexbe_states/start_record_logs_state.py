#!/usr/bin/env python

from flexbe_core import EventState, Logger
import subprocess
import os
import signal
import rospy

"""Created on Oct. 17, 2014

@author: Spyros Maniatopoulos
"""

class StartRecordLogsState(EventState):
	"""
	A state that records the contents of the specified ROS topics in a bag file.
	Logging is done by creating a rosbag subprocess which is afterwards accessible using the output key rosbag_process.
	This state is typically combined with a StopRecordLogsState which gets the subprocess in order to stop logging.

	-- topics_to_record string[]	A list of topics (strings) that this state will log in a bagfile (.bag) using rosbag record.
									They are usually specified in a config (.yaml) file, which is added as a behavior parameter.

	># bagfile_name		string		Full path of the bagfile to be created by this logging.

	#> rosbag_process 	subprocess	The process that is executing rosbag record.
									The ID of this subprocess object can be used to kill it later.

	<= logging						Indicates that rosbag record was started.

	"""
	
	def __init__(self, topics_to_record):
		"""Constructor"""
		super(StartRecordLogsState, self).__init__(outcomes=['logging'],
												input_keys=['bagfile_name'],
												output_keys=['rosbag_process'])

		self._topics_to_record = topics_to_record
		self._bag_process = None

	def execute(self, userdata):
		"""Execute this state"""
		
		# State has already started recording upon enter
		return 'logging'
	
	def on_enter(self, userdata):
		"""Upon entering the state""" 
		
		bash_command = ["/bin/bash", "--norc", "-c"]

		# Start Bagging
		bag_command = "rosbag record -O {}".format(userdata.bagfile_name) + " " +  self._topics_to_record.replace(",", "")
		self._bag_process = subprocess.Popen(bash_command + [bag_command], stdout=subprocess.PIPE, preexec_fn=os.setsid)
		userdata.rosbag_process = self._bag_process
		Logger.loginfo('Recording topics to %s' % userdata.bagfile_name)

	def on_stop(self):
		"""Kill any rosbag record processes when behavior execution is stopped"""
		try:
			if self._bag_process is not None:
				os.killpg(self._bag_process.pid, signal.SIGINT)
		except Exception as e:
			rospy.logwarn("Unable to kill process %s:\n%s" % (str(self._bag_process.pid), str(e)))
