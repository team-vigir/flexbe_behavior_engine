#!/usr/bin/env python
import rospy
import actionlib

from flexbe_msgs.msg import *
from rospkg import RosPack
from flexbe_core import BehaviorLibrary

from std_msgs.msg import String, Empty

import zlib
import difflib
import os
import xml.etree.ElementTree as ET


class BehaviorActionServer(object):

	def __init__(self):
		self._behavior_started = False
		self._current_state = None
		self._engine_status = None

		self._pub = rospy.Publisher('flexbe/start_behavior', BehaviorSelection, queue_size=100)
		self._preempt_pub = rospy.Publisher('flexbe/command/preempt', Empty, queue_size=100)
		self._status_sub = rospy.Subscriber('flexbe/status', BEStatus, self._status_cb)
		self._state_sub = rospy.Subscriber('flexbe/behavior_update', String, self._state_cb)

		self._as = actionlib.SimpleActionServer('flexbe/execute_behavior', BehaviorExecutionAction, self._execute_cb, False)

		self._rp = RosPack()
		self._behavior_lib = BehaviorLibrary()

		# start action server after all member variables have been initialized
		self._as.start()

		rospy.loginfo("%d behaviors available, ready for start request." % self._behavior_lib.count_behaviors())


	def _execute_cb(self, goal):
		rospy.loginfo('Received a new request to start behavior: %s' % goal.behavior_name)
		be_id, behavior = self._behavior_lib.find_behavior(goal.behavior_name)
		if be_id is None:
			Logger.logerr("Did not find behavior with requested name: %s" % goal.behavior_name)
			self._as.set_preempted()
			return

		be_selection = BehaviorSelection()
		be_selection.behavior_id = be_id
		be_selection.autonomy_level = 255
		be_selection.arg_keys = goal.arg_keys
		be_selection.arg_values = goal.arg_values
		be_selection.input_keys = goal.input_keys
		be_selection.input_values = goal.input_values

		# check for local modifications of the behavior to send them to the onboard behavior
		be_filepath_new = os.path.join(self._rp.get_path(behavior["package"]), 'src/' + behavior["package"] + '/' + behavior["file"] + '.py')
		with open(be_filepath_new, "r") as f:
			be_content_new = f.read()

		be_filepath_old = os.path.join(self._rp.get_path(behavior["package"]), 'src/' + behavior["package"] + '/' + behavior["file"] + '_tmp.py')
		if not os.path.isfile(be_filepath_old):
			be_selection.behavior_checksum = zlib.adler32(be_content_new)
		else:
			with open(be_filepath_old, "r") as f:
				be_content_old = f.read()

			sqm = difflib.SequenceMatcher(a=be_content_old, b=be_content_new)
			diffs = [x[1] for x in sqm.get_grouped_opcodes(0)]
			for opcode, a0, a1, b0, b1 in diffs:
				content = be_content_new[b0:b1]
				be_selection.modifications.append(BehaviorModification(a0, a1, content))

			be_selection.behavior_checksum = zlib.adler32(be_content_new)

		# reset state before starting new behavior
		self._current_state = None
		self._behavior_started = False

		# start new behavior
		self._pub.publish(be_selection)

		try:
			rate = rospy.Rate(10)
			while not rospy.is_shutdown():
				if self._current_state is not None:
					self._as.publish_feedback(BehaviorExecutionFeedback(self._current_state))
					self._current_state = None

				# check if goal has been preempted first
				if self._as.is_preempt_requested():
					rospy.loginfo('Behavior execution preempt requested!')
					self._preempt_pub.publish()
					self._as.set_preempted('')
					break

				if self._engine_status is None:
					rospy.loginfo('No behavior engine status received yet. Waiting for it...')
					rate.sleep()
					continue

				if self._engine_status.code == BEStatus.ERROR:
					rospy.logerr('Failed to run behavior! Check onboard terminal for further infos.')
					self._as.set_aborted('')
					break

				if not self._behavior_started:
					rospy.logdebug('Behavior execution has not yet started. Waiting for it...')
					rate.sleep()
					continue

				if self._engine_status.code == BEStatus.FINISHED:
					result = self._engine_status.args[0] \
						if len(self._engine_status.args) >= 1 else ''
					rospy.loginfo('Finished behavior execution with result "%s"!' % result)
					self._as.set_succeeded(BehaviorExecutionResult(outcome=result))
					break

				if self._engine_status.code == BEStatus.FAILED:
					rospy.logerr('Behavior execution failed in state %s!' % str(self._current_state))
					self._as.set_aborted('')
					break

				rate.sleep()

			rospy.loginfo('Ready for next behavior start request.')

		except rospy.ROSInterruptException:
			pass # allow clean exit on ROS shutdown


	def _status_cb(self, msg):
		self._engine_status = msg

		# check for behavior start here, to avoid race condition between execute_cb and status_cb threads
		if not self._behavior_started and self._engine_status.code == BEStatus.STARTED:
			self._behavior_started = True
			rospy.loginfo('Behavior execution has started!')


	def _state_cb(self, msg):
		self._current_state = msg.data
		rospy.loginfo('Current state: %s' % self._current_state)
