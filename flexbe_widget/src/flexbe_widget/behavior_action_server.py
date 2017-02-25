#!/usr/bin/env python
import rospy
import actionlib

from flexbe_msgs.msg import *
from rospkg import RosPack
from flexbe_core import BehaviorLibrary

from std_msgs.msg import String, Empty

import pickle
import zlib
import difflib
import os
import xml.etree.ElementTree as ET


class BehaviorActionServer(object):

	def __init__(self):
		self._pub = rospy.Publisher('flexbe/start_behavior', BehaviorSelection, queue_size=100)
		self._preempt_pub = rospy.Publisher('flexbe/command/preempt', Empty, queue_size=100)
		self._status_sub = rospy.Subscriber('flexbe/status', BEStatus, self._status_cb)
		self._state_sub = rospy.Subscriber('flexbe/behavior_update', String, self._state_cb)

		self._behavior_running = False
		self._current_state = None
		self._engine_status = None

		self._as = actionlib.SimpleActionServer('flexbe/execute_behavior', BehaviorExecutionAction, self._execute_cb, False)
		self._as.start()

		self._rp = RosPack()
		self._behavior_lib = BehaviorLibrary()

		rospy.loginfo("%d behaviors available, ready for start request." % self._behavior_lib.count_behaviors())


	def _execute_cb(self, goal):
		rospy.loginfo('Received a new request to start behavior: %s' % goal.behavior_name)
		be_id, behavior = self._behavior_lib.find_behavior(goal.behavior_name)
		if be_id is None:
			Logger.logerr("Did not find behavior with requested name: %s" % goal.behavior_name)
			return

		be_selection = BehaviorSelection()
		be_selection.behavior_id = be_id
		be_selection.autonomy_level = 255
		be_selection.arg_keys = goal.arg_keys
		be_selection.arg_values = goal.arg_values
		be_selection.input_keys = goal.input_keys
		be_selection.input_values = goal.input_values

		be_structure = ContainerStructure()
		be_structure.containers = self._create_behavior_structure(goal.behavior_name)

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

		self._pub.publish(be_selection)
		self._behavior_running = False

		rate = rospy.Rate(10)
		while not rospy.is_shutdown():
			if self._current_state is not None:
				self._as.publish_feedback(BehaviorExecutionFeedback(self._current_state))
				self._current_state = None

			if self._engine_status.code == BEStatus.ERROR:
				rospy.logerr('Failed to run behavior! Check onboard terminal for further infos.')
				self._as.set_aborted('')
				break

			if not self._behavior_running:
				if self._engine_status.code == BEStatus.STARTED:
					self._behavior_running = True
					rospy.loginfo('Behavior execution has started!')
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
			if self._as.is_preempt_requested():
				rospy.loginfo('Preempt Requested!')
				self._preempt_pub.publish()
				self._as.set_preempted('')
				break

			rate.sleep()
			
		rospy.loginfo('Ready for next start request.')


	def _status_cb(self, msg):
		self._engine_status = msg


	def _state_cb(self, msg):
		self._current_state = msg.data
		rospy.loginfo('Current state: %s' % self._current_state)


	def _create_behavior_structure(self, behavior_name):
		return list()