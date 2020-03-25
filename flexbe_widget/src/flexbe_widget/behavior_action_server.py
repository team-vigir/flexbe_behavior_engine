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
import yaml
import xml.etree.ElementTree as ET


class BehaviorActionServer(object):

	def __init__(self):
		self._behavior_started = False
		self._current_state = None
		self._active_behavior_id = None

		self._pub = rospy.Publisher('flexbe/start_behavior', BehaviorSelection, queue_size=100)
		self._preempt_pub = rospy.Publisher('flexbe/command/preempt', Empty, queue_size=100)
		self._status_sub = rospy.Subscriber('flexbe/status', BEStatus, self._status_cb)
		self._state_sub = rospy.Subscriber('flexbe/behavior_update', String, self._state_cb)

		self._as = actionlib.SimpleActionServer('flexbe/execute_behavior', BehaviorExecutionAction, None, False)
		self._as.register_preempt_callback(self._preempt_cb)
		self._as.register_goal_callback(self._goal_cb)

		self._rp = RosPack()
		self._behavior_lib = BehaviorLibrary()

		# start action server after all member variables have been initialized
		self._as.start()

		rospy.loginfo("%d behaviors available, ready for start request." % self._behavior_lib.count_behaviors())


	def _goal_cb(self):
		if not self._as.is_new_goal_available():
			return
		goal = self._as.accept_new_goal()
		rospy.loginfo('Received a new request to start behavior: %s' % goal.behavior_name)
		be_id, behavior = self._behavior_lib.find_behavior(goal.behavior_name)
		if be_id is None:
			Logger.logerr("Did not find behavior with requested name: %s" % goal.behavior_name)
			self._as.set_preempted()
			return

		be_selection = BehaviorSelection()
		be_selection.behavior_id = be_id
		be_selection.autonomy_level = 255
		try:
			for k, v in zip(goal.arg_keys, goal.arg_values):
				if v.startswith('file://'):
					v = v.replace('file://', '', 1)
					path = v.split(':')[0]
					if len(v.split(':')) > 1:
						ns = v.split(':')[1]
					else:
						ns = ''
					if path.startswith('~') or path.startswith('/'):
						filepath = os.path.expanduser(path)
					else:
						filepath = os.path.join(self._rp.get_path(path.split('/')[0]), '/'.join(path.split('/')[1:]))
					with open(filepath, 'r') as f:
						content = f.read()
					if ns != '':
						content = yaml.load(content)
						if ns in content:
							content = content[ns]
						content = yaml.dump(content)
					be_selection.arg_keys.append(k)
					be_selection.arg_values.append(content)
				else:
					be_selection.arg_keys.append(k)
					be_selection.arg_values.append(v)
		except Exception as e:
			rospy.logwarn('Failed to parse and substitute behavior arguments, will use direct input.\n%s' % str(e))
			be_selection.arg_keys = goal.arg_keys
			be_selection.arg_values = goal.arg_values
		be_selection.input_keys = goal.input_keys
		be_selection.input_values = goal.input_values

		# check for local modifications of the behavior to send them to the onboard behavior
		be_filepath_new = self._behavior_lib.get_sourcecode_filepath(be_id)
		with open(be_filepath_new, "r") as f:
			be_content_new = f.read()

		be_filepath_old = self._behavior_lib.get_sourcecode_filepath(be_id, add_tmp=True)
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


	def _preempt_cb(self):
		self._preempt_pub.publish()
		rospy.loginfo('Behavior execution preempt requested!')


	def _status_cb(self, msg):
		if not self._behavior_started and msg.code == BEStatus.STARTED:
			self._behavior_started = True
			self._active_behavior_id = msg.behavior_id
			rospy.loginfo('Behavior execution has started!')
		# Ignore status until behavior start was received
		if not self._behavior_started:
			return

		if msg.behavior_id != self._active_behavior_id:
			rospy.logwarn('Ignored status because behavior id differed ({} vs {})!'.format(msg.behavior_id, self._active_behavior_id))
			return

		if msg.code == BEStatus.ERROR:
			rospy.logerr('Failed to run behavior! Check onboard terminal for further infos.')
			self._as.set_aborted('')
		elif msg.code == BEStatus.FINISHED:
			result = msg.args[0] if len(msg.args) >= 1 else ''
			rospy.loginfo('Finished behavior execution with result "%s"!' % result)
			self._as.set_succeeded(BehaviorExecutionResult(outcome=result))
		elif msg.code == BEStatus.FAILED:
			rospy.logerr('Behavior execution failed in state %s!' % str(self._current_state))
			self._as.set_aborted('')

		# Start new goal if available and current is not active anymore
		if not self._as.is_active() and self._as.is_new_goal_available():
			self._goal_cb()


	def _state_cb(self, msg):
		self._current_state = msg.data
		if self._as.is_active():
			self._as.publish_feedback(BehaviorExecutionFeedback(self._current_state))
		rospy.loginfo('Current state: %s' % self._current_state)
