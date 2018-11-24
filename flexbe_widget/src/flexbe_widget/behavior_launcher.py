#!/usr/bin/env python

import rospy
from flexbe_msgs.msg import *
from rospkg import RosPack, ResourceNotFound

from flexbe_core import Logger, BehaviorLibrary
from std_msgs.msg import String

import pickle
import zlib
import difflib
import os
import yaml
import xml.etree.ElementTree as ET

class BehaviorLauncher(object):

	MIN_VERSION = '2.0.0'

	def __init__(self):
		Logger.initialize()

		self._sub = rospy.Subscriber("flexbe/request_behavior", BehaviorRequest, self._callback)
		self._version_sub = rospy.Subscriber("flexbe/ui_version", String, self._version_callback)

		self._pub = rospy.Publisher("flexbe/start_behavior", BehaviorSelection, queue_size=100)
		self._status_pub = rospy.Publisher("flexbe/status", BEStatus, queue_size=100)
		self._mirror_pub = rospy.Publisher("flexbe/mirror/structure", ContainerStructure, queue_size=100)

		self._rp = RosPack()
		self._behavior_lib = BehaviorLibrary()

		rospy.loginfo("%d behaviors available, ready for start request." % self._behavior_lib.count_behaviors())


	def _callback(self, msg):
		be_id, behavior = self._behavior_lib.find_behavior(msg.behavior_name)
		if be_id is None:
			Logger.logerr("Did not find behavior with requested name: %s" % msg.behavior_name)
			self._status_pub.publish(BEStatus(code=BEStatus.ERROR))
			return

		rospy.loginfo("Request for behavior " + behavior["name"])

		be_selection = BehaviorSelection()
		be_selection.behavior_id = be_id
		be_selection.autonomy_level = msg.autonomy_level
		try:
			for k, v in zip(msg.arg_keys, msg.arg_values):
				if k.startswith('/YAML:'):
					key = k.replace('/YAML:', '/', 1)
					path = v.split(':')[0]
					ns = v.split(':')[1]
					if path.startswith('~') or path.startswith('/'):
						yamlpath = os.path.expanduser(path)
					else:
						yamlpath = os.path.join(self._rp.get_path(path.split('/')[0]), '/'.join(path.split('/')[1:]))
					with open(yamlpath, 'r') as f:
						content = yaml.load(f)
					if ns != '' and ns in content:
						content = content[ns]
					be_selection.arg_keys.append(key)
					be_selection.arg_values.append(yaml.dump(content))
				else:
					be_selection.arg_keys.append(k)
					be_selection.arg_values.append(v)
		except Exception as e:
			rospy.logwarn('Failed to parse and substitute behavior arguments, will use direct input.\n%s' % str(e))
			be_selection.arg_keys = msg.arg_keys
			be_selection.arg_values = msg.arg_values

		be_structure = ContainerStructure()
		be_structure.containers = msg.structure

		try:
			be_filepath_new = os.path.join(self._rp.get_path(behavior["package"]), 'src/' + behavior["package"] + '/' + behavior["file"] + '.py')
		except ResourceNotFound:
			rospy.logerr("Could not find behavior package '%s'" % (behavior["package"]))
			rospy.loginfo("Have you updated your ROS_PACKAGE_PATH after creating the behavior?")
			return

		with open(be_filepath_new, "r") as f:
			be_content_new = f.read()

		be_filepath_old = os.path.join(self._rp.get_path(behavior["package"]), 'src/' + behavior["package"] + '/' + behavior["file"] + '_tmp.py')
		if not os.path.isfile(be_filepath_old):
			be_selection.behavior_checksum = zlib.adler32(be_content_new)
			if msg.autonomy_level != 255:
				be_structure.behavior_id = be_selection.behavior_checksum
				self._mirror_pub.publish(be_structure)
			self._pub.publish(be_selection)
			rospy.loginfo("No changes to behavior version.")
			return

		with open(be_filepath_old, "r") as f:
			be_content_old = f.read()

		sqm = difflib.SequenceMatcher(a=be_content_old, b=be_content_new)
		diffs = [x[1] for x in sqm.get_grouped_opcodes(0)]
		for opcode, a0, a1, b0, b1 in diffs:
			content = be_content_new[b0:b1]
			be_selection.modifications.append(BehaviorModification(a0, a1, content))

		be_selection.behavior_checksum = zlib.adler32(be_content_new)
		if msg.autonomy_level != 255:
			be_structure.behavior_id = be_selection.behavior_checksum
			self._mirror_pub.publish(be_structure)

		self._pub.publish(be_selection)

	def _version_callback(self, msg):
		vui = self._parse_version(msg.data)
		vex = self._parse_version(BehaviorLauncher.MIN_VERSION)
		if vui < vex:
			Logger.logwarn('FlexBE App needs to be updated!\n' \
				+ 'Require at least version %s, but have %s\n' % (BehaviorLauncher.MIN_VERSION, msg.data) \
				+ 'Please run a "git pull" in "roscd flexbe_app".')

	def _parse_version(self, v):
		result = 0
		offset = 1
		for n in reversed(v.split('.')):
			result += int(n) * offset
			offset *= 100
		return result
