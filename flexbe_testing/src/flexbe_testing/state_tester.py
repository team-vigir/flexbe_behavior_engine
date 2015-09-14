#!/usr/bin/env python
import inspect
import rospkg
import rospy
import os
import rosbag
import smach

from flexbe_core import OperatableStateMachine, EventState
from flexbe_core.core.loopback_state import LoopbackState


class StateTester(object):

	def __init__(self):
		self._counter = 0
		self._rp = rospkg.RosPack()


	def run_test(self, name, config):
		self._counter += 1
		print '\033[34;1m#%2d %s \033[0m\033[34m(%s > %s)\033[0m' % (self._counter, name, config['class'], config['outcome'])

		# import state
		state = None
		try:
			package = __import__(config['path'], fromlist=[config['path']])
			clsmembers = inspect.getmembers(package, lambda member: inspect.isclass(member) and member.__module__ == package.__name__)
			StateClass = next(c for n,c in clsmembers if n == config['class'])
			if config.has_key('params'):
				for key, value in config['params'].iteritems():
					if isinstance(value, basestring) and value.startswith('lambda '):
						config['params'][key] = eval(value)
				state = StateClass(**config['params'])
			else:
				state = StateClass()
			print '\033[1m+++\033[0m state imported'
		except Exception as e:
			print '\033[31;1m>>>\033[0m\033[31m unable to import state %s (%s):\n\t%s\033[0m' % (config['class'], config['path'], str(e))
			return 0

		# prepare rosbag if available
		bag = None
		if config.has_key('data'):
			bagpath = ''
			if config['data'].startswith('~') or config['data'].startswith('/'):
				bagpath = os.path.expanduser(config['data'])
			else:
				try:
					bagpath = os.path.join(self._rp.get_path(config['data'].split('/')[0]), '/'.join(config['data'].split('/')[1:]))
				except Exception as e:
					print '\033[31;1m>>>\033[0m\033[31m unable to get input bagfile %s:\n\t%s\033[0m' % (config['data'], str(e))
					return 0
			bag = rosbag.Bag(bagpath)
			print '\033[1m+++\033[0m using data source: %s' % bagpath

		# set input values
		userdata = smach.UserData()
		if config.has_key('input'):
			for input_key, input_value in config['input'].iteritems():
				if isinstance(input_value, basestring) and len(input_value) > 1 and input_value[0] == '/' and input_value[1] != '/' and bag is not None:
					(_, input_value, _) = list(bag.read_messages(topics=[input_value]))[0]
				elif isinstance(input_value, basestring) and input_value.startswith('lambda '):
					input_value = eval(input_value)
				userdata[input_key] = input_value

		# set output values
		expected = dict()
		if config.has_key('output'):
			for output_key, output_value in config['output'].iteritems():
				if isinstance(output_value, basestring) and len(output_value) > 1 and output_value[0] == '/' and output_value[1] != '/' and bag is not None:
					(_, output_value, _) = bag.read_messages(topics=[output_value])[0]
				expected[output_key] = output_value

		# execute state
		outcome = LoopbackState._loopback_name
		while outcome == LoopbackState._loopback_name and not rospy.is_shutdown():
			outcome = state.execute(userdata)

		# evaluate output
		output_ok = True
		for expected_key, expected_value in expected.iteritems():
			if expected_key in userdata.keys():
				if expected_value != userdata[expected_key]:
					print '\033[1m---\033[0m wrong result for %s: %s != %s' % (expected_key, userdata[expected_key], expected_value)
					output_ok = False
		if len(expected) > 0 and output_ok:
			print '\033[1m+++\033[0m all result outputs match expected'

		# evaluate outcome
		outcome_ok = outcome == config['outcome']
		if outcome_ok:
			print '\033[1m+++\033[0m correctly returned outcome %s' % outcome
		else:
			print '\033[1m---\033[0m wrong outcome: %s' % outcome

		# report result
		if outcome_ok and output_ok:
			print '\033[32;1m>>>\033[0m\033[32m %s completed!\033[0m' % name
			return 1
		else:
			print '\033[31;1m>>>\033[0m\033[31m %s failed!\033[0m' % name
			return 0
