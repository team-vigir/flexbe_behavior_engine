#!/usr/bin/env python
import inspect
import rospkg
import rospy
import os
import re
import rosbag
import smach
import roslaunch
import unittest
import rosunit
import traceback

from flexbe_core.core.loopback_state import LoopbackState


class StateTester(object):

	def __init__(self):
		self._counter = 0
		self._rp = rospkg.RosPack()
		self._evaluation_tests = dict()

		self._run_id = rospy.get_param('/run_id')

		self._print_debug_positive = rospy.get_param('~print_debug_positive', True)
		self._print_debug_negative = rospy.get_param('~print_debug_negative', True)
		self._mute_info = rospy.get_param('~mute_info', False)
		self._mute_warn = rospy.get_param('~mute_warn', False)
		self._mute_error = rospy.get_param('~mute_error', False)
		self._compact_format = rospy.get_param('~compact_format', False)

		if self._compact_format:
			self._print_debug_positive = False
			self._print_debug_negative = True
			self._mute_info = True
			self._mute_warn = True
			self._mute_error = False


	def run_test(self, name, config):
		if self._mute_info:
			rospy.loginfo = rospy.logdebug
		if self._mute_warn:
			rospy.logwarn = rospy.logdebug
		if self._mute_error:
			rospy.logerror = rospy.logdebug

		if not self._compact_format: print ''
		self._counter += 1

		import_only = config.get('import_only', False)
		print '\033[34;1m#%2d %s \033[0m\033[34m(%s%s)\033[0m' % (self._counter, name, config['class'], ' > %s' % config['outcome'] if not import_only else '')
		prefix = '>>>' if not self._compact_format else '  >'

		# load and start launch file
		if not import_only and config.has_key('launch'):
			launchpath = None
			launchcontent = None
			if config['launch'].startswith('~') or config['launch'].startswith('/'):
				launchpath = os.path.expanduser(config['launch'])
			elif re.match(r'.+\.launch$', config['launch']):
				launchpath = os.path.join(self._rp.get_path(config['launch'].split('/')[0]), '/'.join(config['launch'].split('/')[1:]))
			else:
				launchcontent = config['launch']
			launchconfig = roslaunch.config.ROSLaunchConfig()
			loader = roslaunch.xmlloader.XmlLoader()
			if launchpath is not None:
				loader.load(launchpath, launchconfig, verbose = False)
			else:
				loader.load_string(launchcontent, launchconfig, verbose = False)
			launchrunner = roslaunch.launch.ROSLaunchRunner(self._run_id, launchconfig)

			#print '\033[35m'
			run = launchrunner.launch()
			launchrunner.spin_once()
			if self._print_debug_positive: print '\033[0m\033[1m  +\033[0m launchfile running'

			if config.has_key('wait_cond'):
				try:
					check_running_rate = rospy.Rate(10)
					is_running = False
					while not is_running:
						is_running = eval(config['wait_cond'])
						check_running_rate.sleep()
					if self._print_debug_positive: print '\033[0m\033[1m  +\033[0m waiting condition satisfied'
				except Exception as e:
					print '\033[31;1m%s\033[0m\033[31m unable to check waiting condition:\n\t%s\033[0m' % (prefix, str(e))
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
					print '\033[31;1m%s\033[0m\033[31m unable to get input bagfile %s:\n\t%s\033[0m' % (prefix, config['data'], str(e))
					return 0
			bag = rosbag.Bag(bagpath)
			if self._print_debug_positive: print '\033[1m  +\033[0m using data source: %s' % bagpath

		# import state
		try:
			package = __import__(config['path'], fromlist=[config['path']])
			clsmembers = inspect.getmembers(package, lambda member: inspect.isclass(member) and member.__module__ == package.__name__)
			StateClass = next(c for n,c in clsmembers if n == config['class'])
		except Exception as e:
			print '\033[31;1m%s\033[0m\033[31m unable to import state %s (%s):\n\t%s\033[0m' % (prefix, config['class'], config['path'], str(e))
			traceback.print_exc()
			return 0
		if self._print_debug_positive: print '\033[1m  +\033[0m state imported'

		if not import_only:
			# prepare parameters
			params = None
			if config.has_key('params'):
				for key, value in config['params'].iteritems():
					try:
						config['params'][key] = self._parse_data_value(value, bag)
					except Exception as e:
						if not self._compact_format:
							print '\033[33;1m  >\033[0m\033[33m unable to get message from topic %s: ignoring replacement...\033[0m' % (str(value))
				params = config['params']

			# instatiate state
			state = None
			try:
				if params is None:
					state = StateClass()
				else:
					state = StateClass(**params)
			except Exception as e:
				print '\033[31;1m%s\033[0m\033[31m unable to instantiate state %s (%s) with params:\n\t%s\n\t%s\033[0m' % (prefix, config['class'], config['path'], str(params), str(e))
				traceback.print_exc()
				return 0
			if self._print_debug_positive: print '\033[1m  +\033[0m state instantiated'

			# set input values
			userdata = smach.UserData()
			if config.has_key('input'):
				for input_key, input_value in config['input'].iteritems():
					userdata[input_key] = self._parse_data_value(input_value, bag)

			# set output values
			expected = dict()
			if config.has_key('output'):
				for output_key, output_value in config['output'].iteritems():
					expected[output_key] = self._parse_data_value(output_value, bag)

			# execute state
			try:
				state.on_start()
				outcome = LoopbackState._loopback_name
				while outcome == LoopbackState._loopback_name and not rospy.is_shutdown():
					outcome = state.execute(userdata)
					if config.has_key('launch'):
						launchrunner.spin_once()
				state.on_stop()
			except Exception as e:
				print '\033[31;1m%s\033[0m\033[31m failed to execute state %s (%s)\n\t%s\033[0m' % (prefix, config['class'], config['path'], str(e))
				traceback.print_exc()
				return 0

			if config.has_key('launch'):
				#print '\033[35m'
				launchrunner.stop()
				if self._print_debug_positive: print '\033[0m\033[1m  +\033[0m launchfile stopped'

			# evaluate output
			output_ok = True
			for expected_key, expected_value in expected.iteritems():
				if expected_key in userdata.keys():
					equals = userdata[expected_key] == expected_value
					self._evaluation_tests['test_%s_output_%s' % (name.split('.')[0], expected_key)] = \
						self._test_output(userdata[expected_key], expected_value)
					if not equals:
						if self._print_debug_negative: print '\033[1m  -\033[0m wrong result for %s: %s != %s' % (expected_key, userdata[expected_key], expected_value)
						output_ok = False
				else:
					if self._print_debug_negative: print '\033[1m  -\033[0m no result for %s' % expected_key
					output_ok = False
			if len(expected) > 0 and output_ok:
				if self._print_debug_positive: print '\033[1m  +\033[0m all result outputs match expected'

			# evaluate outcome
			outcome_ok = outcome == config['outcome']
			self._evaluation_tests['test_%s_outcome' % name.split('.')[0]] = self._test_outcome(outcome, config['outcome'])
			if outcome_ok:
				if self._print_debug_positive: print '\033[1m  +\033[0m correctly returned outcome %s' % outcome
			else:
				if self._print_debug_negative: print '\033[1m  -\033[0m wrong outcome: %s' % outcome
			
		# report result
		if import_only or outcome_ok and output_ok:
			print '\033[32;1m%s\033[0m\033[32m %s completed!\033[0m' % (prefix, name)
			self._evaluation_tests['test_%s_pass' % name.split('.')[0]] = self._test_pass(True)
			return 1
		else:
			print '\033[31;1m%s\033[0m\033[31m %s failed!\033[0m' % (prefix, name)
			self._evaluation_tests['test_%s_pass' % name.split('.')[0]] = self._test_pass(False)
			return 0

	def perform_rostest(self, test_pkg):
		TestCase = type(test_pkg + '_test_class', (unittest.TestCase,), self._evaluation_tests)
		rosunit.unitrun(test_pkg, test_pkg + '_flexbe_tests', TestCase)


	def _parse_data_value(self, data_value, bag):
		# message data
		if isinstance(data_value, basestring) and len(data_value) > 1 and data_value[0] == '/' and data_value[1] != '/' and bag is not None:
			(_, data_value, _) = list(bag.read_messages(topics=[data_value]))[0]

		# anonymous function
		elif isinstance(data_value, basestring) and data_value.startswith('lambda '):
			data_value = eval(data_value)

		# None
		elif data_value == 'None':
			data_value = None

		# escaped backslash at the beginning
		elif isinstance(data_value, basestring) and len(data_value) > 1 and data_value[0] == '/' and data_value[1] == '/':
			data_value = data_value[1:]

		return data_value


	# ROSUNIT callbacks

	def _test_output(self, value, expected):
		def _test_call(test_self):
			test_self.assertEquals(value, expected, "Output value %s does not match expected %s" % (value, expected))
		return _test_call

	def _test_outcome(self, outcome, expected):
		def _test_call(test_self):
			test_self.assertEquals(outcome, expected, "Outcome %s does not match expected %s" % (outcome, expected))
		return _test_call

	def _test_pass(self, passed):
		def _test_call(test_self):
			test_self.assertTrue(passed, "State did not pass flexbe tests.")
		return _test_call
