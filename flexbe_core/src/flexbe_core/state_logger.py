#!/usr/bin/env python
import rospy
import os
import time
import logging

'''
Created on 02/18/2015

@author: Philipp Schillinger
'''
class StateLogger(object):
	'''
	Realizes logging of active states.
	'''

	LOG_FOLDER = "~/logs/"


	@staticmethod
	def initialize(be_name = None):
		log_folder = os.path.expanduser(StateLogger.LOG_FOLDER)
		if not os.path.exists(log_folder):
			os.makedirs(log_folder)

		name = "states"
		if be_name is not None:
			name = be_name.replace(" ", "_").replace(",", "_").replace(".", "_").replace("/", "_").lower()

		filename = os.path.join(log_folder, name + "_" + time.strftime("%Y-%m-%d-%H_%M_%S") + ".log")

		logger = logging.getLogger('state_logger')
		handler = logging.FileHandler(filename)
		formatter = logging.Formatter('%(message)s')
		handler.setFormatter(formatter)
		logger.addHandler(handler)

		# log starting time for being able to calculate execution time of first state
		logger.info(str(rospy.get_time()) + "," + be_name + ",INIT,INIT,1,1")

		StateLogger._logger = logger
		StateLogger._handler = handler


	@staticmethod
	def log_state_execution(statepath, stateclass, outcome, is_autonomous, is_executed):
		"""
		Logs the execution of a state.
		Should be called once when the state returns an outcome.
		"""
		StateLogger._logger.info(
			str(rospy.get_time()) + ","					# timestamp in ros time
			+ statepath + ","							# path of the executed state
			+ stateclass + ","							# class of the executed state
			+ outcome + ","								# resulting outcome
			+ ('1' if is_autonomous else '0') + ","		# Outcome triggered by behavior or operator?
			+ ('1' if is_executed else '0')				# Outcome actually executed or only requested?
		)


	@staticmethod
	def shutdown():
		StateLogger._handler.close()
		StateLogger._logger.removeHandler(StateLogger._handler)



