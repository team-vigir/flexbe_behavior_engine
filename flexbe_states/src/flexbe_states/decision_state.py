#!/usr/bin/env python

import rospy
from flexbe_core import EventState, Logger
from rospy.exceptions import ROSInterruptException

'''
Created on 11.06.2013

@author: Philipp Schillinger
'''

class DecisionState(EventState):
	'''
	Evaluates a condition function in order to return one of the specified outcomes.
	This state can be used if the further control flow of the behavior depends on an advanced condition.

	-- outcomes 	string[]	A list containing all possible outcomes of this state
	-- conditions 	function	Implements the condition check and returns one of the available outcomes.
								Has to expect one parameter which will be set to input_value.

	># input_value	object		Input to the condition function.

	'''


	def __init__(self, outcomes, conditions):
		'''
		Constructor
		'''
		super(DecisionState, self).__init__(outcomes=outcomes,
											input_keys=['input_value'])
		
		self._conditions = conditions
		self._my_outcomes = outcomes
		
		
	def execute(self, userdata):
		'''
		Execute this state
		'''
		
		if self._conditions is not None:
			outcome = DecisionState._loopback_name
			try:
				outcome = str(self._conditions(userdata.input_value))
			except Exception as e:
				Logger.logwarn('Passed no function as predicate!\n%s' % str(e))
				outcome = DecisionState._loopback_name
			if outcome is not None:
				return outcome
			
	
	def on_enter(self, userdata):
		try:
			rospy.sleep(0.2) # TODO: check why this has been added
		except ROSInterruptException:
			rospy.logwarn('Skipped sleep.')
