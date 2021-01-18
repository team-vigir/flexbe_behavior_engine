#!/usr/bin/env python
# -*- coding: utf-8 -*-

#
# Content is only intended for the flexbe_testing self-test
#

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, EventState, Logger


class SelftestBehaviorSM(Behavior):
	''' Simple behavior for the flexbe_testing self-test of behaviors. '''

	def __init__(self):
		super(SelftestBehaviorSM, self).__init__()
		self.name = 'Selftest Behavior'
		self.add_parameter('value', 'wrong')

	def create(self):
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['data'], output_keys=['result'])
		_state_machine.userdata.data = None
		_state_machine.userdata.result = None

		with _state_machine:
			OperatableStateMachine.add('Modify Data',
										SelftestBehaviorSM._CalculationState(calculation=lambda x: x * 2),
										transitions={'done': 'Decide Param'},
										autonomy={'done': Autonomy.Off},
										remapping={'input_value': 'data', 'output_value': 'result'})
			OperatableStateMachine.add('Decide Param',
										SelftestBehaviorSM._DecisionState(outcomes=['finished', 'failed'], conditions=lambda x: 'finished' if self.value == 'correct' else 'failed'),
										transitions={'finished': 'finished', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'input_value': 'data'})
		return _state_machine

	class _CalculationState(EventState):
		''' Copy of the flexbe_states.CalculationState for use in the test behavior. '''

		def __init__(self, calculation):
			super(SelftestBehaviorSM._CalculationState, self).__init__(outcomes=['done'], input_keys=['input_value'], output_keys=['output_value'])
			self._calculation = calculation
			self._calculation_result = None

		def execute(self, userdata):
			userdata.output_value = self._calculation_result
			return 'done'

		def on_enter(self, userdata):
			if self._calculation is not None:
				try:
					self._calculation_result = self._calculation(userdata.input_value)
				except Exception as e:
					Logger.logwarn('Failed to execute calculation function!\n%s' % str(e))
			else:
				Logger.logwarn('Passed no calculation!')

	class _DecisionState(EventState):
		''' Copy of the flexbe_states.DecisionState for use in the test behavior. '''

		def __init__(self, outcomes, conditions):
			super(SelftestBehaviorSM._DecisionState, self).__init__(outcomes=outcomes, input_keys=['input_value'])
			self._conditions = conditions

		def execute(self, userdata):
			if self._conditions is not None:
				outcome = None
				try:
					outcome = str(self._conditions(userdata.input_value))
				except Exception as e:
					Logger.logwarn('Passed no function as predicate!\n%s' % str(e))
					outcome = None
				if outcome is not None and outcome in self._outcomes:
					return outcome
