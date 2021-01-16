#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from flexbe_testing.calculation_state import CalculationState
from flexbe_testing.decision_state import DecisionState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Fri Apr 17 2020
@author: Philipp Schillinger
'''
class SelftestBehaviorSM(Behavior):
	'''
	Simple behavior for the flexbe_testing self-test of behaviors.
	'''


	def __init__(self):
		super(SelftestBehaviorSM, self).__init__()
		self.name = 'Selftest Behavior'

		# parameters of this behavior
		self.add_parameter('value', 'wrong')

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:30 y:365, x:130 y:365
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['data'], output_keys=['result'])
		_state_machine.userdata.data = None
		_state_machine.userdata.result = None

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:40 y:73
			OperatableStateMachine.add('Modify Data',
										CalculationState(calculation=lambda x: x * 2),
										transitions={'done': 'Decide Param'},
										autonomy={'done': Autonomy.Off},
										remapping={'input_value': 'data', 'output_value': 'result'})

			# x:37 y:201
			OperatableStateMachine.add('Decide Param',
										DecisionState(outcomes=['finished', 'failed'], conditions=lambda x: 'finished' if self.value == 'correct' else 'failed'),
										transitions={'finished': 'finished', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'input_value': 'data'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
