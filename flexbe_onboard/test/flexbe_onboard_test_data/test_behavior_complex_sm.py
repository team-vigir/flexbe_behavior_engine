#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_INVALID import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from flexbe_states.wait_state import WaitState
from flexbe_states.decision_state import DecisionState
from flexbe_states.log_state import LogState as flexbe_states__LogState
from flexbe_states.calculation_state import CalculationState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]
raise ValueError("TODO: Remove!")
# [/MANUAL_IMPORT]


'''
Created on Mon Mar 30 2020
@author: Philipp Schillinger
'''
class TestBehaviorComplexSM(Behavior):
    '''
    A more complex behavior for testing the onboard engine.
Note: This behavior contains intentional errors that are fixed by sent modifications.
    '''


    def __init__(self):
        super(TestBehaviorComplexSM, self).__init__()
        self.name = 'Test Behavior Complex'

        # parameters of this behavior
        self.add_parameter('param', 'value_1')

        # references to used behaviors

        # Additional initialization code can be added inside the following tags
        # [MANUAL_INIT]
        
        # [/MANUAL_INIT]

        # Behavior comments:



    def create(self):
        # x:67 y:463, x:336 y:160
        _state_machine = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['data'])
        _state_machine.userdata.data = 0

        # Additional creation code can be added inside the following tags
        # [MANUAL_CREATE]
        
        # [/MANUAL_CREATE]


        with _state_machine:
            # x:30 y:40
            OperatableStateMachine.add('Wait',
                                        WaitState(wait_time=0.5),
                                        transitions={'done': 'Calculate'},
                                        autonomy={'done': Autonomy.Off})

            # x:36 y:240
            OperatableStateMachine.add('Log Param',
                                        flexbe_states__LogState(text=self.param, severity=2),
                                        transitions={'done': 'Verify Input'},
                                        autonomy={'done': Autonomy.Off})

            # x:32 y:340
            OperatableStateMachine.add('Verify Input',
                                        DecisionState(outcomes=['accepted', 'rejected'], conditions=lambda x: 'accepted' if x > 3 else 'rejected'),
                                        transitions={'accepted': 'finished', 'rejected': 'failed'},
                                        autonomy={'accepted': Autonomy.Off, 'rejected': Autonomy.Off},
                                        remapping={'input_value': 'data'})

            # x:28 y:136
            OperatableStateMachine.add('Calculate',
                                        CalculationState(calculation=self._calculate),
                                        transitions={'done': 'Log Param'},
                                        autonomy={'done': Autonomy.Off},
                                        remapping={'input_value': 'data', 'output_value': 'data'})


        return _state_machine


    # Private functions can be added inside the following tags
    # [MANUAL_FUNC]
    def _calculate(self, data):
        return data**2
    # [/MANUAL_FUNC]
