#!/usr/bin/env python
import unittest
import rospy

from flexbe_core import EventState, OperatableStateMachine
from flexbe_core.core.exceptions import StateError, StateMachineError, UserDataError


class TestExceptions(unittest.TestCase):

    def test_invalid_outcome(self):
        class ReturnInvalidOutcome(EventState):

            def __init__(self):
                super(ReturnInvalidOutcome, self).__init__(outcomes=['done'])

            def execute(self, userdata):
                return 'invalid'

        sm = OperatableStateMachine(outcomes=['done'])
        with sm:
            OperatableStateMachine.add('state', ReturnInvalidOutcome(), transitions={'done': 'done'})

        outcome = sm.execute(None)
        self.assertIsNone(outcome)
        self.assertIsInstance(sm._last_exception, StateError)

    def test_invalid_transition(self):
        class ReturnDone(EventState):

            def __init__(self):
                super(ReturnDone, self).__init__(outcomes=['done'])

            def execute(self, userdata):
                return 'done'

        inner_sm = OperatableStateMachine(outcomes=['done'])
        with inner_sm:
            OperatableStateMachine.add('state', ReturnDone(), transitions={'done': 'invalid'})
        sm = OperatableStateMachine(outcomes=['done'])
        with sm:
            OperatableStateMachine.add('inner', inner_sm, transitions={'done': 'done'})

        outcome = sm.execute(None)
        self.assertIsNone(outcome)
        self.assertIsInstance(sm._last_exception, StateMachineError)

    def test_invalid_userdata_input(self):
        class AccessInvalidInput(EventState):

            def __init__(self):
                super(AccessInvalidInput, self).__init__(outcomes=['done'], input_keys=['input'])

            def execute(self, userdata):
                print(userdata.invalid)
                return 'done'

        sm = OperatableStateMachine(outcomes=['done'])
        with sm:
            OperatableStateMachine.add('state', AccessInvalidInput(), transitions={'done': 'done'})

        outcome = sm.execute(None)
        self.assertIsNone(outcome)
        self.assertIsInstance(sm._last_exception, UserDataError)

    def test_invalid_userdata_output(self):
        class SetInvalidOutput(EventState):

            def __init__(self):
                super(SetInvalidOutput, self).__init__(outcomes=['done'], output_keys=['output'])

            def execute(self, userdata):
                userdata.invalid = False
                return 'done'

        sm = OperatableStateMachine(outcomes=['done'])
        with sm:
            OperatableStateMachine.add('state', SetInvalidOutput(), transitions={'done': 'done'})

        outcome = sm.execute(None)
        self.assertIsNone(outcome)
        self.assertIsInstance(sm._last_exception, UserDataError)

    def test_missing_userdata(self):
        class AccessValidInput(EventState):

            def __init__(self):
                super(AccessValidInput, self).__init__(outcomes=['done'], input_keys=['missing'])

            def execute(self, userdata):
                print(userdata.missing)
                return 'done'

        sm = OperatableStateMachine(outcomes=['done'])
        with sm:
            OperatableStateMachine.add('state', AccessValidInput(), transitions={'done': 'done'})

        outcome = sm.execute(None)
        self.assertIsNone(outcome)
        self.assertIsInstance(sm._last_exception, UserDataError)

    def test_modify_input_key(self):
        class ModifyInputKey(EventState):

            def __init__(self):
                super(ModifyInputKey, self).__init__(outcomes=['done'], input_keys=['only_input'])

            def execute(self, userdata):
                userdata.only_input['new'] = 'not_allowed'
                return 'done'

        sm = OperatableStateMachine(outcomes=['done'])
        sm.userdata.only_input = {'existing': 'is_allowed'}
        with sm:
            OperatableStateMachine.add('state', ModifyInputKey(), transitions={'done': 'done'})

        outcome = sm.execute(None)
        self.assertIsNone(outcome)
        self.assertIsInstance(sm._last_exception, UserDataError)


if __name__ == '__main__':
    rospy.init_node('test_flexbe_exceptions')
    import rostest
    rostest.rosrun('flexbe_core', 'test_flexbe_exceptions', TestExceptions)
