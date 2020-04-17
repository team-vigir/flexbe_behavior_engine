#!/usr/bin/env python
from flexbe_core import EventState, Logger


class CheckConditionState(EventState):
    '''
    Checks if the given condition is true and returns the corresponding outcome.
    This state can be used if the further control flow of the behavior depends on a simple condition.

    -- predicate    function	The condition whose truth value will be evaluated.
                                Has to expect one parameter which will be set to input_value and return a boolean.

    ># input_value	object		Input to the predicate function.

    <= true 					Returned if the condition evaluates to True
    <= false 					Returned if the condition evaluates to False
    '''

    def __init__(self, predicate):
        super(CheckConditionState, self).__init__(outcomes=['true', 'false'],
                                                  input_keys=['input_value'])
        self._predicate = predicate
        self._outcome = 'false'

    def execute(self, userdata):
        return self._outcome

    def on_enter(self, userdata):
        try:
            self._outcome = 'true' if self._predicate(userdata.input_value) else 'false'
        except Exception as e:
            Logger.logwarn('Failed to execute condition function!\n%s' % str(e))
