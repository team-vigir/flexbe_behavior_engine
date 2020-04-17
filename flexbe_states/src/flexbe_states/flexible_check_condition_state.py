#!/usr/bin/env python
from flexbe_core import EventState, Logger


class FlexibleCheckConditionState(EventState):
    '''
    Implements a state that checks if the given condition is true based on multiple userdata inputs
    provided as a list to the calculation function and returns the corresponding outcome.
    This state can be used if the further control flow of the behavior depends on a simple condition.

    -- predicate    function    The condition whose truth value will be evaluated.
                                It could be a private function (self.foo) manually defined in a behavior's source code
                                or a lambda function (e.g., lambda x: x[0]^2 + x[1]^2).
    -- input_keys   string[]    List of available input keys.

    ># input_keys   object[]    Input(s) to the calculation function as a list of userdata.
                                The individual inputs can be accessed as list elements (see lambda expression example).

    <= true                     Returned if the condition evaluates to True
    <= false                    Returned if the condition evaluates to False
    '''

    def __init__(self, predicate, input_keys):
        '''Constructor'''
        super(FlexibleCheckConditionState, self).__init__(outcomes=['true', 'false'],
                                                          input_keys=input_keys)
        self._predicate = predicate
        self._outcome = 'false'

    def execute(self, userdata):
        return self._outcome

    def on_enter(self, userdata):
        if self._predicate is not None:
            try:
                self._outcome = "true" if self._predicate([userdata[key] for key in self._input_keys]) else 'false'
            except Exception as e:
                Logger.logwarn('Failed to execute condition function!\n%s' % str(e))
        else:
            Logger.logwarn('Passed no predicate!')
