#!/usr/bin/env python
from flexbe_core import EventState, Logger


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
