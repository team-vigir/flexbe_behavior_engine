#!/usr/bin/env python
from flexbe_core import EventState, Logger


class OperatorDecisionState(EventState):
    '''
    Implements a state where the operator has to manually choose an outcome.
    Autonomy Level of all outcomes should be set to Full,
    because this state is not able to choose an outcome on its own.
    Only exception is the suggested outcome, which will be returned immediately by default.
    This state can be used to create alternative execution paths
    by setting the suggestion to High autonomy instead of Full.

    -- outcomes     string[]    A list of all possible outcomes of this state.
    -- hint         string      Text displayed to the operator to give instructions how to decide.
    -- suggestion   string      The outcome which is suggested.
                                Will be returned if the level of autonomy is high enough.
    '''

    def __init__(self, outcomes, hint=None, suggestion=None):
        super(OperatorDecisionState, self).__init__(outcomes=outcomes)
        self._hint = hint
        self._suggestion = suggestion

    def execute(self, userdata):
        if self._suggestion is not None and self._suggestion in self._outcomes:
            return self._suggestion

    def on_enter(self, userdata):
        if self._hint is not None:
            Logger.loghint(self._hint)
