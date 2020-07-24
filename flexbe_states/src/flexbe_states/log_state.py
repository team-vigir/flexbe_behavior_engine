#!/usr/bin/env python
from flexbe_core import EventState, Logger


class LogState(EventState):
    '''
    A state that can log a predefined message to precisely inform the operator
    about what happened to the behavior.

    -- text  	    string  The message to be logged to the terminal.
    -- severity     uint8  	Type of logging (Logger.REPORT_INFO / WARN / HINT / ERROR)

    <= done				    Indicates that the message has been logged.
    '''

    def __init__(self, text, severity=Logger.REPORT_HINT):
        super(LogState, self).__init__(outcomes=['done'])
        self._text = text
        self._severity = severity

    def execute(self, userdata):
        # Already logged. No need to wait for anything.
        return 'done'

    def on_enter(self, userdata):
        '''Log upon entering the state.'''
        Logger.log(self._text, self._severity)
