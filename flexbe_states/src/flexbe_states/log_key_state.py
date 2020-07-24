#!/usr/bin/env python
from flexbe_core import EventState, Logger


class LogKeyState(EventState):
    '''
    A state that can log a predefined message including an input key
    to precisely inform the operator about what happened to the behavior.

    -- text  	    string 	    The message to be logged to the terminal Example:  'Counter value:  {}'.
    -- severity     uint8  	    Type of logging (Logger.REPORT_INFO / WARN / HINT / ERROR)

    #> data  	    object 		The data provided to be printed in the message. The exact type depends on the request.

    <= done				        Indicates that the message has been logged.
    '''

    def __init__(self, text, severity=Logger.REPORT_HINT):
        super(LogKeyState, self).__init__(outcomes=['done'],
                                          input_keys=['data'])
        self._text = text
        self._severity = severity

    def execute(self, userdata):
        # Already logged. No need to wait for anything.
        return 'done'

    def on_enter(self, userdata):
        '''Log upon entering the state.'''
        Logger.log(self._text.format(userdata.data), self._severity)
