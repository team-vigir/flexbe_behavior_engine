#!/usr/bin/env python
import os
import time
from flexbe_core import EventState, Logger

class LogFileKeyState(EventState):
    '''
    A state that can log a predefined message including an input key
    to precisely inform the operator about what happened to the behavior.
    The state will also write the message into a defined file.

    -- filepath     string         The absolute path to the file where the message is logged.
    -- text         string         The message to be logged to the terminal Example:  'Counter value:  {}'.
    -- severity     uint8          Type of logging (Logger.REPORT_INFO / WARN / HINT / ERROR)

    #> data         object         The data provided to be printed in the message. The exact type depends on the request.

    <= done                        Indicates that the message has been logged.
    '''

    def __init__(self, text, filepath="~/.flexbe_logs/log_file.csv", severity=Logger.REPORT_HINT):
        super(LogFileKeyState, self).__init__(outcomes=['done'],
                                          input_keys=['data'])
        self._filepath = filepath
        self._text = text
        self._severity = severity

    def execute(self, userdata):
        # Already logged. No need to wait for anything.
        return 'done'

    def on_enter(self, userdata):
        '''Log upon entering the state.'''
        Logger.log(self._text.format(userdata.data), self._severity)
        with open(os.path.expanduser(self._filepath), "a") as file_object:
            file_object.write(str(time.time()) + ";"+ self._text.format(userdata.data)+"\n")
