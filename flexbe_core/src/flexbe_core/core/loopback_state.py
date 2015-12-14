#!/usr/bin/env python
import rospy

from flexbe_core.core.lockable_state import LockableState

class LoopbackState(LockableState):
    """
    A state that can refer back to itself.
    It periodically transitions to itself while no other outcome is fulfilled.
    """
    
    _loopback_name = 'loopback'
    def __init__(self, *args, **kwargs):
        self._rate = rospy.Rate(10)
        # add loopback outcome
        if len(args) > 0  and type(args[0]) is list:
            # need this ugly check for list type, because first argument in CBState is the callback
            args[0].append(self._loopback_name)
        else:
            outcomes = kwargs.get('outcomes', [])
            outcomes.append(self._loopback_name)
            kwargs['outcomes'] = outcomes
            
        super(LoopbackState, self).__init__(*args, **kwargs)
        self.__execute = self.execute
        self.execute = self._loopback_execute

    def _loopback_execute(self, *args, **kwargs):
        result = self.__execute(*args, **kwargs)
            
        if result is None or result == 'None':
            result = self._loopback_name
        return result
