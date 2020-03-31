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
        # add loopback outcome
        if len(args) > 0  and type(args[0]) is list:
            # need this ugly check for list type, because first argument in CBState is the callback
            args[0].append(self._loopback_name)
        else:
            outcomes = kwargs.get('outcomes', [])
            outcomes.append(self._loopback_name)
            kwargs['outcomes'] = outcomes
            
        super(LoopbackState, self).__init__(*args, **kwargs)
        self._rate = rospy.Rate(10)
        self.__execute = self.execute
        self.execute = self._loopback_execute

    def _loopback_execute(self, *args, **kwargs):
        result = self.__execute(*args, **kwargs)
            
        if result is None or result == 'None':
            result = self._loopback_name
        return result

    def set_rate(self, rate):
        """
        Set the execution rate of this state,
        i.e., the rate with which the execute method is being called.

        Note: The rate is best-effort,
              a rospy.Rate does not guarantee real-time properties.
        
        @type label: float
        @param label: The desired rate in Hz.
        """
        self._rate = rospy.Rate(rate)
