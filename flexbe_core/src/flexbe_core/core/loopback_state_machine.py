#!/usr/bin/env python
import rospy

from flexbe_core.core.lockable_state_machine import LockableStateMachine


class LoopbackStateMachine(LockableStateMachine):
    """
    A state machine that can loop back to itself.
    """
    
    _loopback_name = 'loopback'
    
    def __init__(self, *args, **kwargs):
        # add loopback outcome
        if len(args) > 0:
            args[0].append(self._loopback_name)
        else:
            outcomes = kwargs.get('outcomes', [])
            outcomes.append(self._loopback_name)
            kwargs['outcomes'] = outcomes
            
        super(LoopbackStateMachine, self).__init__(*args, **kwargs)