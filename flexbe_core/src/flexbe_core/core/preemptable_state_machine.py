#!/usr/bin/env python
import rospy

from flexbe_core.core.loopback_state_machine import LoopbackStateMachine


class PreemptableStateMachine(LoopbackStateMachine):
    """
    A state machine that can be preempted.
    If preempted, the state machine will return the outcome preempted.
    """
    
    _preempted_name = 'preempted'
    
    def __init__(self, *args, **kwargs):
        # add preempted outcome
        if len(args) > 0:
            args[0].append(self._preempted_name)
        else:
            outcomes = kwargs.get('outcomes', [])
            outcomes.append(self._preempted_name)
            kwargs['outcomes'] = outcomes
            
        super(PreemptableStateMachine, self).__init__(*args, **kwargs)