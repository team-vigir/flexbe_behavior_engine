#!/usr/bin/env python

from smach.state_machine import StateMachine

from .preemptable_state_machine import PreemptableStateMachine
from flexbe_core.core.loopback_state import LoopbackState
import rospy


class SilentStateMachine(PreemptableStateMachine):
    """
    A state machine that runs in background and does not report any transition.
    """
    
    def __init__(self, *args, **kwargs):
        super(SilentStateMachine, self).__init__(*args, **kwargs)
        self.name = None
        self.transitions = None
        self.autonomy = None
        self._parent = None
        
    @staticmethod
    def add(label, state, transitions = None, autonomy = None, remapping = None):
        """
        Add a state to the opened state machine.
        
        @type label: string
        @param label: The label of the state being added.
        
        @param state: An instance of a class implementing the L{State} interface.
        
        @param transitions: A dictionary mapping state outcomes to other state
        labels or container outcomes.
        
        @param autonomy: A dictionary mapping state outcomes to their required
        autonomy level. Not relevant for this class.

        @param remapping: A dictrionary mapping local userdata keys to userdata
        keys in the container.
        """
        self = StateMachine._currently_opened_container()
        
        # add loopback transition to loopback states
        if isinstance(state, LoopbackState):
            transitions[LoopbackState._loopback_name] = label
        
        StateMachine.add(label, state, transitions, remapping)
        
        state.name = label
        state.transitions = transitions
        state.autonomy = None
        state._parent = self
        state._mute = True
            
    def destroy(self):
        pass
