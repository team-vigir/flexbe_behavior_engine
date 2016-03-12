#!/usr/bin/env python
import traceback
import smach

from smach.state_machine import StateMachine

from flexbe_core.core.operatable_state_machine import OperatableStateMachine


class PriorityContainer(OperatableStateMachine):
    """
    A state machine that is always executed alone when becoming active.
    """
    active_container = None
    
    def __init__(self, conditions=dict(), *args, **kwargs):
        super(PriorityContainer, self).__init__(*args, **kwargs)

        self._parent_active_container = None

    def execute(self, *args, **kwargs):
        if PriorityContainer.active_container != self._get_path():
            self._parent_active_container = PriorityContainer.active_container
            PriorityContainer.active_container = self._get_path()

        outcome = OperatableStateMachine.execute(self, *args, **kwargs)

        if outcome != self._loopback_name:
            PriorityContainer.active_container = self._parent_active_container

        return outcome
