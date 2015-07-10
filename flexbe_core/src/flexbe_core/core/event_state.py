#!/usr/bin/env python

from flexbe_core.core.operatable_state import OperatableState
from flexbe_core.core.preemptable_state import PreemptableState


class EventState(OperatableState):
    """
    A state that allows implementing certain events.
    """
    
    def __init__(self, *args, **kwargs):
        super(EventState, self).__init__(*args, **kwargs)
        
        self._entering = True
        self.__execute = self.execute
        self.execute = self._event_execute
        
    def _event_execute(self, *args, **kwargs):
        if self._entering:
            self._entering = False
            self.on_enter(*args, **kwargs)
        
        execute_outcome = self.__execute(*args, **kwargs)
        
        if execute_outcome != self._loopback_name and not PreemptableState.switching:
            self._entering = True
            self.on_exit(*args, **kwargs)
            
        return execute_outcome
    
    
    # Events
    # (just implement the ones you need)

    def on_start(self):
        """
        Will be executed once when the behavior starts.
        """
        pass

    def on_stop(self):
        """
        Will be executed once when the behavior stops or is preempted.
        """
        pass
    
    def on_enter(self, userdata):
        """
        Will be executed each time the state is entered from any other state (but not from itself).
        """
        pass
    
    def on_exit(self, userdata):
        """
        Will be executed each time the state will be left to any other state (but not to itself).
        """
        pass