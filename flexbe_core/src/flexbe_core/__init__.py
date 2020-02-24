#!/usr/bin/env python


#
#    Please use EventState as parent class for new states because it extends all other parent classes.
#    For a behavior, choose OperatableStateMachine as state machine.
#    For each state machine in the background, choose SilentStateMachine or JumpableStateMachine (only do this if you know what you are doing).
#

from .core import EventState
from .core import OperatableStateMachine, JumpableStateMachine, SilentStateMachine, LockableStateMachine, ConcurrencyContainer, PriorityContainer

from .behavior import Behavior

from .behavior_library import BehaviorLibrary

from .logger import Logger


class Autonomy:
    """
    Provides constants for the available required Autonomy Levels.
    """
    
    Inherit = 0
    """
    Use this whenever you want to rely on an already defined level of autonomy
    Typical situations: Outcomes of a contained state machine or behavior, outcome of the input request state
    """
    
    Off = 0
    """
    Use this when no level of autonomy is required, the Autonomy Level needs to be at least 'Off'.
    Typical situations: Outcomes that report internal software errors or thrown exceptions.
    """

    Low = 1
    """
    Use this for reliable decisions that only need validation in the 'low' autonomy mode.
    """
    
    High = 2
    """
    Use this for more important or high level decisions that will only be executed autonomously when in full autonomy and need validation in the 'high' autonomy mode.
    """

    Full = 3
    """
    Use this for outcomes that always need an operator input.
    A use of this level is not recommended.
    """
    

            

        

        
    




    

