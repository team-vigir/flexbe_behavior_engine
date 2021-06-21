#!/usr/bin/env python

#
#    Please use EventState as parent class for new states because it extends all other parent classes.
#    For a behavior, choose OperatableStateMachine as state machine.
#

from .core import EventState  # noqa: F401
from .core import OperatableStateMachine, ConcurrencyContainer, PriorityContainer  # noqa: F401

from .behavior import Behavior  # noqa: F401

from .behavior_library import BehaviorLibrary  # noqa: F401

from .logger import Logger  # noqa: F401
from .state_logger import StateLogger  # noqa: F401


def set_node(node):
    from .proxy import initialize_proxies
    from .core import RosState, RosStateMachine
    Logger.initialize(node)
    StateLogger.initialize_ros(node)
    initialize_proxies(node)
    RosState.initialize_ros(node)
    RosStateMachine.initialize_ros(node)


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
    Use this for more important or high level decisions that will only be executed autonomously
    when in full autonomy and need validation in the 'high' autonomy mode.
    """

    Full = 3
    """
    Use this for outcomes that always need an operator input.
    A use of this level is not recommended.
    """
