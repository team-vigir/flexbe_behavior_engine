#!/usr/bin/env python
import rospy
import smach_ros
import smach
import importlib
import sys
import string

from flexbe_core import OperatableStateMachine, LockableStateMachine
from flexbe_core.core import PreemptableState

'''
Created on 20.05.2013

@author: Philipp Schillinger
'''

class Behavior(object):
    '''
    This is the superclass for all implemented behaviors.
    '''

    def __init__(self):
        '''
        Please call this superclass constructor first when overriding it with your behavior.
        '''
        self._state_machine = None
        self.name = "unnamed behavior"
        self.id = 0
        
        self.contains = {}
        self._behaviors = {}
        
        self._autonomy_level = 3
        self._debug = False

        self.requested_state_path = None
        
    
    # Please implement those:
    
    def create(self):
        """
        Should create the state machine for this behavior and return it.
        It is called immediately before executing the behavior, so used parameters will have their final value when called.
        
        @return The complete state machine for this behavior.
        """
        pass
    
    
    # Use those if you need them:
    
    def add_parameter(self, name, default):
        """
        Adds a parameter to this behavior.
        The parameter should be declared in the behavior manifest.
        
        @type name: string
        @param name: The name of the parameter.
        
        @type default: object
        @param default: The default value of this parameter. Be sure to set it to the right type.
        """
        setattr(self, name, default)

    def add_behavior(self, behavior_class, behavior_id):
        """
        Adds another behavior as part of this behavior.
        This other behavior should be declared as contained in the behavior manifest.
        
        @type behavior_class: class
        @param behavior_class: The class implementing the other behavior.
        
        @type behavior_id: string
        @param behavior_id: Unique identifier for this behavior instance.
        """
        if not hasattr(self, 'contains'):
            rospy.logerr('Behavior was not initialized! Please call superclass constructor.')

        instance = behavior_class()
        self.contains[behavior_id] = instance
    
    def use_behavior(self, behavior_class, behavior_id, default_keys=None):
        """
        Creates a state machine implementing the given behavior to use it in the behavior state machine.
        Behavior has to be added first.
        
        @type behavior_class: class
        @param behavior_class: The class implementing the other behavior.
        
        @type behavior_id: string
        @param behavior_id: Same identifier as used for adding.
        
        @type default_keys: list
        @param default_keys: List of input keys of the behavior which should be ignored and instead use the default values as given by the behavior.
        """
        if not self.contains.has_key(behavior_id):
            rospy.logerr('Tried to use not added behavior!')
            return None
        
        state_machine = self.contains[behavior_id]._get_state_machine()

        if default_keys is not None:
            state_machine._input_keys = list(set(state_machine._input_keys) - set(default_keys))
        
        return state_machine


    # Lifecycle

    def prepare_for_execution(self, input_data = {}):
        """
        Prepares this behavior for execution by building its state machine.
        """
        OperatableStateMachine.autonomy_level = self._autonomy_level

        self._state_machine = self.create()
        self._state_machine._input_keys = {}
        self._state_machine._output_keys = {}

        for k, v in input_data.items():
           if k in self._state_machine.userdata:
                self._state_machine.userdata[k] = v


    def confirm(self):
        """
        Confirms that this behavior is ready for execution.
        """
        LockableStateMachine.path_for_switch = self.requested_state_path

        self._state_machine.confirm(self.name, self.id)
        
        
    def execute(self):
        """
        Called when the behavior is executed.
        Need to call self.execute_behavior when ready to start the state machine and return its result.
        
        @return: A string containing the execution result such as finished or failed.
        """
        PreemptableState.switching = False
        result = self._state_machine.execute()

        self._state_machine.destroy()

        return result


    def prepare_for_switch(self, state):
        """
        Prepares the behavior for being executed after a behavior switch.
        
        @type name: string
        @param name: The name of this behavior.
        """
        states = self._get_states_of_path(state._get_path(), self._state_machine)
        if states is None:
            raise smach.InvalidConstructionError("Did not find locked state in new behavior!")
        state_container = state._parent
        for sm in states[1:]:
            sm.set_initial_state([sm._initial_state_label], state_container.userdata)
            #rospy.loginfo("Set userdata for %s from %s; (keys: %d): %s", str(sm.name), str(state_container.name), len(state_container.userdata._data), str(state_container.userdata._data))
            state_container = state_container._parent
        states[1].replace(state)

        self.requested_state_path = state._get_path()


    def get_current_state(self):
        return self._state_machine._get_deep_state()

    def get_locked_state(self):
        state = self._state_machine._get_deep_state()
        while not state is None:
            if state.is_locked():
                return state
            else:
                state = state._parent
        return None

    def preempt(self):
        PreemptableState.preempt = True

    def preempt_for_switch(self):
        PreemptableState.switching = True
        PreemptableState.preempt = True
    
    
    # For internal use only
    
    def _get_state_machine(self):
        if self._state_machine is None:
            self._state_machine = self.create()
        return self._state_machine
    
    def _get_muted_state_machine(self):
        if self._state_machine is None:
            self._state_machine = self.create()
        self._mute_state_machine(self._state_machine)
        return self._state_machine

    def _mute_state_machine(self, sm):
        for state in sm._ordered_states:
            if isinstance(state, OperatableStateMachine):
                self._mute_state_machine(state)
            else:
                state._is_controlled = False

    
    def set_up(self, id, autonomy_level, debug):
        self.id = id
        self._autonomy_level = autonomy_level
        self._debug = debug

    def _get_states_of_path(self, path, container):
        path_elements = path.split('/')
        if len(path_elements) < 2:
            return [container]
        state_label = path_elements[1]
        new_path = "/".join(path_elements[1:])

        if container.get_children().has_key(state_label):
            childlist = self._get_states_of_path(new_path, container.get_children()[state_label])
            if childlist is None:
                return None
            childlist.append(container)
            return childlist
        else:
            return None





























