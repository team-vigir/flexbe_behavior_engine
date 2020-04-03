#!/usr/bin/env python
from flexbe_core.logger import Logger
from flexbe_core.core import PreemptableState, OperatableStateMachine, LockableStateMachine


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
        It is called immediately before executing the behavior,
        so used parameters will have their final value when called.

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
            Logger.logerr('Behavior was not initialized! Please call superclass constructor.')
        instance = behavior_class()
        self.contains[behavior_id] = instance

    def use_behavior(self, behavior_class, behavior_id, default_keys=None, parameters=None):
        """
        Creates a state machine implementing the given behavior to use it in the behavior state machine.
        Behavior has to be added first.

        @type behavior_class: class
        @param behavior_class: The class implementing the other behavior.

        @type behavior_id: string
        @param behavior_id: Same identifier as used for adding.

        @type default_keys: list
        @param default_keys: List of input keys of the behavior which should be ignored
                             and instead use the default values as given by the behavior.

        @type parameters: dict
        @param parameters: Optional assignment of values to behavior parameters.
                           Any assigned parameter will be ignored for runtime customization,
                           i.e., cannot be overwritten by a user who runs the behavior.
        """
        if behavior_id not in self.contains:
            Logger.logerr('Tried to use a behavior without adding it!')
            return None

        if parameters is not None:
            for parameter, value in parameters.items():
                setattr(self.contains[behavior_id], parameter, value)

        state_machine = self.contains[behavior_id]._get_state_machine()

        if default_keys is not None:
            state_machine._input_keys = list(set(state_machine._input_keys) - set(default_keys))

        return state_machine

    # Lifecycle

    def prepare_for_execution(self, input_data=None):
        """
        Prepares this behavior for execution by building its state machine.
        """
        OperatableStateMachine.autonomy_level = self._autonomy_level

        self._state_machine = self.create()

        if input_data is None:
            input_data = dict()
        for k, v in input_data.items():
            if k in self._state_machine.userdata:
                self._state_machine.userdata[k] = v

    def set_parameter(self, name, value):
        """
        Set the given parameter of this behavior or any sub-behavior.
        Use a path specification to refer to the corresponding sub-behavior.
        The parameter value is propagated to all sub-behaviors according to the propagation rules.
        Also, the value is casted to the type of the parameter if required.

        @type name: string
        @param name: Name of the parameter, possibly including the path to a sub-behavior.

        @type value: object
        @param value: New value of the parameter of same type as before (or can be casted to that type).

        @return: Whether the parameter existed and could be set.
        """
        name_split = name.rsplit('/', 1)
        behavior = name_split[0] if len(name_split) == 2 else ''
        key = name_split[-1]
        behaviors = self.get_contained_behaviors()
        behaviors[''] = self  # add this behavior as well
        found = False
        for b in behaviors:
            if b.startswith(behavior) and hasattr(behaviors[b], key):
                behaviors[b]._set_typed_attribute(key, value)
                found = True
        return found

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
        result = self._state_machine.spin()
        self._state_machine.destroy()
        return result

    def prepare_for_switch(self, state):
        """
        Prepares the behavior for being executed after a behavior switch.

        @type name: string
        @param name: The name of this behavior.
        """
        state._locked = True  # make sure the state cannot transition during preparations
        states = self._get_states_of_path(state.path, self._state_machine)
        if states is None:
            raise RuntimeError("Did not find locked state in new behavior!")
        state_container = state._parent
        state_container.remove_state(state)  # remove from old state machine
        for sm in states[1:]:
            # update userdata in new state machine
            sm.replace_userdata(state_container.userdata)
            state_container = state_container._parent
        states[1].replace_state(state)  # add to new state machine
        self.requested_state_path = state.path  # set start after switch

    def get_current_state(self):
        return self._state_machine.get_deep_state()

    def get_locked_state(self):
        state = self._state_machine.get_deep_state()
        while state is not None:
            if state.is_locked():
                return state
            else:
                state = state._parent
        return None

    def preempt(self):
        PreemptableState.preempt = True

    # For internal use only

    def _get_state_machine(self):
        if self._state_machine is None:
            self._state_machine = self.create()
        return self._state_machine

    def _collect_contained(self, obj, path):
        contain_list = {path+"/"+key: value for (key, value) in getattr(obj, 'contains', {}).items()}
        add_to_list = {}
        for b_id, b_inst in contain_list.items():
            add_to_list.update(self._collect_contained(b_inst, b_id))
        contain_list.update(add_to_list)
        return contain_list

    def get_contained_behaviors(self):
        return self._collect_contained(self, '')

    def _set_typed_attribute(self, name, value):
        attr = getattr(self, name)
        # convert type if required
        if not isinstance(value, type(attr)):
            if type(attr) is int:
                value = int(value)
            elif type(attr) is float:
                value = float(value)
            elif type(attr) is bool:
                value = (value != "0")
            elif type(attr) is dict:
                import yaml
                value = yaml.load(value)
        setattr(self, name, value)

    def set_up(self, id, autonomy_level, debug):
        self.id = id
        self._autonomy_level = autonomy_level
        self._debug = debug

    def _get_states_of_path(self, path, container):
        path_elements = path.split('/')
        if len(path_elements) < 2:
            return [container]  # actually a state in this case
        state_label = path_elements[1]
        new_path = "/".join(path_elements[1:])
        # collect along the path and append self
        if state_label in container:
            childlist = self._get_states_of_path(new_path, container[state_label])
            if childlist is None:
                return None
            childlist.append(container)
            return childlist
        else:
            return None
