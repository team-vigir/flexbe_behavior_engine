#!/usr/bin/env python
from flexbe_core.core.state import State
from flexbe_core.core.user_data import UserData
from flexbe_core.core.exceptions import StateError, StateMachineError


class StateMachine(State):

    _currently_opened_container = None

    def __init__(self, *args, **kwargs):
        super(StateMachine, self).__init__(*args, **kwargs)
        self._states = list()
        self._labels = dict()
        self._transitions = dict()
        self._remappings = dict()
        self._current_state = None
        self._own_userdata = UserData()
        self._userdata = None
        self._previously_opened_container = None

    def __enter__(self):
        self._previously_opened_container = StateMachine._currently_opened_container
        StateMachine._currently_opened_container = self

    def __exit__(self, *args):
        StateMachine._currently_opened_container = self._previously_opened_container
        self._previously_opened_container = None

    def __contains__(self, label):
        return label in self._labels

    def __getitem__(self, label):
        return self._labels[label]

    def __iter__(self):
        return iter(state.name for state in self._states)

    # construction

    @staticmethod
    def add(label, state, transitions, remapping=None):
        self = StateMachine.get_opened_container()
        if self is None:
            raise StateMachineError("No container openend, activate one first by: 'with state_machine:'")
        if label in self._labels:
            raise StateMachineError("The label %s has already been added to this state machine!" % label)
        if label in self._outcomes:
            raise StateMachineError("The label %s is an outcome of this state machine!" % label)
        # add state to this state machine
        self._states.append(state)
        self._labels[label] = state
        self._transitions[label] = transitions
        self._remappings[label] = remapping or dict()
        # update state instance
        state.set_name(label)
        state.set_parent(self)

    @staticmethod
    def get_opened_container():
        return StateMachine._currently_opened_container

    # execution

    def spin(self, userdata=None):
        outcome = None
        while True:
            outcome = self.execute(userdata)
            if outcome is not None:
                break
            self.sleep()
        return outcome

    def execute(self, userdata):
        if self._current_state is None:
            self.assert_consistent_transitions()
            self._current_state = self.initial_state
            self._userdata = userdata or UserData()
            self._userdata(add_from=self._own_userdata)
        outcome = self._execute_current_state()
        return outcome

    def sleep(self):
        if self._current_state is not None:
            self._current_state.sleep()

    def _execute_current_state(self):
        with UserData(reference=self._userdata, remap=self._remappings[self._current_state.name],
                      input_keys=self._current_state.input_keys, output_keys=self._current_state.output_keys
                      ) as userdata:
            outcome = self._current_state.execute(userdata)
        if outcome is not None:
            try:
                target = self._transitions[self._current_state.name][outcome]
            except KeyError as e:
                outcome = None
                raise StateError("Returned outcome '%s' is not registered as transition!" % str(e))
            self._current_state = self._labels.get(target)
            if self._current_state is None:
                return target

    # properties

    @property
    def userdata(self):
        return self._own_userdata

    @property
    def current_state(self):
        if self._current_state is not None:
            return self._current_state
        else:
            raise StateMachineError("No state active!")

    @property
    def current_state_label(self):
        return self.current_state.name

    @property
    def initial_state(self):
        if len(self._states) > 0:
            return self._states[0]
        else:
            raise StateMachineError("No states added yet!")

    @property
    def initial_state_label(self):
        return self.initial_state.name

    @property
    def sleep_duration(self):
        if self._current_state is not None:
            return self._current_state.sleep_duration
        else:
            return 0.

    # consistency checks

    @property
    def _valid_targets(self):
        return self._labels.keys() + self.outcomes

    def assert_consistent_transitions(self):
        for transitions in self._transitions.values():
            for transition_target in transitions.values():
                if transition_target not in self._valid_targets:
                    raise StateMachineError("Transition target '%s' missing in %s" %
                                            (transition_target, str(self._valid_targets)))
