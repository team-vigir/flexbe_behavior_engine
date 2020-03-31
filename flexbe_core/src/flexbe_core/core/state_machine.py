#!/usr/bin/env python
from .state import State
from .user_data import UserData


class StateMachine(State):

    _currently_opened_container = None

    def __init__(self, *args, **kwargs):
        super(StateMachine, self).__init__(*args, **kwargs)
        self._states = list()
        self._labels = dict()
        self._transitions = dict()
        self._remappings = dict()
        self._current_state = None
        self._userdata = UserData()
        self._previously_opened_container = None

    def __enter__(self):
        self._previously_opened_container = StateMachine._currently_opened_container
        StateMachine._currently_opened_container = self

    def __exit__(self, *args):
        StateMachine._currently_opened_container = self._previously_opened_container
        self._previously_opened_container = None

    # construction

    @staticmethod
    def add(label, state, transitions, remappings):
        self = StateMachine._currently_opened_container
        if label in self._labels:
            raise ValueError("The label %s has already been added to this state machine!" % label)
        if label in self._outcomes:
            raise ValueError("The label %s is an outcome of this state machine!" % label)
        # add state to this state machine
        self._states.append(state)
        self._labels[label] = state
        self._transitions[label] = transitions
        self._remappings[label] = remappings
        # update state instance
        state.set_name(label)
        state.set_parent(self)

    # execution

    def spin(self, userdata=None):
        self._userdata = userdata or self._userdata
        outcome = None
        while True:
            outcome = self.execute(self._userdata)
            if outcome is not None:
                break
            self.sleep()
        return outcome

    def execute(self, userdata):
        if self._current_state is None:
            self._current_state = self.initial_state
            self._userdata = userdata
        outcome = self._update_once()
        return outcome

    def sleep(self):
        if self._current_state is not None:
            self._current_state.sleep()

    def _update_once(self):
        outcome = self._current_state.execute(
            UserData(reference=self.userdata, input_keys=self._current_state.input_keys,
                     output_keys=self._current_state.output_keys, remap=self._remappings[self._current_state.name])
        )
        if outcome is not None:
            target = self._transitions[self._current_state.name][outcome]
            self._current_state = self._labels.get(target)
            if self._current_state is None and target in self._outcomes:
                return target

    # properties

    @property
    def userdata(self):
        return self._userdata

    @property
    def current_state(self):
        if self._current_state is not None:
            return self._current_state
        else:
            raise ValueError("No state active!")

    @property
    def current_state_label(self):
        return self.current_state.name

    @property
    def initial_state(self):
        if len(self._states) > 0:
            return self._states[0]
        else:
            raise ValueError("No states added yet!")

    @property
    def initial_state_label(self):
        return self.initial_state.name

    @property
    def sleep_duration(self):
        if self._current_state is not None:
            return self._current_state.sleep_duration
        else:
            return 0.
