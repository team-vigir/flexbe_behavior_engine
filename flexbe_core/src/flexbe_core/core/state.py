#!/usr/bin/env python
from flexbe_core.core.exceptions import StateError


class State(object):

    def __init__(self, *args, **kwargs):
        self._outcomes = list(set(kwargs.get('outcomes', [])))
        io_keys = kwargs.get('io_keys', [])
        self._input_keys = list(set(kwargs.get('input_keys', []) + io_keys))
        self._output_keys = list(set(kwargs.get('output_keys', []) + io_keys))
        # properties of instances of a state machine
        self._name = None
        self._parent = None

    def execute(self, userdata):
        pass

    def sleep(self):
        pass

    @property
    def sleep_duration(self):
        return 0.

    @property
    def outcomes(self):
        return self._outcomes

    @property
    def input_keys(self):
        return self._input_keys

    @property
    def output_keys(self):
        return self._output_keys

    # instance properties

    @property
    def name(self):
        return self._name

    def set_name(self, value):
        if self._name is not None:
            raise StateError("Cannot change the name of a state!")
        else:
            self._name = value

    @property
    def parent(self):
        return self._parent

    def set_parent(self, value):
        if self._parent is not None:
            raise StateError("Cannot change the parent of a state!")
        else:
            self._parent = value

    @property
    def path(self):
        return "" if self.parent is None else self.parent.path + "/" + self.name
