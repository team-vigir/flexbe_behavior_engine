#!/usr/bin/env python


class State(object):

    def __init__(self, *args, **kwargs):
        self._outcomes = set(kwargs.get('outcomes', []))
        io_keys = kwargs.get('io_keys', [])
        self._input_keys = set(kwargs.get('input_keys', []) + io_keys)
        self._output_keys = set(kwargs.get('output_keys', []) + io_keys)
        self._rate = None
        # properties of instances of a state machine
        self._name = None
        self._parent = None

    def execute(self, userdata):
        pass

    def sleep(self):
        if self._rate is not None:
            self._rate.sleep()

    # instance properties

    @property
    def name(self):
        return self._name

    def set_name(self, value):
        if self._name is not None:
            raise ValueError("Cannot change the name of a state!")
        else:
            self._name = value

    @property
    def parent(self):
        return self._parent

    def set_parent(self, value):
        if self._parent is not None:
            raise ValueError("Cannot change the parent of a state!")
        else:
            self._parent = value

    @property
    def outcomes(self):
        return self._outcomes

    @property
    def input_keys(self):
        return self._input_keys

    @property
    def output_keys(self):
        return self._output_keys

    @property
    def sleep_duration(self):
        if self._rate is not None:
            return self._rate.remaining().to_sec()
        else:
            return 0.
