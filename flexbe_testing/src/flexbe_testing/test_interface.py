#!/usr/bin/env python
import inspect
import rospy

from flexbe_core.core import EventState
from .logger import Logger


class TestInterface(object):
    """ Interface to states and behaviors that are subject to testing. """

    def __init__(self, path, classname):
        package = __import__(path, fromlist=[path])
        clsmembers = inspect.getmembers(package, lambda member: (
            inspect.isclass(member) and member.__module__ == package.__name__
        ))
        self._class = next(c for name, c in clsmembers if name == classname)
        self._instance = None
        Logger.print_positive('%s imported' % self.get_base_name())

    def is_state(self):
        return issubclass(self._class, EventState)

    def get_base_name(self):
        return "state" if self.is_state() else "behavior"

    # instantiate

    def instantiate(self, params=None):
        if self.is_state():
            self._instance = self._instantiate_state(params=params)
        else:
            self._instance = self._instantiate_behavior(params=params)
        Logger.print_positive('%s instantiated' % self.get_base_name())

    def _instantiate_state(self, params=None):
        if params is None:
            return self._class()
        else:
            return self._class(**params)

    def _instantiate_behavior(self, params=None):
        be = self._class()
        if params is not None:
            for name, value in params.items():
                be.set_parameter(name, value)
        be.set_up(id=0, autonomy_level=255, debug=False)
        return be

    # execute

    def execute(self, userdata, spin_cb=None):
        spin_cb = spin_cb or (lambda: None)
        if self.is_state():
            outcome = self._execute_state(userdata, spin_cb)
        else:
            outcome = self._execute_behavior(userdata, spin_cb)
        Logger.print_positive('finished %s execution' % self.get_base_name())
        return outcome

    def _execute_state(self, userdata, spin_cb):
        self._instance.on_start()
        outcome = None
        while outcome is None and not rospy.is_shutdown():
            outcome = self._instance.execute(userdata)
            self._instance.sleep()
            spin_cb()
        self._instance.on_stop()
        return outcome

    def _execute_behavior(self, userdata, spin_cb):
        self._instance.prepare_for_execution(userdata._data)
        self._instance.confirm()
        # do not execute behavior directly, instead explicitly spin its state machine
        # this is required here for spinning ROS and processing roslaunch context callbacks
        outcome = None
        sm = self._instance._state_machine
        while outcome is None and not rospy.is_shutdown():
            outcome = sm.execute(userdata)
            sm.sleep()
            spin_cb()
        return outcome
