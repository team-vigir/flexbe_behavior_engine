#!/usr/bin/env python
from flexbe_core.core.preemptable_state import PreemptableState
from flexbe_core.proxy import ProxySubscriberCached

from std_msgs.msg import Empty

from flexbe_core.core.lockable_state_machine import LockableStateMachine


class PreemptableStateMachine(LockableStateMachine):
    """
    A state machine that can be preempted.
    If preempted, the state machine will return the outcome preempted.
    """

    _preempted_name = 'preempted'

    def __init__(self, *args, **kwargs):
        super(PreemptableStateMachine, self).__init__(*args, **kwargs)
        # always listen to preempt so that the behavior can be stopped even if unsupervised
        self._preempt_topic = 'flexbe/command/preempt'
        self._sub = ProxySubscriberCached({self._preempt_topic: Empty})
        self._sub.set_callback(self._preempt_topic, self._preempt_cb)

    def _preempt_cb(self, msg):
        if not self._is_controlled:
            PreemptableState.preempt = True

    @staticmethod
    def add(label, state, transitions=None, remapping=None):
        transitions[PreemptableState._preempted_name] = PreemptableStateMachine._preempted_name
        LockableStateMachine.add(label, state, transitions, remapping)

    @property
    def _valid_targets(self):
        return super(PreemptableStateMachine, self)._valid_targets + [PreemptableStateMachine._preempted_name]
