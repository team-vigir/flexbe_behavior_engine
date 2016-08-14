#!/usr/bin/env python
import rospy

from flexbe_core.core.loopback_state_machine import LoopbackStateMachine
from flexbe_core.core.preemptable_state import PreemptableState

from flexbe_core.proxy import ProxySubscriberCached
from std_msgs.msg import Empty


class PreemptableStateMachine(LoopbackStateMachine):
    """
    A state machine that can be preempted.
    If preempted, the state machine will return the outcome preempted.
    """
    
    _preempted_name = 'preempted'
    
    def __init__(self, *args, **kwargs):
        # add preempted outcome
        if len(args) > 0:
            args[0].append(self._preempted_name)
        else:
            outcomes = kwargs.get('outcomes', [])
            outcomes.append(self._preempted_name)
            kwargs['outcomes'] = outcomes
            
        super(PreemptableStateMachine, self).__init__(*args, **kwargs)

        # always listen to preempt so that the behavior can be stopped even if unsupervised
        self._preempt_topic = 'flexbe/command/preempt'
        self._sub = ProxySubscriberCached({self._preempt_topic: Empty})
        self._sub.set_callback(self._preempt_topic, self._preempt_cb)


    def _preempt_cb(self, msg):
        if not self._is_controlled:
            PreemptableState.preempt = True