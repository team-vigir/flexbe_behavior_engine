#!/usr/bin/env python
import traceback
import smach

from smach.state_machine import StateMachine

from flexbe_core.core.event_state import EventState
from flexbe_core.core.operatable_state_machine import OperatableStateMachine


class ConcurrencyContainer(EventState, OperatableStateMachine):
    """
    A state machine that can be operated.
    It synchronizes its current state with the mirror and supports some control mechanisms.
    """
    
    def __init__(self, conditions=dict(), *args, **kwargs):
        super(ConcurrencyContainer, self).__init__(*args, **kwargs)
        self._conditions = conditions
        self._returned_outcomes = dict()


    def execute(self, *args, **kwargs):
        # use state machine execute, not state execute
        return OperatableStateMachine.execute(self, *args, **kwargs)


    def _build_msg(self, *args, **kwargs):
        # use state machine _build_msg, not state execute
        return OperatableStateMachine._build_msg(self, *args, **kwargs)


    def _update_once(self):
        #self._state_transitioning_lock.release()
        for state in self._ordered_states:
            if state.name in self._returned_outcomes.keys() and self._returned_outcomes[state.name] != self._loopback_name:
                continue
            try:
                ud = smach.Remapper(
                            self.userdata,
                            state.get_registered_input_keys(),
                            state.get_registered_output_keys(),
                            self._remappings[state.name])
                if isinstance(state, OperatableStateMachine):
                    if state._current_state is None:
                        state._set_current_state(state._initial_state_label)
                    self._returned_outcomes[state.name] = state._async_execute(ud)
                    if self._returned_outcomes[state.name] is None:
                        self._returned_outcomes[state.name] = self._loopback_name
                else:
                    self._returned_outcomes[state.name] = state.execute(ud)
                #print 'execute %s --> %s' % (state.name, self._returned_outcomes[state.name])
            except smach.InvalidUserCodeError as ex:
                smach.logerr("State '%s' failed to execute." % state.name)
                raise ex
            except:
                raise smach.InvalidUserCodeError("Could not execute state '%s' of type '%s': " %
                                                 (state.name, state)
                                                 + traceback.format_exc())
        #self._state_transitioning_lock.acquire()

        # Determine outcome
        outcome = self._loopback_name
        for item in self._conditions:
            (oc, cond) = item
            if all(self._returned_outcomes[sn] == o for sn,o in cond):
                outcome = oc
                break
        
        # preempt (?)
        if outcome == self._loopback_name:
            return None

        if outcome in self.get_registered_outcomes():
            # Call termination callbacks
            self.call_termination_cbs([s.name for s in self._ordered_states],outcome)
            self._returned_outcomes = dict()

            return outcome
        else:
            raise smach.InvalidTransitionError("Outcome '%s' of state '%s' with transition target '%s' is neither a registered state nor a registered container outcome." %
                    (outcome, self.name, outcome))


    def _notify_start(self):
        state = self._ordered_states[0]
        if isinstance(state, EventState):
            state.on_start()
        if isinstance(state, OperatableStateMachine):
            state._notify_start()

    def _enable_ros_control(self):
        state = self._ordered_states[0]
        if isinstance(state, EventState):
            state._enable_ros_control()
        if isinstance(state, OperatableStateMachine):
            state._enable_ros_control()

    def _notify_stop(self):
        state = self._ordered_states[0]
        if isinstance(state, EventState):
            state.on_stop()
            state._disable_ros_control()
        if isinstance(state, OperatableStateMachine):
            state._notify_stop()

    def _disable_ros_control(self):
        state = self._ordered_states[0]
        if isinstance(state, EventState):
            state._disable_ros_control()
        if isinstance(state, OperatableStateMachine):
            state._disable_ros_control()