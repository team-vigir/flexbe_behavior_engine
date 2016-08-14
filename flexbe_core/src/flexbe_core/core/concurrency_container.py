#!/usr/bin/env python
import traceback
import smach

from smach.state_machine import StateMachine

from flexbe_core.core.event_state import EventState
from flexbe_core.core.operatable_state_machine import OperatableStateMachine
from flexbe_core.core.preemptable_state import PreemptableState
from flexbe_core.core.priority_container import PriorityContainer


class ConcurrencyContainer(EventState, OperatableStateMachine):
    """
    A state machine that can be operated.
    It synchronizes its current state with the mirror and supports some control mechanisms.
    """
    
    def __init__(self, conditions=dict(), *args, **kwargs):
        super(ConcurrencyContainer, self).__init__(*args, **kwargs)
        self._conditions = conditions
        self._returned_outcomes = dict()

        self.__execute = self.execute
        self.execute = self._concurrency_execute


    def _concurrency_execute(self, *args, **kwargs):
        # use state machine execute, not state execute
        return OperatableStateMachine.execute(self, *args, **kwargs)


    def _build_msg(self, *args, **kwargs):
        # use state machine _build_msg, not state execute
        return OperatableStateMachine._build_msg(self, *args, **kwargs)


    def _update_once(self):
        # Check if a preempt was requested before or while the last state was running
        if self.preempt_requested() or PreemptableState.preempt:
            return self._preempted_name

        #self._state_transitioning_lock.release()
        for state in self._ordered_states:
            if state.name in self._returned_outcomes.keys() and self._returned_outcomes[state.name] != self._loopback_name:
                continue
            if PriorityContainer.active_container is not None \
            and not PriorityContainer.active_container.startswith(state._get_path()) \
            and not state._get_path().startswith(PriorityContainer.active_container):
                if isinstance(state, EventState):
                    state._notify_skipped()
                elif state._get_deep_state() is not None:
                    state._get_deep_state()._notify_skipped()
                continue
            self._returned_outcomes[state.name] = self._execute_state(state)
        #self._state_transitioning_lock.acquire()

        # Determine outcome
        outcome = self._loopback_name
        for item in self._conditions:
            (oc, cond) = item
            if all(self._returned_outcomes.has_key(sn) and self._returned_outcomes[sn] == o for sn,o in cond):
                outcome = oc
                break
        
        # preempt (?)
        if outcome == self._loopback_name:
            return None

        if outcome in self.get_registered_outcomes():
            # Call termination callbacks
            self.call_termination_cbs([s.name for s in self._ordered_states],outcome)
            self.on_exit(self.userdata, states = filter(lambda s: s.name not in self._returned_outcomes.keys() or self._returned_outcomes[s.name] == self._loopback_name, self._ordered_states))
            self._returned_outcomes = dict()
            # right now, going out of a cc may break sync
            # thus, as a quick fix, explicitly sync again
            self._parent._inner_sync_request = True
            self._current_state = None

            return outcome
        else:
            raise smach.InvalidTransitionError("Outcome '%s' of state '%s' with transition target '%s' is neither a registered state nor a registered container outcome." %
                    (outcome, self.name, outcome))

    def _execute_state(self, state, force_exit=False):
        result = None
        try:
            ud = smach.Remapper(
                        self.userdata,
                        state.get_registered_input_keys(),
                        state.get_registered_output_keys(),
                        self._remappings[state.name])
            if force_exit:
                state._entering = True
                state.on_exit(ud)
            else:
                result = state.execute(ud)
            #print 'execute %s --> %s' % (state.name, self._returned_outcomes[state.name])
        except smach.InvalidUserCodeError as ex:
            smach.logerr("State '%s' failed to execute." % state.name)
            raise ex
        except:
            raise smach.InvalidUserCodeError("Could not execute state '%s' of type '%s': " %
                                             (state.name, state)
                                             + traceback.format_exc())
        return result


    def _notify_start(self):
        for state in self._ordered_states:
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
        for state in self._ordered_states:
            if isinstance(state, EventState):
                state.on_stop()
            if isinstance(state, OperatableStateMachine):
                state._notify_stop()
            if state._is_controlled:
                state._disable_ros_control()

    def _disable_ros_control(self):
        state = self._ordered_states[0]
        if isinstance(state, EventState):
            state._disable_ros_control()
        if isinstance(state, OperatableStateMachine):
            state._disable_ros_control()

    def on_exit(self, userdata, states = None):
        for state in self._ordered_states if states is None else states:
            self._execute_state(state, force_exit=True)