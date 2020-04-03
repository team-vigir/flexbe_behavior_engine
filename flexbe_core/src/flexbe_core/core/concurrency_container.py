#!/usr/bin/env python
from flexbe_core.logger import Logger
from flexbe_core.core.user_data import UserData
from flexbe_core.core.event_state import EventState
from flexbe_core.core.priority_container import PriorityContainer

from flexbe_core.core.operatable_state_machine import OperatableStateMachine


class ConcurrencyContainer(OperatableStateMachine):
    """
    A state machine that can be operated.
    It synchronizes its current state with the mirror and supports some control mechanisms.
    """

    def __init__(self, conditions=dict(), *args, **kwargs):
        super(ConcurrencyContainer, self).__init__(*args, **kwargs)
        self._conditions = conditions
        self._returned_outcomes = dict()
        self._sleep_dur = None

    def sleep(self):
        if self._sleep_dur is not None:
            self.wait(seconds=self._sleep_dur)
            self._sleep_dur = None

    @property
    def sleep_duration(self):
        return self._sleep_dur or 0.

    def _execute_current_state(self):
        # execute all states that are done with sleeping and determine next sleep duration
        sleep_dur = None
        for state in self._states:
            if state.name in list(self._returned_outcomes.keys()) and self._returned_outcomes[state.name] is not None:
                continue  # already done with executing
            if (PriorityContainer.active_container is not None
                and not all(a == s for a, s in zip(PriorityContainer.active_container.split('/'),
                                                   state.path.split('/')))):
                if isinstance(state, EventState):
                    state._notify_skipped()
                elif state.get_deep_state() is not None:
                    state.get_deep_state()._notify_skipped()
                continue  # other state has priority
            if state.sleep_duration <= 0:  # ready to execute
                self._returned_outcomes[state.name] = self._execute_single_state(state)
                # check again in case the sleep has already been handled by the child
                if state.sleep_duration <= 0:
                    # this sleep returns immediately since sleep duration is negative,
                    # but is required here to reset the sleep time after executing
                    state.sleep()
            sleep_dur = state.sleep_duration if sleep_dur is None else min(sleep_dur, state.sleep_duration)
        if sleep_dur > 0:
            self._sleep_dur = sleep_dur

        # Determine outcome
        outcome = None
        if any(self._returned_outcomes[state.name] == state._preempted_name
               for state in self._states if state.name in self._returned_outcomes):
            return self._preempted_name  # handle preemption if required
        # check conditions
        for item in self._conditions:
            (oc, cond) = item
            if all(sn in self._returned_outcomes and self._returned_outcomes[sn] == o for sn, o in cond):
                outcome = oc
                break

        if outcome is None:
            return None

        # trigger on_exit for those states that are not done yet
        self.on_exit(self.userdata,
                     states=[s for s in self._states if (s.name not in list(self._returned_outcomes.keys()) or
                                                         self._returned_outcomes[s.name] is None)])
        self._returned_outcomes = dict()
        # right now, going out of a cc may break sync
        # thus, as a quick fix, explicitly sync again
        self._parent._inner_sync_request = True
        self._current_state = None
        return outcome

    def _execute_single_state(self, state, force_exit=False):
        result = None
        try:
            ud = UserData(reference=self.userdata, input_keys=state.input_keys,
                          output_keys=state.output_keys, remap=self._remappings[state.name])
            if force_exit:
                state._entering = True
                state.on_exit(ud)
            else:
                result = state.execute(ud)
        except Exception as e:
            result = None
            self._last_exception = e
            Logger.logerr('Failed to execute state %s:\n%s' % (self.current_state_label, str(e)))
        return result

    def _enable_ros_control(self):
        state = self._states[0]
        if isinstance(state, EventState):
            state._enable_ros_control()
        if isinstance(state, OperatableStateMachine):
            state._enable_ros_control()

    def _disable_ros_control(self):
        state = self._states[0]
        if isinstance(state, EventState):
            state._disable_ros_control()
        if isinstance(state, OperatableStateMachine):
            state._disable_ros_control()

    def on_exit(self, userdata, states=None):
        for state in self._states if states is None else states:
            if state in self._returned_outcomes:
                continue  # skip states that already exited themselves
            self._execute_single_state(state, force_exit=True)
