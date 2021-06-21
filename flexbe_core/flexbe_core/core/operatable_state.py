#!/usr/bin/env python
from flexbe_core.logger import Logger
from flexbe_core.state_logger import StateLogger

from flexbe_msgs.msg import OutcomeRequest
from std_msgs.msg import UInt8, String

from flexbe_core.core.preemptable_state import PreemptableState


@StateLogger.log_outcomes('flexbe.outcomes')
class OperatableState(PreemptableState):
    """
    A state that supports autonomy levels and silent mode.
    Also, it allows being tracked by a GUI or anything else
    as it reports each transition and its initial structure.
    An additional method is provided to report custom status messages to the widget.
    """

    def __init__(self, *args, **kwargs):
        super(OperatableState, self).__init__(*args, **kwargs)
        self.__execute = self.execute
        self.execute = self._operatable_execute

        self._last_requested_outcome = None

        self._outcome_topic = 'flexbe/mirror/outcome'
        self._request_topic = 'flexbe/outcome_request'
        self._debug_topic = 'flexbe/debug/current_state'

    def _operatable_execute(self, *args, **kwargs):
        outcome = self.__execute(*args, **kwargs)

        if self._is_controlled:
            # reset previously requested outcome if applicable
            if self._last_requested_outcome is not None and outcome is None:
                self._pub.publish(self._request_topic, OutcomeRequest(outcome=255, target=self.path))
                self._last_requested_outcome = None

            # request outcome because autonomy level is too low
            if not self._force_transition and (not self.parent.is_transition_allowed(self.name, outcome) or
                                               outcome is not None and self.is_breakpoint):
                if outcome != self._last_requested_outcome:
                    self._pub.publish(self._request_topic, OutcomeRequest(outcome=self.outcomes.index(outcome),
                                                                          target=self.path))
                    Logger.localinfo("<-- Want result: %s > %s" % (self.name, outcome))
                    StateLogger.log('flexbe.operator', self, type='request', request=outcome,
                                    autonomy=self.parent.autonomy_level,
                                    required=self.parent.get_required_autonomy(outcome))
                    self._last_requested_outcome = outcome
                outcome = None

            # autonomy level is high enough, report the executed transition
            elif outcome is not None and outcome in self.outcomes:
                Logger.localinfo("State result: %s > %s" % (self.name, outcome))
                self._pub.publish(self._outcome_topic, UInt8(self.outcomes.index(outcome)))
                self._pub.publish(self._debug_topic, String("%s > %s" % (self.path, outcome)))
                if self._force_transition:
                    StateLogger.log('flexbe.operator', self, type='forced', forced=outcome,
                                    requested=self._last_requested_outcome)
                self._last_requested_outcome = None

        self._force_transition = False
        return outcome

    def _enable_ros_control(self):
        super(OperatableState, self)._enable_ros_control()
        self._pub.createPublisher(self._outcome_topic, UInt8)
        self._pub.createPublisher(self._debug_topic, String)
        self._pub.createPublisher(self._request_topic, OutcomeRequest)
