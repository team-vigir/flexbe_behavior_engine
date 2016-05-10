#!/usr/bin/env python
import rospy
import smach

from flexbe_core.proxy import ProxyPublisher, ProxySubscriberCached
from flexbe_msgs.msg import CommandFeedback, OutcomeRequest
from flexbe_core.logger import Logger

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus


class MonitoringState(smach.State):
    """
    A state to monitor a custom set of conditions.
    For each conditions, an outcome is added or mapped which will be returned if the condition is not met. 
    """

    def __init__(self, *args, **kwargs):
        super(MonitoringState, self).__init__(*args, **kwargs)

        outcomes = kwargs.get('outcomes', [])
        self._outcome_list = list(outcomes)
        self._outcome_list.remove('loopback')

        self.name = None
        self._parent = None
        self._is_controlled = False
        self._force_monitoring = False

        self._monitored_keys = dict()
        self._sent_keys = list()
        self._current_status = None
        
        self.__execute = self.execute
        self.execute = self._monitoring_execute

        self._diagnostics_topic = 'diagnostics_agg'
        self._sub = ProxySubscriberCached()


    def _monitoring_execute(self, *args, **kwargs):
        new_status = None
        had_warning = False
        if (self._force_monitoring or self._is_controlled) and self._sub.has_buffered(self._diagnostics_topic):
            new_status = ""
            diag_msg = self._sub.get_from_buffer(self._diagnostics_topic)
            for status in diag_msg.status:
                if not status.name in self._monitored_keys.keys(): continue
                if status.level == DiagnosticStatus.WARN:
                    had_warning = True
                    if not status.name + "_warn" in self._sent_keys:
                        self._sent_keys.append(status.name + "_warn")
                        Logger.logwarn("%s: %s" % (status.name, status.message))
                if status.level == DiagnosticStatus.ERROR:
                    if not status.name + "_err" in self._sent_keys:
                        self._sent_keys.append(status.name + "_err")
                        Logger.logerr("%s: %s" % (status.name, status.message))
                    new_status = status.name

        if new_status == "":
            self._current_status = None
            new_status = None
            if not had_warning:
                self._sent_keys = list()

        if new_status is None or self._current_status == new_status:
            return self.__execute(*args, **kwargs)

        self._current_status = new_status

        return self._monitored_keys[self._current_status]


    def monitor(self, key, outcome = None):
        oc = outcome if not outcome is None else key
        self._monitored_keys[key] = oc
        if not oc in self._outcomes:
            self.register_outcomes([oc])
            self._outcome_list.append(oc)

    def force_monitoring(self):
        self._force_monitoring = True
        if not self._is_controlled:
            self._sub.subscribe(self._diagnostics_topic, DiagnosticArray)
            self._sub.enable_buffer(self._diagnostics_topic)

    def _enable_ros_control(self):
        self._is_controlled = True
        self._sub.subscribe(self._diagnostics_topic, DiagnosticArray)
        self._sub.enable_buffer(self._diagnostics_topic)

    def _disable_ros_control(self):
        self._is_controlled = False
        self._sub.unsubscribe_topic(self._diagnostics_topic)


    def _get_path(self):
        return self._parent._get_path() + "/" + self.name
