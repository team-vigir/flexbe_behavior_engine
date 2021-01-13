#!/usr/bin/env python
import rospy
from rospy.exceptions import ROSInterruptException
from flexbe_core import EventState

from flexbe_core.proxy import ProxyPublisher, ProxySubscriberCached
from std_msgs.msg import String, UInt8


class MirrorState(EventState):
    '''
    This state will display its possible outcomes as buttons in the GUI
    and is designed in a way to be created dynamically.
    '''

    def __init__(self, target_name, target_path, given_outcomes, outcome_autonomy):
        super(MirrorState, self).__init__(outcomes=given_outcomes)
        self.set_rate(100)
        self._target_name = target_name
        self._target_path = target_path

        self._outcome_topic = 'flexbe/mirror/outcome'

        self._pub = ProxyPublisher()
        self._sub = ProxySubscriberCached({self._outcome_topic: UInt8})

    def execute(self, userdata):
        if self._sub.has_buffered(self._outcome_topic):
            msg = self._sub.get_from_buffer(self._outcome_topic)
            if msg.data < len(self.outcomes):
                rospy.loginfo("State update: %s > %s", self._target_name, self.outcomes[msg.data])
                return self.outcomes[msg.data]
        try:
            self.sleep()
        except ROSInterruptException:
            print('Interrupted mirror sleep.')

    def on_enter(self, userdata):
        self._pub.publish('flexbe/behavior_update', String("/" + "/".join(self._target_path.split("/")[1:])))
