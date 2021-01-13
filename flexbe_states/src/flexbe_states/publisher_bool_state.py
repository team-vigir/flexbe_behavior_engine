#!/usr/bin/env python
from flexbe_core import EventState
from flexbe_core.proxy import ProxyPublisher
from std_msgs.msg import Bool


class PublisherBoolState(EventState):
    '''
    Publishes an empty (std_msgs/Bool) message on a given topic name.

    -- topic    string  The topic on which should be published.

    >= value            Value of bool.

    <= done             Done publishing.
    '''

    def __init__(self, topic):
        super(PublisherBoolState, self).__init__(outcomes=['done'], input_keys=['value'])
        self._topic = topic
        self._pub = ProxyPublisher({self._topic: Bool})

    def execute(self, userdata):
        return 'done'

    def on_enter(self, userdata):
        val = Bool()
        val.data = userdata.value
        self._pub.publish(self._topic, val)
