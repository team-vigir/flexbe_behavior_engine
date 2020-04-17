#!/usr/bin/env python
from flexbe_core import EventState
from flexbe_core.proxy import ProxyPublisher
from std_msgs.msg import String


class PublisherStringState(EventState):
    '''
    Publishes a string (std_msgs/String) message on a given topic name.

    -- topic	string		The topic on which should be published.

    >= value 					Value of string.

    <= done 					Done publishing.
    '''

    def __init__(self, topic):
        super(PublisherStringState, self).__init__(outcomes=['done'], input_keys=['value'])
        self._topic = topic
        self._pub = ProxyPublisher({self._topic: String})

    def execute(self, userdata):
        return 'done'

    def on_enter(self, userdata):
        val = String()
        val.data = userdata.value
        self._pub.publish(self._topic, val)
