#!/usr/bin/env python
from flexbe_core import EventState
from flexbe_core.proxy import ProxyPublisher
from std_msgs.msg import Empty


class PublisherEmptyState(EventState):
    '''
    Publishes an empty (std_msgs/Empty) message on a given topic name.

    -- topic    string      The topic on which should be published.

    <= done                 Done publishing.
    '''

    def __init__(self, topic):
        super(PublisherEmptyState, self).__init__(outcomes=['done'])
        self._topic = topic
        self._pub = ProxyPublisher({self._topic: Empty})

    def execute(self, userdata):
        return 'done'

    def on_enter(self, userdata):
        self._pub.publish(self._topic, Empty())
