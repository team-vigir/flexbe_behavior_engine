#!/usr/bin/env python
import rospy
import rostopic
from flexbe_core import EventState, Logger

from flexbe_core.proxy import ProxySubscriberCached


class SubscriberState(EventState):
    '''
    Gets the latest message on the given topic and stores it to userdata.

    -- topic 		string		The topic on which should be listened.
    -- blocking 	bool 		Blocks until a message is received.
    -- clear 		bool 		Drops last message on this topic on enter
                                in order to only handle message received since this state is active.

    #> message		object		Latest message on the given topic of the respective type.

    <= received 				Message has been received and stored in userdata or state is not blocking.
    <= unavailable 				The topic is not available when this state becomes actives.
    '''

    def __init__(self, topic, blocking=True, clear=False):
        super(SubscriberState, self).__init__(outcomes=['received', 'unavailable'],
                                              output_keys=['message'])
        self._topic = topic
        self._blocking = blocking
        self._clear = clear
        self._connected = False

        if not self._connect():
            Logger.logwarn('Topic %s for state %s not yet available.\n'
                           'Will try again when entering the state...' % (self._topic, self.name))

    def execute(self, userdata):
        if not self._connected:
            userdata.message = None
            return 'unavailable'

        if self._sub.has_msg(self._topic) or not self._blocking:
            userdata.message = self._sub.get_last_msg(self._topic)
            self._sub.remove_last_msg(self._topic)
            return 'received'

    def on_enter(self, userdata):
        if not self._connected:
            if self._connect():
                Logger.loginfo('Successfully subscribed to previously unavailable topic %s' % self._topic)
            else:
                Logger.logwarn('Topic %s still not available, giving up.' % self._topic)

        if self._connected and self._clear and self._sub.has_msg(self._topic):
            self._sub.remove_last_msg(self._topic)

    def _connect(self):
        global_topic = self._topic
        if global_topic[0] != '/':
            global_topic = rospy.get_namespace() + global_topic
        msg_type, msg_topic, _ = rostopic.get_topic_class(global_topic)
        if msg_topic == global_topic:
            self._sub = ProxySubscriberCached({self._topic: msg_type})
            self._connected = True
            return True
        return False
