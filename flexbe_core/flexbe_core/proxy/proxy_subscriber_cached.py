from functools import partial
from collections import defaultdict

from flexbe_core.logger import Logger
from flexbe_core.proxy.qos import QOS_DEFAULT


class ProxySubscriberCached(object):
    """
    A proxy for subscribing topics that caches and buffers received messages.
    """
    _node = None
    _topics = {}
    _persistant_topics = []

    def _initialize(node):
        ProxySubscriberCached._node = node

    def __init__(self, topics={}, qos=None):
        """
        Initializes the proxy with optionally a given set of topics.

        @type topics: dictionary string - message_class
        @param topics: A dictionary containing a collection of topic - message type pairs.
        """
        for topic, msg_type in topics.items():
            self.subscribe(topic, msg_type, qos=qos)

    def subscribe(self, topic, msg_type, callback=None, buffered=False, qos=None):
        """
        Adds a new subscriber to the proxy.

        @type topic: string
        @param topic: The topic to subscribe.

        @type msg_type: a message class
        @param msg_type: The type of messages of this topic.

        @type callback: function
        @param callback: A function to be called when receiving messages.

        @type buffered: boolean
        @param buffered: True if all messages should be bufferd, False if only the last message should be cached.
        """
        if topic not in ProxySubscriberCached._topics:
            qos = qos or QOS_DEFAULT
            sub = ProxySubscriberCached._node.create_subscription(msg_type, topic,
                                                                  partial(self._callback, topic=topic), qos)
            ProxySubscriberCached._topics[topic] = {'subscriber': sub,
                                                    'last_msg': None,
                                                    'buffered': buffered,
                                                    'msg_queue': [],
                                                    'callbacks': defaultdict(list)}
        if callback is not None:
            ProxySubscriberCached._topics[topic]['callbacks'].append(callback)

    def _callback(self, msg, topic):
        """
        Standard callback that is executed when a message is received.

        @type topic: message
        @param topic: The latest message received on this topic.

        @type topic: string
        @param topic: The topic to which this callback belongs.
        """
        if topic not in ProxySubscriberCached._topics:
            return
        ProxySubscriberCached._topics[topic]['last_msg'] = msg
        if ProxySubscriberCached._topics[topic]['buffered']:
            ProxySubscriberCached._topics[topic]['msg_queue'].append(msg)
        for callback in ProxySubscriberCached._topics[topic]['callbacks']:
            callback(msg)

    def set_callback(self, topic, callback):
        """
        Adds the given callback to the topic subscriber.

        @type topic: string
        @param topic: The topic to add the callback to.

        @type callback: function
        @param callback: The callback to be added.
        """
        ProxySubscriberCached._topics[topic]['callbacks'].append(callback)

    def enable_buffer(self, topic):
        """
        Enables the buffer on the given topic.

        @type topic: string
        @param topic: The topic of interest.
        """
        ProxySubscriberCached._topics[topic]['buffered'] = True

    def disable_buffer(self, topic):
        """
        Disables the buffer on the given topic.

        @type topic: string
        @param topic: The topic of interest.
        """
        ProxySubscriberCached._topics[topic]['buffered'] = False
        ProxySubscriberCached._topics[topic]['msg_queue'] = []

    def is_available(self, topic):
        """
        Checks if the subscriber on the given topic is available.

        @type topic: string
        @param topic: The topic of interest.
        """
        return topic in ProxySubscriberCached._topics

    def get_last_msg(self, topic):
        """
        Returns the latest cached message of the given topic.

        @type topic: string
        @param topic: The topic of interest.
        """
        return ProxySubscriberCached._topics[topic]['last_msg']

    def get_from_buffer(self, topic):
        """
        Pops the oldest buffered message of the given topic.

        @type topic: string
        @param topic: The topic of interest.
        """
        if not ProxySubscriberCached._topics[topic]['buffered']:
            Logger.warning('Attempted to access buffer of non-buffered topic!')
            return None
        if len(ProxySubscriberCached._topics[topic]['msg_queue']) == 0:
            return None
        msg = ProxySubscriberCached._topics[topic]['msg_queue'][0]
        ProxySubscriberCached._topics[topic]['msg_queue'] = ProxySubscriberCached._topics[topic]['msg_queue'][1:]
        return msg

    def has_msg(self, topic):
        """
        Determines if the given topic has a message in its cache.

        @type topic: string
        @param topic: The topic of interest.
        """
        return ProxySubscriberCached._topics[topic]['last_msg'] is not None

    def has_buffered(self, topic):
        """
        Determines if the given topic has any messages in its buffer.

        @type topic: string
        @param topic: The topic of interest.
        """
        return len(ProxySubscriberCached._topics[topic]['msg_queue']) > 0

    def remove_last_msg(self, topic, clear_buffer=False):
        """
        Removes the cached message of the given topic and optionally clears its buffer.

        @type topic: string
        @param topic: The topic of interest.

        @type topic: boolean
        @param topic: Set to true if the buffer of the given topic should be cleared as well.
        """
        if topic in ProxySubscriberCached._persistant_topics:
            return
        ProxySubscriberCached._topics[topic]['last_msg'] = None
        if clear_buffer:
            ProxySubscriberCached._topics[topic]['msg_queue'] = []

    def make_persistant(self, topic):
        """
        Makes the given topic persistant which means messages can no longer be removed
        (remove_last_msg will have no effect), only overwritten by a new message.

        @type topic: string
        @param topic: The topic of interest.
        """
        if topic not in ProxySubscriberCached._persistant_topics:
            ProxySubscriberCached._persistant_topics.append(topic)

    def has_topic(self, topic):
        """
        Determines if the given topic is already subscribed.

        @type topic: string
        @param topic: The topic of interest.
        """
        Logger.warning('Deprecated (ProxySubscriberCached): use "is_available(topic)" instead of "has_topic(topic)".')
        return self.is_available(topic)

    def unsubscribe_topic(self, topic):
        """
        Removes the given topic from the list of subscribed topics.

        @type topic: string
        @param topic: The topic of interest.
        """
        if topic in ProxySubscriberCached._topics:
            ProxySubscriberCached._topics[topic]['subscriber'].unregister()
            ProxySubscriberCached._topics.pop(topic)

    def shutdown(self):
        """ Shuts this proxy down by unregistering all subscribers. """
        try:
            for topic in ProxySubscriberCached._topics:
                try:
                    ProxySubscriberCached._topics[topic]['subscriber'].unregister()
                except Exception as e:
                    rospy.logerr('Something went wrong during shutdown of proxy subscriber!\n%s', str(e))
        except Exception as e:
            rospy.logerr('Something went wrong during shutdown of proxy subscriber!\n%s', str(e))
        ProxySubscriberCached._topics.clear()
