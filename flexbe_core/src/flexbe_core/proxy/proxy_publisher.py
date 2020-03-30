#!/usr/bin/env python
import rospy
from threading import Timer

from flexbe_core.logger import Logger


class ProxyPublisher(object):
    """
    A proxy for publishing topics.
    """
    _topics = {}

    def __init__(self, topics={}, _latch=False, _queue_size=100):
        """
        Initializes the proxy with optionally a given set of topics.
        Automatically creates a publisher for sending status messages.

        @type topics: dictionary string - message class
        @param topics: A dictionay containing a collection of topic - message type pairs.

        @type _latch: bool
        @param: _latch: Defines if messages on the given topics should be latched.

        @type _queue_size: int
        @param: _queue_size: Defines the queue size of the new publishers.
        """
        for topic, msg_type in topics.items():
            self.createPublisher(topic, msg_type, _latch, _queue_size)

    def createPublisher(self, topic, msg_type, _latch=False, _queue_size=100):
        """
        Adds a new publisher to the proxy.

        @type topic: string
        @param topic: The topic to publish on.

        @type msg_type: a message class
        @param msg_type: The type of messages of this topic.

        @type _latch: bool
        @param: _latch: Defines if messages on the given topics should be latched.

        @type _queue_size: int
        @param: _queue_size: Defines the queue size of the publisher.
        """
        if topic not in ProxyPublisher._topics:
            ProxyPublisher._topics[topic] = rospy.Publisher(topic, msg_type, latch=_latch, queue_size=_queue_size)

    def is_available(self, topic):
        """
        Checks if the publisher on the given topic is available.

        @type topic: string
        @param topic: The topic of interest.
        """
        return topic in ProxyPublisher._topics

    def publish(self, topic, msg):
        """
        Publishes a message on the specified topic.

        @type topic: string
        @param topic: The topic to publish on.

        @type msg: message class (defined when created publisher)
        @param msg: The message to publish.
        """
        if topic not in ProxyPublisher._topics:
            Logger.logwarn('ProxyPublisher: topic %s not yet registered!' % topic)
            return
        try:
            ProxyPublisher._topics[topic].publish(msg)
        except Exception as e:
            Logger.logwarn('Something went wrong when publishing to %s!\n%s' % (topic, str(e)))

    def wait_for_any(self, topic, timeout=5.0):
        """
        Blocks until there are any subscribers to the given topic.

        @type topic: string
        @param topic: The topic to publish on.

        @type timeout: float
        @param timeout: How many seconds should be the maximum blocked time.
        """
        pub = ProxyPublisher._topics.get(topic)
        if pub is None:
            Logger.logerr("Publisher %s not yet registered, need to add it first!" % topic)
            return False
        t = Timer(.5, self._print_wait_warning, [topic])
        t.start()
        available = self._wait_for_subscribers(pub, timeout)
        warning_sent = False
        try:
            t.cancel()
        except Exception:
            # already printed the warning
            warning_sent = True

        if not available:
            Logger.logerr("Waiting for subscribers on %s timed out!" % topic)
            return False
        else:
            if warning_sent:
                Logger.loginfo("Finally found subscriber on %s..." % (topic))
        return True

    def _print_wait_warning(self, topic):
        Logger.logwarn("Waiting for subscribers on %s..." % (topic))

    def _wait_for_subscribers(self, pub, timeout=5.0):
        starting_time = rospy.get_rostime()
        rate = rospy.Rate(100)
        while not rospy.is_shutdown():
            elapsed = rospy.get_rostime() - starting_time
            if elapsed.to_sec() >= timeout:
                return False
            if pub.get_num_connections() >= 1:
                return True
            rate.sleep()
        return False
