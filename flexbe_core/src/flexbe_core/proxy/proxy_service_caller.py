#!/usr/bin/env python
import rospy
from threading import Timer

from flexbe_core.logger import Logger


class ProxyServiceCaller(object):
    """
    A proxy for calling services.
    """
    _services = {}

    def __init__(self, topics={}, persistent=False, wait_duration=10):
        """
        Initializes the proxy with optionally a given set of clients.

        @type topics: dictionary string - message class
        @param topics: A dictionay containing a collection of topic - message type pairs.

        @type persistent: bool
        @param persistent: Defines if the service callers are persistent.

        @type wait_duration: int
        @param wait_duration: Defines how long to wait for the given services if not available right now.
        """
        for topic, msg_type in topics.items():
            self.setupService(topic, msg_type, persistent, wait_duration)

    def setupService(self, topic, msg_type, persistent=False, wait_duration=10):
        """
        Tries to set up a service caller for calling it later.

        @type topic: string
        @param topic: The topic of the service to call.

        @type msg_type: service class
        @param msg_type: The type of messages of this service.

        @type persistent: bool
        @param persistent: Defines if this service caller is persistent.

        @type wait_duration: int
        @param wait_duration: Defines how long to wait for the given service if it is not available right now.
        """
        if topic not in ProxyServiceCaller._services:
            ProxyServiceCaller._services[topic] = rospy.ServiceProxy(topic, msg_type, persistent)
            self._check_service_available(topic, wait_duration)

    def is_available(self, topic):
        """
        Checks if the service on the given topic is available.

        @type topic: string
        @param topic: The topic of interest.
        """
        return self._check_service_available(topic)

    def call(self, topic, request):
        """
        Performs a service call on the given topic.

        @type topic: string
        @param topic: The topic to call.

        @type request: service
        @param request: The request to send to this service.
        """
        if not self._check_service_available(topic):
            raise ValueError('Cannot call service client %s: Topic not available.' % topic)
        # call service (forward any exceptions)
        return ProxyServiceCaller._services[topic].call(request)

    def _check_service_available(self, topic, wait_duration=1):
        """
        Checks whether a service is available.

        @type topic: string
        @param topic: The topic of the service.

        @type wait_duration: int
        @param wait_duration: Defines how long to wait for the given service if it is not available right now.
        """
        client = ProxyServiceCaller._services.get(topic)
        if client is None:
            Logger.logerr("Service client %s not yet registered, need to add it first!" % topic)
            return False
        warning_sent = False
        available = False
        try:
            t = Timer(1, self._print_wait_warning, [topic])
            t.start()
            rospy.wait_for_service(topic, wait_duration)
            available = True
        except rospy.exceptions.ROSException:
            available = False

        try:
            t.cancel()
        except Exception:
            # already printed the warning
            warning_sent = True

        if not available:
            Logger.logerr("Service client %s timed out!" % topic)
            return False
        else:
            if warning_sent:
                Logger.loginfo("Finally found service %s..." % (topic))
        return True

    def _print_wait_warning(self, topic):
        Logger.logwarn("Waiting for service %s..." % (topic))
