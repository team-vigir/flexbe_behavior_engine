#!/usr/bin/env python

import roslib; roslib.load_manifest('flexbe_core')
import rospy
from threading import Timer
import time

from flexbe_core.logger import Logger


class ProxyServiceCaller(object):
    """
    A proxy for calling services.
    """
    _services = {}

    def __init__(self, topics = {}):
        """
        Initializes the proxy with optionally a given set of clients.
        
        @type topics: dictionary string - message class
        @param topics: A dictionay containing a collection of topic - message type pairs.
        """

        for topic, msg_type in topics.iteritems():
            self.setupService(topic, msg_type)
    

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
            warning_sent = False
            available = False
            try:
                t = Timer(1, self._print_wait_warning, [topic])
                t.start()
                rospy.wait_for_service(topic, wait_duration)
                available = True
            except rospy.exceptions.ROSException, e:
                available = False

            try:
                t.cancel()
            except Exception as ve:
                # already printed the warning
                warning_sent = True

            if not available:
                Logger.logerr("Service client %s timed out!" % topic)
            else:
                ProxyServiceCaller._services[topic] = rospy.ServiceProxy(topic, msg_type, persistent)
                if warning_sent:
                    Logger.loginfo("Finally found action client %s..." % (topic))


    def is_available(self, topic):
        """
        Checks if the service on the given topic is available.
        
        @type topic: string
        @param topic: The topic of interest.
        """
        return topic in ProxyServiceCaller._services
            
            
    def call(self, topic, request):
        """
        Performs a service call on the given topic.
        
        @type topic: string
        @param topic: The topic to call.
        
        @type request: service
        @param request: The request to send to this service.
        """
        if topic not in ProxyServiceCaller._services:
            rospy.logwarn('ProxyServiceCaller: topic not yet registered!')
            return
        
        try:
            return ProxyServiceCaller._services[topic].call(request)
        except Exception, e:
            print 'error: ' + str(e)
            raise


    def _print_wait_warning(self, topic):
        Logger.logwarn("Waiting for service client %s..." % (topic))
