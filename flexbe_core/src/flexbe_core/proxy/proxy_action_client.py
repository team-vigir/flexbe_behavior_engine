#!/usr/bin/env python

import roslib; roslib.load_manifest('flexbe_core')
import rospy
import actionlib
from threading import Timer
import time

from flexbe_core.logger import Logger


class ProxyActionClient(object):
    """
    A proxy for calling actions.
    """
    _clients = {}

    def __init__(self, topics = {}):
        """
        Initializes the proxy with optionally a given set of clients.
        
        @type topics: dictionary string - message class
        @param topics: A dictionay containing a collection of topic - message type pairs.
        """

        for topic, msg_type in topics.iteritems():
            self.setupClient(topic, msg_type)
    

    def setupClient(self, topic, msg_type, wait_duration=10):
        """
        Tries to set up an action client for calling it later.
        
        @type topic: string
        @param topic: The topic of the action to call.
        
        @type msg_type: msg type
        @param msg_type: The type of messages of this action client.
        
        @type wait_duration: int
        @param wait_duration: Defines how long to wait for the given client if it is not available right now.
        """
        if topic not in ProxyActionClient._clients:
            client = actionlib.SimpleActionClient(topic, msg_type)
            t = Timer(1, self._print_warning, [topic])
            t.start()
            available = client.wait_for_server(rospy.Duration.from_sec(wait_duration))
            warning_sent = False
            try:
                t.cancel()
            except Exception as ve:
                # already printed the warning
                warning_sent = True

            if not available:
                Logger.logerr("Action client %s timed out!" % topic)
            else:
                ProxyActionClient._clients[topic] = client
                if warning_sent:
                    Logger.loginfo("Finally found action client %s..." % (topic))
            
            
    def send_goal(self, topic, goal):
        """
        Performs an action call on the given topic.
        
        @type topic: string
        @param topic: The topic to call.
        
        @type goal: action goal
        @param goal: The request to send to the action server.
        """
        if topic not in ProxyActionClient._clients:
            raise ValueError('ProxyActionClient: topic %s not yet registered!' % topic)
        
        ProxyActionClient._clients[topic].send_goal(goal)


    def has_result(self, topic):
        return ProxyActionClient._clients[topic].wait_for_result(rospy.Duration.from_sec(0.1))

    def get_result(self, topic):
        return ProxyActionClient._clients[topic].get_result()

    def cancel(self, topic):
        ProxyActionClient._clients[topic].cancel_goal()

    def _print_warning(self, topic):
        Logger.logwarn("Waiting for action client %s..." % (topic))
