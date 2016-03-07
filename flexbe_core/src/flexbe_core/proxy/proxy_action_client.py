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

    _result = {}
    _feedback = {}

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
            t = Timer(1, self._print_wait_warning, [topic])
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
        
        ProxyActionClient._result[topic] = None
        ProxyActionClient._feedback[topic] = None
        
        ProxyActionClient._clients[topic].send_goal(goal,
            done_cb = lambda ts, r: self._done_callback(topic, ts, r),
            feedback_cb = lambda f: self._feedback_callback(topic, f)
        )

    def _done_callback(self, topic, terminal_state, result):
        ProxyActionClient._result[topic] = result

    def _feedback_callback(self, topic, feedback):
        ProxyActionClient._feedback[topic] = feedback


    def is_available(self, topic):
        """
        Checks if the client on the given action topic is available.
        
        @type topic: string
        @param topic: The topic of interest.
        """
        return topic in ProxyActionClient._clients

    def has_result(self, topic):
        """
        Checks if the given action call already has a result.
        
        @type topic: string
        @param topic: The topic of interest.
        """
        return ProxyActionClient._result[topic] is not None

    def get_result(self, topic):
        """
        Returns the result message of the given action call.
        
        @type topic: string
        @param topic: The topic of interest.
        """
        return ProxyActionClient._result[topic]

    def has_feedback(self, topic):
        """
        Checks if the given action call has any feedback.
        
        @type topic: string
        @param topic: The topic of interest.
        """
        return ProxyActionClient._feedback[topic] is not None

    def get_feedback(self, topic):
        """
        Returns the latest feedback message of the given action call.
        
        @type topic: string
        @param topic: The topic of interest.
        """
        return ProxyActionClient._feedback[topic]

    def get_state(self, topic):
        """
        Determines the current actionlib state of the given action topic.
        A list of possible states is defined in actionlib_msgs/GoalStatus.
        
        @type topic: string
        @param topic: The topic of interest.
        """
        return ProxyActionClient._clients[topic].get_state()

    def is_active(self, topic):
        """
        Determines if an action request is already being processed on the given topic.
        
        @type topic: string
        @param topic: The topic of interest.
        """
        return ProxyActionClient._clients[topic].simple_state != actionlib.SimpleGoalState.DONE

    def cancel(self, topic):
        """
        Cancels the current action call on the given action topic.
        
        @type topic: string
        @param topic: The topic of interest.
        """
        ProxyActionClient._clients[topic].cancel_goal()

    def _print_wait_warning(self, topic):
        Logger.logwarn("Waiting for action client %s..." % (topic))
