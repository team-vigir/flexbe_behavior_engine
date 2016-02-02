#!/usr/bin/env python

import roslib; roslib.load_manifest('flexbe_core')
import rospy
from flexbe_msgs.msg import BEStatus

import time
import random


class ProxyPublisher(object):
    """
    A proxy for publishing topics.
    """
    _simulate_delay = False
    
    _topics = {}
    
    def __init__(self, topics = {}, _latch=False):
        """
        Initializes the proxy with optionally a given set of topics.
        Automatically creates a publisher for sending status messages.
        
        @type topics: dictionary string - message class
        @param topics: A dictionay containing a collection of topic - message type pairs.
        
        @type _latch: bool
        @param: _latch: Defines if messages on the given topics should be latched.
        """
        for topic, msg_type in topics.iteritems():
            self.createPublisher(topic, msg_type, _latch)
            
    
    def createPublisher(self, topic, msg_type, _latch = False):
        """
        Adds a new publisher to the proxy.
        
        @type topic: string
        @param topic: The topic to publish on.
        
        @type msg_type: a message class
        @param msg_type: The type of messages of this topic.
        
        @type _latch: bool
        @param: _latch: Defines if messages on the given topics should be latched.
        """
        if topic not in ProxyPublisher._topics:
            pub = rospy.Publisher(topic, msg_type, latch = _latch, queue_size=100)
            ProxyPublisher._topics[topic] = pub
            

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
        if ProxyPublisher._simulate_delay:
            time.sleep(max(0, random.gauss(1.5, 1))) # for simulating comms_bridge delay

        if topic not in ProxyPublisher._topics:
            rospy.logwarn('ProxyPublisher: topic '+ topic +' not yet registered!')
            return
        try:
            ProxyPublisher._topics[topic].publish(msg)
        except Exception as e:
            rospy.logwarn('Something went wrong when publishing to %s!\n%s', topic, str(e))
            
    
    def wait_for_any(self, topic, timeout=5.0):
        """
        Blocks until there are any subscribers to the given topic.
        
        @type topic: string
        @param topic: The topic to publish on.
        
        @type timeout: float
        @param timeout: How many seconds should be the maximum blocked time.
        """
        pub = ProxyPublisher._topics[topic]
        starting_time = rospy.get_rostime()
        while pub.get_num_connections() < 1:
            elapsed = rospy.get_rostime() - starting_time
            if elapsed.to_sec() >= timeout:
                return False
        return True
