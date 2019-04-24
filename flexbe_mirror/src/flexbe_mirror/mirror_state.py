#!/usr/bin/env python

import roslib; roslib.load_manifest('flexbe_mirror')
import rospy
from rospy.exceptions import ROSInterruptException
from flexbe_core import EventState, JumpableStateMachine

from flexbe_core.proxy import ProxyPublisher, ProxySubscriberCached
from std_msgs.msg import String, UInt8

'''
Created on 07.05.2013

@author: Philipp Schillinger
'''

class MirrorState(EventState):
    '''
    This state will display its possible outcomes as buttons in the GUI and is designed in a way to be created dynamically.
    '''


    def __init__(self, target_name, target_path, given_outcomes, outcome_autonomy):
        '''
        Constructor
        '''
        super(MirrorState, self).__init__(outcomes=given_outcomes)
        self._rate = rospy.Rate(100)
        self._given_outcomes = given_outcomes
        self._outcome_autonomy = outcome_autonomy
        self._target_name = target_name
        self._target_path = target_path
        
        self._outcome_topic = 'flexbe/mirror/outcome'

        self._pub = ProxyPublisher() #{'flexbe/behavior_update': String}
        self._sub = ProxySubscriberCached({self._outcome_topic: UInt8})
        
        
    def execute(self, userdata):
        '''
        Execute this state
        '''
        if JumpableStateMachine.refresh:
            JumpableStateMachine.refresh = False
            self.on_enter(userdata)

        if self._sub.has_buffered(self._outcome_topic):
            msg = self._sub.get_from_buffer(self._outcome_topic)
            if msg.data < len(self._given_outcomes):
                rospy.loginfo("State update: %s > %s", self._target_name, self._given_outcomes[msg.data])
                return self._given_outcomes[msg.data]

        try:
            self._rate.sleep()
        except ROSInterruptException:
            print 'Interrupted mirror sleep.'
    
    
    def on_enter(self, userdata):
        #rospy.loginfo("Mirror entering %s", self._target_path)
        self._pub.publish('flexbe/behavior_update', String("/" + "/".join(self._target_path.split("/")[1:])))
    
    
    
    
    
    
    
