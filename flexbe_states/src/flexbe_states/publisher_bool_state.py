#!/usr/bin/env python


from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyPublisher
from std_msgs.msg import Bool

'''
Created on 31.01.2017

@author: Alberto Romay
'''


class PublisherBoolState(EventState):
    '''
	Publishes an empty (std_msgs/Bool) message on a given topic name.

	-- topic	string		The topic on which should be published.

	-- value 					Value of bool.

	<= done 					Done publishing.

	'''

    def __init__(self, topic, value = False):
        '''
		Constructor
		'''
        super(PublisherBoolState, self).__init__(outcomes=['done'])

        self._topic = topic
        self._pub = ProxyPublisher({self._topic: Bool})
        self._value = value

    def execute(self, userdata):
        return 'done'

    def on_enter(self, userdata):
        val = Bool()
        val.data = self._value
        self._pub.publish(self._topic, val)
