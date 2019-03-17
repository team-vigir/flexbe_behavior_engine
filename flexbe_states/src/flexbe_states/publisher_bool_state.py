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
	-- latched	bool		Defines if messages on the given topics should be latched.

	>= value 					Value of bool.

	<= done 					Done publishing.

	'''

    def __init__(self, topic, latched=False):
        '''
		Constructor
		'''
        super(PublisherBoolState, self).__init__(outcomes=['done'], input_keys=['value'])

        self._topic = topic
        self._latched = latched
        self._pub = ProxyPublisher({self._topic: Bool}, self._latched)

    def execute(self, userdata):
        return 'done'

    def on_enter(self, userdata):
        val = Bool()
        val.data = userdata.value
        self._pub.publish(self._topic, val)
