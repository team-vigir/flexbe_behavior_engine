#!/usr/bin/env python


from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyPublisher
from std_msgs.msg import String

'''
Created on 9.11.2017

@author: Alireza Hosseini
'''


class PublisherStringState(EventState):
    '''
	Publishes a string (std_msgs/String) message on a given topic name.

	-- topic	string		The topic on which should be published.

	>= value 					Value of string.

	<= done 					Done publishing.

	'''

    def __init__(self, topic):
        '''
		Constructor
		'''
        super(PublisherStringState, self).__init__(outcomes=['done'], input_keys=['value'])

        self._topic = topic
        self._pub = ProxyPublisher({self._topic: String})

    def execute(self, userdata):
        return 'done'

    def on_enter(self, userdata):
        val = String()
        val.data = userdata.value
        self._pub.publish(self._topic, val)
