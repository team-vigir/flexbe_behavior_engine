#!/usr/bin/env python
import rospy
import rostopic
import inspect
from flexbe_core import EventState, Logger

from flexbe_core.proxy import ProxySubscriberCached

'''
Created on 11.06.2013

@author: Philipp Schillinger
'''

class SubscriberState(EventState):
	'''
	Gets the latest message on the given topic and stores it to userdata.

	-- topic 		string		The topic on which should be listened.
	-- blocking 	bool 		Blocks until a message is received.
	-- clear 		bool 		Drops last message on this topic on enter
								in order to only handle message received since this state is active.

	#> message		object		Latest message on the given topic of the respective type.

	<= received 				Message has been received and stored in userdata or state is not blocking.
	<= unavailable 				The topic is not available when this state becomes actives.

	'''


	def __init__(self, topic, blocking = True, clear = False):
		'''
		Constructor
		'''
		super(SubscriberState, self).__init__(outcomes=['received', 'unavailable'],
											output_keys=['message'])
		
		self._topic = topic
		self._blocking = blocking
		self._clear = clear
		self._connected = False

		(msg_path, msg_topic, fn) = rostopic.get_topic_type(self._topic)

		if msg_topic == self._topic:
			msg_type = self._get_msg_from_path(msg_path)
			self._sub = ProxySubscriberCached({self._topic: msg_type})
			self._connected = True
		else:
			Logger.logwarn('Topic %s for state %s not yet available.\nFound: %s\nWill try again when entering the state...' % (self._topic, self.name, str(msg_topic)))
		
		
	def execute(self, userdata):
		'''
		Execute this state
		'''
		if not self._connected:
			userdata.message = None
			return 'unavailable'

		if self._sub.has_msg(self._topic) or not self._blocking:
			userdata.message = self._sub.get_last_msg(self._topic)
			self._sub.remove_last_msg(self._topic)
			return 'received'
			
	
	def on_enter(self, userdata):
		if not self._connected:
			(msg_path, msg_topic, fn) = rostopic.get_topic_type(self._topic)
			if msg_topic == self._topic:
				msg_type = self._get_msg_from_path(msg_path)
				self._sub = ProxySubscriberCached({self._topic: msg_type})
				self._connected = True
				Logger.loginfo('Successfully subscribed to previously unavailable topic %s' % self._topic)
			else:
				Logger.logwarn('Topic %s still not available, giving up.' % self._topic)

		if self._connected and self._clear and self._sub.has_msg(self._topic):
			self._sub.remove_last_msg(self._topic)


	def _get_msg_from_path(self, msg_path):
		msg_import = msg_path.split('/')
		msg_module = '%s.msg' % (msg_import[0])
		package = __import__(msg_module, fromlist=[msg_module])
		clsmembers = inspect.getmembers(package, lambda member: inspect.isclass(member) and member.__module__.endswith(msg_import[1]))
		return clsmembers[0][1]
