#!/usr/bin/env python

import rospy
import actionlib
import pickle

from flexbe_core import EventState, Logger
from flexbe_msgs.msg import BehaviorInputAction, BehaviorInputGoal, BehaviorInputResult
from flexbe_core.proxy import ProxyActionClient

'''
Created on 02/13/2015

@author: Philipp Schillinger
'''

class InputState(EventState):
	'''
	Implements a state where the state machine needs an input from the operator.
	Requests of different types, such as requesting a waypoint, a template, or a pose, can be specified.

	-- request 	uint8 		One of the class constants to specify the type of request.
	-- message 	string 		Message displayed to the operator to let him know what to do.

	#> data 	object 		The data provided by the operator. The exact type depends on the request.

	<= received 			Returned as soon as valid data is available.
	<= aborted 				The operator declined to provide the requested data.
	<= no_connection 		No request could be sent to the operator.
	<= data_error 			Data has been received, but could not be deserialized successfully.

	'''

	POINT_LOCATION			= BehaviorInputGoal.POINT_LOCATION
	SELECTED_OBJECT_ID		= BehaviorInputGoal.SELECTED_OBJECT_ID
	WAYPOINT_GOAL_POSE		= BehaviorInputGoal.WAYPOINT_GOAL_POSE
	GHOST_JOINT_STATES		= BehaviorInputGoal.GHOST_JOINT_STATES
	FOOTSTEP_PLAN_HEADER	= BehaviorInputGoal.FOOTSTEP_PLAN_HEADER


	def __init__(self, request, message):
		'''
		Constructor
		'''
		super(InputState, self).__init__(outcomes=['received', 'aborted', 'no_connection', 'data_error'],
												output_keys=['data'])
		
		self._action_topic = 'flexbe/behavior_input'

		self._client = ProxyActionClient({self._action_topic: BehaviorInputAction})

		self._request = request
		self._message = message
		self._connected = True
		self._received = False
		
		
	def execute(self, userdata):
		'''
		Execute this state
		'''
		if not self._connected:
			return 'no_connection'
		if self._received:
			return 'received'

		if self._client.has_result(self._action_topic):
			result = self._client.get_result(self._action_topic)
			if result.result_code != BehaviorInputResult.RESULT_OK:
				userdata.data = None
				return 'aborted'
			else:
				try:
					response_data = pickle.loads(result.data)
					#print 'Input request response:', response_data
					userdata.data = response_data
					# If this was a template ID request, log that template ID:
					if self._request == BehaviorInputGoal.SELECTED_OBJECT_ID:
						Logger.loginfo('Received template ID: %d' % int(response_data))
				
				except Exception as e:
					Logger.logwarn('Was unable to load provided data:\n%s' % str(e))
					userdata.data = None
					return 'data_error'

				self._received = True
				return 'received'
			
	
	def on_enter(self, userdata):

		self._received = False

		if not self._connected: return

		action_goal = BehaviorInputGoal()
		action_goal.request_type = self._request
		action_goal.msg = self._message

		try:
			self._client.send_goal(self._action_topic, action_goal)
		except Exception as e:
			Logger.logwarn('Was unable to send data request:\n%s' % str(e))
			self._connected = False
	
	













