#!/usr/bin/env python

import roslib; roslib.load_manifest('flexbe_input')
import rospy
import pickle
import actionlib
import threading

from flexbe_msgs.msg import BehaviorInputAction, BehaviorInputFeedback, BehaviorInputResult, BehaviorInputGoal
from complex_action_server import ComplexActionServer
'''
Created on 02/13/2015

@author: Philipp Schillinger, Brian Wright
'''

class BehaviorInput(object):

	def __init__(self):
		'''
		Constructor
		'''
		#onboard connection
		self._as = ComplexActionServer('flexbe/behavior_input', BehaviorInputAction, execute_cb=self.execute_cb, auto_start = False)
		#self._as = actionlib.SimpleActionServer('flexbe/behavior_input', BehaviorInputAction, execute_cb=self.execute_cb, auto_start = False)
		self._as.start()

		rospy.loginfo("Ready for data requests...")			

	def execute_cb(self, goal , goal_handle):
		rospy.loginfo("--> Got a request!")
		rospy.loginfo('"%s"' % goal.msg)
	
		relay_ocs_client_ = actionlib.SimpleActionClient('flexbe/operator_input', BehaviorInputAction)
			
		# wait for data msg
		print "waiting"
		relay_ocs_client_.wait_for_server()
		print "done"

		# Fill in the goal here
		relay_ocs_client_.send_goal(goal)
		print "waiting for result"
		relay_ocs_client_.wait_for_result()
		print "got result"

		result = BehaviorInputResult()
		result = relay_ocs_client_.get_result()
		#result.data now serialized
		data_str = result.data	
		print data_str
		
		if(result.result_code == BehaviorInputResult.RESULT_OK):
			self._as.set_succeeded(BehaviorInputResult(result_code=BehaviorInputResult.RESULT_OK, data=data_str), "ok",goal_handle)

		elif(result.result_code == BehaviorInputResult.RESULT_FAILED):
			# remove
			#data_str = "Request %d not yet implemented." % (goal.request_type)
			self._as.set_succeeded(BehaviorInputResult(result_code=BehaviorInputResult.RESULT_FAILED, data=data_str),"failed",goal_handle)
			rospy.loginfo("<-- Replied with FAILED")

		elif(result.result_code == BehaviorInputResult.RESULT_ABORTED ):
			#data_str = "ABORT" % (goal.request_type)
			self._as.set_succeeded(BehaviorInputResult(result_code=BehaviorInputResult.RESULT_ABORTED, data=data_str),"Aborted",goal_handle)
			rospy.loginfo("<-- Replied with ABORT")


















