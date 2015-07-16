#! /usr/bin/env python
# Copyright (c) 2009, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of the Willow Garage, Inc. nor the names of its
#       contributors may be used to endorse or promote products derived from
#       this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Author: Brian Wright.
# Based on C++ simple_action_server.h by Eitan Marder-Eppstein

import rospy

import threading
import traceback

import Queue

from actionlib_msgs.msg import *

from actionlib import ActionServer
from actionlib.server_goal_handle import ServerGoalHandle;

def nop_cb(goal_handle):
    pass


## @class ComplexActionServer
## @brief The ComplexActionServer
## Operate with concurrent goals in a multi-threaded fashion
class ComplexActionServer:
    ## @brief Constructor for a ComplexActionServer
    ## @param name A name for the action server
    ## @param execute_cb Optional callback that gets called in a separate thread whenever
    ## a new goal is received, allowing users to have blocking callbacks.
    ## Adding an execute callback also deactivates the goalCallback.
    ## @param  auto_start A boolean value that tells the ActionServer wheteher or not to start publishing as soon as it comes up. THIS SHOULD ALWAYS BE SET TO FALSE TO AVOID RACE CONDITIONS and start() should be called after construction of the server.
    def __init__(self, name, ActionSpec, execute_cb = None, auto_start = True):

        self.goals_received_ = 0;
        self.goal_queue_ = Queue.Queue()

        self.new_goal = False

        self.execute_callback = execute_cb;
        self.goal_callback = None;

        self.need_to_terminate = False
        self.terminate_mutex = threading.RLock();

        # since the internal_goal/preempt_callbacks are invoked from the
        # ActionServer while holding the self.action_server.lock
        # self.lock must always be locked after the action server lock
        # to avoid an inconsistent lock acquisition order
        self.lock = threading.RLock();

        self.execute_condition = threading.Condition(self.lock);

        self.current_goal = ServerGoalHandle();
        self.next_goal = ServerGoalHandle();

        if self.execute_callback:
            self.execute_thread = threading.Thread(None, self.executeLoop);
            self.execute_thread.start();
        else:
            self.execute_thread = None
        

        #create the action server
        self.action_server = ActionServer(name, ActionSpec, self.internal_goal_callback,self.internal_preempt_callback,auto_start);


    def __del__(self):
        if hasattr(self, 'execute_callback') and self.execute_callback:
            with self.terminate_mutex:
                self.need_to_terminate = True;

            assert(self.execute_thread);
            self.execute_thread.join();


    ## @brief Accepts a new goal when one is available The status of this
    ## goal is set to active upon acceptance, 
    def accept_new_goal(self):
        with self.action_server.lock, self.lock:

            rospy.logdebug("Accepting a new goal");

            self.goals_received_ -= 1;

			#get from queue 
            current_goal = self.goal_queue_.get() 

            #set the status of the current goal to be active
            current_goal.set_accepted("This goal has been accepted by the simple action server");

            return current_goal#.get_goal();


    ## @brief Allows  polling implementations to query about the availability of a new goal
    ## @return True if a new goal is available, false otherwise
    def is_new_goal_available(self):
        return self.goals_received_ > 0


    ## @brief Allows  polling implementations to query about the status of the current goal
    ## @return True if a goal is active, false otherwise
    def is_active(self):
       if not self.current_goal.get_goal():
           return False;

       status = self.current_goal.get_goal_status().status;
       return status == actionlib_msgs.msg.GoalStatus.ACTIVE #or status == actionlib_msgs.msg.GoalStatus.PREEMPTING;

    ## @brief Sets the status of the active goal to succeeded
    ## @param  result An optional result to send back to any clients of the goal
    def set_succeeded(self,result=None, text="", goal_handle=None):
      with self.action_server.lock, self.lock:
          if not result:
              result=self.get_default_result();
          #self.current_goal.set_succeeded(result, text);
          goal_handle.set_succeeded(result,text)	

    ## @brief Sets the status of the active goal to aborted
    ## @param  result An optional result to send back to any clients of the goal
    def set_aborted(self, result = None, text="" , goal_handle=None):
        with self.action_server.lock, self.lock:
            if not result:
                result=self.get_default_result();
            #self.current_goal.set_aborted(result, text);
            goal_handle.set_aborted(result,text)

    ## @brief Publishes feedback for a given goal
    ## @param  feedback Shared pointer to the feedback to publish
    def publish_feedback(self,feedback):
        self.current_goal.publish_feedback(feedback);


    def get_default_result(self):
        return self.action_server.ActionResultType();

    ## @brief Allows users to register a callback to be invoked when a new goal is available
    ## @param cb The callback to be invoked
    def register_goal_callback(self,cb):
        if self.execute_callback:
            rospy.logwarn("Cannot call ComplexActionServer.register_goal_callback() because an executeCallback exists. Not going to register it.");
        else:
            self.goal_callback = cb;


    ## @brief Explicitly start the action server, used it auto_start is set to false
    def start(self):
        self.action_server.start();


    ## @brief Callback for when the ActionServer receives a new goal and passes it on
    def internal_goal_callback(self, goal):
          self.execute_condition.acquire();

          try:
              rospy.logdebug("A new goal %shas been recieved by the single goal action server",goal.get_goal_id().id);


              print "got a goal"
              self.next_goal = goal;
              self.new_goal = True;
              self.goals_received_ += 1
				
              #add goal to queue
              self.goal_queue_.put(goal)

                  #Trigger runLoop to call execute()
              self.execute_condition.notify();
              self.execute_condition.release();

          except Exception, e:
              rospy.logerr("ComplexActionServer.internal_goal_callback - exception %s",str(e))
              self.execute_condition.release();

       
    ## @brief Callback for when the ActionServer receives a new preempt and passes it on
    def internal_preempt_callback(self,preempt):
    	return

    ## @brief Called from a separate thread to call blocking execute calls
    def executeLoop(self):
          loop_duration = rospy.Duration.from_sec(.1);

          while (not rospy.is_shutdown()):
              rospy.logdebug("SAS: execute");

              with self.terminate_mutex:
                  if (self.need_to_terminate):
                      break;

              if (self.is_new_goal_available()):
                  # accept_new_goal() is performing its own locking
                  goal_handle = self.accept_new_goal();
                  if not self.execute_callback:
                      rospy.logerr("execute_callback_ must exist. This is a bug in ComplexActionServer");
                      return

                  try:
                  	
                      print "run new executecb"
                      thread = threading.Thread(target=self.run_goal,args=(goal_handle.get_goal(),goal_handle));
                      thread.start()

                  except Exception, ex:
                      rospy.logerr("Exception in your execute callback: %s\n%s", str(ex),
                                   traceback.format_exc())
                      self.set_aborted(None, "Exception in execute callback: %s" % str(ex))

              with self.execute_condition:
                  self.execute_condition.wait(loop_duration.to_sec());
                  
                  
                  
                  
    def run_goal(self,goal, goal_handle):
       print 'new thread'
       self.execute_callback(goal,goal_handle);      


            
