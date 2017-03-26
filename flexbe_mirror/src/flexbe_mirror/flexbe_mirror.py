#!/usr/bin/env python

import roslib; roslib.load_manifest('flexbe_mirror')
import rospy
import smach
import threading
import zlib

from flexbe_core import JumpableStateMachine, LockableStateMachine
from flexbe_core.core import PreemptableState
from mirror_state import MirrorState

from flexbe_core.proxy import ProxyPublisher, ProxySubscriberCached
from flexbe_msgs.msg import ContainerStructure, BehaviorSync, BEStatus
from std_msgs.msg import Empty, String, Int32, UInt8

'''
Created on 08.05.2013

@author: Philipp Schillinger
'''
class VigirBehaviorMirror(object):
    '''
    classdocs
    '''


    def __init__(self):
        '''
        Constructor
        '''
        self._sm = None

        smach.set_loggers (
            rospy.logdebug, # hide SMACH transition log spamming
            rospy.logwarn,
            rospy.logdebug,
            rospy.logerr
        )
        
        # set up proxys for sm <--> GUI communication
        # publish topics
        self._pub = ProxyPublisher({'flexbe/behavior_update': String,
                                    'flexbe/request_mirror_structure': Int32})
            
        self._running = False
        self._stopping = False
        self._active_id = 0
        self._starting_path = None
        self._current_struct = None

        self._outcome_topic = 'flexbe/mirror/outcome'

        self._struct_buffer = list()
        
        # listen for mirror message
        self._sub = ProxySubscriberCached()
        self._sub.subscribe(self._outcome_topic, UInt8)
        self._sub.enable_buffer(self._outcome_topic)

        self._sub.subscribe('flexbe/mirror/structure', ContainerStructure, self._mirror_callback)
        self._sub.subscribe('flexbe/status', BEStatus, self._status_callback)
        self._sub.subscribe('flexbe/mirror/sync', BehaviorSync, self._sync_callback)
        self._sub.subscribe('flexbe/mirror/preempt', Empty, self._preempt_callback)
    
    
    def _mirror_callback(self, msg):
        rate = rospy.Rate(10)
        while self._stopping:
            rate.sleep()
            
        if self._running:
            rospy.logwarn('Received a new mirror structure while mirror is already running, adding to buffer (checksum: %s).' % str(msg.behavior_id))
        elif self._active_id != 0 and msg.behavior_id != self._active_id:
            rospy.logwarn('checksum of received mirror structure (%s) does not match expected checksum (%s), will ignore.' % (str(msg.behavior_id), str(self._active_id)))
            return
        else:
            rospy.loginfo('Received a new mirror structure for checksum %s' % str(msg.behavior_id))

        self._struct_buffer.append(msg)

        if self._active_id == msg.behavior_id:
            self._struct_buffer = list()
            self._mirror_state_machine(msg)
            rospy.loginfo('Mirror built.')

            self._execute_mirror()


    def _status_callback(self, msg):
        if msg.code == BEStatus.STARTED:
            thread = threading.Thread(target=self._start_mirror, args=[msg])
            thread.daemon = True
            thread.start()
        else:
            thread = threading.Thread(target=self._stop_mirror, args=[msg])
            thread.daemon = True
            thread.start()


    def _start_mirror(self, msg):
        rate = rospy.Rate(10)
        while self._stopping:
            rate.sleep()

        if self._running:
            rospy.logwarn('Tried to start mirror while it is already running, will ignore.')
            return

        if len(msg.args) > 0:
            self._starting_path = "/" + msg.args[0][1:].replace("/", "_mirror/") + "_mirror"

        self._active_id = msg.behavior_id

        while self._sm is None and len(self._struct_buffer) > 0:
            struct = self._struct_buffer[0]
            self._struct_buffer = self._struct_buffer[1:]
            if struct.behavior_id == self._active_id:
                self._mirror_state_machine(struct)
                rospy.loginfo('Mirror built.')
            else:
                rospy.logwarn('Discarded mismatching buffered structure for checksum %s' % str(struct.behavior_id))

        if self._sm is None:
            rospy.logwarn('Missing correct mirror structure, requesting...')
            rospy.sleep(0.2) # no clean way to wait for publisher to be ready...
            self._pub.publish('flexbe/request_mirror_structure', Int32(msg.behavior_id))
            self._active_id = msg.behavior_id
            return

        self._execute_mirror()


    def _stop_mirror(self, msg):
        self._stopping = True
        if self._sm is not None and self._running:
            if msg is not None and msg.code == BEStatus.FINISHED:
                rospy.loginfo('Onboard behavior finished successfully.')
                self._pub.publish('flexbe/behavior_update', String())
            elif msg is not None and msg.code == BEStatus.SWITCHING:
                self._starting_path = None
                rospy.loginfo('Onboard performing behavior switch.')
            elif msg is not None and msg.code == BEStatus.READY:
                rospy.loginfo('Onboard engine just started, stopping currently running mirror.')
                self._pub.publish('flexbe/behavior_update', String())
            elif msg is not None:
                rospy.logwarn('Onboard behavior failed!')
                self._pub.publish('flexbe/behavior_update', String())

            PreemptableState.preempt = True
            rate = rospy.Rate(10)
            while self._running:
                rate.sleep()
        else:
            rospy.loginfo('No onboard behavior is active.')

        self._active_id = 0
        self._sm = None
        self._current_struct = None
        self._sub.remove_last_msg(self._outcome_topic, clear_buffer=True)

        if msg is not None and msg.code != BEStatus.SWITCHING:
            rospy.loginfo('\033[92m--- Behavior Mirror ready! ---\033[0m')

        self._stopping = False


    def _sync_callback(self, msg):
        if msg.behavior_id == self._active_id:
            thread = threading.Thread(target=self._restart_mirror, args=[msg])
            thread.daemon = True
            thread.start()
        else:
            rospy.logerr('Cannot synchronize! Different behavior is running onboard, please stop execution!')
            thread = threading.Thread(target=self._stop_mirror, args=[None])
            thread.daemon = True
            thread.start()


    def _restart_mirror(self, msg):
        rospy.loginfo('Restarting mirror for synchronization...')
        self._sub.remove_last_msg(self._outcome_topic, clear_buffer=True)
        if self._sm is not None and self._running:
            PreemptableState.preempt = True
            rate = rospy.Rate(10)
            while self._running:
                rate.sleep()
            self._sm = None
        if msg.current_state_checksum in self._state_checksums:
            current_state_path = self._state_checksums[msg.current_state_checksum]
            self._starting_path = "/" + current_state_path[1:].replace("/", "_mirror/") + "_mirror"
            rospy.loginfo('Current state: %s' % current_state_path)
        try:
            self._mirror_state_machine(self._current_struct)
            rospy.loginfo('Mirror built.')
            self._execute_mirror()
        except (AttributeError, smach.InvalidStateError):
            rospy.loginfo('Stopping synchronization because behavior has stopped.')
        

    def _execute_mirror(self):
        self._running = True

        rospy.loginfo("Executing mirror...")
        if self._starting_path is not None:
            LockableStateMachine.path_for_switch = self._starting_path
            rospy.loginfo("Starting mirror in state " + self._starting_path)
            self._starting_path = None

        result = 'preempted'
        try:
            result = self._sm.execute()
        except Exception as e:
            rospy.loginfo('Caught exception on preempt:\n%s' % str(e))
            result = 'preempted'

        if self._sm is not None:
            self._sm.destroy()
        self._running = False

        rospy.loginfo('Mirror finished with result ' + result)
    
    
    def _mirror_state_machine(self, msg):
        self._current_struct = msg
        self._state_checksums = dict()
        root = None
        for con_msg in msg.containers:
            if con_msg.path.find('/') == -1:
                root = con_msg.path
                break
        self._add_node(msg, root)
        # calculate checksums of all states
        for con_msg in msg.containers:
            if con_msg.path.find('/') != -1:
                self._state_checksums[zlib.adler32(con_msg.path)] = con_msg.path
        
    
    def _add_node(self, msg, path):
        #rospy.loginfo('Add node: '+path)
        container = None
        for con_msg in msg.containers:
            if con_msg.path == path:
                container = con_msg
                break
            
        transitions = None
        if container.transitions is not None:
            transitions = {}
            for i in range(len(container.transitions)):
                transitions[container.outcomes[i]] = container.transitions[i] + '_mirror'
            
        path_frags = path.split('/')
        container_name = path_frags[len(path_frags)-1]
        if len(container.children) > 0:
            sm_outcomes = []
            for outcome in container.outcomes: sm_outcomes.append(outcome + '_mirror')
            sm = JumpableStateMachine(outcomes=sm_outcomes)
            with sm:
                for child in container.children: 
                    self._add_node(msg, path+'/'+child)
            if len(transitions) > 0: 
                container_transitions = {}
                for i in range(len(container.transitions)): container_transitions[sm_outcomes[i]] = transitions[container.outcomes[i]]
                JumpableStateMachine.add(container_name + '_mirror', sm, transitions=container_transitions)
            else:
                self._sm = sm
        else:
            JumpableStateMachine.add(container_name + '_mirror', MirrorState(container_name, path, container.outcomes, container.autonomy), transitions=transitions)
    
    
    def _jump_callback(self, msg):
        start_time = rospy.Time.now()
        current_elapsed = 0
        jumpable = True
        while self._sm is None or not self._sm.is_running():
            current_elapsed = rospy.Time.now() - start_time
            if current_elapsed > rospy.Duration.from_sec(10):
                jumpable = False
                break
            rospy.sleep(0.05)
        if jumpable:
            self._sm.jump_to(msg.stateName)
        else:
            rospy.logwarn('Could not jump in mirror: Mirror does not exist or is not running!')
            

    def _preempt_callback(self, msg):
        if self._sm is not None and self._sm.is_running():
            rospy.logwarn('Explicit preempting is currently ignored, mirror should be preempted by onboard behavior.')
        else:
            rospy.logwarn('Could not preempt mirror because it seems not to be running!')




