#!/usr/bin/env python
import rospy
import zlib
import smach

from rospy.exceptions import ROSInterruptException

from flexbe_core.core.preemptable_state_machine import PreemptableStateMachine
from flexbe_core.core.lockable_state_machine import LockableStateMachine

from smach.state_machine import StateMachine

from flexbe_core.proxy import ProxyPublisher, ProxySubscriberCached
from flexbe_msgs.msg import Container, ContainerStructure, OutcomeRequest, BehaviorSync, CommandFeedback, BehaviorLog
from std_msgs.msg import Empty, UInt8, Int32

from flexbe_core.core.loopback_state import LoopbackState
from flexbe_core.state_logger import StateLogger


class OperatableStateMachine(PreemptableStateMachine):
    """
    A state machine that can be operated.
    It synchronizes its current state with the mirror and supports some control mechanisms.
    """
    
    autonomy_level = 3
    silent_mode = False
    
    def __init__(self, *args, **kwargs):
        super(OperatableStateMachine, self).__init__(*args, **kwargs)
        self._message = None
        self._rate = rospy.Rate(10)

        self.id = None
        self.autonomy = None

        self._autonomy = {}
        self._ordered_states = []
        self._inner_sync_request = False
        
        self._pub = ProxyPublisher()

        self._sub = ProxySubscriberCached()


    def execute(self, parent_ud = smach.UserData()):
        # First time in this state machine
        if self._current_state is None:
            # Spew some info
            smach.loginfo("State machine starting in initial state '%s' with userdata: \n\t%s" %
                    (self._current_label, list(self.userdata.keys())))
            # Set initial state
            self._set_current_state(self._initial_state_label)

            # Initialize preempt state
            self._preempted_label = None
            self._preempted_state = None

            # Call start callbacks
            self.call_start_cbs()

            # Copy input keys
            self._copy_input_keys(parent_ud, self.userdata)

        container_outcome = self._loopback_name
        if self.id is not None:
            # only top-level statemachine should loop for outcome
            while container_outcome == self._loopback_name and not smach.is_shutdown():
                # Update the state machine
                container_outcome = self._async_execute(parent_ud)
        else:
            container_outcome = self._async_execute(parent_ud)

        if smach.is_shutdown():
            container_outcome = 'preempted'

        return container_outcome


    def _async_execute(self, parent_ud = smach.UserData()):
        """Run the state machine on entry to this state.
        This will set the "closed" flag and spin up the execute thread. Once
        this flag has been set, it will prevent more states from being added to
        the state machine. 
        """

        # This will prevent preempts from getting propagated to non-existent children
        with self._state_transitioning_lock:
            # Check state consistency
            try:
                self.check_consistency()
            except (smach.InvalidStateError, smach.InvalidTransitionError):
                smach.logerr("Container consistency check failed.")
                return None

            # Set running flag
            self._is_running = True

            # Initialize container outcome
            container_outcome = None

            # Step through state machine
            if self._is_running and not smach.is_shutdown():
                # Update the state machine
                container_outcome = self._update_once()
                if self._current_state is not None:
                    try:
                        self._current_state._rate.sleep()
                    except ROSInterruptException:
                        rospy.logwarn('Interrupted rate sleep.')

            if container_outcome is not None and container_outcome != self._loopback_name:
                # Copy output keys
                self._copy_output_keys(self.userdata, parent_ud)
            else:
                container_outcome = self._loopback_name

            # provide explicit sync as back-up functionality
            # should be used only if there is no other choice
            # since it requires additional 8 byte + header update bandwith and time to restart mirror
            if self._inner_sync_request and self._get_deep_state() is not None:
                self._inner_sync_request = False
                if self.id is None:
                    self._parent._inner_sync_request = True
                else:
                    msg = BehaviorSync()
                    msg.behavior_id = self.id
                    msg.current_state_checksum = zlib.adler32(self._get_deep_state()._get_path())
                    self._pub.publish('flexbe/mirror/sync', msg)

            # We're no longer running
            self._is_running = False

        return container_outcome


    @staticmethod
    def add(label, state, transitions = None, autonomy = None, remapping = None):
        """
        Add a state to the opened state machine.
        
        @type label: string
        @param label: The label of the state being added.
        
        @param state: An instance of a class implementing the L{State} interface.
        
        @param transitions: A dictionary mapping state outcomes to other state
        labels or container outcomes.
        
        @param autonomy: A dictionary mapping state outcomes to their required
        autonomy level

        @param remapping: A dictionary mapping local userdata keys to userdata
        keys in the container.
        """
        self = StateMachine._currently_opened_container()
        
        # add loopback transition to loopback states
        if isinstance(state, LoopbackState):
            transitions[LoopbackState._loopback_name] = label
            autonomy[LoopbackState._loopback_name] = -1
        if isinstance(state, OperatableStateMachine):
            transitions[OperatableStateMachine._loopback_name] = label
            autonomy[OperatableStateMachine._loopback_name] = -1
            
        self._ordered_states.append(state)
        state.name = label
        state.transitions = transitions
        state.autonomy = autonomy
        state._parent = self
            
        StateMachine.add(label, state, transitions, remapping)
        self._autonomy[label] = autonomy

    def replace(self, new_state):
        old_state = self._states[new_state.name]
        new_state.transitions = old_state.transitions
        new_state.autonomy = old_state.autonomy
        new_state._parent = old_state._parent

        self._ordered_states[self._ordered_states.index(old_state)] = new_state
        self._states[new_state.name] = new_state
            
            
    def destroy(self):
        self._notify_stop()
        self._disable_ros_control()
        self._sub.unsubscribe_topic('flexbe/command/autonomy')
        self._sub.unsubscribe_topic('flexbe/command/sync')
        self._sub.unsubscribe_topic('flexbe/command/attach')
        self._sub.unsubscribe_topic('flexbe/request_mirror_structure')
        StateLogger.shutdown()
        
        
    def confirm(self, name, id):
        """
        Confirms the state machine and triggers the creation of the structural message.
        It is mandatory to call this function at the top-level state machine
        between building it and starting its execution.
        
        @type name: string
        @param name: The name of this state machine to identify it.
        """
        self.name = name
        self.id = id

        self._pub.createPublisher('flexbe/mirror/sync', BehaviorSync, _latch = True)   # Update mirror with currently active state (high bandwidth mode)
        self._pub.createPublisher('flexbe/mirror/preempt', Empty, _latch = True)       # Preempts the mirror
        self._pub.createPublisher('flexbe/mirror/structure', ContainerStructure)       # Sends the current structure to the mirror
        self._pub.createPublisher('flexbe/log', BehaviorLog)                           # Topic for logs to the GUI
        self._pub.createPublisher('flexbe/command_feedback', CommandFeedback)          # Gives feedback about executed commands to the GUI

        self._sub.subscribe('flexbe/command/autonomy', UInt8, self._set_autonomy_level)
        self._sub.subscribe('flexbe/command/sync', Empty, self._sync_callback)
        self._sub.subscribe('flexbe/command/attach', UInt8, self._attach_callback)
        self._sub.subscribe('flexbe/request_mirror_structure', Int32, self._mirror_structure_callback)

        StateLogger.initialize(name)
        if OperatableStateMachine.autonomy_level != 255:
            self._enable_ros_control()

        rospy.sleep(0.5) # no clean way to wait for publisher to be ready...
        self._notify_start()

            
    def _set_autonomy_level(self, msg):
        """ Sets the current autonomy level. """
        if OperatableStateMachine.autonomy_level != msg.data:
            rospy.loginfo('--> Autonomy changed to %d', msg.data)
            
        if msg.data < 0:
            self.preempt()
        else:
            OperatableStateMachine.autonomy_level = msg.data

        self._pub.publish('flexbe/command_feedback', CommandFeedback(command="autonomy", args=[]))


    def _sync_callback(self, msg):
        rospy.loginfo("--> Synchronization requested...")
        msg = BehaviorSync()
        msg.behavior_id = self.id
        while self._get_deep_state() is None:
            rospy.sleep(0.1)
        msg.current_state_checksum = zlib.adler32(self._get_deep_state()._get_path())
        self._pub.publish('flexbe/mirror/sync', msg)
        self._pub.publish('flexbe/command_feedback', CommandFeedback(command="sync", args=[]))
        rospy.loginfo("<-- Sent synchronization message for mirror.")


    def _attach_callback(self, msg):
        rospy.loginfo("--> Enabling control...")
        # set autonomy level
        OperatableStateMachine.autonomy_level = msg.data
        # enable control of states
        self._enable_ros_control()
        self._inner_sync_request = True
        # send command feedback
        cfb = CommandFeedback(command="attach")
        cfb.args.append(self.name)
        self._pub.publish('flexbe/command_feedback', cfb)
        rospy.loginfo("<-- Sent attach confirm.")


    def _mirror_structure_callback(self, msg):
        rospy.loginfo("--> Creating behavior structure for mirror...")
        msg = self._build_msg('')
        msg.behavior_id = self.id
        self._pub.publish('flexbe/mirror/structure', msg)
        rospy.loginfo("<-- Sent behavior structure for mirror.")


    def _transition_allowed(self, label, outcome):
        return self._autonomy[label][outcome] < OperatableStateMachine.autonomy_level
            
            
    def _build_msg(self, prefix, msg = None):
        """
        Adds this state machine to the initial structure message.
        
        @type prefix: string
        @param prefix: A path consisting of the container hierarchy containing this state.
        
        @type msg: ContainerStructure
        @param msg: The message that will finally contain the structure message.
        """
        # set children
        children = []
        for state in self._ordered_states:
            children.append(str(state.name))
            
        # set name
        name = prefix + (self.name if self.id is None else '')
        
        if msg is None:
            # top-level state machine (has no transitions)
            self._message = ContainerStructure()
            outcomes = list(self._outcomes)
            transitions = None
            autonomy = None
        else:
            # lower-level state machine
            self._message = msg
            outcomes = list(self.transitions)
            # set transitions and autonomy
            transitions = []
            autonomy = []
            for i in range(len(self.transitions)):
                outcome = outcomes[i]
                if outcome == 'preempted':      # set preempt transition
                    transitions.append('preempted')
                    autonomy.append(-1)
                else:
                    transitions.append(str(self.transitions[outcome]))
                    autonomy.append(self.autonomy[outcome])
        
        # add to message
        self._message.containers.append(Container(name, children, outcomes, transitions, autonomy))
            
        # build message for children
        for state in self._ordered_states:
            state._build_msg(name+'/', self._message)
        
        # top-level state machine returns the message
        if msg is None:
            return self._message


    def _notify_start(self):
        for state in self._ordered_states:
            if isinstance(state, LoopbackState):
                state.on_start()
            if isinstance(state, OperatableStateMachine):
                state._notify_start()

    def _enable_ros_control(self):
        self._is_controlled = True
        for state in self._ordered_states:
            if isinstance(state, LoopbackState):
                state._enable_ros_control()
            if isinstance(state, OperatableStateMachine):
                state._enable_ros_control()

    def _notify_stop(self):
        for state in self._ordered_states:
            if isinstance(state, LoopbackState):
                state.on_stop()
                state._disable_ros_control()
            if isinstance(state, OperatableStateMachine):
                state._notify_stop()

    def _disable_ros_control(self):
        self._is_controlled = False
        for state in self._ordered_states:
            if isinstance(state, LoopbackState):
                state._disable_ros_control()
            if isinstance(state, OperatableStateMachine):
                state._disable_ros_control()

    def on_exit(self, userdata):
        if self._current_state is not None:
            ud = smach.Remapper(
                self.userdata,
                self._current_state.get_registered_input_keys(),
                self._current_state.get_registered_output_keys(),
                self._remappings[self._current_state.name]
            )
            self._current_state._entering = True
            self._current_state.on_exit(ud)
            self._current_state = None