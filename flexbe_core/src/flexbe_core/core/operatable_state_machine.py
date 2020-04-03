#!/usr/bin/env python
import zlib
from flexbe_core.core.user_data import UserData
from flexbe_core.logger import Logger
from flexbe_core.state_logger import StateLogger
from flexbe_core.core.operatable_state import OperatableState

from flexbe_msgs.msg import Container, ContainerStructure, BehaviorSync, CommandFeedback
from std_msgs.msg import Empty, UInt8, Int32

from flexbe_core.core.preemptable_state_machine import PreemptableStateMachine


class OperatableStateMachine(PreemptableStateMachine):
    """
    A state machine that can be operated.
    It synchronizes its current state with the mirror and supports some control mechanisms.
    """

    autonomy_level = 3

    def __init__(self, *args, **kwargs):
        super(OperatableStateMachine, self).__init__(*args, **kwargs)
        self.id = None
        self._autonomy = {}
        self._inner_sync_request = False

    # construction

    @staticmethod
    def add(label, state, transitions, autonomy=None, remapping=None):
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
        self = OperatableStateMachine.get_opened_container()
        PreemptableStateMachine.add(label, state, transitions, remapping)
        self._autonomy[label] = autonomy

    def _build_structure_msg(self):
        """
        Creates a message to describe the structure of this state machine.
        """
        structure_msg = ContainerStructure()
        container_msg = self._add_to_structure_msg(structure_msg)
        container_msg.outcomes = self.outcomes
        structure_msg.behavior_id = self.id
        return structure_msg

    def _add_to_structure_msg(self, structure_msg):
        """
        Adds this state machine and all children to the structure message.

        @type structure_msg: ContainerStructure
        @param structure_msg: The message that will finally contain the structure message.
        """
        # add self to message
        container_msg = Container()
        container_msg.path = self.path
        container_msg.children = [state.name for state in self._states]
        structure_msg.containers.append(container_msg)
        # add children to message
        for state in self._states:
            # create and add children
            if isinstance(state, OperatableStateMachine):
                state_msg = state._add_to_structure_msg(structure_msg)
            else:
                state_msg = Container(path=state.path)
                structure_msg.containers.append(state_msg)
            # complete structure info for children
            state_msg.outcomes = state.outcomes
            state_msg.transitions = [self._transitions[state.name][outcome] for outcome in state.outcomes]
            state_msg.autonomy = [self._autonomy[state.name][outcome] for outcome in state.outcomes]
        return container_msg

    # execution

    def _execute_current_state(self):
        outcome = super(OperatableStateMachine, self)._execute_current_state()
        # provide explicit sync as back-up functionality
        # should be used only if there is no other choice
        # since it requires additional 8 byte + header update bandwith and time to restart mirror
        if self._inner_sync_request and self.get_deep_state() is not None:
            self._inner_sync_request = False
            if self.id is None:
                self.parent._inner_sync_request = True
            else:
                msg = BehaviorSync()
                msg.behavior_id = self.id
                msg.current_state_checksum = zlib.adler32(self.get_deep_state().path)
                self._pub.publish('flexbe/mirror/sync', msg)
        return outcome

    def is_transition_allowed(self, label, outcome):
        return self._autonomy[label].get(outcome, -1) < OperatableStateMachine.autonomy_level

    def get_required_autonomy(self, outcome):
        return self._autonomy[self.current_state_label][outcome]

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
        self.set_name(name)
        self.id = id

        # Update mirror with currently active state (high bandwidth mode)
        self._pub.createPublisher('flexbe/mirror/sync', BehaviorSync)
        # Sends the current structure to the mirror
        self._pub.createPublisher('flexbe/mirror/structure', ContainerStructure)
        # Gives feedback about executed commands to the GUI
        self._pub.createPublisher('flexbe/command_feedback', CommandFeedback)

        self._sub.subscribe('flexbe/command/autonomy', UInt8, self._set_autonomy_level)
        self._sub.subscribe('flexbe/command/sync', Empty, self._sync_callback)
        self._sub.subscribe('flexbe/command/attach', UInt8, self._attach_callback)
        self._sub.subscribe('flexbe/request_mirror_structure', Int32, self._mirror_structure_callback)

        StateLogger.initialize(name)
        StateLogger.log('flexbe.initialize', None, behavior=name, autonomy=OperatableStateMachine.autonomy_level)
        if OperatableStateMachine.autonomy_level != 255:
            self._enable_ros_control()

        self.wait(seconds=0.2)  # no clean way to wait for publisher to be ready...
        self._notify_start()

    # operator callbacks

    def _set_autonomy_level(self, msg):
        """ Sets the current autonomy level. """
        if OperatableStateMachine.autonomy_level != msg.data:
            Logger.localinfo('--> Autonomy changed to %d' % msg.data)
        if msg.data < 0:
            self.preempt()
        else:
            OperatableStateMachine.autonomy_level = msg.data
        self._pub.publish('flexbe/command_feedback', CommandFeedback(command="autonomy", args=[]))

    def _sync_callback(self, msg):
        Logger.localinfo("--> Synchronization requested...")
        msg = BehaviorSync()
        msg.behavior_id = self.id
        # make sure we are already executing
        self.wait(condition=lambda: self.get_deep_state() is not None)
        msg.current_state_checksum = zlib.adler32(self.get_deep_state().path)
        self._pub.publish('flexbe/mirror/sync', msg)
        self._pub.publish('flexbe/command_feedback', CommandFeedback(command="sync", args=[]))
        Logger.localinfo("<-- Sent synchronization message for mirror.")

    def _attach_callback(self, msg):
        Logger.localinfo("--> Enabling control...")
        # set autonomy level
        OperatableStateMachine.autonomy_level = msg.data
        # enable control of states
        self._enable_ros_control()
        self._inner_sync_request = True
        # send command feedback
        cfb = CommandFeedback(command="attach")
        cfb.args.append(self.name)
        self._pub.publish('flexbe/command_feedback', cfb)
        Logger.localinfo("<-- Sent attach confirm.")

    def _mirror_structure_callback(self, msg):
        Logger.localinfo("--> Creating behavior structure for mirror...")
        self._pub.publish('flexbe/mirror/structure', self._build_structure_msg())
        Logger.localinfo("<-- Sent behavior structure for mirror.")

    # handle state events

    def _notify_start(self):
        for state in self._states:
            if isinstance(state, OperatableState):
                state.on_start()
            if isinstance(state, OperatableStateMachine):
                state._notify_start()

    def _notify_stop(self):
        for state in self._states:
            if isinstance(state, OperatableState):
                state.on_stop()
            if isinstance(state, OperatableStateMachine):
                state._notify_stop()
            if state._is_controlled:
                state._disable_ros_control()

    def on_exit(self, userdata):
        if self._current_state is not None:
            ud = UserData(reference=self.userdata, input_keys=self._current_state.input_keys,
                          output_keys=self._current_state.output_keys,
                          remap=self._remappings[self._current_state.name])
            self._current_state._entering = True
            self._current_state.on_exit(ud)
            self._current_state = None
