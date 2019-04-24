#!/usr/bin/env python
import rospy
import smach
import time

from flexbe_msgs.msg import OutcomeRequest
from ..proxy import ProxySubscriberCached

from .silent_state_machine import SilentStateMachine


class JumpableStateMachine(SilentStateMachine):
    """
    A state machine that runs in background and does not report any transition.
    It can jump to any of its states, executing all states on a random way to the target
    (so make sure its states have no side effects).
    """
    refresh = False
    
    def __init__(self, *args, **kwargs):
        super(JumpableStateMachine, self).__init__(*args, **kwargs)
        
        self._sub = ProxySubscriberCached()
    
        
    def jump_to(self, target_state_label):
        """
        Jumps to the specified state.
        
        @type target_state_label: string
        @param target_state_label: The name of the state to jump to.
            Should be unique. If not, the first occurrence will be chosen.
        """
        # set current state and current container
        container = self
        while isinstance(container._current_state, smach.StateMachine):
            container = container._current_state
        current_state = container._current_state
        
        # determine path
        path = self._search_state(current_state, target_state_label, container, [], '')
        if path is None:
            rospy.logerr('No path found from '+current_state.name+' to '+target_state_label)
            return
        
        rospy.loginfo('Path: '+path)
        if path == '': 
            JumpableStateMachine.refresh = True
            return
        
        # traverse path
        outcomes = path.split('/')
        for i in range(len(outcomes)-1):
            current_state = self._get_deep_state()
            current_state._requested_outcome = outcomes[i]
            if not self._wait_for_next_state(current_state, 1):
                rospy.logerr('Unable to jump to '+target_state_label+': Stuck in state '+current_state.name)
                return
        
        
    def _search_state(self, origin_state, target_state_label, container, seen_states, path):
        """
        Searches the given state to create an outcome path to it.
        This is implemented by a hierarchical depth-first search.
        
        @type origin_state: State
        @param origin_state: The current origin for this search.
        
        @type target_state_label: string
        @param target_state_label: The name of the state to search.
        
        @type container: StateMachine
        @param container: The current container inside which the search is currently done.
        
        @type seen_states: list of State
        @param seen_states: A list of all states that already have been traversed.
        
        @type path: string
        @param path: The outcome path to the current origin_state
        
        @return: The whole outcome path to the searched state.
        """
        # saw this state
        seen_states.append(origin_state)

        # look if we found our target
        if origin_state.name == target_state_label:
            return path
        target_path = None
        
        # enter the state machine (next lower hierarchy level)
        if isinstance(origin_state, smach.StateMachine):
            # do not need to enter, inital state is implicitly active
            if origin_state._initial_state_label == target_state_label:
                return path
            # search inside this state machine
            child_state = origin_state._states[origin_state._initial_state_label]
            if seen_states.count(child_state) == 0:
                target_path = self._search_state(child_state, target_state_label, origin_state, seen_states, path)
            # found target inside, search finished
            if target_path is not None:
                return target_path
        
        # proceed with the neighbors
        else:   
            for outcome in origin_state._outcomes:
                next_state_label = origin_state.transitions[outcome]
                if next_state_label == 'preempted' or next_state_label == 'preempted_mirror': continue  # skip preempt outcomes
                next_container = container
                reached_exit = False
                
                # determine the next state
                # (consider that it might be necessary to go to the next higher hierarchy level)
                while not next_container._states.has_key(next_state_label): 
                    if next_container._parent is None:
                        reached_exit = True
                        break
                    next_state_label = next_container.transitions[next_state_label]
                    next_container = next_container._parent
                # reached one outcome of the whole state machine (no higher hierarchy level)
                if reached_exit: break
                
                # search the neighbor if not already seen
                next_state = next_container._states[next_state_label]
                if seen_states.count(next_state) == 0:
                    target_path = self._search_state(next_state, target_state_label, next_container, seen_states, path+outcome+'/')
                # found target in neighbors
                if target_path is not None:
                    return target_path
                
        # unfortunately, no success
        return None
            
            
    def _wait_for_next_state(self, current_state, timeout, period=0.05):
        """
        Sleeps until the state machine proceeded with the next state.
        
        @type current_state: State
        @param current_state: The current state when this method is called.
        
        @type timeout: float
        @param timeout: The time in seconds when this method should stop waiting when no transition is done.
        
        @type period: float
        @param period: The time in seconds how often this method should check the current state.
        
        @return: True if waiting was successful, False if current_state is still the same (timeout).
        """
        end_time = time.time() + timeout
        while time.time() < end_time:
            new_state = self._get_deep_state()
            if new_state != current_state: 
                return True
            time.sleep(period)
        return False


    def _transition_callback(self, msg):
        """
        Forces the state machine to do a transition with a specified outcome.
        
        @type msg: OutcomeRequest
        @param msg: A message containing all relevant information.
        """
        state = self._get_deep_state()
        
        # wait if state machine is currently not active
        if state is None:
            if not self._wait_for_next_state(None, 5):
                rospy.logwarn('Updating mirror while it is not running!')
                return # not running
            state = self._get_deep_state()
            rospy.logwarn('Had to wait for mirror starting to request transition of state %s!', msg.target)
            
        # wait for the target state to not trigger a wrong transition
        if msg.target != state.name:
            self._wait_for_next_state(state, 2)
            state = self._get_deep_state()
            if msg.target != state.name: # waited for next state, so test current state again (but only once to not block anything)
                rospy.logwarn('Wrong state name: requested %s but is %s!', msg.target, state.name)
                return
            rospy.logwarn('Had to wait for state %s to request transition!', msg.target)
            
        # trigger desired outcome or preempt
        if msg.outcome == 255:
            state._requested_outcome = 'preempted'
        else:
            state._requested_outcome = state._outcome_list[msg.outcome]
        
    def destroy(self):
        pass