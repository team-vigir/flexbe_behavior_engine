#!/usr/bin/env python
import rospy

from flexbe_core.core.operatable_state import OperatableState
from flexbe_core.core.preemptable_state import PreemptableState
from flexbe_core.core.priority_container import PriorityContainer

from flexbe_core.proxy import ProxyPublisher, ProxySubscriberCached
from flexbe_msgs.msg import CommandFeedback
from std_msgs.msg import Bool, Empty


class EventState(OperatableState):
    """
    A state that allows implementing certain events.
    """
    
    def __init__(self, *args, **kwargs):
        super(EventState, self).__init__(*args, **kwargs)
        
        self._entering = True
        self._skipped = False
        self.__execute = self.execute
        self.execute = self._event_execute

        self._paused = False
        self._last_active_container = None

        self._feedback_topic = 'flexbe/command_feedback'
        self._repeat_topic = 'flexbe/command/repeat'
        self._pause_topic = 'flexbe/command/pause'

        self._pub = ProxyPublisher()
        self._sub = ProxySubscriberCached()
        

    def _event_execute(self, *args, **kwargs):
        if self._is_controlled and self._sub.has_msg(self._pause_topic):
            msg = self._sub.get_last_msg(self._pause_topic)
            if msg.data:
                self._sub.remove_last_msg(self._pause_topic)
                rospy.loginfo("--> Pausing in state %s", self.name)
                self._pub.publish(self._feedback_topic, CommandFeedback(command="pause"))
                self._last_active_container = PriorityContainer.active_container
                PriorityContainer.active_container = ''
                self._paused = True

        if self._paused and not PreemptableState.preempt:
            self._notify_skipped()
            return self._loopback_name

        if self._entering:
            self._entering = False
            self.on_enter(*args, **kwargs)
        if self._skipped:
            self._skipped = False
            self.on_resume(*args, **kwargs)
        
        execute_outcome = self.__execute(*args, **kwargs)

        repeat = False
        if self._is_controlled and self._sub.has_msg(self._repeat_topic):
            rospy.loginfo("--> Repeating state %s", self.name)
            self._sub.remove_last_msg(self._repeat_topic)
            self._pub.publish(self._feedback_topic, CommandFeedback(command="repeat"))
            repeat = True
        
        if execute_outcome != self._loopback_name and not PreemptableState.switching \
        or repeat:
            self._entering = True
            self.on_exit(*args, **kwargs)
            
        return execute_outcome

    def _notify_skipped(self):
        if not self._skipped:
            self.on_pause()
            self._skipped = True
        if self._is_controlled and self._sub.has_msg(self._pause_topic):
            msg = self._sub.get_last_msg(self._pause_topic)
            if not msg.data:
                self._sub.remove_last_msg(self._pause_topic)
                rospy.loginfo("--> Resuming in state %s", self.name)
                self._pub.publish(self._feedback_topic, CommandFeedback(command="resume"))
                PriorityContainer.active_container = self._last_active_container
                self._last_active_container = None
                self._paused = False
        super(EventState, self)._notify_skipped()


    def _enable_ros_control(self):
        super(EventState, self)._enable_ros_control()
        self._pub.createPublisher(self._feedback_topic, CommandFeedback)
        self._sub.subscribe(self._repeat_topic, Empty)
        self._sub.subscribe(self._pause_topic, Bool)

    def _disable_ros_control(self):
        super(EventState, self)._disable_ros_control()
        self._sub.unsubscribe_topic(self._repeat_topic)
        self._sub.unsubscribe_topic(self._pause_topic)
        self._last_active_container = None
        if self._paused:
            PriorityContainer.active_container = None
    
    
    # Events
    # (just implement the ones you need)

    def on_start(self):
        """
        Will be executed once when the behavior starts.
        """
        pass

    def on_stop(self):
        """
        Will be executed once when the behavior stops or is preempted.
        """
        pass

    def on_pause(self):
        """
        Will be executed each time this state is paused.
        """
        pass

    def on_resume(self, userdata):
        """
        Will be executed each time this state is resumed.
        """
        pass
    
    def on_enter(self, userdata):
        """
        Will be executed each time the state is entered from any other state (but not from itself).
        """
        pass
    
    def on_exit(self, userdata):
        """
        Will be executed each time the state will be left to any other state (but not to itself).
        """
        pass