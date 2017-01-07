#!/usr/bin/env python
import rospy

from flexbe_core.core.manually_transitionable_state import ManuallyTransitionableState

from flexbe_core.proxy import ProxyPublisher, ProxySubscriberCached
from flexbe_msgs.msg import CommandFeedback
from std_msgs.msg import String


class LockableState(ManuallyTransitionableState):
    """
    A state that can be locked.
    When locked, no transition can be done regardless of the resulting outcome.
    However, if any outcome would be triggered, the outcome will be stored
    and the state won't be executed anymore until it is unlocked and the stored outcome is set.
    """
    
    def __init__(self, *args, **kwargs):
        super(LockableState, self).__init__(*args, **kwargs)

        self._locked = False
        self._stored_outcome = None
        
        self.__execute = self.execute
        self.execute = self._lockable_execute

        self._feedback_topic = 'flexbe/command_feedback'
        self._lock_topic = 'flexbe/command/lock'
        self._unlock_topic = 'flexbe/command/unlock'

        self._pub = ProxyPublisher()
        self._sub = ProxySubscriberCached()


    def _lockable_execute(self, *args, **kwargs):
        if self._is_controlled and self._sub.has_msg(self._lock_topic):
            msg = self._sub.get_last_msg(self._lock_topic)
            self._sub.remove_last_msg(self._lock_topic)
            self._execute_lock(msg.data)

        if self._is_controlled and self._sub.has_msg(self._unlock_topic):
            msg = self._sub.get_last_msg(self._unlock_topic)
            self._sub.remove_last_msg(self._unlock_topic)
            self._execute_unlock(msg.data)

        if self._locked:
            if self._stored_outcome is None or self._stored_outcome == 'None':
                self._stored_outcome = self.__execute(*args, **kwargs)
            return None
            
        if not self._locked and not self._stored_outcome is None and not self._stored_outcome == 'None':
            if self._parent.transition_allowed(self.name, self._stored_outcome):
                outcome = self._stored_outcome
                self._stored_outcome = None
                return outcome
            else:
                return None

        outcome = self.__execute(*args, **kwargs)

        if outcome is None or outcome == 'None':
            return None

        if not self._parent is None and not self._parent.transition_allowed(self.name, outcome):
            self._stored_outcome = outcome
            return None

        return outcome


    def _execute_lock(self, target):
        if target == self._get_path() or target == '':
            target = self._get_path()
            found_target = True
            self._locked = True
        else:
            found_target = self._parent.lock(target)

        self._pub.publish(self._feedback_topic, CommandFeedback(command="lock", args=[target, target if found_target else ""]))
        if not found_target:
            rospy.logwarn("--> Wanted to lock %s, but could not find it in current path %s.", target, self._get_path())
        else:
            rospy.loginfo("--> Locking in state %s", target)


    def _execute_unlock(self, target):
        if target == self._get_path() or (self._locked and target == ''):
            target = self._get_path()
            found_target = True
            self._locked = False
        else:
            found_target = self._parent.unlock(target)

        self._pub.publish(self._feedback_topic, CommandFeedback(command="unlock", args=[target, target if found_target else ""]))
        if not found_target:
            rospy.logwarn("--> Wanted to unlock %s, but could not find it in current path %s.", target, self._get_path())
        else:
            rospy.loginfo("--> Unlocking in state %s", target)


    def _enable_ros_control(self):
        super(LockableState, self)._enable_ros_control()
        self._pub.createPublisher(self._feedback_topic, CommandFeedback)
        self._sub.subscribe(self._lock_topic, String)
        self._sub.subscribe(self._unlock_topic, String)

    def _disable_ros_control(self):
        super(LockableState, self)._disable_ros_control()
        self._sub.unsubscribe_topic(self._lock_topic)
        self._sub.unsubscribe_topic(self._unlock_topic)


    def is_locked(self):
        return self._locked