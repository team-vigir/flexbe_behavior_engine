#!/usr/bin/env python
import rospy

from flexbe_core.proxy import ProxyPublisher
from flexbe_msgs.msg import BehaviorLog


'''
Created on 12/17/2014

@author: Philipp Schillinger
'''
class Logger(object):
    '''
    Realizes behavior-specific logging.
    '''
    REPORT_INFO = BehaviorLog.INFO
    REPORT_WARN = BehaviorLog.WARN
    REPORT_HINT = BehaviorLog.HINT
    REPORT_ERROR = BehaviorLog.ERROR

    LOGGING_TOPIC = 'flexbe/log'

    _last_status_update = None
    _pub = None

    @staticmethod
    def initialize():
    	Logger._pub = ProxyPublisher({Logger.LOGGING_TOPIC: BehaviorLog})

    @staticmethod
    def log(text, severity):
        if Logger._last_status_update is not None:
            elapsed = rospy.get_rostime() - Logger._last_status_update;
            if (elapsed.to_sec() < 0.1):
                rospy.sleep(0.05)
        Logger._last_status_update = rospy.get_rostime()

        msg = BehaviorLog()
        msg.text = str(text)
        msg.status_code = severity
        Logger._pub.publish(Logger.LOGGING_TOPIC, msg)

        if severity == Logger.REPORT_INFO:
            rospy.loginfo(text)
        elif severity == Logger.REPORT_WARN:
            rospy.logwarn(text)
        elif severity == Logger.REPORT_HINT:
            rospy.loginfo('\033[94mBehavior Hint: %s\033[0m', text)
        elif severity == Logger.REPORT_ERROR:
            rospy.logerr(text)
        

    @staticmethod    
    def loginfo(text):
        Logger.log(text, Logger.REPORT_INFO)
    
    @staticmethod       
    def logwarn(text):
        Logger.log(text, Logger.REPORT_WARN)
    
    @staticmethod       
    def loghint(text):
        Logger.log(text, Logger.REPORT_HINT)
    
    @staticmethod       
    def logerr(text):
        Logger.log(text, Logger.REPORT_ERROR)