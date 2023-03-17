#!/usr/bin/env python
import rospy

from flexbe_msgs.msg import BehaviorLog


class Logger(object):
    '''
    Realizes behavior-specific logging.
    '''
    REPORT_INFO = BehaviorLog.INFO
    REPORT_WARN = BehaviorLog.WARN
    REPORT_HINT = BehaviorLog.HINT
    REPORT_ERROR = BehaviorLog.ERROR
    REPORT_DEBUG = BehaviorLog.DEBUG

    LOGGING_TOPIC = 'flexbe/log'

    _pub = None

    @staticmethod
    def initialize():
        Logger._pub = rospy.Publisher(Logger.LOGGING_TOPIC, BehaviorLog, queue_size=100)
        Logger._last_logged = {}

    @staticmethod
    def log(text, severity):
        if Logger._pub is None:
            Logger.initialize()
        # send message with logged text
        msg = BehaviorLog()
        msg.text = str(text)
        msg.status_code = severity
        Logger._pub.publish(msg)
        # also log locally
        Logger.local(text, severity)

    @staticmethod
    def log_throttle(period, text, severity):
        # creat unique identifier for each logging message
        log_id = str(severity)  + "_" + text
        # only log when it's the first time or period time has passed for the logging message
        if not log_id in Logger._last_logged.keys() or \
            rospy.Time.now().to_sec() - Logger._last_logged[log_id].to_sec() > period:
                Logger.log(text, severity)
                Logger._last_logged.update({log_id: rospy.Time.now()})

    @staticmethod
    def local(text, severity):
        if severity == Logger.REPORT_INFO:
            rospy.loginfo(text)
        elif severity == Logger.REPORT_WARN:
            rospy.logwarn(text)
        elif severity == Logger.REPORT_HINT:
            rospy.loginfo('\033[94mBehavior Hint: %s\033[0m', text)
        elif severity == Logger.REPORT_ERROR:
            rospy.logerr(text)
        elif severity == Logger.REPORT_DEBUG:
            rospy.logdebug(text)
        else:
            rospy.logdebug(text + ' (unknown log level %s)' % str(severity))

    @staticmethod
    def logdebug(text: str, *args):
        Logger.log(text % args, Logger.REPORT_DEBUG)

    @staticmethod
    def loginfo(text: str, *args):
        Logger.log(text % args, Logger.REPORT_INFO)

    @staticmethod
    def logwarn(text: str, *args):
        Logger.log(text % args, Logger.REPORT_WARN)

    @staticmethod
    def loghint(text: str, *args):
        Logger.log(text % args, Logger.REPORT_HINT)

    @staticmethod
    def logerr(text: str, *args):
        Logger.log(text % args, Logger.REPORT_ERROR)

    @staticmethod
    def logdebug_throttle(period: float, text: str, *args):
        Logger.log_throttle(period, text % args, Logger.REPORT_DEBUG)

    @staticmethod
    def loginfo_throttle(period: float, text: str, *args):
        Logger.log_throttle(period, text % args, Logger.REPORT_INFO)


    @staticmethod
    def logwarn_throttle(period: float, text: str, *args):
        Logger.log_throttle(period, text % args, Logger.REPORT_WARN)

    @staticmethod
    def loghint_throttle(period: float, text: str, *args):
        Logger.log_throttle(period, text % args, Logger.REPORT_HINT)


    @staticmethod
    def logerr_throttle(period: float, text: str, *args):
        Logger.log_throttle(period, text % args, Logger.REPORT_ERROR)


    @staticmethod
    def localdebug(text: str, *args):
        Logger.local(text % args, Logger.REPORT_DEBUG)

    @staticmethod
    def localinfo(text: str, *args):
        Logger.local(text % args, Logger.REPORT_INFO)

    @staticmethod
    def localwarn(text: str, *args):
        Logger.local(text % args, Logger.REPORT_WARN)

    @staticmethod
    def localhint(text: str, *args):
        Logger.local(text % args, Logger.REPORT_HINT)

    @staticmethod
    def localerr(text: str, *args):
        Logger.local(text % args, Logger.REPORT_ERROR)
