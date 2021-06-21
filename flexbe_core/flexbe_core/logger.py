#!/usr/bin/env python
from rclpy.node import Node

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
    _node = None

    @staticmethod
    def initialize(node: Node):
        Logger._node = node
        Logger._pub = node.create_publisher(BehaviorLog, Logger.LOGGING_TOPIC, 100)

    @staticmethod
    def log(text: str, severity: int):
        if Logger._node is None:
            raise RuntimeError('Unable to log, run "Logger.initialize" first to define the target ROS node.')
        # send message with logged text
        msg = BehaviorLog()
        msg.text = str(text)
        msg.status_code = severity
        Logger._pub.publish(msg)
        # also log locally
        Logger.local(text, severity)

    @staticmethod
    def local(text: str, severity: int):
        if Logger._node is None:
            raise RuntimeError('Unable to log, run "Logger.initialize" first to define the target ROS node.')
        if severity == Logger.REPORT_INFO:
            Logger._node.get_logger().info(text)
        elif severity == Logger.REPORT_WARN:
            Logger._node.get_logger().warning(text)
        elif severity == Logger.REPORT_HINT:
            Logger._node.get_logger().info('\033[94mBehavior Hint: %s\033[0m', text)
        elif severity == Logger.REPORT_ERROR:
            Logger._node.get_logger().error(text)
        elif severity == Logger.REPORT_DEBUG:
            Logger._node.get_logger().debug(text)
        else:
            Logger._node.get_logger().debug(text + ' (unknown log level %s)' % str(severity))

    @staticmethod
    def logdebug(text, *args):
        Logger.log(text % args, Logger.REPORT_DEBUG)

    @staticmethod
    def loginfo(text, *args):
        Logger.log(text % args, Logger.REPORT_INFO)

    @staticmethod
    def logwarn(text, *args):
        Logger.log(text % args, Logger.REPORT_WARN)

    @staticmethod
    def loghint(text, *args):
        Logger.log(text % args, Logger.REPORT_HINT)

    @staticmethod
    def logerr(text, *args):
        Logger.log(text % args, Logger.REPORT_ERROR)

    @staticmethod
    def localdebug(text, *args):
        Logger.local(text % args, Logger.REPORT_DEBUG)

    @staticmethod
    def localinfo(text, *args):
        Logger.local(text % args, Logger.REPORT_INFO)

    @staticmethod
    def debug(text, *args):
        Logger.logdebug(text, *args)

    @staticmethod
    def info(text, *args):
        Logger.loginfo(text, *args)

    @staticmethod
    def warning(text, *args):
        Logger.logwarn(text, *args)

    @staticmethod
    def hint(text, *args):
        Logger.loghint(text, *args)

    @staticmethod
    def error(text, *args):
        Logger.logerr(text, *args)
