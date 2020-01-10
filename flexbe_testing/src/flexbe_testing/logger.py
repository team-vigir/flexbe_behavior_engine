#!/usr/bin/env python
import traceback
import rospy


class Logger(object):
    """ Bundles static methods for test case logging. """

    @classmethod
    def _param_positive(cls):
        return not cls._param_compact() and rospy.get_param('~print_debug_positive', True)

    @classmethod
    def _param_negative(cls):
        return cls._param_compact() or rospy.get_param('~print_debug_negative', True)

    @classmethod
    def _param_compact(cls):
        return rospy.get_param('~compact_format', False)

    @classmethod
    def _prefix(cls):
        return '  >' if cls._param_compact() else '>>>'

    @classmethod
    def _counter(cls):
        cls._counter_value += 1
        return cls._counter_value
    _counter_value = 0

    @classmethod
    def mute_rospy(cls):
        """ Conditionally mute the rospy logging channels. """
        if cls._param_compact() or rospy.get_param('~mute_info', False):
            rospy.loginfo = rospy.logdebug
        if cls._param_compact() or rospy.get_param('~mute_warn', False):
            rospy.logwarn = rospy.logdebug
        if not cls._param_compact() and rospy.get_param('~mute_error', False):
            rospy.logerr = rospy.logdebug

    @classmethod
    def print_positive(cls, text):
        """ Print a positive intermediate result. """
        if cls._param_positive():
            print('\033[0m\033[1m  +\033[0m %s' % str(text))

    @classmethod
    def print_negative(cls, text):
        """ Print a negative intermediate result. """
        if cls._param_negative():
            print('\033[0m\033[1m  -\033[0m %s' % str(text))

    @classmethod
    def print_title(cls, test_name, test_class, result=None):
        """ Print the title of the test, should be called once and before any other print method. """
        test_result = ' > %s' % result if result is not None else ''
        print('\033[34;1m#%2d %s \033[0m\033[34m(%s%s)\033[0m' % (
            cls._counter(), test_name, test_class, test_result
        ))

    @classmethod
    def print_result(cls, test_name, success):
        """ Print the result, should be called once and after any other print method. """
        test_result = 'completed' if success else 'failed'
        color = '32' if success else '31'
        print('\033[%s;1m%s\033[0m\033[%sm %s %s!\033[0m' % (color, cls._prefix(), color, test_name, test_result))

    @classmethod
    def print_failure(cls, text):
        """ Instead of a result, print the failure of a test case once after any other print method. """
        traceback.print_exc()
        print('\033[31;1m%s\033[0m\033[31m %s\033[0m' % (cls._prefix(), str(text)))

    @classmethod
    def print_error(cls, text):
        """ Print an internal error that might cause unexpected behavior, but does not cause failure itself. """
        print('\033[33;1m   \033[0m\033[33m %s\033[0m' % str(text))

    def __init__(self):
        """ DO NOT USE: use class print methods instead. """
        raise NotImplementedError("use static methods and attributes")
