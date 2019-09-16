#!/usr/bin/env python
import os
import re
import rospy
import rospkg
import roslaunch

from .logger import Logger


class TestContext(object):
    """
    Default context for a test case.
    Use as a 'with' statement and run 'verify' to check whether the context is valid.
    """

    def __init__(self):
        pass

    def __enter__(self):
        pass

    def verify(self):
        return True

    def spin_once(self):
        pass

    def __exit__(self, exception_type, exception_value, traceback):
        pass


class LaunchContext(TestContext):
    """ Test context that runs a specified launch file configuration. """

    def __init__(self, launch_config, wait_cond="True"):
        self._run_id = rospy.get_param('/run_id')
        launchpath = None
        launchcontent = None

        # load from system path
        if launch_config.startswith('~') or launch_config.startswith('/'):
            launchpath = os.path.expanduser(launch_config)
        # load from package path
        elif re.match(r'.+\.launch$', launch_config):
            rp = rospkg.RosPack()
            pkgpath = rp.get_path(launch_config.split('/')[0])
            launchpath = os.path.join(pkgpath, '/'.join(launch_config.split('/')[1:]))
        # load from config definition
        else:
            launchcontent = launch_config

        launchconfig = roslaunch.config.ROSLaunchConfig()
        loader = roslaunch.xmlloader.XmlLoader()
        if launchpath is not None:
            loader.load(launchpath, launchconfig, verbose=False)
        else:
            loader.load_string(launchcontent, launchconfig, verbose=False)
        self._launchrunner = roslaunch.launch.ROSLaunchRunner(self._run_id, launchconfig)
        self._wait_cond = wait_cond
        self._valid = True

    def __enter__(self):
        self._launchrunner.launch()
        self._launchrunner.spin_once()
        Logger.print_positive('launchfile running')
        self._valid = True

        try:
            check_running_rate = rospy.Rate(10)
            is_running = False
            while not is_running:
                is_running = eval(self._wait_cond)
                check_running_rate.sleep()
            Logger.print_positive('waiting condition satisfied')
        except Exception as e:
            self._valid = False
            Logger.print_negative('unable to check waiting condition:\n\t%s' % str(e))

    def verify(self):
        return self._valid

    def spin_once(self):
        self._launchrunner.spin_once()

    def __exit__(self, exception_type, exception_value, traceback):
        self._launchrunner.stop()
        Logger.print_positive('launchfile stopped')
