#!/usr/bin/env python
import os
import re
import rospy
import rospkg
import roslaunch

from .logger import Logger


class Callback(roslaunch.pmon.ProcessListener):
    def __init__(self, callback):
        self._callback = callback

    def process_died(self, process_name, exit_code):
        rospy.loginfo("Process {} exited with {}".format(process_name, exit_code))
        self._callback(process_name, exit_code)


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

    def wait_for_finishing(self):
        pass

    @property
    def success(self):
        return True


class LaunchContext(TestContext):
    """ Test context that runs a specified launch file configuration. """

    def __init__(self, launch_config, wait_cond="True"):
        self._run_id = rospy.get_param('/run_id')
        launchpath = None
        launchcontent = None

        self._launched_proc_names = []
        self._exit_codes = {}

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

        def store(process_name, exit_code):
            self._exit_codes[process_name] = exit_code
        self._launchrunner.add_process_listener(Callback(store))
        self._wait_cond = wait_cond
        self._valid = True

    def __enter__(self):
        self._launchrunner.launch()
        self._launchrunner.spin_once()
        Logger.print_positive('launchfile running')
        self._valid = True

        self._launched_proc_names = [p.name for p in self._launchrunner.pm.procs]

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

    def wait_for_finishing(self):
        check_exited_rate = rospy.Rate(10)
        rospy.loginfo("Waiting for all launched nodes to exit")
        while not all(name in self._exit_codes for name in self._launched_proc_names):
            check_exited_rate.sleep()

    def __exit__(self, exception_type, exception_value, traceback):
        self._launchrunner.stop()
        Logger.print_positive('launchfile stopped')

    @property
    def success(self):
        return not any(code > 0 for code in self._exit_codes.values())
