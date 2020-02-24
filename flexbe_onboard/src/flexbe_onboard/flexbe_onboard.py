#!/usr/bin/env python

import roslib; roslib.load_manifest('flexbe_onboard')
import rospy
import rospkg
import os
import sys
import inspect
import tempfile
import threading
import time
import smach
import random
import yaml
import zlib
import xml.etree.ElementTree as ET
from ast import literal_eval as cast

from flexbe_core import Logger, BehaviorLibrary
from flexbe_core.reload_importer import ReloadImporter

from flexbe_msgs.msg import BehaviorSelection, BEStatus, ContainerStructure, CommandFeedback
from flexbe_core.proxy import ProxyPublisher, ProxySubscriberCached

from std_msgs.msg import Int32, Empty


'''
Created on 20.05.2013

@author: Philipp Schillinger
'''

class VigirBeOnboard(object):
    '''
    Implements an idle state where the robot waits for a behavior to be started.
    '''

    TEMP_MODULE_NAME_TEMPLATE = 'tmp_%d'

    def __init__(self):
        '''
        Constructor
        '''
        self.be = None
        self._current_behavior = None
        Logger.initialize()
        smach.set_loggers (
            rospy.logdebug, # hide SMACH transition log spamming
            rospy.logwarn,
            rospy.logdebug,
            rospy.logerr
        )

        #ProxyPublisher._simulate_delay = True
        #ProxySubscriberCached._simulate_delay = True

        # prepare temp folder
        rp = rospkg.RosPack()
        self._tmp_folder = tempfile.mkdtemp()
        sys.path.append(self._tmp_folder)
        rospy.on_shutdown(self._cleanup_tempdir)
        
        # prepare manifest folder access
        self._behavior_lib = BehaviorLibrary()
        
        # enable automatic reloading of all subsequent modules on reload
        reload_importer = ReloadImporter()
        reload_importer.add_reload_path(self._tmp_folder)
        for pkg in self._behavior_lib.behavior_packages:
            reload_importer.add_reload_path(self._behavior_lib.get_sourcecode_filepath(pkg))
        reload_importer.enable()

        self._pub = ProxyPublisher()
        self._sub = ProxySubscriberCached()

        self.status_topic = 'flexbe/status'
        self.feedback_topic = 'flexbe/command_feedback'

        self._pub.createPublisher(self.status_topic, BEStatus, _latch = True)
        self._pub.createPublisher(self.feedback_topic, CommandFeedback)

        # listen for new behavior to start
        self._running = False
        self._switching = False
        self._sub.subscribe('flexbe/start_behavior', BehaviorSelection, self._behavior_callback)

        # heartbeat
        self._pub.createPublisher('flexbe/heartbeat', Empty)
        self._execute_heartbeat()

        rospy.sleep(0.5) # wait for publishers etc to really be set up
        self._pub.publish(self.status_topic, BEStatus(code=BEStatus.READY))
        rospy.loginfo('\033[92m--- Behavior Engine ready! ---\033[0m')


    def _behavior_callback(self, msg):
        thread = threading.Thread(target=self._behavior_execution, args=[msg])
        thread.daemon = True
        thread.start()

    def _behavior_execution(self, msg):
        if self._running:
            Logger.loginfo('--> Initiating behavior switch...')
            self._pub.publish(self.feedback_topic, CommandFeedback(command="switch", args=['received']))
        else:
            Logger.loginfo('--> Starting new behavior...')

        be = self._prepare_behavior(msg)
        if be is None:
            Logger.logerr('Dropped behavior start request because preparation failed.')
            if self._running:
                self._pub.publish(self.feedback_topic, CommandFeedback(command="switch", args=['failed']))
            else:
                rospy.loginfo('\033[92m--- Behavior Engine ready! ---\033[0m')
            return

        if self._running:
            if self._switching:
                Logger.logwarn('Already switching, dropped new start request.')
                return
            self._pub.publish(self.feedback_topic, CommandFeedback(command="switch", args=['start']))
            if not self._is_switchable(be):
                Logger.logerr('Dropped behavior start request because switching is not possible.')
                self._pub.publish(self.feedback_topic, CommandFeedback(command="switch", args=['not_switchable']))
                return
            self._switching = True
            active_state = self.be.get_current_state()
            rospy.loginfo("Current state %s is kept active.", active_state.name)
            try:
                be.prepare_for_switch(active_state)
                self._pub.publish(self.feedback_topic, CommandFeedback(command="switch", args=['prepared']))
            except Exception as e:
                Logger.logerr('Failed to prepare behavior switch:\n%s' % str(e))
                self._switching = False
                self._pub.publish(self.feedback_topic, CommandFeedback(command="switch", args=['failed']))
                return
            rospy.loginfo('Preempting current behavior version...')
            self.be.preempt_for_switch()
            rate = rospy.Rate(10)
            while self._running:
                rate.sleep()
            self._switching = False

        self._running = True
        self.be = be

        result = ""
        try:
            rospy.loginfo('Behavior ready, execution starts now.')
            rospy.loginfo('[%s : %s]', be.name, msg.behavior_checksum)
            self.be.confirm()
            args = [self.be.requested_state_path] if self.be.requested_state_path is not None else []
            self._pub.publish(self.status_topic, BEStatus(behavior_id=self.be.id, code=BEStatus.STARTED, args=args))
            result = self.be.execute()
            if self._switching:
                self._pub.publish(self.status_topic, BEStatus(behavior_id=self.be.id, code=BEStatus.SWITCHING))
            else:
                self._pub.publish(self.status_topic, BEStatus(behavior_id=self.be.id, code=BEStatus.FINISHED, args=[str(result)]))
        except Exception as e:
            self._pub.publish(self.status_topic, BEStatus(behavior_id=msg.behavior_checksum, code=BEStatus.FAILED))
            Logger.logerr('Behavior execution failed!\n%s' % str(e))
            import traceback
            Logger.loginfo(traceback.format_exc())
            result = "failed"

        try:
            self._cleanup_behavior(msg.behavior_checksum)
        except Exception as e:
            rospy.logerr('Failed to clean up behavior:\n%s' % str(e))

        self.be = None
        if not self._switching:
            rospy.loginfo('Behavior execution finished with result %s.', str(result))
            rospy.loginfo('\033[92m--- Behavior Engine ready! ---\033[0m')
        self._running = False


    def _prepare_behavior(self, msg):
        # get sourcecode from ros package
        try:
            rp = rospkg.RosPack()
            # use BehaviorSelection.BEHAVIOR_ID_CURRENT as id to update current behavior
            if self._current_behavior is not None and msg.behavior_id == BehaviorSelection.BEHAVIOR_ID_CURRENT:
                behavior = self._current_behavior
            else:
                behavior = self._behavior_lib.get_behavior(msg.behavior_id)
                if behavior is None:
                    raise ValueError(msg.behavior_id)
            be_filepath = self._behavior_lib.get_sourcecode_filepath(msg.behavior_id, add_tmp=True)
            if os.path.isfile(be_filepath):
                be_file = open(be_filepath, "r")
                rospy.logwarn("Found a tmp version of the referred behavior! Assuming local test run.")
            else:
                be_filepath = self._behavior_lib.get_sourcecode_filepath(msg.behavior_id)
                be_file = open(be_filepath, "r")
            be_content = be_file.read()
            be_file.close()
        except Exception as e:
            Logger.logerr('Failed to retrieve behavior from library:\n%s' % str(e))
            self._pub.publish(self.status_topic, BEStatus(behavior_id=msg.behavior_checksum, code=BEStatus.ERROR))
            return

        self._current_behavior = behavior

        # apply modifications if any
        try:
            file_content = ""
            last_index = 0
            for mod in msg.modifications:
                file_content += be_content[last_index:mod.index_begin] + mod.new_content
                last_index = mod.index_end
            file_content += be_content[last_index:]
            checksum = zlib.adler32(file_content)
            # make checksum check pass when updating current behavior
            if msg.behavior_id == BehaviorSelection.BEHAVIOR_ID_CURRENT:
                msg.behavior_checksum = checksum
            if msg.behavior_checksum != checksum:
                mismatch_msg = ("Checksum mismatch of behavior versions! \n"
                                "Attempted to load behavior: %s\n"
                                "Make sure that all computers are on the same version a.\n"
                                "Also try: rosrun flexbe_widget clear_cache" % str(be_filepath))
                raise Exception(mismatch_msg)
            else:
                rospy.loginfo("Successfully applied %d modifications." % len(msg.modifications))
        except Exception as e:
            Logger.logerr('Failed to apply behavior modifications:\n%s' % str(e))
            self._pub.publish(self.status_topic, BEStatus(behavior_id=msg.behavior_checksum, code=BEStatus.ERROR))
            return

        # create temp file for behavior class
        temp_module_name = self.TEMP_MODULE_NAME_TEMPLATE % msg.behavior_checksum
        try:
            file_path = os.path.join(self._tmp_folder, temp_module_name + '.py')
            sc_file = open(file_path, "w")
            sc_file.write(file_content)
            sc_file.close()
        except Exception as e:
            Logger.logerr('Failed to create temporary file for behavior class:\n%s' % str(e))
            self._pub.publish(self.status_topic, BEStatus(behavior_id=msg.behavior_checksum, code=BEStatus.ERROR))
            return

        # import temp class file and initialize behavior
        try:
            package = __import__(temp_module_name, fromlist=[temp_module_name])
            clsmembers = inspect.getmembers(package, lambda member: inspect.isclass(member) and member.__module__ == package.__name__)
            beclass = clsmembers[0][1]
            be = beclass()
            rospy.loginfo('Behavior ' + be.name + ' created.')
        except Exception as e:
            Logger.logerr('Exception caught in behavior definition:\n%s' % str(e))
            self._pub.publish(self.status_topic, BEStatus(behavior_id=msg.behavior_checksum, code=BEStatus.ERROR))
            return

        # initialize behavior parameters
        if len(msg.arg_keys) > 0:
            rospy.loginfo('The following parameters will be used:')
        try:
            for i in range(len(msg.arg_keys)):
                # action call has empty string as default, not a valid param key
                if msg.arg_keys[i] == '':
                    continue
                found = be.set_parameter(msg.arg_keys[i], msg.arg_values[i])
                if found:
                    name_split = msg.arg_keys[i].rsplit('/', 1)
                    behavior = name_split[0] if len(name_split) == 2 else ''
                    key = name_split[-1]
                    suffix = ' (' + behavior + ')' if behavior != '' else ''
                    rospy.loginfo(key + ' = ' + msg.arg_values[i] + suffix)
                else:
                    rospy.logwarn('Parameter ' + msg.arg_keys[i] + ' (set to ' + msg.arg_values[i] + ') not defined')

        except Exception as e:
            Logger.logerr('Failed to initialize parameters:\n%s' % str(e))
            self._pub.publish(self.status_topic, BEStatus(behavior_id=msg.behavior_checksum, code=BEStatus.ERROR))
            return

        # build state machine
        try:
            be.set_up(id=msg.behavior_checksum, autonomy_level=msg.autonomy_level, debug=False)
            be.prepare_for_execution(self._convert_input_data(msg.input_keys, msg.input_values))
            rospy.loginfo('State machine built.')
        except Exception as e:
           Logger.logerr('Behavior construction failed!\n%s' % str(e))
           self._pub.publish(self.status_topic, BEStatus(behavior_id=msg.behavior_checksum, code=BEStatus.ERROR))
           return

        return be


    def _is_switchable(self, be):
        if self.be.name != be.name:
            Logger.logerr('Unable to switch behavior, names do not match:\ncurrent: %s <--> new: %s' % (self.be.name, be.name))
            return False
        # locked inside
        # locked state exists in new behavior
        #if self.be.id == be.id:
            #Logger.logwarn('Behavior version ID is the same.')
        #    Logger.logwarn('Skipping behavior switch, version ID is the same.')
        #    return False
        # ok, can switch
        return True


    def _cleanup_behavior(self, behavior_checksum):
        temp_module_name = self.TEMP_MODULE_NAME_TEMPLATE % behavior_checksum
        if temp_module_name in sys.modules:
            del(sys.modules[temp_module_name])
        file_path = os.path.join(self._tmp_folder, '%s.py' % temp_module_name)
        try:
            os.remove(file_path)
        except OSError:
            pass
        try:
            os.remove(file_path + 'c')
        except OSError:
            pass


    def _cleanup_tempdir(self):
        try:
            os.remove(self._tmp_folder)
        except OSError:
            pass

    def _convert_input_data(self, keys, values):
        result = dict()

        for k, v in zip(keys, values):
            try:
                result[k] = self._convert_dict(cast(v))
            except ValueError:
                # unquoted strings will raise a ValueError, so leave it as string in this case
                result[k] = str(v)
            except SyntaxError as se:
                Logger.loginfo('Unable to parse input value for key "%s", assuming string:\n%s\n%s' % (k, str(v), str(se)))
                result[k] = str(v)

        return result

    def _execute_heartbeat(self):
        thread = threading.Thread(target=self._heartbeat_worker)
        thread.daemon = True
        thread.start()

    def _heartbeat_worker(self):
        while True:
            self._pub.publish('flexbe/heartbeat', Empty())
            time.sleep(1) # sec

    class _attr_dict(dict): __getattr__ = dict.__getitem__
    def _convert_dict(self, o):
        if isinstance(o, list):
            return [self._convert_dict(e) for e in o]
        elif isinstance(o, dict):
            return self._attr_dict((k, self._convert_dict(v)) for k, v in list(o.items()))
        else:
            return o
