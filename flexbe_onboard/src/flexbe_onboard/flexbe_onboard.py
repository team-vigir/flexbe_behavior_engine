#!/usr/bin/env python

import roslib; roslib.load_manifest('flexbe_onboard')
import rospy
import rospkg
import os
import sys
import inspect
import threading
import time
import smach
import random
import yaml
import zlib
import xml.etree.ElementTree as ET

from flexbe_core import Logger
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


    def __init__(self):
        '''
        Constructor
        '''
        self.be = None
        Logger.initialize()
        smach.set_loggers (
            rospy.logdebug, # hide SMACH transition log spamming
            rospy.logwarn,
            rospy.logdebug,
            rospy.logerr
        )

        #ProxyPublisher._simulate_delay = True
        #ProxySubscriberCached._simulate_delay = True

        behaviors_package = "flexbe_behaviors"
        if rospy.has_param("~behaviors_package"):
            behaviors_package = rospy.get_param("~behaviors_package")
            rospy.loginfo("Using custom behaviors package: %s" % behaviors_package)
        else:
            rospy.loginfo("Using default behaviors package: %s" % behaviors_package)

        # prepare temp folder
        rp = rospkg.RosPack()
        self._tmp_folder = os.path.join(rp.get_path('flexbe_onboard'), 'tmp/')
        if not os.path.exists(self._tmp_folder):
            os.makedirs(self._tmp_folder)
        sys.path.append(self._tmp_folder)
        
        # prepare manifest folder access
        manifest_folder = os.path.join(rp.get_path(behaviors_package), 'behaviors/')
        rospy.loginfo("Parsing available behaviors...")
        file_entries = [os.path.join(manifest_folder, filename) for filename in os.listdir(manifest_folder) if not filename.startswith('#')]
        manifests = sorted([xmlpath for xmlpath in file_entries if not os.path.isdir(xmlpath)])
        self._behavior_lib = dict()
        for i in range(len(manifests)):
            m = ET.parse(manifests[i]).getroot()
            e = m.find("executable")
            self._behavior_lib[i] = {"name": m.get("name"), "package": e.get("package_path").split(".")[0], "file": e.get("package_path").split(".")[1], "class": e.get("class")}
#            rospy.loginfo("+++ " + self._behavior_lib[i]["name"])
        
        # enable automatic reloading of all subsequent modules on reload
        _reload_importer = ReloadImporter()
        _reload_importer.add_reload_path(manifest_folder)
        _reload_importer.enable()
        
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
            behavior = self._behavior_lib[msg.behavior_id]
            be_filepath = os.path.join(rp.get_path(behavior["package"]), 'src/' + behavior["package"] + '/' + behavior["file"] + '_tmp.py')
            if os.path.isfile(be_filepath):
                be_file = open(be_filepath, "r")
                rospy.logwarn("Found a tmp version of the referred behavior! Assuming local test run.")
            else:
                be_filepath = os.path.join(rp.get_path(behavior["package"]), 'src/' + behavior["package"] + '/' + behavior["file"] + '.py')
                be_file = open(be_filepath, "r")
            be_content = be_file.read()
            be_file.close()
        except Exception as e:
            Logger.logerr('Failed to retrieve behavior from library:\n%s' % str(e))
            self._pub.publish(self.status_topic, BEStatus(behavior_id=msg.behavior_checksum, code=BEStatus.ERROR))
            return

        # apply modifications if any
        try:
            file_content = ""
            last_index = 0
            for mod in msg.modifications:
                file_content += be_content[last_index:mod.index_begin] + mod.new_content
                last_index = mod.index_end
            file_content += be_content[last_index:]
            if zlib.adler32(file_content) != msg.behavior_checksum:
                mismatch_msg = ("Checksum mismatch of behavior versions! \n"
                                "Attempted to load behavior: %s\n"
                                "Make sure that all computers are on the same version."  % str(be_filepath))
                raise Exception(mismatch_msg)
            else:
                rospy.loginfo("Successfully applied %d modifications." % len(msg.modifications))
        except Exception as e:
            Logger.logerr('Failed to apply behavior modifications:\n%s' % str(e))
            self._pub.publish(self.status_topic, BEStatus(behavior_id=msg.behavior_checksum, code=BEStatus.ERROR))
            return

        # create temp file for behavior class
        try:
            file_path = os.path.join(self._tmp_folder, 'tmp_%d.py' % msg.behavior_checksum)
            sc_file = open(file_path, "w")
            sc_file.write(file_content)
            sc_file.close()
        except Exception as e:
            Logger.logerr('Failed to create temporary file for behavior class:\n%s' % str(e))
            self._pub.publish(self.status_topic, BEStatus(behavior_id=msg.behavior_checksum, code=BEStatus.ERROR))
            return

        # import temp class file and initialize behavior
        try:
            package = __import__("tmp_%d" % msg.behavior_checksum, fromlist=["tmp_%d" % msg.behavior_checksum])
            clsmembers = inspect.getmembers(package, lambda member: inspect.isclass(member) and member.__module__ == package.__name__)
            beclass = clsmembers[0][1]
            be = beclass()
            rospy.loginfo('Behavior ' + be.name + ' created.')
        except Exception as e:
            Logger.logerr('Exception caught in behavior definition:\n%s' % str(e))
            self._pub.publish(self.status_topic, BEStatus(behavior_id=msg.behavior_checksum, code=BEStatus.ERROR))
            return
        
        # import contained behaviors
        contain_list = {}
        try:
            contain_list = self._build_contains(be, "")
        except Exception as e:
            Logger.logerr('Failed to load contained behaviors:\n%s' % str(e))
            return
            
        # initialize behavior parameters
        if len(msg.arg_keys) > 0:
            rospy.loginfo('The following parameters will be used:')
        try:
            for i in range(len(msg.arg_keys)):
                if msg.arg_keys[i] == '':
                    # action call has empty string as default, not a valid param key
                    continue 
                key_splitted = msg.arg_keys[i].rsplit('/', 1)
                if len(key_splitted) == 1:
                    behavior = ''
                    key = key_splitted[0]
                    rospy.logwarn('Parameter key %s has no path specification, assuming: /%s' % (key, key))
                else:
                    behavior = key_splitted[0]
                    key = key_splitted[1]
                found = False
                
                if behavior == '' and hasattr(be, key):
                    self._set_typed_attribute(be, key, msg.arg_values[i])
                    # propagate to contained behaviors
                    for b in contain_list:
                        if hasattr(contain_list[b], key):
                            self._set_typed_attribute(contain_list[b], key, msg.arg_values[i], b)
                    found = True

                for b in contain_list:
                    if b == behavior and hasattr(contain_list[b], key):
                        self._set_typed_attribute(contain_list[b], key, msg.arg_values[i], b)
                        found = True
                            
                if not found:   
                    rospy.logwarn('Parameter ' + msg.arg_keys[i] + ' (set to ' + msg.arg_values[i] + ') not implemented')

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
        del(sys.modules["tmp_%d" % behavior_checksum])
        file_path = os.path.join(self._tmp_folder, 'tmp_%d.pyc' % behavior_checksum)
        os.remove(file_path)
        
        
    def _set_typed_attribute(self, obj, name, value, behavior=''):
        attr = getattr(obj, name)
        if type(attr) is int:
            value = int(value)
        elif type(attr) is long:
            value = long(value)
        elif type(attr) is float:
            value = float(value)
        elif type(attr) is bool:
            value = (value != "0")
        elif type(attr) is dict:
            value = yaml.load(value)
        setattr(obj, name, value)
        suffix = ' (' + behavior + ')' if behavior != '' else ''
        rospy.loginfo(name + ' = ' + str(value) + suffix)


    def _convert_input_data(self, keys, values):
        # there is no reliable clean way to check type conversion in Python
        result = dict()
        for k,v in zip(keys, values):
            result[k] = v
            try:
                result[k] = int(v)
                continue
            except ValueError:
                pass
            try:
                result[k] = float(v)
                continue
            except ValueError:
                pass
            if v.lower() == 'false':
                result[k] = False
                continue
            if v.lower() == 'true':
                result[k] = True
                continue
            if len(v) >= 2 and v[0] == '[' and v[-1] == ']':
                result[k] = v[1:-1].split(',')
        return result

        

    def _build_contains(self, obj, path):
        contain_list = dict((path+"/"+key,value) for (key,value) in getattr(obj, 'contains', {}).items())
        add_to_list = {}
        for b_id, b_inst in contain_list.items():
            add_to_list.update(self._build_contains(b_inst, b_id))
        contain_list.update(add_to_list)
        return contain_list


    def _execute_heartbeat(self):
        thread = threading.Thread(target=self._heartbeat_worker)
        thread.daemon = True
        thread.start()

    def _heartbeat_worker(self):
        while True:
            self._pub.publish('flexbe/heartbeat', Empty())
            time.sleep(1) # sec










        
