#!/usr/bin/env python
import rospy
import os
import sys
import inspect
import tempfile
import threading
import time
import zlib
import contextlib
from ast import literal_eval as cast

from flexbe_core import Logger, BehaviorLibrary
from flexbe_core.proxy import ProxyPublisher, ProxySubscriberCached

from flexbe_msgs.msg import BehaviorSelection, BEStatus, CommandFeedback
from std_msgs.msg import Empty

import diagnostic_msgs
import diagnostic_updater

class FlexbeOnboard(object):
    """
    Controls the execution of robot behaviors.
    """

    def __init__(self):
        self.be = None
        Logger.initialize()
        self._tracked_imports = list()
        # prepare temp folder
        self._tmp_folder = tempfile.mkdtemp()
        sys.path.append(self._tmp_folder)
        rospy.on_shutdown(self._cleanup_tempdir)

        # prepare manifest folder access
        self._behavior_lib = BehaviorLibrary()

        self._diagnostic_updater = diagnostic_updater.Updater()
        self._diagnostic_updater.setHardwareID('flexbe_onboard')
        self._diagnostic_updater.add("method updater", self.produce_diagnostics)
        self._diag_lock = threading.Lock()

        # prepare communication
        self.status_topic = 'flexbe/status'
        self.feedback_topic = 'flexbe/command_feedback'
        self._pub = ProxyPublisher({
            self.feedback_topic: CommandFeedback,
            'flexbe/heartbeat': Empty
        })
        self._pub.createPublisher(self.status_topic, BEStatus, _latch=True)

        # listen for new behavior to start
        self._enable_clear_imports = rospy.get_param('~enable_clear_imports', False)
        self._running = False
        self._run_lock = threading.Lock()
        self._switching = False
        self._switch_lock = threading.Lock()
        self._sub = ProxySubscriberCached()
        self._sub.subscribe('flexbe/start_behavior', BehaviorSelection, self._behavior_callback)

        rospy.sleep(0.5)  # wait for publishers etc to really be set up
        self._pub.publish(self.status_topic, BEStatus(code=BEStatus.READY))

        self._execute_heartbeat()  # start hearbeat and diagnostics thread last
        rospy.loginfo('\033[92m--- Behavior Engine ready! ---\033[0m')

    def _behavior_callback(self, msg):
        thread = threading.Thread(target=self._behavior_execution, args=[msg])
        thread.daemon = True
        thread.start()

    # =================== #
    # Main execution loop #
    # ------------------- #

    def _behavior_execution(self, msg):
        # sending a behavior while one is already running is considered as switching
        if self._running:
            Logger.loginfo('--> Initiating behavior switch...')
            self._pub.publish(self.feedback_topic, CommandFeedback(command="switch", args=['received']))
        else:
            Logger.loginfo('--> Starting new behavior...')

        # construct the behavior that should be executed
        be = self._prepare_behavior(msg)
        if be is None:
            Logger.logerr('Dropped behavior start request because preparation failed.')
            if self._running:
                self._pub.publish(self.feedback_topic, CommandFeedback(command="switch", args=['failed']))
            else:
                rospy.loginfo('\033[92m--- Behavior Engine ready! ---\033[0m')
            return

        # perform the behavior switch if required
        with self._switch_lock:
            self._switching = True
            if self._running:
                self._pub.publish(self.feedback_topic, CommandFeedback(command="switch", args=['start']))
                # ensure that switching is possible
                if not self._is_switchable(be):
                    Logger.logerr('Dropped behavior start request because switching is not possible.')
                    self._pub.publish(self.feedback_topic, CommandFeedback(command="switch", args=['not_switchable']))
                    return
                # wait if running behavior is currently starting or stopping
                rate = rospy.Rate(100)
                while not rospy.is_shutdown():
                    active_state = self.be.get_current_state()
                    if active_state is not None or not self._running:
                        break
                    rate.sleep()
                # extract the active state if any
                if active_state is not None:
                    rospy.loginfo("Current state %s is kept active.", active_state.name)
                    try:
                        be.prepare_for_switch(active_state)
                        self._pub.publish(self.feedback_topic, CommandFeedback(command="switch", args=['prepared']))
                    except Exception as e:
                        Logger.logerr('Failed to prepare behavior switch:\n%s' % str(e))
                        self._pub.publish(self.feedback_topic, CommandFeedback(command="switch", args=['failed']))
                        return
                    # stop the rest
                    rospy.loginfo('Preempting current behavior version...')
                    self.be.preempt()

        # execute the behavior
        with self._run_lock:
            self._switching = False
            self.be = be
            with self._diag_lock:
                self._running = True

            result = None
            try:
                rospy.loginfo('Behavior ready, execution starts now.')
                rospy.loginfo('[%s : %s]', be.name, msg.behavior_checksum)
                self.be.confirm()
                args = [self.be.requested_state_path] if self.be.requested_state_path is not None else []
                self._pub.publish(self.status_topic,
                                  BEStatus(behavior_id=self.be.id, code=BEStatus.STARTED, args=args))
                result = self.be.execute()
                if self._switching:
                    self._pub.publish(self.status_topic,
                                      BEStatus(behavior_id=self.be.id, code=BEStatus.SWITCHING))
                else:
                    self._pub.publish(self.status_topic,
                                      BEStatus(behavior_id=self.be.id, code=BEStatus.FINISHED, args=[str(result)]))
            except Exception as e:
                self._pub.publish(self.status_topic, BEStatus(behavior_id=msg.behavior_checksum, code=BEStatus.FAILED))
                Logger.logerr('Behavior execution failed!\n%s' % str(e))
                import traceback
                Logger.loginfo(traceback.format_exc())
                result = result or "exception"  # only set result if not executed

            # done, remove left-overs like the temporary behavior file
            try:
                # do not clear imports for now, not working correctly (e.g., flexbe/flexbe_app#66)
                # only if specifically enabled
                if not self._switching and self._enable_clear_imports:
                    self._clear_imports()
                self._cleanup_behavior(msg.behavior_checksum)
            except Exception as e:
                rospy.logerr('Failed to clean up behavior:\n%s' % str(e))

            if not self._switching:
                Logger.loginfo('Behavior execution finished with result %s.', str(result))
                rospy.loginfo('\033[92m--- Behavior Engine ready! ---\033[0m')
            with self._diag_lock:
                self._running = False
            self.be = None

    # ==================================== #
    # Preparation of new behavior requests #
    # ------------------------------------ #

    def _prepare_behavior(self, msg):
        # get sourcecode from ros package
        try:
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
            try:
                be_content = be_file.read()
            finally:
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
            if zlib.adler32(file_content.encode()) & 0x7fffffff != msg.behavior_checksum:
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
        try:
            file_path = os.path.join(self._tmp_folder, 'tmp_%d.py' % msg.behavior_checksum)
            with open(file_path, "w") as sc_file:
                sc_file.write(file_content)
        except Exception as e:
            Logger.logerr('Failed to create temporary file for behavior class:\n%s' % str(e))
            self._pub.publish(self.status_topic, BEStatus(behavior_id=msg.behavior_checksum, code=BEStatus.ERROR))
            return

        # import temp class file and initialize behavior
        try:
            with self._track_imports():
                package = __import__("tmp_%d" % msg.behavior_checksum, fromlist=["tmp_%d" % msg.behavior_checksum])
                clsmembers = inspect.getmembers(package, lambda member: (inspect.isclass(member) and
                                                                         member.__module__ == package.__name__))
                beclass = clsmembers[0][1]
                be = beclass()
            rospy.loginfo('Behavior ' + be.name + ' created.')
        except Exception as e:
            Logger.logerr('Exception caught in behavior definition:\n%s\n'
                          'See onboard terminal for more information.' % str(e))
            import traceback
            traceback.print_exc()
            self._pub.publish(self.status_topic, BEStatus(behavior_id=msg.behavior_checksum, code=BEStatus.ERROR))
            if self._enable_clear_imports:
                self._clear_imports()
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
            Logger.logerr('Behavior construction failed!\n%s\n'
                          'See onboard terminal for more information.' % str(e))
            import traceback
            traceback.print_exc()
            self._pub.publish(self.status_topic, BEStatus(behavior_id=msg.behavior_checksum, code=BEStatus.ERROR))
            if self._enable_clear_imports:
                self._clear_imports()
            return

        return be

    # ================ #
    # Helper functions #
    # ---------------- #

    def _is_switchable(self, be):
        if self.be.name != be.name:
            Logger.logerr('Unable to switch behavior, names do not match:\ncurrent: %s <--> new: %s' %
                          (self.be.name, be.name))
            return False
        # locked inside
        # locked state exists in new behavior
        # ok, can switch
        return True

    def _cleanup_behavior(self, behavior_checksum):
        file_path = os.path.join(self._tmp_folder, 'tmp_%d.pyc' % behavior_checksum)
        try:
            os.remove(file_path)
        except OSError:
            pass
        try:
            os.remove(file_path + 'c')
        except OSError:
            pass

    def _clear_imports(self):
        for module in self._tracked_imports:
            if module in sys.modules:
                del sys.modules[module]
        self._tracked_imports = list()

    def _cleanup_tempdir(self):
        try:
            os.remove(self._tmp_folder)
        except OSError:
            pass

    def _convert_input_data(self, keys, values):
        result = dict()
        for k, v in zip(keys, values):
            # action call has empty string as default, not a valid input key
            if k == '':
                continue
            try:
                result[k] = self._convert_dict(cast(v))
            except ValueError:
                # unquoted strings will raise a ValueError, so leave it as string in this case
                result[k] = str(v)
            except SyntaxError as se:
                Logger.loginfo('Unable to parse input value for key "%s", assuming string:\n%s\n%s' %
                               (k, str(v), str(se)))
                result[k] = str(v)
        return result

    def _execute_heartbeat(self):
        thread = threading.Thread(target=self._heartbeat_worker)
        thread.daemon = True
        thread.start()

    def _heartbeat_worker(self):
        while True:
            self._pub.publish('flexbe/heartbeat', Empty())
            self._diagnostic_updater.update()
            time.sleep(1)

    def produce_diagnostics(self, stat):
        with self._diag_lock:
            if self._running:
                stat.summary(diagnostic_msgs.msg.DiagnosticStatus.OK,
                            "behavior running")
            else:
                stat.summary(diagnostic_msgs.msg.DiagnosticStatus.WARN,
                            "no behavior running")

            if self.be:
                stat.add("behavior.id", self.be.id)
                stat.add("behavior.name", self.be.name)

                active_state = self.be.get_current_state()
                if active_state:
                    stat.add("behavior.active_state", active_state.name)
                else:
                    stat.mergeSummary(diagnostic_msgs.msg.DiagnosticStatus.WARN,
                                "no state active")
            else:
                stat.mergeSummary(diagnostic_msgs.msg.DiagnosticStatus.WARN,
                            "no behavior started")

    def _convert_dict(self, o):
        if isinstance(o, list):
            return [self._convert_dict(e) for e in o]
        elif isinstance(o, dict):
            return self._attr_dict((k, self._convert_dict(v)) for k, v in list(o.items()))
        else:
            return o

    class _attr_dict(dict):
        __getattr__ = dict.__getitem__

    @contextlib.contextmanager
    def _track_imports(self):
        previous_modules = set(sys.modules.keys())
        try:
            yield
        finally:
            self._tracked_imports.extend(set(sys.modules.keys()) - previous_modules)
