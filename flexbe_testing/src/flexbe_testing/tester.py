#!/usr/bin/env python
import smach
import rosunit
import unittest

from .logger import Logger
from .test_interface import TestInterface
from .test_context import TestContext, LaunchContext
from .data_provider import DataProvider


class Tester(object):

    def __init__(self):
        self._tests = dict()

    def run_test(self, name, config):
        try:
            self._verify_config(config)
        except Exception as e:
            Logger.print_title(name, 'Invalid', None)
            Logger.print_error('invalid test specification:\n\t%s' % str(e))
            Logger.print_result(name, False)
            self._tests['test_%s_pass' % name] = self._test_config_invalid(str(e))
            return 0

        import_only = config.get('import_only', False)
        Logger.print_title(name, config['class'], config['outcome'] if not import_only else None)

        # import test subject
        try:
            test_interface = TestInterface(config['path'], config['class'])
        except Exception as e:
            Logger.print_failure('unable to import state %s (%s):\n\t%s' %
                                 (config['class'], config['path'], str(e)))
            self._tests['test_%s_pass' % name] = self._test_pass(False)
            return 0

        if not import_only:
            # prepare test context
            context = None
            if 'launch' in config:
                context = LaunchContext(config['launch'], config.get('wait_cond', 'True'))
            else:
                context = TestContext()

            # load data source
            try:
                data = DataProvider(bagfile=config.get('data', None))
            except Exception as e:
                Logger.print_failure('unable to load data source %s:\n\t%s' %
                                     (config['data'], str(e)))
                self._tests['test_%s_pass' % name] = self._test_pass(False)
                return 0

            # run test context
            with context:
                if not context.verify():
                    Logger.print_error('failed to initialize test context:\n\t%s' % config['launch'])
                    self._tests['test_%s_pass' % name] = self._test_pass(False)
                    return 0

                # instantiate test subject
                params = {key: data.parse(value) for key, value in config.get('params', dict()).items()}
                try:
                    test_interface.instantiate(params)
                except Exception as e:
                    Logger.print_failure('unable to instantiate %s (%s) with params:\n\t%s\n\t%s' %
                                         (config['class'], config['path'], str(params), str(e)))
                    self._tests['test_%s_pass' % name] = self._test_pass(False)
                    return 0

                # prepare user data
                userdata = smach.UserData()
                for input_key, input_value in config.get('input', dict()).items():
                    userdata[input_key] = data.parse(input_value)
                expected = {key: data.parse(value) for key, value in config.get('output', dict()).items()}

                # run test subject
                try:
                    outcome = test_interface.execute(userdata, spin_cb=context.spin_once)
                except Exception as e:
                    Logger.print_failure('failed to execute %s (%s)\n\t%s' %
                                         (config['class'], config['path'], str(e)))
                    self._tests['test_%s_pass' % name] = self._test_pass(False)
                    return 0

            # evaluate outcome
            self._tests['test_%s_outcome' % name] = self._test_outcome(outcome, config['outcome'])
            outcome_ok = outcome == config['outcome']
            if outcome_ok:
                Logger.print_positive('correctly returned outcome %s' % outcome)
            else:
                Logger.print_negative('wrong outcome: %s' % outcome)

            # evaluate output
            output_ok = True
            for expected_key, expected_value in expected.items():
                if expected_key in userdata.keys():
                    equals = userdata[expected_key] == expected_value
                    self._tests['test_%s_output_%s' % (name, expected_key)] = \
                        self._test_output(userdata[expected_key], expected_value)
                    if not equals:
                        Logger.print_negative('wrong result for %s: %s != %s' %
                                              (expected_key, userdata[expected_key], expected_value))
                        output_ok = False
                else:
                    Logger.print_negative('no result for %s' % expected_key)
                    output_ok = False
            if len(expected) > 0 and output_ok:
                Logger.print_positive('all result outputs match expected')

        # report result
        success = import_only or outcome_ok and output_ok
        Logger.print_result(name, success)
        self._tests['test_%s_pass' % name] = self._test_pass(success)
        return 1 if success else 0

    def _verify_config(self, config):
        if not isinstance(config, dict):
            raise AssertionError('config needs to be a dictionary but is:\n\t%s' % str(config))
        assert 'path' in config
        assert 'class' in config
        assert 'outcome' in config or config.get('import_only', False)

    # ROSUNIT interface

    def perform_rostest(self, test_pkg):
        TestCase = type(test_pkg + '_test_class', (unittest.TestCase,), self._tests)
        rosunit.unitrun(test_pkg, test_pkg + '_flexbe_tests', TestCase)

    def _test_output(self, value, expected):
        def _test_call(test_self):
            test_self.assertEquals(value, expected, "Output value %s does not match expected %s" % (value, expected))
        return _test_call

    def _test_outcome(self, outcome, expected):
        def _test_call(test_self):
            test_self.assertEquals(outcome, expected, "Outcome %s does not match expected %s" % (outcome, expected))
        return _test_call

    def _test_pass(self, passed):
        def _test_call(test_self):
            test_self.assertTrue(passed, "Did not pass configured tests.")
        return _test_call

    def _test_config_invalid(self, config):
        def _test_call(test_self):
            test_self.fail("Test config is invalid: %s" % config)
        return _test_call
