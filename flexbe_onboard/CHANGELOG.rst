^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package flexbe_onboard
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
1.4.0 (2023-05-18)
------------------
* Updates for Melodic and Noetic releases on github.com/FlexBE
* [flexbe_onboard] Merge pull request #165 from HannesBachter/feature/get_userdata - get userdata by service

1.3.1 (2020-12-11)
------------------
* [flexbe_onboard] Fix assertion in onboard test
* [flexbe_onboard] Offer option to enable clearing of imports
  (see `#135 <https://github.com/team-vigir/flexbe_behavior_engine/issues/135>`_)
* [flexbe_onboard] Print stack trace on behavior import errors
* Contributors: Philipp Schillinger

1.3.0 (2020-11-19)
------------------
* Merge remote-tracking branch 'origin/feature/core_rework' into develop
  # Conflicts:
  #	flexbe_core/src/flexbe_core/core/operatable_state_machine.py
  #	flexbe_onboard/src/flexbe_onboard/flexbe_onboard.py
* Add support for python3
* [flexbe_onboard] Remove clearing imports, not working robustly
  (see e.g. `flexbe/flexbe_app#66 <https://github.com/flexbe/flexbe_app/issues/66>`_)
* Major clean-up of most core components
* Remove smach dependency
* Contributors: Philipp Schillinger

1.2.5 (2020-06-14)
------------------
* [flexbe_onboard] Un-import behavior-specific imports after execution
* [flexbe_onboard] Add test cases for onboard engine
* Merge pull request `#113 <https://github.com/team-vigir/flexbe_behavior_engine/issues/113>`_ from team-vigir/feature/state_logger_rework
  State Logger Rework
* Merge branch 'develop' into feature/state_logger_rework
* [flexbe_onboard] Cleanup onboard and add thread locks
  (see `#117 <https://github.com/team-vigir/flexbe_behavior_engine/issues/117>`_)
* [flexbe_onboard] Expose new state logger args in onboard launch file
* Contributors: Philipp Schillinger

1.2.4 (2020-03-25)
------------------
* Merge pull request `#110 <https://github.com/team-vigir/flexbe_behavior_engine/issues/110>`_ from team-vigir/fix/catkin_install
  Let behavior library find sourcecode in devel or install spaces
* Let behavior library find sourcecode in devel or install spaces
  (fix `#104 <https://github.com/team-vigir/flexbe_behavior_engine/issues/104>`_)
* Merge branch 'fmessmer-feature/python3_compatibility' into develop
* python3 compatibility via 2to3
* Contributors: Philipp Schillinger, fmessmer

1.2.3 (2020-01-10)
------------------
* Merge pull request `#97 <https://github.com/team-vigir/flexbe_behavior_engine/issues/97>`_ from team-vigir/feature/test_behaviors
  flexbe_testing support for behaviors
* [flexbe_core] [flexbe_onboard] Move behavior parametrization to core
* Merge remote-tracking branch 'origin/develop' into feature/test_behaviors
  # Conflicts:
  #	flexbe_testing/bin/testing_node
  #	flexbe_testing/src/flexbe_testing/state_tester.py
* [flexbe_onboard] Use proper tempdir to avoid access right issues for multi-user setups
* Merge pull request `#82 <https://github.com/team-vigir/flexbe_behavior_engine/issues/82>`_ from grejj/fix/loglevel
  Changed loglevel from logdebug to loginfo because logdebug doesn't exist in Logger
* Changed loglevel from logdebug to loginfo because logdebug doesn't exist in Logger
* Contributors: Philipp Schillinger, grejj

1.2.2 (2019-09-16)
------------------

1.2.1 (2019-06-02)
------------------
* Merge pull request `#72 <https://github.com/team-vigir/flexbe_behavior_engine/issues/72>`_ from mgruhler/fix/filemodes
  fix filemodes: those files should not be executable
* fix filemodes: those files should not be executable
* Merge remote-tracking branch 'origin/feature/sub_parameters' into develop
* Bump required flexbe_app version
* Merge remote-tracking branch 'origin/master' into develop
* Contributors: Matthias Gruhler, Philipp Schillinger

1.1.2 (2019-04-09)
------------------
* Merge remote-tracking branch 'origin/develop'
* [flexbe_onboard] Change input parsing warning to debug level
* Merge pull request `#70 <https://github.com/team-vigir/flexbe_behavior_engine/issues/70>`_ from henroth/bugfix/fix_incorrect_warning_format
  Fix formatting error that crashes behavior construction
* In certain cases if an input key has a weird value (such as 0_degrees) it can cause an exception that prevents the behavior from being built. This is due to incorrect formatting in a warning message. This fixes the warning message formatting
* Merge remote-tracking branch 'origin/master' into develop
* Contributors: Henry Roth, Philipp Schillinger

1.1.1 (2018-12-18)
------------------
* Merge remote-tracking branch 'origin/master' into develop
* Contributors: Philipp Schillinger

1.1.0 (2018-12-01)
------------------
* Merge branch 'develop'
* Merge branch 'feature/flexbe_app' into develop
* Update maintainer information
* State logger is optional and off by default
* Merge pull request `#59 <https://github.com/team-vigir/flexbe_behavior_engine/issues/59>`_ from synapticon/feat_make_installable
  Fix issues of installed packages
* fix: Change tmp directory to "/tmp"
  Do not store temporary files inside the package directory but in "/tmp".
  This is needed since the package directory is not writeable without root
  permission when the package is installed.
* Merge branch 'develop' into feature/flexbe_app
  Conflicts:
  flexbe_mirror/src/flexbe_mirror/flexbe_mirror.py
  flexbe_onboard/src/flexbe_onboard/flexbe_onboard.py
  flexbe_widget/bin/flexbe_app
  flexbe_widget/src/flexbe_widget/behavior_action_server.py
* Merge remote-tracking branch 'origin/master' into develop
  Conflicts:
  flexbe_onboard/src/flexbe_onboard/flexbe_onboard.py
* Merge remote-tracking branch 'origin/develop'
  Conflicts:
  flexbe_onboard/src/flexbe_onboard/flexbe_onboard.py
* Merge pull request `#30 <https://github.com/team-vigir/flexbe_behavior_engine/issues/30>`_ from ckchow/feature/json_decode
  use json parser to load data, remove whitespace, javascript object style
* iterate over subdictionary to make sure sub-subdictionaries are converted
* [flexbe_onboard] Remove dependency on addict and preserve conversion of primitive types
* add javascript-style object conversion
* use json parser to load data
* [flexbe_onboard] [flexbe_widget] Removed old launch files
* Find behaviors by export tag and execute via checksum
* Merge branch 'automatic_reload' into develop
* allow onboard reloading of the current behavior
* flexbe_onboard: catch xml parsing error for manifests
* Added comment suggestion to fix checksum mismatch error
* Merge pull request `#26 <https://github.com/team-vigir/flexbe_behavior_engine/issues/26>`_ from jgdo/automatic_reload
  Automatic reload
* automatic reload of imported behaviors upon sm creation
* Merge remote-tracking branch 'origin/develop'
* [flexbe_onboard] Show info on traceback when a behavior fails
* Merge remote-tracking branch 'origin/master' into develop
* Merge remote-tracking branch 'origin/master'
* Merge remote-tracking branch 'origin/develop'
* [flexbe_onboard] Publish execution result in status args if FINISHED
* Merge remote-tracking branch 'origin/master' into develop
* Merge pull request `#10 <https://github.com/team-vigir/flexbe_behavior_engine/issues/10>`_ from team-vigir/cnurobotics
  Fix `#11 <https://github.com/team-vigir/flexbe_behavior_engine/issues/11>`_
* modify to read and allow parameterizing default behaviors_package in launch files
* Merge remote-tracking branch 'origin/develop'
* [flexbe_onboard] Skip empty parameter keys on behavior start
* Provide option to set userdata input on behavior action calls
* [flexbe_onboard] Fixed setting of namespaced behavior parameters
* Merge remote-tracking branch 'origin/feature/multirobot'
* Merge remote-tracking branch 'origin/master' into feature/multirobot
  Conflicts:
  flexbe_core/src/flexbe_core/core/monitoring_state.py
  flexbe_core/src/flexbe_core/core/operatable_state.py
* [flexbe_onboard] Handle parameter keys without namespace specification
* [flexbe_onboard] [flexbe_widget] Improved support for yaml files
* [flexbe_onboard] Removed deprecated launch file
* Changed absolute topic references to relative
* [flexbe_onboard] [flexbe_mirror] Hide default SMACH transition log spamming
* [flexbe_onboard] Removed deprecated flexbe_behaviors dependency and allow to set package name as parameter
* Removed some old and unused project files
* Initial commit of software
* Contributors: Alberto Romay, Chris Chow, David Conner, Dorian Scholz, DorianScholz, Felix Widmaier, Mark Prediger, Philipp Schillinger
