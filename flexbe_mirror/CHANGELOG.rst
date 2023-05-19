^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package flexbe_mirror
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
1.4.0 (2023-05-18)
------------------
* Updates for Melodic and Noetic releases on github.com/FlexBE

1.3.1 (2020-12-11)
------------------

1.3.0 (2020-11-19)
------------------
* Merge remote-tracking branch 'origin/feature/core_rework' into develop
  # Conflicts:
  #	flexbe_core/src/flexbe_core/core/operatable_state_machine.py
  #	flexbe_onboard/src/flexbe_onboard/flexbe_onboard.py
* Add support for python3
* [flexbe_mirror] Minor cleanup of mirror
* Major clean-up of most core components
* Remove smach dependency
* Contributors: Philipp Schillinger

1.2.5 (2020-06-14)
------------------
* Merge branch 'develop' into feature/state_logger_rework
* Contributors: Philipp Schillinger

1.2.4 (2020-03-25)
------------------
* Merge branch 'fmessmer-feature/python3_compatibility' into develop
* python3 compatibility via 2to3
* Contributors: Philipp Schillinger, fmessmer

1.2.3 (2020-01-10)
------------------
* Revise internal dependencies
* Merge remote-tracking branch 'origin/develop' into feature/test_behaviors
  # Conflicts:
  #	flexbe_testing/bin/testing_node
  #	flexbe_testing/src/flexbe_testing/state_tester.py
* [flexbe_mirror] Fix mirror sync lock (see `FlexBE/flexbe_app#47 <https://github.com/FlexBE/flexbe_app/issues/47>`_)
* Contributors: Philipp Schillinger

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
* Merge remote-tracking branch 'origin/master' into develop
* Contributors: Philipp Schillinger

1.1.1 (2018-12-18)
------------------
* Merge remote-tracking branch 'origin/develop'
* Merge remote-tracking branch 'origin/master' into develop
* [flexbe_mirror] Fix race condition in mirror restarts
* Contributors: Philipp Schillinger

1.1.0 (2018-12-01)
------------------
* Merge branch 'develop'
* Merge branch 'feature/flexbe_app' into develop
* Update maintainer information
* Merge branch 'develop' into feature/flexbe_app
  Conflicts:
  flexbe_mirror/src/flexbe_mirror/flexbe_mirror.py
  flexbe_onboard/src/flexbe_onboard/flexbe_onboard.py
  flexbe_widget/bin/flexbe_app
  flexbe_widget/src/flexbe_widget/behavior_action_server.py
* Merge remote-tracking branch 'origin/develop'
  Conflicts:
  flexbe_onboard/src/flexbe_onboard/flexbe_onboard.py
* Find behaviors by export tag and execute via checksum
* Merge branch 'automatic_reload' into develop
* flexbe mirror: small fix of mission member variable initialization
* Merge pull request `#27 <https://github.com/team-vigir/flexbe_behavior_engine/issues/27>`_ from jgdo/automatic_reload
  fix of behavior_mirror: both switch and requesting the newest sm structure works now
* fix of behavior_mirror: both switch and requesting the newest sm structure works now
* Merge pull request `#26 <https://github.com/team-vigir/flexbe_behavior_engine/issues/26>`_ from jgdo/automatic_reload
  Automatic reload
* removed auto-starting behavior after structure callback
* Merge remote-tracking branch 'origin/master' into develop
* Merge pull request `#10 <https://github.com/team-vigir/flexbe_behavior_engine/issues/10>`_ from team-vigir/cnurobotics
  Fix `#11 <https://github.com/team-vigir/flexbe_behavior_engine/issues/11>`_
* fix some shutdown issues on ctrl-c
* Merge branch 'master' into cnurobotics
* Merge remote-tracking branch 'origin/develop'
* [flexbe_mirror] Skip synchronization if mirror gets preempted
* Merge remote-tracking branch 'origin/develop'
* [flexbe_core] [flexbe_mirror] Improved robustness of fast repeated synchronization
* [flexbe_mirror] Fixed mirror rate sleep to reduce CPU load
* Merge remote-tracking branch 'origin/feature/multirobot'
* Merge remote-tracking branch 'origin/master' into feature/multirobot
  Conflicts:
  flexbe_core/src/flexbe_core/core/monitoring_state.py
  flexbe_core/src/flexbe_core/core/operatable_state.py
* Changed absolute topic references to relative
* [flexbe_onboard] [flexbe_mirror] Hide default SMACH transition log spamming
* Removed some old and unused project files
* Initial commit of software
* Contributors: David C. Conner, David Conner, Dorian Scholz, DorianScholz, Mark Prediger, Philipp Schillinger
