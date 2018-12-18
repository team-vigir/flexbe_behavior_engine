^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package flexbe_mirror
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
