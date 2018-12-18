^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package flexbe_onboard
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
