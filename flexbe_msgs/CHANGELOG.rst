^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package flexbe_msgs
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
1.4.0 (2023-05-18)
------------------
* Updates for Melodic and Noetic releases on github.com/FlexBE
* [flexbe_msgs] Merge pull request #165 from HannesBachter/feature/get_userdata - get userdata by service

1.3.1 (2020-12-11)
------------------

1.3.0 (2020-11-19)
------------------
* Merge remote-tracking branch 'origin/feature/core_rework' into develop
  # Conflicts:
  #	flexbe_core/src/flexbe_core/core/operatable_state_machine.py
  #	flexbe_onboard/src/flexbe_onboard/flexbe_onboard.py
* Remove smach dependency
* Contributors: Philipp Schillinger

1.2.5 (2020-06-14)
------------------
* Merge branch 'develop' into feature/state_logger_rework
* [flexbe_msgs] Remove deprecated constants from input action
* Contributors: Philipp Schillinger

1.2.4 (2020-03-25)
------------------
* [flexbe_msgs] [flexbe_core] Add debug level to logger
  (see `#101 <https://github.com/team-vigir/flexbe_behavior_engine/issues/101>`_)
* Contributors: Philipp Schillinger

1.2.3 (2020-01-10)
------------------
* Merge remote-tracking branch 'origin/develop' into feature/test_behaviors
  # Conflicts:
  #	flexbe_testing/bin/testing_node
  #	flexbe_testing/src/flexbe_testing/state_tester.py
* Contributors: Philipp Schillinger

1.2.2 (2019-09-16)
------------------

1.2.1 (2019-06-02)
------------------
* Merge remote-tracking branch 'origin/feature/sub_parameters' into develop
* Bump required flexbe_app version
* Merge remote-tracking branch 'origin/master' into develop
* Contributors: Philipp Schillinger

1.1.2 (2019-04-09)
------------------
* Merge remote-tracking branch 'origin/master' into develop
* Contributors: Philipp Schillinger

1.1.1 (2018-12-18)
------------------
* Merge remote-tracking branch 'origin/master' into develop
* Contributors: Philipp Schillinger

1.1.0 (2018-12-01)
------------------
* Merge branch 'develop'
* Merge branch 'feature/flexbe_app' into develop
* Update maintainer information
* Merge remote-tracking branch 'origin/develop'
* Merge remote-tracking branch 'origin/develop' into feature/flexbe_app
* Merge pull request `#52 <https://github.com/team-vigir/flexbe_behavior_engine/issues/52>`_ from ruvu/fix/catkin_lint_errors_and_warnings
  chore: ran catkin_lint and fixed warnings and errors
* chore: ran catkin_lint and fixed warnings and errors
* Find behaviors by export tag and execute via checksum
* Merge remote-tracking branch 'origin/develop'
* [flexbe_msgs] Increase field size of behavior modification index
* Provide option to set userdata input on behavior action calls
* Merge remote-tracking branch 'origin/master' into feature/multirobot
  Conflicts:
  flexbe_core/src/flexbe_core/core/monitoring_state.py
  flexbe_core/src/flexbe_core/core/operatable_state.py
* [flexbe_msgs] Added priority container to state class options
* [flexbe_msgs] Changed autonomy encoding in StateInstantation to prevent Python issues
* [flexbe_msgs] Extended behavior synthesis interface
  * Added support for concurrency container
  * Can now set input and output keys for containers, including root
  * Can now specify positions of states in the editor for improved visualization
* [flexbe_msgs] Added new message type for UI commands from ROS
* [flexbe_msgs] Added default synthesis message types
* [flexbe_msgs] Added action message for behavior execution
* Removed some old and unused project files
* Initial commit of software
* Contributors: Philipp Schillinger, Rein Appeldoorn
