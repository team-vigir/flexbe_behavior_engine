^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package flexbe_core
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
* Merge pull request `#72 <https://github.com/team-vigir/flexbe_behavior_engine/issues/72>`_ from mgruhler/fix/filemodes
  fix filemodes: those files should not be executable
* fix filemodes: those files should not be executable
* Merge remote-tracking branch 'origin/feature/sub_parameters' into develop
* Bump required flexbe_app version
* [flexbe_core] Accept explicit definition of parameters for included behaviors
* Merge remote-tracking branch 'origin/master' into develop
* Contributors: Matthias Gruhler, Philipp Schillinger

1.1.2 (2019-04-09)
------------------
* Merge remote-tracking branch 'origin/develop'
* Merge pull request `#66 <https://github.com/team-vigir/flexbe_behavior_engine/issues/66>`_ from ksm0709/add_remove_result
  add remove_result function to proxy action client
* add remove_result function to proxy action client
* Merge remote-tracking branch 'origin/master' into develop
* Contributors: Philipp Schillinger, taehokang

1.1.1 (2018-12-18)
------------------
* Merge remote-tracking branch 'origin/develop'
* Revise run dependencies
* Merge remote-tracking branch 'origin/master' into develop
* Contributors: Philipp Schillinger

1.1.0 (2018-12-01)
------------------
* Merge branch 'develop'
* Merge branch 'feature/flexbe_app' into develop
* Update maintainer information
* State logger is optional and off by default
* Merge remote-tracking branch 'origin/action_client_remove_feedback' into feature/flexbe_app
* Merge pull request `#62 <https://github.com/team-vigir/flexbe_behavior_engine/issues/62>`_ from team-vigir/action_client_remove_feedback
  Added remove_feedback function to ensure new feedback is received
* Added remove_feedback function to ensure new feedback is received
* Merge pull request `#58 <https://github.com/team-vigir/flexbe_behavior_engine/issues/58>`_ from alireza-hosseini/feat-action-client-wait-param
  feat: Add `wait_duration` parameter to `ProxyActionClient`
* feat: Add wait_duration parameter to ProxyActionClient
  - So that the wait duration can be specified in the states definition
* [flexbe_core] Allow to use behavior default userdata (see `#38 <https://github.com/team-vigir/flexbe_behavior_engine/issues/38>`_)
* [flexbe_core] Update behavior lib if behavior is not found (see `Flexbe/flexbe_app#4 <https://github.com/Flexbe/flexbe_app/issues/4>`_)
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
* Merge pull request `#31 <https://github.com/team-vigir/flexbe_behavior_engine/issues/31>`_ from fmauch/reset_entering
  reset entering of currently active state when exiting a state machine
* reset entering member of currently active state when exiting a state machine
* Find behaviors by export tag and execute via checksum
* Merge branch 'automatic_reload' into develop
* allow locking and unlocking of current state without knowing the current path
* remove manual reloading code, as this is done already by the reload importer
* Merge pull request `#26 <https://github.com/team-vigir/flexbe_behavior_engine/issues/26>`_ from jgdo/automatic_reload
  Automatic reload
* automatic reload of imported behaviors upon sm creation
* Reload class definition before instantiating a contained behavior inside a behavior
* Merge remote-tracking branch 'origin/master' into develop
* [flexbe_core] Fixed event triggering in concurrency container (resolve `#18 <https://github.com/team-vigir/flexbe_behavior_engine/issues/18>`_)
* Merge remote-tracking branch 'origin/master'
* Merge remote-tracking branch 'origin/develop'
* [flexbe_core] Only call on_exit on cc leave for states which are still looping (fix `#17 <https://github.com/team-vigir/flexbe_behavior_engine/issues/17>`_)
* Merge branch 'develop'
* [flexbe_core] Fixed sm on_exit to propagate own ud instead of parent ud
* Merge branch 'master' into cnurobotics
* Merge remote-tracking branch 'origin/develop'
* [flexbe_core] Properly reset current state when leaving state machine (fix `#7 <https://github.com/team-vigir/flexbe_behavior_engine/issues/7>`_)
* Merge remote-tracking branch 'origin/develop'
* [flexbe_core] Fixed reset of current state on leave in cc and related concurrency userdata problems
* [flexbe_core] Use aggregated diagnostics topic instead of raw
* [flexbe_core] Correctly execute concurrency inside priority container
* [flexbe_core] Can always preempt behavior even if not supervised
* Merge remote-tracking branch 'origin/develop'
* Provide option to set userdata input on behavior action calls
* [flexbe_core] Fixed occasional problems to resume a paused state
* [flexbe_core] [flexbe_mirror] Improved robustness of fast repeated synchronization
* Merge branch 'feature/late_connect' into develop
* [flexbe_core] Added command to attach to running behavior execution
* Merge branch 'feature/pause_repeat' into develop
* [flexbe_core] Handle pause and repeat commands
* [flexbe_core] Propagate skipped notification on pause in order to react on preemption commands even if paused
* [flexbe_core] Added function to check if a goal is already active on a proxy client
* [flexbe_core] Fix for backup sync
* [flexbe_core] Fixed sync issues after leaving CC by explicitly syncing automatically
* [flexbe_core] Fixed calling on_exit on all states in CC
* Merge remote-tracking branch 'origin/feature/multirobot'
* Merge remote-tracking branch 'origin/master' into feature/multirobot
  Conflicts:
  flexbe_core/src/flexbe_core/core/monitoring_state.py
  flexbe_core/src/flexbe_core/core/operatable_state.py
* [flexbe_core] Added availability checks to proxies
* [flexbe_core] Added onboard debug topic for current state
* [flexbe_core] Convert all logged messages to string before sending ros message in logger
* [flexbe_core] Added priority container
* [flexbe_core] Added some more documentation
* [flexbe_core] Fixed initialization of input userdata in inner statemachines
* [flexbe_core] Correctly preempt auxilliary control flows in concurrency container
* [flexbe_core] Fixed a bug with concurrent execution:
  State machines inside state machine inside concurrency containers still blocked during execution.
* [flexbe_core] Slightly reworked monitoring state
* [flexbe_core] Fixed preemption of concurrency container
* [flexbe_core] Added container for concurrent execution
* Changed absolute topic references to relative
* [flexbe_core] Improved proxy interface
* [flexbe_core] Reverted last change, will only publish state updates when being controlled
* [flexbe_core] Always send outcome update, even if not being controlled
* Removed some old and unused project files
* Initial commit of software
* Contributors: Alberto, Alireza, David Conner, Dorian Scholz, DorianScholz, Felix Mauch, Mark Prediger, Philipp Schillinger
