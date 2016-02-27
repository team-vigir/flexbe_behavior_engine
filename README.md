# FlexBE Behavior Engine
FlexBE is a high-level behavior engine coordinating the capabilities of a robot in order to solve complex tasks.
Behaviors are modeled as hierarchical state machines where states correspond to active actions and transitions describe the reaction to outcomes.
Main advantage over similar approaches is the good operator integration and extensive user interface.
Besides executing behaviors in full autonomy, the operator can restrict execution of certain transitions or trigger them manually.
Furthermore, it is even supported to modify the whole structure of a behavior during its execution without restarting it.
The user interface features a runtime control interface as well as a graphical editor for state machines.

Please refer to the FlexBE Homepage ([flexbe.github.io](http://flexbe.github.io)) for further information, tutorials, application examples, and much more.

## Installation

Execute the following commands to install FlexBE:

    roscd && cd ../src
    git clone https://github.com/team-vigir/flexbe_behavior_engine.git
    
Furthermore, create your own repository for behavior development (contains examples):
    
    rosrun flexbe_widget create_repo [your_project_name]
    
Finally, install the user interface (Google Chrome App) by following [these steps](http://philserver.bplaced.net/fbe/download.php).

## Usage

Default launch file for running the onboard engine:

    roslaunch flexbe_onboard behavior_onboard.launch
    
Default launch file for running the operator control station (OCS):

    roslaunch flexbe_widget behavior_ocs.launch
    
You can also create a program shortcut for the FlexBE Chrome App. When just developing behaviors and not executing them, it is not necessary to have ROS running.

## Next Steps

- Do some of the [tutorials](http://philserver.bplaced.net/fbe/documentation.php).
- Visit the [FlexBE GitHub Organization](https://github.com/FlexBE) for additional available states.

## Maintainer

- Philipp Schillinger ([@pschillinger](https://github.com/pschillinger), schillinger@sim.tu-darmstadt.de, [Contact](http://philserver.bplaced.net/fbe/contact.php))
