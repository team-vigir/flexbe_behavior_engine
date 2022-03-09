# FlexBE Behavior Engine

> NOTE:  As of 9-March-2022 development of the ROS 2 version of FlexBE
> will be maintained at https://github.com/FlexBE/flexbe_behavior_engine.
>
> It is suggested that you migrate to there for new ROS 1 work as well.
>
> This site is being maintained for historical reasons, but future work
> should use the https://github.com/FlexBE site, which will be the one-stop
> shop for all things FlexBE going forward.



FlexBE is a high-level behavior engine coordinating the capabilities of a robot in order to solve complex tasks.
Behaviors are modeled as hierarchical state machines where states correspond to active actions and transitions describe the reaction to outcomes.
Main advantage over similar approaches is the good operator integration and extensive user interface.
Besides executing behaviors in full autonomy, the operator can restrict execution of certain transitions or trigger them manually.
Furthermore, it is even supported to modify the whole structure of a behavior during its execution without restarting it.
The user interface features a runtime control interface as well as a graphical editor for state machines.

Please refer to the FlexBE Homepage ([flexbe.github.io](http://flexbe.github.io)) for further information, tutorials, application examples, and much more.

![FlexBE CI](https://github.com/team-vigir/flexbe_behavior_engine/workflows/FlexBE%20CI/badge.svg)

## Installation

Execute the following commands to install FlexBE:

    roscd && cd ../src
    git clone https://github.com/team-vigir/flexbe_behavior_engine.git

Furthermore, create your own repository for behavior development (contains examples):

    rosrun flexbe_widget create_repo [your_project_name]

Finally, it is recommended to install the FlexBE App user interface by following [these steps](http://philserver.bplaced.net/fbe/download.php).

## Usage

Use the following launch file for running the onboard engine:

    roslaunch flexbe_onboard behavior_onboard.launch

Use the following launch file for running the operator control station (requires the FlexBE App):

    roslaunch flexbe_app flexbe_ocs.launch

Use the following lunach file to run both of the above, e.g., for testing on a single computer:

    roslaunch flexbe_app flexbe_full.launch

## Next Steps

- Do some of the [tutorials](http://philserver.bplaced.net/fbe/documentation.php).
- Visit the [FlexBE GitHub Organization](https://github.com/FlexBE) for additional available states.

## Publications

Please use the following publication for reference when using FlexBE:

Philipp Schillinger, Stefan Kohlbrecher, and Oskar von Stryk, ["Human-Robot Collaborative High-Level Control with Application to Rescue Robotics"](http://dx.doi.org/10.1109/ICRA.2016.7487442), IEEE International Conference on Robotics and Automation (ICRA), Stockholm, Sweden, May 2016.

    @INPROCEEDINGS{2016:ICRA_Schillinger-etal,
        author = {Philipp Schillinger and Stefan Kohlbrecher and Oskar von Stryk},
        title = {Human-Robot Collaborative High-Level Control with Application to Rescue Robotics},
        year = {2016},
        pages = {2796-2802},
        booktitle = {Proc. IEEE Int. Conf. on Robotics and Automation (ICRA)},
    }

### Further Publications

Refer for example to the following publications to get an impression about ways to use FlexBE.
Let us know if you know a paper which should be added to the list.

- Stefan Kohlbrecher et al. ["A Comprehensive Software Framework for Complex Locomotion and Manipulation Tasks Applicable to Different Types of Humanoid Robots."](http://dx.doi.org/10.3389/frobt.2016.00031) Frontiers in Robotics and AI 3 (2016): 31.

- Spyros Maniatopoulos, Philipp Schillinger, Vitchyr Pong, David C. Conner, and Hadas Kress-Gazit, ["Reactive High-level Behavior Synthesis for an Atlas Humanoid Robot"](http://dx.doi.org/10.1109/ICRA.2016.7487613), IEEE International Conference on Robotics and Automation (ICRA), Stockholm, Sweden, May 2016.

## Maintainer

- Philipp Schillinger ([@pschillinger](https://github.com/pschillinger), [Contact](http://philserver.bplaced.net/fbe/contact.php))
