# dvs_integrator

Event-by-event processing in ROS C++ for the purpose of image reconstruction (using a leaky integrator). Based on the paper by Scheerlinck et al., ACCV 2018 (see bottom of page).

The package reads events from a topic and outputs the reconstructed images using per-pixel temporal filters (leaky integrators).

## Slides and videos
- [Slides](https://drive.google.com/file/d/1MtbzVMKebJq2I0FaG6CtmH0gEomAJ19h/view)
- [Direct integrator](https://youtu.be/BNwFeG4CSE4)
- [Leaky integrator](https://youtu.be/4u-I9LNW4gw)

## Input / Output
**Input**:
- Events (topic)

**Output**:
- State (for illustration purposes): time surface
- Reconstructed intensity. It is almost the brightness state (just propagated--decayed-- to the current time).

**Parameters** (dynamic reconfigure):
- Decaying parameter ("cutoff frequency" alpha) of the leaky integrator.

## Dependencies

### Create a catkin workspace

Create a catkin workspace (if there is none yet). For example, from your home folder:

	cd
	mkdir -p catkin_ws/src
	cd catkin_ws
	catkin config --init --mkdirs --extend /opt/ros/melodic --merge-devel --cmake-args -DCMAKE_BUILD_TYPE=Release

Depending on the ROS distribution you installed, you might have to use `kinetic` instead of `melodic` in the previous command.

### Add packages to the catkin workspace

Clone this repository into the `src` folder of your catkin workspace.

The catkin package dependencies are:
- [catkin simple](https://github.com/catkin/catkin_simple)
- ROS messages for DVS ([rpg_dvs_ros](https://github.com/uzh-rpg/rpg_dvs_ros))

The above dependencies are specified in the [dependencies.yaml](dependencies.yaml) file. They can be installed with the following commands from the `src` folder of your catkin workspace:

	cd catkin_ws/src
	sudo apt-get install python3-vcstool
	vcs-import < dvs_integrator/dependencies.yaml

The previous command should clone the repositories into folders *catkin_simple* and *rpg_dvs_ros* inside the src/ folder of your catkin workspace, at the same level as this repository *dvs_integrator*. They should NOT be inside the *dvs_integrator* folder.


## Compile

**Preliminaries**:
First, build `catkin simple` and the `davis_ros_driver`. The most important commands are:

	catkin build catkin_simple
	catkin build davis_ros_driver

Then, incorporate the packages to the path, so that ROS finds them:

	source ~/catkin_ws/devel/setup.bash

**Compile this package**:

	catkin build dvs_integrator --force-cmake

The flag `--force-cmake` is optional.
After building, at least the first time, remember to run:

	source ~/catkin_ws/devel/setup.bash

Sometimes (in case of strange errors) it is useful to delete the build folder before compilation:

	rm -rf build/dvs_integrator/

An alternative command to start from scratch (cleaning all catkin packages) is (to be used with *caution*): `catkin clean`


## Run example
Download a ROS bag dataset, for example [slider_depth](http://rpg.ifi.uzh.ch/datasets/davis/slider_depth.bag) to play it in a terminal, that is, to use it as source of data (as if an event camera was connected to the computer).

Every time that you open a terminal, you may need to run:

	source ~/catkin_ws/devel/setup.bash

to have access to the ROS commands.

Run in a terminal (you may need to "source" first):

	roscore

Open the rqt perspective view with four visualizers. This file is in the launch folder. Run the following command in a terminal, from the `dvs_integrator` package folder:

	rqt --perspective-file launch/integrator_view.perspective

(Optional) Run the visualizer node in another terminal:

	roslaunch dvs_displayer display_monocular.launch

Run the image reconstruction node in another terminal:

	roslaunch dvs_integrator integrator.launch

In another terminal play the bag with the event data:

	rosbag play -r 0.1 --pause path_to_file/slider_depth.bag

Press the space bar to  play / pause the reproduction of the bag. Event messages are accumulated in the subscriber of dvs_integrator, to avoid losing events for the reconstruction.

In another terminal open the dynamic reconfigure and play around with the parameter(s) in the window named `dvs_integrator_one`

	rosrun rqt_reconfigure rqt_reconfigure

Using the same roscore during execution of multiple runs of the dvs_integrator allows the dynamic parameters to persist even if the nodes are killed. The next time they are started, they configuration parameters will be set using the ones from the previous session.

End the program execution with `Ctrl + C` keyboard shortcut.


## (Optional) Possible extensions:
- Interaction (using dynamic reconfigure). Add a parameter in the dynamic reconfigure GUI to change the contrast sensitivity online.
- Currently we are displaying the events as they are arranged in the ROS messages. Add a parameter that allows to control the rate at which the reconstructed image is published. Use, for example a fixed number of events or a fixed time interval.


## Reference
- Scheerlinck et al., *[Continuous-time Intensity Estimation Using Event Cameras](https://www.cedricscheerlinck.com/continuous-time-intensity-estimation)*, Asian Conf. Computer Vision (ACCV), 2018.
