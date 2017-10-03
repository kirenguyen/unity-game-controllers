# Unity Game Controllers
---------------------

This repository contains the Controllers, Models, and various Utilities for Unity-backed game environments.

The overall architecture is a Model-View-Controller architecture [(MVC)](https://en.wikipedia.org/wiki/Model%E2%80%93view%E2%80%93controller).

The **Models** include the Game State, implemented in Python and closely coupled with ROS. Models of a Student and Agent can also be found here.

The **View** is implemented in C# and the Unity framework, running on the Tablet.

The **Controller** includes agent decision-making algorithm and game logic, implemented in Python and closely coupled with ROS.

In addition, we have a number of **Sensors** that provide real-time interactive features to **Models**



Basics
--------------

This project is based on a best-practices template Python project which integrates several different tools. It saves you work by setting up a number of things like style-checking, unit testing, data generation, etc. with a [Make](https://en.wikipedia.org/wiki/Make_(software))-based worflow.

If you are new to Python or new to creating Python projects, see Kenneth Reitz's [Hitchhiker's Guide to Python](http://docs.python-guide.org/en/latest/) for an explanation of some of the patterns/tools used here.

Project Setup
---------------

### System Dependencies

This project was built and tested on Ubuntu 14.04, with a full desktop installation of ROS Indigo

We recommend using Anaconda with Python 2.7 as your default system-wide Python environment (for ROS) and creating an Anaconda with Python 3.6 virtual env to execute the controller code [see here for details](https://uoa-eresearch.github.io/eresearch-cookbook/recipe/2014/11/20/conda/).

**All Python code should be Python 3 compatible.**

### Install the project's non-python development and runtime requirements::	
	
	UBUNTU 14.04 System dependencies
	$ sudo apt-get install portaudio19-dev
	$ sudo apt-get install xdotool
	$ sudo apt-get install wmctrl
	$ sudo apt-get install ros-indigo-rosbridge-server
	
	# frequently needed for ROS-related code to work)
	$ pip install rospkg
	$ pip install catkin_pkg
	$ pip install pymongo
	$ pip install twisted
	
	# External catkin repos necessary for message passing and other functions
	# Clone these to ~/catkin_ws/src, then run "catkin_make" from ~/catkin_ws
	$ https://github.com/bosch-ros-pkg/usb_cam.git
	$ https://github.com/mitmedialab/unity_game_msgs.git
	$ https://github.com/mitmedialab/r1d1_msgs.git (to communicate w Tega)
	$ https://github.com/mitmedialab/jibo_msgs.git (to communicate w Jibo)


### Install the project's python development and runtime requirements::

    $ make init
	
**Project setup is now complete!**


## Running a Unity Game Controller
---------------

To run a Unity Game Controller, there are three major components of the back-end system

- A ROSCore. Manages all communication between nodes in the network. 
	- Create one in a new terminal by running `$ roscore`; requires successful install of ROS)
- ROSBridge Webserver: Handles communication between tablet and ROSCore
	- Create one in a new terminal by running `$ roslaunch rosbridge_server rosbridge_websocket.launch`; Requires install of Rosbridge (not included with regular ROS Desktop install), above.
- *GameController (any

### Tap Game
-------------

After installing the dependencies, you can launch the Rosbridge Webserver and ROSCore together by running `$ ./scripts/startROS.sh`

Then, you can run the TapGame Controller by running the following script in a  Python 3 environment:

`$ python -m scripts.start_tap_game_controller [participant_id] [experimenter] [experiment_phase]`

e.g.

`$ python -m scripts.start_tap_game_controller p01 sam experiment`

You can test the tap game in `practice` or `posttest` mode, by passing those arguments in as the `experiment_phase`

### ISpy Game
-------------

After installing the dependencies, you can launch the Rosbridge Webserver and ROSCore together by running `$ ./scripts/startROS.sh`

Then, you can run the iSpyGame Controller by running the following script in a  Python 3 environment:

`$ python -m scripts.start_ispy_game_controller`





Troubleshooting:
------------------
- **Problem**: `"cannot find module 'clev"`
	- **Solution**: Navigate to the install site of the `weighted-levenshtein` package, edit __init__.py, and remove
- **Problem**: `"cannot find module 'em'"`
	- **Solution**: `pip install empy`
