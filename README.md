# Unity Game Controllers
---------------------

This repository is a codebase for research on Agents and Robots interacting with Humans via Unity-backed games
The overall architecture is very similar to a Model-View-Controller architecture [(MVC)](https://en.wikipedia.org/wiki/Model%E2%80%93view%E2%80%93controller). 

**Models** include the Game State, Student, and Agent Models.

**Views** are implemented in C# and the Unity framework, running on the Tablet. Examples include the [Tap Game](https://github.com/mitmedialab/tap-game-unity)

**Controllers** include game logic, and are implemented in Python and closely coupled with ROS.

In addition, we also plan to support a number of **Sensors** that provide real-time interactive features to **Models**


Basics
--------------
This repository contains:

- Controllers, Models, and various Utility functions for managing different Unity games.
- ROS message definitions for communicating with the Unity implemented games via RosBridge.
- Launch scripts and experiment infrastructure for setting up an interaction, logging, cleaning, and analyzing data. 

This project is based on a best-practices template Python project which integrates several different tools. It saves you work by setting up a number of things like style-checking, unit testing, data generation, etc. with a [Make](https://en.wikipedia.org/wiki/Make_(software))-based worflow.

If you are new to Python or new to creating Python projects, see Kenneth Reitz's [Hitchhiker's Guide to Python](http://docs.python-guide.org/en/latest/) for an explanation of some of the patterns/tools used here.

Project Setup
---------------
## Tools

We recommend using [PyCharm](https://www.jetbrains.com/pycharm/download/#section=linux) as your primary editor for large Python projects like this one. Sublime Text is OK for quick edits, but PyCharm gives you the full power of static analysis, environment management, style checking, and an integrated terminal etc. in an attractive all-in-one package.

## Installation

This project was built and tested on Ubuntu 16.04, with a full desktop installation of ROS Indigo

We recommend using Python 2.7 as your default system-wide Python environment (for ROS) and creating a virtual environment with Python 3.6 virtual env to execute the controller code

**All Python code should be written in Python 3!**

### Create and source virtualenv

If you haven't done this yet, set up and source a python3 virtual env with:
`mkdir -p ~/python-virtualenvs`
`virtualenv -p $(which python3) ~/python-virtualenvs/unity-game-controllers --system-site-packages`

Then, activate it with
`source ~/python-virtualenvs/unity-game-controllers/bin/activate`

### Install the project's non-python development and runtime requirements::	
	
	UBUNTU 16.04 System dependencies
	$ sudo apt-get install portaudio19-dev
	$ sudo apt-get install xdotool
	$ sudo apt-get install wmctrl
	$ sudo apt-get install ros-indigo-rosbridge-server
	$ sudo apt-get install python3-tk
	
	# frequently needed for ROS-related code to work)
	$ pip install rospkg
	$ pip install catkin_pkg
	$ pip install pymongo
	$ pip install twisted
	
	# External catkin repos necessary for message passing and other functions
	# Clone these to ~/catkin_ws/src, then run "catkin_make" from ~/catkin_ws
	$ https://github.com/bosch-ros-pkg/usb_cam.git
	$ https://github.com/mitmedialab/r1d1_msgs.git (to communicate w Tega)
	$ https://github.com/mitmedialab/jibo_msgs.git (to communicate w Jibo)
	$ https://github.com/mitmedialab/affdex_ros_msgs.git (to get realtime Affdex facial info)
	$ https://github.com/mitmedialab/asr_google_cloud.git (to connect to google ASR for iSpy)

### Install the project's python development and runtime requirements::

    $ make init
	
**Project setup is now complete!**


## Running a Unity Game Controller
---------------


0. Activate the virtualenv
```bashrc
source ~/python-virtualenvs/unity-game-controllers/bin/activate
```
1. Start `roscore` and `rosbridge` *# do not switch to a different terminal window until done loading*
```bashrc
$ ./scripts/startROS.sh
```

2. Start a game controller. E.g., you can run the TapGame Controller by running the following script:

`$ python -m scripts.start_tap_game_controller [participant_id] [experimenter] [experiment_phase]`

e.g.

`$ python -m scripts.start_tap_game_controller p01 sam experiment`

You can test the tap game in `practice` or `posttest` mode, by passing those arguments in as the `experiment_phase`

Troubleshooting:
------------------
- **Problem**: `"cannot find module 'clev"`
	- **Solution**: Navigate to the install site of the `weighted-levenshtein` package, edit __init__.py, and remove
- **Problem**: `"cannot find module 'em'"`
	- **Solution**: `pip install empy`
