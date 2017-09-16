Unity Game Controllers
=========================



This repository contains the Controllers, Models, and various Utilities for Unity-backed game environments.

The overall architecture is a Model-View-Controller architecture [(MVC)](https://en.wikipedia.org/wiki/Model%E2%80%93view%E2%80%93controller).

The **Models** include the Game State, implemented in Python and closely coupled with ROS. Models of a Student and Agent can also be found here.

The **View** is implemented in C# and the Unity framework, running on the Tablet.

The **Controller** includes agent decision-making algorithm and game logic, implemented in Python and closely coupled with ROS.

In addition, we have a number of **Sensors** that provide real-time interactive features to **Models**

NOTES
--------------

This project is based on a best-practices template Python project which integrates several different tools. It saves you work by setting up a number of things like style-checking and unit testing.

If you are new to Python or new to creating Python projects, see Kenneth Reitz's `Hitchhiker's Guide to Python`_ for an explanation of some of the tools used here.

- [Hitchhiker's Guide to Python](http://docs.python-guide.org/en/latest/)

## Project Setup

### System Dependencies

This project was built and tested on Ubuntu 14.04, with a full desktop installation of ROS Indigo

We recommend using Anaconda with Python 2.7 as your default system-wide Python environment (for ROS) and creating an Anaconda with Python 3.6 virtual env to execute the controller code [see here for details](https://uoa-eresearch.github.io/eresearch-cookbook/recipe/2014/11/20/conda/).

**All Python code should be Python 3 compatible.**

### Install the project's non-python development and runtime requirements::

	OS X
	`brew install portaudio`
	
	UBUNTU
	`sudo apt-get install portaudio19-dev`
	`pip install rospkg`
	`pip install catkin_pkg`
	`pip install pymongo`
	`pip install twisted`

### Install the project's python development and runtime requirements::

        pip install -r requirements-dev.txt

**Project setup is now complete!**
