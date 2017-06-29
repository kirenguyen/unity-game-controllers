Tap Game Controller
=========================

The Tap Game is an integrated educational game environment for agents.

The overall architecture is a Model-View-Controller architecture [(MVC)](https://en.wikipedia.org/wiki/Model%E2%80%93view%E2%80%93controller).

The **Model** is the Game State, implemented in Python and closely coupled with ROS. It includes both a student model and a model of the current game state.

The **View** is implemented in C# and the Unity framework, running on the Tablet.

The **Controller** is the agent algorithm and the game logic, implemented in Python and closely coupled with ROS.

In addition, we have a number of **Sensors** that provide real-time interactive features to the **Model**

NOTES
--------------

This project is based on a best-practices template Python project which integrates several different tools. It saves you work by setting up a number of things, including documentation, code checking, and unit testing.

* pytest_ for unit testing

If you are new to Python or new to creating Python projects, see Kenneth Reitz's `Hitchhiker's Guide to Python`_ for an explanation of some of the tools used here.

- [Sphinx](http://sphinx-doc.org/)
- [flake8](https://pypi.python.org/pypi/flake8)
- [pytest](http://pytest.org/latest/)
- [Hitchhiker's Guide to Python](http://docs.python-guide.org/en/latest/)

##Project Setup

###. Install the project's non-python development and runtime requirements::

	brew install portaudio

###. Install the project's python development and runtime requirements::

        pip install -r requirements-dev.txt

**Project setup is now complete!**
