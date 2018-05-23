## iSpyGameController Overview

### 1. Main Scripts
Contains scripts that are essential for running iSpy game.

1. `iSpyGameFSM.py`
	* it connects iSpy game, ROS, different sensors (vision, speech), interaction FSM  all together
	* It handles ROS messages sent from iSpy and other sensors back to the controller
2. `GameModeFSMs.py`
	* It contains different game modes (always-mission mode, always-explore mode, complete mode)
	* Each mode has its own FSM
	* Always-mission mode: doesn't allow the user to switch back to the explore mode when the user has entered the mission mode. this mode is primarily used for the role switching project
	* The classes in this script are instantiated by `iSpyGameFSM.py`
3. `iSpyTaskController.py`
	* This class contains iSpy task-related functions
4. `iSpyDataTracking.py`
	* Outputs csv file corresponding to data recorded 
	* Accounts for data from child-robot interaction and ispy-game
5. `ROSNodeMgr.py`
	* Manages ROS Node Communication
	* Handles both robot and ispy command 
	* Establishes publisher and listener for ispy-game transition, tega state, asr, child-robot interaction 
	
### 2. `RobotBehaviorList`
Contains scripts for all robot-related behaviors and actions.

1. `RobotBehaviorList.py`
	* Contains robot's behaviors and actions for iSpy game
	* Contains different robot's roles (competent, novice, expert)
	* Connects the iSpyGameController with the physical robot(s)
	* Currently instantiated by `ChildRobotInteractionFSM.py`. 
2. `JiboBehaviors.py`
	* Jibo's behaviors
	* Called by `RobotBehaviorList.py`
3. `TegaBehaviors.py`
	* Tega's behaviors
	* Called by `RobotBehaviorList.py`
	
### 3. `RoleSwitchingPrj`
This folder contains python scripts that are primarily used for the role switching project.

1. `ChildRobotInteractionFSM.py`
	* It is a interaction FSM currently used for the role switching project
	* Child and robot take turns spying objects
	* It communicates with `iSpyGameFSM`, reinforcement learning agent models, robot's behaviors (`RobotBehaviorList.py`) and `ChildStates.py`
2. `ChildStates.py`
	* It informs the RL model of the child's states
	* Two parts: learning states and affective states
3. `AgentModel.py`
	* Functions for using RL model
4. Robot Role Switching Policy Model
	* Currently, a reinforcemnet learning model is used. 
	* Relevant scripts: `RLAgent.py`,`RLiSpyEnvironment.py`
	
### 4. `AffdexAnalysis`
Contains python scripts that are used for Affdex analysis 

1. `node_AffdexResponse.py`
	* Outputs affdex data log files that includes emotions, expressions and measurements
	
### 5. `res`
Contains json files corresponding to Q&A pair, action mapping and participant assignment

### 6. `saved_rl`
Contains pkl files of child states, regressor model, episode and action history for all participants
