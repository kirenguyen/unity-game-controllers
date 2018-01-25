# ROS Message (child robot interaction)


# current game task index: 
int8 gameTask 


# vocab word in the current game task
string taskVocab 

# start time for the given new task
string taskStartTime

string taskEndTime

string taskDuration

# current turn index in the task. initialize again when a new task is out
int8 taskTurnIndex

# current turn: child or robot?
string whoseTurn

# robot's current role: expert or novice?
string robotRole

string turnStartTime

string turnEndTime

string turnDuration

# number of retrieved object for the given task
int8[] numFinishedObjectsForTask

# total attempts for the current task
int8[] numTotalAttemptsForTask

int8 numChildClickCancelForTurn

int8[] numQAForTurn 

# game state trigger (e.g., object clicked, object found, object pronounced) 
string gameStateTrigger

# current interaction FSM state
string currentInteractionState

string currentGameState

# robot's current behavior (action): question asking, feedback, hints? 
string robotBehavior


# robot's clicked object
string robotClickedObj

# clicked object (either robot's or child's) right or wrong (bool)
string clickedRightObject  

# clicked object name (either robot's or child's)
string clickedObjName

# no tablet touch alert 
int8 numTouchAbsenceAlertPerTask

# whether the child pronoucnes the word to retrieve the object. 
bool objectWordPronounced


###########

