# ROS Message (child robot interaction)

# current turn: child or robot?
string whoseTurn

# robot's current role: expert or novice?
string robotRole

# robot's current behavior (action): question asking, feedback, hints? 
string robotBehavior


# robot's clicked object
string robotClickedObj

# clicked object (either robot's or child's) right or wrong (bool)
bool clickedRightObject  

# clicked object name (either robot's or child's)
string clickedObjName

# current game task index: 
int8 gameTask 

# vocab word in the current game task
string taskVocab 

# number of retrieved object for the given task
int8 numFinishedObjects

# number of questions the robot asked
int8 numQsAsked 
 
# number of questions the child answered
int8 numQsAnswered 

# number of child's attempts of retrieving an object
int8 numChildAttempts 

# number of child's correct attempts (num of objs collected by the child)
int8 numChildCorrectAttempts 

# game state trigger (e.g., object clicked, object found, object pronounced)
string gameStateTrigger
